using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// The two body integration passes of a substep, eight bodies at a time. A group of bodies is copied into a small staging block (one row of eight floats per quantity, one body per lane), the arithmetic
/// runs over <see cref="ILane{T}"/>, and the results are written back body by body. Every lane does what the scalar code for one body does, in the same order; the branches of the scalar code
/// (a body at rest, a runaway quaternion, a singular gyroscopic Jacobian, the speed limits) are selects, so every width gives the same bits as the one-lane instantiation.
/// Velocities: gravity and forces, the torque, the gyroscopic term as one implicit step (Catto, "Numerical Methods", GDC 2015), damping and the speed limits.
/// Positions: displacement, the accumulated and the body's rotation (Baraff: q' = q + h/2 (w, 0) q, renormalised), the world inverse inertia and the packed row the contact kernels read.
/// </summary>
[System.Runtime.CompilerServices.SkipLocalsInit]
internal static class IntegrateKernel3D
{
    private const int Lanes = 8;

    // Staging rows for the velocity pass.
    private const int VLinX = 0, VAngX = 3, VGravityScale = 6, VInvMass = 7, VForce = 8, VTorque = 11, VLinDamp = 14, VAngDamp = 15, VWInvI = 16, VGyro = 22, VRot = 23, VInertia = 27, VScratch = 33, VFields = 48;

    // Staging rows for the position pass; outputs reuse the rows of the quantities they replace.
    private const int PLin = 0, PAng = 3, PDp = 6, PDq = 9, PRot = 13, PInvI = 17, PInvMass = 23, PWInvI = 24, PTurning = 30, PFields = 31;

    private struct V3<T> where T : struct, ILane<T>
    {
        public T X, Y, Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public V3(T x, T y, T z) { X = x; Y = y; Z = z; }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Row<T>(ReadOnlySpan<float> stage, int f, int l0) where T : struct, ILane<T> => T.LoadAt(stage, f * Lanes + l0);

    /// <summary>q v q⁻¹ for a unit quaternion: v + w·t + u × t with t = 2 u × v.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Rotate<T>(T x, T y, T z, T w, V3<T> v, T two) where T : struct, ILane<T>
    {
        T tx = two * (y * v.Z - z * v.Y), ty = two * (z * v.X - x * v.Z), tz = two * (x * v.Y - y * v.X);
        return new V3<T>(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Abs<T>(T a) where T : struct, ILane<T> => T.Select(T.GreaterThan(T.Zero, a), T.Negate(a), a);

    /// <summary>Lanes where the value is finite (infinity and NaN compare false against infinity).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Finite<T>(T a) where T : struct, ILane<T> => T.GreaterThan(T.Broadcast(float.PositiveInfinity), Abs(a));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T NonZero<T>(T a) where T : struct, ILane<T> => Lane.Or(T.GreaterThan(a, T.Zero), T.GreaterThan(T.Zero, a));

    // ------------------------------------------------------------------ velocities

    /// <summary>
    /// The gyroscopic term ω × Iω as one implicit step, in the body frame: one Newton iteration on f(ω) = I(ω − ω₀) + h ω × Iω with the exact Jacobian J = I + h (ω̃ I − (Iω)~).
    /// Conserves angular momentum much better than an explicit step and, unlike a full solve, never blows up for a fast spin because the step is clamped by the speed limit.
    /// A singular or non-finite Jacobian leaves the velocity as it was.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static V3<T> Gyroscopic<T>(Span<float> st, int l0, V3<T> w, T h, T two) where T : struct, ILane<T>
    {
        GyroSetup(st, l0, w, h, two);
        return GyroSolve(st, l0, w, h, two);
    }

    // The two halves are methods of their own (the runtime stops inlining lane operations into a method once its budget is spent); the Jacobian and the force travel between them in the staging block.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GyroSetup<T>(Span<float> st, int l0, V3<T> w, T h, T two) where T : struct, ILane<T>
    {
        T qx = Row<T>(st, VRot, l0), qy = Row<T>(st, VRot + 1, l0), qz = Row<T>(st, VRot + 2, l0), qw = Row<T>(st, VRot + 3, l0);
        var wl = Rotate(T.Negate(qx), T.Negate(qy), T.Negate(qz), qw, w, two);
        T a = Row<T>(st, VInertia, l0), bb = Row<T>(st, VInertia + 1, l0), c = Row<T>(st, VInertia + 2, l0), d = Row<T>(st, VInertia + 3, l0), e = Row<T>(st, VInertia + 4, l0), ff = Row<T>(st, VInertia + 5, l0);
        T ix = a * wl.X + bb * wl.Y + c * wl.Z, iy = bb * wl.X + d * wl.Y + e * wl.Z, iz = c * wl.X + e * wl.Y + ff * wl.Z;
        ((wl.Y * iz - wl.Z * iy) * h).StoreAt(st, (VScratch + 0) * Lanes + l0); ((wl.Z * ix - wl.X * iz) * h).StoreAt(st, (VScratch + 1) * Lanes + l0); ((wl.X * iy - wl.Y * ix) * h).StoreAt(st, (VScratch + 2) * Lanes + l0);
        wl.X.StoreAt(st, (VScratch + 3) * Lanes + l0); wl.Y.StoreAt(st, (VScratch + 4) * Lanes + l0); wl.Z.StoreAt(st, (VScratch + 5) * Lanes + l0);
        T x = wl.X, y = wl.Y, z = wl.Z;
        // J = I + h (skew(wl) I − skew(iw)); rows below.
        T m00 = a + h * (T.Negate(z) * bb + y * c), m01 = bb + h * (T.Negate(z) * d + y * e), m02 = c + h * (T.Negate(z) * e + y * ff);
        T m10 = bb + h * (z * a - x * c), m11 = d + h * (z * bb - x * e), m12 = e + h * (z * c - x * ff);
        T m20 = c + h * (T.Negate(y) * a + x * bb), m21 = e + h * (T.Negate(y) * bb + x * d), m22 = ff + h * (T.Negate(y) * c + x * e);
        m01 = m01 - h * T.Negate(iz); m02 = m02 - h * iy; m10 = m10 - h * iz; m12 = m12 - h * T.Negate(ix); m20 = m20 - h * T.Negate(iy); m21 = m21 - h * ix;
        m00.StoreAt(st, (VScratch + 6) * Lanes + l0); m01.StoreAt(st, (VScratch + 7) * Lanes + l0); m02.StoreAt(st, (VScratch + 8) * Lanes + l0);
        m10.StoreAt(st, (VScratch + 9) * Lanes + l0); m11.StoreAt(st, (VScratch + 10) * Lanes + l0); m12.StoreAt(st, (VScratch + 11) * Lanes + l0);
        m20.StoreAt(st, (VScratch + 12) * Lanes + l0); m21.StoreAt(st, (VScratch + 13) * Lanes + l0); m22.StoreAt(st, (VScratch + 14) * Lanes + l0);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static V3<T> GyroSolve<T>(Span<float> st, int l0, V3<T> w, T h, T two) where T : struct, ILane<T>
    {
        T qx = Row<T>(st, VRot, l0), qy = Row<T>(st, VRot + 1, l0), qz = Row<T>(st, VRot + 2, l0), qw = Row<T>(st, VRot + 3, l0);
        T fx = Row<T>(st, VScratch, l0), fy = Row<T>(st, VScratch + 1, l0), fz = Row<T>(st, VScratch + 2, l0);
        T wlx = Row<T>(st, VScratch + 3, l0), wly = Row<T>(st, VScratch + 4, l0), wlz = Row<T>(st, VScratch + 5, l0);
        T m00 = Row<T>(st, VScratch + 6, l0), m01 = Row<T>(st, VScratch + 7, l0), m02 = Row<T>(st, VScratch + 8, l0);
        T m10 = Row<T>(st, VScratch + 9, l0), m11 = Row<T>(st, VScratch + 10, l0), m12 = Row<T>(st, VScratch + 11, l0);
        T m20 = Row<T>(st, VScratch + 12, l0), m21 = Row<T>(st, VScratch + 13, l0), m22 = Row<T>(st, VScratch + 14, l0);
        T c00 = m11 * m22 - m12 * m21, c01 = m12 * m20 - m10 * m22, c02 = m10 * m21 - m11 * m20;
        T det = m00 * c00 + m01 * c01 + m02 * c02;
        T okDet = Lane.And(T.GreaterThan(Abs(det), T.Broadcast(1e-30f)), Finite(det));
        T inv = T.Broadcast(1f) / det;
        T dx = ((m11 * m22 - m12 * m21) * fx + (m02 * m21 - m01 * m22) * fy + (m01 * m12 - m02 * m11) * fz) * inv;
        T dy = (c01 * fx + (m00 * m22 - m02 * m20) * fy + (m02 * m10 - m00 * m12) * fz) * inv;
        T dz = (c02 * fx + (m01 * m20 - m00 * m21) * fy + (m00 * m11 - m01 * m10) * fz) * inv;
        var nl = new V3<T>(wlx - dx, wly - dy, wlz - dz);
        T ok = Lane.And(okDet, Finite(nl.X + nl.Y + nl.Z));
        var back = Rotate(qx, qy, qz, qw, nl, two);
        return new V3<T>(T.Select(ok, back.X, w.X), T.Select(ok, back.Y, w.Y), T.Select(ok, back.Z, w.Z));
    }

    private static bool AllDynamic(ReadOnlySpan<byte> type, ReadOnlySpan<int> chunk)
    {
        foreach (int b in chunk) { if (Mem.At(type, b) != (byte)BodyType.Dynamic) { return false; } }
        return true;
    }

    /// <summary>
    /// The velocity pass's staging rows for eight dynamic bodies, by row loads and transposes where a body's floats sit together (r4 h06: the velocities from the packed velocity rows, the world inverse
    /// inertia from the packed inertia rows, which hold the same floats as WInvI throughout the solve, rotation and local inertia by exact four-float loads) and lane by lane for the rest. The gyroscopic
    /// rows are zero in the lanes whose mask in the VGyro row (written by the caller) is clear, as in the scalar staging.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GatherVelocities256(Span<float> st, ReadOnlySpan<int> ids, ReadOnlySpan<float> pk, ReadOnlySpan<float> gravityScale, ReadOnlySpan<float> invMass,
        ReadOnlySpan<float> force, ReadOnlySpan<float> torque, ReadOnlySpan<float> linDamping, ReadOnlySpan<float> angDamping, ReadOnlySpan<float> rot, ReadOnlySpan<float> inertia, bool anyGyro)
    {
        var v = Lanes256.GatherRows(pk, ids, Engine3D.PkStride, Engine3D.PkVelocity);
        v.R0.StoreAt(st, VLinX * Lanes); v.R1.StoreAt(st, (VLinX + 1) * Lanes); v.R2.StoreAt(st, (VLinX + 2) * Lanes);
        v.R3.StoreAt(st, VAngX * Lanes); v.R4.StoreAt(st, (VAngX + 1) * Lanes); v.R5.StoreAt(st, (VAngX + 2) * Lanes);
        Lanes256.GatherOne(gravityScale, ids).StoreAt(st, VGravityScale * Lanes); Lanes256.GatherOne(invMass, ids).StoreAt(st, VInvMass * Lanes);
        Lanes256.GatherOne(force, ids, 3, 0).StoreAt(st, VForce * Lanes); Lanes256.GatherOne(force, ids, 3, 1).StoreAt(st, (VForce + 1) * Lanes); Lanes256.GatherOne(force, ids, 3, 2).StoreAt(st, (VForce + 2) * Lanes);
        Lanes256.GatherOne(torque, ids, 3, 0).StoreAt(st, VTorque * Lanes); Lanes256.GatherOne(torque, ids, 3, 1).StoreAt(st, (VTorque + 1) * Lanes); Lanes256.GatherOne(torque, ids, 3, 2).StoreAt(st, (VTorque + 2) * Lanes);
        Lanes256.GatherOne(linDamping, ids).StoreAt(st, VLinDamp * Lanes); Lanes256.GatherOne(angDamping, ids).StoreAt(st, VAngDamp * Lanes);
        var i = Lanes256.GatherRows(pk, ids, Engine3D.PkStride, Engine3D.PkInertia);
        i.R0.StoreAt(st, VWInvI * Lanes); i.R1.StoreAt(st, (VWInvI + 1) * Lanes); i.R2.StoreAt(st, (VWInvI + 2) * Lanes);
        i.R3.StoreAt(st, (VWInvI + 3) * Lanes); i.R4.StoreAt(st, (VWInvI + 4) * Lanes); i.R5.StoreAt(st, (VWInvI + 5) * Lanes);
        if (!anyGyro)
        {
            for (int f = VRot; f < VInertia + 6; f++) { Lanes256.Zero.StoreAt(st, f * Lanes); }
            return;
        }

        Lanes256 gyro = Lanes256.LoadAt(st, VGyro * Lanes);
        Lanes256.GatherRows4(rot, ids, 4, 0, out var r0, out var r1, out var r2, out var r3);
        Lane.And(gyro, r0).StoreAt(st, VRot * Lanes); Lane.And(gyro, r1).StoreAt(st, (VRot + 1) * Lanes); Lane.And(gyro, r2).StoreAt(st, (VRot + 2) * Lanes); Lane.And(gyro, r3).StoreAt(st, (VRot + 3) * Lanes);
        Lanes256.GatherRows4(inertia, ids, 6, 0, out var i0, out var i1, out var i2, out var i3);
        Lanes256.GatherRows4(inertia, ids, 6, 2, out _, out _, out var i4, out var i5);
        Lane.And(gyro, i0).StoreAt(st, VInertia * Lanes); Lane.And(gyro, i1).StoreAt(st, (VInertia + 1) * Lanes); Lane.And(gyro, i2).StoreAt(st, (VInertia + 2) * Lanes);
        Lane.And(gyro, i3).StoreAt(st, (VInertia + 3) * Lanes); Lane.And(gyro, i4).StoreAt(st, (VInertia + 4) * Lanes); Lane.And(gyro, i5).StoreAt(st, (VInertia + 5) * Lanes);
    }

    /// <summary>One group of up to eight bodies: velocities after gravity, forces, torque, the gyroscopic term, damping and the speed limits. Lanes with <c>write[lane]</c> false are not written back.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void VelocityLanes<T>(Span<float> st, int l0, T hh, T maxSpeed, T maxSpeed2, T maxSpin, T maxSpin2, T gx, T gy, T gz, bool observeDamping) where T : struct, ILane<T>
    {
        T two = T.Broadcast(2f), one = T.Broadcast(1f);
        T gs = Row<T>(st, VGravityScale, l0), im = Row<T>(st, VInvMass, l0);
        T vx = Row<T>(st, VLinX, l0) + hh * (gs * gx + im * Row<T>(st, VForce, l0));
        T vy = Row<T>(st, VLinX + 1, l0) + hh * (gs * gy + im * Row<T>(st, VForce + 1, l0));
        T vz = Row<T>(st, VLinX + 2, l0) + hh * (gs * gz + im * Row<T>(st, VForce + 2, l0));
        var w = new V3<T>(Row<T>(st, VAngX, l0), Row<T>(st, VAngX + 1, l0), Row<T>(st, VAngX + 2, l0));
        T tx = Row<T>(st, VTorque, l0), ty = Row<T>(st, VTorque + 1, l0), tz = Row<T>(st, VTorque + 2, l0);
        T hasTorque = Lane.Or(Lane.Or(NonZero(tx), NonZero(ty)), NonZero(tz));
        T i0 = Row<T>(st, VWInvI, l0), i1 = Row<T>(st, VWInvI + 1, l0), i2 = Row<T>(st, VWInvI + 2, l0), i3 = Row<T>(st, VWInvI + 3, l0), i4 = Row<T>(st, VWInvI + 4, l0), i5 = Row<T>(st, VWInvI + 5, l0);
        T ax = (i0 * tx + i1 * ty + i2 * tz) * hh, ay = (i1 * tx + i3 * ty + i4 * tz) * hh, az = (i2 * tx + i4 * ty + i5 * tz) * hh;
        w = new V3<T>(T.Select(hasTorque, w.X + ax, w.X), T.Select(hasTorque, w.Y + ay, w.Y), T.Select(hasTorque, w.Z + az, w.Z));
        T gyro = Row<T>(st, VGyro, l0);
        var g = Gyroscopic(st, l0, w, hh, two);
        w = new V3<T>(T.Select(gyro, g.X, w.X), T.Select(gyro, g.Y, w.Y), T.Select(gyro, g.Z, w.Z));
        // Zero damping scales by exactly one (the division of 1 by 1 + h·0 is exact), so every lane takes the same multiplication.
        T dl = one / (one + hh * Row<T>(st, VLinDamp, l0)), da = one / (one + hh * Row<T>(st, VAngDamp, l0));
        if (observeDamping)
        {
            T undampedX = vx, undampedY = vy, undampedZ = vz;
            var undampedW = w;
            vx = vx * dl; vy = vy * dl; vz = vz * dl;
            w = new V3<T>(w.X * da, w.Y * da, w.Z * da);
            // The gyroscopic scratch is dead here; its first six rows now hold damping observations.
            (vx - undampedX).StoreAt(st, VScratch * Lanes + l0); (vy - undampedY).StoreAt(st, (VScratch + 1) * Lanes + l0); (vz - undampedZ).StoreAt(st, (VScratch + 2) * Lanes + l0);
            (w.X - undampedW.X).StoreAt(st, (VScratch + 3) * Lanes + l0); (w.Y - undampedW.Y).StoreAt(st, (VScratch + 4) * Lanes + l0); (w.Z - undampedW.Z).StoreAt(st, (VScratch + 5) * Lanes + l0);
        }
        else
        {
            vx = vx * dl; vy = vy * dl; vz = vz * dl;
            w = new V3<T>(w.X * da, w.Y * da, w.Z * da);
        }
        T v2 = vx * vx + vy * vy + vz * vz;
        T fast = T.GreaterThan(v2, maxSpeed2);
        T scale = maxSpeed / T.Sqrt(v2);
        vx = T.Select(fast, vx * scale, vx); vy = T.Select(fast, vy * scale, vy); vz = T.Select(fast, vz * scale, vz);
        T w2 = w.X * w.X + w.Y * w.Y + w.Z * w.Z;
        T spin = T.GreaterThan(w2, maxSpin2);
        T sscale = maxSpin / T.Sqrt(w2);
        w = new V3<T>(T.Select(spin, w.X * sscale, w.X), T.Select(spin, w.Y * sscale, w.Y), T.Select(spin, w.Z * sscale, w.Z));
        vx.StoreAt(st, VLinX * Lanes + l0); vy.StoreAt(st, (VLinX + 1) * Lanes + l0); vz.StoreAt(st, (VLinX + 2) * Lanes + l0);
        w.X.StoreAt(st, VAngX * Lanes + l0); w.Y.StoreAt(st, (VAngX + 1) * Lanes + l0); w.Z.StoreAt(st, (VAngX + 2) * Lanes + l0);
    }

    public static void Velocities<T>(Engine3D e, ReadOnlySpan<int> list, float h) where T : struct, ILane<T>
    {
        int width = T.Count;
        var bodies = e.Core.Bodies;
        // r4 h06: the velocities are read from and written to the bodies' packed velocity rows (see Engine3D.Pk), not the body store.
        Span<float> pk = e.Pk;
        ReadOnlySpan<float> invMass = bodies.InverseMass, rot = bodies.Rotation;
        ReadOnlySpan<byte> type = bodies.Type;
        ReadOnlySpan<float> gravityScale = e.GravityScale, force = e.Force, torque = e.Torque, linDamping = e.LinearDamping, angDamping = e.AngularDamping, wInvI = e.WInvI, inertia = e.Inertia;
        ReadOnlySpan<byte> flags = e.BodyFlags;
        var g = e.Core.Options.Gravity;
        float maxSpeed = e.Opt.MaxLinearSpeed, maxSpin = e.Opt.MaxRotationPerSubstep / h;
        float allOnes = BitConverter.Int32BitsToSingle(-1);
        Span<float> st = stackalloc float[VFields * Lanes];
        Span<int> ids = stackalloc int[Lanes];
        T hh = T.Broadcast(h), ms = T.Broadcast(maxSpeed), ms2 = T.Broadcast(maxSpeed * maxSpeed), mw = T.Broadcast(maxSpin), mw2 = T.Broadcast(maxSpin * maxSpin);
        T gx = T.Broadcast(g.X), gy = T.Broadcast(g.Y), gz = T.Broadcast(g.Z);
        // Every body index below is a body of the awake list (below the world's high-water mark) and every per-body array was sized for that mark when the step began, so the unchecked accessors
        // stay in range; the Checked build verifies each access. The staging block is written for every lane (zeros for lanes that are not dynamic bodies) before the arithmetic reads it.
        for (int i0 = 0; i0 < list.Length; i0 += Lanes)
        {
            int n = Math.Min(Lanes, list.Length - i0);
            int live = 0;
            bool observeDamping = false;
            // A full chunk of dynamic bodies at eight lanes gathers whole rows and transposes them in registers (same values as the scalar staging below, which every other case uses).
            if (typeof(T) == typeof(Lanes256) && n == Lanes && System.Runtime.Intrinsics.X86.Avx.IsSupported && !e.Opt.DetailedInspection && AllDynamic(type, list.Slice(i0, Lanes)))
            {
                var chunk = list.Slice(i0, Lanes);
                bool anyGyro = false;
                for (int l = 0; l < Lanes; l++)
                {
                    int b = chunk[l];
                    ids[l] = b;
                    bool gyro = (flags[b] & 2) != 0;
                    st[VGyro * Lanes + l] = gyro ? allOnes : 0f;
                    anyGyro |= gyro;
                }

                live = Lanes;
                GatherVelocities256(st, chunk, pk, gravityScale, invMass, force, torque, linDamping, angDamping, rot, inertia, anyGyro);
                for (int l0 = 0; l0 < Lanes; l0 += width) { VelocityLanes<T>(st, l0, hh, ms, ms2, mw, mw2, gx, gy, gz, false); }
                // Every lane is a dynamic body: its whole velocity row goes back with one store.
                Lanes256 zero = Lanes256.Zero;
                Lanes256.ScatterRows(Lanes256.LoadAt(st, VLinX * Lanes).AsVector256, Lanes256.LoadAt(st, (VLinX + 1) * Lanes).AsVector256, Lanes256.LoadAt(st, (VLinX + 2) * Lanes).AsVector256,
                    Lanes256.LoadAt(st, VAngX * Lanes).AsVector256, Lanes256.LoadAt(st, (VAngX + 1) * Lanes).AsVector256, Lanes256.LoadAt(st, (VAngX + 2) * Lanes).AsVector256, zero.AsVector256, zero.AsVector256,
                    pk, ref MemoryMarshal.GetReference(chunk), Lanes, Engine3D.PkStride, Engine3D.PkVelocity);
                continue;
            }
            else
            {
                for (int l = 0; l < Lanes; l++)
                {
                    int b = -1;
                    if (l < n)
                    {
                        int cand = list[i0 + l];
                        if (Mem.At(type, cand) == (byte)BodyType.Dynamic) { b = cand; live++; }
                    }

                    ids[l] = b;
                    if (b < 0)
                    {
                        for (int f = 0; f < VFields; f++) { st[f * Lanes + l] = 0f; }
                        continue;
                    }

                    int b3 = 3 * b, b6 = 6 * b, b4 = 4 * b, bv = Engine3D.PkStride * b + Engine3D.PkVelocity;
                    st[VLinX * Lanes + l] = Mem.At(pk, bv); st[(VLinX + 1) * Lanes + l] = Mem.At(pk, bv + 1); st[(VLinX + 2) * Lanes + l] = Mem.At(pk, bv + 2);
                    st[VAngX * Lanes + l] = Mem.At(pk, bv + 3); st[(VAngX + 1) * Lanes + l] = Mem.At(pk, bv + 4); st[(VAngX + 2) * Lanes + l] = Mem.At(pk, bv + 5);
                    st[VGravityScale * Lanes + l] = Mem.At(gravityScale, b); st[VInvMass * Lanes + l] = Mem.At(invMass, b);
                    st[VForce * Lanes + l] = Mem.At(force, b3); st[(VForce + 1) * Lanes + l] = Mem.At(force, b3 + 1); st[(VForce + 2) * Lanes + l] = Mem.At(force, b3 + 2);
                    st[VTorque * Lanes + l] = Mem.At(torque, b3); st[(VTorque + 1) * Lanes + l] = Mem.At(torque, b3 + 1); st[(VTorque + 2) * Lanes + l] = Mem.At(torque, b3 + 2);
                    st[VLinDamp * Lanes + l] = Mem.At(linDamping, b); st[VAngDamp * Lanes + l] = Mem.At(angDamping, b);
                    observeDamping |= e.Opt.DetailedInspection && (st[VLinDamp * Lanes + l] != 0f || st[VAngDamp * Lanes + l] != 0f);
                    for (int k = 0; k < 6; k++) { st[(VWInvI + k) * Lanes + l] = Mem.At(wInvI, b6 + k); }
                    bool gyro = (flags[b] & 2) != 0;
                    st[VGyro * Lanes + l] = gyro ? allOnes : 0f;
                    for (int k = 0; k < 4; k++) { st[(VRot + k) * Lanes + l] = gyro ? Mem.At(rot, b4 + k) : 0f; }
                    for (int k = 0; k < 6; k++) { st[(VInertia + k) * Lanes + l] = gyro ? Mem.At(inertia, b6 + k) : 0f; }
                }
            }

            if (live == 0) { continue; }
            for (int l0 = 0; l0 < Lanes; l0 += width) { VelocityLanes<T>(st, l0, hh, ms, ms2, mw, mw2, gx, gy, gz, observeDamping); }
            for (int l = 0; l < Lanes; l++)
            {
                int b = ids[l];
                if (b < 0) { continue; }
                int bv = Engine3D.PkStride * b + Engine3D.PkVelocity;
                if (observeDamping && (linDamping[b] != 0f || angDamping[b] != 0f))
                {
                    double mass = 1.0 / invMass[b];
                    var dw = new RecursiveAI.OneMath.Vector3F(st[(VScratch + 3) * Lanes + l], st[(VScratch + 4) * Lanes + l], st[(VScratch + 5) * Lanes + l]);
                    e.Core.BodyReadback.Damping(b, new(st[VScratch * Lanes + l] * mass, st[(VScratch + 1) * Lanes + l] * mass, st[(VScratch + 2) * Lanes + l] * mass), e.InspectionDampingTorque(b, dw));
                }
                Mem.At(pk, bv) = st[VLinX * Lanes + l]; Mem.At(pk, bv + 1) = st[(VLinX + 1) * Lanes + l]; Mem.At(pk, bv + 2) = st[(VLinX + 2) * Lanes + l];
                Mem.At(pk, bv + 3) = st[VAngX * Lanes + l]; Mem.At(pk, bv + 4) = st[(VAngX + 1) * Lanes + l]; Mem.At(pk, bv + 5) = st[(VAngX + 2) * Lanes + l];
            }
        }
    }

    // ------------------------------------------------------------------ positions

    /// <summary>q' = q + h/2 (w, 0) q, renormalised; a runaway state (a zero or non-finite magnitude) keeps the last valid orientation.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void IntegrateRotation<T>(ref T qx, ref T qy, ref T qz, ref T qw, V3<T> w, T hh) where T : struct, ILane<T>
    {
        T dx = w.X * qw + w.Y * qz - w.Z * qy;
        T dy = w.Y * qw + w.Z * qx - w.X * qz;
        T dz = w.Z * qw + w.X * qy - w.Y * qx;
        T dw = T.Negate(w.X * qx + w.Y * qy + w.Z * qz);
        T nx = qx + hh * dx, ny = qy + hh * dy, nz = qz + hh * dz, nw = qw + hh * dw;
        T mag = T.Sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        T ok = Lane.And(T.GreaterThan(mag, T.Zero), Finite(mag));
        T inv = T.Broadcast(1f) / mag;
        qx = T.Select(ok, nx * inv, qx); qy = T.Select(ok, ny * inv, qy); qz = T.Select(ok, nz * inv, qz); qw = T.Select(ok, nw * inv, qw);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void PositionLanes<T>(Span<float> st, int l0, T h) where T : struct, ILane<T>
    {
        Displace<T>(st, l0, h);
        Rotations<T>(st, l0, h);
        WorldInertia<T>(st, l0);
    }

    // Row PLin leaves holding the displacement of this substep (the caller adds it to the double position).
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Displace<T>(Span<float> st, int l0, T h) where T : struct, ILane<T>
    {
        T dx = h * Row<T>(st, PLin, l0), dy = h * Row<T>(st, PLin + 1, l0), dz = h * Row<T>(st, PLin + 2, l0);
        dx.StoreAt(st, PLin * Lanes + l0); dy.StoreAt(st, (PLin + 1) * Lanes + l0); dz.StoreAt(st, (PLin + 2) * Lanes + l0);
        (Row<T>(st, PDp, l0) + dx).StoreAt(st, PDp * Lanes + l0); (Row<T>(st, PDp + 1, l0) + dy).StoreAt(st, (PDp + 1) * Lanes + l0); (Row<T>(st, PDp + 2, l0) + dz).StoreAt(st, (PDp + 2) * Lanes + l0);
    }

    /// <summary>Turns the accumulated rotation and the body's rotation by the angular velocity; a body at rest keeps both. Row PTurning gets the mask of lanes that turned.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Rotations<T>(Span<float> st, int l0, T h) where T : struct, ILane<T>
    {
        T hh = T.Broadcast(0.5f) * h;
        var w = new V3<T>(Row<T>(st, PAng, l0), Row<T>(st, PAng + 1, l0), Row<T>(st, PAng + 2, l0));
        T turning = Lane.Or(Lane.Or(NonZero(w.X), NonZero(w.Y)), NonZero(w.Z));
        turning.StoreAt(st, PTurning * Lanes + l0);
        T ax = Row<T>(st, PDq, l0), ay = Row<T>(st, PDq + 1, l0), az = Row<T>(st, PDq + 2, l0), aw = Row<T>(st, PDq + 3, l0);
        T ox = ax, oy = ay, oz = az, ow = aw;
        IntegrateRotation(ref ax, ref ay, ref az, ref aw, w, hh);
        T sx = Row<T>(st, PRot, l0), sy = Row<T>(st, PRot + 1, l0), sz = Row<T>(st, PRot + 2, l0), sw = Row<T>(st, PRot + 3, l0);
        T px = sx, py = sy, pz = sz, pw = sw;
        IntegrateRotation(ref sx, ref sy, ref sz, ref sw, w, hh);
        T.Select(turning, ax, ox).StoreAt(st, PDq * Lanes + l0); T.Select(turning, ay, oy).StoreAt(st, (PDq + 1) * Lanes + l0); T.Select(turning, az, oz).StoreAt(st, (PDq + 2) * Lanes + l0); T.Select(turning, aw, ow).StoreAt(st, (PDq + 3) * Lanes + l0);
        T.Select(turning, sx, px).StoreAt(st, PRot * Lanes + l0); T.Select(turning, sy, py).StoreAt(st, (PRot + 1) * Lanes + l0); T.Select(turning, sz, pz).StoreAt(st, (PRot + 2) * Lanes + l0); T.Select(turning, sw, pw).StoreAt(st, (PRot + 3) * Lanes + l0);
    }

    /// <summary>World inverse inertia R · I⁻¹ · Rᵀ from the new rotation; zeros for a body without inverse mass; unchanged for a body that did not turn.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void WorldInertia<T>(Span<float> st, int l0) where T : struct, ILane<T>
    {
        T two = T.Broadcast(2f), one = T.Broadcast(1f);
        T sx = Row<T>(st, PRot, l0), sy = Row<T>(st, PRot + 1, l0), sz = Row<T>(st, PRot + 2, l0), sw = Row<T>(st, PRot + 3, l0);
        T xx = sx * sx, yy = sy * sy, zz = sz * sz, xy = sx * sy, xz = sx * sz, yz = sy * sz, qsx = sw * sx, qsy = sw * sy, qsz = sw * sz;
        T r00 = one - two * (yy + zz), r01 = two * (xy - qsz), r02 = two * (xz + qsy);
        T r10 = two * (xy + qsz), r11 = one - two * (xx + zz), r12 = two * (yz - qsx);
        T r20 = two * (xz - qsy), r21 = two * (yz + qsx), r22 = one - two * (xx + yy);
        T ia = Row<T>(st, PInvI, l0), ib = Row<T>(st, PInvI + 1, l0), ic = Row<T>(st, PInvI + 2, l0), id = Row<T>(st, PInvI + 3, l0), ie = Row<T>(st, PInvI + 4, l0), ifz = Row<T>(st, PInvI + 5, l0);
        T t00 = r00 * ia + r01 * ib + r02 * ic, t01 = r00 * ib + r01 * id + r02 * ie, t02 = r00 * ic + r01 * ie + r02 * ifz;
        T t10 = r10 * ia + r11 * ib + r12 * ic, t11 = r10 * ib + r11 * id + r12 * ie, t12 = r10 * ic + r11 * ie + r12 * ifz;
        T t20 = r20 * ia + r21 * ib + r22 * ic, t21 = r20 * ib + r21 * id + r22 * ie, t22 = r20 * ic + r21 * ie + r22 * ifz;
        T dynamic = T.GreaterThan(Row<T>(st, PInvMass, l0), T.Zero), turning = Row<T>(st, PTurning, l0), zero = T.Zero;
        Store(st, 0, l0, turning, dynamic, t00 * r00 + t01 * r01 + t02 * r02, zero);
        Store(st, 1, l0, turning, dynamic, t00 * r10 + t01 * r11 + t02 * r12, zero);
        Store(st, 2, l0, turning, dynamic, t00 * r20 + t01 * r21 + t02 * r22, zero);
        Store(st, 3, l0, turning, dynamic, t10 * r10 + t11 * r11 + t12 * r12, zero);
        Store(st, 4, l0, turning, dynamic, t10 * r20 + t11 * r21 + t12 * r22, zero);
        Store(st, 5, l0, turning, dynamic, t20 * r20 + t21 * r21 + t22 * r22, zero);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Store<T>(Span<float> st, int k, int l0, T turning, T dynamic, T fresh, T zero) where T : struct, ILane<T> =>
        T.Select(turning, T.Select(dynamic, fresh, zero), Row<T>(st, PWInvI + k, l0)).StoreAt(st, (PWInvI + k) * Lanes + l0);

    /// <summary>
    /// The position pass's staging rows for eight bodies, by whole-row loads and transposes: velocities, world inverse inertia, displacement and accumulated rotation from the three packed rows
    /// (<see cref="Engine3D.Pk"/>: row 0 is the velocity through the solve, rows 1 and 2 hold the same floats as WInvI, Dp and Dq, and this pass is what writes them), rotation and local inverse inertia by
    /// exact four-float loads, the inverse mass lane by lane.
    /// Returns the mask of lanes (bit l = ids[l]) whose body is dynamic and whose velocity <see cref="SaneRow"/> could change: a component NaN or beyond its cap, or a squared length above
    /// (a hair under) the squared cap. Sane changes nothing for any other lane, so the caller only runs SaneRow for these and gathers again (r4 h14; the test is conservative, never short).
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int GatherPositions256(Span<float> st, ReadOnlySpan<int> ids, ReadOnlySpan<float> pk, ReadOnlySpan<float> rot, ReadOnlySpan<float> invI, ReadOnlySpan<float> invMass, float speedCap, float spinCap)
    {
        var v = Lanes256.GatherRows(pk, ids, Engine3D.PkStride, Engine3D.PkVelocity);
        var im = Lanes256.GatherOne(invMass, ids);
        Vector256<float> bad = Vector256.BitwiseOr(Unsane(v.R0.AsVector256, v.R1.AsVector256, v.R2.AsVector256, speedCap), Unsane(v.R3.AsVector256, v.R4.AsVector256, v.R5.AsVector256, spinCap));
        int mask = (int)Vector256.ExtractMostSignificantBits(Vector256.BitwiseAnd(bad, Vector256.GreaterThan(im.AsVector256, Vector256<float>.Zero)));
        v.R0.StoreAt(st, PLin * Lanes); v.R1.StoreAt(st, (PLin + 1) * Lanes); v.R2.StoreAt(st, (PLin + 2) * Lanes);
        v.R3.StoreAt(st, PAng * Lanes); v.R4.StoreAt(st, (PAng + 1) * Lanes); v.R5.StoreAt(st, (PAng + 2) * Lanes);
        var p = Lanes256.GatherRows(pk, ids, Engine3D.PkStride, Engine3D.PkPose);
        p.R0.StoreAt(st, PDp * Lanes); p.R1.StoreAt(st, (PDp + 1) * Lanes); p.R2.StoreAt(st, (PDp + 2) * Lanes);
        p.R3.StoreAt(st, PDq * Lanes); p.R4.StoreAt(st, (PDq + 1) * Lanes); p.R5.StoreAt(st, (PDq + 2) * Lanes); p.R6.StoreAt(st, (PDq + 3) * Lanes);
        var i = Lanes256.GatherRows(pk, ids, Engine3D.PkStride, Engine3D.PkInertia);
        i.R0.StoreAt(st, PWInvI * Lanes); i.R1.StoreAt(st, (PWInvI + 1) * Lanes); i.R2.StoreAt(st, (PWInvI + 2) * Lanes);
        i.R3.StoreAt(st, (PWInvI + 3) * Lanes); i.R4.StoreAt(st, (PWInvI + 4) * Lanes); i.R5.StoreAt(st, (PWInvI + 5) * Lanes);
        Lanes256.GatherRows4(rot, ids, 4, 0, out var r0, out var r1, out var r2, out var r3);
        r0.StoreAt(st, PRot * Lanes); r1.StoreAt(st, (PRot + 1) * Lanes); r2.StoreAt(st, (PRot + 2) * Lanes); r3.StoreAt(st, (PRot + 3) * Lanes);
        Lanes256.GatherRows4(invI, ids, 6, 0, out var i0, out var i1, out var i2, out var i3);
        Lanes256.GatherRows4(invI, ids, 6, 2, out _, out _, out var i4, out var i5);
        i0.StoreAt(st, PInvI * Lanes); i1.StoreAt(st, (PInvI + 1) * Lanes); i2.StoreAt(st, (PInvI + 2) * Lanes); i3.StoreAt(st, (PInvI + 3) * Lanes); i4.StoreAt(st, (PInvI + 4) * Lanes); i5.StoreAt(st, (PInvI + 5) * Lanes);
        im.StoreAt(st, PInvMass * Lanes);
        return mask;
    }

    /// <summary>Lanes where <see cref="Sane"/> with this cap could change the vector: !(|c| &lt;= cap) catches NaN and a component past the cap; the length test uses a cap squared a hair low, so rounding can only add lanes.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector256<float> Unsane(Vector256<float> x, Vector256<float> y, Vector256<float> z, float cap)
    {
        var c = Vector256.Create(cap);
        var ok = Vector256.BitwiseAnd(Vector256.BitwiseAnd(Vector256.LessThanOrEqual(Vector256.Abs(x), c), Vector256.LessThanOrEqual(Vector256.Abs(y), c)), Vector256.LessThanOrEqual(Vector256.Abs(z), c));
        var s2 = x * x + y * y + z * z;
        return Vector256.BitwiseOr(Vector256.OnesComplement(ok), Vector256.GreaterThan(s2, Vector256.Create(cap * cap * 0.999f)));
    }

    /// <summary>
    /// A velocity the solve left absurd (a stiff constraint on a light body with a long lever arm can multiply a velocity by thousands in one pass) becomes finite and bounded before it moves anything:
    /// NaN becomes zero, an infinite component the cap, and a vector longer than the cap is scaled to it. The cap is far above the integration's own limits (16 times), so a body that is merely fast is never touched.
    /// </summary>
    private static bool Sane(ref float x, ref float y, ref float z, float cap)
    {
        float ox = x, oy = y, oz = z;
        if (float.IsNaN(x)) { x = 0f; } else if (x > cap) { x = cap; } else if (x < -cap) { x = -cap; }
        if (float.IsNaN(y)) { y = 0f; } else if (y > cap) { y = cap; } else if (y < -cap) { y = -cap; }
        if (float.IsNaN(z)) { z = 0f; } else if (z > cap) { z = cap; } else if (z < -cap) { z = -cap; }
        float s2 = x * x + y * y + z * z;
        if (s2 > cap * cap)
        {
            float m = Math.Max(Math.Abs(x), Math.Max(Math.Abs(y), Math.Abs(z)));
            float k = cap / (m * 1.7320509f);
            x *= k; y *= k; z *= k;
        }

        return x != ox || y != oy || z != oz;
    }

    public static void Positions<T>(Engine3D e, ReadOnlySpan<int> list, float h) where T : struct, ILane<T>
    {
        int width = T.Count;
        var bodies = e.Core.Bodies;
        float speedCap = Math.Min(16f * e.Opt.MaxLinearSpeed, 1e18f), spinCap = Math.Min(16f * e.Opt.MaxRotationPerSubstep / h, 1e18f);
        // r4 h06: the velocities are the packed velocity rows (see Engine3D.Pk); the clamp below reads and writes them there.
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        Span<double> pos = bodies.Position;
        Span<float> rot = bodies.Rotation, dp = e.Dp, dq = e.Dq, wInvI = e.WInvI, pk = e.Pk;
        Span<float> st = stackalloc float[PFields * Lanes];
        T th = T.Broadcast(h);
        // As in Velocities: the list holds bodies below the high-water mark and every per-body array was sized for it, so the unchecked accessors stay in range (verified in Checked).
        for (int i0 = 0; i0 < list.Length; i0 += Lanes)
        {
            int n = Math.Min(Lanes, list.Length - i0);
            // A full chunk at eight lanes gathers whole rows and transposes them in registers (same values as the scalar staging below, which every other width and the last partial chunk use).
            bool rows = typeof(T) == typeof(Lanes256) && n == Lanes && System.Runtime.Intrinsics.X86.Avx.IsSupported;
            if (rows)
            {
                // r4 h14: gather first and test the eight velocities in lanes; only a chunk with a lane Sane could change runs the scalar clamp on those lanes and gathers again (same staged values
                // as clamping every body first, which is what the scalar path below does).
                var ids = list.Slice(i0, Lanes);
                int bad = GatherPositions256(st, ids, pk, rot, invI, invMass, speedCap, spinCap);
                if (bad != 0)
                {
                    for (int l = 0; l < Lanes; l++) { if ((bad & (1 << l)) != 0) { SaneRow(pk, Engine3D.PkStride * ids[l] + Engine3D.PkVelocity, speedCap, spinCap); } }
                    GatherPositions256(st, ids, pk, rot, invI, invMass, speedCap, spinCap);
                }
            }
            else
            {
                for (int l = 0; l < Lanes; l++)
                {
                    if (l >= n)
                    {
                        for (int f = 0; f < PFields; f++) { st[f * Lanes + l] = 0f; }
                        continue;
                    }

                    int b = list[i0 + l];
                    int b3 = 3 * b, b4 = 4 * b, b6 = 6 * b, bv = Engine3D.PkStride * b + Engine3D.PkVelocity;
                    if (Mem.At(invMass, b) > 0f) { SaneRow(pk, bv, speedCap, spinCap); }

                    for (int k = 0; k < 3; k++)
                    {
                        st[(PLin + k) * Lanes + l] = Mem.At(pk, bv + k); st[(PAng + k) * Lanes + l] = Mem.At(pk, bv + 3 + k); st[(PDp + k) * Lanes + l] = Mem.At(dp, b3 + k);
                    }

                    for (int k = 0; k < 4; k++) { st[(PDq + k) * Lanes + l] = Mem.At(dq, b4 + k); st[(PRot + k) * Lanes + l] = Mem.At(rot, b4 + k); }
                    for (int k = 0; k < 6; k++) { st[(PInvI + k) * Lanes + l] = Mem.At(invI, b6 + k); st[(PWInvI + k) * Lanes + l] = Mem.At(wInvI, b6 + k); }
                    st[PInvMass * Lanes + l] = Mem.At(invMass, b);
                }
            }

            for (int l0 = 0; l0 < Lanes; l0 += width) { PositionLanes<T>(st, l0, th); }
            for (int l = 0; l < n; l++)
            {
                int b = list[i0 + l];
                int b3 = 3 * b, b4 = 4 * b, b6 = 6 * b;
                pos[b3] += WorldCore.Grid(st[PLin * Lanes + l]); pos[b3 + 1] += WorldCore.Grid(st[(PLin + 1) * Lanes + l]); pos[b3 + 2] += WorldCore.Grid(st[(PLin + 2) * Lanes + l]);
                for (int k = 0; k < 3; k++) { Mem.At(dp, b3 + k) = st[(PDp + k) * Lanes + l]; }
                for (int k = 0; k < 4; k++) { Mem.At(dq, b4 + k) = st[(PDq + k) * Lanes + l]; Mem.At(rot, b4 + k) = st[(PRot + k) * Lanes + l]; }
                for (int k = 0; k < 6; k++) { Mem.At(wInvI, b6 + k) = st[(PWInvI + k) * Lanes + l]; }
            }

            // The packed inertia and pose rows: whole-row stores from the staging rows when the chunk was gathered as rows, else float by float.
            if (rows)
            {
                var ids = list.Slice(i0, Lanes);
                Vector256<float> z = Vector256<float>.Zero;
                Lanes256.ScatterRows(Lanes256.LoadAt(st, PWInvI * Lanes).AsVector256, Lanes256.LoadAt(st, (PWInvI + 1) * Lanes).AsVector256, Lanes256.LoadAt(st, (PWInvI + 2) * Lanes).AsVector256,
                    Lanes256.LoadAt(st, (PWInvI + 3) * Lanes).AsVector256, Lanes256.LoadAt(st, (PWInvI + 4) * Lanes).AsVector256, Lanes256.LoadAt(st, (PWInvI + 5) * Lanes).AsVector256, z, z,
                    pk, ref MemoryMarshal.GetReference(ids), Lanes, Engine3D.PkStride, Engine3D.PkInertia);
                Lanes256.ScatterRows(Lanes256.LoadAt(st, PDp * Lanes).AsVector256, Lanes256.LoadAt(st, (PDp + 1) * Lanes).AsVector256, Lanes256.LoadAt(st, (PDp + 2) * Lanes).AsVector256,
                    Lanes256.LoadAt(st, PDq * Lanes).AsVector256, Lanes256.LoadAt(st, (PDq + 1) * Lanes).AsVector256, Lanes256.LoadAt(st, (PDq + 2) * Lanes).AsVector256, Lanes256.LoadAt(st, (PDq + 3) * Lanes).AsVector256, z,
                    pk, ref MemoryMarshal.GetReference(ids), Lanes, Engine3D.PkStride, Engine3D.PkPose);
            }
            else
            {
                for (int l = 0; l < n; l++)
                {
                    int b = list[i0 + l];
                    int o = Engine3D.PkStride * b;
                    for (int k = 0; k < 6; k++) { Mem.At(pk, o + Engine3D.PkInertia + k) = st[(PWInvI + k) * Lanes + l]; }
                    for (int k = 0; k < 3; k++) { Mem.At(pk, o + Engine3D.PkPose + k) = st[(PDp + k) * Lanes + l]; }
                    for (int k = 0; k < 4; k++) { Mem.At(pk, o + Engine3D.PkPose + 3 + k) = st[(PDq + k) * Lanes + l]; }
                }
            }
        }
    }

    /// <summary><see cref="Sane"/> on the linear and the angular velocity of one packed velocity row (at <paramref name="at"/>), writing back a part that it changed.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SaneRow(Span<float> pk, int at, float speedCap, float spinCap)
    {
        // at is a body's packed velocity row (the body is in the awake list, below the high-water mark Pk was sized for).
        Mem.Range(pk, at, 6);
        float vx = Mem.At(pk, at), vy = Mem.At(pk, at + 1), vz = Mem.At(pk, at + 2), wx = Mem.At(pk, at + 3), wy = Mem.At(pk, at + 4), wz = Mem.At(pk, at + 5);
        if (Sane(ref vx, ref vy, ref vz, speedCap)) { Mem.At(pk, at) = vx; Mem.At(pk, at + 1) = vy; Mem.At(pk, at + 2) = vz; }
        if (Sane(ref wx, ref wy, ref wz, spinCap)) { Mem.At(pk, at + 3) = wx; Mem.At(pk, at + 4) = wy; Mem.At(pk, at + 5) = wz; }
    }
}
