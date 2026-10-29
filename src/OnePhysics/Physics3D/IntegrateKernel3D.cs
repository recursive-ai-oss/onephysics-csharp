using System;
using System.Runtime.CompilerServices;
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

    /// <summary>One group of up to eight bodies: velocities after gravity, forces, torque, the gyroscopic term, damping and the speed limits. Lanes with <c>write[lane]</c> false are not written back.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void VelocityLanes<T>(Span<float> st, int l0, T hh, T maxSpeed, T maxSpeed2, T maxSpin, T maxSpin2, T gx, T gy, T gz) where T : struct, ILane<T>
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
        vx = vx * dl; vy = vy * dl; vz = vz * dl;
        w = new V3<T>(w.X * da, w.Y * da, w.Z * da);
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
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
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

                int b3 = 3 * b, b6 = 6 * b, b4 = 4 * b;
                st[VLinX * Lanes + l] = Mem.At(lin, b3); st[(VLinX + 1) * Lanes + l] = Mem.At(lin, b3 + 1); st[(VLinX + 2) * Lanes + l] = Mem.At(lin, b3 + 2);
                st[VAngX * Lanes + l] = Mem.At(ang, b3); st[(VAngX + 1) * Lanes + l] = Mem.At(ang, b3 + 1); st[(VAngX + 2) * Lanes + l] = Mem.At(ang, b3 + 2);
                st[VGravityScale * Lanes + l] = Mem.At(gravityScale, b); st[VInvMass * Lanes + l] = Mem.At(invMass, b);
                st[VForce * Lanes + l] = Mem.At(force, b3); st[(VForce + 1) * Lanes + l] = Mem.At(force, b3 + 1); st[(VForce + 2) * Lanes + l] = Mem.At(force, b3 + 2);
                st[VTorque * Lanes + l] = Mem.At(torque, b3); st[(VTorque + 1) * Lanes + l] = Mem.At(torque, b3 + 1); st[(VTorque + 2) * Lanes + l] = Mem.At(torque, b3 + 2);
                st[VLinDamp * Lanes + l] = Mem.At(linDamping, b); st[VAngDamp * Lanes + l] = Mem.At(angDamping, b);
                for (int k = 0; k < 6; k++) { st[(VWInvI + k) * Lanes + l] = Mem.At(wInvI, b6 + k); }
                bool gyro = (flags[b] & 2) != 0;
                st[VGyro * Lanes + l] = gyro ? allOnes : 0f;
                for (int k = 0; k < 4; k++) { st[(VRot + k) * Lanes + l] = gyro ? Mem.At(rot, b4 + k) : 0f; }
                for (int k = 0; k < 6; k++) { st[(VInertia + k) * Lanes + l] = gyro ? Mem.At(inertia, b6 + k) : 0f; }
            }

            if (live == 0) { continue; }
            for (int l0 = 0; l0 < Lanes; l0 += width) { VelocityLanes<T>(st, l0, hh, ms, ms2, mw, mw2, gx, gy, gz); }
            for (int l = 0; l < Lanes; l++)
            {
                int b = ids[l];
                if (b < 0) { continue; }
                int b3 = 3 * b;
                Mem.At(lin, b3) = st[VLinX * Lanes + l]; Mem.At(lin, b3 + 1) = st[(VLinX + 1) * Lanes + l]; Mem.At(lin, b3 + 2) = st[(VLinX + 2) * Lanes + l];
                Mem.At(ang, b3) = st[VAngX * Lanes + l]; Mem.At(ang, b3 + 1) = st[(VAngX + 1) * Lanes + l]; Mem.At(ang, b3 + 2) = st[(VAngX + 2) * Lanes + l];
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

    public static void Positions<T>(Engine3D e, ReadOnlySpan<int> list, float h) where T : struct, ILane<T>
    {
        int width = T.Count;
        var bodies = e.Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        Span<double> pos = bodies.Position;
        Span<float> rot = bodies.Rotation, dp = e.Dp, dq = e.Dq, wInvI = e.WInvI, pk = e.Pk;
        Span<float> st = stackalloc float[PFields * Lanes];
        T th = T.Broadcast(h);
        // As in Velocities: the list holds bodies below the high-water mark and every per-body array was sized for it, so the unchecked accessors stay in range (verified in Checked).
        for (int i0 = 0; i0 < list.Length; i0 += Lanes)
        {
            int n = Math.Min(Lanes, list.Length - i0);
            for (int l = 0; l < Lanes; l++)
            {
                if (l >= n)
                {
                    for (int f = 0; f < PFields; f++) { st[f * Lanes + l] = 0f; }
                    continue;
                }

                int b = list[i0 + l];
                int b3 = 3 * b, b4 = 4 * b, b6 = 6 * b;
                for (int k = 0; k < 3; k++)
                {
                    st[(PLin + k) * Lanes + l] = Mem.At(lin, b3 + k); st[(PAng + k) * Lanes + l] = Mem.At(ang, b3 + k); st[(PDp + k) * Lanes + l] = Mem.At(dp, b3 + k);
                }

                for (int k = 0; k < 4; k++) { st[(PDq + k) * Lanes + l] = Mem.At(dq, b4 + k); st[(PRot + k) * Lanes + l] = Mem.At(rot, b4 + k); }
                for (int k = 0; k < 6; k++) { st[(PInvI + k) * Lanes + l] = Mem.At(invI, b6 + k); st[(PWInvI + k) * Lanes + l] = Mem.At(wInvI, b6 + k); }
                st[PInvMass * Lanes + l] = Mem.At(invMass, b);
            }

            for (int l0 = 0; l0 < Lanes; l0 += width) { PositionLanes<T>(st, l0, th); }
            for (int l = 0; l < n; l++)
            {
                int b = list[i0 + l];
                int b3 = 3 * b, b4 = 4 * b, b6 = 6 * b, o = 16 * b;
                pos[b3] += WorldCore.Grid(st[PLin * Lanes + l]); pos[b3 + 1] += WorldCore.Grid(st[(PLin + 1) * Lanes + l]); pos[b3 + 2] += WorldCore.Grid(st[(PLin + 2) * Lanes + l]);
                for (int k = 0; k < 3; k++) { Mem.At(dp, b3 + k) = st[(PDp + k) * Lanes + l]; Mem.At(pk, o + k) = st[(PDp + k) * Lanes + l]; }
                for (int k = 0; k < 4; k++) { Mem.At(dq, b4 + k) = st[(PDq + k) * Lanes + l]; Mem.At(rot, b4 + k) = st[(PRot + k) * Lanes + l]; Mem.At(pk, o + 3 + k) = st[(PDq + k) * Lanes + l]; }
                for (int k = 0; k < 6; k++) { Mem.At(wInvI, b6 + k) = st[(PWInvI + k) * Lanes + l]; Mem.At(pk, o + 7 + k) = st[(PWInvI + k) * Lanes + l]; }
            }
        }
    }
}
