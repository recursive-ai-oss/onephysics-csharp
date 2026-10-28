using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Layout of one group of up to eight point (ball-socket) joints in <see cref="Engine3D.WideJ"/>: fields of eight floats (one per lane) so every field loads as one vector.
/// Per joint: the two anchors from the centres of mass and the centre-of-mass offset at the start of the step, both inverse masses, and the accumulated impulse; then the gathered body state and the
/// velocities on their way out, as for contacts (<see cref="W3D"/>).
/// </summary>
internal static class J3D
{
    public const int RA = 0, RB = 3, D = 6, InvMassA = 9, InvMassB = 10, Impulse = 11;
    public const int Fields = 14;
    public const int Lanes = 8;
    public const int StageOffset = Fields * Lanes, StageFloats = 0, OutOffset = StageOffset + StageFloats, OutFloats = 12 * 8;
    public const int GroupFloats = OutOffset + OutFloats;

    /// <summary>Integers per group in <see cref="Engine3D.WideJIdx"/>: body rows of A and B, then the same rows for bodies the kernel writes (or -1), eight lanes each.</summary>
    public const int IdxPerGroup = 32;
}

/// <summary>Soft coefficients of a joint's rigid rows for the solve pass: impulse = -K⁻¹ (Ms·Cdot + Kk·C) - Isc·accumulated (Catto, "Soft Step Solver", GDC 2024).</summary>
internal readonly struct JointKernelParams
{
    public readonly float MassScale, BiasTerm, ImpulseScale;

    public JointKernelParams(float massScale, float biasTerm, float impulseScale) { MassScale = massScale; BiasTerm = biasTerm; ImpulseScale = impulseScale; }
}

/// <summary>
/// The 3D point-joint solver kernel, written once over <see cref="ILane{T}"/> like <see cref="ContactKernel3D"/>: a batch of up to eight point joints of one color is gathered into lanes, solved with
/// the same operations in the same order at every width and scattered back. The three rows (the anchors coincide) are solved together as one 3×3 block through the closed-form inverse of
/// K = (mA + mB)·1 + [rA]ᵀ·IA·[rA] + [rB]ᵀ·IB·[rB], where [r] is the cross-product matrix. The joints of a batch touch disjoint dynamic bodies, so lanes never depend on each other.
/// </summary>
internal static class JointKernel3D
{
    private struct V3<T> where T : struct, ILane<T>
    {
        public T X, Y, Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public V3(T x, T y, T z) { X = x; Y = y; Z = z; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> operator +(V3<T> a, V3<T> b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> operator -(V3<T> a, V3<T> b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> operator *(V3<T> a, T s) => new(a.X * s, a.Y * s, a.Z * s);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static V3<T> Cross(V3<T> a, V3<T> b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static T Dot(V3<T> a, V3<T> b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    }

    /// <summary>A symmetric 3×3 tensor stored as xx xy xz yy yz zz.</summary>
    private struct Sym<T> where T : struct, ILane<T>
    {
        public T M0, M1, M2, M3, M4, M5;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public V3<T> Apply(V3<T> v) => new(M0 * v.X + M1 * v.Y + M2 * v.Z, M1 * v.X + M3 * v.Y + M4 * v.Z, M2 * v.X + M4 * v.Y + M5 * v.Z);
    }

    private struct BodyLanes<T> where T : struct, ILane<T>
    {
        public V3<T> V, W, Dp;
        public T Qx, Qy, Qz, Qw;
        public Sym<T> I;
        public T InvMass;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Field<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.LoadAt(wide, f * 8 + lane0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Vec<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => new(Field<T>(wide, f, lane0), Field<T>(wide, f + 1, lane0), Field<T>(wide, f + 2, lane0));

    /// <summary>
    /// The state of the bodies on one side of a batch, one body per lane, by whole-row loads and a transpose (the same data movement as the contact kernel's): velocities from the velocity arrays,
    /// displacement, accumulated rotation and world inverse inertia from the packed rows (<see cref="Engine3D.Pk"/>).
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GatherBody<T>(out BodyLanes<T> r, ReadOnlySpan<float> lin, ReadOnlySpan<float> ang, ReadOnlySpan<float> pk, ReadOnlySpan<int> bodyIds, Span<int> offs, T invMass) where T : struct, ILane<T>
    {
        int width = T.Count;
        for (int l = 0; l < width; l++) { offs[l] = 3 * bodyIds[l]; }
        var v = T.GatherRows(lin, offs);
        var w = T.GatherRows(ang, offs);
        for (int l = 0; l < width; l++) { offs[l] = 16 * bodyIds[l]; }
        var g0 = T.GatherRows(pk, offs);
        for (int l = 0; l < width; l++) { offs[l] += 8; }
        var g1 = T.GatherRows(pk, offs);
        r = new BodyLanes<T>
        {
            V = new V3<T>(v.R0, v.R1, v.R2), W = new V3<T>(w.R0, w.R1, w.R2), Dp = new V3<T>(g0.R0, g0.R1, g0.R2),
            Qx = g0.R3, Qy = g0.R4, Qz = g0.R5, Qw = g0.R6,
            I = new Sym<T> { M0 = g0.R7, M1 = g1.R0, M2 = g1.R1, M3 = g1.R2, M4 = g1.R3, M5 = g1.R4 },
            InvMass = invMass,
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ScatterBody(Span<float> lin, Span<float> ang, ReadOnlySpan<float> os, int at, int first, int width)
    {
        // at is a live dynamic body's slot (3 floats a body, set when the batch was prepared) and first + 5 * width + 1 stays inside the group's output rows.
        Mem.Range(lin, at, 3); Mem.Range(ang, at, 3); Mem.Range(os, first, 5 * width + 1);
        Mem.At(lin, at) = Mem.At(os, first); Mem.At(lin, at + 1) = Mem.At(os, first + width); Mem.At(lin, at + 2) = Mem.At(os, first + 2 * width);
        Mem.At(ang, at) = Mem.At(os, first + 3 * width); Mem.At(ang, at + 1) = Mem.At(os, first + 4 * width); Mem.At(ang, at + 2) = Mem.At(os, first + 5 * width);
    }

    /// <summary>q v q⁻¹ for a unit quaternion: v + w·t + u × t with t = 2 u × v.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Rotate<T>(T x, T y, T z, T w, V3<T> v, T two) where T : struct, ILane<T>
    {
        T tx = two * (y * v.Z - z * v.Y), ty = two * (z * v.X - x * v.Z), tz = two * (x * v.Y - y * v.X);
        return new V3<T>(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    /// <summary>The anchor impulse p acts on A with the opposite sign: A loses momentum p at its anchor, B gains it.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Apply<T>(ref BodyLanes<T> a, ref BodyLanes<T> b, V3<T> rA, V3<T> rB, V3<T> p) where T : struct, ILane<T>
    {
        a.V = a.V - p * a.InvMass;
        a.W = a.W - a.I.Apply(V3<T>.Cross(rA, p));
        b.V = b.V + p * b.InvMass;
        b.W = b.W + b.I.Apply(V3<T>.Cross(rB, p));
    }

    /// <summary>The 3×3 block K of the point constraint, one inertia term per body: column i of [r]ᵀ·I·[r] is I applied to r × eᵢ, dotted with r × eⱼ.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddInertiaTerm<T>(Sym<T> inertia, V3<T> r, ref T k00, ref T k01, ref T k02, ref T k11, ref T k12, ref T k22) where T : struct, ILane<T>
    {
        T zero = T.Zero;
        var cx = new V3<T>(zero, r.Z, T.Negate(r.Y)); // r × e_x
        var cy = new V3<T>(T.Negate(r.Z), zero, r.X); // r × e_y
        var cz = new V3<T>(r.Y, T.Negate(r.X), zero); // r × e_z
        var wx = inertia.Apply(cx);
        var wy = inertia.Apply(cy);
        var wz = inertia.Apply(cz);
        k00 = k00 + V3<T>.Dot(cx, wx);
        k01 = k01 + V3<T>.Dot(cx, wy);
        k02 = k02 + V3<T>.Dot(cx, wz);
        k11 = k11 + V3<T>.Dot(cy, wy);
        k12 = k12 + V3<T>.Dot(cy, wz);
        k22 = k22 + V3<T>.Dot(cz, wz);
    }

    /// <summary>
    /// Runs lanes <paramref name="lane0"/> to <paramref name="lane0"/> + <paramref name="n"/> - 1 of a group, <c>T.Count</c> at a time. Lanes past the end of the batch compute on whatever the group holds and are never written back.
    /// The restitution pass does nothing for joints.
    /// </summary>
    public static void Run<T>(Engine3D e, int group, int lane0, int n, SolvePhase phase, in JointKernelParams kp) where T : struct, ILane<T>
    {
        if (phase == SolvePhase.Restitution) { return; }
        int width = T.Count;
        var wide = e.WideJ.AsSpan(group * J3D.GroupFloats, J3D.GroupFloats);
        var idx = e.WideJIdx.AsSpan(group * J3D.IdxPerGroup, J3D.IdxPerGroup);
        var bodies = e.Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocityPadded, ang = bodies.AngularVelocityPadded;
        ReadOnlySpan<float> pk = e.Pk;
        Span<int> offs = stackalloc int[8];
        Span<float> linW = bodies.LinearVelocity, angW = bodies.AngularVelocity;
        Span<float> os = e.WideJ.AsSpan(group * J3D.GroupFloats + J3D.OutOffset, J3D.OutFloats);
        T two = T.Broadcast(2f), zero = T.Zero;
        bool warm = phase == SolvePhase.WarmStart;
        bool useBias = phase == SolvePhase.Solve;
        // The relax pass solves the rows rigidly: no bias, full mass, no softening of the accumulated impulse.
        T ms = T.Broadcast(useBias ? kp.MassScale : 1f), kk = T.Broadcast(useBias ? kp.BiasTerm : 0f), isc = T.Broadcast(useBias ? kp.ImpulseScale : 0f);

        for (int l0 = lane0; l0 < lane0 + n; l0 += width)
        {
            GatherBody<T>(out var a, lin, ang, pk, idx.Slice(l0, width), offs, Field<T>(wide, J3D.InvMassA, l0));
            GatherBody<T>(out var b2, lin, ang, pk, idx.Slice(8 + l0, width), offs, Field<T>(wide, J3D.InvMassB, l0));
            // The anchors turn with their bodies' rotation accumulated this step; the separation of the anchors follows from the bodies' displacements.
            var rA = Rotate(a.Qx, a.Qy, a.Qz, a.Qw, Vec<T>(wide, J3D.RA, l0), two);
            var rB = Rotate(b2.Qx, b2.Qy, b2.Qz, b2.Qw, Vec<T>(wide, J3D.RB, l0), two);
            var imp = Vec<T>(wide, J3D.Impulse, l0);

            if (warm)
            {
                Apply(ref a, ref b2, rA, rB, imp);
            }
            else
            {
                var d = Vec<T>(wide, J3D.D, l0) + (b2.Dp - a.Dp);
                var c = (d + rB) - rA;
                T k00 = a.InvMass + b2.InvMass, k11 = k00, k22 = k00, k01 = zero, k02 = zero, k12 = zero;
                AddInertiaTerm(a.I, rA, ref k00, ref k01, ref k02, ref k11, ref k12, ref k22);
                AddInertiaTerm(b2.I, rB, ref k00, ref k01, ref k02, ref k11, ref k12, ref k22);
                T c00 = k11 * k22 - k12 * k12, c01 = k02 * k12 - k01 * k22, c02 = k01 * k12 - k02 * k11;
                T det = k00 * c00 + k01 * c01 + k02 * c02;
                T nonZero = Lane.Or(T.GreaterThan(det, zero), T.GreaterThan(zero, det));
                T inv = T.Select(nonZero, T.Broadcast(1f) / det, zero);
                T c11 = k00 * k22 - k02 * k02, c12 = k01 * k02 - k00 * k12, c22 = k00 * k11 - k01 * k01;
                var dv = (b2.V + V3<T>.Cross(b2.W, rB)) - (a.V + V3<T>.Cross(a.W, rA));
                T r0 = ms * dv.X + kk * c.X, r1 = ms * dv.Y + kk * c.Y, r2 = ms * dv.Z + kk * c.Z;
                T x0 = inv * (c00 * r0 + c01 * r1 + c02 * r2);
                T x1 = inv * (c01 * r0 + c11 * r1 + c12 * r2);
                T x2 = inv * (c02 * r0 + c12 * r1 + c22 * r2);
                var step = new V3<T>(T.Negate(x0) - isc * imp.X, T.Negate(x1) - isc * imp.Y, T.Negate(x2) - isc * imp.Z);
                imp = imp + step;
                Apply(ref a, ref b2, rA, rB, step);
            }

            a.V.X.StoreAt(os, 0 * width); a.V.Y.StoreAt(os, 1 * width); a.V.Z.StoreAt(os, 2 * width);
            a.W.X.StoreAt(os, 3 * width); a.W.Y.StoreAt(os, 4 * width); a.W.Z.StoreAt(os, 5 * width);
            b2.V.X.StoreAt(os, 6 * width); b2.V.Y.StoreAt(os, 7 * width); b2.V.Z.StoreAt(os, 8 * width);
            b2.W.X.StoreAt(os, 9 * width); b2.W.Y.StoreAt(os, 10 * width); b2.W.Z.StoreAt(os, 11 * width);
            for (int l = 0; l < width && l0 + l < lane0 + n; l++)
            {
                // Scatter offsets are body indices of dynamic bodies, or -1 for a body that keeps its velocity.
                int ia = idx[16 + l0 + l];
                if (ia >= 0) { ScatterBody(linW, angW, os, 3 * ia, l, width); }
                int ib = idx[24 + l0 + l];
                if (ib >= 0) { ScatterBody(linW, angW, os, 3 * ib, l + 6 * width, width); }
            }

            if (warm) { continue; }
            imp.X.StoreAt(wide, (J3D.Impulse + 0) * 8 + l0); imp.Y.StoreAt(wide, (J3D.Impulse + 1) * 8 + l0); imp.Z.StoreAt(wide, (J3D.Impulse + 2) * 8 + l0);
        }
    }
}
