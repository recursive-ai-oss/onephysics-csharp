using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Layout of one group of up to eight point (ball-socket) joints in <see cref="Engine3D.WideJ"/>: fields of eight floats (one per lane) so every field loads as one vector.
/// Per joint: the two anchors from the centres of mass and the centre-of-mass offset at the start of the step, both inverse masses, and the accumulated impulse and the inverse of the 3×3 block K (six entries, computed once per step); then the gathered body state and the
/// velocities on their way out, as for contacts (<see cref="W3D"/>).
/// </summary>
internal static class J3D
{
    public const int RA = 0, RB = 3, D = 6, InvMassA = 9, InvMassB = 10, Impulse = 11, InvK = 14;
    public const int Fields = 20;
    public const int Lanes = 8;
    public const int StageOffset = Fields * Lanes, StageFloats = 38 * 8, OutOffset = StageOffset + StageFloats, OutFloats = 12 * 8;
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
    private static T Field<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => T.Load(wide.Slice(f * 8 + lane0, T.Count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Vec<T>(ReadOnlySpan<float> wide, int f, int lane0) where T : struct, ILane<T> => new(Field<T>(wide, f, lane0), Field<T>(wide, f + 1, lane0), Field<T>(wide, f + 2, lane0));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Stage<T>(ReadOnlySpan<float> stage, int f) where T : struct, ILane<T> => T.Load(stage.Slice(f * T.Count, T.Count));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> StageVec<T>(ReadOnlySpan<float> stage, int f) where T : struct, ILane<T> => new(Stage<T>(stage, f), Stage<T>(stage, f + 1), Stage<T>(stage, f + 2));

    private static BodyLanes<T> LoadBody<T>(ReadOnlySpan<float> stage, int f0, T invMass) where T : struct, ILane<T> => new()
    {
        V = StageVec<T>(stage, f0), W = StageVec<T>(stage, f0 + 3), Dp = StageVec<T>(stage, f0 + 6),
        Qx = Stage<T>(stage, f0 + 9), Qy = Stage<T>(stage, f0 + 10), Qz = Stage<T>(stage, f0 + 11), Qw = Stage<T>(stage, f0 + 12),
        I = new Sym<T> { M0 = Stage<T>(stage, f0 + 13), M1 = Stage<T>(stage, f0 + 14), M2 = Stage<T>(stage, f0 + 15), M3 = Stage<T>(stage, f0 + 16), M4 = Stage<T>(stage, f0 + 17), M5 = Stage<T>(stage, f0 + 18) },
        InvMass = invMass,
    };

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
    /// Once per step, after a batch's lanes are filled: the inverse of K = (mA + mB)·1 + [rA]ᵀ·IA·[rA] + [rB]ᵀ·IB·[rB] at the start-of-step anchors, with the inverse inertias of the bodies at that time. A singular block (no dynamic mass) gets a zero inverse.
    /// </summary>
    public static void Prepare<T>(Engine3D e, int group, int n) where T : struct, ILane<T>
    {
        int width = T.Count;
        var wide = e.WideJ.AsSpan(group * J3D.GroupFloats, J3D.GroupFloats);
        var idx = e.WideJIdx.AsSpan(group * J3D.IdxPerGroup, J3D.IdxPerGroup);
        ReadOnlySpan<float> winv = e.WInvI;
        Span<float> stage = e.WideJ.AsSpan(group * J3D.GroupFloats + J3D.StageOffset, J3D.StageFloats);
        T zero = T.Zero;
        for (int l0 = 0; l0 < n; l0 += width)
        {
            for (int l = 0; l < width; l++)
            {
                for (int side = 0; side < 2; side++)
                {
                    int b = idx[8 * side + l0 + l];
                    for (int k = 0; k < 6; k++) { stage[(6 * side + k) * width + l] = winv[6 * b + k]; }
                }
            }

            var ia = new Sym<T> { M0 = Stage<T>(stage, 0), M1 = Stage<T>(stage, 1), M2 = Stage<T>(stage, 2), M3 = Stage<T>(stage, 3), M4 = Stage<T>(stage, 4), M5 = Stage<T>(stage, 5) };
            var ib = new Sym<T> { M0 = Stage<T>(stage, 6), M1 = Stage<T>(stage, 7), M2 = Stage<T>(stage, 8), M3 = Stage<T>(stage, 9), M4 = Stage<T>(stage, 10), M5 = Stage<T>(stage, 11) };
            var rA = Vec<T>(wide, J3D.RA, l0);
            var rB = Vec<T>(wide, J3D.RB, l0);
            T k00 = Field<T>(wide, J3D.InvMassA, l0) + Field<T>(wide, J3D.InvMassB, l0), k11 = k00, k22 = k00, k01 = zero, k02 = zero, k12 = zero;
            AddInertiaTerm(ia, rA, ref k00, ref k01, ref k02, ref k11, ref k12, ref k22);
            AddInertiaTerm(ib, rB, ref k00, ref k01, ref k02, ref k11, ref k12, ref k22);
            T c00 = k11 * k22 - k12 * k12, c01 = k02 * k12 - k01 * k22, c02 = k01 * k12 - k02 * k11;
            T det = k00 * c00 + k01 * c01 + k02 * c02;
            T nonZero = Lane.Or(T.GreaterThan(det, zero), T.GreaterThan(zero, det));
            T inv = T.Select(nonZero, T.Broadcast(1f) / det, zero);
            T c11 = k00 * k22 - k02 * k02, c12 = k01 * k02 - k00 * k12, c22 = k00 * k11 - k01 * k01;
            (inv * c00).Store(wide.Slice((J3D.InvK + 0) * 8 + l0, width)); (inv * c01).Store(wide.Slice((J3D.InvK + 1) * 8 + l0, width)); (inv * c02).Store(wide.Slice((J3D.InvK + 2) * 8 + l0, width));
            (inv * c11).Store(wide.Slice((J3D.InvK + 3) * 8 + l0, width)); (inv * c12).Store(wide.Slice((J3D.InvK + 4) * 8 + l0, width)); (inv * c22).Store(wide.Slice((J3D.InvK + 5) * 8 + l0, width));
        }
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
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> pk = e.Pk;
        Span<float> linW = bodies.LinearVelocity, angW = bodies.AngularVelocity;
        Span<float> stage = e.WideJ.AsSpan(group * J3D.GroupFloats + J3D.StageOffset, J3D.StageFloats);
        Span<float> os = e.WideJ.AsSpan(group * J3D.GroupFloats + J3D.OutOffset, J3D.OutFloats);
        T two = T.Broadcast(2f), zero = T.Zero;
        bool warm = phase == SolvePhase.WarmStart;
        bool useBias = phase == SolvePhase.Solve;
        // The relax pass solves the rows rigidly: no bias, full mass, no softening of the accumulated impulse.
        T ms = T.Broadcast(useBias ? kp.MassScale : 1f), kk = T.Broadcast(useBias ? kp.BiasTerm : 0f), isc = T.Broadcast(useBias ? kp.ImpulseScale : 0f);

        for (int l0 = lane0; l0 < lane0 + n; l0 += width)
        {
            for (int l = 0; l < width; l++)
            {
                for (int side = 0; side < 2; side++)
                {
                    int b = idx[8 * side + l0 + l];
                    int f = 19 * side;
                    stage[(f + 0) * width + l] = lin[3 * b]; stage[(f + 1) * width + l] = lin[3 * b + 1]; stage[(f + 2) * width + l] = lin[3 * b + 2];
                    stage[(f + 3) * width + l] = ang[3 * b]; stage[(f + 4) * width + l] = ang[3 * b + 1]; stage[(f + 5) * width + l] = ang[3 * b + 2];
                    var row = pk.Slice(16 * b, 13);
                    for (int k = 0; k < 13; k++) { stage[(f + 6 + k) * width + l] = row[k]; }
                }
            }

            var a = LoadBody<T>(stage, 0, Field<T>(wide, J3D.InvMassA, l0));
            var b2 = LoadBody<T>(stage, 19, Field<T>(wide, J3D.InvMassB, l0));
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
                // The block K is that of the start of the step (Catto, "Soft Step Solver": effective masses are prepared once per step); only the anchors follow the bodies' rotation.
                T i00 = Field<T>(wide, J3D.InvK, l0), i01 = Field<T>(wide, J3D.InvK + 1, l0), i02 = Field<T>(wide, J3D.InvK + 2, l0);
                T i11 = Field<T>(wide, J3D.InvK + 3, l0), i12 = Field<T>(wide, J3D.InvK + 4, l0), i22 = Field<T>(wide, J3D.InvK + 5, l0);
                var dv = (b2.V + V3<T>.Cross(b2.W, rB)) - (a.V + V3<T>.Cross(a.W, rA));
                T r0 = ms * dv.X + kk * c.X, r1 = ms * dv.Y + kk * c.Y, r2 = ms * dv.Z + kk * c.Z;
                T x0 = i00 * r0 + i01 * r1 + i02 * r2;
                T x1 = i01 * r0 + i11 * r1 + i12 * r2;
                T x2 = i02 * r0 + i12 * r1 + i22 * r2;
                var step = new V3<T>(T.Negate(x0) - isc * imp.X, T.Negate(x1) - isc * imp.Y, T.Negate(x2) - isc * imp.Z);
                imp = imp + step;
                Apply(ref a, ref b2, rA, rB, step);
            }

            a.V.X.Store(os.Slice(0 * width, width)); a.V.Y.Store(os.Slice(1 * width, width)); a.V.Z.Store(os.Slice(2 * width, width));
            a.W.X.Store(os.Slice(3 * width, width)); a.W.Y.Store(os.Slice(4 * width, width)); a.W.Z.Store(os.Slice(5 * width, width));
            b2.V.X.Store(os.Slice(6 * width, width)); b2.V.Y.Store(os.Slice(7 * width, width)); b2.V.Z.Store(os.Slice(8 * width, width));
            b2.W.X.Store(os.Slice(9 * width, width)); b2.W.Y.Store(os.Slice(10 * width, width)); b2.W.Z.Store(os.Slice(11 * width, width));
            for (int l = 0; l < width && l0 + l < lane0 + n; l++)
            {
                // Scatter offsets are body indices of dynamic bodies, or -1 for a body that keeps its velocity.
                int ia = idx[16 + l0 + l];
                if (ia >= 0)
                {
                    linW[3 * ia] = os[l]; linW[3 * ia + 1] = os[width + l]; linW[3 * ia + 2] = os[2 * width + l];
                    angW[3 * ia] = os[3 * width + l]; angW[3 * ia + 1] = os[4 * width + l]; angW[3 * ia + 2] = os[5 * width + l];
                }

                int ib = idx[24 + l0 + l];
                if (ib >= 0)
                {
                    linW[3 * ib] = os[6 * width + l]; linW[3 * ib + 1] = os[7 * width + l]; linW[3 * ib + 2] = os[8 * width + l];
                    angW[3 * ib] = os[9 * width + l]; angW[3 * ib + 1] = os[10 * width + l]; angW[3 * ib + 2] = os[11 * width + l];
                }
            }

            if (warm) { continue; }
            imp.X.Store(wide.Slice((J3D.Impulse + 0) * 8 + l0, width)); imp.Y.Store(wide.Slice((J3D.Impulse + 1) * 8 + l0, width)); imp.Z.Store(wide.Slice((J3D.Impulse + 2) * 8 + l0, width));
        }
    }
}
