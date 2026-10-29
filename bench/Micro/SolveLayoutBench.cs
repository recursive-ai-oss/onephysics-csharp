using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Bench;

/// <summary>
/// One contact solve pass (friction rows, then normal rows, soft step with bias, no rolling resistance) over the batches of a settled pile, in two layouts.
/// A is the library's kernel as it is: the group rows plus a per-pass gather of velocity and the packed pose/inertia rows of both bodies, and the separation worked out from the pose each pass.
/// B is a lane-major layout written once by a prepare step: the group also carries the two bodies' inverse inertia and the current separation of every point, so the pass streams its group and gathers
/// only a six-float velocity row per body (linear, angular, two spare floats), with the gather offsets precomputed. Both use the same arithmetic in the same order; B's results are compared with A's bit for bit.
/// Usage: Micro --solve-layout [steps] [passes] [repeats]
/// </summary>
internal static class SolveLayoutBench
{
    // ---- B layout: field-major rows of eight lanes.
    private const int N = 0, T1 = 3, T2 = 6, Fr = 9, MA = 10, MB = 11, Bias = 12, MS = 13, IS = 14, IA = 15, IB = 21, P0 = 27, PS = 15;
    // point fields
    private const int RA = 0, RB = 3, Sep = 6, NM = 7, T1M = 8, T2M = 9, NI = 10, T1I = 11, T2I = 12, MaxN = 13, Mask = 14;
    private const int Rows = P0 + 4 * PS, GroupB = Rows * 8 + 12 * 8; // plus 12 rows of staging for outgoing velocities
    private const int OutB = Rows * 8;

    public static int Run(string[] args)
    {
        int steps = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : 400;
        int passes = args.Length > 2 ? int.Parse(args[2], CultureInfo.InvariantCulture) : 20;
        int repeats = args.Length > 3 ? int.Parse(args[3], CultureInfo.InvariantCulture) : 15;

        var world = BuildPile(steps);
        var e = world.Engine;
        var core = e.Core; var cs = core.Constraints; var bodies = core.Bodies;

        // Groups of this step: lanes packed from 0, taken from the constraint-to-slot map. Groups holding overflow-set constraints (which may share bodies) are left out.
        var laneCount = new Dictionary<int, int>(); var bad = new HashSet<int>();
        int manifolds = 0, points = 0;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) == ConstraintKind.Joint || cs.PointCount[c] == 0 || e.SlotOf[c] < 0) { continue; }
            int slot = e.SlotOf[c], g = slot >> 3;
            if (cs.Color[c] == ConstraintStore.Overflow) { bad.Add(g); continue; }
            laneCount[g] = Math.Max(laneCount.GetValueOrDefault(g), (slot & 7) + 1);
        }
        var groups = new List<int>(); var counts = new List<int>();
        foreach (var kv in laneCount) { if (!bad.Contains(kv.Key)) { groups.Add(kv.Key); } }
        groups.Sort();
        foreach (int g in groups) { counts.Add(laneCount[g]); }
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) == ConstraintKind.Joint || cs.PointCount[c] == 0 || e.SlotOf[c] < 0) { continue; }
            if (laneCount.ContainsKey(e.SlotOf[c] >> 3) && !bad.Contains(e.SlotOf[c] >> 3)) { manifolds++; points += cs.PointCount[c]; }
        }

        int nb = bodies.HighWater, ng = groups.Count;
        // The pass without rolling resistance in both layouts.
        foreach (int g in groups) { e.WideIdx[g * W3D.IdxPerGroup + W3D.AnyRollingSlot] = 0; }
        var kp = new KernelParams(60f * e.Opt.Substeps, e.Opt.PushOutVelocity, e.Opt.ContactSlop, e.Opt.RestitutionThreshold);
        bool bias = true;

        // Saved start state.
        var wide0 = (float[])e.Wide.Clone();
        var lin0 = bodies.LinearVelocity.ToArray(); var ang0 = bodies.AngularVelocity.ToArray();

        // ---- B: prepare once.
        var gb = new float[ng * GroupB];
        var offA = new int[ng * 8]; var offB = new int[ng * 8]; var scA = new int[ng * 8]; var scB = new int[ng * 8]; var maxPts = new int[ng];
        var vel = new float[8 * nb];
        long t0 = Stopwatch.GetTimestamp();
        for (int i = 0; i < ng; i++) { Prepare<Lanes256>(e, groups[i], gb.AsSpan(i * GroupB, GroupB), offA.AsSpan(i * 8, 8), offB.AsSpan(i * 8, 8), scA.AsSpan(i * 8, 8), scB.AsSpan(i * 8, 8), out maxPts[i]); }
        double prepareMs = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
        void ResetB()
        {
            for (int b = 0; b < nb; b++) { for (int k = 0; k < 3; k++) { vel[8 * b + k] = lin0[3 * b + k]; vel[8 * b + 3 + k] = ang0[3 * b + k]; } vel[8 * b + 6] = 0; vel[8 * b + 7] = 0; }
            // The impulses start from the saved group.
            for (int i = 0; i < ng; i++) { CopyImpulses(wide0.AsSpan(groups[i] * W3D.GroupFloats, W3D.GroupFloats), gb.AsSpan(i * GroupB, GroupB), maxPts[i]); }
        }

        void ResetA()
        {
            wide0.CopyTo(e.Wide, 0); lin0.AsSpan().CopyTo(e.Core.Bodies.LinearVelocity); ang0.AsSpan().CopyTo(e.Core.Bodies.AngularVelocity);
        }

        // ---- Equality: a few passes of each from the same start.
        ResetA(); for (int p = 0; p < 3; p++) { PassA(e, groups, counts, kp); }
        var aLin = bodies.LinearVelocity.ToArray(); var aAng = bodies.AngularVelocity.ToArray(); var aWide = (float[])e.Wide.Clone();
        ResetB(); for (int p = 0; p < 3; p++) { PassB<Lanes256>(gb, vel, offA, offB, scA, scB, maxPts, counts, ng, kp, bias); }
        int diff = 0, checkedVals = 0;
        for (int b = 0; b < nb; b++)
        {
            for (int k = 0; k < 3; k++)
            {
                checkedVals += 2;
                if (BitConverter.SingleToInt32Bits(vel[8 * b + k]) != BitConverter.SingleToInt32Bits(aLin[3 * b + k])) { diff++; }
                if (BitConverter.SingleToInt32Bits(vel[8 * b + 3 + k]) != BitConverter.SingleToInt32Bits(aAng[3 * b + k])) { diff++; }
            }
        }

        for (int i = 0; i < ng; i++)
        {
            var a = aWide.AsSpan(groups[i] * W3D.GroupFloats, W3D.GroupFloats); var b = gb.AsSpan(i * GroupB, GroupB);
            for (int pt = 0; pt < maxPts[i]; pt++)
            {
                for (int l = 0; l < counts[i]; l++)
                {
                    int fa = W3D.Point0 + pt * W3D.PointStride, fb = P0 + pt * PS;
                    checkedVals += 4;
                    if (BitConverter.SingleToInt32Bits(a[(fa + W3D.NormalImpulse) * 8 + l]) != BitConverter.SingleToInt32Bits(b[(fb + NI) * 8 + l])) { diff++; }
                    if (BitConverter.SingleToInt32Bits(a[(fa + W3D.Tangent1Impulse) * 8 + l]) != BitConverter.SingleToInt32Bits(b[(fb + T1I) * 8 + l])) { diff++; }
                    if (BitConverter.SingleToInt32Bits(a[(fa + W3D.Tangent2Impulse) * 8 + l]) != BitConverter.SingleToInt32Bits(b[(fb + T2I) * 8 + l])) { diff++; }
                    if (BitConverter.SingleToInt32Bits(a[(fa + W3D.MaxNormalImpulse) * 8 + l]) != BitConverter.SingleToInt32Bits(b[(fb + MaxN) * 8 + l])) { diff++; }
                }
            }
        }

        // Paired groups: same results required.
        var plan = PairPlan(e, groups); int pairs = 0; foreach (bool bpair in plan) { if (bpair) { pairs++; } }
        ResetB(); for (int p = 0; p < 3; p++) { PassB2<Lanes256>(gb, vel, offA, offB, scA, scB, maxPts, counts, ng, plan, kp); }
        int diff2 = 0;
        for (int b = 0; b < nb; b++) { for (int k = 0; k < 3; k++) { if (BitConverter.SingleToInt32Bits(vel[8 * b + k]) != BitConverter.SingleToInt32Bits(aLin[3 * b + k])) { diff2++; } if (BitConverter.SingleToInt32Bits(vel[8 * b + 3 + k]) != BitConverter.SingleToInt32Bits(aAng[3 * b + k])) { diff2++; } } }
        double bestP = double.MaxValue;
        for (int r = 0; r < repeats; r++)
        {
            ResetB();
            long s = Stopwatch.GetTimestamp();
            for (int p = 0; p < passes; p++) { PassB2<Lanes256>(gb, vel, offA, offB, scA, scB, maxPts, counts, ng, plan, kp); }
            bestP = Math.Min(bestP, Stopwatch.GetElapsedTime(s).TotalMilliseconds / passes);
        }

        // ---- Timing, interleaved, best of repeats.
        double bestA = double.MaxValue, bestB = double.MaxValue;
        for (int r = 0; r < repeats + 2; r++)
        {
            ResetA();
            long s = Stopwatch.GetTimestamp();
            for (int p = 0; p < passes; p++) { PassA(e, groups, counts, kp); }
            double ta = Stopwatch.GetElapsedTime(s).TotalMilliseconds / passes;
            ResetB();
            s = Stopwatch.GetTimestamp();
            for (int p = 0; p < passes; p++) { PassB<Lanes256>(gb, vel, offA, offB, scA, scB, maxPts, counts, ng, kp, bias); }
            double tb = Stopwatch.GetElapsedTime(s).TotalMilliseconds / passes;
            if (r >= 2) { bestA = Math.Min(bestA, ta); bestB = Math.Min(bestB, tb); }
        }

        // What binds B: the same pass with the velocity gathers pointing at one hot row, with the scatters off, and both (results are meaningless; only the time is read).
        var zeroOff = new int[ng * 8]; var noScatter = new int[ng * 8]; Array.Fill(noScatter, -1);
        double bestNoGather = double.MaxValue, bestNoScatter = double.MaxValue, bestNeither = double.MaxValue;
        for (int r = 0; r < repeats; r++)
        {
            ResetB();
            long s = Stopwatch.GetTimestamp();
            for (int p = 0; p < passes; p++) { PassB<Lanes256>(gb, vel, zeroOff, zeroOff, scA, scB, maxPts, counts, ng, kp, bias); }
            bestNoGather = Math.Min(bestNoGather, Stopwatch.GetElapsedTime(s).TotalMilliseconds / passes);
            ResetB();
            s = Stopwatch.GetTimestamp();
            for (int p = 0; p < passes; p++) { PassB<Lanes256>(gb, vel, offA, offB, noScatter, noScatter, maxPts, counts, ng, kp, bias); }
            bestNoScatter = Math.Min(bestNoScatter, Stopwatch.GetElapsedTime(s).TotalMilliseconds / passes);
            ResetB();
            s = Stopwatch.GetTimestamp();
            for (int p = 0; p < passes; p++) { PassB<Lanes256>(gb, vel, zeroOff, zeroOff, noScatter, noScatter, maxPts, counts, ng, kp, bias); }
            bestNeither = Math.Min(bestNeither, Stopwatch.GetElapsedTime(s).TotalMilliseconds / passes);
        }

        double bestR = double.MaxValue;
        for (int r = 0; r < repeats; r++)
        {
            long s = Stopwatch.GetTimestamp();
            for (int i = 0; i < ng; i++) { Refresh<Lanes256>(e, groups[i], gb.AsSpan(i * GroupB, GroupB), maxPts[i]); }
            bestR = Math.Min(bestR, Stopwatch.GetElapsedTime(s).TotalMilliseconds);
        }

        int sumPts = 0; for (int i = 0; i < ng; i++) { sumPts += maxPts[i]; }
        double avgMax = (double)sumPts / ng;
        Console.WriteLine($"solve layout: {ng} groups, {manifolds} manifolds, {points} points (largest point count per group {avgMax:F2} on average), {nb} bodies; {passes} passes per repeat, best of {repeats}");
        Console.WriteLine($"equality after 3 passes: {checkedVals} values compared, {diff} differ");
        Console.WriteLine($"B prepare (one-off, all groups): {prepareMs:F3} ms");
        Console.WriteLine($"A: {bestA:F4} ms per pass, {bestA * 1e6 / manifolds:F2} ns per manifold, {bestA * 1e6 / points:F2} ns per point, {bestA * 1e6 / ng:F1} ns per group");
        Console.WriteLine($"B: {bestB:F4} ms per pass, {bestB * 1e6 / manifolds:F2} ns per manifold, {bestB * 1e6 / points:F2} ns per point, {bestB * 1e6 / ng:F1} ns per group");
        Console.WriteLine($"A / B = {bestA / bestB:F3}");
        Console.WriteLine($"B with {pairs} disjoint pairs of groups interleaved: {bestP:F4} ms per pass, {bestP * 1e6 / ng:F1} ns per group, vs B alone {bestB / bestP:F3}x; velocities differing from A: {diff2}");
        Console.WriteLine($"B knockouts, ns per group: gathers on one hot row {bestNoGather * 1e6 / ng:F1}, scatters off {bestNoScatter * 1e6 / ng:F1}, both {bestNeither * 1e6 / ng:F1} (full {bestB * 1e6 / ng:F1})");
        Console.WriteLine($"B pose refresh (inertia rows and separations, after each position integration): {bestR:F4} ms per call; it serves the warm start, solve and relax passes of a substep");
        Console.WriteLine($"B with the refresh charged to two passes: {bestB + bestR / 2:F4} ms per pass, A / B = {bestA / (bestB + bestR / 2):F3}; charged to one pass: {bestB + bestR:F4} ms, A / B = {bestA / (bestB + bestR):F3}");
        return diff == 0 ? 0 : 1;
    }

    private static void PassA(Engine3D e, List<int> groups, List<int> counts, in KernelParams kp)
    {
        for (int i = 0; i < groups.Count; i++) { ContactKernel3D.Run<Lanes256>(e, groups[i], 0, counts[i], SolvePhase.Solve, in kp); }
    }

    private static PhysicsWorld3D BuildPile(int steps)
    {
        var o = new WorldOptions3D { ThreadCount = 1 };
        var w = new PhysicsWorld3D(o);
        w.AddBox(new Vector3F(20f, 1f, 20f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 80f, 18f), new Vector3D(-9.5, 40, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 80f, 18f), new Vector3D(9.5, 40, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(18f, 80f, 1f), new Vector3D(0, 40, -9.5), type: BodyType.Static);
        w.AddBox(new Vector3F(18f, 80f, 1f), new Vector3D(0, 40, 9.5), type: BodyType.Static);
        uint s = 3;
        float Next() { s = s * 1664525u + 1013904223u; return (s >> 8) / 16777216f; }
        const int cols = 14;
        for (int i = 0; i < 3000; i++)
        {
            int layer = i / (cols * cols), k = i % (cols * cols);
            var at = new Vector3D(-7.8 + (k % cols) * 1.2, 2 + layer * 1.2, -7.8 + (k / cols) * 1.2);
            float x = Next() - 0.5f, y = Next() - 0.5f, z = Next() - 0.5f, q = Next() - 0.5f;
            float l = float.Sqrt(x * x + y * y + z * z + q * q); if (!(l > 1e-3f)) { x = 0; y = 0; z = 0; q = 1; l = 1; }
            var rot = new QuaternionF(x / l, y / l, z / l, q / l);
            switch (i % 5)
            {
                case 0: w.AddSphere(0.5f, at); break;
                case 1: w.AddCapsule(0.4f, 0.3f, at, rot); break;
                default: w.AddBox(new Vector3F(1f, 1f, 1f), at, rot); break;
            }
        }

        for (int i = 0; i < steps; i++) { w.Step(1f / 60f); }
        return w;
    }

    // ---------------------------------------------------------------- B

    private struct V3<T> where T : struct, ILane<T>
    {
        public T X, Y, Z;
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public V3(T x, T y, T z) { X = x; Y = y; Z = z; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static V3<T> operator +(V3<T> a, V3<T> b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static V3<T> operator -(V3<T> a, V3<T> b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static V3<T> operator *(V3<T> a, T s) => new(a.X * s, a.Y * s, a.Z * s);
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static V3<T> Cross(V3<T> a, V3<T> b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static T Dot(V3<T> a, V3<T> b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static V3<T> Select(T mask, V3<T> whenSet, V3<T> whenClear) => new(T.Select(mask, whenSet.X, whenClear.X), T.Select(mask, whenSet.Y, whenClear.Y), T.Select(mask, whenSet.Z, whenClear.Z));
    }

    private struct Sym<T> where T : struct, ILane<T>
    {
        public T M0, M1, M2, M3, M4, M5;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public V3<T> Apply(V3<T> v) => new(M0 * v.X + M1 * v.Y + M2 * v.Z, M1 * v.X + M3 * v.Y + M4 * v.Z, M2 * v.X + M4 * v.Y + M5 * v.Z);
    }

    private struct Body<T> where T : struct, ILane<T>
    {
        public V3<T> V, W;
        public Sym<T> I;
        public T InvMass;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T F<T>(ReadOnlySpan<float> g, int f, int l0) where T : struct, ILane<T> => T.LoadAt(g, f * 8 + l0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Vec<T>(ReadOnlySpan<float> g, int f, int l0) where T : struct, ILane<T> => new(F<T>(g, f, l0), F<T>(g, f + 1, l0), F<T>(g, f + 2, l0));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Sym<T> SymAt<T>(ReadOnlySpan<float> g, int f, int l0) where T : struct, ILane<T> => new() { M0 = F<T>(g, f, l0), M1 = F<T>(g, f + 1, l0), M2 = F<T>(g, f + 2, l0), M3 = F<T>(g, f + 3, l0), M4 = F<T>(g, f + 4, l0), M5 = F<T>(g, f + 5, l0) };

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static T Clamp0<T>(T x) where T : struct, ILane<T> => T.Select(T.GreaterThan(x, T.Zero), x, T.Zero);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Rotate<T>(T x, T y, T z, T w, V3<T> v, T two) where T : struct, ILane<T>
    {
        T tx = two * (y * v.Z - z * v.Y), ty = two * (z * v.X - x * v.Z), tz = two * (x * v.Y - y * v.X);
        return new V3<T>(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Apply<T>(ref Body<T> a, ref Body<T> b, V3<T> rA, V3<T> rB, V3<T> p, T mask) where T : struct, ILane<T>
    {
        var vA = a.V - p * a.InvMass;
        var wA = a.W - a.I.Apply(V3<T>.Cross(rA, p));
        var vB = b.V + p * b.InvMass;
        var wB = b.W + b.I.Apply(V3<T>.Cross(rB, p));
        a.V = V3<T>.Select(mask, vA, a.V); a.W = V3<T>.Select(mask, wA, a.W);
        b.V = V3<T>.Select(mask, vB, b.V); b.W = V3<T>.Select(mask, wB, b.W);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static V3<T> Rel<T>(in Body<T> a, in Body<T> b, V3<T> rA, V3<T> rB) where T : struct, ILane<T> => (b.V + V3<T>.Cross(b.W, rB)) - (a.V + V3<T>.Cross(a.W, rA));

    /// <summary>Writes a group in layout B from the library's group, the bodies' packed rows and the body indices: once per step in a real implementation.</summary>
    private static void Prepare<T>(Engine3D e, int group, Span<float> g, Span<int> offA, Span<int> offB, Span<int> scA, Span<int> scB, out int maxPts) where T : struct, ILane<T>
    {
        var w = e.Wide.AsSpan(group * W3D.GroupFloats, W3D.GroupFloats);
        var idx = e.WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        maxPts = idx[W3D.MaxPointsSlot];
        for (int l = 0; l < 8; l++) { offA[l] = 8 * idx[l]; offB[l] = 8 * idx[8 + l]; scA[l] = idx[16 + l] >= 0 ? 8 * idx[16 + l] : -1; scB[l] = idx[24 + l] >= 0 ? 8 * idx[24 + l] : -1; }
        // Constants.
        for (int f = 0; f <= 9; f++) { w.Slice(f * 8, 8).CopyTo(g.Slice(f * 8, 8)); }
        W(g, MA, w, W3D.InvMassA); W(g, MB, w, W3D.InvMassB); W(g, Bias, w, W3D.BiasRate); W(g, MS, w, W3D.MassScale); W(g, IS, w, W3D.ImpulseScale);
        Refresh<T>(e, group, g, maxPts);
        for (int p = 0; p < maxPts; p++)
        {
            int fa = W3D.Point0 + p * W3D.PointStride, fb = P0 + p * PS;
            for (int k = 0; k < 6; k++) { W(g, fb + RA + k, w, fa + W3D.ArmA + k); }
            W(g, fb + NM, w, fa + W3D.NormalMass); W(g, fb + T1M, w, fa + W3D.Tangent1Mass); W(g, fb + T2M, w, fa + W3D.Tangent2Mass);
            W(g, fb + Mask, w, fa + W3D.Mask);
            // Impulses are copied by CopyImpulses at the start of each run.
        }
    }

    /// <summary>The part of prepare that follows the bodies' poses: their world inverse inertia and the current separation of every point. Needed again after each position integration.</summary>
    private static void Refresh<T>(Engine3D e, int group, Span<float> g, int maxPts) where T : struct, ILane<T>
    {
        var w = e.Wide.AsSpan(group * W3D.GroupFloats, W3D.GroupFloats);
        var idx = e.WideIdx.AsSpan(group * W3D.IdxPerGroup, W3D.IdxPerGroup);
        var pk = e.Pk;
        Span<int> offs = stackalloc int[8];
        var two = T.Broadcast(2f);
        for (int l0 = 0; l0 < 8; l0 += T.Count)
        {
            var ga = Gather<T>(pk, idx.Slice(l0, T.Count), offs, out var ia0, out var ia1);
            var gbb = Gather<T>(pk, idx.Slice(8 + l0, T.Count), offs, out var ib0, out var ib1);
            Store(ga.I, g, IA, l0); Store(gbb.I, g, IB, l0);
            var n = Vec<T>(w, W3D.N, l0);
            for (int p = 0; p < maxPts; p++)
            {
                int fa = W3D.Point0 + p * W3D.PointStride, fb = P0 + p * PS;
                var rA = Vec<T>(w, fa + W3D.ArmA, l0); var rB = Vec<T>(w, fa + W3D.ArmB, l0);
                var sepVec = (gbb.Dp - ga.Dp) + (Rotate(gbb.Qx, gbb.Qy, gbb.Qz, gbb.Qw, rB, two) - Rotate(ga.Qx, ga.Qy, ga.Qz, ga.Qw, rA, two));
                (V3<T>.Dot(sepVec, n) + F<T>(w, fa + W3D.AdjustedSeparation, l0)).StoreAt(g, (fb + Sep) * 8 + l0);
            }
        }

    }

    private static void W(Span<float> g, int to, Span<float> w, int from) => w.Slice(from * 8, 8).CopyTo(g.Slice(to * 8, 8));

    private static void CopyImpulses(Span<float> w, Span<float> g, int maxPts)
    {
        for (int p = 0; p < maxPts; p++)
        {
            int fa = W3D.Point0 + p * W3D.PointStride, fb = P0 + p * PS;
            W(g, fb + NI, w, fa + W3D.NormalImpulse); W(g, fb + T1I, w, fa + W3D.Tangent1Impulse); W(g, fb + T2I, w, fa + W3D.Tangent2Impulse); W(g, fb + MaxN, w, fa + W3D.MaxNormalImpulse);
        }
    }

    private struct Pose<T> where T : struct, ILane<T>
    {
        public V3<T> Dp;
        public T Qx, Qy, Qz, Qw;
        public Sym<T> I;
    }

    private static void Store<T>(Sym<T> s, Span<float> g, int f, int l0) where T : struct, ILane<T>
    {
        s.M0.StoreAt(g, f * 8 + l0); s.M1.StoreAt(g, (f + 1) * 8 + l0); s.M2.StoreAt(g, (f + 2) * 8 + l0); s.M3.StoreAt(g, (f + 3) * 8 + l0); s.M4.StoreAt(g, (f + 4) * 8 + l0); s.M5.StoreAt(g, (f + 5) * 8 + l0);
    }

    private static Pose<T> Gather<T>(ReadOnlySpan<float> pk, ReadOnlySpan<int> ids, Span<int> offs, out int d0, out int d1) where T : struct, ILane<T>
    {
        d0 = 0; d1 = 0;
        int width = T.Count;
        for (int l = 0; l < width; l++) { offs[l] = 16 * ids[l]; }
        var g0 = T.GatherRows(pk, offs);
        for (int l = 0; l < width; l++) { offs[l] += 8; }
        var g1 = T.GatherRows(pk, offs);
        return new Pose<T>
        {
            Dp = new V3<T>(g0.R0, g0.R1, g0.R2), Qx = g0.R3, Qy = g0.R4, Qz = g0.R5, Qw = g0.R6,
            I = new Sym<T> { M0 = g0.R7, M1 = g1.R0, M2 = g1.R1, M3 = g1.R2, M4 = g1.R3, M5 = g1.R4 },
        };
    }


    /// <summary>Which groups can run as a pair: a group and the next one when they share no dynamic body (so the order of the two cannot matter).</summary>
    private static bool[] PairPlan(Engine3D e, List<int> groups)
    {
        var pair = new bool[groups.Count];
        for (int i = 0; i + 1 < groups.Count; i++)
        {
            if (pair[i]) { continue; }
            var x = e.WideIdx.AsSpan(groups[i] * W3D.IdxPerGroup, W3D.IdxPerGroup); var y = e.WideIdx.AsSpan(groups[i + 1] * W3D.IdxPerGroup, W3D.IdxPerGroup);
            bool ok = true;
            for (int a = 16; a < 32 && ok; a++) { if (x[a] < 0) { continue; } for (int b = 16; b < 32; b++) { if (x[a] == y[b]) { ok = false; break; } } }
            // Static bodies are only read, so sharing one is fine; shared dynamic bodies are not.
            if (ok) { pair[i] = true; i++; }
        }

        return pair;
    }

    private static void PassB2<T>(float[] gb, float[] vel, int[] offA, int[] offB, int[] scA, int[] scB, int[] maxPts, List<int> counts, int ng, bool[] pair, in KernelParams kp) where T : struct, ILane<T>
    {
        T zero = T.Zero, one = T.Broadcast(1f);
        T invH = T.Broadcast(kp.InverseSubstepDt), slop = T.Broadcast(kp.LinearSlop), negPush = T.Broadcast(-kp.PushOutVelocity);
        for (int i = 0; i < ng; i++)
        {
            if (pair[i])
            {
                int j = i + 1;
                RunPair<T>(gb.AsSpan(i * GroupB, GroupB), gb.AsSpan(j * GroupB, GroupB), vel, offA.AsSpan(i * 8, 8), offB.AsSpan(i * 8, 8), scA.AsSpan(i * 8, 8), scB.AsSpan(i * 8, 8),
                    offA.AsSpan(j * 8, 8), offB.AsSpan(j * 8, 8), scA.AsSpan(j * 8, 8), scB.AsSpan(j * 8, 8), maxPts[i], maxPts[j], counts[i], counts[j], invH, slop, negPush, zero, one);
                i++;
            }
            else
            {
                RunGroup<T>(gb.AsSpan(i * GroupB, GroupB), vel, offA.AsSpan(i * 8, 8), offB.AsSpan(i * 8, 8), scA.AsSpan(i * 8, 8), scB.AsSpan(i * 8, 8), maxPts[i], counts[i], true, invH, slop, negPush, zero, one);
            }
        }
    }

    private static void RunPair<T>(Span<float> gx, Span<float> gy, Span<float> vel, ReadOnlySpan<int> oAx, ReadOnlySpan<int> oBx, ReadOnlySpan<int> sAx, ReadOnlySpan<int> sBx,
        ReadOnlySpan<int> oAy, ReadOnlySpan<int> oBy, ReadOnlySpan<int> sAy, ReadOnlySpan<int> sBy, int mx, int my, int nx, int ny, T invH, T slop, T negPush, T zero, T one) where T : struct, ILane<T>
    {
        int width = T.Count;
        // Only whole-vector groups (width 8, up to eight lanes) are paired.
        var xa = T.GatherRows(vel, oAx); var xb = T.GatherRows(vel, oBx);
        var ya = T.GatherRows(vel, oAy); var yb = T.GatherRows(vel, oBy);
        var ax = new Body<T> { V = new V3<T>(xa.R0, xa.R1, xa.R2), W = new V3<T>(xa.R3, xa.R4, xa.R5), I = SymAt<T>(gx, IA, 0), InvMass = F<T>(gx, MA, 0) };
        var bx = new Body<T> { V = new V3<T>(xb.R0, xb.R1, xb.R2), W = new V3<T>(xb.R3, xb.R4, xb.R5), I = SymAt<T>(gx, IB, 0), InvMass = F<T>(gx, MB, 0) };
        var ay = new Body<T> { V = new V3<T>(ya.R0, ya.R1, ya.R2), W = new V3<T>(ya.R3, ya.R4, ya.R5), I = SymAt<T>(gy, IA, 0), InvMass = F<T>(gy, MA, 0) };
        var by = new Body<T> { V = new V3<T>(yb.R0, yb.R1, yb.R2), W = new V3<T>(yb.R3, yb.R4, yb.R5), I = SymAt<T>(gy, IB, 0), InvMass = F<T>(gy, MB, 0) };
        var nX = Vec<T>(gx, N, 0); var t1X = Vec<T>(gx, T1, 0); var t2X = Vec<T>(gx, T2, 0); T frX = F<T>(gx, Fr, 0);
        var nY = Vec<T>(gy, N, 0); var t1Y = Vec<T>(gy, T1, 0); var t2Y = Vec<T>(gy, T2, 0); T frY = F<T>(gy, Fr, 0);
        int mn = Math.Min(mx, my);
        for (int i = 0; i < mn; i++) { Friction(ref ax, ref bx, gx, P0 + i * PS, 0, t1X, t2X, frX); Friction(ref ay, ref by, gy, P0 + i * PS, 0, t1Y, t2Y, frY); }
        for (int i = mn; i < mx; i++) { Friction(ref ax, ref bx, gx, P0 + i * PS, 0, t1X, t2X, frX); }
        for (int i = mn; i < my; i++) { Friction(ref ay, ref by, gy, P0 + i * PS, 0, t1Y, t2Y, frY); }
        T brX = F<T>(gx, Bias, 0), msX = F<T>(gx, MS, 0), isX = F<T>(gx, IS, 0), brY = F<T>(gy, Bias, 0), msY = F<T>(gy, MS, 0), isY = F<T>(gy, IS, 0);
        for (int i = 0; i < mn; i++) { Normal(ref ax, ref bx, gx, P0 + i * PS, 0, nX, true, brX, msX, isX, invH, slop, negPush, zero, one); Normal(ref ay, ref by, gy, P0 + i * PS, 0, nY, true, brY, msY, isY, invH, slop, negPush, zero, one); }
        for (int i = mn; i < mx; i++) { Normal(ref ax, ref bx, gx, P0 + i * PS, 0, nX, true, brX, msX, isX, invH, slop, negPush, zero, one); }
        for (int i = mn; i < my; i++) { Normal(ref ay, ref by, gy, P0 + i * PS, 0, nY, true, brY, msY, isY, invH, slop, negPush, zero, one); }
        Out(ref ax, ref bx, gx, vel, sAx, sBx, nx, width);
        Out(ref ay, ref by, gy, vel, sAy, sBy, ny, width);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Out<T>(ref Body<T> a, ref Body<T> b, Span<float> g, Span<float> vel, ReadOnlySpan<int> scA, ReadOnlySpan<int> scB, int n, int width) where T : struct, ILane<T>
    {
        Span<float> os = g.Slice(OutB, 12 * 8);
        a.V.X.StoreAt(os, 0 * width); a.V.Y.StoreAt(os, 1 * width); a.V.Z.StoreAt(os, 2 * width);
        a.W.X.StoreAt(os, 3 * width); a.W.Y.StoreAt(os, 4 * width); a.W.Z.StoreAt(os, 5 * width);
        b.V.X.StoreAt(os, 6 * width); b.V.Y.StoreAt(os, 7 * width); b.V.Z.StoreAt(os, 8 * width);
        b.W.X.StoreAt(os, 9 * width); b.W.Y.StoreAt(os, 10 * width); b.W.Z.StoreAt(os, 11 * width);
        for (int l = 0; l < width && l < n; l++)
        {
            int ia = scA[l];
            if (ia >= 0) { Scatter(vel, os, ia, l, width); }
            int ib = scB[l];
            if (ib >= 0) { Scatter(vel, os, ib, l + 6 * width, width); }
        }
    }

    /// <summary>One solve pass over all groups in layout B.</summary>
    private static void PassB<T>(float[] gb, float[] vel, int[] offA, int[] offB, int[] scA, int[] scB, int[] maxPts, List<int> counts, int ng, in KernelParams kp, bool bias) where T : struct, ILane<T>
    {
        T zero = T.Zero, one = T.Broadcast(1f);
        T invH = T.Broadcast(kp.InverseSubstepDt), slop = T.Broadcast(kp.LinearSlop), negPush = T.Broadcast(-kp.PushOutVelocity);
        for (int i = 0; i < ng; i++)
        {
            RunGroup<T>(gb.AsSpan(i * GroupB, GroupB), vel, offA.AsSpan(i * 8, 8), offB.AsSpan(i * 8, 8), scA.AsSpan(i * 8, 8), scB.AsSpan(i * 8, 8), maxPts[i], counts[i], bias, invH, slop, negPush, zero, one);
        }
    }

    private static void RunGroup<T>(Span<float> g, Span<float> vel, ReadOnlySpan<int> offA, ReadOnlySpan<int> offB, ReadOnlySpan<int> scA, ReadOnlySpan<int> scB, int maxPts, int n, bool useBias, T invH, T slop, T negPush, T zero, T one) where T : struct, ILane<T>
    {
        int width = T.Count;
        for (int l0 = 0; l0 < n; l0 += width)
        {
            var va = T.GatherRows(vel, offA.Slice(l0, width));
            var vb = T.GatherRows(vel, offB.Slice(l0, width));
            var a = new Body<T> { V = new V3<T>(va.R0, va.R1, va.R2), W = new V3<T>(va.R3, va.R4, va.R5), I = SymAt<T>(g, IA, l0), InvMass = F<T>(g, MA, l0) };
            var b = new Body<T> { V = new V3<T>(vb.R0, vb.R1, vb.R2), W = new V3<T>(vb.R3, vb.R4, vb.R5), I = SymAt<T>(g, IB, l0), InvMass = F<T>(g, MB, l0) };
            var nrm = Vec<T>(g, N, l0); var t1 = Vec<T>(g, T1, l0); var t2 = Vec<T>(g, T2, l0);
            T friction = F<T>(g, Fr, l0);
            for (int i = 0; i < maxPts; i++) { Friction(ref a, ref b, g, P0 + i * PS, l0, t1, t2, friction); }
            T biasRate = F<T>(g, Bias, l0), massScale = F<T>(g, MS, l0), impulseScale = F<T>(g, IS, l0);
            for (int i = 0; i < maxPts; i++) { Normal(ref a, ref b, g, P0 + i * PS, l0, nrm, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, one); }

            Span<float> os = g.Slice(OutB, 12 * 8);
            a.V.X.StoreAt(os, 0 * width); a.V.Y.StoreAt(os, 1 * width); a.V.Z.StoreAt(os, 2 * width);
            a.W.X.StoreAt(os, 3 * width); a.W.Y.StoreAt(os, 4 * width); a.W.Z.StoreAt(os, 5 * width);
            b.V.X.StoreAt(os, 6 * width); b.V.Y.StoreAt(os, 7 * width); b.V.Z.StoreAt(os, 8 * width);
            b.W.X.StoreAt(os, 9 * width); b.W.Y.StoreAt(os, 10 * width); b.W.Z.StoreAt(os, 11 * width);
            for (int l = 0; l < width && l0 + l < n; l++)
            {
                int ia = scA[l0 + l];
                if (ia >= 0) { Scatter(vel, os, ia, l, width); }
                int ib = scB[l0 + l];
                if (ib >= 0) { Scatter(vel, os, ib, l + 6 * width, width); }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Scatter(Span<float> vel, ReadOnlySpan<float> os, int at, int first, int width)
    {
        // at is a body row (8 floats) and first + 5 * width + 1 stays inside the group's output rows.
        Mem.Range(vel, at, 6); Mem.Range(os, first, 5 * width + 1);
        for (int k = 0; k < 6; k++) { Mem.At(vel, at + k) = Mem.At(os, first + k * width); }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Friction<T>(ref Body<T> a, ref Body<T> b, Span<float> g, int f, int l0, V3<T> t1, V3<T> t2, T friction) where T : struct, ILane<T>
    {
        var imp = FrictionImpulse(in a, in b, g, f, l0, t1, t2, friction);
        Apply(ref a, ref b, Vec<T>(g, f + RA, l0), Vec<T>(g, f + RB, l0), imp, F<T>(g, f + Mask, l0));
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static V3<T> FrictionImpulse<T>(in Body<T> a, in Body<T> b, Span<float> g, int f, int l0, V3<T> t1, V3<T> t2, T friction) where T : struct, ILane<T>
    {
        V3<T> rA = Vec<T>(g, f + RA, l0), rB = Vec<T>(g, f + RB, l0);
        var dv = Rel(in a, in b, rA, rB);
        T l1 = F<T>(g, f + T1I, l0), l2 = F<T>(g, f + T2I, l0), pm = F<T>(g, f + Mask, l0);
        T n1 = l1 - F<T>(g, f + T1M, l0) * V3<T>.Dot(dv, t1);
        T n2 = l2 - F<T>(g, f + T2M, l0) * V3<T>.Dot(dv, t2);
        T maxF = friction * F<T>(g, f + NI, l0);
        T m2 = n1 * n1 + n2 * n2;
        T clamp = T.GreaterThan(m2, maxF * maxF);
        T k = maxF / T.Sqrt(m2);
        n1 = T.Select(clamp, n1 * k, n1);
        n2 = T.Select(clamp, n2 * k, n2);
        n1 = T.Select(pm, n1, l1); n2 = T.Select(pm, n2, l2);
        n1.StoreAt(g, (f + T1I) * 8 + l0); n2.StoreAt(g, (f + T2I) * 8 + l0);
        return t1 * (n1 - l1) + t2 * (n2 - l2);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void Normal<T>(ref Body<T> a, ref Body<T> b, Span<float> g, int f, int l0, V3<T> n, bool useBias, T biasRate, T massScale, T impulseScale, T invH, T slop, T negPush, T zero, T one) where T : struct, ILane<T>
    {
        T lambda = NormalImpulse(in a, in b, g, f, l0, n, useBias, biasRate, massScale, impulseScale, invH, slop, negPush, zero, one);
        Apply(ref a, ref b, Vec<T>(g, f + RA, l0), Vec<T>(g, f + RB, l0), n * lambda, F<T>(g, f + Mask, l0));
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static T NormalImpulse<T>(in Body<T> a, in Body<T> b, Span<float> g, int f, int l0, V3<T> n, bool useBias, T biasRate, T massScale, T impulseScale, T invH, T slop, T negPush, T zero, T one) where T : struct, ILane<T>
    {
        V3<T> rA = Vec<T>(g, f + RA, l0), rB = Vec<T>(g, f + RB, l0);
        T pm = F<T>(g, f + Mask, l0);
        T sep = F<T>(g, f + Sep, l0);
        T apart = T.GreaterThan(sep, zero);
        T bias, ms, isc;
        T biasApart = sep * invH;
        if (useBias)
        {
            T x = sep + slop;
            x = T.Select(T.GreaterThan(x, zero), zero, x);
            T soft = biasRate * x;
            soft = T.Select(T.GreaterThan(negPush, soft), negPush, soft);
            bias = T.Select(apart, biasApart, soft);
            ms = T.Select(apart, one, massScale);
            isc = T.Select(apart, zero, impulseScale);
        }
        else
        {
            bias = T.Select(apart, biasApart, zero);
            ms = one;
            isc = zero;
        }

        var dv = Rel(in a, in b, rA, rB);
        T vn = V3<T>.Dot(dv, n);
        T old = F<T>(g, f + NI, l0);
        T lambda = T.Negate(F<T>(g, f + NM, l0)) * ms * (vn + bias) - isc * old;
        T nw = Clamp0(old + lambda);
        lambda = nw - old;
        T.Select(pm, nw, old).StoreAt(g, (f + NI) * 8 + l0);
        T maxN = F<T>(g, f + MaxN, l0);
        T.Select(Lane.And(pm, T.GreaterThan(nw, maxN)), nw, maxN).StoreAt(g, (f + MaxN) * 8 + l0);
        return lambda;
    }
}
