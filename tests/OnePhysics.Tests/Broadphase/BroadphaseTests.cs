using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Broadphase;

/// <summary>Dimension-specific box and ray construction so each test is written once.</summary>
internal abstract class Dim<TBox> where TBox : struct, IBox<TBox>
{
    public abstract int Axes { get; }

    public abstract TBox Make(ReadOnlySpan<double> min, ReadOnlySpan<double> max);

    public abstract RayProbe<TBox> RandomRay(TestRng rng, double world);

    public static double Unit(TestRng rng) => (rng.NextU64() >> 11) * (1.0 / 9007199254740992.0);

    public TBox RandomBox(TestRng rng, double world, double maxHalf)
    {
        var mn = new double[Axes];
        var mx = new double[Axes];
        for (int a = 0; a < Axes; a++)
        {
            double c = ((Unit(rng) * 2.0) - 1.0) * world, e = 0.05 + (Unit(rng) * maxHalf);
            mn[a] = c - e;
            mx[a] = c + e;
        }

        return Make(mn, mx);
    }

    // Moves a box by a random step; returns the displacement used.
    public TBox Shift(TBox b, TestRng rng, double step, double[] disp)
    {
        var mn = new double[Axes];
        var mx = new double[Axes];
        for (int a = 0; a < Axes; a++)
        {
            double d = ((Unit(rng) * 2.0) - 1.0) * step;
            disp[a] = d;
            mn[a] = b.Min(a) + d;
            mx[a] = b.Max(a) + d;
        }

        return Make(mn, mx);
    }
}

internal abstract class RayProbe<TBox> where TBox : struct, IBox<TBox>
{
    public abstract double Cast<TCb>(Broadphase<TBox> bp, double max, ref TCb cb, BodyTypeMask mask) where TCb : IRayCallback;

    public abstract bool Hit(TBox box, double max, out double enter);
}

internal sealed class Dim2 : Dim<Aabb2D>
{
    public override int Axes => 2;

    public override Aabb2D Make(ReadOnlySpan<double> min, ReadOnlySpan<double> max) => new(min[0], min[1], max[0], max[1]);

    public override RayProbe<Aabb2D> RandomRay(TestRng rng, double world)
    {
        double R() => ((Unit(rng) * 2.0) - 1.0) * world;
        if (rng.Next(2) == 0) { return new Probe(new Ray2D(R(), R(), R() * 0.5, R() * 0.5)); }
        return new Probe(new BoxCast2D(RandomBox(rng, world, 1.5), R() * 0.5, R() * 0.5));
    }

    private sealed class Probe : RayProbe<Aabb2D>
    {
        private readonly IRay<Aabb2D> _ray;

        public Probe(Ray2D r) => _ray = r;

        public Probe(BoxCast2D r) => _ray = r;

        public override double Cast<TCb>(Broadphase<Aabb2D> bp, double max, ref TCb cb, BodyTypeMask mask) =>
            _ray is Ray2D r ? bp.RayCast(r, max, ref cb, mask) : bp.RayCast((BoxCast2D)_ray, max, ref cb, mask);

        public override bool Hit(Aabb2D box, double max, out double enter) => _ray.Hit(box, max, out enter);
    }
}

internal sealed class Dim3 : Dim<Aabb3D>
{
    public override int Axes => 3;

    public override Aabb3D Make(ReadOnlySpan<double> min, ReadOnlySpan<double> max) => new(min[0], min[1], min[2], max[0], max[1], max[2]);

    public override RayProbe<Aabb3D> RandomRay(TestRng rng, double world)
    {
        double R() => ((Unit(rng) * 2.0) - 1.0) * world;
        if (rng.Next(2) == 0) { return new Probe(new Ray3D(R(), R(), R(), R() * 0.5, R() * 0.5, R() * 0.5)); }
        return new Probe(new BoxCast3D(RandomBox(rng, world, 1.5), R() * 0.5, R() * 0.5, R() * 0.5));
    }

    private sealed class Probe : RayProbe<Aabb3D>
    {
        private readonly IRay<Aabb3D> _ray;

        public Probe(Ray3D r) => _ray = r;

        public Probe(BoxCast3D r) => _ray = r;

        public override double Cast<TCb>(Broadphase<Aabb3D> bp, double max, ref TCb cb, BodyTypeMask mask) =>
            _ray is Ray3D r ? bp.RayCast(r, max, ref cb, mask) : bp.RayCast((BoxCast3D)_ray, max, ref cb, mask);

        public override bool Hit(Aabb3D box, double max, out double enter) => _ray.Hit(box, max, out enter);
    }
}

public class BroadphaseTests
{
    private struct ProxyInfo
    {
        public int Key;
        public BodyType Type;
        public int User;
    }

    private struct Collect : IOverlapCallback
    {
        public List<int> Keys;
        public bool Report(int proxy, int userId)
        {
            Keys.Add(proxy);
            return true;
        }
    }

    private struct RayAll : IRayCallback
    {
        public List<int> Keys;
        public double Report(int proxy, int userId, double maxFraction)
        {
            Keys.Add(proxy);
            return maxFraction;
        }
    }

    // Clips the ray to the nearest box entry seen: after the cast the max fraction is the smallest entry among reachable boxes.
    private struct RayClip : IRayCallback
    {
        public Func<int, double> Enter;
        public double Report(int proxy, int userId, double maxFraction) => Enter(proxy);
    }

    private struct EveryOther : IPairFilter
    {
        public bool ShouldCollide(int a, int b) => ((a ^ b) & 1) == 0 || a == b;
        public bool IsSensor(int userId) => false;
    }

    private static readonly double[] threeValues = { 1, 2, 3 };

    private static bool Allowed(BodyType a, BodyType b) => a == BodyType.Dynamic || b == BodyType.Dynamic;

    private static BodyType RandomType(TestRng rng) => rng.Next(10) < 2 ? BodyType.Static : rng.Next(10) < 2 ? BodyType.Kinematic : BodyType.Dynamic;

    private static List<ProxyPair> Brute<TBox, TFilter>(Broadphase<TBox> bp, List<ProxyInfo> infos, HashSet<int> moved, TFilter filter)
        where TBox : struct, IBox<TBox> where TFilter : IPairFilter
    {
        var result = new List<ProxyPair>();
        for (int i = 0; i < infos.Count; i++)
        {
            TBox bi = bp.GetFatBounds(infos[i].Key);
            for (int j = i + 1; j < infos.Count; j++)
            {
                if (!Allowed(infos[i].Type, infos[j].Type)) { continue; }
                if (!moved.Contains(infos[i].Key) && !moved.Contains(infos[j].Key)) { continue; }
                if (!TBox.Overlaps(bi, bp.GetFatBounds(infos[j].Key))) { continue; }
                ProxyInfo a = infos[i], b = infos[j];
                if (!filter.ShouldCollide(a.User, b.User)) { continue; }
                bool aFirst = a.User < b.User || (a.User == b.User && a.Key < b.Key);
                result.Add(aFirst ? new ProxyPair(a.Key, b.Key, a.User, b.User) : new ProxyPair(b.Key, a.Key, b.User, a.User));
            }
        }

        return result.OrderBy(p => p.UserA).ThenBy(p => p.UserB).ThenBy(p => p.ProxyA).ThenBy(p => p.ProxyB).ToList();
    }

    private static void Churn<TBox>(Dim<TBox> dim, Broadphase<TBox> bp, TestRng rng, List<ProxyInfo> infos, HashSet<int> moved, int ops, double world, double half) where TBox : struct, IBox<TBox>
    {
        var disp = new double[dim.Axes];
        for (int o = 0; o < ops; o++)
        {
            int r = rng.Next(100);
            if (r < 8 || infos.Count == 0)
            {
                BodyType t = RandomType(rng);
                int user = rng.Next(1 << 20);
                int key = bp.CreateProxy(t, dim.RandomBox(rng, world, half), user);
                infos.Add(new ProxyInfo { Key = key, Type = t, User = user });
                moved.Add(key);
            }
            else if (r < 14)
            {
                int i = rng.Next(infos.Count);
                bp.DestroyProxy(infos[i].Key);
                moved.Remove(infos[i].Key);
                infos[i] = infos[^1];
                infos.RemoveAt(infos.Count - 1);
            }
            else if (r < 16)
            {
                int i = rng.Next(infos.Count);
                bp.Touch(infos[i].Key);
                moved.Add(infos[i].Key);
            }
            else
            {
                int i = rng.Next(infos.Count);
                TBox cur = bp.GetFatBounds(infos[i].Key);
                // Tight box = current fat box shrunk a little, then moved: small moves stay inside the fat box, big ones leave it.
                double step = rng.Next(4) == 0 ? half * 4 : half * 0.05;
                TBox moved1 = dim.Shift(cur, rng, step, disp);
                if (bp.MoveProxy(infos[i].Key, moved1, disp)) { moved.Add(infos[i].Key); }
            }
        }
    }

    private static void RunPairs<TBox>(Dim<TBox> dim, int count, int rounds, int opsPerRound) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(99);
        var bp = new Broadphase<TBox>();
        var infos = new List<ProxyInfo>();
        var moved = new HashSet<int>();
        double world = Math.Pow(count, 1.0 / dim.Axes) * 1.5;
        Churn(dim, bp, rng, infos, moved, count, world, 0.9);
        for (int round = 0; round < rounds; round++)
        {
            List<ProxyPair> expected = Brute(bp, infos, moved, new AcceptAllPairs());
            ProxyPair[] got = bp.UpdatePairs(new AcceptAllPairs()).ToArray();
            Assert.Equal(expected, got);
            moved.Clear();
            bp.Validate();
            Churn(dim, bp, rng, infos, moved, opsPerRound, world, 0.9);
        }

        // Everything at once, with a filter.
        bp.TouchAll();
        foreach (var info in infos) { moved.Add(info.Key); }
        Assert.Equal(Brute(bp, infos, moved, new EveryOther()), bp.UpdatePairs(new EveryOther()).ToArray());
    }

    [Fact]
    public void Pairs_equal_brute_force_2d() => RunPairs(new Dim2(), 10000, 3, 6000);

    [Fact]
    public void Pairs_equal_brute_force_3d() => RunPairs(new Dim3(), 10000, 3, 6000);

    [Fact]
    public void Pairs_equal_brute_force_small_many_rounds()
    {
        RunPairs(new Dim2(), 200, 40, 150);
        RunPairs(new Dim3(), 200, 40, 150);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Tree_invariants_hold_after_random_operations(int dims)
    {
        if (dims == 2) { Invariants(new Dim2()); } else { Invariants(new Dim3()); }
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Enlarged_trees_stay_valid_and_rebuilding_the_enlarged_part_keeps_every_proxy(int dims)
    {
        if (dims == 2) { EnlargeAndRebuild(new Dim2()); } else { EnlargeAndRebuild(new Dim3()); }
    }

    private static void EnlargeAndRebuild<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(11);
        var tree = new DynamicTree<TBox>(2);
        var live = new List<int>();
        var disp = new double[dim.Axes];
        for (int i = 0; i < 700; i++) { live.Add(tree.CreateProxy(dim.RandomBox(rng, 50, 3), i)); }
        for (int round = 0; round < 60; round++)
        {
            int moves = 1 + rng.Next(80);
            for (int m = 0; m < moves; m++) { int p = live[rng.Next(live.Count)]; tree.EnlargeProxy(p, dim.Shift(tree.GetBounds(p), rng, 10, disp)); }
            tree.Validate();
            Assert.True(tree.EnlargedCount > 0);
            if (round % 3 == 0)
            {
                tree.RebuildEnlarged();
                Assert.Equal(0, tree.EnlargedCount);
                tree.Validate();
            }
            else if (round % 7 == 1)
            {
                // A structural change flushes the enlarged region first.
                int i = rng.Next(live.Count);
                tree.DestroyProxy(live[i]);
                live[i] = tree.CreateProxy(dim.RandomBox(rng, 50, 3), 1000 + round);
                Assert.Equal(0, tree.EnlargedCount);
                tree.Validate();
            }

            // Every proxy is still reachable: a query over the whole space reports all of them.
            var wide = new Collect { Keys = new List<int>() };
            var root = tree.GetBounds(tree.Root);
            tree.Query(root, ref wide);
            Assert.Equal(live.Count, wide.Keys.Count);
        }

        // A snapshot taken with a flagged region restores to a tree that rebuilds the same way.
        var w = StateWriter.ToBuffer();
        tree.Write(w);
        var copy = new DynamicTree<TBox>();
        copy.Read(new StateReader(w.ToArray()));
        Assert.Equal(tree.EnlargedCount, copy.EnlargedCount);
        tree.RebuildEnlarged(); copy.RebuildEnlarged();
        var a = StateWriter.ToBuffer(); tree.Write(a);
        var b = StateWriter.ToBuffer(); copy.Write(b);
        Assert.Equal(a.ToArray(), b.ToArray());
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Rebuilding_the_enlarged_part_on_several_threads_gives_the_serial_tree(int dims)
    {
        if (dims == 2) { ParallelRebuild(new Dim2()); } else { ParallelRebuild(new Dim3()); }
    }

    private static void ParallelRebuild<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        var executor = new TaskExecutor(8);
        for (int round = 0; round < 6; round++)
        {
            // Two trees built and enlarged the same way; one is rebuilt serially and one with the executor.
            var serial = new DynamicTree<TBox>(2);
            var threaded = new DynamicTree<TBox>(2);
            var rngA = new TestRng((ulong)(100 + round));
            var rngB = new TestRng((ulong)(100 + round));
            int count = 3000 + (round * 700);
            var keysA = new List<int>();
            var keysB = new List<int>();
            var dispA = new double[dim.Axes];
            var dispB = new double[dim.Axes];
            for (int i = 0; i < count; i++)
            {
                keysA.Add(serial.CreateProxy(dim.RandomBox(rngA, 80, 2), i));
                keysB.Add(threaded.CreateProxy(dim.RandomBox(rngB, 80, 2), i));
            }

            int moves = count / 4;
            for (int m = 0; m < moves; m++)
            {
                int i = rngA.Next(count);
                Assert.Equal(i, rngB.Next(count));
                serial.EnlargeProxy(keysA[i], dim.Shift(serial.GetBounds(keysA[i]), rngA, 6, dispA));
                threaded.EnlargeProxy(keysB[i], dim.Shift(threaded.GetBounds(keysB[i]), rngB, 6, dispB));
            }

            Assert.Equal(serial.EnlargedCount, threaded.EnlargedCount);
            serial.RebuildEnlarged();
            threaded.RebuildEnlarged(executor);
            threaded.Validate();
            var a = StateWriter.ToBuffer(); serial.Write(a);
            var b = StateWriter.ToBuffer(); threaded.Write(b);
            Assert.Equal(a.ToArray(), b.ToArray());
        }
    }

    private static void Invariants<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(5);
        var tree = new DynamicTree<TBox>(2);
        var live = new List<int>();
        var disp = new double[dim.Axes];
        for (int op = 0; op < 6000; op++)
        {
            int r = rng.Next(100);
            if (r < 45 || live.Count == 0) { live.Add(tree.CreateProxy(dim.RandomBox(rng, 50, 3), op)); }
            else if (r < 65)
            {
                int i = rng.Next(live.Count);
                tree.DestroyProxy(live[i]);
                live[i] = live[^1];
                live.RemoveAt(live.Count - 1);
            }
            else if (r < 98) { int p = live[rng.Next(live.Count)]; tree.MoveProxy(p, dim.Shift(tree.GetBounds(p), rng, 10, disp)); }
            else { tree.Rebuild(); }
            if (op % 25 == 0) { tree.Validate(); }
        }

        tree.Validate();
        Assert.Equal(live.Count, tree.ProxyCount);
        double before = tree.TotalInternalArea;
        tree.Rebuild();
        tree.Validate();
        var w1 = StateWriter.ToBuffer();
        tree.Write(w1);
        tree.Rebuild();
        var w2 = StateWriter.ToBuffer();
        tree.Write(w2);
        Assert.Equal(w1.ToArray(), w2.ToArray());   // rebuilding is idempotent
        Assert.True(tree.TotalInternalArea < before * 1.5, "rebuild should not make the tree much worse");
        foreach (int p in live) { Assert.True(tree.IsProxy(p)); }
        Assert.True(tree.Height < 60);

        // Empty it completely.
        foreach (int p in live) { tree.DestroyProxy(p); }
        tree.Validate();
        Assert.Equal(-1, tree.Root);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Queries_equal_brute_force(int dims)
    {
        if (dims == 2) { Queries(new Dim2()); } else { Queries(new Dim3()); }
    }

    private static void Queries<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(31);
        var bp = new Broadphase<TBox>();
        var infos = new List<ProxyInfo>();
        Churn(dim, bp, rng, infos, new HashSet<int>(), 4000, 40, 1.2);
        bp.Rebuild(BodyTypeMask.Static);
        for (int q = 0; q < 300; q++)
        {
            BodyTypeMask mask = (BodyTypeMask)(1 + rng.Next(7));
            bool In(ProxyInfo p) => ((int)mask & (1 << (int)p.Type)) != 0;

            // Overlap.
            TBox box = dim.RandomBox(rng, 40, 6);
            var got = new List<int>();
            var cb = new Collect { Keys = got };
            bp.QueryAabb(box, ref cb, mask);
            var expect = infos.Where(p => In(p) && TBox.Overlaps(bp.GetFatBounds(p.Key), box)).Select(p => p.Key).ToList();
            Assert.Equal(expect.OrderBy(k => k), got.OrderBy(k => k));

            // Ray or box sweep, every hit reported.
            RayProbe<TBox> probe = dim.RandomRay(rng, 40);
            double max = 0.5 + Dim<TBox>.Unit(rng);
            var hits = new List<int>();
            var all = new RayAll { Keys = hits };
            Assert.Equal(max, probe.Cast(bp, max, ref all, mask));
            var expectHits = infos.Where(p => In(p) && probe.Hit(bp.GetFatBounds(p.Key), max, out _)).Select(p => p.Key).ToList();
            Assert.Equal(expectHits.OrderBy(k => k), hits.OrderBy(k => k));

            // Clipping to the nearest entry gives the minimum entry fraction.
            var clip = new RayClip
            {
                Enter = key =>
                {
                    probe.Hit(bp.GetFatBounds(key), double.PositiveInfinity, out double enter);
                    return enter == 0.0 ? 1e-300 : enter;   // 0 would mean stop
                },
            };
            double f = probe.Cast(bp, max, ref clip, mask);
            double best = max;
            foreach (ProxyInfo p in infos)
            {
                if (In(p) && probe.Hit(bp.GetFatBounds(p.Key), max, out double e)) { best = Math.Min(best, e == 0.0 ? 1e-300 : e); }
            }

            Assert.Equal(best, f);
        }
    }

    [Fact]
    public void A_callback_returning_zero_stops_the_cast()
    {
        var bp = new Broadphase2D();
        for (int i = 0; i < 50; i++) { bp.CreateProxy(BodyType.Dynamic, new Aabb2D(i, -1, i + 0.5, 1), i); }
        var stop = new RayClip { Enter = _ => 0.0 };
        Assert.Equal(0.0, bp.RayCast(new Ray2D(-1, 0, 100, 0), 1.0, ref stop));
        var hits = new List<int>();
        var ignore = new RayAll { Keys = hits };
        Assert.Equal(1.0, bp.RayCast(new Ray2D(-1, 0, 100, 0), 1.0, ref ignore));
        Assert.Equal(50, hits.Count);
    }

    [Fact]
    public void Ray_box_test_handles_parallel_and_touching_rays()
    {
        var box = new Aabb2D(1, 1, 2, 2);
        Assert.True(new Ray2D(0, 1.5, 1, 0).Hit(box, 1.0, out double t) && t == 1.0);
        Assert.False(new Ray2D(0, 2.5, 1, 0).Hit(box, 1.0, out _));
        Assert.True(new Ray2D(1, 0, 0, 1).Hit(box, 1.0, out t) && t == 1.0);   // runs along the box's edge
        Assert.False(new Ray2D(0, 1.5, 1, 0).Hit(box, 0.5, out _));
        Assert.True(new Ray2D(1.5, 1.5, 0, 0).Hit(box, 1.0, out t) && t == 0.0);
        Assert.True(new Ray3D(0, 0, 0, 1, 1, 1).Hit(new Aabb3D(1, 1, 1, 2, 2, 2), 1.0, out t) && t == 1.0);
        Assert.True(new Ray2D(0, 1.5, 1e-310, 0).Hit(new Aabb2D(-1, 1, 1, 2), 1.0, out _));
        Assert.True(new BoxCast2D(new Aabb2D(0, 0, 1, 1), 10, 0).Hit(new Aabb2D(5, 0, 6, 1), 1.0, out t) && Math.Abs(t - 0.4) < 1e-12);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Pairs_and_structure_do_not_depend_on_thread_count(int dims)
    {
        if (dims == 2) { ThreadCounts(new Dim2()); } else { ThreadCounts(new Dim3()); }
    }

    private static void ThreadCounts<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        int[] counts = { 1, 2, 8, Environment.ProcessorCount };
        var pairRuns = new List<ProxyPair[]>();
        var states = new List<byte[]>();
        foreach (int threads in counts.Append(1))
        {
            var rng = new TestRng(77);
            var bp = new Broadphase<TBox>();
            var infos = new List<ProxyInfo>();
            var moved = new HashSet<int>();
            var exec = new TaskExecutor(threads);
            var all = new List<ProxyPair>();
            Churn(dim, bp, rng, infos, moved, 3000, 30, 0.8);
            for (int round = 0; round < 6; round++)
            {
                all.AddRange(bp.UpdatePairs(new AcceptAllPairs(), exec).ToArray());
                Churn(dim, bp, rng, infos, moved, 1500, 30, 0.8);
            }

            var w = StateWriter.ToBuffer();
            bp.Write(w);
            pairRuns.Add(all.ToArray());
            states.Add(w.ToArray());
        }

        for (int i = 1; i < pairRuns.Count; i++)
        {
            Assert.Equal(pairRuns[0], pairRuns[i]);
            Assert.Equal(states[0], states[i]);
        }
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Refitting_the_enlarged_part_keeps_the_tree_valid_and_never_loosens_it(int dims)
    {
        if (dims == 2) { Refit(new Dim2()); } else { Refit(new Dim3()); }
    }

    private static void Refit<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(31);
        var tree = new DynamicTree<TBox>(2);
        var live = new List<int>();
        var disp = new double[dim.Axes];
        for (int i = 0; i < 600; i++) { live.Add(tree.CreateProxy(dim.RandomBox(rng, 50, 3), i)); }
        for (int round = 0; round < 40; round++)
        {
            for (int m = 0, moves = 1 + rng.Next(120); m < moves; m++) { int p = live[rng.Next(live.Count)]; tree.EnlargeProxy(p, dim.Shift(tree.GetBounds(p), rng, 10, disp)); }
            double before = tree.TotalInternalArea;
            tree.RefitEnlarged();
            Assert.Equal(0, tree.EnlargedCount);
            tree.Validate();
            Assert.True(tree.TotalInternalArea <= before);
            var wide = new Collect { Keys = new List<int>() };
            tree.Query(tree.GetBounds(tree.Root), ref wide);
            Assert.Equal(live.Count, wide.Keys.Count);
        }
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Refit_schedule_finds_the_same_pairs_and_snapshots_continue_identically(int dims)
    {
        if (dims == 2) { RefitSchedule(new Dim2()); } else { RefitSchedule(new Dim3()); }
    }

    private static void RefitSchedule<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        // The refit schedule changes the tree's shape, never which pairs exist: a broadphase that refits between rebuilds reports what one that always rebuilds reports.
        // Only motion happens after the setup (creating or destroying a proxy flushes the enlarged part by itself), so the schedule is what is exercised.
        BroadphaseOptions Opt(int refits) => new() { EnlargedRebuildShift = 1, EnlargedRefitSteps = refits };
        var always = new Broadphase<TBox>(Opt(0));
        var refits = new Broadphase<TBox>(Opt(4));
        var rng = new TestRng(77);
        var keysA = new List<int>(); var keysB = new List<int>();
        for (int i = 0; i < 900; i++)
        {
            var box = dim.RandomBox(rng, 30, 1.5);
            keysA.Add(always.CreateProxy(BodyType.Dynamic, box, i));
            keysB.Add(refits.CreateProxy(BodyType.Dynamic, box, i));
        }

        Assert.Equal(always.UpdatePairs(new AcceptAllPairs()).ToArray(), refits.UpdatePairs(new AcceptAllPairs()).ToArray());
        Broadphase<TBox>? restored = null;
        var keysC = new List<int>();
        var disp = new double[dim.Axes];
        for (int round = 0; round < 30; round++)
        {
            // One script of moves per round, applied to every broadphase.
            var script = new List<(int Index, TBox Box)>();
            for (int m = 0, moves = 150 + rng.Next(250); m < moves; m++)
            {
                int i = rng.Next(keysA.Count);
                script.Add((i, dim.Shift(always.GetFatBounds(keysA[i]), rng, 6, disp)));
            }

            foreach (var (i, box) in script)
            {
                always.MoveProxy(keysA[i], box, ReadOnlySpan<double>.Empty);
                refits.MoveProxy(keysB[i], box, ReadOnlySpan<double>.Empty);
                restored?.MoveProxy(keysC[i], box, ReadOnlySpan<double>.Empty);
            }

            var expected = always.UpdatePairs(new AcceptAllPairs()).ToArray();
            Assert.Equal(expected, refits.UpdatePairs(new AcceptAllPairs()).ToArray());
            if (restored != null) { Assert.Equal(expected, restored.UpdatePairs(new AcceptAllPairs()).ToArray()); }
            if (round == 9)
            {
                // The number of refits since the last rebuild is part of the state: a restored broadphase must rebuild on the same steps from here on.
                var w = StateWriter.ToBuffer();
                refits.Write(w);
                restored = new Broadphase<TBox>(Opt(4));
                restored.Read(new StateReader(w.ToArray()));
                restored.Validate();
                keysC.AddRange(keysB);
            }
        }

        refits.Validate();
        var a = StateWriter.ToBuffer(); refits.Write(a);
        var b = StateWriter.ToBuffer(); restored!.Write(b);
        Assert.Equal(a.ToArray(), b.ToArray());
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    public void Snapshot_restore_continues_bit_identically(int dims)
    {
        if (dims == 2) { Snapshot(new Dim2()); } else { Snapshot(new Dim3()); }
    }

    private static void Snapshot<TBox>(Dim<TBox> dim) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(123);
        var bp = new Broadphase<TBox>();
        var infos = new List<ProxyInfo>();
        var moved = new HashSet<int>();
        Churn(dim, bp, rng, infos, moved, 2500, 30, 0.8);
        bp.UpdatePairs(new AcceptAllPairs());
        Churn(dim, bp, rng, infos, moved, 800, 30, 0.8);   // leaves a populated move buffer

        var w = StateWriter.ToBuffer();
        bp.Write(w);
        byte[] bytes = w.ToArray();
        var restored = new Broadphase<TBox>();
        restored.Read(new StateReader(bytes));
        restored.Validate();
        var w2 = StateWriter.ToBuffer();
        restored.Write(w2);
        Assert.Equal(bytes, w2.ToArray());

        // The same operations on both give the same pairs and the same bytes.
        var rngA = new TestRng(555);
        var rngB = new TestRng(555);
        var infosB = new List<ProxyInfo>(infos);
        var movedA = new HashSet<int>();
        var movedB = new HashSet<int>();
        for (int round = 0; round < 5; round++)
        {
            Assert.Equal(bp.UpdatePairs(new AcceptAllPairs()).ToArray(), restored.UpdatePairs(new AcceptAllPairs()).ToArray());
            Churn(dim, bp, rngA, infos, movedA, 700, 30, 0.8);
            Churn(dim, restored, rngB, infosB, movedB, 700, 30, 0.8);
        }

        var wa = StateWriter.ToBuffer();
        bp.Write(wa);
        var wb = StateWriter.ToBuffer();
        restored.Write(wb);
        Assert.Equal(wa.ToArray(), wb.ToArray());

        // Hash mode sees the same primitives.
        var h1 = StateWriter.ForHash();
        bp.Write(h1);
        var h2 = StateWriter.ForHash();
        restored.Write(h2);
        Assert.Equal(h1.Hash, h2.Hash);
    }

    [Fact]
    public void Corrupt_snapshots_are_rejected_and_leave_the_broadphase_unchanged()
    {
        var bp = new Broadphase3D();
        var rng = new TestRng(1);
        var dim = new Dim3();
        for (int i = 0; i < 100; i++) { bp.CreateProxy(BodyType.Dynamic, dim.RandomBox(rng, 10, 1), i); }
        var w = StateWriter.ToBuffer();
        bp.Write(w);
        byte[] good = w.ToArray();

        var target = new Broadphase3D();
        target.CreateProxy(BodyType.Static, new Aabb3D(0, 0, 0, 1, 1, 1), 5);
        var before = StateWriter.ToBuffer();
        target.Write(before);

        for (int cut = 0; cut < good.Length; cut += 97)
        {
            Assert.Throws<SnapshotFormatException>(() => target.Read(new StateReader(good[..cut])));
        }

        var sr = new TestRng(9);
        for (int i = 0; i < 400; i++)
        {
            byte[] bad = (byte[])good.Clone();
            for (int k = 0; k < 3; k++) { bad[sr.Next(bad.Length)] ^= (byte)(1 + sr.Next(255)); }
            try { target.Read(new StateReader(bad)); }
            catch (SnapshotFormatException) { }
            // Either rejected (and unchanged) or a different but valid broadphase; it must always validate.
            target.Validate();
            target.Read(new StateReader(before.ToArray()));
        }

        var after = StateWriter.ToBuffer();
        target.Write(after);
        Assert.Equal(before.ToArray(), after.ToArray());
    }

    [Fact]
    public void Hostile_input_is_rejected_with_clear_errors()
    {
        var bp = new Broadphase2D();
        Assert.Throws<ArgumentOutOfRangeException>(() => bp.CreateProxy(BodyType.Dynamic, new Aabb2D(double.NaN, 0, 1, 1), 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => bp.CreateProxy(BodyType.Dynamic, new Aabb2D(0, 0, double.PositiveInfinity, 1), 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => bp.CreateProxy(BodyType.Dynamic, new Aabb2D(2, 0, 1, 1), 0));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => bp.CreateProxy(BodyType.Dynamic, new Aabb2D(0, 0, 1e9, 1), 0));
        Assert.Contains("4194304", ex.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => bp.CreateProxy((BodyType)7, new Aabb2D(0, 0, 1, 1), 0));
        int k = bp.CreateProxy(BodyType.Dynamic, new Aabb2D(0, 0, 1, 1), 3);
        Assert.Throws<ArgumentOutOfRangeException>(() => bp.MoveProxy(k, new Aabb2D(0, 0, 100, double.NaN), 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => bp.MoveProxy(k, new Aabb2D(50, 50, 51, 51), double.NaN, 0));
        Assert.Throws<ArgumentException>(() => bp.MoveProxy(k, new Aabb2D(50, 50, 51, 51), threeValues));
        bp.DestroyProxy(k);
        Assert.Throws<ArgumentException>(() => bp.DestroyProxy(k));
        Assert.Throws<ArgumentException>(() => bp.GetFatBounds(k));
        Assert.Throws<ArgumentException>(() => bp.GetUserId(-5));
        Assert.Throws<ArgumentException>(() => bp.GetUserId(int.MaxValue));
        Assert.False(bp.IsProxy(3));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Ray2D(double.NaN, 0, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Ray3D(0, 0, 0, 1, double.PositiveInfinity, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new BoxCast2D(new Aabb2D(1, 1, 0, 0), 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Broadphase2D(new BroadphaseOptions { Margin = -1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Broadphase2D(new BroadphaseOptions { DisplacementMultiplier = double.NaN }));

        // Inside the box, at the coordinate limit, and with huge but legal boxes.
        int far = bp.CreateProxy(BodyType.Static, new Aabb2D(-4194304, -4194304, 4194304, 4194304), 1);
        Assert.True(bp.IsProxy(far));
        bp.Validate();
    }

    [Fact]
    public void Small_moves_leave_the_tree_alone_and_large_ones_do_not()
    {
        var bp = new Broadphase3D();
        int k = bp.CreateProxy(BodyType.Dynamic, new Aabb3D(0, 0, 0, 1, 1, 1), 1);
        bp.UpdatePairs(new AcceptAllPairs());
        Assert.Equal(0, bp.PendingMoves);
        Assert.False(bp.MoveProxy(k, new Aabb3D(0.05, 0, 0, 1.05, 1, 1), 0.05, 0, 0));
        Assert.Equal(0, bp.PendingMoves);
        Assert.True(bp.MoveProxy(k, new Aabb3D(1, 0, 0, 2, 1, 1), 1, 0, 0));
        Assert.Equal(1, bp.PendingMoves);
        // Margin 0.1 on every side plus 2 * 1 ahead in +x.
        Assert.Equal(new Aabb3D(0.9, -0.1, -0.1, 4.1, 1.1, 1.1), bp.GetFatBounds(k));
    }

    [Fact]
    public void Static_and_kinematic_proxies_pair_only_with_dynamic_ones()
    {
        var bp = new Broadphase2D();
        int s = bp.CreateProxy(BodyType.Static, new Aabb2D(0, 0, 1, 1), 1);
        int k = bp.CreateProxy(BodyType.Kinematic, new Aabb2D(0, 0, 1, 1), 2);
        Assert.Equal(0, bp.UpdatePairs(new AcceptAllPairs()).Length);
        int d = bp.CreateProxy(BodyType.Dynamic, new Aabb2D(0.5, 0.5, 1.5, 1.5), 3);
        ProxyPair[] pairs = bp.UpdatePairs(new AcceptAllPairs()).ToArray();
        Assert.Equal(new[] { new ProxyPair(s, d, 1, 3), new ProxyPair(k, d, 2, 3) }, pairs);
        Assert.Equal(BodyType.Kinematic, bp.GetBodyType(k));
    }

    [Fact]
    public void Hash_is_the_same_on_every_platform()
    {
        ulong h = BroadphaseHash.Compute();
        Assert.Equal(BroadphaseHash.Compute(), h);
        Assert.Equal(BroadphaseHash.Expected, h);
    }
}
