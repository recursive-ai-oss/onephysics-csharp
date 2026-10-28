using System;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Bench;

/// <summary>Broadphase micro-benchmarks: build, rebuild, move, pair update and queries over a cloud of boxes. Every benchmark is allocation-free in steady state except the build.</summary>
[MemoryDiagnoser]
public abstract class BroadphaseBench<TBox> where TBox : struct, IBox<TBox>
{
    private const int Queries = 1000;

    private Broadphase<TBox> _bp = null!;
    private int[] _keys = null!;
    private TBox[] _boxes = null!;
    private TBox[] _queryBoxes = null!;
    private TBox[] _moveTo = null!;
    private TaskExecutor _one = null!;
    private TaskExecutor _many = null!;
    private int _flip;

    [Params(10000)]
    public int Count { get; set; }

    protected abstract int Axes { get; }

    protected abstract TBox Make(double[] min, double[] max);

    protected abstract double CastRays(Broadphase<TBox> bp, int i, double world);

    private TBox RandomBox(ulong seed, double world, double half)
    {
        var mn = new double[Axes];
        var mx = new double[Axes];
        for (int a = 0; a < Axes; a++)
        {
            seed = Mix(seed + (ulong)a + 1);
            double c = (((seed >> 11) * (1.0 / 9007199254740992.0)) * 2.0 - 1.0) * world;
            seed = Mix(seed);
            double e = 0.1 + ((seed >> 11) * (1.0 / 9007199254740992.0) * half);
            mn[a] = c - e;
            mx[a] = c + e;
        }

        return Make(mn, mx);
    }

    private static ulong Mix(ulong z)
    {
        z += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private double World => Math.Pow(Count, 1.0 / Axes) * 1.5;

    [GlobalSetup]
    public void Setup()
    {
        _one = new TaskExecutor(1);
        _many = new TaskExecutor(8);
        _boxes = new TBox[Count];
        _moveTo = new TBox[Count];
        for (int i = 0; i < Count; i++)
        {
            _boxes[i] = RandomBox((ulong)i * 3, World, 1.0);
            _moveTo[i] = RandomBox((ulong)i * 3 + 1, World, 1.0);
        }

        _queryBoxes = new TBox[Queries];
        for (int i = 0; i < Queries; i++) { _queryBoxes[i] = RandomBox(1000003UL + (ulong)i, World, 4.0); }
        _keys = new int[Count];
        _bp = BuildNew();
        _bp.UpdatePairs(new AcceptAllPairs());
    }

    private Broadphase<TBox> BuildNew()
    {
        var bp = new Broadphase<TBox>();
        for (int i = 0; i < Count; i++) { _keys[i] = bp.CreateProxy(i % 10 == 0 ? BodyType.Static : BodyType.Dynamic, _boxes[i], i); }
        return bp;
    }

    [Benchmark]
    public int Build() => BuildNew().GetProxyCount(BodyType.Dynamic);

    [Benchmark]
    public void RebuildTrees() => _bp.Rebuild();

    // Moves a tenth of the proxies far enough to leave their fat boxes, alternating between two layouts.
    [Benchmark]
    public int MoveTenth()
    {
        int moved = 0;
        TBox[] to = (_flip++ & 1) == 0 ? _moveTo : _boxes;
        for (int i = 1; i < Count; i += 10)
        {
            if (_bp.MoveProxy(_keys[i], to[i], ReadOnlySpan<double>.Empty)) { moved++; }
        }

        return moved;
    }

    [Benchmark]
    public int UpdatePairsOneThread()
    {
        TouchTenth();
        return _bp.UpdatePairs(new AcceptAllPairs(), _one).Length;
    }

    [Benchmark]
    public int UpdatePairsEightThreads()
    {
        TouchTenth();
        return _bp.UpdatePairs(new AcceptAllPairs(), _many).Length;
    }

    private void TouchTenth()
    {
        for (int i = 3; i < Count; i += 10) { _bp.Touch(_keys[i]); }
    }

    private struct Count1 : IOverlapCallback
    {
        public int N;
        public bool Report(int proxy, int userId)
        {
            N++;
            return true;
        }
    }

    [Benchmark(OperationsPerInvoke = Queries)]
    public int QueryAabb()
    {
        var cb = new Count1();
        for (int i = 0; i < Queries; i++) { _bp.QueryAabb(_queryBoxes[i], ref cb); }
        return cb.N;
    }

    [Benchmark(OperationsPerInvoke = Queries)]
    public double RayCast()
    {
        double sum = 0;
        for (int i = 0; i < Queries; i++) { sum += CastRays(_bp, i, World); }
        return sum;
    }
}

/// <summary>The 2D broadphase benchmarks.</summary>
public class Broadphase2DBench : BroadphaseBench<Aabb2D>
{
    protected override int Axes => 2;

    protected override Aabb2D Make(double[] min, double[] max) => new(min[0], min[1], max[0], max[1]);

    private struct Nearest : IRayCallback
    {
        public double Report(int proxy, int userId, double maxFraction) => maxFraction * 0.5;
    }

    protected override double CastRays(Broadphase<Aabb2D> bp, int i, double world)
    {
        double a = i * 0.37;
        var cb = new Nearest();
        return bp.RayCast(new Ray2D(-world, ((i % 97) / 97.0 - 0.5) * world, 2 * world, a % 5.0 - 2.5), 1.0, ref cb);
    }
}

/// <summary>The 3D broadphase benchmarks.</summary>
public class Broadphase3DBench : BroadphaseBench<Aabb3D>
{
    protected override int Axes => 3;

    protected override Aabb3D Make(double[] min, double[] max) => new(min[0], min[1], min[2], max[0], max[1], max[2]);

    private struct Nearest : IRayCallback
    {
        public double Report(int proxy, int userId, double maxFraction) => maxFraction * 0.5;
    }

    protected override double CastRays(Broadphase<Aabb3D> bp, int i, double world)
    {
        double a = i * 0.37;
        var cb = new Nearest();
        return bp.RayCast(new Ray3D(-world, ((i % 97) / 97.0 - 0.5) * world, ((i % 89) / 89.0 - 0.5) * world, 2 * world, a % 5.0 - 2.5, a % 3.0 - 1.5), 1.0, ref cb);
    }
}
