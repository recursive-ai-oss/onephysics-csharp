using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Bench;

internal static class Program
{
    private static void Main(string[] args) => BenchmarkSwitcher.FromAssembly(typeof(GravityBench).Assembly).Run(args);
}

[MemoryDiagnoser]
public class GravityBench
{
    private readonly Vector3F _v = new(3f, 4f, 12f);

    [Benchmark]
    public float Strength() => Gravity.Strength(_v);
}
