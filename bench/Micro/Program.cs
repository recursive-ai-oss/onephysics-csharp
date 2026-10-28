using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Bench;

internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--position-cost") { return PositionCost.Run(args); }
        BenchmarkSwitcher.FromAssembly(typeof(StepBench).Assembly).Run(args);
        return 0;
    }
}

[MemoryDiagnoser]
public class StepBench
{
    private ParticleScene _scene = null!;

    [GlobalSetup]
    public void Setup()
    {
        _scene = ParticleScene.CreatePile(2000, 1);
    }

    [Benchmark]
    public void Step() => _scene.Step(1f / 60f);
}
