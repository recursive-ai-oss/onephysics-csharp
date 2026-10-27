using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.Experiments.Batching;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// The width-independence experiment as scenarios: the same pile, solved with scalar, 128-, 256- and 512-bit kernels and
/// <c>Vector&lt;T&gt;</c>, one scenario per width. Each hashes only the world, never the width, so within one manifest the
/// five lines of a family must be equal, and across columns every line must match too. "pile" is 4096 spheres for 1000
/// steps (well over 10 000 contacts once settled); "small" is 512 spheres for 200 steps, for slow platforms.
/// </summary>
internal static class BatchingScenarios
{
    private static readonly (string Name, Width Width)[] s_widths =
    {
        ("scalar", Width.Scalar), ("v128", Width.V128), ("v256", Width.V256), ("v512", Width.V512), ("vector", Width.VectorT),
    };

    [ModuleInitializer]
    internal static void Init()
    {
        foreach (var (name, width) in s_widths)
        {
            Width w = width;
            Scenarios.Register($"batching/pile/{name}", ctx => Run(ctx, PileOptions.Large, 1000, w));
            Scenarios.Register($"batching/small/{name}", ctx => Run(ctx, PileOptions.Small, 200, w));
        }
    }

    private static void Run(ScenarioContext ctx, PileOptions options, int steps, Width width)
    {
        using var pile = new Pile(options, ctx.Threads);
        for (int step = 0; step < steps; step++)
        {
            pile.Step(width);
            (long h1, long h2) = pile.StateDigest();
            ctx.Write(h1);
            ctx.Write(h2);
            ctx.Write(pile.ContactCount);
            ctx.Write(pile.OverflowCount);
            ctx.EndStep();
        }
    }
}
