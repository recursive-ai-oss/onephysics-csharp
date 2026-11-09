using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;

namespace Bar;

// Indicative decomposition timings (best of N, warm) at 1 and 8 threads for the generated torus and mug.
internal static class ToolsTiming
{
    internal static int Run()
    {
        var cases = new (string Name, TestMesh Mesh, ConvexDecompositionOptions Options)[]
        {
            ("torus-48x16", TestMeshes.Torus(1f, 0.35f, 48, 16), new ConvexDecompositionOptions { MaxHulls = 40, MaxConcavity = 0.01f }),
            ("mug", TestMeshes.Mug(), new ConvexDecompositionOptions { MaxHulls = 32, MaxConcavity = 0.05f }),
            ("torus-res64", TestMeshes.Torus(1f, 0.35f, 48, 16), new ConvexDecompositionOptions { MaxHulls = 40, MaxConcavity = 0.01f, Resolution = 64 }),
        };
        foreach (var (name, mesh, opt) in cases)
        {
            string? firstBits = null;
            foreach (int threads in new[] { 1, 8 })
            {
                var o = new ConvexDecompositionOptions { MaxHulls = opt.MaxHulls, MaxConcavity = opt.MaxConcavity, Resolution = opt.Resolution, MaxThreads = threads };
                ConvexDecomposition? d = null;
                double best = double.MaxValue;
                for (int rep = 0; rep < 4; rep++)
                {
                    var sw = Stopwatch.StartNew();
                    d = ConvexDecomposer.Decompose(mesh.V, mesh.I, o);
                    if (rep > 0) best = Math.Min(best, sw.Elapsed.TotalMilliseconds);
                }

                string bits = d!.Hulls.Count + ":" + string.Join(",", d.Hulls.Select(h => h.Vertices.Length));
                firstBits ??= bits;
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"decompose {name} threads={threads} ms={best:F1} hulls={d.Hulls.Count} maxConcavity={d.MaxConcavity:F4} thresholdMet={d.ThresholdMet} same={bits == firstBits}"));
            }
        }

        return 0;
    }
}
