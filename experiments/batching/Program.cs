using System;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace RecursiveAI.OnePhysics.Experiments.Batching;

/// <summary>
/// Command line for the batching experiment.
///   check [--small] [--steps N]            runs the pile at every width and thread count in-process and compares per-step digests
///   metrics [--small] [--steps N]          settling: contacts, kinetic energy, penetration and speed every 100 steps
///   bench [--small] [--colors N] [--steps N] [--warmup N] [--repeats N] [--threads 1,8,32] [--widths scalar,v128,...]
///                                          times collide and solve per step for each width and thread count (CSV on stdout)
/// </summary>
internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length == 0)
        {
            Console.Error.WriteLine("usage: BatchingExperiment check|bench [options]");
            return 2;
        }
        PileOptions options = PileOptions.Large;
        int steps = 0, warmup = 100, repeats = 3;
        int[] threads = { 1, 8 };
        Width[] widths = { Width.Scalar, Width.V128, Width.V256, Width.V512, Width.VectorT };
        for (int i = 1; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--small": options = PileOptions.Small; break;
                case "--colors": options = options with { Colors = int.Parse(args[++i], CultureInfo.InvariantCulture) }; break;
                case "--steps": steps = int.Parse(args[++i], CultureInfo.InvariantCulture); break;
                case "--warmup": warmup = int.Parse(args[++i], CultureInfo.InvariantCulture); break;
                case "--repeats": repeats = int.Parse(args[++i], CultureInfo.InvariantCulture); break;
                case "--threads": threads = Array.ConvertAll(args[++i].Split(','), s => int.Parse(s, CultureInfo.InvariantCulture)); break;
                case "--widths": widths = Array.ConvertAll(args[++i].Split(','), ParseWidth); break;
                default: Console.Error.WriteLine($"unknown argument {args[i]}"); return 2;
            }
        }
        PrintEnvironment();
        return args[0] switch
        {
            "check" => Check(options, steps == 0 ? 300 : steps),
            "metrics" => Metrics(options, steps == 0 ? 1000 : steps),
            "bench" => Bench(options, steps == 0 ? 200 : steps, warmup, repeats, threads, widths),
            _ => 2,
        };
    }

    private static Width ParseWidth(string s) => s switch
    {
        "scalar" => Width.Scalar,
        "v128" => Width.V128,
        "v256" => Width.V256,
        "v512" => Width.V512,
        "vector" => Width.VectorT,
        _ => throw new ArgumentException($"unknown width {s}"),
    };

    private static void PrintEnvironment()
    {
        Console.WriteLine($"# runtime {RuntimeInformation.FrameworkDescription} {RuntimeInformation.OSArchitecture} {RuntimeInformation.OSDescription}");
        Console.WriteLine($"# Vector128 accelerated={Vector128.IsHardwareAccelerated} Vector256={Vector256.IsHardwareAccelerated} Vector512={Vector512.IsHardwareAccelerated} Vector<float>.Count={System.Numerics.Vector<float>.Count}");
        Console.WriteLine($"# logical processors reported: {Environment.ProcessorCount}");
        try
        {
            foreach (string line in File.ReadLines("/proc/cpuinfo"))
            {
                if (line.StartsWith("model name", StringComparison.Ordinal)) { Console.WriteLine("# " + line); break; }
            }
        }
        catch (IOException) { }
    }

    private static int Check(PileOptions options, int steps)
    {
        // Negation of zero is where vector and scalar types can disagree on a sign; the kernels avoid it, this documents why.
        Console.WriteLine($"# -0 check: scalar bits {BitConverter.SingleToInt32Bits(-0f):x8}, Vector128 negate bits {BitConverter.SingleToInt32Bits((-Vector128<float>.Zero).GetElement(0)):x8}");

        long[] reference = Array.Empty<long>();
        string referenceName = string.Empty;
        int bad = 0;
        foreach (int t in new[] { 1, 3, 8 })
        {
            foreach (Width w in new[] { Width.Scalar, Width.V128, Width.V256, Width.V512, Width.VectorT })
            {
                using var pile = new Pile(options, t);
                var digests = new long[steps * 2];
                for (int s = 0; s < steps; s++)
                {
                    pile.Step(w);
                    (digests[2 * s], digests[2 * s + 1]) = pile.StateDigest();
                    if (s == 0 || s == steps / 4 || s == steps - 1)
                    {
                        var (ke, pen, speed) = pile.Metrics();
                        if (t == 1 && w == Width.Scalar)
                        {
                            Console.WriteLine($"step {s + 1}: contacts {pile.ContactCount} overflow {pile.OverflowCount} slots {pile.SlotCount} colours {pile.ColorsUsed} kinetic energy {ke:G6} max penetration {pen:G4} max speed {speed:G4}");
                        }
                    }
                }
                if (!pile.IsFinite()) { Console.WriteLine("NON-FINITE state"); bad++; }
                if (reference.Length == 0)
                {
                    reference = digests;
                    referenceName = $"{w}/t{t}";
                }
                int first = -1;
                for (int s = 0; s < steps && first < 0; s++)
                {
                    if (digests[2 * s] != reference[2 * s] || digests[2 * s + 1] != reference[2 * s + 1]) { first = s; }
                }
                Console.WriteLine(first < 0 ? $"equal    {w,-8} threads {t}  (vs {referenceName})" : $"DIFFERS  {w,-8} threads {t}  first differing step {first + 1}");
                if (first >= 0) { bad++; }
            }
        }
        return bad == 0 ? 0 : 1;
    }

    private static int Metrics(PileOptions options, int steps)
    {
        using var pile = new Pile(options, 1);
        for (int s = 1; s <= steps; s++)
        {
            pile.Step(Width.V256);
            if (s == 1 || s % 100 == 0)
            {
                var (ke, pen, speed) = pile.Metrics();
                Console.WriteLine($"step {s}: contacts {pile.ContactCount} overflow {pile.OverflowCount} colours {pile.ColorsUsed} kinetic energy {ke:G4} max penetration {pen:G4} max speed {speed:G4}");
            }
        }
        return pile.IsFinite() ? 0 : 1;
    }

    private static int Bench(PileOptions options, int steps, int warmup, int repeats, int[] threads, Width[] widths)
    {
        Console.WriteLine("width,threads,lanes,solve_ms_per_step,collide_ms_per_step,contacts,overflow");
        foreach (int t in threads)
        {
            foreach (Width w in widths)
            {
                double bestSolve = double.MaxValue, bestCollide = double.MaxValue;
                int contacts = 0, overflow = 0;
                for (int r = 0; r < repeats; r++)
                {
                    using var pile = new Pile(options, t);
                    for (int s = 0; s < warmup; s++) { pile.Step(w); }
                    long solve = 0, collide = 0;
                    for (int s = 0; s < steps; s++)
                    {
                        long a = Stopwatch.GetTimestamp();
                        pile.Collide();
                        long b = Stopwatch.GetTimestamp();
                        pile.SolveSubsteps(w);
                        long c = Stopwatch.GetTimestamp();
                        pile.StoreImpulses();
                        collide += b - a;
                        solve += c - b;
                    }
                    contacts = pile.ContactCount;
                    overflow = pile.OverflowCount;
                    bestSolve = Math.Min(bestSolve, solve * 1000.0 / Stopwatch.Frequency / steps);
                    bestCollide = Math.Min(bestCollide, collide * 1000.0 / Stopwatch.Frequency / steps);
                }
                int lanes = w switch { Width.Scalar => 1, Width.V128 => 4, Width.V256 => 8, Width.V512 => 16, _ => System.Numerics.Vector<float>.Count };
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{w},{t},{lanes},{bestSolve:F3},{bestCollide:F3},{contacts},{overflow}"));
            }
        }
        return 0;
    }
}
