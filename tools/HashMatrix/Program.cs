using System;
using System.Globalization;
using System.IO;
using System.Text;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Runs every registered scenario and prints a manifest: one <c>name sha256</c> line per scenario in ordinal name
/// order, then a <c>manifest sha256</c> line hashing those lines. Two platforms agree when their manifest lines are
/// identical. With <c>--steps</c> the per-step digests follow the manifest as <c>step name index digest16</c> lines.
/// Options: <c>--threads N</c> (default 1; always explicit, never read from the machine), <c>--filter PREFIX</c>,
/// <c>--out FILE</c> (manifest only), <c>--steps</c>, <c>--list</c>.
/// </summary>
public static class Program
{
    /// <summary>Entry point.</summary>
    public static int Main(string[] args)
    {
        int threads = 1;
        string filter = string.Empty;
        string? outFile = null;
        bool list = false;
        bool steps = false;
        for (int i = 0; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--threads" when i + 1 < args.Length:
                    threads = int.Parse(args[++i], CultureInfo.InvariantCulture);
                    break;
                case "--filter" when i + 1 < args.Length:
                    filter = args[++i];
                    break;
                case "--out" when i + 1 < args.Length:
                    outFile = args[++i];
                    break;
                case "--list":
                    list = true;
                    break;
                case "--steps":
                    steps = true;
                    break;
                default:
                    Console.Error.WriteLine($"unknown argument: {args[i]}");
                    return 2;
            }
        }
        if (threads < 1)
        {
            Console.Error.WriteLine("--threads must be at least 1");
            return 2;
        }

        var manifest = new StringBuilder();
        var stepLines = new StringBuilder();
        foreach (var (name, run) in Scenarios.All())
        {
            if (!name.StartsWith(filter, StringComparison.Ordinal))
            {
                continue;
            }
            if (list)
            {
                manifest.Append(name).Append('\n');
                continue;
            }
            var ctx = new ScenarioContext(threads);
            run(ctx);
            byte[] digest = ctx.Finish();
            manifest.Append(name).Append(' ').Append(Convert.ToHexStringLower(digest)).Append('\n');
            if (steps)
            {
                for (int s = 0; s < ctx.Steps; s++)
                {
                    stepLines.Append("step ").Append(name).Append(' ').Append(s.ToString(CultureInfo.InvariantCulture)).Append(' ').Append(ctx.StepDigest(s)).Append('\n');
                }
            }
        }
        if (!list)
        {
            string lines = manifest.ToString();
            manifest.Append("manifest ").Append(Convert.ToHexStringLower(Sha256.Hash(Encoding.UTF8.GetBytes(lines)))).Append('\n');
        }
        if (outFile is not null)
        {
            File.WriteAllText(outFile, manifest.ToString());
        }
        Console.Out.Write(manifest.ToString());
        Console.Out.Write(stepLines.ToString());
        return 0;
    }
}
