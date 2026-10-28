using System.Diagnostics;
using System.Globalization;
using System.Text;

namespace Bar;

/// <summary>
/// Per-stage times of this library's step, read from the stage boundaries the library reports when it is built in the Profile configuration (the STAGE_TIMERS symbol). The clock lives here, in the benchmark.
/// Enabled by the OP_PROFILE environment variable, like the reference drivers' profile output; a normal build reports the stages as unavailable.
/// </summary>
static class StageProfile
{
    public static readonly string[] Names = ["boxes", "pairs", "narrowphase", "apply", "coloring", "prepare", "solve", "finalize", "store", "sleep", "events"];

    public static bool Requested => Environment.GetEnvironmentVariable("OP_PROFILE") != null;

#if STAGE_TIMERS
    public const bool Available = true;
    static readonly long[] stamp = new long[16];
    static readonly int[] order = new int[16];
    static int marks;
    static readonly List<double[]> perStep = [];

    /// <summary>Starts recording boundaries.</summary>
    public static void Install()
    {
        RecursiveAI.OnePhysics.LowLevel.StageProbe.OnMark = stage => { stamp[marks] = Stopwatch.GetTimestamp(); order[marks++] = stage; };
    }

    /// <summary>Call before a step.</summary>
    public static void Begin() => marks = 0;

    /// <summary>Call after a step: turns the boundaries into stage durations (ms); a step the library cut short (no boundary reached) is dropped. The last boundary is the end of the step.</summary>
    public static void End(int step, int warmup)
    {
        if (step < warmup || marks < 2) return;
        var d = new double[Names.Length];
        for (int i = 0; i + 1 < marks; i++)
            if (order[i] < d.Length) d[order[i]] += (stamp[i + 1] - stamp[i]) * 1000.0 / Stopwatch.Frequency;
        perStep.Add(d);
    }

    /// <summary>Appends prof_*_ms lines holding the median of each stage over the recorded steps.</summary>
    public static void Write(StringBuilder sb)
    {
        if (perStep.Count == 0) return;
        double total = 0;
        var totals = perStep.Select(d => d.Sum()).OrderBy(x => x).ToArray();
        total = totals[totals.Length / 2];
        for (int k = 0; k < Names.Length; k++)
        {
            var col = perStep.Select(d => d[k]).OrderBy(x => x).ToArray();
            sb.Append(CultureInfo.InvariantCulture, $"prof_{Names[k]}_ms={col[col.Length / 2]:F5}\n");
        }
        sb.Append(CultureInfo.InvariantCulture, $"prof_step_ms={total:F5}\n");
    }
#else
    public const bool Available = false;
    public static void Install() { Console.Error.WriteLine("onephysics stage times need a build in the Profile configuration (dotnet build -c Profile); reporting none"); }
    public static void Begin() { }
    public static void End(int step, int warmup) { }
    public static void Write(StringBuilder sb) { }
#endif
}
