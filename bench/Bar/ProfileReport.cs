using System.Globalization;
using System.Text;

namespace Bar;

/// <summary>One engine's per-stage medians on one scene, as the driver reported them under the engine's own stage names.</summary>
sealed record ProfileRow(string Scene, string Engine, int Threads, double StepMedianMs, Dictionary<string, double> Stages, Dictionary<string, double> Counts);

/// <summary>
/// Maps each engine's own stage names onto common buckets and renders the side-by-side table. The mapping is documented in docs/bench.md; stages that contain other stages are left out of the sums
/// (they are shown in the native table) so nothing is counted twice, and "other" is the engine's own step total minus the mapped stages.
/// </summary>
static class ProfileReport
{
    public const string BroadPhase = "broadphase/pairs", Narrow = "narrowphase", Prepare = "solver prepare", Solve = "solve+integrate", Finalize = "finalize/store", Islands = "islands/sleep", Other = "other";
    public static readonly string[] Buckets = [BroadPhase, Narrow, Prepare, Solve, Finalize, Islands];

    // Stage name -> bucket, per engine. A stage that is missing here and is not an aggregate counts as "other".
    static readonly Dictionary<string, Dictionary<string, string>> Map = new()
    {
        ["onephysics"] = new()
        {
            ["boxes"] = BroadPhase, ["pairs"] = BroadPhase, ["narrowphase"] = Narrow, ["apply"] = Narrow, ["prepare"] = Prepare, ["solve"] = Solve,
            ["finalize"] = Finalize, ["store"] = Finalize, ["sleep"] = Islands, ["coloring"] = Other, ["events"] = Other,
        },
        ["box2d"] = new()
        {
            ["pairs"] = BroadPhase, ["refit"] = BroadPhase, ["collide"] = Narrow, ["solverSetup"] = Prepare, ["prepareConstraints"] = Prepare,
            ["integrateVelocities"] = Solve, ["warmStart"] = Solve, ["solveImpulses"] = Solve, ["integratePositions"] = Solve, ["relaxImpulses"] = Solve, ["restitution"] = Solve,
            ["storeImpulses"] = Finalize, ["transforms"] = Finalize, ["hitEvents"] = Finalize, ["jointEvents"] = Finalize, ["bullets"] = Finalize, ["sensors"] = Finalize, ["sensorHits"] = Finalize,
            ["splitIslands"] = Islands, ["sleepIslands"] = Islands,
        },
        ["rapier"] = new()
        {
            ["broad_phase"] = BroadPhase, ["narrow_phase"] = Narrow, ["assembly"] = Prepare, ["velocity_resolution"] = Solve, ["velocity_update"] = Solve,
            ["ccd"] = Finalize, ["update"] = Finalize, ["island_construction"] = Islands,
        },
    };

    // Stages that contain other stages: reported natively, never summed.
    static readonly Dictionary<string, HashSet<string>> Aggregates = new()
    {
        ["onephysics"] = [],
        ["box2d"] = ["step", "solve", "constraints"],
        ["rapier"] = ["step", "collision_detection", "solver"],
    };

    static string Family(string engine) => engine.StartsWith("rapier", StringComparison.Ordinal) ? "rapier" : engine;

    /// <summary>Bucket totals (ms) for one row plus the engine's own step total: the step total is the engine's "step" stage, and "other" is whatever of it no bucket claims (never negative; a negative remainder means the engine's own stages overlap and is reported as 0).</summary>
    public static Dictionary<string, double> Bucketize(string engine, IReadOnlyDictionary<string, double> stages)
    {
        var fam = Family(engine);
        var r = new Dictionary<string, double>();
        foreach (var b in Buckets) r[b] = 0;
        r[Other] = 0;
        if (!Map.TryGetValue(fam, out var map)) return r;
        double mapped = 0, explicitOther = 0;
        foreach (var (name, ms) in stages)
        {
            if (name == "step" || Aggregates[fam].Contains(name)) continue;
            if (map.TryGetValue(name, out var bucket))
            {
                if (bucket == Other) explicitOther += ms; else { r[bucket] += ms; mapped += ms; }
            }
        }
        double total = stages.TryGetValue("step", out var t) ? t : mapped + explicitOther;
        r[Other] = Math.Max(0, total - mapped);
        r["step total"] = total;
        return r;
    }

    /// <summary>Reads prof_NAME_ms and count_NAME lines of a driver result.</summary>
    public static void Parse(IEnumerable<string> lines, Dictionary<string, double> prof, Dictionary<string, double> counts)
    {
        foreach (var l in lines)
        {
            int i = l.IndexOf('=', StringComparison.Ordinal);
            if (i <= 0) continue;
            var k = l[..i];
            if (!double.TryParse(l[(i + 1)..], NumberStyles.Float, CultureInfo.InvariantCulture, out var v)) continue;
            if (k.StartsWith("prof_", StringComparison.Ordinal) && k.EndsWith("_ms", StringComparison.Ordinal)) prof[k[5..^3]] = v;
            else if (k.StartsWith("count_", StringComparison.Ordinal)) counts[k[6..]] = v;
        }
    }

    public static string Render(List<ProfileRow> rows, string meta)
    {
        var sb = new StringBuilder("# Per-stage comparison\n\n");
        sb.AppendLine("```").Append(meta).AppendLine("```\n");
        sb.AppendLine("Medians over the measured steps, in ms. Each engine's own stage names are mapped onto common buckets (docs/bench.md); \"step total\" is the engine's own step timer, so the profile build's overhead is included in it. Ours is built in the Profile configuration.\n");
        foreach (var g in rows.GroupBy(r => (r.Scene, r.Threads)).OrderBy(g => g.Key.Scene, StringComparer.Ordinal).ThenBy(g => g.Key.Threads))
        {
            var engines = g.OrderBy(r => r.Engine, StringComparer.Ordinal).ToList();
            var b = engines.ToDictionary(r => r.Engine, r => Bucketize(r.Engine, r.Stages));
            sb.AppendLine($"## {g.Key.Scene}, {g.Key.Threads} thread(s)\n");
            sb.AppendLine($"| bucket | {string.Join(" | ", engines.Select(e => e.Engine))} |\n|---|{string.Concat(engines.Select(_ => "---|"))}");
            foreach (var name in Buckets.Append(Other).Append("step total"))
                sb.AppendLine($"| {name} | {string.Join(" | ", engines.Select(e => b[e.Engine].TryGetValue(name, out var v) && e.Stages.Count > 0 ? v.ToString("F3", CultureInfo.InvariantCulture) : "-"))} |");
            sb.AppendLine($"| step median (driver clock) | {string.Join(" | ", engines.Select(e => e.StepMedianMs.ToString("F3", CultureInfo.InvariantCulture)))} |");
            if (engines.Count > 1 && engines.Any(e => e.Engine == "onephysics"))
            {
                sb.AppendLine($"\nOurs divided by each reference, per bucket:\n\n| bucket | {string.Join(" | ", engines.Where(e => e.Engine != "onephysics").Select(e => e.Engine))} |\n|---|{string.Concat(engines.Where(e => e.Engine != "onephysics").Select(_ => "---|"))}");
                var ours = b["onephysics"];
                foreach (var name in Buckets.Append(Other).Append("step total"))
                    sb.AppendLine($"| {name} | {string.Join(" | ", engines.Where(e => e.Engine != "onephysics").Select(e => b[e.Engine].TryGetValue(name, out var rv) && rv > 0.005 && ours.TryGetValue(name, out var ov) ? (ov / rv).ToString("F1", CultureInfo.InvariantCulture) + "x" : "-"))} |");
            }
            sb.AppendLine("\nNative stages:\n");
            foreach (var e in engines)
                sb.AppendLine($"- {e.Engine}: {string.Join(", ", e.Stages.Select(kv => $"{kv.Key} {kv.Value.ToString("F3", CultureInfo.InvariantCulture)}"))}{(e.Counts.Count > 0 ? "; counters " + string.Join(", ", e.Counts.Select(kv => $"{kv.Key} {kv.Value.ToString("F0", CultureInfo.InvariantCulture)}")) : "")}");
            sb.AppendLine();
        }
        return sb.ToString();
    }

    public static int SelfTest()
    {
        int fail = 0;
        void Check(bool ok, string what) { if (!ok) { Console.Error.WriteLine("profile selftest failed: " + what); fail++; } }
        var prof = new Dictionary<string, double>(); var counts = new Dictionary<string, double>();
        Parse(["engine=box2d", "prof_step_ms=1.5", "prof_pairs_ms=0.25", "prof_collide_ms=0.5", "prof_solve_ms=0.7", "prof_warmStart_ms=0.1", "prof_solveImpulses_ms=0.3", "prof_transforms_ms=0.05", "count_contactCount=9650.0", "prof_bad_ms=abc", "noise"], prof, counts);
        Check(prof.Count == 7 && prof["pairs"] == 0.25 && counts["contactCount"] == 9650, "parse");
        var b = Bucketize("box2d", prof);
        Check(Math.Abs(b[BroadPhase] - 0.25) < 1e-9 && Math.Abs(b[Narrow] - 0.5) < 1e-9, "box2d broadphase/narrowphase");
        Check(Math.Abs(b[Solve] - 0.4) < 1e-9, "box2d solve leaves are summed, the aggregate is not");
        Check(Math.Abs(b[Finalize] - 0.05) < 1e-9 && Math.Abs(b[Other] - 0.3) < 1e-9 && b["step total"] == 1.5, "box2d finalize and other");
        var rap = new Dictionary<string, double> { ["step"] = 2, ["collision_detection"] = 1.2, ["broad_phase"] = 0.4, ["narrow_phase"] = 0.7, ["solver"] = 0.7, ["assembly"] = 0.2, ["velocity_resolution"] = 0.4 };
        var rb = Bucketize("rapier3d", rap);
        Check(Math.Abs(rb[Prepare] - 0.2) < 1e-9 && Math.Abs(rb[Solve] - 0.4) < 1e-9 && Math.Abs(rb[Other] - 0.3) < 1e-9, "rapier buckets");
        var ours = new Dictionary<string, double> { ["boxes"] = 0.1, ["pairs"] = 0.2, ["narrowphase"] = 0.4, ["apply"] = 0.1, ["coloring"] = 0.05, ["prepare"] = 0.1, ["solve"] = 0.8, ["finalize"] = 0.05, ["store"] = 0.05, ["sleep"] = 0.02, ["events"] = 0.01, ["step"] = 1.9 };
        var ob = Bucketize("onephysics", ours);
        Check(Math.Abs(ob[BroadPhase] - 0.3) < 1e-9 && Math.Abs(ob[Narrow] - 0.5) < 1e-9 && Math.Abs(ob[Finalize] - 0.1) < 1e-9, "ours buckets");
        Check(Math.Abs(ob[Other] - (1.9 - 1.82 + 0)) < 1e-9, "ours other = step total minus mapped (coloring and events are not mapped)");
        Check(Render([new ProfileRow("s", "box2d", 1, 1.5, prof, counts), new ProfileRow("s", "onephysics", 1, 1.9, ours, new())], "m\n").Contains("Ours divided", StringComparison.Ordinal), "render");
        return fail;
    }
}
