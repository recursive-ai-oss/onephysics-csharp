using System.Globalization;
using System.Text;

namespace Bar;

/// <summary>
/// The bar verdict: per scene and thread count, this library's step time against the fastest managed .NET engine
/// (Bepu, Jitter2, Box2D.NET) that is stable on that scene. A managed engine whose run on the scene breaks the scene's
/// stability limits does not bind. Native engines are listed beside it for information.
/// </summary>
static class Verdict
{
    public const double Ratio = 1.15;
    /// <summary>A median under this is a run in which the bodies are asleep: the row measures sleeping, not stepping.</summary>
    public const double AsleepMs = 0.01;
    const string Ours = "onephysics";
    static readonly string[] Managed = ["bepu", "jitter2", "box2dnet"];
    static readonly string[] Native = ["box2d", "rapier2d", "jolt", "box3d", "rapier3d"];

    sealed record Cell(double Median, double Mean);

    /// <summary>Stability limits per scene (maximum allowed, robust metrics where they exist). Metrics not listed for a scene are not judged; nan, escapes and tunnelled must be 0 wherever they are reported.</summary>
    static readonly Dictionary<string, (string metric, double max)[]> Limits = new()
    {
        ["pyramid-20"] = [("maxSpeed", 0.1), ("maxPenetration", 0.01), ("jitter", 0.01), ("restKE", 0.0001), ("finalPenetration", 0.005)],
        ["pyramid-100"] = [("maxSpeed", 3), ("maxPenetration", 0.05), ("jitter", 0.2), ("restKE", 0.005), ("finalPenetration", 0.03)],
        ["large-pile"] = [("maxSpeed", 35), ("maxPenetration", 0.75), ("jitter", 0.15), ("restKE", 0.015), ("finalPenetration", 0.1)],
        ["tumbler"] = [("maxSpeed", 25), ("maxPenetration", 0.3), ("finalPenetration", 0.07)],
        ["joint-grid"] = [("maxSpeed", 5), ("jointError", 0.1)],
        ["chain"] = [("maxSpeed", 55), ("jointError", 0.4)],
        ["sleep-wake"] = [("maxSpeed", 25), ("maxPenetration", 0.4), ("finalPenetration", 0.1)],
        ["mixed-terrain"] = [("maxSpeed", 25), ("maxPenetration", 0.5), ("finalPenetration", 0.01)],
        ["bullets"] = [("maxSpeed", 160)],
        ["queries"] = [("maxSpeed", 40)],
        ["pyramid-3d-10"] = [("maxSpeed", 0.1), ("maxPenetration", 0.01), ("jitter", 0.01), ("restKE", 0.0001), ("finalPenetration", 0.005)],
        ["pyramid-3d-30"] = [("maxSpeed", 1), ("maxPenetration", 0.05), ("jitter", 0.01), ("restKE", 0.0001), ("finalPenetration", 0.005)],
        ["large-pile-3d"] = [("maxSpeed", 15), ("maxPenetration", 0.5), ("jitter", 0.01), ("restKE", 0.0001), ("finalPenetration", 0.05)],
        ["joint-chains-3d"] = [("maxSpeed", 45), ("jointError", 1.0)],
        ["joint-grid-3d"] = [("maxSpeed", 5), ("jointError", 0.1)],
        ["ragdoll-pile-3d"] = [("maxSpeed", 30), ("maxPenetration", 0.4), ("jointError", 0.3), ("restKE", 0.05), ("finalPenetration", 0.05)],
        ["mesh-terrain-3d"] = [("maxSpeed", 15), ("maxPenetration", 0.15), ("finalPenetration", 0.03)],
        ["heightfield-terrain-3d"] = [("maxSpeed", 15), ("maxPenetration", 0.15), ("finalPenetration", 0.05)],
        ["sleep-wake-3d"] = [("maxSpeed", 45), ("maxPenetration", 1.5), ("finalPenetration", 0.3)],
        ["queries-3d"] = [("maxSpeed", 25)],
        ["cloth-drape-3d"] = [("softMaxSpeed", 10), ("softPenetration", 0.07), ("softFinalPenetration", 0.015), ("softRestKE", 0.0003), ("edgeStrain", 0.15)],
        ["soft-pile-3d"] = [("softMaxSpeed", 10), ("softPenetration", 0.02), ("softFinalPenetration", 0.01), ("softRestKE", 0.02), ("volumeError", 0.05)],
        ["soft-rigid-3d"] = [("softMaxSpeed", 10), ("softPenetration", 0.07), ("softFinalPenetration", 0.01), ("softRestKE", 0.001), ("volumeError", 0.03), ("maxSpeed", 10), ("restKE", 0.1)],
        ["soft-mesh-3d"] = [("softMaxSpeed", 10), ("softPenetration", 0.02), ("softFinalPenetration", 0.01), ("softRestKE", 0.01), ("volumeError", 0.03)],
    };
    static readonly string[] MustBeZero = ["nan", "escapes", "tunnelled", "softNan", "softEscapes", "softTunnelled"];

    /// <summary>Engines recorded stable on a scene by the stability sweeps, used when a result set carries no metrics for that engine. Scenes not listed: every engine that runs them.</summary>
    static readonly Dictionary<string, string[]> RecordedStable = new()
    {
        ["pyramid-20"] = ["box2d", "rapier2d", "box2dnet"],
        ["pyramid-100"] = ["box2d", "rapier2d", "box2dnet"],
        ["large-pile"] = ["box2d", "rapier2d", "box2dnet"],
        ["tumbler"] = ["box2d", "rapier2d", "box2dnet"],
        ["sleep-wake"] = ["box2d", "rapier2d", "box2dnet"],
        ["mixed-terrain"] = ["box2d", "rapier2d", "box2dnet"],
        ["pyramid-3d-10"] = ["jolt", "box3d", "rapier3d", "bepu", "jitter2"],
        ["large-pile-3d"] = ["jolt", "box3d", "rapier3d", "bepu", "jitter2"],
        ["ragdoll-pile-3d"] = ["jolt", "box3d", "rapier3d", "bepu", "jitter2"],
        ["sleep-wake-3d"] = ["jolt", "box3d", "rapier3d", "bepu", "jitter2"],
        ["queries-3d"] = ["jolt", "box3d", "rapier3d", "bepu", "jitter2"],
        ["pyramid-3d-30"] = ["jolt", "box3d", "rapier3d"],
        ["joint-chains-3d"] = ["box3d", "rapier3d"],
        ["joint-grid-3d"] = ["jolt", "box3d", "rapier3d", "bepu"],
        ["mesh-terrain-3d"] = ["jolt", "box3d", "rapier3d", "bepu"],
        ["heightfield-terrain-3d"] = ["jolt", "rapier3d", "bepu"],
    };

    /// <summary>True when the engine's metrics (if the result sets have any) are within the scene's limits; null when it has none.</summary>
    static bool? StableByMetrics(Dictionary<(string scene, string engine, string metric), double> m, string scene, string engine)
    {
        var mine = m.Where(kv => kv.Key.scene == scene && kv.Key.engine == engine).ToList();
        if (mine.Count == 0) return null;
        foreach (var kv in mine)
        {
            double v = kv.Value;
            if (double.IsNaN(v)) return false;
            if (MustBeZero.Contains(kv.Key.metric, StringComparer.Ordinal) && v != 0) return false;
            if (Limits.TryGetValue(scene, out var lim) && lim.FirstOrDefault(l => l.metric == kv.Key.metric) is { metric: not null } l2 && v > l2.max) return false;
        }
        return true;
    }

    public static (bool stable, string how) IsStable(Dictionary<(string, string, string), double> metrics, string scene, string engine)
    {
        var byMetrics = StableByMetrics(metrics, scene, engine);
        if (byMetrics is { } b) return (b, "metrics");
        if (RecordedStable.TryGetValue(scene, out var rec)) return (rec.Contains(engine), "recorded");
        return (true, "assumed");
    }

    /// <param name="timingDirs">Result directories (each with timing.csv); a later directory replaces an earlier row for the same scene, engine and thread count.</param>
    /// <param name="metricFiles">metrics.csv files from stability runs; empty is fine.</param>
    public static string Render(IEnumerable<string> timingDirs, IEnumerable<string> metricFiles, out string csv)
    {
        var cells = new Dictionary<(string scene, string engine, int threads), Cell>();
        var versions = new Dictionary<string, string>();
        foreach (var d in timingDirs)
        {
            var p = Path.Combine(d, "timing.csv");
            if (!File.Exists(p)) continue;
            foreach (var l in File.ReadLines(p).Skip(1))
            {
                var c = Runner.CsvSplit(l);
                cells[(c[0], c[1], int.Parse(c[3], CultureInfo.InvariantCulture))] = new Cell(D(c[6]), D(c[11]));
                versions[c[1]] = c[2];
            }
        }
        var metrics = new Dictionary<(string, string, string), double>();
        foreach (var f in metricFiles)
            if (File.Exists(f))
                foreach (var l in File.ReadLines(f).Skip(1))
                {
                    var c = l.Split(',');
                    if (c.Length >= 5) metrics[(c[0], c[1], c[3])] = D(c[4]);
                }

        var ourRows = cells.Keys.Where(k => k.engine == Ours).OrderBy(k => k.scene, StringComparer.Ordinal).ThenBy(k => k.threads).ToList();
        var sb = new StringBuilder();
        var cs = new StringBuilder("scene,threads,ours_ms,binding_engine,binding_ms,binding_mean_ms,binding_asleep,ratio,verdict,native_engine,native_ms,native_ratio\n");
        sb.AppendLine($"## Bar verdict (within {Ratio:0.00}× of the fastest stable managed engine)\n");
        sb.AppendLine("Binding reference: the fastest of Bepu, Jitter2 and Box2D.NET (a single-threaded engine: its 1-thread row at every thread count) that is stable on the scene. Stability comes from the scene limits applied to the supplied metrics; where an engine has none, from the recorded sweeps (`recorded`), else it is assumed stable (`assumed`). A thread count with no managed row (the managed runs did not include it) has no verdict. A reference median under 0.01 ms is a run asleep: the gap is a sleep-timing gap, not a speed loss. Native engines are information only. For the scenes with queries (`queries`, `queries-3d`) the step time is the settle phase before the queries; query throughput is in the query table of the summary.\n");
        sb.AppendLine("| scene | threads | ours ms | binding | ref median ms (mean) | ratio | verdict | fastest native | native ms | ratio |\n|---|---|---|---|---|---|---|---|---|---|");
        int pass = 0, miss = 0, gap = 0, none = 0;
        foreach (var (scene, _, th) in ourRows)
        {
            var ours = cells[(scene, Ours, th)];
            var cands = new List<(string eng, Cell cell, string how)>();
            foreach (var e in Managed)
            {
                if (!cells.TryGetValue((scene, e, th), out var cell) && !(e == "box2dnet" && cells.TryGetValue((scene, e, 1), out cell))) continue;
                var (stable, how) = IsStable(metrics, scene, e);
                if (stable) cands.Add((e, cell!, how));
            }
            string verdict, bind = "—", refCell = "—", ratioTxt = "—", bindEng = "", bindMs = "", bindMean = "", asleepTxt = "", ratioCsv = "";
            if (cands.Count == 0)
            {
                bool anyManaged = Managed.Any(e => cells.ContainsKey((scene, e, th)) || (e == "box2dnet" && cells.ContainsKey((scene, e, 1))));
                verdict = anyManaged ? "no binding row (no managed engine stable)" : "no managed row";
                none++;
            }
            else
            {
                var best = cands.OrderBy(c => c.cell.Median).ThenBy(c => c.eng, StringComparer.Ordinal).First();
                bool asleep = best.cell.Median < AsleepMs;
                double r = ours.Median / best.cell.Median;
                bindEng = best.eng; bindMs = F(best.cell.Median); bindMean = F(best.cell.Mean); asleepTxt = asleep ? "yes" : "no"; ratioCsv = F(r);
                bind = best.eng + (best.how == "metrics" ? "" : $" ({best.how})") + (asleep ? " **asleep**" : "");
                refCell = $"{best.cell.Median:F3} ({best.cell.Mean:F3})";
                ratioTxt = $"{r:F2}×";
                if (r <= Ratio) { verdict = "PASS"; pass++; }
                else if (asleep)
                {
                    verdict = "SLEEP-GAP";
                    var awake = cands.Where(c => c.cell.Median >= AsleepMs).OrderBy(c => c.cell.Median).FirstOrDefault();
                    if (awake.eng != null) verdict += $" (vs {awake.eng} {awake.cell.Median:F3}: {(ours.Median / awake.cell.Median <= Ratio ? "PASS" : "MISS")} {ours.Median / awake.cell.Median:F2}×)";
                    gap++;
                }
                else { verdict = "MISS"; miss++; }
            }
            string nat = "—", natMs = "—", natRatio = "—", natEngCsv = "", natMsCsv = "", natRatioCsv = "";
            var natRows = Native.Where(e => cells.ContainsKey((scene, e, th))).Select(e => (e, cell: cells[(scene, e, th)])).OrderBy(x => x.cell.Median).ThenBy(x => x.e, StringComparer.Ordinal).ToList();
            if (natRows.Count > 0)
            {
                var n = natRows[0];
                nat = n.e + (n.cell.Median < AsleepMs ? " asleep" : ""); natMs = $"{n.cell.Median:F3} ({n.cell.Mean:F3})"; natRatio = $"{ours.Median / n.cell.Median:F2}×";
                natEngCsv = n.e; natMsCsv = F(n.cell.Median); natRatioCsv = F(ours.Median / n.cell.Median);
            }
            sb.AppendLine($"| {scene} | {th} | {ours.Median:F3} | {bind} | {refCell} | {ratioTxt} | {verdict} | {nat} | {natMs} | {natRatio} |");
            cs.AppendLine($"{scene},{th},{F(ours.Median)},{bindEng},{bindMs},{bindMean},{asleepTxt},{ratioCsv},\"{verdict}\",{natEngCsv},{natMsCsv},{natRatioCsv}");
        }
        sb.AppendLine($"\nRows: {pass} PASS, {miss} MISS, {gap} sleep-timing gaps, {none} without a binding row.");
        csv = cs.ToString();
        return sb.ToString();
    }

    static double D(string s) => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>`bar verdict --timing DIR[,DIR...] [--metrics FILE[,FILE...]] [--out DIR]`.</summary>
    public static int Run(Opts o)
    {
        var dirs = o.Get("timing", "").Split(',', StringSplitOptions.RemoveEmptyEntries);
        var mets = o.Get("metrics", "").Split(',', StringSplitOptions.RemoveEmptyEntries);
        if (dirs.Length == 0) { Console.Error.WriteLine("verdict needs --timing DIR[,DIR...]"); return 2; }
        var md = Render(dirs, mets, out var csv);
        var outDir = o.Get("out", "");
        if (outDir != "") { Directory.CreateDirectory(outDir); File.WriteAllText(Path.Combine(outDir, "verdict.md"), md); File.WriteAllText(Path.Combine(outDir, "verdict.csv"), csv); }
        Console.WriteLine(md);
        return 0;
    }

    /// <summary>Self-test on a synthetic result set: the binding engine skips an unstable faster one, an asleep reference is a sleep-timing gap, a scene with no stable managed engine has no binding row.</summary>
    public static int SelfTest()
    {
        var dir = Path.Combine(Path.GetTempPath(), "bar-verdict-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            File.WriteAllText(Path.Combine(dir, "timing.csv"), string.Join('\n',
                "scene,engine,version,threads,bodies,n,median_ms,p90,p99,min,max,mean_ms",
                "pyramid-3d-30,onephysics,x,1,1,1,0.2,0,0,0,0,0.2",
                "pyramid-3d-30,bepu,x,1,1,1,29,0,0,0,0,29",
                "pyramid-3d-30,jitter2,x,1,1,1,0.01163,0,0,0,0,4",
                "joint-chains-3d,onephysics,x,1,1,1,0.6,0,0,0,0,0.6",
                "joint-chains-3d,bepu,x,1,1,1,0.1,0,0,0,0,0.1",
                "joint-chains-3d,jitter2,x,1,1,1,0.25,0,0,0,0,0.25",
                "pyramid-20,onephysics,x,1,1,1,0.005,0,0,0,0,0.005",
                "pyramid-20,box2dnet,x,1,1,1,0.00066,0,0,0,0,0.0008",
                "large-pile,onephysics,x,1,1,1,1.7,0,0,0,0,1.7",
                "large-pile,box2dnet,x,1,1,1,16.4,0,0,0,0,15",
                "large-pile,box2d,x,1,1,1,0.94,0,0,0,0,1") + "\n");
            var md = Render([dir], [], out _);
            int bad = 0;
            void Expect(string line, string frag) { if (!md.Split('\n').Any(l => l.Contains(line, StringComparison.Ordinal) && l.Contains(frag, StringComparison.Ordinal))) { Console.Error.WriteLine($"verdict selftest: '{line}' lacks '{frag}'"); bad++; } }
            Expect("| pyramid-3d-30 |", "no binding row");
            Expect("| joint-chains-3d |", "no binding row");
            Expect("| pyramid-20 |", "SLEEP-GAP");
            Expect("| large-pile |", "PASS");
            Expect("| large-pile |", "box2d ");
            return bad;
        }
        finally { Directory.Delete(dir, true); }
    }
}
