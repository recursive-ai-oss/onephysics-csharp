using System.Globalization;
using System.Text;

namespace Bar;

using Scene = RecursiveAI.OnePhysics.Scene.Scene;

/// <summary>
/// The stability gate: one list of everything that is judged for stability, and one table of the results.
///   bar gate-plan --scene-dir DIR [--threads 1,8]    one line per item: id TAB arguments for `bar` ({ITEM} is the item's output directory)
///   bar gate-table --in DIR --scene-dir DIR          reads DIR/items/*, prints the TSV table (scene, dim, ...) and a summary on stderr
/// The scene lists come from the code: the generated scene files, the scene-name arrays of the shared scene definitions
/// and the stability commands themselves. tools/bench/stability-gate.sh runs the plan and builds the table.
/// </summary>
static class Gate
{
    /// <summary>Stability commands that judge their own scenes (every scene of the command when run without --scene).</summary>
    static readonly string[] Commands =
    [
        "fields-stability", "decomposed-pile-stability", "soft2d-stability", "cloth-stability", "destruction-stability",
        "authoring-stability", "vehicles2d-stability", "active-stability", "walkers3d-stability", "railbike-stability",
        "articulation-stability", "actuators-stability", "fluid-stability", "water-stability", "swim-stability",
        "worldscale-stability", "character-abilities-stability", "terrain-edit-stability",
    ];
    /// <summary>Commands that ignore --threads (fixed single-thread scenes).</summary>
    static readonly string[] NoThreads = ["fields-stability", "decomposed-pile-stability", "soft2d-stability", "destruction-stability"];

    /// <summary>Limits for scenes whose limits are not in the bar verdict's table (those have no managed reference): docs/spec/scenes.md.</summary>
    static readonly Dictionary<string, (string metric, double max)[]> ExtraMax = new()
    {
        ["characters"] = [("maxSpeed", 20), ("maxPenetration", 0.25), ("charJitter", 0.01)],
        ["characters-3d"] = [("maxSpeed", 25), ("maxPenetration", 0.05), ("charJitter", 0.01), ("slopeViolations", 0)],
        ["vehicles-3d"] = [("maxSpeed", 25), ("maxPenetration", 0.05), ("rollovers", 0), ("wheelThroughGround", 0), ("suspensionJitter", 0.01)],
        ["buoyancy-3d"] = [("maxSpeed", 8), ("equilibriumDepthError", 0.02), ("sinkersAfloat", 0)],
        ["tracked-3d"] = [("maxSpeed", 8), ("maxPenetration", 0.05), ("suspensionJitter", 0.01), ("rollovers", 0), ("wheelThroughGround", 0)],
        ["motorcycle-3d"] = [("maxSpeed", 12), ("maxPenetration", 0.05), ("suspensionJitter", 0.01), ("fallen", 0), ("leanError", 0.05), ("wheelThroughGround", 0)],
        ["ragdoll-powered-3d"] = [("maxSpeed", 10), ("poseErrorP95", 0.10), ("fallen", 0), ("jointError", 0.05)],
    };
    static readonly Dictionary<string, (string metric, double min)[]> ExtraMin = new()
    {
        ["characters-3d"] = [("stairSuccess", 1)],
    };
    static readonly string[] AlwaysZero = ["nan", "escapes", "tunnelled", "softNan", "softEscapes", "softTunnelled"];

    public static int Plan(Opts o)
    {
        string sceneDir = o.Get("scene-dir", "scenes/generated");
        var threadList = o.Get("threads", "1").Split(',');
        foreach (var p in Directory.GetFiles(sceneDir, "*.scene").OrderBy(x => x, StringComparer.Ordinal))
        {
            string name = Path.GetFileNameWithoutExtension(p);
            var scene = Scene.Load(p);
            // Only scenes with a driver of ours; a scene that names other engines only is listed too (the engines list always includes ours).
            Console.WriteLine($"run-{name}\trun --mode stability --engines onephysics --threads 1 --scenes {name} --scene-dir {sceneDir} --out {{ITEM}}");
        }
        foreach (var t in threadList)
        {
            string sfx = threadList.Length > 1 ? $"-t{t}" : "";
            foreach (var c in Commands)
                Console.WriteLine($"{c}{sfx}\t{c}" + (NoThreads.Contains(c) ? "" : $" --threads {t}"));
            foreach (var n in RodRuns.Names) Console.WriteLine($"rod-run-{n}{sfx}\trod-run --scene {n} --threads {t}");
            foreach (var n in PulleyScene.Names) Console.WriteLine($"pulley-run-{n}{sfx}\tpulley-run --scene {n} --threads {t}");
        }
        return 0;
    }

    sealed record Row(string Source, string Scene, string Dim, string Threads, string Metric, string Threshold, string Ours, string Verdict);

    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    static string DimOf(string scene, string sceneDir)
    {
        var f = Path.Combine(sceneDir, scene + ".scene");
        if (File.Exists(f)) return Scene.Load(f).Dim.ToString(CultureInfo.InvariantCulture);
        var m = System.Text.RegularExpressions.Regex.Match(scene, @"-([23])d(/|$)");
        return m.Success ? m.Groups[1].Value : "?";
    }

    public static int Table(Opts o)
    {
        string dir = o.Get("in", ".");
        string sceneDir = o.Get("scene-dir", "scenes/generated");
        var rows = new List<Row>();
        var itemsDir = Path.Combine(dir, "items");
        foreach (var item in Directory.GetDirectories(itemsDir).Concat(Directory.GetFiles(itemsDir, "*.log").Where(f => !Directory.Exists(Path.ChangeExtension(f, null)))).OrderBy(x => x, StringComparer.Ordinal))
        {
            string id = Path.GetFileName(item);
            if (Directory.Exists(item)) { RunItem(id, item, sceneDir, rows); continue; }
            id = Path.GetFileNameWithoutExtension(item);
            CommandItem(id, item, sceneDir, rows);
        }
        var sb = new StringBuilder("source\tscene\tdim\tthreads\tmetric\tthreshold\tours\tverdict\n");
        foreach (var r in rows) sb.Append($"{r.Source}\t{r.Scene}\t{r.Dim}\t{r.Threads}\t{r.Metric}\t{r.Threshold}\t{r.Ours}\t{r.Verdict}\n");
        var outFile = o.Get("out", "");
        if (outFile != "") File.WriteAllText(outFile, sb.ToString()); else Console.Write(sb.ToString());
        var sceneRows = rows.Where(r => r.Metric == "(scene)").ToList();
        int fail = sceneRows.Count(r => r.Verdict != "PASS");
        Console.Error.WriteLine($"gate table: {sceneRows.Select(r => r.Scene + "@" + r.Threads).Distinct().Count()} scene results ({sceneRows.Select(r => r.Scene).Distinct().Count()} distinct scenes), {sceneRows.Count - fail} PASS, {fail} not PASS");
        foreach (var r in sceneRows.Where(r => r.Verdict != "PASS")) Console.Error.WriteLine($"  {r.Verdict} {r.Source} {r.Scene} t{r.Threads} {r.Ours}");
        return fail == 0 ? 0 : 1;
    }

    static void RunItem(string id, string itemDir, string sceneDir, List<Row> rows)
    {
        var mp = Path.Combine(itemDir, "metrics.csv");
        string scene = id.StartsWith("run-", StringComparison.Ordinal) ? id[4..] : id;
        int rc = ReadRc(itemDir + ".rc");
        if (!File.Exists(mp)) { rows.Add(new Row("run", scene, DimOf(scene, sceneDir), "1", "(scene)", "", $"no metrics.csv (rc {rc}); see items/{id}.log", "FAIL")); return; }
        var byScene = File.ReadAllLines(mp).Skip(1).Select(l => l.Split(',')).Where(c => c.Length >= 5 && c[1] == "onephysics").GroupBy(c => c[0]);
        foreach (var g in byScene)
        {
            string dim = DimOf(g.Key, sceneDir);
            bool all = true;
            string threads = g.First()[2];
            var metrics = g.ToDictionary(c => c[3], c => double.Parse(c[4], NumberStyles.Float, CultureInfo.InvariantCulture), StringComparer.Ordinal);
            var judged = new HashSet<string>(StringComparer.Ordinal);
            void Judge(string metric, double max, bool isMin)
            {
                if (!metrics.TryGetValue(metric, out var v)) { rows.Add(new Row("run", g.Key, dim, threads, metric, (isMin ? ">= " : "<= ") + F(max), "(not reported)", "FAIL")); all = false; return; }
                judged.Add(metric);
                bool ok = !double.IsNaN(v) && (isMin ? v >= max : v <= max);
                if (!ok) all = false;
                rows.Add(new Row("run", g.Key, dim, threads, metric, (isMin ? ">= " : "<= ") + F(max), F(v), ok ? "PASS" : "FAIL"));
            }
            foreach (var (m, mx) in Verdict.Limits.TryGetValue(g.Key, out var lim) ? lim : ExtraMax.TryGetValue(g.Key, out var ex) ? ex : []) Judge(m, mx, false);
            foreach (var (m, mn) in ExtraMin.TryGetValue(g.Key, out var em) ? em : []) Judge(m, mn, true);
            foreach (var z in AlwaysZero) if (metrics.ContainsKey(z)) Judge(z, 0, false);
            bool hasLimits = Verdict.Limits.ContainsKey(g.Key) || ExtraMax.ContainsKey(g.Key);
            if (!hasLimits) { rows.Add(new Row("run", g.Key, dim, threads, "(scene)", "", "no threshold defined for this scene", "NOTHRESHOLD")); }
            foreach (var kv in metrics.OrderBy(k => k.Key, StringComparer.Ordinal).Where(k => !judged.Contains(k.Key)))
                rows.Add(new Row("run", g.Key, dim, threads, kv.Key, "", F(kv.Value), "INFO"));
            if (hasLimits)
            {
                string detail = all ? "" : string.Join("; ", rows.Where(r => r.Source == "run" && r.Scene == g.Key && r.Verdict == "FAIL").Select(r => $"{r.Metric} {r.Ours} vs {r.Threshold}"));
                rows.Add(new Row("run", g.Key, dim, threads, "(scene)", "", detail, all ? "PASS" : "FAIL"));
            }
        }
        if (rc != 0 && !byScene.Any()) rows.Add(new Row("run", scene, DimOf(scene, sceneDir), "1", "(scene)", "", $"rc {rc}", "FAIL"));
    }

    static int ReadRc(string f) => File.Exists(f) && int.TryParse(File.ReadAllText(f).Trim(), out var v) ? v : -1;

    static readonly string[] VerdictWords = ["PASS", "FAIL", "STABLE", "UNSTABLE"];

    /// <summary>A stability command's log: one line per scene, "NAME key=value ... PASS|FAIL", or the rod and pulley form ("scene=NAME ..." then "metrics ... stable=True|False").</summary>
    static void CommandItem(string id, string log, string sceneDir, List<Row> rows)
    {
        string source = id;
        int rc = ReadRc(Path.ChangeExtension(log, ".rc"));
        string threads = "1";
        var tm = System.Text.RegularExpressions.Regex.Match(id, @"-t(\d+)$");
        if (tm.Success) threads = tm.Groups[1].Value;
        string current = "";
        int judged = 0;
        foreach (var line in File.ReadAllLines(log))
        {
            var toks = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (toks.Length == 0) continue;
            string scene; int start;
            if (toks[0].StartsWith("scene=", StringComparison.Ordinal)) { current = toks[0][6..]; continue; }
            if (toks[0] == "metrics" || toks[0] == "indicative") { if (toks[0] == "indicative") continue; scene = current; start = 1; }
            else if (toks[0].Contains('=') || toks[0].Contains(':')) continue;
            else { scene = toks[0]; start = 1; }
            if (scene == "" || !(toks.Skip(start).Any(t => t.Contains('=')))) continue;
            string verdict = "";
            var kv = new List<(string k, string v)>();
            for (int i = start; i < toks.Length; i++)
            {
                var t = toks[i];
                if (VerdictWords.Contains(t, StringComparer.Ordinal)) { verdict = t is "PASS" or "STABLE" ? "PASS" : "FAIL"; continue; }
                int eq = t.IndexOf('=');
                if (eq <= 0) continue;
                string k = t[..eq], v = t[(eq + 1)..];
                if (k == "stable") { verdict = v == "True" ? "PASS" : "FAIL"; continue; }
                kv.Add((k, v));
            }
            if (verdict == "") continue; // a progress or detail line, not a scene's result
            string dim = DimOf(scene, sceneDir);
            string th = kv.FirstOrDefault(x => x.k == "threads").v ?? threads; // the command's own report wins over the item id
            foreach (var (k, v) in kv) rows.Add(new Row(source, scene, dim, th, k, "(in command)", v, "INFO"));
            rows.Add(new Row(source, scene, dim, th, "(scene)", "(in command)", verdict == "PASS" ? "" : string.Join(' ', kv.Select(x => x.k + "=" + x.v)), verdict));
            judged++;
        }
        if (judged == 0 || rc != 0 && !rows.Any(r => r.Source == source && r.Verdict == "FAIL"))
            rows.Add(new Row(source, id, "?", threads, "(scene)", "", judged == 0 ? $"no scene result in log (rc {rc})" : $"rc {rc} with no FAIL line (crash after output?)", "FAIL"));
    }
}
