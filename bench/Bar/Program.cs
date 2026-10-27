using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Text;

namespace Bar;

public static class Program
{
    public static int Main(string[] args)
    {
        if (args.Length == 0) return Usage();
        var opts = Opts.Parse(args.Skip(1));
        switch (args[0])
        {
            case "gen": Gen.WriteAll(opts.Get("out", "scenes/generated")); return 0;
            case "metrics":
            {
                var s = Scene.Load(opts.Pos(0));
                foreach (var kv in Metrics.Compute(s, opts.Pos(1))) Console.WriteLine($"{kv.Key}={kv.Value.ToString("R", CultureInfo.InvariantCulture)}");
                return 0;
            }
            case "selftest": return Metrics.SelfTest() + Metrics3.SelfTest();
            case "run": return Runner.Run(opts);
            case "summarize": Runner.Summarize(opts.Pos(0)); return 0;
            case "box2dnet-driver": return Box2DNetDriver.Run(opts);
            case "bepu-driver": return BepuDriver.Run(opts);
            case "jitter2-driver": return Jitter2Driver.Run(opts);
            default: return Usage();
        }
    }

    static int Usage()
    {
        Console.Error.WriteLine("""
            usage: bar gen [--out DIR]
                   bar metrics SCENE DUMP
                   bar run --mode timing|stability [--scenes a,b|all] [--engines box2d,rapier2d,box2dnet,jolt,box3d,rapier3d,bepu,jitter2] [--threads 1,8,all]
                           [--scene-dir DIR] [--state DIR] [--out DIR] [--repeat N] [--job-id ID] [--lock-held]
                   bar summarize RESULT_DIR
            """);
        return 2;
    }
}

public sealed class Opts
{
    readonly Dictionary<string, string> named = new();
    readonly List<string> pos = [];
    public static Opts Parse(IEnumerable<string> a)
    {
        var o = new Opts(); var l = a.ToList();
        for (int i = 0; i < l.Count; i++)
        {
            if (l[i].StartsWith("--", StringComparison.Ordinal))
            {
                var k = l[i][2..];
                if (i + 1 < l.Count && !l[i + 1].StartsWith("--", StringComparison.Ordinal)) o.named[k] = l[++i]; else o.named[k] = "true";
            }
            else o.pos.Add(l[i]);
        }
        return o;
    }
    public string Get(string k, string d) => named.TryGetValue(k, out var v) ? v : d;
    public bool Has(string k) => named.ContainsKey(k);
    public string Pos(int i) => pos[i];
}

static class Runner
{
    static readonly string[] Engines3D = ["jolt", "box3d", "rapier3d", "bepu", "jitter2"];
    static readonly string[] InProcess = ["box2dnet", "bepu", "jitter2"]; // run through this program's own drivers

    sealed class Row
    {
        public string Scene = "", Engine = "", Version = "", Config = "";
        public int Threads, Bodies, Measured;
        public double Median, P90, P99, Min, Max, Mean, PeakRssMb, WallSec;
        public Dictionary<string, double> Query = new();
    }

    public static int Run(Opts o)
    {
        string mode = o.Get("mode", "stability");
        bool timing = mode == "timing";
        string sceneDir = o.Get("scene-dir", "scenes/generated");
        string state = o.Get("state", Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "onephysics"));
        string outDir = o.Get("out", Path.Combine(state, "bench", "results", DateTime.UtcNow.ToString("yyyyMMdd-HHmmss", CultureInfo.InvariantCulture)));
        int repeat = int.Parse(o.Get("repeat", "1"), CultureInfo.InvariantCulture);
        string jobId = o.Get("job-id", Environment.GetEnvironmentVariable("BAR_JOB_ID") ?? "");
        var engines = o.Get("engines", "box2d,rapier2d,box2dnet,jolt,box3d,rapier3d,bepu,jitter2").Split(',');
        var scenes = o.Get("scenes", "all") == "all"
            ? Directory.GetFiles(sceneDir, "*.scene").Select(p => Path.GetFileNameWithoutExtension(p)).OrderBy(x => x, StringComparer.Ordinal).ToArray()
            : o.Get("scenes", "").Split(',');
        int cores = Environment.ProcessorCount; // a benchmark setting, not a simulation input
        var threadList = o.Get("threads", timing ? "1,8,all" : "1").Split(',').Select(t => t == "all" ? cores : int.Parse(t, CultureInfo.InvariantCulture)).Distinct().ToArray();
        if (timing && !o.Has("lock-held") && !o.Has("no-lock-check"))
        {
            Console.Error.WriteLine("timing runs must hold the shared benchmark lock; run tools/bench/timing.sh (or pass --lock-held from inside a job that holds it)");
            return 2;
        }
        Directory.CreateDirectory(outDir);
        string wipe = Path.Combine(outDir, "raw"); Directory.CreateDirectory(wipe);

        var meta = new StringBuilder();
        meta.AppendLine($"mode={mode}\njob_id={jobId}\nmachine={Environment.MachineName}\ncpu={CpuModel()}\ncores={cores}\nos={System.Runtime.InteropServices.RuntimeInformation.OSDescription}\nrepeat={repeat}");
        meta.AppendLine($"runtime={System.Runtime.InteropServices.RuntimeInformation.FrameworkDescription}\nload_before={Load()}");
        var during = new List<string>();
        var rows = new List<Row>(); var metricRows = new List<(string scene, string engine, int threads, string key, double v)>();

        foreach (var sceneName in scenes)
        {
            var scenePath = Path.Combine(sceneDir, sceneName + ".scene");
            var scene = Scene.Load(scenePath);
            foreach (var eng in engines)
                foreach (var th in threadList)
                {
                    if (eng == "onephysics") { Console.WriteLine("onephysics: slot reserved, nothing to run yet"); continue; }
                    if (Engines3D.Contains(eng) != (scene.Dim == 3)) continue;
                    if (eng == "box2dnet" && th != 1) continue; // no documented multithreading
                    if (!timing && th != threadList[0]) continue;
                    Row? best = null; string? bestDump = null;
                    for (int rep = 0; rep < repeat; rep++)
                    {
                        string tag = $"{sceneName}.{eng}.t{th}";
                        string result = Path.Combine(wipe, tag + ".result"), dump = Path.Combine(wipe, tag + ".dump");
                        var row = RunOne(eng, scenePath, th, timing, result, dump, state, sceneName, scene.Warmup);
                        during.Add(Load().Split(' ')[0]);
                        if (row == null) { Console.Error.WriteLine($"FAILED {tag}"); continue; }
                        if (best == null || row.Median < best.Median) { best = row; bestDump = dump; }
                    }
                    if (best == null) continue;
                    rows.Add(best);
                    Console.WriteLine($"{sceneName,-14} {eng,-9} t={th,-3} median={best.Median:F3} ms p99={best.P99:F3} rss={best.PeakRssMb:F0} MB");
                    if (!timing && bestDump != null && File.Exists(bestDump))
                        foreach (var kv in Metrics.Compute(scene, bestDump)) metricRows.Add((sceneName, eng, th, kv.Key, kv.Value));
                }
        }
        meta.AppendLine($"load1_after_each_run={string.Join(',', during)}");
        meta.AppendLine($"load_after={Load()}");
        File.WriteAllText(Path.Combine(outDir, "meta.txt"), meta.ToString());
        WriteCsv(outDir, rows, metricRows);
        Summarize(outDir);
        return 0;
    }

    static Row? RunOne(string eng, string scene, int threads, bool timing, string result, string dump, string state, string sceneName, int warmup)
    {
        string exe; var args = new List<string>();
        switch (eng)
        {
            case "box2d": exe = Path.Combine(state, "bin", "box2d-driver"); break;
            case "rapier2d": exe = Path.Combine(state, "bin", "rapier-driver"); break;
            case "jolt": exe = Path.Combine(state, "bin", "jolt-driver"); break;
            case "box3d": exe = Path.Combine(state, "bin", "box3d-driver"); break;
            case "rapier3d": exe = Path.Combine(state, "bin", "rapier3d-driver"); break;
            case "box2dnet": case "bepu": case "jitter2": exe = Environment.ProcessPath!; args.Add(Assembly.GetEntryAssembly()!.Location); args.Add(eng + "-driver"); break;
            default: throw new ArgumentException("unknown engine " + eng);
        }
        args.AddRange(["--scene", scene, "--threads", threads.ToString(CultureInfo.InvariantCulture), "--mode", timing ? "timing" : "stability", "--result", result]);
        if (!timing) args.AddRange(["--dump", dump]);

        // Peak resident memory comes from the system time utility; a failure to find it leaves the column empty.
        bool mac = OperatingSystem.IsMacOS();
        string rssFile = result + ".time";
        var psi = new ProcessStartInfo { UseShellExecute = false, RedirectStandardError = true, RedirectStandardOutput = true };
        if (File.Exists("/usr/bin/time"))
        {
            psi.FileName = "/usr/bin/time";
            if (mac) psi.ArgumentList.Add("-l"); else { psi.ArgumentList.Add("-v"); psi.ArgumentList.Add("-o"); psi.ArgumentList.Add(rssFile); }
            psi.ArgumentList.Add(exe);
        }
        else psi.FileName = exe;
        foreach (var a in args) psi.ArgumentList.Add(a);
        var sw = Stopwatch.StartNew();
        using var p = Process.Start(psi)!;
        string stderr = p.StandardError.ReadToEnd(); p.StandardOutput.ReadToEnd();
        p.WaitForExit(); sw.Stop();
        if (p.ExitCode != 0 || !File.Exists(result)) { Console.Error.WriteLine(stderr.Length > 2000 ? stderr[^2000..] : stderr); return null; }

        double rssMb = 0;
        if (mac) { foreach (var l in stderr.Split('\n')) if (l.Contains("maximum resident set size", StringComparison.Ordinal)) rssMb = double.Parse(l.Trim().Split(' ')[0], CultureInfo.InvariantCulture) / 1048576.0; }
        else if (File.Exists(rssFile)) foreach (var l in File.ReadLines(rssFile)) if (l.Contains("Maximum resident set size", StringComparison.Ordinal)) rssMb = double.Parse(l.Split(':')[1].Trim(), CultureInfo.InvariantCulture) / 1024.0;

        var kv = new Dictionary<string, string>();
        foreach (var l in File.ReadLines(result)) { int i = l.IndexOf('='); if (i > 0) kv[l[..i]] = l[(i + 1)..]; }
        var ns = kv["step_ns"].Split(',').Select(x => double.Parse(x, CultureInfo.InvariantCulture) / 1e6).Skip(Math.Min(warmup, kv["step_ns"].Split(',').Length - 1)).OrderBy(x => x).ToArray();
        double Pct(double q) => ns[Math.Min(ns.Length - 1, (int)(q * ns.Length))];
        var row = new Row
        {
            Scene = sceneName, Engine = eng, Version = kv["version"], Config = kv["config"], Threads = threads, Bodies = int.Parse(kv["bodies"], CultureInfo.InvariantCulture),
            Measured = ns.Length, Median = Pct(0.5), P90 = Pct(0.9), P99 = Pct(0.99), Min = ns[0], Max = ns[^1], Mean = ns.Average(), PeakRssMb = rssMb, WallSec = sw.Elapsed.TotalSeconds,
        };
        foreach (var k in new[] { "ray", "overlap", "cast" })
            if (kv.TryGetValue($"query_{k}_ns", out var qn))
            {
                double t = double.Parse(qn, CultureInfo.InvariantCulture) / 1e9, c = double.Parse(kv[$"query_{k}_count"], CultureInfo.InvariantCulture);
                row.Query[$"{k}_qps"] = t > 0 ? c / t : 0; row.Query[$"{k}_hits"] = double.Parse(kv[$"query_{k}_hits"], CultureInfo.InvariantCulture);
            }
        return row;
    }

    static void WriteCsv(string dir, List<Row> rows, List<(string scene, string engine, int threads, string key, double v)> metrics)
    {
        var sb = new StringBuilder("scene,engine,version,threads,bodies,steps_measured,median_ms,p90_ms,p99_ms,min_ms,max_ms,mean_ms,peak_rss_mb,wall_s,ray_qps,overlap_qps,cast_qps,ray_hits,overlap_hits,cast_hits,config\n");
        string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
        foreach (var r in rows)
        {
            double Q(string k) => r.Query.TryGetValue(k, out var v) ? v : double.NaN;
            sb.Append($"{r.Scene},{r.Engine},{r.Version},{r.Threads},{r.Bodies},{r.Measured},{F(r.Median)},{F(r.P90)},{F(r.P99)},{F(r.Min)},{F(r.Max)},{F(r.Mean)},{F(r.PeakRssMb)},{F(r.WallSec)},");
            sb.Append($"{(r.Query.Count > 0 ? F(Q("ray_qps")) : "")},{(r.Query.Count > 0 ? F(Q("overlap_qps")) : "")},{(r.Query.Count > 0 ? F(Q("cast_qps")) : "")},");
            sb.Append($"{(r.Query.Count > 0 ? F(Q("ray_hits")) : "")},{(r.Query.Count > 0 ? F(Q("overlap_hits")) : "")},{(r.Query.Count > 0 ? F(Q("cast_hits")) : "")},\"{r.Config}\"\n");
        }
        File.WriteAllText(Path.Combine(dir, "timing.csv"), sb.ToString());
        if (metrics.Count > 0)
            File.WriteAllLines(Path.Combine(dir, "metrics.csv"), new[] { "scene,engine,threads,metric,value" }.Concat(metrics.Select(m => $"{m.scene},{m.engine},{m.threads},{m.key},{F(m.v)}")));
    }

    public static void Summarize(string dir)
    {
        var sb = new StringBuilder();
        var meta = Path.Combine(dir, "meta.txt");
        if (File.Exists(meta)) { sb.AppendLine("```"); sb.Append(File.ReadAllText(meta)); sb.AppendLine("```\n"); }
        var timing = Path.Combine(dir, "timing.csv");
        if (File.Exists(timing))
        {
            var lines = File.ReadAllLines(timing).Skip(1).Select(CsvSplit).ToList();
            sb.AppendLine("## Step time (ms, warm-up excluded)\n\n| scene | engine | version | threads | bodies | median | mean | p90 | p99 | max | peak RSS MB |\n|---|---|---|---|---|---|---|---|---|---|---|");
            foreach (var c in lines) sb.AppendLine($"| {c[0]} | {c[1]} | {c[2]} | {c[3]} | {c[4]} | {Fmt(c[6])} | {Fmt(c[11])} | {Fmt(c[7])} | {Fmt(c[8])} | {Fmt(c[10])} | {Fmt(c[12], 0)} |");
            var q = lines.Where(c => c[14] != "").ToList();
            if (q.Count > 0)
            {
                sb.AppendLine("\n## Query throughput (queries/s)\n\n| scene | engine | threads | ray | overlap | shape cast | hits ray/overlap/cast |\n|---|---|---|---|---|---|---|");
                foreach (var c in q) sb.AppendLine($"| {c[0]} | {c[1]} | {c[3]} | {Fmt(c[14], 0)} | {Fmt(c[15], 0)} | {Fmt(c[16], 0)} | {c[17]}/{c[18]}/{c[19]} |");
            }
        }
        var mp = Path.Combine(dir, "metrics.csv");
        if (File.Exists(mp))
        {
            var rows = File.ReadAllLines(mp).Skip(1).Select(l => l.Split(',')).ToList();
            sb.AppendLine("\n## Stability metrics\n");
            foreach (var g in rows.GroupBy(r => r[0]).OrderBy(g => g.Key, StringComparer.Ordinal))
            {
                var engines = g.Select(r => r[1]).Distinct().OrderBy(x => x, StringComparer.Ordinal).ToList();
                sb.AppendLine($"### {g.Key}\n\n| metric | {string.Join(" | ", engines)} |\n|---|{string.Concat(engines.Select(_ => "---|"))}");
                foreach (var m in g.Select(r => r[3]).Distinct().OrderBy(x => x, StringComparer.Ordinal))
                    sb.AppendLine($"| {m} | {string.Join(" | ", engines.Select(e => g.Where(r => r[1] == e && r[3] == m).Select(r => Fmt(r[4], 4)).FirstOrDefault() ?? "-"))} |");
                sb.AppendLine();
            }
        }
        File.WriteAllText(Path.Combine(dir, "summary.md"), sb.ToString());
    }

    static string Fmt(string s, int d = 3) => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v.ToString("F" + d, CultureInfo.InvariantCulture) : s;

    static string[] CsvSplit(string line)
    {
        var r = new List<string>(); var sb = new StringBuilder(); bool q = false;
        foreach (var ch in line) { if (ch == '"') q = !q; else if (ch == ',' && !q) { r.Add(sb.ToString()); sb.Clear(); } else sb.Append(ch); }
        r.Add(sb.ToString()); return r.ToArray();
    }

    static string CpuModel()
    {
        try
        {
            if (File.Exists("/proc/cpuinfo")) return File.ReadLines("/proc/cpuinfo").First(l => l.StartsWith("model name", StringComparison.Ordinal)).Split(':')[1].Trim();
            var psi = new ProcessStartInfo("sysctl", "-n machdep.cpu.brand_string") { RedirectStandardOutput = true };
            using var p = Process.Start(psi)!; var s = p.StandardOutput.ReadToEnd().Trim(); p.WaitForExit(); return s;
        }
        catch (Exception) { return "unknown"; }
    }

    static string Load()
    {
        try
        {
            if (File.Exists("/proc/loadavg")) return string.Join(' ', File.ReadAllText("/proc/loadavg").Split(' ').Take(3));
            var psi = new ProcessStartInfo("sysctl", "-n vm.loadavg") { RedirectStandardOutput = true };
            using var p = Process.Start(psi)!; var s = p.StandardOutput.ReadToEnd().Trim(); p.WaitForExit(); return s.Trim('{', '}', ' ');
        }
        catch (Exception) { return "unknown"; }
    }
}
