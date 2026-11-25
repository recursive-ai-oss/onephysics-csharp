using System.Globalization;
using System.Text;
using RecursiveAI.OnePhysics.Diagnosis;

namespace Bar;

using Scene = RecursiveAI.OnePhysics.Scene.Scene;

/// <summary>
/// Runs the world diagnosis over the generated scenes: once as built and once after a number of settling steps, printing one line per scene and each finding. Used to check that no standard scene
/// produces an error-severity finding and to see which warnings the thresholds give on real content. Scenes the plain builders do not cover (vehicles, soft bodies, machines) are skipped by name.
/// </summary>
static class DiagnoseRuns
{
    public static int Run(Opts o)
    {
        string dir = o.Get("scenes", "scenes/generated");
        int settle = int.Parse(o.Get("settle", "120"), CultureInfo.InvariantCulture);
        int probe = int.Parse(o.Get("probe", "90"), CultureInfo.InvariantCulture);
        string only = o.Get("only", "");
        int errors = 0;
        var report = new StringBuilder();
        foreach (var path in Directory.GetFiles(dir, "*.scene").OrderBy(p => p, StringComparer.Ordinal))
        {
            string name = Path.GetFileNameWithoutExtension(path);
            if (only != "" && !only.Split(',').Contains(name)) { continue; }
            var s = Scene.Load(path);
            for (int pass = 0; pass < 2; pass++)
            {
                try
                {
                    DiagnosisReport r;
                    var opt = new DiagnoseOptions { ProbeSteps = probe, Dt = (float)s.Dt };
                    if (s.Dim == 3)
                    {
                        var (w, _, _) = SceneBuilder3D.Build(s, 1);
                        for (int i = 0; i < (pass == 0 ? 0 : settle); i++) { w.Step((float)s.Dt); }
                        var t0 = System.Diagnostics.Stopwatch.GetTimestamp();
                        r = w.Diagnose(opt);
                        Line(report, name, pass, r, System.Diagnostics.Stopwatch.GetElapsedTime(t0).TotalMilliseconds);
                    }
                    else
                    {
                        var (w, _, _) = SceneBuilder2D.Build(s, 1, out _);
                        for (int i = 0; i < (pass == 0 ? 0 : settle); i++) { w.Step((float)s.Dt); }
                        var t0 = System.Diagnostics.Stopwatch.GetTimestamp();
                        r = w.Diagnose(opt);
                        Line(report, name, pass, r, System.Diagnostics.Stopwatch.GetElapsedTime(t0).TotalMilliseconds);
                    }

                    if (r.HasErrors) { errors++; }
                    foreach (var g in r.Findings.GroupBy(f => f.Id))
                    {
                        var list = g.ToList();
                        report.Append("    ").Append(g.Key).Append(" x").Append(list.Count).Append(" worst ").Append(list.Max(f => f.Severity)).Append('\n');
                        foreach (var f in list.Take(2)) { report.Append("        bodies ").Append(string.Join(",", f.Bodies.Select(b => b.Index))).Append(" joints ").Append(string.Join(",", f.Joints.Select(j => j.Index))).Append(" [").Append(string.Join("; ", f.Values)).Append("]\n"); }
                    }

                }
                catch (Exception e) when (e is not OutOfMemoryException)
                {
                    report.Append(name).Append(pass == 0 ? " (built)" : " (settled)").Append(": skipped, ").Append(e.GetType().Name).Append(": ").Append(e.Message.Length > 100 ? e.Message[..100] : e.Message).Append('\n');
                }
            }
        }

        Console.Write(report);
        Console.WriteLine("scenes with error findings: " + errors);
        return errors == 0 ? 0 : 1;
    }

    static void Line(StringBuilder sb, string name, int pass, DiagnosisReport r, double ms)
    {
        sb.Append(name).Append(pass == 0 ? " (built)" : " (settled)").Append(": ").Append(r.Findings.Count).Append(" finding(s), worst ").Append(r.Worst?.ToString() ?? "none").Append(", ").Append(ms.ToString("F1", CultureInfo.InvariantCulture)).Append(" ms (indicative)\n");
    }
}
