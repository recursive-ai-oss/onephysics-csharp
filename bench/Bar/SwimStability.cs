using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace Bar;

/// <summary>
/// Stability rows for the swimming scenes (no reference engine has swimming characters in water coupled to buoyancy volumes, shallow water and particle fluid). Over the steps until every
/// character has reached its goal (at most 3600):
///  reached (characters that arrived, of all), maxSpeed (m/s, any character), charJitter (95th percentile of the change of a swimmer's vertical speed from one step to the next while it swims at the surface away from the log and the dock, m/s: a smooth swimmer changes it by millimetres per second per step, a bobbing one by centimetres),
///  nan and escapes (characters outside the scene or below the floor), and the final world hash.
/// </summary>
internal static class SwimStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>Own thresholds: 1.5 times the worse of the x64 and arm64 runs, rounded up (see docs/spec/scenes.md).</summary>
    internal static class Thresholds
    {
        internal const double Speed3 = 8.0, Jitter3 = 0.05, Speed2 = 9.0, Jitter2 = 0.05;
    }

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        int fails = 0;
        if (only.Length == 0 || only == "swim-3d") { fails += Run3D(threads); }
        if (only.Length == 0 || only == "swim-2d") { fails += Run2D(threads); }
        return fails;
    }

    static double Percentile(List<double> v, double q)
    {
        if (v.Count == 0) { return 0; }
        v.Sort();
        return v[(int)(q * (v.Count - 1))];
    }

    static int Run3D(int threads)
    {
        var s = SwimScenes.Build3D(threads);
        var w = s.World;
        double maxSpeed = 0, nan = 0, escapes = 0;
        var bob = new List<double>();
        var prevVy = new float[s.Characters.Length];
        int step = 0;
        for (; step < s.Steps && !SwimScenes.AllDone(s); step++)
        {
            SwimScenes.Drive3D(s, step);
            w.Step(SwimScenes.Dt);
            for (int i = 0; i < s.Characters.Length; i++)
            {
                var c = s.Characters[i]; var p = c.Position; var v = c.Velocity;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, v.Length());
                if (p.X < -8 || p.X > 75 || p.Z < -2 || p.Z > 16 || p.Y < -3 || p.Y > 12) { escapes++; }
                bool lake = i < 12, holding = c.IsSwimming && c.Mode == CharacterMode.Free && (!lake || (p.X < SwimScenes.LogX - 3.2 && p.X > 10) || (p.X > SwimScenes.LogX + 3.2 && p.X < SwimScenes.DockFace - 2.5)) && s.Swimmers[i].SwimSteps > 120 && p.Y > SwimScenes.Level - 1.0;
                if (holding && step > 300) { bob.Add(Math.Abs(v.Y - prevVy[i])); }
                prevVy[i] = v.Y;
            }
        }

        int reached = s.Swimmers.Count(m => m.Phase == 6);
        double jitter = Percentile(bob, 0.95);
        bool ok = reached == s.Swimmers.Length && nan == 0 && escapes == 0 && maxSpeed <= Thresholds.Speed3 && jitter <= Thresholds.Jitter3;
        Console.WriteLine($"swim-3d steps={step} threads={threads} reached={reached}/{s.Swimmers.Length} maxSpeed={F(maxSpeed)} charJitter={F(jitter)} nan={F(nan)} escapes={F(escapes)} particles={s.ParticleCount} hash={w.ComputeHash():x16}" + (ok ? " PASS" : " FAIL"));
        return ok ? 0 : 1;
    }

    static int Run2D(int threads)
    {
        var s = SwimScenes.Build2D(threads);
        var w = s.World;
        double maxSpeed = 0, nan = 0, escapes = 0;
        var bob = new List<double>();
        var prevVy = new float[s.Characters.Length];
        int step = 0;
        for (; step < s.Steps && !SwimScenes.AllDone(s); step++)
        {
            SwimScenes.Drive2D(s, step);
            w.Step(SwimScenes.Dt);
            for (int i = 0; i < s.Characters.Length; i++)
            {
                if (s.Swimmers[i].Phase == 6) { continue; }
                var c = s.Characters[i]; var p = c.Position; var v = c.Velocity;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, v.Length());
                if (p.X < -8 || p.X > 75 || p.Y < -3 || p.Y > 12) { escapes++; }
                bool lake = i < 8, holding = c.IsSwimming && c.Mode == CharacterMode.Free && (!lake || (p.X < SwimScenes.LogX - 3.2 && p.X > 10) || (p.X > SwimScenes.LogX + 3.2 && p.X < SwimScenes.DockFace - 2.5)) && s.Swimmers[i].SwimSteps > 120 && p.Y > SwimScenes.Level - 1.0;
                if (holding && step > 300) { bob.Add(Math.Abs(v.Y - prevVy[i])); }
                prevVy[i] = v.Y;
            }
        }

        int reached = s.Swimmers.Count(m => m.Phase == 6);
        double jitter = Percentile(bob, 0.95);
        bool ok = reached == s.Swimmers.Length && nan == 0 && escapes == 0 && maxSpeed <= Thresholds.Speed2 && jitter <= Thresholds.Jitter2;
        Console.WriteLine($"swim-2d steps={step} threads={threads} reached={reached}/{s.Swimmers.Length} maxSpeed={F(maxSpeed)} charJitter={F(jitter)} nan={F(nan)} escapes={F(escapes)} particles={s.ParticleCount} hash={w.ComputeHash():x16}" + (ok ? " PASS" : " FAIL"));
        return ok ? 0 : 1;
    }
}
