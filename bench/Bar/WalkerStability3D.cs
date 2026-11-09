#pragma warning disable OPHYS0003
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// Stability rows for walkers-3d: sixteen walkers per run on one terrain (flat, up, down: straight 50 m at 3 degrees; path: a 1.5 m wide path with a 90 degree turn after 25 m), two seeds per terrain (32 trials).
/// A trial reaches the goal when its path progress (metres of path, a projection of the pelvis on the centre line) reaches 50 m; it falls when the pelvis is lower than 0.6 m above the ground, and on the path it also fails when
/// the pelvis is more than 0.75 m from the centre line (off the path). speedError is the largest |mean path speed from 10 m to the goal - command| over the trials that reached it; the others are as for the other active scenes.
/// </summary>
internal static class WalkerStability3D
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
    const int Steps = 6000;

    internal static int Run(Opts opts)
    {
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        string only = opts.Get("terrain", "");
        int steps = int.Parse(opts.Get("steps", Steps.ToString(CultureInfo.InvariantCulture)), CultureInfo.InvariantCulture);
        bool verbose = opts.Has("verbose");
        if (opts.Has("push")) { var pr = opts.Get("push", "0,0").Split(','); pushX = double.Parse(pr[0], CultureInfo.InvariantCulture); pushZ = double.Parse(pr[1], CultureInfo.InvariantCulture); pushStep = int.Parse(opts.Get("pushstep", "900"), CultureInfo.InvariantCulture); } trace = int.Parse(opts.Get("trace", "0"), CultureInfo.InvariantCulture);
        int fails = 0;
        foreach (var t in new[] { Walkers3DScene.Terrain.Flat, Walkers3DScene.Terrain.Up, Walkers3DScene.Terrain.Down, Walkers3DScene.Terrain.Path })
        {
            if (only.Length != 0 && !string.Equals(only, t.ToString(), StringComparison.OrdinalIgnoreCase)) continue;
            double prog = 0; int reached = 0, trials = 0; double maxSpeed = 0, nan = 0, escapes = 0, maxPen = 0, speedErr = 0, jointErr = 0, offMax = 0; int falls = 0, off = 0, stopFalls = 0;
            foreach (int seed in new[] { 0, 1 })
            {
                var r = Trial(t, seed, threads, steps, verbose);
                prog += r.Prog / 2; stopFalls += r.StopFalls; reached += r.Reached; trials += Walkers3DScene.Count; falls += r.Falls; off += r.Off;
                maxSpeed = Math.Max(maxSpeed, r.MaxSpeed); nan += r.Nan; escapes += r.Escapes; maxPen = Math.Max(maxPen, r.MaxPen); speedErr = Math.Max(speedErr, r.SpeedErr); jointErr = Math.Max(jointErr, r.JointErr); offMax = Math.Max(offMax, r.OffMax);
            }

            var res = new SortedDictionary<string, double> { ["meanProgress"] = prog, ["reached"] = reached, ["trials"] = trials, ["falls"] = falls, ["stopFalls"] = stopFalls, ["offPath"] = off, ["speedError"] = speedErr, ["maxSpeed"] = maxSpeed, ["nan"] = nan, ["escapes"] = escapes, ["maxPenetration"] = maxPen, ["jointError"] = jointErr, ["maxOffset"] = offMax };
            bool ok = reached >= 30 && nan == 0 && escapes == 0 && speedErr <= 0.2;
            Console.WriteLine($"walkers-3d/{t} steps={steps} threads={threads} " + string.Join(' ', res.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));
            if (!ok) fails++;
        }

        return fails;
    }

    internal readonly record struct Result(double Prog, int StopFalls, int Reached, int Falls, int Off, double MaxSpeed, double Nan, double Escapes, double MaxPen, double SpeedErr, double JointErr, double OffMax);

    static int trace; static double pushX, pushZ; static int pushStep = 900;
    static Result Trial(Walkers3DScene.Terrain terrain, int seed, int threads, int steps, bool verbose)
    {
        int n = Walkers3DScene.Count;
        var sc = Walkers3DScene.Create(terrain, seed, threads, 320 + seed);
        var w = sc.World;
        var def = P3.RagdollDef.Humanoid;
        double maxSpeed = 0, nan = 0, escapes = 0, maxPen = 0, offMax = 0;
        var fell = new int[n]; var fellAfter = new int[n]; var off = new bool[n]; var t10 = new int[n]; var tGoal = new int[n];
        for (int step = 0; step < steps; step++)
        {
            Walkers3DScene.Command(sc, step);
            if ((pushX != 0 || pushZ != 0) && step >= pushStep && step < pushStep + 2 * n && ((step - pushStep) & 1) == 0)
            {
                int k = (step - pushStep) / 2;   // one walker every second step: the pushes land at different phases of the gait
                double sg = ((k + seed) & 1) == 0 ? 1.0 : -1.0;
                sc.Rags[k].GetBody(0).ApplyImpulse(new Vector3F((float)(pushX * sg), 0f, (float)(pushZ * sg)));
            }

            w.Step(Walkers3DScene.Dt);
            if (trace > 0 && step % trace == 0 && seed == 0)
            {
                int tw = 0; var r0 = sc.Rags[tw]; var inf = r0.GetActiveInfo(); var pp = r0.GetBody(0).Position; var lf = r0.GetBody(9).Position; var rf = r0.GetBody(12).Position;
                var q = r0.GetBody(0).Rotation; var up = q.Rotate(new Vector3F(0, 1, 0));
                Console.WriteLine($"t={step} ph={inf.Phase} st={inf.StepCount} pel=({pp.X - Walkers3DScene.LaneX(tw):0.000},{pp.Y:0.000},{pp.Z:0.000}) com=({inf.CenterOfMassX - Walkers3DScene.LaneX(tw):0.000},{inf.CenterOfMassZ:0.000}) cap=({inf.CapturePointX - Walkers3DScene.LaneX(tw):0.000},{inf.CapturePointZ:0.000}) Lf=({lf.X - Walkers3DScene.LaneX(tw):0.000},{lf.Y:0.000},{lf.Z:0.000}) Rf=({rf.X - Walkers3DScene.LaneX(tw):0.000},{rf.Y:0.000},{rf.Z:0.000}) tilt=({up.X:0.000},{up.Z:0.000}) yaw={Math.Atan2(q.Rotate(new Vector3F(0, 0, 1)).X, q.Rotate(new Vector3F(0, 0, 1)).Z):0.00} g={inf.GroundedFeet}");
            }

            for (int k = 0; k < n; k++)
            {
                var r = sc.Rags[k];
                var pel = r.GetBody(0).Position;
                double ground = terrain == Walkers3DScene.Terrain.Path ? 0 : Walkers3DScene.GroundHeight(terrain, pel.Z);
                double s, offset;
                if (terrain == Walkers3DScene.Terrain.Path) (s, offset) = Walkers3DScene.Project(k, pel.X, pel.Z); else (s, offset) = (pel.Z, Math.Abs(pel.X - Walkers3DScene.LaneX(k)));
                if (pel.Y - ground < 0.6 && fell[k] == 0 && fellAfter[k] == 0 && tGoal[k] != 0) { fellAfter[k] = step + 1; if (verbose) Console.WriteLine($"  seed {seed} walker {k} fell after the goal at step {step}"); }
                if (pel.Y - ground < 0.6 && fell[k] == 0 && tGoal[k] == 0) { fell[k] = step + 1; if (verbose) Console.WriteLine($"  seed {seed} walker {k} fell at step {step} s={F(s)} off={F(offset)}"); }
                if (terrain == Walkers3DScene.Terrain.Path && offset > 0.5 * Walkers3DScene.PathWidth && !off[k] && fell[k] == 0 && tGoal[k] == 0) { off[k] = true; if (verbose) Console.WriteLine($"  seed {seed} walker {k} left the path at step {step} s={F(s)}"); }
                if (fell[k] == 0 && tGoal[k] == 0) offMax = Math.Max(offMax, offset);
                if (t10[k] == 0 && s >= 10) t10[k] = step + 1;
                if (tGoal[k] == 0 && s >= Walkers3DScene.Goal - 0.05) tGoal[k] = step + 1;
                if ((step & 7) != 0) continue;
                for (int part = 0; part < r.PartCount; part++)
                {
                    var b = r.GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                    if (p.Y < ground - 1 || p.Y > ground + 20 || Math.Abs(p.Z) > 120 || p.X > 80 || p.X < -Walkers3DScene.LaneSpacing * n - 80) escapes++;
                    foreach (var c in w.GetContacts(b)) for (int i = 0; i < c.PointCount; i++) maxPen = Math.Max(maxPen, -(double)c.Separation(i));
                }
            }

            if (verbose && step % 600 == 599)
            {
                var sb = new System.Text.StringBuilder($"  seed {seed} t={step / 60}s s:");
                for (int k = 0; k < n; k++) { var pel = sc.Rags[k].GetBody(0).Position; var (s, _) = terrain == Walkers3DScene.Terrain.Path ? Walkers3DScene.Project(k, pel.X, pel.Z) : (pel.Z, 0.0); sb.Append(fell[k] != 0 ? " F" : " " + s.ToString("0.0", CultureInfo.InvariantCulture)); }
                Console.WriteLine(sb.ToString());
            }
        }

        double jointErr = 0;
        for (int k = 0; k < n; k++)
            for (int part = 1; part < def.Parts.Length; part++)
            {
                var pd = def.Parts[part]; var par = def.Parts[pd.Parent];
                var a = Anchor(sc.Rags[k].GetBody(part), pd, pd.JointPosition); var b = Anchor(sc.Rags[k].GetBody(pd.Parent), par, pd.JointPosition);
                jointErr = Math.Max(jointErr, Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z)));
            }

        int reached = 0, falls = 0, offCount = 0; double speedErr = 0, progSum = 0;
        for (int k = 0; k < n; k++) { var pp = sc.Rags[k].GetBody(0).Position; progSum += terrain == Walkers3DScene.Terrain.Path ? Walkers3DScene.Project(k, pp.X, pp.Z).S : pp.Z; }
        for (int k = 0; k < n; k++)
        {
            if (fell[k] != 0) falls++;
            if (off[k]) offCount++;
            if (tGoal[k] != 0 && fell[k] == 0 && !off[k])
            {
                reached++;
                double speed = (Walkers3DScene.Goal - 0.05 - 10.0) / ((tGoal[k] - t10[k]) * (double)Walkers3DScene.Dt);
                speedErr = Math.Max(speedErr, Math.Abs(speed - sc.Speed[k]));
                if (verbose) Console.WriteLine($"  seed {seed} walker {k} cmd {sc.Speed[k]:0.00} measured {speed:0.000}");
            }
        }

        int stopFalls = 0; for (int k = 0; k < n; k++) if (fellAfter[k] != 0) stopFalls++;
        return new Result(progSum / n, stopFalls, reached, falls, offCount, maxSpeed, nan, escapes, maxPen, speedErr, jointErr, offMax);
    }

    static (double X, double Y, double Z) Anchor(P3.Body body, P3.RagdollPartDef d, Vector3F joint)
    {
        var q = body.Rotation; var p = body.Position;
        var off = q.Rotate(d.Rotation.Conjugated().Rotate(joint - d.Position));
        return (p.X + off.X, p.Y + off.Y, p.Z + off.Z);
    }
}
