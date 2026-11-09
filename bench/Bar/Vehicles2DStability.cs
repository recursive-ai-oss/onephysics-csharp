#pragma warning disable OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

namespace Bar;

/// <summary>
/// Stability and timing rows for the side-view vehicle and ragdoll scenes (no reference engine has either). Metrics, over a fixed 600 steps:
///  maxSpeed (m/s, any body), nan and escapes (bodies outside the scene bounds), rollovers (a car whose chassis passes 1.8 rad from upright), fallen (a motorcycle past 1.2 rad), wheelThroughGround (steps with a wheel
///  centre under half a radius above the terrain), suspensionJitter (vehicles at rest in the last frame: the largest change of a wheel's suspension length from the frame before, m), minProgress (the shortest distance any
///  vehicle drove, m; 0 means a stuck vehicle), maxPenetration (m, deepest contact of any body), jointError (ragdolls: the largest gap between the two anchors of a joint, m), limitViolation (largest joint-limit overshoot, rad)
///  and restKE (the 90th-percentile per-body kinetic energy over the last 3 frames, J).
/// </summary>
internal static class Vehicles2DStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    static float Wrap(float a) { const float Pi = 3.1415927f; while (a > Pi) a -= 2 * Pi; while (a < -Pi) a += 2 * Pi; return a; }

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        bool timing = opts.Get("timing", "") != "";
        int fails = 0;
        foreach (var name in Vehicles2DScene.Names)
        {
            if (only.Length != 0 && only != name) continue;
            fails += RunOne(name, threads, timing);
        }

        return fails;
    }

    // World position of a joint's anchor as seen from one of its two parts: the part's body pose applied to the anchor's offset in the part's bind frame.
    static (double X, double Y) Anchor(Body body, RagdollPartDef d, Vector2F joint)
    {
        float a = -d.Angle, c = MathF.Cos(a), s = MathF.Sin(a);
        double lx = joint.X - d.Position.X, ly = joint.Y - d.Position.Y;
        double ox = c * lx - s * ly, oy = s * lx + c * ly;
        float ba = body.Angle, bc = MathF.Cos(ba), bs = MathF.Sin(ba);
        var p = body.Position;
        return (p.X + bc * ox - bs * oy, p.Y + bs * ox + bc * oy);
    }

    static int RunOne(string name, int threads, bool timing)
    {
        var sc = Vehicles2DScene.Build(name, threads);
        var w = sc.World;
        var r = new SortedDictionary<string, double>();
        double maxSpeed = 0, nan = 0, escapes = 0, through = 0, maxPen = 0, jointErr = 0, limitViol = 0;
        var rolled = new HashSet<int>(); var fell = new HashSet<int>();
        var lastLen = new float[sc.Vehicles.Length][];
        var curLen = new float[sc.Vehicles.Length][];
        var times = new List<double>();
        var sw = new Stopwatch();
        var ke = new List<double>();
        bool pile = name == "ragdoll-pile-2d";
        float rolloverLimit = name == "motorcycle-2d" ? 1.2f : 1.8f;
        double xmax = pile ? 7.0 : sc.Terrain.Length + 20;
        for (int step = 0; step < sc.Steps; step++)
        {
            sc.Script(step);
            sw.Restart();
            w.Step(Vehicles2DScene.Dt);
            if (timing) times.Add(sw.Elapsed.TotalMilliseconds);
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) continue;
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type != BodyType.Dynamic) continue;
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                double sp = Math.Sqrt(v.X * v.X + v.Y * v.Y);
                maxSpeed = Math.Max(maxSpeed, sp);
                if (p.Y < -5 || p.Y > 200 || p.X < (pile ? -xmax : -20) || p.X > xmax) escapes++;
                if (step >= sc.Steps - 3) ke.Add(0.5 * body.Mass * sp * sp + 0.5 * body.Inertia * body.AngularVelocity * body.AngularVelocity);
                foreach (var c in w.GetContacts(body))
                    for (int i = 0; i < c.PointCount; i++) maxPen = Math.Max(maxPen, -(double)c.Separation(i));
            }

            for (int k = 0; k < sc.Vehicles.Length; k++)
            {
                var v = sc.Vehicles[k];
                if (Math.Abs(Wrap(v.Chassis.Angle)) > rolloverLimit) { if (name == "motorcycle-2d") fell.Add(k); else rolled.Add(k); }
                curLen[k] = new float[v.WheelCount];
                for (int i = 0; i < v.WheelCount; i++)
                {
                    var ws = v.GetWheel(i);
                    curLen[k][i] = ws.SuspensionLength;
                    double ground = Vehicles2DScene.HeightAt(sc.Terrain, sc.TerrainX0, ws.Center.X);
                    if (ws.Center.Y < ground + 0.5 * ws.Radius) through++;
                }
            }

            if (Environment.GetEnvironmentVariable("VEH2D_TRACE") is { Length: > 0 } tv && step % 5 == 0 && step >= 420 && step < 520 && int.Parse(tv, CultureInfo.InvariantCulture) < sc.Vehicles.Length)
            {
                var tvh = sc.Vehicles[int.Parse(tv, CultureInfo.InvariantCulture)];
                Console.WriteLine($"  t{step} ang {tvh.Chassis.Angle:F2} y {tvh.Chassis.Position.Y:F2} x {tvh.Chassis.Position.X - sc.StretchStart[int.Parse(tv, CultureInfo.InvariantCulture)]:F2} vx {tvh.Chassis.LinearVelocity.X:F2} gear {tvh.Gear} rpm {tvh.EngineRpm:F0} w0 {tvh.GetWheel(0).AngularVelocity:F1} slip {tvh.GetWheel(1).SlipRatio:F2} fx {tvh.GetWheel(1).LongitudinalForce:F0} contact {tvh.GetWheel(1).IsInContact}");
            }

            if (step == sc.Steps - 2) for (int k = 0; k < sc.Vehicles.Length; k++) lastLen[k] = curLen[k];
            if (step == sc.Steps - 1 && sc.Vehicles.Length != 0)
            {
                double worst = 0;
                for (int k = 0; k < sc.Vehicles.Length; k++)
                {
                    var vel = sc.Vehicles[k].Chassis.LinearVelocity;
                    if (Math.Sqrt(vel.X * vel.X + vel.Y * vel.Y) > 0.2) continue;
                    for (int i = 0; i < curLen[k].Length; i++) worst = Math.Max(worst, Math.Abs(curLen[k][i] - lastLen[k][i]));
                }
                r["suspensionJitter"] = worst;
            }

            if (sc.Ragdolls.Length != 0 && step % 10 == 9)
            {
                var def = sc.RagdollDefinition!;
                foreach (var rd in sc.Ragdolls)
                    for (int part = 1; part < def.Parts.Length; part++)
                    {
                        if (!rd.IsAttached(part)) continue;
                        var pd = def.Parts[part]; var par = def.Parts[pd.Parent];
                        var bc = rd.GetBody(part); var bp = rd.GetBody(pd.Parent);
                        var (cx, cy) = Anchor(bc, pd, pd.JointPosition);
                        var (px, py) = Anchor(bp, par, pd.JointPosition);
                        jointErr = Math.Max(jointErr, Math.Sqrt((cx - px) * (cx - px) + (cy - py) * (cy - py)));
                        float ja = Wrap(bc.Angle - bp.Angle - (pd.Angle - par.Angle));
                        limitViol = Math.Max(limitViol, Math.Max(pd.LowerAngle - ja, ja - pd.UpperAngle));
                    }
            }
        }

        double minProgress = double.MaxValue;
        for (int k = 0; k < sc.Vehicles.Length; k++) minProgress = Math.Min(minProgress, sc.Vehicles[k].Chassis.Position.X - sc.StretchStart[k]);
        if (Environment.GetEnvironmentVariable("VEH2D_DETAIL") == "1")
            for (int k = 0; k < sc.Vehicles.Length; k++) Console.WriteLine($"  vehicle {k}: progress {sc.Vehicles[k].Chassis.Position.X - sc.StretchStart[k]:F2} y {sc.Vehicles[k].Chassis.Position.Y:F2} angle {sc.Vehicles[k].Chassis.Angle:F2} gear {sc.Vehicles[k].Gear} rpm {sc.Vehicles[k].EngineRpm:F0}");
        r["maxSpeed"] = maxSpeed; r["nan"] = nan; r["escapes"] = escapes; r["maxPenetration"] = maxPen;
        if (sc.Vehicles.Length != 0)
        {
            r["wheelThroughGround"] = through; r["minProgress"] = minProgress;
            if (name == "motorcycle-2d") r["fallen"] = fell.Count; else r["rollovers"] = rolled.Count;
        }
        else
        {
            ke.Sort();
            r["restKE"] = ke.Count == 0 ? 0 : ke[(int)(0.9 * (ke.Count - 1))];
            r["jointError"] = jointErr; r["limitViolation"] = limitViol;
        }

        bool ok = r["nan"] == 0 && r["escapes"] == 0;
        switch (name)
        {
            case "vehicles-2d": ok &= r["maxSpeed"] <= 30 && r["maxPenetration"] <= 0.2 && r["rollovers"] == 0 && r["wheelThroughGround"] == 0 && r["suspensionJitter"] <= 0.01 && r["minProgress"] > 30; break;
            case "tracked-2d": ok &= r["maxSpeed"] <= 15 && r["maxPenetration"] <= 0.01 && r["rollovers"] == 0 && r["wheelThroughGround"] == 0 && r["suspensionJitter"] <= 0.01 && r["minProgress"] > 20; break;
            case "motorcycle-2d": ok &= r["maxSpeed"] <= 30 && r["maxPenetration"] <= 0.01 && r["fallen"] == 0 && r["wheelThroughGround"] == 0 && r["suspensionJitter"] <= 0.01 && r["minProgress"] > 30; break;
            case "ragdoll-pile-2d": ok &= r["maxSpeed"] <= 50 && r["maxPenetration"] <= 0.5 && r["jointError"] <= 0.15 && r["limitViolation"] <= 1.5 && r["restKE"] <= 0.0001; break;
        }

        Console.WriteLine($"{name} steps={sc.Steps} threads={threads} " + string.Join(' ', r.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));
        if (timing)
        {
            times.Sort();
            Console.WriteLine($"{name} threads={threads} stepMs median={F(times[times.Count / 2])} p90={F(times[(int)(0.9 * (times.Count - 1))])} mean={F(times.Average())}");
        }

        return ok ? 0 : 1;
    }
}
