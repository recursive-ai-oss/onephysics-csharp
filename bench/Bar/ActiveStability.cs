#pragma warning disable OPHYS0003
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// Stability rows for the active-ragdoll scenes (no reference engine has an equivalent). Over a fixed number of 60 Hz steps:
///  falls (figures whose pelvis dropped below 0.6 m), comHeightError (largest |centre-of-mass height - its standing value| of a figure that did not fall, m), speedError (walkers: |mean speed over the last 20 s - command|, m/s),
///  maxSpeed (m/s, any body), nan and escapes (bodies outside the scene), maxPenetration (m, deepest contact against the ground), jointError (largest gap between the two anchors of a joint, m; 3D rows) and armError
///  (blend-3d: mean absolute difference between the animated arm angle and the simulated one at weight 0.3, rad).
/// Scenes: balance-3d (four standing humanoids pushed 12 N s forward or back every 2 s in turn, 600 steps), blend-3d (three standing humanoids with a waving arm animation at blend weight 0.3 and hit reactions on an arm, 600 steps),
/// walkers-2d (six walkers on flat ground, 100 m apart, commands 0.8 and 1.0 m/s alternating, starts staggered by 3 steps, 1200 steps). The 3D walkers scene is `walkers3d-stability` (WalkerStability3D.cs).
/// </summary>
internal static class ActiveStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
    const float Dt = 1f / 60f;
    const float Push = 12f;

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        int fails = 0;
        foreach (var name in new[] { "balance-3d", "blend-3d", "walkers-2d" })
        {
            if (only.Length != 0 && only != name) continue;
            fails += name == "walkers-2d" ? Walkers2D(threads) : Standing3D(name, threads);
        }

        return fails;
    }

    static int Report(string name, int steps, int threads, SortedDictionary<string, double> r, bool ok)
    {
        Console.WriteLine($"{name} steps={steps} threads={threads} " + string.Join(' ', r.Select(kv => kv.Key + "=" + F(kv.Value))) + (ok ? " PASS" : " FAIL"));
        return ok ? 0 : 1;
    }

    static int Standing3D(string name, int threads)
    {
        bool blend = name == "blend-3d";
        var o = new P3.WorldOptions3D { ThreadCount = threads, WorldId = 318 };
        P3.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P3.PhysicsWorld3D(o);
        w.AddBox(new Vector3F(400f, 1f, 400f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        int count = blend ? 3 : 4;
        var rags = new P3.Ragdoll[count];
        var def = P3.RagdollDef.Humanoid; def.CategoryBits = 2u; def.MaskBits = 1u;   // figures collide with the ground, not each other
        for (int k = 0; k < count; k++)
        {
            rags[k] = w.AddRagdoll(def, new Vector3D(3 * k, 0, 0));
            rags[k].EnableActive();
            rags[k].Stand();
            if (blend) for (int part = 3; part <= 6; part++) rags[k].SetBlendWeight(part, 0.3f);
        }

        int steps = 600;
        double maxSpeed = 0, nan = 0, escapes = 0, maxPen = 0, comErr = 0, armErrSum = 0; long armErrN = 0;
        var fell = new HashSet<int>();
        var pose = new QuaternionF[def.Parts.Length];
        for (int step = 0; step < steps; step++)
        {
            if (blend)
            {
                for (int i = 0; i < pose.Length; i++)
                {
                    double a = (i >= 3 && i <= 6) ? 0.5 * Math64.Sin(0.06 * step + i) : 0.0;
                    pose[i] = new QuaternionF((float)Math64.Sin(a * 0.5), 0f, 0f, (float)Math64.Cos(a * 0.5));
                }

                for (int k = 0; k < count; k++) rags[k].SetAnimationPose(pose, new Vector3D(3 * k, 0, 0), QuaternionF.Identity);
                if (step % 90 == 60) for (int k = 0; k < count; k++) rags[k].Hit(4 + (k % 2), new Vector3F(k == 1 ? -20f : 20f, 0, 10f));
            }
            else if (step >= 90 && step % 120 == 90)
            {
                int k = (step / 120) % count;
                rags[k].GetBody(1).ApplyImpulse(new Vector3F(0, 0, (step / 120) % 2 == 0 ? Push : -Push));
            }

            w.Step(Dt);
            for (int k = 0; k < count; k++)
            {
                var r = rags[k];
                var pel = r.GetBody(0).Position;
                if (pel.Y < 0.6) fell.Add(k);
                else if (step > 90) { var info = r.GetActiveInfo(); comErr = Math.Max(comErr, Math.Abs(info.CenterOfMassHeight - 0.96)); }
                for (int part = 0; part < r.PartCount; part++)
                {
                    var b = r.GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                    if (p.Y < -1 || p.Y > 20 || Math.Abs(p.X) > 30 || Math.Abs(p.Z) > 30) escapes++;
                    foreach (var c in w.GetContacts(b)) for (int i = 0; i < c.PointCount; i++) maxPen = Math.Max(maxPen, -(double)c.Separation(i));
                }

                if (blend && step > 120)
                {
                    // Arm joint angle about x: the simulated rotation of the upper arm relative to the torso against the animated one (the hit reactions lift the weight, so this is the average).
                    for (int part = 3; part <= 3; part++)
                    {
                        var qa = r.GetBody(part).Rotation; var qt = r.GetBody(1).Rotation;
                        var rel = qt.Conjugated() * qa;
                        double angle = 2.0 * Math64.Atan2(rel.X, rel.W); if (angle > Math.PI) angle -= 2 * Math.PI; if (angle < -Math.PI) angle += 2 * Math.PI;
                        double animated = 0.5 * Math64.Sin(0.06 * step + part);
                        armErrSum += Math.Abs(angle - animated); armErrN++;
                    }
                }
            }
        }

        double jointErr = 0;
        var parts = def.Parts;
        for (int k = 0; k < count; k++)
            for (int part = 1; part < parts.Length; part++)
            {
                var pd = parts[part]; var par = parts[pd.Parent];
                var a = P3Anchor(rags[k].GetBody(part), pd, pd.JointPosition); var b = P3Anchor(rags[k].GetBody(pd.Parent), par, pd.JointPosition);
                jointErr = Math.Max(jointErr, Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z)));
            }

        var res = new SortedDictionary<string, double> { ["falls"] = fell.Count, ["comHeightError"] = comErr, ["maxSpeed"] = maxSpeed, ["nan"] = nan, ["escapes"] = escapes, ["maxPenetration"] = maxPen, ["jointError"] = jointErr };
        if (blend) res["armError"] = armErrN == 0 ? 0 : armErrSum / armErrN;
        bool ok = nan == 0 && escapes == 0 && fell.Count == 0;
        if (blend) ok &= res["maxSpeed"] <= Th(name, "maxSpeed") && res["comHeightError"] <= Th(name, "com") && res["jointError"] <= Th(name, "joint") && res["maxPenetration"] <= Th(name, "pen") && res["armError"] <= Th(name, "arm");
        else ok &= res["maxSpeed"] <= Th(name, "maxSpeed") && res["comHeightError"] <= Th(name, "com") && res["jointError"] <= Th(name, "joint") && res["maxPenetration"] <= Th(name, "pen");
        return Report(name, steps, threads, res, ok);
    }

    // Thresholds: 1.5 x the worst value measured on x64 (see docs/spec/scenes.md).
    static double Th(string name, string key) => (name, key) switch
    {
        ("balance-3d", "maxSpeed") => TH_B_SPEED, ("balance-3d", "com") => TH_B_COM, ("balance-3d", "joint") => TH_B_JOINT, ("balance-3d", "pen") => TH_B_PEN,
        ("blend-3d", "maxSpeed") => TH_L_SPEED, ("blend-3d", "com") => TH_L_COM, ("blend-3d", "joint") => TH_L_JOINT, ("blend-3d", "pen") => TH_L_PEN, ("blend-3d", "arm") => TH_L_ARM,
        _ => double.MaxValue,
    };

    const double TH_B_SPEED = 1.5, TH_B_COM = 0.025, TH_B_JOINT = 0.006, TH_B_PEN = 0.01;
    const double TH_L_SPEED = 15, TH_L_COM = 0.02, TH_L_JOINT = 0.005, TH_L_PEN = 0.02, TH_L_ARM = 0.06;

    static (double X, double Y, double Z) P3Anchor(P3.Body body, P3.RagdollPartDef d, Vector3F joint)
    {
        var q = body.Rotation; var p = body.Position;
        var off = q.Rotate(d.Rotation.Conjugated().Rotate(joint - d.Position));
        return (p.X + off.X, p.Y + off.Y, p.Z + off.Z);
    }

    static int Walkers2D(int threads)
    {
        var o = new P2.WorldOptions2D { ThreadCount = threads, WorldId = 319 };
        P2.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P2.PhysicsWorld2D(o);
        w.AddBox(new Vector2F(2000f, 1f), at: new Vector2D(900, -0.5), type: BodyType.Static);
        int count = 6; int steps = 1200;
        var rags = new P2.Ragdoll[count];
        var def2 = P2.RagdollDef.Humanoid; def2.CategoryBits = 2u; def2.MaskBits = 1u;
        var cmd = new float[count];
        for (int k = 0; k < count; k++)
        {
            rags[k] = w.AddRagdoll(def2, new Vector2D(100 * k, 0));   // far apart: each walks its own 100 m of ground
            rags[k].EnableActive();
            rags[k].Stand();
            cmd[k] = k % 2 == 0 ? 1.0f : 0.8f;
        }

        double maxSpeed = 0, nan = 0, escapes = 0, maxPen = 0, comErr = 0;
        var fell = new HashSet<int>(); var x20 = new double[count];
        for (int step = 0; step < steps; step++)
        {
            for (int k = 0; k < count; k++) if (step == 61 + 3 * k) rags[k].Walk(cmd[k]);   // staggered starts, as the tuning scenes
            w.Step(Dt);
            for (int k = 0; k < count; k++)
            {
                var r = rags[k]; var pel = r.GetBody(0).Position;
                if (pel.Y < 0.6) fell.Add(k);
                if (step == steps - 20 * 60 - 1) x20[k] = pel.X - 100 * k;
                else if (step > 90 && pel.Y >= 0.6) comErr = Math.Max(comErr, Math.Abs(r.GetActiveInfo().CenterOfMassHeight - 0.94));
                for (int part = 0; part < r.PartCount; part++)
                {
                    var b = r.GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y));
                    if (p.Y < -1 || p.Y > 10 || p.X < -5 || p.X > 100 * count + 50) escapes++;
                    foreach (var c in w.GetContacts(b)) for (int i = 0; i < c.PointCount; i++) maxPen = Math.Max(maxPen, -(double)c.Separation(i));
                }
            }
        }

        double speedErr = 0;
        for (int k = 0; k < count; k++) speedErr = Math.Max(speedErr, Math.Abs((rags[k].GetBody(0).Position.X - 100 * k - x20[k]) / 20.0 - cmd[k]));
        var res = new SortedDictionary<string, double> { ["falls"] = fell.Count, ["comHeightError"] = comErr, ["speedError"] = speedErr, ["maxSpeed"] = maxSpeed, ["nan"] = nan, ["escapes"] = escapes, ["maxPenetration"] = maxPen };
        bool ok = nan == 0 && escapes == 0 && fell.Count == 0 && maxSpeed <= 12.5 && speedErr <= 0.16 && comErr <= 0.12 && maxPen <= 0.025;
        return Report("walkers-2d", steps, threads, res, ok);
    }
}
