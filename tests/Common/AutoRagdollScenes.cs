#pragma warning disable OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Tools;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>What a run of an auto-ragdoll scene measured. Thresholds are the scene's own (no other engine builds a ragdoll from a skeleton).</summary>
public readonly record struct AutoRagdollMetrics(double Nan, double Escapes, double MaxSpeed, double FinalSpeed95, double JointError, double LowestPelvis, int Falls, double MaxHeightError)
{
    public bool Passes(bool standing) => Nan == 0 && Escapes == 0 && MaxSpeed <= 30 && JointError <= (standing ? 0.15 : 0.3) && (standing ? Falls == 0 && MaxHeightError <= 0.2 : FinalSpeed95 <= 0.5 && LowestPelvis >= 0.0);
}

/// <summary>
/// Ragdolls made by <see cref="AutoRagdoller"/> from a generated skeleton and skinned mesh: <c>autoragdoll-pile-3d</c> (eight limp figures of different size, mass and layout dropped onto a floor, 600 steps),
/// <c>autoragdoll-stand-3d</c> (one figure standing under the balance controller with the bone mapping the tool made, pushed once), and the 2D pile and stand. Shared by the tests, the stability runner and the hash matrix.
/// </summary>
public static class AutoRagdollScenes
{
    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static float Unit(ref ulong s) => (Next(ref s) >> 40) * (1f / 16777216f);

    /// <summary>Swing half-angle floor of the pile's figures: the engine's swing-limit solver blows limp ragdolls up in a large share of tumbling drops (see docs), so the pile runs with the swing cones opened to this many radians; twist limits stay.</summary>
    public const float PileMinSwing = 3.0f;

    public static AutoRagdoll3D Make3D(int variant, float height, float mass, float minSwing = 0f)
    {
        var sk = TestSkeleton.Humanoid(height, rightFirst: (variant & 1) != 0);
        var o = new AutoRagdollOptions { TotalMass = mass, IncludeHands = variant % 3 == 1, SplitTorso = variant % 3 == 2, MinSwingHalfAngle = minSwing };
        return AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions, sk.Skin, o);
    }

    public static AutoRagdoll2D Make2D(int variant, float height, float mass)
    {
        var sk = TestSkeleton.Humanoid(height, rightFirst: (variant & 1) != 0);
        var o = new AutoRagdollOptions { TotalMass = mass, IncludeHands = variant % 3 == 1, Facing = 1 };
        return AutoRagdoller.Create2D(sk.Names, sk.Parents, sk.Positions2D(), sk.Skin2D(), o);
    }

    /// <summary>Builds the 3D pile: a floor and <paramref name="count"/> limp figures dropped from a staggered grid with seeded orientations.</summary>
    public static P3.PhysicsWorld3D BuildPile3D(out P3.Ragdoll[] rags, out P3.RagdollDef[] defs, int count = 8, int threads = 1, ulong seed = 4242, float minSwing = PileMinSwing, bool engineHumanoid = false, bool figuresCollide = true)
    {
        var o = new P3.WorldOptions3D { ThreadCount = threads, WorldId = 91 };
        P3.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P3.PhysicsWorld3D(o);
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        rags = new P3.Ragdoll[count]; defs = new P3.RagdollDef[count];
        for (int i = 0; i < count; i++)
        {
            float height = 1.5f + 0.06f * i, mass = 50f + 7f * i;
            var def = engineHumanoid ? P3.RagdollDef.CreateHumanoid(height, mass) : Make3D(i, height, mass, minSwing).Definition;
            if (!figuresCollide) { def.CategoryBits = 2u; def.MaskBits = 1u; }
            defs[i] = def;
            double x = (i % 4) * 2.6 - 3.9, z = (i / 4) * 2.6 - 1.3, y = 2.3 + 0.6 * (i % 3);
            var axis = new Vector3F(Unit(ref seed) - 0.5f, Unit(ref seed) + 0.3f, Unit(ref seed) - 0.5f).Normalized();
            var q = QuaternionF.CreateFromAxisAngle(axis, Unit(ref seed) * 2f);
            rags[i] = w.AddRagdoll(def, new Vector3D(x, y, z), q);
        }

        return w;
    }

    /// <summary>The largest gap between the two anchors of any joint of the ragdoll, m.</summary>
    public static double JointError(P3.Ragdoll r, P3.RagdollDef def)
    {
        double worst = 0;
        for (int i = 1; i < def.Parts.Length; i++)
        {
            var p = def.Parts[i]; var pp = def.Parts[p.Parent];
            var bp = r.GetBody(p.Parent); var bc = r.GetBody(i);
            var qp = bp.Rotation * pp.Rotation.Conjugated(); var qc = bc.Rotation * p.Rotation.Conjugated();
            var ap = qp.Rotate(p.JointPosition - pp.Position); var ac = qc.Rotate(p.JointPosition - p.Position);
            var pa = bp.Position; var pc = bc.Position;
            double dx = pa.X + ap.X - pc.X - ac.X, dy = pa.Y + ap.Y - pc.Y - ac.Y, dz = pa.Z + ap.Z - pc.Z - ac.Z;
            worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy + dz * dz));
        }

        return worst;
    }

    /// <summary>The world hash after each of <paramref name="steps"/> steps of the 3D pile, folded in order; for the thread-count and platform checks.</summary>
    public static ulong HashPile3D(int steps, int threads)
    {
        var w = BuildPile3D(out _, out _, 8, threads);
        ulong h = 14695981039346656037UL;
        for (int s = 0; s < steps; s++) { w.Step(1f / 60f); h = (h ^ w.ComputeHash()) * 1099511628211UL; }
        return h;
    }

    public static AutoRagdollMetrics RunPile3D(int steps = 600, int threads = 1, ulong seed = 4242, float minSwing = PileMinSwing, bool engineHumanoid = false, bool figuresCollide = true)
    {
        var w = BuildPile3D(out var rags, out var defs, 8, threads, seed, minSwing, engineHumanoid, figuresCollide);
        double maxSpeed = 0, nan = 0, escapes = 0, jointErr = 0;
        for (int s = 0; s < steps; s++)
        {
            w.Step(1f / 60f);
            for (int k = 0; k < rags.Length; k++)
            {
                for (int part = 0; part < rags[k].PartCount; part++)
                {
                    var b = rags[k].GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                    if (p.Y < -0.5 || p.Y > 20 || Math.Abs(p.X) > 30 || Math.Abs(p.Z) > 30) { escapes++; }
                }

                jointErr = Math.Max(jointErr, JointError(rags[k], defs[k]));
            }
        }

        var speeds = new System.Collections.Generic.List<double>();
        double lowest = double.MaxValue;
        for (int k = 0; k < rags.Length; k++)
        {
            lowest = Math.Min(lowest, rags[k].GetBody(0).Position.Y);
            for (int part = 0; part < rags[k].PartCount; part++) { var v = rags[k].GetBody(part).LinearVelocity; speeds.Add(Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z)); }
        }

        speeds.Sort();
        return new AutoRagdollMetrics(nan, escapes, maxSpeed, speeds[(int)(0.95 * (speeds.Count - 1))], jointErr, lowest, 0, 0);
    }

    /// <summary>A figure made from the skeleton standing under the balance controller, with the bone mapping the tool produced; <paramref name="push"/> gives it an impulse at step 120.</summary>
    public static P3.PhysicsWorld3D BuildStand3D(out P3.Ragdoll rag, out AutoRagdoll3D auto, int threads = 1)
    {
        var o = new P3.WorldOptions3D { ThreadCount = threads, WorldId = 92 };
        P3.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P3.PhysicsWorld3D(o);
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        auto = Make3D(0, 1.8f, 75f);
        rag = w.AddRagdoll(auto.Definition, new Vector3D(0, 0, 0));
        rag.EnableActive(new ActiveRagdollDef { Bones = auto.Biped! });
        rag.Stand();
        return w;
    }

    public static AutoRagdollMetrics RunStand3D(int steps = 600, float push = 12f)
    {
        var w = BuildStand3D(out var rag, out var auto);
        double standY = auto.Definition.Parts[0].Position.Y;
        double maxSpeed = 0, nan = 0, escapes = 0, jointErr = 0, hErr = 0; int falls = 0;
        for (int s = 0; s < steps; s++)
        {
            if (s == 120) { rag.GetBody(auto.Biped!.Torso).ApplyImpulse(new Vector3F(0, 0, push)); }
            w.Step(1f / 60f);
            for (int part = 0; part < rag.PartCount; part++)
            {
                var b = rag.GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                if (p.Y < -0.5 || p.Y > 20 || Math.Abs(p.X) > 30 || Math.Abs(p.Z) > 30) { escapes++; }
            }

            jointErr = Math.Max(jointErr, JointError(rag, auto.Definition));
            double py = rag.GetBody(0).Position.Y;
            if (py < 0.6 * standY) { falls = 1; }
            if (s > 60) { hErr = Math.Max(hErr, Math.Abs(py - standY)); }
        }

        return new AutoRagdollMetrics(nan, escapes, maxSpeed, 0, jointErr, rag.GetBody(0).Position.Y, falls, hErr);
    }

    // ------------------------------------------------------------------ 2D

    public static P2.PhysicsWorld2D BuildPile2D(out P2.Ragdoll[] rags, out P2.RagdollDef[] defs, int count = 4, int threads = 1)
    {
        var o = new P2.WorldOptions2D { ThreadCount = threads, WorldId = 93 };
        P2.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P2.PhysicsWorld2D(o);
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        rags = new P2.Ragdoll[count]; defs = new P2.RagdollDef[count];
        for (int i = 0; i < count; i++)
        {
            var a = Make2D(i, 1.5f + 0.1f * i, 50f + 10f * i);
            defs[i] = a.Definition;
            rags[i] = w.AddRagdoll(a.Definition, new Vector2D(i * 3.0 - 4.5, 1.0 + 0.5 * i), 0.9f * (i - 1.5f));
        }

        return w;
    }

    public static double JointError(P2.Ragdoll r, P2.RagdollDef def)
    {
        double worst = 0;
        for (int i = 1; i < def.Parts.Length; i++)
        {
            var p = def.Parts[i]; var pp = def.Parts[p.Parent];
            var bp = r.GetBody(p.Parent); var bc = r.GetBody(i);
            double ap = bp.Angle - pp.Angle, ac = bc.Angle - p.Angle;
            var (sp, cp) = Math64.SinCos(ap); var (sc, cc) = Math64.SinCos(ac);
            double lpx = p.JointPosition.X - pp.Position.X, lpy = p.JointPosition.Y - pp.Position.Y, lcx = p.JointPosition.X - p.Position.X, lcy = p.JointPosition.Y - p.Position.Y;
            var pa = bp.Position; var pc = bc.Position;
            double dx = pa.X + cp * lpx - sp * lpy - pc.X - (cc * lcx - sc * lcy), dy = pa.Y + sp * lpx + cp * lpy - pc.Y - (sc * lcx + cc * lcy);
            worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy));
        }

        return worst;
    }

    public static AutoRagdollMetrics RunPile2D(int steps = 600, int threads = 1)
    {
        var w = BuildPile2D(out var rags, out var defs, 4, threads);
        double maxSpeed = 0, nan = 0, escapes = 0, jointErr = 0;
        for (int s = 0; s < steps; s++)
        {
            w.Step(1f / 60f);
            for (int k = 0; k < rags.Length; k++)
            {
                for (int part = 0; part < rags[k].PartCount; part++)
                {
                    var b = rags[k].GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y));
                    if (p.Y < -0.5 || p.Y > 20 || Math.Abs(p.X) > 30) { escapes++; }
                }

                jointErr = Math.Max(jointErr, JointError(rags[k], defs[k]));
            }
        }

        var speeds = new System.Collections.Generic.List<double>();
        double lowest = double.MaxValue;
        for (int k = 0; k < rags.Length; k++)
        {
            lowest = Math.Min(lowest, rags[k].GetBody(0).Position.Y);
            for (int part = 0; part < rags[k].PartCount; part++) { var v = rags[k].GetBody(part).LinearVelocity; speeds.Add(Math.Sqrt(v.X * v.X + v.Y * v.Y)); }
        }

        speeds.Sort();
        return new AutoRagdollMetrics(nan, escapes, maxSpeed, speeds[(int)(0.95 * (speeds.Count - 1))], jointErr, lowest, 0, 0);
    }

    public static P2.PhysicsWorld2D BuildStand2D(out P2.Ragdoll rag, out AutoRagdoll2D auto, int threads = 1)
    {
        var o = new P2.WorldOptions2D { ThreadCount = threads, WorldId = 94 };
        P2.Ragdoll.ApplyRecommendedOptions(o);
        var w = new P2.PhysicsWorld2D(o);
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        auto = Make2D(0, 1.8f, 75f);
        rag = w.AddRagdoll(auto.Definition, new Vector2D(0, 0));
        rag.EnableActive(new ActiveRagdollDef { Bones = auto.Biped! });
        rag.Stand();
        return w;
    }

    public static AutoRagdollMetrics RunStand2D(int steps = 600, float push = 25f)
    {
        var w = BuildStand2D(out var rag, out var auto);
        double standY = auto.Definition.Parts[0].Position.Y;
        double maxSpeed = 0, nan = 0, escapes = 0, jointErr = 0, hErr = 0; int falls = 0;
        for (int s = 0; s < steps; s++)
        {
            if (s == 120) { rag.GetBody(auto.Biped!.Torso).ApplyImpulse(new Vector2F(push, 0)); }
            w.Step(1f / 60f);
            for (int part = 0; part < rag.PartCount; part++)
            {
                var b = rag.GetBody(part); var p = b.Position; var v = b.LinearVelocity;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y));
                if (p.Y < -0.5 || p.Y > 20 || Math.Abs(p.X) > 30) { escapes++; }
            }

            jointErr = Math.Max(jointErr, JointError(rag, auto.Definition));
            double py = rag.GetBody(0).Position.Y;
            if (py < 0.6 * standY) { falls = 1; }
            if (s > 60) { hErr = Math.Max(hErr, Math.Abs(py - standY)); }
        }

        return new AutoRagdollMetrics(nan, escapes, maxSpeed, 0, jointErr, rag.GetBody(0).Position.Y, falls, hErr);
    }
}
