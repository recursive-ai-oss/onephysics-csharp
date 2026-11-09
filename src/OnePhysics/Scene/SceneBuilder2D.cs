using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Scene;

/// <summary>Builds a 2D world from a scene definition. Shared by the bar runner and the hash-matrix scenarios. Revolute joints (the only 2D kind the scene format has) are created; a joint between two non-dynamic bodies is dropped.</summary>
internal static partial class SceneBuilder2D
{
    /// <summary>Applied to the world options before the world is created; the stability sweeps use it to perturb contact stiffness or the substep count. Null in every other use.</summary>

    public static (PhysicsWorld2D World, Body[] Bodies, WorldOptions2D Options) Build(Scene s, int threads) => Build(s, threads, out _);

    /// <summary>Builds the scene; bodies with a <c>character</c> script become characters (their bodies are in the array), returned with their scripts in body order.</summary>
    public static (PhysicsWorld2D World, Body[] Bodies, WorldOptions2D Options) Build(Scene s, int threads, out List<(Character Character, CharacterScript Script, int Body)> characters)
    {
        var opt = new WorldOptions2D { Gravity = new Vector2F((float)s.Gx, (float)s.Gy), ThreadCount = threads };
        if (s.Params.TryGetValue("world-id", out var ids)) opt.WorldId = int.Parse(ids[0], CultureInfo.InvariantCulture);
        if (s.Substeps > 0) opt.Substeps = s.Substeps;
        if (s.WorldLimits is BodyControlSettings worldLimits) { opt.MaxLinearSpeed = worldLimits.MaxLinearSpeed; opt.MaxAngularSpeed = worldLimits.MaxAngularSpeed; opt.PushOutVelocity = worldLimits.MaxDepenetrationSpeed; }
        var world = new PhysicsWorld2D(opt);

        if (s.Bodies.Exists(b => b.Script != null)) world.SetLayerCollision(1, 1, false); // characters pass through each other (layer 1); the world and the crates are layer 0
        var bodies = new Body[s.Bodies.Count];
        characters = new List<(Character, CharacterScript, int)>();
        for (int i = 0; i < bodies.Length; i++)
        {
            var ob = s.Bodies[i]; s.ActiveLine = ob.SourceLine;
            if (ob.Script != null)
            {
                var cap = ob.Shapes[0].P;
                float radius = (float)ob.Shapes[0].Radius;
                var cdef = new CharacterDef { Radius = radius, Height = (float)(cap[3] - cap[1]) + 2f * radius, PushStrength = (float)ob.Script.Push, Layer = 1, LayerMask = ~(1u << 1) };
                var ch = world.AddCharacter(new Vector2D(ob.X, ob.Y), cdef);
                bodies[i] = ch.Body;
                characters.Add((ch, ob.Script, i));
                continue;
            }

            var bd = new P2.BodyDef
            {
                Type = ob.Type == "static" ? BodyType.Static : ob.Type == "kinematic" ? BodyType.Kinematic : BodyType.Dynamic,
                Position = new Vector2D(ob.X, ob.Y),
                Angle = (float)ob.Angle,
                LinearVelocity = new Vector2F((float)ob.Vx, (float)ob.Vy),
                AngularVelocity = (float)ob.W,
                IsBullet = ob.Bullet,
                Controls = ob.Controls, SleepSettings = ob.SleepSettings, AllowSleep = !ob.NeverSleep, MassOverride = ob.MassOverride,
                CenterOfMassOverride = ob.CenterOverride.HasValue ? new Vector2F(ob.CenterOverride.Value.X, ob.CenterOverride.Value.Y) : null,
                InertiaOverride = ob.InertiaOverride.HasValue ? ob.InertiaOverride.Value.X : null,

            };
            bodies[i] = world.CreateBody(in bd);
            foreach (var sh in ob.Shapes) { s.ActiveLine = sh.SourceLine; CreateShape(world, bodies[i], sh, s); }
            if (ob.TargetPosition is Vector3D target) bodies[i].MoveKinematic(new(target.X,target.Y), ob.TargetAngle);
        }
        foreach (var j in s.Joints)
        {
            s.ActiveLine = j.SourceLine;
            if (BuildMachine(world, bodies, j)) { continue; }
            var ba = s.Bodies[j.A]; var bb = s.Bodies[j.B];
            if (ba.Type != "dynamic" && bb.Type != "dynamic") continue; // the scene format drops a joint between two non-dynamic bodies
            if (j.Kind != "revolute") throw new InvalidDataException("unsupported 2D joint kind " + j.Kind);
            var jd = new RevoluteJointDef
            {
                BodyA = bodies[j.A], BodyB = bodies[j.B],
                LocalAnchorA = new Vector2F((float)j.Ax, (float)j.Ay), LocalAnchorB = new Vector2F((float)j.Bx, (float)j.By),
                ReferenceAngle = (float)(bb.Angle - ba.Angle),
                EnableLimit = j.HasLimit, LowerAngle = (float)j.Lower, UpperAngle = (float)j.Upper,
                EnableMotor = j.HasMotor, MotorSpeed = (float)j.MotorSpeed, MaxMotorTorque = (float)j.MaxTorque,
            };
            var joint = world.CreateJoint(in jd); joint.BreakForce = j.BreakForce; joint.BreakTorque = j.BreakTorque; joint.LifetimeStepsRemaining = j.LifetimeSteps;
        }
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Asleep) bodies[i].Sleep();

        return (world, bodies, opt);
    }

    static void CreateShape(PhysicsWorld2D world, Body body, ShapeDef sh, Scene s)
    {
        var def = new P2.ShapeDef
        {
            Density = (float)sh.Density,
            Surface = sh.Surface,
            Friction = (float)s.Materials[sh.Material].Friction,
            Restitution = (float)s.Materials[sh.Material].Restitution,
        };
        var p = sh.P;
        switch (sh.Kind)
        {
            case "circle": world.CreateCircle(body, (float)sh.Radius, new Vector2F((float)p[0], (float)p[1]), def); break;
            case "capsule": world.CreateCapsule(body, new Vector2F((float)p[0], (float)p[1]), new Vector2F((float)p[2], (float)p[3]), (float)sh.Radius, def); break;
            case "box": world.CreateBox(body, new Vector2F(2f * ((float)p[0]), 2f * ((float)p[1])), new Vector2F((float)p[2], (float)p[3]), (float)p[4], (float)sh.Radius, def); break;
            case "polygon":
            {
                var pts = new Vector2F[p.Length / 2];
                for (int i = 0; i < pts.Length; i++) pts[i] = new Vector2F((float)p[2 * i], (float)p[2 * i + 1]);
                world.CreatePolygon(body, pts, (float)sh.Radius, def); break;
            }
            case "segment": world.CreateSegment(body, new Vector2F((float)p[0], (float)p[1]), new Vector2F((float)p[2], (float)p[3]), def); break;
            case "chain":
                // The scene format asks for one two-sided segment per link, as the references build it.
                if (sh.MaterialIndices.Length != 0 && sh.MaterialIndices.Length != p.Length / 2 - 1) throw new InvalidDataException("one material index per chain segment");
                for (int i = 0; i + 3 < p.Length; i += 2)
                {
                    var segment = world.CreateSegment(body, new Vector2F((float)p[i], (float)p[i + 1]), new Vector2F((float)p[i + 2], (float)p[i + 3]), def);
                    if (sh.MaterialIndices.Length != 0) segment.SetMaterials(IndexedMaterials(s), new[] { sh.MaterialIndices[i / 2] });
                }
                break;
            default: throw new InvalidDataException("unknown shape " + sh.Kind);
        }
        if (sh.MaterialIndices.Length != 0 && sh.Kind != "chain") body.GetShapes()[^1].SetMaterials(IndexedMaterials(s), sh.MaterialIndices);
    }
    private static P2.Material2D[] IndexedMaterials(Scene scene)
    {
        var result = new P2.Material2D[scene.Materials.Count];
        for (int i = 0; i < result.Length; i++) result[i] = new((float)scene.Materials[i].Friction, (float)scene.Materials[i].Restitution);
        return result;
    }
}
