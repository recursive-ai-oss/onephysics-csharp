using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace Bar;

/// <summary>Builds a 2D world from a scene definition. Shared by the bar runner and the hash-matrix scenarios. Revolute joints (the only 2D kind the scene format has) are created; a joint between two non-dynamic bodies is dropped.</summary>
public static class SceneBuilder2D
{
    /// <summary>Applied to the world options before the world is created; the stability sweeps use it to perturb contact stiffness or the substep count. Null in every other use.</summary>
    public static Action<WorldOptions2D>? Tweak { get; set; }

    public static (PhysicsWorld2D World, Body[] Bodies, WorldOptions2D Options) Build(Scene s, int threads) => Build(s, threads, out _);

    /// <summary>Builds the scene; bodies with a <c>character</c> script become characters (their bodies are in the array), returned with their scripts in body order.</summary>
    public static (PhysicsWorld2D World, Body[] Bodies, WorldOptions2D Options) Build(Scene s, int threads, out List<(Character Character, CharacterScript Script, int Body)> characters)
    {
        var opt = new WorldOptions2D { Gravity = new Vector2F((float)s.Gx, (float)s.Gy), ThreadCount = threads };
        if (s.Substeps > 0) opt.Substeps = s.Substeps;
        Tweak?.Invoke(opt);
        var world = new PhysicsWorld2D(opt);

        if (s.Bodies.Exists(b => b.Script != null)) world.SetLayerCollision(1, 1, false); // characters pass through each other (layer 1); the world and the crates are layer 0
        var bodies = new Body[s.Bodies.Count];
        characters = new List<(Character, CharacterScript, int)>();
        for (int i = 0; i < bodies.Length; i++)
        {
            var ob = s.Bodies[i];
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
            };
            bodies[i] = world.CreateBody(in bd);
            foreach (var sh in ob.Shapes) CreateShape(world, bodies[i], sh, s);
        }
        foreach (var j in s.Joints)
        {
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
            world.CreateJoint(in jd);
        }
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Asleep) bodies[i].Sleep();

        return (world, bodies, opt);
    }

    static void CreateShape(PhysicsWorld2D world, Body body, ShapeDef sh, Scene s)
    {
        var def = new P2.ShapeDef
        {
            Density = (float)sh.Density,
            Friction = (float)s.Materials[sh.Material].Friction,
            Restitution = (float)s.Materials[sh.Material].Restitution,
        };
        var p = sh.P;
        switch (sh.Kind)
        {
            case "circle": world.CreateCircle(body, (float)sh.Radius, new Vector2F((float)p[0], (float)p[1]), def); break;
            case "capsule": world.CreateCapsule(body, new Vector2F((float)p[0], (float)p[1]), new Vector2F((float)p[2], (float)p[3]), (float)sh.Radius, def); break;
            case "box": world.CreateBox(body, (float)p[0], (float)p[1], new Vector2F((float)p[2], (float)p[3]), (float)p[4], (float)sh.Radius, def); break;
            case "polygon":
            {
                var pts = new Vector2F[p.Length / 2];
                for (int i = 0; i < pts.Length; i++) pts[i] = new Vector2F((float)p[2 * i], (float)p[2 * i + 1]);
                world.CreatePolygon(body, pts, (float)sh.Radius, def); break;
            }
            case "segment": world.CreateSegment(body, new Vector2F((float)p[0], (float)p[1]), new Vector2F((float)p[2], (float)p[3]), def); break;
            case "chain":
                // The scene format asks for one two-sided segment per link, as the references build it.
                for (int i = 0; i + 3 < p.Length; i += 2)
                    world.CreateSegment(body, new Vector2F((float)p[i], (float)p[i + 1]), new Vector2F((float)p[i + 2], (float)p[i + 3]), def);
                break;
            default: throw new InvalidDataException("unknown shape " + sh.Kind);
        }
    }
}
