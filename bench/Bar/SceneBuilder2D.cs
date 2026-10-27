using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

namespace Bar;

/// <summary>Builds a 2D world from a scene definition. Shared by the bar runner and the hash-matrix scenarios. Joints are not supported yet and are skipped.</summary>
public static class SceneBuilder2D
{
    public static (PhysicsWorld2D World, Body[] Bodies, WorldOptions2D Options) Build(Scene s, int threads)
    {
        var opt = new WorldOptions2D { Gravity = new Vector2F((float)s.Gx, (float)s.Gy), ThreadCount = threads };
        if (s.Substeps > 0) opt.Substeps = s.Substeps;
        var world = new PhysicsWorld2D(opt);

        var bodies = new Body[s.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            var ob = s.Bodies[i];
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
