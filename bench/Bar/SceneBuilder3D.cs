using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>Builds a 3D world from a scene definition. Shared by the bar runner and the hash-matrix scenarios. Joints are not supported yet and are skipped; queries are not run.</summary>
public static class SceneBuilder3D
{
    public static (P3.PhysicsWorld3D World, P3.Body[] Bodies, P3.WorldOptions3D Options) Build(Scene s, int threads)
    {
        var opt = new P3.WorldOptions3D { Gravity = new Vector3F((float)s.Gx, (float)s.Gy, (float)s.Gz), ThreadCount = threads };
        if (s.Substeps > 0) opt.Substeps = s.Substeps;
        var world = new P3.PhysicsWorld3D(opt);

        var bodies = new P3.Body[s.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            var ob = s.Bodies[i];
            var bd = new P3.BodyDef
            {
                Type = ob.Type == "static" ? BodyType.Static : ob.Type == "kinematic" ? BodyType.Kinematic : BodyType.Dynamic,
                Position = new Vector3D(ob.X, ob.Y, ob.Z),
                Rotation = new QuaternionF((float)ob.Qx, (float)ob.Qy, (float)ob.Qz, (float)ob.Qw),
                LinearVelocity = new Vector3F((float)ob.Vx, (float)ob.Vy, (float)ob.Vz),
                AngularVelocity = new Vector3F((float)ob.Wx, (float)ob.Wy, (float)ob.W),
                IsBullet = ob.Bullet,
            };
            bodies[i] = world.CreateBody(in bd);
            foreach (var sh in ob.Shapes) CreateShape(world, bodies[i], sh, s);
        }
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Asleep) bodies[i].Sleep();
        return (world, bodies, opt);
    }

    static void CreateShape(P3.PhysicsWorld3D world, P3.Body body, ShapeDef sh, Scene s)
    {
        var def = new P3.ShapeDef
        {
            Density = (float)sh.Density,
            Friction = (float)s.Materials[sh.Material].Friction,
            Restitution = (float)s.Materials[sh.Material].Restitution,
        };
        var p = sh.P;
        float r = (float)sh.Radius;
        switch (sh.Kind)
        {
            case "sphere":
                def.LocalPosition = new Vector3F((float)p[0], (float)p[1], (float)p[2]);
                world.CreateSphere(body, r, def); break;
            case "box":
                def.LocalPosition = new Vector3F((float)p[3], (float)p[4], (float)p[5]);
                def.LocalRotation = new QuaternionF((float)p[6], (float)p[7], (float)p[8], (float)p[9]);
                world.CreateBox(body, new Vector3F((float)p[0], (float)p[1], (float)p[2]), 0f, def); break;
            case "capsule":
            {
                // Capsule scenes run along y; the endpoints give the half length and the shape's centre.
                var a = new Vector3F((float)p[0], (float)p[1], (float)p[2]); var b = new Vector3F((float)p[3], (float)p[4], (float)p[5]);
                def.LocalPosition = (a + b) * 0.5f;
                world.CreateCapsule(body, (b - a).Length() * 0.5f, r, def); break;
            }
            case "hull":
            {
                var pts = new Vector3F[p.Length / 3];
                for (int i = 0; i < pts.Length; i++) pts[i] = new Vector3F((float)p[3 * i], (float)p[3 * i + 1], (float)p[3 * i + 2]);
                world.CreateConvexHull(body, pts, 0f, def); break;
            }
            case "mesh":
            {
                int nv = (int)p[0], ni = (int)p[1];
                var v = new Vector3F[nv]; var idx = new int[ni];
                for (int i = 0; i < nv; i++) v[i] = new Vector3F((float)p[2 + 3 * i], (float)p[3 + 3 * i], (float)p[4 + 3 * i]);
                for (int i = 0; i < ni; i++) idx[i] = (int)p[2 + 3 * nv + i];
                world.CreateMesh(body, v, idx, def); break;
            }
            case "heightfield":
            {
                int nx = (int)p[0], nz = (int)p[1]; float cell = (float)p[2];
                var h = new float[nx * nz];
                for (int i = 0; i < h.Length; i++) h[i] = (float)p[6 + i];
                def.LocalPosition = new Vector3F((float)p[3], (float)p[4], (float)p[5]);
                world.CreateHeightField(body, h, nx, nz, new Vector3F(cell, 1f, cell), default, def); break;
            }
            default: throw new InvalidDataException("unknown shape " + sh.Kind);
        }
    }
}

/// <summary>This library's 3D world behind the shared 3D driver loop.</summary>
sealed class OnePhysics3Sim : ISim3
{
    P3.PhysicsWorld3D world = null!;
    P3.Body[] bodies = [];
    int threads;

    public string Name => "onephysics";
    public string Version => "0.1.0-dev";
    public string Config => $"threads={threads}, substeps=4, default options";

    public void Build(Scene s, int threads)
    {
        this.threads = threads;
        (world, bodies, _) = SceneBuilder3D.Build(s, threads);
    }

    public void Step(double dt) => world.Step((float)dt);

    public void WakeAll(Scene s)
    {
        for (int i = 0; i < bodies.Length; i++) if (s.Bodies[i].Type == "dynamic") bodies[i].Wake();
    }

    public void Snapshot(double[] d)
    {
        for (int i = 0; i < bodies.Length; i++)
        {
            int o = i * 14; var b = bodies[i];
            var p = b.Position; var q = b.Rotation; var v = b.LinearVelocity; var w = b.AngularVelocity;
            d[o] = p.X; d[o + 1] = p.Y; d[o + 2] = p.Z; d[o + 3] = q.X; d[o + 4] = q.Y; d[o + 5] = q.Z; d[o + 6] = q.W;
            d[o + 7] = v.X; d[o + 8] = v.Y; d[o + 9] = v.Z; d[o + 10] = w.X; d[o + 11] = w.Y; d[o + 12] = w.Z;
            d[o + 13] = b.IsAwake ? 1 : 0;
        }
    }
}
