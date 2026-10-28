using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>Builds a 3D world from a scene definition. Shared by the bar runner and the hash-matrix scenarios. Point joints are built from the scene (the only 3D joint kind the scenes use); queries are not run.</summary>
public static class SceneBuilder3D
{
    public static (P3.PhysicsWorld3D World, P3.Body[] Bodies, P3.WorldOptions3D Options) Build(Scene s, int threads) => Build(s, threads, out _);

    /// <summary>Builds the scene; bodies with a <c>character</c> script become virtual characters (their bodies are in the array), returned with their scripts in body order.</summary>
    public static (P3.PhysicsWorld3D World, P3.Body[] Bodies, P3.WorldOptions3D Options) Build(Scene s, int threads, out List<(P3.Character Character, CharacterScript Script, int Body)> characters) =>
        Build(s, threads, out characters, out _);

    /// <summary>As above, and bodies with a <c>vehicle</c> script get a vehicle built from the named preset (see docs/scenes.md); they are returned with their scripts in body order.</summary>
    public static (P3.PhysicsWorld3D World, P3.Body[] Bodies, P3.WorldOptions3D Options) Build(Scene s, int threads, out List<(P3.Character Character, CharacterScript Script, int Body)> characters, out List<(P3.Vehicle Vehicle, VehicleScript Script, int Body)> vehicles)
    {
        var opt = new P3.WorldOptions3D { Gravity = new Vector3F((float)s.Gx, (float)s.Gy, (float)s.Gz), ThreadCount = threads };
        if (s.Substeps > 0) opt.Substeps = s.Substeps;
        var world = new P3.PhysicsWorld3D(opt);

        if (s.Bodies.Exists(b => b.Script != null)) world.SetLayerCollision(1, 1, false); // characters pass through each other (layer 1); the world and the crates are layer 0
        var bodies = new P3.Body[s.Bodies.Count];
        characters = new List<(P3.Character, CharacterScript, int)>();
        vehicles = new List<(P3.Vehicle, VehicleScript, int)>();
        for (int i = 0; i < bodies.Length; i++)
        {
            var ob = s.Bodies[i];
            if (ob.Script != null)
            {
                var cap = ob.Shapes[0].P;
                float radius = (float)ob.Shapes[0].Radius;
                var cdef = new P3.CharacterDef { Radius = radius, Height = (float)(cap[4] - cap[1]) + 2f * radius, PushStrength = (float)ob.Script.Push, Layer = 1, LayerMask = ~(1u << 1) };
                var ch = world.AddCharacter(new Vector3D(ob.X, ob.Y, ob.Z), cdef);
                bodies[i] = ch.Body;
                characters.Add((ch, ob.Script, i));
                continue;
            }
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
            if (ob.Vehicle != null)
            {
                if (ob.Vehicle.Preset != "car") throw new InvalidDataException("unknown vehicle preset " + ob.Vehicle.Preset);
                vehicles.Add((world.AddVehicle(bodies[i], P3.VehicleDef.Car), ob.Vehicle, i));
            }
        }
        foreach (var j in s.Joints)
        {
            if (j.Kind != "point") throw new InvalidDataException("3D joint kind not supported by the builder: " + j.Kind);
            var jd = new P3.PointJointDef
            {
                BodyA = bodies[j.A], BodyB = bodies[j.B],
                LocalFrameA = new P3.JointFrame3D(new Vector3F((float)j.Ax, (float)j.Ay, (float)j.Az), QuaternionF.Identity),
                LocalFrameB = new P3.JointFrame3D(new Vector3F((float)j.Bx, (float)j.By, (float)j.Bz), QuaternionF.Identity),
            };
            world.CreateJoint(in jd);
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
