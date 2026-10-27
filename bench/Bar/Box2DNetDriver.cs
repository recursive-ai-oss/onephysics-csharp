using System.Diagnostics;
using System.Globalization;
using System.Text;
using Box2D.NET;
using static Box2D.NET.B2Worlds;
using static Box2D.NET.B2Bodies;
using static Box2D.NET.B2Shapes;
using static Box2D.NET.B2Geometries;
using static Box2D.NET.B2Hulls;
using static Box2D.NET.B2Joints;
using static Box2D.NET.B2RevoluteJoints;
using static Box2D.NET.B2Types;
using static Box2D.NET.B2MathFunction;
using static Box2D.NET.B2Distances;

namespace Bar;

/// <summary>Runs a scene through the Box2D.NET package (a C# port of Box2D 3.1), single-threaded, following the same driver protocol as the native drivers.</summary>
static class Box2DNetDriver
{
    public static int Run(Opts o)
    {
        var s = Scene.Load(o.Get("scene", ""));
        bool timing = o.Get("mode", "stability") == "timing";
        string result = o.Get("result", ""), dumpPath = o.Get("dump", "");

        var wd = b2DefaultWorldDef();
        wd.gravity = new B2Vec2((float)s.Gx, (float)s.Gy);
        var world = b2CreateWorld(wd);
        int subSteps = s.Substeps > 0 ? s.Substeps : 4;

        var ids = new B2BodyId[s.Bodies.Count];
        for (int i = 0; i < s.Bodies.Count; i++)
        {
            var ob = s.Bodies[i];
            var bd = b2DefaultBodyDef();
            bd.type = ob.Type == "static" ? B2BodyType.b2_staticBody : ob.Type == "kinematic" ? B2BodyType.b2_kinematicBody : B2BodyType.b2_dynamicBody;
            bd.position = new B2Vec2((float)ob.X, (float)ob.Y);
            bd.rotation = b2MakeRot((float)ob.Angle);
            bd.linearVelocity = new B2Vec2((float)ob.Vx, (float)ob.Vy);
            bd.angularVelocity = (float)ob.W;
            bd.isBullet = ob.Bullet;
            bd.isAwake = !ob.Asleep;
            ids[i] = b2CreateBody(world, bd);
            foreach (var sh in ob.Shapes) CreateShape(ids[i], sh, s);
        }
        foreach (var oj in s.Joints)
        {
            if (s.Bodies[oj.A].Type != "dynamic" && s.Bodies[oj.B].Type != "dynamic") continue;
            var jd = b2DefaultRevoluteJointDef();
            jd.@base.bodyIdA = ids[oj.A]; jd.@base.bodyIdB = ids[oj.B];
            jd.@base.localFrameA.p = new B2Vec2((float)oj.Ax, (float)oj.Ay);
            jd.@base.localFrameB.p = new B2Vec2((float)oj.Bx, (float)oj.By);
            jd.@base.localFrameA.q = b2MakeRot(0f);
            jd.@base.localFrameB.q = b2MakeRot((float)(s.Bodies[oj.A].Angle - s.Bodies[oj.B].Angle));
            jd.@base.collideConnected = false;
            if (oj.HasLimit) { jd.enableLimit = true; jd.lowerAngle = (float)oj.Lower; jd.upperAngle = (float)oj.Upper; }
            if (oj.HasMotor) { jd.enableMotor = true; jd.motorSpeed = (float)oj.MotorSpeed; jd.maxMotorTorque = (float)oj.MaxTorque; }
            b2CreateRevoluteJoint(world, jd);
        }

        int n = s.Bodies.Count;
        var d = new double[n * 7];
        void Snap()
        {
            for (int i = 0; i < n; i++)
            {
                var p = b2Body_GetPosition(ids[i]); var r = b2Body_GetRotation(ids[i]); var v = b2Body_GetLinearVelocity(ids[i]);
                d[i * 7] = p.X; d[i * 7 + 1] = p.Y; d[i * 7 + 2] = b2Rot_GetAngle(r); d[i * 7 + 3] = v.X; d[i * 7 + 4] = v.Y;
                d[i * 7 + 5] = b2Body_GetAngularVelocity(ids[i]); d[i * 7 + 6] = b2Body_IsAwake(ids[i]) ? 1 : 0;
            }
        }
        BinaryWriter? dump = null;
        if (!timing && dumpPath != "")
        {
            dump = new BinaryWriter(File.Create(dumpPath));
            dump.Write(Encoding.ASCII.GetBytes("OPDUMP01")); dump.Write((uint)n); dump.Write((uint)0);
        }
        void Frame(int step) { Snap(); dump!.Write((uint)step); dump.Write((uint)0); foreach (var v in d) dump.Write(v); }
        if (dump != null) Frame(0);

        var stepNs = new double[s.Steps];
        double nsPerTick = 1e9 / Stopwatch.Frequency;
        for (int step = 0; step < s.Steps; step++)
        {
            foreach (var e in s.Events) if (e.Step == step) for (int i = 0; i < n; i++) if (s.Bodies[i].Type == "dynamic") b2Body_SetAwake(ids[i], true);
            long t0 = Stopwatch.GetTimestamp();
            b2World_Step(world, (float)s.Dt, subSteps);
            stepNs[step] = (Stopwatch.GetTimestamp() - t0) * nsPerTick;
            if (dump != null && ((step + 1) % s.DumpEvery == 0 || step + 1 == s.Steps)) Frame(step + 1);
        }
        dump?.Dispose();

        var sb = new StringBuilder();
        sb.Append(CultureInfo.InvariantCulture, $"engine=box2dnet\nversion=3.1.654\nconfig=single-threaded, substeps={subSteps}, default world definition\nscene={s.Name}\nthreads=1\nsteps={s.Steps}\nwarmup={s.Warmup}\nbodies={n}\n");
        sb.Append("step_ns=").Append(string.Join(',', stepNs.Select(v => v.ToString("F0", CultureInfo.InvariantCulture)))).Append('\n');
        if (s.Queries.Count > 0) Queries(world, s, sb);
        File.WriteAllText(result, sb.ToString());
        b2DestroyWorld(world);
        return 0;
    }

    static void CreateShape(B2BodyId body, ShapeDef sh, Scene s)
    {
        var def = b2DefaultShapeDef();
        def.density = (float)sh.Density;
        def.material.friction = (float)s.Materials[sh.Material].Friction;
        def.material.restitution = (float)s.Materials[sh.Material].Restitution;
        var p = sh.P;
        switch (sh.Kind)
        {
            case "circle": { var c = new B2Circle(new B2Vec2((float)p[0], (float)p[1]), (float)sh.Radius); b2CreateCircleShape(body, def, c); break; }
            case "capsule": { var c = new B2Capsule(new B2Vec2((float)p[0], (float)p[1]), new B2Vec2((float)p[2], (float)p[3]), (float)sh.Radius); b2CreateCapsuleShape(body, def, c); break; }
            case "box":
            {
                var poly = b2MakeOffsetRoundedBox((float)p[0], (float)p[1], new B2Vec2((float)p[2], (float)p[3]), b2MakeRot((float)p[4]), (float)sh.Radius);
                b2CreatePolygonShape(body, def, poly); break;
            }
            case "polygon":
            {
                var pts = new B2Vec2[p.Length / 2];
                for (int i = 0; i < pts.Length; i++) pts[i] = new B2Vec2((float)p[2 * i], (float)p[2 * i + 1]);
                var hull = b2ComputeHull(pts, pts.Length);
                var poly = b2MakePolygon(hull, (float)sh.Radius);
                b2CreatePolygonShape(body, def, poly); break;
            }
            case "segment": { var seg = new B2Segment(new B2Vec2((float)p[0], (float)p[1]), new B2Vec2((float)p[2], (float)p[3])); b2CreateSegmentShape(body, def, seg); break; }
            case "chain":
                for (int i = 0; i + 3 < p.Length; i += 2)
                {
                    var seg = new B2Segment(new B2Vec2((float)p[i], (float)p[i + 1]), new B2Vec2((float)p[i + 2], (float)p[i + 3]));
                    b2CreateSegmentShape(body, def, seg);
                }
                break;
            default: throw new InvalidDataException("unknown shape " + sh.Kind);
        }
    }

    static void Queries(B2WorldId world, Scene s, StringBuilder sb)
    {
        var filter = b2DefaultQueryFilter();
        var ns = new double[3]; var count = new int[3]; var hits = new long[3];
        double nsPerTick = 1e9 / Stopwatch.Frequency;
        for (int rep = 0; rep < s.QueryReps; rep++)
        {
            bool measure = s.QueryReps == 1 || rep > 0;
            for (int kind = 0; kind < 3; kind++)
            {
                string name = kind == 0 ? "ray" : kind == 1 ? "overlap" : "cast";
                long h = 0; int c = 0; long t0 = Stopwatch.GetTimestamp();
                foreach (var q in s.Queries)
                {
                    if (q.Kind != name) continue;
                    c++;
                    var P = q.P;
                    if (kind == 0)
                    {
                        var r = b2World_CastRayClosest(world, new B2Vec2((float)P[0], (float)P[1]), new B2Vec2((float)(P[2] * P[4]), (float)(P[3] * P[4])), filter);
                        if (r.hit) h++;
                    }
                    else if (kind == 1)
                    {
                        var pts = new[] { new B2Vec2((float)P[0], (float)P[1]) };
                        var proxy = b2MakeProxy(pts, 1, (float)P[2]);
                        long cnt = 0;
                        b2World_OverlapShape(world, ref proxy, filter, (shape, ctx) => { cnt++; return true; }, null);
                        h += cnt;
                    }
                    else
                    {
                        var pts = new[] { new B2Vec2((float)P[0], (float)P[1]) };
                        var proxy = b2MakeProxy(pts, 1, (float)P[2]);
                        long hit = 0;
                        b2World_CastShape(world, ref proxy, new B2Vec2((float)P[3], (float)P[4]), filter, (shape, point, normal, fraction, ctx) => { hit = 1; return fraction; }, null);
                        h += hit;
                    }
                }
                if (measure) { ns[kind] += (Stopwatch.GetTimestamp() - t0) * nsPerTick; count[kind] += c; hits[kind] = h; }
            }
        }
        string[] names = ["ray", "overlap", "cast"];
        for (int k = 0; k < 3; k++) sb.Append(CultureInfo.InvariantCulture, $"query_{names[k]}_ns={ns[k]:F0}\nquery_{names[k]}_count={count[k]}\nquery_{names[k]}_hits={hits[k]}\n");
    }
}
