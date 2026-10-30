using System.Numerics;
using Jitter2;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.SoftBodies;

namespace Bar;

/// <summary>Jitter2 driver, written from the project's published documentation and API reference.</summary>
static class Jitter2Driver
{
    public static int Run(Opts o) => Driver3.Run(o, new Jitter2Sim());

    /// <summary>Bodies joined by a constraint do not collide with each other (the pair is looked up by scene body index).</summary>
    sealed class JointedPairs : IBroadPhaseFilter
    {
        public readonly HashSet<long> Pairs = new();
        public bool Filter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
        {
            if (proxyA is not RigidBodyShape a || proxyB is not RigidBodyShape b) return true;
            if (a.RigidBody.Tag is not int x || b.RigidBody.Tag is not int y) return true;
            if (x > y) (x, y) = (y, x);
            return !Pairs.Contains(((long)x << 32) | (uint)y);
        }
    }

    sealed class Jitter2Sim : ISim3
    {
        public string Name => "jitter2";
        public string Version => "2.13.0";
        public string Config { get; private set; } = "";
        World world = null!;
        RigidBody[] bodies = [];
        bool[] isBody = [];
        Vector3[] shapeOffset = [];
        Scene scene = null!;
        int threads;
        readonly List<RigidBody[]> softVertices = [];

        public void Build(Scene s, int threadCount)
        {
            scene = s; threads = threadCount;
            Jitter2.Parallelization.ThreadPool.Instance.ChangeThreadCount(threadCount);
            world = new World();
            world.Gravity = new JVector((float)s.Gx, (float)s.Gy, (float)s.Gz);
            var (it, rel) = world.SolverIterations;
            Config = $"defaults (solver iterations {it}, relaxation {rel}, substeps {world.SubstepCount}, regular solve mode, sleeping on), thread pool of {threadCount} (caller included), multithreaded step only above 1 thread, heightfields as triangle meshes, queries not driven";
            int n = s.Bodies.Count;
            bodies = new RigidBody[n]; isBody = new bool[n]; shapeOffset = new Vector3[n];
            var filter = new JointedPairs();
            foreach (var j in s.Joints) { int a = Math.Min(j.A, j.B), b = Math.Max(j.A, j.B); filter.Pairs.Add(((long)a << 32) | (uint)b); }
            if (filter.Pairs.Count > 0) world.BroadPhaseFilter = filter;

            for (int i = 0; i < n; i++)
            {
                var b = s.Bodies[i];
                var rot = new Quaternion((float)b.Qx, (float)b.Qy, (float)b.Qz, (float)b.Qw);
                var pos = new Vector3((float)b.X, (float)b.Y, (float)b.Z);
                var body = world.CreateRigidBody();
                body.Tag = i;
                isBody[i] = true; bodies[i] = body;
                bool dynamic = b.Type == "dynamic" && b.Shapes.Count > 0;
                if (b.Type == "static")
                {
                    body.MotionType = MotionType.Static;
                    body.Position = ToJ(pos); body.Orientation = ToJ(rot);
                    var list = new List<RigidBodyShape>();
                    foreach (var sh in b.Shapes) AddStaticShapes(list, sh);
                    if (list.Count > 0) body.AddShapes(list, MassInertiaUpdateMode.Preserve); // static geometry has no mass properties
                    isBody[i] = false;
                    continue;
                }
                if (!dynamic)
                {
                    body.MotionType = MotionType.Kinematic;
                    body.Position = ToJ(pos); body.Orientation = ToJ(rot);
                    body.Velocity = new JVector((float)b.Vx, (float)b.Vy, (float)b.Vz); body.AngularVelocity = new JVector((float)b.Wx, (float)b.Wy, (float)b.W);
                    if (b.Shapes.Count == 1) { var (shp, off) = MakeShape(b.Shapes[0]); shapeOffset[i] = off; body.AddShape(shp); }
                    continue;
                }
                var sh0 = b.Shapes[0];
                var (shape, offset) = MakeShape(sh0);
                shapeOffset[i] = offset;
                body.Position = ToJ(pos + Vector3.Transform(offset, rot)); body.Orientation = ToJ(rot);
                body.AddShape(shape);
                body.SetMassInertia((float)(sh0.Density * Driver3.ShapeVolume(sh0))); // shapes assume density 1; the scene gives the mass
                body.Friction = (float)s.Materials[sh0.Material].Friction; body.Restitution = (float)s.Materials[sh0.Material].Restitution;
                body.Velocity = new JVector((float)b.Vx, (float)b.Vy, (float)b.Vz); body.AngularVelocity = new JVector((float)b.Wx, (float)b.Wy, (float)b.W);
                if (b.Bullet) body.EnableCcd = true;
                if (b.Asleep) body.SetActivationState(false);
            }
            foreach (var j in s.Joints)
            {
                if (s.Bodies[j.A].Type != "dynamic" && s.Bodies[j.B].Type != "dynamic") continue;
                var ba = s.Bodies[j.A];
                var rot = new Quaternion((float)ba.Qx, (float)ba.Qy, (float)ba.Qz, (float)ba.Qw);
                var anchor = new Vector3((float)ba.X, (float)ba.Y, (float)ba.Z) + Vector3.Transform(new Vector3((float)j.Ax, (float)j.Ay, (float)j.Az), rot);
                var c = world.CreateConstraint<BallSocket>(bodies[j.A], bodies[j.B]);
                c.Initialize(ToJ(anchor)); // world-space anchor, taken from the first body's pose at build time
            }
            BuildSofts(s);
        }

        // Soft bodies as the documentation describes them: every vertex a rigid body (a mass point), every edge a spring, and a triangle or tetrahedron shape per face or cell for collision.
        // The documentation gives no stiffness values, so the springs use 20 Hz and a damping ratio of 0.5 (cloth, rubber) or 8 Hz and 0.3 (jelly) at the step's time step.
        void BuildSofts(Scene s)
        {
            if (s.Softs.Count == 0) return;
            world.BroadPhaseFilter = new BroadPhaseCollisionFilter(world);
            world.DynamicTree.Filter = DynamicTreeCollisionFilter.Filter;
            foreach (var d in s.Softs)
            {
                var m = SoftGen.Build(d);
                var sb = new SoftBody(world);
                var q = new Quaternion((float)d.Qx, (float)d.Qy, (float)d.Qz, (float)d.Qw);
                var origin = new Vector3((float)d.X, (float)d.Y, (float)d.Z);
                var verts = new RigidBody[m.Count];
                var edges = SoftGen.Edges(m);
                double meanEdge = 0; foreach (var (a, b) in edges) meanEdge += Math.Sqrt(Dist2(m, a, b));
                meanEdge /= edges.Count;
                for (int i = 0; i < verts.Length; i++)
                {
                    var rb = world.CreateRigidBody();
                    rb.Position = ToJ(origin + Vector3.Transform(new Vector3(m.V[3 * i], m.V[3 * i + 1], m.V[3 * i + 2]), q));
                    rb.SetMassInertia((float)(d.Mass / m.Count));
                    verts[i] = rb; sb.Vertices.Add(rb);
                }
                foreach (int pin in m.Pins) verts[pin].MotionType = MotionType.Static;
                float freq = d.Preset == "jelly" ? 8f : 20f, damp = d.Preset == "jelly" ? 0.3f : 0.5f;
                foreach (var (a, b) in edges)
                {
                    var spring = world.CreateConstraint<SpringConstraint>(verts[a], verts[b]);
                    spring.Initialize(verts[a].Position, verts[b].Position);
                    spring.SetSpringParameters(freq, damp, (float)s.Dt);
                    sb.AddSpring(spring);
                }

                if (m.Tets.Length > 0)
                    for (int t = 0; t < m.Tets.Length; t += 4) sb.AddShape(new SoftBodyTetrahedron(sb, verts[m.Tets[t]], verts[m.Tets[t + 1]], verts[m.Tets[t + 2]], verts[m.Tets[t + 3]]));
                else
                    for (int t = 0; t < m.Tris.Length; t += 3) sb.AddShape(new SoftBodyTriangle(sb, verts[m.Tris[t]], verts[m.Tris[t + 1]], verts[m.Tris[t + 2]]) { Thickness = (float)(0.5 * meanEdge) });
                softVertices.Add(verts);
            }
        }

        static double Dist2(SoftMeshData m, int a, int b)
        {
            double dx = m.V[3 * a] - m.V[3 * b], dy = m.V[3 * a + 1] - m.V[3 * b + 1], dz = m.V[3 * a + 2] - m.V[3 * b + 2];
            return dx * dx + dy * dy + dz * dz;
        }

        public int SoftDoubles { get { int n = 0; foreach (var v in softVertices) n += 6 * v.Length; return n; } }

        public void SoftSnapshot(double[] d)
        {
            int k = 0;
            foreach (var verts in softVertices)
                foreach (var rb in verts)
                {
                    d[k++] = rb.Position.X; d[k++] = rb.Position.Y; d[k++] = rb.Position.Z; d[k++] = rb.Velocity.X; d[k++] = rb.Velocity.Y; d[k++] = rb.Velocity.Z;
                }
        }

        static JVector ToJ(Vector3 v) => new(v.X, v.Y, v.Z);
        static JQuaternion ToJ(Quaternion q) => new(q.X, q.Y, q.Z, q.W);

        void AddStaticShapes(List<RigidBodyShape> list, ShapeDef sh)
        {
            if (sh.Kind is "mesh" or "heightfield")
            {
                var tris = BepuDriver.BepuSim.TerrainTriangles(sh);
                var verts = new Vector3[tris.Count * 3]; var idx = new int[tris.Count * 3];
                for (int i = 0; i < tris.Count; i++) { verts[3 * i] = tris[i].Item1; verts[3 * i + 1] = tris[i].Item2; verts[3 * i + 2] = tris[i].Item3; idx[3 * i] = 3 * i; idx[3 * i + 1] = 3 * i + 1; idx[3 * i + 2] = 3 * i + 2; }
                var mesh = TriangleMesh.Create<Vector3>(verts, idx, true); // coincident vertices are merged by the library
                for (int i = 0; i < mesh.Indices.Length; i++) list.Add(new TriangleShape(mesh, i));
                return;
            }
            var (shp, off) = MakeShape(sh);
            var total = off; // MakeShape already includes a box's or sphere's centre
            if (total == Vector3.Zero) list.Add(shp);
            else list.Add(new TransformedShape(shp, ToJ(total), JMatrix.Identity));
        }

        // Returns the shape (centred on its own origin) and the offset of that origin from the scene's body origin.
        static (RigidBodyShape, Vector3) MakeShape(ShapeDef sh)
        {
            var p = sh.P; float r = (float)sh.Radius;
            switch (sh.Kind)
            {
                case "sphere": return (new SphereShape(r), new Vector3((float)p[0], (float)p[1], (float)p[2]));
                case "box": return (new BoxShape(2 * (float)p[0], 2 * (float)p[1], 2 * (float)p[2]), new Vector3((float)p[3], (float)p[4], (float)p[5]));
                case "capsule":
                {
                    var a = new Vector3((float)p[0], (float)p[1], (float)p[2]); var b = new Vector3((float)p[3], (float)p[4], (float)p[5]);
                    return (new CapsuleShape(r, (b - a).Length()), 0.5f * (a + b)); // length of the cylindrical part; aligned with y in the scenes
                }
                case "hull":
                {
                    var pts = new Vector3[p.Length / 3]; var c = Vector3.Zero;
                    for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector3((float)p[3 * i], (float)p[3 * i + 1], (float)p[3 * i + 2]); c += pts[i]; }
                    c /= pts.Length; // vertex average, close to the centre of mass for the scenes' hulls
                    var list = new List<JVector>(); foreach (var q in pts) list.Add(ToJ(q - c));
                    return (new PointCloudShape(list), c);
                }
                default: throw new InvalidDataException("unknown shape " + sh.Kind);
            }
        }

        public void Step(double dt) => world.Step((float)dt, threads > 1);

        public void WakeAll(Scene s)
        {
            for (int i = 0; i < bodies.Length; i++) if (isBody[i] && s.Bodies[i].Type == "dynamic") bodies[i].SetActivationState(true);
        }

        public void Snapshot(double[] d)
        {
            for (int i = 0; i < scene.Bodies.Count; i++)
            {
                int o = i * 14; var b = scene.Bodies[i];
                if (!isBody[i])
                {
                    d[o] = b.X; d[o + 1] = b.Y; d[o + 2] = b.Z; d[o + 3] = b.Qx; d[o + 4] = b.Qy; d[o + 5] = b.Qz; d[o + 6] = b.Qw;
                    for (int k = 7; k < 14; k++) d[o + k] = 0; continue;
                }
                var r = bodies[i]; var q = r.Orientation; var qq = new Quaternion(q.X, q.Y, q.Z, q.W);
                var pos = new Vector3(r.Position.X, r.Position.Y, r.Position.Z) - Vector3.Transform(shapeOffset[i], qq);
                d[o] = pos.X; d[o + 1] = pos.Y; d[o + 2] = pos.Z; d[o + 3] = q.X; d[o + 4] = q.Y; d[o + 5] = q.Z; d[o + 6] = q.W;
                d[o + 7] = r.Velocity.X; d[o + 8] = r.Velocity.Y; d[o + 9] = r.Velocity.Z;
                d[o + 10] = r.AngularVelocity.X; d[o + 11] = r.AngularVelocity.Y; d[o + 12] = r.AngularVelocity.Z; d[o + 13] = r.IsActive ? 1 : 0;
            }
        }
    }
}
