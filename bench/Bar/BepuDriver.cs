using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;

namespace Bar;

static class BepuDriver
{
    public static int Run(Opts o) => Driver3.Run(o, new BepuSim());

    struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public static HashSet<int> Jointed = new(); // joined body handle pairs, encoded (min << 16 | max)
        public void Initialize(Simulation simulation) { }
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            if (a.Mobility != CollidableMobility.Dynamic && b.Mobility != CollidableMobility.Dynamic) return false;
            if (a.Mobility != CollidableMobility.Static && b.Mobility != CollidableMobility.Static)
            {
                int x = a.BodyHandle.Value, y = b.BodyHandle.Value; if (x > y) (x, y) = (y, x);
                if (Jointed.Contains(x << 16 | y)) return false;
            }
            return true;
        }
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB) => true;
        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial.FrictionCoefficient = 0.6f; pairMaterial.MaximumRecoveryVelocity = 2f; pairMaterial.SpringSettings = new SpringSettings(30, 1);
            return true;
        }
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold) => true;
        public void Dispose() { }
    }

    struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 Gravity; Vector3Wide gravityDt;
        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        public readonly bool AllowSubstepsForUnconstrainedBodies => false;
        public readonly bool IntegrateVelocityForKinematics => false;
        public void Initialize(Simulation simulation) { }
        public void PrepareForIntegration(float dt) { gravityDt = Vector3Wide.Broadcast(Gravity * dt); }
        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
        {
            velocity.Linear += gravityDt;
        }
    }

    internal sealed class BepuSim : ISim3
    {
        public string Name => "bepu";
        public string Version => "2.5.0-beta.33";
        public string Config { get; private set; } = "";
        BufferPool pool = new();
        Simulation sim = null!;
        ThreadDispatcher? dispatcher;
        BodyHandle[] handles = [];
        bool[] isBody = [];
        Vector3[] shapeOffset = []; // hulls are re-centred by the engine; the offset restores the scene's body origin
        Scene scene = null!;

        public void Build(Scene s, int threads)
        {
            scene = s;
            dispatcher = threads > 1 ? new ThreadDispatcher(threads) : null;
            sim = Simulation.Create(pool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks { Gravity = new Vector3((float)s.Gx, (float)s.Gy, (float)s.Gz) }, new SolveDescription(8, 1));
            Config = $"default solve description (8 velocity iterations, 1 substep), ThreadDispatcher({threads}), contact spring 30 Hz / damping 1, heightfields as triangle meshes, queries not driven";
            int n = s.Bodies.Count;
            handles = new BodyHandle[n]; isBody = new bool[n]; shapeOffset = new Vector3[n];
            var hasJoint = new bool[n];
            foreach (var j in s.Joints) { hasJoint[j.A] = hasJoint[j.B] = true; }
            for (int i = 0; i < n; i++)
            {
                var b = s.Bodies[i];
                var rot = new Quaternion((float)b.Qx, (float)b.Qy, (float)b.Qz, (float)b.Qw);
                var pos = new Vector3((float)b.X, (float)b.Y, (float)b.Z);
                bool asBody = b.Type != "static" || hasJoint[i];
                if (!asBody)
                {
                    foreach (var sh in b.Shapes) // statics: one per shape (the containers are the only multi-shape statics)
                    {
                        var (idx, off, _) = MakeShape(sh);
                        var center = sh.Kind == "box" ? new Vector3((float)sh.P[3], (float)sh.P[4], (float)sh.P[5]) : Vector3.Zero;
                        sim.Statics.Add(new StaticDescription(pos + Vector3.Transform(center + off, rot), rot, idx));
                    }
                    continue;
                }
                isBody[i] = true;
                var lin = new Vector3((float)b.Vx, (float)b.Vy, (float)b.Vz); var ang = new Vector3((float)b.Wx, (float)b.Wy, (float)b.W);
                if (b.Shapes.Count == 0 || b.Type != "dynamic")
                {
                    TypedIndex idx = default; Vector3 off = Vector3.Zero;
                    if (b.Shapes.Count == 1) (idx, off, _) = MakeShape(b.Shapes[0]);
                    var c = b.Shapes.Count == 1 ? new CollidableDescription(idx, 0.1f) : new CollidableDescription(sim.Shapes.Add(new Sphere(0.01f)), 0f);
                    shapeOffset[i] = off;
                    handles[i] = sim.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(pos + Vector3.Transform(off, rot), rot), new BodyVelocity(lin, ang), c, new BodyActivityDescription(-1f)));
                    continue;
                }
                var sh0 = b.Shapes[0];
                var (shape, offset, inertia) = MakeShape(sh0, (float)(sh0.Density * Driver3.ShapeVolume(sh0)));
                shapeOffset[i] = offset;
                handles[i] = sim.Bodies.Add(BodyDescription.CreateDynamic(new RigidPose(pos + Vector3.Transform(offset, rot), rot), new BodyVelocity(lin, ang), inertia, new CollidableDescription(shape, 0.1f), new BodyActivityDescription(0.01f)));
                if (b.Asleep) { var rf = sim.Bodies.GetBodyReference(handles[i]); rf.Awake = false; }
            }
            foreach (var j in s.Joints)
            {
                if (s.Bodies[j.A].Type != "dynamic" && s.Bodies[j.B].Type != "dynamic") continue;
                int x = handles[j.A].Value, y = handles[j.B].Value; if (x > y) (x, y) = (y, x);
                NarrowPhaseCallbacks.Jointed.Add(x << 16 | y);
                var offA = shapeOffset[j.A]; var offB = shapeOffset[j.B];
                sim.Solver.Add(handles[j.A], handles[j.B], new BallSocket { LocalOffsetA = new Vector3((float)j.Ax, (float)j.Ay, (float)j.Az) - offA, LocalOffsetB = new Vector3((float)j.Bx, (float)j.By, (float)j.Bz) - offB, SpringSettings = new SpringSettings(30, 1) });
            }
        }

        // Returns the shape index, the offset of the engine's body origin from the scene's body origin (in body space), and the inertia for the given mass.
        (TypedIndex, Vector3, BodyInertia) MakeShape(ShapeDef sh, float mass = 1f)
        {
            var p = sh.P; float r = (float)sh.Radius;
            switch (sh.Kind)
            {
                case "sphere": { var x = new Sphere(r); return (sim.Shapes.Add(x), Vector3.Zero, x.ComputeInertia(mass)); }
                case "box": { var x = new Box(2 * (float)p[0], 2 * (float)p[1], 2 * (float)p[2]); return (sim.Shapes.Add(x), Vector3.Zero, x.ComputeInertia(mass)); }
                case "capsule":
                {
                    var a = new Vector3((float)p[0], (float)p[1], (float)p[2]); var b = new Vector3((float)p[3], (float)p[4], (float)p[5]);
                    var x = new Capsule(r, (b - a).Length()); return (sim.Shapes.Add(x), 0.5f * (a + b), x.ComputeInertia(mass));
                }
                case "hull":
                {
                    var pts = new Vector3[p.Length / 3]; for (int i = 0; i < pts.Length; i++) pts[i] = new Vector3((float)p[3 * i], (float)p[3 * i + 1], (float)p[3 * i + 2]);
                    var x = new ConvexHull(pts, pool, out var center); return (sim.Shapes.Add(x), center, x.ComputeInertia(mass));
                }
                case "mesh": case "heightfield":
                {
                    var tris = TerrainTriangles(sh);
                    pool.Take<Triangle>(tris.Count, out var buf);
                    for (int i = 0; i < tris.Count; i++) buf[i] = new Triangle(tris[i].Item1, tris[i].Item3, tris[i].Item2); // the engine's mesh triangles are one-sided; this winding faces up for the scenes
                    var x = new Mesh(buf, Vector3.One, pool); return (sim.Shapes.Add(x), Vector3.Zero, default);
                }
                default: throw new InvalidDataException("unknown shape " + sh.Kind);
            }
        }

        public static List<(Vector3, Vector3, Vector3)> TerrainTriangles(ShapeDef sh)
        {
            var p = sh.P; var list = new List<(Vector3, Vector3, Vector3)>();
            if (sh.Kind == "mesh")
            {
                int nv = (int)p[0], ni = (int)p[1];
                Vector3 V(int idx) { int o = 2 + 3 * (int)p[2 + 3 * nv + idx]; return new Vector3((float)p[o], (float)p[o + 1], (float)p[o + 2]); }
                for (int t = 0; t < ni; t += 3) list.Add((V(t), V(t + 1), V(t + 2)));
            }
            else
            {
                int nx = (int)p[0], nz = (int)p[1]; double cell = p[2];
                Vector3 At(int i, int j) => new Vector3((float)(p[3] + i * cell), (float)(p[4] + p[6 + j * nx + i]), (float)(p[5] + j * cell));
                for (int j = 0; j + 1 < nz; j++) for (int i = 0; i + 1 < nx; i++) { list.Add((At(i, j), At(i, j + 1), At(i + 1, j))); list.Add((At(i + 1, j), At(i, j + 1), At(i + 1, j + 1))); }
            }
            return list;
        }

        public void Step(double dt) => sim.Timestep((float)dt, dispatcher);

        public void WakeAll(Scene s)
        {
            for (int i = 0; i < handles.Length; i++) if (isBody[i] && s.Bodies[i].Type == "dynamic") sim.Awakener.AwakenBody(handles[i]);
        }

        public void Snapshot(double[] d)
        {
            for (int i = 0; i < scene.Bodies.Count; i++)
            {
                int o = i * 14; var b = scene.Bodies[i];
                if (!isBody[i])
                {
                    d[o] = b.X; d[o + 1] = b.Y; d[o + 2] = b.Z; d[o + 3] = b.Qx; d[o + 4] = b.Qy; d[o + 5] = b.Qz; d[o + 6] = b.Qw;
                    for (int k = 7; k < 13; k++) d[o + k] = 0; d[o + 13] = 0; continue;
                }
                var r = sim.Bodies.GetBodyReference(handles[i]); var q = r.Pose.Orientation; var pos = r.Pose.Position - Vector3.Transform(shapeOffset[i], q);
                d[o] = pos.X; d[o + 1] = pos.Y; d[o + 2] = pos.Z; d[o + 3] = q.X; d[o + 4] = q.Y; d[o + 5] = q.Z; d[o + 6] = q.W;
                d[o + 7] = r.Velocity.Linear.X; d[o + 8] = r.Velocity.Linear.Y; d[o + 9] = r.Velocity.Linear.Z;
                d[o + 10] = r.Velocity.Angular.X; d[o + 11] = r.Velocity.Angular.Y; d[o + 12] = r.Velocity.Angular.Z; d[o + 13] = r.Awake ? 1 : 0;
            }
        }
    }
}
