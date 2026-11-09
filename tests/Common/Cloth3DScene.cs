using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Physics3D;

// The 3D fabric scenes: a flag in steady wind on a pole (cloth-flag-3d), a cape draped on a walking kinematic capsule (cloth-character-3d), a sheet thrown at 20 m/s into a pole (cloth-throw-3d) and a garment made of two
// stitched panels hung by tethers on a cylinder (cloth-garment-3d). Built in code (no files), shared by the tests, the hash-matrix runner and the stability command of the bar runner.
internal static class Cloth3DScene
{
    internal const float Dt = 1f / 60f;

    internal static readonly string[] Names = { "cloth-flag-3d", "cloth-character-3d", "cloth-throw-3d", "cloth-garment-3d" };

    private static readonly QuaternionF Upright = new(0.7071068f, 0f, 0f, 0.7071068f);   // the sheet stands up: local x stays x, local z becomes -y, the face looks along +z

    private sealed class Extras
    {
        public BodyHandle Walker;
        public int Front = -1, Back = -1;
    }

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<PhysicsWorld3D, Extras> Parts = new();

    internal static PhysicsWorld3D Build(string name, int threads, int worldId = 78)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = worldId });
        w.AddBox(new Vector3F(40f, 1f, 40f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var ex = Parts.GetOrCreateValue(w);
        switch (name)
        {
            case "cloth-flag-3d": Flag(w); break;
            case "cloth-character-3d": Cape(w, ex); break;
            case "cloth-throw-3d": Throw(w); break;
            case "cloth-garment-3d": Garment(w, ex); break;
            default: throw new ArgumentException("unknown scene " + name, nameof(name));
        }

        return w;
    }

    // 1.8 x 1.2 m of cotton on a 3 m pole, pinned along its left edge, in a steady 6 m/s wind along +x with a 1.5 m/s cross component.
    private static void Flag(PhysicsWorld3D w)
    {
        w.AddCylinder(1.5f, 0.03f, new Vector3D(0, 1.5, 0), type: BodyType.Static);
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-20, -1, -20), Maximum = new Vector3D(20, 10, 20), Flow = new Vector3F(6f, 0f, 1.5f) });
        var def = new ClothDef { Width = 1.8f, Depth = 1.2f, SegmentsX = 30, SegmentsZ = 20, Position = new Vector3D(0.93, 2.6, 0), Rotation = Upright, Material = ClothMaterial.Cotton };
        def.PinnedParticles = ClothDef.PinEdge(30, 20, 3);
        w.AddCloth(def);
    }

    // A 1.6 x 1.6 m cape dropped over a standing capsule (radius 0.3, 1.6 m tall) that walks off at 1.4 m/s from step 120 to 360.
    private static void Cape(PhysicsWorld3D w, Extras ex)
    {
        ex.Walker = w.AddCapsule(0.5f, 0.3f, new Vector3D(0, 1.1, 0), type: BodyType.Kinematic).RawHandle;
        var def = new ClothDef { Width = 1.6f, Depth = 1.6f, SegmentsX = 24, SegmentsZ = 24, Position = new Vector3D(0, 2.4, 0), Material = ClothMaterial.Cotton };
        w.AddCloth(def);
    }

    // A 1.2 x 1.2 m sheet standing up, thrown at 20 m/s along +x into a 0.2 m pole.
    private static void Throw(PhysicsWorld3D w)
    {
        w.AddCylinder(1.5f, 0.1f, new Vector3D(3, 1.5, 0), type: BodyType.Static);
        var def = new ClothDef { Width = 1.2f, Depth = 1.2f, SegmentsX = 20, SegmentsZ = 20, Position = new Vector3D(-3, 1.4, 0), Rotation = new QuaternionF(0f, 0f, 0.7071068f, 0.7071068f), Material = DenseCloth() };
        def.LinearVelocity = new Vector3F(20f, 0f, 0f);
        w.AddCloth(def);
    }

    // A wet cloth: air drag would stop a light sheet thrown at 20 m/s within a few steps, so the throw uses a sheet that does not feel it.
    private static ClothMaterial DenseCloth() { var m = ClothMaterial.Cotton; m.Drag = 0f; m.ArealDensity = 0.5f; return m; }

    // Two 0.6 x 0.8 m panels laid half way round a 0.2 m cylinder, front and back, stitched along both sides and hung by their top edges with tethers.
    private static void Garment(PhysicsWorld3D w, Extras ex)
    {
        w.AddCylinder(0.6f, 0.2f, new Vector3D(0, 1.6, 0), type: BodyType.Static);
        var top = ClothDef.PinEdge(10, 10, 0);
        SoftBody Panel(float side)
        {
            var def = new ClothDef { Width = 0.6f, Depth = 0.8f, SegmentsX = 10, SegmentsZ = 10, Position = new Vector3D(0, 1.8, 0), Rotation = Upright, Material = ClothMaterial.Denim, PinnedParticles = top, Tethers = true, TetherStrain = 0.02f };
            var sd = def.ToSoftBodyDef();
            // the flat rest sheet starts curved round the cylinder: arc length along x is kept, the face looks outwards
            const double r = 0.215;
            var start = new Vector3F[sd.Mesh!.VertexCount];
            for (int i = 0; i < start.Length; i++)
            {
                var v = sd.Mesh.Vertices[i];
                double th = v.X / r;
                start[i] = new Vector3F((float)(r * Math.Sin(th)), (float)(side * r * Math.Cos(th)), v.Z);
            }

            sd.StartVertices = start;
            return w.AddSoftBody(sd);
        }

        var front = Panel(1f);
        var back = Panel(-1f);
        ex.Front = front.Slot; ex.Back = back.Slot;
        w.AddSeam(front, ClothDef.PinEdge(10, 10, 1), back, ClothDef.PinEdge(10, 10, 1), 1e-4f);
        w.AddSeam(front, ClothDef.PinEdge(10, 10, 3), back, ClothDef.PinEdge(10, 10, 3), 1e-4f);
    }

    internal static void Drive(PhysicsWorld3D w, string name, int step)
    {
        if (name != "cloth-character-3d" || !Parts.TryGetValue(w, out var ex)) { return; }
        if (step == 120) { w.GetBody(ex.Walker).LinearVelocity = new Vector3F(1.4f, 0f, 0f); }
        if (step == 360) { w.GetBody(ex.Walker).LinearVelocity = default; }
    }

    internal static ulong Run(string name, int threads, bool restoreHalfway, Action<ulong>? perStep = null, int steps = 150)
    {
        var w = Build(name, threads);
        ulong hash = 14695981039346656037UL;
        for (int step = 0; step < steps; step++)
        {
            if (step == steps / 2 && restoreHalfway)
            {
                var snap = w.Snapshot();
                var extras = Parts.TryGetValue(w, out var e0) ? e0 : null;
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 78 });
                w.Restore(snap);
                if (extras != null) { Parts.Add(w, extras); }
            }

            Drive(w, name, step);
            w.Step(Dt);
            hash = (hash ^ w.ComputeHash()) * 1099511628211UL;
            perStep?.Invoke(hash);
        }

        return hash;
    }

    internal readonly record struct Measurement(string Scene, int Steps, int Particles, double MaxSpeed, int MaxIntersections, double MaxPenetration, double MaxSeamGap, double MaxStrain, int Escapes, int NaN);

    /// <summary>Runs a scene for 600 steps and measures it: the fastest particle, the triangle pairs of one body that cross (every 10th step), the deepest a particle sits inside the capsule or cylinder it drapes on (the surface, in metres),
    /// the widest gap of a seam, the largest edge strain, particles that left the yard and non-finite values.</summary>
    internal static Measurement Measure(string name, int steps = 600, int threads = 1)
    {
        var w = Build(name, threads);
        Parts.TryGetValue(w, out var ex);
        double maxSpeed = 0, maxPen = 0, maxGap = 0, maxStrain = 0;
        int maxInter = 0, escapes = 0, nan = 0, particles = 0;
        for (int step = 0; step < steps; step++)
        {
            Drive(w, name, step);
            w.Step(Dt);
            for (int s = 0; s < w.SoftBodyCount; s++)
            {
                var b = w.GetSoftBody(s);
                var pos = b.RawPositions; var vel = b.RawVelocities;
                if (step == 0) { particles += b.ParticleCount; }
                for (int i = 0; i < b.ParticleCount; i++)
                {
                    double sp = Math.Sqrt((double)vel[3 * i] * vel[3 * i] + (double)vel[3 * i + 1] * vel[3 * i + 1] + (double)vel[3 * i + 2] * vel[3 * i + 2]);
                    if (!double.IsFinite(sp) || !double.IsFinite(pos[3 * i]) || !double.IsFinite(pos[3 * i + 1]) || !double.IsFinite(pos[3 * i + 2])) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, sp);
                    if (Math.Abs(pos[3 * i]) > 18 || Math.Abs(pos[3 * i + 2]) > 18 || pos[3 * i + 1] < -0.5 || pos[3 * i + 1] > 12) { escapes++; }
                    maxPen = Math.Max(maxPen, Penetration(w, name, ex, pos[3 * i], pos[3 * i + 1], pos[3 * i + 2], step));
                }

                if (step % 10 == 9) { maxInter = Math.Max(maxInter, SoftFracture3DScene.TriangleIntersections(b)); }
                if (step % 10 == 9) { maxStrain = Math.Max(maxStrain, Strain(b)); }
            }

            if (name == "cloth-garment-3d" && ex != null && step >= 300) { maxGap = Math.Max(maxGap, SeamGap(w, ex)); }
        }

        return new Measurement(name, steps, particles, maxSpeed, maxInter, maxPen, maxGap, maxStrain, escapes, nan);
    }

    // How deep a particle centre is inside the object the scene drapes on (the capsule of the walker, the cylinder of the garment); 0 when it is outside or the scene has none.
    private static double Penetration(PhysicsWorld3D w, string name, Extras? ex, double x, double y, double z, int step)
    {
        if (name == "cloth-character-3d" && ex != null)
        {
            var p = w.GetBody(ex.Walker).Position;
            // the capsule stands along y: half length 0.5 and radius 0.3 about its centre
            double cy = Math.Clamp(y, p.Y - 0.5, p.Y + 0.5);
            double dx = x - p.X, dy = y - cy, dz = z - p.Z;
            return Math.Max(0.0, 0.3 - Math.Sqrt(dx * dx + dy * dy + dz * dz));
        }

        if (name == "cloth-garment-3d")
        {
            if (y < 1.0 || y > 2.2) { return 0.0; }
            return Math.Max(0.0, 0.2 - Math.Sqrt(x * x + z * z));
        }

        return 0.0;
    }

    private static double Strain(SoftBody b)
    {
        var pos = b.RawPositions;
        var mesh = b.Mesh;
        var tris = mesh.Triangles;
        double worst = 0.0;
        for (int t = 0; t < tris.Length / 3; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                int a = tris[3 * t + k], c = tris[3 * t + (k + 1) % 3];
                double rx = (double)mesh.Vertices[a].X - mesh.Vertices[c].X, ry = (double)mesh.Vertices[a].Y - mesh.Vertices[c].Y, rz = (double)mesh.Vertices[a].Z - mesh.Vertices[c].Z;
                double dx = pos[3 * a] - pos[3 * c], dy = pos[3 * a + 1] - pos[3 * c + 1], dz = pos[3 * a + 2] - pos[3 * c + 2];
                worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy + dz * dz) / Math.Sqrt(rx * rx + ry * ry + rz * rz) - 1.0);
            }
        }

        return worst;
    }

    private static double SeamGap(PhysicsWorld3D w, Extras ex)
    {
        double worst = 0.0;
        var front = w.GetSoftBody(ex.Front); var back = w.GetSoftBody(ex.Back);
        foreach (int edge in new[] { 1, 3 })
        {
            var e = ClothDef.PinEdge(10, 10, edge);
            // the top row is pinned on both panels and stays where it was hung: only the seam below it is measured
            for (int i = 0; i < e.Length; i++) { if (e[i] > 10) { worst = Math.Max(worst, (front.GetPosition(e[i]) - back.GetPosition(e[i])).Length()); } }
        }

        return worst;
    }

    internal static string Format(Measurement m) => string.Create(CultureInfo.InvariantCulture,
        $"{m.Scene} steps={m.Steps} particles={m.Particles} maxSpeed={m.MaxSpeed:R} maxIntersections={m.MaxIntersections} maxPenetration={m.MaxPenetration:R} maxSeamGap={m.MaxSeamGap:R} maxStrain={m.MaxStrain:R} escapes={m.Escapes} nan={m.NaN}");
}
