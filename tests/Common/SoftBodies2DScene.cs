using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

// The 2D soft-body scenes: a box full of mixed soft bodies, soft bodies under and on rigid stacks, and balloons pressed into terrain by a heavy slab.
// They are built in code (a seeded generator, no files), shared by the tests, the hash-matrix runner, the stability command of the bar runner and the smoke apps.
internal static class SoftBodies2DScene
{
    internal const float Dt = 1f / 60f;

    internal static readonly string[] Names = { "soft-pile-2d", "soft-rigid-2d", "balloon-2d" };

    // A small seeded generator so the scene is the same on every platform (System.Random is not specified to be).
    private sealed class Lcg
    {
        private ulong _s;
        public Lcg(ulong seed) { _s = seed; }
        public double Next() { _s = _s * 6364136223846793005UL + 1442695040888963407UL; return (double)(_s >> 11) / 9007199254740992.0; }
    }

    internal static PhysicsWorld2D Build(string name, int threads, int worldId = 91)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = worldId });
        switch (name)
        {
            case "soft-pile-2d": Pile(w); break;
            case "soft-rigid-2d": Rigid(w); break;
            case "balloon-2d": Balloons(w); break;
            default: throw new ArgumentException("unknown scene " + name, nameof(name));
        }

        return w;
    }

    private static void Container(PhysicsWorld2D w, double half, double height)
    {
        w.AddBox(new Vector2F((float)(2 * half + 2), 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, (float)height), at: new Vector2D(-half - 0.5, height / 2), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, (float)height), at: new Vector2D(half + 0.5, height / 2), type: BodyType.Static);
    }

    // Soft bodies of a given areal density (kg per square metre of rest area): the library default of 1000 is a slab of water, which is far heavier than the rigid default of 1.
    private static void Add(PhysicsWorld2D w, SoftBodyDef def, double area, float density) { def.Mass = (float)(density * area); w.AddSoftBody(def); }

    private static void AddBox(PhysicsWorld2D w, Vector2F size, Vector2D at, SoftMaterial m, int cells, float density)
    {
        var mesh = SoftBodyMesh.Box(size, cells, cells);
        Add(w, m == SoftMaterial.Rubber ? SoftBodyDef.Rubber(mesh, at) : SoftBodyDef.Jelly(mesh, at), size.X * size.Y, density);
    }

    private static void AddDisc(PhysicsWorld2D w, float radius, Vector2D at, SoftMaterial m, int rings, float density)
    {
        var mesh = SoftBodyMesh.Disc(radius, rings, Math.Max(6, 6 * rings));
        Add(w, m == SoftMaterial.Rubber ? SoftBodyDef.Rubber(mesh, at) : SoftBodyDef.Jelly(mesh, at), Math.PI * radius * radius, density);
    }

    private static void AddBalloon(PhysicsWorld2D w, float radius, Vector2D at, int segments, float density)
        => Add(w, SoftBodyDef.Balloon(SoftBodyMesh.Ring(radius, segments), at), Math.PI * radius * radius, density);

    private static void AddBlob(PhysicsWorld2D w, float radius, Vector2D at, int rings, float density)
        => Add(w, SoftBodyDef.Blob(SoftBodyMesh.Disc(radius, rings, Math.Max(6, 6 * rings)), at), Math.PI * radius * radius, density);

    private static void Pile(PhysicsWorld2D w)
    {
        Container(w, 6.0, 40.0);
        var rng = new Lcg(20260108);
        // 120 bodies of five kinds dropped in a loose grid, 15 columns by 8 rows, 20 kg per square metre.
        for (int i = 0; i < 120; i++)
        {
            int col = i % 15, row = i / 15;
            var at = new Vector2D(-5.6 + col * 0.8 + (rng.Next() - 0.5) * 0.1, 1.0 + row * 1.0 + (rng.Next() - 0.5) * 0.1);
            switch (i % 5)
            {
                case 0: AddBox(w, new Vector2F(0.6f, 0.6f), at, SoftMaterial.Jelly, 3, 20f); break;
                case 1: AddDisc(w, 0.3f, at, SoftMaterial.Rubber, 2, 20f); break;
                case 2: AddBalloon(w, 0.3f, at, 14, 20f); break;
                case 3: AddBlob(w, 0.3f, at, 2, 20f); break;
                default: AddBox(w, new Vector2F(0.7f, 0.4f), at, SoftMaterial.Rubber, 2, 20f); break;
            }
        }
    }

    private static void Rigid(PhysicsWorld2D w)
    {
        Container(w, 8.0, 40.0);
        // A rigid stack with soft bodies on top, and a heavy rigid slab pressed down on soft cushions.
        for (int row = 0; row < 5; row++) { for (int i = 0; i < 5 - row; i++) { w.AddBox(new Vector2F(0.8f, 0.5f), at: new Vector2D(-5 + row * 0.4 + i * 0.82, 0.25 + row * 0.5), density: 200f); } }
        AddBox(w, new Vector2F(0.8f, 0.8f), new Vector2D(-4.0, 3.2), SoftMaterial.Jelly, 3, 20f);
        AddDisc(w, 0.4f, new Vector2D(-3.0, 3.6), SoftMaterial.Rubber, 3, 20f);
        AddBalloon(w, 0.4f, new Vector2D(-2.2, 4.2), 18, 20f);
        for (int i = 0; i < 3; i++) { AddBox(w, new Vector2F(0.9f, 0.5f), new Vector2D(3.0 + i * 1.0, 0.3), SoftMaterial.Jelly, 2, 20f); }
        w.AddBox(new Vector2F(3.6f, 0.4f), new Vector2D(4.0, 1.4), density: 20f);
        w.AddBox(new Vector2F(0.6f, 0.6f), new Vector2D(4.0, 2.4), density: 20f);
        for (int i = 0; i < 4; i++) { w.AddCircle(0.25f, new Vector2D(0.5 + 0.3 * i, 3 + i * 0.6), density: 40f); }
    }

    private static void Balloons(PhysicsWorld2D w)
    {
        // Terrain: a height field valley with a flat bed, and a heavy slab dropped on balloons in the valley.
        var h = new float[41];
        for (int i = 0; i < h.Length; i++) { double x = (i - 20) * 0.5; h[i] = (float)(Math.Min(4.0, 0.15 * x * x)); }
        w.AddHeightField(h, 0.5f, new Vector2D(-10, 0));
        w.AddBox(new Vector2F(1f, 30f), at: new Vector2D(-10.5, 10), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 30f), at: new Vector2D(10.5, 10), type: BodyType.Static);
        for (int i = 0; i < 8; i++) { AddBalloon(w, 0.45f, new Vector2D(-3.0 + 0.9 * (i % 4), 1.6 + 1.0 * (i / 4)), 20, 10f); }
        AddBalloon(w, 0.8f, new Vector2D(2.8, 2.2), 28, 10f);
        w.AddBox(new Vector2F(3f, 0.3f), new Vector2D(0, 6.5), density: 20f);
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
                w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 91 });
                w.Restore(snap);
            }

            w.Step(Dt);
            hash = (hash ^ w.ComputeHash()) * 1099511628211UL;
            perStep?.Invoke(hash);
        }

        return hash;
    }

    internal readonly record struct Measurement(string Scene, int Steps, int Bodies, int Particles, double MaxSpeed, double MaxPenetration, double AreaError, double MaxSoftOverlap, int Escapes, int NaN, double FinalSpeed99);

    /// <summary>Runs a scene and measures it: fastest particle, deepest particle into a rigid shape (disc radius minus distance, from the world's closest-point query), worst relative area error of the pressure bodies, deepest overlap of particles of
    /// different bodies at every 100th step, particles outside the container, and non-finite values. The speed window is the last 3 steps.</summary>
    internal static Measurement Measure(string name, int steps = 600, int threads = 1)
    {
        var w = Build(name, threads);
        double maxSpeed = 0, maxPen = 0, areaErr = 0, softOverlap = 0;
        int escapes = 0, nan = 0, particles = 0, bodies = w.SoftBodyCount;
        var window = new System.Collections.Generic.List<double>();
        for (int step = 0; step < steps; step++)
        {
            w.Step(Dt);
            window.Clear();
            for (int s = 0; s < w.SoftBodyCount; s++)
            {
                var b = w.GetSoftBody(s);
                int n = b.ParticleCount;
                if (step == 0) { particles += n; }
                var pos = b.RawPositions; var vel = b.RawVelocities;
                for (int i = 0; i < n; i++)
                {
                    double vx = vel[2 * i], vy = vel[2 * i + 1];
                    double sp = Math.Sqrt(vx * vx + vy * vy);
                    if (!double.IsFinite(sp) || !double.IsFinite(pos[2 * i]) || !double.IsFinite(pos[2 * i + 1])) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, sp);
                    if (step >= steps - 3) { window.Add(sp); }
                    if (Math.Abs(pos[2 * i]) > 40 || pos[2 * i + 1] < -1 || pos[2 * i + 1] > 100) { escapes++; }
                    if (step % 10 == 0 && w.ClosestPoint(new Vector2D(pos[2 * i], pos[2 * i + 1]), out var hit, b.ParticleRadius))
                    {
                        maxPen = Math.Max(maxPen, b.ParticleRadius - hit.Distance);
                    }
                }

                if (b.Definition.InternalPressure > 0f && step >= steps - 3) { areaErr = Math.Max(areaErr, Math.Abs(b.Area / (b.Definition.InternalPressure * b.RestArea) - 1.0)); }
            }

            if (step % 100 == 99) { softOverlap = Math.Max(softOverlap, SoftOverlap(w)); }
        }

        window.Sort();
        double p99 = window.Count == 0 ? 0 : window[Math.Min(window.Count - 1, (int)(0.99 * window.Count))];
        return new Measurement(name, steps, bodies, particles, maxSpeed, maxPen, areaErr, softOverlap, escapes, nan, p99);
    }

    // Deepest overlap of two particle discs of different bodies, found with a sweep over x.
    private static double SoftOverlap(PhysicsWorld2D w)
    {
        var items = new System.Collections.Generic.List<(double X, double Y, float R, int Body)>();
        for (int s = 0; s < w.SoftBodyCount; s++)
        {
            var b = w.GetSoftBody(s);
            var pos = b.RawPositions;
            for (int i = 0; i < b.ParticleCount; i++) { items.Add((pos[2 * i], pos[2 * i + 1], b.ParticleRadius, s)); }
        }

        items.Sort((a, b) => a.X.CompareTo(b.X));
        double worst = 0;
        for (int i = 0; i < items.Count; i++)
        {
            for (int j = i + 1; j < items.Count && items[j].X - items[i].X < 1.0; j++)
            {
                if (items[i].Body == items[j].Body) { continue; }
                double dx = items[j].X - items[i].X, dy = items[j].Y - items[i].Y;
                double d = Math.Sqrt(dx * dx + dy * dy);
                worst = Math.Max(worst, items[i].R + items[j].R - d);
            }
        }

        return worst;
    }

    internal static string Format(Measurement m) => string.Create(CultureInfo.InvariantCulture,
        $"{m.Scene} steps={m.Steps} bodies={m.Bodies} particles={m.Particles} maxSpeed={m.MaxSpeed:R} maxPenetration={m.MaxPenetration:R} areaError={m.AreaError:R} maxSoftOverlap={m.MaxSoftOverlap:R} finalSpeed99={m.FinalSpeed99:R} escapes={m.Escapes} nan={m.NaN}");
}
