using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// The 3D scenes of surface contact and tearing: a vertical cloth that topples onto a floor and folds over itself (cloth-fold-3d), and a cloth and a tetrahedral bar torn by kinematic hooks (tear-3d).
// Built in code (no files), shared by the tests, the hash-matrix runner and the stability command of the bar runner.
internal static class SoftFracture3DScene
{
    internal const float Dt = 1f / 60f;

    internal static readonly string[] Names = { "cloth-fold-3d", "tear-3d" };

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<PhysicsWorld3D, List<Body>> Hooks = new();

    internal static PhysicsWorld3D Build(string name, int threads, bool selfCollision = true, int worldId = 77)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = worldId });
        w.AddBox(new Vector3F(30f, 1f, 30f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        switch (name)
        {
            case "cloth-fold-3d": Fold(w, selfCollision); break;
            case "tear-3d": Tear(w); break;
            default: throw new ArgumentException("unknown scene " + name, nameof(name));
        }

        return w;
    }

    // A 2.4 m cloth of 24 x 24 cells stood on end 0.05 m above the floor with a push to the side: it topples and crumples onto itself.
    private static void Fold(PhysicsWorld3D w, bool selfCollision)
    {
        var def = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(2.4f, 2.4f, 24, 24), new Vector3D(0, 1.25, 0));
        def.Mass = 1.5f; def.SelfCollision = selfCollision; def.SurfaceCollision = selfCollision;
        def.Rotation = new QuaternionF(0.7071068f, 0f, 0f, 0.7071068f);
        def.LinearVelocity = new Vector3F(0.6f, 0f, 0.3f);
        w.AddSoftBody(def);
    }

    // A cloth hung by its top row and a jelly bar pinned at both ends, each with a kinematic sphere that moves through it (the cloth) or drags it down (the bar) until the strain tears edges.
    private static void Tear(PhysicsWorld3D w)
    {
        var cloth = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(1.6f, 1.2f, 16, 12), new Vector3D(-1.2, 1.6, 0));
        cloth.Mass = 0.6f; cloth.EdgeCompliance = 2e-4f; cloth.TearStrain = 0.2f; cloth.SelfCollision = true; cloth.SurfaceCollision = true;
        cloth.Rotation = new QuaternionF(0.7071068f, 0f, 0f, 0.7071068f);
        var top = new List<int>();
        for (int i = 0; i <= 16; i++) { top.Add(12 * 17 + i); }
        cloth.PinnedParticles = top;
        w.AddSoftBody(cloth);
        var hook = w.AddSphere(0.15f, new Vector3D(-1.2, 1.2, -0.9), type: BodyType.Kinematic);
        hook.LinearVelocity = new Vector3F(0f, 0f, 1.2f);
        var hooks = Hooks.GetOrCreateValue(w); hooks.Add(hook);

        var bar = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(1.4f, 0.2f, 0.2f), 14, 2, 2), new Vector3D(1.4, 1.2, 0));
        bar.Mass = 2f; bar.EdgeCompliance = 2e-4f; bar.VolumeCompliance = 1e-6f; bar.TearStrain = 0.15f; bar.PlasticYield = 0.06f; bar.SurfaceCollision = true;
        var ends = new List<int>();
        for (int k = 0; k <= 2; k++) { for (int j = 0; j <= 2; j++) { ends.Add((k * 3 + j) * 15); ends.Add((k * 3 + j) * 15 + 14); } }
        bar.PinnedParticles = ends;
        w.AddSoftBody(bar);
        var hook2 = w.AddSphere(0.12f, new Vector3D(1.4, 1.75, 0), type: BodyType.Kinematic);
        hook2.LinearVelocity = new Vector3F(0f, -1.0f, 0f);
        hooks.Add(hook2);
    }

    // The hooks stop after 240 steps.
    internal static void Drive(PhysicsWorld3D w, string name, int step)
    {
        if (name == "tear-3d" && step == 240 && Hooks.TryGetValue(w, out var list))
        {
            foreach (var b in list) { b.LinearVelocity = default; }
        }
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
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 77 });
                w.Restore(snap);
            }

            Drive(w, name, step);
            w.Step(Dt);
            hash = (hash ^ w.ComputeHash()) * 1099511628211UL;
            perStep?.Invoke(hash);
        }

        return hash;
    }

    internal readonly record struct Measurement(string Scene, int Steps, int Particles, double MaxSpeed, int Intersections, int MaxIntersections, int Tears, int Escapes, int NaN, int Asleep, int Bodies);

    /// <summary>Runs a scene for 600 steps and measures it: the fastest particle, the triangle pairs that intersect (counted every 10th step, pairs that share a vertex excluded), the edges that tore, particles outside the yard, and non-finite values.</summary>
    internal static Measurement Measure(string name, int steps = 600, int threads = 1, bool selfCollision = true)
    {
        var w = Build(name, threads, selfCollision);
        double maxSpeed = 0; int maxInter = 0, inter = 0, tears = 0, escapes = 0, nan = 0, particles = 0;
        w.SoftBodyTorn += _ => tears++;
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
                    if (Math.Abs(pos[3 * i]) > 15 || Math.Abs(pos[3 * i + 2]) > 15 || pos[3 * i + 1] < -0.5 || pos[3 * i + 1] > 20) { escapes++; }
                }

                if (step % 10 == 9 && name == "cloth-fold-3d") { inter = TriangleIntersections(b); maxInter = Math.Max(maxInter, inter); }
            }
        }

        int asleep = 0;
        for (int s = 0; s < w.SoftBodyCount; s++) { if (w.GetSoftBody(s).IsAsleep) { asleep++; } }
        return new Measurement(name, steps, particles, maxSpeed, inter, maxInter, tears, escapes, nan, asleep, w.SoftBodyCount);
    }

    /// <summary>The number of pairs of surface triangles of a body that cross each other (a segment of one through the other), not counting pairs that share a vertex. Brute force over a sweep along x.</summary>
    internal static int TriangleIntersections(SoftBody b)
    {
        var tris = b.Mesh.Triangles.ToArray();
        int nt = tris.Length / 3, n = b.ParticleCount;
        var pos = b.RawPositions.ToArray();
        var lo = new double[nt]; var hi = new double[nt]; var idx = new int[nt];
        for (int t = 0; t < nt; t++)
        {
            idx[t] = t;
            double a = double.MaxValue, c = double.MinValue;
            for (int k = 0; k < 3; k++) { double x = pos[3 * tris[3 * t + k]]; a = Math.Min(a, x); c = Math.Max(c, x); }
            lo[t] = a; hi[t] = c;
        }

        Array.Sort(idx, (p, q) => { int r = lo[p].CompareTo(lo[q]); return r != 0 ? r : p.CompareTo(q); });
        int count = 0;
        for (int ii = 0; ii < nt; ii++)
        {
            int t1 = idx[ii];
            for (int jj = ii + 1; jj < nt && lo[idx[jj]] <= hi[t1]; jj++)
            {
                int t2 = idx[jj];
                bool share = false;
                for (int a = 0; a < 3 && !share; a++) { for (int c = 0; c < 3; c++) { if (tris[3 * t1 + a] == tris[3 * t2 + c]) { share = true; break; } } }
                if (share) { continue; }
                if (Crosses(pos, tris, t1, t2) || Crosses(pos, tris, t2, t1)) { count++; }
            }
        }

        _ = n;
        return count;
    }

    private static bool Crosses(double[] pos, int[] tris, int e, int t)
    {
        for (int k = 0; k < 3; k++)
        {
            if (SegmentTriangle(pos, tris[3 * e + k], tris[3 * e + (k + 1) % 3], tris[3 * t], tris[3 * t + 1], tris[3 * t + 2])) { return true; }
        }

        return false;
    }

    // Moeller-Trumbore on the open segment.
    private static bool SegmentTriangle(double[] p, int i, int j, int a, int b, int c)
    {
        double dx = p[3 * j] - p[3 * i], dy = p[3 * j + 1] - p[3 * i + 1], dz = p[3 * j + 2] - p[3 * i + 2];
        double e1x = p[3 * b] - p[3 * a], e1y = p[3 * b + 1] - p[3 * a + 1], e1z = p[3 * b + 2] - p[3 * a + 2];
        double e2x = p[3 * c] - p[3 * a], e2y = p[3 * c + 1] - p[3 * a + 1], e2z = p[3 * c + 2] - p[3 * a + 2];
        double hx = dy * e2z - dz * e2y, hy = dz * e2x - dx * e2z, hz = dx * e2y - dy * e2x;
        double det = e1x * hx + e1y * hy + e1z * hz;
        if (Math.Abs(det) < 1e-14) { return false; }
        double inv = 1.0 / det;
        double sx = p[3 * i] - p[3 * a], sy = p[3 * i + 1] - p[3 * a + 1], sz = p[3 * i + 2] - p[3 * a + 2];
        double u = inv * (sx * hx + sy * hy + sz * hz);
        if (u < 0 || u > 1) { return false; }
        double qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
        double v = inv * (dx * qx + dy * qy + dz * qz);
        if (v < 0 || u + v > 1) { return false; }
        double t = inv * (e2x * qx + e2y * qy + e2z * qz);
        return t > 0 && t < 1;
    }

    internal static string Format(Measurement m) => string.Create(CultureInfo.InvariantCulture,
        $"{m.Scene} steps={m.Steps} bodies={m.Bodies} particles={m.Particles} maxSpeed={m.MaxSpeed:R} maxIntersections={m.MaxIntersections} finalIntersections={m.Intersections} tears={m.Tears} asleep={m.Asleep} escapes={m.Escapes} nan={m.NaN}");
}
