using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>What a run of <see cref="TetDropScene"/> measured; thresholds are the scene's own.</summary>
public readonly record struct TetDropMetrics(double Nan, double Escapes, double MaxSpeed, int InvertedTetrahedra, int FinalInverted, double MinVolumeRatio, double MaxVolumeRatio, double LowestPoint)
{
    public bool Passes => Nan == 0 && Escapes == 0 && MaxSpeed <= 25 && InvertedTetrahedra <= 40 && FinalInverted <= 10 && MinVolumeRatio >= 0.5 && MaxVolumeRatio <= 1.5 && LowestPoint >= -0.05;
}

/// <summary>
/// <c>tet-drop-3d</c>: soft bodies built from tetrahedralized meshes (a character-like blob, a torus and a sphere, each made by <see cref="Tetrahedralizer"/> from its triangle surface) dropped onto a floor for 600 steps at 60 Hz.
/// Own thresholds (no other engine tetrahedralizes a mesh): no NaN, nothing escaping, top speed at most 25 m/s, at most 40 tetrahedra (of 10,000) ever turned inside out during the impact and at most 10 at the end, volume between half and one and a half times the rest volume, no particle more than 5 cm below the floor.
/// </summary>
public static class TetDropScene
{
    public static TetrahedralMesh[] Meshes(int threads = 1)
    {
        var blob = TetBlob.Build(40, 48, 0.45f);
        var torus = TestMeshes.Torus(0.5f, 0.2f, 32, 16);
        var sphere = TestMeshes.Sphere(0.4f, 24, 32);
        var result = new TetrahedralMesh[3];
        var inputs = new[] { blob, torus, sphere };
        for (int i = 0; i < 3; i++) { result[i] = Tetrahedralizer.Generate(inputs[i].V, inputs[i].I, new TetrahedralizeOptions { TargetEdgeLength = i == 0 ? 0.14f : 0.1f, MaxThreads = threads }); }
        return result;
    }

    public static PhysicsWorld3D Build(out SoftBody[] bodies, TetrahedralMesh[]? meshes = null, int threads = 1)
    {
        meshes ??= Meshes(threads);
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 95 });
        w.AddBox(new Vector3F(30f, 1f, 30f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        bodies = new SoftBody[meshes.Length];
        for (int i = 0; i < meshes.Length; i++)
        {
            // Each body starts 0.3 m above the floor.
            double lowest = double.MaxValue;
            foreach (var v in meshes[i].Vertices) { lowest = Math.Min(lowest, v.Y); }
            bodies[i] = w.AddSoftBody(meshes[i].ToSoftBodyMesh(), new Vector3D(i * 1.6 - 1.6, 0.3 - lowest, 0), SoftMaterial.Rubber);
        }
        return w;
    }

    public static int CountInverted(SoftBody body, Span<Vector3D> scratch, out double minVolumeRatio)
    {
        body.CopyPositions(scratch);
        var mesh = body.Mesh;
        var tets = mesh.Tetrahedra;
        int inverted = 0; minVolumeRatio = double.MaxValue;
        for (int t = 0; t < tets.Length / 4; t++)
        {
            var a = scratch[tets[4 * t]]; var b = scratch[tets[4 * t + 1]]; var c = scratch[tets[4 * t + 2]]; var d = scratch[tets[4 * t + 3]];
            double ux = b.X - a.X, uy = b.Y - a.Y, uz = b.Z - a.Z, vx = c.X - a.X, vy = c.Y - a.Y, vz = c.Z - a.Z, wx = d.X - a.X, wy = d.Y - a.Y, wz = d.Z - a.Z;
            double vol = wx * (uy * vz - uz * vy) + wy * (uz * vx - ux * vz) + wz * (ux * vy - uy * vx);
            if (!(vol > 0)) { inverted++; }
        }

        return inverted;
    }

    public static TetDropMetrics Run(int steps = 600, int threads = 1)
    {
        var w = Build(out var bodies, null, threads);
        double maxSpeed = 0, nan = 0, escapes = 0, lowest = double.MaxValue, minRatio = double.MaxValue, maxRatio = 0;
        int inverted = 0, finalInverted = 0;
        var scratch = new Vector3D[bodies[0].ParticleCount > 0 ? bodies.Max(b => b.ParticleCount) : 1];
        for (int s = 0; s < steps; s++)
        {
            w.Step(1f / 60f);
            foreach (var b in bodies)
            {
                int n = b.ParticleCount;
                for (int i = 0; i < n; i++)
                {
                    var p = b.GetPosition(i); var v = b.GetVelocity(i);
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                    if (p.Y > 20 || Math.Abs(p.X) > 15 || Math.Abs(p.Z) > 15) { escapes++; }
                    lowest = Math.Min(lowest, p.Y);
                }

                if (s % 10 == 9) { inverted = Math.Max(inverted, CountInverted(b, scratch.AsSpan(0, n), out _)); }
                if (s == steps - 1) { finalInverted += CountInverted(b, scratch.AsSpan(0, n), out _); }
                double ratio = b.Volume / b.RestVolume;
                minRatio = Math.Min(minRatio, ratio); maxRatio = Math.Max(maxRatio, ratio);
            }
        }

        return new TetDropMetrics(nan, escapes, maxSpeed, inverted, finalInverted, minRatio, maxRatio, lowest);
    }
}

/// <summary>What a run of <see cref="TriDropScene2D"/> measured.</summary>
public readonly record struct TriDropMetrics(double Nan, double Escapes, double MaxSpeed, int InvertedTriangles, int FinalInverted, double MinAreaRatio, double MaxAreaRatio, double LowestPoint, double MinAngle)
{
    public bool Passes => Nan == 0 && Escapes == 0 && MaxSpeed <= 25 && InvertedTriangles <= 20 && FinalInverted <= 12 && MinAreaRatio >= 0.5 && MaxAreaRatio <= 1.5 && LowestPoint >= -0.05 && MinAngle >= 24.5;
}

/// <summary>
/// <c>tri-drop-2d</c>: 2D soft bodies built by <see cref="TriangleMesher"/> from polygons with holes (an annulus, a square with two holes, a star) dropped on a floor for 600 steps. Own thresholds: no NaN, nothing escaping,
/// top speed at most 25 m/s, at most 20 triangles (of about 700) ever turned inside out during the impact and at most 12 at the end, area between half and one and a half times the rest area, no particle more than 5 cm below the floor, smallest mesh angle at least 24.5 degrees.
/// </summary>
public static class TriDropScene2D
{
    public static (Vector2F[] Outline, Vector2F[][]? Holes)[] Shapes()
    {
        Vector2F[] Circle(float r, int n, bool ccw, float cx = 0, float cy = 0)
        {
            var p = new Vector2F[n];
            for (int i = 0; i < n; i++) { double a = 2 * Math.PI * i / n * (ccw ? 1 : -1); p[i] = new Vector2F(cx + (float)(r * Math64.Cos(a)), cy + (float)(r * Math64.Sin(a))); }
            return p;
        }

        var star = new Vector2F[10];
        for (int i = 0; i < 10; i++) { double a = Math.PI * i / 5; double r = (i & 1) == 0 ? 0.6 : 0.35; star[i] = new Vector2F((float)(r * Math64.Cos(a)), (float)(r * Math64.Sin(a))); }
        return new (Vector2F[], Vector2F[][]?)[]
        {
            (Circle(0.5f, 40, true), new[] { Circle(0.2f, 20, false) }),
            (new[] { new Vector2F(-0.5f, -0.4f), new Vector2F(0.5f, -0.4f), new Vector2F(0.5f, 0.4f), new Vector2F(-0.5f, 0.4f) }, new[] { Circle(0.12f, 12, false, -0.25f, 0f), Circle(0.12f, 12, false, 0.25f, 0f) }),
            (star, null),
        };
    }

    public static TriangleMeshResult[] Meshes() => Shapes().Select(s => TriangleMesher.Generate(s.Outline, s.Holes, new TriangleMeshOptions { TargetEdgeLength = 0.07f })).ToArray();

    public static RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D Build(out RecursiveAI.OnePhysics.Physics2D.SoftBody[] bodies, TriangleMeshResult[]? meshes = null)
    {
        meshes ??= Meshes();
        var w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { WorldId = 96 });
        w.AddBox(new Vector2F(30f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        bodies = new RecursiveAI.OnePhysics.Physics2D.SoftBody[meshes.Length];
        for (int i = 0; i < meshes.Length; i++)
        {
            double lowest = double.MaxValue;
            foreach (var v in meshes[i].Vertices) { lowest = Math.Min(lowest, v.Y); }
            bodies[i] = w.AddSoftBody(meshes[i].ToSoftBodyMesh(), new Vector2D(i * 1.5 - 1.5, 0.3 - lowest), RecursiveAI.OnePhysics.Physics2D.SoftMaterial.Rubber);
        }

        return w;
    }

    public static TriDropMetrics Run(int steps = 600)
    {
        var meshes = Meshes();
        var w = Build(out var bodies, meshes);
        double maxSpeed = 0, nan = 0, escapes = 0, lowest = double.MaxValue, minRatio = double.MaxValue, maxRatio = 0, minAngle = double.MaxValue;
        foreach (var m in meshes) { minAngle = Math.Min(minAngle, m.MinAngleDegrees); }
        var restArea = meshes.Select(m => m.Area).ToArray();
        int inverted = 0, finalInverted = 0;
        var scratch = new Vector2D[bodies.Max(b => b.ParticleCount)];
        for (int s = 0; s < steps; s++)
        {
            w.Step(1f / 60f);
            foreach (var b in bodies)
            {
                int n = b.ParticleCount;
                for (int i = 0; i < n; i++)
                {
                    var p = b.GetPosition(i); var v = b.GetVelocity(i);
                    if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y));
                    if (p.Y > 20 || Math.Abs(p.X) > 15) { escapes++; }
                    lowest = Math.Min(lowest, p.Y);
                }

                if (s % 10 == 9 || s == steps - 1)
                {
                    b.CopyPositions(scratch.AsSpan(0, n));
                    var tri = b.Mesh.Triangles;
                    int bad = 0; double area = 0;
                    for (int t = 0; t < tri.Length / 3; t++)
                    {
                        var a = scratch[tri[3 * t]]; var bb = scratch[tri[3 * t + 1]]; var c = scratch[tri[3 * t + 2]];
                        double twice = (bb.X - a.X) * (c.Y - a.Y) - (bb.Y - a.Y) * (c.X - a.X);
                        if (!(twice > 0)) { bad++; }
                        area += 0.5 * twice;
                    }

                    inverted = Math.Max(inverted, bad);
                    if (s == steps - 1) { finalInverted += bad; }
                    double ratio = area / restArea[Array.IndexOf(bodies, b)];
                    minRatio = Math.Min(minRatio, ratio); maxRatio = Math.Max(maxRatio, ratio);
                }
            }
        }

        return new TriDropMetrics(nan, escapes, maxSpeed, inverted, finalInverted, minRatio, maxRatio, lowest, minAngle);
    }
}
