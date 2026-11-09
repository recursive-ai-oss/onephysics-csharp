using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Physics2D;

// strip-2d: a 1.2 m strip of fabric pinned at one end above a box, blown by a gusting upward wind, so it flutters, falls on the box and lifts again. Built in code, shared by the tests, the hash-matrix runner and the
// stability command of the bar runner.
internal static class Strip2DScene
{
    internal const float Dt = 1f / 60f;

    internal static readonly string[] Names = { "strip-2d" };

    internal static PhysicsWorld2D Build(int threads, int worldId = 79)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = worldId });
        w.AddBox(new Vector2F(30f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(0.4f, 0.8f), at: new Vector2D(1.2, 0.4), type: BodyType.Static);
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-20, 0.05, -1), Maximum = new Vector3D(20, 10, 1), Flow = new Vector3F(1f, 7f, 0f) });
        w.AddStrip(1.2f, 0.04f, 24, new Vector2D(0.6, 2.2), 0.08f, pinStart: true);
        w.AddStrip(0.8f, 0.04f, 16, new Vector2D(2.6, 1.6), 0.08f, pinStart: true);
        return w;
    }

    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null, int steps = 150)
    {
        var w = Build(threads);
        ulong hash = 14695981039346656037UL;
        for (int step = 0; step < steps; step++)
        {
            if (step == steps / 2 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 79 });
                w.Restore(snap);
            }

            w.Step(Dt);
            hash = (hash ^ w.ComputeHash()) * 1099511628211UL;
            perStep?.Invoke(hash);
        }

        return hash;
    }

    internal readonly record struct Measurement(string Scene, int Steps, int Particles, double MaxSpeed, double MaxStrain, int Escapes, int NaN, double MaxHeight);

    internal static Measurement Measure(int steps = 600, int threads = 1)
    {
        var w = Build(threads);
        double maxSpeed = 0, maxStrain = 0, maxHeight = 0;
        int escapes = 0, nan = 0, particles = 0;
        for (int step = 0; step < steps; step++)
        {
            w.Step(Dt);
            for (int s = 0; s < w.SoftBodyCount; s++)
            {
                var b = w.GetSoftBody(s);
                var pos = b.RawPositions; var vel = b.RawVelocities;
                if (step == 0) { particles += b.ParticleCount; }
                for (int i = 0; i < b.ParticleCount; i++)
                {
                    double sp = Math.Sqrt((double)vel[2 * i] * vel[2 * i] + (double)vel[2 * i + 1] * vel[2 * i + 1]);
                    if (!double.IsFinite(sp) || !double.IsFinite(pos[2 * i]) || !double.IsFinite(pos[2 * i + 1])) { nan++; continue; }
                    maxSpeed = Math.Max(maxSpeed, sp);
                    maxHeight = Math.Max(maxHeight, pos[2 * i + 1]);
                    if (Math.Abs(pos[2 * i]) > 15 || pos[2 * i + 1] < -0.5 || pos[2 * i + 1] > 12) { escapes++; }
                }

                if (step % 10 == 9)
                {
                    var mesh = b.Mesh; var tris = mesh.Triangles;
                    for (int t = 0; t < tris.Length / 3; t++)
                    {
                        for (int k = 0; k < 3; k++)
                        {
                            int a = tris[3 * t + k], c = tris[3 * t + (k + 1) % 3];
                            double rx = (double)mesh.Vertices[a].X - mesh.Vertices[c].X, ry = (double)mesh.Vertices[a].Y - mesh.Vertices[c].Y;
                            double dx = pos[2 * a] - pos[2 * c], dy = pos[2 * a + 1] - pos[2 * c + 1];
                            maxStrain = Math.Max(maxStrain, Math.Sqrt(dx * dx + dy * dy) / Math.Sqrt(rx * rx + ry * ry) - 1.0);
                        }
                    }
                }
            }
        }

        return new Measurement("strip-2d", steps, particles, maxSpeed, maxStrain, escapes, nan, maxHeight);
    }

    internal static string Format(Measurement m) => string.Create(CultureInfo.InvariantCulture,
        $"{m.Scene} steps={m.Steps} particles={m.Particles} maxSpeed={m.MaxSpeed:R} maxStrain={m.MaxStrain:R} maxHeight={m.MaxHeight:R} escapes={m.Escapes} nan={m.NaN}");
}
