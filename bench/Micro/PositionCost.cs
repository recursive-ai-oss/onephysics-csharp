using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;

namespace RecursiveAI.OnePhysics.Bench;

/// <summary>
/// What do double-precision body positions cost against a float32 store? Measured on the particle scene in two parts:
/// the memory the position column takes, and the time of the loops that touch positions (position integration and the
/// separation/direction computation of every constraint), each run over the same data stored as double and as float, next to the time of a whole step.
/// Usage: Micro --position-cost [bodies] [steps] [repeats]
/// </summary>
internal static class PositionCost
{
    public static int Run(string[] args)
    {
        int bodies = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : 5000;
        int steps = args.Length > 2 ? int.Parse(args[2], CultureInfo.InvariantCulture) : 200;
        int repeats = args.Length > 3 ? int.Parse(args[3], CultureInfo.InvariantCulture) : 7;

        long before = GC.GetTotalMemory(true);
        var scene = ParticleScene.CreatePile(bodies, 1);
        for (int i = 0; i < 60; i++) { scene.Step(1f / 60f); }
        long after = GC.GetTotalMemory(true);
        var w = scene.World;
        int n = w.Bodies.HighWater;

        // Whole-step time, best of repeats, on the live (double) world; the snapshot is restored before each repeat so every repeat does the same work.
        byte[] snap = w.Snapshot();
        double bestStep = double.MaxValue;
        for (int r = 0; r < repeats; r++)
        {
            w.Restore(snap);
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < steps; i++) { scene.Step(1f / 60f); }
            bestStep = Math.Min(bestStep, sw.Elapsed.TotalMilliseconds / steps);
        }

        // The position-touching loops, double against float, over this world's bodies and constraints.
        var pd = w.Bodies.Position.ToArray();
        var pf = new float[pd.Length];
        for (int i = 0; i < pd.Length; i++) { pf[i] = (float)pd[i]; }
        var vel = w.Bodies.LinearVelocity.ToArray();
        var pairs = new List<(int A, int B)>();
        for (int c = 0; c < w.Constraints.HighWater; c++)
        {
            if (w.Constraints.IsLive(c) && w.Constraints.Kind[c] != (byte)ConstraintKind.Sensor) { pairs.Add((w.Constraints.BodyA[c], w.Constraints.BodyB[c])); }
        }

        var pa = pairs.ToArray();
        int substeps = w.Options.Substeps;
        int passes = 1 + substeps * (w.Options.VelocityIterations + w.Options.RelaxIterations);
        double integrateD = Best(repeats, steps, () => IntegrateD(pd, vel, n, substeps));
        double integrateF = Best(repeats, steps, () => IntegrateF(pf, vel, n, substeps));
        double dirD = Best(repeats, steps, () => DirectionD(pd, pa, passes));
        double dirF = Best(repeats, steps, () => DirectionF(pf, pa, passes));

        double colD = 8.0 * w.Layout.PositionLanes * n, colF = 4.0 * w.Layout.PositionLanes * n;
        Console.WriteLine("position cost: bodies " + n + ", constraints " + pa.Length + ", substeps " + substeps + ", " + passes + " direction passes per step");
        Console.WriteLine("| quantity | double positions | float32 positions | difference |");
        Console.WriteLine("|---|---|---|---|");
        Console.WriteLine("| position column | " + colD.ToString("N0", CultureInfo.InvariantCulture) + " B | " + colF.ToString("N0", CultureInfo.InvariantCulture) + " B | +" + (colD - colF).ToString("N0", CultureInfo.InvariantCulture) + " B (" + ((colD - colF) / n).ToString("F0", CultureInfo.InvariantCulture) + " B/body) |");
        Console.WriteLine("| whole world (GC heap, bodies+constraints+islands) | " + (after - before).ToString("N0", CultureInfo.InvariantCulture) + " B | " + (after - before - (long)(colD - colF)).ToString("N0", CultureInfo.InvariantCulture) + " B | +" + ((colD - colF) / (after - before) * 100).ToString("F1", CultureInfo.InvariantCulture) + " % |");
        Console.WriteLine("| integrate positions (ms/step) | " + F(integrateD) + " | " + F(integrateF) + " | " + F(integrateD - integrateF) + " |");
        Console.WriteLine("| constraint directions (ms/step) | " + F(dirD) + " | " + F(dirF) + " | " + F(dirD - dirF) + " |");
        Console.WriteLine("| position-touching loops total (ms/step) | " + F(integrateD + dirD) + " | " + F(integrateF + dirF) + " | " + F(integrateD + dirD - integrateF - dirF) + " |");
        Console.WriteLine("| whole step, double world (ms/step, best of " + repeats + ") | " + F(bestStep) + " | n/a | extra cost bounded by " + ((integrateD + dirD - integrateF - dirF) / bestStep * 100).ToString("F1", CultureInfo.InvariantCulture) + " % of the step |");
        Console.WriteLine("checksum " + (pd[0] + pf[0] + vel[0]).ToString("R", CultureInfo.InvariantCulture));
        return 0;
    }

    private static string F(double v) => v.ToString("F4", CultureInfo.InvariantCulture);

    private static double Best(int repeats, int iterations, Action body)
    {
        double best = double.MaxValue;
        body();
        for (int r = 0; r < repeats; r++)
        {
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < iterations; i++) { body(); }
            best = Math.Min(best, sw.Elapsed.TotalMilliseconds / iterations);
        }

        return best;
    }

    private static void IntegrateD(double[] p, float[] v, int n, int substeps)
    {
        double h = 1.0 / 240.0;
        for (int s = 0; s < substeps; s++)
        {
            for (int i = 0; i < n * 3; i++) { p[i] += v[i] * h; }
        }
    }

    private static void IntegrateF(float[] p, float[] v, int n, int substeps)
    {
        float h = 1f / 240f;
        for (int s = 0; s < substeps; s++)
        {
            for (int i = 0; i < n * 3; i++) { p[i] += v[i] * h; }
        }
    }

    private static double s_sink;

    private static void DirectionD(double[] p, (int A, int B)[] pairs, int passes)
    {
        double acc = 0;
        for (int k = 0; k < passes; k++)
        {
            foreach (var (a, b) in pairs)
            {
                double dx = p[b * 3] - p[a * 3], dy = p[b * 3 + 1] - p[a * 3 + 1], dz = p[b * 3 + 2] - p[a * 3 + 2];
                double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                acc += len > 1e-9 ? dx / len : 0;
            }
        }

        s_sink += acc;
    }

    private static void DirectionF(float[] p, (int A, int B)[] pairs, int passes)
    {
        float acc = 0;
        for (int k = 0; k < passes; k++)
        {
            foreach (var (a, b) in pairs)
            {
                float dx = p[b * 3] - p[a * 3], dy = p[b * 3 + 1] - p[a * 3 + 1], dz = p[b * 3 + 2] - p[a * 3 + 2];
                float len = MathF.Sqrt(dx * dx + dy * dy + dz * dz);
                acc += len > 1e-9f ? dx / len : 0;
            }
        }

        s_sink += acc;
    }
}
