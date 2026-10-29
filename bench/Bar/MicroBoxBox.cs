using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace Bar;

/// <summary>
/// Microbenchmark of the box-against-box manifold routine on pairs taken from a running scene: the scene is stepped to the given steps, every pair of dynamic or static boxes whose centres are
/// close enough to matter is collected with its relative pose, and <see cref="ManifoldGenerator3D.Collide"/> is timed over all of them, several repetitions, best repetition kept.
/// Uses only the public collision API, so the same file runs against any build. Prints ns per call, ns per call that made a manifold, and a digest of every manifold so two builds can be compared.
/// </summary>
static class MicroBoxBox
{
    public static int Run(Opts o)
    {
        var scene = Scene.Load(o.Get("scene", ""));
        int reps = int.Parse(o.Get("reps", "30"), CultureInfo.InvariantCulture);
        var stepList = new List<int>();
        foreach (var t in o.Get("steps", "150,250,300,340").Split(',')) stepList.Add(int.Parse(t, CultureInfo.InvariantCulture));
        var sim = new OnePhysics3Sim();
        sim.Build(scene, 1);
        int nb = scene.Bodies.Count;
        var d = new double[14 * nb];
        int stepNo = 0;
        double totalNs = 0; long calls = 0, making = 0, points = 0;
        ulong digest = 14695981039346656037UL;
        foreach (int target in stepList)
        {
            while (stepNo < target)
            {
                if (stepNo == 120) sim.WakeAll(scene);
                sim.Step(scene.Dt); stepNo++;
            }

            sim.Snapshot(d);
            var half = new Vector3F[nb];
            var isBox = new bool[nb];
            for (int i = 0; i < nb; i++)
            {
                var b = scene.Bodies[i];
                if (b.Shapes.Count == 1 && b.Shapes[0].Kind == "box") { isBox[i] = true; var p = b.Shapes[0].P; half[i] = new Vector3F((float)p[0], (float)p[1], (float)p[2]); }
            }

            var pairs = new List<(int a, int b, RigidTransformF rel)>();
            for (int i = 0; i < nb; i++)
            {
                if (!isBox[i]) continue;
                for (int j = i + 1; j < nb; j++)
                {
                    if (!isBox[j]) continue;
                    if (scene.Bodies[i].Type == "static" && scene.Bodies[j].Type == "static") continue;
                    double dx = d[14 * j] - d[14 * i], dy = d[14 * j + 1] - d[14 * i + 1], dz = d[14 * j + 2] - d[14 * i + 2];
                    float reach = half[i].Length() + half[j].Length() + 0.05f;
                    if (scene.Bodies[i].Type == "static" || scene.Bodies[j].Type == "static")
                    {
                        // A large static slab: keep pairs whose dynamic box is near its top face only.
                        int s = scene.Bodies[i].Type == "static" ? i : j, m = s == i ? j : i;
                        if (Math.Abs(d[14 * m + 1] - d[14 * s + 1]) > half[s].Y + half[m].Length() + 0.05) continue;
                    }
                    else if (dx * dx + dy * dy + dz * dz > (double)reach * reach) continue;
                    var qa = new QuaternionF((float)d[14 * i + 3], (float)d[14 * i + 4], (float)d[14 * i + 5], (float)d[14 * i + 6]);
                    var qb = new QuaternionF((float)d[14 * j + 3], (float)d[14 * j + 4], (float)d[14 * j + 5], (float)d[14 * j + 6]);
                    var inv = qa.Conjugated();
                    var rel = new RigidTransformF(inv * qb, inv.Rotate(new Vector3F((float)dx, (float)dy, (float)dz)));
                    pairs.Add((i, j, rel));
                }
            }

            var gen = new ManifoldGenerator3D();
            var shapes = new ConvexShape3D[nb];
            for (int i = 0; i < nb; i++) if (isBox[i]) shapes[i] = ConvexShape3D.Box(half[i]);
            double best = double.MaxValue; long nMaking = 0, nPoints = 0;
            ulong dg = 0;
            for (int r = 0; r < reps; r++)
            {
                nMaking = 0; nPoints = 0; if (r == 0) dg = 14695981039346656037UL;
                long t0 = Stopwatch.GetTimestamp();
                foreach (var (a, b, rel) in pairs)
                {
                    var m = gen.Collide(shapes[a], shapes[b], rel, 0.02f);
                    if (m.PointCount > 0) nMaking++;
                    nPoints += m.PointCount;
                    if (r == 0)
                    {
                        dg = (dg ^ (uint)m.PointCount) * 1099511628211UL;
                        for (int k = 0; k < m.PointCount; k++) dg = (dg ^ (ulong)BitConverter.SingleToUInt32Bits(m[k].Separation)) * 1099511628211UL;
                    }
                }

                double ns = (Stopwatch.GetTimestamp() - t0) * 1e9 / Stopwatch.Frequency;
                if (ns < best) best = ns;
            }

            Console.WriteLine($"step {target}: {pairs.Count} pairs, {nMaking} made a manifold ({nPoints} points), best {best / pairs.Count:F1} ns per call, {best / Math.Max(1, nMaking):F1} ns per manifold made, digest {dg:x16}");
            totalNs += best; calls += pairs.Count; making += nMaking; points += nPoints; digest = (digest ^ dg) * 1099511628211UL;
        }

        Console.WriteLine($"all: {calls} calls, {making} manifolds, {points} points, {totalNs / calls:F1} ns per call, {totalNs / Math.Max(1, making):F1} ns per manifold made, digest {digest:x16}");
        return 0;
    }
}
