#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Scenes for geometry that is edited or sampled at run time: bodies piling on a signed distance field terrain, a voxel terrain that is dug under bodies and characters, and a bitmask terrain
// carved by projectiles. The hash-matrix runs a shortened version of each (snapshotted and restored half-way); the stability runner plays the full 600 steps and reads the scene's own measurements. Scripts use
// only arithmetic on the step number and the state of the world, so the scenes are byte-identical on every platform.
internal static class TerrainEditScenes
{
    internal const float Dt = 1f / 60f;
    internal static readonly string[] Names = ["sdf-pile-3d", "voxel-dig-3d", "carve-2d"];

    internal sealed class Scene
    {
        internal required string Name;
        internal int Steps = 600;
        internal Action<int> Script = _ => { };
        internal required Action Step;
        internal required Func<ulong> Hash;
        /// <summary>Snapshot, fork, restore and carry on in the fork.</summary>
        internal required Action Replace;
        internal Action<int> Observe = _ => { };
        internal Func<double> Probe = () => 0.0;
        /// <summary>Where every body is and how fast it moves, for reading a failing run.</summary>
        internal Func<string> Describe = () => "";
        internal SortedDictionary<string, double> Metrics = new();
    }

    /// <summary>The scenes' own limits: 1.5 times the worst measured value on the worker, rounded up (docs/spec/scenes.md); `nan` and `escapes` are always 0.</summary>
    internal static readonly Dictionary<string, (string Metric, double Min, double Max)[]> Limits = new()
    {
        ["sdf-pile-3d"] = [("nan", 0, 0), ("escapes", 0, 0), ("maxSpeed", 0, 22.5), ("maxPenetration", -1, 0.01), ("finalSpeed99", 0, 0.5)],
        ["voxel-dig-3d"] = [("nan", 0, 0), ("escapes", 0, 0), ("maxSpeed", 0, 10), ("standerDrift", 0, 0.001), ("walkerMinY", 0.5, 100), ("edits", 100, 1e9)],
        ["carve-2d"] = [("nan", 0, 0), ("escapes", 0, 0), ("maxSpeed", 0, 26), ("carved", 50, 1e9), ("detached", 1, 1e9)],
    };

    internal static Scene Build(string name, int threads) => name switch
    {
        "sdf-pile-3d" => SdfPile(threads),
        "voxel-dig-3d" => VoxelDig(threads),
        "carve-2d" => Carve(threads),
        _ => throw new ArgumentException("unknown scene " + name),
    };

    internal static ulong Run(string name, int threads, bool restoreHalfway, int steps = 160, Action<ulong>? perStep = null)
    {
        var s = Build(name, threads);
        ulong acc = 14695981039346656037UL;
        for (int i = 0; i < steps; i++)
        {
            if (restoreHalfway && i == steps / 2) { s.Replace(); }
            s.Script(i);
            s.Step();
            s.Observe(i);
            ulong h = s.Hash() * 1099511628211UL ^ (ulong)BitConverter.DoubleToInt64Bits(s.Probe());
            acc = (acc ^ h) * 1099511628211UL;
            perStep?.Invoke(h);
        }

        return acc;
    }

    private static double Speed(Vector3F v) => Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z);
    private static double Speed(Vector2F v) => Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y);
    private static double P99(List<double> values) { values.Sort(); return values.Count == 0 ? 0 : values[(int)(0.99 * (values.Count - 1))]; }

    // ------------------------------------------------------------------ sdf-pile-3d

    private static Scene SdfPile(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 101 });
        // Terrain: a floor with two hills, a ridge and a pit, as a union and difference of exact primitives.
        var floor = SdfGrid3D.HalfSpace(default, new Vector3F(0, 1, 0));
        var shape = SdfGrid3D.Difference(SdfGrid3D.Union(SdfGrid3D.Union(floor, SdfGrid3D.Sphere(new Vector3F(-4, 0, -3), 2.0f)), SdfGrid3D.Union(SdfGrid3D.Sphere(new Vector3F(5, 0, 4), 2.5f), SdfGrid3D.Box(new Vector3F(0, 0.3f, 6), new Vector3F(3, 0.4f, 1)))),
            SdfGrid3D.Sphere(new Vector3F(1, 0.2f, -5), 1.5f));
        var terrain = SdfGrid3D.FromFunction(new Vector3F(-12, -3, -12), new Vector3F(12, 4, 12), 0.25f, shape);
        w.AddSdf(terrain, material: new P3.Material3D(0.7f));
        // Walls round the edge keep rolling bodies on the slab.
        foreach (var (wx, wz, sx, sz) in new[] { (12.5, 0.0, 1f, 26f), (-12.5, 0.0, 1f, 26f), (0.0, 12.5, 26f, 1f), (0.0, -12.5, 26f, 1f) }) w.AddBox(new Vector3F(sx, 4f, sz), new Vector3D(wx, 2, wz), type: BodyType.Static);
        // Dynamic fields: a torus built from a mesh, a sphere and a cube.
        var tv = new List<Vector3F>(); var ti = new List<int>(); const int Nu = 20, Nv = 10;
        for (int i = 0; i < Nu; i++) for (int j = 0; j < Nv; j++)
        {
            var (su, cu) = Math64.SinCos(2 * Math.PI * i / Nu); var (sv, cv) = Math64.SinCos(2 * Math.PI * j / Nv);
            tv.Add(new Vector3F((float)((1.0 + 0.4 * cv) * cu), (float)(0.4 * sv), (float)((1.0 + 0.4 * cv) * su)));
        }

        for (int i = 0; i < Nu; i++) for (int j = 0; j < Nv; j++)
        {
            int a = i * Nv + j, b = ((i + 1) % Nu) * Nv + j, c = ((i + 1) % Nu) * Nv + (j + 1) % Nv, d = i * Nv + (j + 1) % Nv;
            ti.AddRange([a, d, c, a, c, b]);
        }

        var torus = SdfGrid3D.FromMesh(tv.ToArray(), ti.ToArray(), 0.1f, 2);
        var ball = SdfGrid3D.FromFunction(new Vector3F(-0.7f, -0.7f, -0.7f), new Vector3F(0.7f, 0.7f, 0.7f), 0.1f, SdfGrid3D.Sphere(default, 0.5f));
        var cube = SdfGrid3D.FromFunction(new Vector3F(-0.7f, -0.7f, -0.7f), new Vector3F(0.7f, 0.7f, 0.7f), 0.1f, SdfGrid3D.Box(default, new Vector3F(0.4f, 0.4f, 0.4f)));
        var handles = new List<BodyHandle>(); var spheres = new List<(BodyHandle Body, float Radius)>();
        int k = 0;
        for (int row = 0; row < 5; row++) for (int col = 0; col < 5; col++, k++)
        {
            double x = -8 + col * 3.2 + (row % 2) * 0.4, z = -8 + row * 3.2, y = 3 + (k % 5) * 1.3;
            P3.Body b;
            switch (k % 3)
            {
                case 0: b = w.AddBox(new Vector3F(0.8f, 0.6f, 0.8f), new Vector3D(x, y, z)); break;
                case 1: b = w.AddSphere(0.4f, new Vector3D(x, y, z)); spheres.Add((b.RawHandle, 0.4f)); break;
                default: b = w.AddCapsule(0.4f, 0.25f, new Vector3D(x, y, z)); break;
            }

            handles.Add(b.RawHandle);
        }

        for (int i = 0; i < 3; i++)
        {
            handles.Add(w.AddSdf(torus, new Vector3D(-6 + i * 5, 7 + i, 1), type: BodyType.Dynamic).RawHandle);
            handles.Add(w.AddSdf(ball, new Vector3D(-5 + i * 5, 9 + i, -2), type: BodyType.Dynamic).RawHandle);
            handles.Add(w.AddSdf(cube, new Vector3D(-4 + i * 5, 11 + i, 3), type: BodyType.Dynamic).RawHandle);
        }

        var s = new Scene { Name = "sdf-pile-3d", Step = () => w.Step(Dt), Hash = () => w.ComputeHash(), Replace = () => { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); } };
        double maxSpeed = 0, maxPen = 0, nan = 0, escapes = 0; var speeds = new List<double>();
        s.Observe = step =>
        {
            speeds.Clear(); escapes = 0;
            foreach (var h in handles)
            {
                var b = w.GetBody(h); var v = b.LinearVelocity; double sp = Speed(v); if (!double.IsFinite(sp)) { nan = 1; continue; }
                maxSpeed = Math.Max(maxSpeed, sp); speeds.Add(sp);
                var p = b.Position; if (Math.Abs(p.X) > 13 || Math.Abs(p.Z) > 13 || p.Y < -3) escapes++;
            }

            if (step > 240) foreach (var (h, r) in spheres) { var p = w.GetBody(h).Position; maxPen = Math.Max(maxPen, r - terrain.Value(new Vector3F((float)p.X, (float)p.Y, (float)p.Z))); }
            s.Metrics["nan"] = nan; s.Metrics["maxSpeed"] = maxSpeed; s.Metrics["escapes"] = escapes; s.Metrics["maxPenetration"] = maxPen; s.Metrics["finalSpeed99"] = P99(speeds);
        };
        s.Probe = () => w.GetBody(handles[handles.Count - 1]).Position.Y;
        s.Describe = () => { var sb = new System.Text.StringBuilder(); for (int i = 0; i < handles.Count; i++) { var b = w.GetBody(handles[i]); sb.Append(i).Append(' ').Append(b.Position).Append(" v ").Append(Speed(b.LinearVelocity).ToString("F2")).Append('\n'); } return sb.ToString(); };
        return s;
    }

    // ------------------------------------------------------------------ voxel-dig-3d

    private static Scene VoxelDig(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 102 });
        const int N = 60, H = 12, Top = 6; // 0.5 m cells: a 30 m square floor with its surface at y = 3
        var cells = new bool[N * H * N];
        for (int z = 0; z < N; z++) for (int y = 0; y < Top; y++) for (int x = 0; x < N; x++) cells[(z * H + y) * N + x] = true;
        var ground = w.AddVoxelGrid(N, H, N, cells, 0.5f, material: new P3.Material3D(0.7f));
        var voxels = ground.Voxels;
        var handles = new List<BodyHandle>(); var pits = new List<Vector3D>();
        int k = 0;
        for (int row = 0; row < 4; row++) for (int col = 0; col < 6; col++, k++)
        {
            double x = 10 + col * 1.6, z = 10 + row * 1.6;
            handles.Add((k % 3 == 1 ? w.AddSphere(0.4f, new Vector3D(x, 3.4, z)) : w.AddBox(new Vector3F(0.8f, 0.8f, 0.8f), new Vector3D(x, 3.45, z))).RawHandle);
            if ((row + col) % 3 == 0) pits.Add(new Vector3D(x, 3.0, z));
        }

        var stander = w.AddCharacter(new Vector3D(25, 3.2, 5)); var walker = w.AddCharacter(new Vector3D(8, 3.2, 12));
        double standerY = double.NaN, standerDrift = 0, walkerMinY = 9, maxSpeed = 0, nan = 0, escapes = 0, edits = 0; float walkerVy = 0, standerVy = 0;
        var s = new Scene { Name = "voxel-dig-3d", Step = () => w.Step(Dt), Hash = () => w.ComputeHash(), Replace = () => { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); ground = FindStatic(w); voxels = ground.Voxels; stander = w.GetCharacter(stander.RawHandle); walker = w.GetCharacter(walker.RawHandle); } };
        s.Script = step =>
        {
            // Dig under a body every 70 steps, fill an earlier pit every 70 steps in between.
            if (step % 70 == 35 && step / 70 < pits.Count) { edits += voxels.DigSphere(pits[step / 70], 1.1); }
            if (step % 70 == 0 && step >= 70 && (step / 70 - 1) < pits.Count && (step / 70) % 2 == 0) { var p = pits[step / 70 - 1]; edits += voxels.SetVoxels((int)(p.X / 0.5) - 3, 0, (int)(p.Z / 0.5) - 3, 6, Top, 6, true); }
            standerVy = stander.IsGrounded && standerVy < 0 ? -1f : standerVy - 9.81f * Dt; stander.Move(new Vector3F(0, standerVy * Dt, 0), Dt);
            walkerVy = walker.IsGrounded && walkerVy < 0 ? -1f : walkerVy - 9.81f * Dt; walker.Move(new Vector3F(0.015f, walkerVy * Dt, 0.002f), Dt);
        };
        s.Observe = step =>
        {
            escapes = 0;
            for (int i = 0; i < handles.Count; i++)
            {
                var b = w.GetBody(handles[i]); double sp = Speed(b.LinearVelocity); if (!double.IsFinite(sp)) { nan = 1; continue; }
                maxSpeed = Math.Max(maxSpeed, sp); var p = b.Position; if (p.X < 0 || p.X > 30 || p.Z < 0 || p.Z > 30 || p.Y < 0.2) escapes++;
            }

            double sy = stander.Position.Y; if (step == 60) standerY = sy; if (step > 60) standerDrift = Math.Max(standerDrift, Math.Abs(sy - standerY));
            walkerMinY = Math.Min(walkerMinY, walker.Position.Y);
            s.Metrics["nan"] = nan; s.Metrics["maxSpeed"] = maxSpeed; s.Metrics["escapes"] = escapes; s.Metrics["standerDrift"] = standerDrift; s.Metrics["walkerMinY"] = walkerMinY; s.Metrics["edits"] = edits;
        };
        s.Probe = () => stander.Position.Y + walker.Position.Y;
        return s;
    }

    private static P3.Body FindStatic(P3.PhysicsWorld3D w) { foreach (var b in w.Bodies) if (b.Type == BodyType.Static) return b; throw new InvalidOperationException("no static body"); }

    // ------------------------------------------------------------------ carve-2d

    private static Scene Carve(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = 103 });
        const int W = 160, H = 64, Top = 32; // 0.25 m cells: a 40 x 16 m field with the ground surface at y = 8
        var mask = new bool[W * H];
        for (int y = 0; y < Top; y++) for (int x = 0; x < W; x++) mask[y * W + x] = true;
        // A raised block on the right, joined to the ground by a thin neck that the scene later cuts.
        for (int y = Top; y < Top + 3; y++) mask[y * W + 130] = true;
        for (int y = Top + 3; y < Top + 9; y++) for (int x = 126; x < 136; x++) mask[y * W + x] = true;
        var terrain = w.AddBitmaskTerrain(W, H, mask, 0.25f, material: new P2.Material2D(0.7f));
        var handles = new List<BodyHandle>(); var projectiles = new List<BodyHandle>();
        for (int i = 0; i < 36; i++)
        {
            double x = 9 + (i % 12) * 1.1, y = 8.5 + (i / 12) * 1.2;
            var b = (i % 3) switch { 0 => w.AddBox(new Vector2F(0.8f, 0.6f), new Vector2D(x, y)), 1 => w.AddCircle(0.35f, new Vector2D(x, y)), _ => w.AddCapsule(0.3f, 0.2f, new Vector2D(x, y), 0.3f) };
            handles.Add(b.RawHandle);
        }

        double maxSpeed = 0, nan = 0, escapes = 0, carved = 0, detached = 0, fired = 0; int next = 0;
        var s = new Scene { Name = "carve-2d", Step = () => w.Step(Dt), Hash = () => w.ComputeHash(), Replace = () => { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); terrain = FindStatic2(w); } };
        s.Script = step =>
        {
            if (step % 45 == 20 && step >= 20)
            {
                double x = 10 + (next % 8) * 1.4; next++;
                var p = w.AddCircle(0.25f, new Vector2D(x, 13), density: 4f); p.LinearVelocity = new Vector2F(0, -22); projectiles.Add(p.RawHandle); fired++;
            }

            // A projectile that reached the ground carves a crater and is gone.
            for (int i = projectiles.Count - 1; i >= 0; i--)
            {
                var p = w.GetBody(projectiles[i]);
                if (p.Position.Y < 8.5) { carved += terrain.CarveCircle(p.Position, 1.2); p.Destroy(); projectiles.RemoveAt(i); }
            }

            if (step % 150 == 100) { terrain.CarveCircle(new Vector2D(32.5, 8.2), 0.45); detached += terrain.DetachIslands().Length; }
        };
        s.Observe = step =>
        {
            escapes = 0;
            foreach (var h in handles)
            {
                var b = w.GetBody(h); double sp = Speed(b.LinearVelocity); if (!double.IsFinite(sp)) { nan = 1; continue; }
                maxSpeed = Math.Max(maxSpeed, sp); var p = b.Position; if (p.X < 0 || p.X > 40 || p.Y < 0.2) escapes++;
            }

            s.Metrics["nan"] = nan; s.Metrics["maxSpeed"] = maxSpeed; s.Metrics["escapes"] = escapes; s.Metrics["carved"] = carved; s.Metrics["detached"] = detached; s.Metrics["fired"] = fired;
        };
        s.Probe = () => w.GetBody(handles[handles.Count - 1]).Position.Y;
        return s;
    }

    private static P2.Body FindStatic2(P2.PhysicsWorld2D w) { foreach (var b in w.Bodies) if (b.Type == BodyType.Static) return b; throw new InvalidOperationException("no static body"); }
}
