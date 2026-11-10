using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Scenes for world scale (streamed terrain tiles, simulation level of detail, local time scale). Terrain comes from a function of the position built from integer hashing and
// arithmetic only, so every platform builds the same heights bit for bit.
internal static class WorldScaleScenes
{
    internal const float Dt = 1f / 60f;
    internal const int TileCells = 32;
    internal const float Spacing = 1f;
    internal const double TileSize = TileCells * Spacing;

    internal static readonly string[] Names = { "streaming-3d", "streaming-2d", "lod-3d", "timescale-2d" };

    // ------------------------------------------------------------------ terrain

    private static float Lattice(int ix, int iz)
    {
        uint h = unchecked((uint)(ix * 374761393) + (uint)(iz * 668265263));
        h = unchecked((h ^ (h >> 13)) * 1274126177u);
        h ^= h >> 16;
        return (h & 0xFFFFFF) / 16777216f;
    }

    private static float Smooth(float t) => t * t * (3f - 2f * t);

    /// <summary>Gently rolling ground sloping down along +x: value noise on an 8 m lattice (up to 0.9 m high) minus 12 cm per metre.</summary>
    internal static float Height(int gx, int gz)
    {
        float fx = gx / 8f, fz = gz / 8f;
        int ix = (int)MathF.Floor(fx), iz = (int)MathF.Floor(fz);
        float tx = Smooth(fx - ix), tz = Smooth(fz - iz);
        float a = Lattice(ix, iz), b = Lattice(ix + 1, iz), c = Lattice(ix, iz + 1), d = Lattice(ix + 1, iz + 1);
        float n = (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * tz;
        return (n * 0.9f) - (0.12f * gx);
    }

    /// <summary>The 2D ground profile (a function of x only, with the same lattice along z = 0).</summary>
    internal static float Height2D(int gx) => Height(gx, 0);

    internal static P3.TileDef3D Tile3D(int tx, int tz, bool edges, int cells = TileCells, Func<int, int, float>? height = null)
    {
        var Height = height ?? WorldScaleScenes.Height;
        int n = cells + 1;
        var h = new float[n * n];
        int x0 = tx * cells, z0 = tz * cells;
        for (int j = 0; j < n; j++) { for (int i = 0; i < n; i++) { h[j * n + i] = Height(x0 + i, z0 + j); } }
        var def = new P3.TileDef3D { Origin = new Vector3D(x0 * (double)Spacing, 0, z0 * (double)Spacing) };
        var part = new P3.TileHeightField3D { Heights = h, SamplesX = n, SamplesZ = n, Scale = new Vector3F(Spacing, 1f, Spacing) };
        if (edges)
        {
            var e = new RecursiveAI.OnePhysics.LowLevel.HeightFieldEdges3D { West = new float[n], East = new float[n], South = new float[n], North = new float[n] };
            for (int k = 0; k < n; k++)
            {
                e.West![k] = Height(x0 - 1, z0 + k); e.East![k] = Height(x0 + cells + 1, z0 + k);
                e.South![k] = Height(x0 + k, z0 - 1); e.North![k] = Height(x0 + k, z0 + cells + 1);
            }

            part.Edges = e;
        }

        def.HeightFields.Add(part);
        return def;
    }

    internal static P2.TileDef2D Tile2D(int tx, bool edges, int cells = TileCells, Func<int, float>? height = null)
    {
        var Height2D = height ?? WorldScaleScenes.Height2D;
        int n = cells + 1, x0 = tx * cells;
        var h = new float[n];
        for (int i = 0; i < n; i++) { h[i] = Height2D(x0 + i); }
        var def = new P2.TileDef2D { Origin = new Vector2D(x0 * (double)Spacing, 0) };
        def.HeightFields.Add(new P2.TileHeightField2D { Heights = h, Spacing = Spacing, HeightBefore = edges ? Height2D(x0 - 1) : null, HeightAfter = edges ? Height2D(x0 + cells + 1) : null });
        return def;
    }

    // ------------------------------------------------------------------ streaming-3d

    internal const int Tiles = 16;

    /// <summary>
    /// A 16 x 16 tile field of rolling ground (512 m square) streamed in a 5 x 5 window that follows the mean position of 194 boxes and spheres, a car and six walkers.
    /// Everything starts near tile (3, 7) and drifts down the slope over about 20 seconds, crossing tile seams the whole time; tiles behind the window are unloaded once nothing touches them.
    /// </summary>
    internal sealed class Streaming3D
    {
        internal required P3.PhysicsWorld3D World { get; set; }
        internal readonly List<P3.Body> Bodies = new();
        internal P3.Vehicle Car;
        internal P3.Character[] Walkers = Array.Empty<P3.Character>();
        internal int FocusX = -1, FocusZ = -1, Loads, Unloads, MaxLoaded, Steps = 1200;
        internal double MaxSpeed, SeamStep;
        internal bool Unsplit;
        private float[] _vy = Array.Empty<float>();
        internal int Tunnelled, Escapes, NonFinite;
        internal ulong CharacterHash = 0xCBF29CE484222325UL;

        internal void Window()
        {
            if (Unsplit) { return; }
            double sx = 0, sz = 0; int n = 0;
            foreach (var b in Bodies) { var p = b.Position; sx += p.X; sz += p.Z; n++; }
            foreach (var c in Walkers) { var p = c.Position; sx += p.X; sz += p.Z; n++; }
            int fx = Math.Clamp((int)Math.Floor(sx / n / TileSize), 0, Tiles - 1), fz = Math.Clamp((int)Math.Floor(sz / n / TileSize), 0, Tiles - 1);
            if (fx == FocusX && fz == FocusZ) { return; }
            FocusX = fx; FocusZ = fz;
            foreach (var key in World.GetLoadedTiles())
            {
                if (Math.Abs(key.X - fx) > 2 || Math.Abs(key.Y - fz) > 2) { if (World.UnloadTile(key)) { Unloads++; } }
            }

            for (int tx = Math.Max(0, fx - 2); tx <= Math.Min(Tiles - 1, fx + 2); tx++)
            {
                for (int tz = Math.Max(0, fz - 2); tz <= Math.Min(Tiles - 1, fz + 2); tz++)
                {
                    var key = new TileKey(tx, tz);
                    if (World.GetTileState(key) == TileState.Absent) { World.LoadTile(key, Tile3D(tx, tz, true)); Loads++; }
                }
            }
        }

        internal void Step(int step)
        {
            if (step % 4 == 0 || FocusX < 0) { Window(); }
            float carSpeed = Bodies[^1].LinearVelocity.Length();
            Car.Drive(carSpeed < 3.5f ? 0.4f : 0f, 0f, carSpeed > 4.5f ? 0.6f : 0f);
            for (int i = 0; i < Walkers.Length; i++)
            {
                var cc = Walkers[i].MoveVelocity(new Vector3F(1.5f, -2f, 0f), Dt);
                CharacterHash = (CharacterHash ^ (ulong)cc) * 0x100000001B3UL; CharacterHash ^= CharacterHash >> 29;
            }

            World.Step(Dt);
            Escapes = 0;
            if (_vy.Length == 0) { _vy = new float[Bodies.Count]; }
            int index = -1;
            foreach (var b in Bodies)
            {
                index++;
                var p = b.Position; var v = b.LinearVelocity;
                if (index < 120 && step > 20)
                {
                    double dx = Math.Abs(p.X - Math.Round(p.X / TileSize) * TileSize), dz = Math.Abs(p.Z - Math.Round(p.Z / TileSize) * TileSize);
                    if ((dx < 0.45 || dz < 0.45) && float.IsFinite(v.Y)) { SeamStep = Math.Max(SeamStep, Math.Abs(v.Y - _vy[index])); }
                }

                _vy[index] = v.Y;
                if (!double.IsFinite(p.X + p.Y + p.Z) || !float.IsFinite(v.X + v.Y + v.Z)) { NonFinite++; continue; }
                MaxSpeed = Math.Max(MaxSpeed, v.Length());
                int gx = (int)Math.Floor(p.X), gz = (int)Math.Floor(p.Z);
                if (gx >= 0 && gz >= 0 && gx < Tiles * TileCells && gz < Tiles * TileCells && p.Y < Height(gx, gz) - 2.5) { Escapes++; }
            }

            Tunnelled = Math.Max(Tunnelled, Escapes);
            MaxLoaded = Math.Max(MaxLoaded, World.TileCount);
        }

        internal ulong Hash => World.ComputeHash();
    }

    internal static Streaming3D BuildStreaming3D(int threads, int worldId = 1400, bool unsplit = false)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = worldId });
        var s = new Streaming3D { World = w, Unsplit = unsplit };
        if (unsplit)
        {
            int n = Tiles * TileCells + 1;
            var h = new float[n * n];
            for (int j = 0; j < n; j++) { for (int i = 0; i < n; i++) { h[j * n + i] = Height(i, j); } }
            w.AddHeightField(h, n, n, new Vector3F(Spacing, 1f, Spacing));
        }

        var roll = new P3.Material3D(0.6f, 0f);
        // The first window, built before the first step (the host loads what it needs before simulating).
        s.Bodies.Capacity = 200;
        int placed = 0;
        for (int i = 0; i < 12 && placed < 120; i++)
        {
            for (int j = 0; j < 10 && placed < 120; j++)
            {
                double x = 78 + i * 2.6, z = 206 + j * 4.8;
                s.Bodies.Add(w.AddSphere(0.35f + 0.05f * ((i + j) % 3), new Vector3D(x, Height((int)x, (int)z) + 0.9, z), material: roll)); placed++;
            }
        }

        for (int i = 0; i < 14; i++)
        {
            for (int j = 0; j < 5; j++)
            {
                double x = 76 + i * 2.4, z = 206 + j * 11.0 + (i % 2) + (j > 1 ? 11 : 0);
                s.Bodies.Add(w.AddBox(new Vector3F(0.9f, 0.9f, 0.9f), new Vector3D(x, Height((int)x, (int)z) + 0.8, z), material: new P3.Material3D(0.5f, 0f))); placed++;
            }
        }

        var chassis = w.AddBox(new Vector3F(1.8f, 0.5f, 4.2f), new Vector3D(66, Height(66, 232) + 1.0, 232), QuaternionF.CreateFromAxisSinCos(new Vector3F(0f, 1f, 0f), 0.70710678f, 0.70710678f), density: 1500f / (1.8f * 0.5f * 4.2f));
        s.Bodies.Add(chassis);
        s.Car = w.AddVehicle(chassis, P3.VehicleDef.Car);
        s.Walkers = new P3.Character[6];
        for (int i = 0; i < s.Walkers.Length; i++) { s.Walkers[i] = w.AddCharacter(new Vector3D(70, Height(70, 218 + i * 6) + 1.0, 218 + i * 6)); }
        s.Window();
        return s;
    }

    internal static ulong RunStreaming3D(int threads, bool restoreHalfway, Action<ulong>? onStep = null, int steps = 300)
    {
        var s = BuildStreaming3D(threads);
        ulong hash = 0xCBF29CE484222325UL;
        for (int i = 0; i < steps; i++)
        {
            if (i == steps / 2 && restoreHalfway)
            {
                var snap = s.World.Snapshot();
                var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 1400 });
                w.Restore(snap);
                s.World = w;
                for (int k = 0; k < s.Bodies.Count; k++) { s.Bodies[k] = w.GetBody(s.Bodies[k].RawHandle); }
                s.Car = w.GetVehicles()[0];
                s.Walkers = w.GetCharacters();
            }

            s.Step(i);
            ulong h = s.Hash ^ s.CharacterHash;
            hash = (hash ^ h) * 0x100000001B3UL; hash ^= hash >> 29;
            onStep?.Invoke(h);
        }

        return hash;
    }

    // ------------------------------------------------------------------ streaming-2d

    /// <summary>A 16 tile ground profile (512 m) streamed in a window of five tiles that follows the mean position of 70 circles, 40 boxes and three walkers.</summary>
    internal sealed class Streaming2D
    {
        internal required P2.PhysicsWorld2D World { get; set; }
        internal readonly List<P2.Body> Bodies = new();
        internal P2.Character[] Walkers = Array.Empty<P2.Character>();
        internal int Focus = -1, Loads, Unloads, MaxLoaded, Steps = 1200, Tunnelled, NonFinite;
        internal double MaxSpeed, SeamStep;
        internal bool Unsplit;
        internal ulong CharacterHash = 0xCBF29CE484222325UL;
        private float[] _vy = Array.Empty<float>();

        internal void Window()
        {
            if (Unsplit) { return; }
            double lo = double.MaxValue, hi = double.MinValue;
            foreach (var b in Bodies) { double x = b.Position.X; if (x < lo) { lo = x; } if (x > hi) { hi = x; } }
            foreach (var c in Walkers) { double x = c.Position.X; if (x < lo) { lo = x; } if (x > hi) { hi = x; } }
            int f = Math.Clamp((int)Math.Floor((lo + hi) * 0.5 / TileSize), 0, Tiles - 1);
            if (f == Focus) { return; }
            Focus = f;
            foreach (var key in World.GetLoadedTiles()) { if (Math.Abs(key.X - f) > 2) { if (World.UnloadTile(key)) { Unloads++; } } }
            for (int tx = Math.Max(0, f - 2); tx <= Math.Min(Tiles - 1, f + 2); tx++)
            {
                var key = new TileKey(tx, 0);
                if (World.GetTileState(key) == TileState.Absent) { World.LoadTile(key, Tile2D(tx, true)); Loads++; }
            }
        }

        internal void Step(int step)
        {
            if (step % 4 == 0 || Focus < 0) { Window(); }
            for (int i = 0; i < Walkers.Length; i++)
            {
                var cc = Walkers[i].Move(new Vector2F(1.5f * Dt, -2f * Dt), Dt);
                CharacterHash = (CharacterHash ^ (ulong)cc) * 0x100000001B3UL; CharacterHash ^= CharacterHash >> 29;
            }

            World.Step(Dt);
            if (_vy.Length == 0) { _vy = new float[Bodies.Count]; }
            int tunnelled = 0, index = -1;
            foreach (var b in Bodies)
            {
                index++;
                var p = b.Position; var v = b.LinearVelocity;
                if (!double.IsFinite(p.X + p.Y) || !float.IsFinite(v.X + v.Y)) { NonFinite++; continue; }
                MaxSpeed = Math.Max(MaxSpeed, v.Length());
                if (index < 70 && step > 20 && Math.Abs(p.X - Math.Round(p.X / TileSize) * TileSize) < 0.45) { SeamStep = Math.Max(SeamStep, Math.Abs(v.Y - _vy[index])); }
                _vy[index] = v.Y;
                int gx = (int)Math.Floor(p.X);
                if (gx >= 0 && gx < Tiles * TileCells && p.Y < Height2D(gx) - 2.5) { tunnelled++; }
            }

            Tunnelled = Math.Max(Tunnelled, tunnelled);
            MaxLoaded = Math.Max(MaxLoaded, World.TileCount);
        }
    }

    internal static Streaming2D BuildStreaming2D(int threads, int worldId = 1410, bool unsplit = false)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = worldId });
        var s = new Streaming2D { World = w, Unsplit = unsplit };
        if (unsplit)
        {
            int n = Tiles * TileCells + 1;
            var h = new float[n];
            for (int i = 0; i < n; i++) { h[i] = Height2D(i); }
            w.AddHeightField(h, Spacing);
        }

        for (int i = 0; i < 70; i++)
        {
            double x = 80 + i * 0.9 + (i % 3) * 0.2; int gx = (int)x;
            s.Bodies.Add(w.AddCircle(0.3f + 0.05f * (i % 3), new Vector2D(x, Height2D(gx) + 0.6 + 0.35 * (i % 4)), material: new P2.Material2D(0.6f, 0f, 0.012f)));
        }

        for (int i = 0; i < 40; i++)
        {
            double x = 74 + i * 1.7; int gx = (int)x;
            s.Bodies.Add(w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(x, Height2D(gx) + 0.9), material: new P2.Material2D(0.55f, 0f)));
        }

        s.Walkers = new P2.Character[3];
        for (int i = 0; i < 3; i++) { s.Walkers[i] = w.AddCharacter(new Vector2D(66 + i * 2, Height2D(66 + i * 2) + 1.0)); }
        s.Window();
        return s;
    }

    internal static ulong RunStreaming2D(int threads, bool restoreHalfway, Action<ulong>? onStep = null, int steps = 300)
    {
        var s = BuildStreaming2D(threads);
        ulong hash = 0xCBF29CE484222325UL;
        for (int i = 0; i < steps; i++)
        {
            if (i == steps / 2 && restoreHalfway)
            {
                var snap = s.World.Snapshot();
                var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = 1410 });
                w.Restore(snap);
                s.World = w;
                for (int k = 0; k < s.Bodies.Count; k++) { s.Bodies[k] = w.GetBody(s.Bodies[k].RawHandle); }
                s.Walkers = w.GetCharacters();
            }

            s.Step(i);
            ulong h = s.World.ComputeHash() ^ s.CharacterHash;
            hash = (hash ^ h) * 0x100000001B3UL; hash ^= hash >> 29;
            onStep?.Invoke(h);
        }

        return hash;
    }

    // ------------------------------------------------------------------ lod-3d

    /// <summary>
    /// Twenty-four walls of 25 boxes (5 x 5) along a 340 m ground slab, a focus point that walks the length of it, and distance bands: within 30 m every step, to 80 m every second, to 150 m every fourth, beyond that every eighth.
    /// With <paramref name="neverSleep"/> the boxes stay awake, which is what a timing run needs.
    /// </summary>
    internal sealed class Lod3D
    {
        internal required P3.PhysicsWorld3D World { get; set; }
        internal readonly List<P3.Body> Bodies = new();
        internal readonly List<Vector3D> Start = new();
        internal int Steps = 600, NonFinite;
        internal double MaxSpeed, MaxDrift;

        internal static Vector3D Focus(int step) => new(10 + 0.1 * step, 1, 0);

        internal void Step(int step)
        {
            if (step % 8 == 0) { World.SetLodFocus([Focus(step)]); }
            World.Step(Dt);
        }

        internal void Measure()
        {
            for (int i = 0; i < Bodies.Count; i++)
            {
                var p = Bodies[i].Position; var v = Bodies[i].LinearVelocity;
                if (!double.IsFinite(p.X + p.Y + p.Z) || !float.IsFinite(v.X + v.Y + v.Z)) { NonFinite++; continue; }
                MaxSpeed = Math.Max(MaxSpeed, v.Length());
                MaxDrift = Math.Max(MaxDrift, Math.Abs(p.X - Start[i].X) + Math.Abs(p.Z - Start[i].Z));
            }
        }
    }

    internal static Lod3D BuildLod3D(int threads, int worldId = 1420, bool neverSleep = false, bool lod = true, int walls = 24)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = worldId });
        w.AddBox(new Vector3F(walls * 14f + 40f, 1f, 30f), new Vector3D(walls * 7.0, -0.5, 0), type: BodyType.Static);
        var s = new Lod3D { World = w };
        for (int k = 0; k < walls; k++)
        {
            for (int row = 0; row < 5; row++)
            {
                for (int col = 0; col < 5; col++)
                {
                    var at = new Vector3D(k * 14.0 + col * 1.02 + 0.3 * (row % 2), 0.5 + row * 1.01, (k % 3 - 1) * 4.0);
                    var b = w.AddBox(new Vector3F(1f, 1f, 1f), at);
                    if (neverSleep) { b.NeverSleep = true; }
                    s.Bodies.Add(b); s.Start.Add(at);
                }
            }
        }

        if (lod) { w.SetSimulationLod([new Vector3D(10, 1, 0)], [new LodBand(30, 1), new LodBand(80, 2), new LodBand(150, 4)], 8); }
        return s;
    }

    internal static ulong RunLod3D(int threads, bool restoreHalfway, Action<ulong>? onStep = null, int steps = 200)
    {
        var s = BuildLod3D(threads);
        ulong hash = 0xCBF29CE484222325UL;
        for (int i = 0; i < steps; i++)
        {
            if (i == steps / 2 && restoreHalfway)
            {
                var snap = s.World.Snapshot();
                var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 1420 });
                w.Restore(snap);
                s.World = w;
                for (int k = 0; k < s.Bodies.Count; k++) { s.Bodies[k] = w.GetBody(s.Bodies[k].RawHandle); }
            }

            s.Step(i);
            ulong h = s.World.ComputeHash();
            hash = (hash ^ h) * 0x100000001B3UL; hash ^= hash >> 29;
            onStep?.Invoke(h);
        }

        return hash;
    }

    // ------------------------------------------------------------------ timescale-2d

    /// <summary>
    /// A slow-motion bubble (time scale 0.25) next to normal time on a flat floor: a pyramid of boxes in each, a pendulum with its own scale of 0.5, and a ball that is thrown from normal time through the bubble's edge into the bubble's pyramid.
    /// </summary>
    internal sealed class TimeScale2D
    {
        internal required P2.PhysicsWorld2D World { get; set; }
        internal readonly List<P2.Body> Bodies = new();
        internal P2.Body Ball, Bob;
        internal int Steps = 900, NonFinite;
        internal double MaxSpeed;

        internal void Step(int step)
        {
            World.Step(Dt);
        }

        internal void Measure()
        {
            foreach (var b in Bodies)
            {
                var p = b.Position; var v = b.LinearVelocity;
                if (!double.IsFinite(p.X + p.Y) || !float.IsFinite(v.X + v.Y)) { NonFinite++; continue; }
                MaxSpeed = Math.Max(MaxSpeed, v.Length());
            }
        }
    }

    internal static TimeScale2D BuildTimeScale2D(int threads, int worldId = 1430)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = worldId });
        var s = new TimeScale2D { World = w };
        w.AddBox(new Vector2F(80f, 1f), new Vector2D(10, -0.5), type: BodyType.Static);
        for (int level = 0; level < 6; level++)
        {
            for (int i = 0; i < 6 - level; i++)
            {
                s.Bodies.Add(w.AddBox(new Vector2F(1f, 1f), new Vector2D(-14 + i * 1.05 + level * 0.525, 0.5 + level * 1.01)));
                s.Bodies.Add(w.AddBox(new Vector2F(1f, 1f), new Vector2D(24 + i * 1.05 + level * 0.525, 0.5 + level * 1.01)));
            }
        }

        w.SetTimeScaleRegions([new P2.TimeScaleRegion2D(new(20, -5), new(40, 30), 0.25f)]);
        var anchor = w.AddBox(new Vector2F(0.2f, 0.2f), new Vector2D(0, 12), type: BodyType.Static);
        s.Bob = w.AddCircle(0.25f, new Vector2D(2.0, 12 - 1.4));
        w.AddPoint(anchor, s.Bob, new Vector2D(0, 12));
        s.Bob.TimeScale = 0.5f;
        s.Bodies.Add(s.Bob);
        s.Ball = w.AddCircle(0.4f, new Vector2D(8, 0.4), material: new P2.Material2D(0.4f, 0f), density: 3f);
        s.Ball.LinearVelocity = new Vector2F(9f, 0f);
        s.Bodies.Add(s.Ball);
        return s;
    }

    internal static ulong RunTimeScale2D(int threads, bool restoreHalfway, Action<ulong>? onStep = null, int steps = 300)
    {
        var s = BuildTimeScale2D(threads);
        ulong hash = 0xCBF29CE484222325UL;
        for (int i = 0; i < steps; i++)
        {
            if (i == steps / 2 && restoreHalfway)
            {
                var snap = s.World.Snapshot();
                var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = 1430 });
                w.Restore(snap);
                s.World = w;
                for (int k = 0; k < s.Bodies.Count; k++) { s.Bodies[k] = w.GetBody(s.Bodies[k].RawHandle); }
            }

            s.Step(i);
            ulong h = s.World.ComputeHash();
            hash = (hash ^ h) * 0x100000001B3UL; hash ^= hash >> 29;
            onStep?.Invoke(h);
        }

        return hash;
    }
}
