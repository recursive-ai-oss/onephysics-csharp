#pragma warning disable OPHYS0002, OPHYS0003
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
}
