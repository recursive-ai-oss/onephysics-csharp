#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;

// Scenes for shallow water: the hash-matrix and smoke scenes (a wave tank with a boat, a barge, crates and spheres, snapshotted and restored half-way; the same side-on) and the two stability scenes (shallow-water-3d and
// shallow-water-2d). Scripts use only arithmetic on the step number, so the scenes are byte-identical on every platform.
internal static class WaterScenes
{
    internal const float Dt = 1f / 60f;

    internal static readonly string[] Names = { "shallow-water-3d", "shallow-water-2d" };

    // ------------------------------------------------------------------ 3D

    internal sealed class Stability3D
    {
        internal required string Name;
        internal required RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D World { get; set; }
        internal RecursiveAI.OnePhysics.Physics3D.ShallowWater Water;
        internal RecursiveAI.OnePhysics.Physics3D.Boat[] Boats = Array.Empty<RecursiveAI.OnePhysics.Physics3D.Boat>();
        internal RecursiveAI.OnePhysics.Physics3D.Body[] Craft = Array.Empty<RecursiveAI.OnePhysics.Physics3D.Body>();
        internal Action<int> Script = _ => { };
        internal int Steps = 900;
        internal double InitialVolume, Added;
        internal double MinX = -0.5, MaxX = 40.5, MinZ = -0.5, MaxZ = 24.5;
    }

    internal static Stability3D Build3D(int threads)
    {
        var w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { ThreadCount = threads, WorldId = 130 });
        Tank3D(w, out var water, out var craft, out var boats);
        var s = new Stability3D { Name = "shallow-water-3d", World = w, Water = water, Craft = craft, Boats = boats };
        s.InitialVolume = water.Volume;
        s.Script = step => Script3D(s, step);
        return s;
    }

    private static void Script3D(Stability3D s, int step)
    {
        // Waves: a pulse of water at the west end every two seconds from the first.
        if (step >= 60 && (step - 60) % 120 == 0) { s.Water.AddVolume(0, 0, 2, 24, 20f); s.Added += 20.0; }
        for (int i = 0; i < s.Boats.Length; i++)
        {
            bool right = (step + 61 * i) / 150 % 2 == 0;
            s.Boats[i].Throttle = i == 0 ? 0.7f : 0.25f;
            s.Boats[i].Steer = right ? 0.5f : -0.5f;
        }
    }

    private static void Tank3D(RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D w, out RecursiveAI.OnePhysics.Physics3D.ShallowWater water, out RecursiveAI.OnePhysics.Physics3D.Body[] craft, out RecursiveAI.OnePhysics.Physics3D.Boat[] boats)
    {
        // A basin 40 m by 24 m: a floor with its top at y = 0, four walls, and a ramp for a beach at the east end.
        w.AddBox(new Vector3F(44f, 1f, 28f), new Vector3D(20, -0.5, 12), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 6f, 26f), new Vector3D(-0.5, 3, 12), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 6f, 26f), new Vector3D(40.5, 3, 12), type: BodyType.Static);
        w.AddBox(new Vector3F(42f, 6f, 1f), new Vector3D(20, 3, -0.5), type: BodyType.Static);
        w.AddBox(new Vector3F(42f, 6f, 1f), new Vector3D(20, 3, 24.5), type: BodyType.Static);
        w.AddBox(new Vector3F(12f, 0.5f, 24f), new Vector3D(34, 0.8, 12), QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), 0.35f), BodyType.Static);
        water = w.CreateShallowWater(new RecursiveAI.OnePhysics.Physics3D.ShallowWaterDef { CellsX = 80, CellsZ = 48, CellSize = 0.5f, Level = 2.0, FloorHeight = -1.0, BottomFriction = 0.01f });

        var c = new List<RecursiveAI.OnePhysics.Physics3D.Body>(); var b = new List<RecursiveAI.OnePhysics.Physics3D.Boat>();
        void Hull(RecursiveAI.OnePhysics.Physics3D.BoatDef def, float l, float beam, float h, float m, double x, double z, float yaw)
        {
            var (hull, boat) = Actuators3DScene.AddHull(w, new Vector3D(x, 2.0, z), def, l, beam, h, m, yaw);
            c.Add(hull); b.Add(boat);
        }

        Hull(RecursiveAI.OnePhysics.Physics3D.BoatDef.Dinghy, 3f, 1.2f, 0.5f, 150f, 8, 8, 0.3f);
        Hull(RecursiveAI.OnePhysics.Physics3D.BoatDef.Barge, 10f, 3f, 1f, 6000f, 22, 16, 0.1f);
        for (int i = 0; i < 10; i++) { w.AddBox(new Vector3F(0.7f, 0.7f, 0.7f), new Vector3D(6 + 3 * (i % 5), 2.2, 18 + 2 * (i / 5)), density: 300f + 50f * i); }
        for (int i = 0; i < 3; i++) { w.AddSphere(0.3f, new Vector3D(14 + 3 * i, 3.0 + 0.5 * i, 5), density: 400f + 200f * i); }
        w.AddBox(new Vector3F(1.5f, 1.5f, 1.5f), new Vector3D(20, 6, 8), density: 900f);
        craft = c.ToArray(); boats = b.ToArray();
    }

    /// <summary>Runs 240 steps of the tank; <paramref name="perStep"/> receives the running hash after each. At step 110 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run3D(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { ThreadCount = threads, WorldId = 132 });
        Tank3D(w, out var water, out var craft, out var boats);
        var s = new Stability3D { Name = "shallow-water-3d", World = w, Water = water, Craft = craft, Boats = boats };
        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        for (int step = 0; step < 240; step++)
        {
            if (step == 110 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D(new RecursiveAI.OnePhysics.Physics3D.WorldOptions3D { ThreadCount = threads, WorldId = 132 });
                w.Restore(snap);
                s.World = w;
                s.Water = w.GetShallowWater(s.Water.RawHandle);
                for (int i = 0; i < s.Craft.Length; i++)
                {
                    var h = s.Craft[i].RawHandle;
                    s.Craft[i] = w.GetBody(new BodyHandle(w.Id, h.Index, h.Generation));
                    if (w.TryGetBoat(s.Craft[i], out var bt)) { s.Boats[i] = bt; }
                }
            }

            s.Script(step);
            w.Step(Dt);
            Mix(w.ComputeHash());
            Mix((ulong)w.WaterEvents.Length);
            // Samples of the surface along the tank's middle, bit for bit.
            for (int ix = 0; ix < 80; ix += 9) { Mix(BitConverter.DoubleToUInt64Bits(s.Water.CellSurface(ix, 24))); }
            Mix(BitConverter.DoubleToUInt64Bits(s.Water.Volume));
            perStep?.Invoke(hash);
        }

        return hash;
    }

    // ------------------------------------------------------------------ 2D

    internal sealed class Stability2D
    {
        internal required string Name;
        internal required RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D World { get; set; }
        internal RecursiveAI.OnePhysics.Physics2D.ShallowWater Water;
        internal RecursiveAI.OnePhysics.Physics2D.Boat[] Boats = Array.Empty<RecursiveAI.OnePhysics.Physics2D.Boat>();
        internal RecursiveAI.OnePhysics.Physics2D.Body[] Craft = Array.Empty<RecursiveAI.OnePhysics.Physics2D.Body>();
        internal Action<int> Script = _ => { };
        internal int Steps = 900;
        internal double InitialVolume, Added;
        internal double MinX = -0.5, MaxX = 40.5;
    }

    internal static Stability2D Build2D(int threads)
    {
        var w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 131 });
        Tank2D(w, out var water, out var craft, out var boats);
        var s = new Stability2D { Name = "shallow-water-2d", World = w, Water = water, Craft = craft, Boats = boats };
        s.InitialVolume = water.Volume;
        s.Script = step => Script2D(s, step);
        return s;
    }

    private static void Script2D(Stability2D s, int step)
    {
        if (step >= 60 && (step - 60) % 120 == 0) { s.Water.AddVolume(0, 2, 0.8f); s.Added += 0.8f; }
        for (int i = 0; i < s.Boats.Length; i++)
        {
            s.Boats[i].Throttle = (i == 0 ? 0.7f : 0.25f) * ((step + 90 * i) / 200 % 2 == 0 ? 1f : -1f);
        }
    }

    private static void Tank2D(RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D w, out RecursiveAI.OnePhysics.Physics2D.ShallowWater water, out RecursiveAI.OnePhysics.Physics2D.Body[] craft, out RecursiveAI.OnePhysics.Physics2D.Boat[] boats)
    {
        w.AddBox(new Vector2F(44f, 1f), new Vector2D(20, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 6f), new Vector2D(-0.5, 3), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 6f), new Vector2D(40.5, 3), type: BodyType.Static);
        w.AddBox(new Vector2F(12f, 0.5f), new Vector2D(34, 0.8), angleRadians: 0.35f, type: BodyType.Static);
        water = w.CreateShallowWater(new RecursiveAI.OnePhysics.Physics2D.ShallowWaterDef { Cells = 160, CellSize = 0.25f, Level = 2.0, FloorHeight = -1.0, BottomFriction = 0.01f });
        var c = new List<RecursiveAI.OnePhysics.Physics2D.Body>(); var b = new List<RecursiveAI.OnePhysics.Physics2D.Boat>();
        void Hull(RecursiveAI.OnePhysics.Physics2D.BoatDef def, float l, float h, float m, double x)
        {
            var (hull, boat) = Actuators2DScene.AddHull(w, new Vector2D(x, 2.0), def, l, h, m);
            c.Add(hull); b.Add(boat);
        }

        Hull(RecursiveAI.OnePhysics.Physics2D.BoatDef.Dinghy, 3f, 0.5f, 150f, 8);
        Hull(RecursiveAI.OnePhysics.Physics2D.BoatDef.Barge, 10f, 1f, 4000f, 24);
        for (int i = 0; i < 8; i++) { w.AddBox(new Vector2F(0.7f, 0.7f), new Vector2D(5 + 2.5 * i, 2.2), density: 300f + 60f * i); }
        for (int i = 0; i < 3; i++) { w.AddCircle(0.3f, new Vector2D(14 + 3 * i, 3.0 + 0.5 * i), density: 400f + 200f * i); }
        w.AddBox(new Vector2F(1.5f, 1.5f), new Vector2D(20, 6), density: 900f);
        craft = c.ToArray(); boats = b.ToArray();
    }

    internal static ulong Run2D(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 133 });
        Tank2D(w, out var water, out var craft, out var boats);
        var s = new Stability2D { Name = "shallow-water-2d", World = w, Water = water, Craft = craft, Boats = boats };
        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        for (int step = 0; step < 240; step++)
        {
            if (step == 110 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { ThreadCount = threads, WorldId = 133 });
                w.Restore(snap);
                s.World = w;
                s.Water = w.GetShallowWater(s.Water.RawHandle);
                for (int i = 0; i < s.Craft.Length; i++)
                {
                    var h = s.Craft[i].RawHandle;
                    s.Craft[i] = w.GetBody(new BodyHandle(w.Id, h.Index, h.Generation));
                    if (w.TryGetBoat(s.Craft[i], out var bt)) { s.Boats[i] = bt; }
                }
            }

            s.Script(step);
            w.Step(Dt);
            Mix(w.ComputeHash());
            Mix((ulong)w.WaterEvents.Length);
            for (int ix = 0; ix < 160; ix += 17) { Mix(BitConverter.DoubleToUInt64Bits(s.Water.CellSurface(ix))); }
            Mix(BitConverter.DoubleToUInt64Bits(s.Water.Volume));
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
