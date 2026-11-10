#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using P2 = RecursiveAI.OnePhysics.Physics2D;

// Scenes for swimming characters. swim-3d: sixteen characters, twelve of them wading into a shallow-water lake with a current, swimming across it, diving under a floating log and climbing out on a dock,
// two crossing a buoyancy-volume pool and two crossing a tank of particle fluid. swim-2d: the side-on version with a lake, a log, a dock and a pool. Scripts use only arithmetic on the step number and
// the characters' own state, so the scenes are byte-identical on every platform.
internal static class SwimScenes
{
    internal const float Dt = 1f / 60f;
    internal const double Level = 2.0;                 // lake surface, metres
    internal const double DockFace = 27.0, DockTop = 2.45;
    internal const double LogX = 14.0;
    internal static readonly string[] Names = { "swim-3d", "swim-2d" };

    // One character's script. Phase 0: walk, wade and swim east, diving under the log; 5: pulling up onto the dock; 6: done.
    internal sealed class Swimmer
    {
        internal int Phase;
        internal float Vy;
        internal double Lane;                // z of the character's lane (3D)
        internal double GoalX;               // x at which the character has arrived
        internal bool Dock;                  // climbs out on the dock (otherwise stops at GoalX)
        internal bool Dive;                  // dives under the log
        internal int DoneStep = -1;
        internal double MaxSpeed, MinY = 1e9;
        internal int SwimSteps, DiveSteps;
        internal int StartDelay;             // steps the character waits before it starts (2D, where characters do not queue by lane)
    }

    internal sealed class Scene3D
    {
        internal required P3.PhysicsWorld3D World;
        internal P3.Character[] Characters = Array.Empty<P3.Character>();
        internal Swimmer[] Swimmers = Array.Empty<Swimmer>();
        internal P3.Body Log;
        internal P3.ShallowWater Lake, Pond;
        internal P3.SoftBody Ball;
        internal P3.Rod Rope;
        internal P3.Body Paddle;
        internal int Steps = 3600;
        internal int ParticleCount;
    }

    private static double Clamp(double v, double lo, double hi) => v < lo ? lo : v > hi ? hi : v;

    // ------------------------------------------------------------------ 3D

    internal static Scene3D Build3D(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 133 });
        var s = new Scene3D { World = w };
        // Shore: land at y = 2.4 for x < 4, a ramp down to the lake floor at y = -0.5 by x = 9, the floor, and the dock face at x = 27.
        w.AddBox(new Vector3F(10f, 1f, 16f), new Vector3D(-1, 1.9, 7), type: BodyType.Static);
        double run = 5.0, drop = 2.9, len = Math.Sqrt(run * run + drop * drop), ang = Math.Atan2(drop, run);
        double cx = 4.0 + 0.5 * run - Math.Sin(ang) * 0.5, cy = 2.4 - 0.5 * drop - Math.Cos(ang) * 0.5;
        w.AddBox(new Vector3F((float)len, 1f, 16f), new Vector3D(cx, cy, 7), QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), (float)-ang), BodyType.Static);
        w.AddBox(new Vector3F(26f, 1f, 16f), new Vector3D(22, -1.0, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(6f, 3.45f, 16f), new Vector3D(DockFace + 3, DockTop - 1.725, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(34f, 8f, 1f), new Vector3D(15, 3, -0.5), type: BodyType.Static);
        w.AddBox(new Vector3F(34f, 8f, 1f), new Vector3D(15, 3, 14.5), type: BodyType.Static);
        s.Lake = w.CreateShallowWater(new P3.ShallowWaterDef
        {
            MinX = 0, MinZ = 0, CellsX = 54, CellsZ = 28, CellSize = 0.5f, Level = Level, FloorHeight = -0.6, Damping = 0.2f,
            South = WaterBoundary.Inflow(Level, 0.25f), North = WaterBoundary.Open,
        });
        s.Log = w.AddBox(new Vector3F(0.5f, 0.5f, 12f), new Vector3D(LogX, Level, 7), density: 450f);

        var chars = new List<P3.Character>(); var sw = new List<Swimmer>();
        var def = new P3.CharacterDef { CanClimb = true };
        for (int i = 0; i < 12; i++)
        {
            double z = 1.5 + 0.95 * i;
            chars.Add(w.AddCharacter(new Vector3D(0.8 + 0.4 * (i % 3), 3.35, z), def));
            sw.Add(new Swimmer { Lane = z, GoalX = 28.4, Dock = true, Dive = true });
        }

        // A pool with a buoyancy volume (AddFluid) in a walled box beside the lake; the characters swim its length.
        w.AddBox(new Vector3F(14f, 1f, 8f), new Vector3D(45, -1.0, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 5f, 8f), new Vector3D(37.5, 1.5, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 5f, 8f), new Vector3D(52.5, 1.5, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(14f, 5f, 1f), new Vector3D(45, 1.5, 2.5), type: BodyType.Static);
        w.AddBox(new Vector3F(14f, 5f, 1f), new Vector3D(45, 1.5, 11.5), type: BodyType.Static);
        w.AddFluid(new P3.FluidDef { SurfaceOffset = Level, LimitToRegion = true, RegionMin = new Vector3D(38, -1, 3), RegionMax = new Vector3D(52, 4, 11), Flow = new Vector3F(0f, 0f, 0f) });
        for (int i = 0; i < 2; i++)
        {
            double z = 5 + 2.0 * i;
            chars.Add(w.AddCharacter(new Vector3D(39.0, 2.95, z), def));
            sw.Add(new Swimmer { Lane = z, GoalX = 51.0 });
        }

        // A tank of particle water, a few metres of it, in a walled box.
        w.AddBox(new Vector3F(9f, 1f, 6f), new Vector3D(63, -0.5, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 5f, 6f), new Vector3D(58.5, 2, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 5f, 6f), new Vector3D(67.5, 2, 7), type: BodyType.Static);
        w.AddBox(new Vector3F(9f, 5f, 1f), new Vector3D(63, 2, 4.5), type: BodyType.Static);
        w.AddBox(new Vector3F(9f, 5f, 1f), new Vector3D(63, 2, 9.5), type: BodyType.Static);
        var fluid = w.AddParticleWater(0.25f);
        s.ParticleCount = w.FillFluidBox(fluid, new Vector3D(59.1, 0.1, 5.1), new Vector3D(66.9, 1.9, 8.9));
        for (int i = 0; i < 2; i++)
        {
            double z = 6.0 + 1.0 * i;
            chars.Add(w.AddCharacter(new Vector3D(59.8, 2.95, z), def));
            sw.Add(new Swimmer { Lane = z, GoalX = 66.0 });
        }


        // A pond beside the others with the deformables and the wave maker: a soft ball and a rope float in it, a kinematic paddle makes waves, and contact events are on.
        w.AddBox(new Vector3F(14f, 1f, 14f), new Vector3D(76, -1.0, 6), type: BodyType.Static);
        s.Pond = w.AddShallowWater(69, 0, 83, 12, Level, 0.5f, floor: -0.6);
        var ballMesh = P3.SoftBodyMesh.Sphere(0.4f, 4);
        s.Ball = w.AddSoftBall(0.4f, new Vector3D(74, 3.2, 6), segments: 4);
        var pts = new Vector3D[13];
        for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector3D(72 + 0.5 * i, 2.4, 3); }
        s.Rope = w.AddRod(new P3.RodDef { Points = pts, Material = new RecursiveAI.OnePhysics.Rods.RodMaterial(1e7f, 4e6f, 0.03f, 400f) });
        s.Paddle = w.AddBox(new Vector3F(0.4f, 0.6f, 6f), new Vector3D(70.5, 2.1, 6), type: BodyType.Kinematic);
        w.TrackDeformableContacts = true;
        _ = ballMesh;

        s.Characters = chars.ToArray(); s.Swimmers = sw.ToArray();
        return s;
    }

    /// <summary>Drives every character for one step (before the world step).</summary>
    internal static void Drive3D(Scene3D s, int step)
    {
        const float g = 9.81f;
        // The paddle bobs up and down by a triangle wave of the step number: 0.15 m/s for 40 steps each way.
        s.Paddle.LinearVelocity = new Vector3F(0f, (step / 40 % 2 == 0 ? -0.15f : 0.15f), 0f);
        for (int i = 0; i < s.Characters.Length; i++)
        {
            var c = s.Characters[i]; var m = s.Swimmers[i];
            if (m.Phase == 6) { c.Move(default, Dt); continue; }
            var p = c.Position;
            double zc = Clamp(1.5 * (m.Lane - p.Z), -0.6, 0.6);
            if (c.Mode == CharacterMode.Hanging)
            {
                if (c.ClimbUp()) { m.Phase = 5; }
                continue;
            }

            if (m.Phase == 5)
            {
                // On the dock: step forward and stand.
                m.Vy = c.IsGrounded ? -1f : m.Vy - g * Dt;
                c.Move(new Vector3F(1.2f * Dt, m.Vy * Dt, 0f), Dt);
                if (p.X > m.GoalX - 0.4 && c.IsGrounded) { m.Phase = 6; m.DoneStep = step; }
                continue;
            }

            bool swimming = c.IsSwimming;
            float vx, vy;
            if (swimming)
            {
                double ty;
                bool nearLog = m.Dive && p.X > LogX - 2.6 && p.X < LogX + 2.6;
                if (nearLog) { ty = 0.5; vy = (float)Clamp(2.5 * (ty - p.Y), -1.0, 1.0); }
                else { ty = Level - 0.6; vy = (float)Clamp(2.5 * (ty - p.Y), -0.4, 0.9); }
                vx = 1.1f;
                if (m.Dock && p.X > DockFace - 1.2) { vx = p.X < DockFace - 0.45 ? 0.5f : 0.1f; if (p.X > DockFace - 0.8) { vy = 1.8f; } }
                m.Vy = 0f;
            }
            else
            {
                m.Vy = c.IsGrounded && m.Vy < 0f ? -1f : m.Vy - g * Dt;
                vx = 2.0f; vy = m.Vy;
                if (m.Dock && p.X > DockFace - 1.2) { vx = 0.3f; }
            }

            if (!m.Dock && p.X > m.GoalX) { vx = 0f; if (m.Phase != 6) { m.Phase = 6; m.DoneStep = step; } }
            if (m.Dock && c.CanClimb && c.WaterState != CharacterWaterState.Dry && p.X > DockFace - 0.8) { c.TryGrabLedge(new Vector3F(1f, 0f, 0f)); }
            if (c.Mode == CharacterMode.Hanging) { continue; }
            c.Move(new Vector3F(vx * Dt, vy * Dt, (float)zc * Dt), Dt);
            if (swimming) { m.SwimSteps++; if (p.Y < Level - 1.0) { m.DiveSteps++; } }
            if (p.Y < m.MinY) { m.MinY = p.Y; }
            double speed = c.Velocity.Length();
            if (speed > m.MaxSpeed) { m.MaxSpeed = speed; }
        }
    }

    internal static bool AllDone(Scene3D s)
    {
        foreach (var m in s.Swimmers) { if (m.Phase != 6) { return false; } }
        return true;
    }

    // ------------------------------------------------------------------ 2D

    internal sealed class Scene2D
    {
        internal required P2.PhysicsWorld2D World;
        internal P2.Character[] Characters = Array.Empty<P2.Character>();
        internal Swimmer[] Swimmers = Array.Empty<Swimmer>();
        internal P2.Body Log;
        internal P2.ShallowWater Lake, Pond;
        internal P2.SoftBody Ball;
        internal P2.Rod Rope;
        internal P2.Body Paddle;
        internal int Steps = 3600;
        internal int ParticleCount;
    }

    internal static Scene2D Build2D(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = 134 });
        var s = new Scene2D { World = w };
        w.AddBox(new Vector2F(10f, 1f), new Vector2D(-1, 1.9), type: BodyType.Static);
        double run = 5.0, drop = 2.9, len = Math.Sqrt(run * run + drop * drop), ang = Math.Atan2(drop, run);
        double cx = 4.0 + 0.5 * run - Math.Sin(ang) * 0.5, cy = 2.4 - 0.5 * drop - Math.Cos(ang) * 0.5;
        w.AddBox(new Vector2F((float)len, 1f), new Vector2D(cx, cy), (float)-ang, BodyType.Static);
        w.AddBox(new Vector2F(26f, 1f), new Vector2D(22, -1.0), type: BodyType.Static);
        w.AddBox(new Vector2F(6f, 3.45f), new Vector2D(DockFace + 3, DockTop - 1.725), type: BodyType.Static);
        s.Lake = w.CreateShallowWater(new P2.ShallowWaterDef { MinX = 0, Cells = 54, CellSize = 0.5f, Level = Level, FloorHeight = -0.6, Damping = 0.2f });
        s.Log = w.AddBox(new Vector2F(2f, 0.5f), new Vector2D(LogX, Level), density: 450f);

        var chars = new List<P2.Character>(); var sw = new List<Swimmer>();
        var def = new P2.CharacterDef { CanClimb = true };
        for (int i = 0; i < 8; i++)
        {
            chars.Add(w.AddCharacter(new Vector2D(0.8 - 0.3 * (i % 4), 3.35), def));
            sw.Add(new Swimmer { GoalX = 32.4 - 0.6 * i, Dock = true, Dive = true, StartDelay = 40 * i });
        }

        // A pool beside the lake with a buoyancy volume that flows along x.
        w.AddBox(new Vector2F(14f, 1f), new Vector2D(45, -1.0), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 5f), new Vector2D(37.5, 1.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 5f), new Vector2D(52.5, 1.5), type: BodyType.Static);
        w.AddFluid(new P2.FluidDef { SurfaceOffset = Level, LimitToRegion = true, RegionMin = new Vector2D(38, -1), RegionMax = new Vector2D(52, 4), Flow = new Vector2F(0.25f, 0f) });
        for (int i = 0; i < 2; i++)
        {
            chars.Add(w.AddCharacter(new Vector2D(39.0 + 0.5 * i, 2.95), def));
            sw.Add(new Swimmer { GoalX = 51.0, StartDelay = 60 * i });
        }

        // A tank of particle water.
        w.AddBox(new Vector2F(9f, 1f), new Vector2D(63, -0.5), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 5f), new Vector2D(58.5, 2), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 5f), new Vector2D(67.5, 2), type: BodyType.Static);
        var fluid = w.AddParticleWater(0.125f);
        s.ParticleCount = w.FillFluidBox(fluid, new Vector2D(59.1, 0.1), new Vector2D(66.9, 1.9));
        for (int i = 0; i < 2; i++)
        {
            chars.Add(w.AddCharacter(new Vector2D(59.8 + 0.4 * i, 2.95), def));
            sw.Add(new Swimmer { GoalX = 66.0, StartDelay = 60 * i });
        }


        w.AddBox(new Vector2F(14f, 1f), new Vector2D(76, -1.0), type: BodyType.Static);
        s.Pond = w.AddShallowWater(69, 83, Level, 0.25f, floor: -0.6);
        s.Ball = w.AddSoftCircle(0.4f, new Vector2D(74, 3.2));
        var pts = new Vector2D[13];
        for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector2D(72 + 0.5 * i, 2.4); }
        s.Rope = w.AddRod(new P2.RodDef { Points = pts, Material = new RecursiveAI.OnePhysics.Rods.RodMaterial(1e7f, 4e6f, 0.03f, 400f) });
        s.Paddle = w.AddBox(new Vector2F(0.4f, 0.6f), new Vector2D(70.5, 2.1), type: BodyType.Kinematic);
        w.TrackDeformableContacts = true;

        s.Characters = chars.ToArray(); s.Swimmers = sw.ToArray();
        return s;
    }

    internal static void Drive2D(Scene2D s, int step)
    {
        const float g = 9.81f;
        s.Paddle.LinearVelocity = new Vector2F(0f, (step / 40 % 2 == 0 ? -0.15f : 0.15f));
        for (int i = 0; i < s.Characters.Length; i++)
        {
            var c = s.Characters[i]; var m = s.Swimmers[i];
            if (m.Phase == 6) { continue; }
            if (step < m.StartDelay) { c.Move(default, Dt); continue; }
            var p = c.Position;
            if (c.Mode == CharacterMode.Hanging)
            {
                if (c.ClimbUp()) { m.Phase = 5; }
                continue;
            }

            if (m.Phase == 5)
            {
                m.Vy = c.IsGrounded ? -1f : m.Vy - g * Dt;
                c.Move(new Vector2F(1.2f * Dt, m.Vy * Dt), Dt);
                if (p.X > m.GoalX - 0.5 && c.IsGrounded) { m.Phase = 6; m.DoneStep = step; c.Destroy(); }
                continue;
            }

            bool swimming = c.IsSwimming;
            float vx, vy;
            if (swimming)
            {
                bool nearLog = m.Dive && p.X > LogX - 2.6 && p.X < LogX + 2.6;
                if (nearLog) { vy = (float)Clamp(2.5 * (0.5 - p.Y), -1.0, 1.0); }
                else { vy = (float)Clamp(2.5 * (Level - 0.6 - p.Y), -0.4, 0.9); }
                vx = 1.1f;
                if (m.Dock && p.X > DockFace - 1.2) { vx = p.X < DockFace - 0.45 ? 0.5f : 0.1f; if (p.X > DockFace - 0.8) { vy = 1.8f; } }
                m.Vy = 0f;
            }
            else
            {
                m.Vy = c.IsGrounded && m.Vy < 0f ? -1f : m.Vy - g * Dt;
                vx = 2.0f; vy = m.Vy;
                if (m.Dock && p.X > DockFace - 1.2) { vx = 0.3f; }
            }

            if (!m.Dock && p.X > m.GoalX) { vx = 0f; if (m.Phase != 6) { m.Phase = 6; m.DoneStep = step; c.Destroy(); continue; } }
            if (m.Dock && c.CanClimb && c.WaterState != CharacterWaterState.Dry && p.X > DockFace - 0.8) { c.TryGrabLedge(new Vector2F(1f, 0f)); }
            if (c.Mode == CharacterMode.Hanging) { continue; }
            c.Move(new Vector2F(vx * Dt, vy * Dt), Dt);
            if (swimming) { m.SwimSteps++; if (p.Y < Level - 1.0) { m.DiveSteps++; } }
            if (p.Y < m.MinY) { m.MinY = p.Y; }
            double speed = c.Velocity.Length();
            if (speed > m.MaxSpeed) { m.MaxSpeed = speed; }
        }
    }

    internal static bool AllDone(Scene2D s)
    {
        foreach (var m in s.Swimmers) { if (m.Phase != 6) { return false; } }
        return true;
    }

    // ------------------------------------------------------------------ hash scenes

    private static ulong Mix(ulong hash, ulong x) { hash = (hash ^ x) * 0x100000001B3UL; return hash ^ (hash >> 29); }

    /// <summary>
    /// Runs the 3D scene for 700 steps (the swimmers wade in, swim and begin their dive; the pond's ball, rope and paddle float and make waves; deformable queries and events are mixed in);
    /// <paramref name="perStep"/> receives the running hash after each. At step 350 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/>.
    /// </summary>
    internal static ulong Run3D(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var s = Build3D(threads);
        var w = s.World;
        ulong hash = 0xCBF29CE484222325UL;
        var hits = new P3.DeformableHit3D[8];
        for (int step = 0; step < 700; step++)
        {
            if (step == 350 && restoreHalfway)
            {
                var snap = w.Snapshot();
                var fresh = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = threads, WorldId = 133 });
                fresh.Restore(snap);
                fresh.TrackDeformableContacts = true;
                for (int i = 0; i < s.Characters.Length; i++) { var h = s.Characters[i].RawHandle; s.Characters[i] = fresh.GetCharacter(h); }
                s.Paddle = fresh.GetBody(new BodyHandle(fresh.Id, s.Paddle.RawHandle.Index, s.Paddle.RawHandle.Generation));
                s.Ball = fresh.GetSoftBody(s.Ball.Slot);
                w = fresh; s.World = fresh;
            }

            Drive3D(s, step);
            w.Step(Dt);
            hash = Mix(hash, w.ComputeHash());
            hash = Mix(hash, (ulong)w.CharacterWaterEvents.Length * 31UL + (ulong)w.DeformableContactEvents.Length);
            for (int i = 0; i < s.Characters.Length; i++)
            {
                var c = s.Characters[i];
                hash = Mix(hash, (ulong)c.WaterState + 4UL * BitConverter.SingleToUInt32Bits(c.Immersion));
            }

            if (step % 20 == 0)
            {
                int n = w.RaycastDeformablesAll(new Vector3D(74, 6, 6), new Vector3F(0f, -1f, 0f), hits);
                for (int k = 0; k < n; k++) { hash = Mix(hash, BitConverter.DoubleToUInt64Bits(hits[k].Point.Y) ^ (ulong)hits[k].Index); }
                if (w.ClosestDeformable(new Vector3D(74, 2.5, 6), out var cl)) { hash = Mix(hash, BitConverter.SingleToUInt32Bits(cl.Distance) ^ (ulong)cl.Index); }
            }

            perStep?.Invoke(hash);
        }

        return hash;
    }

    /// <summary>The 2D scene for 700 steps, with the same restore, pond and queries.</summary>
    internal static ulong Run2D(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var s = Build2D(threads);
        var w = s.World;
        ulong hash = 0xCBF29CE484222325UL;
        var hits = new P2.DeformableHit2D[8];
        for (int step = 0; step < 700; step++)
        {
            if (step == 350 && restoreHalfway)
            {
                var snap = w.Snapshot();
                var fresh = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = threads, WorldId = 134 });
                fresh.Restore(snap);
                fresh.TrackDeformableContacts = true;
                for (int i = 0; i < s.Characters.Length; i++) { var h = s.Characters[i].RawHandle; if (fresh.IsCharacterAlive(h)) { s.Characters[i] = fresh.GetCharacter(h); } }
                s.Paddle = fresh.GetBody(new BodyHandle(fresh.Id, s.Paddle.RawHandle.Index, s.Paddle.RawHandle.Generation));
                w = fresh; s.World = fresh;
            }

            Drive2D(s, step);
            w.Step(Dt);
            hash = Mix(hash, w.ComputeHash());
            hash = Mix(hash, (ulong)w.CharacterWaterEvents.Length * 31UL + (ulong)w.DeformableContactEvents.Length);
            for (int i = 0; i < s.Characters.Length; i++)
            {
                if (s.Swimmers[i].Phase == 6) { continue; }
                var c = s.Characters[i];
                hash = Mix(hash, (ulong)c.WaterState + 4UL * BitConverter.SingleToUInt32Bits(c.Immersion));
            }

            if (step % 20 == 0)
            {
                int n = w.RaycastDeformablesAll(new Vector2D(74, 6), new Vector2F(0f, -1f), hits);
                for (int k = 0; k < n; k++) { hash = Mix(hash, BitConverter.DoubleToUInt64Bits(hits[k].Point.Y) ^ (ulong)hits[k].Index); }
                if (w.ClosestDeformable(new Vector2D(74, 2.5), out var cl)) { hash = Mix(hash, BitConverter.SingleToUInt32Bits(cl.Distance) ^ (ulong)cl.Index); }
            }

            perStep?.Invoke(hash);
        }

        return hash;
    }
}
