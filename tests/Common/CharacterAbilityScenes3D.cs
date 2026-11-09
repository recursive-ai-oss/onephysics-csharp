#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Rods;

// 3D scenes for what characters do besides walking on rigid ground: stand on and push soft bodies, walk a rope bridge, swing on a rope, grab ledges and climb, ride vehicles and boats. The hash-matrix
// runs a shortened version of each (snapshotted and restored half-way); the stability runner plays the full 600 steps and reads the scene's own measurements. Scripts use only arithmetic on the step number
// and the state of the world, so the scenes are byte-identical on every platform.
internal static class CharacterAbilityScenes3D
{
    internal const float Dt = 1f / 60f;
    internal const float G = 9.81f;

    internal static readonly string[] Names = ["characters-soft-3d", "characters-ropes-3d", "climbing-3d", "riders-3d"];

    internal sealed class Scene
    {
        internal required string Name;
        internal required PhysicsWorld3D World { get; set; }
        internal int Steps = 600;
        internal Action<int> Script = _ => { };
        /// <summary>The scene's own measurements, updated after every step; names and meanings in docs/spec/scenes.md.</summary>
        internal SortedDictionary<string, double> Metrics = new();
        internal List<CharacterHandle> Characters = new();
        internal Action<int> Observe = _ => { };

        internal Character C(int i) => World.GetCharacter(Characters[i]);

        /// <summary>After the world was replaced by a fork or a restore: the handles stay valid, so only the world changes.</summary>
        internal void Rebind(PhysicsWorld3D w) => World = w;
    }

    private static PhysicsWorld3D NewWorld(int threads, int id) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    private static Body Floor(PhysicsWorld3D w, double top = 0.0) => w.AddBox(new Vector3F(120f, 1f, 120f), new Vector3D(0, top - 0.5, 0), type: BodyType.Static);

    /// <summary>Gravity into a vertical speed that resets on the ground, the way a game drives a character.</summary>
    private sealed class Mover
    {
        internal CharacterHandle H;
        internal float Vx, Vy, Vz;

        internal void Tick(PhysicsWorld3D w)
        {
            var c = w.GetCharacter(H);
            if (c.Mode != CharacterMode.Free) { return; }
            Vy = c.IsGrounded && Vy < 0f ? -1f : Vy - G * Dt;
            c.Move(new Vector3F(Vx * Dt, Vy * Dt, Vz * Dt), Dt);
        }

        internal void Release(Vector3F v) { Vx = v.X; Vy = v.Y; Vz = v.Z; }
    }

    private static double Dist(Vector3D a, Vector3D b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));

    internal static Scene Build(string name, int threads) => name switch
    {
        "characters-soft-3d" => SoftScene(threads),
        "characters-ropes-3d" => RopesScene(threads),
        "climbing-3d" => ClimbingScene(threads),
        "riders-3d" => RidersScene(threads),
        _ => throw new ArgumentException("unknown scene " + name),
    };

    // ------------------------------------------------------------------ characters-soft-3d

    private static Scene SoftScene(int threads)
    {
        var w = NewWorld(threads, 81);
        Floor(w);
        // A jelly pad to walk across, a hammock of cloth to stand on, and a jelly cube to push.
        var pad = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector3F(3f, 0.4f, 3f), 12, 2, 12), new Vector3D(-3, 0.21, 0));
        pad.ParticleRadius = 0.15f;
        w.AddSoftBody(pad);
        const int Nx = 30, Nz = 8;
        var hammock = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(3.0f, 1.2f, Nx, Nz), new Vector3D(4, 1.5, 0));
        hammock.ParticleRadius = 0.09f; hammock.Mass = 400f; hammock.EdgeCompliance = 0f; hammock.BendCompliance = 1e-2f; hammock.LinearDamping = 0.5f;
        var pins = new List<int>();
        for (int j = 0; j <= Nz; j++) { pins.Add(j * (Nx + 1)); pins.Add(j * (Nx + 1) + Nx); }
        hammock.PinnedParticles = pins;
        var cloth = w.AddSoftBody(hammock);
        for (int i = 0; i < cloth.ParticleCount; i++) { var p = cloth.GetPosition(i); cloth.SetPosition(i, new Vector3D(4 + (p.X - 4) * 0.9, p.Y, p.Z)); }
        w.AddSoftBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(-3, 0.31, 6), SoftMaterial.Jelly, 4);

        var s = new Scene { Name = "characters-soft-3d", World = w };
        var walker = new Mover { H = w.AddCharacter(new Vector3D(-4.9, 0.91, 0)).RawHandle };
        var stander = new Mover { H = w.AddCharacter(new Vector3D(4, 1.5 + 0.9 + 0.2, 0)).RawHandle };
        var pd = CharacterDef.Humanoid; pd.PushStrength = 1f;
        var pusher = new Mover { H = w.AddCharacter(new Vector3D(-3, 0.91, 4.4), pd).RawHandle };
        s.Characters.AddRange([walker.H, stander.H, pusher.H]);
        double standerLast = double.NaN, jitter = 0; int jitterN = 0;
        s.Script = step =>
        {
            // The walker crosses the pad and comes back; the pusher walks into the cube.
            walker.Vx = ((step / 200) % 2 == 0) ? 0.9f : -0.9f;
            pusher.Vz = 0.5f;
            walker.Tick(s.World); stander.Tick(s.World); pusher.Tick(s.World);
        };
        s.Observe = step =>
        {
            var c = s.C(1);
            if (step >= s.Steps - 200)
            {
                double y = c.Position.Y;
                if (!double.IsNaN(standerLast)) { jitter += Math.Abs(y - standerLast); jitterN++; }
                standerLast = y;
            }

            double grounded = 0, falls = 0;
            for (int i = 0; i < 3; i++) { var ch = s.C(i); if (ch.IsGrounded) { grounded++; } if (ch.Position.Y < -1.0) { falls++; } }
            s.Metrics["grounded"] = grounded; s.Metrics["falls"] = falls;
            s.Metrics["standerJitter"] = jitterN == 0 ? 0 : jitter / jitterN;
        };
        return s;
    }

    // ------------------------------------------------------------------ characters-ropes-3d

    internal static RodDef Rope(Vector3D from, Vector3D to, int segments, float radius, float young, bool pinEnd = false)
    {
        var pts = new Vector3D[segments + 1];
        for (int i = 0; i <= segments; i++) { pts[i] = from + (to - from) * ((double)i / segments); }
        return new RodDef { Points = pts, Material = new RodMaterial(young, young * 0.4f, radius, 1000f), PinnedParticles = pinEnd ? [0, segments] : [0], Damping = pinEnd ? 2f : 0.1f };
    }

    private static Scene RopesScene(int threads)
    {
        var w = NewWorld(threads, 82);
        Floor(w);
        // A rope bridge: a taut cable between two posts, with a walker going back and forth along it.
        w.AddBox(new Vector3F(1f, 3f, 1f), new Vector3D(-6.7, 1.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 3f, 1f), new Vector3D(6.7, 1.5, 0), type: BodyType.Static);
        var bridge = w.AddRod(Rope(new Vector3D(-6, 3, 0), new Vector3D(6, 3, 0), 48, 0.01f, 2e9f, pinEnd: true));
        // A swinging rope with a rider who climbs it and lets go; a hanging rope walked through.
        var swing = w.AddRod(Rope(new Vector3D(0, 6, -8), new Vector3D(Math64.Sin(0.4) * 3, 6 - Math64.Cos(0.4) * 3, -8), 30, 0.003f, 2e10f));
        var curtain = w.AddRod(Rope(new Vector3D(14, 6, 0), new Vector3D(14, 0.1, 0), 24, 0.01f, 1e8f));
        var s = new Scene { Name = "characters-ropes-3d", World = w };
        var walker = new Mover { H = w.AddCharacter(new Vector3D(-4.5, 3.92, 0)).RawHandle };
        var swinger = new Mover { H = w.AddCharacter(new Vector3D(Math64.Sin(0.4) * 3, 3, -8)).RawHandle };
        var pd = CharacterDef.Humanoid; pd.PushStrength = 1f;
        var pusher = new Mover { H = w.AddCharacter(new Vector3D(12, 0.91, 0), pd).RawHandle };
        s.Characters.AddRange([walker.H, swinger.H, pusher.H]);
        w.GetCharacter(swinger.H).Grab(swing, 30f);
        int bridgeIndex = bridge.Index, swingIndex = swing.Index, curtainIndex = curtain.Index;
        _ = swingIndex; _ = bridgeIndex;
        double deflection = 0, swingTop = 0;
        s.Script = step =>
        {
            walker.Vx = ((step / 220) % 2 == 0) ? 0.8f : -0.8f;
            pusher.Vx = 0.7f;
            var sw = s.World.GetCharacter(swinger.H);
            if (sw.Mode == CharacterMode.Roped)
            {
                if (step >= 150 && step < 210) { sw.Climb(0.02f); }
                if (step >= 330) { swinger.Release(sw.Release()); }
            }

            walker.Tick(s.World); swinger.Tick(s.World); pusher.Tick(s.World);
        };
        s.Observe = step =>
        {
            var cur = s.World.GetRod(curtainIndex);
            for (int i = 0; i <= 24; i++) { deflection = Math.Max(deflection, Math.Abs(cur.GetPosition(i).X - 14)); }
            var c1 = s.C(1);
            swingTop = Math.Max(swingTop, c1.Position.Y);
            double falls = 0;
            for (int i = 0; i < 3; i++) { if (s.C(i).Position.Y < -1.0) { falls++; } }
            s.Metrics["falls"] = falls;
            s.Metrics["curtainDeflection"] = deflection;
            s.Metrics["pusherX"] = s.C(2).Position.X;
            s.Metrics["walkerGrounded"] = s.C(0).IsGrounded ? 1 : 0;
            s.Metrics["swingerLanded"] = s.C(1).Mode == CharacterMode.Free && s.C(1).IsGrounded ? 1 : 0;
            s.Metrics["swingRise"] = swingTop;
        };
        return s;
    }

    // ------------------------------------------------------------------ climbing-3d

    private enum Kind { Box, Mesh, HeightField, Voxel }

    private static void AddLedge(PhysicsWorld3D w, Kind kind, double x, double top)
    {
        switch (kind)
        {
            case Kind.Box:
                w.AddBox(new Vector3F(3f, (float)top, 4f), at: new Vector3D(x, top / 2, 4), type: BodyType.Static);
                break;
            case Kind.Mesh:
            {
                float t = (float)top, x0 = (float)x - 1.5f, x1 = (float)x + 1.5f;
                Vector3F[] v = [new(x0, 0, 2), new(x1, 0, 2), new(x1, 0, 6), new(x0, 0, 6), new(x0, t, 2), new(x1, t, 2), new(x1, t, 6), new(x0, t, 6)];
                w.AddMesh(v, [0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3, 3, 7, 4, 3, 4, 0, 4, 7, 6, 4, 6, 5, 0, 1, 2, 0, 2, 3]);
                break;
            }
            case Kind.HeightField:
            {
                const int N = 41;
                var h = new float[N * N];
                for (int j = 0; j < N; j++) { for (int i = 0; i < N; i++) { h[j * N + i] = j >= 20 ? (float)top : 0f; } }
                w.AddHeightField(h, N, N, new Vector3F(0.1f, 1f, 0.1f), new Vector3D(x - 2, 0, 0));
                break;
            }
            case Kind.Voxel:
            {
                int ny = (int)Math.Round(top / 0.5);
                var occ = new bool[6 * ny * 8];
                Array.Fill(occ, true);
                w.AddVoxelGrid(6, ny, 8, occ, 0.5f, new Vector3D(x - 1.5, 0, 2));
                break;
            }
        }
    }

    private static Scene ClimbingScene(int threads)
    {
        var w = NewWorld(threads, 83);
        Floor(w);
        var s = new Scene { Name = "climbing-3d", World = w };
        var def = CharacterDef.Humanoid; def.CanClimb = true;
        var movers = new List<Mover>();
        void Climber(double x) { var m = new Mover { H = w.AddCharacter(new Vector3D(x, 0.91, 1.65), def).RawHandle }; movers.Add(m); s.Characters.Add(m.H); }
        // Four ledges of 2 m on four kinds of geometry, a wall too tall to grab (the fifth climber must not grab it), and a wide ledge to shimmy along.
        double[] xs = [-16, -11, -6, -1, 4];
        AddLedge(w, Kind.Box, xs[0], 2.0); AddLedge(w, Kind.Mesh, xs[1], 2.0); AddLedge(w, Kind.HeightField, xs[2], 2.0); AddLedge(w, Kind.Voxel, xs[3], 2.0);
        w.AddBox(new Vector3F(3f, 6f, 4f), at: new Vector3D(xs[4], 3, 4), type: BodyType.Static);
        for (int i = 0; i < 5; i++) { Climber(xs[i]); }
        // A climbable wall with a climber, and a wide ledge with a shimmier.
        var wall = w.AddBox(new Vector3F(2f, 4f, 4f), at: new Vector3D(9, 2, 4), type: BodyType.Static);
        wall.GetShapes()[0].Climbable = true;
        Climber(9);
        w.AddBox(new Vector3F(12f, 2f, 4f), at: new Vector3D(22, 1, 4), type: BodyType.Static);
        Climber(20);
        var phase = new int[movers.Count];     // 0 waiting, 1 airborne, 2 hanging, 3 climbing up / walking, 4 done
        var hangFrom = new int[movers.Count];
        double grabs = 0, falseGrabs = 0, onTop = 0, ladderTop = 0;
        s.Script = step =>
        {
            for (int i = 0; i < movers.Count; i++)
            {
                var m = movers[i]; var c = s.World.GetCharacter(m.H);
                if (i < 5)
                {
                    // Jump at step 20 + 3 i, grab on the way up, hang for 40 steps, climb up and walk on.
                    if (phase[i] == 0 && step == 20 + 3 * i) { m.Vy = 7.6f; phase[i] = 1; }
                    if (phase[i] == 1 && c.Mode == CharacterMode.Free && m.Vy > 0f && c.TryGrabLedge(new Vector3F(0, 0, 1)))
                    {
                        phase[i] = 2; hangFrom[i] = step;
                        if (i < 4) { grabs++; } else { falseGrabs++; }
                    }

                    if (phase[i] == 2 && step - hangFrom[i] >= 40) { if (c.ClimbUp()) { phase[i] = 3; m.Vy = 0; m.Vz = 1.2f; } else { c.Release(); phase[i] = 4; } }
                    if (phase[i] == 3 && step - hangFrom[i] >= 100) { m.Vz = 0; }
                }
                else if (i == 5)
                {
                    if (phase[i] == 0 && step == 20) { if (c.TryClimb(new Vector3F(0, 0, 1))) { phase[i] = 2; } }
                    if (phase[i] == 2 && c.Mode == CharacterMode.Climbing) { c.Climb(0.025f); }
                    if (phase[i] == 2 && c.Mode == CharacterMode.Free) { phase[i] = 3; m.Vy = 0; m.Vz = 0.3f; ladderTop = 1; }
                    if (phase[i] == 3 && step > 500) { m.Vz = 0; }
                }
                else
                {
                    // The shimmier hangs, slides along the ledge to the right and back.
                    if (phase[i] == 0 && step == 20) { m.Vy = 7.6f; phase[i] = 1; }
                    if (phase[i] == 1 && c.Mode == CharacterMode.Free && m.Vy > 0f && c.TryGrabLedge(new Vector3F(0, 0, 1))) { phase[i] = 2; hangFrom[i] = step; grabs++; }
                    if (phase[i] == 2 && c.Mode == CharacterMode.Hanging) { c.Move(new Vector3F(((step - hangFrom[i]) / 90) % 2 == 0 ? 0.02f : -0.02f, 0, 0), Dt); }
                    if (phase[i] == 2 && step - hangFrom[i] >= 360 && c.ClimbUp()) { phase[i] = 3; m.Vy = 0; m.Vz = 1f; }
                }

                m.Tick(s.World);
            }
        };
        s.Observe = step =>
        {
            onTop = 0; double falls = 0;
            for (int i = 0; i < movers.Count; i++)
            {
                var c = s.C(i);
                if (i < 4 && c.IsGrounded && c.Position.Y > 2.8) { onTop++; }
                if (c.Position.Y < -1.0) { falls++; }
            }

            s.Metrics["grabs"] = grabs; s.Metrics["falseGrabs"] = falseGrabs; s.Metrics["onTop"] = onTop; s.Metrics["ladderTop"] = ladderTop; s.Metrics["falls"] = falls;
            s.Metrics["ladderOnTop"] = s.C(5).IsGrounded && s.C(5).Position.Y > 4.8 ? 1 : 0;
        };
        return s;
    }

    // ------------------------------------------------------------------ riders-3d

    private static Scene RidersScene(int threads)
    {
        var w = NewWorld(threads, 84);
        w.AddWater(0.0);
        // A quay on the left (its top at y = 0) and a basin on the right with a floor sunk to -8.
        w.AddBox(new Vector3F(60f, 1f, 600f), new Vector3D(-35, -0.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(46f, 1f, 600f), new Vector3D(18, -8.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 8f, 600f), new Vector3D(41.5, -4, 0), type: BodyType.Static);
        var s = new Scene { Name = "riders-3d", World = w };
        // A car with a driver on the quay (a strip of floor 60 m long on the other side of the basin).
        var chassis = w.AddBox(new Vector3F(2f, 0.5f, 4f), at: new Vector3D(-30, 1.2, -20));
        w.AddVehicle(chassis, VehicleDef.Car);
        var driverDef = CharacterDef.Humanoid; driverDef.CanClimb = true; driverDef.ExactPlatformCarry = true;
        var driver = w.AddCharacter(new Vector3D(-30, 3, -20), driverDef);
        driver.Mount(chassis, new Vector3F(-0.4f, 0.5f, 0.2f));
        // A barge with a passenger standing on its deck, and a turning platform with another.
        float length = 20f, beam = 5f, height = 1.5f, mass = 20000f;
        float draft = mass / (1000f * length * beam);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector3D(10, 0, 30));
        w.CreateBox(hull, new Vector3F(beam, height, length), 0f, new ShapeDef { Density = mass / (length * beam * height), LocalPosition = new Vector3F(0f, 0.5f * height - draft, 0f), Friction = 0.2f });
        w.AddBoat(hull, BoatDef.Barge);
        var passengerDef = CharacterDef.Humanoid; passengerDef.ExactPlatformCarry = true;
        double deckTop = 0.5 * height - draft + 0.5 * height;
        var passenger = w.AddCharacter(new Vector3D(10.5, deckTop + 0.9, 33), passengerDef);
        var platform = w.AddBox(new Vector3F(8f, 0.5f, 8f), new Vector3D(-15, 0.25, 30), type: BodyType.Kinematic);
        var onPlatform = w.AddCharacter(new Vector3D(-13, 1.41, 31), passengerDef);
        s.Characters.AddRange([driver.RawHandle, passenger.RawHandle, onPlatform.RawHandle]);
        var hullHandle = hull.RawHandle; var platformHandle = platform.RawHandle; var chassisHandle = chassis.RawHandle;
        var pass = new Mover { H = passenger.RawHandle }; var plat = new Mover { H = onPlatform.RawHandle };
        double seatError = 0, deckDrift = 0, platformDrift = 0;
        double l0x = double.NaN, l0z = double.NaN, p0x = double.NaN, p0z = double.NaN;
        s.Script = step =>
        {
            var ww = s.World;
            if (ww.TryGetVehicle(ww.GetBody(chassisHandle), out var cv)) { cv.Drive(step < 250 ? 0.35f : 0f, 0.3f * Math32.Sin(step * 0.03f), step < 250 ? 0f : 0.6f); }
            if (ww.TryGetBoat(ww.GetBody(hullHandle), out var bg)) { bg.Throttle = 0.5f; bg.Steer = 0.3f * Math32.Sin(step * 0.02f); }
            var pf = ww.GetBody(platformHandle);
            pf.AngularVelocity = new Vector3F(0, 0.4f, 0); pf.LinearVelocity = new Vector3F(0.4f * Math32.Sin(step * 0.04f), 0, 0.3f * Math32.Cos(step * 0.04f));
            pass.Tick(ww); plat.Tick(ww);
        };
        s.Observe = step =>
        {
            var ww = s.World;
            var ch = ww.GetBody(chassisHandle);
            var want = ch.Position + ToVector3D(ch.Rotation.Rotate(new Vector3F(-0.4f, 0.5f, 0.2f)));
            seatError = Math.Max(seatError, Dist(want, s.C(0).Position));
            var h = ww.GetBody(hullHandle); var pl = ww.GetBody(platformHandle);
            Vector3D Local(Body b, Vector3D p) { var l = b.Rotation.Conjugated().Rotate(new Vector3F((float)(p.X - b.Position.X), (float)(p.Y - b.Position.Y), (float)(p.Z - b.Position.Z))); return new Vector3D(l.X, l.Y, l.Z); }
            if (step == 60) { var a = Local(h, s.C(1).Position); l0x = a.X; l0z = a.Z; var b2 = Local(pl, s.C(2).Position); p0x = b2.X; p0z = b2.Z; }
            if (step > 60)
            {
                var a = Local(h, s.C(1).Position); deckDrift = Math.Max(deckDrift, Math.Sqrt((a.X - l0x) * (a.X - l0x) + (a.Z - l0z) * (a.Z - l0z)));
                var b2 = Local(pl, s.C(2).Position); platformDrift = Math.Max(platformDrift, Math.Sqrt((b2.X - p0x) * (b2.X - p0x) + (b2.Z - p0z) * (b2.Z - p0z)));
            }

            double falls = 0;
            for (int i = 1; i < 3; i++) { if (s.C(i).Position.Y < -1.0) { falls++; } }
            s.Metrics["seatError"] = seatError; s.Metrics["deckDrift"] = deckDrift; s.Metrics["platformDrift"] = platformDrift; s.Metrics["falls"] = falls;
            s.Metrics["driverSpeed"] = Math.Sqrt(s.C(0).Velocity.LengthSquared());
        };
        return s;
    }

    private static Vector3D ToVector3D(Vector3F v) => new(v.X, v.Y, v.Z);

    // ------------------------------------------------------------------ the hash-matrix and smoke run

    /// <summary>Runs <paramref name="steps"/> steps; <paramref name="perStep"/> receives the running hash after each (the world hash and every character's mode and position). At half-way the world is snapshotted and restored into a fork when asked.</summary>
    internal static ulong Run(string name, int threads, bool restoreHalfway, int steps = 160, Action<ulong>? perStep = null)
    {
        var s = Build(name, threads);
        var w = s.World;
        ulong acc = 14695981039346656037UL;
        for (int i = 0; i < steps; i++)
        {
            if (restoreHalfway && i == steps / 2) { var snap = w.Snapshot(); w = w.Fork(); w.Restore(snap); s.Rebind(w); }
            s.Script(i);
            w.Step(Dt);
            s.Observe(i);
            ulong h = w.ComputeHash();
            for (int c = 0; c < s.Characters.Count; c++)
            {
                var ch = s.C(c);
                h = h * 1099511628211UL ^ (ulong)ch.Mode;
                h = h * 1099511628211UL ^ (ulong)BitConverter.DoubleToInt64Bits(ch.Position.Y);
            }

            acc = (acc ^ h) * 1099511628211UL;
            perStep?.Invoke(h);
        }

        return acc;
    }
}
