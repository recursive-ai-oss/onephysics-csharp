using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Physics3D;

// Scenes for boats, hovercraft, gliders and thrusters in 3D: the hash-matrix and smoke scene (a harbour boat, a hovercraft over a pond and a hill, a glider in a gust, a thruster rig, snapshotted and restored half-way) and the three
// stability scenes (boats-3d, hovercraft-3d, gliders-3d). Scripts use only integer arithmetic on the step number and the state of the craft, so the scenes are byte-identical on every platform.
internal static class Actuators3DScene
{
    internal const float Dt = 1f / 60f;

    internal sealed class Stability
    {
        internal required string Name;
        internal required PhysicsWorld3D World { get; set; }
        internal Boat[] Boats = Array.Empty<Boat>();
        internal Hovercraft[] Hovers = Array.Empty<Hovercraft>();
        internal Body[] Craft = Array.Empty<Body>();
        internal Action<int> Script = _ => { };
        internal int Steps = 600;

        /// <summary>After the world was replaced by a fork or a restore: the same craft in the new world.</summary>
        internal void Rebind(PhysicsWorld3D w)
        {
            World = w;
            for (int i = 0; i < Craft.Length; i++)
            {
                var h = Craft[i].RawHandle;
                Craft[i] = w.GetBody(new BodyHandle(w.Id, h.Index, h.Generation));
                if (i < Boats.Length && w.TryGetBoat(Craft[i], out var b)) { Boats[i] = b; }
            }

            if (Hovers.Length != 0)
            {
                var hs = new List<Hovercraft>();
                foreach (var c in Craft) { if (w.TryGetHovercraft(c, out var hv)) { hs.Add(hv); } }
                Hovers = hs.ToArray();
            }
        }
    }

    private static PhysicsWorld3D NewWorld(int threads, int id) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    /// <summary>
    /// A boat's hull: a box of the given length, beam and height with the mass given, its shape lifted so that the body origin sits on the waterline when it floats (the presets put the propeller and foils relative to that),
    /// and the preset's foils and propellers. The hull's centre of mass is where the box's is.
    /// </summary>
    internal static (Body Hull, Boat Boat) AddHull(PhysicsWorld3D w, Vector3D at, BoatDef def, float length, float beam, float height, float mass, float yaw = 0f)
    {
        float draft = mass / (1000f * length * beam);
        var rot = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 1f, 0f), yaw);
        var hull = w.CreateBody(BodyType.Dynamic, at, rot);
        w.CreateBox(hull, new Vector3F(beam, height, length), 0f, new ShapeDef { Density = mass / (length * beam * height), LocalPosition = new Vector3F(0f, 0.5f * height - draft, 0f), Friction = 0.2f });
        return (hull, w.AddBoat(hull, def));
    }

    // ------------------------------------------------------------------ boats-3d

    private static Stability BoatsScene(int threads)
    {
        var w = NewWorld(threads, 61);
        w.AddWater(0.0);
        // A harbour: a floor, four quay walls and a breakwater, all static.
        w.AddBox(new Vector3F(80f, 1f, 80f), new Vector3D(0, -7, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 8f, 62f), new Vector3D(-30.5, -1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 8f, 62f), new Vector3D(30.5, -1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(62f, 8f, 1f), new Vector3D(0, -1, -30.5), type: BodyType.Static);
        w.AddBox(new Vector3F(62f, 8f, 1f), new Vector3D(0, -1, 30.5), type: BodyType.Static);
        w.AddBox(new Vector3F(10f, 8f, 2f), new Vector3D(0, -1, 0), type: BodyType.Static);

        var craft = new List<Body>(); var boats = new List<Boat>();
        void Add(BoatDef def, float l, float b, float h, float m, double x, double z, float yaw)
        {
            var (hull, boat) = AddHull(w, new Vector3D(x, 0, z), def, l, b, h, m, yaw);
            craft.Add(hull); boats.Add(boat);
        }

        Add(BoatDef.Dinghy, 3f, 1.2f, 0.5f, 150f, -20, -20, 0f);
        Add(BoatDef.Dinghy, 3f, 1.2f, 0.5f, 150f, 20, -20, 1.5f);
        Add(BoatDef.Dinghy, 3f, 1.2f, 0.5f, 150f, -20, 20, 3f);
        Add(BoatDef.Speedboat, 6f, 1.8f, 0.8f, 800f, 20, 20, -1.5f);
        Add(BoatDef.Speedboat, 6f, 1.8f, 0.8f, 800f, 0, 20, 0.5f);
        Add(BoatDef.Barge, 20f, 5f, 1.5f, 20000f, 0, -18, 1.57f);
        // Crates floating loose in the basin.
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector3F(0.7f, 0.7f, 0.7f), new Vector3D(-24 + 4 * (i % 4), 0.2, -8 + 5 * (i / 4)), density: 300f); }

        var boatArray = boats.ToArray();
        var s = new Stability { Name = "boats-3d", World = w, Boats = boatArray, Craft = craft.ToArray() };
        float[] throttle = { 0.8f, 0.8f, 0.7f, 0.35f, 0.35f, 0.9f };
        s.Script = step =>
        {
            for (int i = 0; i < s.Boats.Length; i++)
            {
                // Each boat sails with its own throttle and swings its helm from side to side on its own period (3 to 5 s).
                int period = 180 + 30 * (i % 3);
                bool right = (step + 47 * i) / (period / 2) % 2 == 0;
                s.Boats[i].Throttle = throttle[i];
                s.Boats[i].Steer = right ? 0.6f : -0.4f;
            }
        };
        return s;
    }

    // ------------------------------------------------------------------ hovercraft-3d

    /// <summary>A rolling 80 m square of terrain on a 2 m grid with a pond sunk in the middle; heights come from integer arithmetic on the cell.</summary>
    internal static float[] Terrain(int n, out float pondLevel)
    {
        var h = new float[n * n];
        double c = (n - 1) / 2.0;
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                double dx = i - c, dz = j - c;
                double r = Math.Sqrt(dx * dx + dz * dz);
                // Two soft hills (smoothstep bumps) around the pond, which is a bowl 1.2 m deep in the middle.
                double hill1 = Bump(i - 8, j - 30, 7, 1.0), hill2 = Bump(i - 32, j - 10, 8, 0.8);
                double bowl = r < 9 ? -1.2 * (1 - Smooth(r / 9)) : 0;
                h[j * n + i] = (float)(hill1 + hill2 + bowl);
            }
        }

        pondLevel = -0.5f;
        return h;
    }

    private static double Smooth(double t) { t = Math.Min(1, Math.Max(0, t)); return t * t * (3 - 2 * t); }

    private static double Bump(double dx, double dz, double radius, double height) => height * (1 - Smooth(Math.Sqrt(dx * dx + dz * dz) / radius));

    private static Stability HovercraftScene(int threads)
    {
        var w = NewWorld(threads, 62);
        const int n = 41;
        var h = Terrain(n, out float pond);
        w.AddHeightField(h, n, n, new Vector3F(2f, 1f, 2f), new Vector3D(-40, 0, -40), material: new Material3D(0.6f, 0f, 0f));
        // A wall round the terrain, so that nothing drives off the map.
        w.AddBox(new Vector3F(1f, 4f, 84f), new Vector3D(-40.5, 1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1f, 4f, 84f), new Vector3D(40.5, 1, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(84f, 4f, 1f), new Vector3D(0, 1, -40.5), type: BodyType.Static);
        w.AddBox(new Vector3F(84f, 4f, 1f), new Vector3D(0, 1, 40.5), type: BodyType.Static);
        // The pond: water only over the bowl.
        w.AddFluid(new FluidDef { SurfaceOffset = pond, ViscousDamping = 1f, LimitToRegion = true, RegionMin = new Vector3D(-14, -5, -14), RegionMax = new Vector3D(14, 5, 14) });
        var craft = new List<Body>(); var hovers = new List<Hovercraft>();
        void Add(float mass, double x, double z)
        {
            var b = w.CreateBody(BodyType.Dynamic, new Vector3D(x, 1.2, z));
            w.CreateBox(b, new Vector3F(2.2f, 0.4f, 3.4f), 0f, new ShapeDef { Density = mass / (2.2f * 0.4f * 3.4f), Friction = 0.3f });
            var hc = w.AddHovercraft(b, HovercraftDef.ForMass(mass));
            craft.Add(b); hovers.Add(hc);
        }

        Add(250f, -30, -30);
        Add(400f, 30, -30);
        Add(300f, -30, 30);
        var s = new Stability { Name = "hovercraft-3d", World = w, Hovers = hovers.ToArray(), Craft = craft.ToArray() };
        s.Script = step =>
        {
            for (int i = 0; i < s.Hovers.Length; i++)
            {
                s.Hovers[i].Lift = 1f;
                // Spool the fans up over the first second, then drive on a throttle that changes every two seconds and a helm that swings.
                if (step < 60) { s.Hovers[i].Lift = step / 60f; s.Hovers[i].Throttle = 0f; continue; }
                s.Hovers[i].Throttle = (step / 120 + i) % 3 == 0 ? 0.3f : 0.7f;
                s.Hovers[i].Steer = (step / 50 + i) % 2 == 0 ? 0.5f : -0.5f;
            }
        };
        return s;
    }

    // ------------------------------------------------------------------ gliders-3d

    /// <summary>A glider: a fuselage, a wing 0.08 m behind the centre of mass and a tail set at −0.13 rad (see the actuator tests); scaled in area by <paramref name="scale"/> and carrying <paramref name="ballast"/> kg.</summary>
    internal static Body AddGlider(PhysicsWorld3D w, Vector3D at, float scale, float ballast, float speed, float yaw)
    {
        var b = w.CreateBody(BodyType.Dynamic, at, QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 1f, 0f), yaw));
        w.CreateBox(b, new Vector3F(0.2f, 0.2f, 1.5f), 0f, new ShapeDef { Density = 20f + ballast, Aero = AeroSurface.Body(0.1f, 0.03f) });
        var wing = AeroSurface.Wing(1.2f * scale, 8f); wing.CenterOfPressure = new Vector3F(0f, 0f, -0.08f);
        var tail = AeroSurface.Wing(0.25f * scale, 4f); tail.CenterOfPressure = new Vector3F(0f, 0f, -1.2f); tail.Forward = new Vector3F(0f, -0.13f, 1f);
        w.CreateBox(b, new Vector3F(0.02f, 0.02f, 0.02f), 0f, new ShapeDef { Density = 1f, Aero = wing });
        w.CreateBox(b, new Vector3F(0.02f, 0.02f, 0.02f), 0f, new ShapeDef { Density = 1f, Aero = tail });
        var fwd = b.Rotation.Rotate(new Vector3F(0f, 0f, 1f));
        b.LinearVelocity = new Vector3F(fwd.X * speed, -0.8f, fwd.Z * speed);
        return b;
    }

    private static Stability GlidersScene(int threads)
    {
        var w = NewWorld(threads, 63);
        w.AddWind(new Vector3F(2f, 0f, -1f));
        // A gust: a column of stronger wind that the gliders cross.
        w.AddWind(new Vector3F(0f, 0.8f, 6f), new Vector3D(-10, 0, 30), new Vector3D(30, 1000, 60));
        var craft = new List<Body>();
        float[] scale = { 1f, 1.3f, 0.8f, 1f, 1.1f, 0.9f };
        float[] ballast = { 0f, 5f, 0f, 20f, 0f, 10f };
        for (int i = 0; i < scale.Length; i++)
        {
            // The trimmed speed grows with the root of the wing loading (6 m/s at 1.2 kg on 1.2 m²): launch near it, so the first seconds are not a lunge.
            float loading = (1.2f + 0.06f * ballast[i]) / 1.2f / scale[i];
            craft.Add(AddGlider(w, new Vector3D(-25 + 10 * i, 400, 0), scale[i], ballast[i], 6f * MathF.Sqrt(loading), 0.2f * i));
        }

        return new Stability { Name = "gliders-3d", World = w, Craft = craft.ToArray() };
    }

    internal static Stability Build(string name, int threads) => name switch
    {
        "boats-3d" => BoatsScene(threads),
        "hovercraft-3d" => HovercraftScene(threads),
        "gliders-3d" => GlidersScene(threads),
        _ => throw new ArgumentException("unknown actuator scene " + name),
    };

    internal static readonly string[] Names = { "boats-3d", "hovercraft-3d", "gliders-3d" };

    // ------------------------------------------------------------------ hash-matrix and smoke scene

    /// <summary>Runs 240 steps of a composite scene; <paramref name="perStep"/> receives the running hash after each (the world hash and the actuators' own readbacks). At step 110 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = NewWorld(threads, 64);
        w.AddWater(0.0);
        w.AddWind(new Vector3F(1.5f, 0f, -2f));
        w.AddWind(new Vector3F(0f, 1f, 5f), new Vector3D(-40, 0, -10), new Vector3D(-10, 500, 10));
        w.AddBox(new Vector3F(300f, 1f, 300f), new Vector3D(0, -7, 0), type: BodyType.Static);
        const int n = 41;
        var h = Terrain(n, out float pond);
        w.AddHeightField(h, n, n, new Vector3F(2f, 1f, 2f), new Vector3D(40, 0, -40), material: new Material3D(0.6f, 0f, 0f));
        w.AddFluid(new FluidDef { SurfaceOffset = pond, ViscousDamping = 1f, LimitToRegion = true, RegionMin = new Vector3D(66, -5, -14), RegionMax = new Vector3D(94, 5, 14) });

        var (hull, boat) = AddHull(w, new Vector3D(0, 0, 0), BoatDef.Dinghy, 3f, 1.2f, 0.5f, 150f);
        var (hull2, boat2) = AddHull(w, new Vector3D(10, 0, 0), BoatDef.Speedboat, 6f, 1.8f, 0.8f, 800f, 0.7f);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(45, 2, -35));
        w.CreateBox(craft, new Vector3F(2.2f, 0.4f, 3.4f), 0f, new ShapeDef { Density = 300f / (2.2f * 0.4f * 3.4f) });
        var hover = w.AddHovercraft(craft, HovercraftDef.ForMass(300f));
        var glider = AddGlider(w, new Vector3D(-30, 200, 0), 1f, 0f, 13f, 0.4f);
        var rig = w.CreateBody(BodyType.Dynamic, new Vector3D(-20, 60, -30));
        w.CreateBox(rig, new Vector3F(0.6f, 0.1f, 0.6f), 0f, new ShapeDef { Density = 300f });
        var rotorDefs = ThrusterRig.Quadcopter(0.3f, 10.8f * 9.81f / 4f, 0.02f);
        var rotors = new Thruster[rotorDefs.Length];
        for (int i = 0; i < rotors.Length; i++) { rotors[i] = rig.AddThruster(rotorDefs[i]); }
        var bluff = w.CreateBody(BodyType.Dynamic, new Vector3D(-15, 80, 0));
        w.CreateSphere(bluff, 0.4f, new ShapeDef { Density = 50f, Aero = AeroSurface.Body(0.47f) });

        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        void MixF(float f) => Mix(BitConverter.SingleToUInt32Bits(f));
        for (int step = 0; step < 240; step++)
        {
            if (step == 110 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = NewWorld(threads, 64);
                w.Restore(snap);
                hull = w.GetBody(new BodyHandle(w.Id, hull.RawHandle.Index, hull.RawHandle.Generation));
                hull2 = w.GetBody(new BodyHandle(w.Id, hull2.RawHandle.Index, hull2.RawHandle.Generation));
                craft = w.GetBody(new BodyHandle(w.Id, craft.RawHandle.Index, craft.RawHandle.Generation));
                glider = w.GetBody(new BodyHandle(w.Id, glider.RawHandle.Index, glider.RawHandle.Generation));
                rig = w.GetBody(new BodyHandle(w.Id, rig.RawHandle.Index, rig.RawHandle.Generation));
                bluff = w.GetBody(new BodyHandle(w.Id, bluff.RawHandle.Index, bluff.RawHandle.Generation));
                w.TryGetBoat(hull, out boat); w.TryGetBoat(hull2, out boat2); w.TryGetHovercraft(craft, out hover);
                rotors = w.GetThrusters();
            }

            boat.Throttle = 0.9f; boat.Steer = (step / 40) % 2 == 0 ? 0.5f : -0.5f;
            boat2.Throttle = step < 150 ? 0.4f : -0.2f; boat2.Steer = (step / 55) % 2 == 0 ? -0.3f : 0.3f;
            hover.Lift = Math.Min(1f, step / 60f); hover.Throttle = 0.6f; hover.Steer = (step / 70) % 2 == 0 ? 0.4f : -0.4f;
            for (int i = 0; i < rotors.Length; i++) { rotors[i].Throttle = 1f + (i == 0 && step > 100 ? 0.2f : 0f); }
            w.Step(Dt);
            Mix(w.ComputeHash());
            MixF(boat.ForwardSpeed); MixF(boat.RudderAngle); MixF(boat.HullForce.X); MixF(boat.HullForce.Z);
            MixF(boat2.ForwardSpeed); MixF(boat2.HullForce.Y);
            for (int k = 0; k < hover.FanCount; k++) { MixF(hover.GetFanGap(k)); MixF(hover.GetFanForce(k)); }
            foreach (var r in rotors) { MixF(r.Output); MixF(r.Force.Y); }
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
