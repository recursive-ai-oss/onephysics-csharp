#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

// Side-view vehicle scenes: the hash-matrix and smoke scene (cars, a truck, a tank and a motorcycle on a height field with bumps and a ramp, snapshotted and restored half-way) and the four stability scenes
// (vehicles-2d, tracked-2d, motorcycle-2d, ragdoll-pile-2d). Every random choice comes from a seeded generator written here, so the scenes are byte-identical on every platform.
internal static class Vehicles2DScene
{
    internal const float Dt = 1f / 60f;

    /// <summary>A small seeded generator (splitmix64), so scenes do not depend on <c>System.Random</c>.</summary>
    internal sealed class Rng
    {
        private ulong _s;
        internal Rng(ulong seed) { _s = seed; }
        internal ulong Next() { ulong z = (_s += 0x9E3779B97F4A7C15UL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL; return z ^ (z >> 31); }
        internal double Unit() => (Next() >> 11) * (1.0 / 9007199254740992.0);
        internal double Range(double a, double b) => a + (b - a) * Unit();
    }

    /// <summary>Terrain samples every metre: rolling hills (smoothstep between random knots every <paramref name="knot"/> m, amplitude <paramref name="amp"/>), flat for the first <paramref name="flat"/> m.</summary>
    internal static float[] Hills(int samples, ulong seed, float amp, int knot, int flat)
    {
        var rng = new Rng(seed);
        var h = new float[samples];
        int knots = samples / knot + 2;
        var k = new double[knots];
        for (int i = 0; i < knots; i++) { k[i] = rng.Range(-amp, amp); }
        for (int i = 0; i < samples; i++)
        {
            int a = i / knot; double f = (i - a * knot) / (double)knot; f = f * f * (3 - 2 * f);
            double y = k[a] * (1 - f) + k[a + 1] * f;
            double fade = Math.Min(1.0, Math.Max(0.0, (i - flat) / 8.0));
            h[i] = (float)(y * fade);
        }

        return h;
    }

    /// <summary>Adds a triangular bump (height <paramref name="height"/>, half-width <paramref name="half"/> m) centred on sample <paramref name="centre"/>.</summary>
    internal static void Bump(float[] h, int centre, float height, int half)
    {
        for (int i = Math.Max(0, centre - half); i <= Math.Min(h.Length - 1, centre + half); i++) { h[i] += height * (1f - Math.Abs(i - centre) / (float)half); }
    }

    /// <summary>A ramp: rises linearly by <paramref name="rise"/> over <paramref name="run"/> samples from <paramref name="start"/>, stays up for <paramref name="top"/> samples, then falls back over <paramref name="descent"/> samples (1 is a step down: a jump).</summary>
    internal static void Ramp(float[] h, int start, float rise, int run, int top, int descent = -1)
    {
        if (descent < 0) { descent = run; }
        for (int i = start; i < Math.Min(h.Length, start + run + top + descent + 1); i++)
        {
            int u = i - start;
            float add = u < run ? rise * u / run : u < run + top ? rise : rise * (1f - (u - run - top) / (float)descent);
            h[i] += Math.Max(0f, add);
        }
    }

    internal static double HeightAt(float[] h, double x0, double x)
    {
        double u = x - x0;
        if (u <= 0) { return h[0]; }
        int i = (int)u;
        if (i >= h.Length - 1) { return h[^1]; }
        double f = u - i;
        return h[i] * (1 - f) + h[i + 1] * f;
    }

    private static PhysicsWorld2D NewWorld(int threads, int id, bool ragdolls = false)
    {
        var o = new WorldOptions2D { ThreadCount = threads, WorldId = id };
        if (ragdolls) { Ragdoll.ApplyRecommendedOptions(o); }
        return new PhysicsWorld2D(o);
    }

    // ---- hash scene -------------------------------------------------------------------------------

    /// <summary>Runs 300 steps; <paramref name="perStep"/> receives the running hash after each. At step 130 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.
    /// Wheel state (spin, suspension length, slip, force) enters the hash beside the world hash.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = NewWorld(threads, 52);
        var h = Hills(400, 7, 0.25f, 7, 12);
        Bump(h, 60, 0.15f, 2); Bump(h, 64, 0.12f, 2); Ramp(h, 120, 0.5f, 8, 3, 8); Bump(h, 200, 0.2f, 3);
        w.AddHeightField(h, 1f, new Vector2D(0, 0), 0f, new Material2D(0.8f, 0f, 0f));
        w.AddHeightField(h, 1f, new Vector2D(500, 0), 0f, new Material2D(0.8f, 0f, 0f));
        // A few crates to drive into.
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector2F(0.6f, 0.6f), new Vector2D(35 + 0.7 * (i % 2), 0.31 + 0.62 * (i / 2) + HeightAt(h, 0, 35)), density: 150f); }

        var lsd = VehicleDef.Car;
        lsd.Differentials[0].Ratio = 3.8f;
        lsd.Transmission.Automatic = false;
        var vehicles = new Vehicle[5];
        vehicles[0] = w.AddCar(new Vector2D(3, HeightAt(h, 0, 3)));
        vehicles[1] = w.AddVehicle(new Vector2D(500 + 3, 0), lsd, new Vector2F(3.2f, 0.8f), 1500f);
        vehicles[2] = w.AddTruck(new Vector2D(500 + 20, HeightAt(h, 0, 20)));
        vehicles[3] = w.AddTank(new Vector2D(500 + 40, HeightAt(h, 0, 40)));
        vehicles[4] = w.AddMotorcycle(new Vector2D(500 + 70, HeightAt(h, 0, 70)));
        vehicles[1].Gear = 2;

        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        for (int step = 0; step < 300; step++)
        {
            if (step == 130 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = NewWorld(threads, 52);
                w.Restore(snap);
                vehicles = w.GetVehicles();
            }

            for (int i = 0; i < vehicles.Length; i++)
            {
                float pitch = (step / 50 + i) % 3 == 0 ? 0.5f : (step / 50 + i) % 3 == 1 ? -0.3f : 0f;
                vehicles[i].Drive(0.4f + 0.1f * i, pitch, step > 230 ? 0.5f : 0f, step > 250 && i != 2 ? 1f : 0f);
            }

            w.Step(Dt);
            Mix(w.ComputeHash());
            for (int i = 0; i < vehicles.Length; i++)
            {
                Mix(BitConverter.SingleToUInt32Bits(vehicles[i].EngineRpm)); Mix((ulong)(long)vehicles[i].Gear);
                for (int k = 0; k < vehicles[i].WheelCount; k++)
                {
                    var s = vehicles[i].GetWheel(k);
                    Mix(BitConverter.SingleToUInt32Bits(s.AngularVelocity)); Mix(BitConverter.SingleToUInt32Bits(s.SuspensionLength)); Mix(BitConverter.SingleToUInt32Bits(s.SlipRatio));
                    Mix(BitConverter.SingleToUInt32Bits(s.LongitudinalForce)); Mix(BitConverter.SingleToUInt32Bits(s.SpinAngle));
                }
            }

            perStep?.Invoke(hash);
        }

        return hash;
    }

    // ---- stability scenes -------------------------------------------------------------------------

    /// <summary>One stability scene: a world, the vehicles with their scripts and the terrain they drive on.</summary>
    internal sealed class Stability
    {
        internal required string Name;
        internal required PhysicsWorld2D World;
        internal Vehicle[] Vehicles = Array.Empty<Vehicle>();
        internal float[] Terrain = Array.Empty<float>();
        internal double TerrainX0;
        internal float[] WheelRadius = Array.Empty<float>();
        internal double[] StretchStart = Array.Empty<double>();
        internal Action<int> Script = _ => { };
        internal int Steps = 600;
        internal Ragdoll[] Ragdolls = Array.Empty<Ragdoll>();
        internal RagdollDef? RagdollDefinition;
    }

    private const double Stretch = 160.0;

    /// <summary>Eight stretches of rough road of 160 m, one vehicle on each (six cars and two trucks), driven at different throttles with a stretch of braking at the end; crates stand on the road.</summary>
    internal static Stability VehiclesScene(int threads)
    {
        var w = NewWorld(threads, 53);
        const int N = 8;
        var h = new float[(int)(N * Stretch) + 1];
        for (int k = 0; k < N; k++)
        {
            var seg = Hills((int)Stretch, 100 + (ulong)k, 0.35f, 6, 12);
            for (int i = 0; i < (int)Stretch; i++) { h[k * (int)Stretch + i] = seg[i]; }
        }

        for (int k = 0; k < N; k++)
        {
            int o = k * (int)Stretch;
            foreach (int c in new[] { 30, 55, 95, 130 }) { Bump(h, o + c, 0.12f, 2); }
            Ramp(h, o + 70, 0.4f, 5, 2);
        }

        w.AddHeightField(h, 1f, new Vector2D(0, 0), 0f, new Material2D(0.8f, 0f, 0f));
        var vehicles = new Vehicle[N];
        var radius = new float[N];
        var starts = new double[N];
        for (int k = 0; k < N; k++)
        {
            double x0 = k * Stretch + 5;
            starts[k] = x0;
            vehicles[k] = k % 4 == 3 ? w.AddTruck(new Vector2D(x0, HeightAt(h, 0, x0))) : w.AddCar(new Vector2D(x0, HeightAt(h, 0, x0)));
            radius[k] = k % 4 == 3 ? 0.5f : 0.34f;
            for (int c = 0; c < 3; c++)
            {
                double cx = k * Stretch + 105 + 0.7 * c;
                w.AddBox(new Vector2F(0.6f, 0.6f), new Vector2D(cx, HeightAt(h, 0, cx) + 0.31), density: 100f);
            }
        }

        var s = new Stability { Name = "vehicles-2d", World = w, Vehicles = vehicles, Terrain = h, TerrainX0 = 0, WheelRadius = radius, StretchStart = starts };
        s.Script = step =>
        {
            for (int k = 0; k < N; k++)
            {
                float throttle = 0.35f + 0.04f * k;
                float pitch = step > 120 && step < 400 ? 0.3f * (((step - 120) / 40 + k) % 2 == 0 ? 1f : -1f) : 0f;
                vehicles[k].Drive(step < 450 ? throttle : 0f, pitch, step >= 450 ? 0.5f : 0f, 0f);
            }
        };
        return s;
    }

    /// <summary>Four tanks on rough ground (120 m each), full throttle ramping with the steps, then braking.</summary>
    internal static Stability TrackedScene(int threads)
    {
        var w = NewWorld(threads, 54);
        const int N = 4; const int L = 120;
        var h = new float[N * L + 1];
        for (int k = 0; k < N; k++)
        {
            var seg = Hills(L, 200 + (ulong)k, 0.5f, 8, 14);
            for (int i = 0; i < L; i++) { h[k * L + i] = seg[i]; }
            Bump(h, k * L + 40, 0.3f, 3); Ramp(h, k * L + 70, 0.6f, 6, 2);
        }

        w.AddHeightField(h, 1f, new Vector2D(0, 0), 0f, new Material2D(0.9f, 0f, 0f));
        var tanks = new Vehicle[N]; var starts = new double[N]; var radius = new float[N];
        for (int k = 0; k < N; k++)
        {
            double x0 = k * L + 6;
            starts[k] = x0;
            tanks[k] = w.AddTank(new Vector2D(x0, HeightAt(h, 0, x0)));
            radius[k] = 0.3f;
        }

        var s = new Stability { Name = "tracked-2d", World = w, Vehicles = tanks, Terrain = h, TerrainX0 = 0, WheelRadius = radius, StretchStart = starts };
        s.Script = step =>
        {
            for (int k = 0; k < N; k++) { tanks[k].Drive(step < 450 ? 0.4f + 0.1f * k : 0f, 0f, step >= 450 ? 1f : 0f); }
        };
        return s;
    }

    /// <summary>Four motorcycles that ride at the end of a straight onto a ramp (1 m rise over 6 m) and jump a gap; an air-control rider keeps each level, then they brake.</summary>
    internal static Stability MotorcycleScene(int threads)
    {
        var w = NewWorld(threads, 55);
        const int N = 4; const int L = 200;
        var h = new float[N * L + 1];
        for (int k = 0; k < N; k++)
        {
            var seg = Hills(L, 300 + (ulong)k, 0.15f, 8, 14);
            for (int i = 0; i < L; i++) { h[k * L + i] = seg[i]; }
            Ramp(h, k * L + 80 + 4 * k, 1.0f, 6, 2, 1);
        }

        w.AddHeightField(h, 1f, new Vector2D(0, 0), 0f, new Material2D(0.9f, 0f, 0f));
        var bikes = new Vehicle[N]; var starts = new double[N]; var radius = new float[N];
        for (int k = 0; k < N; k++)
        {
            double x0 = k * L + 6;
            starts[k] = x0;
            bikes[k] = w.AddMotorcycle(new Vector2D(x0, HeightAt(h, 0, x0)));
            radius[k] = 0.35f;
        }

        var s = new Stability { Name = "motorcycle-2d", World = w, Vehicles = bikes, Terrain = h, TerrainX0 = 0, WheelRadius = radius, StretchStart = starts };
        s.Script = step =>
        {
            for (int k = 0; k < N; k++)
            {
                var body = bikes[k].Chassis;
                bool air = !bikes[k].GetWheel(0).IsInContact && !bikes[k].GetWheel(1).IsInContact;
                float pitch = air ? Math.Clamp(-(1.5f * body.Angle + 0.6f * body.AngularVelocity), -1f, 1f) : 0f;
                bikes[k].Drive(step < 400 ? 0.25f + 0.05f * k : 0f, pitch, step >= 400 ? 1f : 0f);
            }
        };
        return s;
    }

    /// <summary>A pit (floor and two walls, 14 m wide) with 120 humanoid ragdolls dropped in a seeded scatter; every fifth one is powered towards its bind pose at half strength.</summary>
    internal static Stability RagdollPileScene(int threads)
    {
        var w = NewWorld(threads, 56, ragdolls: true);
        var mat = new Material2D(0.8f, 0f, 0f);
        w.AddBox(new Vector2F(40f, 1f), new Vector2D(0, -0.5), type: BodyType.Static, material: mat);
        w.AddBox(new Vector2F(1f, 60f), new Vector2D(-7.5, 29.5), type: BodyType.Static, material: mat);
        w.AddBox(new Vector2F(1f, 60f), new Vector2D(7.5, 29.5), type: BodyType.Static, material: mat);
        var def = RagdollDef.Humanoid;
        var rng = new Rng(77);
        const int N = 120;
        var rs = new Ragdoll[N];
        var pose = new float[def.Parts.Length];
        for (int i = 0; i < N; i++)
        {
            double x = -5.5 + (i % 12) * 1.0 + rng.Range(-0.1, 0.1), y = 1.2 + (i / 12) * 2.2 + rng.Range(0, 0.2);
            rs[i] = w.AddRagdoll(def, new Vector2D(x, y), (float)rng.Range(-0.8, 0.8));
            if (i % 5 == 0) { rs[i].DriveToPose(pose, 0.5f); }
        }

        return new Stability { Name = "ragdoll-pile-2d", World = w, Ragdolls = rs, RagdollDefinition = def, Terrain = Array.Empty<float>() };
    }

    internal static Stability Build(string name, int threads) => name switch
    {
        "vehicles-2d" => VehiclesScene(threads),
        "tracked-2d" => TrackedScene(threads),
        "motorcycle-2d" => MotorcycleScene(threads),
        "ragdoll-pile-2d" => RagdollPileScene(threads),
        _ => throw new ArgumentException("unknown scene " + name),
    };

    internal static readonly string[] Names = { "vehicles-2d", "tracked-2d", "motorcycle-2d", "ragdoll-pile-2d" };
}
