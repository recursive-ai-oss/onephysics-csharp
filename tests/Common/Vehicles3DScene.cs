using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene with wheeled vehicles: a car and a limited-slip car on a height field with a bump, a truck on a triangle mesh with a ramp, a kart crossing a dynamic slab, all driven by scripted throttle, steering, brake and
// handbrake. Shared by the hash-matrix runner and both smoke apps. Wheel state (spin, suspension length, slip, forces) enters the per-step hash beside the world hash, so any platform-dependent value shows up.
internal static class Vehicles3DScene
{
    /// <summary>Runs 240 steps; <paramref name="perStep"/> receives the running hash after each. At step 110 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 51 });
        w.AddBox(new Vector3F(200f, 1f, 200f), new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));

        // A height field (21 x 41 samples of 1 m) with a ridge across it at z = 12, left of the origin.
        var h = new float[21 * 41];
        for (int j = 0; j < 41; j++) { for (int i = 0; i < 21; i++) { h[j * 21 + i] = j == 12 || j == 13 ? 0.12f : 0f; } }
        w.AddHeightField(h, 21, 41, new Vector3F(1f, 1f, 1f), new Vector3D(-30, 0.001, 0));

        // A mesh strip with a ramp (rise 0.5 m over 6 m) and a flat top, right of the origin.
        var v = new Vector3F[2 * 5];
        float[] zs = { 0f, 14f, 20f, 24f, 40f }, ys = { 0f, 0f, 0.5f, 0.5f, 0f };
        for (int k = 0; k < 5; k++) { v[2 * k] = new Vector3F(0f, ys[k], zs[k]); v[2 * k + 1] = new Vector3F(8f, ys[k], zs[k]); }
        var idx = new int[4 * 6];
        for (int k = 0; k < 4; k++) { int a = 2 * k, b = a + 1, c = a + 2, d = a + 3; idx[6 * k] = a; idx[6 * k + 1] = c; idx[6 * k + 2] = b; idx[6 * k + 3] = b; idx[6 * k + 4] = c; idx[6 * k + 5] = d; }
        w.AddMesh(v, idx, new Vector3D(20, 0.002, 0));

        // A dynamic slab on the floor, driven over by the kart.
        w.AddBox(new Vector3F(5f, 0.3f, 12f), new Vector3D(0, 0.15, 5), density: 600f);

        var cars = new Vehicle[4];
        Vehicle Make(double x, double y, double z, VehicleDef def, float mass)
        {
            var size = new Vector3F(1.8f, 0.5f, 4.2f);
            if (mass < 500f) { size = new Vector3F(1f, 0.2f, 1.4f); }
            if (mass > 5000f) { size = new Vector3F(2.2f, 0.8f, 6f); }
            var body = w.AddBox(size, at: new Vector3D(x, y, z), density: mass / (size.X * size.Y * size.Z));
            return w.AddVehicle(body, def);
        }

        var lsd = VehicleDef.Car;
        lsd.Differentials[0].LimitedSlipRatio = 4f;
        lsd.Transmission.Automatic = false;
        cars[0] = Make(-24, 0.75, 1, VehicleDef.Car, 1500f);
        cars[1] = Make(-18, 0.75, 1, lsd, 1500f);
        cars[2] = Make(24, 1.0, 1, VehicleDef.Truck, 8000f);
        cars[3] = Make(0, 0.62, -3, VehicleDef.Kart, 150f);
        cars[1].Gear = 2;

        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        const float dt = 1f / 60f;
        for (int step = 0; step < 240; step++)
        {
            if (step == 110 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 51 });
                w.Restore(snap);
                cars = w.GetVehicles();
            }

            for (int i = 0; i < cars.Length; i++)
            {
                float steer = 0.5f * MathF.Min(1f, step / 60f) * ((step / 70 + i) % 2 == 0 ? 1f : -1f) * (i == 3 ? 0.4f : 0.15f);
                cars[i].Drive(i == 3 ? 1f : 0.7f, steer, step > 180 ? 0.5f : 0f, step > 200 && i != 2 ? 1f : 0f);
            }

            w.Step(dt);
            Mix(w.ComputeHash());
            for (int i = 0; i < cars.Length; i++)
            {
                Mix(BitConverter.SingleToUInt32Bits(cars[i].EngineRpm)); Mix((ulong)cars[i].Gear);
                for (int k = 0; k < cars[i].WheelCount; k++)
                {
                    var s = cars[i].GetWheel(k);
                    Mix(BitConverter.SingleToUInt32Bits(s.AngularVelocity)); Mix(BitConverter.SingleToUInt32Bits(s.SuspensionLength)); Mix(BitConverter.SingleToUInt32Bits(s.SlipRatio));
                    Mix(BitConverter.SingleToUInt32Bits(s.SlipAngle)); Mix(BitConverter.SingleToUInt32Bits(s.LongitudinalForce)); Mix(BitConverter.SingleToUInt32Bits(s.LateralForce));
                }
            }

            perStep?.Invoke(hash);
        }

        return hash;
    }
}
