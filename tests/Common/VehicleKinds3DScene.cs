using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

// A 3D scene with tracked vehicles and motorcycles: two tanks (one pivoting, one driving up a ramp and braking), a motorcycle in a steady turn and one braking to a stop, all on scripted inputs. Shared by the hash-matrix
// runner and both smoke apps. Wheel and track state and the bikes' lean enter the per-step hash beside the world hash, so any platform-dependent value shows up.
internal static class VehicleKinds3DScene
{
    /// <summary>Runs 300 steps; <paramref name="perStep"/> receives the running hash after each. At step 130 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change the result.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 52 });
        w.AddBox(new Vector3F(400f, 1f, 400f), new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));

        // A 25 % ramp for the second tank.
        float ang = Math32.Atan(0.25f), len = 40f, th = 1f;
        var rot = QuaternionF.CreateFromAxisSinCos(new Vector3F(1f, 0f, 0f), Math32.Sin(-0.5f * ang), Math32.Cos(-0.5f * ang));
        w.AddBox(new Vector3F(20f, th, len), new Vector3D(30, 0.5f * len * Math32.Sin(ang) - 0.5f * th * Math32.Cos(ang), 10f + 0.5f * len * Math32.Cos(ang) + 0.5f * th * Math32.Sin(ang)), rot, BodyType.Static, new Material3D(0.8f, 0f, 0f));

        w.AddTank(new Vector3D(0, 0, 0));
        w.AddTank(new Vector3D(30, 0, -4));
        w.AddMotorcycle(new Vector3D(-20, 0, 0));
        var braker = MotorcycleDef.Road;
        braker.MaxLeanTorque = 3000f;
        w.AddMotorcycle(new Vector3D(-30, 0, 0), braker);
        var vs = w.GetVehicles();

        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        const float dt = 1f / 60f;
        for (int step = 0; step < 300; step++)
        {
            if (step == 130 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 52 });
                w.Restore(snap);
                vs = w.GetVehicles();
            }

            float ramp = Math.Min(1f, step / 60f);
            vs[0].SetInput(step < 150 ? 0f : 0.8f, step < 150 ? 0.25f : -0.15f * ramp);
            vs[1].SetInput(step < 250 ? 1f : 0f, 0f, step >= 250 ? 1f : 0f);
            vs[2].SetInput(Math.Clamp(0.25f * (8f - vs[2].ForwardSpeed) + 0.15f, 0f, 1f), step < 100 ? 0f : (step < 220 ? 0.3f : -0.3f));
            vs[3].SetInput(step < 160 ? Math.Clamp(0.25f * (10f - vs[3].ForwardSpeed) + 0.15f, 0f, 1f) : 0f, 0f, step >= 160 ? 0.8f : 0f);

            w.Step(dt);
            Mix(w.ComputeHash());
            for (int i = 0; i < vs.Length; i++)
            {
                Mix((ulong)vs[i].Kind);
                for (int k = 0; k < vs[i].WheelCount; k++)
                {
                    var s = vs[i].GetWheel(k);
                    Mix(BitConverter.SingleToUInt32Bits(s.AngularVelocity)); Mix(BitConverter.SingleToUInt32Bits(s.SuspensionLength)); Mix(BitConverter.SingleToUInt32Bits(s.SlipRatio));
                    Mix(BitConverter.SingleToUInt32Bits(s.SlipAngle)); Mix(BitConverter.SingleToUInt32Bits(s.LongitudinalForce)); Mix(BitConverter.SingleToUInt32Bits(s.LateralForce));
                }

                if (vs[i].Kind == VehicleKind.Motorcycle) { Mix(BitConverter.SingleToUInt32Bits(vs[i].LeanAngle)); Mix(BitConverter.SingleToUInt32Bits(vs[i].TargetLeanAngle)); }
                else { Mix(BitConverter.SingleToUInt32Bits(vs[i].GetTrackSpeed(0))); Mix(BitConverter.SingleToUInt32Bits(vs[i].GetTrackSpeed(1))); }
            }

            perStep?.Invoke(hash);
        }

        return hash;
    }
}
