using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Common;

public static class ContactSurfaceScene
{
    private static void RawBatch(ContactManifoldBatch batch)
    {
        for (int i = 0; i < batch.Count; i++)
        {
            var c = batch.GetManifold(i);
            c.NormalImpulseLimit = 0.25f;
            if (i % 11 == 0) c.Enabled = false;
        }
    }
    public static ulong Planar(int threads, int width = 0, bool restore = false, Action<ulong>? step = null)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 9402, ThreadCount = threads, SleepEnabled = false, Events = EventMask.All });
        if (width != 0) w.Engine.KernelWidth = width;
        w.ContactMotionEnabled = true;
        w.Core.ContactBatchModifier = RawBatch;
        var floor = w.CreateBody(new() { Type = BodyType.Static });
        var points = new Vector2F[33]; var indices = new int[32];
        for (int i = 0; i < points.Length; i++) points[i] = new(i - 16, 0);
        for (int i = 0; i < indices.Length; i++) indices[i] = i % 2;
        w.CreateChain(floor, points, new P2.Material2D[] { new(0.2f), new(0.8f) }, indices, def: new() { Surface = new() { Velocity = new(0.5f, 0, 0), OneWayDirection = new(0, 1, 0) } });
        for (int i = 0; i < 32; i++)
        {
            var b = i % 3 == 0 ? w.AddCircle(0.4f, at: new(i % 16 * 1.5 - 11, i / 16 * 1.2 + 0.4)) : w.AddBox(new(0.8f, 0.8f), at: new(i % 16 * 1.5 - 11, i / 16 * 1.2 + 0.4));
            b.LinearVelocity = new((i % 3 - 1) * 0.2f, i % 7 == 0 ? 1f : 0);
        }
        for (int i = 0; i < 90; i++)
        {
            if (restore && i == 45) w.Restore(w.Snapshot());
            w.Step(1f / 60); step?.Invoke(w.ComputeHash());
        }
        return w.ComputeHash();
    }
    public static ulong Spatial(int threads, int width = 0, bool restore = false, Action<ulong>? step = null)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 9403, ThreadCount = threads, SleepEnabled = false, Events = EventMask.All });
        if (width != 0) w.Engine.KernelWidth = width;
        w.ContactMotionEnabled = true;
        w.Core.ContactBatchModifier = RawBatch;
        var floor = w.CreateBody(new() { Type = BodyType.Static });
        var shape = w.CreateHeightField(floor, new float[17 * 17], 17, 17, new(1, 1, 1), def: new() { LocalPosition = new(-8, 0, -8), Surface = new() { Velocity = new(0.3f, 0, 0.1f), OneWayDirection = new(0, 1, 0), FrictionAxis = new(1, 0, 0), LongitudinalFriction = 0.05f, TransverseFriction = 1, SpinningFriction = 0.05f } });
        var indices = new int[16 * 16 * 2]; for (int i = 0; i < indices.Length; i++) indices[i] = i / 4 % 2;
        shape.SetMaterials(new P3.Material3D[] { new(0.2f), new(0.8f) }, indices);
        for (int i = 0; i < 32; i++)
        {
            var b = i % 3 == 0 ? w.AddSphere(0.4f, at: new(i % 8 * 1.8 - 6, 0.4, i / 8 * 1.8 - 3)) : w.AddBox(new(0.8f, 0.8f, 0.8f), at: new(i % 8 * 1.8 - 6, 0.4, i / 8 * 1.8 - 3));
            b.LinearVelocity = new((i % 3 - 1) * 0.3f, i % 7 == 0 ? 1f : 0, 0.2f);
            b.AngularVelocity = new(0, i % 3, 0);
        }
        for (int i = 0; i < 90; i++)
        {
            if (restore && i == 45) w.Restore(w.Snapshot());
            w.Step(1f / 60); step?.Invoke(w.ComputeHash());
        }
        return w.ComputeHash();
    }
}
