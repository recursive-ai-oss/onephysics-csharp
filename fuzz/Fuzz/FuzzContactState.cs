using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class FuzzContactState
{
    public static byte[] Seed(int dimension)
    {
        if (dimension == 2)
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId = 9902 }); w.ContactMotionEnabled = true;
            var g = w.CreateBody(new() { Type = BodyType.Static });
            w.CreateChain(g, new Vector2F[] { new(-4, 0), new(0, 0), new(4, 0) }, new P2.Material2D[] { new(0), new(1) }, new[] { 0, 1 }, def: new() { Surface = new() { Velocity = new(1, 0, 0), OneWayDirection = new(0, 1, 0) } });
            w.AddBox(new(1, 1), at: new(0, 0.5)); w.Step(1f / 60); return w.Snapshot();
        }
        var v = new P3.PhysicsWorld3D(new() { WorldId = 9903 }); v.ContactMotionEnabled = true;
        var floor = v.CreateBody(new() { Type = BodyType.Static });
        var shape = v.CreateHeightField(floor, new float[9], 3, 3, new(1, 1, 1), def: new() { LocalPosition = new(-1, 0, -1), Surface = new() { Velocity = new(1, 0, 0), OneWayDirection = new(0, 1, 0), FrictionAxis = new(1, 0, 0), LongitudinalFriction = 0.1f, TransverseFriction = 1, SpinningFriction = 0.2f } });
        shape.SetMaterials(new P3.Material3D[] { new(0.1f), new(1) }, new[] { 0, 0, 1, 1, 0, 0, 1, 1 });
        v.AddBox(new(1, 1, 1), at: new(0, 0.5, 0)); v.Step(1f / 60); return v.Snapshot();
    }
    public static void Run(byte[] bytes)
    {
        if (bytes.Length > 65536) return;
        int dimension = bytes.Length >= 12 ? BinaryPrimitives.ReadInt32LittleEndian(bytes.AsSpan(8)) : 2;
        if (dimension == 3)
        {
            var world = new P3.PhysicsWorld3D(new() { WorldId = 9903 }); byte[] original = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!world.Snapshot().AsSpan().SequenceEqual(original)) throw new InvalidOperationException("Rejected contact state changed the spatial world."); return; }
            if (!world.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Accepted spatial contact state changed bytes.");
        }
        else
        {
            var world = new P2.PhysicsWorld2D(new() { WorldId = 9902 }); byte[] original = world.Snapshot();
            try { world.Restore(bytes); }
            catch (SnapshotFormatException) { if (!world.Snapshot().AsSpan().SequenceEqual(original)) throw new InvalidOperationException("Rejected contact state changed the planar world."); return; }
            if (!world.Snapshot().AsSpan().SequenceEqual(bytes)) throw new InvalidOperationException("Accepted planar contact state changed bytes.");
        }
    }
}
