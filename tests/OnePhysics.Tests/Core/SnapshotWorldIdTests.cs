using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Core;

/// <summary>A world's id is a handle namespace, not state: the same world saved under different ids gives the same bytes.</summary>
public sealed class SnapshotWorldIdTests
{
    private static P2.PhysicsWorld2D World2(int id)
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = id });
        w.AddBox(new Vector2F(30, 1), at: new Vector2D(0, -1), type: BodyType.Static);
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector2F(1, 1), at: new Vector2D(-5 + 1.05 * (i % 6), 0.8 + 1.3 * (i / 6))); }
        return w;
    }

    private static P3.PhysicsWorld3D World3(int id)
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = id });
        w.AddBox(new Vector3F(30, 1, 30), at: new Vector3D(0, -1, 0), type: BodyType.Static);
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(-5 + 1.05 * (i % 6), 0.8 + 1.3 * (i / 6), 0)); }
        return w;
    }

    [Fact]
    public void SnapshotBytesDoNotDependOnTheWorldId2D()
    {
        var a = World2(5); var b = World2(901);
        for (int s = 0; s < 90; s++)
        {
            a.Step(1f / 60); b.Step(1f / 60);
            if (s % 6 == 0) { Assert.True(a.Snapshot().AsSpan().SequenceEqual(b.Snapshot()), "step " + s); }
        }
    }

    [Fact]
    public void SnapshotBytesDoNotDependOnTheWorldId3D()
    {
        var a = World3(5); var b = World3(901);
        for (int s = 0; s < 90; s++)
        {
            a.Step(1f / 60); b.Step(1f / 60);
            if (s % 6 == 0) { Assert.True(a.Snapshot().AsSpan().SequenceEqual(b.Snapshot()), "step " + s); }
        }
    }

    [Fact]
    public void RestoredEventsBelongToTheRestoringWorld()
    {
        var a = World2(5);
        for (int s = 0; s < 200 && a.Events.Length == 0; s++) { a.Step(1f / 60); }
        var snap = a.Snapshot();
        var b = new P2.PhysicsWorld2D(new() { WorldId = 77 });
        b.Restore(snap);
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        var ra = a.Events.ToArray(); var rb = b.Events.ToArray();
        Assert.Equal(ra.Length, rb.Length);
        Assert.NotEmpty(rb);
        for (int i = 0; i < rb.Length; i++)
        {
            Assert.Equal(77, rb[i].BodyA.World);
            Assert.Equal(ra[i].BodyA.Index, rb[i].BodyA.Index);
        }
    }
}
