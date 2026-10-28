using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>A snapshot taken while bodies overlap a sensor restores into a world that continues with the same hashes, events and bytes.</summary>
public class SensorSnapshot2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D Build()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7, Events = EventMask.All });
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -1), type: BodyType.Static);
        for (int i = 0; i < 12; i++) { w.AddBox(new Vector2F(1, 1), at: new Vector2D((i % 4) - 1.5, 6 + (i / 4 * 1.2))); }
        var zone = w.CreateBody(BodyType.Static, new Vector2D(0, 3.5));
        w.CreateBox(zone, 6, 1, def: new ShapeDef { IsSensor = true });
        return w;
    }

    private static string Events(PhysicsWorld2D w) => string.Join(";", w.Events.ToArray().Select(e => e.Kind + ":" + e.ShapeA.Index + ":" + e.ShapeB.Index));

    [Fact]
    public void SnapshotMidOverlapRoundTripsBitIdentically()
    {
        var w = Build();
        int entered = 0;
        for (int i = 0; i < 200 && entered == 0; i++) { w.Step(Dt); entered += w.GetEvents(EventKind.SensorEnter).Length; }
        Assert.True(entered > 0, "the snapshot must be taken with sensor overlaps on the books");
        byte[] snap = w.Snapshot();
        ulong h0 = w.ComputeHash();
        var hashes = new List<ulong>();
        var events = new List<string>();
        for (int i = 0; i < 150; i++) { w.Step(Dt); hashes.Add(w.ComputeHash()); events.Add(Events(w)); }
        Assert.Contains(events, e => e.Contains("SensorExit"));
        byte[] end = w.Snapshot();

        w.Restore(snap);
        Assert.Equal(h0, w.ComputeHash());
        Assert.Equal(snap, w.Snapshot());
        var fresh = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7, Events = EventMask.All });
        fresh.Restore(snap);
        for (int i = 0; i < 150; i++)
        {
            w.Step(Dt);
            fresh.Step(Dt);
            Assert.Equal(hashes[i], w.ComputeHash());
            Assert.Equal(hashes[i], fresh.ComputeHash());
            Assert.Equal(events[i], Events(w));
            Assert.Equal(events[i], Events(fresh));
        }

        Assert.Equal(end, w.Snapshot());
        Assert.Equal(end, fresh.Snapshot());
    }
}
