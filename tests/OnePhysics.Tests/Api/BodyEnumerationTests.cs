using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>BodyCount, Bodies, CharacterCount and Characters: live items in slot order, identical on both worlds, no allocation.</summary>
public class BodyEnumerationTests
{
    [Fact]
    public void BodiesAreTheLiveBodiesInSlotOrder2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 21 });
        Assert.Equal(0, w.BodyCount);
        foreach (var _ in w.Bodies) { Assert.Fail("an empty world has no bodies"); }

        var made = new List<P2.Body>();
        for (int i = 0; i < 5; i++) { made.Add(w.AddBox(new Vector2F(1, 1), at: new Vector2D(i * 3, 0))); }
        made[1].Destroy();
        var player = w.AddCharacter(at: new Vector2D(20, 0));
        Assert.Equal(5, w.BodyCount);

        var seen = new List<P2.Body>();
        foreach (P2.Body b in w.Bodies) { seen.Add(b); }
        Assert.Equal(w.BodyCount, seen.Count);
        // The character took the lowest free slot, the destroyed box's.
        foreach (var b in new[] { made[0], made[2], made[3], made[4] }) { Assert.Contains(b, seen); }
        Assert.DoesNotContain(made[1], seen);
        for (int i = 1; i < seen.Count; i++) { Assert.True(seen[i - 1].RawHandle.Index < seen[i].RawHandle.Index); }

        Assert.Equal(1, w.CharacterCount);
        int n = 0;
        foreach (P2.Character c in w.Characters) { Assert.Equal(player, c); n++; }
        Assert.Equal(1, n);

        long before = GC.GetAllocatedBytesForCurrentThread();
        int count = 0;
        foreach (P2.Body b in w.Bodies) { count += b.IsValid ? 1 : 0; }
        foreach (P2.Character c in w.Characters) { count += c.IsValid ? 1 : 0; }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        Assert.Equal(6, count);
    }

    [Fact]
    public void BodiesAreTheLiveBodiesInSlotOrder3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 22 });
        Assert.Equal(0, w.BodyCount);
        var made = new List<P3.Body>();
        for (int i = 0; i < 5; i++) { made.Add(w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(i * 3, 0, 0))); }
        made[1].Destroy();
        var player = w.AddCharacter(new Vector3D(20, 0, 0));
        Assert.Equal(5, w.BodyCount);

        var seen = new List<P3.Body>();
        foreach (P3.Body b in w.Bodies) { seen.Add(b); }
        Assert.Equal(w.BodyCount, seen.Count);
        foreach (var b in new[] { made[0], made[2], made[3], made[4] }) { Assert.Contains(b, seen); }
        Assert.DoesNotContain(made[1], seen);
        for (int i = 1; i < seen.Count; i++) { Assert.True(seen[i - 1].RawHandle.Index < seen[i].RawHandle.Index); }

        Assert.Equal(1, w.CharacterCount);
        int n = 0;
        foreach (P3.Character c in w.Characters) { Assert.Equal(player, c); n++; }
        Assert.Equal(1, n);

        long before = GC.GetAllocatedBytesForCurrentThread();
        int count = 0;
        foreach (P3.Body b in w.Bodies) { count += b.IsValid ? 1 : 0; }
        foreach (P3.Character c in w.Characters) { count += c.IsValid ? 1 : 0; }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        Assert.Equal(6, count);
    }
}
