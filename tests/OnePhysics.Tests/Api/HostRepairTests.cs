using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

public class HostRepairTests
{
    [Fact]
    public void RegionRepairCanonicalizesFullHistory2D()
    {
        var a = new P2.PhysicsWorld2D();
        a.AddBox(new Vector2F(30, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var body = a.AddCircle(0.4f, new Vector2D(0, 4));
        a.AddCircle(0.4f, new Vector2D(20, 4));
        for (int i = 0; i < 20; i++) { a.Step(1f / 60); }
        var b = a.Fork();
        var archive = a.CaptureRegion(new Vector2D(-2, -2), new Vector2D(2, 10));
        var peer = new P2.Body(b, body.RawHandle);
        peer.Position = new Vector2D(3, 7);
        b.RestoreRegion(archive);
        Assert.NotEqual(a.ComputeHash(), b.ComputeHash());
        a.CanonicalizeBroadphase(); b.CanonicalizeBroadphase();
        Assert.Equal(a.Snapshot(), b.Snapshot());
        var canonical = a.Snapshot(); a.CanonicalizeBroadphase(); Assert.Equal(canonical, a.Snapshot());
        for (int i = 0; i < 300; i++)
        {
            a.Step(1f / 60); b.Step(1f / 60);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
        Assert.Equal(a.Snapshot(), b.Snapshot());
    }

    [Fact]
    public void RegionRepairCanonicalizesFullHistory3D()
    {
        var a = new P3.PhysicsWorld3D();
        a.AddBox(new Vector3F(30, 1, 30), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var body = a.AddSphere(0.4f, new Vector3D(0, 4, 0));
        a.AddSphere(0.4f, new Vector3D(20, 4, 0));
        for (int i = 0; i < 20; i++) { a.Step(1f / 60); }
        var b = a.Fork();
        var archive = a.CaptureRegion(new Vector3D(-2, -2, -2), new Vector3D(2, 10, 2));
        var peer = new P3.Body(b, body.RawHandle);
        peer.Position = new Vector3D(3, 7, 0);
        b.RestoreRegion(archive);
        Assert.NotEqual(a.ComputeHash(), b.ComputeHash());
        a.CanonicalizeBroadphase(); b.CanonicalizeBroadphase();
        Assert.Equal(a.Snapshot(), b.Snapshot());
        var canonical = a.Snapshot(); a.CanonicalizeBroadphase(); Assert.Equal(canonical, a.Snapshot());
        for (int i = 0; i < 300; i++)
        {
            a.Step(1f / 60); b.Step(1f / 60);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
        Assert.Equal(a.Snapshot(), b.Snapshot());
    }

    [Fact]
    public void WorldBoundaryIncludesFixedBridgeEndpoints()
    {
        const double bound = 2097152;
        var a = new P2.PhysicsWorld2D(); var b = new P3.PhysicsWorld3D();
        foreach (double x in new[] { -bound, bound })
        {
            Assert.Equal(x, a.CreateBody(new P2.BodyDef { Position = new Vector2D(x, 0) }).Position.X);
            Assert.Equal(x, b.CreateBody(new P3.BodyDef { Position = new Vector3D(x, 0, 0) }).Position.X);
            Assert.False(FixedBridge.IsInRange(x));
            long raw = (long)(x * 4294967296.0);
            Assert.Equal(x, raw / 4294967296.0);
            Assert.Equal(raw, (long)((raw / 4294967296.0) * 4294967296.0));
        }
        Assert.Throws<ArgumentOutOfRangeException>(() => a.CreateBody(new P2.BodyDef { Position = new Vector2D(bound + 1, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.CreateBody(new P3.BodyDef { Position = new Vector3D(bound + 1, 0, 0) }));
    }

    [Fact]
    public void ShiftOriginHasNoSteadyStateAllocationAndRejectsHalfGrid()
    {
        var a = new P2.PhysicsWorld2D(); a.AddCircle(1);
        var b = new P3.PhysicsWorld3D(); b.AddSphere(1);
        for (int i = 0; i < 100; i++) { a.ShiftOrigin(new Vector2D(1, 0)); a.ShiftOrigin(new Vector2D(-1, 0)); b.ShiftOrigin(new Vector3D(1, 0, 0)); b.ShiftOrigin(new Vector3D(-1, 0, 0)); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 100; i++) { a.ShiftOrigin(new Vector2D(1, 0)); a.ShiftOrigin(new Vector2D(-1, 0)); b.ShiftOrigin(new Vector3D(1, 0, 0)); b.ShiftOrigin(new Vector3D(-1, 0, 0)); }
        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
        double grid = 1.0 / 2147483648.0;
        a.ShiftOrigin(new Vector2D(grid, 0)); b.ShiftOrigin(new Vector3D(grid, 0, 0));
        var sa = a.Snapshot(); var sb = b.Snapshot();
        Assert.Throws<ArgumentException>(() => a.ShiftOrigin(new Vector2D(grid / 2, 0)));
        Assert.Throws<ArgumentException>(() => b.ShiftOrigin(new Vector3D(grid / 2, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => a.ShiftOrigin(new Vector2D(double.NaN, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.ShiftOrigin(new Vector3D(4194305, 0, 0)));
        Assert.Equal(sa, a.Snapshot()); Assert.Equal(sb, b.Snapshot());
    }

    [Fact]
    public void SlopeInputsAreStoredInRadiansAndSnapshotsPreserveThem()
    {
        var a = new P2.PhysicsWorld2D(); var b = new P3.PhysicsWorld3D();
        float[] angles = { 0f, -0f, 0.1234567f, 0.5625f, 0.9f, (float)(Math.PI / 4), (float)(Math.PI / 2) };
        var ca = new P2.Character[angles.Length]; var cb = new P3.Character[angles.Length];
        int index = 0;
        foreach (float angle in angles)
        {
            var d2 = new P2.CharacterDef { SlopeLimitRadians = angle };
            var d3 = new P3.CharacterDef { SlopeLimitRadians = angle };
            Assert.Equal(BitConverter.SingleToInt32Bits(angle), BitConverter.SingleToInt32Bits(d2.SlopeLimitRadians));
            Assert.Equal(BitConverter.SingleToInt32Bits(angle), BitConverter.SingleToInt32Bits(d3.SlopeLimitRadians));
            ca[index] = a.AddCharacter(new Vector2D(index * 3, 4), d2); cb[index] = b.AddCharacter(new Vector3D(index * 3, 4, 0), d3);
            index++;
        }
        var sa = a.Snapshot(); var sb = b.Snapshot();
        a.Restore(sa); b.Restore(sb);
        Assert.Equal(sa, a.Snapshot()); Assert.Equal(sb, b.Snapshot());
        for (int i = 0; i < angles.Length; i++)
        {
            Assert.Equal(BitConverter.SingleToInt32Bits(angles[i]), BitConverter.SingleToInt32Bits(ca[i].SlopeLimitRadians));
            Assert.Equal(BitConverter.SingleToInt32Bits(angles[i]), BitConverter.SingleToInt32Bits(cb[i].SlopeLimitRadians));
        }
        var fa = a.Fork(); var fb = b.Fork();
        for (int i = 0; i < 3; i++) { a.Step(1f / 60); fa.Step(1f / 60); b.Step(1f / 60); fb.Step(1f / 60); Assert.Equal(a.ComputeHash(), fa.ComputeHash()); Assert.Equal(b.ComputeHash(), fb.ComputeHash()); }
        Assert.Throws<ArgumentOutOfRangeException>(() => a.AddCharacter(default, new P2.CharacterDef { SlopeLimitRadians = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => b.AddCharacter(default, new P3.CharacterDef { SlopeLimitRadians = -0.1f }));
    }
}
