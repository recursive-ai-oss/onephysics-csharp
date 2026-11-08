using System;
using System.Buffers.Binary;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

public class SnapshotRegionAndDesyncTests
{
    [Fact]
    public void FindsSingleNamedVelocityLaneInWorldsAndSnapshots()
    {
        var a = new P3.PhysicsWorld3D(); var body = a.AddSphere(1, new Vector3D(0, 4, 0));
        var b = a.Fork();
        b.Core.Bodies.LinearVelocityOf(body.RawHandle.Index)[1] = 2;
        var report = a.FindDifferences(b);
        Assert.Single(report.Differences);
        Assert.Equal("bodies", report.First!.Section); Assert.Equal("body", report.First.Entity);
        Assert.Equal(body.RawHandle.Index, report.First.Index); Assert.Equal(body.RawHandle.Generation, report.First.Generation);
        Assert.Equal("linear velocity y", report.First.Field); Assert.Contains("a=0", report.ToString()); Assert.Contains("b=2", report.ToString());
        var raw = SnapshotDesync.Compare(a.Snapshot(), b.Snapshot());
        Assert.Single(raw.Differences); Assert.Equal(report.ToString(), raw.ToString());
        Assert.Equal(0, SnapshotDesync.Compare(a.Snapshot(), a.Snapshot()).Count);
    }

    [Fact]
    public void NamedOffsetsCoverOptionsEngineSoftAndBroadphase()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddSphere(1, new Vector3D(0, 4, 0));
        w.AddSoftBall(0.3f, new Vector3D(5, 4, 0), segments: 2);
        var bytes = w.Snapshot(); var fields = SnapshotDesync.Inspect(bytes);
        Assert.Contains(fields, x => x.Section == "broadphase3d" && x.Name == "MinX");
        Assert.Contains(fields, x => x.Section == "engine3d" && x.Entity == "body" && x.Name == "Force[0]");
        Assert.Contains(fields, x => x.Section == "softbody3d" && x.Entity == "softbody" && x.Name == "Vel[0]");
        foreach (var f in new[]
        {
            fields.First(x => x.Section == "options" && x.Name == "Gravity.Y"),
            fields.First(x => x.Section == "engine3d" && x.Entity == "body" && x.Name == "Force[0]"),
            fields.First(x => x.Section == "softbody3d" && x.Entity == "softbody" && x.Name == "Vel[0]"),
        })
        {
            var changed = (byte[])bytes.Clone(); BinaryPrimitives.WriteInt32LittleEndian(changed.AsSpan(f.Offset), BitConverter.SingleToInt32Bits(2f));
            var diff = SnapshotDesync.Compare(bytes, changed);
            Assert.Single(diff.Differences); Assert.Equal(f.Section, diff.First!.Section); Assert.Equal(f.Name, diff.First.Field);
        }
    }

    [Fact]
    public void RegionHashIgnoresOutsideMovementAndClosesJoints2D()
    {
        var w = new P2.PhysicsWorld2D();
        var inside = w.AddCircle(0.3f, new Vector2D(0, 0));
        var linked = w.AddCircle(0.3f, new Vector2D(3, 0));
        var outside = w.AddCircle(0.3f, new Vector2D(30, 0));
        w.AddDistance(inside, linked, inside.Position, linked.Position);
        var min = new Vector2D(-1, -1); var max = new Vector2D(1, 1);
        var region = w.CaptureRegion(min, max); var hash = region.ComputeHash();
        Assert.Contains(region.Fields, f => f.Entity == "body" && f.Index == linked.RawHandle.Index);
        Assert.DoesNotContain(region.Fields, f => f.Entity == "body" && f.Index == outside.RawHandle.Index);
        Assert.Contains(region.Fields, f => f.Entity == "joint");
        var encoded = region.ToArray(); Assert.Equal(encoded, RegionSnapshot.Read(encoded).ToArray());
        outside.Position = new Vector2D(40, 0);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        linked.LinearVelocity = new Vector2F(1, 0);
        Assert.NotEqual(hash, w.ComputeRegionHash(min, max));
        w.RestoreRegion(encoded);
        Assert.Equal(new Vector2F(0, 0), linked.LinearVelocity);
        Assert.Equal(new Vector2D(40, 0), outside.Position);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
    }

    [Fact]
    public void RegionSoftBodiesAreWholeAndPartialRestoreIsAtomic3D()
    {
        var w = new P3.PhysicsWorld3D();
        var inside = w.AddSphere(0.2f, new Vector3D(0, 0, 0));
        var outside = w.AddSphere(0.2f, new Vector3D(20, 0, 0));
        w.AddSoftBall(0.5f, new Vector3D(0, 0, 0), segments: 2);
        var min = new Vector3D(-1, -1, -1); var max = new Vector3D(1, 1, 1);
        var region = w.CaptureRegion(min, max); var hash = region.ComputeHash();
        Assert.Contains(region.Fields, f => f.Entity == "softbody" && f.Name == "Pos[0]");
        outside.LinearVelocity = new Vector3F(3, 0, 0); Assert.Equal(hash, w.ComputeRegionHash(min, max));
        inside.LinearVelocity = new Vector3F(4, 0, 0); Assert.NotEqual(hash, w.ComputeRegionHash(min, max));
        w.RestoreRegion(RegionSnapshot.Read(region.ToArray()));
        Assert.Equal(new Vector3F(0, 0, 0), inside.LinearVelocity); Assert.Equal(new Vector3F(3, 0, 0), outside.LinearVelocity);
        Assert.Equal(hash, w.ComputeRegionHash(min, max));
        var stale = region.ToArray(); inside.Destroy(); var before = w.Snapshot();
        Assert.Throws<SnapshotFormatException>(() => w.RestoreRegion(stale)); Assert.Equal(before, w.Snapshot());
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CaptureRegion(new Vector3D(double.NaN, 0, 0), max));
    }

    [Fact]
    public void SingleFlippedFieldsIdentifyEveryBuiltInSectionAndServiceKind()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new Vector3F(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 0.4, 0));
        w.AddCharacter(new Vector3D(5, 3, 0));
        w.AddVehicle(w.AddBox(new Vector3F(1.8f, 0.5f, 4.2f), new Vector3D(10, 3, 0), density: 400), P3.VehicleDef.Car);
        w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(15, 3, 0));
        w.AddSoftBall(0.3f, new Vector3D(20, 3, 0), segments: 2);
        w.AddFluid(new P3.FluidDef());
        w.Step(1f / 60);
        var original = w.Snapshot(); var fields = SnapshotDesync.Inspect(original);
        var choices = new[]
        {
            fields.First(f => f.Section == "constraints" && f.Entity == "contact" && f.Name == "solver payload[3]"),
            fields.First(f => f.Section == "islands" && f.Name == "removed[i]"),
            fields.First(f => f.Section == "events" && f.Name == "Value"),
            fields.First(f => f.Section == "engine3d" && f.Entity == "character" && f.Name == "CharVx[c]"),
            fields.First(f => f.Section == "engine3d" && f.Entity == "vehicle" && f.Name == "Forward"),
            fields.First(f => f.Section == "engine3d" && f.Entity == "ragdoll" && f.Name == "Strength"),
            fields.First(f => f.Section == "engine3d" && f.Entity == "fluid" && f.Name == "Density"),
            fields.First(f => f.Section == "engine3d" && f.Entity == "joint" && f.Name == "MotorSpeed"),
            fields.First(f => f.Section == "broadphase3d" && f.Name == "MinX"),
        };
        foreach (var f in choices)
        {
            var changed = (byte[])original.Clone();
            if (f.ValueType == "float32") { BinaryPrimitives.WriteInt32LittleEndian(changed.AsSpan(f.Offset), BitConverter.SingleToInt32Bits(0.5f)); }
            else if (f.ValueType == "float64") { BinaryPrimitives.WriteInt64LittleEndian(changed.AsSpan(f.Offset), BitConverter.DoubleToInt64Bits(-100)); }
            else { BinaryPrimitives.WriteInt32LittleEndian(changed.AsSpan(f.Offset), 1); }
            var diff = SnapshotDesync.Compare(original, changed);
            Assert.Single(diff.Differences);
            Assert.Equal(f.Section, diff.First!.Section); Assert.Equal(f.Entity, diff.First.Entity); Assert.Equal(f.Name, diff.First.Field);
        }
    }

    [Fact]
    public void NondefaultConstructionOptionsAndTwoDimensionalSnapshotsAreInspectable()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { BroadphaseMargin = 0.137789 });
        var b = w.AddCircle(0.2f, new Vector2D(0, 0));
        var before = w.Snapshot(); b.LinearVelocity = new Vector2F(0, 1);
        var diff = SnapshotDesync.Compare(before, w.Snapshot());
        Assert.Single(diff.Differences); Assert.Equal("linear velocity y", diff.First!.Field);
        Assert.Single(w.FindDifferences(before).Differences);
    }

    [Fact]
    public void RegionRejectsMalformedInputAndLimitsBeforeMutation()
    {
        var w = new P2.PhysicsWorld2D(); w.AddCircle(0.5f, default);
        var bytes = w.SnapshotRegion(new Vector2D(-1, -1), new Vector2D(1, 1));
        for (int i = 0; i < Math.Min(bytes.Length, 128); i++)
        { int n = i; Assert.Throws<SnapshotFormatException>(() => RegionSnapshot.Read(bytes.AsSpan(0, n))); }
        Assert.Throws<SnapshotFormatException>(() => RegionSnapshot.Read(bytes.AsSpan(0, bytes.Length - 1)));
        Assert.Throws<SnapshotFormatException>(() => RegionSnapshot.Read(bytes, bytes.Length - 1));
        Assert.Throws<SnapshotFormatException>(() => RegionSnapshot.Read(bytes, maximumFields: 0));
        var malformed = (byte[])bytes.Clone(); BinaryPrimitives.WriteInt64LittleEndian(malformed.AsSpan(12), BitConverter.DoubleToInt64Bits(double.NaN));
        var before = w.Snapshot(); Assert.Throws<SnapshotFormatException>(() => w.RestoreRegion(malformed)); Assert.Equal(before, w.Snapshot());
    }

    [Fact]
    public void StaticAnchorsDoNotPullUnrelatedDynamicIslandsIntoRegion()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new Vector2F(100, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddCircle(0.5f, new Vector2D(0, 0.4));
        var far = w.AddCircle(0.5f, new Vector2D(20, 0.4)); w.Step(1f / 60);
        var min = new Vector2D(-1, -1); var max = new Vector2D(1, 1);
        var region = w.CaptureRegion(min, max);
        Assert.DoesNotContain(region.Fields, f => f.Entity == "body" && f.Index == far.RawHandle.Index);
        var hash = region.ComputeHash(); far.LinearVelocity = new Vector2F(3, 4); Assert.Equal(hash, w.ComputeRegionHash(min, max));
    }

    [Fact]
    public void SoftRigidCouplingClosureIncludesLargeBodyWhoseOriginIsOutsideBox()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddSoftBall(0.25f, default, segments: 2);
        var large = w.AddBox(new Vector3F(12, 1, 2), new Vector3D(5, 0, 0));
        w.AddSoftBall(0.25f, new Vector3D(8, 0, 0), segments: 2);
        var region = w.CaptureRegion(new Vector3D(-1, -1, -1), new Vector3D(1, 1, 1));
        Assert.Contains(region.Fields, f => f.Entity == "body" && f.Index == large.RawHandle.Index);
        Assert.Contains(region.Fields, f => f.Entity == "softbody" && f.Index == 1);
    }
}
