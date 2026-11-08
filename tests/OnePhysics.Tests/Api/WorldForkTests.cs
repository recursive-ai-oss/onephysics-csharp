using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

public class WorldForkTests
{
    [Fact]
    public void ForkMatchesRestoreForThreeHundredSteps2D()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new Vector2F(40, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var b = w.AddBox(new Vector2F(1, 1), new Vector2D(0, 2));
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); }
        var bytes = w.Snapshot(); var f = w.Fork(); var r = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = w.Id }); r.Restore(bytes);
        Assert.Equal(bytes, f.Snapshot()); Assert.Equal(bytes, w.Snapshot());
        for (int i = 0; i < 300; i++)
        {
            w.Step(1f / 60); f.Step(1f / 60); r.Step(1f / 60);
            Assert.Equal(w.ComputeHash(), f.ComputeHash()); Assert.Equal(r.ComputeHash(), f.ComputeHash());
            Assert.Equal(w.Snapshot(), f.Snapshot()); Assert.Equal(r.Snapshot(), f.Snapshot());
        }
        var frozen = f.Snapshot(); b.LinearVelocity = new Vector2F(3, 4); w.Step(1f / 60);
        Assert.Equal(frozen, f.Snapshot()); var original = w.Snapshot();
        f.AddCircle(1, new Vector2D(10, 5)); f.Step(1f / 60); Assert.Equal(original, w.Snapshot());
    }

    [Fact]
    public void ForkMatchesRestoreAndIsolatesOptionalSections3D()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new Vector3F(80, 1, 80), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 2, 0));
        w.AddCharacter(new Vector3D(5, 2, 0));
        w.AddSoftBall(0.3f, new Vector3D(10, 2, 0), segments: 2);
        w.AddVehicle(w.AddBox(new Vector3F(1.8f, 0.5f, 4.2f), new Vector3D(15, 2, 0), density: 400), P3.VehicleDef.Car);
        w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(20, 2, 0));
        w.AddFluid(new P3.FluidDef { SurfaceOffset = 0.2 });
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); }
        var bytes = w.Snapshot(); var f = w.Fork(); var r = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = w.Id }); r.Restore(bytes);
        Assert.Equal(bytes, f.Snapshot()); Assert.Equal(bytes, w.Snapshot());
        for (int i = 0; i < 300; i++)
        {
            w.Step(1f / 60); f.Step(1f / 60); r.Step(1f / 60);
            Assert.Equal(w.ComputeHash(), f.ComputeHash()); Assert.Equal(r.ComputeHash(), f.ComputeHash());
            Assert.Equal(w.Snapshot(), f.Snapshot()); Assert.Equal(r.Snapshot(), f.Snapshot());
        }
        var frozen = f.Snapshot(); b.LinearVelocity = new Vector3F(3, 4, 5); w.Step(1f / 60);
        Assert.Equal(frozen, f.Snapshot()); var original = w.Snapshot();
        f.AddSphere(1, new Vector3D(30, 5, 0)); f.Step(1f / 60); Assert.Equal(original, w.Snapshot());
    }

    [Fact]
    public void ForkStartsWithNoHandlersAndClonesCustomSections()
    {
        var w = new P2.PhysicsWorld2D();
        var body = w.AddCircle(0.5f, new Vector2D(0, 0.2));
        w.AddBox(new Vector2F(10, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        int callbacks = 0; body.OnCollisionEnter += _ => callbacks++;
        w.Core.AddSection(new TestSection { Force = 3 });
        var fork = w.Fork();
        Assert.Equal(w.Snapshot(), fork.Snapshot());
        fork.Step(1f / 60); Assert.Equal(0, callbacks);
        w.Step(1f / 60); Assert.True(callbacks > 0);
        Assert.Equal(w.ComputeHash(), fork.ComputeHash());
    }


    [Fact]
    public void ForkCopiesGrownShapeCachesAndDoesNotInvalidateBorrowedSourceSpans()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new Vector2F(100, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 32; i++) { w.AddCircle(0.3f, new Vector2D(-30 + i * 2, 3)); }
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); }
        var f = w.Fork();
        for (int i = 0; i < 300; i++) { w.Step(1f / 60); f.Step(1f / 60); Assert.Equal(w.ComputeHash(), f.ComputeHash()); }
        var borrowed = w.Core.Bodies.LinearVelocity;
        var other = w.Fork(); float before = other.Core.Bodies.LinearVelocity[2];
        borrowed[2] = 42;
        Assert.Equal(before, other.Core.Bodies.LinearVelocity[2]);
    }

    private sealed class TestSection : RecursiveAI.OnePhysics.LowLevel.ISnapshotSection
    {
        public float Force;
        public int SectionId => 85;
        public void Write(RecursiveAI.OnePhysics.LowLevel.StateWriter writer)
        { int at = writer.Length; writer.WriteSingle(Force); writer.DescribeField("force", at, 4, "float32"); }
        public void Read(RecursiveAI.OnePhysics.LowLevel.StateReader reader) => Force = reader.ReadSingle();
        public RecursiveAI.OnePhysics.LowLevel.ISnapshotSection Fork(RecursiveAI.OnePhysics.LowLevel.WorldCore destination) => new TestSection { Force = Force };
    }
}
