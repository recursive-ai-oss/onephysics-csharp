using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;
using CharacterDef3 = RecursiveAI.OnePhysics.Physics3D.CharacterDef;
using Body2 = RecursiveAI.OnePhysics.Physics2D.Body;
using Body3 = RecursiveAI.OnePhysics.Physics3D.Body;
using ShapeDef2 = RecursiveAI.OnePhysics.Physics2D.ShapeDef;
using ShapeDef3 = RecursiveAI.OnePhysics.Physics3D.ShapeDef;

namespace RecursiveAI.OnePhysics.Tests.Core;

/// <summary>The snapshot size bound: never below the real size while a pile lands, makes contact and settles; fixed while storage does not grow.</summary>
public class SnapshotBoundTests
{
    private const float Dt = 1f / 60f;
    private static void Note(string line) => TestContext.Current.SendDiagnosticMessage(line);

    private static PhysicsWorld2D Pile2D(int seed, int bodies, int contactCapacity, int pairCapacity)
    {
        var rng = new Random(seed);
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 7, Events = EventMask.All, ContactCapacity = contactCapacity, PairCapacity = pairCapacity });
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < bodies; i++)
        {
            var at = new Vector2D(rng.NextDouble() * 12 - 6, 1 + rng.NextDouble() * 14);
            if (rng.Next(3) == 0) { w.AddCircle(0.2f + 0.3f * (float)rng.NextDouble(), at); }
            else { w.AddBox(new Vector2F(0.4f + (float)rng.NextDouble(), 0.4f + (float)rng.NextDouble()), at); }
        }

        return w;
    }

    private static PhysicsWorld3D Pile3D(int seed, int bodies, int contactCapacity, int pairCapacity)
    {
        var rng = new Random(seed);
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 7, Events = EventMask.All, ContactCapacity = contactCapacity, PairCapacity = pairCapacity });
        w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < bodies; i++)
        {
            var at = new Vector3D(rng.NextDouble() * 6 - 3, 1 + rng.NextDouble() * 10, rng.NextDouble() * 6 - 3);
            if (rng.Next(3) == 0) { w.AddSphere(0.2f + 0.3f * (float)rng.NextDouble(), at); }
            else { w.AddBox(new Vector3F(0.4f + (float)rng.NextDouble(), 0.4f + (float)rng.NextDouble(), 0.4f + (float)rng.NextDouble()), at); }
        }

        return w;
    }

    [Theory]
    [InlineData(1, 60, 0, 0)]
    [InlineData(2, 150, 0, 0)]
    [InlineData(3, 150, 2000, 2000)]
    public void Bound2DCoversTheSnapshotFromCreationThroughFirstContactToSettling(int seed, int bodies, int contacts, int pairs)
    {
        var w = Pile2D(seed, bodies, contacts, pairs);
        int first = w.GetSnapshotSizeBound(), worstSize = 0, jumps = 0, prevSize = w.GetSnapshotSize();
        Assert.True(prevSize <= first);
        for (int step = 0; step < 700; step++)
        {
            w.Step(Dt);
            int size = w.GetSnapshotSize(), bound = w.GetSnapshotSizeBound();
            Assert.True(size <= bound, "step " + step + ": size " + size + " > bound " + bound);
            if (size > prevSize + prevSize / 2) { jumps++; }
            worstSize = Math.Max(worstSize, size);
            prevSize = size;
        }

        Note($"2D seed {seed}: first bound {first}, last bound {w.GetSnapshotSizeBound()}, largest size {worstSize}, size jumps over 50% {jumps}");
    }

    [Theory]
    [InlineData(1, 60, 0, 0)]
    [InlineData(2, 120, 0, 0)]
    [InlineData(3, 120, 2000, 2000)]
    public void Bound3DCoversTheSnapshotFromCreationThroughFirstContactToSettling(int seed, int bodies, int contacts, int pairs)
    {
        var w = Pile3D(seed, bodies, contacts, pairs);
        int first = w.GetSnapshotSizeBound(), worstSize = 0, jumps = 0, prevSize = w.GetSnapshotSize();
        Assert.True(prevSize <= first);
        for (int step = 0; step < 600; step++)
        {
            w.Step(Dt);
            int size = w.GetSnapshotSize(), bound = w.GetSnapshotSizeBound();
            Assert.True(size <= bound, "step " + step + ": size " + size + " > bound " + bound);
            if (size > prevSize + prevSize / 2) { jumps++; }
            worstSize = Math.Max(worstSize, size);
            prevSize = size;
        }

        Note($"3D seed {seed}: first bound {first}, last bound {w.GetSnapshotSizeBound()}, largest size {worstSize}, size jumps over 50% {jumps}");
    }

    [Fact]
    public void BoundDoesNotChangeWhileCapacitiesDoNot2D()
    {
        var w = Pile2D(4, 100, 3000, 3000);
        int bound = w.GetSnapshotSizeBound();
        for (int step = 0; step < 500; step++)
        {
            w.Step(Dt);
            Assert.Equal(bound, w.GetSnapshotSizeBound());
        }
    }

    [Fact]
    public void BoundDoesNotChangeWhileCapacitiesDoNot3D()
    {
        var w = Pile3D(4, 100, 3000, 3000);
        int bound = w.GetSnapshotSizeBound();
        for (int step = 0; step < 500; step++)
        {
            w.Step(Dt);
            Assert.Equal(bound, w.GetSnapshotSizeBound());
        }
    }

    [Fact]
    public void BoundGrowsOnlyWhenStorageOrTheSceneGrows3D()
    {
        var w = Pile3D(5, 20, 4000, 4000);
        int bound = w.GetSnapshotSizeBound();
        w.AddBox(new Vector3F(1f, 1f, 1f), new Vector3D(0, 20, 0));
        Assert.True(w.GetSnapshotSizeBound() > bound);
        bound = w.GetSnapshotSizeBound();
        w.Options.Events = EventMask.None;
        w.Step(Dt);   // option changes apply from the next step
        Assert.True(w.GetSnapshotSizeBound() < bound);
    }

    [Fact]
    public void BoundCoversAWorldWithCharactersVehiclesJointsSensorsRagdollsAndFluids3D()
    {
        var opts = new WorldOptions3D { WorldId = 9, Events = EventMask.All, ContactCapacity = 1500, PairCapacity = 1500 };
        Ragdoll.ApplyRecommendedOptions(opts);
        var w = new PhysicsWorld3D(opts);
        w.AddBox(new Vector3F(300f, 1f, 300f), new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f, 0f, 0f));
        w.AddCharacter(new Vector3D(8, 1, 0));
        w.AddCharacter(new Vector3D(10, 1, 0), CharacterDef3.RigidHumanoid);
        var chassis = w.AddBox(new Vector3F(1.8f, 0.5f, 4.2f), new Vector3D(-10, 1.0, 0), density: 1500f / (1.8f * 0.5f * 4.2f));
        w.AddVehicle(chassis, VehicleDef.Car);
        w.AddRagdoll(RagdollDef.Humanoid, new Vector3D(0, 1.5, 6));
        w.AddFluid(new FluidDef { SurfaceOffset = 0.3, LimitToRegion = false });
        Body3? prev = null;
        for (int i = 0; i < 30; i++)
        {
            var b = i % 2 == 0 ? w.AddSphere(0.4f, new Vector3D((i % 5) * 1.0 - 2, 0.5 + (i / 5) * 1.0, 3 + (i % 3))) : w.AddBox(new Vector3F(0.8f, 0.8f, 0.8f), new Vector3D((i % 5) * 1.0 - 2, 0.5 + (i / 5) * 1.0, 3 + (i % 3)));
            if (i == 10)
            {
                var j = new PointJointDef { BodyA = prev!.Value, BodyB = b };
                w.CreateJoint(in j);
            }

            prev = b;
        }

        var sensorBody = w.CreateBody(BodyType.Static, new Vector3D(0, 2, 3));
        w.CreateBox(sensorBody, new Vector3F(4f, 4f, 4f), 0f, new ShapeDef3 { IsSensor = true, Density = 0f });
        for (int i = 0; i < 6; i++) { w.AddSphere(0.3f, new Vector3D(0.2 * i, 6 + i, 3)); }

        int bound = w.GetSnapshotSizeBound(), maxSize = 0;
        Assert.True(w.GetSnapshotSize() <= bound);
        for (int step = 0; step < 500; step++)
        {
            w.Step(Dt);
            int size = w.GetSnapshotSize();
            maxSize = Math.Max(maxSize, size);
            Assert.True(size <= w.GetSnapshotSizeBound(), "step " + step);
            Assert.Equal(bound, w.GetSnapshotSizeBound());
        }

        Note($"3D mixed: bound {bound}, largest size {maxSize}");
    }

    [Fact]
    public void BoundCoversAWorldWithJointsAndSensors2D()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 8, Events = EventMask.All, ContactCapacity = 800, PairCapacity = 800 });
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddCharacter(new Vector2D(8, 1));
        Body2? prev = null;
        for (int i = 0; i < 40; i++)
        {
            var b = w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D((i % 8) * 0.9 - 3, 0.5 + (i / 8) * 0.9));
            if (i == 10)
            {
                var j = new RevoluteJointDef { BodyA = prev!.Value, BodyB = b, LocalAnchorA = new Vector2F(0.4f, 0f), LocalAnchorB = new Vector2F(-0.4f, 0f) };
                w.CreateJoint(in j);
            }

            prev = b;
        }

        var sensorBody = w.CreateBody(BodyType.Static, new Vector2D(0, 2));
        w.CreateBox(sensorBody, new Vector2F(2f, 2f), default, 0f, 0f, new ShapeDef2 { IsSensor = true });
        for (int i = 0; i < 8; i++) { w.AddCircle(0.3f, new Vector2D(0.2 * i - 1, 6 + i)); }

        int bound = w.GetSnapshotSizeBound();
        for (int step = 0; step < 600; step++)
        {
            w.Step(Dt);
            Assert.True(w.GetSnapshotSize() <= w.GetSnapshotSizeBound(), "step " + step);
            Assert.Equal(bound, w.GetSnapshotSizeBound());
        }
    }
}
