using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Rods;
using Xunit;
using SoftBodyMesh = RecursiveAI.OnePhysics.Physics3D.SoftBodyMesh;
using SoftBodyDef = RecursiveAI.OnePhysics.Physics3D.SoftBodyDef;
using RodDef = RecursiveAI.OnePhysics.Physics3D.RodDef;

namespace RecursiveAI.OnePhysics.Tests.Water;

/// <summary>Shallow water coupled to kinematic bodies (wave makers), soft bodies and rods.</summary>
public class DeformableWaterTests
{
    private const float Dt = 1f / 60f;

    [Fact]
    public void AKinematicPaddleMakesWavesThatTravelAtTheShallowWaterSpeed()
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = 1, WorldId = 351 });
        w.AddBox(new Vector2F(80f, 1f), new Vector2D(20, -1.5), type: BodyType.Static);
        // 40 m channel, 1 m deep.
        var water = w.CreateShallowWater(new Physics2D.ShallowWaterDef { Cells = 400, CellSize = 0.1f, Level = 1.0, FloorHeight = 0.0, Damping = 0f, SampleBed = false });
        var paddle = w.AddBox(new Vector2F(1f, 0.6f), new Vector2D(5, 1.299), type: BodyType.Kinematic);
        double t1 = -1, t2 = -1;
        for (int i = 0; i < 700; i++)
        {
            // Lower the paddle into the water for 0.4 s, slowly enough for a small wave (the speed of a large one is amplitude dependent).
            paddle.LinearVelocity = i < 24 ? new Vector2F(0f, -0.1f) : default;
            w.Step(Dt);
            water.TryGetSample(15, out var a);
            water.TryGetSample(30, out var b);
            if (t1 < 0 && a.Surface > 1.0 + 0.0005) { t1 = (i + 1) * Dt; }
            if (t2 < 0 && b.Surface > 1.0 + 0.0005) { t2 = (i + 1) * Dt; }
        }

        Assert.True(t1 > 0 && t2 > t1, "front times " + t1 + " " + t2);
        double speed = 15.0 / (t2 - t1);
        double expected = Math.Sqrt(9.81 * 1.0);
        Assert.InRange(speed, expected * 0.95, expected * 1.05);
    }

    private static double MeshVolume(SoftBodyMesh m)
    {
        var v = m.Vertices; var t = m.Triangles;
        double sum = 0;
        for (int i = 0; i < t.Length; i += 3)
        {
            var a = v[t[i]]; var b = v[t[i + 1]]; var c = v[t[i + 2]];
            sum += a.X * (b.Y * c.Z - b.Z * c.Y) - a.Y * (b.X * c.Z - b.Z * c.X) + a.Z * (b.X * c.Y - b.Y * c.X);
        }

        return Math.Abs(sum) / 6.0;
    }

    [Fact]
    public void ASoftBallFloatsAtItsDensityRatioInShallowWater()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 352 });
        w.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -1.5, 10), type: BodyType.Static);
        w.AddShallowWater(0, 0, 20, 20, 1.0, 0.5f, floor: -1.0);
        var mesh = SoftBodyMesh.Sphere(0.5f, 5);
        double volume = MeshVolume(mesh);
        var ball = w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Position = new Vector3D(10, 2.0, 10), Mass = (float)(0.5 * 1000.0 * volume), InternalPressure = 1f, LinearDamping = 0.5f });
        for (int i = 0; i < 900; i++) { w.Step(Dt); }
        // Half the water's density: the centre rides at the surface (level 1 plus the little the ball's own displacement raises it).
        Assert.InRange(ball.CenterOfMass.Y, 0.93, 1.1);
        Assert.InRange(ball.GetVelocity(0).Length(), 0f, 0.2f);
        // A heavier ball sinks lower, a lighter one rides higher.
        var w2 = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 353 });
        w2.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -1.5, 10), type: BodyType.Static);
        w2.AddShallowWater(0, 0, 20, 20, 1.0, 0.5f, floor: -1.0);
        var heavy = w2.AddSoftBody(new SoftBodyDef { Mesh = mesh, Position = new Vector3D(10, 2.0, 10), Mass = (float)(0.8 * 1000.0 * volume), InternalPressure = 1f, LinearDamping = 0.5f });
        for (int i = 0; i < 900; i++) { w2.Step(Dt); }
        Assert.True(heavy.CenterOfMass.Y < ball.CenterOfMass.Y - 0.1, "heavy " + heavy.CenterOfMass.Y + " half " + ball.CenterOfMass.Y);
    }

    [Fact]
    public void ALightRopeFloatsAndAHeavyOneSinksInShallowWater()
    {
        double Settle(float density, int id)
        {
            var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = id });
            w.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -1.5, 10), type: BodyType.Static);
            w.AddShallowWater(0, 0, 20, 20, 1.0, 0.5f, floor: -1.0);
            var pts = new Vector3D[21];
            for (int i = 0; i < pts.Length; i++) { pts[i] = new Vector3D(5 + 0.5 * i, 1.4, 10); }
            var rod = w.AddRod(new RodDef { Points = pts, Material = new RodMaterial(1e7f, 4e6f, 0.02f, density) });
            for (int i = 0; i < 600; i++) { w.Step(Dt); }
            double sum = 0;
            for (int i = 0; i < rod.ParticleCount; i++) { sum += rod.GetPosition(i).Y; }
            return sum / rod.ParticleCount;
        }

        double light = Settle(400f, 354), heavy = Settle(2500f, 355);
        Assert.InRange(light, 0.97, 1.05);
        Assert.True(heavy < 0.0, "heavy rope at " + heavy);
    }

    [Fact]
    public void WaterDisplacedBySoftBodiesIsTakenOutAgainAndSurvivesSnapshots()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 356 });
        w.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -1.5, 10), type: BodyType.Static);
        var water = w.AddShallowWater(0, 0, 20, 20, 1.0, 0.5f, floor: -1.0);
        double before = water.Volume;
        var mesh = SoftBodyMesh.Sphere(0.4f, 4);
        var ball = w.AddSoftBody(new SoftBodyDef { Mesh = mesh, Position = new Vector3D(10, 1.2, 10), Mass = (float)(0.5 * 1000.0 * MeshVolume(mesh)), InternalPressure = 1f, LinearDamping = 0.5f });
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        var snap = w.Snapshot();
        var hashes = new ulong[60];
        for (int i = 0; i < 60; i++) { w.Step(Dt); hashes[i] = w.ComputeHash(); }
        var after = w.Snapshot();
        w.Restore(snap);
        for (int i = 0; i < 60; i++) { w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.Equal(after, w.Snapshot());
        var fresh = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = 1, WorldId = 356 });
        fresh.Restore(snap);
        for (int i = 0; i < 60; i++) { fresh.Step(Dt); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        // The ball leaves the world: the displaced water goes back.
        ball.Destroy();
        for (int i = 0; i < 5; i++) { w.Step(Dt); }
        Assert.InRange(water.Volume, before - 1e-3, before + 1e-3);
    }
}
