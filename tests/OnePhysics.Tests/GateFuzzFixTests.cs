using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Hand-written cases for the findings of the long fuzz campaign in 3D, hulls and the authoring tools (the fuzz inputs themselves replay in <see cref="FuzzRegressionTests"/>).</summary>
public sealed class GateFuzzFixTests
{
    [Fact]
    public void HullOfSubnormalCoordinatesBuilds()
    {
        // Every coordinate is a float32 subnormal (about 2^-141): the exact orientation test refuses such numbers unless they are scaled first.
        const float u = 3.3e-43f;
        var points = new Vector3F[]
        {
            new(0, 0, 0), new(u, 0, 0), new(0, u, 0), new(0, 0, u), new(u, u, u), new(u, u, 0), new(u, 0, u), new(0, u, u),
        };
        var hull = ConvexHull3D.Build(points);
        Assert.True(hull.Vertices.Length >= 4);
        Assert.True(hull.Faces.Length >= 4);
    }

    [Fact]
    public void HingeAnchorFarFromItsBodyIsRefused()
    {
        var w = new P3.PhysicsWorld3D();
        var a = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 0, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(1, 0, 0));
        var far = Assert.Throws<InvalidJointException>(() => w.AddHinge(a, b, new Vector3D(500, 0, 0), Vector3F.UnitY));
        Assert.Contains("anchor", far.Message, StringComparison.Ordinal);
        // Hinged at a sensible offset (a few body sizes away) is still fine, and so is an anchor on a static body.
        w.AddHinge(a, b, new Vector3D(0.5, 0, 0), Vector3F.UnitY);
        var ground = w.AddBox(new Vector3F(100, 1, 100), new Vector3D(0, -5, 0), type: BodyType.Static);
        var c = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(300, 0, 0));
        w.AddHinge(ground, c, new Vector3D(300, 0, 0), Vector3F.UnitY);
    }

    [Fact]
    public void RagdollJointFarFromItsPartsIsRefusedBeforeAnythingIsMade()
    {
        var def = P3.RagdollDef.Humanoid;
        def.Parts[3].JointPosition = new Vector3F(90f, 100f, 80f);
        var w = new P3.PhysicsWorld3D();
        var bodiesBefore = w.BodyCount;
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddRagdoll(def, new Vector3D(0, 1, 0)));
        Assert.Contains("JointPosition", e.Message, StringComparison.Ordinal);
        Assert.Equal(bodiesBefore, w.BodyCount);
    }

    [Fact]
    public void UnboundCustomGravityRefusesTheStepBeforeChangingAnything()
    {
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var ball = w.AddSphere(.5f, at: new(0, 10, 0));
        var field = w.AddField(new() { Kind = FieldKind.CustomGravity, CallbackId = 7 });
        var hash = w.ComputeHash();
        var velocity = ball.LinearVelocity;
        var e = Assert.Throws<FieldCallbackException>(() => w.Step(1f / 60));
        Assert.Contains("callback", e.Message, StringComparison.Ordinal);
        Assert.Equal(hash, w.ComputeHash());
        Assert.Equal(velocity, ball.LinearVelocity);
        w.BindGravityField(7, (p, t) => new(0, -3, 0));
        w.Step(1f / 60);
        Assert.True(ball.LinearVelocity.Y < 0f);
        _ = field;
    }

    [Fact]
    public void AutomaticParticleRadiusOfAHugeClothStaysWithinTheLimitAndRoundTrips()
    {
        var mesh = P3.SoftBodyMesh.FromTriangles([new(0, 0, 0), new(4000, 0, 0), new(0, 0, 4000)], [0, 1, 2]);
        var w = new P3.PhysicsWorld3D(new() { WorldId = 77 });
        var cloth = w.AddSoftBody(P3.SoftBodyDef.Cloth(mesh, new(0, 1, 0)));
        Assert.True(cloth.ParticleRadius <= 100f);
        var bytes = w.Snapshot();
        var copy = new P3.PhysicsWorld3D(new() { WorldId = 77 });
        copy.Restore(bytes);
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
    }

    [Fact]
    public void LightNeedleHitByAHeavyCapsuleKeepsFiniteSpeeds()
    {
        // A 0.1 kg thin box moving at the speed limit through a 270 kg capsule: friction and the normal row used to feed each other through the box's spin until the numbers overflowed.
        var w = new P3.PhysicsWorld3D(new() { WorldId = 78, SleepEnabled = false });
        var capsule = w.AddCapsule(0.5f, 1.0f, new Vector3D(0, 0, 0), density: 70f);
        var needle = w.AddBox(new Vector3F(0.05f, 1.0f, 0.5f), new Vector3D(0.1, 0.4, 0.05), density: 1.3f);
        for (int i = 0; i < 12; i++)
        {
            needle.LinearVelocity = new Vector3F(0f, 400f, 0f);
            needle.AngularVelocity = new Vector3F(90f, 0f, 0f);
            w.Step(1f / 60);
            foreach (var body in new[] { capsule, needle })
            {
                var v = body.LinearVelocity; var a = body.AngularVelocity;
                Assert.True(float.IsFinite(v.X + v.Y + v.Z + a.X + a.Y + a.Z));
                Assert.True(Math.Abs(v.X) < 1e6f && Math.Abs(v.Y) < 1e6f && Math.Abs(v.Z) < 1e6f);
            }
        }
    }
}
