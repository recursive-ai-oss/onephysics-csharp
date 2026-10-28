using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Argument errors are argument exceptions that name the caller's parameter, in the caller's terms.</summary>
public class ExceptionContractTests
{
    private static string ParamOf(Action a)
    {
        var e = Assert.IsAssignableFrom<ArgumentException>(Record.Exception(a));
        Assert.False(string.IsNullOrEmpty(e.ParamName), e.GetType().Name + ": " + e.Message);
        return e.ParamName!;
    }

    [Fact]
    public void HappyPath2DNamesTheCallersParameter()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 7 });
        Assert.Equal("at", ParamOf(() => w.AddCircle(0.5f, at: new Vector2D(double.NaN, 1))));
        Assert.Equal("at", ParamOf(() => w.AddBox(new Vector2F(1, 1), at: new Vector2D(3e6, 0))));
        Assert.Equal("size", ParamOf(() => w.AddBox(new Vector2F(0, 1))));
        Assert.Equal("radius", ParamOf(() => w.AddCircle(-1)));
        Assert.Equal("points", ParamOf(() => w.AddPolygon(new[] { new Vector2F(0, 0), new Vector2F(1, 0) })));
        Assert.Equal("layerA", ParamOf(() => w.IgnoreLayerCollision(40, 1)));
        Assert.Equal("layerB", ParamOf(() => w.SetLayerCollision(1, -1, true)));
        Assert.Equal("position", ParamOf(() => w.CreateBody(BodyType.Dynamic, new Vector2D(double.PositiveInfinity, 0))));
        var bd = new P2.BodyDef { Position = new Vector2D(double.NaN, 0) };
        Assert.Equal("def", ParamOf(() => w.CreateBody(in bd)));
        var b = w.AddCircle(0.5f);
        Assert.Equal("force", ParamOf(() => b.ApplyForce(new Vector2F(float.NaN, 0))));
        Assert.Equal("impulse", ParamOf(() => b.ApplyImpulse(new Vector2F(0, float.PositiveInfinity))));
    }

    [Fact]
    public void FailedHappyPathCallsLeaveNoBodyBehind2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 7 });
        Assert.ThrowsAny<ArgumentException>(() => w.AddBox(new Vector2F(0, 1)));
        Assert.ThrowsAny<ArgumentException>(() => w.AddCircle(-1));
        Assert.ThrowsAny<ArgumentException>(() => w.AddCapsule(0f, 0.5f));
        Assert.Equal(0, w.Core.Bodies.Count);
    }

    [Fact]
    public void BoxSizeIsReportedAsAFullSize2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 7 });
        var e = Assert.Throws<InvalidShapeException>(() => w.AddBox(new Vector2F(0, 1)));
        Assert.Equal("size", e.ParamName);
        Assert.Contains("size is (0, 1)", e.Message, StringComparison.Ordinal);
        Assert.DoesNotContain("half", e.Message, StringComparison.OrdinalIgnoreCase);
        Assert.Equal(ShapeError.OutOfRange, e.Error);
    }

    [Fact]
    public void HappyPath3DNamesTheCallersParameter()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 7 });
        Assert.Equal("at", ParamOf(() => w.AddSphere(0.5f, at: new Vector3D(double.NaN, 1, 0))));
        Assert.Equal("at", ParamOf(() => w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(3e6, 0, 0))));
        Assert.Equal("size", ParamOf(() => w.AddBox(new Vector3F(0, 1, 1))));
        Assert.Equal("radius", ParamOf(() => w.AddSphere(-1)));
        Assert.Equal("layerA", ParamOf(() => w.IgnoreLayerCollision(40, 1)));
        Assert.Equal("position", ParamOf(() => w.CreateBody(BodyType.Dynamic, new Vector3D(double.PositiveInfinity, 0, 0))));
        var bd = new P3.BodyDef { Position = new Vector3D(double.NaN, 0, 0) };
        Assert.Equal("def", ParamOf(() => w.CreateBody(in bd)));
        var b = w.AddSphere(0.5f);
        Assert.Equal("force", ParamOf(() => b.ApplyForce(new Vector3F(float.NaN, 0, 0))));
        Assert.Equal(1, w.Core.Bodies.Count);
    }

    [Fact]
    public void InvalidJointIsAnArgumentExceptionNamingTheParameter()
    {
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 8 });
        var a2 = w2.AddCircle(0.5f);
        var e2 = Assert.Throws<InvalidJointException>(() => w2.AddHinge(a2, a2, new Vector2D(0, 0)));
        Assert.IsAssignableFrom<ArgumentException>(e2);
        Assert.Equal("b", e2.ParamName);
        var def2 = default(P2.RevoluteJointDef);
        def2.BodyA = a2; def2.BodyB = a2;
        Assert.Equal("def", Assert.Throws<InvalidJointException>(() => w2.CreateJoint(def2)).ParamName);

        var w3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 8 });
        var s1 = w3.AddBox(new Vector3F(1, 1, 1), type: BodyType.Static);
        var s2 = w3.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(3, 0, 0), type: BodyType.Static);
        var e3 = Assert.Throws<InvalidJointException>(() => w3.AddHinge(s1, s2, new Vector3D(1, 0, 0), new Vector3F(0, 1, 0)));
        Assert.Equal("b", e3.ParamName);
    }

    [Fact]
    public void ShapeErrorsShareOneKindPropertyAndANeutralZero()
    {
        Assert.Equal(ShapeError.None, default(ShapeError));
        Assert.Equal(ShapeError.NotFinite, new InvalidShapeException(ShapeError.NotFinite, "x").Error);
        Assert.Equal(HullBuildError.TooFewPoints, new HullBuildException(HullBuildError.TooFewPoints, "x").Error);
        Assert.Equal(MeshBuildError.NoVertices, new MeshBuildException(MeshBuildError.NoVertices, "x").Error);
        Assert.Equal(ShapeCompositionError.InvalidScale, new ShapeCompositionException(ShapeCompositionError.InvalidScale, "x").Error);
    }

    [Fact]
    public void OutOfBoundsExceptionCarriesTheBody2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 9, Gravity = default });
        var rocket = w.AddCircle(0.2f, at: new Vector2D(2.0e6, 0));
        w.ShiftOrigin(new Vector2D(-2.19e6, 0));
        rocket.LinearVelocity = new Vector2F(4000, 0);
        P2.BodyOutOfBoundsException? caught = null;
        for (int i = 0; i < 4000 && caught == null; i++)
        {
            try { w.Step(1f / 60); }
            catch (P2.BodyOutOfBoundsException e) { caught = e; }
        }

        Assert.NotNull(caught);
        Assert.Equal(rocket, caught!.Body);
        caught.Body.Destroy();
        w.Step(1f / 60);
    }
}
