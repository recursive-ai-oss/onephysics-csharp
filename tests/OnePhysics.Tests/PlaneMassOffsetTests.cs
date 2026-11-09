using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class PlaneMassOffsetTests
{
    [Fact]
    public void PlaneAnalyticContactsCastsAndDistances()
    {
        var plane = ConvexShape3D.Plane(new(0, 2, 0), 2);
        var ball = ConvexShape3D.Sphere(1);
        var pose = new RigidTransformF(QuaternionF.Identity, new(0, 1.5f, 0));
        var gen = new ManifoldGenerator3D();
        var m = gen.Collide(plane, ball, pose);
        Assert.Equal(1, m.PointCount); Assert.Equal(-0.5f, m.MinSeparation); Assert.Equal(Vector3F.UnitY, m.Normal);
        var flip = gen.Collide(ball, plane, pose.Inverted());
        Assert.Equal(-0.5f, flip.MinSeparation); Assert.Equal(-Vector3F.UnitY, flip.Normal);
        var box = gen.Collide(plane, ConvexShape3D.Box(new(1,1,1)), pose);
        Assert.Equal(4, box.PointCount); Assert.Equal(-0.5f, box.MinSeparation);
        Assert.True(Gjk3D.RayCast(plane, RigidTransformF.Identity, new(0,3,0), new(0,-4,0), 1, out var ray));
        Assert.Equal(0.5f, ray.Fraction); Assert.Equal(Vector3F.UnitY, ray.Normal);
        Assert.True(Gjk3D.ShapeCast(ball, plane, RigidTransformF.Identity, new(0,-4,0), out var inside));
        Assert.True(inside.StartsOverlapping);
        var ballInPlane = new RigidTransformF(QuaternionF.Identity, new(0,4,0));
        Assert.True(Gjk3D.ShapeCast(plane, ball, ballInPlane, new(0,4,0), out var cast));
        Assert.Equal(0.5f, cast.Fraction);
        Assert.Equal(2f, Gjk3D.Distance(plane, ball, ballInPlane).Distance);
        Assert.Equal(-0.5f, new Epa3D().Collide(plane, ball, pose).Separation);
    }

    [Fact]
    public void PlaneWorldSettlesRestoresAndRejectsNonStatic()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddPlane(Vector3F.UnitY);
        var body = w.AddBox(new Vector3F(1,1,1), new(0,2,0));
        for (int i=0;i<120;i++) w.Step(1f/60);
        Assert.InRange(body.Position.Y, 0.48, 0.52);
        var bytes = w.Snapshot(); var fresh = new P3.PhysicsWorld3D(); fresh.Restore(bytes);
        Assert.Equal(bytes, fresh.Snapshot());
        for (int i=0;i<10;i++) { w.Step(1f/60); fresh.Step(1f/60); Assert.Equal(w.ComputeHash(),fresh.ComputeHash()); }
        var bad = w.CreateBody(new P3.BodyDef { Type = BodyType.Dynamic });
        Assert.Throws<StaticOnlyShapeException>(()=>w.CreateShape(bad,P3.ShapeSpec3D.Plane(Vector3F.UnitY)));
        bad.Destroy();
        Assert.Throws<StaleHandleException>(()=>w.CreateShape(bad,P3.ShapeSpec3D.Plane(Vector3F.UnitY)));
    }

    [Fact]
    public void PerShapeMassOffsetsDoNotMoveGeometryAndComposeBeforeBodyCombination()
    {
        var circle = Shape2D.CreateCircle(default, 1);
        var offset = circle.WithCenterOfMassOffset(new(2,0));
        var a = circle.ComputeMass(); var b = offset.ComputeMass();
        Assert.Equal(a.Area,b.Area); Assert.Equal(2,b.CentroidX); Assert.Equal(a.InertiaAtCentroid+4*a.Area,b.InertiaAtCentroid);
        Assert.Equal(circle.RayCast(new(-3,0),new(6,0)).Fraction,offset.RayCast(new(-3,0),new(6,0)).Fraction);
        var w2 = new P2.PhysicsWorld2D(); var body2=w2.CreateBody(new P2.BodyDef { Type = BodyType.Dynamic });
        w2.CreateShape(body2, offset); w2.CreateShape(body2, circle);
        Assert.Equal(1,body2.CenterOfMass.X);
        var bytes2=w2.Snapshot(); var f2=new P2.PhysicsWorld2D(); f2.Restore(bytes2); Assert.Equal(bytes2,f2.Snapshot());
        var sphere=ConvexShape3D.Sphere(1); var shifted=sphere.WithCenterOfMassOffset(new(2,0,0));
        var sm=sphere.GetMassProperties();var om=shifted.GetMassProperties();
        Assert.Equal(sm.Mass,om.Mass);Assert.Equal(new Vector3F(2,0,0),om.Center);Assert.Equal(sm.Inertia.M22+4*sm.Mass,om.Inertia.M22);
        Assert.Equal(sphere.Support(Vector3F.UnitX),shifted.Support(Vector3F.UnitX));
        var w3=new P3.PhysicsWorld3D();var body3=w3.CreateBody(new P3.BodyDef { Type = BodyType.Dynamic });
        w3.CreateShape(body3,P3.ShapeSpec3D.Sphere(1).WithCenterOfMassOffset(new(2,0,0)));w3.CreateSphere(body3,1);
        Assert.Equal(1,body3.CenterOfMass.X);
        var bytes3=w3.Snapshot();var f3=new P3.PhysicsWorld3D();f3.Restore(bytes3);Assert.Equal(bytes3,f3.Snapshot());
        Assert.Equal(sphere.GetMassProperties(),shifted.WithCenterOfMassOffset(new(-2,0,0)).GetMassProperties());
    }

    [Fact]
    public void PlaneDomainDoesNotPreventLargeOriginShifts()
    {
        var w=new P3.PhysicsWorld3D();w.AddPlane(Vector3F.UnitY,at:new(1000000,0,0));
        w.AddSphere(0.5f,at:new(1000000,2,0));
        w.ShiftOrigin(new(1000000,0,0));
        Assert.True(w.Raycast(new(3,4,0),-Vector3F.UnitY,out var ray,10));Assert.InRange(ray.Point.Y,-1e-5,1e-5);
        var bytes=w.Snapshot();w.Restore(bytes);Assert.Equal(bytes,w.Snapshot());
        for(int i=0;i<60;i++)w.Step(1f/60);
    }

    [Fact]
    public void OffsetSectionRejectsNonFiniteBytesAtomicallyAndDepthIsBounded()
    {
        var w=new P2.PhysicsWorld2D();var b=w.CreateBody(new P2.BodyDef());w.CreateShape(b,Shape2D.CreateCircle(default,1).WithCenterOfMassOffset(new(1,2)));
        var bytes=w.Snapshot();var bad=(byte[])bytes.Clone();int marker=bad.AsSpan().IndexOf(new byte[]{0x43,0x4f,0x4d,0x32});Assert.True(marker>0);
        System.Buffers.Binary.BinaryPrimitives.WriteSingleLittleEndian(bad.AsSpan(marker+20),float.NaN);
        Assert.Throws<SnapshotFormatException>(()=>w.Restore(bad));Assert.Equal(bytes,w.Snapshot());
        b.Destroy();Assert.Throws<StaleHandleException>(()=>w.CreateShape(b,Shape2D.CreateCircle(default,1)));
        var spec=P3.ShapeSpec3D.Sphere(1);for(int i=0;i<17;i++)spec=spec.WithCenterOfMassOffset(new(0.01f,0,0));
        var spatial=new P3.PhysicsWorld3D();var body=spatial.CreateBody(new P3.BodyDef());
        Assert.Throws<SnapshotFormatException>(()=>spatial.CreateShape(body,spec));
    }

    [Fact]
    public void HappyPathDecoratorsReturnBodiesAndPreserveGeometry()
    {
        var planar=new P2.PhysicsWorld2D();var a=planar.AddWithCenterOfMassOffset(Shape2D.CreateCircle(default,1),new(0,-0.5f),at:new(0,3));Assert.Equal(-0.5f,a.CenterOfMass.Y);Assert.Equal(3,a.Position.Y);
        var spatial=new P3.PhysicsWorld3D();var b=spatial.AddWithCenterOfMassOffset(P3.ShapeSpec3D.Sphere(1),new(0,-0.5f,0),at:new(0,3,0));Assert.Equal(-0.5f,b.CenterOfMass.Y);Assert.Equal(3,b.Position.Y);
        var kinematic=spatial.CreateBody(new P3.BodyDef{Type=BodyType.Kinematic});Assert.Throws<StaticOnlyShapeException>(()=>spatial.CreateShape(kinematic,P3.ShapeSpec3D.Plane(Vector3F.UnitY)));
        Assert.Throws<ShapeCompositionException>(()=>Compound3D.Create([new CompoundChild3D(ConvexShape3D.Plane(Vector3F.UnitY),RigidTransformF.Identity)]));
    }

    [Fact]
    public void HostilePlaneAndOffsetsAreTyped()
    {
        Assert.Throws<ArgumentOutOfRangeException>(()=>ConvexShape3D.Plane(default));
        Assert.Throws<ArgumentOutOfRangeException>(()=>ConvexShape3D.Plane(new(float.NaN,0,0)));
        Assert.Throws<ArgumentOutOfRangeException>(()=>ConvexShape3D.Plane(Vector3F.UnitY,float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(()=>ConvexShape3D.Sphere(1).WithCenterOfMassOffset(new(float.NaN,0,0)));
        Assert.Throws<InvalidShapeException>(()=>Shape2D.CreateCircle(default,1).WithCenterOfMassOffset(new(float.PositiveInfinity,0)));
    }
}
