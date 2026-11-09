using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class EmptyShapeTests
{
    [Fact]
    public void RawEmptyIsNotAPointInEitherDimension()
    {
        var a=Shape2D.Empty; var ball=Shape2D.CreateCircle(default,1);
        Assert.Equal(0,a.Count);
        Assert.Equal(0,Collide2D.Collide(a,ball,Transform2.Identity).PointCount);
        var queryWorld=new P2.PhysicsWorld2D(); queryWorld.AddCircle(1);
        Assert.False(queryWorld.OverlapShapeAny(Shape2D.Empty,default,0));
        var queryWorld3=new P3.PhysicsWorld3D(); queryWorld3.AddSphere(1);
        Assert.False(queryWorld3.OverlapShapeAny(ConvexShape3D.Empty,default,default));
        var cache=default(SimplexCache2D);
        Assert.True(float.IsPositiveInfinity(Distance2D.Compute(a,ball,Transform2.Identity,ref cache).Distance));
        Assert.False(a.RayCast(new(-1,0),new(2,0)).Hit);
        var empty=ConvexShape3D.Empty; var sphere=ConvexShape3D.Sphere(1);
        Assert.False(Gjk3D.Distance(empty,sphere,RigidTransformF.Identity).Overlapping);
        Assert.True(float.IsPositiveInfinity(Gjk3D.Distance(empty,sphere,RigidTransformF.Identity).Distance));
        Assert.Equal(ConvexShapeKind3D.Empty,ScaledShape3D.Create(empty,new Vector3F(2,3,4)).Kind);
        var compound=Compound3D.Create([new CompoundChild3D(empty,RigidTransformF.Identity)]);
        Assert.Equal(0,compound.GetMassProperties().Mass);
        Assert.False(new Epa3D().Collide(empty,sphere,RigidTransformF.Identity).Colliding);
        Assert.Equal(0,new ManifoldGenerator3D().Collide(empty,sphere,RigidTransformF.Identity).PointCount);
        Assert.False(Gjk3D.RayCast(empty,RigidTransformF.Identity,new(-1,0,0),new(2,0,0),1,out _));
    }

    [Fact]
    public void PlaceholdersKeepOverridesAcrossRestoreForkAndStepping()
    {
        var a=new P2.PhysicsWorld2D(); var b=a.AddEmpty(mass:3,inertia:5);
        var x=new P3.PhysicsWorld3D(); var y=x.AddEmpty(mass:3,inertia:new Vector3F(5,6,7));
        Assert.Equal(3,b.Mass); Assert.Equal(5,b.Inertia); Assert.Equal(3,y.Mass);
        Assert.False(a.OverlapShapeAny(Shape2D.CreateCircle(default,10),default,0));
        Assert.False(x.OverlapSphereAny(default,10));
        Assert.False(x.ClosestPoint(default,out _,100));
        var debug2=new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        var debug3=new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        a.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.All,debug2);
        x.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.All,debug3);
        Assert.Equal(RecursiveAI.OnePhysics.Debug.DebugShapeKind.Empty,debug2.Shapes[0].Kind);
        Assert.Equal(RecursiveAI.OnePhysics.Debug.DebugShapeKind.Empty,debug3.Shapes[0].Kind);
        var bytes2=a.Snapshot(); var bytes3=x.Snapshot();
        a.Restore(bytes2);x.Restore(bytes3);
        Assert.Equal(bytes2,a.Snapshot());Assert.Equal(bytes3,x.Snapshot());
        var fork2=a.Fork();var fork3=x.Fork();
        for(int i=0;i<10;i++) { a.Step(1f/60);x.Step(1f/60);fork2.Step(1f/60);fork3.Step(1f/60);Assert.Equal(a.ComputeHash(),fork2.ComputeHash());Assert.Equal(x.ComputeHash(),fork3.ComputeHash()); }
        Assert.Equal(3,b.Mass);Assert.Equal(5,b.Inertia);Assert.Equal(3,y.Mass);
        Assert.True(b.Position.Y<0 && y.Position.Y<0);
    }
}
