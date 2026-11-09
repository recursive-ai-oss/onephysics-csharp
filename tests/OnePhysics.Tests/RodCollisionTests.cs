using System;
using RecursiveAI.OneMath;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RodCollisionTests(ITestOutputHelper output)
{
    [Fact]
    public void ThirtyMetrePerSecondSegmentsDoNotCrossThinWalls()
    {
        var a=new P2.PhysicsWorld2D(new(){Gravity=default,SleepEnabled=false});a.AddBox(new(.01f,20),type:BodyType.Static);
        var r=a.AddRod(new(){Points=[new(-.5,-1),new(-.5,0),new(-.5,1)],Velocity=new(30,0),Damping=0,Substeps=1});
        var b=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});b.AddBox(new(.01f,20,20),type:BodyType.Static);
        var s=b.AddRod(new(){Points=[new(-.5,-1,0),new(-.5,0,0),new(-.5,1,0)],Velocity=new(30,0,0),Damping=0,Substeps=1});
        double max2=-10,max3=-10;
        for(int k=0;k<20;k++)
        {
            a.Step(1f/60);b.Step(1f/60);
            for(int i=0;i<3;i++){max2=Math.Max(max2,r.GetPosition(i).X);max3=Math.Max(max3,s.GetPosition(i).X);}
        }
        output.WriteLine($"thin wall maximum particle x planar={max2:R}, spatial={max3:R}");
        Assert.True(max2<-.004,$"planar rope crossed x={max2:R}");Assert.True(max3<-.004,$"spatial rope crossed x={max3:R}");
    }
    [Fact]
    public void ContactModifierCanDisableRodResponse()
    {
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});world.AddBox(new(.01f,20,20),type:BodyType.Static);int calls=0;
        world.ContactModifier=(ref ContactModification c)=>{if(c.IsRodContact){calls++;c.Enabled=false;}};
        var r=world.AddRod(new(){Points=[new(-.5,-1,0),new(-.5,1,0)],Velocity=new(30,0,0),Damping=0,Substeps=1});
        for(int i=0;i<4;i++)world.Step(1f/60);
        Assert.True(calls>0);Assert.True(r.GetPosition(0).X>.5);
    }
    [Fact]
    public void SweptInteriorsOfTwoFastRodsDoNotPassThroughEachOther()
    {
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});
        var a=world.AddRod(new(){Points=[new(-1,0,-.5),new(1,0,-.5)],Velocity=new(0,0,30),Damping=0,Substeps=1});
        var b=world.AddRod(new(){Points=[new(0,-1,.5),new(0,1,.5)],Velocity=new(0,0,-30),Damping=0,Substeps=1});
        world.Step(1f/30);
        double separation=b.GetPosition(0).Z-a.GetPosition(0).Z;
        output.WriteLine($"rod-rod post-sweep signed separation={separation:R}");Assert.InRange(separation,.019,.03);
        var snapshot=world.Snapshot();var copy=world.Fork();copy.Restore(snapshot);
        for(int i=0;i<20;i++){world.Step(1f/60);copy.Step(1f/60);Assert.Equal(world.ComputeHash(),copy.ComputeHash());}
    }
    [Fact]
    public void SelfCollisionSeparatesNonAdjacentMaterial()
    {
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});
        var rod=world.AddRod(new(){Points=[new(-1,0,0),new(1,0,0),new(2,2,0),new(0,1,0),new(0,-1,0)],PinnedParticles=[0,1],SelfCollision=true});
        world.Step(1f/60);
        RecursiveAI.OnePhysics.Rods.RodGeometry.Closest(rod.GetPosition(0),rod.GetPosition(1),rod.GetPosition(3),rod.GetPosition(4),out _,out _,out var a,out var b);
        double gap=(a-b).Length();output.WriteLine($"self-collision core gap={gap:R}");Assert.InRange(gap,.019,1);
    }

    [Fact]
    public void PlaneConveyorAndIndexedTriangleMaterialsReachRodContacts()
    {
        var w=new P3.PhysicsWorld3D();var floor=w.CreateBody(new(){Type=BodyType.Static});
        w.CreateShape(floor,P3.ShapeSpec3D.Plane(new(0,1,0)),new(){Friction=1,Surface=new(){Velocity=new(2,0,0)}});
        var rod=w.AddRod(new(){Points=[new(-.5,.02,0),new(.5,.02,0)],Friction=1,Damping=0});
        for(int i=0;i<120;i++)w.Step(1f/60);
        output.WriteLine($"plane conveyor vx={rod.GetVelocity(0).X:R}, y={rod.GetPosition(0).Y:R}");
        Assert.True(rod.GetVelocity(0).X>.2f);Assert.InRange(rod.GetPosition(0).Y,.009,.03);
        var mesh=new P3.PhysicsWorld3D();var terrain=mesh.CreateBody(new(){Type=BodyType.Static});
        var shape=mesh.CreateMesh(terrain,[new(-2,0,-2),new(-2,0,2),new(2,0,-2),new(2,0,2)],[0,1,2,2,1,3]);
        shape.SetMaterials([new P3.Material3D(.81f),new P3.Material3D(.04f)],[0,1]);
        float observed=-1;
        mesh.ContactModifier=(ref ContactModification c)=>{if(c.IsRodContact&&c.SubShapeA==0)observed=c.Friction;};
        mesh.AddRod(new(){Points=[new(-1,.03,-1),new(-.5,.03,-1)],Friction=.25f});
        for(int i=0;i<20;i++)mesh.Step(1f/60);
        Assert.InRange(observed,.4499f,.4501f);
    }

    [Fact]
    public void RestitutionUsesThePreImpactSpeedNotTheClippedDisplacement()
    {
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});world.AddBox(new(.01f,10,10),type:BodyType.Static,material:new(0,1,0));
        var rod=world.AddRod(new(){Points=[new(-.5,-1,0),new(-.5,1,0)],Velocity=new(30,0,0),Substeps=1,Damping=0,Friction=0});
        world.Step(1f/60);output.WriteLine($"rebound speeds={rod.GetVelocity(0).X:R}, {rod.GetVelocity(1).X:R}");
        Assert.InRange(rod.GetVelocity(0).X,-30.01f,-29.99f);Assert.InRange(rod.GetVelocity(1).X,-30.01f,-29.99f);
    }

}
