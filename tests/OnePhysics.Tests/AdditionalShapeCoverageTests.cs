using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class AdditionalShapeCoverageTests
{
    [Fact]
    public void AuthoringRecordsBuildNewGeometryAndMassOffsets()
    {
        const string planar="scene 1\ndim 2\nmaterial 0 0.6 0\nbody static 0 0 0 0 0 0 -\nshape heightfield 0 0 0 3 1 0 0 0 0 1\nbody dynamic 0 2 0 0 0 0 -\nshape offset-circle 0 1 0.5 0 0 0 -0.2\nshape empty 0 0 0\n";
        var scene=RecursiveAI.OnePhysics.Scene.Scene.Read(new StringReader(planar));
        var w2=scene.CreateWorld2D();Assert.Equal(2,w2.BodyCount);
        const string spatial="scene 1\ndim 3\nmaterial 0 0.6 0\nbody static 0 0 0 0 0 0 1 0 0 0 0 0 0 -\nshape plane 0 0 0 0 1 0 0\nbody dynamic 0 3 0 0 0 0 1 0 0 0 0 0 0 -\nshape offset-frustum 0 1 0 0.4 0.2 0.3 0 -0.1 0\nbody static 5 0 0 0 0 0 1 0 0 0 0 0 0 -\nshape triangle 0 1 0 -1 0 -1 0 0 1 1 0 -1\nshape empty 0 0 0\n";
        var scene3=RecursiveAI.OnePhysics.Scene.Scene.Read(new StringReader(spatial));
        var w3=scene3.CreateWorld3D();Assert.Equal(3,w3.BodyCount);
        using var text=new StringWriter();scene3.Write(text);Assert.Equal(w3.BodyCount,RecursiveAI.OnePhysics.Scene.Scene.Read(new StringReader(text.ToString())).CreateWorld3D().BodyCount);
    }

    [Fact]
    public void PlaneSupportsSoftParticlesAndOffsetPlanarCreationIsRecorded()
    {
        var w=new P3.PhysicsWorld3D();w.AddPlane(Vector3F.UnitY);
        var mesh=P3.SoftBodyMesh.FromTriangles([new(0,0,0),new(1,0,0),new(0,0,1)],[0,1,2]);
        var def=P3.SoftBodyDef.Cloth(mesh,new(0,1,0));def.ParticleRadius=0.02f;
        var soft=w.AddSoftBody(def);
        for(int i=0;i<120;i++)w.Step(1f/60);
        for(int i=0;i<3;i++)Assert.InRange(soft.GetPosition(i).Y,-0.005,0.1);
        var a=new P2.PhysicsWorld2D(new(){WorldId=998});using var stream=new MemoryStream();
        using(var recording=a.StartRecording(stream))
        {
            var body=a.CreateBody(new P2.BodyDef());a.CreateShape(body,Shape2D.CreateCircle(default,1).WithCenterOfMassOffset(new(0,-0.2f)));a.Step(1f/60);
        }
        stream.Position=0;var player=new ReplayPlayer(stream);var b=new P2.PhysicsWorld2D(new(){WorldId=998});Assert.True(player.ReadNextStep(b));Assert.Equal(a.Snapshot(),b.Snapshot());
    }

    [Fact]
    public void SampledPlanarTerrainAndMassDecoratorsCoverEveryCoreKind()
    {
        var floor=new HeightField2D([0,0,0,0],10).GetSegment(1);
        Shape2D[] shapes=[Shape2D.CreateCircle(default,0.3f),Shape2D.CreateCapsule(new(-0.3f,0),new(0.3f,0),0.3f),Shape2D.CreateBox(0.4f,0.3f),Shape2D.CreateBox(0.3f,0.2f,radius:0.1f),Shape2D.CreatePolygon([new(-0.3f,-0.3f),new(0.3f,-0.3f),new(0,0.3f)])];
        foreach(var shape in shapes)
        {
            var decorated=shape.WithCenterOfMassOffset(new(1,-2));
            var pose=new Transform2(new(15,0.28f),Rot2.Identity);
            var m=Collide2D.Collide(floor,shape,pose);var d=Collide2D.Collide(floor,decorated,pose);
            Assert.True(m.PointCount>0,shape.Kind.ToString());Assert.InRange(m.Point0.Separation,-0.021f,-0.019f);
            Assert.Equal(m.PointCount,d.PointCount);Assert.Equal(m.Normal,d.Normal);Assert.Equal(m.Point0.Separation,d.Point0.Separation);
            Assert.Equal(0,Collide2D.Collide(Shape2D.Empty,shape,pose).PointCount);
        }
        foreach(var line in new[]{Shape2D.CreateSegment(new(-1,0),new(1,0)),floor,Shape2D.Empty})Assert.Equal(0,line.WithCenterOfMassOffset(new(1,2)).ComputeMass().Area);
        var w=new P2.PhysicsWorld2D();w.AddHeightField([0,0,0],spacing:10);
        Assert.True(w.Raycast(new(5,2),new(0,-1),out var ray,4));Assert.InRange(ray.Distance,1.9999f,2.0001f);
        Assert.True(w.RaycastAny(new(5,2),new(0,-1),4));Assert.Single(w.RaycastAll(new(5,2),new(0,-1),4));
        Assert.True(w.ShapeCast(Shape2D.CreateCircle(default,0.5f),new(5,2),0,new(0,-1),out var cast,4));Assert.InRange(cast.Distance,1.499f,1.501f);
        Assert.True(w.OverlapCircleAny(new(5,0.2),0.3f));Assert.Single(w.OverlapCircleAll(new(5,0.2),0.3f));
        Assert.True(w.ClosestPoint(new(5,2),out var near,4));Assert.InRange(near.Point.Y,-1e-5,1e-5);
    }

    private static ConvexShape3D[] Convexes()
    {
        var box=ConvexShape3D.Box(new(0.3f,0.4f,0.3f));
        return [ConvexShape3D.Sphere(0.3f),box,ConvexShape3D.Capsule(0.2f,0.2f),ConvexShape3D.TaperedCapsule(0.3f,0.2f,0.3f),ConvexShape3D.Cylinder(0.4f,0.3f),
            ConvexShape3D.FromHull(ConvexHull3D.Build([new(-0.3f,-0.4f,-0.3f),new(0.3f,-0.4f,-0.3f),new(0,-0.4f,0.3f),new(0,0.4f,0)])),
            ConvexShape3D.TaperedCylinder(0.4f,0.2f,0.3f),ConvexShape3D.Triangle(new(-0.3f,0,-0.3f),new(0,0,0.3f),new(0.3f,0,-0.3f),0.1f),
            box.WithCenterOfMassOffset(new(1,-2,3))];
    }

    [Fact]
    public void NewSurfacesAgainstEveryConvexAndCompoundHaveKnownDepth()
    {
        ConvexShape3D[] surfaces=[ConvexShape3D.Plane(Vector3F.UnitY),ConvexShape3D.Triangle(new(-8,0,-8),new(0,0,8),new(8,0,-8)),ConvexShape3D.TaperedCylinder(1,3,4)];
        var gen=new ManifoldGenerator3D();int pairs=0;
        foreach(var surface in surfaces)foreach(var shape in Convexes())
        {
            float height=surface.Kind==ConvexShapeKind3D.TaperedCylinder?1:0;
            var pose=new RigidTransformF(QuaternionF.Identity,new(0,height-shape.Support(-Vector3F.UnitY).Y-0.02f,0));
            var m=gen.Collide(surface,shape,pose);
            Assert.True(m.PointCount>0,$"{surface.Kind}/{shape.Kind}");
            Assert.InRange(m.MinSeparation,-0.023f,-0.017f);Assert.InRange(m.Normal.Y,0.998f,1.002f);
            var empty=gen.Collide(ConvexShape3D.Empty,shape,pose);Assert.Equal(0,empty.PointCount);
            var compound=Compound3D.Create([new CompoundChild3D(shape,RigidTransformF.Identity)]);
            var hits=new List<ChildContact3D>();gen.CollideConvexCompound(surface,compound,pose,hits);
            Assert.Single(hits);Assert.InRange(hits[0].Manifold.MinSeparation,-0.023f,-0.017f);
            var apart=new RigidTransformF(QuaternionF.Identity,pose.Translation+new Vector3F(0,1,0));
            Assert.InRange(Gjk3D.Distance(surface,shape,apart).Distance,0.979f,0.981f);
            pairs++;
        }
        Assert.Equal(27,pairs);
    }

    [Fact]
    public void FrustumAndDecoratorsMeetMeshesAndHeightFieldsAndSensors()
    {
        foreach(bool field in new[]{false,true})foreach(bool sensor in new[]{false,true})
        {
            var w=new P3.PhysicsWorld3D();var floor=w.CreateBody(new P3.BodyDef { Type=BodyType.Static });
            var fd=new P3.ShapeDef { IsSensor=sensor };
            if(field)w.CreateHeightField(floor,[0,0,0,0],2,2,new(4,1,4),default,fd);
            else w.CreateMesh(floor,[new(0,0,0),new(0,0,4),new(4,0,0),new(4,0,4)],[0,1,2,2,1,3],fd);
            var b=w.CreateBody(new P3.BodyDef { Position=new(1,0.38,1) });
            w.CreateShape(b,P3.ShapeSpec3D.TaperedCylinder(0.4f,0.2f,0.3f).WithCenterOfMassOffset(new(0,-0.1f,0)));
            Assert.True(w.OverlapSphereAny(new(1,0.05,1),0.1f,new P3.QueryFilter3D { IncludeSensors=true }));
            w.Step(1f/60);
            Assert.True(w.Events.Length>0);
        }
    }

    [Fact]
    public void PlaneWorldQueryFormsAreAnalyticIncludingFarAndRotatedPlanes()
    {
        var w=new P3.PhysicsWorld3D();w.AddPlane(new(0,2,0),2,at:new(0,10,0));
        var from=new Vector3D(2000000,14,0);var down=-Vector3F.UnitY;
        Assert.True(w.Raycast(from,down,out var ray,10));Assert.InRange(ray.Distance,2.9999f,3.0001f);
        Assert.True(w.RaycastAny(from,down,10));Assert.Single(w.RaycastAll(from,down,10));
        Assert.True(w.Raycast(new(4194304,14,0),down,out _,10));
        var mover=ConvexShape3D.Sphere(0.5f);
        Assert.True(w.ShapeCast(mover,from,default,down,out var cast,10));Assert.InRange(cast.Distance,2.4999f,2.5001f);
        Assert.True(w.ShapeCastAny(mover,from,default,down,10));Assert.Single(w.ShapeCastAll(mover,from,default,down,10));
        Assert.True(w.OverlapSphere(new(0,11.2,0),0.3f,out _));Assert.True(w.OverlapSphereAny(new(0,11.2,0),0.3f));Assert.Single(w.OverlapSphereAll(new(0,11.2,0),0.3f));
        Assert.True(w.ClosestPoint(from,out var nearest,10));Assert.InRange(nearest.Point.Y,10.9999,11.0001);
        Assert.True(w.ClosestPointAny(from,10));Assert.Single(w.ClosestPointAll(from,10));
        var rotated=new P3.PhysicsWorld3D();rotated.AddPlane(Vector3F.UnitX,2);
        Assert.True(rotated.Raycast(new(5,3,4),-Vector3F.UnitX,out var x,10));Assert.InRange(x.Point.X,1.99999,2.00001);
    }

    [Fact]
    public void MassOffsetsComposeInCompoundsAndSurviveForkSceneAndReplay()
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=997});using var stream=new MemoryStream();
        using(var record=w.StartRecording(stream))
        {
            w.AddPlane(Vector3F.UnitY);
            var b=w.CreateBody(new P3.BodyDef { Position=new(0,2,0) });
            var compound=P3.ShapeSpec3D.Compound([new(P3.ShapeSpec3D.Sphere(1).WithCenterOfMassOffset(new(2,0,0)),default),new(P3.ShapeSpec3D.Sphere(1),default)]);
            w.CreateShape(b,compound.WithCenterOfMassOffset(new(0,-0.5f,0)));
            Assert.Equal(new Vector3F(1,-0.5f,0),b.CenterOfMass);
            w.Step(1f/60);
        }
        var clone=P3.PhysicsWorld3D.LoadScene(new StringReader(w.SaveScene()));Assert.Equal(w.Snapshot(),clone.Snapshot());
        var fork=w.Fork();for(int i=0;i<30;i++){w.Step(1f/60);fork.Step(1f/60);Assert.Equal(w.ComputeHash(),fork.ComputeHash());}
        stream.Position=0;var player=new ReplayPlayer(stream);var replay=new P3.PhysicsWorld3D(new(){WorldId=997});Assert.True(player.ReadNextStep(replay));Assert.Equal(clone.ComputeHash(),replay.ComputeHash());
        var w2=new P2.PhysicsWorld2D();var body=w2.CreateBody(new P2.BodyDef());w2.CreateShape(body,Shape2D.CreateBox(0.5f,0.5f).WithCenterOfMassOffset(new(0,-0.2f)));
        var f2=w2.Fork();for(int i=0;i<30;i++){w2.Step(1f/60);f2.Step(1f/60);Assert.Equal(w2.ComputeHash(),f2.ComputeHash());}
        Assert.Equal(w2.Snapshot(),P2.PhysicsWorld2D.LoadScene(new StringReader(w2.SaveScene())).Snapshot());
    }
}
