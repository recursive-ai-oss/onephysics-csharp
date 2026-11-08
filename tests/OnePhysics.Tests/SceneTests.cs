using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Scene;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;
using Document = RecursiveAI.OnePhysics.Scene.Scene;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class SceneTests
{
    private const string Planar = "scene 1\nname test\ndim 2\nmaterial 0 0.6 0\nbody dynamic 1.0000000002328306 3 0 0 0 0 -\nshape box 0 1 0 0.5 0.5 0 0 0\n";
    [Fact]
    public void LegacyDefinitionIsPublicAndDoublePositionsRoundTrip()
    {
        var scene = Document.Read(new StringReader(Planar), "example");
        using var output = new StringWriter(); scene.Write(output);
        var copy = Document.Read(new StringReader(output.ToString()));
        using var second = new StringWriter(); copy.Write(second);
        Assert.Equal(output.ToString(), second.ToString());
        Assert.Equal(BitConverter.DoubleToInt64Bits(1.0000000002328306), BitConverter.DoubleToInt64Bits(copy.Bodies[0].X));
        Assert.Equal(1, scene.CreateWorld2D().BodyCount);
    }

    [Theory]
    [InlineData("scene 1\ndim 2\nbody dynamic 0 0\n",3)]
    [InlineData("scene 1\ndim 2\nshape circle 0 1 1 0 0\n",3)]
    [InlineData("scene 1\ndt NaN\n",2)]
    [InlineData("scene 3\n",1)]
    [InlineData("scene 1\nmaterial 1 0.6 0\n",2)]
    [InlineData("scene 1\nbody dynamic 2097153 0 0 0 0 0 -\n",2)]
    [InlineData("scene 1\nmaterial 0 0.6 0\nbody dynamic 0 0 0 0 0 0 -\nshape polygon 0 1 0 0 0 1 0 2 0\n",4)]
    [InlineData("scene 1\njoint revolute 0 1 0 0 0 0\n",2)]
    [InlineData("scene 2\ndim 2\nworld_id 1\nstate 67108865\n",4)]
    [InlineData("scene 2\ndim 2\nworld_id 1\nstate 4\ndata AQID\nend\n",6)]
    public void HostileTextHasTypedNamedLineErrors(string text,int line)
    {
        var e=Assert.Throws<SceneFormatException>(()=>Document.Read(new StringReader(text),"hostile"));
        Assert.Equal("hostile",e.InputName); Assert.Equal(line,e.Line);
    }

    [Fact]
    public void PlanarWorldSavesAllStateCanonically()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId=82 });
        w.AddBox(new(20,1),at:new(0,-0.5),type:BodyType.Static);
        var b=w.AddCircle(0.4f,at:new(0,1));
        w.AddCharacter(new(5,1));
        w.SetLayerCollision(2,3,false);
        for(int i=0;i<12;i++)w.Step(1f/60);
        b.LinearVelocity=new(1,2);
        w.Options.Gravity=new(0,-4);
        ulong before=w.ComputeHash();
        string text=w.SaveScene();
        Assert.Equal(before,w.ComputeHash());
        var copy=P2.PhysicsWorld2D.LoadScene(new StringReader(text));
        Assert.Equal(w.ComputeHash(),copy.ComputeHash());
        Assert.Equal(text,copy.SaveScene());
        for(int i=0;i<10;i++){w.Step(1f/60);copy.Step(1f/60);Assert.Equal(w.ComputeHash(),copy.ComputeHash());}
    }

    [Fact]
    public void SpatialWorldIncludesServicesRagdollsSoftBodiesAndCaches()
    {
        var w=new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId=83 });
        w.AddBox(new(100,1,100),at:new(0,-0.5,0),type:BodyType.Static);
        w.AddRagdoll(P3.RagdollDef.Humanoid,new(8,1,0));
        var chassis=w.AddBox(new(2,0.5f,4),at:new(0,3,0));
        w.AddVehicle(chassis,P3.VehicleDef.Car);
        w.AddWater(0);
        w.AddCharacter(new(5,1,0));
        var mesh=P3.SoftBodyMesh.FromTriangles(new Vector3F[]{new(0,0,0),new(1,0,0),new(0,0,1)},new[]{0,1,2});
        w.AddSoftBody(P3.SoftBodyDef.Cloth(mesh,new(12,4,0)));
        for(int i=0;i<8;i++)w.Step(1f/60);
        w.ShiftOrigin(new(2,0,0));
        string text=w.SaveScene();
        var copy=P3.PhysicsWorld3D.LoadScene(new StringReader(text));
        Assert.Equal(w.ComputeHash(),copy.ComputeHash()); Assert.Equal(text,copy.SaveScene());
        for(int i=0;i<8;i++){w.Step(1f/60);copy.Step(1f/60);Assert.Equal(w.ComputeHash(),copy.ComputeHash());}
    }

    [Fact]
    public void CorruptedStatePayloadIsTypedAndDimensionChecked()
    {
        var scene=Document.FromSnapshot(2,1,new byte[]{1,2,3});
        Assert.Throws<SceneFormatException>(()=>scene.CreateWorld2D());
        Assert.Throws<SceneFormatException>(()=>scene.CreateWorld3D());
    }
}
