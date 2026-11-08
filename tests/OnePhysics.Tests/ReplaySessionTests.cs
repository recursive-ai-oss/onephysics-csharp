using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class ReplaySessionTests
{
    [Fact]
    public void RecordedSpatialSessionReconstructsAllServicesAndEveryHash()
    {
        var w=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=97,BroadphaseMargin=0.25});
        w.AddBox(new(100,1,100),at:new(0,-0.5,0),type:BodyType.Static);
        var carBody=w.AddBox(new(2,0.5f,4),at:new(0,3,0));
        var car=w.AddVehicle(carBody,P3.VehicleDef.Car);
        var character=w.AddCharacter(new(5,1,0));
        var rag=w.AddRagdoll(P3.RagdollDef.Humanoid,new(10,2,0));
        var poses=new QuaternionF[rag.PartCount];Array.Fill(poses,QuaternionF.Identity);
        var mesh=P3.SoftBodyMesh.FromTriangles(new Vector3F[]{new(0,0,0),new(1,0,0),new(0,0,1)},new[]{0,1,2});
        var soft=w.AddSoftBody(P3.SoftBodyDef.Cloth(mesh,new(15,4,0)));
        w.AddWater(-2);
        var anchor=w.AddBox(new(1,1,1),at:new(-8,4,0),type:BodyType.Static);
        var bob=w.AddSphere(0.4f,at:new(-7,4,0));
        var joint=w.AddFixed(anchor,bob,new(-7.5,4,0));joint.BreakForce=0.001f;
        int broken=0;joint.OnJointBreak += _=>{broken++;bob.ApplyImpulse(new(0,1,0));};
        var hashes=new ulong[32];using var stream=new MemoryStream();
        using(var recorder=w.StartRecording(stream))
        {
            for(int i=0;i<hashes.Length;i++)
            {
                car.Drive(0.2f,0.05f);
                character.MoveVelocity(new(0.25f,0,0),1f/60);
                if(i%4==0){soft.ApplyImpulse(0,new(0.001f,0,0));rag.DriveToPose(poses,0.05f);}
                if(i==3)w.AddCapsule(0.5f,0.2f,at:new(-3,6,0));
                if(i==8)carBody.GetShapes()[0].Friction=0.7f;
                if(i==10)w.ShiftOrigin(new(1,0,0));
                if(i==16)w.Options.Gravity=new(0,-8,0);
                if(i==24)soft.SetPinned(1,true);
                w.Step(1f/60);hashes[i]=w.ComputeHash();
            }
        }
        Assert.True(broken>0);
        stream.Position=0;var player=new ReplayPlayer(stream);var copy=player.CreateWorld3D();
        Assert.Equal(97,copy.Id);int n=0;
        while(player.ReadNextStep(copy))Assert.Equal(hashes[n++],copy.ComputeHash());
        Assert.Equal(hashes.Length,n);Assert.Equal(w.Snapshot(),copy.Snapshot());
    }

    [Fact]
    public void PlanarCharactersBreakingJointsAndDestructionReplayAndSeek()
    {
        var w=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=98});
        var floor=w.AddBox(new(30,1),at:new(0,-0.5),type:BodyType.Static);
        var c=w.AddCharacter(new(4,1));
        var b=w.AddBox(new(1,1),at:new(0,3));
        var joint=w.AddFixed(floor,b,new(0,1));joint.BreakForce=0.001f;
        int broken=0;joint.OnJointBreak += _=>broken++;
        using var h=w.StartHistory(new HistoryOptions{KeyframeInterval=4,MemoryBudgetBytes=8*1024*1024});
        var hashes=new ulong[25];hashes[0]=w.ComputeHash();P2.Body extra=default;
        for(int i=1;i<hashes.Length;i++)
        {
            c.MoveVelocity(new(0.5f,0),1f/60);
            if(i==3)extra=w.AddCircle(0.3f,at:new(-3,4));
            if(i==7)extra.Destroy();
            if(i==12)w.ShiftOrigin(new(1,0));
            w.Step(1f/60);hashes[i]=w.ComputeHash();
        }
        Assert.True(broken>0);
        var copy=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=98});
        for(int j=0;j<25;j++){int i=j*11%25;h.Seek(i,copy);Assert.Equal(hashes[i],copy.ComputeHash());}
    }

    [Fact]
    public void SavedSpatialHistorySeeksAcrossServiceInputsAndOriginShift()
    {
        var w=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=99});
        w.AddBox(new(60,1,60),at:new(0,-0.5,0),type:BodyType.Static);
        var chassis=w.AddBox(new(2,0.5f,4),at:new(0,3,0));var car=w.AddVehicle(chassis,P3.VehicleDef.Car);
        var c=w.AddCharacter(new(5,1,0));var rag=w.AddRagdoll(P3.RagdollDef.Humanoid,new(10,1,0));
        var poses=new QuaternionF[rag.PartCount];Array.Fill(poses,QuaternionF.Identity);
        var mesh=P3.SoftBodyMesh.FromTriangles(new Vector3F[]{new(0,0,0),new(1,0,0),new(0,0,1)},new[]{0,1,2});
        var soft=w.AddSoftBody(P3.SoftBodyDef.Cloth(mesh,new(14,4,0)));
        w.AddWater(-2);
        using var h=w.StartHistory(new HistoryOptions{KeyframeInterval=3,MemoryBudgetBytes=16*1024*1024});
        var hashes=new ulong[19];hashes[0]=w.ComputeHash();
        for(int i=1;i<hashes.Length;i++)
        {
            car.Drive(0.2f);c.MoveVelocity(new(0.1f,0,0),1f/60);rag.DriveToPose(poses,0.01f);soft.AddVelocity(new(0.001f,0,0));
            if(i==6)w.ShiftOrigin(new(1,0,0));
            w.Step(1f/60);hashes[i]=w.ComputeHash();
        }
        var copy=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=99});
        for(int j=0;j<19;j++){int i=j*7%19;h.Seek(i,copy);Assert.Equal(hashes[i],copy.ComputeHash());}
    }
}
