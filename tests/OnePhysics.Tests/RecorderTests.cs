using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RecorderTests
{
    [Fact]
    public void EveryPlanarStepMatchesWithMutationsOptionsRawWritesAndCallbackInputs()
    {
        var w=new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId=91 });
        w.AddBox(new(30,1),at:new(0,-0.5),type:BodyType.Static);
        var b=w.AddBox(new(1,1),at:new(0,0.51));
        int called=0;
        b.OnCollisionEnter += _ => { called++; b.ApplyImpulse(new(1,2)); w.Options.Gravity=new(0,-9); };
        using var stream=new MemoryStream();
        var expected=new List<ulong>();
        using(var recorder=w.StartRecording(stream))
        {
            Assert.Same(recorder,w.Recorder);
            Assert.Throws<InvalidOperationException>(()=>w.StartRecording(stream));
            for(int i=0;i<50;i++)
            {
                if(i==3) b.LinearVelocity=new(0.25f,0);
                if(i==8) { var p=w.Core.Bodies.PositionOf(b.RawHandle.Index); p[0]+=0.25; }
                if(i==12) w.AddCircle(0.2f,at:new(4,2));
                if(i==15) w.IgnoreLayerCollision(1,2);
                if(i==20) w.ShiftOrigin(new(1,0));
                if(i==30) w.Options.TimeToSleep=0.7f;
                w.Step(1f/60); expected.Add(w.ComputeHash());
            }
            Assert.Equal(50,recorder.RecordedSteps);
        }
        Assert.Null(w.Recorder); Assert.True(called>0);
        stream.Position=0;
        var player=new ReplayPlayer(stream);
        var copy=new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId=91 });
        int step=0;
        while(player.ReadNextStep(copy)) Assert.Equal(expected[step++],copy.ComputeHash());
        Assert.Equal(expected.Count,step); Assert.Equal(w.Snapshot(),copy.Snapshot());
        stream.Position=0; var reader=new ReplayReader(stream); var names=new HashSet<string>();
        while(reader.ReadNext()) if(reader.Kind==ReplayRecordKind.Mutation) names.Add(Encoding.UTF8.GetString(reader.OperationName));
        Assert.Contains("Body.ApplyImpulse",names); Assert.Contains("WorldOptions2D.Gravity",names); Assert.Contains("Body.LinearVelocity",names); Assert.Contains("PhysicsWorld2D.IgnoreLayerCollision",names);
    }

    [Fact]
    public void RecordingDoesNotChangeHashWhenCallbacksEditLiveOptions()
    {
        var a=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=95});
        var b=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=95});
        foreach(var w in new[]{a,b})
        {
            w.AddBox(new(20,1),at:new(0,-0.5),type:BodyType.Static);
            var ball=w.AddCircle(0.5f,at:new(0,0.49));
            ball.OnCollisionEnter += _=> { w.Options.Gravity=new(0,-7); ball.ApplyImpulse(new(0.1f,2)); };
        }
        using var stream=new MemoryStream();using var r=a.StartRecording(stream);
        for(int i=0;i<12;i++){a.Step(1f/60);b.Step(1f/60);Assert.Equal(b.ComputeHash(),a.ComputeHash());Assert.Equal(b.Snapshot(),a.Snapshot());}
    }

    [Fact]
    public void SnapshotRestoreCommandsMayRewindWorldCounterWithoutLosingRecordedSteps()
    {
        var w=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=92});
        w.AddSphere(0.5f,at:new(0,4,0));
        byte[] start=w.Snapshot();
        using var output=new MemoryStream();
        var expected=new List<ulong>();
        using(var r=w.StartRecording(output))
        {
            for(int i=0;i<12;i++)
            {
                if(i==6)w.Restore(start);
                w.Step(1f/60);expected.Add(w.ComputeHash());
            }
            Assert.Equal(12,r.RecordedSteps);Assert.Equal(6,w.StepCount);
        }
        output.Position=0;var p=new ReplayPlayer(output);var copy=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=92});int n=0;
        while(p.ReadNextStep(copy))Assert.Equal(expected[n++],copy.ComputeHash());
        Assert.Equal(12,n);Assert.Equal(w.Snapshot(),copy.Snapshot());
    }

    [Fact]
    public void RecorderCanBeDisposedInsideCallbackAndStreamIsLeftOpen()
    {
        var w=new P2.PhysicsWorld2D();
        w.AddBox(new(10,1),at:new(0,-0.5),type:BodyType.Static);
        var b=w.AddCircle(0.5f,at:new(0,0.49));
        using var output=new MemoryStream();var r=w.StartRecording(output);
        b.OnCollisionEnter += _=>r.Dispose();
        for(int i=0;i<10 && !r.IsStopped;i++)w.Step(1f/60);
        Assert.True(r.IsStopped);Assert.Null(w.Recorder);Assert.True(output.CanWrite);
        output.Position=0;var p=new ReplayPlayer(output);var copy=new P2.PhysicsWorld2D();while(p.ReadNextStep(copy)){}
    }

    [Fact]
    public void FailedMutatorsLeaveAPlayableUnchangedState()
    {
        var w=new P2.PhysicsWorld2D();var b=w.AddCircle(0.5f,at:new(0,2));
        using var output=new MemoryStream();
        using(var r=w.StartRecording(output))
        {
            Assert.Throws<ArgumentOutOfRangeException>(()=>b.LinearVelocity=new(float.NaN,0));
            w.Step(1f/60);
        }
        output.Position=0;var p=new ReplayPlayer(output);var copy=new P2.PhysicsWorld2D();Assert.True(p.ReadNextStep(copy));Assert.Equal(w.ComputeHash(),copy.ComputeHash());Assert.False(p.ReadNextStep(copy));
    }
}
