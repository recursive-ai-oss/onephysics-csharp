using System;
using RecursiveAI.OnePhysics.Replay;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class HistoryTests
{
    [Fact]
    public void PlanarRandomSeeksMatchCompletedStepsNotFollowingInputMutations()
    {
        var w=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=93});
        w.AddBox(new(30,1),at:new(0,-0.5),type:BodyType.Static);
        var b=w.AddCircle(0.4f,at:new(0,3));
        var expected=new ulong[61];expected[0]=w.ComputeHash();
        using var h=w.StartHistory(new HistoryOptions{KeyframeInterval=7,MemoryBudgetBytes=4*1024*1024});
        for(int i=1;i<=60;i++)
        {
            if(i%3==0)b.ApplyImpulse(new(0.1f,0));
            if(i==30)w.ShiftOrigin(new(1,0));
            w.Step(1f/60);expected[i]=w.ComputeHash();
        }
        b.ApplyForce(new(3,0)); // not part of the state returned by Step 60
        var copy=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=93});
        for(int j=0;j<61;j++) { int i=j*17%61;h.Seek(i,copy);Assert.Equal(expected[i],copy.ComputeHash()); }
        Assert.Equal(60,h.LastStep);Assert.True(h.RetainedBytes<=h.MemoryBudgetBytes);
        long count=w.Recorder!.RecordedSteps;
        h.Seek(7,w);Assert.Equal(expected[7],w.ComputeHash());Assert.Equal(count,w.Recorder.RecordedSteps);
        h.Stop();Assert.False(h.IsRecording);h.Seek(60,copy);Assert.Equal(expected[60],copy.ComputeHash());
    }

    [Fact]
    public void BudgetEvictsWholeIntervalsAndAllRetainedSpatialStepsStillMatch()
    {
        var w=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=94,Gravity=default});
        var b=w.AddSphere(0.5f,at:new(0,4,0));
        int budget=Math.Max(65536,w.GetSnapshotSizeBound()*4+8192);
        using var h=w.StartHistory(new HistoryOptions{KeyframeInterval=3,MemoryBudgetBytes=budget});
        var expected=new ulong[401];expected[0]=w.ComputeHash();
        for(int i=1;i<=400;i++){b.LinearVelocity=new(0.1f,0,0);w.Step(1f/60);expected[i]=w.ComputeHash();}
        Assert.True(h.FirstStep>0);Assert.Equal(400,h.LastStep);Assert.True(h.RetainedBytes<=budget);
        var copy=new P3.PhysicsWorld3D(new P3.WorldOptions3D{WorldId=94});
        for(long i=h.FirstStep;i<=h.LastStep;i++){h.Seek(i,copy);Assert.Equal(expected[i],copy.ComputeHash());}
        Assert.Throws<ArgumentOutOfRangeException>(()=>h.Seek(h.FirstStep-1,copy));
        Assert.Throws<ArgumentOutOfRangeException>(()=>h.Seek(h.LastStep+1,copy));
        Assert.Throws<ArgumentException>(()=>h.Seek(h.LastStep,new P2.PhysicsWorld2D()));
    }

    [Fact]
    public void ImpossibleBudgetsFailClearlyAndDisposedHistoryCannotSeek()
    {
        var w=new P2.PhysicsWorld2D();w.AddCircle(0.5f,at:new(0,2));
        Assert.Throws<HistoryCapacityException>(()=>w.StartHistory(new HistoryOptions{MemoryBudgetBytes=4096}));
        Assert.Null(w.Recorder);
        var h=w.StartHistory();h.Dispose();Assert.Null(w.Recorder);
        Assert.Throws<ObjectDisposedException>(()=>h.Seek(0,w));
    }
}
