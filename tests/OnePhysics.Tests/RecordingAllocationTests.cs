using System;
using System.IO;
using RecursiveAI.OnePhysics.Replay;
using P2=RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

[Collection("Allocation")]
public sealed class RecordingAllocationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void WarmRecordingAndHistoryAllocateNothingPerStep(bool history)
    {
        var w=new P2.PhysicsWorld2D(new P2.WorldOptions2D{WorldId=103,Gravity=default,SleepEnabled=false});
        var body=w.AddCircle(0.5f,at:new(0,4));w.Warmup();
        using IDisposable recording=history?w.StartHistory(new HistoryOptions{KeyframeInterval=3,MemoryBudgetBytes=4*1024*1024}):w.StartRecording(Stream.Null);
        for(int i=0;i<300;i++){body.ApplyImpulse(new(0.0001f,0));w.Step(1f/60);}
        long before=GC.GetAllocatedBytesForCurrentThread();
        for(int i=0;i<100;i++){body.ApplyImpulse(new(0.0001f,0));w.Step(1f/60);}
        Assert.Equal(0,GC.GetAllocatedBytesForCurrentThread()-before);
    }
}
