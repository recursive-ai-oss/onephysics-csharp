using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RodSnapshotStepTests(ITestOutputHelper output)
{
    private static byte[] Baseline(bool planar)
    {
        if (planar)
        {
            var w=new P2.PhysicsWorld2D(new(){WorldId=36});w.AddBox(new(4f,1f),at:new(0,-.5),type:BodyType.Static);var disc=w.AddCircle(.1f,at:new(1,.5));
            var line=w.AddRope(new(0,.5),new(1,.5),6);line.Attach(6,disc,new(.05f,0),breakForce:1e6f);w.Step(1f/60);return w.Snapshot();
        }
        var world=new P3.PhysicsWorld3D(new(){WorldId=36});world.AddBox(new(4f,1f,4f),at:new(0,-.5,0),type:BodyType.Static);var ball=world.AddSphere(.1f,at:new(1,.5,0));
        var rope=world.AddRope(new(0,.5,0),new(1,.5,0),6);rope.Attach(6,ball,new(.05f,0,0),breakForce:1e6f);world.Step(1f/60);return world.Snapshot();
    }
    // The contract: a snapshot restore either rejects the bytes or yields a world whose Step completes, or raises a typed
    // physics exception and leaves the world exactly as it was.
    private static string? Try(bool planar,byte[] bytes)
    {
        if(planar)
        {
            var w=new P2.PhysicsWorld2D(new(){WorldId=36});
            try{w.Restore(bytes);}catch(SnapshotFormatException){return null;}
            ulong before=0;
            try{for(int i=0;i<4;i++){before=w.ComputeHash();w.Step(1f/60);}}
            catch(Exception e){return Check(e,before,w.ComputeHash());}
        }
        else
        {
            var w=new P3.PhysicsWorld3D(new(){WorldId=36});
            try{w.Restore(bytes);}catch(SnapshotFormatException){return null;}
            ulong before=0;
            try{for(int i=0;i<4;i++){before=w.ComputeHash();w.Step(1f/60);}}
            catch(Exception e){return Check(e,before,w.ComputeHash());}
        }
        return null;
    }
    private static string? Check(Exception e,ulong before,ulong after)
    {
        if(e is not PhysicsException&&e is not ArgumentOutOfRangeException)return e.GetType().Name+": "+e.Message;
        if(before!=after)return "world changed by a failed step: "+e.GetType().Name+": "+e.Message;
        return null;
    }
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void DamagedRodSnapshotsNeverHalfStep(bool planar)
    {
        var baseline=Baseline(planar);var rng=new Random(1234);int bad=0;var seen=new HashSet<string>();
        for(int n=0;n<800;n++)
        {
            var bytes=(byte[])baseline.Clone();int flips=1+rng.Next(3);
            for(int f=0;f<flips;f++){int off=bytes.Length-1-rng.Next(Math.Min(1200,bytes.Length));bytes[off]^=(byte)(1<<rng.Next(8));if(rng.Next(3)==0)bytes[off]=(byte)rng.Next(256);}
            var problem=Try(planar,bytes);
            if(problem!=null){bad++;if(seen.Add(problem))output.WriteLine(problem);}
        }
        Assert.Equal(0,bad);
    }
    // Minimal repros: one flipped exponent bit in a particle's velocity (a finite but enormous value) used to throw out of the middle
    // of the step, after the rigid bodies had moved.
    [Theory]
    [InlineData(true,361,6)]
    [InlineData(false,361,6)]
    [InlineData(true,393,5)]
    [InlineData(false,442,5)]
    public void EnormousRodVelocityDoesNotHalfStep(bool planar,int back,int bit)
    {
        var bytes=Baseline(planar);bytes[bytes.Length-1-back]^=(byte)(1<<bit);
        Assert.Null(Try(planar,bytes));
    }
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void SingleBitScan(bool planar)
    {
        var baseline=Baseline(planar);output.WriteLine("length "+baseline.Length);int bad=0;
        for(int back=330;back<660;back++)
            for(int bit=0;bit<8;bit++)
            {
                var bytes=(byte[])baseline.Clone();bytes[bytes.Length-1-back]^=(byte)(1<<bit);
                var problem=Try(planar,bytes);if(problem!=null){bad++;output.WriteLine($"back {back} bit {bit}: {problem}");}
            }
        Assert.Equal(0,bad);
    }
}
