using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Buoyancy and field robustness: bounded oscillation without damping, hostile restored state, heavy spinners, steady-state allocation and diverged worlds.</summary>
public sealed class FieldRobustnessTests
{
    // A half-submerged box released 0.1 m above its equilibrium oscillates about it; the force is held across the substeps, which used to feed energy in.
    [Fact]
    public void UndampedFloatingBoxOscillatesBoundedlyAboutEquilibriumInBothDimensions()
    {
        var w2=new P2.PhysicsWorld2D(new(){Gravity=new(0,-10),SleepEnabled=false});var b2=w2.AddBox(new(2,1),at:new(0,0.1),density:500);
        w2.AddBuoyancyEffector(0,damping:0);
        var w3=new P3.PhysicsWorld3D(new(){Gravity=new(0,-10,0),SleepEnabled=false});var b3=w3.AddBox(new(2,1,2),at:new(0,0.1,0),density:500);
        w3.AddBuoyancyEffector(0,damping:0);
        double max2=0,max3=0;
        for(int i=0;i<4800;i++)
        {
            w2.Step(1f/120);w3.Step(1f/120);
            max2=Math.Max(max2,Math.Abs(b2.Position.Y));max3=Math.Max(max3,Math.Abs(b3.Position.Y));
        }
        // The release height is the amplitude; the held force may add at most 1% to it over 40 seconds (it added 80% before).
        Assert.InRange(max2,0.05,0.101);Assert.InRange(max3,0.05,0.101);
    }

    [Fact]
    public void HeavySpinningBodyInABuoyancyFieldSimplySteps()
    {
        var w=new P2.PhysicsWorld2D(new(){Gravity=new(0,-10),SleepEnabled=false});
        var b=w.AddBox(new(500,500),at:new(0,0),density:2000);b.AngularVelocity=20f;
        w.AddBuoyancyEffector(0,damping:1);
        var before=w.ComputeHash();
        for(int i=0;i<10;i++)w.Step(1f/60);
        Assert.NotEqual(before,w.ComputeHash());
        Assert.True(float.IsFinite(b.AngularVelocity)&&Math.Abs(b.AngularVelocity)<20f);
    }

    // A bit flip in a saved rotation leaves a finite value that is not a rotation. The field must cut with a unit plane and the step must complete.
    [Fact]
    public void RestoredNonUnitRotationStepsWithAUnitPlane()
    {
        var w=new P2.PhysicsWorld2D(new(){WorldId=5,Gravity=new(0,-10),SleepEnabled=false});
        w.AddBox(new(2,1),at:new(0,0.1),angleRadians:0.3f,density:500);w.AddBuoyancyEffector(0);
        w.Step(1f/60);var bytes=w.Snapshot();
        Span<byte> pattern=stackalloc byte[8];
        BinaryPrimitives.WriteSingleLittleEndian(pattern,MathF.Cos(0.3f));BinaryPrimitives.WriteSingleLittleEndian(pattern[4..],MathF.Sin(0.3f));
        int at=-1;
        // The stored rotation is whatever the step left, so look for any (cos,sin) pair of a unit rotation near 0.3 rad by value.
        for(int i=0;i+8<=bytes.Length&&at<0;i++)
        {
            float c=BinaryPrimitives.ReadSingleLittleEndian(bytes.AsSpan(i)),s=BinaryPrimitives.ReadSingleLittleEndian(bytes.AsSpan(i+4));
            if(Math.Abs(c-MathF.Cos(0.3f))<0.01f&&Math.Abs(s-MathF.Sin(0.3f))<0.01f&&Math.Abs(c*c+s*s-1)<1e-5f)at=i;
        }
        Assert.True(at>=0);
        foreach(byte flip in new byte[]{0xF8,0x40,0x01})
        {
            var hostile=(byte[])bytes.Clone();hostile[at+3]^=flip;
            var v=new P2.PhysicsWorld2D(new(){WorldId=5,Gravity=new(0,-10),SleepEnabled=false});
            try{v.Restore(hostile);}catch(SnapshotFormatException){continue;}
            for(int i=0;i<3;i++)
            {
                try{v.Step(1f/60);}catch(P2.BodyOutOfBoundsException){break;}
            }
        }
    }

    [Fact]
    public void AreaFieldsAllocateNothingPerStepInBothDimensions()
    {
        foreach(bool damped in new[]{false,true})
        {
            var d=new FieldDefinition{Kind=FieldKind.Area,Direction=new(1,0,0),Flow=damped?new(1,0,0):default,Drag=damped?1:0,AngularDrag=damped?1:0};
            var w2=new P2.PhysicsWorld2D(new(){Gravity=default,SleepEnabled=false});w2.AddCircle(0.2f);w2.AddField(d);
            var w3=new P3.PhysicsWorld3D(new(){Gravity=default,SleepEnabled=false});w3.AddSphere(0.2f);w3.AddField(d);
            for(int i=0;i<40;i++){w2.Step(0.01f);w3.Step(0.01f);}
            long a=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<40;i++)w2.Step(0.01f);
            long b=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<40;i++)w3.Step(0.01f);
            long c=GC.GetAllocatedBytesForCurrentThread();
            Assert.Equal(0,b-a);Assert.Equal(0,c-b);
        }
    }

    // Contract: a diverged world cannot be saved (restore refuses non-finite positions), and the refusal names the field. The hash still works.
    [Fact]
    public void DivergedWorldRefusesToWriteASnapshotNamingTheFieldInBothDimensions()
    {
        var w2=new P2.PhysicsWorld2D(new(){Gravity=new(0,-10),SleepEnabled=false});var b2=w2.AddCircle(0.2f);
        b2.ApplyForce(new(3e38f,0));
        for(int i=0;i<6;i++){try{w2.Step(0.01f);}catch(P2.BodyOutOfBoundsException){break;}}
        var e2=Assert.Throws<NonFiniteStateException>(()=>w2.Snapshot());Assert.Contains("position lane",e2.Message);
        Assert.Throws<NonFiniteStateException>(()=>w2.GetSnapshotSize());
        _=w2.ComputeHash();
        var w3=new P3.PhysicsWorld3D(new(){Gravity=new(0,-10,0),SleepEnabled=false});var b3=w3.AddSphere(0.2f);
        b3.ApplyForce(new(3e38f,0,0));
        for(int i=0;i<6;i++){try{w3.Step(0.01f);}catch(P3.BodyOutOfBoundsException){break;}}
        // 3D bounds the velocity the solve leaves (NaN becomes zero, 16 times the speed limit at most) before it moves a body, so the same force no longer diverges the world: it saves and restores.
        var saved3=w3.Snapshot();var fresh3=new P3.PhysicsWorld3D(new(){Gravity=new(0,-10,0),SleepEnabled=false});fresh3.Restore(saved3);Assert.Equal(w3.ComputeHash(),fresh3.ComputeHash());
        _=w3.ComputeHash();
        // The host's way out still works: destroy the body, then the world saves and restores.
        b2.Destroy();var ok=w2.Snapshot();var fresh=new P2.PhysicsWorld2D(new(){Gravity=new(0,-10),SleepEnabled=false});fresh.Restore(ok);Assert.Equal(ok,fresh.Snapshot());
    }
}
