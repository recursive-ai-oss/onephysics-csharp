using System;
using System.IO;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class RodsTests(ITestOutputHelper output)
{
    [Fact]
    public void FreeTranslationIsIdenticalInBothDimensions()
    {
        var a = new P2.PhysicsWorld2D(new() { Gravity = default });
        var b = new P3.PhysicsWorld3D(new() { Gravity = default });
        var r = a.AddRod(new() { Points = [new(0,0),new(1,0),new(2,0)], Velocity = new(1,2), Damping = 0 });
        var s = b.AddRod(new() { Points = [new(0,0,0),new(1,0,0),new(2,0,0)], Velocity = new(1,2,0), Damping = 0 });
        for (int k = 0; k < 60; k++) { a.Step(1f/60); b.Step(1f/60); }
        for (int p = 0; p < 3; p++)
        {
            Assert.Equal(r.GetPosition(p).X,s.GetPosition(p).X); Assert.Equal(r.GetPosition(p).Y,s.GetPosition(p).Y);
            Assert.InRange(r.GetPosition(p).X,p+0.9999,p+1.0001); Assert.InRange(r.GetPosition(p).Y,1.9999,2.0001);
        }
    }

    [Fact]
    public void SnapshotRestoreForkAndStaleHandles()
    {
        var w = new P3.PhysicsWorld3D(); var rod = w.AddRope(new(0,2,0),new(2,2,0),8);
        for (int i = 0; i < 10; i++) w.Step(1f/60);
        var bytes = w.Snapshot(); var hash = w.ComputeHash();
        var fork = w.Fork(); Assert.Equal(hash,fork.ComputeHash());
        for (int i = 0; i < 10; i++) { w.Step(1f/60); fork.Step(1f/60); Assert.Equal(w.ComputeHash(),fork.ComputeHash()); }
        w.Restore(bytes); Assert.Equal(hash,w.ComputeHash()); Assert.Equal(bytes,w.Snapshot());
        rod.Destroy(); Assert.False(rod.IsValid); Assert.Throws<InvalidOperationException>(()=>rod.GetPosition(0));
        var next = w.AddRope(default,new(1,0,0),4); Assert.Equal(rod.Index,next.Index); Assert.False(rod.IsValid);
    }

    [Fact]
    public void CantileverUnderOwnWeightMatchesBeamTheory()
    {
        // Uniform distributed load q=rho*A*g; delta_tip=q*L^4/(8*E*I).
        var material = new RodMaterial(1e9f,4e8f,0.05f,1000);
        double expected = 1000*9.81/(2*1e9*0.05*0.05);
        foreach (int segments in new[] {8,16})
        foreach (int substeps in new[] {2,4,8})
        {
            var world = new P2.PhysicsWorld2D(); var points = new Vector2D[segments+1];
            for (int i = 0; i <= segments; i++) points[i] = new((double)i/segments,0);
            var rod = world.AddRod(new() { Points=points, Material=material, PinnedParticles=[0], ClampStart=true, Damping=10, Substeps=substeps });
            for (int i = 0; i < 300; i++) world.Step(1f/60);
            double actual = -rod.GetPosition(segments).Y;
            output.WriteLine($"beam segments={segments} substeps={substeps} deflection={actual:R} expected={expected:R} stretch={rod.MaximumStretch:R}");
            Assert.True(Math.Abs(actual-expected)/expected < 0.08,$"n={segments}, actual={actual:R}, expected={expected:R}, stretch={rod.MaximumStretch:R}");
        }
    }

    [Fact]
    public void HostileDefinitionsAreRejectedBeforeMutation()
    {
        var world = new P3.PhysicsWorld3D(); ulong hash = world.ComputeHash();
        Assert.Throws<ArgumentOutOfRangeException>(()=>world.AddRod(new() { Points=[default,default] }));
        Assert.Throws<ArgumentOutOfRangeException>(()=>world.AddRod(new() { Points=[new(double.NaN,0,0),new(1,0,0)] }));
        Assert.Throws<ArgumentOutOfRangeException>(()=>world.AddRod(new() { Points=[default,new(1,0,0)], PinnedParticles=[2] }));
        Assert.Throws<ArgumentOutOfRangeException>(()=>world.AddRod(new() { Points=[default,new(1,0,0)], Material=default }));
        Assert.Equal(hash,world.ComputeHash());
    }
    [Fact]
    public void TargetsSceneReplayRegionDebugAndAllocationFreeSteps()
    {
        var world = new P2.PhysicsWorld2D(new() { WorldId=785 });
        var rod = world.AddRope(new(0,2),new(1,2),8);
        using var stream = new MemoryStream(); var hashes = new List<ulong>();
        using (var recorder = world.StartRecording(stream))
        {
            for (int i = 0; i < 20; i++)
            {
                if (i == 4) rod.SetTarget(0,new(0.5,2.5));
                world.Step(1f/60); hashes.Add(world.ComputeHash());
            }
        }
        stream.Position=0; var player = new ReplayPlayer(stream); var copy = new P2.PhysicsWorld2D(new() { WorldId=785 }); int step=0;
        while (player.ReadNextStep(copy)) Assert.Equal(hashes[step++],copy.ComputeHash());
        Assert.Equal(20,step); Assert.Equal(world.Snapshot(),copy.Snapshot());
        Assert.Equal(world.Snapshot(),P2.PhysicsWorld2D.LoadScene(new StringReader(world.SaveScene())).Snapshot());
        var region = world.CaptureRegion(new(-2,-2),new(2,4));
        var divergent = world.Fork(); 
        var data = divergent.Engine.Rods.Data[rod.Index]!; data.PositionData[4] += new Vector3D(0.01,0,0);
        divergent.RestoreRegion(region); Assert.Equal(world.ComputeRegionHash(new(-2,-2),new(2,4)),divergent.ComputeRegionHash(new(-2,-2),new(2,4)));
        var debug = new DebugGeometry(); world.GetDebugGeometry(DebugContent.Rods,debug); Assert.Equal(8,debug.RodSegments.Length);
        debug.Clear(); Assert.Empty(debug.RodSegments.ToArray());
        for (int i = 0; i < 40; i++) world.Step(1f/60);
        long bytes = GC.GetAllocatedBytesForCurrentThread(); for (int i = 0; i < 40; i++) world.Step(1f/60);
        Assert.Equal(bytes,GC.GetAllocatedBytesForCurrentThread());
        world.ShiftOrigin(new(16,-8)); var snapshot=world.Snapshot(); var shifted=world.Fork(); shifted.Restore(snapshot); Assert.Equal(snapshot,shifted.Snapshot());
        world.Step(1f/60); shifted.Step(1f/60); Assert.Equal(world.ComputeHash(),shifted.ComputeHash());
    }

    [Fact]
    public void FlexibleRopeStaticSagApproachesCatenary()
    {
        const int n=32; const double length=2.4, span=2;
        double low=0.05, high=100;
        for (int k=0; k<80; k++) { double mid=(low+high)*0.5; if (2*mid*Math.Sinh(span/(2*mid))>length) low=mid; else high=mid; }
        double a=(low+high)*0.5, expected=a*(Math.Cosh(span/(2*a))-1);
        var points=new Vector2D[n+1]; double initialSag=Math.Sqrt(length*length/4-span*span/4);
        for (int i=0;i<=n;i++) points[i]=new(-1+2.0*i/n,-initialSag*(1-Math.Abs(2.0*i/n-1)));
        var world=new P2.PhysicsWorld2D(); var rod=world.AddRod(new() { Points=points, Material=new(1e8f,4e7f,0.001f,1000), PinnedParticles=[0,n], Damping=10, Substeps=8 });
        for (int i=0;i<300;i++) world.Step(1f/60);
        double actual=-rod.GetPosition(n/2).Y;
        output.WriteLine($"catenary sag={actual:R} expected={expected:R} relativeError={Math.Abs(actual/expected-1):R} maxStretch={rod.MaximumStretch:R}");
        Assert.InRange(Math.Abs(actual/expected-1),0,0.05); Assert.InRange(rod.MaximumStretch,0,0.01);
    }

    [Fact]
    public void FreePendulumDoesNotGainEnergyOverTenThousandSteps()
    {
        const int segments=8;
        var material=new RodMaterial(1e9f,4e8f,0.01f,1000);
        var points=new Vector2D[segments+1]; for (int i=0;i<=segments;i++) points[i]=new((double)i/segments,0);
        var world=new P2.PhysicsWorld2D();
        var rod=world.AddRod(new() { Points=points, Material=material, PinnedParticles=[0], Damping=0, Substeps=4 });
        double maxEnergy=double.NegativeInfinity, lastEnergy=0;
        var data=rod.Particles;
        for (int step=0;step<10000;step++)
        {
            world.Step(1f/60); double energy=0;
            for (int i=0;i<=segments;i++)
            {
                double mass=(i==0||i==segments?0.5:1)*Math.PI*0.01*0.01*1000/segments;
                var velocity=rod.GetVelocity(i); energy+=mass*(0.5*(velocity.X*velocity.X+velocity.Y*velocity.Y)+9.81*rod.GetPosition(i).Y);
            }
            for (int i=0;i<segments;i++) { double omega=data.AngularVelocity[i].Z; energy+=0.5*omega*omega/data.InverseBendInertia[i]; }
            maxEnergy=Math.Max(maxEnergy,energy); lastEnergy=energy;
        }
        output.WriteLine($"pendulum steps=10000 maximumEnergy={maxEnergy:R} finalEnergy={lastEnergy:R} initialEnergy=0 stretch={rod.MaximumStretch:R}");
        Assert.True(maxEnergy<0.001,$"Energy grew to {maxEnergy:R} J.");
    }

    [Fact]
    public void InvalidRawStateAndUnderflowingDurationLeaveTheWorldUnchanged()
    {
        var world=new P3.PhysicsWorld3D(); var rod=world.AddRope(default,new(1,0,0),4);
        var before=world.Snapshot();
        Assert.Throws<ArgumentOutOfRangeException>(()=>world.Step(float.Epsilon)); Assert.Equal(before,world.Snapshot());
        rod.Particles.Frames[1]=new QuaternionF(float.NaN,0,0,1); var invalid=world.Snapshot();
        Assert.Throws<RodStateException>(()=>world.Step(1f/60)); Assert.Equal(invalid,world.Snapshot());
        world.Restore(before); var current=world.GetRod(rod.Index); current.SetPinned(2,true); current.SetTarget(2,new(1,2,3));
        world.Step(1f/60); Assert.Equal(new Vector3D(1,2,3),current.GetPosition(2));
    }

}
