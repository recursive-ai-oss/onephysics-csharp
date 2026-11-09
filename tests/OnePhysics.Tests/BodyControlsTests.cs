using System;
using System.IO;
using System.Linq;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class BodyControlsTests
{
    [Fact]
    public void EveryLockCombinationHoldsForTenThousandContactAndJointSteps()
    {
        var a = new P2.PhysicsWorld2D(new() { SleepEnabled = false });
        var b = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        var ground2 = a.AddBox(new(200,1),at:new(30,-0.5),type:BodyType.Static);
        var ground3 = b.AddBox(new(400,1,10),at:new(100,-0.5,0),type:BodyType.Static);
        var planar = new P2.Body[8]; var spatial = new P3.Body[64];
        for (int i=0;i<8;i++) { var x=a.AddBox(new(1,1),at:new(i*3,0.45)); x.Constraints=(RigidbodyConstraints)((i&3)|((i&4)<<3)); a.AddFixed(ground2,x,x.Position); planar[i]=x; }
        for (int i=0;i<64;i++) { var x=b.AddBox(new(1,1,1),at:new(i*3,0.45,0)); x.Constraints=(RigidbodyConstraints)i; b.AddFixed(ground3,x,x.Position); spatial[i]=x; }
        for(int step=0;step<10000;step++)
        {
            if (step%100==0) { foreach(var x in planar) { x.ApplyForce(new(1,-2)); x.ApplyTorque(1); } foreach(var x in spatial) { x.ApplyForce(new(1,-2,1)); x.ApplyTorque(new(1,2,3)); } }
            a.Step(1f/60); b.Step(1f/60);
            for(int i=0;i<8;i++) { var x=planar[i]; if((i&1)!=0) { Assert.Equal(i*3d,x.WorldCenterOfMass.X); Assert.Equal(0,x.LinearVelocity.X); } if((i&2)!=0) { Assert.Equal(0.45,x.WorldCenterOfMass.Y); Assert.Equal(0,x.LinearVelocity.Y); } if((i&4)!=0) { Assert.Equal(0,x.AngularVelocity); Assert.Equal(0,x.Angle); } }
            for(int i=0;i<64;i++) { var x=spatial[i]; var p=x.WorldCenterOfMass; var v=x.LinearVelocity; var w=x.AngularVelocity; if((i&1)!=0) { Assert.Equal(i*3d,p.X); Assert.Equal(0,v.X); } if((i&2)!=0) { Assert.Equal(0.45,p.Y); Assert.Equal(0,v.Y); } if((i&4)!=0) { Assert.Equal(0,p.Z); Assert.Equal(0,v.Z); } if((i&8)!=0) Assert.Equal(0,w.X); if((i&16)!=0) Assert.Equal(0,w.Y); if((i&32)!=0) Assert.Equal(0,w.Z); }
        }
    }

    [Fact]
    public void MassCenterAndPrincipalInertiaGiveAnalyticImpulseAndTorqueResponses()
    {
        var a=new P2.PhysicsWorld2D(new(){Gravity=default}); var x=a.AddBox(new(2,2));
        float computed=x.Mass; x.Mass=4; x.SetInertia(2); x.CenterOfMass=new(1,0);
        Assert.Equal(new Vector2D(0,0),x.Position); Assert.Equal(new Vector2D(1,0),x.WorldCenterOfMass);
        x.ApplyForce(new(8,0)); x.ApplyTorque(4); a.Step(0.5f); Assert.InRange(x.LinearVelocity.X,0.99999f,1.00001f); Assert.InRange(x.AngularVelocity,0.99999f,1.00001f);
        x.ResetMass(); Assert.Equal(computed,x.Mass); x.ResetCenterOfMass(); Assert.Equal(default,x.CenterOfMass); x.ResetInertia(); Assert.True(x.Inertia>0);
        var b=new P3.PhysicsWorld3D(new(){Gravity=default}); var y=b.AddBox(new(2,2,2)); float natural=y.Mass;
        y.Mass=4; y.SetInertia(new(2,3,4),QuaternionF.CreateFromAxisAngle(new(0,0,1),1.5707964f)); y.CenterOfMass=new(1,0,0);
        Assert.Equal(new Vector3D(0,0,0),y.Position); y.ApplyForce(new(8,0,0)); y.ApplyTorque(new(6,0,0)); b.Step(0.5f);
        Assert.InRange(y.LinearVelocity.X,0.9999f,1.0001f); Assert.InRange(y.AngularVelocity.X,0.99f,1.01f);
        y.ResetMass(); Assert.Equal(natural,y.Mass); y.ResetCenterOfMass(); y.ResetInertia(); Assert.Equal(default,y.CenterOfMass);
    }

    [Fact]
    public void TargetsReachExactGridPoseAndQueuedComponentsCompose()
    {
        var a=new P2.PhysicsWorld2D(); var x=a.CreateBody(new(){Type=BodyType.Kinematic});
        var b=new P3.PhysicsWorld3D(); var y=b.CreateBody(new(){Type=BodyType.Kinematic});
        x.MovePosition(new(1.25,2.5)); x.MoveRotation(0.5f); y.MovePosition(new(1.25,2.5,3.75)); var rotation=QuaternionF.CreateFromAxisAngle(new(0,1,0),0.5f); y.MoveRotation(rotation);
        a.Step(1f/60); b.Step(1f/60); Assert.Equal(new Vector2D(1.25,2.5),x.Position); Assert.Equal(new Vector3D(1.25,2.5,3.75),y.Position);
        Assert.Equal(P3.Engine3D.Normalize(rotation,"rotation"),y.Rotation);
        var p=x.Position; var q=y.Position; a.Step(1f/60); b.Step(1f/60); Assert.Equal(p,x.Position); Assert.Equal(q,y.Position);
    }

    [Fact]
    public void KinematicPlatformsCarryAndPushDynamicBoxes()
    {
        var a=new P2.PhysicsWorld2D(); var platform=a.AddBox(new(8,1),at:new(0,-0.5),type:BodyType.Kinematic); var x=a.AddBox(new(1,1),at:new(0,0.51));
        var b=new P3.PhysicsWorld3D(); var platform3=b.AddBox(new(8,1,8),at:new(0,-0.5,0),type:BodyType.Kinematic); var y=b.AddBox(new(1,1,1),at:new(0,0.51,0));
        for(int i=0;i<60;i++){a.Step(1f/60);b.Step(1f/60);}
        for(int i=1;i<=60;i++){platform.MovePosition(new(i*0.025,-0.5));platform3.MovePosition(new(i*0.025,-0.5,0));a.Step(1f/60);b.Step(1f/60);}
        Assert.True(x.Position.X>0.8);Assert.True(y.Position.X>0.8);
    }

    [Fact]
    public void CapsAndOverridesSurviveFreshSnapshotsScenesForksAndReplay()
    {
        var a=new P2.PhysicsWorld2D(new(){Gravity=default,MaxAngularSpeed=0.25f});var x=a.AddBox(new(1,1));
        using var stream=new MemoryStream();
        using(a.StartRecording(stream)) { x.Mass=3;x.CenterOfMass=new(0.2f,0);x.SetInertia(2);x.Controls=new(RigidbodyConstraints.FreezePositionY,0.5f,0.1f,0.2f);x.LinearVelocity=new(20,20);x.AngularVelocity=20;a.Step(1f/60); }
        Assert.InRange(x.LinearVelocity.X,0,0.5f);Assert.Equal(0,x.LinearVelocity.Y);Assert.InRange(x.AngularVelocity,-0.1f,0.1f);
        var copy=new P2.PhysicsWorld2D();copy.Restore(a.Snapshot());Assert.Equal(a.Snapshot(),copy.Snapshot());Assert.Equal(a.Snapshot(),a.Fork().Snapshot());Assert.Equal(a.Snapshot(),P2.PhysicsWorld2D.LoadScene(new StringReader(a.SaveScene())).Snapshot());
        stream.Position=0;var player=new ReplayPlayer(stream);var played=player.CreateWorld2D();Assert.True(player.ReadNextStep(played));Assert.Equal(a.Snapshot(),played.Snapshot());
        var b=new P3.PhysicsWorld3D(new(){Gravity=default,MaxAngularSpeed=0.2f});var y=b.AddBox(new(1,1,1));y.SetInertia(new(1,2,3),QuaternionF.Identity);y.Mass=3;y.CenterOfMass=new(0.1f,0,0);y.Controls=new(RigidbodyConstraints.FreezePositionZ,0.5f,0.1f,0.1f);y.LinearVelocity=new(20,20,20);y.AngularVelocity=new(20,20,20);b.Step(1f/60);
        Assert.True(y.LinearVelocity.Length()<=0.50001f);Assert.True(y.AngularVelocity.Length()<=0.10001f);Assert.Equal(0,y.LinearVelocity.Z);
        var copy3=new P3.PhysicsWorld3D();copy3.Restore(b.Snapshot());Assert.Equal(b.Snapshot(),copy3.Snapshot());Assert.Equal(b.Snapshot(),b.Fork().Snapshot());Assert.Equal(b.Snapshot(),P3.PhysicsWorld3D.LoadScene(new StringReader(b.SaveScene())).Snapshot());
    }
    [Fact]
    public void AuthoringGrammarRoundTripsAllControlPropertiesInBothDimensions()
    {
        const string planar = "scene 1\nname controls\ndim 2\nworld_limits 200 20 1\nmaterial 0 0.6 0\nbody dynamic 0 3 0 0 0 0 -\ncontrols 32 4 3 0.1\nsleep 1 0.1 0.2 0.3\nmass 2\ncenter 0.25 0\ninertia 3\nshape box 0 1 0 0.5 0.5 0 0 0\nbody kinematic 0 0 0 0 0 0 -\ntarget 1 0 0.25\n";
        const string spatial = "scene 1\nname controls\ndim 3\nmaterial 0 0.6 0\nbody dynamic 0 3 0 0 0 0 1 0 0 0 0 0 0 -\ncontrols 8 4 3 0.1\nsleep 1 0.1 0.2 0.3\nmass 2\ncenter 0.25 0 0\ninertia 2 3 4 0 0 0 1\nshape box 0 1 0 0.5 0.5 0.5 0 0 0 0 0 0 1\nbody kinematic 0 0 0 0 0 0 1 0 0 0 0 0 0 -\ntarget 1 0 0 0 0 0 1\n";
        foreach(string text in new[]{planar,spatial})
        {
            var scene=Scene.Scene.Read(new StringReader(text));using var output=new StringWriter();scene.Write(output);var copy=Scene.Scene.Read(new StringReader(output.ToString()));using var second=new StringWriter();copy.Write(second);Assert.Equal(output.ToString(),second.ToString());
            if(scene.Dim==2){var w=copy.CreateWorld2D();var body=w.Bodies.GetEnumerator();Assert.True(body.MoveNext());Assert.Equal(2,body.Current.Mass);Assert.True(body.Current.NeverSleep);Assert.Equal(20,w.Options.MaxAngularSpeed);w.Step(1f/60);}
            else {var w=copy.CreateWorld3D();var body=w.Bodies.GetEnumerator();Assert.True(body.MoveNext());Assert.Equal(2,body.Current.Mass);Assert.True(body.Current.NeverSleep);w.Step(1f/60);}
        }
        Assert.Throws<Scene.SceneFormatException>(()=>Scene.Scene.Read(new StringReader(planar.Replace("controls 32", "controls 8",StringComparison.Ordinal))));
        Assert.Throws<Scene.SceneFormatException>(()=>Scene.Scene.Read(new StringReader(planar.Replace("mass 2", "mass -1",StringComparison.Ordinal))));
    }

    [Fact]
    public void PendingTargetsAndLiveAngularDefaultsReplayAndRestoreExactly()
    {
        var w=new P3.PhysicsWorld3D(new(){WorldId=887});var body=w.AddBox(new(1,1,1),type:BodyType.Kinematic);
        using var stream=new MemoryStream();
        using(w.StartRecording(stream)) { w.Options.MaxAngularSpeed=12;body.MoveKinematic(new(1,2,3),QuaternionF.Identity);var copy=w.Fork();copy.Restore(w.Snapshot());w.Step(1f/60);copy.Step(1f/60);Assert.Equal(w.Snapshot(),copy.Snapshot()); }
        stream.Position=0;var player=new ReplayPlayer(stream);var played=player.CreateWorld3D();Assert.True(player.ReadNextStep(played));Assert.Equal(w.Snapshot(),played.Snapshot());Assert.Equal(12,played.Options.MaxAngularSpeed);
    }

    [Fact]
    public void CenterOffsetChangesTippingAndDepenetrationIsCapped()
    {
        var a=new P2.PhysicsWorld2D(new(){Gravity=default});var body=a.AddBox(new(1,1));body.CenterOfMass=new(0.25f,0);body.SetInertia(2);body.ApplyImpulseAt(new(0,4),body.Position);Assert.Equal(-0.5f,body.AngularVelocity);
        var b=new P3.PhysicsWorld3D(new(){Gravity=default});var spatial=b.AddBox(new(1,1,1));spatial.CenterOfMass=new(0.25f,0,0);spatial.SetInertia(new(2,2,2),QuaternionF.Identity);spatial.ApplyImpulseAt(new(0,4,0),spatial.Position);Assert.Equal(-0.5f,spatial.AngularVelocity.Z);
        a.AddBox(new(10,1),at:new(0,-0.5),type:BodyType.Static);body.SetTransform(new(0,0.25),0);body.LinearVelocity=default;body.AngularVelocity=0;body.MaxDepenetrationSpeed=0.1f;body.Constraints=RigidbodyConstraints.FreezeRotationZ;double start=body.WorldCenterOfMass.Y;a.Step(1f/60);Assert.InRange(body.WorldCenterOfMass.Y-start,0,0.1/60+1e-5);
    }

    [Fact]
    public void HostileControlInputsFailBeforeMutatingWorld()
    {
        var w=new P3.PhysicsWorld3D();var body=w.AddBox(new(1,1,1));byte[] snapshot=w.Snapshot();
        foreach(float bad in new[]{-1f,float.NaN,float.PositiveInfinity}) { Assert.Throws<ArgumentOutOfRangeException>(()=>body.SetMass(bad));Assert.Throws<ArgumentOutOfRangeException>(()=>body.SetInertia(new(1,bad,1),QuaternionF.Identity));Assert.Throws<ArgumentOutOfRangeException>(()=>new BodyControlSettings(maxLinearSpeed:bad));Assert.Equal(snapshot,w.Snapshot()); }
        Assert.Throws<ArgumentException>(()=>body.MovePosition(new(1,2,3)));Assert.Equal(snapshot,w.Snapshot());
        Assert.Throws<ArgumentOutOfRangeException>(()=>w.CreateBody(new(){MassOverride=-1}));Assert.Equal(snapshot,w.Snapshot());
    }
    [Fact]
    public void AngularLockUsesRestrictedInertiaRatherThanAnInverseTensorMask()
    {
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});var body=world.AddBox(new(1,1,1));
        body.SetInertia(new(2,4,6),QuaternionF.CreateFromAxisAngle(new(0,0,1),0.7853982f));
        body.Constraints=RigidbodyConstraints.FreezeRotationY|RigidbodyConstraints.FreezeRotationZ;
        body.ApplyTorque(new(6,0,0));world.Step(0.5f);
        Assert.InRange(body.AngularVelocity.X,0.9999f,1.0001f);Assert.Equal(0,body.AngularVelocity.Y);Assert.Equal(0,body.AngularVelocity.Z);
    }
    [Fact]
    public void ControlSnapshotReaderAndTargetConflictsAreAtomic()
    {
        var world=new P3.PhysicsWorld3D();var body=world.AddBox(new(1,1,1));body.MaxLinearSpeed=1;
        byte[] snapshot=world.Snapshot();var field=SnapshotDesync.Inspect(world.Core).Single(f=>f.Section=="body controls"&&f.Name.EndsWith("MaxLinearSpeed",StringComparison.Ordinal));
        foreach(float bad in new[]{-1f,float.NaN,float.PositiveInfinity}) {byte[] damaged=(byte[])snapshot.Clone();BinaryPrimitives.WriteSingleLittleEndian(damaged.AsSpan(field.Offset),bad);Assert.Throws<SnapshotFormatException>(()=>world.Restore(damaged));Assert.Equal(snapshot,world.Snapshot());}
        var kinematic=world.CreateBody(new(){Type=BodyType.Kinematic,Controls=new(RigidbodyConstraints.FreezePositionX)});kinematic.MovePosition(new(1,0,0));snapshot=world.Snapshot();Assert.Throws<ArgumentException>(()=>world.Step(1f/60));Assert.Equal(snapshot,world.Snapshot());
        var copy=new P3.PhysicsWorld3D();copy.Restore(snapshot);Assert.Equal(snapshot,copy.Snapshot());
        Assert.Throws<ArgumentOutOfRangeException>(()=>world.CreateBody(new(){InertiaOverride=new Vector3F(1,2,3),InertiaRotationOverride=default(QuaternionF)}));Assert.Equal(snapshot,world.Snapshot());
    }
    [Fact]
    public void ExplicitVelocitySwitchesKinematicTargetsBackToVelocityControl()
    {
        var planar=new P2.PhysicsWorld2D();var a=planar.CreateBody(new(){Type=BodyType.Kinematic});a.MovePosition(new(1,0));planar.Step(0.125f);a.LinearVelocity=new(2,0);planar.Step(0.125f);Assert.Equal(1.25,a.Position.X);
        var spatial=new P3.PhysicsWorld3D();var b=spatial.CreateBody(new(){Type=BodyType.Kinematic});b.MovePosition(new(1,0,0));spatial.Step(0.125f);b.LinearVelocity=new(2,0,0);spatial.Step(0.125f);Assert.Equal(1.25,b.Position.X);
    }
    [Fact]
    public void CreationOverridesDoNotIntroduceSpuriousSleepWakeEvents()
    {
        var a=new P2.PhysicsWorld2D(new(){Events=EventMask.All});var body=a.CreateBody(new(){StartAsleep=true,MassOverride=2,Controls=new(RigidbodyConstraints.FreezeRotationZ)});int sleeps=0,wakes=0;body.OnSleep+=_=>sleeps++;body.OnWake+=_=>wakes++;a.Step(1f/60);Assert.Equal(1,sleeps);Assert.Equal(0,wakes);Assert.False(body.IsAwake);
        var b=new P3.PhysicsWorld3D(new(){Events=EventMask.All});var spatial=b.CreateBody(new(){StartAsleep=true,MassOverride=2,Controls=new(RigidbodyConstraints.FreezeRotationZ)});sleeps=0;wakes=0;spatial.OnSleep+=_=>sleeps++;spatial.OnWake+=_=>wakes++;b.Step(1f/60);Assert.Equal(1,sleeps);Assert.Equal(0,wakes);Assert.False(spatial.IsAwake);
    }
    [Fact]
    public void LegacyFixedRotationRemainsInfiniteInertiaWithAdditionalAxisLocks()
    {
        var world=new P3.PhysicsWorld3D(new(){Gravity=default});var body=world.CreateBody(new(){FixedRotation=true,Controls=new(RigidbodyConstraints.FreezeRotationY)});body.ApplyTorque(new(1,2,3));world.Step(0.125f);Assert.Equal(default,body.AngularVelocity);Assert.Equal(QuaternionF.Identity,body.Rotation);
    }
    [Fact]
    public void RollingResistanceStillActsOnTheRemainingFreeAngularAxis()
    {
        var world=new P3.PhysicsWorld3D();var floor=world.CreateBody(new(){Type=BodyType.Static,Position=new(0,-0.5,0)});world.CreateBox(floor,new(10,1,10),def:new(){Friction=0,RollingResistance=0.3f});
        var body=world.CreateBody(new(){Position=new(0,0.51,0),AngularVelocity=new(2,0,0),Controls=new(RigidbodyConstraints.FreezePositionX|RigidbodyConstraints.FreezePositionZ|RigidbodyConstraints.FreezeRotationY|RigidbodyConstraints.FreezeRotationZ)});world.CreateSphere(body,0.5f,def:new(){Friction=0,RollingResistance=0.3f});
        for(int i=0;i<60;i++)world.Step(1f/60);Assert.InRange(Math.Abs(body.AngularVelocity.X),0,0.5f);Assert.Equal(0,body.AngularVelocity.Y);Assert.Equal(0,body.AngularVelocity.Z);
    }
    [Fact]
    public void ConfiguredContactsAreBitIdenticalAtScalar128And256Bits()
    {
        var planar=new P2.PhysicsWorld2D[3];var spatial=new P3.PhysicsWorld3D[3];int[] widths=[1,4,8];
        for(int mode=0;mode<3;mode++)
        {
            var a=planar[mode]=new(new(){WorldId=896,SleepEnabled=false});var b=spatial[mode]=new(new(){WorldId=897,SleepEnabled=false});a.Engine.KernelWidth=widths[mode];b.Engine.KernelWidth=widths[mode];
            a.AddBox(new(100,1),at:new(20,-0.5),type:BodyType.Static);b.AddBox(new(100,1,20),at:new(20,-0.5,0),type:BodyType.Static);
            for(int j=0;j<11;j++)
            {
                var x=a.AddBox(new(1,1),at:new(j*3,0.5+j*0.01));x.Mass=2;x.SetInertia(3);x.Controls=new(RigidbodyConstraints.FreezePositionX|RigidbodyConstraints.FreezeRotationZ,2,3,0.2f);
                var y=b.AddBox(new(1,1,1),at:new(j*3,0.5+j*0.01,0));y.SetInertia(new(2,3,4),QuaternionF.Identity);y.Controls=new(RigidbodyConstraints.FreezePositionX|RigidbodyConstraints.FreezeRotationZ,2,3,0.2f);
            }
        }
        for(int step=0;step<200;step++)
        {
            for(int mode=0;mode<3;mode++){planar[mode].Step(1f/60);spatial[mode].Step(1f/60);}
            for(int mode=1;mode<3;mode++){Assert.Equal(planar[0].ComputeHash(),planar[mode].ComputeHash());Assert.Equal(spatial[0].ComputeHash(),spatial[mode].ComputeHash());}
        }
        for(int mode=1;mode<3;mode++){Assert.Equal(planar[0].Snapshot(),planar[mode].Snapshot());Assert.Equal(spatial[0].Snapshot(),spatial[mode].Snapshot());}
    }
}
