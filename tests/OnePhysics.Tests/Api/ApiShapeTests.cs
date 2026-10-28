using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Handle fields, one size and angle convention, typed joint views, character, ragdoll and vehicle handles, and event text.</summary>
public class ApiShapeTests
{
    private static P2.PhysicsWorld2D World2D() => new(new P2.WorldOptions2D { WorldId = 11 });

    private static P3.PhysicsWorld3D World3D() => new(new P3.WorldOptions3D { WorldId = 12 });

    // ------------------------------------------------------------------ handle fields

    [Fact]
    public void HandleFieldsOfEventsAndHitsCanBeSetThrough2D()
    {
        var w = World2D();
        w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var a = w.AddCircle(0.5f, at: new Vector2D(0, 1));
        var b = w.AddCircle(0.5f, at: new Vector2D(0.9, 3));
        int fired = 0;

        // The lines are the point: a property result is not a variable (CS1612), a readonly field is.
        a.OnCollisionEnter += c => { if (!c.Other.IsStatic) { c.Other.LinearVelocity = new Vector2F(0, 0); fired++; } };
        b.OnCollisionEnter += c => { if (!c.Other.IsStatic) { c.Other.LinearVelocity = new Vector2F(0, 0); fired++; } };
        for (int i = 0; i < 120; i++) { w.Step(1f / 60); }
        Assert.True(fired > 0);

        var target = w.AddCircle(0.5f, at: new Vector2D(30, 5));
        Assert.True(w.Raycast(new Vector2D(30, 9), new Vector2F(0, -1), out var hit));
        Assert.Equal(target, hit.Body);
        hit.Body.LinearVelocity = new Vector2F(0, 9);
        Assert.Equal(9f, hit.Body.LinearVelocity.Y);
        Assert.Equal(hit.Shape.Body, hit.Body);

        var list = new List<P2.Body> { a, b };
        list[0].SetLinearVelocity(new Vector2F(1, 2));
        list[1].SetAngularVelocity(3f);
        list[1].SetPosition(new Vector2D(5, 6));
        Assert.Equal(new Vector2F(1, 2), a.LinearVelocity);
        Assert.Equal(3f, b.AngularVelocity);
        Assert.Equal(new Vector2D(5, 6), b.Position);
    }

    [Fact]
    public void HandleFieldsOfEventsAndHitsCanBeSetThrough3D()
    {
        var w = World3D();
        w.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var a = w.AddSphere(0.5f, at: new Vector3D(0, 1, 0));
        int fired = 0;
        var b = w.AddSphere(0.5f, at: new Vector3D(0.2, 3, 0));
        a.OnCollisionEnter += c => { if (!c.Other.IsStatic) { c.Other.LinearVelocity = new Vector3F(0, 0, 0); fired++; } };
        b.OnCollisionEnter += c => { if (!c.Other.IsStatic) { c.Other.LinearVelocity = new Vector3F(0, 0, 0); fired++; } };
        for (int i = 0; i < 120; i++) { w.Step(1f / 60); }
        Assert.True(fired > 0);

        var target = w.AddSphere(0.5f, at: new Vector3D(30, 5, 0));
        Assert.True(w.Raycast(new Vector3D(30, 9, 0), new Vector3F(0, -1, 0), out var hit));
        Assert.Equal(target, hit.Body);
        hit.Body.LinearVelocity = new Vector3F(0, 9, 0);
        Assert.Equal(9f, hit.Body.LinearVelocity.Y);

        var list = new List<P3.Body> { a };
        list[0].SetLinearVelocity(new Vector3F(1, 2, 3));
        list[0].SetAngularVelocity(new Vector3F(4, 5, 6));
        list[0].SetPosition(new Vector3D(7, 8, 9));
        Assert.Equal(new Vector3F(1, 2, 3), a.LinearVelocity);
        Assert.Equal(new Vector3F(4, 5, 6), a.AngularVelocity);
        Assert.Equal(new Vector3D(7, 8, 9), a.Position);
        Assert.Throws<ArgumentOutOfRangeException>(() => list[0].SetLinearVelocity(new Vector3F(float.NaN, 0, 0)));
    }

    [Fact]
    public void HandleFieldsOfJointBreaksAndControllerHitsExist()
    {
        var w = World2D();
        var ground = w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var bob = w.AddCircle(0.3f, at: new Vector2D(1, 3));
        var hinge = w.AddHinge(ground, bob, new Vector2D(0, 3));
        hinge.BreakForce = 1f;
        P2.JointBreak got = default;
        hinge.OnJointBreak += b => got = b;
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); }
        Assert.Equal(bob, got.BodyB);
        got.BodyB.LinearVelocity = new Vector2F(1, 1);
        Assert.Contains("JointBreak", got.ToString());
        Assert.Contains("value", got.ToString());
    }

    // ------------------------------------------------------------------ sizes and angles

    [Fact]
    public void CreateBoxTakesTheFullSizeLikeAddBox()
    {
        var w2 = World2D();
        var viaAdd = w2.AddBox(new Vector2F(2, 2));
        var body = w2.CreateBody(BodyType.Dynamic, new Vector2D(10, 0));
        w2.CreateBox(body, new Vector2F(2, 2));
        Assert.Equal(viaAdd.Mass, body.Mass);
        Assert.Equal(4f, body.Mass);

        var w3 = World3D();
        var box3 = w3.AddBox(new Vector3F(2, 2, 2));
        var body3 = w3.CreateBody(BodyType.Dynamic, new Vector3D(10, 0, 0));
        w3.CreateBox(body3, new Vector3F(2, 2, 2));
        Assert.Equal(box3.Mass, body3.Mass);
        Assert.Equal(8f, body3.Mass);
        var spec = P3.ShapeSpec3D.Box(new Vector3F(2, 2, 2));
        var body4 = w3.CreateBody(BodyType.Dynamic, new Vector3D(20, 0, 0));
        w3.CreateShape(body4, spec);
        Assert.Equal(8f, body4.Mass);
    }

    [Fact]
    public void CapsuleParameterIsCalledHalfLengthInBothWorlds()
    {
        var c2 = World2D().AddCapsule(halfLength: 1f, radius: 0.5f);
        var c3 = World3D().AddCapsule(halfLength: 1f, radius: 0.5f);
        Assert.True(c2.Mass > 0f && c3.Mass > 0f);
        var w3 = World3D();
        Assert.False(w3.CapsuleCast(new Vector3D(0, 0, 0), halfLength: 1f, radius: 0.5f, QuaternionF.Identity, new Vector3F(1, 0, 0), out _));
        Assert.False(World2D().CapsuleCast(new Vector2D(0, 0), halfLength: 1f, radius: 0.5f, angleRadians: 0f, new Vector2F(1, 0), out _));
    }

    [Fact]
    public void AnglesAreRadiansAndNamedSo()
    {
        var w = World2D();
        var b = w.AddBox(new Vector2F(1, 1), angleRadians: MathF.PI / 4);
        Assert.InRange(b.Angle, 0.7853f, 0.7855f);
        b.SetTransform(new Vector2D(0, 0), angleRadians: 0.5f);
        Assert.InRange(b.Angle, 0.4999f, 0.5001f);
    }

    [Fact]
    public void SlopeLimitIsRadiansAndKeepsTheSavedDegrees()
    {
        var def = new P2.CharacterDef();
        Assert.Equal(MathF.PI / 4, def.SlopeLimit, 6);
        def.SlopeLimit = 30f * MathF.PI / 180f;
        Assert.InRange(def.SlopeLimit * 180f / MathF.PI, 29.999f, 30.001f);
        Assert.Equal(MathF.PI / 4, P3.CharacterDef.Humanoid.SlopeLimit, 6);
        Assert.Equal(40f * MathF.PI / 180f, P3.CharacterDef.Giant.SlopeLimit, 5);

        var w = World2D();
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new P2.CharacterDef { SlopeLimit = 45f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new P2.CharacterDef { SlopeLimit = -0.1f }));
        var c = w.AddCharacter(new Vector2D(0, 1));
        c.SlopeLimit = 0.5f;
        Assert.InRange(c.SlopeLimit, 0.4999f, 0.5001f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.SlopeLimit = 91f);

        var w3 = World3D();
        var c3 = w3.AddCharacter(new Vector3D(0, 1, 0));
        c3.SlopeLimit = 1f;
        Assert.InRange(c3.SlopeLimit, 0.9999f, 1.0001f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c3.SlopeLimit = 2f);
    }

    // ------------------------------------------------------------------ typed joint views

    [Fact]
    public void OneLinersReturnTypedViewsThatConvertToJoint2D()
    {
        var w = World2D();
        var ground = w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3));
        var b = w.AddBox(new Vector2F(1, 1), at: new Vector2D(2, 3));

        P2.HingeJoint hinge = w.AddHinge(a, b, new Vector2D(1, 3));
        hinge.EnableLimit = true;
        hinge.SetLimits(-0.5f, 0.5f);
        hinge.EnableMotor = true;
        hinge.MaxMotorTorque = 7f;
        hinge.MotorSpeed = 2f;
        Assert.Equal(-0.5f, hinge.LowerAngle);
        Assert.Equal(0.5f, hinge.UpperAngle);
        Assert.Equal(7f, hinge.MaxMotorTorque);
        Assert.Equal(P2.JointType2D.Revolute, hinge.Type);

        P2.Joint general = hinge;
        Assert.Equal(P2.JointType2D.Revolute, general.Type);
        Assert.Equal(hinge.RawHandle, general.RawHandle);
        Assert.Equal(7f, general.MaxMotorTorque);
        var back = (P2.HingeJoint)general;
        Assert.Equal(hinge, back);
        Assert.Throws<InvalidJointException>(() => _ = (P2.SliderJoint)general);

        P2.SliderJoint slider = w.AddSlider(ground, a, new Vector2D(0, 3), new Vector2F(1, 0));
        slider.SetLimits(-1f, 1f);
        slider.MaxMotorForce = 3f;
        Assert.Equal(-1f, slider.LowerTranslation);
        Assert.Equal(3f, slider.MaxMotorForce);

        P2.FixedJoint weld = w.AddFixed(a, b, new Vector2D(1, 3));
        weld.SetWeldSprings(5f, 0.5f, 5f, 0.5f);
        P2.DistanceJoint rod = w.AddDistance(a, b, new Vector2D(0, 3), new Vector2D(2, 3));
        P2.DistanceJoint spring = w.AddSpring(a, b, new Vector2D(0, 3), new Vector2D(2, 3));
        Assert.True(spring.EnableSpring);
        P2.WheelJoint wheel = w.AddWheel(a, b, new Vector2D(1, 3), new Vector2F(0, 1));
        wheel.MaxMotorTorque = 4f;
        P2.TargetJoint target = w.AddTarget(ground, a, new Vector2D(0, 3));
        target.Target = new Vector2D(1, 4);
        Assert.Equal(new Vector2D(1, 4), target.Target);
        P2.RelativeJoint rel = w.AddRelative(a, b);
        rel.MaxTorque = 5f;
        Assert.Equal(5f, rel.MaxTorque);

        P2.Joint[] all = [hinge, slider, weld, rod, spring, wheel, target, rel];
        Assert.Equal(8, all.Length);
        foreach (var j in all) { Assert.True(j.IsValid); }
        hinge.Destroy();
        Assert.False(general.IsValid);
        Assert.Throws<StaleHandleException>(() => hinge.EnableMotor = false);
    }

    [Fact]
    public void WrongKindMembersOnTheGeneralJointThrowATypedError2D()
    {
        var w = World2D();
        var a = w.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3));
        var b = w.AddBox(new Vector2F(1, 1), at: new Vector2D(2, 3));
        P2.Joint hinge = w.AddHinge(a, b, new Vector2D(1, 3));
        P2.Joint slider = w.AddSlider(a, b, new Vector2D(1, 3), new Vector2F(1, 0));
        P2.Joint rod = w.AddDistance(a, b, new Vector2D(0, 3), new Vector2D(2, 3));

        Assert.Throws<InvalidJointException>(() => hinge.Length = 3f);
        Assert.Throws<InvalidJointException>(() => hinge.Target = new Vector2D(0, 0));
        Assert.Throws<InvalidJointException>(() => hinge.SetOffset(default, 0f));
        Assert.Throws<InvalidJointException>(() => _ = hinge.LowerTranslation);
        Assert.Throws<InvalidJointException>(() => _ = hinge.MinLength);
        Assert.Throws<InvalidJointException>(() => _ = slider.LowerAngle);
        Assert.Throws<InvalidJointException>(() => _ = rod.UpperAngle);

        // A rotational motor never takes a force, a linear one never a torque.
        var e1 = Assert.Throws<InvalidJointException>(() => hinge.MaxMotorForce = 5f);
        Assert.Contains("MaxMotorTorque", e1.Message, StringComparison.Ordinal);
        Assert.Throws<InvalidJointException>(() => _ = hinge.MaxMotorForce);
        var e2 = Assert.Throws<InvalidJointException>(() => slider.MaxMotorTorque = 5f);
        Assert.Contains("MaxMotorForce", e2.Message, StringComparison.Ordinal);
        Assert.Throws<InvalidJointException>(() => rod.MaxMotorTorque = 5f);
        hinge.MaxMotorTorque = 5f;
        slider.MaxMotorForce = 5f;
        rod.MaxMotorForce = 5f;
        Assert.Equal(5f, hinge.MaxMotorTorque);
        Assert.Equal(5f, slider.MaxMotorForce);
        Assert.Equal(5f, rod.MaxMotorForce);
    }

    [Fact]
    public void OneLinersReturnTypedViewsThatConvertToJoint3D()
    {
        var w = World3D();
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(2, 3, 0));
        var c = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(4, 3, 0));

        P3.HingeJoint hinge = w.AddHinge(a, b, new Vector3D(1, 3, 0), new Vector3F(0, 0, 1));
        hinge.SetLimits(-0.5f, 0.5f);
        hinge.EnableLimit = true;
        hinge.MaxMotorTorque = 9f;
        Assert.Equal(-0.5f, hinge.LowerAngle);
        Assert.Equal(P3.JointType3D.Hinge, hinge.Type);

        P3.SliderJoint slider = w.AddSlider(b, c, new Vector3D(3, 3, 0), new Vector3F(1, 0, 0));
        slider.MaxMotorForce = 4f;
        P3.FixedJoint fixedJoint = w.AddFixed(a, c, new Vector3D(2, 3, 0));
        P3.PointJoint point = w.AddPoint(a, b, new Vector3D(1, 3, 0));
        P3.DistanceJoint spring = w.AddSpring(a, b, new Vector3D(0, 3, 0), new Vector3D(2, 3, 0));
        Assert.True(spring.EnableSpring);
        P3.DistanceJoint rod = w.AddDistance(a, b, new Vector3D(0, 3, 0), new Vector3D(2, 3, 0));
        rod.Length = 2f;
        P3.ConeJoint cone = w.AddCone(a, b, new Vector3D(1, 3, 0), new Vector3F(1, 0, 0), 0.5f);
        cone.HalfConeAngle = 0.6f;
        P3.SwingTwistJoint st = w.AddCharacterJoint(a, b, new Vector3D(1, 3, 0), new Vector3F(1, 0, 0));
        st.SetPoseDrive(QuaternionF.Identity, 1f, 1f, 1f);
        Assert.True(st.IsPoseDriven);
        P3.SixDofJoint six = w.AddConfigurable(a, b, new Vector3D(1, 3, 0));
        six.SetAxisMotor(P3.JointAxes.AngularX, true, 1f, 2f);

        P3.Joint[] all = [hinge, slider, fixedJoint, point, spring, rod, cone, st, six];
        foreach (var j in all) { Assert.True(j.IsValid); }
        P3.Joint general = hinge;
        Assert.Equal(9f, general.MaxMotorTorque);
        Assert.Equal(hinge, (P3.HingeJoint)general);
        Assert.Throws<InvalidJointException>(() => _ = (P3.SliderJoint)general);
    }

    [Fact]
    public void WrongKindMembersOnTheGeneralJointThrowATypedError3D()
    {
        var w = World3D();
        var a = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(2, 3, 0));
        P3.Joint hinge = w.AddHinge(a, b, new Vector3D(1, 3, 0), new Vector3F(0, 0, 1));
        P3.Joint slider = w.AddSlider(a, b, new Vector3D(1, 3, 0), new Vector3F(1, 0, 0));

        Assert.Throws<InvalidJointException>(() => hinge.Length = 3f);
        Assert.Throws<InvalidJointException>(() => _ = hinge.LowerTranslation);
        Assert.Throws<InvalidJointException>(() => _ = slider.LowerAngle);
        var e1 = Assert.Throws<InvalidJointException>(() => hinge.MaxMotorForce = 5f);
        Assert.Contains("MaxMotorTorque", e1.Message, StringComparison.Ordinal);
        var e2 = Assert.Throws<InvalidJointException>(() => slider.MaxMotorTorque = 5f);
        Assert.Contains("MaxMotorForce", e2.Message, StringComparison.Ordinal);
        Assert.Throws<InvalidJointException>(() => hinge.SetAxisMode(P3.JointAxes.LinearX, P3.JointAxisMode.Free));
        Assert.Throws<InvalidJointException>(() => hinge.SetPoseDrive(QuaternionF.Identity, 1f, 1f, 1f));
    }

    [Fact]
    public void DefinitionAndViewMembersShareNames()
    {
        // Every public member of a typed view that mirrors a definition field has the definition's name and type.
        void Check(Type view, Type def, params string[] names)
        {
            foreach (string n in names)
            {
                var dp = def.GetProperty(n);
                var vp = view.GetProperty(n);
                Assert.NotNull(dp);
                Assert.NotNull(vp);
                Assert.Equal(dp!.PropertyType, vp!.PropertyType);
            }
        }

        Check(typeof(P2.HingeJoint), typeof(P2.RevoluteJointDef), "EnableLimit", "EnableMotor", "MotorSpeed", "MaxMotorTorque", "EnableSpring", "SpringHertz", "SpringDampingRatio", "BreakForce", "BreakTorque");
        Check(typeof(P2.SliderJoint), typeof(P2.PrismaticJointDef), "EnableLimit", "EnableMotor", "MotorSpeed", "MaxMotorForce", "EnableSpring", "SpringHertz", "SpringDampingRatio");
        Check(typeof(P2.WheelJoint), typeof(P2.WheelJointDef), "EnableMotor", "MaxMotorTorque", "EnableSpring");
        Check(typeof(P2.DistanceJoint), typeof(P2.DistanceJointDef), "Length", "EnableLimit", "MinLength", "MaxLength", "EnableMotor", "MaxMotorForce", "EnableSpring");
        Check(typeof(P2.TargetJoint), typeof(P2.MouseJointDef), "Target", "MaxForce");
        Check(typeof(P2.RelativeJoint), typeof(P2.MotorJointDef), "MaxForce", "MaxTorque");
        Check(typeof(P3.HingeJoint), typeof(P3.HingeJointDef), "EnableLimit", "EnableMotor", "MotorSpeed", "MaxMotorTorque", "EnableSpring", "SpringHertz");
        Check(typeof(P3.SliderJoint), typeof(P3.SliderJointDef), "EnableLimit", "MaxMotorForce", "EnableSpring");
        Check(typeof(P3.DistanceJoint), typeof(P3.DistanceJointDef), "Length", "MinLength", "MaxLength", "EnableLimit", "EnableSpring");
        Check(typeof(P3.ConeJoint), typeof(P3.ConeJointDef), "HalfConeAngle");
        Check(typeof(P3.GearJoint), typeof(P3.GearJointDef), "Ratio");
        Check(typeof(P3.PathJoint), typeof(P3.PathJointDef), "EnableMotor", "MotorSpeed", "MaxMotorForce");

        // The old drifting names are gone.
        foreach (var t in new[] { typeof(P2.Joint), typeof(P3.Joint), typeof(P2.HingeJoint), typeof(P3.HingeJoint) })
        {
            Assert.Null(t.GetProperty("LimitEnabled"));
            Assert.Null(t.GetProperty("MotorEnabled"));
            Assert.Null(t.GetProperty("SpringEnabled"));
            Assert.Null(t.GetProperty("LowerLimit"));
        }
    }

    // ------------------------------------------------------------------ character, ragdoll and vehicle handles

    [Fact]
    public void CharacterPresetsLiveOnCharacterDef()
    {
        Assert.Equal(1.8f, P2.CharacterDef.Humanoid.Height);
        Assert.Equal(1.2f, P2.CharacterDef.Child.Height);
        Assert.Equal(3f, P3.CharacterDef.Giant.Height);
        Assert.Equal(P3.CharacterKind.Rigid, P3.CharacterDef.RigidHumanoid.Kind);
        Assert.Null(typeof(P2.Character).GetProperty("Humanoid"));
        Assert.Null(typeof(P3.Character).GetProperty("Giant"));
    }

    [Fact]
    public void CharacterHandlesRoundTripAndFailClearly()
    {
        var w = World2D();
        var ch = w.AddCharacter(new Vector2D(0, 1));
        var h = ch.RawHandle;
        Assert.False(h.IsNull);
        Assert.Equal(w.Id, h.World);
        Assert.True(w.IsCharacterAlive(h));
        Assert.Equal(ch, w.GetCharacter(h));
        Assert.Contains("Character", h.ToString(), StringComparison.Ordinal);
        Assert.False(w.IsCharacterAlive(default));
        Assert.Throws<InvalidHandleException>(() => w.GetCharacter(default));
        var other = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 99 });
        Assert.False(other.IsCharacterAlive(h));
        Assert.Throws<ForeignHandleException>(() => other.GetCharacter(h));
        ch.Body.Destroy();
        Assert.False(w.IsCharacterAlive(h));
        Assert.Throws<StaleHandleException>(() => w.GetCharacter(h));

        var w3 = World3D();
        var c3 = w3.AddCharacter(new Vector3D(0, 1, 0));
        Assert.Equal(c3, w3.GetCharacter(c3.RawHandle));
        Assert.True(w3.IsCharacterAlive(c3.RawHandle));
    }

    [Fact]
    public void RagdollAndVehicleHandlesRoundTripAndFailClearly()
    {
        var w = World3D();
        var r = w.AddRagdoll(P3.RagdollDef.Humanoid, new Vector3D(0, 2, 0));
        var rh = r.RawHandle;
        Assert.True(w.IsRagdollAlive(rh));
        Assert.Equal(r, w.GetRagdoll(rh));
        Assert.False(w.IsRagdollAlive(default));
        Assert.Throws<InvalidHandleException>(() => w.GetRagdoll(default));
        Assert.Throws<ForeignHandleException>(() => new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 77 }).GetRagdoll(rh));

        var chassis = w.AddBox(new Vector3F(2, 0.5f, 4), at: new Vector3D(20, 2, 0));
        var v = w.AddVehicle(chassis, P3.VehicleDef.Car);
        var vh = v.RawHandle;
        Assert.True(w.IsVehicleAlive(vh));
        Assert.Equal(v, w.GetVehicle(vh));
        Assert.False(w.IsVehicleAlive(default));
        Assert.Throws<ForeignHandleException>(() => new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 77 }).GetVehicle(vh));

        r.Remove();
        Assert.False(w.IsRagdollAlive(rh));
        Assert.Throws<StaleHandleException>(() => w.GetRagdoll(rh));

        // The bare-int overloads are gone.
        Assert.Null(typeof(P3.PhysicsWorld3D).GetMethod("GetRagdoll", new[] { typeof(int), typeof(int) }));
        Assert.Null(typeof(P3.PhysicsWorld3D).GetMethod("GetVehicle", new[] { typeof(int), typeof(int) }));
        Assert.Null(typeof(P3.PhysicsWorld3D).GetMethod("GetCharacter", new[] { typeof(int), typeof(int) }));
        Assert.Null(typeof(P2.PhysicsWorld2D).GetMethod("IsCharacterAlive", new[] { typeof(int), typeof(int) }));
    }

    [Fact]
    public void EventStructsHaveReadableText()
    {
        var e = new PhysicsEvent(EventKind.JointBreak, new BodyHandle(1, 2, 1), new BodyHandle(1, 3, 1), default, default, 0, 0, new JointHandle(1, 4, 1), 12.5f);
        string s = e.ToString();
        Assert.Contains("JointBreak", s, StringComparison.Ordinal);
        Assert.Contains("12.5", s, StringComparison.Ordinal);
        Assert.DoesNotContain("PhysicsEvent", s, StringComparison.Ordinal);

        var w = World2D();
        w.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var ball = w.AddCircle(0.5f, at: new Vector2D(0, 1));
        string collisionText = "";
        ball.OnCollisionEnter += c => collisionText = c.ToString();
        for (int i = 0; i < 60; i++) { w.Step(1f / 60); }
        Assert.StartsWith("Collision(", collisionText, StringComparison.Ordinal);
        Assert.True(w.Raycast(new Vector2D(0, 9), new Vector2F(0, -1), out var hit));
        Assert.StartsWith("QueryHit(", hit.ToString(), StringComparison.Ordinal);
        Assert.Equal("QueryHit(none)", default(P2.QueryHit2D).ToString());
    }

    // ------------------------------------------------------------------ restore reports options

    [Fact]
    public void RestoreReportsWhichSavedOptionsItReplaced()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Substeps = 4, WorldId = 5 });
        w.AddCircle(0.5f, at: new Vector2D(0, 3));
        byte[] snap = w.Snapshot();
        Assert.Equal(OptionChanges.None, w.Restore(snap));

        var other = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Substeps = 8, WorldId = 5, TimeToSleep = 1f });
        var changed = other.Restore(snap);
        Assert.True(changed.HasFlag(OptionChanges.Substeps));
        Assert.True(changed.HasFlag(OptionChanges.TimeToSleep));
        Assert.False(changed.HasFlag(OptionChanges.Gravity));
        Assert.Equal(4, other.Options.Substeps);

        var w3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 6 });
        byte[] snap3 = w3.Snapshot();
        var o3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 6, Gravity = new Vector3F(0, -1, 0) });
        Assert.Equal(OptionChanges.Gravity, o3.Restore(snap3));
        Assert.Equal(OptionChanges.None, o3.Restore((ReadOnlySpan<byte>)snap3));
    }
}
