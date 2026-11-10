using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Members that exist in one dimension and now in the other too: each is checked against its own analytic answer or against the counterpart.</summary>
public class ParityGapTests
{
    private const float Dt = 1f / 60f;

    // ------------------------------------------------------------------ 2D fluid handle

    [Fact]
    public void FluidHandleHasValueEqualityAndTextIn2D()
    {
        var world = new P2.PhysicsWorld2D();
        var a = world.AddWater(0.0);
        var b = world.AddWater(1.0);
        var copy = a;
        Assert.True(a == copy);
        Assert.True(a != b);
        Assert.Equal(a.GetHashCode(), copy.GetHashCode());
        Assert.False(a.Equals(b));
        Assert.False(a.Equals((object)"not a fluid"));
        Assert.True(default(P2.Fluid) == default);
        Assert.Equal("Fluid(default)", default(P2.Fluid).ToString());
        Assert.StartsWith("Fluid(#", a.ToString(), StringComparison.Ordinal);
        var set = new HashSet<P2.Fluid> { a, b, copy };
        Assert.Equal(2, set.Count);
    }

    // ------------------------------------------------------------------ 2D shape local pose

    [Fact]
    public void LocalPositionAndAngleMoveA2DShapeInTheBodyFrame()
    {
        var world = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default });
        var body = world.AddCircle(0.5f, at: new Vector2D(10, 0), type: BodyType.Static);
        // A second shape: a 2 x 0.2 bar, offset 3 m up and turned a quarter turn, so it stands 2 m tall at (13, 3).
        world.CreateBox(body, new Vector2F(2f, 0.2f), def: new P2.ShapeDef { LocalPosition = new Vector2F(3f, 0f), LocalAngle = (float)(Math.PI / 2) });
        Assert.True(world.Raycast(new Vector2D(13, 5), new Vector2F(0, -1), out var hit));
        Assert.Equal(1.0, hit.Point.Y, 3);   // top of the bar: one metre above the body's origin, half the bar length
        Assert.Equal(13.0, hit.Point.X, 3);
        // The bar does not lie flat where an unrotated one would: a ray along x at y = 0.5 passes through x = 13 only if it is upright.
        Assert.True(world.Raycast(new Vector2D(15, 0.5), new Vector2F(-1, 0), out var side));
        Assert.Equal(13.1, side.Point.X, 3);
    }

    [Fact]
    public void LocalPoseGivesTheSameShapeAsPlacingItDirectly()
    {
        var world = new P2.PhysicsWorld2D();
        var a = world.AddCircle(0.5f, at: default, type: BodyType.Static);
        var posed = world.CreatePolygon(a, new[] { new Vector2F(-1, -0.5f), new Vector2F(1, -0.5f), new Vector2F(1, 0.5f), new Vector2F(-1, 0.5f) }, 0f,
            new P2.ShapeDef { LocalPosition = new Vector2F(5, 2), LocalAngle = 0.5f });
        var b = world.AddCircle(0.5f, at: default, type: BodyType.Static);
        var (s, c) = Math64.SinCos(0.5f);
        var direct = world.CreateBox(b, new Vector2F(2f, 1f), center: new Vector2F(5, 2), angleRadians: 0.5f);
        Assert.NotEqual(default, posed);
        for (int i = 0; i < 20; i++)
        {
            double y = -4 + 0.4 * i;
            var origin = new Vector2D(20, y);
            bool h1 = posed.Raycast(origin, new Vector2F(-1, 0), out var r1);
            bool h2 = direct.Raycast(origin, new Vector2F(-1, 0), out var r2);
            Assert.Equal(h2, h1);
            if (h1) { Assert.Equal(r2.Point.X, r1.Point.X, 4); }
        }
        _ = s; _ = c;
    }

    [Fact]
    public void LocalPoseValidationAndTileGridRefusal()
    {
        var world = new P2.PhysicsWorld2D();
        var body = world.AddCircle(0.5f, at: default, type: BodyType.Static);
        Assert.Throws<ArgumentOutOfRangeException>(() => world.CreateCircle(body, 0.3f, default, new P2.ShapeDef { LocalAngle = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => world.CreateCircle(body, 0.3f, default, new P2.ShapeDef { LocalPosition = new Vector2F(float.PositiveInfinity, 0) }));
        var floor = world.AddBox(new Vector2F(1, 1), at: new Vector2D(50, 0), type: BodyType.Static);
        var grid = new TileGrid2D(2, 2, [TileKind2D.Solid, TileKind2D.Empty, TileKind2D.Empty, TileKind2D.Solid]);
        Assert.Throws<ArgumentException>(() => { _ = world.CreateTileGrid(floor, grid, new P2.ShapeDef { LocalPosition = new Vector2F(1, 0) }); });
    }

    [Fact]
    public void ALocalPoseSurvivesSnapshotRestore()
    {
        var world = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 7 });
        var body = world.AddCircle(0.5f, at: default, type: BodyType.Static);
        world.CreateCircle(body, 0.25f, default, new P2.ShapeDef { LocalPosition = new Vector2F(4, 0) });
        ulong hash = world.ComputeHash();
        var bytes = world.Snapshot();
        var other = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 7 });
        other.Restore(bytes);
        Assert.Equal(hash, other.ComputeHash());
        Assert.True(other.Raycast(new Vector2D(8, 0), new Vector2F(-1, 0), out var hit));
        Assert.Equal(4.25, hit.Point.X, 3);
    }

    // ------------------------------------------------------------------ 2D signed distance field on a body

    [Fact]
    public void CreateSdfAttachesTheSameShapesAsAddSdf2D()
    {
        var field = SdfGrid2D.FromFunction(new Vec2(-3, -3), new Vec2(3, 3), 0.25f, p => float.Sqrt(p.X * p.X + p.Y * p.Y) - 2f);
        var w1 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 5 });
        var viaAdd = w1.AddSdf(field);
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 5 });
        var body = w2.CreateBody(BodyType.Static, default);
        var first = w2.CreateSdf(body, field, new P2.ShapeDef { Density = 0f });
        Assert.NotEqual(default, first);
        Assert.Equal(viaAdd.GetShapes().Length, body.GetShapes().Length);
        Assert.True(w1.ComputeHash() == w2.ComputeHash());
        // Dynamic: a convex cover with mass from the disc's area.
        var w3 = new P2.PhysicsWorld2D();
        var moving = w3.CreateBody(BodyType.Dynamic, default);
        Assert.NotEqual(default, w3.CreateSdf(moving, field, new P2.ShapeDef { Density = 2f }));
        Assert.InRange(moving.Mass, 0.9 * 2 * Math.PI * 4, 1.1 * 2 * Math.PI * 4);
    }

    [Fact]
    public void CreateSdfRefusesNullFieldAndForeignBody()
    {
        var w1 = new P2.PhysicsWorld2D();
        var w2 = new P2.PhysicsWorld2D();
        var foreign = w2.CreateBody(BodyType.Static, default);
        var field = SdfGrid2D.FromFunction(new Vec2(-1, -1), new Vec2(1, 1), 0.25f, p => p.X);
        Assert.Throws<ArgumentNullException>(() => w1.CreateSdf(w1.CreateBody(BodyType.Static, default), null!));
        Assert.Throws<ArgumentException>(() => w1.CreateSdf(foreign, field));
    }

    // ------------------------------------------------------------------ 2D speculative distance

    [Fact]
    public void SpeculativeDistanceDefaultKeepsSnapshotBytesAndAChangeRoundTrips()
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 11 });
        var b = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 11, SpeculativeDistance = 0.04f });
        Assert.Equal(0.02f, a.Options.SpeculativeDistance);
        Assert.True(b.GetSnapshotSize() > a.GetSnapshotSize());
        Assert.NotEqual(a.ComputeHash(), b.ComputeHash());
        // Restoring a snapshot that carries the value brings it back; one without it puts the default back.
        a.Restore(b.Snapshot());
        Assert.Equal(0.04f, a.Options.SpeculativeDistance);
        Assert.Equal(b.ComputeHash(), a.ComputeHash());
        var plain = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 11 });
        a.Restore(plain.Snapshot());
        Assert.Equal(0.02f, a.Options.SpeculativeDistance);
        Assert.Equal(plain.ComputeHash(), a.ComputeHash());
        var fork = b.Fork();
        Assert.Equal(0.04f, fork.Options.SpeculativeDistance);
        Assert.Equal(b.ComputeHash(), fork.ComputeHash());
    }

    [Fact]
    public void SpeculativeDistanceValidates()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new P2.PhysicsWorld2D(new P2.WorldOptions2D { SpeculativeDistance = -0.01f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new P2.PhysicsWorld2D(new P2.WorldOptions2D { SpeculativeDistance = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new P2.PhysicsWorld2D(new P2.WorldOptions2D { SpeculativeDistance = 2f }));
    }

    [Fact]
    public void ALargerSpeculativeDistanceMakesContactSooner()
    {
        int StepsUntilContact(float spec)
        {
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default, SpeculativeDistance = spec, BroadphaseMargin = 0.2 });
            w.AddBox(new Vector2F(10, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
            var ball = w.AddCircle(0.5f, at: new Vector2D(0, 0.62));   // 0.12 m above the floor
            int contacts = 0;
            for (int i = 0; i < 1; i++) { w.Step(Dt); foreach (var c in ball.Contacts) { if (c.PointCount > 0) { contacts++; } } }
            return contacts;
        }

        Assert.Equal(0, StepsUntilContact(0.02f));
        Assert.True(StepsUntilContact(0.15f) > 0);
    }

    // ------------------------------------------------------------------ query hit sub-shape ids

    [Fact]
    public void QueryHitsCarryASubShapeIdIn2D()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new Vector2F(2, 2), at: default, type: BodyType.Static);
        Assert.True(w.Raycast(new Vector2D(-5, 0), new Vector2F(1, 0), out var ray));
        Assert.Equal(0, ray.SubShapeId);
        Assert.True(w.OverlapCircle(new Vector2D(0, 0), 0.5f, out var overlap));
        Assert.Equal(-1, overlap.SubShapeId);
        Assert.True(w.ClosestPoint(new Vector2D(5, 0), out var closest));
        Assert.Equal(0, closest.SubShapeId);
        Assert.Equal(0, default(P2.QueryHit2D).SubShapeId);
    }

    // ------------------------------------------------------------------ 3D tangent impulse

    [Fact]
    public void ASlidingBoxReportsATangentImpulseIn3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Events = EventMask.All, SleepEnabled = false });
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var box = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        box.LinearVelocity = new Vector3F(4, 0, 0);
        float tangent = 0f, normal = 0f; bool resting = false;
        box.OnCollisionStay += c => { tangent = c.TangentImpulse; normal = c.NormalImpulse; resting = c.PointCount > 0; };
        for (int i = 0; i < 12; i++) { w.Step(Dt); }
        Assert.True(resting);
        Assert.True(normal > 0f);
        // Coulomb friction at the default coefficient 0.6: the friction impulse is at most mu times the normal impulse, and a sliding box is at the limit.
        Assert.InRange(tangent, 0.3f * normal, 0.62f * normal);
        // A box at rest on the ground carries (almost) none.
        var w2 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Events = EventMask.All, SleepEnabled = false });
        w2.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var still = w2.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 0.5, 0));
        float stillTangent = 1f;
        still.OnCollisionStay += c => stillTangent = c.TangentImpulse;
        for (int i = 0; i < 30; i++) { w2.Step(Dt); }
        Assert.True(stillTangent < 0.02f, "tangent impulse of a resting box: " + stillTangent);
    }

    // ------------------------------------------------------------------ 3D soft body overlap

    [Fact]
    public void OverlapSoftBodiesFindsBodiesByParticleSpheresIn3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default });
        var mesh = P3.SoftBodyMesh.Sphere(0.5f, 4);
        var near = w.AddSoftBody(new P3.SoftBodyDef { Mesh = mesh, Position = new Vector3D(0, 0, 0), Mass = 1f });
        var far = w.AddSoftBody(new P3.SoftBodyDef { Mesh = mesh, Position = new Vector3D(20, 0, 0), Mass = 1f });
        var found = new P3.SoftBody[4];
        Assert.Equal(1, w.OverlapSoftBodies(ConvexShape3D.Sphere(0.3f), new Vector3D(0.6, 0, 0), default, found));
        Assert.Equal(near, found[0]);
        Assert.Equal(1, w.OverlapSoftBodies(ConvexShape3D.Sphere(0.3f), new Vector3D(20.4, 0, 0), default, found));
        Assert.Equal(far, found[0]);
        Assert.Equal(0, w.OverlapSoftBodies(ConvexShape3D.Sphere(0.3f), new Vector3D(10, 0, 0), default, found));
        // A large box spanning both finds both in ascending slot order; a short buffer keeps the lowest slots.
        var box = ConvexShape3D.Box(new Vector3F(60, 4, 4));
        Assert.Equal(2, w.OverlapSoftBodies(box, new Vector3D(10, 0, 0), default, found));
        Assert.Equal(near, found[0]);
        Assert.Equal(far, found[1]);
        Assert.Equal(1, w.OverlapSoftBodies(box, new Vector3D(10, 0, 0), default, found.AsSpan(0, 1)));
        Assert.Equal(near, found[0]);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.OverlapSoftBodies(box, new Vector3D(double.NaN, 0, 0), default, found));
    }

    // ------------------------------------------------------------------ typed 2D machine joints

    [Fact]
    public void MachineJointsAreTypedViewsIn2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { Gravity = default });
        var a = w.AddCircle(0.5f, at: new Vector2D(0, 0));
        var b = w.AddCircle(0.5f, at: new Vector2D(3, 0));
        P2.GearJoint gear = w.AddGear(a, b, 2f);
        Assert.Equal(2f, gear.Ratio);
        gear.Ratio = 3f;
        Assert.Equal(3f, ((P2.Joint)gear).Ratio);
        Assert.Equal(P2.JointType2D.Gear, gear.Type);
        P2.Joint general = gear;
        Assert.Equal(gear, (P2.GearJoint)general);
        Assert.True(gear == (P2.GearJoint)general);
        Assert.Throws<InvalidJointException>(() => (P2.RackAndPinionJoint)general);
        gear.Destroy();
        Assert.False(gear.IsValid);

        var rack = w.AddRackAndPinion(a, b, new Vector2F(1, 0), 0.5f);
        Assert.Equal(0.5f, rack.Ratio);
        var pulley = w.AddPulley(a, b, new Vector2D(0, 0), new Vector2D(3, 0), new Vector2D(0, 5), new Vector2D(3, 5));
        Assert.Equal(1f, pulley.Ratio);

        var c = w.AddBox(new Vector2F(1, 1), at: new Vector2D(10, 0), type: BodyType.Static);
        var d = w.AddBox(new Vector2F(1, 1), at: new Vector2D(10, 0));
        var path = w.AddPath(c, d, new[] { new Vector2D(10, 0), new Vector2D(14, 0), new Vector2D(18, 0) });
        path.EnableMotor = true; path.MotorSpeed = 1f; path.MaxMotorForce = 100f;
        Assert.True(path.EnableMotor);
        Assert.Equal(1f, path.MotorSpeed);
        Assert.Equal(100f, path.MaxMotorForce);
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        Assert.InRange(d.Position.X, 10.5, 11.5);   // one second at 1 m/s along the path
    }

    [Fact]
    public void Every2DJointDefinitionSharesTheBodyInterface()
    {
        var w = new P2.PhysicsWorld2D();
        var a = w.AddCircle(0.5f, at: default);
        var b = w.AddCircle(0.5f, at: new Vector2D(2, 0));
        Assert.Equal(a, SetBodies(new P2.GearJointDef(), a, b).BodyA);
        Assert.Equal(b, SetBodies(new P2.PathJointDef(), a, b).BodyB);
        Assert.IsAssignableFrom<P2.IJointDef2D>(new P2.RevoluteJointDef());
        Assert.IsAssignableFrom<P2.IJointDef2D>(new P2.WeldJointDef());
        Assert.IsAssignableFrom<P2.IJointDef2D>(new P2.PathJointDef());
    }

    private static T SetBodies<T>(T def, P2.Body a, P2.Body b) where T : struct, P2.IJointDef2D { def.BodyA = a; def.BodyB = b; return def; }

    // ------------------------------------------------------------------ 3D distance joint motor

    [Fact]
    public void ADistanceMotorChangesTheLengthAtItsSpeedIn3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false });
        var anchor = w.AddBox(new Vector3F(1, 1, 1), at: default, type: BodyType.Static);
        var weight = w.AddSphere(0.25f, at: new Vector3D(2, 0, 0));
        var joint = w.CreateJoint(new P3.DistanceJointDef
        {
            BodyA = anchor, BodyB = weight, Length = 2f, MinLength = 0.5f, MaxLength = 10f, EnableSpring = true, SpringHertz = 0.5f, SpringDampingRatio = 1f,
            EnableMotor = true, MotorSpeed = 0.5f, MaxMotorForce = 1e5f,
        });
        Assert.True(joint.EnableMotor);
        Assert.Equal(0.5f, joint.MotorSpeed);
        Assert.Equal(1e5f, joint.MaxMotorForce);
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        // Two seconds at half a metre a second: the weight is about a metre further out (the soft spring pulls it back a little, so allow a margin).
        Assert.InRange(weight.Position.X, 2.85, 3.1);
        Assert.InRange(Math.Abs(weight.Position.Y) + Math.Abs(weight.Position.Z), 0.0, 1e-3);
        joint.MotorSpeed = -0.5f;
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.InRange(weight.Position.X, 1.9, 2.2);
        joint.EnableMotor = false;
        Assert.False(joint.EnableMotor);
        Assert.Throws<ArgumentOutOfRangeException>(() => joint.MaxMotorForce = -1f);
    }

    [Fact]
    public void ADistanceMotorIsInTheSnapshotAndDeterministicIn3D()
    {
        P3.PhysicsWorld3D Make()
        {
            var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false, WorldId = 21 });
            var anchor = w.AddBox(new Vector3F(1, 1, 1), at: default, type: BodyType.Static);
            var weight = w.AddSphere(0.25f, at: new Vector3D(2, 0, 0));
            w.CreateJoint(new P3.DistanceJointDef { BodyA = anchor, BodyB = weight, Length = 2f, MinLength = 0.5f, MaxLength = 10f, EnableSpring = true, SpringHertz = 1f, EnableMotor = true, MotorSpeed = 1f, MaxMotorForce = 50f });
            return w;
        }

        var one = Make();
        for (int i = 0; i < 30; i++) { one.Step(Dt); }
        var bytes = one.Snapshot();
        var two = Make();
        two.Restore(bytes);
        for (int i = 0; i < 60; i++) { one.Step(Dt); two.Step(Dt); Assert.Equal(one.ComputeHash(), two.ComputeHash()); }
    }

    // ------------------------------------------------------------------ 3D fixed joint springs

    private static double OscillationHertz(Func<int, double> sample, int steps)
    {
        int crossings = 0; double first = 0, last = 0; double prev = sample(0);
        for (int i = 1; i <= steps; i++)
        {
            double v = sample(i);
            if (prev > 0 && v <= 0 || prev < 0 && v >= 0)
            {
                double t = (i - 1 + prev / (prev - v)) * Dt;
                if (crossings == 0) { first = t; }
                last = t; crossings++;
            }
            prev = v;
        }
        return crossings < 3 ? 0 : (crossings - 1) / (2.0 * (last - first));
    }

    [Fact]
    public void AFixedJointLinearSpringOscillatesAtItsFrequencyIn3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false });
        var anchor = w.AddBox(new Vector3F(1, 1, 1), at: default, type: BodyType.Static);
        var mass = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: new Vector3D(2, 0, 0));
        var joint = w.CreateJoint(new P3.FixedJointDef
        {
            BodyA = anchor, BodyB = mass, LinearHertz = 2f, LinearDampingRatio = 0f,
            LocalFrameA = new P3.JointFrame3D(new Vector3F(2, 0, 0), QuaternionF.Identity),
        });
        mass.LinearVelocity = new Vector3F(0, 0, 0);
        mass.SetPosition(new Vector3D(2.2, 0, 0));
        double hz = OscillationHertz(i => { if (i > 0) { w.Step(Dt); } return mass.Position.X - 2.0; }, 240);
        Assert.InRange(hz, 1.8, 2.2);
        // Rigid again once the spring is switched off: the displaced mass is pulled back to the frame.
        joint.SetFixedSprings(0f, 0f, 0f, 0f);
        mass.SetPosition(new Vector3D(2.3, 0, 0));
        for (int i = 0; i < 120; i++) { w.Step(Dt); }
        Assert.InRange(Math.Abs(mass.Position.X - 2.0), 0.0, 0.01);
    }

    [Fact]
    public void AFixedJointAngularSpringOscillatesAtItsFrequencyIn3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default, SleepEnabled = false });
        var anchor = w.AddBox(new Vector3F(1, 1, 1), at: default, type: BodyType.Static);
        var rotor = w.AddBox(new Vector3F(0.5f, 0.5f, 0.5f), at: default);
        w.CreateJoint(new P3.FixedJointDef { BodyA = anchor, BodyB = rotor, AngularHertz = 1.5f, AngularDampingRatio = 0f });
        rotor.AngularVelocity = new Vector3F(0, 0, 0.3f);
        double hz = OscillationHertz(i => { if (i > 0) { w.Step(Dt); } return 2.0 * rotor.Rotation.Z; }, 300);
        Assert.InRange(hz, 1.35, 1.65);
        Assert.InRange(rotor.Position.Length(), 0.0, 1e-4);   // the position stays rigid while only the angle is soft
    }

    [Fact]
    public void FixedJointSpringsValidateAndKeepTheRigidDefault()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = default });
        var a = w.AddBox(new Vector3F(1, 1, 1), at: default, type: BodyType.Static);
        var b = w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(3, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateJoint(new P3.FixedJointDef { BodyA = a, BodyB = b, LinearHertz = -1f }));
        var joint = w.CreateJoint(new P3.FixedJointDef { BodyA = a, BodyB = b });
        Assert.Throws<ArgumentOutOfRangeException>(() => joint.SetFixedSprings(1f, -1f, 0f, 0f));
        var hinge = w.AddHinge(a, w.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 5, 0)), new Vector3D(0, 2.5, 0), new Vector3F(0, 0, 1));
        Assert.Throws<InvalidJointException>(() => ((P3.Joint)hinge).SetFixedSprings(1f, 0f, 1f, 0f));
    }
}
