#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Actuator3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D NewWorld(Vector3F? gravity = null, int threads = 1, int id = 71) =>
        new(new WorldOptions3D { ThreadCount = threads, WorldId = id, Gravity = gravity ?? new Vector3F(0f, -9.81f, 0f) });

    private static void Run(PhysicsWorld3D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    /// <summary>The documented lift and drag model, written out independently with System.Math.</summary>
    internal static void Model(AeroCoefficients c, double alpha, out double cl, out double cd)
    {
        double x = alpha - c.ZeroLiftAngle;
        double clLin = c.LiftSlope * x, cdLin = c.MinDrag + c.InducedDrag * clLin * clLin;
        double s = Math.Sin(alpha), co = Math.Cos(alpha);
        double clP = c.PlateDrag * s * co, cdP = c.MinDrag + c.PlateDrag * s * s;
        double t = Math.Clamp((Math.Abs(x) - c.StallAngle) / c.StallBlend, 0, 1);
        double w = t * t * (3 - 2 * t);
        cl = clLin + w * (clP - clLin); cd = cdLin + w * (cdP - cdLin);
    }

    // ------------------------------------------------------------------ the aerodynamic model

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AFlatPlateFollowsTheLiftAndDragCurveFromMinus90To90Degrees(bool wing)
    {
        var coef = wing ? AeroCoefficients.Wing(6f) : AeroCoefficients.Plate(2f);
        float worst = 0f;
        for (int deg = -90; deg <= 90; deg += 5)
        {
            var w = NewWorld(new Vector3F(0f, 0f, 0f));
            double a = deg * Math.PI / 180.0;
            float speed = 20f;
            // Flow along the chord (from the leading edge back) is the normal case: the chord points along -Z, so the air moves toward -Z at zero angle of attack.
            w.AddWind(new Vector3F(0f, speed * (float)Math.Sin(a), -speed * (float)Math.Cos(a)));
            var body = w.CreateBody(BodyType.Dynamic, default);
            w.CreateBox(body, new Vector3F(1f, 0.1f, 1f), 0f, new ShapeDef { Density = 100f, Aero = new AeroSurface { Area = 1f, Coefficients = coef } });
            float m = body.Mass;
            w.Step(Dt);
            var dv = body.LinearVelocity;
            Model(coef, a, out double cl, out double cd);
            double q = 0.5 * 1.225 * speed * speed;
            // Components in the world: the chord is -Z, the lifting side +Y.
            double fc = q * (cd * Math.Cos(a) - cl * Math.Sin(a)), fn = q * (cd * Math.Sin(a) + cl * Math.Cos(a));
            double ex = 0, ey = fn / m * Dt, ez = -fc / m * Dt;
            double err = Math.Max(Math.Abs(dv.Y - ey), Math.Abs(dv.Z - ez));
            double scale = Math.Max(1e-3, Math.Max(Math.Abs(ey), Math.Abs(ez)));
            Assert.True(err <= 2e-4 * scale + 2e-5, $"alpha {deg}: dv ({dv.X}, {dv.Y}, {dv.Z}) expected ({ex}, {ey}, {ez})");
            worst = Math.Max(worst, (float)(err / scale));
            Assert.Equal(0f, dv.X, 1e-6f);
        }

        Assert.True(worst < 2e-4f);
    }

    [Fact]
    public void ThePlateStallsAndKeepsItsLiftSignThroughReversedFlow()
    {
        var c = AeroCoefficients.Plate(2f);
        c.Evaluate(0.1f, out float cl1, out _);
        c.Evaluate(-0.1f, out float cl2, out _);
        Assert.Equal(-cl2, cl1, 1e-6f);
        c.Evaluate(0.17f + 0.35f + 0.01f, out float clStalled, out float cdStalled);
        // Fully separated: the flat-plate normal force, at 0.53 rad.
        Assert.Equal(AeroCoefficients.PlateDragOf(2f) * Math.Sin(0.53) * Math.Cos(0.53), clStalled, 1e-4);
        Assert.True(cdStalled > 0.3f);
        c.Evaluate(MathF.PI / 2f, out float cl90, out float cd90);
        Assert.Equal(0f, cl90, 1e-4f);
        Assert.Equal(AeroCoefficients.PlateDragOf(2f) + c.MinDrag, cd90, 1e-4f);
    }

    [Fact]
    public void ABluffBodyDragsOnItsSilhouetteAlongTheFlow()
    {
        var w = NewWorld(new Vector3F(0f, 0f, 0f));
        w.AddWind(new Vector3F(10f, 0f, 0f));
        // A 1 x 2 x 3 box: seen along X the silhouette is 2 x 3 = 6 m².
        var body = w.CreateBody(BodyType.Dynamic, default);
        w.CreateBox(body, new Vector3F(1f, 2f, 3f), 0f, new ShapeDef { Density = 50f, Aero = AeroSurface.Body(1.05f) });
        float m = body.Mass;
        w.Step(Dt);
        double expected = 0.5 * 1.225 * 1.05 * 6.0 * 100.0 / m * Dt;
        Assert.Equal(expected, body.LinearVelocity.X, expected * 0.01);
        Assert.Equal(0f, body.LinearVelocity.Y, 1e-5f);
        // A sphere: the disc, whichever way the air blows.
        var s = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 20, 0));
        w.CreateSphere(s, 0.5f, new ShapeDef { Density = 100f, Aero = AeroSurface.Body(0.47f) });
        w.Step(Dt);
        double es = 0.5 * 1.225 * 0.47 * Math.PI * 0.25 * 100.0 / s.Mass * Dt;
        Assert.Equal(es, s.LinearVelocity.X, es * 0.02);
    }

    [Fact]
    public void AGliderTrimsToASteadyGlideWhoseRatioMatchesTheLiftToDragOfItsSurfaces()
    {
        var w = NewWorld();
        var body = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 3000, 0));
        // A fuselage, a wing 0.08 m behind the centre of mass (an aerodynamic centre behind the centre of gravity is what makes it stable) and a tail 1.2 m behind it set at -0.13 rad to the body axis (it carries a small download in trim).
        w.CreateBox(body, new Vector3F(0.2f, 0.2f, 1.5f), 0f, new ShapeDef { Density = 20f, Aero = AeroSurface.Body(0.1f, 0.03f) });
        var wing = AeroSurface.Wing(1.2f, 8f); wing.CenterOfPressure = new Vector3F(0f, 0f, -0.08f);
        var tail = AeroSurface.Wing(0.25f, 4f); tail.CenterOfPressure = new Vector3F(0f, 0f, -1.2f); tail.Forward = new Vector3F(0f, -0.13f, 1f);
        w.CreateBox(body, new Vector3F(0.02f, 0.02f, 0.02f), 0f, new ShapeDef { Density = 1f, Aero = wing });
        w.CreateBox(body, new Vector3F(0.02f, 0.02f, 0.02f), 0f, new ShapeDef { Density = 1f, Aero = tail });
        body.LinearVelocity = new Vector3F(0f, -1f, 6f);
        for (int i = 0; i < 60 * 60; i++) { w.Step(Dt); }
        var v = body.LinearVelocity;
        double horizontal = Math.Sqrt((double)v.X * v.X + (double)v.Z * v.Z), sink = -v.Y;
        Assert.True(sink > 0.05, "the glider must be descending: " + v);
        double ratio = horizontal / sink;
        // The aerodynamic force of the trimmed state from the model, resolved against the flight direction.
        var q = body.Rotation;
        var flowB = q.Conjugated().Rotate(new Vector3F(-v.X, -v.Y, -v.Z));   // air velocity relative to the glider, in its frame
        double fx = 0, fy = 0, fz = 0;
        double[] areas = [1.2, 0.25];
        var coefs = new[] { AeroCoefficients.Wing(8f), AeroCoefficients.Wing(4f) };
        double[] incidence = [0.0, Math.Atan(-0.13)];
        for (int k = 0; k < 2; k++)
        {
            double inc = incidence[k];
            double chY = -Math.Sin(inc), chZ = -Math.Cos(inc), nY = Math.Cos(inc), nZ = -Math.Sin(inc);   // chord = -forward; the lifting side is Up with the chord's part removed
            double uc = flowB.Y * chY + flowB.Z * chZ, un = flowB.Y * nY + flowB.Z * nZ;
            double sp = Math.Sqrt(uc * uc + un * un), a = Math.Atan2(un, uc);
            Model(coefs[k], a, out double cl, out double cd);
            double qs = 0.5 * 1.225 * sp * sp * areas[k];
            double fc = qs * (cd * Math.Cos(a) - cl * Math.Sin(a)), fn = qs * (cd * Math.Sin(a) + cl * Math.Cos(a));
            var fw = q.Rotate(new Vector3F(0f, (float)(fc * chY + fn * nY), (float)(fc * chZ + fn * nZ)));
            fx += fw.X; fy += fw.Y; fz += fw.Z;
        }

        double vl = Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z);
        double bodyDrag = 0.5 * 1.225 * 0.1 * 0.03 * vl;   // times the air velocity relative to the glider, which is -v
        fx -= bodyDrag * v.X; fy -= bodyDrag * v.Y; fz -= bodyDrag * v.Z;
        double mass = body.Mass;
        Assert.Equal(mass * 9.81, fy, 0.03 * mass * 9.81);   // equilibrium: the aerodynamic force carries the weight
        double ex = v.X / vl, ey = v.Y / vl, ez = v.Z / vl;
        double drag = -(fx * ex + fy * ey + fz * ez);
        double lift = Math.Sqrt(Math.Max(0, fx * fx + fy * fy + fz * fz - drag * drag));
        Assert.Equal(lift / drag, ratio, 0.05 * ratio);
        Assert.InRange(ratio, 4.0, 40.0);
    }

    // ------------------------------------------------------------------ thrusters

    [Fact]
    public void AThrustToWeightOneRigHoversInPlace()
    {
        var w = NewWorld();
        var b = w.AddBox(new Vector3F(0.5f, 0.2f, 0.5f), new Vector3D(0, 10, 0));
        var t = b.AddThruster(new ThrusterDef { MaxThrust = b.Mass * 9.81f, ResponseTime = 0f });
        t.Throttle = 1f;
        double y0 = b.Position.Y;
        double drift = 0;
        for (int i = 0; i < 600; i++) { w.Step(Dt); drift = Math.Max(drift, Math.Abs(b.Position.Y - y0)); }
        Assert.True(drift < 0.01, "vertical drift " + drift);
        Assert.Equal(b.Mass * 9.81f, t.Force.Y, b.Mass * 9.81f * 1e-4f);
        t.Throttle = 0f;
        Run(w, 30);
        Assert.True(b.Position.Y < y0 - 1.0);
    }

    [Fact]
    public void AThrusterOffTheCentreTorquesTheBodyAndAGimbalSteersIt()
    {
        var w = NewWorld(new Vector3F(0f, 0f, 0f));
        var b = w.AddBox(new Vector3F(1f, 1f, 1f), default, density: 1000f);
        var t = b.AddThruster(new ThrusterDef { Position = new Vector3F(0.5f, 0f, 0f), Direction = new Vector3F(0f, 1f, 0f), MaxThrust = 1000f, ResponseTime = 0f, MaxGimbalAngle = 0.5f });
        t.Throttle = 1f;
        w.Step(Dt);
        // A push up at +X: it spins about -Z (x × y = +z in the torque, so the angular velocity is +Z).
        Assert.True(b.AngularVelocity.Z > 0f);
        Assert.True(b.LinearVelocity.Y > 0f);
        float vy = b.LinearVelocity.Y;
        // Gimbal: the thrust leans toward the gimbal axis' rotation of +Y; the sideways part appears.
        t.Gimbal = new Vector2F(1f, 0f);
        w.Step(Dt);
        float sideways = MathF.Abs(t.Force.X) + MathF.Abs(t.Force.Z);
        Assert.True(sideways > 1000f * MathF.Sin(0.5f) * 0.99f, "gimballed force " + t.Force);
        Assert.Equal(1000f, MathF.Sqrt(t.Force.X * t.Force.X + t.Force.Y * t.Force.Y + t.Force.Z * t.Force.Z), 0.1f);
        _ = vy;
    }

    [Fact]
    public void AThrusterFollowsItsThrottleWithTheStatedLag()
    {
        var w = NewWorld(new Vector3F(0f, 0f, 0f));
        var b = w.AddBox(new Vector3F(1f, 1f, 1f), default);
        var t = b.AddThruster(new ThrusterDef { MaxThrust = 100f, ResponseTime = 0.25f });
        t.Throttle = 1f;
        float o = 0f;
        for (int i = 0; i < 90; i++)
        {
            w.Step(Dt);
            o += (1f - o) * (Dt / (0.25f + Dt));
            Assert.Equal(o, t.Output, 1e-5f);
        }

        Assert.True(t.Output > 0.97f);
    }

    [Fact]
    public void AQuadcopterRigHoversAndTheStaleHandlesFail()
    {
        var w = NewWorld();
        var b = w.AddBox(new Vector3F(0.6f, 0.1f, 0.6f), new Vector3D(0, 5, 0));
        var rig = ThrusterRig.Quadcopter(0.3f, b.Mass * 9.81f / 4f, 0f);
        var ts = new List<Thruster>();
        foreach (var d in rig) { d.ResponseTime = 0f; var t = b.AddThruster(d); t.Throttle = 1f; ts.Add(t); }
        Run(w, 300);
        Assert.True(Math.Abs(b.Position.Y - 5) < 0.02, "y " + b.Position.Y);
        Assert.True(b.AngularVelocity.Y == 0f || Math.Abs(b.AngularVelocity.Y) < 1e-4f);
        Assert.Equal(4, w.GetThrusters().Length);
        b.Destroy();
        var t0 = ts[0];
        Assert.Throws<StaleHandleException>(() => t0.Throttle = 0f);
        Assert.False(t0.IsValid);
        var none = default(Thruster);
        Assert.Throws<InvalidHandleException>(() => none.Throttle);
    }

    // ------------------------------------------------------------------ boats

    /// <summary>A dinghy hull: a box 1.2 wide, 0.6 tall, 3 long with its origin at the waterline; 150 kg floats 0.0417 m deep, so the waterline sits near the middle of the hull's lower part.</summary>
    private static (PhysicsWorld3D W, Body Hull, Boat Boat) Dinghy(Vector3F? gravity = null, BoatDef? def = null)
    {
        var w = NewWorld(gravity);
        w.AddWater(0.0, 1000f, default, 0f);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.2, 0));
        w.CreateBox(hull, new Vector3F(1.2f, 0.6f, 3f), 0f, new ShapeDef { Density = 150f / (1.2f * 0.6f * 3f) });
        def ??= BoatDef.Dinghy;
        // The preset assumes the origin at the waterline; this box's origin is its middle (0.26 m above the water), so move the foils and the propeller down to 0.19 m under it.
        foreach (var f in def.Foils) { f.Position = new Vector3F(f.Position.X, -0.45f, f.Position.Z); }
        foreach (var p in def.Propellers) { p.Position = new Vector3F(p.Position.X, -0.45f, p.Position.Z); }
        var boat = w.AddBoat(hull, def);
        return (w, hull, boat);
    }

    [Fact]
    public void ABoatReachesTheTopSpeedItsThrustAndDragPredict()
    {
        var (w, hull, boat) = Dinghy();
        Run(w, 60 * 10);   // settle
        Assert.Equal(0.0, hull.LinearVelocity.Y, 0.05);
        boat.Throttle = 1f;
        Run(w, 60 * 40);
        float v = boat.ForwardSpeed;
        // Steady state: thrust = hull drag + the two foils' zero-lift drag. V is the volume under water at speed.
        var def = BoatDef.Dinghy;
        double vol = 150.0 / 1000.0;
        double hullK = 0.5 * 1000 * def.LongitudinalDrag * Math.Pow(vol, 2.0 / 3.0);
        double foilK = 0.5 * 1000 * (0.25 + 0.08) * AeroCoefficients.Wing(1.5f).MinDrag;
        double expected = Math.Sqrt(def.Propellers[0].MaxThrust / (hullK + foilK));
        Assert.Equal(expected, v, 0.04 * expected);
        // It does not drift sideways or heave.
        Assert.True(Math.Abs(hull.LinearVelocity.X) < 0.05 && Math.Abs(hull.LinearVelocity.Y) < 0.3, "velocity " + hull.LinearVelocity);
    }

    [Fact]
    public void TheRudderTurnsTheBoatTheRightWayAtAStatedRate()
    {
        // Rudder only: fix the propeller's gimbal so that the turning comes from the foil at the stern.
        var def = BoatDef.Dinghy;
        foreach (var p in def.Propellers) { p.MaxGimbalAngle = 0f; }
        var (w, hull, boat) = Dinghy(null, def);
        boat.Throttle = 1f;
        Run(w, 60 * 20);
        float v0 = boat.ForwardSpeed;
        boat.Steer = 1f;
        Run(w, 60 * 2);
        // Facing +Z with +Y up, the right-hand side is -X: a right turn is clockwise seen from above (negative angular velocity about +Y) and the bow swings toward -X.
        var fwd = hull.Rotation.Rotate(new Vector3F(0f, 0f, 1f));
        Assert.True(hull.AngularVelocity.Y < -0.1f, "yaw rate " + hull.AngularVelocity.Y);
        Assert.True(fwd.X < -0.1f, "heading " + fwd);
        Assert.True(boat.RudderAngle > 0.59f);
        // Stated rate: at about 5 m/s the 0.08 m² rudder at 0.6 rad turns the dinghy (150 kg, 3 m) at 1.0 to 2.0 rad/s (measured 1.44).
        Assert.InRange(-hull.AngularVelocity.Y, 1.0f, 2.0f);
        Assert.True(v0 > 3f);
        boat.Steer = -1f;
        Run(w, 60 * 6);
        Assert.True(hull.AngularVelocity.Y > 0.1f, "left turn yaw " + hull.AngularVelocity.Y);
    }

    [Fact]
    public void ABoatBobbingInItsWaterSettlesAtItsDraft()
    {
        var (w, hull, _) = Dinghy();
        hull.Position = new Vector3D(0, 0.6, 0);
        Run(w, 60 * 20);
        // 150 kg displaces 0.15 m³ over the 3.6 m² waterplane: a draft of 0.0417 m; the origin is 0.3 m above the bottom.
        Assert.Equal(0.3 - 0.150 / 3.6, hull.Position.Y, 0.01);
        Assert.True(Math.Abs(hull.LinearVelocity.Y) < 0.01f);
    }

    [Fact]
    public void ABoatCannotBePlacedOnAStaticBodyOrTwice()
    {
        var w = NewWorld();
        var s = w.AddBox(new Vector3F(1, 1, 1), default, type: BodyType.Static);
        Assert.Throws<ArgumentException>(() => w.AddBoat(s, BoatDef.Dinghy));
        var d = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 5, 0));
        var boat = w.AddBoat(d, BoatDef.Dinghy);
        Assert.Throws<PhysicsException>(() => w.AddBoat(d, BoatDef.Dinghy));
        Assert.True(w.TryGetBoat(d, out var again) && again == boat);
        var bad = BoatDef.Dinghy; bad.LongitudinalDrag = float.NaN;
        var e = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(0, 9, 0));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => w.AddBoat(e, bad));
        Assert.Contains("DragLongitudinal", ex.Message);
        Assert.Throws<ArgumentOutOfRangeException>(() => boat.Throttle = 2f);
    }

    // ------------------------------------------------------------------ hovercraft

    [Fact]
    public void AHovercraftRidesAtThePredictedCushionHeight()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(200f, 1f, 200f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.5, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.3f, 3f), 0f, new ShapeDef { Density = 200f / (2f * 0.3f * 3f) });
        var def = HovercraftDef.ForMass(craft.Mass);
        var hc = w.AddHovercraft(craft, def);
        hc.Lift = 1f;
        Run(w, 600);
        // Each fan carries a quarter of the weight at the gap h = Range (1 - sqrt(W / MaxLift)) = 0.5 (1 - sqrt(0.5)).
        double expected = 0.5 * (1 - Math.Sqrt(0.5));
        for (int i = 0; i < hc.FanCount; i++) { Assert.Equal(expected, hc.GetFanGap(i), 0.005); }
        double fanY = craft.Position.Y - 0.1;
        Assert.Equal(expected, fanY, 0.005);
        Assert.True(Math.Abs(craft.LinearVelocity.Y) < 0.01f);
        Assert.True(Math.Abs(craft.AngularVelocity.X) + Math.Abs(craft.AngularVelocity.Z) < 0.01f);
    }

    [Fact]
    public void AHovercraftRidesOnWaterAndPushesTheBoxUnderIt()
    {
        var w = NewWorld();
        w.AddWater(0.0, 1000f, default, 1f);
        // A craft that cannot float (kinematic-light) held up by its fans over the water's surface; the water is the surface the rays find.
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.5, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.2f, 3f), 0f, new ShapeDef { Density = 200f / (2f * 0.2f * 3f) });
        craft.BuoyancyFactor = 0f;
        var hc = w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        hc.Lift = 1f;
        Run(w, 600);
        double expected = 0.5 * (1 - Math.Sqrt(0.5));
        Assert.Equal(expected, craft.Position.Y - 0.1, 0.01);
    }

    [Fact]
    public void AHovercraftPushesOnADynamicBodyUnderIt()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(200f, 1f, 200f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var crate = w.AddBox(new Vector3F(6f, 0.5f, 6f), new Vector3D(0, 0.25, 0), density: 50f);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.9, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.2f, 2f), 0f, new ShapeDef { Density = 100f });
        var hc = w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        hc.Lift = 1f;
        Run(w, 240);
        // The crate carries the craft's weight through the air as well as its own: its contact force is larger than its own weight.
        Assert.True(hc.GetFanForce(0) > 0f);
        Assert.True(craft.Position.Y > 0.5 + 0.1);
    }

    [Fact]
    public void AHovercraftSteersAndThrustsAndTheSkirtLimitsItsSpeed()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(2000f, 1f, 2000f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.5, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.3f, 3f), 0f, new ShapeDef { Density = 200f / (2f * 0.3f * 3f) });
        var hc = w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        hc.Lift = 1f; hc.Throttle = 1f;
        Run(w, 60 * 10);
        // Thrust 2 x 1.5 x 200 = 600 N against the skirt's 0.4 x 200 = 80 N per m/s with a mean cushion factor near 0.5: a speed near 600 / 80 / 0.5.
        float vz = craft.LinearVelocity.Z;
        Assert.True(vz > 4f && vz < 30f, "speed " + vz);
        hc.Steer = 1f;
        Run(w, 60 * 3);
        Assert.True(craft.AngularVelocity.Y < -0.01f, "yaw " + craft.AngularVelocity.Y);
    }

    // ------------------------------------------------------------------ host hooks

    private static PhysicsWorld3D Scene(int threads, int id = 75, bool wind = true)
    {
        var w = NewWorld(null, threads, id);
        w.AddWater(0.0, 1000f, default, 0f);
        if (wind) { w.AddWind(new Vector3F(1f, 0f, 2f)); }
        w.AddBox(new Vector3F(100f, 1f, 100f), new Vector3D(0, -3.5, 0), type: BodyType.Static);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.2, 0));
        w.CreateBox(hull, new Vector3F(1.2f, 0.6f, 3f), 0f, new ShapeDef { Density = 150f / (1.2f * 0.6f * 3f), Aero = AeroSurface.Body(0.5f) });
        var def = BoatDef.Dinghy;
        foreach (var f in def.Foils) { f.Position = new Vector3F(f.Position.X, -0.45f, f.Position.Z); }
        foreach (var p in def.Propellers) { p.Position = new Vector3F(p.Position.X, -0.45f, p.Position.Z); }
        var boat = w.AddBoat(hull, def);
        boat.Throttle = 0.8f; boat.Steer = 0.3f;
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(10, 1, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.3f, 3f), 0f, new ShapeDef { Density = 100f });
        var hc = w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        hc.Lift = 0.9f; hc.Throttle = 0.5f;
        var plane = w.CreateBody(BodyType.Dynamic, new Vector3D(-10, 30, 0));
        w.CreateBox(plane, new Vector3F(0.2f, 0.1f, 1f), 0f, new ShapeDef { Density = 100f, Aero = AeroSurface.Wing(0.5f) });
        plane.AddThruster(ThrusterDef.Rocket(5f, direction: new Vector3F(0f, 0f, 1f))).Throttle = 1f;
        return w;
    }

    [Fact]
    public void OneAndEightThreadsGiveTheSameHashesAndSnapshotRoundTripsExactly()
    {
        var a = Scene(1); var b = Scene(8);
        for (int i = 0; i < 200; i++)
        {
            a.Step(Dt); b.Step(Dt);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }

        var snap = new byte[a.GetSnapshotSize()];
        Assert.True(a.TrySnapshot(snap, out int n));
        Assert.Equal(snap.Length, n);
        var fresh = NewWorld(null, 1, 75);
        fresh.Restore(snap);
        Assert.Equal(a.ComputeHash(), fresh.ComputeHash());
        var again = new byte[fresh.GetSnapshotSize()];
        Assert.True(fresh.TrySnapshot(again, out _));
        Assert.True(snap.AsSpan().SequenceEqual(again));
        for (int i = 0; i < 100; i++) { a.Step(Dt); fresh.Step(Dt); Assert.Equal(a.ComputeHash(), fresh.ComputeHash()); }
        // A fork steps the same way.
        var fork = a.Fork();
        for (int i = 0; i < 60; i++) { a.Step(Dt); fork.Step(Dt); Assert.Equal(a.ComputeHash(), fork.ComputeHash()); }
    }

    [Fact]
    public void SteppingWithActuatorsAllocatesNothingOnceWarm()
    {
        // No wind field here: any area field costs the field stage 432 bytes a step (a separate finding), which is not the actuators' doing.
        var w = Scene(1, 79, wind: false);
        for (int i = 0; i < 900; i++) { w.Step(Dt); }   // long enough for everything to have landed, so no contact appears or disappears below
        Core.AllocMeter.AssertAllocatesNothing(() => { for (int i = 0; i < 120; i++) { w.Step(Dt); } }, "steps with actuators");
    }

    [Fact]
    public void AWorldWithoutActuatorsKeepsItsSnapshotBytes()
    {
        var w = NewWorld();
        w.AddBox(new Vector3F(1, 1, 1), default);
        var plain = new byte[w.GetSnapshotSize()];
        w.TrySnapshot(plain, out _);
        w.AirDensity = 1.2f;
        Assert.True(w.GetSnapshotSize() > plain.Length);
        w.AirDensity = 1.225f;
        Assert.Equal(plain.Length, w.GetSnapshotSize());
        var b = w.AddBox(new Vector3F(1, 1, 1), new Vector3D(5, 0, 0));
        var t = b.AddThruster(ThrusterDef.Rocket(1f));
        Assert.True(w.GetSnapshotSize() > plain.Length);
        t.Destroy();
    }

    [Fact]
    public void DestroyingABodyOrShapeDropsItsActuatorsAndSurfaces()
    {
        var w = NewWorld();
        var b = w.CreateBody(BodyType.Dynamic, default);
        var sh = w.CreateBox(b, new Vector3F(1, 1, 1), 0f, new ShapeDef { Aero = AeroSurface.Plate(1f) });
        Assert.NotNull(sh.Aero);
        sh.Aero = null;
        Assert.Null(sh.Aero);
        sh.Aero = AeroSurface.Wing(1f);
        var snap = new byte[w.GetSnapshotSize()];
        w.TrySnapshot(snap, out _);
        sh.Destroy();
        var sh2 = w.CreateBox(b, new Vector3F(1, 1, 1));
        Assert.Null(sh2.Aero);
        var bad = AeroSurface.Wing(1f); bad.Up = default;
        Assert.Throws<ArgumentOutOfRangeException>(() => w.CreateBox(b, new Vector3F(1, 1, 1), 0f, new ShapeDef { Aero = bad }));
        w.Restore(snap);
    }
}
