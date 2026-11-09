#pragma warning disable OPHYS0003
using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class Actuator2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(Vector2F? gravity = null, int threads = 1, int id = 72) =>
        new(new WorldOptions2D { ThreadCount = threads, WorldId = id, Gravity = gravity ?? new Vector2F(0f, -9.81f) });

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AFlatPlateFollowsTheLiftAndDragCurveFromMinus90To90Degrees(bool wing)
    {
        var coef = wing ? AeroCoefficients.Wing(6f) : AeroCoefficients.Plate(2f);
        for (int deg = -90; deg <= 90; deg += 5)
        {
            var w = NewWorld(new Vector2F(0f, 0f));
            double a = deg * Math.PI / 180.0;
            float speed = 20f;
            // The chord points along -X: at zero angle of attack the air moves toward -X.
            w.AddWind(new Vector2F(-speed * (float)Math.Cos(a), speed * (float)Math.Sin(a)));
            var body = w.CreateBody(BodyType.Dynamic, default);
            w.CreateBox(body, new Vector2F(1f, 0.1f), default, 0f, 0f, new ShapeDef { Density = 100f, Aero = new AeroSurface { Area = 1f, Coefficients = coef } });
            float m = body.Mass;
            w.Step(Dt);
            var dv = body.LinearVelocity;
            Actuator3DModel(coef, a, out double cl, out double cd);
            double q = 0.5 * 1.225 * speed * speed;
            double fc = q * (cd * Math.Cos(a) - cl * Math.Sin(a)), fn = q * (cd * Math.Sin(a) + cl * Math.Cos(a));
            double ex = -fc / m * Dt, ey = fn / m * Dt;
            double err = Math.Max(Math.Abs(dv.X - ex), Math.Abs(dv.Y - ey));
            double scale = Math.Max(1e-3, Math.Max(Math.Abs(ex), Math.Abs(ey)));
            Assert.True(err <= 2e-4 * scale + 2e-5, $"alpha {deg}: dv {dv} expected ({ex}, {ey})");
        }
    }

    private static void Actuator3DModel(AeroCoefficients c, double alpha, out double cl, out double cd) => World3D.Actuator3DTests.Model(c, alpha, out cl, out cd);

    [Fact]
    public void ABluffBodyDragsOnItsWidthAcrossTheFlow()
    {
        var w = NewWorld(new Vector2F(0f, 0f));
        w.AddWind(new Vector2F(10f, 0f));
        // A 1 x 2 box seen along X is 2 m wide.
        var body = w.CreateBody(BodyType.Dynamic, default);
        w.CreateBox(body, new Vector2F(1f, 2f), default, 0f, 0f, new ShapeDef { Density = 50f, Aero = AeroSurface.Body(1.05f) });
        w.Step(Dt);
        double expected = 0.5 * 1.225 * 1.05 * 2.0 * 100.0 / body.Mass * Dt;
        Assert.Equal(expected, body.LinearVelocity.X, expected * 0.01);
        // A circle is its diameter whichever way the air blows.
        var c = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 20));
        w.CreateCircle(c, 0.5f, default, new ShapeDef { Density = 100f, Aero = AeroSurface.Body(0.47f) });
        w.Step(Dt);
        double ec = 0.5 * 1.225 * 0.47 * 1.0 * 100.0 / c.Mass * Dt;
        Assert.Equal(ec, c.LinearVelocity.X, ec * 0.02);
    }

    [Fact]
    public void AGliderTrimsToASteadyGlideAndItsRatioMatchesTheLiftToDragOfItsSurfaces()
    {
        var w = NewWorld();
        var body = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 3000));
        // Per metre of depth: a fuselage, a wing 0.08 m behind the centre of mass and a tail 1.2 m behind it set at -0.13 rad to the body axis.
        w.CreateBox(body, new Vector2F(1.5f, 0.2f), default, 0f, 0f, new ShapeDef { Density = 20f, Aero = AeroSurface.Body(0.1f, 0.03f) });
        var wing = AeroSurface.Wing(1.2f, 8f); wing.CenterOfPressure = new Vector2F(-0.08f, 0f);
        var tail = AeroSurface.Wing(0.25f, 4f); tail.CenterOfPressure = new Vector2F(-1.2f, 0f); tail.Forward = new Vector2F(1f, -0.13f);
        w.CreateBox(body, new Vector2F(0.02f, 0.02f), new Vector2F(0f, 0.2f), 0f, 0f, new ShapeDef { Density = 1f, Aero = wing });
        w.CreateBox(body, new Vector2F(0.02f, 0.02f), new Vector2F(0f, 0.2f), 0f, 0f, new ShapeDef { Density = 1f, Aero = tail });
        body.LinearVelocity = new Vector2F(13.4f, -1.1f);   // near the trimmed speed, so that the phugoid (a long, lightly damped oscillation) has died away by the end
        for (int i = 0; i < 60 * 60; i++) { w.Step(Dt); }
        var v = body.LinearVelocity;
        double sink = -v.Y;
        Assert.True(sink > 0.05, "the glider must be descending: " + v);
        double ratio = v.X / sink;
        // The trimmed state's aerodynamic force from the model: the surfaces' forces and the fuselage's drag against the flight direction.
        double ang = body.Angle;
        double ca = Math.Cos(ang), sa = Math.Sin(ang);
        double vbx = ca * -v.X + sa * -v.Y, vby = -sa * -v.X + ca * -v.Y;   // air velocity relative to the glider in its frame
        double fx = 0, fy = 0;
        double[] areas = [1.2, 0.25];
        var coefs = new[] { AeroCoefficients.Wing(8f), AeroCoefficients.Wing(4f) };
        double[] incidence = [0.0, Math.Atan(-0.13)];
        for (int k = 0; k < 2; k++)
        {
            double inc = incidence[k];
            double chX = -Math.Cos(inc), chY = -Math.Sin(inc), nX = -Math.Sin(inc), nY = Math.Cos(inc);
            double uc = vbx * chX + vby * chY, un = vbx * nX + vby * nY;
            double sp = Math.Sqrt(uc * uc + un * un), a = Math.Atan2(un, uc);
            Actuator3DModel(coefs[k], a, out double cl, out double cd);
            double qs = 0.5 * 1.225 * sp * sp * areas[k];
            double fcv = qs * (cd * Math.Cos(a) - cl * Math.Sin(a)), fnv = qs * (cd * Math.Sin(a) + cl * Math.Cos(a));
            double bx = fcv * chX + fnv * nX, by = fcv * chY + fnv * nY;
            fx += ca * bx - sa * by; fy += sa * bx + ca * by;
        }

        double vl = Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y);
        double bodyDrag = 0.5 * 1.225 * 0.1 * 0.03 * vl;
        fx -= bodyDrag * v.X; fy -= bodyDrag * v.Y;
        double mass = body.Mass;
        Assert.Equal(mass * 9.81, fy, 0.03 * mass * 9.81);
        double ex = v.X / vl, ey = v.Y / vl;
        double drag = -(fx * ex + fy * ey);
        double lift = Math.Sqrt(Math.Max(0, fx * fx + fy * fy - drag * drag));
        Assert.Equal(lift / drag, ratio, 0.05 * ratio);
        Assert.InRange(ratio, 4.0, 40.0);
    }

    [Fact]
    public void AThrustToWeightOneRigHoversInPlaceAndALanderRespondsToItsThrusters()
    {
        var w = NewWorld();
        var b = w.AddBox(new Vector2F(0.5f, 0.2f), new Vector2D(0, 10));
        var t = b.AddThruster(new ThrusterDef { MaxThrust = b.Mass * 9.81f, ResponseTime = 0f });
        t.Throttle = 1f;
        double y0 = b.Position.Y, drift = 0;
        for (int i = 0; i < 600; i++) { w.Step(Dt); drift = Math.Max(drift, Math.Abs(b.Position.Y - y0)); }
        Assert.True(drift < 0.01, "vertical drift " + drift);
        t.Throttle = 0f;
        Run(w, 30);
        Assert.True(b.Position.Y < y0 - 1.0);

        var w2 = NewWorld(new Vector2F(0f, 0f));
        var lander = w2.AddBox(new Vector2F(2f, 2f), default, density: 100f);
        var rig = ThrusterRig.Lander(10000f, 500f, 1f, 1f);
        var main = lander.AddThruster(rig[0]); var left = lander.AddThruster(rig[1]); var right = lander.AddThruster(rig[2]);
        left.Throttle = 1f;
        Run(w2, 30);
        // The left thruster pushes toward +X from the upper left: the lander moves right and, being pushed above its centre, turns clockwise (negative angular velocity).
        Assert.True(lander.LinearVelocity.X > 0.1f);
        Assert.True(lander.AngularVelocity < 0f);
        Assert.True(Math.Abs(lander.LinearVelocity.Y) < 0.05f);
        left.Throttle = 0f; right.Throttle = 1f; main.Throttle = 1f;
        Run(w2, 60);
        Assert.True(lander.LinearVelocity.Y > 1f);
        main.Gimbal = 1f;
        w2.Step(Dt);
        Assert.True(Math.Abs(main.Force.X) > 10000f * Math.Sin(0.1) * 0.99);
    }

    private static (PhysicsWorld2D W, Body Hull, Boat Boat) Dinghy(BoatDef? def = null)
    {
        var w = NewWorld();
        w.AddBuoyancyEffector(0.0, 1000f);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.2));
        w.CreateBox(hull, new Vector2F(3f, 0.6f), default, 0f, 0f, new ShapeDef { Density = 150f / (3f * 0.6f) });
        def ??= BoatDef.Dinghy;
        foreach (var f in def.Foils) { f.Position = new Vector2F(f.Position.X, -0.45f); }
        foreach (var p in def.Propellers) { p.Position = new Vector2F(p.Position.X, -0.45f); }
        return (w, hull, w.AddBoat(hull, def));
    }

    [Fact]
    public void ABoatReachesTheTopSpeedItsThrustAndDragPredict()
    {
        var (w, hull, boat) = Dinghy();
        Run(w, 60 * 10);
        boat.Throttle = 1f;
        Run(w, 60 * 40);
        var def = BoatDef.Dinghy;
        double area = 150.0 / 1000.0;   // m² under water per metre of depth
        double hullK = 0.5 * 1000 * def.LongitudinalDrag * Math.Sqrt(area);
        double foilK = 0.5 * 1000 * (0.25 + 0.08) * AeroCoefficients.Wing(1.5f).MinDrag;
        double expected = Math.Sqrt(def.Propellers[0].MaxThrust / (hullK + foilK));
        Assert.Equal(expected, boat.ForwardSpeed, 0.05 * expected);
        Assert.True(Math.Abs(hull.LinearVelocity.Y) < 0.3f, "heave " + hull.LinearVelocity);
    }

    [Fact]
    public void ABoatSettlesAtItsDraftAndATrimFoilRotatesWithTheSteeringCommand()
    {
        var (w, hull, boat) = Dinghy();
        hull.Position = new Vector2D(0, 0.6);
        Run(w, 60 * 20);
        Assert.Equal(0.3 - 0.150 / 3.0, hull.Position.Y, 0.01);
        boat.Steer = 1f;
        Run(w, 60);
        Assert.Equal(0.6f, boat.RudderAngle, 1e-6f);
        Assert.Throws<ArgumentOutOfRangeException>(() => boat.Steer = 3f);
        Assert.True(w.TryGetBoat(hull, out var again) && again == boat);
        Assert.Throws<PhysicsException>(() => w.AddBoat(hull, BoatDef.Dinghy));
    }

    [Fact]
    public void AHovercraftRidesAtThePredictedCushionHeightAndOnWater()
    {
        var w = NewWorld();
        w.AddBox(new Vector2F(400f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.5));
        w.CreateBox(craft, new Vector2F(2f, 0.3f), default, 0f, 0f, new ShapeDef { Density = 200f / 0.6f });
        var hc = w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        hc.Lift = 1f;
        Run(w, 600);
        double expected = 0.5 * (1 - Math.Sqrt(0.5));
        for (int i = 0; i < hc.FanCount; i++) { Assert.Equal(expected, hc.GetFanGap(i), 0.005); }
        Assert.Equal(expected, craft.Position.Y - 0.1, 0.005);

        var w2 = NewWorld();
        w2.AddBuoyancyEffector(0.0, 1000f, default, 1f);
        var c2 = w2.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.5));
        w2.CreateBox(c2, new Vector2F(2f, 0.2f), default, 0f, 0f, new ShapeDef { Density = 200f / 0.4f });
        c2.BuoyancyFactor = 0f;
        var h2 = w2.AddHovercraft(c2, HovercraftDef.ForMass(c2.Mass));
        h2.Lift = 1f;
        Run(w2, 600);
        Assert.Equal(expected, c2.Position.Y - 0.1, 0.01);
    }

    private static PhysicsWorld2D Scene(int threads)
    {
        var w = NewWorld(null, threads, 76);
        w.AddBuoyancyEffector(0.0, 1000f);
        w.AddWind(new Vector2F(1f, 0.5f));
        w.AddBox(new Vector2F(100f, 1f), new Vector2D(0, -3.5), type: BodyType.Static);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.2));
        w.CreateBox(hull, new Vector2F(3f, 0.6f), default, 0f, 0f, new ShapeDef { Density = 150f / 1.8f, Aero = AeroSurface.Body(0.5f) });
        var def = BoatDef.Dinghy;
        foreach (var f in def.Foils) { f.Position = new Vector2F(f.Position.X, -0.45f); }
        foreach (var p in def.Propellers) { p.Position = new Vector2F(p.Position.X, -0.45f); }
        var boat = w.AddBoat(hull, def);
        boat.Throttle = 0.8f; boat.Steer = 0.3f;
        var craft = w.CreateBody(BodyType.Dynamic, new Vector2D(10, 1));
        w.CreateBox(craft, new Vector2F(2f, 0.3f), default, 0f, 0f, new ShapeDef { Density = 100f });
        var hc = w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        hc.Lift = 0.9f; hc.Throttle = 0.5f;
        var plane = w.CreateBody(BodyType.Dynamic, new Vector2D(-10, 30));
        w.CreateBox(plane, new Vector2F(1f, 0.1f), default, 0f, 0f, new ShapeDef { Density = 100f, Aero = AeroSurface.Wing(0.5f) });
        plane.AddThruster(ThrusterDef.Rocket(5f, direction: new Vector2F(1f, 0f))).Throttle = 1f;
        return w;
    }

    [Fact]
    public void OneAndEightThreadsGiveTheSameHashesAndSnapshotRoundTripsExactly()
    {
        var a = Scene(1); var b = Scene(8);
        for (int i = 0; i < 200; i++) { a.Step(Dt); b.Step(Dt); Assert.Equal(a.ComputeHash(), b.ComputeHash()); }
        var snap = new byte[a.GetSnapshotSize()];
        Assert.True(a.TrySnapshot(snap, out int n));
        Assert.Equal(snap.Length, n);
        var fresh = NewWorld(null, 1, 76);
        fresh.Restore(snap);
        Assert.Equal(a.ComputeHash(), fresh.ComputeHash());
        var again = new byte[fresh.GetSnapshotSize()];
        Assert.True(fresh.TrySnapshot(again, out _));
        Assert.True(snap.AsSpan().SequenceEqual(again));
        for (int i = 0; i < 100; i++) { a.Step(Dt); fresh.Step(Dt); Assert.Equal(a.ComputeHash(), fresh.ComputeHash()); }
        var fork = a.Fork();
        for (int i = 0; i < 60; i++) { a.Step(Dt); fork.Step(Dt); Assert.Equal(a.ComputeHash(), fork.ComputeHash()); }
    }

    [Fact]
    public void AWorldWithoutActuatorsKeepsItsSnapshotBytesAndOutOfPlaneInputIsRefused()
    {
        var w = NewWorld();
        var b = w.AddBox(new Vector2F(1, 1), default);
        int plain = w.GetSnapshotSize();
        var t = b.AddThruster(ThrusterDef.Rocket(1f));
        Assert.True(w.GetSnapshotSize() > plain);
        t.Destroy();
        Assert.Equal(plain, w.GetSnapshotSize());
        Assert.Throws<StaleHandleException>(() => t.Throttle = 1f);
        var bad = ThrusterDef.Rocket(1f); bad.Direction = new Vector2F(float.NaN, 1f);
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => b.AddThruster(bad));
        Assert.Contains("Direction", ex.Message);
        var s = w.AddBox(new Vector2F(1, 1), new Vector2D(5, 0), type: BodyType.Static);
        Assert.Throws<ArgumentException>(() => w.AddThruster(s, ThrusterDef.Rocket(1f)));
    }
}
