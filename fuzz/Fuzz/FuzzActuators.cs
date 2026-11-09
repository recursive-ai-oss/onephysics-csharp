#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using AeroSurface3 = RecursiveAI.OnePhysics.Physics3D.AeroSurface;
using AeroSurface2 = RecursiveAI.OnePhysics.Physics2D.AeroSurface;
using ThrusterDef3 = RecursiveAI.OnePhysics.Physics3D.ThrusterDef;
using ThrusterDef2 = RecursiveAI.OnePhysics.Physics2D.ThrusterDef;
using Thruster3 = RecursiveAI.OnePhysics.Physics3D.Thruster;
using Thruster2 = RecursiveAI.OnePhysics.Physics2D.Thruster;
using Boat3 = RecursiveAI.OnePhysics.Physics3D.Boat;
using Boat2 = RecursiveAI.OnePhysics.Physics2D.Boat;
using Hovercraft3 = RecursiveAI.OnePhysics.Physics3D.Hovercraft;
using Hovercraft2 = RecursiveAI.OnePhysics.Physics2D.Hovercraft;
using Shape3 = RecursiveAI.OnePhysics.Physics3D.Shape;
using Shape2 = RecursiveAI.OnePhysics.Physics2D.Shape;
using BoatFoilDef3 = RecursiveAI.OnePhysics.Physics3D.BoatFoilDef;
using BoatFoilDef2 = RecursiveAI.OnePhysics.Physics2D.BoatFoilDef;
using HoverFanDef3 = RecursiveAI.OnePhysics.Physics3D.HoverFanDef;
using HoverFanDef2 = RecursiveAI.OnePhysics.Physics2D.HoverFanDef;
using BoatDef3 = RecursiveAI.OnePhysics.Physics3D.BoatDef;
using BoatDef2 = RecursiveAI.OnePhysics.Physics2D.BoatDef;
using HovercraftDef3 = RecursiveAI.OnePhysics.Physics3D.HovercraftDef;
using HovercraftDef2 = RecursiveAI.OnePhysics.Physics2D.HovercraftDef;
using ShapeDef3 = RecursiveAI.OnePhysics.Physics3D.ShapeDef;
using ShapeDef2 = RecursiveAI.OnePhysics.Physics2D.ShapeDef;
using Body2 = RecursiveAI.OnePhysics.Physics2D.Body;
using Body3 = RecursiveAI.OnePhysics.Physics3D.Body;

// Fuzz targets for aerodynamic surfaces, thrusters, boats and hovercraft.
// "world3d-actuators" and "world2d-actuators" turn bytes into a program: bodies, hostile definitions (NaN and infinite vectors, zero axes, huge and tiny areas, coefficients, ranges and counts), surfaces on shapes, thrusters, boats and
// hovercraft, inputs, wind and air, stepping, destroying bodies, shapes and components, stale handles, origin shifts and snapshot round trips. Only argument errors and typed physics errors may come out.
// "world3d-actuator-snapshot" and "world2d-actuator-snapshot" flip bytes in the snapshot of a world with all four: rejected with the world unchanged, or accepted and re-serialized to the same bytes.
internal static class FuzzActuators
{
    private static int s_made, s_refused, s_steps, s_accepted, s_rejected;
    private static byte[]? s_base3, s_base2;

    static FuzzActuators()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("actuator targets: made " + s_made + ", refused " + s_refused + ", steps " + s_steps + "; snapshots accepted " + s_accepted + ", rejected " + s_rejected);
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;
        public Reader(byte[] d) { _d = d; }
        public bool More => _i < _d.Length;
        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e6f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 256f;
        }

        public Vector2F V2F() => new(Float(), Float());

        /// <summary>A box size within 0.05 to 20 m (NaN included: it becomes 0.05): the targets are about the actuators, and a body of mass up to 1e30 in a 2D buoyancy field is refused by that field's own limit on the torque it would add.</summary>
        public float Side() { float f = Math.Abs(Float()); return f >= 0.05f && f <= 20f ? f : 0.05f + (Byte() % 100) * 0.1f; }
        public float Density() { float f = Math.Abs(Float()); return f <= 50f ? f : 1f; }
        public Vector3F V3F() => new(Float(), Float(), Float());
        public bool Bool() => (Byte() & 1) != 0;
        public T Pick<T>(List<T> l) => l.Count == 0 ? default! : l[Byte() % l.Count];
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static AeroCoefficients Coef(Reader r)
    {
        if (r.Byte() < 160) { return r.Bool() ? AeroCoefficients.Wing(1f + r.Byte() / 16f) : AeroCoefficients.Plate(1f + r.Byte() / 16f); }
        return new AeroCoefficients { LiftSlope = r.Float(), ZeroLiftAngle = r.Float(), StallAngle = r.Float(), StallBlend = r.Float(), MinDrag = r.Float(), InducedDrag = r.Float(), PlateDrag = r.Float() };
    }

    // ---- 3D ---------------------------------------------------------------------------------------

    private static AeroSurface3 Surface3(Reader r)
    {
        var s = r.Byte() % 3 == 0 ? AeroSurface3.Body(r.Float(), r.Float()) : AeroSurface3.Wing(r.Float(), 1f + r.Byte() / 16f);
        if (r.Byte() < 100) { s.Coefficients = Coef(r); }
        if (r.Byte() < 100) { s.Forward = r.V3F(); }
        if (r.Byte() < 100) { s.Up = r.V3F(); }
        if (r.Byte() < 100) { s.CenterOfPressure = r.V3F(); }
        s.Medium = (AeroMedium)(r.Byte() % 3);
        s.Kind = r.Byte() % 5 == 0 ? (AeroSurfaceKind)(r.Byte() % 3) : s.Kind;
        return s;
    }

    private static ThrusterDef3 Thruster3(Reader r)
    {
        var t = r.Byte() < 140 ? ThrusterDef3.Rocket(10f + r.Byte()) : new ThrusterDef3 { Position = r.V3F(), Direction = r.V3F(), MaxThrust = r.Float(), ResponseTime = r.Float(), MaxGimbalAngle = r.Float(), GimbalAxis = r.V3F(), ReverseFraction = r.Float(), SpinTorque = r.Float(), PitchSpeed = r.Float(), ImmersionDepth = r.Float() };
        t.Medium = (AeroMedium)(r.Byte() % 3);
        return t;
    }

    private static ThrusterDef3[] Thrusters3(Reader r) { var a = new ThrusterDef3[r.Byte() % 4]; for (int i = 0; i < a.Length; i++) { a[i] = Thruster3(r); } return a; }

    private static BoatDef3 Boat3(Reader r)
    {
        var d = r.Byte() % 3 == 0 ? BoatDef3.Speedboat : r.Bool() ? BoatDef3.Dinghy : BoatDef3.Barge;
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.Forward = r.V3F(); }
        if (r.Bool()) { d.Up = r.V3F(); }
        if (r.Bool()) { d.LongitudinalDrag = r.Float(); }
        if (r.Bool()) { d.LateralDrag = r.Float(); }
        if (r.Bool()) { d.VerticalDrag = r.Float(); }
        if (r.Bool()) { d.HeaveDamping = r.Float(); }
        if (r.Bool()) { d.MaxRudderAngle = r.Float(); }
        if (r.Bool()) { d.RudderRate = r.Float(); }
        if (r.Bool()) { var f = new BoatFoilDef3[r.Byte() % 5]; for (int i = 0; i < f.Length; i++) { f[i] = new BoatFoilDef3 { Position = r.V3F(), Area = r.Float(), AspectRatio = r.Float(), Horizontal = r.Bool(), SteerGain = r.Float() }; } d.Foils = f; }
        if (r.Bool()) { d.Propellers = Thrusters3(r); }
        if (r.Byte() < 20) { d.Foils = new BoatFoilDef3[] { null! }; }
        return d;
    }

    private static HovercraftDef3 Hover3(Reader r)
    {
        var d = HovercraftDef3.ForMass(1f + r.Byte() * 4f);
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.Forward = r.V3F(); }
        if (r.Bool()) { d.Up = r.V3F(); }
        if (r.Bool()) { d.SkirtDrag = r.Float(); }
        if (r.Bool()) { d.AngularDamping = r.Float(); }
        if (r.Bool()) { var f = new HoverFanDef3[r.Byte() % 6]; for (int i = 0; i < f.Length; i++) { f[i] = new HoverFanDef3 { Position = r.V3F(), Direction = r.V3F(), MaxLift = r.Float(), Range = r.Float(), Damping = r.Float() }; } d.Fans = f; }
        if (r.Bool()) { d.Thrusters = Thrusters3(r); }
        return d;
    }

    private static void StepTolerant(PhysicsWorld3D w)
    {
        try { w.StepChecked(1f / 60f); s_steps++; }
        catch (RecursiveAI.OnePhysics.Physics3D.BodyOutOfBoundsException e) { e.Body.Destroy(); }
    }

    private static void StepTolerant(PhysicsWorld2D w)
    {
        try { w.StepChecked(1f / 60f); s_steps++; }
        catch (RecursiveAI.OnePhysics.Physics2D.BodyOutOfBoundsException e) { e.Body.Destroy(); }
        // A 2D buoyancy field refuses, at the step, a force or torque beyond 1e12 (a body of enormous mass or inertia spinning in water; an accepted hostile snapshot can hold one). That limit is the field's, not the actuators': not a finding here.
        catch (ArgumentOutOfRangeException e) when (e.ParamName != null && e.ParamName.StartsWith("buoyancy", StringComparison.Ordinal)) { s_refused++; }
    }

    private static PhysicsWorld3D NewWorld3() => new(new WorldOptions3D { WorldId = 9, Substeps = 4 });

    private static PhysicsWorld2D NewWorld2() => new(new WorldOptions2D { WorldId = 9, Substeps = 4 });

    private static void Scene3(PhysicsWorld3D w)
    {
        w.AddBox(new Vector3F(100f, 1f, 100f), new Vector3D(0, -3.5, 0), type: BodyType.Static);
        w.AddWater(0.0);
        w.AddWind(new Vector3F(1f, 0f, 2f));
    }

    private static void Scene2(PhysicsWorld2D w)
    {
        w.AddBox(new Vector2F(100f, 1f), new Vector2D(0, -3.5), type: BodyType.Static);
        w.AddBuoyancyEffector(0.0, 1000f);
        w.AddWind(new Vector2F(1f, 0.5f));
    }

    private static void Op3(PhysicsWorld3D w, List<Body3> bodies, List<Shape3> shapes, List<Thruster3> ts, List<Boat3> boats, List<Hovercraft3> hovers, Reader r)
    {
        switch (r.Byte() % 18)
        {
            case 0: bodies.Add(w.AddBox(new Vector3F(r.Side(), r.Side(), r.Side()), new Vector3D(r.Float() % 40f, r.Float() % 6f, r.Float() % 40f), density: r.Density())); break;
            case 1: { var b = w.CreateBody(BodyType.Dynamic, new Vector3D(r.Float() % 30f, r.Float() % 5f, r.Float() % 30f)); bodies.Add(b); shapes.Add(w.CreateBox(b, new Vector3F(1f + r.Byte() / 32f, 0.3f + r.Byte() / 64f, 2f), 0f, new ShapeDef3 { Aero = r.Bool() ? Surface3(r) : null })); break; }
            case 2: { var s = r.Pick(shapes); if (s.IsValid) { s.Aero = r.Bool() ? Surface3(r) : null; _ = s.Aero; } break; }
            case 3: { var b = r.Pick(bodies); if (b.IsValid) { ts.Add(b.AddThruster(Thruster3(r))); s_made++; } break; }
            case 4: { var b = r.Pick(bodies); if (b.IsValid) { boats.Add(w.AddBoat(b, Boat3(r))); s_made++; } break; }
            case 5: { var b = r.Pick(bodies); if (b.IsValid) { hovers.Add(w.AddHovercraft(b, Hover3(r))); s_made++; } break; }
            case 6: { var t = r.Pick(ts); if (t.IsValid) { t.Throttle = r.Float(); t.Gimbal = r.V2F(); t.MaxThrust = r.Float(); _ = t.Force; _ = t.Output; } break; }
            case 7: { var b = r.Pick(boats); if (b.IsValid) { b.Throttle = r.Float(); b.Steer = r.Float(); _ = b.ForwardSpeed; _ = b.RudderAngle; _ = b.HullForce; } break; }
            case 8: { var h = r.Pick(hovers); if (h.IsValid) { h.Lift = r.Float(); h.Throttle = r.Float(); h.Steer = r.Float(); for (int i = 0; i < h.FanCount; i++) { _ = h.GetFanGap(i); _ = h.GetFanForce(i); } _ = h.GetFanGap(r.Byte()); } break; }
            case 9: w.AirDensity = r.Float(); break;
            case 10: w.AddWind(r.V3F()); break;
            case 11: for (int i = 0, n = 1 + r.Byte() % 8; i < n; i++) { StepTolerant(w); } break;
            case 12: { var b = r.Pick(bodies); if (b.IsValid) { b.Destroy(); } break; }
            case 13: { var s = r.Pick(shapes); if (s.IsValid) { s.Destroy(); } break; }
            case 14: { var t = r.Pick(ts); if (t.IsValid) { t.Destroy(); } var b = r.Pick(boats); if (b.IsValid && r.Bool()) { b.Destroy(); } var h = r.Pick(hovers); if (h.IsValid && r.Bool()) { h.Destroy(); } break; }
            case 15: { var t = r.Pick(ts); if (!t.IsValid && ts.Count > 0) { try { t.Throttle = 1f; } catch (StaleHandleException) { } catch (InvalidHandleException) { } } break; }
            case 16: w.ShiftOrigin(new Vector3D(r.Byte() % 8, 0, r.Byte() % 8)); break;
            default: { var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry(); w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Aero, g); break; }
        }
    }

    private static void RoundTrip3(PhysicsWorld3D w)
    {
        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld3();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    public static void Run3D(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld3();
        Scene3(w);
        var bodies = new List<Body3>(); var shapes = new List<Shape3>(); var ts = new List<Thruster3>(); var boats = new List<Boat3>(); var hovers = new List<Hovercraft3>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op3(w, bodies, shapes, ts, boats, hovers, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        RoundTrip3(w);
    }

    // ---- 2D ---------------------------------------------------------------------------------------

    private static AeroSurface2 Surface2(Reader r)
    {
        var s = r.Byte() % 3 == 0 ? AeroSurface2.Body(r.Float(), r.Float()) : AeroSurface2.Wing(r.Float(), 1f + r.Byte() / 16f);
        if (r.Byte() < 100) { s.Coefficients = Coef(r); }
        if (r.Byte() < 100) { s.Forward = r.V2F(); }
        if (r.Byte() < 100) { s.Up = r.V2F(); }
        if (r.Byte() < 100) { s.CenterOfPressure = r.V2F(); }
        s.Medium = (AeroMedium)(r.Byte() % 3);
        s.Kind = r.Byte() % 5 == 0 ? (AeroSurfaceKind)(r.Byte() % 3) : s.Kind;
        return s;
    }

    private static ThrusterDef2 Thruster2(Reader r)
    {
        var t = r.Byte() < 140 ? ThrusterDef2.Rocket(10f + r.Byte()) : new ThrusterDef2 { Position = r.V2F(), Direction = r.V2F(), MaxThrust = r.Float(), ResponseTime = r.Float(), MaxGimbalAngle = r.Float(), ReverseFraction = r.Float(), PitchSpeed = r.Float(), ImmersionDepth = r.Float() };
        t.Medium = (AeroMedium)(r.Byte() % 3);
        return t;
    }

    private static ThrusterDef2[] Thrusters2(Reader r) { var a = new ThrusterDef2[r.Byte() % 4]; for (int i = 0; i < a.Length; i++) { a[i] = Thruster2(r); } return a; }

    private static BoatDef2 Boat2(Reader r)
    {
        var d = r.Byte() % 3 == 0 ? BoatDef2.Speedboat : r.Bool() ? BoatDef2.Dinghy : BoatDef2.Barge;
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.Forward = r.V2F(); }
        if (r.Bool()) { d.Up = r.V2F(); }
        if (r.Bool()) { d.LongitudinalDrag = r.Float(); }
        if (r.Bool()) { d.VerticalDrag = r.Float(); }
        if (r.Bool()) { d.HeaveDamping = r.Float(); }
        if (r.Bool()) { d.MaxRudderAngle = r.Float(); }
        if (r.Bool()) { d.RudderRate = r.Float(); }
        if (r.Bool()) { var f = new BoatFoilDef2[r.Byte() % 5]; for (int i = 0; i < f.Length; i++) { f[i] = new BoatFoilDef2 { Position = r.V2F(), Area = r.Float(), AspectRatio = r.Float(), SteerGain = r.Float() }; } d.Foils = f; }
        if (r.Bool()) { d.Propellers = Thrusters2(r); }
        return d;
    }

    private static HovercraftDef2 Hover2(Reader r)
    {
        var d = HovercraftDef2.ForMass(1f + r.Byte() * 4f);
        if (r.Byte() < 140) { return d; }
        if (r.Bool()) { d.Forward = r.V2F(); }
        if (r.Bool()) { d.Up = r.V2F(); }
        if (r.Bool()) { d.SkirtDrag = r.Float(); }
        if (r.Bool()) { d.AngularDamping = r.Float(); }
        if (r.Bool()) { var f = new HoverFanDef2[r.Byte() % 6]; for (int i = 0; i < f.Length; i++) { f[i] = new HoverFanDef2 { Position = r.V2F(), Direction = r.V2F(), MaxLift = r.Float(), Range = r.Float(), Damping = r.Float() }; } d.Fans = f; }
        if (r.Bool()) { d.Thrusters = Thrusters2(r); }
        return d;
    }

    private static void Op2(PhysicsWorld2D w, List<Body2> bodies, List<Shape2> shapes, List<Thruster2> ts, List<Boat2> boats, List<Hovercraft2> hovers, Reader r)
    {
        switch (r.Byte() % 18)
        {
            case 0: bodies.Add(w.AddBox(new Vector2F(r.Side(), r.Side()), new Vector2D(r.Float() % 40f, r.Float() % 6f), r.Float(), density: r.Density())); break;
            case 1: { var b = w.CreateBody(BodyType.Dynamic, new Vector2D(r.Float() % 30f, r.Float() % 5f)); bodies.Add(b); shapes.Add(w.CreateBox(b, new Vector2F(1f + r.Byte() / 32f, 0.3f + r.Byte() / 64f), default, 0f, 0f, new ShapeDef2 { Aero = r.Bool() ? Surface2(r) : null })); break; }
            case 2: { var s = r.Pick(shapes); if (s.IsValid) { s.Aero = r.Bool() ? Surface2(r) : null; _ = s.Aero; } break; }
            case 3: { var b = r.Pick(bodies); if (b.IsValid) { ts.Add(b.AddThruster(Thruster2(r))); s_made++; } break; }
            case 4: { var b = r.Pick(bodies); if (b.IsValid) { boats.Add(w.AddBoat(b, Boat2(r))); s_made++; } break; }
            case 5: { var b = r.Pick(bodies); if (b.IsValid) { hovers.Add(w.AddHovercraft(b, Hover2(r))); s_made++; } break; }
            case 6: { var t = r.Pick(ts); if (t.IsValid) { t.Throttle = r.Float(); t.Gimbal = r.Float(); t.MaxThrust = r.Float(); _ = t.Force; _ = t.Output; } break; }
            case 7: { var b = r.Pick(boats); if (b.IsValid) { b.Throttle = r.Float(); b.Steer = r.Float(); _ = b.ForwardSpeed; _ = b.RudderAngle; _ = b.HullForce; } break; }
            case 8: { var h = r.Pick(hovers); if (h.IsValid) { h.Lift = r.Float(); h.Throttle = r.Float(); h.Steer = r.Float(); for (int i = 0; i < h.FanCount; i++) { _ = h.GetFanGap(i); _ = h.GetFanForce(i); } _ = h.GetFanGap(r.Byte()); } break; }
            case 9: w.AirDensity = r.Float(); break;
            case 10: w.AddWind(r.V2F()); break;
            case 11: for (int i = 0, n = 1 + r.Byte() % 8; i < n; i++) { StepTolerant(w); } break;
            case 12: { var b = r.Pick(bodies); if (b.IsValid) { b.Destroy(); } break; }
            case 13: { var s = r.Pick(shapes); if (s.IsValid) { s.Destroy(); } break; }
            case 14: { var t = r.Pick(ts); if (t.IsValid) { t.Destroy(); } var b = r.Pick(boats); if (b.IsValid && r.Bool()) { b.Destroy(); } var h = r.Pick(hovers); if (h.IsValid && r.Bool()) { h.Destroy(); } break; }
            case 15: { var t = r.Pick(ts); if (!t.IsValid && ts.Count > 0) { try { t.Throttle = 1f; } catch (StaleHandleException) { } catch (InvalidHandleException) { } } break; }
            case 16: w.ShiftOrigin(new Vector2D(r.Byte() % 8, 0)); break;
            default: { var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry(); w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Aero, g); break; }
        }
    }

    private static void RoundTrip2(PhysicsWorld2D w)
    {
        for (int i = 0; i < 4; i++) { StepTolerant(w); }
        byte[] snap = w.Snapshot();
        var w2 = NewWorld2();
        try { w2.Restore(snap); }
        catch (SnapshotFormatException e) when (e.Message.Contains("out of range", StringComparison.Ordinal) || e.Message.Contains("non-finite", StringComparison.Ordinal)) { return; }
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world hash differs"); }
        if (!w2.Snapshot().AsSpan().SequenceEqual(snap)) { throw new InvalidOperationException("restored world re-serializes differently"); }
        StepTolerant(w); StepTolerant(w2);
        if (w2.ComputeHash() != w.ComputeHash()) { throw new InvalidOperationException("restored world diverged after a step"); }
    }

    public static void Run2D(byte[] data)
    {
        var r = new Reader(data);
        var w = NewWorld2();
        Scene2(w);
        var bodies = new List<Body2>(); var shapes = new List<Shape2>(); var ts = new List<Thruster2>(); var boats = new List<Boat2>(); var hovers = new List<Hovercraft2>();
        int ops = 4 + r.Byte() % 48;
        for (int n = 0; n < ops && r.More; n++)
        {
            try { Op2(w, bodies, shapes, ts, boats, hovers, r); }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }

        RoundTrip2(w);
    }

    // ---- snapshots --------------------------------------------------------------------------------

    private static byte[] Base3()
    {
        if (s_base3 != null) { return s_base3; }
        var w = NewWorld3();
        Scene3(w);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.2, 0));
        w.CreateBox(hull, new Vector3F(1.2f, 0.6f, 3f), 0f, new ShapeDef3 { Density = 83f, Aero = AeroSurface3.Body(0.5f) });
        var boat = w.AddBoat(hull, BoatDef3.Dinghy); boat.Throttle = 0.7f; boat.Steer = 0.3f;
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(10, 1, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.3f, 3f), 0f, new ShapeDef3 { Density = 100f });
        var hc = w.AddHovercraft(craft, HovercraftDef3.ForMass(craft.Mass)); hc.Lift = 0.9f; hc.Throttle = 0.4f;
        var plane = w.CreateBody(BodyType.Dynamic, new Vector3D(-10, 30, 0));
        w.CreateBox(plane, new Vector3F(0.2f, 0.1f, 1f), 0f, new ShapeDef3 { Density = 100f, Aero = AeroSurface3.Wing(0.5f) });
        plane.AddThruster(ThrusterDef3.Rocket(5f, direction: new Vector3F(0f, 0f, 1f))).Throttle = 1f;
        for (int s = 0; s < 40; s++) { w.StepChecked(1f / 60f); }
        return s_base3 = w.Snapshot();
    }

    private static byte[] Base2()
    {
        if (s_base2 != null) { return s_base2; }
        var w = NewWorld2();
        Scene2(w);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.2));
        w.CreateBox(hull, new Vector2F(3f, 0.6f), default, 0f, 0f, new ShapeDef2 { Density = 83f, Aero = AeroSurface2.Body(0.5f) });
        var boat = w.AddBoat(hull, BoatDef2.Dinghy); boat.Throttle = 0.7f; boat.Steer = 0.3f;
        var craft = w.CreateBody(BodyType.Dynamic, new Vector2D(10, 1));
        w.CreateBox(craft, new Vector2F(2f, 0.3f), default, 0f, 0f, new ShapeDef2 { Density = 100f });
        var hc = w.AddHovercraft(craft, HovercraftDef2.ForMass(craft.Mass)); hc.Lift = 0.9f; hc.Throttle = 0.4f;
        var plane = w.CreateBody(BodyType.Dynamic, new Vector2D(-10, 30));
        w.CreateBox(plane, new Vector2F(1f, 0.1f), default, 0f, 0f, new ShapeDef2 { Density = 100f, Aero = AeroSurface2.Wing(0.5f) });
        plane.AddThruster(ThrusterDef2.Rocket(5f, direction: new Vector2F(1f, 0f))).Throttle = 1f;
        for (int s = 0; s < 40; s++) { w.StepChecked(1f / 60f); }
        return s_base2 = w.Snapshot();
    }

    private static byte[] Mutate(byte[] data, byte[] snapshot)
    {
        var bytes = (byte[])snapshot.Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 5 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }
        return bytes;
    }

    public static void Snapshot3D(byte[] data)
    {
        var bytes = Mutate(data, Base3());
        var w = NewWorld3();
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        foreach (var t in w.GetThrusters()) { _ = t.Force; t.Throttle = 1f; }
        StepTolerant(w); StepTolerant(w);
    }

    public static void Snapshot2D(byte[] data)
    {
        var bytes = Mutate(data, Base2());
        var w = NewWorld2();
        byte[] empty = w.Snapshot();
        try { w.Restore(bytes); }
        catch (SnapshotFormatException)
        {
            s_rejected++;
            if (!w.Snapshot().AsSpan().SequenceEqual(empty)) { throw new InvalidOperationException("a rejected snapshot changed the world"); }
            return;
        }

        s_accepted++;
        if (!w.Snapshot().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        foreach (var t in w.GetThrusters()) { _ = t.Force; t.Throttle = 1f; }
        StepTolerant(w); StepTolerant(w);
    }
}
