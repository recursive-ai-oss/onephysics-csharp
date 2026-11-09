using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Aero;

/// <summary>A body of water as a boat sees it: the half-space below a plane, with its density and flow.</summary>
internal struct WaterSample
{
    public Vector3F Normal;
    public double Offset;
    public float Density;
    public Vector3F Flow;
    public bool Limited;
    public Vector3D Min, Max;

    /// <summary>Height of a point above the surface, metres (negative under water).</summary>
    public readonly float HeightOf(Vector3D p) => (float)(Normal.X * p.X + Normal.Y * p.Y + Normal.Z * p.Z - Offset);

    /// <summary>False when the water is a pool and the point (grown by <paramref name="reach"/>) lies outside it.</summary>
    public readonly bool Covers(Vector3D p, float reach)
    {
        if (!Limited) { return true; }
        double r = reach;
        return !(p.X + r < Min.X || p.X - r > Max.X || p.Y + r < Min.Y || p.Y - r > Max.Y || p.Z + r < Min.Z || p.Z - r > Max.Z);
    }
}

/// <summary>What the actuator store needs from an engine. Planar engines present their bodies as 3D vectors with z = 0 and drop the components out of the plane when they take forces.</summary>
internal interface IActuatorHost
{
    WorldCore Core { get; }
    bool Planar { get; }
    int EngineSectionId { get; }
    ActuatorStore Actuators { get; }
    FieldStore FieldsOf { get; }
    float MassOf(int body);
    float ReachOf(int body);
    Vector3F LocalCenterOf(int body);
    Vector3D PositionOf(int body);
    Vector3F Rotate(int body, Vector3F v);
    Vector3F RotateInverse(int body, Vector3F v);
    Vector3F LinearVelocityOf(int body);
    Vector3F AngularVelocityOf(int body);

    /// <summary>Adds a force (world axes, through the centre of mass) and a torque about it to the body's accumulators for the coming step.</summary>
    void AddForce(int body, Vector3F force, Vector3F torque);

    /// <summary>The effective inverse mass of the body for a force along <paramref name="direction"/> (unit) acting at <paramref name="arm"/> from the centre of mass: how much the velocity of that point changes per newton-second along it.</summary>
    float InverseMassAt(int body, Vector3F arm, Vector3F direction);

    /// <summary>The inverse moment of inertia about a world axis (unit); 0 for an axis a planar body cannot turn about.</summary>
    float InverseInertiaAbout(int body, Vector3F axis);

    bool ShapeIsSensor(int shape);
    Vector3F ShapeToBody(int shape, Vector3F v);
    Vector3F ShapeOffset(int shape);

    /// <summary>Area of the shape's silhouette seen along <paramref name="directionInBodyFrame"/>, m² (a planar world: the width across it per metre of depth); 0 when the shape has none.</summary>
    float SilhouetteArea(int shape, Vector3F directionInBodyFrame);

    int FluidCount { get; }
    bool GetFluid(int index, out WaterSample water);

    /// <summary>The nearest shape along the ray (not belonging to <paramref name="ignoreBody"/>).</summary>
    bool Cast(Vector3D origin, Vector3F direction, float maxDistance, int ignoreBody, out float distance, out Vector3F normal, out int hitBody);
}

/// <summary>A thruster as the world keeps it: its definition, its commands and the response it has reached.</summary>
internal sealed class ThrusterState
{
    public int Body, BodyGen;
    public ThrusterSpec Spec = new();
    public float Throttle, Gimbal1, Gimbal2, Output;
    public Vector3F LastForce;   // world axes, not saved
}

internal sealed class BoatState
{
    public int Body, BodyGen;
    public BoatSpec Spec = new();
    public float Throttle, Steer, Rudder;
    public float[] PropOutput = Array.Empty<float>();
    public Vector3F LastHullForce;   // not saved
}

internal sealed class HoverState
{
    public int Body, BodyGen;
    public HoverSpec Spec = new();
    public float Lift, Throttle, Steer;
    public float[] ThrusterOutput = Array.Empty<float>();
    public float[] FanHeight = Array.Empty<float>();   // distance to the surface along each fan's axis as of the last step, or the range when none; not saved
    public float[] FanForce = Array.Empty<float>();    // not saved
}

internal sealed class SurfaceState
{
    public SurfaceSpec Spec = new();
    public int ShapeGen;
    public float AutoArea = float.NaN;   // planform area derived from the shape; not saved
}

/// <summary>
/// Everything that moves a body through air and water or holds it above a surface: lift and drag of shapes, thrusters, boats and hovercraft, one store shared by both engines. It runs as a serial stage before the world steps,
/// in slot order (shape surfaces by shape slot, then thrusters, boats and hovercraft by slot), adds forces to the bodies' accumulators and keeps its own state in an optional snapshot section.
/// </summary>
internal sealed partial class ActuatorStore : ISnapshotSection
{
    /// <summary>Section id: "AERO" read as a little-endian integer.</summary>
    public const int Id = 0x4F524541;

    public const float DefaultAirDensity = 1.225f;

    private readonly IActuatorHost _host;
    public float AirDensity = DefaultAirDensity;

    public HandleTable ThrusterHandles = new(), BoatHandles = new(), HoverHandles = new();
    public ThrusterState?[] Thrusters = new ThrusterState?[4];
    public BoatState?[] Boats = new BoatState?[4];
    public HoverState?[] Hovers = new HoverState?[4];
    public SurfaceState?[] Surfaces = new SurfaceState?[8];
    private int _surfaceCount;
    private int[] _boatOfBody = Array.Empty<int>(), _hoverOfBody = Array.Empty<int>();
    private float[] _submerged = Array.Empty<float>();

    public ActuatorStore(IActuatorHost host) { _host = host; }

    public int SurfaceCount => _surfaceCount;

    public bool HasWork => _surfaceCount > 0 || ThrusterHandles.Count > 0 || BoatHandles.Count > 0 || HoverHandles.Count > 0;

    // ------------------------------------------------------------------ bookkeeping

    private static void Index(ref int[] map, int body, int value)
    {
        if (body >= map.Length)
        {
            int old = map.Length;
            Array.Resize(ref map, Math.Max(body + 1, Math.Max(8, old * 2)));
            for (int i = old; i < map.Length; i++) { map[i] = -1; }
        }

        map[body] = value;
    }

    public int BoatOfBody(int body) => body < _boatOfBody.Length ? _boatOfBody[body] : -1;
    public int HoverOfBody(int body) => body < _hoverOfBody.Length ? _hoverOfBody[body] : -1;

    /// <summary>True when the body is a boat: the fluids leave its quadratic hull drag to the boat's own model.</summary>
    public bool OwnsHullDrag(int body) => body < _boatOfBody.Length && _boatOfBody[body] >= 0;

    /// <summary>Called by the fluid code with the volume (area in 2D) of a body under water this step; boats read it.</summary>
    public void AddSubmerged(int body, float measure)
    {
        if (BoatHandles.Count == 0) { return; }
        if (body >= _submerged.Length) { Array.Resize(ref _submerged, Math.Max(body + 1, Math.Max(8, _submerged.Length * 2))); }
        _submerged[body] += measure;
    }

    /// <summary>Starts a step: forgets last step's submerged measures.</summary>
    public void BeginStep()
    {
        if (BoatHandles.Count != 0) { Array.Clear(_submerged); }
    }

    private void CheckDynamic(BodyHandle body, out int b)
    {
        b = _host.Core.Resolve(body);
        if (_host.Core.Bodies.TypeOf(b) != BodyType.Dynamic) { throw new ArgumentException("The body must be dynamic.", nameof(body)); }
    }

    public int CreateThruster(BodyHandle body, ThrusterSpec spec)
    {
        CheckDynamic(body, out _);
        spec.Prepare(_host.Planar);
        int i = ThrusterHandles.Allocate();
        if (i >= Thrusters.Length) { Array.Resize(ref Thrusters, Math.Max(i + 1, Thrusters.Length * 2)); }
        Thrusters[i] = new ThrusterState { Body = body.Index, BodyGen = body.Generation, Spec = spec };
        return i;
    }

    public void DestroyThruster(int i) { Thrusters[i] = null; ThrusterHandles.Free(i); }

    public int CreateBoat(BodyHandle body, BoatSpec spec)
    {
        CheckDynamic(body, out int b);
        if (BoatOfBody(b) >= 0) { throw new PhysicsException("The body already is a boat."); }
        spec.Prepare(_host.Planar);
        if (!_host.Planar) { BindSteerAxis(spec.Propellers, spec.Up); }
        int i = BoatHandles.Allocate();
        if (i >= Boats.Length) { Array.Resize(ref Boats, Math.Max(i + 1, Boats.Length * 2)); }
        Boats[i] = new BoatState { Body = body.Index, BodyGen = body.Generation, Spec = spec, PropOutput = new float[spec.Propellers.Length] };
        Index(ref _boatOfBody, b, i);
        return i;
    }

    public void DestroyBoat(int i)
    {
        var s = Boats[i]!;
        if (s.Body < _boatOfBody.Length) { _boatOfBody[s.Body] = -1; }
        Boats[i] = null; BoatHandles.Free(i);
    }

    public int CreateHover(BodyHandle body, HoverSpec spec)
    {
        CheckDynamic(body, out int b);
        if (HoverOfBody(b) >= 0) { throw new PhysicsException("The body already is a hovercraft."); }
        spec.Prepare(_host.Planar);
        if (!_host.Planar) { BindSteerAxis(spec.Thrusters, spec.Up); }
        int i = HoverHandles.Allocate();
        if (i >= Hovers.Length) { Array.Resize(ref Hovers, Math.Max(i + 1, Hovers.Length * 2)); }
        var h = new HoverState { Body = body.Index, BodyGen = body.Generation, Spec = spec };
        h.ThrusterOutput = new float[spec.Thrusters.Length]; h.FanHeight = new float[spec.Fans.Length]; h.FanForce = new float[spec.Fans.Length];
        for (int k = 0; k < h.FanHeight.Length; k++) { h.FanHeight[k] = spec.Fans[k].Range; }
        Hovers[i] = h;
        Index(ref _hoverOfBody, b, i);
        return i;
    }

    public void DestroyHover(int i)
    {
        var s = Hovers[i]!;
        if (s.Body < _hoverOfBody.Length) { _hoverOfBody[s.Body] = -1; }
        Hovers[i] = null; HoverHandles.Free(i);
    }

    /// <summary>Steering gimbals of a boat's or hovercraft's thrusters turn about the craft's up axis unless the definition names another (a planar world turns them about z on its own).</summary>
    private void BindSteerAxis(ThrusterSpec[] thrusters, Vector3F up)
    {
        foreach (var t in thrusters)
        {
            if (t.GimbalAxis.X == 0f && t.GimbalAxis.Y == 0f && t.GimbalAxis.Z == 0f) { t.GimbalAxis = _host.Planar ? new Vector3F(0f, 0f, 1f) : up; }
        }
    }

    public void SetSurface(ShapeHandle shape, SurfaceSpec? spec, bool prepared = false)
    {
        int s = _host.Core.Resolve(shape);
        if (spec == null) { DropShape(s); return; }
        if (!prepared) { spec.Prepare(_host.Planar, areaRequired: false); }
        if (s >= Surfaces.Length) { Array.Resize(ref Surfaces, Math.Max(s + 1, Surfaces.Length * 2)); }
        if (Surfaces[s] == null) { _surfaceCount++; }
        Surfaces[s] = new SurfaceState { Spec = spec, ShapeGen = shape.Generation };
    }

    /// <summary>Called when a shape goes away (or its slot is reused): its surface goes with it.</summary>
    public void DropShape(int s)
    {
        if (s < Surfaces.Length && Surfaces[s] != null) { Surfaces[s] = null; _surfaceCount--; }
    }

    /// <summary>Called when a body is destroyed: its thrusters, boat and hovercraft go with it. (Its shapes' surfaces are dropped shape by shape.)</summary>
    public void DropBody(int body)
    {
        if (ThrusterHandles.Count > 0)
        {
            for (int i = 0; i < ThrusterHandles.HighWater; i++) { if (ThrusterHandles.IsLive(i) && Thrusters[i]!.Body == body) { DestroyThruster(i); } }
        }

        int bt = BoatOfBody(body);
        if (bt >= 0) { DestroyBoat(bt); }
        int hv = HoverOfBody(body);
        if (hv >= 0) { DestroyHover(hv); }
    }

    /// <summary>After a restore: every component names a live dynamic body of the generation it was saved with, and every surface a live shape.</summary>
    public bool Consistent()
    {
        var core = _host.Core;
        var bodies = core.Bodies;
        for (int i = 0; i < ThrusterHandles.HighWater; i++)
        {
            if (!ThrusterHandles.IsLive(i)) { continue; }
            var t = Thrusters[i]!;
            if (!bodies.Handles.IsAlive(t.Body, t.BodyGen) || bodies.TypeOf(t.Body) != BodyType.Dynamic) { return false; }
        }

        for (int i = 0; i < BoatHandles.HighWater; i++)
        {
            if (!BoatHandles.IsLive(i)) { continue; }
            var t = Boats[i]!;
            if (!bodies.Handles.IsAlive(t.Body, t.BodyGen) || bodies.TypeOf(t.Body) != BodyType.Dynamic || BoatOfBody(t.Body) != i) { return false; }
        }

        for (int i = 0; i < HoverHandles.HighWater; i++)
        {
            if (!HoverHandles.IsLive(i)) { continue; }
            var t = Hovers[i]!;
            if (!bodies.Handles.IsAlive(t.Body, t.BodyGen) || bodies.TypeOf(t.Body) != BodyType.Dynamic || HoverOfBody(t.Body) != i) { return false; }
        }

        for (int s = 0; s < Surfaces.Length; s++)
        {
            if (Surfaces[s] == null) { continue; }
            if (s >= core.Shapes.HighWater || !core.Shapes.Handles.IsAlive(s, Surfaces[s]!.ShapeGen)) { return false; }
        }

        return true;
    }
}
