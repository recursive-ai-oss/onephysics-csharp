using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A thruster on a body: throttle it, gimbal it, read the thrust it made. Like <see cref="Body"/> it is a small struct handle (world, slot, generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the thruster or its body
/// is destroyed. Thrusters run once per step before the solver, in creation order, and add their force to the body (a sleeping body is woken by a non-zero throttle).
/// </summary>
/// <example><c>var rocket = body.AddThruster(ThrusterDef.Rocket(2000f)); rocket.Throttle = 1f; world.Step(1f / 60f);</c></example>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct Thruster : IEquatable<Thruster>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index, _generation;

    internal Thruster(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private ThrusterState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Thruster is not attached to a world."); }
        var s = _world.Engine.Actuators;
        if (!s.ThrusterHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Thruster(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the thruster or its body was destroyed.");
        }

        return s.Thrusters[_index]!;
    }

    /// <summary>True while the thruster exists.</summary>
    public bool IsValid => _world != null && _world.Engine.Actuators.ThrusterHandles.IsAlive(_index, _generation);

    /// <summary>The body it pushes.</summary>
    public Body Body { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Body, s.BodyGen)); } }

    /// <summary>The throttle: 1 is full thrust, 0 off, negative reverse (down to −<c>ReverseFraction</c> of full; values beyond are clamped when applied). Finite, within ±1e6.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting NaN, infinity or a value beyond ±1e6.</exception>
    public float Throttle
    {
        get => Resolve().Throttle;
        set { using var replayMutation = _world?.RecordMutation("Thruster.Throttle"); var s = Resolve(); CheckInput(value, nameof(value)); s.Throttle = value; }
    }

    /// <summary>The gimbal command as fractions of the maximum gimbal angle: X turns the thrust about the gimbal axis, Y about the axis perpendicular to it; each is clamped to ±1 when applied.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite component.</exception>
    public Vector2F Gimbal
    {
        get { var s = Resolve(); return new Vector2F(s.Gimbal1, s.Gimbal2); }
        set { using var replayMutation = _world?.RecordMutation("Thruster.Gimbal"); var s = Resolve(); CheckInput(value.X, nameof(value)); CheckInput(value.Y, nameof(value)); s.Gimbal1 = value.X; s.Gimbal2 = value.Y; }
    }

    /// <summary>The thrust fraction the response has reached, −1 to 1.</summary>
    public float Output => Resolve().Output;

    /// <summary>The force the thruster made in the last step, world axes, newtons (zero before the first step).</summary>
    public Vector3F Force => Resolve().LastForce;

    /// <summary>Thrust at full throttle, newtons, 0 to 10^9.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside that range.</exception>
    public float MaxThrust
    {
        get => Resolve().Spec.MaxThrust;
        set { using var replayMutation = _world?.RecordMutation("Thruster.MaxThrust"); var s = Resolve(); V3.CheckRange(value, 0f, 1e9f, nameof(value)); s.Spec.MaxThrust = value; }
    }

    /// <summary>Removes the thruster.</summary>
    /// <exception cref="StaleHandleException">It was already removed.</exception>
    public void Destroy() { using var replayMutation = _world?.RecordMutation("Thruster.Destroy"); Resolve(); _world!.Engine.Actuators.DestroyThruster(_index); }

    internal static void CheckInput(float v, string name)
    {
        if (!(Math.Abs(v) <= 1e6f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and within ±1e6."); }
    }

    /// <inheritdoc/>
    public bool Equals(Thruster other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Thruster t && Equals(t);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Thruster a, Thruster b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Thruster a, Thruster b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _world == null ? "Thruster(default)" : "Thruster(#" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}

/// <summary>
/// A boat on a dynamic body (see <see cref="BoatDef"/>): set <see cref="Throttle"/> and <see cref="Steer"/>, step the world. A struct handle like <see cref="Body"/>; it fails with a <see cref="StaleHandleException"/> once the boat or its body is destroyed.
/// </summary>
/// <example><c>var boat = world.AddBoat(hull, BoatDef.Dinghy); boat.Throttle = 1f; boat.Steer = 0.3f; world.Step(1f / 60f);</c></example>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct Boat : IEquatable<Boat>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index, _generation;

    internal Boat(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private BoatState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Boat is not attached to a world."); }
        var s = _world.Engine.Actuators;
        if (!s.BoatHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Boat(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the boat or its body was destroyed.");
        }

        return s.Boats[_index]!;
    }

    /// <summary>True while the boat exists.</summary>
    public bool IsValid => _world != null && _world.Engine.Actuators.BoatHandles.IsAlive(_index, _generation);

    /// <summary>The hull body.</summary>
    public Body Hull { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Body, s.BodyGen)); } }

    /// <summary>The propeller throttle, −1 (full reverse, limited by each propeller's reverse fraction) to 1. Default 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite value or one outside −1..1.</exception>
    public float Throttle
    {
        get => Resolve().Throttle;
        set { using var replayMutation = _world?.RecordMutation("Boat.Throttle"); var s = Resolve(); Check(value, nameof(value)); s.Throttle = value; }
    }

    /// <summary>The steering command, −1 to 1: positive turns to the right (clockwise seen from above), through the rudder and the propellers' gimbals. Default 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite value or one outside −1..1.</exception>
    public float Steer
    {
        get => Resolve().Steer;
        set { using var replayMutation = _world?.RecordMutation("Boat.Steer"); var s = Resolve(); Check(value, nameof(value)); s.Steer = value; }
    }

    /// <summary>The rudder angle the rudder has reached, radians (it moves at the definition's rate); positive is to the right.</summary>
    public float RudderAngle => Resolve().Rudder;

    /// <summary>Speed along the boat's forward axis, m/s (negative when going astern).</summary>
    public float ForwardSpeed
    {
        get
        {
            var s = Resolve();
            var e = _world!.Engine;
            var f = e.RotationOfBody(s.Body).Rotate(s.Spec.Forward);
            var v = _world.Core.Bodies.LinearVelocityOf(s.Body);
            return v[0] * f.X + v[1] * f.Y + v[2] * f.Z;
        }
    }

    /// <summary>The hull drag of the last step, world axes, newtons.</summary>
    public Vector3F HullForce => Resolve().LastHullForce;

    internal static void Check(float v, string name)
    {
        if (!(v >= -1f && v <= 1f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be within −1..1."); }
    }

    /// <summary>Removes the boat component (the body and its buoyancy stay).</summary>
    /// <exception cref="StaleHandleException">It was already removed.</exception>
    public void Destroy() { using var replayMutation = _world?.RecordMutation("Boat.Destroy"); Resolve(); _world!.Engine.Actuators.DestroyBoat(_index); }

    /// <inheritdoc/>
    public bool Equals(Boat other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Boat t && Equals(t);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Boat a, Boat b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Boat a, Boat b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _world == null ? "Boat(default)" : "Boat(#" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}

/// <summary>
/// A hovercraft on a dynamic body (see <see cref="HovercraftDef"/>): set <see cref="Lift"/>, <see cref="Throttle"/> and <see cref="Steer"/>, step the world. A struct handle like <see cref="Body"/>; it fails with a <see cref="StaleHandleException"/> once the
/// hovercraft or its body is destroyed.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct Hovercraft : IEquatable<Hovercraft>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index, _generation;

    internal Hovercraft(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private HoverState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Hovercraft is not attached to a world."); }
        var s = _world.Engine.Actuators;
        if (!s.HoverHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Hovercraft(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the hovercraft or its body was destroyed.");
        }

        return s.Hovers[_index]!;
    }

    /// <summary>True while the hovercraft exists.</summary>
    public bool IsValid => _world != null && _world.Engine.Actuators.HoverHandles.IsAlive(_index, _generation);

    /// <summary>The craft's body.</summary>
    public Body Hull { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Body, s.BodyGen)); } }

    /// <summary>Fan power, 0 (off) to 1. Default 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite value or one outside 0..1.</exception>
    public float Lift
    {
        get => Resolve().Lift;
        set { using var replayMutation = _world?.RecordMutation("Hovercraft.Lift"); var s = Resolve(); if (!(value >= 0f && value <= 1f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "Lift must be within 0..1."); } s.Lift = value; }
    }

    /// <summary>The propulsion throttle, −1 to 1 (reverse where the thrusters allow it). Default 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite value or one outside −1..1.</exception>
    public float Throttle
    {
        get => Resolve().Throttle;
        set { using var replayMutation = _world?.RecordMutation("Hovercraft.Throttle"); var s = Resolve(); Boat.Check(value, nameof(value)); s.Throttle = value; }
    }

    /// <summary>The steering command, −1 to 1: positive turns to the right, by deflecting the thrusters. Default 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite value or one outside −1..1.</exception>
    public float Steer
    {
        get => Resolve().Steer;
        set { using var replayMutation = _world?.RecordMutation("Hovercraft.Steer"); var s = Resolve(); Boat.Check(value, nameof(value)); s.Steer = value; }
    }

    /// <summary>Number of lift fans.</summary>
    public int FanCount => Resolve().Spec.Fans.Length;

    /// <summary>The gap under fan <paramref name="index"/> along its axis as of the last step, metres; the fan's range when nothing is in reach.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..FanCount−1.</exception>
    public float GetFanGap(int index) { var s = Resolve(); if ((uint)index >= (uint)s.FanHeight.Length) { throw new ArgumentOutOfRangeException(nameof(index), index, "The fan index must be within 0.." + (s.FanHeight.Length - 1).ToString(CultureInfo.InvariantCulture) + "."); } return s.FanHeight[index]; }

    /// <summary>The push of fan <paramref name="index"/> in the last step, newtons.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..FanCount−1.</exception>
    public float GetFanForce(int index) { var s = Resolve(); if ((uint)index >= (uint)s.FanForce.Length) { throw new ArgumentOutOfRangeException(nameof(index), index, "The fan index must be within 0.." + (s.FanForce.Length - 1).ToString(CultureInfo.InvariantCulture) + "."); } return s.FanForce[index]; }

    /// <summary>Removes the hovercraft component.</summary>
    /// <exception cref="StaleHandleException">It was already removed.</exception>
    public void Destroy() { using var replayMutation = _world?.RecordMutation("Hovercraft.Destroy"); Resolve(); _world!.Engine.Actuators.DestroyHover(_index); }

    /// <inheritdoc/>
    public bool Equals(Hovercraft other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Hovercraft t && Equals(t);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Hovercraft a, Hovercraft b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Hovercraft a, Hovercraft b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _world == null ? "Hovercraft(default)" : "Hovercraft(#" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}

public sealed partial class PhysicsWorld3D
{
    /// <summary>Density of the air for aerodynamic surfaces and air thrusters, kg/m³, 0 to 10^4. Default 1.225 (sea level). Setting it wakes nothing; sleeping bodies feel no aerodynamic force.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a non-finite value or one outside the range.</exception>
    public float AirDensity
    {
        get => _engine.Actuators.AirDensity;
        set
        {
            if (!(value >= 0f && value <= 1e4f)) { throw new ArgumentOutOfRangeException(nameof(value), value, "AirDensity must be within 0..10^4 kg/m³."); }
            using var replayMutation = RecordMutation("PhysicsWorld3D.AirDensity");
            _engine.Actuators.AirDensity = value;
        }
    }

    /// <summary>Adds wind: an area field whose flow is <paramref name="velocity"/> (m/s) everywhere or inside the box. It moves nothing by itself; aerodynamic surfaces, bluff bodies and air thrusters see the air moving by it. Winds add where they overlap.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, or the box is not ordered.</exception>
    public Field AddWind(Vector3F velocity, Vector3D? minimum = null, Vector3D? maximum = null)
    {
        var d = new FieldDefinition { Flow = velocity };
        if (minimum.HasValue) { d.Minimum = minimum.Value; }
        if (maximum.HasValue) { d.Maximum = maximum.Value; }
        return AddField(d);
    }

    /// <summary>Adds a thruster to a dynamic body. Thrusters run once per step before the solver, in creation order.</summary>
    /// <param name="body">A dynamic body of this world.</param>
    /// <param name="def">The thruster; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public Thruster AddThruster(Body body, ThrusterDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddThruster");
        ArgumentNullException.ThrowIfNull(def);
        if (body.World != this) { throw new ArgumentException("The body belongs to another world.", nameof(body)); }
        int i = _engine.Actuators.CreateThruster(body.Handle, def.ToSpec());
        return new Thruster(this, i, _engine.Actuators.ThrusterHandles.GenerationOf(i));
    }

    /// <summary>Makes a dynamic body a boat. The body should float: put it in water made by <see cref="AddWater"/> or a buoyancy effector. Boats run once per step before the solver, in creation order.</summary>
    /// <param name="hull">A dynamic body of this world that is not a boat yet.</param>
    /// <param name="def">The boat, for example <see cref="BoatDef.Dinghy"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="PhysicsException">The body is a boat already.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public Boat AddBoat(Body hull, BoatDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddBoat");
        ArgumentNullException.ThrowIfNull(def);
        if (hull.World != this) { throw new ArgumentException("The hull belongs to another world.", nameof(hull)); }
        int i = _engine.Actuators.CreateBoat(hull.Handle, def.ToSpec());
        return new Boat(this, i, _engine.Actuators.BoatHandles.GenerationOf(i));
    }

    /// <summary>Makes a dynamic body a hovercraft. Hovercraft run once per step before the solver, in creation order.</summary>
    /// <param name="craft">A dynamic body of this world that is not a hovercraft yet.</param>
    /// <param name="def">The hovercraft, for example <see cref="HovercraftDef.Small"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world, or the definition has no fans.</exception>
    /// <exception cref="PhysicsException">The body is a hovercraft already.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public Hovercraft AddHovercraft(Body craft, HovercraftDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddHovercraft");
        ArgumentNullException.ThrowIfNull(def);
        if (craft.World != this) { throw new ArgumentException("The craft belongs to another world.", nameof(craft)); }
        int i = _engine.Actuators.CreateHover(craft.Handle, def.ToSpec());
        return new Hovercraft(this, i, _engine.Actuators.HoverHandles.GenerationOf(i));
    }

    /// <summary>The boat whose hull is <paramref name="body"/>, if there is one.</summary>
    public bool TryGetBoat(Body body, out Boat boat)
    {
        boat = default;
        if (!body.IsValid || body.World != this) { return false; }
        int i = _engine.Actuators.BoatOfBody(body.Handle.Index);
        if (i < 0) { return false; }
        boat = new Boat(this, i, _engine.Actuators.BoatHandles.GenerationOf(i));
        return true;
    }

    /// <summary>The hovercraft whose body is <paramref name="body"/>, if there is one.</summary>
    public bool TryGetHovercraft(Body body, out Hovercraft craft)
    {
        craft = default;
        if (!body.IsValid || body.World != this) { return false; }
        int i = _engine.Actuators.HoverOfBody(body.Handle.Index);
        if (i < 0) { return false; }
        craft = new Hovercraft(this, i, _engine.Actuators.HoverHandles.GenerationOf(i));
        return true;
    }

    /// <summary>All live thrusters of the world in slot order (allocates the array).</summary>
    public Thruster[] GetThrusters()
    {
        var a = _engine.Actuators;
        var list = new List<Thruster>();
        for (int i = 0; i < a.ThrusterHandles.HighWater; i++) { if (a.ThrusterHandles.IsLive(i)) { list.Add(new Thruster(this, i, a.ThrusterHandles.GenerationOf(i))); } }
        return list.ToArray();
    }
}

public readonly partial struct Body
{
    /// <summary>Adds a thruster to this body (see <see cref="PhysicsWorld3D.AddThruster"/>).</summary>
    public Thruster AddThruster(ThrusterDef def) { var (w, _) = Resolve(); return w.AddThruster(this, def); }
}

public readonly partial struct Shape
{
    /// <summary>The shape's aerodynamic surface (a copy), or null for none. Setting a surface (or null) changes it at once; setting is checked like <see cref="ShapeDef.Aero"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a surface with a value out of range.</exception>
    public AeroSurface? Aero
    {
        get { int s = Resolve(); var st = _world!.Engine.Actuators.Surfaces; return s < st.Length && st[s] != null ? AeroSurface.FromSpec(st[s]!.Spec) : null; }
        set { using var replayMutation = _world?.RecordMutation("Shape.Aero"); Resolve(); _world!.Engine.Actuators.SetSurface(_handle, value?.ToSpec()); }
    }
}
