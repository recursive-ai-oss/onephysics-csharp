using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The state of one wheel after the last step (<see cref="Vehicle.GetWheel"/>).</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct WheelState
{
    internal WheelState(bool contact, Vector2D point, Vector2F normal, Body ground, float length, float springForce, float omega, float spin, float slipRatio, float fx, Vector2D center, float radius)
    {
        IsInContact = contact; ContactPoint = point; ContactNormal = normal; GroundBody = ground; SuspensionLength = length; SuspensionForce = springForce; AngularVelocity = omega;
        SpinAngle = spin; SlipRatio = slipRatio; LongitudinalForce = fx; Center = center; Radius = radius;
    }

    /// <summary>True when the wheel's sweep found ground within the suspension's travel.</summary>
    public bool IsInContact { get; }

    /// <summary>Where the tyre touches the ground, world coordinates; zero when not in contact.</summary>
    public Vector2D ContactPoint { get; }

    /// <summary>Unit normal of the ground at the contact; zero when not in contact.</summary>
    public Vector2F ContactNormal { get; }

    /// <summary>The body under the wheel (it may be static); the default (invalid) body when not in contact.</summary>
    public Body GroundBody { get; }

    /// <summary>Suspension length, m: the distance from the top of the suspension to the wheel centre. The maximum length when the wheel hangs free.</summary>
    public float SuspensionLength { get; }

    /// <summary>Load the suspension carried in the last step along the contact normal, N; 0 when not in contact.</summary>
    public float SuspensionForce { get; }

    /// <summary>Spin speed, rad/s; positive when the wheel rolls the vehicle forward.</summary>
    public float AngularVelocity { get; }

    /// <summary>Accumulated spin angle, radians within -π..π, for drawing the wheel; positive rolls the vehicle forward.</summary>
    public float SpinAngle { get; }

    /// <summary>Slip ratio: (wheel surface speed − ground speed) over the larger of the ground speed and the tyre's low-speed floor. Positive when the wheel spins faster than it rolls.</summary>
    public float SlipRatio { get; }

    /// <summary>Tyre force along the rolling direction on the chassis, N.</summary>
    public float LongitudinalForce { get; }

    /// <summary>Wheel centre in world coordinates.</summary>
    public Vector2D Center { get; }

    /// <summary>Wheel radius, m.</summary>
    public float Radius { get; }
}

/// <summary>
/// A wheeled vehicle on a dynamic chassis body, in side view. Every wheel sweeps a circle along its suspension to find the ground (against any shape); a spring and damper hold the chassis up, a tyre curve turns slip into
/// force and an engine, clutch, gearbox and differentials turn the throttle into wheel torque. Like <see cref="Body"/> it is a small struct handle (world, slot, generation) that allocates nothing and fails with a
/// <see cref="StaleHandleException"/> once the vehicle is destroyed (its chassis body destroyed or <see cref="Destroy"/> called); the state lives in the world and in its snapshots.
/// </summary>
/// <example><c>var car = world.AddCar(new Vector2D(0, 1)); car.Drive(throttle: 1f); world.Step(1f / 60f);</c></example>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct Vehicle : IEquatable<Vehicle>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Vehicle(PhysicsWorld2D world, int index, int generation)
    {
        _world = world;
        _index = index;
        _generation = generation;
    }

    private VehicleState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Vehicle is not attached to a world."); }
        var e = _world.Engine;
        if (!e.VehHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Vehicle(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the vehicle was destroyed.");
        }

        return e.Vehicles[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld2D.GetVehicle"/> turns it back into the vehicle.</summary>
    public VehicleHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the vehicle exists.</summary>
    public bool IsValid => _world != null && _world.Engine.VehHandles.IsAlive(_index, _generation);

    /// <summary>The chassis body.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public Body Chassis { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Body, s.BodyGen)); } }

    /// <summary>Number of wheels.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public int WheelCount => Resolve().Wheels;

    /// <summary>Which controller drives this vehicle.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public VehicleKind Kind => Resolve().Kind;

    /// <summary>
    /// Sets the driver's inputs for the next steps (they stay until the next call).
    /// </summary>
    /// <param name="throttle">-1..1: positive drives forward, negative reverses (an automatic box brakes first while the vehicle still rolls forward; a manual one uses the gear you set and brakes when the sign opposes it).</param>
    /// <param name="pitch">-1..1: positive lifts the nose (air control, a motorcycle rider's lean, weight shift on the ground where the definition gives a ground torque); negative lowers it. There is no steering in a side view.</param>
    /// <param name="brake">0..1 service brake.</param>
    /// <param name="handbrake">0..1 handbrake, on the wheels that have a <see cref="WheelDef.MaxHandbrakeTorque"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public void Drive(float throttle, float pitch = 0f, float brake = 0f, float handbrake = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Vehicle.Drive");
        var s = Resolve();
        if (!(throttle >= -1f && throttle <= 1f)) { throw new ArgumentOutOfRangeException(nameof(throttle), throttle, "The throttle must be within -1..1."); }
        if (!(pitch >= -1f && pitch <= 1f)) { throw new ArgumentOutOfRangeException(nameof(pitch), pitch, "The pitch input must be within -1..1."); }
        if (!(brake >= 0f && brake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(brake), brake, "The brake must be within 0..1."); }
        if (!(handbrake >= 0f && handbrake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(handbrake), handbrake, "The handbrake must be within 0..1."); }
        s.Forward = throttle; s.Pitch = pitch; s.Brake = brake; s.Handbrake = handbrake;
    }

    /// <summary>Sets the driver's inputs for any kind of vehicle; the handbrake is released. The same as <see cref="Drive"/> without the handbrake.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public void SetInput(float throttle, float pitch = 0f, float brake = 0f) { using var replayMutation = _world?.RecordMutation("Vehicle.SetInput"); Drive(throttle, pitch, brake, 0f); }

    /// <summary>The throttle of the last <see cref="Drive"/>.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float Throttle => Resolve().Forward;

    /// <summary>The pitch input of the last <see cref="Drive"/>.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float Pitch => Resolve().Pitch;

    /// <summary>Current gear: -1 reverse, 0 neutral, 1.. forward. Setting it shifts (the clutch is open for the box's switch time); on an automatic box the box overrides it again.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The gear is outside -1..number of gears.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public int Gear
    {
        get => Resolve().Gear;
        set
        {
            using var replayMutation = _world?.RecordMutation("Vehicle.Gear");
            var s = Resolve();
            if (value < -1 || value > s.Ratios.Length) { throw new ArgumentOutOfRangeException(nameof(value), value, "The gear must be within -1.." + s.Ratios.Length.ToString(CultureInfo.InvariantCulture) + "."); }
            if (s.Gear != value) { s.Gear = value; s.ShiftTimer = s.Def.Transmission.SwitchTime; }
        }
    }

    /// <summary>Engine speed in revolutions per minute.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float EngineRpm => Resolve().EngineW * (60f / 6.2831855f);

    /// <summary>Speed along the vehicle's forward direction (the first wheel's), m/s; negative when rolling backwards.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float ForwardSpeed
    {
        get
        {
            var s = Resolve();
            var rot = _world!.Core.Bodies.RotationOf(s.Body);
            var f = s.Def.Wheels[0].ForwardDirection;
            float fx = rot[0] * f.X - rot[1] * f.Y, fy = rot[1] * f.X + rot[0] * f.Y;
            var v = _world.Core.Bodies.LinearVelocityOf(s.Body);
            return v[0] * fx + v[1] * fy;
        }
    }

    /// <summary>Belt speed of a tracked vehicle as of the last step, m/s; positive moves the hull forward.</summary>
    /// <exception cref="InvalidOperationException">The vehicle is not tracked.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float TrackSpeed
    {
        get
        {
            var s = Resolve();
            if (s.Kind != VehicleKind.Tracked) { throw new InvalidOperationException("Only a tracked vehicle has a track."); }
            return Engine2D.TrackSpeedOf(s);
        }
    }

    /// <summary>The state of wheel <paramref name="index"/> as of the last step.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..<see cref="WheelCount"/>-1.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public WheelState GetWheel(int index)
    {
        var s = Resolve();
        if ((uint)index >= (uint)s.Wheels) { throw new ArgumentOutOfRangeException(nameof(index), index, "The wheel index must be within 0.." + (s.Wheels - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        var w = _world!;
        var e = w.Engine;
        var bodies = w.Core.Bodies;
        var wd = s.Def.Wheels[index];
        var rot = bodies.RotationOf(s.Body);
        float qc = rot[0], qs = rot[1];
        float tx = wd.Position.X - e.LocalCenter[2 * s.Body], ty = wd.Position.Y - e.LocalCenter[2 * s.Body + 1];
        var pp = bodies.PositionOf(s.Body);
        float dx = qc * wd.SuspensionDirection.X - qs * wd.SuspensionDirection.Y, dy = qs * wd.SuspensionDirection.X + qc * wd.SuspensionDirection.Y;
        float len = s.Length[index];
        var center = new Vector2D(pp[0] + (double)(qc * tx - qs * ty) + (double)(dx * len), pp[1] + (double)(qs * tx + qc * ty) + (double)(dy * len));
        bool contact = s.Contact[index] != 0;
        Body ground = default;
        if (contact && bodies.Handles.IsAlive(s.GroundBody[index], s.GroundGen[index])) { ground = new Body(w, new BodyHandle(w.Id, s.GroundBody[index], s.GroundGen[index])); }
        return new WheelState(contact, contact ? new Vector2D(s.Cx[index], s.Cy[index]) : default, contact ? new Vector2F(s.Nx[index], s.Ny[index]) : default, ground, len, s.SpringForce[index],
            s.Omega[index], s.Spin[index], s.SlipRatio[index], s.Fx[index], center, wd.Radius);
    }

    /// <summary>Removes the vehicle; the chassis body stays. Handles to it become stale.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("Vehicle.Destroy");
        Resolve();
        _world!.Engine.DestroyVehicle(_index);
    }

    /// <inheritdoc/>
    public bool Equals(Vehicle other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Vehicle v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(_index * 397 + _generation);

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Vehicle a, Vehicle b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Vehicle a, Vehicle b) => !a.Equals(b);
}

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Makes <paramref name="chassis"/> a wheeled vehicle. The chassis keeps its own shapes for collisions (the wheels are not shapes: they sweep the world for ground); give it a mass for the springs to carry.
    /// Vehicles run once per step, before the solver, in creation order.
    /// </summary>
    /// <param name="chassis">A dynamic body of this world that carries no vehicle yet.</param>
    /// <param name="def">The vehicle, for example <see cref="VehicleDef.Car"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="PhysicsException">The body already carries a vehicle.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddVehicle(Body chassis, VehicleDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddVehicle");
        ArgumentNullException.ThrowIfNull(def);
        if (chassis.World != this) { throw new ArgumentException("The chassis belongs to another world.", nameof(chassis)); }
        int v = _engine.CreateVehicle(chassis.Handle, def);
        return new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
    }

    /// <summary>Makes <paramref name="chassis"/> a tracked vehicle. Same rules as <see cref="AddVehicle(Body, VehicleDef)"/>.</summary>
    /// <param name="chassis">A dynamic body of this world that carries no vehicle yet.</param>
    /// <param name="def">The track, for example <see cref="TrackedVehicleDef.Tank"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="PhysicsException">The body already carries a vehicle.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddVehicle(Body chassis, TrackedVehicleDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddVehicle");
        ArgumentNullException.ThrowIfNull(def);
        if (chassis.World != this) { throw new ArgumentException("The chassis belongs to another world.", nameof(chassis)); }
        int v = _engine.CreateVehicle(chassis.Handle, def.ToWheeled(), track: def);
        return new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
    }

    /// <summary>Makes <paramref name="chassis"/> a motorcycle (front wheel is wheel 0, rear wheel 1). Same rules as <see cref="AddVehicle(Body, VehicleDef)"/>.</summary>
    /// <param name="chassis">A dynamic body of this world that carries no vehicle yet.</param>
    /// <param name="def">The motorcycle, for example <see cref="MotorcycleDef.Road"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="PhysicsException">The body already carries a vehicle.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddVehicle(Body chassis, MotorcycleDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddVehicle");
        ArgumentNullException.ThrowIfNull(def);
        if (chassis.World != this) { throw new ArgumentException("The chassis belongs to another world.", nameof(chassis)); }
        int v = _engine.CreateVehicle(chassis.Handle, def.ToWheeled(), bike: def);
        return new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
    }

    /// <summary>
    /// Happy path: a vehicle resting on the ground at <paramref name="ground"/> (the ground point under the chassis centre), facing +X, with a chassis box of <paramref name="chassisSize"/> and <paramref name="chassisMass"/>.
    /// <c>var car = world.AddVehicle(ground, VehicleDef.Car, new Vector2F(3.2f, 0.8f), 1500f);</c> The presets <see cref="AddCar"/>, <see cref="AddTruck"/> and <see cref="AddKart"/> choose the chassis for you.
    /// </summary>
    /// <param name="ground">Ground point under the chassis centre.</param>
    /// <param name="def">The vehicle.</param>
    /// <param name="chassisSize">Length and height of the chassis box, m; positive.</param>
    /// <param name="chassisMass">Mass of the chassis, kg; positive.</param>
    /// <param name="angleRadians">Orientation of the chassis. Default 0 (upright, facing +X).</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddVehicle(Vector2D ground, VehicleDef def, Vector2F chassisSize, float chassisMass, float angleRadians = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddVehicle");
        ArgumentNullException.ThrowIfNull(def);
        def.Validate();
        var body = AddChassis(ground, def.Wheels, chassisSize, chassisMass, angleRadians);
        try { return AddVehicle(body, def); }
        catch (Exception) { body.Destroy(); throw; }
    }

    /// <summary>Happy path: <see cref="VehicleDef.Car"/> on a 3.2 × 0.8 m, 1500 kg chassis resting on the ground at <paramref name="ground"/>.</summary>
    /// <param name="ground">Ground point under the chassis centre.</param>
    /// <param name="angleRadians">Orientation of the chassis.</param>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddCar(Vector2D ground, float angleRadians = 0f) => AddVehicle(ground, VehicleDef.Car, new Vector2F(3.2f, 0.8f), 1500f, angleRadians);

    /// <summary>Happy path: <see cref="VehicleDef.Truck"/> on a 6 × 1.4 m, 8000 kg chassis resting on the ground at <paramref name="ground"/>.</summary>
    /// <param name="ground">Ground point under the chassis centre.</param>
    /// <param name="angleRadians">Orientation of the chassis.</param>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddTruck(Vector2D ground, float angleRadians = 0f) => AddVehicle(ground, VehicleDef.Truck, new Vector2F(6f, 1.4f), 8000f, angleRadians);

    /// <summary>Happy path: <see cref="VehicleDef.Kart"/> on a 1.4 × 0.2 m, 150 kg chassis resting on the ground at <paramref name="ground"/>.</summary>
    /// <param name="ground">Ground point under the chassis centre.</param>
    /// <param name="angleRadians">Orientation of the chassis.</param>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddKart(Vector2D ground, float angleRadians = 0f) => AddVehicle(ground, VehicleDef.Kart, new Vector2F(1.4f, 0.2f), 150f, angleRadians);

    /// <summary>
    /// Happy path: a tank resting on the ground at <paramref name="ground"/> (the ground point under the hull's centre), facing +X, built from a hull box of the preset's size and mass.
    /// <c>var tank = world.AddTank(at); tank.SetInput(throttle: 1f);</c>
    /// </summary>
    /// <param name="ground">Ground point under the hull centre.</param>
    /// <param name="def">The track; null (default) is <see cref="TrackedVehicleDef.Tank"/>.</param>
    /// <param name="angleRadians">Orientation of the hull.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddTank(Vector2D ground, TrackedVehicleDef? def = null, float angleRadians = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddTank");
        def ??= TrackedVehicleDef.Tank;
        def.Validate();
        var body = AddChassis(ground, def.Wheels, def.ChassisSize, def.ChassisMass, angleRadians);
        try { return AddVehicle(body, def); }
        catch (Exception) { body.Destroy(); throw; }
    }

    /// <summary>
    /// Happy path: a motorcycle resting on its wheels at <paramref name="ground"/> (the ground point under the chassis centre), facing +X, built from a chassis box of the preset's size and mass.
    /// <c>var bike = world.AddMotorcycle(at); bike.SetInput(throttle: 0.5f);</c>
    /// </summary>
    /// <param name="ground">Ground point under the chassis centre.</param>
    /// <param name="def">The motorcycle; null (default) is <see cref="MotorcycleDef.Road"/>.</param>
    /// <param name="angleRadians">Orientation of the chassis.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle AddMotorcycle(Vector2D ground, MotorcycleDef? def = null, float angleRadians = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddMotorcycle");
        def ??= MotorcycleDef.Road;
        def.Validate();
        var body = AddChassis(ground, new[] { def.Front, def.Rear }, def.ChassisSize, def.ChassisMass, angleRadians);
        try { return AddVehicle(body, def); }
        catch (Exception) { body.Destroy(); throw; }
    }

    // The chassis box, placed so the lowest wheel's tyre is just above the ground point (its suspension a little short of fully extended), for an upright chassis.
    private Body AddChassis(Vector2D ground, WheelDef[] wheels, Vector2F size, float mass, float angle)
    {
        VehicleCheck2D.Vec(size, nameof(size));
        if (!(size.X > 0f && size.Y > 0f)) { throw new ArgumentOutOfRangeException(nameof(size), size, "The chassis size must be positive."); }
        VehicleCheck2D.Positive(mass, nameof(mass));
        float drop = 0f;
        foreach (var w in wheels) { drop = Math.Max(drop, w.Radius + w.SuspensionMaxLength - 0.05f - w.Position.Y); }
        return AddBox(size, new Vector2D(ground.X, ground.Y + drop), angle, density: mass / (size.X * size.Y));
    }

    /// <summary>The <see cref="Vehicle"/> for a raw handle (from <see cref="Vehicle.RawHandle"/>).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world (<see cref="ForeignHandleException"/>) or refers to a removed vehicle (<see cref="StaleHandleException"/>).</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle GetVehicle(VehicleHandle handle)
    {
        if (handle.IsNull) { throw new InvalidHandleException("The null vehicle handle was used."); }
        if (handle.World != Id) { throw new ForeignHandleException(handle + " belongs to world " + handle.World.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!_engine.VehHandles.IsAlive(handle.Index, handle.Generation)) { throw new StaleHandleException(handle + " is stale: the vehicle was removed."); }
        return new Vehicle(this, handle.Index, handle.Generation);
    }

    /// <summary>True while the vehicle exists in this world. False for null handles, handles of another world and removed vehicles (never throws). Allocates nothing.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public bool IsVehicleAlive(VehicleHandle handle) => !handle.IsNull && handle.World == Id && _engine.VehHandles.IsAlive(handle.Index, handle.Generation);

    /// <summary>The vehicle whose chassis is <paramref name="body"/>, if there is one.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public bool TryGetVehicle(Body body, out Vehicle vehicle)
    {
        vehicle = default;
        if (!body.IsValid || body.World != this) { return false; }
        int v = _engine.VehicleOfBody(body.Handle.Index);
        if (v < 0) { return false; }
        vehicle = new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
        return true;
    }

    /// <summary>All live vehicles in slot order (allocates the array).</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Vehicle[] GetVehicles()
    {
        var list = new List<Vehicle>();
        for (int v = 0; v < _engine.VehHandles.HighWater; v++)
        {
            if (_engine.VehHandles.IsLive(v)) { list.Add(new Vehicle(this, v, _engine.VehHandles.GenerationOf(v))); }
        }

        return list.ToArray();
    }
}
