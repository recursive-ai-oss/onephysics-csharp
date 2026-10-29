using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The state of one wheel after the last step (<see cref="Vehicle.GetWheel"/>).</summary>
public readonly struct WheelState
{
    internal WheelState(bool contact, Vector3D point, Vector3F normal, Body ground, float length, float springForce, float omega, float spin, float steer, float slipRatio, float slipAngle, float fx, float fy, Vector3D center, float radius)
    {
        IsInContact = contact; ContactPoint = point; ContactNormal = normal; GroundBody = ground; SuspensionLength = length; SuspensionForce = springForce; AngularVelocity = omega;
        SpinAngle = spin; SteerAngle = steer; SlipRatio = slipRatio; SlipAngle = slipAngle; LongitudinalForce = fx; LateralForce = fy; Center = center; Radius = radius;
    }

    /// <summary>True when the wheel's sweep found ground within the suspension's travel.</summary>
    public bool IsInContact { get; }

    /// <summary>Where the tyre touches the ground, world coordinates; zero when not in contact.</summary>
    public Vector3D ContactPoint { get; }

    /// <summary>Unit normal of the ground at the contact; zero when not in contact.</summary>
    public Vector3F ContactNormal { get; }

    /// <summary>The body under the wheel (it may be static); the default (invalid) body when not in contact.</summary>
    public Body GroundBody { get; }

    /// <summary>Suspension length, m: the distance from the top of the suspension to the wheel centre. The maximum length when the wheel hangs free.</summary>
    public float SuspensionLength { get; }

    /// <summary>Load the suspension carried in the last step along the contact normal, N; 0 when not in contact.</summary>
    public float SuspensionForce { get; }

    /// <summary>Spin speed, rad/s; positive when the wheel rolls the vehicle forward.</summary>
    public float AngularVelocity { get; }

    /// <summary>Accumulated spin angle about the axle, radians within -π..π, for drawing the wheel.</summary>
    public float SpinAngle { get; }

    /// <summary>Steering angle of this wheel, radians.</summary>
    public float SteerAngle { get; }

    /// <summary>Longitudinal slip ratio: (wheel surface speed − ground speed) over the larger of the ground speed and the tyre's low-speed floor. Positive when the wheel spins faster than it rolls.</summary>
    public float SlipRatio { get; }

    /// <summary>Slip angle, radians: atan of the sideways ground speed over the same reference speed; positive when the tyre slides towards the wheel's lateral direction.</summary>
    public float SlipAngle { get; }

    /// <summary>Tyre force along the rolling direction on the chassis, N.</summary>
    public float LongitudinalForce { get; }

    /// <summary>Tyre force sideways on the chassis, N.</summary>
    public float LateralForce { get; }

    /// <summary>Wheel centre in world coordinates.</summary>
    public Vector3D Center { get; }

    /// <summary>Wheel radius, m.</summary>
    public float Radius { get; }
}

/// <summary>
/// A wheeled vehicle on a dynamic chassis body. Every wheel sweeps a cylinder (or a sphere) along its suspension to find the ground; a spring and damper hold the chassis up, a tyre curve turns slip into force and an engine, clutch, gearbox and
/// differentials turn the throttle into wheel torque. Like <see cref="Body"/> it is a small struct handle (world, slot, generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the vehicle is destroyed
/// (its chassis body destroyed or <see cref="Destroy"/> called); the state lives in the world and in its snapshots.
/// </summary>
/// <example><c>var car = world.AddVehicle(chassis, VehicleDef.Car); car.Drive(throttle: 1f, steer: 0.2f); world.Step(1f / 60f);</c></example>
public readonly partial struct Vehicle : IEquatable<Vehicle>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Vehicle(PhysicsWorld3D world, int index, int generation)
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

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld3D.GetVehicle"/> turns it back into the vehicle.</summary>
    public VehicleHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the vehicle exists.</summary>
    public bool IsValid => _world != null && _world.Engine.VehHandles.IsAlive(_index, _generation);

    /// <summary>The chassis body.</summary>
    public Body Chassis { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Body, s.BodyGen)); } }

    /// <summary>Number of wheels.</summary>
    public int WheelCount => Resolve().Wheels;

    /// <summary>
    /// Sets the driver's inputs for the next steps (they stay until the next call).
    /// </summary>
    /// <param name="throttle">-1..1: positive drives forward, negative reverses (an automatic box brakes first while the vehicle still rolls forward; a manual one uses the gear you set and brakes when the sign opposes it).</param>
    /// <param name="steer">-1..1: positive steers towards +X of the chassis frame (right in the presets); each wheel turns by this times its <see cref="WheelDef.MaxSteerAngle"/>.</param>
    /// <param name="brake">0..1 service brake.</param>
    /// <param name="handbrake">0..1 handbrake, on the wheels that have a <see cref="WheelDef.MaxHandbrakeTorque"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public void Drive(float throttle, float steer = 0f, float brake = 0f, float handbrake = 0f)
    {
        var s = Resolve();
        if (!(throttle >= -1f && throttle <= 1f)) { throw new ArgumentOutOfRangeException(nameof(throttle), throttle, "The throttle must be within -1..1."); }
        if (!(steer >= -1f && steer <= 1f)) { throw new ArgumentOutOfRangeException(nameof(steer), steer, "The steering must be within -1..1."); }
        if (!(brake >= 0f && brake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(brake), brake, "The brake must be within 0..1."); }
        if (!(handbrake >= 0f && handbrake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(handbrake), handbrake, "The handbrake must be within 0..1."); }
        s.Forward = throttle; s.Right = steer; s.Brake = brake; s.Handbrake = handbrake;
    }

    /// <summary>The throttle of the last <see cref="Drive"/>.</summary>
    public float Throttle => Resolve().Forward;

    /// <summary>The steering of the last <see cref="Drive"/>.</summary>
    public float Steering => Resolve().Right;

    /// <summary>Current gear: -1 reverse, 0 neutral, 1.. forward. Setting it shifts (the clutch is open for the box's switch time); on an automatic box the box overrides it again.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The gear is outside -1..number of gears.</exception>
    public int Gear
    {
        get => Resolve().Gear;
        set
        {
            var s = Resolve();
            if (value < -1 || value > s.Ratios.Length) { throw new ArgumentOutOfRangeException(nameof(value), value, "The gear must be within -1.." + s.Ratios.Length.ToString(CultureInfo.InvariantCulture) + "."); }
            if (s.Gear != value) { s.Gear = value; s.ShiftTimer = s.Def.Transmission.SwitchTime; }
        }
    }

    /// <summary>Engine speed in revolutions per minute.</summary>
    public float EngineRpm => Resolve().EngineW * (60f / 6.2831855f);

    /// <summary>Speed along the vehicle's forward direction (the first wheel's), m/s; negative when rolling backwards.</summary>
    public float ForwardSpeed
    {
        get
        {
            var s = Resolve();
            var e = _world!.Engine;
            var f = e.RotationOfBody(s.Body).Rotate(s.Def.Wheels[0].ForwardDirection);
            var v = _world.Core.Bodies.LinearVelocityOf(s.Body);
            return v[0] * f.X + v[1] * f.Y + v[2] * f.Z;
        }
    }

    /// <summary>The state of wheel <paramref name="index"/> as of the last step.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..<see cref="WheelCount"/>-1.</exception>
    public WheelState GetWheel(int index)
    {
        var s = Resolve();
        if ((uint)index >= (uint)s.Wheels) { throw new ArgumentOutOfRangeException(nameof(index), index, "The wheel index must be within 0.." + (s.Wheels - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        var w = _world!;
        var e = w.Engine;
        var bodies = w.Core.Bodies;
        var wd = s.Def.Wheels[index];
        var q = e.RotationOfBody(s.Body);
        var lc = e.LocalCenterOf(s.Body);
        var pp = bodies.PositionOf(s.Body);
        var rTop = q.Rotate(wd.Position - lc);
        var dir = q.Rotate(wd.SuspensionDirection);
        float len = s.Length[index];
        var center = new Vector3D(pp[0] + (double)rTop.X + (double)(dir.X * len), pp[1] + (double)rTop.Y + (double)(dir.Y * len), pp[2] + (double)rTop.Z + (double)(dir.Z * len));
        bool contact = s.Contact[index] != 0;
        Body ground = default;
        if (contact && bodies.Handles.IsAlive(s.GroundBody[index], s.GroundGen[index])) { ground = new Body(w, new BodyHandle(w.Id, s.GroundBody[index], s.GroundGen[index])); }
        return new WheelState(contact, contact ? new Vector3D(s.Cx[index], s.Cy[index], s.Cz[index]) : default, contact ? new Vector3F(s.Nx[index], s.Ny[index], s.Nz[index]) : default, ground, len, s.SpringForce[index],
            s.Omega[index], s.Spin[index], s.Steer[index], s.SlipRatio[index], s.SlipAngle[index], s.Fx[index], s.Fy[index], center, wd.Radius);
    }

    /// <summary>Removes the vehicle; the chassis body stays. Handles to it become stale.</summary>
    public void Destroy()
    {
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

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Makes <paramref name="chassis"/> a wheeled vehicle. The chassis keeps its own shapes for collisions (the wheels are not shapes: they sweep the world for ground); give it a mass for the springs to carry.
    /// Vehicles run once per step, before the solver, in creation order.
    /// </summary>
    /// <param name="chassis">A dynamic body of this world that carries no vehicle yet.</param>
    /// <param name="def">The vehicle, for example <see cref="VehicleDef.Car"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic.</exception>
    /// <exception cref="PhysicsException">The body already carries a vehicle.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public Vehicle AddVehicle(Body chassis, VehicleDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        if (chassis.World != this) { throw new ArgumentException("The chassis belongs to another world.", nameof(chassis)); }
        int v = _engine.CreateVehicle(chassis.Handle, def);
        return new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
    }

    /// <summary>The <see cref="Vehicle"/> for a raw handle (from <see cref="Vehicle.RawHandle"/>).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world (<see cref="ForeignHandleException"/>) or refers to a removed vehicle (<see cref="StaleHandleException"/>).</exception>
    public Vehicle GetVehicle(VehicleHandle handle)
    {
        if (handle.IsNull) { throw new InvalidHandleException("The null vehicle handle was used."); }
        if (handle.World != Id) { throw new ForeignHandleException(handle + " belongs to world " + handle.World.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!_engine.VehHandles.IsAlive(handle.Index, handle.Generation)) { throw new StaleHandleException(handle + " is stale: the vehicle was removed."); }
        return new Vehicle(this, handle.Index, handle.Generation);
    }

    /// <summary>True while the vehicle exists in this world. False for null handles, handles of another world and removed vehicles (never throws). Allocates nothing.</summary>
    public bool IsVehicleAlive(VehicleHandle handle) => !handle.IsNull && handle.World == Id && _engine.VehHandles.IsAlive(handle.Index, handle.Generation);

    /// <summary>The vehicle whose chassis is <paramref name="body"/>, if there is one.</summary>
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
