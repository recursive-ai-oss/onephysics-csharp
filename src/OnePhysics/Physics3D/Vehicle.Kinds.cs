using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Vehicle
{
    /// <summary>Which controller drives this vehicle.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public VehicleKind Kind => Resolve().Kind;

    /// <summary>
    /// Sets the driver's inputs for any kind of vehicle (they stay until the next call); the handbrake is released. For a car this is <see cref="Drive"/> without the handbrake; for a tracked vehicle throttle drives both tracks and
    /// steering speeds up the left one and slows the right one; for a motorcycle steering turns the front wheel (within its speed-dependent limit) and the balance assist leans the bike into the turn.
    /// </summary>
    /// <param name="throttle">-1..1: positive forward, negative reverse.</param>
    /// <param name="steer">-1..1: positive turns right, towards +X of the chassis frame.</param>
    /// <param name="brake">0..1 service brake.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public void SetInput(float throttle, float steer = 0f, float brake = 0f) => Drive(throttle, steer, brake, 0f);

    /// <summary>Belt speed of a tracked vehicle's track (0 left, 1 right) as of the last step, m/s; positive moves the hull forward.</summary>
    /// <exception cref="InvalidOperationException">The vehicle is not tracked.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The track is not 0 or 1.</exception>
    public float GetTrackSpeed(int track)
    {
        var s = Resolve();
        if (s.Kind != VehicleKind.Tracked) { throw new InvalidOperationException("Only a tracked vehicle has tracks."); }
        if ((uint)track > 1u) { throw new ArgumentOutOfRangeException(nameof(track), track, "The track must be 0 (left) or 1 (right)."); }
        return Engine3D.TrackSpeedOf(s, track);
    }

    /// <summary>Number of road wheels on a tracked vehicle's left track; the right track's wheels follow it in <see cref="GetWheel"/>'s numbering.</summary>
    /// <exception cref="InvalidOperationException">The vehicle is not tracked.</exception>
    public int LeftTrackWheelCount
    {
        get
        {
            var s = Resolve();
            if (s.Kind != VehicleKind.Tracked) { throw new InvalidOperationException("Only a tracked vehicle has tracks."); }
            return s.TrackLeft;
        }
    }

    /// <summary>A motorcycle's lean as of the last step, radians: the tilt of the chassis' up axis from vertical about its heading, positive to the right.</summary>
    /// <exception cref="InvalidOperationException">The vehicle is not a motorcycle.</exception>
    public float LeanAngle => ResolveBike().Lean;

    /// <summary>The lean the balance assist steered towards in the last step (what the speed and steering ask for), radians.</summary>
    /// <exception cref="InvalidOperationException">The vehicle is not a motorcycle.</exception>
    public float TargetLeanAngle => ResolveBike().LeanTarget;

    /// <summary>The largest front-wheel angle allowed at the motorcycle's present speed, radians (full steering input turns the wheel by this).</summary>
    /// <exception cref="InvalidOperationException">The vehicle is not a motorcycle.</exception>
    public float SteerLimit
    {
        get
        {
            var s = ResolveBike();
            return Engine3D.BikeSteerLimitAt(s, ForwardSpeed);
        }
    }

    private VehicleState ResolveBike()
    {
        var s = Resolve();
        if (s.Kind != VehicleKind.Motorcycle) { throw new InvalidOperationException("Only a motorcycle has a lean angle."); }
        return s;
    }
}

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Makes <paramref name="chassis"/> a tracked vehicle. Same rules as <see cref="AddVehicle(Body, VehicleDef)"/>: a dynamic body with a mass, no vehicle yet; vehicles run once per step before the solver, in creation order.
    /// </summary>
    /// <param name="chassis">A dynamic body of this world that carries no vehicle yet.</param>
    /// <param name="def">The tracks, for example <see cref="TrackedVehicleDef.Tank"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="PhysicsException">The body already carries a vehicle.</exception>
    public Vehicle AddVehicle(Body chassis, TrackedVehicleDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        if (chassis.World != this) { throw new ArgumentException("The chassis belongs to another world.", nameof(chassis)); }
        var wheeled = def.ToWheeled();
        int v = _engine.CreateVehicle(chassis.Handle, wheeled, track: def);
        return new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
    }

    /// <summary>
    /// Makes <paramref name="chassis"/> a motorcycle (front wheel is wheel 0, rear wheel 1). Same rules as <see cref="AddVehicle(Body, VehicleDef)"/>.
    /// </summary>
    /// <param name="chassis">A dynamic body of this world that carries no vehicle yet.</param>
    /// <param name="def">The motorcycle, for example <see cref="MotorcycleDef.Road"/>; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    /// <exception cref="ArgumentException">The body is not dynamic or belongs to another world.</exception>
    /// <exception cref="PhysicsException">The body already carries a vehicle.</exception>
    public Vehicle AddVehicle(Body chassis, MotorcycleDef def)
    {
        ArgumentNullException.ThrowIfNull(def);
        if (chassis.World != this) { throw new ArgumentException("The chassis belongs to another world.", nameof(chassis)); }
        var wheeled = def.ToWheeled();
        int v = _engine.CreateVehicle(chassis.Handle, wheeled, bike: def);
        return new Vehicle(this, v, _engine.VehHandles.GenerationOf(v));
    }

    /// <summary>
    /// Happy path: a tank resting on the ground at <paramref name="at"/> (the ground point under the hull's centre), facing +Z, built from a hull box of the preset's size and mass.
    /// <c>var tank = world.AddTank(at); tank.SetInput(throttle: 1f, steer: 0.3f);</c>
    /// </summary>
    /// <param name="at">Ground point under the hull centre.</param>
    /// <param name="def">The tracks; null (default) is <see cref="TrackedVehicleDef.Tank"/>.</param>
    /// <param name="rotation">Orientation of the hull; default faces +Z.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    public Vehicle AddTank(Vector3D at, TrackedVehicleDef? def = null, QuaternionF rotation = default)
    {
        def ??= TrackedVehicleDef.Tank;
        def.Validate();
        var wheel = def.LeftWheels[0];
        var size = def.ChassisSize;
        var body = AddBox(size, RestingPoint(at, wheel), rotation, density: def.ChassisMass / (size.X * size.Y * size.Z));
        return AddVehicle(body, def);
    }

    /// <summary>
    /// Happy path: a motorcycle resting on its wheels at <paramref name="at"/> (the ground point under the chassis centre), facing +Z, built from a chassis box of the preset's size and mass.
    /// <c>var bike = world.AddMotorcycle(at); bike.SetInput(throttle: 0.5f, steer: 0.2f);</c>
    /// </summary>
    /// <param name="at">Ground point under the chassis centre.</param>
    /// <param name="def">The motorcycle; null (default) is <see cref="MotorcycleDef.Road"/>.</param>
    /// <param name="rotation">Orientation of the chassis; default faces +Z.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    public Vehicle AddMotorcycle(Vector3D at, MotorcycleDef? def = null, QuaternionF rotation = default)
    {
        def ??= MotorcycleDef.Road;
        def.Validate();
        var size = def.ChassisSize;
        var body = AddBox(size, RestingPoint(at, def.Front), rotation, density: def.ChassisMass / (size.X * size.Y * size.Z));
        return AddVehicle(body, def);
    }

    /// <summary>The chassis position that puts a wheel's tyre just above the ground point (the suspension a little short of fully extended), for an upright chassis.</summary>
    private static Vector3D RestingPoint(Vector3D ground, WheelDef wheel)
    {
        float drop = wheel.Radius + wheel.SuspensionMaxLength - 0.05f - wheel.Position.Y;
        return new Vector3D(ground.X, ground.Y + drop, ground.Z);
    }
}
