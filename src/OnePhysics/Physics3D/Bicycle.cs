using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A bicycle: a rear frame and a front assembly joined by a steering hinge, each carrying one wheel (a one-wheel <see cref="Vehicle"/>). Like <see cref="Body"/> it is a small struct handle (world, slot,
/// generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the bicycle is destroyed (either body destroyed or <see cref="Destroy"/> called).
/// Uncontrolled, the benchmark bicycle (<see cref="BicycleDef.Benchmark"/>) balances itself between its weave and capsize speeds; below and above that band the rider has to lean and steer.
/// </summary>
/// <example><c>var bike = world.AddBicycle(new Vector3D(0, 0, 0), BicycleDef.Benchmark, speed: 5f); bike.Ride(pedal: 0.2f); world.Step(1f / 60f);</c></example>
public readonly struct Bicycle : IEquatable<Bicycle>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Bicycle(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private BikeState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Bicycle is not attached to a world."); }
        var e = _world.Engine;
        if (!e.BikeHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Bicycle(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the bicycle was destroyed.");
        }

        return e.Bikes[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld3D.GetBicycle"/> turns it back into the bicycle.</summary>
    public BicycleHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the bicycle exists.</summary>
    public bool IsValid => _world != null && _world.Engine.BikeHandles.IsAlive(_index, _generation);

    /// <summary>The rear frame (with the rider). Its origin is its centre of mass.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public Body Frame { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Frame, s.FrameGen)); } }

    /// <summary>The front assembly: fork, handlebar and front wheel. Its origin is its centre of mass.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public Body Fork { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Fork, s.ForkGen)); } }

    /// <summary>The rear wheel as a vehicle (for its contact, slip and spin readback).</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public Vehicle RearWheel { get { var s = Resolve(); return new Vehicle(_world!, s.RearVeh, s.RearVehGen); } }

    /// <summary>The front wheel as a vehicle.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public Vehicle FrontWheel { get { var s = Resolve(); return new Vehicle(_world!, s.FrontVeh, s.FrontVehGen); } }

    /// <summary>A copy of the definition the bicycle was made from.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public BicycleDef Definition => Resolve().Def.Clone();

    /// <summary>
    /// Sets the rider's inputs for the next steps (they stay until the next call). The bicycle is not balanced for you.
    /// </summary>
    /// <param name="steer">-1..1: torque on the handlebar about the steering axis, positive to the right, times <see cref="BicycleDef.MaxSteerTorque"/>.</param>
    /// <param name="pedal">-1..1: torque of the pedals on the rear wheel, times <see cref="BicycleDef.MaxPedalTorque"/> (negative back-pedals).</param>
    /// <param name="frontBrake">0..1 front brake lever.</param>
    /// <param name="rearBrake">0..1 rear brake lever.</param>
    /// <param name="lean">-1..1: the rider's weight shift as a roll torque on the frame, positive to the right, times <see cref="BicycleDef.MaxLeanTorque"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public void Ride(float steer = 0f, float pedal = 0f, float frontBrake = 0f, float rearBrake = 0f, float lean = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Bicycle.Ride");
        var s = Resolve();
        if (!(steer >= -1f && steer <= 1f)) { throw new ArgumentOutOfRangeException(nameof(steer), steer, "The steer input must be within -1..1."); }
        if (!(pedal >= -1f && pedal <= 1f)) { throw new ArgumentOutOfRangeException(nameof(pedal), pedal, "The pedal input must be within -1..1."); }
        if (!(frontBrake >= 0f && frontBrake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(frontBrake), frontBrake, "The front brake must be within 0..1."); }
        if (!(rearBrake >= 0f && rearBrake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(rearBrake), rearBrake, "The rear brake must be within 0..1."); }
        if (!(lean >= -1f && lean <= 1f)) { throw new ArgumentOutOfRangeException(nameof(lean), lean, "The lean input must be within -1..1."); }
        s.Steer = steer; s.Pedal = pedal; s.FrontBrake = frontBrake; s.RearBrake = rearBrake; s.Lean = lean;
    }

    /// <summary>Forward speed of the frame's centre of mass along its heading, m/s.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public float Speed
    {
        get
        {
            var s = Resolve();
            var q = _world!.Engine.RotationOfBody(s.Frame);
            var f = q.Rotate(new Vector3F(0f, 0f, 1f));
            var v = _world.Core.Bodies.LinearVelocityOf(s.Frame);
            return v[0] * f.X + v[1] * f.Y + v[2] * f.Z;
        }
    }

    /// <summary>Roll of the frame from the vertical, radians, positive when the bicycle leans to its right.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public float LeanAngle => _world!.Engine.BikeLean(Resolve());

    /// <summary>Steering angle about the steering axis, radians, positive to the right.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public float SteerAngle => _world!.Engine.BikeSteerAngle(Resolve());

    /// <summary>True when the roll has passed <see cref="BicycleDef.FallAngle"/>.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public bool HasFallen { get { var s = Resolve(); return Math.Abs(_world!.Engine.BikeLean(s)) > s.Def.FallAngle; } }

    /// <summary>Removes the bicycle: its hinge, its two wheels' vehicles and its two bodies. Handles to any of them become stale.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("Bicycle.Destroy");
        var s = Resolve();
        var w = _world!;
        var frame = new Body(w, new BodyHandle(w.Id, s.Frame, s.FrameGen));
        var fork = new Body(w, new BodyHandle(w.Id, s.Fork, s.ForkGen));
        if (frame.IsValid) { frame.Destroy(); }
        if (fork.IsValid) { fork.Destroy(); }
        if (w.Engine.BikeHandles.IsAlive(_index, _generation)) { w.Engine.DestroyBicycle(_index); }
    }

    /// <inheritdoc/>
    public bool Equals(Bicycle other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Bicycle b && Equals(b);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(_index * 397 + _generation);

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Bicycle a, Bicycle b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Bicycle a, Bicycle b) => !a.Equals(b);
}

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Happy path: a bicycle standing (or rolling) upright with its rear contact point at <paramref name="at"/> on the ground, heading along +Z rotated by <paramref name="heading"/>.
    /// Creates the two bodies, the steering hinge and the two wheels. Bicycles run once per step, before the vehicles.
    /// </summary>
    /// <param name="at">World position of the rear wheel's contact point; the ground is the plane through it.</param>
    /// <param name="def">The bicycle; null is <see cref="BicycleDef.Benchmark"/>. Copied.</param>
    /// <param name="speed">Initial forward speed, m/s; the wheels start rolling at it. Default 0.</param>
    /// <param name="heading">Orientation about the vertical axis. Default identity (heading +Z).</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value or the speed is out of range, or the position is not finite within ±2^21.</exception>
    public Bicycle AddBicycle(Vector3D at, BicycleDef? def = null, float speed = 0f, QuaternionF heading = default)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddBicycle");
        def = (def ?? BicycleDef.Benchmark).Clone();
        def.Validate();
        if (!(Math.Abs(speed) <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speed must be finite and within ±1000 m/s."); }
        if (heading.Equals(default(QuaternionF))) { heading = QuaternionF.Identity; }
        var rot = heading.Normalized();
        float w = def.WheelBase, c = def.Trail, lam = def.HeadAngle;
        float sinL = Math32.Sin(lam), cosL = Math32.Cos(lam);

        // Bicycle axes (x forward, y right, z down, origin under the rear contact point) to the world's (+X right, +Y up, +Z forward).
        static Vector3F Map(float x, float z) => new(0f, -z, x);
        Vector3F World(float x, float z) => rot.Rotate(Map(x, z));
        Vector3D Pos(float x, float z) { var v = World(x, z); return new Vector3D(at.X + v.X, at.Y + v.Y, at.Z + v.Z); }

        var rearPart = def.RearFrame; var frontPart = def.Front;
        var frame = CreatePart(rearPart, Pos(rearPart.CenterX, rearPart.CenterZ), rot);
        Body fork;
        try { fork = CreatePart(frontPart, Pos(frontPart.CenterX, frontPart.CenterZ), rot); }
        catch (Exception) { frame.Destroy(); throw; }
        try { AddBikeShapes(def, frame, fork); }
        catch (Exception) { fork.Destroy(); frame.Destroy(); throw; }

        try
        {
            // Static wheel loads from the moment balance about the rear contact point (the contact points lie under the wheel centres).
            float mTotal = rearPart.Mass + frontPart.Mass;
            float xCg = (rearPart.Mass * rearPart.CenterX + frontPart.Mass * frontPart.CenterX) / mTotal;
            float frontLoad = Math.Min(Math.Max(mTotal * xCg / w, 0.05f * mTotal), 0.95f * mTotal);
            float rearLoad = mTotal - frontLoad;
            var rearDef = BikeWheelVehicle(def, def.RearWheel, rearPart, 0f, rearLoad, def.MaxRearBrakeTorque);
            var frontDef = BikeWheelVehicle(def, def.FrontWheel, frontPart, w, frontLoad, def.MaxFrontBrakeTorque);
            var rearVeh = AddVehicle(frame, rearDef);
            Vehicle frontVeh;
            try { frontVeh = AddVehicle(fork, frontDef); }
            catch (Exception) { rearVeh.Destroy(); throw; }

            // The steering hinge: a point on the steering axis (the axis foot ahead of the front contact point by the trail, then up the axis), turning about the axis.
            var footX = w + c;
            var anchor = Pos(footX - 0.5f * sinL, -0.5f * cosL);
            var axisWorld = rot.Rotate(new Vector3F(0f, cosL, -sinL));
            AddHinge(frame, fork, anchor, axisWorld);

            var dir = rot.Rotate(new Vector3F(0f, 0f, 1f));
            var vel = dir * speed;
            frame.LinearVelocity = vel; fork.LinearVelocity = vel;
            Engine.Vehicles[rearVeh.RawHandle.Index]!.Omega[0] = speed / def.RearWheel.Radius;
            Engine.Vehicles[frontVeh.RawHandle.Index]!.Omega[0] = speed / def.FrontWheel.Radius;

            var state = new BikeState
            {
                Frame = frame.Handle.Index, FrameGen = frame.Handle.Generation, Fork = fork.Handle.Index, ForkGen = fork.Handle.Generation,
                RearVeh = rearVeh.RawHandle.Index, RearVehGen = rearVeh.RawHandle.Generation, FrontVeh = frontVeh.RawHandle.Index, FrontVehGen = frontVeh.RawHandle.Generation, Def = def,
            };
            int i = Engine.CreateBicycle(state);
            return new Bicycle(this, i, Engine.BikeHandles.GenerationOf(i));
        }
        catch (Exception)
        {
            fork.Destroy(); frame.Destroy();
            throw;
        }
    }

    // Collision shapes that give a crashed bicycle something to lie on and let a rider hit things: a capsule for the rider and frame, a fork strut and the handlebar. They carry no mass (the parts' mass and inertia
    // are the paper's), sit well above the ground while the bicycle is upright, and are invisible to the bicycle's own wheel casts (Engine3D.BikeCategory); the two bodies share a negative group so they never push on each other.
    private void AddBikeShapes(BicycleDef def, Body frame, Body fork)
    {
        int group = -(0x40000000 + frame.Handle.Index);
        // Paper axes (x forward, z down) to the part's local axes (X right, Y up, Z forward), relative to the part's centre of mass.
        static Vector3F Local(BicyclePart p, float x, float up) => new(0f, up + p.CenterZ, x - p.CenterX);
        var common = new ShapeDef { Density = 0f, Friction = 0.8f, GroupIndex = group, CategoryBits = Engine3D.BikeCategory | 1u };
        var rider = common; rider.LocalPosition = Local(def.RearFrame, def.RearFrame.CenterX, 0.75f);
        CreateCapsule(frame, 0.18f, 0.12f, rider);
        float w = def.WheelBase;
        var strut = common; strut.LocalPosition = Local(def.Front, w, 0.65f);
        CreateCapsule(fork, 0.3f, 0.04f, strut);
        var bar = common; bar.LocalPosition = Local(def.Front, w - 0.05f, 0.98f);
        bar.LocalRotation = QuaternionF.CreateFromAxisSinCos(new Vector3F(0f, 0f, 1f), -0.70710677f, 0.70710677f);
        CreateCapsule(fork, 0.22f, 0.03f, bar);
    }

    // A rigid part: an empty body at its centre of mass with the part's mass and inertia tensor (mapped into world axes and diagonalised).
    private Body CreatePart(BicyclePart p, Vector3D com, QuaternionF rot)
    {
        // Paper tensor [[Ixx,0,Ixz],[0,Iyy,0],[Ixz,0,Izz]] in (x forward, y right, z down) becomes, in (X right, Y up, Z forward): XX = Iyy, YY = Izz, ZZ = Ixx and YZ = -Ixz.
        var tensor = new Matrix3x3F(p.Iyy, 0f, 0f, 0f, p.Izz, -p.Ixz, 0f, -p.Ixz, p.Ixx);
        var axes = InertiaF.PrincipalAxes(tensor);
        var body = AddEmpty(com, p.Mass, axes.Moments);
        try
        {
            body.SetInertia(axes.Moments, axes.Orientation);
            body.SetTransform(com, rot);
        }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }

    // One wheel as a vehicle on its part: the wheel's centre is `radius` above the ground at forward position x; the part's origin is its centre of mass.
    private static VehicleDef BikeWheelVehicle(BicycleDef def, BicycleWheel wheel, BicyclePart part, float x, float load, float brake)
    {
        // Wheel centre relative to the part's centre of mass, in world axes (X right, Y up, Z forward): the centre is `radius` above the ground and the part's centre -CenterZ above it.
        var centre = new Vector3F(0f, wheel.Radius + part.CenterZ, x - part.CenterX);
        float hertz = def.TyreHertz;
        float k = hertz * 6.2831855f; k *= k;
        float sag = 9.81f / k;   // the tyre spring carries the load at this compression, whatever the load is
        const float MaxLength = 0.03f;
        var wd = new WheelDef
        {
            Position = centre + new Vector3F(0f, MaxLength - sag, 0f),
            SuspensionDirection = new Vector3F(0f, -1f, 0f), AxleDirection = new Vector3F(1f, 0f, 0f), ForwardDirection = new Vector3F(0f, 0f, 1f), SteerAxis = new Vector3F(0f, 1f, 0f),
            Radius = wheel.Radius, Width = 0.001f, SuspensionMinLength = 0f, SuspensionMaxLength = MaxLength, SpringFrequency = hertz, DampingRatio = 0.4f, PreloadLength = 0f,
            SprungMass = load, Inertia = wheel.SpinInertia, AngularDamping = 0f, MaxSteerAngle = 0f, MaxBrakeTorque = brake, MaxHandbrakeTorque = 0f, CastShape = WheelCastShape.Cylinder,
            Tyre = new TyreDef { Friction = def.TyreFriction, LateralB = def.TyreStiffness, LateralC = 1.3f, LateralE = 0f, LowSpeed = 0.5f },
        };
        return new VehicleDef { Wheels = new[] { wd } };
    }

    /// <summary>The <see cref="Bicycle"/> for a raw handle (from <see cref="Bicycle.RawHandle"/>).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world (<see cref="ForeignHandleException"/>) or refers to a removed bicycle (<see cref="StaleHandleException"/>).</exception>
    public Bicycle GetBicycle(BicycleHandle handle)
    {
        if (handle.IsNull) { throw new InvalidHandleException("The null bicycle handle was used."); }
        if (handle.World != Id) { throw new ForeignHandleException(handle + " belongs to world " + handle.World.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!_engine.BikeHandles.IsAlive(handle.Index, handle.Generation)) { throw new StaleHandleException(handle + " is stale: the bicycle was removed."); }
        return new Bicycle(this, handle.Index, handle.Generation);
    }

    /// <summary>True while the bicycle exists in this world. False for null handles, handles of another world and removed bicycles (never throws). Allocates nothing.</summary>
    public bool IsBicycleAlive(BicycleHandle handle) => !handle.IsNull && handle.World == Id && _engine.BikeHandles.IsAlive(handle.Index, handle.Generation);

    /// <summary>All live bicycles in slot order (allocates the array).</summary>
    public Bicycle[] GetBicycles()
    {
        var list = new List<Bicycle>();
        for (int i = 0; i < _engine.BikeHandles.HighWater; i++)
        {
            if (_engine.BikeHandles.IsLive(i)) { list.Add(new Bicycle(this, i, _engine.BikeHandles.GenerationOf(i))); }
        }

        return list.ToArray();
    }
}
