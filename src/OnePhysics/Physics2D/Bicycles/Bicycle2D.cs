using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A side-view bicycle: a frame body with a two-wheel <see cref="Vehicle"/> (wheel 0 is the front wheel, wheel 1 the rear one). Like <see cref="Body"/> it is a small struct handle (world, slot, generation)
/// that allocates nothing and fails with a <see cref="StaleHandleException"/> once the bicycle is destroyed (its frame destroyed or <see cref="Destroy"/> called).
/// </summary>
/// <example><c>var bike = world.AddBicycle(new Vector2D(0, 0), BicycleDef.Road); bike.Ride(pedal: 1f); world.Step(1f / 60f);</c></example>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct Bicycle : IEquatable<Bicycle>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Bicycle(PhysicsWorld2D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private BikeState2D Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Bicycle is not attached to a world."); }
        var e = _world.Engine;
        if (!e.BikeHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Bicycle(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the bicycle was destroyed.");
        }

        return e.Bikes[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld2D.GetBicycle"/> turns it back into the bicycle.</summary>
    public BicycleHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the bicycle exists.</summary>
    public bool IsValid => _world != null && _world.Engine.BikeHandles.IsAlive(_index, _generation);

    /// <summary>The frame body (with the rider). Its origin is the centre of mass.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public Body Frame { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Frame, s.FrameGen)); } }

    /// <summary>The wheels as a vehicle: wheel 0 the front one, wheel 1 the rear one (readback of contact, load, slip and spin).</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public Vehicle Wheels { get { var s = Resolve(); return new Vehicle(_world!, s.Veh, s.VehGen); } }

    /// <summary>A copy of the definition the bicycle was made from.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public BicycleDef Definition => Resolve().Def.Clone();

    /// <summary>Sets the rider's inputs for the next steps (they stay until the next call).</summary>
    /// <param name="pedal">-1..1: torque of the pedals on the rear wheel, times <see cref="BicycleDef.MaxPedalTorque"/>.</param>
    /// <param name="frontBrake">0..1 front brake lever.</param>
    /// <param name="rearBrake">0..1 rear brake lever.</param>
    /// <param name="lean">-1..1: weight shift as a torque on the frame, times <see cref="BicycleDef.MaxLeanTorque"/>; positive lifts the front.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public void Ride(float pedal = 0f, float frontBrake = 0f, float rearBrake = 0f, float lean = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Bicycle.Ride");
        var s = Resolve();
        if (!(pedal >= -1f && pedal <= 1f)) { throw new ArgumentOutOfRangeException(nameof(pedal), pedal, "The pedal input must be within -1..1."); }
        if (!(frontBrake >= 0f && frontBrake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(frontBrake), frontBrake, "The front brake must be within 0..1."); }
        if (!(rearBrake >= 0f && rearBrake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(rearBrake), rearBrake, "The rear brake must be within 0..1."); }
        if (!(lean >= -1f && lean <= 1f)) { throw new ArgumentOutOfRangeException(nameof(lean), lean, "The lean input must be within -1..1."); }
        s.Pedal = pedal; s.FrontBrake = frontBrake; s.RearBrake = rearBrake; s.Lean = lean;
    }

    /// <summary>Speed of the frame's centre of mass along the bicycle's heading, m/s.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public float Speed => Wheels.ForwardSpeed;

    /// <summary>Pitch of the frame, radians, positive nose up.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public float PitchAngle => _world!.Engine.BikePitch(Resolve());

    /// <summary>True while a wheel's tyre is off the ground.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public bool FrontWheelLifted => !Wheels.GetWheel(0).IsInContact;

    /// <summary>True while the rear wheel's tyre is off the ground.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public bool RearWheelLifted => !Wheels.GetWheel(1).IsInContact;

    /// <summary>True when the pitch has passed <see cref="BicycleDef.FlipAngle"/>.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed.</exception>
    public bool HasFlipped { get { var s = Resolve(); return Math.Abs(_world!.Engine.BikePitch(s)) > s.Def.FlipAngle; } }

    /// <summary>Removes the bicycle with its frame. Handles to it become stale.</summary>
    /// <exception cref="StaleHandleException">The bicycle was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("Bicycle.Destroy");
        var s = Resolve();
        var frame = new Body(_world!, new BodyHandle(_world!.Id, s.Frame, s.FrameGen));
        if (frame.IsValid) { frame.Destroy(); }
        if (_world.Engine.BikeHandles.IsAlive(_index, _generation)) { _world.Engine.DestroyBicycle(_index); }
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

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Happy path: a side-view bicycle standing (or rolling) with its rear contact point at <paramref name="at"/> on the ground, heading +X rotated by <paramref name="angleRadians"/>.
    /// Creates the frame body and the two-wheel vehicle. Bicycles run once per step, before the vehicles.
    /// </summary>
    /// <param name="at">World position of the rear wheel's contact point.</param>
    /// <param name="def">The bicycle; null is <see cref="BicycleDef.Benchmark"/>. Copied.</param>
    /// <param name="speed">Initial forward speed, m/s; the wheels start rolling at it. Default 0.</param>
    /// <param name="angleRadians">Orientation of the bicycle, rotated about the rear contact point. Default 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value or the speed is out of range, or the position is not finite within ±2^21.</exception>
    public Bicycle AddBicycle(Vector2D at, BicycleDef? def = null, float speed = 0f, float angleRadians = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddBicycle");
        def = (def ?? BicycleDef.Benchmark).Clone();
        def.Validate();
        if (!(Math.Abs(speed) <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speed must be finite and within ±1000 m/s."); }
        if (!float.IsFinite(angleRadians)) { throw new ArgumentOutOfRangeException(nameof(angleRadians), angleRadians, "The angle must be finite."); }
        float c = Math32.Cos(angleRadians), sn = Math32.Sin(angleRadians);
        // Frame body at the centre of mass, rotated about the rear contact point.
        double px = at.X + (double)(c * def.CenterX - sn * def.CenterHeight), py = at.Y + (double)(sn * def.CenterX + c * def.CenterHeight);
        var frame = AddEmpty(new Vector2D(px, py), def.Mass, def.PitchInertia);
        try
        {
            frame.SetTransform(new Vector2D(px, py), angleRadians);
            float hertz = def.TyreHertz;
            float k = hertz * 6.2831855f; k *= k;
            float sag = 9.81f / k;
            const float MaxLength = 0.03f;
            float frontLoad = Math.Min(Math.Max(def.Mass * def.CenterX / def.WheelBase, 0.05f * def.Mass), 0.95f * def.Mass);
            WheelDef Wheel(float x, float radius, float spin, float load, float brake) => new()
            {
                // Wheel centre relative to the frame's origin (its centre of mass): `radius` above the ground at forward position x.
                Position = new Vector2F(x - def.CenterX, radius - def.CenterHeight + MaxLength - sag),
                SuspensionDirection = new Vector2F(0f, -1f), ForwardDirection = new Vector2F(1f, 0f), Radius = radius, SuspensionMinLength = 0f, SuspensionMaxLength = MaxLength,
                SpringFrequency = hertz, DampingRatio = 0.4f, SprungMass = load, Inertia = spin, AngularDamping = 0f, MaxBrakeTorque = brake, MaxHandbrakeTorque = 0f,
                Tyre = new TyreDef { Friction = def.TyreFriction, LowSpeed = 0.5f },
            };
            var vd = new VehicleDef
            {
                Wheels = new[] { Wheel(def.WheelBase, def.FrontRadius, def.FrontSpinInertia, frontLoad, def.MaxFrontBrakeTorque), Wheel(0f, def.RearRadius, def.RearSpinInertia, def.Mass - frontLoad, def.MaxRearBrakeTorque) },
                GroundPitchTorque = def.MaxLeanTorque, AirPitchTorque = def.MaxLeanTorque,
            };
            var veh = AddVehicle(frame, vd);
            var dir = new Vector2F(c, sn);
            frame.LinearVelocity = new Vector2F(dir.X * speed, dir.Y * speed);
            var vs = Engine.Vehicles[veh.RawHandle.Index]!;
            vs.Omega[0] = speed / def.FrontRadius; vs.Omega[1] = speed / def.RearRadius;
            var state = new BikeState2D { Frame = frame.Handle.Index, FrameGen = frame.Handle.Generation, Veh = veh.RawHandle.Index, VehGen = veh.RawHandle.Generation, Def = def };
            int i = Engine.CreateBicycle(state);
            return new Bicycle(this, i, Engine.BikeHandles.GenerationOf(i));
        }
        catch (Exception) { frame.Destroy(); throw; }
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
