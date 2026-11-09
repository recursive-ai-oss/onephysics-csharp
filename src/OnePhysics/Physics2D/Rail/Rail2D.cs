using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A side-view railway track in a 2D world: the vertical profile of a <see cref="TrackDef"/> laid out along +X from its start, with a static chain body for the rail surface. Like <see cref="Body"/> it is a small
/// struct handle (world, slot, generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the track is destroyed.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct Track : IEquatable<Track>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Track(PhysicsWorld2D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    internal int Slot => _index;

    internal TrackState2D Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Track is not attached to a world."); }
        var e = _world.Engine;
        if (!e.TrackHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Track(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the track was destroyed.");
        }

        return e.Tracks[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld2D.GetTrack"/> turns it back into the track.</summary>
    public TrackHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the track exists.</summary>
    public bool IsValid => _world != null && _world.Engine.TrackHandles.IsAlive(_index, _generation);

    /// <summary>A copy of the definition (with the current origin applied to <see cref="TrackDef.Start"/>).</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public TrackDef Definition => Resolve().Def.Clone();

    /// <summary>Length of the track, m.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public double Length => Resolve().Geo.Length;

    /// <summary>The point of the rail surface at chainage <paramref name="s"/> and the grade there (rise over run); beyond the ends the track continues level.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public Vector2D SurfaceAt(double s, out float grade)
    {
        var t = Resolve();
        var f = t.Geo.FrameAt(s);
        grade = f.Tangent.X > 1e-6f ? f.Tangent.Y / f.Tangent.X : 0f;
        return new Vector2D(f.Position.X, f.Position.Y);
    }

    /// <summary>Removes the track and its chain body. Rail vehicles on it keep running on whatever else is there.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("Track.Destroy");
        var t = Resolve();
        var w = _world!;
        if (t.Body >= 0 && w.Core.Bodies.Handles.IsAlive(t.Body, t.BodyGen)) { new Body(w, new BodyHandle(w.Id, t.Body, t.BodyGen)).Destroy(); }
        if (w.Engine.TrackHandles.IsAlive(_index, _generation)) { w.Engine.DestroyTrack(_index); }
    }

    /// <inheritdoc/>
    public bool Equals(Track other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Track t && Equals(t);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(_index * 397 + _generation);

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Track a, Track b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Track a, Track b) => !a.Equals(b);
}

/// <summary>
/// A side-view rail vehicle: a car body with a wheeled <see cref="Vehicle"/> (wheel 0 the rear wheelset) and its couplers. Like <see cref="Body"/> it is a small struct handle (world, slot, generation) that
/// allocates nothing and fails with a <see cref="StaleHandleException"/> once the vehicle is destroyed (its car destroyed or <see cref="Destroy"/> called).
/// </summary>
/// <example><c>var track = world.AddTrack(TrackDef.Straight(500)); var loco = world.AddRailVehicle(track, 20, RailDef.Locomotive); loco.Drive(throttle: 1f); world.Step(1f / 60f);</c></example>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct RailVehicle : IEquatable<RailVehicle>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal RailVehicle(PhysicsWorld2D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private RailState2D Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default RailVehicle is not attached to a world."); }
        var e = _world.Engine;
        if (!e.RailHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("RailVehicle(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the vehicle was destroyed.");
        }

        return e.Rails[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld2D.GetRailVehicle"/> turns it back into the vehicle.</summary>
    public RailVehicleHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the vehicle exists.</summary>
    public bool IsValid => _world != null && _world.Engine.RailHandles.IsAlive(_index, _generation);

    /// <summary>The car body.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public Body Car { get { var s = Resolve(); return new Body(_world!, new BodyHandle(_world!.Id, s.Car, s.CarGen)); } }

    /// <summary>The wheelsets as a vehicle (readback of contact, load, slip and spin; wheel 0 is the rear wheelset).</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public Vehicle Wheels { get { var s = Resolve(); return new Vehicle(_world!, s.Veh, s.VehGen); } }

    /// <summary>Number of wheelsets.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public int WheelsetCount => Resolve().Def.Axles;

    /// <summary>A copy of the definition the vehicle was made from.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public RailDef Definition => Resolve().Def.Clone();

    /// <summary>Sets the driver's inputs for the next steps (they stay until the next call).</summary>
    /// <param name="throttle">-1..1: traction force of the driven wheelsets as a fraction of <see cref="RailDef.MaxTractionForce"/> (negative drives backwards).</param>
    /// <param name="brake">0..1: friction brake on every wheelset as a fraction of <see cref="RailDef.MaxBrakeForce"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or outside its range; the inputs stay as they were.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public void Drive(float throttle = 0f, float brake = 0f)
    {
        using var replayMutation = _world?.RecordMutation("RailVehicle.Drive");
        var s = Resolve();
        if (!(throttle >= -1f && throttle <= 1f)) { throw new ArgumentOutOfRangeException(nameof(throttle), throttle, "The throttle must be within -1..1."); }
        if (!(brake >= 0f && brake <= 1f)) { throw new ArgumentOutOfRangeException(nameof(brake), brake, "The brake must be within 0..1."); }
        s.Throttle = throttle; s.Brake = brake;
    }

    /// <summary>The throttle of the last <see cref="Drive"/>.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float Throttle => Resolve().Throttle;

    /// <summary>The brake of the last <see cref="Drive"/>.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float Brake => Resolve().Brake;

    /// <summary>Speed of the car along its own forward axis, m/s.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float Speed => Wheels.ForwardSpeed;

    /// <summary>Couples this vehicle's <paramref name="thisFront"/> end (front is the +X end) to <paramref name="other"/>'s <paramref name="otherFront"/> end; a coupler carries nothing until the vehicles come within its reach.</summary>
    /// <exception cref="PhysicsException">An end is already coupled.</exception>
    /// <exception cref="ArgumentException">The other vehicle belongs to another world or is this vehicle.</exception>
    /// <exception cref="StaleHandleException">A vehicle was destroyed.</exception>
    public void Couple(RailVehicle other, bool thisFront = true, bool otherFront = false)
    {
        using var replayMutation = _world?.RecordMutation("RailVehicle.Couple");
        var s = Resolve(); var o = other.Resolve();
        if (!ReferenceEquals(_world, other._world) || _index == other._index) { throw new ArgumentException("The other vehicle must be another vehicle of the same world.", nameof(other)); }
        int e = thisFront ? 0 : 1, oe = otherFront ? 0 : 1;
        if (s.CoupleVeh[e] >= 0 || o.CoupleVeh[oe] >= 0) { throw new PhysicsException("That coupler is already coupled."); }
        s.CoupleVeh[e] = other._index; s.CoupleGen[e] = other._generation; s.CoupleEnd[e] = oe; s.Broken[e] = 0; s.CouplerForce[e] = 0f;
        o.CoupleVeh[oe] = _index; o.CoupleGen[oe] = _generation; o.CoupleEnd[oe] = e; o.Broken[oe] = 0; o.CouplerForce[oe] = 0f;
    }

    /// <summary>Uncouples this vehicle's front or rear end (nothing happens when it is not coupled).</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public void Uncouple(bool front)
    {
        using var replayMutation = _world?.RecordMutation("RailVehicle.Uncouple");
        var s = Resolve();
        int e = front ? 0 : 1;
        int o = s.CoupleVeh[e];
        if (o >= 0 && _world!.Engine.RailHandles.IsAlive(o, s.CoupleGen[e])) { var os = _world.Engine.Rails[o]!; os.CoupleVeh[s.CoupleEnd[e]] = -1; os.CouplerForce[s.CoupleEnd[e]] = 0f; os.Broken[s.CoupleEnd[e]] = 0; }
        s.CoupleVeh[e] = -1; s.CouplerForce[e] = 0f; s.Broken[e] = 0;
    }

    /// <summary>Force in the coupler at the front or rear end as of the last step, N: positive in tension, negative when the buffers push; 0 when uncoupled or broken.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float CouplerForce(bool front) => Resolve().CouplerForce[front ? 0 : 1];

    /// <summary>True when the coupler at that end was coupled and its draw gear has broken.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public bool IsCouplerBroken(bool front) => Resolve().Broken[front ? 0 : 1] != 0;

    /// <summary>True when every wheelset touches the rail.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public bool IsOnTrack { get { var s = Resolve(); var v = Wheels; for (int k = 0; k < s.Def.Axles; k++) { if (!v.GetWheel(k).IsInContact) { return false; } } return true; } }

    /// <summary>Removes the vehicle with its car. Handles to it become stale.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("RailVehicle.Destroy");
        var s = Resolve();
        var w = _world!;
        if (w.Core.Bodies.Handles.IsAlive(s.Car, s.CarGen)) { new Body(w, new BodyHandle(w.Id, s.Car, s.CarGen)).Destroy(); }
        if (w.Engine.RailHandles.IsAlive(_index, _generation)) { w.Engine.DestroyRailVehicle(_index); }
    }

    /// <inheritdoc/>
    public bool Equals(RailVehicle other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is RailVehicle v && Equals(v);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(_index * 397 + _generation);

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(RailVehicle a, RailVehicle b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(RailVehicle a, RailVehicle b) => !a.Equals(b);
}

public sealed partial class PhysicsWorld2D
{
    /// <summary>Lays out a side-view track in this world: its vertical profile along +X from <see cref="TrackDef.Start"/>, with a static chain body as the rail surface (see <see cref="Track"/>).</summary>
    /// <param name="def">The track; copied. Its plan view must be one straight (a 2D track is a vertical profile).</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range or the plan view is not one straight.</exception>
    public Track AddTrack(TrackDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddTrack");
        ArgumentNullException.ThrowIfNull(def);
        var d = def.Clone();
        d.Validate(true);
        d.StartHeading = 0f;
        var geo = new TrackGeometry(d);
        var state = new TrackState2D { Def = d, Geo = geo };
        if (d.Collide)
        {
            double len = geo.Length;
            int n = (int)Math.Ceiling(len) + 1;
            var pts = new Vector2D[n];
            for (int i = 0; i < n; i++) { var f = geo.FrameAt(Math.Min(len, (double)i)); pts[i] = new Vector2D(f.Position.X, f.Position.Y); }
            var body = AddChain(pts, false, true, new Material2D(d.Friction, 0f, 0f));
            state.Body = body.Handle.Index; state.BodyGen = body.Handle.Generation;
        }

        int slot = _engine.CreateTrack(state);
        return new Track(this, slot, _engine.TrackHandles.GenerationOf(slot));
    }

    /// <summary>
    /// Puts a rail vehicle on a track with its centre at chainage <paramref name="chainage"/>, along the track's slope, standing on its wheelsets. Creates the car body and its wheeled vehicle.
    /// Rail vehicles run once per step, before the vehicles.
    /// </summary>
    /// <param name="track">The track the vehicle stands on.</param>
    /// <param name="chainage">Chainage of the car's centre.</param>
    /// <param name="def">The vehicle, for example <see cref="RailDef.Locomotive"/>; copied.</param>
    /// <param name="speed">Initial speed along the track, m/s; the wheelsets start rolling at it. Default 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value or the speed is out of range.</exception>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public RailVehicle AddRailVehicle(Track track, double chainage, RailDef def, float speed = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddRailVehicle");
        ArgumentNullException.ThrowIfNull(def);
        def = def.Clone();
        def.Validate();
        var ts = track.Resolve();
        if (!(Math.Abs(speed) <= 500f)) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speed must be finite and within ±500 m/s."); }
        if (!(Math.Abs(chainage) <= 1e5)) { throw new ArgumentOutOfRangeException(nameof(chainage), chainage, "The chainage must be finite and within ±100000 m."); }
        var f = ts.Geo.FrameAt(chainage);
        float tx = f.Tangent.X, ty = f.Tangent.Y;
        float tl = float.Sqrt(tx * tx + ty * ty);
        tx /= tl; ty /= tl;
        float angle = Math32.Atan2(ty, tx);
        float lift = def.CarFloorHeight + 0.5f * def.CarSize.Y;
        var center = new Vector2D(f.Position.X - ty * lift, f.Position.Y + tx * lift);
        var car = AddBox(def.CarSize, center, angle, BodyType.Dynamic, new Material2D(0.5f, 0f, 0f), def.CarMass / (def.CarSize.X * def.CarSize.Y));
        try
        {
            float hertz = def.ContactHertz;
            float k = hertz * 6.2831855f; k *= k;
            float sag = 9.81f / k;
            const float MaxLength = 0.03f;
            int n = def.Axles;
            var wheels = new WheelDef[n];
            for (int i = 0; i < n; i++)
            {
                float x = -0.5f * def.AxleSpan + i * def.AxleSpan / (n - 1);
                wheels[i] = new WheelDef
                {
                    Position = new Vector2F(x, def.WheelRadius - lift + MaxLength - sag), SuspensionDirection = new Vector2F(0f, -1f), ForwardDirection = new Vector2F(1f, 0f), Radius = def.WheelRadius,
                    SuspensionMinLength = 0f, SuspensionMaxLength = MaxLength, SpringFrequency = hertz, DampingRatio = 0.6f, SprungMass = def.CarMass / n, Inertia = def.WheelsetSpinInertia, AngularDamping = 0f,
                    MaxBrakeTorque = def.MaxBrakeForce / n * def.WheelRadius, MaxHandbrakeTorque = 0f,
                    Tyre = new TyreDef { Friction = ts.Def.Friction, LongitudinalB = 25f, LongitudinalC = 1.65f, LongitudinalE = 0.9f, LowSpeed = 0.5f },
                };
            }

            var veh = AddVehicle(car, new VehicleDef { Wheels = wheels });
            car.LinearVelocity = new Vector2F(tx * speed, ty * speed);
            var vs = Engine.Vehicles[veh.RawHandle.Index]!;
            for (int i = 0; i < n; i++) { vs.Omega[i] = speed / def.WheelRadius; }
            var state = new RailState2D { Def = def, Car = car.Handle.Index, CarGen = car.Handle.Generation, Veh = veh.RawHandle.Index, VehGen = veh.RawHandle.Generation, Track = track.Slot, TrackGen = _engine.TrackHandles.GenerationOf(track.Slot) };
            int slot = _engine.CreateRailVehicle(state);
            return new RailVehicle(this, slot, _engine.RailHandles.GenerationOf(slot));
        }
        catch (Exception) { car.Destroy(); throw; }
    }

    /// <summary>The <see cref="Track"/> for a raw handle (from <see cref="Track.RawHandle"/>).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world (<see cref="ForeignHandleException"/>) or refers to a removed track (<see cref="StaleHandleException"/>).</exception>
    public Track GetTrack(TrackHandle handle)
    {
        if (handle.IsNull) { throw new InvalidHandleException("The null track handle was used."); }
        if (handle.World != Id) { throw new ForeignHandleException(handle + " belongs to world " + handle.World.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!_engine.TrackHandles.IsAlive(handle.Index, handle.Generation)) { throw new StaleHandleException(handle + " is stale: the track was removed."); }
        return new Track(this, handle.Index, handle.Generation);
    }

    /// <summary>True while the track exists in this world. False for null handles, handles of another world and removed tracks (never throws). Allocates nothing.</summary>
    public bool IsTrackAlive(TrackHandle handle) => !handle.IsNull && handle.World == Id && _engine.TrackHandles.IsAlive(handle.Index, handle.Generation);

    /// <summary>The <see cref="RailVehicle"/> for a raw handle (from <see cref="RailVehicle.RawHandle"/>).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world (<see cref="ForeignHandleException"/>) or refers to a removed vehicle (<see cref="StaleHandleException"/>).</exception>
    public RailVehicle GetRailVehicle(RailVehicleHandle handle)
    {
        if (handle.IsNull) { throw new InvalidHandleException("The null rail vehicle handle was used."); }
        if (handle.World != Id) { throw new ForeignHandleException(handle + " belongs to world " + handle.World.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!_engine.RailHandles.IsAlive(handle.Index, handle.Generation)) { throw new StaleHandleException(handle + " is stale: the rail vehicle was removed."); }
        return new RailVehicle(this, handle.Index, handle.Generation);
    }

    /// <summary>True while the rail vehicle exists in this world. False for null handles, handles of another world and removed vehicles (never throws). Allocates nothing.</summary>
    public bool IsRailVehicleAlive(RailVehicleHandle handle) => !handle.IsNull && handle.World == Id && _engine.RailHandles.IsAlive(handle.Index, handle.Generation);

    /// <summary>All live tracks in slot order (allocates the array).</summary>
    public Track[] GetTracks()
    {
        var list = new List<Track>();
        for (int i = 0; i < _engine.TrackHandles.HighWater; i++) { if (_engine.TrackHandles.IsLive(i)) { list.Add(new Track(this, i, _engine.TrackHandles.GenerationOf(i))); } }
        return list.ToArray();
    }

    /// <summary>All live rail vehicles in slot order (allocates the array).</summary>
    public RailVehicle[] GetRailVehicles()
    {
        var list = new List<RailVehicle>();
        for (int i = 0; i < _engine.RailHandles.HighWater; i++) { if (_engine.RailHandles.IsLive(i)) { list.Add(new RailVehicle(this, i, _engine.RailHandles.GenerationOf(i))); } }
        return list.ToArray();
    }
}
