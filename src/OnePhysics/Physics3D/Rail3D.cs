using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A railway track in a 3D world (a <see cref="TrackDef"/> laid out in place). Like <see cref="Body"/> it is a small struct handle (world, slot, generation) that allocates nothing and fails with a
/// <see cref="StaleHandleException"/> once the track is destroyed. With <see cref="TrackDef.Collide"/> the track owns two static bodies, the ballast bed and the rails, so other bodies rest on and hit it.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct Track : IEquatable<Track>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Track(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    internal int Slot => _index;

    internal TrackState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Track is not attached to a world."); }
        var e = _world.Engine;
        if (!e.TrackHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Track(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the track was destroyed.");
        }

        return e.Tracks[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld3D.GetTrack"/> turns it back into the track.</summary>
    public TrackHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the track exists.</summary>
    public bool IsValid => _world != null && _world.Engine.TrackHandles.IsAlive(_index, _generation);

    /// <summary>A copy of the definition (with the current origin applied to <see cref="TrackDef.Start"/>).</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public TrackDef Definition => Resolve().Def.Clone();

    /// <summary>Length of the plan view, m.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public double Length => Resolve().Geo.Length;

    /// <summary>The frame of the track at chainage <paramref name="s"/> (beyond the ends the track continues straight).</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public TrackFrame FrameAt(double s) => Resolve().Geo.FrameAt(s);

    /// <summary>
    /// Adds a switch (points) at chainage <paramref name="s"/> with a branch track that leaves tangentially from the main track there (the branch's own start and heading are ignored: it starts at the main track's pose).
    /// Wheelsets running forward past the point follow the branch while the switch is set to diverging; wheelsets that run back off the start of the branch rejoin the main track whatever the setting.
    /// </summary>
    /// <param name="s">Chainage of the switch on this track.</param>
    /// <param name="branch">The branch's plan view, profile and cant (copied).</param>
    /// <param name="diverging">The initial setting. Default false: the main line.</param>
    /// <exception cref="ArgumentOutOfRangeException">The chainage lies outside the track or the branch definition is out of range.</exception>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public Switch AddSwitch(double s, TrackDef branch, bool diverging = false) => _world!.AddSwitchCore(this, s, branch, diverging);

    /// <summary>Number of switches on this track.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public int SwitchCount => Resolve().Switches.Length;

    /// <summary>Switch number <paramref name="index"/> (in the order they were added).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is out of range.</exception>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public Switch GetSwitch(int index)
    {
        var t = Resolve();
        if ((uint)index >= (uint)t.Switches.Length) { throw new ArgumentOutOfRangeException(nameof(index), index, "The switch index must be within 0.." + (t.Switches.Length - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        return new Switch(_world!, this, index);
    }

    /// <summary>Removes the track and its static bodies. Rail vehicles that run on it lose their rails.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("Track.Destroy");
        var t = Resolve();
        var w = _world!;
        if (t.Ballast >= 0 && w.Core.Bodies.Handles.IsAlive(t.Ballast, t.BallastGen)) { new Body(w, new BodyHandle(w.Id, t.Ballast, t.BallastGen)).Destroy(); }
        if (t.Rails >= 0 && w.Core.Bodies.Handles.IsAlive(t.Rails, t.RailsGen)) { new Body(w, new BodyHandle(w.Id, t.Rails, t.RailsGen)).Destroy(); }
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

/// <summary>A switch on a <see cref="Track"/>: where a branch leaves, and which way the points are set.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct Switch
{
    private readonly PhysicsWorld3D _world;
    private readonly Track _track;
    private readonly int _index;

    internal Switch(PhysicsWorld3D world, Track track, int index) { _world = world; _track = track; _index = index; }

    private SwitchState State => _track.Resolve().Switches[_index];

    /// <summary>The main track.</summary>
    public Track Main => _track;

    /// <summary>Chainage of the switch on the main track.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public double Chainage => State.S;

    /// <summary>The branch track.</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public Track Branch { get { var s = State; return new Track(_world, s.Branch, s.BranchGen); } }

    /// <summary>True when wheelsets running forward past the switch take the branch. Setting it is a world mutation (recorded, snapshotted).</summary>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public bool Diverging
    {
        get => State.Diverging;
        set { using var replayMutation = _world.RecordMutation("Switch.Diverging"); _ = _track.Resolve(); _world.Engine.Tracks[_track.Slot]!.Switches[_index].Diverging = value; }
    }
}

/// <summary>The state of one wheelset of a rail vehicle after the last step.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct WheelsetState
{
    internal WheelsetState(Body body, Track track, double chainage, float spin, float radius, float normalLeft, float normalRight, float longitudinalLeft, float longitudinalRight, float lateralLeft, float lateralRight, float flangeLeft, float flangeRight, float climb, byte flags, float qLeft, float qRight, float yLeft, float yRight)
    {
        VerticalLoadLeft = qLeft; VerticalLoadRight = qRight; LateralLoadLeft = yLeft; LateralLoadRight = yRight;
        Body = body; Track = track; Chainage = chainage; SpinSpeed = spin; Radius = radius; NormalForceLeft = normalLeft; NormalForceRight = normalRight;
        LongitudinalForceLeft = longitudinalLeft; LongitudinalForceRight = longitudinalRight; LateralForceLeft = lateralLeft; LateralForceRight = lateralRight;
        FlangeForceLeft = flangeLeft; FlangeForceRight = flangeRight; ClimbRatio = climb; _flags = flags;
    }

    private readonly byte _flags;

    /// <summary>The wheelset's body.</summary>
    public Body Body { get; }

    /// <summary>The track the wheelset is running on (invalid when it has no track).</summary>
    public Track Track { get; }

    /// <summary>Chainage of the wheelset on its track, m.</summary>
    public double Chainage { get; }

    /// <summary>Spin speed, rad/s; positive when the wheelset rolls forward.</summary>
    public float SpinSpeed { get; }

    /// <summary>Wheel radius at the tread's nominal point, m.</summary>
    public float Radius { get; }

    /// <summary>Wheel-rail normal force on the left tread, N (0 when it is not touching).</summary>
    public float NormalForceLeft { get; }

    /// <summary>Wheel-rail normal force on the right tread, N.</summary>
    public float NormalForceRight { get; }

    /// <summary>Longitudinal creep force on the left wheel, N, positive forward.</summary>
    public float LongitudinalForceLeft { get; }

    /// <summary>Longitudinal creep force on the right wheel, N.</summary>
    public float LongitudinalForceRight { get; }

    /// <summary>Lateral creep force on the left wheel, N, positive to the right.</summary>
    public float LateralForceLeft { get; }

    /// <summary>Lateral creep force on the right wheel, N.</summary>
    public float LateralForceRight { get; }

    /// <summary>Normal force at the left flange, N (0 when the flange does not touch).</summary>
    public float FlangeForceLeft { get; }

    /// <summary>Normal force at the right flange, N.</summary>
    public float FlangeForceRight { get; }

    /// <summary>The larger ratio of lateral to vertical force on a wheel whose flange touches (Y/Q), 0 without flange contact. Compare it with <see cref="PhysicsWorld3D.NadalLimit"/>: above it the wheel climbs.</summary>
    public float ClimbRatio { get; }

    /// <summary>Everything the left rail does to the left wheel (tread and flange, normal and creep forces) along the track's up axis, N: the wheel's vertical load Q.</summary>
    public float VerticalLoadLeft { get; }

    /// <summary>The same for the right wheel, N.</summary>
    public float VerticalLoadRight { get; }

    /// <summary>Everything the left rail does to the left wheel along the track's right axis, N: the wheel's lateral force Y (positive to the right).</summary>
    public float LateralLoadLeft { get; }

    /// <summary>The same for the right wheel, N.</summary>
    public float LateralLoadRight { get; }

    /// <summary>True while the left tread touches its rail.</summary>
    public bool LeftTreadContact => (_flags & 1) != 0;

    /// <summary>True while the right tread touches its rail.</summary>
    public bool RightTreadContact => (_flags & 2) != 0;

    /// <summary>True while the left flange touches the rail's gauge face.</summary>
    public bool LeftFlangeContact => (_flags & 4) != 0;

    /// <summary>True while the right flange touches the rail's gauge face.</summary>
    public bool RightFlangeContact => (_flags & 8) != 0;

    /// <summary>True when no tread touches a rail: the wheelset has left the track.</summary>
    public bool IsDerailed => (_flags & 3) == 0;
}

/// <summary>
/// A rail vehicle: a car (or a bare wheelset), its bogies and wheelsets as rigid bodies joined by suspension springs, and its couplers. Like <see cref="Body"/> it is a small struct handle (world, slot,
/// generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the vehicle is destroyed (its car or, for a bare wheelset, the wheelset's body destroyed, or <see cref="Destroy"/> called).
/// </summary>
/// <example><c>var track = world.AddTrack(TrackDef.Curve(300, 200, 60, 0.1f)); var loco = world.AddRailVehicle(track, 30, RailDef.Locomotive); loco.Drive(throttle: 0.5f); world.Step(1f / 60f);</c></example>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct RailVehicle : IEquatable<RailVehicle>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal RailVehicle(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private RailState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default RailVehicle is not attached to a world."); }
        var e = _world.Engine;
        if (!e.RailHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("RailVehicle(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the vehicle was destroyed.");
        }

        return e.Rails[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld3D.GetRailVehicle"/> turns it back into the vehicle.</summary>
    public RailVehicleHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the vehicle exists.</summary>
    public bool IsValid => _world != null && _world.Engine.RailHandles.IsAlive(_index, _generation);

    /// <summary>The car body (for a bare wheelset, the wheelset's body).</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public Body Car { get { var s = Resolve(); int b = s.Car >= 0 ? s.Car : s.Ws[0]; int g = s.Car >= 0 ? s.CarGen : s.WsGen[0]; return new Body(_world!, new BodyHandle(_world!.Id, b, g)); } }

    /// <summary>Number of wheelsets.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public int WheelsetCount => Resolve().Wheelsets;

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
    public float Speed
    {
        get
        {
            var c = Car;
            var q = c.Rotation.Rotate(new Vector3F(0f, 0f, 1f));
            var v = c.LinearVelocity;
            return v.X * q.X + v.Y * q.Y + v.Z * q.Z;
        }
    }

    /// <summary>The state of wheelset <paramref name="index"/> after the last step.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside 0..<see cref="WheelsetCount"/>-1.</exception>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public WheelsetState GetWheelset(int index)
    {
        var s = Resolve();
        if ((uint)index >= (uint)s.Wheelsets) { throw new ArgumentOutOfRangeException(nameof(index), index, "The wheelset index must be within 0.." + (s.Wheelsets - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        var w = _world!;
        Track track = s.Trk[index] >= 0 && w.Engine.TrackHandles.IsAlive(s.Trk[index], s.TrkGen[index]) ? new Track(w, s.Trk[index], s.TrkGen[index]) : default;
        return new WheelsetState(new Body(w, new BodyHandle(w.Id, s.Ws[index], s.WsGen[index])), track, s.S[index], s.Spin[index], s.Def.WheelRadius, s.NormalL[index], s.NormalR[index], s.FxL[index], s.FxR[index],
            s.FyL[index], s.FyR[index], s.FlangeL[index], s.FlangeR[index], s.Climb[index], s.Flags[index], s.QL[index], s.QR[index], s.YL[index], s.YR[index]);
    }

    /// <summary>
    /// Couples this vehicle's <paramref name="thisFront"/> end to <paramref name="other"/>'s <paramref name="otherFront"/> end (front is the +Z end of the car). The couplers are buffers and a draw gear with slack, from the
    /// definition of this vehicle; the draw gear breaks at its rating. A coupler carries nothing until the vehicles come within its reach.
    /// </summary>
    /// <exception cref="PhysicsException">An end is already coupled, or a vehicle has no car (a bare wheelset).</exception>
    /// <exception cref="ArgumentException">The other vehicle belongs to another world or is this vehicle.</exception>
    /// <exception cref="StaleHandleException">A vehicle was destroyed.</exception>
    public void Couple(RailVehicle other, bool thisFront = true, bool otherFront = false)
    {
        using var replayMutation = _world?.RecordMutation("RailVehicle.Couple");
        var s = Resolve(); var o = other.Resolve();
        if (!ReferenceEquals(_world, other._world) || (_index == other._index)) { throw new ArgumentException("The other vehicle must be another vehicle of the same world.", nameof(other)); }
        if (s.Car < 0 || o.Car < 0) { throw new PhysicsException("A bare wheelset has no couplers."); }
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

    /// <summary>Force in the coupler at the front or rear end as of the last step, N: positive in tension (the draw gear pulls), negative when the buffers push; 0 when uncoupled or broken.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public float CouplerForce(bool front) => Resolve().CouplerForce[front ? 0 : 1];

    /// <summary>True when the coupler at that end was coupled and its draw gear has broken.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public bool IsCouplerBroken(bool front) => Resolve().Broken[front ? 0 : 1] != 0;

    /// <summary>True when every wheelset still touches the rails with a tread.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed.</exception>
    public bool IsOnTrack { get { var s = Resolve(); for (int k = 0; k < s.Wheelsets; k++) { if ((s.Flags[k] & 3) == 0) { return false; } } return true; } }

    /// <summary>Removes the vehicle with its car, bogies and wheelsets (their bodies and joints). Handles to them become stale.</summary>
    /// <exception cref="StaleHandleException">The vehicle was destroyed already.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("RailVehicle.Destroy");
        var s = Resolve();
        var w = _world!;
        var bodies = w.Core.Bodies;
        void Kill(int b, int g) { if (bodies.Handles.IsAlive(b, g)) { new Body(w, new BodyHandle(w.Id, b, g)).Destroy(); } }
        if (s.Car >= 0) { Kill(s.Car, s.CarGen); }
        for (int k = 0; k < s.Bogie.Length; k++) { Kill(s.Bogie[k], s.BogieGen[k]); }
        for (int k = 0; k < s.Wheelsets; k++) { Kill(s.Ws[k], s.WsGen[k]); }
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

public sealed partial class PhysicsWorld3D
{
    /// <summary>Collision category of the rails' static body: wheelsets do not collide with it (their contact with the rails is the wheel-rail model) while everything else does.</summary>
    internal const uint RailCategory = 1u << 30;

    /// <summary>
    /// The ratio of lateral to vertical force at a flange contact above which a wheel climbs the rail (Nadal 1908): <c>(tan α − μ) / (1 + μ tan α)</c> for flange angle α and wheel-rail friction μ.
    /// </summary>
    /// <param name="flangeAngle">Angle of the flange face from the horizontal, radians.</param>
    /// <param name="friction">Wheel-rail friction coefficient.</param>
    public static float NadalLimit(float flangeAngle, float friction)
    {
        float tan = Math32.Tan(flangeAngle);
        return (tan - friction) / (1f + friction * tan);
    }

    /// <summary>Lays out a track in this world (see <see cref="Track"/>); with <see cref="TrackDef.Collide"/> it also builds the track's static collision bodies.</summary>
    /// <param name="def">The track; copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range.</exception>
    public Track AddTrack(TrackDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddTrack");
        ArgumentNullException.ThrowIfNull(def);
        return CreateTrackCore(def.Clone(), null, 0.0);
    }

    private Track CreateTrackCore(TrackDef def, Track? parent, double parentS)
    {
        def.Validate();
        var state = new TrackState { Def = def, Geo = new TrackGeometry(def) };
        if (parent != null) { state.Parent = parent.Value.Slot; state.ParentGen = _engine.TrackHandles.GenerationOf(parent.Value.Slot); state.ParentS = parentS; }
        if (def.Collide) { BuildTrackBodies(state); }
        int i = _engine.CreateTrack(state);
        return new Track(this, i, _engine.TrackHandles.GenerationOf(i));
    }

    internal Switch AddSwitchCore(Track main, double s, TrackDef branch, bool diverging)
    {
        using var replayMutation = RecordMutation("Track.AddSwitch");
        ArgumentNullException.ThrowIfNull(branch);
        var m = main.Resolve();
        if (!(s >= 0.0 && s <= m.Geo.Length)) { throw new ArgumentOutOfRangeException(nameof(s), s, "The switch must lie on the track (0.." + m.Geo.Length.ToString("R", CultureInfo.InvariantCulture) + " m)."); }
        if (m.Switches.Length >= 256) { throw new PhysicsException("A track has at most 256 switches."); }
        var d = branch.Clone();
        var f = m.Geo.FrameAt(s);
        d.Start = f.Position;
        d.StartHeading = (float)Math64.Atan2(f.Tangent.X, f.Tangent.Z);
        // The branch's height and cant profiles are relative to the main line's height at the switch.
        var br = CreateTrackCore(d, main, s);
        var list = new SwitchState[m.Switches.Length + 1];
        Array.Copy(m.Switches, list, m.Switches.Length);
        list[^1] = new SwitchState { S = s, Branch = br.Slot, BranchGen = _engine.TrackHandles.GenerationOf(br.Slot), Diverging = diverging };
        m.Switches = list;
        return new Switch(this, main, list.Length - 1);
    }

    // The ballast bed and the two rails as static triangle meshes, sampled every two metres along the track.
    private void BuildTrackBodies(TrackState t)
    {
        double len = t.Geo.Length;
        int n = (int)Math.Ceiling(len / 2.0) + 1;
        float br = 0.5f * (t.Def.Gauge + Engine3D.RailHeadWidth);
        var start = t.Def.Start;
        var offs = new Vector3F[n]; var rs = new Vector3F[n]; var us = new Vector3F[n];
        for (int i = 0; i < n; i++)
        {
            var f = t.Geo.FrameAt(len * i / (n - 1));
            offs[i] = new Vector3F((float)(f.Position.X - start.X), (float)(f.Position.Y - start.Y), (float)(f.Position.Z - start.Z));
            rs[i] = f.Right; us[i] = f.Up;
        }

        Vector3F P(int i, float lateral, float down) => offs[i] + rs[i] * lateral - us[i] * down;
        // Ballast: a strip 3.2 m wide, its top 0.18 m below the rail heads. Rails: the heads' tops and both sides down to the bed.
        var bv = new List<Vector3F>(n * 2); var bi = new List<int>();
        for (int i = 0; i < n; i++) { bv.Add(P(i, -1.6f, 0.18f)); bv.Add(P(i, 1.6f, 0.18f)); }
        for (int i = 0; i + 1 < n; i++) { int l0 = 2 * i, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3; bi.Add(l0); bi.Add(l1); bi.Add(r0); bi.Add(r0); bi.Add(l1); bi.Add(r1); }
        var rv = new List<Vector3F>(n * 8); var ri = new List<int>();
        for (int i = 0; i < n; i++)
        {
            foreach (float c in new[] { -br, br })
            {
                float a = c - 0.035f, b = c + 0.035f;
                rv.Add(P(i, a, 0f)); rv.Add(P(i, b, 0f)); rv.Add(P(i, a, 0.18f)); rv.Add(P(i, b, 0.18f));
            }
        }

        for (int i = 0; i + 1 < n; i++)
        {
            for (int rail = 0; rail < 2; rail++)
            {
                int o0 = 8 * i + 4 * rail, o1 = 8 * (i + 1) + 4 * rail;   // TA, TB, BA, BB of station i and i+1
                int ta0 = o0, tb0 = o0 + 1, ba0 = o0 + 2, bb0 = o0 + 3, ta1 = o1, tb1 = o1 + 1, ba1 = o1 + 2, bb1 = o1 + 3;
                ri.Add(ta0); ri.Add(ta1); ri.Add(tb0); ri.Add(tb0); ri.Add(ta1); ri.Add(tb1);       // top
                ri.Add(tb0); ri.Add(tb1); ri.Add(bb0); ri.Add(bb0); ri.Add(tb1); ri.Add(bb1);       // right face
                ri.Add(ta0); ri.Add(ba0); ri.Add(ta1); ri.Add(ba0); ri.Add(ba1); ri.Add(ta1);       // left face
            }
        }

        var ballastBody = CreateBody(BodyType.Static, start, QuaternionF.Identity);
        try
        {
            CreateMesh(ballastBody, bv.ToArray(), bi.ToArray(), new ShapeDef { Friction = 0.6f });
            t.Ballast = ballastBody.Handle.Index; t.BallastGen = ballastBody.Handle.Generation;
            var railBody = CreateBody(BodyType.Static, start, QuaternionF.Identity);
            try
            {
                CreateMesh(railBody, rv.ToArray(), ri.ToArray(), new ShapeDef { Friction = 0.4f, CategoryBits = RailCategory });
                t.Rails = railBody.Handle.Index; t.RailsGen = railBody.Handle.Generation;
            }
            catch (Exception) { railBody.Destroy(); throw; }
        }
        catch (Exception) { if (ballastBody.IsValid) { ballastBody.Destroy(); } t.Ballast = -1; t.Rails = -1; throw; }
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

    private static QuaternionF FrameRotation(TrackFrame f)
    {
        var q = QuaternionF.CreateFromRotationMatrix(new Matrix3x3F(f.Right.X, f.Up.X, f.Tangent.X, f.Right.Y, f.Up.Y, f.Tangent.Y, f.Right.Z, f.Up.Z, f.Tangent.Z));
        return q.Normalized();
    }

    /// <summary>
    /// Puts a rail vehicle on a track with its centre at chainage <paramref name="chainage"/>, heading along the track, standing on its wheelsets (a bare wheelset, <see cref="RailDef.Wheelset"/>, is one body).
    /// Creates the car, the bogies and the wheelsets as bodies and joins them with the suspension springs. Rail vehicles run once per step, before the vehicles and the solver.
    /// </summary>
    /// <param name="track">The track the vehicle's wheelsets are on.</param>
    /// <param name="chainage">Chainage of the vehicle's centre.</param>
    /// <param name="def">The vehicle, for example <see cref="RailDef.Locomotive"/>; copied.</param>
    /// <param name="speed">Initial speed along the track, m/s (negative runs backwards); the wheelsets start rolling at it. Default 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">A definition value or the speed is out of range.</exception>
    /// <exception cref="StaleHandleException">The track was destroyed.</exception>
    public RailVehicle AddRailVehicle(Track track, double chainage, RailDef def, float speed = 0f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddRailVehicle");
        ArgumentNullException.ThrowIfNull(def);
        def = def.Clone();
        def.Validate();
        var ts = track.Resolve();
        if (def.CarMass == 0f && !(def.Bogies == 0 && def.AxlesPerBogie == 1)) { throw new ArgumentOutOfRangeException(nameof(def), "A vehicle without a car (CarMass 0) must be a single wheelset."); }
        if (!(Math.Abs(speed) <= 500f)) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speed must be finite and within ±500 m/s."); }
        if (!(Math.Abs(chainage) <= 1e5)) { throw new ArgumentOutOfRangeException(nameof(chainage), chainage, "The chainage must be finite and within ±100000 m."); }
        var geo = ts.Geo;
        var created = new List<Body>();
        var r = new RailState { Def = def };
        r.Allocate(def.WheelsetCount);
        int slot = _engine.CreateRailVehicle(r);
        try
        {
            int group = -(slot + 1);
            // Positions along the track.
            int nw = def.WheelsetCount, nb = def.Bogies;
            var wsS = new double[nw]; var bogS = new double[nb];
            int a = def.AxlesPerBogie;
            if (nb == 0) { for (int k = 0; k < nw; k++) { wsS[k] = chainage + (k - 0.5 * (a - 1)) * def.AxleSpacing; } }
            else
            {
                for (int g = 0; g < nb; g++)
                {
                    bogS[g] = chainage + (g == 0 ? -0.5 : 0.5) * def.BogieSpacing;
                    for (int k = 0; k < a; k++) { wsS[g * a + k] = bogS[g] + (k - 0.5 * (a - 1)) * def.AxleSpacing; }
                }
            }

            Vector3D Lift(TrackFrame f, float height) => new(f.Position.X + f.Up.X * height, f.Position.Y + f.Up.Y * height, f.Position.Z + f.Up.Z * height);
            Body car = default;
            float carRoll = 0f;
            var fc = geo.FrameAt(chainage);
            if (def.CarMass > 0f)
            {
                float vol = def.CarSize.X * def.CarSize.Y * def.CarSize.Z;
                car = CreateBody(BodyType.Dynamic, Lift(fc, def.CarCenterHeight), FrameRotation(fc));
                created.Add(car);
                float dy = def.CarFloorHeight + 0.5f * def.CarSize.Y - def.CarCenterHeight;   // the box's centre relative to the centre of mass
                CreateBox(car, def.CarSize, 0f, new ShapeDef { Density = def.CarMass / vol, Friction = 0.5f, GroupIndex = group, LocalPosition = new Vector3F(0f, dy, 0f) });
                car.CenterOfMass = new Vector3F(0f, 0f, 0f);   // the body's origin is its centre of mass, whatever the box's offset
                car.Mass = def.CarMass;
                float sx = def.CarSize.X, sy = def.CarSize.Y, sz = def.CarSize.Z, m = def.CarMass;
                carRoll = m * (sx * sx + sy * sy) / 12f + m * dy * dy;
                car.SetInertia(new Vector3F(m * (sy * sy + sz * sz) / 12f + m * dy * dy, m * (sx * sx + sz * sz) / 12f, carRoll), QuaternionF.Identity);
                r.Car = car.Handle.Index; r.CarGen = car.Handle.Generation;
            }

            var bogies = new Body[nb];
            for (int g = 0; g < nb; g++)
            {
                var fb = geo.FrameAt(bogS[g]);
                float bogH = def.WheelRadius + 0.2f;
                var b = AddEmpty(Lift(fb, bogH), def.BogieMass, def.BogieInertia);
                b.SetTransform(Lift(fb, bogH), FrameRotation(fb));
                created.Add(b); bogies[g] = b;
                r.Bogie = Append(r.Bogie, b.Handle.Index); r.BogieGen = Append(r.BogieGen, b.Handle.Generation);
            }

            var wheelsets = new Body[nw];
            for (int k = 0; k < nw; k++)
            {
                var fw = geo.FrameAt(wsS[k]);
                var pos = Lift(fw, def.WheelRadius);
                var body = CreateBody(BodyType.Dynamic, pos, FrameRotation(fw));
                created.Add(body);
                var sd = new ShapeDef { Density = 1f, Friction = 0.5f, GroupIndex = group, MaskBits = ~RailCategory, LocalRotation = QuaternionF.CreateFromAxisSinCos(new Vector3F(0f, 0f, 1f), -0.70710677f, 0.70710677f) };
                CreateCylinder(body, 0.5f * def.TreadSpan + 0.07f, def.WheelRadius, 0f, sd);
                body.Mass = def.WheelsetMass;
                body.SetInertia(new Vector3F(def.WheelsetSpinInertia, def.WheelsetTransverseInertia, def.WheelsetTransverseInertia), QuaternionF.Identity);
                wheelsets[k] = body;
                r.Ws[k] = body.Handle.Index; r.WsGen[k] = body.Handle.Generation; r.Trk[k] = track.Slot; r.TrkGen[k] = _engine.TrackHandles.GenerationOf(track.Slot); r.S[k] = wsS[k];
                r.Spin[k] = speed / def.WheelRadius;
            }

            // Suspension: six-degree-of-freedom spring joints, bogie to car and wheelset to bogie (or to the car).
            float carried = (def.CarMass + nb * def.BogieMass) / nw;
            for (int g = 0; g < nb && def.CarMass > 0f; g++)
            {
                var fb = geo.FrameAt(bogS[g]);
                float meff = 1f / (1f / def.CarMass + 1f / def.BogieMass);
                float lift = def.WheelRadius + 0.2f;
                AddSuspension(car, bogies[g], Lift(fb, lift), FrameRotation(fb), def.SecondaryHertz, def.SecondaryAngularHertz, def.SecondaryDamping, def.CarMass / nb, meff, carRoll / nb, def.BogieInertia.Z, carRoll, def.BogieInertia.Z);
            }

            for (int k = 0; k < nw; k++)
            {
                var fw = geo.FrameAt(wsS[k]);
                Body parent = nb > 0 ? bogies[k / a] : car;
                if (parent.Equals(default(Body))) { break; }
                float mp = nb > 0 ? def.BogieMass : def.CarMass;
                float meff = 1f / (1f / mp + 1f / def.WheelsetMass);
                AddSuspension(parent, wheelsets[k], Lift(fw, def.WheelRadius), FrameRotation(fw), def.PrimaryHertz, def.PrimaryAngularHertz, def.PrimaryDamping, carried, meff, nb > 0 ? 0f : carRoll / nw, nb > 0 ? def.BogieInertia.Z : carRoll, def.WheelsetTransverseInertia, 0f);
            }

            // Everything starts moving along the track at the given speed.
            SetInitialVelocity(created, car, bogies, wheelsets, bogS, wsS, geo, chainage, speed);
            if (def.CarMass <= 0f) { r.Car = -1; }
            _engine.RegisterRailVehicle(slot);
            return new RailVehicle(this, slot, _engine.RailHandles.GenerationOf(slot));
        }
        catch (Exception)
        {
            foreach (var b in created) { if (b.IsValid) { b.Destroy(); } }
            if (_engine.RailHandles.IsAlive(slot, _engine.RailHandles.GenerationOf(slot))) { _engine.DestroyRailVehicle(slot); }
            throw;
        }
    }

    private static int[] Append(int[] a, int v) { var b = new int[a.Length + 1]; Array.Copy(a, b, a.Length); b[^1] = v; return b; }

    private static void SetInitialVelocity(List<Body> created, Body car, Body[] bogies, Body[] wheelsets, double[] bogS, double[] wsS, TrackGeometry geo, double chainage, float speed)
    {
        if (speed == 0f) { return; }
        if (car.IsValid) { var t = geo.FrameAt(chainage).Tangent; car.LinearVelocity = t * speed; }
        for (int g = 0; g < bogies.Length; g++) { bogies[g].LinearVelocity = geo.FrameAt(bogS[g]).Tangent * speed; }
        for (int k = 0; k < wheelsets.Length; k++) { wheelsets[k].LinearVelocity = geo.FrameAt(wsS[k]).Tangent * speed; }
    }

    // A spring joint between parent and child at the world anchor, frame axes lateral (x), vertical (y), longitudinal (z).
    private void AddSuspension(Body parent, Body child, Vector3D anchor, QuaternionF frame, Vector3F hertz, Vector3F angularHertz, float damping, float supported, float meff, float rollSupported, float rollParent, float rollChild, float unused)
    {
        _ = unused;
        // The roll of the car on its springs has no restoring stiffness from the point springs: its frequency is that of the car's own roll inertia (the share this joint carries), against the pair's effective inertia.
        float rollEff = 1f / (1f / rollParent + 1f / rollChild);
        float rollBoost = rollSupported > rollEff ? float.Sqrt(rollSupported / rollEff) : 1f;
        angularHertz = new Vector3F(angularHertz.X, angularHertz.Y, angularHertz.Z * rollBoost);
        float boost = float.Sqrt(Math.Max(supported, meff) / meff);   // the joint's frequency is against the pair's effective mass, ours is against the mass the spring carries
        var def = new SixDofJointDef
        {
            BodyA = parent, BodyB = child, LocalFrameA = FrameOnPublic(parent, anchor, frame), LocalFrameB = FrameOnPublic(child, anchor, frame),
            LinearX = JointAxisMode.Free, LinearY = JointAxisMode.Free, LinearZ = JointAxisMode.Free, AngularX = JointAxisMode.Free, AngularY = JointAxisMode.Free, AngularZ = JointAxisMode.Free,
            SpringAxes = JointAxes.LinearX | JointAxes.LinearY | JointAxes.LinearZ | (angularHertz.X > 0f ? JointAxes.AngularX : 0) | (angularHertz.Y > 0f ? JointAxes.AngularY : 0) | (angularHertz.Z > 0f ? JointAxes.AngularZ : 0),
            SpringHertz = hertz * boost, SpringDampingRatio = new Vector3F(damping, damping, damping), AngularSpringHertz = angularHertz, AngularSpringDampingRatio = new Vector3F(damping, damping, damping),
        };
        CreateJoint(in def);
    }

    private static JointFrame3D FrameOnPublic(Body body, Vector3D anchor, QuaternionF worldFrame)
    {
        var rel = new Vector3D(anchor.X - body.Position.X, anchor.Y - body.Position.Y, anchor.Z - body.Position.Z);
        var inv = body.Rotation.Conjugated();
        var local = inv.Rotate(new Vector3F((float)rel.X, (float)rel.Y, (float)rel.Z));
        return new JointFrame3D(local, inv * worldFrame);
    }
}
