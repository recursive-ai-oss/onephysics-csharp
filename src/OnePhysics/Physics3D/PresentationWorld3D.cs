using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Presentation;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Maps a fracture in the authoritative world to something in the follower. Replaces the default (a burst of shards and the followed fragments). Runs inside <c>Observe</c>.</summary>
public delegate void PresentationFractureHandler3D(PresentationWorld3D follower, PhysicsWorld3D authority, in FractureEvent fracture);

/// <summary>Maps an authoritative event (any kind in <see cref="PresentationWorld3D.MappedEvents"/>) to something in the follower. Runs inside <c>Observe</c>, in the authority's event order.</summary>
public delegate void PresentationEventHandler3D(PresentationWorld3D follower, PhysicsWorld3D authority, in PhysicsEvent e);

/// <summary>
/// A follower world for cosmetics: debris, character cloth and hair, ropes, particles, secondary ragdoll motion. It holds a <see cref="PhysicsWorld3D"/> of its own (<see cref="World"/>, built with the same kernels,
/// open to every normal call) and follows an authoritative world one way: followed bodies appear as kinematic bodies moved to the authoritative poses each step, and the authority's fracture events (and any others the host maps)
/// spawn cosmetic bodies. Nothing in the follower reaches back: the authoritative world is only read, so its bytes and hash are the same with or without a follower.
/// <para>The follower is never part of the authoritative snapshot or hash and is not meant to be sent over a network. It has its own hash (<see cref="ComputeHash"/>), its own recording
/// (<see cref="StartRecording"/>, replayed by <see cref="PresentationPlayer"/> without the authority) and budgets (<see cref="Quality"/>).</para>
/// <para>Per authoritative step: <c>authority.Step(dt)</c>, then <c>follower.Step(authority, dt)</c> (or <c>Observe</c> then <c>Step</c>). The follower steps with callbacks suppressed so that a recording replays exactly; read
/// <c>World.Events</c> after the step for cosmetic reactions and change the world between steps.</para>
/// <para>When the authority rolls back and re-simulates, keep calling <c>Observe</c> and <c>Step</c> with what it now says. A mirror that would have to move faster than <see cref="TeleportSpeed"/> (or turn faster than
/// <see cref="TeleportSpin"/>) in one step is moved with zero velocity instead, so bodies resting on it are not flung; whatever it then overlaps is pushed out at the world's depenetration speed, and after every step
/// no body of type Dynamic in the follower is faster than <see cref="MaxSpeed"/>. This holds for a pose jump of any size; it was tested for rollbacks of up to 120 steps.</para>
/// </summary>
public sealed partial class PresentationWorld3D : IPresentationPlayback
{
    private const double PoseBound = 2097152.0;

    private readonly PhysicsWorld3D _world;
    private PresentationQuality _quality = PresentationQuality.High;
    private PresentationDebris _debris = new();
    private float _maxSpeed = 30f, _teleportSpeed = 30f, _teleportSpin = 30f;
    private PresentationRecorder? _recorder;
    private byte[] _frame = new byte[256];
    private int _frameLength;

    /// <summary>Creates a follower with its own world.</summary>
    /// <param name="world">Options of the follower's world (copied); defaults when null. Sleeping stays on, which suits cosmetics.</param>
    /// <param name="quality">The budgets (copied); <see cref="PresentationQuality.High"/> when null.</param>
    /// <exception cref="ArgumentOutOfRangeException">A world option or a budget is out of range.</exception>
    public PresentationWorld3D(WorldOptions3D? world = null, PresentationQuality? quality = null)
    {
        quality?.Validate();
        _world = new PhysicsWorld3D(world);
        if (quality != null) _quality = quality.Copy();
    }

    /// <summary>The follower's own world. Add hair, cloth, ropes, particles, cosmetic ragdolls and anything else to it with the normal calls; do not step it directly (use <see cref="Step(float)"/>, which also applies the poses and the budgets).</summary>
    public PhysicsWorld3D World => _world;

    /// <summary>Completed follower steps, the world's step count.</summary>
    public long StepCount => _world.StepCount;

    /// <summary>The follower world's hash (the world's own definition). It is independent of the authoritative world and not meant for comparing with it.</summary>
    public ulong ComputeHash() => _world.ComputeHash();

    /// <summary>The follower's budgets; reading returns a copy, assigning copies the value in. Takes effect at the start of the next step.</summary>
    /// <exception cref="ArgumentNullException">Assigning null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range.</exception>
    public PresentationQuality Quality
    {
        get => _quality.Copy();
        set { ArgumentNullException.ThrowIfNull(value); value.Validate(); _quality = value.Copy(); }
    }

    /// <summary>The default fracture response (shards); reading returns a copy, assigning copies the value in.</summary>
    /// <exception cref="ArgumentNullException">Assigning null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range.</exception>
    public PresentationDebris Debris
    {
        get => _debris.Copy();
        set { ArgumentNullException.ThrowIfNull(value); value.Validate(); _debris = value.Copy(); }
    }

    /// <summary>The speed bound in m/s: after every step no body of type Dynamic in the follower is faster (a faster one is scaled down). Above 0, at most 10000. Default 30.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside the range.</exception>
    public float MaxSpeed
    {
        get => _maxSpeed;
        set { if (!(value > 0f) || !(value <= 10000f)) throw new ArgumentOutOfRangeException(nameof(value), value, "MaxSpeed must be above 0 and at most 10000."); _maxSpeed = value; }
    }

    /// <summary>A mirror that would move faster than this in one step (m/s) is teleported with zero velocity instead of being driven. Above 0 and at most the world's <c>MaxLinearSpeed</c>. Default 30, the same as the default <see cref="MaxSpeed"/>, so a driven mirror never pushes anything faster than the speed bound. The step uses the smaller of this and half the world's <c>MaxLinearSpeed</c>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside the range.</exception>
    public float TeleportSpeed
    {
        get => _teleportSpeed;
        set { if (!(value > 0f) || value > _world.Options.MaxLinearSpeed) throw new ArgumentOutOfRangeException(nameof(value), value, "TeleportSpeed must be above 0 and at most the world's MaxLinearSpeed."); _teleportSpeed = value; }
    }

    /// <summary>A mirror that would turn faster than this in one step (rad/s) is teleported with zero velocity. Above 0, at most 1000. Default 30.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside the range.</exception>
    public float TeleportSpin
    {
        get => _teleportSpin;
        set { if (!(value > 0f) || !(value <= 1000f)) throw new ArgumentOutOfRangeException(nameof(value), value, "TeleportSpin must be above 0 and at most 1000."); _teleportSpin = value; }
    }

    /// <summary>Replaces the default fracture response; null (default) restores it.</summary>
    public PresentationFractureHandler3D? FractureMapper { get; set; }

    /// <summary>Called for every authoritative event whose kind is in <see cref="MappedEvents"/>; null (default) calls nothing.</summary>
    public PresentationEventHandler3D? EventMapper { get; set; }

    /// <summary>The event kinds <see cref="EventMapper"/> receives. Default none. The authority's own event mask decides which kinds exist.</summary>
    public EventMask MappedEvents { get; set; }

    /// <summary>Starts recording the follower: the inputs it consumes, replayable without the authoritative world by <see cref="PresentationPlayer"/>. One recorder at a time; dispose it to finish.</summary>
    /// <exception cref="InvalidOperationException">The follower is already being recorded.</exception>
    public PresentationRecorder StartRecording(Stream stream)
    {
        if (_recorder != null) throw new InvalidOperationException("This follower is already being recorded.");
        return new PresentationRecorder(this, stream);
    }

    // ------------------------------------------------------------------ stepping

    /// <summary>Advances the follower one step with the poses gathered by <see cref="Observe"/>: applies the budgets, moves the mirrors, steps the world (callbacks suppressed), bounds the speeds and, when recording, writes the step.</summary>
    /// <param name="dt">Positive finite step length in seconds; use the authority's.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is not positive and finite.</exception>
    public void Step(float dt)
    {
        if (!(dt > 0f) || !float.IsFinite(dt)) throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be positive and finite.");
        Maintain();
        _recorder?.Sync();
        BuildFrame(dt);
        ulong hash = ApplyFrame(_frame.AsSpan(0, _frameLength));
        _recorder?.WriteFrame(_frame.AsSpan(0, _frameLength), hash);
    }

    /// <summary>Reads the authority (<see cref="Observe"/>) and then steps the follower.</summary>
    public void Step(PhysicsWorld3D authority, float dt) { Observe(authority); Step(dt); }

    // The frame: dt, speed bound, pose count, then the poses. The live step and the player both run ApplyFrame on these bytes, so a recording reproduces the step from them alone.
    private void BuildFrame(float dt)
    {
        int count = 0;
        for (int i = 0; i < _mirrors.Count; i++) { var m = _mirrors[i]; if (!m.IsStatic && m.HasTarget) count++; }
        int size = PresentationFormat.FramePrologue + count * PresentationFormat.PoseSize(3);
        if (_frame.Length < size) _frame = new byte[Math.Max(size, _frame.Length * 2)];
        var f = _frame.AsSpan();
        BinaryPrimitives.WriteSingleLittleEndian(f, dt);
        BinaryPrimitives.WriteSingleLittleEndian(f.Slice(4), _maxSpeed);
        BinaryPrimitives.WriteInt32LittleEndian(f.Slice(8), count);
        int at = PresentationFormat.FramePrologue;
        double limit = (double)Math.Min(_teleportSpeed, 0.5f * _world.Options.MaxLinearSpeed) * dt, spin = (double)_teleportSpin * dt;
        for (int i = 0; i < _mirrors.Count; i++)
        {
            var m = _mirrors[i];
            if (m.IsStatic || !m.HasTarget) continue;
            var b = m.Body;
            var cur = b.Position;
            double dx = m.TargetPosition.X - cur.X, dy = m.TargetPosition.Y - cur.Y, dz = m.TargetPosition.Z - cur.Z;
            bool teleport = dx * dx + dy * dy + dz * dz > limit * limit;
            if (!teleport)
            {
                // Rotation jump: |vector part| of target * conjugate(current) is sin(angle/2); twice it is the angle for small turns and still grows to the largest turn.
                var dq = m.TargetRotation * b.Rotation.Conjugated();
                double n2 = (double)dq.X * dq.X + (double)dq.Y * dq.Y + (double)dq.Z * dq.Z;
                double turn = 2.0 * double.Sqrt(n2);
                teleport = turn > spin;
            }

            var h = b.RawHandle;
            BinaryPrimitives.WriteInt32LittleEndian(f.Slice(at), h.Index);
            BinaryPrimitives.WriteInt32LittleEndian(f.Slice(at + 4), h.Generation);
            f[at + 8] = teleport ? (byte)1 : (byte)0;
            BinaryPrimitives.WriteDoubleLittleEndian(f.Slice(at + 9), m.TargetPosition.X);
            BinaryPrimitives.WriteDoubleLittleEndian(f.Slice(at + 17), m.TargetPosition.Y);
            BinaryPrimitives.WriteDoubleLittleEndian(f.Slice(at + 25), m.TargetPosition.Z);
            BinaryPrimitives.WriteSingleLittleEndian(f.Slice(at + 33), m.TargetRotation.X);
            BinaryPrimitives.WriteSingleLittleEndian(f.Slice(at + 37), m.TargetRotation.Y);
            BinaryPrimitives.WriteSingleLittleEndian(f.Slice(at + 41), m.TargetRotation.Z);
            BinaryPrimitives.WriteSingleLittleEndian(f.Slice(at + 45), m.TargetRotation.W);
            at += PresentationFormat.PoseSize(3);
            m.HasTarget = false;
        }

        _frameLength = at;
    }

    /// <summary>Applies one frame: moves or teleports the named kinematic bodies, steps with callbacks suppressed, scales down Dynamic bodies faster than the frame's bound, returns the hash.</summary>
    private ulong ApplyFrame(ReadOnlySpan<byte> frame)
    {
        int poseSize = PresentationFormat.PoseSize(3);
        if (frame.Length < PresentationFormat.FramePrologue) throw new PresentationFrameException("a frame prologue of 12 bytes");
        float dt = BinaryPrimitives.ReadSingleLittleEndian(frame), maxSpeed = BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(4));
        int count = BinaryPrimitives.ReadInt32LittleEndian(frame.Slice(8));
        if (!(dt > 0f) || !float.IsFinite(dt)) throw new PresentationFrameException("a positive finite step length");
        if (!(maxSpeed > 0f) || !float.IsFinite(maxSpeed)) throw new PresentationFrameException("a positive finite speed bound");
        if (count < 0 || (long)count * poseSize != frame.Length - PresentationFormat.FramePrologue) throw new PresentationFrameException("a pose count that matches the frame length");
        var core = _world.Core;
        // Validate every pose before changing anything, so a bad frame leaves the world as it was.
        for (int i = 0, at = PresentationFormat.FramePrologue; i < count; i++, at += poseSize)
        {
            int slot = BinaryPrimitives.ReadInt32LittleEndian(frame.Slice(at)), gen = BinaryPrimitives.ReadInt32LittleEndian(frame.Slice(at + 4));
            if (!core.Bodies.Handles.IsAlive(slot, gen) || core.Bodies.TypeOf(slot) != BodyType.Kinematic) throw new PresentationFrameException("a pose for a live kinematic body (pose " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")");
            if ((frame[at + 8] & ~1) != 0) throw new PresentationFrameException("pose flags 0 or 1");
            for (int k = 0; k < 3; k++)
            {
                double c = BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 9 + 8 * k));
                if (!double.IsFinite(c) || Math.Abs(c) > PoseBound) throw new PresentationFrameException("a finite pose position within 2^21");
            }

            if ((frame[at + 8] & 1) == 0)
            {
                // A driven pose must be reachable within the world's speed limit (the live step flags every larger move as a jump); otherwise the step itself would refuse the target.
                var cur = new Body(_world, core.HandleOfBody(slot)).Position;
                double dx = BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 9)) - cur.X, dy = BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 17)) - cur.Y, dz = BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 25)) - cur.Z;
                double lim = 0.5 * _world.Options.MaxLinearSpeed * dt;
                if (!(dx * dx + dy * dy + dz * dz <= lim * lim)) throw new PresentationFrameException("a pose reachable below half the world's speed limit, or flagged as a jump (pose " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")");
            }

            float qx = BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 33)), qy = BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 37)), qz = BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 41)), qw = BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 45));
            double q2 = (double)qx * qx + (double)qy * qy + (double)qz * qz + (double)qw * qw;
            if (!double.IsFinite(q2) || !(q2 > 1e-12)) throw new PresentationFrameException("a finite nonzero pose rotation");
        }

        for (int i = 0, at = PresentationFormat.FramePrologue; i < count; i++, at += poseSize)
        {
            var body = new Body(_world, core.HandleOfBody(BinaryPrimitives.ReadInt32LittleEndian(frame.Slice(at))));
            var position = new Vector3D(BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 9)), BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 17)), BinaryPrimitives.ReadDoubleLittleEndian(frame.Slice(at + 25)));
            var rotation = new QuaternionF(BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 33)), BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 37)), BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 41)), BinaryPrimitives.ReadSingleLittleEndian(frame.Slice(at + 45)));
            if ((frame[at + 8] & 1) != 0)
            {
                // A jump: place the mirror and stop it, so nothing that touches it is thrown by an invented velocity.
                body.SetTransform(position, rotation);
                body.SetLinearVelocity(default);
                body.SetAngularVelocity(default);
            }
            else
            {
                body.MoveKinematic(position, rotation);
            }
        }

        using (_world.SuppressCallbacks()) { _world.Step(dt); }
        BoundSpeeds(maxSpeed);
        return _world.ComputeHash();
    }

    // Speed bound: scale a faster Dynamic body down to the bound along its direction (IEEE-exact: one square root, one division).
    private void BoundSpeeds(float maxSpeed)
    {
        var bodies = _world.Core.Bodies;
        float limit2 = maxSpeed * maxSpeed;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsDynamic(b)) continue;
            var v = bodies.LinearVelocityOf(b);
            float n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (n2 <= limit2) continue;
            if (!float.IsFinite(n2)) { v[0] = 0f; v[1] = 0f; v[2] = 0f; continue; }
            float s = maxSpeed / float.Sqrt(n2);
            v[0] *= s; v[1] *= s; v[2] *= s;
            // Rounding can leave the scaled speed a hair above the bound; one more pass makes the bound hold exactly.
            float m2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (m2 > limit2) { float t = maxSpeed / float.Sqrt(m2); v[0] *= t; v[1] *= t; v[2] *= t; }
        }
    }

    // ------------------------------------------------------------------ recording and playback

    int IPresentationHost.Dimension => 3;
    long IPresentationHost.StepCount => _world.StepCount;
    ulong IPresentationHost.Hash() => _world.ComputeHash();
    byte[] IPresentationHost.Snapshot() => _world.Snapshot();
    int IPresentationHost.OptionsSize => ReplayOptions3D.Size;
    void IPresentationHost.WriteOptions(Span<byte> bytes) => ((IRecordedWorld)_world).WriteReplayOptions(bytes);
    void IPresentationHost.AttachRecorder(PresentationRecorder? recorder) => _recorder = recorder;

    void IPresentationPlayback.RestoreRecorded(ReadOnlySpan<byte> options, ReadOnlySpan<byte> snapshot)
    {
        _world.Restore(snapshot);
        ((IRecordedWorld)_world).ApplyReplayOptions(options);
    }

    ulong IPresentationPlayback.ApplyFrame(ReadOnlySpan<byte> frame) => ApplyFrame(frame);
}
