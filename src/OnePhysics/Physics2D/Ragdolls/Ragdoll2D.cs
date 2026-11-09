using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A ragdoll: a skeleton of bodies joined by revolute joints, made by <see cref="PhysicsWorld2D.AddRagdoll(RagdollDef, Vector2D, float, ReadOnlySpan{float})"/>. Like <see cref="Body"/> it is a small struct handle
/// (world, slot, generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the ragdoll is removed; the state lives in the world and in its snapshots.
/// </summary>
/// <remarks>
/// <para>Limp by default: the joints hold the parts together inside their limits and nothing else. <see cref="DriveToPose"/> turns on a motor at every joint that turns the part towards a pose with a torque limit,
/// so a strong enough ragdoll stands, takes a knock and returns to the pose; <see cref="StopDriving"/> lets go. <see cref="SetPose"/> teleports the whole figure and <see cref="MoveToPose"/> carries it there by velocity (a keyframed mode).
/// A joint with a break threshold (<see cref="RagdollPartDef.BreakForce"/>, <see cref="RagdollPartDef.BreakTorque"/>) breaks under load and the part comes off; the ragdoll stays valid and <see cref="IsAttached"/> reports it.</para>
/// <para>A pose is one angle per part, radians. Entry <c>i</c> is the joint angle of part <c>i</c>: its angle minus its parent's, minus the same in the bind pose (0 everywhere is the bind pose; positive is counter-clockwise).
/// Entry 0, the root, turns the whole figure in <see cref="SetPose"/> and <see cref="MoveToPose"/> and is ignored by <see cref="DriveToPose"/>.</para>
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly partial struct Ragdoll : IEquatable<Ragdoll>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Ragdoll(PhysicsWorld2D world, int index, int generation)
    {
        _world = world;
        _index = index;
        _generation = generation;
    }

    private RagdollState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Ragdoll is not attached to a world."); }
        var e = _world.Engine;
        if (!e.RagHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Ragdoll(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the ragdoll was removed.");
        }

        return e.Ragdolls[_index]!;
    }

    /// <summary>The raw handle (world id, slot, generation); <see cref="PhysicsWorld2D.GetRagdoll"/> turns it back into the ragdoll.</summary>
    public RagdollHandle RawHandle => new(_world?.Id ?? 0, _index, _generation);

    /// <summary>True while the ragdoll exists (it ends with <see cref="Remove"/>, or when one of its bodies is destroyed on its own).</summary>
    public bool IsValid => _world != null && _world.Engine.RagHandles.IsAlive(_index, _generation);

    /// <summary>Number of parts.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public int PartCount => Resolve().Parts;

    /// <summary>The root part's body.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public Body Root => GetBody(0);

    /// <summary>The body of a part (index into the definition's <see cref="RagdollDef.Parts"/>).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="part"/> is not a part index.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public Body GetBody(int part)
    {
        var s = Resolve();
        Part(s, part);
        return new Body(_world!, new BodyHandle(_world!.Id, s.Body[part], s.BodyGen[part]));
    }

    /// <summary>The revolute joint that attaches a part to its parent. For the root, and for a joint that has broken or been destroyed, the result is a joint whose <see cref="Joint.IsValid"/> is false.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="part"/> is not a part index.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public Joint GetJoint(int part)
    {
        var s = Resolve();
        Part(s, part);
        if (!_world!.Engine.RagdollJointAlive(s, part)) { return default; }
        return new Joint(_world, new JointHandle(_world.Id, s.Joint[part], s.JointGen[part]));
    }

    /// <summary>True while a part is joined to its parent (always true for the root); false once its joint broke or was destroyed.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="part"/> is not a part index.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public bool IsAttached(int part)
    {
        var s = Resolve();
        Part(s, part);
        return part == 0 || _world!.Engine.RagdollJointAlive(s, part);
    }

    /// <summary>True while a pose drive is on (<see cref="DriveToPose"/> with a strength above 0).</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public bool IsPowered => Resolve().Powered;

    /// <summary>The strength of the last <see cref="DriveToPose"/>, 0 to 1; 0 when limp.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public float Strength => Resolve().Strength;

    /// <summary>
    /// Raises the solver settings a powered ragdoll needs to stand: at least 4 velocity iterations per substep. Settings already higher are kept.
    /// </summary>
    /// <param name="options">The options to adjust, for example those passed to <see cref="PhysicsWorld2D"/> or its live <see cref="PhysicsWorld2D.Options"/>.</param>
    public static void ApplyRecommendedOptions(WorldOptions2D options)
    {
        ArgumentNullException.ThrowIfNull(options);
        if (options.VelocityIterations < 4) { options.VelocityIterations = 4; }
    }

    private static void Part(RagdollState s, int part)
    {
        if ((uint)part >= (uint)s.Parts) { throw new ArgumentOutOfRangeException(nameof(part), part, "Part " + part.ToString(CultureInfo.InvariantCulture) + " is not in 0.." + (s.Parts - 1).ToString(CultureInfo.InvariantCulture) + "."); }
    }

    private static void CheckPose(RagdollState s, ReadOnlySpan<float> pose)
    {
        if (pose.Length != s.Parts) { throw new ArgumentException("The pose has " + pose.Length.ToString(CultureInfo.InvariantCulture) + " angles; the ragdoll has " + s.Parts.ToString(CultureInfo.InvariantCulture) + " parts.", nameof(pose)); }
        for (int i = 0; i < pose.Length; i++)
        {
            if (!float.IsFinite(pose[i]) || Math.Abs(pose[i]) > 1e6f) { throw new ArgumentOutOfRangeException(nameof(pose), pose[i], "Pose entry " + i.ToString(CultureInfo.InvariantCulture) + " must be finite and within +-1e6 radians."); }
        }
    }

    /// <summary>
    /// Turns the pose drive on (or updates it): every joint's motor turns its part towards <paramref name="pose"/> at a speed proportional to the angle error (<see cref="RagdollPartDef.DriveHertz"/>, at most
    /// <see cref="RagdollPartDef.MaxDriveSpeed"/>) with a torque of at most <see cref="RagdollPartDef.MaxTorque"/> times <paramref name="strength"/>. Call it as often as you like (once a frame to follow an animation);
    /// a part whose target did not change is not woken. The joint limits keep holding; a strength of 0 is the same as <see cref="StopDriving"/>.
    /// </summary>
    /// <param name="pose">One angle per part (see the remarks of <see cref="Ragdoll"/>); each finite. The root's entry is ignored.</param>
    /// <param name="strength">0 (limp) to 1 (the definition's full torque).</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry is not finite, or the strength is NaN or outside 0..1.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void DriveToPose(ReadOnlySpan<float> pose, float strength = 1f)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.DriveToPose");
        var s = Resolve();
        CheckPose(s, pose);
        if (!(strength >= 0f && strength <= 1f)) { throw new ArgumentOutOfRangeException(nameof(strength), strength, "The strength must be within 0..1."); }
        if (!(strength > 0f)) { StopDriving(); return; }
        var e = _world!.Engine;
        if (e.ActiveOf(_index) != null) { throw new InvalidOperationException("The ragdoll is active: its joints are driven by the active layer. Use SetAnimationPose and the blend weights, or DisableActive first."); }
        bool wake = !s.Powered || s.Strength != strength;
        for (int i = 1; i < s.Parts; i++)
        {
            if (s.Target[i] != pose[i]) { wake = true; }
            s.Target[i] = pose[i];
        }

        if (wake)
        {
            for (int i = 0; i < s.Parts; i++) { _world.Core.Wake(new BodyHandle(_world.Id, s.Body[i], s.BodyGen[i])); }
        }

        s.Powered = true;
        s.Strength = strength;
    }

    /// <summary>Lets go: removes the drive from every joint, so the ragdoll is limp inside its joint limits. Does nothing when it is not powered.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void StopDriving()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.StopDriving");
        var s = Resolve();
        if (!s.Powered) { return; }
        for (int i = 1; i < s.Parts; i++)
        {
            if (!_world!.Engine.RagdollJointAlive(s, i)) { continue; }
            var j = new Joint(_world, new JointHandle(_world.Id, s.Joint[i], s.JointGen[i]));
            j.EnableMotor = false; j.MotorSpeed = 0f; j.MaxMotorTorque = 0f;
        }

        s.Powered = false;
        s.Strength = 0f;
    }

    /// <summary>
    /// Teleports every part into <paramref name="pose"/>, with the ragdoll's space placed at <paramref name="at"/> and turned by <paramref name="angle"/>, and zeroes their velocities. Joints are satisfied by construction.
    /// The pose drive, if on, stays on with its old targets; call <see cref="DriveToPose"/> too to retarget it.
    /// </summary>
    /// <param name="pose">One angle per part; entry 0 turns the whole figure about the root body's centre.</param>
    /// <param name="at">Where the bind pose's space origin goes, world coordinates (within ±2^21).</param>
    /// <param name="angle">Orientation of the ragdoll's space, radians.</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry or <paramref name="angle"/> is not finite, or <paramref name="at"/> is not finite or beyond ±2^21.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void SetPose(ReadOnlySpan<float> pose, Vector2D at, float angle = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.SetPose");
        var s = Resolve();
        CheckPose(s, pose);
        Kinematics(s, pose, at, angle);
        for (int i = 0; i < s.Parts; i++)
        {
            var b = new Body(_world!, new BodyHandle(_world!.Id, s.Body[i], s.BodyGen[i]));
            b.SetTransform(new Vector2D(s.TargetX[i], s.TargetY[i]), s.TargetAngle[i]);
            b.LinearVelocity = default; b.AngularVelocity = 0f;
        }
    }

    /// <summary>
    /// Keyframed mode: sets every part's velocity so that, if nothing else acted, it reaches <paramref name="pose"/> in <paramref name="dt"/> seconds. Call it before <see cref="PhysicsWorld2D.Step"/> each frame with the
    /// animation's pose for the end of the step. The bodies stay dynamic, so contacts, gravity and joint limits still act and the figure is pushed off the pose where it meets something; the next call corrects it again.
    /// </summary>
    /// <param name="pose">One angle per part; see <see cref="SetPose"/>.</param>
    /// <param name="at">Where the bind pose's space origin should be at the end of the step.</param>
    /// <param name="angle">Orientation of the ragdoll's space at the end of the step, radians.</param>
    /// <param name="dt">The step length, seconds; finite and above 0.</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not usable, or <paramref name="dt"/> is not finite and above 0.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void MoveToPose(ReadOnlySpan<float> pose, Vector2D at, float angle, float dt)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.MoveToPose");
        var s = Resolve();
        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and above 0."); }
        CheckPose(s, pose);
        Kinematics(s, pose, at, angle);
        float inv = 1f / dt;
        for (int i = 0; i < s.Parts; i++)
        {
            var b = new Body(_world!, new BodyHandle(_world!.Id, s.Body[i], s.BodyGen[i]));
            var p = b.Position;
            b.LinearVelocity = new Vector2F((float)(s.TargetX[i] - p.X) * inv, (float)(s.TargetY[i] - p.Y) * inv);
            float da = s.TargetAngle[i] - b.Angle;
            const float Pi = 3.1415927f;
            while (da > Pi) { da -= 2f * Pi; }
            while (da < -Pi) { da += 2f * Pi; }
            b.AngularVelocity = da * inv;
        }
    }

    /// <summary>
    /// The largest angle, radians, by which any attached part's joint is off <paramref name="pose"/>. 0 for a single-part ragdoll. A joint that has broken is skipped.
    /// </summary>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry is not finite.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public float GetPoseError(ReadOnlySpan<float> pose)
    {
        var s = Resolve();
        CheckPose(s, pose);
        var e = _world!.Engine;
        float worst = 0f;
        for (int i = 1; i < s.Parts; i++)
        {
            if (!e.RagdollJointAlive(s, i)) { continue; }
            float a = e.RagdollJointError(s, i, pose[i]);
            if (a > worst) { worst = a; }
        }

        return worst;
    }

    /// <summary>Destroys every body and joint of the ragdoll. Every copy of this handle becomes stale.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed already.</exception>
    public void Remove()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.Remove");
        var s = Resolve();
        var w = _world!;
        var bodies = new BodyHandle[s.Parts];
        for (int i = 0; i < s.Parts; i++) { bodies[i] = new BodyHandle(w.Id, s.Body[i], s.BodyGen[i]); }
        w.Engine.ForgetRagdoll(_index);
        foreach (var h in bodies)
        {
            if (w.Core.Bodies.Handles.IsAlive(h.Index, h.Generation)) { w.DestroyBody(h); }
        }
    }

    // Forward kinematics of the bind pose with each part turned by its pose entry: fills TargetX, TargetY and TargetAngle. The body of part i has the
    // accumulated turn of its ancestors plus its own bind angle, and sits where its joint anchor, carried along the parent, meets its own anchor.
    private static void Kinematics(RagdollState s, ReadOnlySpan<float> pose, Vector2D at, float angle)
    {
        const double Bound = 2097152.0;
        if (!double.IsFinite(at.X) || !double.IsFinite(at.Y) || Math.Abs(at.X) > Bound || Math.Abs(at.Y) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(at), at, "Position " + at + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        if (!float.IsFinite(angle) || Math.Abs(angle) > 1e6f) { throw new ArgumentOutOfRangeException(nameof(angle), angle, "The angle must be finite and within +-1e6 radians."); }
        var parts = s.Def.Parts;
        var delta = s.DeltaAngle;
        var (rs, rc) = Math64.SinCos((double)angle);
        double ox = parts[0].Position.X, oy = parts[0].Position.Y;
        s.TargetX[0] = at.X + (rc * ox - rs * oy); s.TargetY[0] = at.Y + (rs * ox + rc * oy);
        delta[0] = angle + pose[0];
        for (int i = 1; i < parts.Length; i++)
        {
            int p = parts[i].Parent;
            delta[i] = delta[p] + pose[i];
            var jp = parts[i].JointPosition;
            double tx = jp.X - parts[p].Position.X, ty = jp.Y - parts[p].Position.Y;
            double fx = jp.X - parts[i].Position.X, fy = jp.Y - parts[i].Position.Y;
            var (sp, cp) = Math64.SinCos((double)delta[p]);
            var (sc, cc) = Math64.SinCos((double)delta[i]);
            s.TargetX[i] = s.TargetX[p] + (cp * tx - sp * ty) - (cc * fx - sc * fy);
            s.TargetY[i] = s.TargetY[p] + (sp * tx + cp * ty) - (sc * fx + cc * fy);
        }

        for (int i = 0; i < parts.Length; i++) { s.TargetAngle[i] = delta[i] + parts[i].Angle; }
    }

    /// <inheritdoc/>
    public bool Equals(Ragdoll other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Ragdoll r && Equals(r);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Handle equality.</summary>
    public static bool operator ==(Ragdoll a, Ragdoll b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(Ragdoll a, Ragdoll b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => "Ragdoll(world " + (_world?.Id ?? -1).ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Builds a ragdoll: one dynamic body per part, joined to its parent by a revolute joint with the part's limits, in the bind pose placed at <paramref name="at"/>. The bodies of a ragdoll do not collide
    /// as <see cref="RagdollDef.SelfCollision"/> says (with <see cref="RagdollSelfCollision.None"/> the parts share the negative collision group −(slot + 1)). Without a drive the ragdoll is limp; see <see cref="Ragdoll.DriveToPose"/>.
    /// </summary>
    /// <param name="def">The skeleton, for example <see cref="RagdollDef.Humanoid"/>; copied.</param>
    /// <param name="at">Where the origin of the ragdoll's space goes, world coordinates (within ±2^21). Default the world origin.</param>
    /// <param name="angle">Orientation of the ragdoll's space, radians. Default 0.</param>
    /// <param name="pose">Optional starting pose (one angle per part, see <see cref="Ragdoll"/>); empty means the bind pose.</param>
    /// <exception cref="InvalidRagdollException">The skeleton is not valid: no parts, too many, no single root, a parent that does not come first.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value in the definition, or <paramref name="at"/>, is out of range; the message names the part and the field. Nothing has been added.</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Ragdoll AddRagdoll(RagdollDef def, Vector2D at = default, float angle = 0f, ReadOnlySpan<float> pose = default)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddRagdoll");
        ArgumentNullException.ThrowIfNull(def);
        var d = def.Clone();
        d.Validate();
        const double Bound = 2097152.0;
        if (!double.IsFinite(at.X) || !double.IsFinite(at.Y) || Math.Abs(at.X) > Bound || Math.Abs(at.Y) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(at), at, "Position " + at + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        if (!float.IsFinite(angle) || Math.Abs(angle) > 1e6f) { throw new ArgumentOutOfRangeException(nameof(angle), angle, "The angle must be finite and within +-1e6 radians."); }
        var (rsd, rcd) = Math64.SinCos((double)angle);
        float rs = (float)rsd, rc = (float)rcd;
        var s = new RagdollState { Def = d };
        s.Allocate();
        int slot = _engine.ReserveRagdoll();
        var made = new List<Body>(d.Parts.Length);
        try
        {
            int group = d.SelfCollision == RagdollSelfCollision.None ? -(slot + 1) : 0;
            for (int i = 0; i < d.Parts.Length; i++)
            {
                var p = d.Parts[i];
                var bd = new BodyDef
                {
                    Type = BodyType.Dynamic,
                    Position = new Vector2D(at.X + (double)(rc * p.Position.X - rs * p.Position.Y), at.Y + (double)(rs * p.Position.X + rc * p.Position.Y)),
                    Angle = angle + p.Angle, AngularDamping = d.AngularDamping,
                };
                var body = CreateBody(in bd);
                made.Add(body);
                var sd = new ShapeDef { Density = p.Mass / p.Area(), Friction = p.Friction, CategoryBits = d.CategoryBits, MaskBits = d.MaskBits, GroupIndex = group };
                switch (p.Shape)
                {
                    case RagdollShapeKind.Circle: CreateCircle(body, p.ShapeHalfSize.X, default, sd); break;
                    case RagdollShapeKind.Capsule: CreateCapsule(body, new Vector2F(-p.ShapeHalfSize.X, 0f), new Vector2F(p.ShapeHalfSize.X, 0f), p.ShapeHalfSize.Y, sd); break;
                    default: CreateBox(body, new Vector2F(2f * p.ShapeHalfSize.X, 2f * p.ShapeHalfSize.Y), default, 0f, 0f, sd); break;
                }

                s.Body[i] = body.Handle.Index; s.BodyGen[i] = body.Handle.Generation;
                if (p.Parent < 0) { continue; }
                var parent = made[p.Parent];
                var pp = d.Parts[p.Parent];
                // Anchors in each body's own frame: the joint position relative to the part's centre, turned back by the part's bind angle.
                var (sa, ca) = Math64.SinCos((double)pp.Angle);
                var (sb, cb) = Math64.SinCos((double)p.Angle);
                float ax = p.JointPosition.X - pp.Position.X, ay = p.JointPosition.Y - pp.Position.Y;
                float bx = p.JointPosition.X - p.Position.X, by = p.JointPosition.Y - p.Position.Y;
                var jd = new RevoluteJointDef
                {
                    BodyA = parent, BodyB = body,
                    LocalAnchorA = new Vector2F((float)(ca * ax + sa * ay), (float)(-sa * ax + ca * ay)),
                    LocalAnchorB = new Vector2F((float)(cb * bx + sb * by), (float)(-sb * bx + cb * by)),
                    ReferenceAngle = p.Angle - pp.Angle,
                    EnableLimit = true, LowerAngle = p.LowerAngle, UpperAngle = p.UpperAngle,
                    CollideConnected = d.SelfCollision == RagdollSelfCollision.All,
                    BreakForce = p.BreakForce, BreakTorque = p.BreakTorque,
                };
                var joint = CreateJoint(in jd).Joint;
                var jh = joint.RawHandle;
                s.Joint[i] = jh.Index; s.JointGen[i] = jh.Generation;
            }

            _engine.RegisterRagdoll(slot, s);
        }
        catch (Exception)
        {
            _engine.CancelRagdoll(slot);
            foreach (var b in made) { if (b.IsValid) { b.Destroy(); } }
            throw;
        }

        var ragdoll = new Ragdoll(this, slot, _engine.RagHandles.GenerationOf(slot));
        if (pose.Length > 0)
        {
            try { ragdoll.SetPose(pose, at, angle); }
            catch (Exception) { ragdoll.Remove(); throw; }
        }

        return ragdoll;
    }

    /// <summary>The <see cref="Ragdoll"/> for a raw handle (from <see cref="Ragdoll.RawHandle"/>).</summary>
    /// <exception cref="InvalidHandleException">The handle is null, belongs to another world (<see cref="ForeignHandleException"/>) or refers to a removed ragdoll (<see cref="StaleHandleException"/>).</exception>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Ragdoll GetRagdoll(RagdollHandle handle)
    {
        if (handle.IsNull) { throw new InvalidHandleException("The null ragdoll handle was used."); }
        if (handle.World != Id) { throw new ForeignHandleException(handle + " belongs to world " + handle.World.ToString(CultureInfo.InvariantCulture) + ", not world " + Id.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!_engine.RagHandles.IsAlive(handle.Index, handle.Generation)) { throw new StaleHandleException(handle + " is stale: the ragdoll was removed."); }
        return new Ragdoll(this, handle.Index, handle.Generation);
    }

    /// <summary>True while the ragdoll exists in this world. False for null handles, handles of another world and removed ragdolls (never throws). Allocates nothing.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public bool IsRagdollAlive(RagdollHandle handle) => !handle.IsNull && handle.World == Id && _engine.RagHandles.IsAlive(handle.Index, handle.Generation);

    /// <summary>The ragdoll that <paramref name="body"/> is a part of, if there is one.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public bool TryGetRagdoll(Body body, out Ragdoll ragdoll)
    {
        ragdoll = default;
        if (!body.IsValid || body.World != this) { return false; }
        int r = _engine.RagdollOfBody(body.Handle.Index);
        if (r < 0) { return false; }
        ragdoll = new Ragdoll(this, r, _engine.RagHandles.GenerationOf(r));
        return true;
    }

    /// <summary>All live ragdolls in slot order (allocates the array).</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public Ragdoll[] GetRagdolls()
    {
        var list = new List<Ragdoll>();
        for (int r = 0; r < _engine.RagHandles.HighWater; r++)
        {
            if (_engine.RagHandles.IsLive(r)) { list.Add(new Ragdoll(this, r, _engine.RagHandles.GenerationOf(r))); }
        }

        return list.ToArray();
    }
}
