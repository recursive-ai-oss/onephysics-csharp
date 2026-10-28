using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A ragdoll: a skeleton of bodies joined by swing-twist joints, made by <see cref="PhysicsWorld3D.AddRagdoll(RagdollDef, Vector3D, QuaternionF, ReadOnlySpan{QuaternionF})"/>. Like <see cref="Body"/> it is a small struct handle
/// (world, slot, generation) that allocates nothing and fails with a <see cref="StaleHandleException"/> once the ragdoll is removed; the state lives in the world and in its snapshots. Ragdolls exist in 3D worlds only.
/// </summary>
/// <remarks>
/// <para>Limp by default: the joints hold the parts together inside their limits and nothing else. <see cref="DriveToPose"/> turns on a spring-damper at every joint that pulls the part towards a pose, with a torque limit,
/// so a strong enough ragdoll stands, takes a knock and returns to the pose; <see cref="StopDriving"/> lets go. <see cref="SetPose"/> teleports the whole figure and <see cref="MoveToPose"/> carries it there by velocity (a keyframed mode).</para>
/// <para>A pose is one rotation per part. Entry <c>i</c> is the rotation of part <c>i</c> relative to its parent in the bind pose's axes: the identity everywhere is the bind pose, and a rotation about a world axis in the bind pose
/// means the same rotation of the limb whatever the parent is doing (the usual local animation rotation when bone axes are aligned with the bind pose). Entry 0, the root, orients the whole figure in <see cref="SetPose"/> and <see cref="MoveToPose"/> and is ignored by <see cref="DriveToPose"/>.</para>
/// </remarks>
public readonly struct Ragdoll : IEquatable<Ragdoll>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Ragdoll(PhysicsWorld3D world, int index, int generation)
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

    /// <summary>The swing-twist joint that attaches a part to its parent. For the root, and for a joint that has broken or been destroyed, the result is a joint whose <see cref="Joint.IsValid"/> is false.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="part"/> is not a part index.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public Joint GetJoint(int part)
    {
        var s = Resolve();
        Part(s, part);
        if (!_world!.Engine.RagdollJointAlive(s, part)) { return default; }
        return new Joint(_world, new JointHandle(_world.Id, s.Joint[part], s.JointGen[part]));
    }

    /// <summary>True while a pose drive is on (<see cref="DriveToPose"/> with a strength above 0).</summary>
    public bool IsPowered => Resolve().Powered;

    /// <summary>The strength of the last <see cref="DriveToPose"/>, 0 to 1; 0 when limp.</summary>
    public float Strength => Resolve().Strength;

    /// <summary>
    /// Raises the solver settings a powered ragdoll needs to stand: at least 4 velocity iterations per substep (the default of 1 leaves a tall stack of stiff drives creeping until it topples; 8 substeps with 1 iteration works too).
    /// Settings already higher are kept.
    /// </summary>
    /// <param name="options">The options to adjust, for example those passed to <see cref="PhysicsWorld3D"/> or its live <see cref="PhysicsWorld3D.Options"/>.</param>
    public static void ApplyRecommendedOptions(WorldOptions3D options)
    {
        ArgumentNullException.ThrowIfNull(options);
        if (options.VelocityIterations < 4) { options.VelocityIterations = 4; }
    }

    private static void Part(RagdollState s, int part)
    {
        if ((uint)part >= (uint)s.Parts) { throw new ArgumentOutOfRangeException(nameof(part), part, "Part " + part.ToString(CultureInfo.InvariantCulture) + " is not in 0.." + (s.Parts - 1).ToString(CultureInfo.InvariantCulture) + "."); }
    }

    private static void CheckPose(RagdollState s, ReadOnlySpan<QuaternionF> pose)
    {
        if (pose.Length != s.Parts) { throw new ArgumentException("The pose has " + pose.Length.ToString(CultureInfo.InvariantCulture) + " rotations; the ragdoll has " + s.Parts.ToString(CultureInfo.InvariantCulture) + " parts.", nameof(pose)); }
        for (int i = 0; i < pose.Length; i++)
        {
            float l2 = pose[i].LengthSquared();
            if (!float.IsFinite(l2) || !(l2 > 1e-30f)) { throw new ArgumentOutOfRangeException(nameof(pose), pose[i], "Pose entry " + i.ToString(CultureInfo.InvariantCulture) + " must be a finite, non-zero quaternion."); }
        }
    }

    /// <summary>
    /// Turns the pose drive on (or updates it): every joint pulls its part towards <paramref name="pose"/> with a spring-damper of the part's <see cref="RagdollPartDef.DriveStiffness"/> and <see cref="RagdollPartDef.DriveDamping"/> and
    /// a torque of at most <see cref="RagdollPartDef.MaxTorque"/>, all three scaled by <paramref name="strength"/>. Call it as often as you like (once a frame to follow an animation); a part whose target did not change
    /// is not woken. The joint limits keep holding; a strength of 0 is the same as <see cref="StopDriving"/>.
    /// </summary>
    /// <param name="pose">One rotation per part (see the remarks of <see cref="Ragdoll"/>); each finite and non-zero, normalized on use. The root's entry is ignored.</param>
    /// <param name="strength">0 (limp) to 1 (the definition's full gains).</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry is not a usable rotation, or the strength is NaN or outside 0..1.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void DriveToPose(ReadOnlySpan<QuaternionF> pose, float strength = 1f)
    {
        var s = Resolve();
        CheckPose(s, pose);
        if (!(strength >= 0f && strength <= 1f)) { throw new ArgumentOutOfRangeException(nameof(strength), strength, "The strength must be within 0..1."); }
        if (!(strength > 0f)) { StopDriving(); return; }
        for (int i = 1; i < s.Parts; i++)
        {
            if (!_world!.Engine.RagdollJointAlive(s, i)) { continue; }
            var p = s.Def.Parts[i];
            var r = s.JointRot[i];
            var d = Engine3D.Normalize(pose[i], nameof(pose));
            var target = (r.Conjugate() * d * r);   // the pose rotation in the joint frame: rel = R^-1 D R
            new Joint(_world, new JointHandle(_world.Id, s.Joint[i], s.JointGen[i])).SetPoseDrive(target, p.DriveStiffness * strength, p.DriveDamping * strength, p.MaxTorque * strength);
        }

        s.Powered = true;
        s.Strength = strength;
    }

    /// <summary>Lets go: removes the drive from every joint, so the ragdoll is limp inside its joint limits. Does nothing when it is not powered.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void StopDriving()
    {
        var s = Resolve();
        for (int i = 1; i < s.Parts; i++)
        {
            if (!_world!.Engine.RagdollJointAlive(s, i)) { continue; }
            new Joint(_world, new JointHandle(_world.Id, s.Joint[i], s.JointGen[i])).ClearPoseDrive();
        }

        s.Powered = false;
        s.Strength = 0f;
    }

    /// <summary>
    /// Teleports every part into <paramref name="pose"/>, with the ragdoll's space placed at <paramref name="at"/> and rotated by <paramref name="rotation"/>, and zeroes their velocities. Joints are satisfied by construction.
    /// The pose drive, if on, stays on with its old targets; call <see cref="DriveToPose"/> too to retarget it.
    /// </summary>
    /// <param name="pose">One rotation per part; entry 0 turns the whole figure about the root body's centre.</param>
    /// <param name="at">Where the bind pose's space origin goes, world coordinates (within ±2^21).</param>
    /// <param name="rotation">Orientation of the ragdoll's space; the default (zero) value means no rotation.</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry or <paramref name="rotation"/> is not a usable rotation, or <paramref name="at"/> is not finite or beyond ±2^21.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void SetPose(ReadOnlySpan<QuaternionF> pose, Vector3D at, QuaternionF rotation = default)
    {
        var s = Resolve();
        CheckPose(s, pose);
        Kinematics(s, pose, at, rotation);
        for (int i = 0; i < s.Parts; i++)
        {
            var b = new Body(_world!, new BodyHandle(_world!.Id, s.Body[i], s.BodyGen[i]));
            b.SetTransform(s.TargetPos[i], s.TargetRot[i]);
            b.LinearVelocity = default; b.AngularVelocity = default;
        }
    }

    /// <summary>
    /// Keyframed mode: sets every part's velocity so that, if nothing else acted, it reaches <paramref name="pose"/> in <paramref name="dt"/> seconds. Call it before <see cref="PhysicsWorld3D.Step"/> each frame with the
    /// animation's pose for the end of the step. The bodies stay dynamic, so contacts, gravity and joint limits still act and the figure is pushed off the pose where it meets something; the next call corrects it again.
    /// </summary>
    /// <param name="pose">One rotation per part; see <see cref="SetPose"/>.</param>
    /// <param name="at">Where the bind pose's space origin should be at the end of the step.</param>
    /// <param name="rotation">Orientation of the ragdoll's space at the end of the step; the default value means no rotation.</param>
    /// <param name="dt">The step length, seconds; finite and above 0.</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not usable, or <paramref name="dt"/> is not finite and above 0.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void MoveToPose(ReadOnlySpan<QuaternionF> pose, Vector3D at, QuaternionF rotation, float dt)
    {
        var s = Resolve();
        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and above 0."); }
        CheckPose(s, pose);
        Kinematics(s, pose, at, rotation);
        float inv = 1f / dt;
        for (int i = 0; i < s.Parts; i++)
        {
            var b = new Body(_world!, new BodyHandle(_world!.Id, s.Body[i], s.BodyGen[i]));
            var p = b.Position;
            var t = s.TargetPos[i];
            b.LinearVelocity = new Vector3F((float)(t.X - p.X) * inv, (float)(t.Y - p.Y) * inv, (float)(t.Z - p.Z) * inv);
            // The rotation from the current to the target orientation as an axis-angle, spread over the step.
            var dq = s.TargetRot[i] * b.Rotation.Conjugate();
            if (dq.W < 0f) { dq = new QuaternionF(-dq.X, -dq.Y, -dq.Z, -dq.W); }
            float sn = float.Sqrt(dq.X * dq.X + dq.Y * dq.Y + dq.Z * dq.Z);
            if (sn > 1e-12f)
            {
                float angle = (float)(2.0 * Math64.Atan2(sn, dq.W));
                float k = angle / sn * inv;
                b.AngularVelocity = new Vector3F(dq.X * k, dq.Y * k, dq.Z * k);
            }
            else { b.AngularVelocity = default; }
        }
    }

    /// <summary>
    /// The largest angle, radians, by which any part's joint is off <paramref name="pose"/> (the angle of the rotation between the joint's current relative rotation and the pose's). 0 for a single-part ragdoll.
    /// A joint that has broken is skipped.
    /// </summary>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry is not a usable rotation.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public float GetPoseError(ReadOnlySpan<QuaternionF> pose)
    {
        var s = Resolve();
        CheckPose(s, pose);
        var e = _world!.Engine;
        float worst = 0f;
        for (int i = 1; i < s.Parts; i++)
        {
            if (!e.RagdollJointAlive(s, i)) { continue; }
            float a = e.RagdollJointError(s, i, Engine3D.Normalize(pose[i], nameof(pose)));
            if (a > worst) { worst = a; }
        }

        return worst;
    }

    /// <summary>Destroys every body and joint of the ragdoll. Every copy of this handle becomes stale.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed already.</exception>
    public void Remove()
    {
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

    // Forward kinematics of the bind pose with each part turned by its pose entry: fills TargetPos and TargetRot. The body of part i has
    // orientation D_i-composed(parents) * bind and sits where its joint anchor, carried along the parent, meets its own anchor.
    private static void Kinematics(RagdollState s, ReadOnlySpan<QuaternionF> pose, Vector3D at, QuaternionF rotation)
    {
        const double Bound = 2097152.0;
        if (!double.IsFinite(at.X) || !double.IsFinite(at.Y) || !double.IsFinite(at.Z) || Math.Abs(at.X) > Bound || Math.Abs(at.Y) > Bound || Math.Abs(at.Z) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(at), at, "Position " + at + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        var rot = rotation.LengthSquared() == 0f ? QuaternionF.Identity : Engine3D.Normalize(rotation, nameof(rotation));
        var parts = s.Def.Parts;
        Span<QuaternionF> delta = s.TargetRot;   // accumulated rotation from the bind pose; the body orientation is delta * bind, written afterwards
        var pos = s.TargetPos;
        var rootOffset = rot.Rotate(parts[0].Position);
        pos[0] = new Vector3D(at.X + rootOffset.X, at.Y + rootOffset.Y, at.Z + rootOffset.Z);
        delta[0] = rot * Engine3D.Normalize(pose[0], nameof(pose));
        for (int i = 1; i < parts.Length; i++)
        {
            int p = parts[i].Parent;
            var d = Engine3D.Normalize(pose[i], nameof(pose));
            delta[i] = delta[p] * d;
            var jp = parts[i].JointPosition;
            var toJoint = delta[p].Rotate(jp - parts[p].Position);
            var fromJoint = delta[i].Rotate(jp - parts[i].Position);
            pos[i] = new Vector3D(pos[p].X + toJoint.X - fromJoint.X, pos[p].Y + toJoint.Y - fromJoint.Y, pos[p].Z + toJoint.Z - fromJoint.Z);
        }

        for (int i = 0; i < parts.Length; i++) { delta[i] = delta[i] * s.BindRot[i]; }
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

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Builds a ragdoll: one dynamic body per part, joined to its parent by a swing-twist joint with the part's limits, in the bind pose placed at <paramref name="at"/>. The bodies of a ragdoll do not collide
    /// as <see cref="RagdollDef.SelfCollision"/> says (with <see cref="RagdollSelfCollision.None"/> the parts share the negative collision group −(slot + 1)). Without a drive the ragdoll is limp; see <see cref="Ragdoll.DriveToPose"/>.
    /// 3D worlds only: there is no 2D ragdoll.
    /// </summary>
    /// <param name="def">The skeleton, for example <see cref="RagdollDef.Humanoid"/>; copied.</param>
    /// <param name="at">Where the origin of the ragdoll's space goes, world coordinates (within ±2^21). Default the world origin.</param>
    /// <param name="rotation">Orientation of the ragdoll's space; the default (zero) value means no rotation.</param>
    /// <param name="pose">Optional starting pose (one rotation per part, see <see cref="Ragdoll"/>); empty means the bind pose.</param>
    /// <exception cref="InvalidRagdollException">The skeleton is not valid: no parts, too many, no single root, a parent that does not come first.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value in the definition, or <paramref name="at"/>, is out of range; the message names the part and the field. Nothing has been added.</exception>
    public Ragdoll AddRagdoll(RagdollDef def, Vector3D at = default, QuaternionF rotation = default, ReadOnlySpan<QuaternionF> pose = default)
    {
        ArgumentNullException.ThrowIfNull(def);
        var d = def.Clone();
        d.Validate();
        const double Bound = 2097152.0;
        if (!double.IsFinite(at.X) || !double.IsFinite(at.Y) || !double.IsFinite(at.Z) || Math.Abs(at.X) > Bound || Math.Abs(at.Y) > Bound || Math.Abs(at.Z) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(at), at, "Position " + at + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        var rot = rotation.LengthSquared() == 0f ? QuaternionF.Identity : Engine3D.Normalize(rotation, nameof(rotation));
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
                var off = rot.Rotate(p.Position);
                var bd = new BodyDef { Type = BodyType.Dynamic, Position = new Vector3D(at.X + off.X, at.Y + off.Y, at.Z + off.Z), Rotation = rot * s.BindRot[i], AngularDamping = d.AngularDamping };
                var body = CreateBody(in bd);
                made.Add(body);
                var sd = new ShapeDef { Density = p.Mass / p.Volume(), Friction = p.Friction, CategoryBits = d.CategoryBits, MaskBits = d.MaskBits, GroupIndex = group };
                switch (p.Shape)
                {
                    case RagdollShapeKind.Sphere: CreateSphere(body, p.ShapeSize.X, sd); break;
                    case RagdollShapeKind.Capsule: CreateCapsule(body, p.ShapeSize.X, p.ShapeSize.Y, sd); break;
                    default: CreateBox(body, p.ShapeSize, 0f, sd); break;
                }

                s.Body[i] = body.Handle.Index; s.BodyGen[i] = body.Handle.Generation;
                if (p.Parent < 0) { continue; }
                var parent = made[p.Parent];
                var pp = d.Parts[p.Parent];
                var qpInv = s.BindRot[p.Parent].Conjugate();
                var qcInv = s.BindRot[i].Conjugate();
                var jr = s.JointRot[i];
                var jd = new SwingTwistJointDef
                {
                    BodyA = parent, BodyB = body,
                    LocalFrameA = new JointFrame3D(qpInv.Rotate(p.JointPosition - pp.Position), qpInv * jr),
                    LocalFrameB = new JointFrame3D(qcInv.Rotate(p.JointPosition - p.Position), qcInv * jr),
                    LowerTwist = p.LowerTwist, UpperTwist = p.UpperTwist, SwingYHalfAngle = p.SwingYHalfAngle, SwingZHalfAngle = p.SwingZHalfAngle,
                    CollideConnected = d.SelfCollision == RagdollSelfCollision.All,
                };
                var joint = CreateJoint(in jd);
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
            try { ragdoll.SetPose(pose, at, rotation); }
            catch (Exception) { ragdoll.Remove(); throw; }
        }

        return ragdoll;
    }

    /// <summary>The <see cref="Ragdoll"/> for a raw slot and generation (from the raw tier); fails when it no longer exists.</summary>
    /// <exception cref="StaleHandleException">There is no such ragdoll.</exception>
    public Ragdoll GetRagdoll(int index, int generation)
    {
        if (!_engine.RagHandles.IsAlive(index, generation)) { throw new StaleHandleException("Ragdoll #" + index.ToString(CultureInfo.InvariantCulture) + " (generation " + generation.ToString(CultureInfo.InvariantCulture) + ") does not exist."); }
        return new Ragdoll(this, index, generation);
    }

    /// <summary>The ragdoll that <paramref name="body"/> is a part of, if there is one.</summary>
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
