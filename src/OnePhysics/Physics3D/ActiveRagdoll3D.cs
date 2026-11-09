using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Active;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Ragdoll
{
    private ActiveState3D ResolveActive()
    {
        var s = Resolve();
        _ = s;
        var a = _world!.Engine.ActiveOf(_index);
        if (a == null) { throw new InvalidOperationException("The ragdoll is not active: call EnableActive first."); }
        return a;
    }

    /// <summary>True while the ragdoll has an active layer (<see cref="EnableActive"/>).</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public bool IsActive { get { Resolve(); return _world!.Engine.ActiveOf(_index) != null; } }

    /// <summary>
    /// Turns the ragdoll into an active one: its joints follow a target pose with stable proportional-derivative torques whose strength is each bone's simulated weight's complement, a balance or walking controller can
    /// drive the legs, and a host can blend an animation in. Calling it again replaces the settings and resets the controller. Every bone starts with blend weight 0, the controller off (<see cref="ActiveMode.Pose"/>) and no animation
    /// pose, so the figure holds the bind pose with the definition's torque limits. It takes over the joint drive: <see cref="DriveToPose"/> throws while the ragdoll is active.
    /// </summary>
    /// <param name="def">The settings; default <see cref="ActiveRagdollDef.Humanoid3D"/>. Copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range or a bone index is not a part; nothing changed.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void EnableActive(ActiveRagdollDef? def = null)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.EnableActive");
        var s = Resolve();
        if (s.Powered) { StopDriving(); }
        _world!.Engine.EnableActive(_index, def ?? ActiveRagdollDef.Humanoid3D);
    }

    /// <summary>Removes the active layer: drives stop, pinned bones become ordinary simulated bodies. Does nothing when the ragdoll is not active.</summary>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void DisableActive()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.DisableActive");
        Resolve();
        _world!.Engine.DropActive(_index);
    }

    /// <summary>The controller mode.</summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public ActiveMode Mode => (ActiveMode)ResolveActive().Mode;

    /// <summary>Standing balance with push recovery (<see cref="ActiveMode.Stand"/>). The legs balance, the arms and head follow the pose and weights; the figure turns to face <paramref name="heading"/>.</summary>
    /// <param name="heading">Yaw about +y, radians, 0 facing +z; finite. Default 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">The heading is not finite.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void Stand(float heading = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.Stand");
        var a = ResolveActive();
        CheckHeading(heading);
        a.CmdX = 0f; a.CmdZ = 0f; a.Heading = heading; SetMode(a, ActiveMode.Stand);
    }

    private static void CheckHeading(float heading)
    {
        if (!float.IsFinite(heading) || Math.Abs(heading) > 1e6f) { throw new ArgumentOutOfRangeException(nameof(heading), heading, "The heading must be finite and within +-1e6 radians."); }
    }

    /// <summary>
    /// Walks: <see cref="ActiveMode.Walk"/> with a commanded horizontal velocity and a heading. The figure walks forward along its own heading and turns towards <paramref name="heading"/>: the speed it takes is the part of
    /// <paramref name="velocity"/> that lies along the heading (a velocity that points away from the heading gives none; sideways walking is not supported), lowered while the figure is still turned away from the heading, and a zero
    /// velocity stands (after a few more steps, if the figure is moving). Call it as often as you like; the gait follows the latest command. A heading that changes by a right angle is turned through over a few steps.
    /// </summary>
    /// <param name="velocity">Commanded velocity in world X and Z, m/s; its length is clamped to <see cref="WalkerDef.MaxSpeed"/>.</param>
    /// <param name="heading">Yaw the figure turns to face, radians about +y (0 faces +z); finite.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void Walk(Vector2F velocity, float heading)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.Walk");
        var a = ResolveActive();
        if (!float.IsFinite(velocity.X) || !float.IsFinite(velocity.Y)) { throw new ArgumentOutOfRangeException(nameof(velocity), velocity, "The velocity must be finite."); }
        CheckHeading(heading);
        a.CmdX = velocity.X; a.CmdZ = velocity.Y; a.Heading = heading;
        SetMode(a, ActiveMode.Walk);
    }

    /// <summary>Walks at <paramref name="speed"/> towards <paramref name="heading"/>: <c>Walk(speed · (sin heading, cos heading), heading)</c>. A speed of 0 stands.</summary>
    /// <param name="heading">Yaw the figure faces and walks along, radians about +y (0 faces +z); finite.</param>
    /// <param name="speed">Speed, m/s; finite, at least 0 (clamped to <see cref="WalkerDef.MaxSpeed"/>).</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, or the speed is negative.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void Walk(float heading, float speed)
    {
        if (!float.IsFinite(speed) || speed < 0f) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speed must be finite and at least 0."); }
        CheckHeading(heading);
        Walk(new Vector2F(speed * (float)Math64.Sin((double)heading), speed * (float)Math64.Cos((double)heading)), heading);
    }

    /// <summary>Turns the controller off: the bones follow the animation pose (or the bind pose) by their weights.</summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void StopController()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.StopController");
        SetMode(ResolveActive(), ActiveMode.Pose);
    }

    private static void SetMode(ActiveState3D a, ActiveMode mode)
    {
        if (a.Mode == (byte)mode) { return; }
        a.Mode = (byte)mode;
        a.Gait = default;
    }

    /// <summary>
    /// Sets the animation pose for the end of the next step: one rotation per part (the convention of <see cref="DriveToPose"/>), and where the animation puts the ragdoll space's origin and its orientation (the root).
    /// Bones with weight 0 are pinned to it exactly, bones in between are driven towards it, and the root follows <see cref="ActiveRagdollDef.RootMode"/>. The pose is an input: it is kept in snapshots, hashes and replays.
    /// </summary>
    /// <param name="pose">One rotation per part; each finite and non-zero, normalized on use.</param>
    /// <param name="rootPosition">World position of the ragdoll space's origin (within ±2^21).</param>
    /// <param name="rootRotation">Orientation of the ragdoll space; the default (zero) value means no rotation.</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not usable; nothing changed.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void SetAnimationPose(ReadOnlySpan<QuaternionF> pose, Vector3D rootPosition = default, QuaternionF rootRotation = default)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.SetAnimationPose");
        var a = ResolveActive();
        var s = Resolve();
        CheckPose(s, pose);
        const double Bound = 2097152.0;
        if (!double.IsFinite(rootPosition.X) || !double.IsFinite(rootPosition.Y) || !double.IsFinite(rootPosition.Z) || Math.Abs(rootPosition.X) > Bound || Math.Abs(rootPosition.Y) > Bound || Math.Abs(rootPosition.Z) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(rootPosition), rootPosition, "Position " + rootPosition + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        if (rootRotation.LengthSquared() != 0f) { Engine3D.Normalize(rootRotation, nameof(rootRotation)); }   // validates; the stored value is used as given, so pinned bones match Ragdoll.SetPose bit for bit
        for (int i = 0; i < pose.Length; i++) { Engine3D.Normalize(pose[i], nameof(pose)); }
        pose.CopyTo(a.Pose);   // stored as given: forward kinematics normalizes once, the same arithmetic as SetPose
        a.Root = rootPosition; a.RootRot = rootRotation;
        a.HasPose = true;
    }

    /// <summary>Forgets the animation pose: bones with weight 0 are driven to the bind pose instead of pinned.</summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void ClearAnimationPose()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.ClearAnimationPose");
        var a = ResolveActive();
        a.HasPose = false;
        for (int i = 0; i < a.Pose.Length; i++) { a.Pose[i] = QuaternionF.Identity; }
        a.Root = default; a.RootRot = QuaternionF.Identity;
    }

    /// <summary>
    /// Sets the blend weight of one bone: 0 follows the animation pose exactly (pinned; without a pose, driven to the bind pose at full strength), 1 is fully simulated (no drive from this layer: bit for bit a plain ragdoll),
    /// in between the bone is driven towards the pose with stiffness, damping and torque limit scaled by 1 − weight. Bones the controller owns (legs and torso in the stand and walk modes) are driven by it with the same strength rule and are never pinned.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The part is not a part index or the weight is not within 0..1.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void SetBlendWeight(int part, float weight)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.SetBlendWeight");
        var a = ResolveActive();
        Part(Resolve(), part);
        if (!(weight >= 0f && weight <= 1f)) { throw new ArgumentOutOfRangeException(nameof(weight), weight, "The blend weight must be within 0..1."); }
        a.Weight[part] = weight;
    }

    /// <summary>Sets every bone's blend weight (see <see cref="SetBlendWeight"/>).</summary>
    /// <exception cref="ArgumentException">The span does not hold one weight per part.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A weight is not within 0..1; nothing changed.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void SetBlendWeights(ReadOnlySpan<float> weights)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.SetBlendWeights");
        var a = ResolveActive();
        if (weights.Length != a.Parts) { throw new ArgumentException("There are " + weights.Length.ToString(CultureInfo.InvariantCulture) + " weights; the ragdoll has " + a.Parts.ToString(CultureInfo.InvariantCulture) + " parts.", nameof(weights)); }
        for (int i = 0; i < weights.Length; i++) { if (!(weights[i] >= 0f && weights[i] <= 1f)) { throw new ArgumentOutOfRangeException(nameof(weights), weights[i], "Weight " + i.ToString(CultureInfo.InvariantCulture) + " must be within 0..1."); } }
        weights.CopyTo(a.Weight);
    }

    /// <summary>The weight set for a bone (without any hit reaction).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The part is not a part index.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public float GetBlendWeight(int part)
    {
        var a = ResolveActive();
        Part(Resolve(), part);
        return a.Weight[part];
    }

    /// <summary>The weight a bone is driven with right now: its set weight raised towards 1 by the hit reaction still running.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The part is not a part index.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public float GetEffectiveBlendWeight(int part)
    {
        var a = ResolveActive();
        Part(Resolve(), part);
        return a.EffectiveWeight(part);
    }

    /// <summary>
    /// A hit reaction at a body: bones within <see cref="ActiveRagdollDef.HitRadius"/> of the part (distance between bodies' centres, falling off linearly) are pushed towards fully simulated by <paramref name="strength"/>
    /// times <see cref="ActiveRagdollDef.HitStrength"/>, and recover exponentially with <see cref="ActiveRagdollDef.HitRecoveryTime"/>. It does not apply an impulse: apply one to the body and call this.
    /// </summary>
    /// <param name="part">The struck part.</param>
    /// <param name="strength">0..1.</param>
    /// <exception cref="ArgumentOutOfRangeException">The part is not a part index or the strength is not within 0..1.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void ApplyHitReaction(int part, float strength = 1f)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.ApplyHitReaction");
        var a = ResolveActive();
        var s = Resolve();
        Part(s, part);
        if (!(strength >= 0f && strength <= 1f)) { throw new ArgumentOutOfRangeException(nameof(strength), strength, "The strength must be within 0..1."); }
        var bodies = _world!.Core.Bodies;
        var pc = bodies.PositionOf(s.Body[part]);
        for (int i = 0; i < s.Parts; i++)
        {
            var q = bodies.PositionOf(s.Body[i]);
            double dx = q[0] - pc[0], dy = q[1] - pc[1], dz = q[2] - pc[2];
            float dist = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
            float fall = 1f - dist / a.Def.HitRadius;
            if (fall > 0f) { a.AddHit(i, strength * fall); }
        }
    }

    /// <summary>Applies an impulse at a part's body and starts a hit reaction there (<see cref="ApplyHitReaction"/>) scaled by the impulse over <paramref name="fullStrengthImpulse"/>.</summary>
    /// <param name="part">The struck part.</param>
    /// <param name="impulse">Linear impulse, N·s (finite).</param>
    /// <param name="fullStrengthImpulse">The impulse magnitude that gives a reaction of strength 1; positive. Default 30.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not valid.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void Hit(int part, Vector3F impulse, float fullStrengthImpulse = 30f)
    {
        var a = ResolveActive();
        Part(Resolve(), part);
        if (!float.IsFinite(impulse.X) || !float.IsFinite(impulse.Y) || !float.IsFinite(impulse.Z)) { throw new ArgumentOutOfRangeException(nameof(impulse), impulse, "The impulse must be finite."); }
        if (!(fullStrengthImpulse > 0f) || !float.IsFinite(fullStrengthImpulse)) { throw new ArgumentOutOfRangeException(nameof(fullStrengthImpulse), fullStrengthImpulse, "Must be finite and above 0."); }
        using var replayMutation = _world?.RecordMutation("Ragdoll.Hit");
        var body = GetBody(part);
        body.ApplyImpulse(impulse);
        float m = float.Sqrt(impulse.X * impulse.X + impulse.Y * impulse.Y + impulse.Z * impulse.Z) / fullStrengthImpulse;
        ApplyHitReaction(part, m > 1f ? 1f : m);
        _ = a;
    }

    /// <summary>
    /// The controller's readout from the last step: mode, gait phase, step count, centre of mass, capture point and support box (world coordinates; Z is 0 in 2D), and how many feet were grounded. All zero before the first step
    /// and after a restore until the next step.
    /// </summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public ActiveRagdollInfo GetActiveInfo()
    {
        var a = ResolveActive();
        return a.Info;
    }
}
