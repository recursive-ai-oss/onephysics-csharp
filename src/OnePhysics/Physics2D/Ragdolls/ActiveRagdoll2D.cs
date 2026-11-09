using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Active;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Ragdoll
{
    private ActiveState2D ResolveActive()
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
    /// <param name="def">The settings; default <see cref="ActiveRagdollDef.Humanoid"/>. Copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range or a bone index is not a part; nothing changed.</exception>
    /// <exception cref="StaleHandleException">The ragdoll was removed.</exception>
    public void EnableActive(ActiveRagdollDef? def = null)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.EnableActive");
        var s = Resolve();
        if (s.Powered) { StopDriving(); }
        _world!.Engine.EnableActive(_index, def ?? ActiveRagdollDef.Humanoid);
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

    /// <summary>Standing balance with push recovery (<see cref="ActiveMode.Stand"/>). The legs balance, the arms and head follow the pose and weights.</summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void Stand()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.Stand");
        var a = ResolveActive();
        a.CmdX = 0f; SetMode(a, ActiveMode.Stand);
    }

    /// <summary>Walks: <see cref="ActiveMode.Walk"/> with a commanded speed along +x (2D walkers face +x). Zero stands. Call it as often as you like; the gait follows the latest command.</summary>
    /// <param name="speed">Commanded horizontal velocity of the centre of mass, m/s, within 0..<see cref="WalkerDef.MaxSpeed"/> (a negative value is treated as 0, a larger one is clamped).</param>
    /// <exception cref="ArgumentOutOfRangeException">The speed is not finite.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void Walk(float speed)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.Walk");
        var a = ResolveActive();
        if (!float.IsFinite(speed)) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speed must be finite."); }
        a.CmdX = speed;
        SetMode(a, ActiveMode.Walk);
    }

    /// <summary>Turns the controller off: the bones follow the animation pose (or the bind pose) by their weights.</summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void StopController()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.StopController");
        SetMode(ResolveActive(), ActiveMode.Pose);
    }

    private static void SetMode(ActiveState2D a, ActiveMode mode)
    {
        if (a.Mode == (byte)mode) { return; }
        a.Mode = (byte)mode;
        a.Gait = default;
    }

    /// <summary>
    /// Sets the animation pose for the end of the next step: one joint angle per part (the convention of <see cref="DriveToPose"/>), and where the animation puts the ragdoll space's origin and its orientation (the root).
    /// Bones with weight 0 are pinned to it exactly, bones in between are driven towards it, and the root follows <see cref="ActiveRagdollDef.RootMode"/>. The pose is an input: it is kept in snapshots, hashes and replays.
    /// </summary>
    /// <param name="pose">One angle per part; finite, within ±1e6.</param>
    /// <param name="rootPosition">World position of the ragdoll space's origin (within ±2^21).</param>
    /// <param name="rootAngle">Orientation of the ragdoll space, radians.</param>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range; nothing changed.</exception>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void SetAnimationPose(ReadOnlySpan<float> pose, Vector2D rootPosition = default, float rootAngle = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.SetAnimationPose");
        var a = ResolveActive();
        var s = Resolve();
        CheckPose(s, pose);
        const double Bound = 2097152.0;
        if (!double.IsFinite(rootPosition.X) || !double.IsFinite(rootPosition.Y) || Math.Abs(rootPosition.X) > Bound || Math.Abs(rootPosition.Y) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(rootPosition), rootPosition, "Position " + rootPosition + " is outside the supported range of +-2^21 (2097152) relative to the world origin.");
        }

        if (!float.IsFinite(rootAngle) || Math.Abs(rootAngle) > 1e6f) { throw new ArgumentOutOfRangeException(nameof(rootAngle), rootAngle, "The angle must be finite and within +-1e6 radians."); }
        pose.CopyTo(a.Pose);
        a.RootX = rootPosition.X; a.RootY = rootPosition.Y; a.RootAngle = rootAngle;
        a.HasPose = true;
    }

    /// <summary>Forgets the animation pose: bones with weight 0 are driven to the bind pose instead of pinned.</summary>
    /// <exception cref="InvalidOperationException">The ragdoll is not active.</exception>
    public void ClearAnimationPose()
    {
        using var replayMutation = _world?.RecordMutation("Ragdoll.ClearAnimationPose");
        var a = ResolveActive();
        a.HasPose = false; Array.Clear(a.Pose);
        a.RootX = 0; a.RootY = 0; a.RootAngle = 0f;
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
            double dx = q[0] - pc[0], dy = q[1] - pc[1];
            float dist = (float)Math.Sqrt(dx * dx + dy * dy);
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
    public void Hit(int part, Vector2F impulse, float fullStrengthImpulse = 30f)
    {
        var a = ResolveActive();
        Part(Resolve(), part);
        if (!float.IsFinite(impulse.X) || !float.IsFinite(impulse.Y)) { throw new ArgumentOutOfRangeException(nameof(impulse), impulse, "The impulse must be finite."); }
        if (!(fullStrengthImpulse > 0f) || !float.IsFinite(fullStrengthImpulse)) { throw new ArgumentOutOfRangeException(nameof(fullStrengthImpulse), fullStrengthImpulse, "Must be finite and above 0."); }
        using var replayMutation = _world?.RecordMutation("Ragdoll.Hit");
        var body = GetBody(part);
        body.ApplyImpulse(impulse);
        float m = float.Sqrt(impulse.X * impulse.X + impulse.Y * impulse.Y) / fullStrengthImpulse;
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
