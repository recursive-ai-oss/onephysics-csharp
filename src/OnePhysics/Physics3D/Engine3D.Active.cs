using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Active;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The saved and live state of one active ragdoll (3D): the common controller state plus the animation pose.</summary>
internal sealed class ActiveState3D : ActiveCommon
{
    public QuaternionF[] Pose = Array.Empty<QuaternionF>();     // animation pose: one rotation per part, like Ragdoll.DriveToPose
    public QuaternionF[] Cur = Array.Empty<QuaternionF>();      // the controller's rate-limited drive rotations (what the drive actually chases)
    public Vector3D Root;                                       // where the animation puts the ragdoll space's origin
    public QuaternionF RootRot = QuaternionF.Identity;          // and its orientation

    // Derived (never saved).
    public float L1, L2, AnkleHeight, FootBack, FootFront, FootHalfWidth, TotalMass;
    public Vector3D[] TargetPos = Array.Empty<Vector3D>();
    public QuaternionF[] TargetRot = Array.Empty<QuaternionF>(), Delta = Array.Empty<QuaternionF>(), Rel = Array.Empty<QuaternionF>(), PoseScratch = Array.Empty<QuaternionF>();
    public bool[] Owned = Array.Empty<bool>(), TorqueMode = Array.Empty<bool>();
    public float[] ISub = Array.Empty<float>(), ILoad = Array.Empty<float>();
    public int SwingLeg = -1;
    public bool HasLanding;
    public double LandX, LandY, LandZ;
    public bool Stance0, Stance1;
    public Func<Shape, bool>? GroundFilter;
    public ActiveRagdollInfo Info;

    public void Allocate(int parts)
    {
        AllocateCommon(parts);
        Pose = new QuaternionF[parts]; Cur = new QuaternionF[parts];
        for (int i = 0; i < parts; i++) { Pose[i] = QuaternionF.Identity; Cur[i] = QuaternionF.Identity; }
        Derive(parts);
    }

    public void Derive(int parts)
    {
        TargetPos = new Vector3D[parts]; TargetRot = new QuaternionF[parts]; Delta = new QuaternionF[parts]; Rel = new QuaternionF[parts]; PoseScratch = new QuaternionF[parts];
        Owned = new bool[parts]; TorqueMode = new bool[parts]; ISub = new float[parts]; ILoad = new float[parts];
        GroundFilter = null; Info = default;
    }

    public ActiveState3D Clone()
    {
        var c = new ActiveState3D();
        CloneCommonInto(c);
        c.Pose = (QuaternionF[])Pose.Clone(); c.Cur = (QuaternionF[])Cur.Clone(); c.Root = Root; c.RootRot = RootRot;
        c.Derive(Parts);
        c.L1 = L1; c.L2 = L2; c.AnkleHeight = AnkleHeight; c.FootBack = FootBack; c.FootFront = FootFront; c.FootHalfWidth = FootHalfWidth; c.TotalMass = TotalMass;
        c.Info = Info;
        return c;
    }
}

/// <summary>Tuned constants of the 3D walking controller's heading and turning (fixed numbers, never read from the environment).</summary>
internal static class Turn3D
{
    internal const float FrameLead = 0.12f;      // how far ahead of the pelvis the gait's frame may point towards the heading, radians
    internal const float TurnFree = 0.3f;        // heading error that costs no speed, radians
    internal const float TurnSlow = 0.6f;        // fraction of the speed lost per radian of heading error beyond that
    internal const float TurnMin = 0.4f;         // the slowest a turning figure goes, as a fraction of the commanded speed
    internal const float YawRate = 2f;           // the pelvis turns towards the heading at this many 1/s times the heading error ...
    internal const float YawRateMax = 1f;        // ... and at most this fast, rad/s
    internal const float SwingTwistRate = 1f;    // a swinging leg turns to face the heading at most this fast, rad/s
}

internal sealed partial class Engine3D
{
    /// <summary>Active layers by ragdoll slot (null where a ragdoll has none).</summary>
    public ActiveState3D?[] Actives = new ActiveState3D?[4];

    public int ActiveCount;

    private ISnapshotSection? _activeSection;

    internal ISnapshotSection ActiveSnapshotSection => _activeSection ??= new ActiveSection3D(this);

    private const float TwoPiF = 6.2831855f;

    internal void DropActive(int r)
    {
        if (r < Actives.Length && Actives[r] != null) { Actives[r] = null; ActiveCount--; }
    }

    internal void AddDebugActive(DebugGeometry g)
    {
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var a = ActiveOf(r);
            if (a == null) { continue; }
            var i = a.Info;
            g.Add(new DebugActiveRagdoll(r, RagHandles.GenerationOf(r), i.Mode, i.Phase, i.GroundedFeet, new Vector3D(i.CenterOfMassX, i.CenterOfMassHeight, i.CenterOfMassZ), new Vector3D(i.CapturePointX, 0, i.CapturePointZ),
                new Vector3D(i.SupportMinX, 0, i.SupportMinZ), new Vector3D(i.SupportMaxX, 0, i.SupportMaxZ), a.HasLanding, new Vector3D(a.LandX, a.LandY, a.LandZ)));
        }
    }

    internal ActiveState3D? ActiveOf(int r) => r < Actives.Length ? Actives[r] : null;

    internal ActiveState3D EnableActive(int r, ActiveRagdollDef def)
    {
        var rg = Ragdolls[r]!;
        var d = def.Clone();
        d.Validate(rg.Parts);
        BipedSkeleton.Check(d.Bones, i => rg.Def.Parts[i].Parent);
        var a = new ActiveState3D { Def = d };
        a.Allocate(rg.Parts);
        DeriveActiveGeometry(a, rg);
        if (r >= Actives.Length) { Array.Resize(ref Actives, Math.Max(r + 1, Actives.Length * 2)); }
        if (Actives[r] == null) { ActiveCount++; }
        else { ReleaseAllDrives(r, Actives[r]!, Ragdolls[r]!); }
        Actives[r] = a;
        return a;
    }

    private static void DeriveActiveGeometry(ActiveState3D a, RagdollState rg)
    {
        var parts = rg.Def.Parts;
        var b = a.Def.Bones;
        float Dist(RagdollPartDef p, RagdollPartDef q) { float dx = p.JointPosition.X - q.JointPosition.X, dy = p.JointPosition.Y - q.JointPosition.Y, dz = p.JointPosition.Z - q.JointPosition.Z; return float.Sqrt(dx * dx + dy * dy + dz * dz); }
        var thigh = parts[b.LeftThigh]; var shin = parts[b.LeftShin]; var foot = parts[b.LeftFoot];
        a.L1 = Math.Max(Dist(thigh, shin), 0.05f); a.L2 = Math.Max(Dist(shin, foot), 0.05f);
        float hx = foot.ShapeHalfSize.X, hy = foot.ShapeHalfSize.Y, hz = foot.ShapeHalfSize.Z;
        float extZ = hz, extY = hy, extX = hx;
        if (foot.Shape == RagdollShapeKind.Sphere) { extX = extY = extZ = hx; }
        else if (foot.Shape == RagdollShapeKind.Capsule) { extZ = hx + hy; extY = hy; extX = hy; }
        a.AnkleHeight = Math.Max(foot.JointPosition.Y - (foot.Position.Y - extY), 0.01f);
        a.FootBack = Math.Max(foot.JointPosition.Z - (foot.Position.Z - extZ), 0.01f);
        a.FootFront = Math.Max((foot.Position.Z + extZ) - foot.JointPosition.Z, 0.01f);
        a.FootHalfWidth = Math.Max(extX, 0.01f);
        float m = 0f;
        for (int i = 0; i < parts.Length; i++) { m += parts[i].Mass; }
        a.TotalMass = m;
    }

    private float Gravity3D { get { float g = -Opt.Gravity.Y; return g > 0.1f ? g : 9.81f; } }

    // ---- helpers ----

    private static QuaternionF Rx(float theta) { var (s, c) = Math64.SinCos((double)theta * 0.5); return new QuaternionF((float)s, 0f, 0f, (float)c); }

    private static QuaternionF Rz(float theta) { var (s, c) = Math64.SinCos((double)theta * 0.5); return new QuaternionF(0f, 0f, (float)s, (float)c); }

    private static QuaternionF Ry(float theta) { var (s, c) = Math64.SinCos((double)theta * 0.5); return new QuaternionF(0f, (float)s, 0f, (float)c); }

    // Where a limb's axis points (the local direction D0, up when up is true and down otherwise), as pitch rho and roll phi about the level frame, as the angle convention in BipedGait describes.
    private static void LimbAngles(QuaternionF delta, QuaternionF yawInv, bool up, out float rho, out float phi)
    {
        var e = yawInv * delta;
        var d = e.Rotate(new Vector3F(0f, up ? 1f : -1f, 0f));
        float dx = d.X, dy = d.Y, dz = d.Z;
        if (up)
        {
            phi = -(float)Math64.Asin(Math.Clamp((double)dx, -1.0, 1.0));
            rho = -(float)Math64.Atan2(dz, dy);
        }
        else
        {
            phi = (float)Math64.Asin(Math.Clamp((double)dx, -1.0, 1.0));
            rho = (float)Math64.Atan2(dz, -dy);
        }
    }

    private static QuaternionF TargetDelta(QuaternionF yaw, float rho, float phi) => yaw * Rx(-rho) * Rz(phi);

    private (double X, double Y, double Z) AnchorOf(int body, float lx, float ly, float lz)
    {
        var p = Core.Bodies.PositionOf(body);
        var q = RotationOfBody(body);
        var off = q.Rotate(new Vector3F(lx, ly, lz) - LocalCenterOf(body));
        return (p[0] + off.X, p[1] + off.Y, p[2] + off.Z);
    }

    private QuaternionF DeltaOf(RagdollState rg, int i) => RotationOfBody(rg.Body[i]) * rg.BindRot[i].Conjugated();

    private bool GroundRay(ActiveState3D a, int r, double x, double y, double z, float length, out double groundY, out Vector3F normal)
    {
        groundY = 0; normal = new Vector3F(0f, 1f, 0f);
        a.GroundFilter ??= sh => RagdollOfBody(sh.Body.RawHandle.Index) < 0;   // no figure is ground, this one's parts or another's (a swing foot would land on a passer-by)
        var filter = new QueryFilter3D { Predicate = a.GroundFilter };
        if (!World!.Raycast(new Vector3D(x, y, z), new Vector3F(0f, -1f, 0f), out var hit, length, in filter)) { return false; }
        groundY = hit.Point.Y; normal = hit.Normal;
        return true;
    }

    // ---- the stage ----

    /// <summary>
    /// Active ragdolls, once per step before the solver, in ragdoll slot order: the controller (when a mode is on) decides joint targets; every bone then follows its target with the joint's stable spring-damper drive
    /// (implicit in the step's end state, Tan, Liu and Turk 2011), is pinned to the animation pose (weight 0), or is left free (weight 1).
    /// </summary>
    public void StepActive(float dt)
    {
        if (ActiveCount == 0) { return; }
        BindJoints();
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var a = r < Actives.Length ? Actives[r] : null;
            if (a == null) { continue; }
            StepActiveOne(r, a, Ragdolls[r]!, dt);
        }
    }

    private void StepActiveOne(int r, ActiveState3D a, RagdollState rg, float dt)
    {
        var bodies = Core.Bodies;
        int parts = rg.Parts;
        a.DecayHits(dt);
        // A layer with nothing to do (no controller, every bone fully simulated) leaves the ragdoll exactly as a plain ragdoll: no drive, no pin, no waking.
        bool idle = a.Mode == (byte)ActiveMode.Pose;
        for (int i = 0; i < parts && idle; i++) { if (a.EffectiveWeight(i) < 1f) { idle = false; } }
        if (idle)
        {
            for (int i = 1; i < parts; i++) { if (a.Driven[i] == 1) { ReleaseJoint(rg, i); } }
            Array.Clear(a.Driven);
            return;
        }

        for (int i = 0; i < parts; i++)
        {
            int b = rg.Body[i];
            if (bodies.IsDynamic(b)) { Core.WakeIsland(bodies.Island[b]); bodies.SleepTime[b] = 0f; }
        }

        Array.Clear(a.Owned); Array.Clear(a.TorqueMode);
        a.SwingLeg = -1; a.Stance0 = a.Stance1 = false;
        ComputeInertias(a, rg);
        for (int i = 0; i < parts; i++) { a.Delta[i] = DeltaOf(rg, i); }
        float[] ankleTq = _ankleTq ??= new float[2];
        if (a.Mode != (byte)ActiveMode.Pose) { RunController(r, a, rg, dt); }

        var b0 = a.Def.Bones;
        bool rootAnimated = a.HasPose && a.Def.RootMode == AnimationRootMode.Animated;
        bool anyPin = false;
        for (int i = 0; i < parts; i++)
        {
            if (a.Owned[i] || !a.HasPose) { continue; }
            if (a.EffectiveWeight(i) == 0f && (i != 0 || rootAnimated) && (i == 0 || RagdollJointAlive(rg, i))) { anyPin = true; }
        }

        if (anyPin)
        {
            if (rootAnimated) { Fk(a, rg, a.Pose, a.Root, a.RootRot); }
            else
            {
                var q0 = RotationOfBody(rg.Body[0]); var p0 = bodies.PositionOf(rg.Body[0]);
                var space = q0 * rg.BindRot[0].Conjugated();
                var off = space.Rotate(rg.Def.Parts[0].Position);
                Array.Copy(a.Pose, a.PoseScratch, parts); a.PoseScratch[0] = QuaternionF.Identity;
                Fk(a, rg, a.PoseScratch, new Vector3D(p0[0] - off.X, p0[1] - off.Y, p0[2] - off.Z), space);
            }
        }

        _ = ankleTq;
        for (int i = 0; i < parts; i++)
        {
            float w = a.EffectiveWeight(i);
            float strength = 1f - w;
            bool owned = a.Owned[i];
            if (i == 0)
            {
                if (a.HasPose && a.Def.RootMode != AnimationRootMode.Physics && w < 1f && !(rootAnimated && w == 0f)) { RootLeash(a, rg, 1f - w, dt); a.Driven[0] = 1; }
                else if (rootAnimated && w == 0f) { PinBody(a, rg, 0, dt); a.Driven[0] = 2; }
                else { a.Driven[0] = 0; }
                continue;
            }

            if (!RagdollJointAlive(rg, i)) { a.Driven[i] = 0; continue; }
            if (!owned && a.HasPose && w == 0f)
            {
                if (a.Driven[i] == 1) { ReleaseJoint(rg, i); }
                PinBody(a, rg, i, dt);
                a.Driven[i] = 2;
                continue;
            }

            if (strength > 0f)
            {
                var target = owned ? a.Rel[i] : (a.HasPose ? a.Pose[i] : QuaternionF.Identity);
                if (owned)
                {
                    // Rate-limit the controller's target; start from where the joint is.
                    if (a.Driven[i] != 1) { a.Cur[i] = ActualRelative(a, rg, i); }
                    a.Cur[i] = Approach(a.Cur[i], target, (a.Mode == (byte)ActiveMode.Walk ? a.Def.Walker.TargetSlew : a.Def.TargetSlew) * dt);
                    target = a.Cur[i];
                }

                bool leg = i == b0.LeftThigh || i == b0.LeftShin || i == b0.LeftFoot || i == b0.RightThigh || i == b0.RightShin || i == b0.RightFoot;
                bool load = (i == b0.LeftFoot || i == b0.RightFoot) && a.TorqueMode[i];
                float torqueScale = 1f;
                bool swingLegPart = a.SwingLeg >= 0 && (i == b0.Leg(a.SwingLeg, 0) || i == b0.Leg(a.SwingLeg, 1) || i == b0.Leg(a.SwingLeg, 2));
                if (swingLegPart) { torqueScale = a.Def.SwingTorqueScale; }
                // While walking the legs that carry the body are driven softly, the swinging leg and the upper body have their own gains (see the 2D engine).
                bool walkingNow = a.Mode == (byte)ActiveMode.Walk;
                float hz = !leg ? (walkingNow ? a.Def.Walker.UpperBodyHertz : a.Def.PdHertz)
                    : walkingNow ? ((a.SwingLeg >= 0 && !swingLegPart) ? a.Def.LoadPdHertz : a.Def.Walker.SwingLegHertz) : a.Def.LegPdHertz;
                // A leg that carries the body is stiff against the whole weight it holds: the drive is designed for the inertia of everything else about the joint (the planted ankle turns the whole body above it).
                float inertia = (walkingNow && leg && !swingLegPart) ? a.ILoad[i] : a.ISub[i];
                DrivePose(a, rg, i, target, strength, hz, inertia, torqueScale * (walkingNow ? a.Def.Walker.TorqueScale : 1f), load);
                a.Driven[i] = 1;
            }
            else { ReleaseJoint(rg, i); a.Driven[i] = 0; }
        }
    }

    private float[]? _ankleTq;

    // A planted ankle is a torque source: a soft, lightly damped spring whose target sits at the actual angle plus torque / stiffness, so that the torque delivered is the one asked for and a lean does not meet a heavy damper.
    internal const float TorqueSourceStiffness = 600f, TorqueSourceDamping = 6f;

    // The rotation a part has relative to its parent now, as a pose entry (in the bind pose's axes): parent delta^-1 * child delta.
    private static QuaternionF ActualRelative(ActiveState3D a, RagdollState rg, int i) => rg.Def.Parts[i].Parent < 0 ? QuaternionF.Identity : Normalized(a.Delta[rg.Def.Parts[i].Parent].Conjugated() * a.Delta[i]);

    private static QuaternionF Normalized(QuaternionF q)
    {
        float l = float.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
        return l > 0f ? new QuaternionF(q.X / l, q.Y / l, q.Z / l, q.W / l) : QuaternionF.Identity;
    }

    // Moves a rotation towards a target by at most maxAngle radians (the shortest way).
    private static QuaternionF Approach(QuaternionF from, QuaternionF to, float maxAngle)
    {
        var dq = to * from.Conjugated();
        if (dq.W < 0f) { dq = new QuaternionF(-dq.X, -dq.Y, -dq.Z, -dq.W); }
        float sn = float.Sqrt(dq.X * dq.X + dq.Y * dq.Y + dq.Z * dq.Z);
        if (sn < 1e-9f) { return to; }
        float angle = (float)(2.0 * Math64.Atan2(sn, dq.W));
        if (angle <= maxAngle) { return to; }
        float k = maxAngle / angle;
        var (s, c) = Math64.SinCos((double)(angle * k) * 0.5);
        float inv = (float)s / sn;
        var step = new QuaternionF(dq.X * inv, dq.Y * inv, dq.Z * inv, (float)c);
        return Normalized(step * from);
    }

    private void DrivePose(ActiveState3D a, RagdollState rg, int part, QuaternionF d, float strength, float hertz, float inertia, float torqueScale, bool torqueSource)
    {
        int c = rg.Joint[part];
        ref var jd = ref Joints[c];
        var rj = rg.JointRot[part];
        var target = Normalized(rj.Conjugated() * Normalized(d) * rj);   // the pose rotation in the joint frame
        float w = TwoPiF * hertz;
        float kp = inertia * w * w * strength, kd = 2f * a.Def.PdDampingRatio * inertia * w * strength;
        if (torqueSource) { kp = TorqueSourceStiffness * strength; kd = TorqueSourceDamping * strength; }
        float cap = rg.Def.Parts[part].MaxTorque * a.Def.TorqueScale * strength * torqueScale;
        if (!(cap > 0f) || !float.IsFinite(kp) || !float.IsFinite(kd)) { ReleaseJoint(rg, part); return; }
        jd.Flags |= JointData3D.FlagDrive;
        jd.DriveTarget = target; jd.Length = kp; jd.Ratio = kd; jd.MaxMotor = cap;
    }

    private void ReleaseJoint(RagdollState rg, int part)
    {
        int c = rg.Joint[part];
        if (c < 0 || !RagdollJointAlive(rg, part)) { return; }
        ref var jd = ref Joints[c];
        if (!jd.Drive) { return; }
        jd.Flags = (byte)(jd.Flags & ~JointData3D.FlagDrive);
        jd.DriveTarget = default; jd.Length = 0f; jd.Ratio = 0f; jd.MaxMotor = 0f;
        var p = Core.Constraints.Payload(c);
        p[JP.Drive] = 0f; p[JP.Drive + 1] = 0f; p[JP.Drive + 2] = 0f;
    }

    private void ReleaseAllDrives(int r, ActiveState3D a, RagdollState rg)
    {
        for (int i = 1; i < rg.Parts; i++) { if (a.Driven[i] == 1) { ReleaseJoint(rg, i); } }
    }

    // Pins a bone to its forward-kinematics target: the velocity that reaches it in one step now (so contacts see it move), the exact pose at the end of the step (EndActive).
    private void PinBody(ActiveState3D a, RagdollState rg, int i, float dt)
    {
        var bodies = Core.Bodies;
        int b = rg.Body[i];
        var p = bodies.PositionOf(b);
        float inv = 1f / dt;
        var t = a.TargetPos[i];
        var v = bodies.LinearVelocityOf(b);
        v[0] = (float)(t.X - p[0]) * inv; v[1] = (float)(t.Y - p[1]) * inv; v[2] = (float)(t.Z - p[2]) * inv;
        var dq = a.TargetRot[i] * RotationOfBody(b).Conjugated();
        if (dq.W < 0f) { dq = new QuaternionF(-dq.X, -dq.Y, -dq.Z, -dq.W); }
        float sn = float.Sqrt(dq.X * dq.X + dq.Y * dq.Y + dq.Z * dq.Z);
        var av = bodies.AngularVelocityOf(b);
        if (sn > 1e-12f)
        {
            float angle = (float)(2.0 * Math64.Atan2(sn, dq.W));
            float k = angle / sn * inv;
            av[0] = dq.X * k; av[1] = dq.Y * k; av[2] = dq.Z * k;
        }
        else { av[0] = 0f; av[1] = 0f; av[2] = 0f; }
    }

    /// <summary>After the step: bones pinned to the animation are put exactly on it (the same bits <c>Ragdoll.SetPose</c> would give).</summary>
    public void EndActive()
    {
        if (ActiveCount == 0) { return; }
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            if (!RagHandles.IsLive(r)) { continue; }
            var a = r < Actives.Length ? Actives[r] : null;
            if (a == null) { continue; }
            var rg = Ragdolls[r]!;
            for (int i = 0; i < rg.Parts; i++)
            {
                if (a.Driven[i] != 2) { continue; }
                new Body(World!, new BodyHandle(World!.Id, rg.Body[i], rg.BodyGen[i])).SetTransform(a.TargetPos[i], a.TargetRot[i]);
            }
        }
    }

    // Forward kinematics of the bind pose with each part turned by its pose entry: the same arithmetic as Ragdoll.SetPose, so pinned bones land on its bits.
    private static void Fk(ActiveState3D a, RagdollState rg, ReadOnlySpan<QuaternionF> pose, Vector3D at, QuaternionF rotation)
    {
        var parts = rg.Def.Parts;
        var rot = rotation.LengthSquared() == 0f ? QuaternionF.Identity : Engine3D.Normalize(rotation, nameof(rotation));
        Span<QuaternionF> delta = a.TargetRot;
        var pos = a.TargetPos;
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

        for (int i = 0; i < parts.Length; i++) { delta[i] = delta[i] * rg.BindRot[i]; }
    }

    // The root leash (CenterOfMassTracked, and a partly weighted animated root): a spring-damper on the whole figure's centre of mass towards the animation's, as an acceleration on every part (internal forces cancel).
    private void RootLeash(ActiveState3D a, RagdollState rg, float strength, float dt)
    {
        var parts = rg.Def.Parts;
        Fk(a, rg, a.Pose, a.Root, a.RootRot);
        double tx = 0, ty = 0, tz = 0, cx = 0, cy = 0, cz = 0, vx = 0, vy = 0, vz = 0, mt = 0;
        var bodies = Core.Bodies;
        for (int i = 0; i < parts.Length; i++)
        {
            float m = parts[i].Mass;
            var p = bodies.PositionOf(rg.Body[i]); var v = bodies.LinearVelocityOf(rg.Body[i]);
            tx += m * a.TargetPos[i].X; ty += m * a.TargetPos[i].Y; tz += m * a.TargetPos[i].Z;
            cx += m * p[0]; cy += m * p[1]; cz += m * p[2]; vx += m * v[0]; vy += m * v[1]; vz += m * v[2]; mt += m;
        }

        float w = TwoPiF * a.Def.RootTrackHertz;
        float kp = w * w * strength, kd = 2f * w * strength;
        float den = 1f + dt * kd + dt * dt * kp;
        float ex = (float)((tx - cx) / mt), ey = (float)((ty - cy) / mt), ez = (float)((tz - cz) / mt);
        float ux = (float)(vx / mt), uy = (float)(vy / mt), uz = (float)(vz / mt);
        float ax = (kp * (ex - dt * ux) - kd * ux) / den, ay = (kp * (ey - dt * uy) - kd * uy) / den, az = (kp * (ez - dt * uz) - kd * uz) / den;
        for (int i = 0; i < parts.Length; i++)
        {
            int b = rg.Body[i];
            float m = parts[i].Mass;
            Force[3 * b] += m * ax; Force[3 * b + 1] += m * ay; Force[3 * b + 2] += m * az;
        }
    }

    // The inertia each joint's drive is designed for: the subtree below the joint about the joint, except a planted foot's ankle, which turns the whole body above it (see the 2D engine's note).
    private void ComputeInertias(ActiveState3D a, RagdollState rg)
    {
        var parts = rg.Def.Parts; var bodies = Core.Bodies;
        int n = parts.Length;
        Span<double> jx = stackalloc double[n], jy = stackalloc double[n], jz = stackalloc double[n];
        for (int i = 0; i < n; i++)
        {
            if (i == 0) { var p0 = bodies.PositionOf(rg.Body[0]); jx[0] = p0[0]; jy[0] = p0[1]; jz[0] = p0[2]; continue; }
            if (!RagdollJointAlive(rg, i)) { var pb = bodies.PositionOf(rg.Body[i]); jx[i] = pb[0]; jy[i] = pb[1]; jz[i] = pb[2]; continue; }   // a part that came off turns about its own centre
            ref var j = ref Joints[rg.Joint[i]];
            var w = AnchorOf(rg.Body[i], j.LBx, j.LBy, j.LBz);
            jx[i] = w.X; jy[i] = w.Y; jz[i] = w.Z;
        }

        for (int i = 0; i < n; i++)
        {
            double sub = 0, all = 0;
            for (int k = 0; k < n; k++)
            {
                var pk = bodies.PositionOf(rg.Body[k]);
                double dx = pk[0] - jx[i], dy = pk[1] - jy[i], dz = pk[2] - jz[i];
                int b = rg.Body[k];
                float ixx = Inertia[6 * b], iyy = Inertia[6 * b + 3], izz = Inertia[6 * b + 5];
                float lo = Math.Min(ixx, Math.Min(iyy, izz));
                double ik = (double)(ixx + iyy + izz - lo) * 0.5;   // the mean of the two larger moments: what a limb turned across its length presents
                double ek = ik + parts[k].Mass * (dx * dx + dy * dy + dz * dz);
                all += ek;
                bool inSub = false;
                for (int q = k; q >= 0; q = parts[q].Parent) { if (q == i) { inSub = true; break; } }
                if (inSub) { sub += ek; }
            }

            a.ISub[i] = (float)Math.Max(sub, 1e-4);
            a.ILoad[i] = (float)Math.Max(all - sub, 1e-4);
        }
    }

    // ---- the controller ----

    private void RunController(int r, ActiveState3D a, RagdollState rg, float dt)
    {
        // A figure that has lost a limb (a broken joint, a destroyed body) has nothing for the controller to drive: it is left to the pose drive.
        var bz = a.Def.Bones;
        if (!RagdollJointAlive(rg, bz.Torso) || !RagdollJointAlive(rg, bz.LeftThigh) || !RagdollJointAlive(rg, bz.LeftShin) || !RagdollJointAlive(rg, bz.LeftFoot)
            || !RagdollJointAlive(rg, bz.RightThigh) || !RagdollJointAlive(rg, bz.RightShin) || !RagdollJointAlive(rg, bz.RightFoot) || (bz.Head >= 0 && !RagdollJointAlive(rg, bz.Head))) { a.HasLanding = false; return; }
        var bodies = Core.Bodies;
        var bn = a.Def.Bones;
        var parts = rg.Def.Parts;
        int pelvis = bn.Pelvis;
        var pp = bodies.PositionOf(rg.Body[pelvis]);
        double px = pp[0], py = pp[1], pz = pp[2];
        float g = Gravity3D;

        // heading of the pelvis and the level frame
        var dp = a.Delta[pelvis];
        var fwd = dp.Rotate(new Vector3F(0f, 0f, 1f));
        float psiPelvis = (float)Math64.Atan2(fwd.X, fwd.Z);
        float psi = psiPelvis;
        float headingErr = WrapAct(a.Heading - psiPelvis);
        // While walking, the gait is planned in a frame turned towards the commanded heading (a little ahead of the pelvis at most), not in the pelvis's own frame: the pelvis yaws a few degrees with every step, and
        // feet placed in a frame that wobbles with it land sideways of where the plan wants them.
        float frameLead = 0f;
        bool walkFrame = a.Mode == (byte)ActiveMode.Walk;
        if (walkFrame) { frameLead = BipedGait.Clamp(headingErr, -Turn3D.FrameLead, Turn3D.FrameLead); psi = psiPelvis + frameLead; }
        float sinP = (float)Math64.Sin((double)psi), cosP = (float)Math64.Cos((double)psi);
        var yaw = Ry(psi); var yawInv = yaw.Conjugated();
        float fx = sinP, fz = cosP, lx = cosP, lz = -sinP;   // forward and left in the world's x-z plane

        double cx = 0, cy = 0, cz = 0, vx = 0, vy = 0, vz = 0, mt = 0;
        for (int i = 0; i < parts.Length; i++)
        {
            float m = parts[i].Mass;
            var p = bodies.PositionOf(rg.Body[i]); var v = bodies.LinearVelocityOf(rg.Body[i]);
            cx += m * p[0]; cy += m * p[1]; cz += m * p[2]; vx += m * v[0]; vy += m * v[1]; vz += m * v[2]; mt += m;
        }

        cx /= mt; cy /= mt; cz /= mt; vx /= mt; vy /= mt; vz /= mt;
        float Fo(double x, double z) => (float)(x * fx + z * fz);
        float Le(double x, double z) => (float)(x * lx + z * lz);

        var inp = new BipedIn { Dt = dt, Gravity = g, Planar = false, Walk = a.Mode == (byte)ActiveMode.Walk, TorsoLean = a.Def.Walker.TorsoLean };
        float cmdX = a.CmdX, cmdZ = a.CmdZ;
        float sp = float.Sqrt(cmdX * cmdX + cmdZ * cmdZ), maxSp = a.Def.Walker.MaxSpeed;
        if (sp > maxSp) { cmdX *= maxSp / sp; cmdZ *= maxSp / sp; }
        inp.CmdF = cmdX * fx + cmdZ * fz; inp.CmdL = cmdX * lx + cmdZ * lz;
        if (walkFrame)
        {
            // The gait walks forward along its own heading: the speed it takes is the part of the commanded velocity that lies along the commanded heading, and it is lowered while the figure is still turned away from that heading.
            float hs = (float)Math64.Sin((double)a.Heading), hc = (float)Math64.Cos((double)a.Heading);
            float along = cmdX * hs + cmdZ * hc;
            if (along < 0f) { along = 0f; }
            float excess = Math.Max(Math.Abs(headingErr) - Turn3D.TurnFree, 0f);
            float scale = BipedGait.Clamp(1f - Turn3D.TurnSlow * excess, Turn3D.TurnMin, 1f);
            inp.CmdF = along * scale; inp.CmdL = 0f;
        }

        inp.ComF = Fo(cx - px, cz - pz); inp.ComL = Le(cx - px, cz - pz); inp.ComU = (float)(cy - py);
        inp.VelF = (float)(vx * fx + vz * fz); inp.VelL = (float)(vx * lx + vz * lz);
        LimbAngles(a.Delta[pelvis], yawInv, true, out float pRho, out float pPhi);
        inp.PelvisRho = pRho; inp.PelvisPhi = pPhi;
        float yawErr = headingErr;
        inp.YawError = walkFrame ? headingErr - frameLead : headingErr;
        inp.Geom = new BipedGeom { L1 = a.L1, L2 = a.L2, AnkleHeight = a.AnkleHeight, FootBack = a.FootBack, FootFront = a.FootFront, FootHalfWidth = a.FootHalfWidth, Mass = (float)mt };

        Span<float> partRho = stackalloc float[parts.Length], partPhi = stackalloc float[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            bool up = i == bn.Pelvis || i == bn.Torso || i == bn.Head || i == bn.LeftFoot || i == bn.RightFoot;
            LimbAngles(a.Delta[i], yawInv, up, out partRho[i], out partPhi[i]);
        }

        for (int k = 0; k < 2; k++)
        {
            int th = bn.Leg(k, 0), sh = bn.Leg(k, 1), ft = bn.Leg(k, 2);
            ref var tj = ref Joints[rg.Joint[th]]; ref var fj = ref Joints[rg.Joint[ft]];
            var hip = AnchorOf(rg.Body[th], tj.LBx, tj.LBy, tj.LBz);
            var ankle = AnchorOf(rg.Body[ft], fj.LBx, fj.LBy, fj.LBz);
            ref var lg = ref (k == 0 ? ref inp.Left : ref inp.Right);
            lg.HipF = Fo(hip.X - px, hip.Z - pz); lg.HipL = Le(hip.X - px, hip.Z - pz); lg.HipU = (float)(hip.Y - py);
            lg.AnkleF = Fo(ankle.X - px, ankle.Z - pz); lg.AnkleL = Le(ankle.X - px, ankle.Z - pz); lg.AnkleU = (float)(ankle.Y - py);
            lg.ThighRho = partRho[th]; lg.ThighPhi = partPhi[th]; lg.FootRho = partRho[ft]; lg.FootPhi = partPhi[ft]; lg.ShinRho = partRho[sh]; lg.ShinPhi = partPhi[sh];
            bool hit = GroundRay(a, r, ankle.X, ankle.Y + 0.05, ankle.Z, a.AnkleHeight + 0.45f, out double gy, out Vector3F n);
            lg.Grounded = hit && ankle.Y - gy <= a.AnkleHeight + 0.04f;
            lg.GroundU = (float)((hit ? gy : ankle.Y - a.AnkleHeight) - py);
            if (hit)
            {
                float nf = n.X * fx + n.Z * fz, nl = n.X * lx + n.Z * lz;
                lg.SlopeRho = (float)Math64.Atan2(-nf, n.Y); lg.SlopePhi = (float)Math64.Atan2(-nl, n.Y);
            }
        }

        var gait = a.Gait;
        if (gait.Phase == (byte)GaitPhase.SwingLeft || gait.Phase == (byte)GaitPhase.SwingRight)
        {
            int swing = gait.Phase == (byte)GaitPhase.SwingLeft ? 0 : 1;
            ref var stance = ref (swing == 0 ? ref inp.Right : ref inp.Left);
            double wx = px + (stance.AnkleF + gait.PlaceF) * fx + (stance.AnkleL + gait.PlaceL) * lx;
            double wz = pz + (stance.AnkleF + gait.PlaceF) * fz + (stance.AnkleL + gait.PlaceL) * lz;
            double ytop = Math.Max(py + inp.Left.AnkleU, py + inp.Right.AnkleU) + 0.3;
            if (GroundRay(a, r, wx, ytop, wz, 1.6f, out double gy, out Vector3F n))
            {
                ref var sw = ref (swing == 0 ? ref inp.Left : ref inp.Right);
                sw.PlaceDU = (float)(gy - (py + stance.GroundU));
                sw.PlaceSlopeRho = (float)Math64.Atan2(-(n.X * fx + n.Z * fz), n.Y);
            }
        }

        // Planning on measurements beyond what a figure can reach overflows into gait state that could not be saved; the controller sits this step out (as in 2D).
        if (!BipedGait.Sane(in inp)) { a.HasLanding = false; return; }
        var o = new BipedOut();
        var ap = new ActiveParams(a.Def.Balance, a.Def.Walker);
        BipedGait.Step(ref a.Gait, in ap, in inp, ref o);
        a.SwingLeg = a.Gait.Phase == (byte)GaitPhase.SwingLeft ? 0 : a.Gait.Phase == (byte)GaitPhase.SwingRight ? 1 : -1;
        a.Stance0 = inp.Left.Grounded && o.Left.Weight > 0f; a.Stance1 = inp.Right.Grounded && o.Right.Weight > 0f;

        // World-frame limb targets, turned into rotations relative to the actual parent.
        // Only a swinging leg turns towards the heading (so its foot lands facing it); a planted leg keeps the pelvis's own yaw, because a planted foot cannot turn and a twist drive against it would spin the pelvis.
        var swingYawQ = a.Mode == (byte)ActiveMode.Walk ? Ry(psi + o.StanceYaw) : yaw;
        bool walkingYaw = a.Mode == (byte)ActiveMode.Walk;
        // The pelvis is turned by the planted hip: each step it moves by the turn rate times the time step (a position servo that is set to a point ahead of the actual angle turns at a rate, not by an angle).
        float yawEps = BipedGait.Clamp(Turn3D.YawRate * yawErr, -Turn3D.YawRateMax, Turn3D.YawRateMax) * dt;
        var torsoT = TargetDelta(yaw, o.TorsoRho, o.TorsoPhi);
        Own(a, bn.Torso, Normalized(a.Delta[pelvis].Conjugated() * torsoT));
        if (bn.Head >= 0) { Own(a, bn.Head, Normalized(a.Delta[bn.Torso].Conjugated() * torsoT)); }
        for (int k = 0; k < 2; k++)
        {
            ref var lo = ref (k == 0 ? ref o.Left : ref o.Right);
            int th = bn.Leg(k, 0), sh = bn.Leg(k, 1), ft = bn.Leg(k, 2);
            QuaternionF thighT, footT;
            float footRho = lo.FootRho, footPhi = lo.FootPhi;
            if (lo.AnkleTorque)
            {
                // Torque control through the drive's own stiffness: the target sits where the spring gives exactly that torque.
                float kp = TorqueSourceStiffness;
                footRho = partRho[ft] + lo.TorqueRho / kp;
                footPhi = partPhi[ft] + lo.TorquePhi / kp;
                a.TorqueMode[ft] = true;
            }

            if (walkingYaw)
            {
                // Walking: where a limb points is decided in the gait's frame, and the twist of the limb about its own axis separately. A swinging leg is twisted to face the heading; a planted leg keeps the twist it has, and its hip
                // turns the pelvis towards the heading by a fraction of the error (the planted foot cannot turn, so the pelvis does).
                float twA = TwistOf(a.Delta[th], partRho[th], partPhi[th], yaw), twFA = TwistOf(a.Delta[ft], partRho[ft], partPhi[ft], yaw);
                float stepTw = Turn3D.SwingTwistRate * dt;   // a swinging leg turns to face the heading over the swing, not in one jerk (the reaction would spin the pelvis)
                float twThigh = k == a.SwingLeg ? twA + BipedGait.Clamp(o.StanceYaw - twA, -stepTw, stepTw) : twA - yawEps;
                float twFoot = k == a.SwingLeg ? twFA + BipedGait.Clamp(o.StanceYaw - twFA, -stepTw, stepTw) : twFA;
                thighT = TargetDelta(yaw, lo.ThighRho, lo.ThighPhi) * Ry(twThigh);
                footT = TargetDelta(yaw, footRho, footPhi) * Ry(twFoot);
            }
            else
            {
                var legYawQ = k == a.SwingLeg ? swingYawQ : yaw;
                thighT = TargetDelta(legYawQ, lo.ThighRho, lo.ThighPhi);
                footT = TargetDelta(legYawQ, footRho, footPhi);
            }

            Own(a, th, Normalized(a.Delta[pelvis].Conjugated() * thighT));
            Own(a, sh, Rx(lo.KneeFlex));
            Own(a, ft, Normalized(a.Delta[sh].Conjugated() * footT));
        }

        a.HasLanding = a.SwingLeg >= 0;
        if (a.HasLanding)
        {
            ref var st = ref (a.SwingLeg == 0 ? ref inp.Right : ref inp.Left);
            a.LandX = px + (st.AnkleF + a.Gait.PlaceF) * fx + (st.AnkleL + a.Gait.PlaceL) * lx; a.LandZ = pz + (st.AnkleF + a.Gait.PlaceF) * fz + (st.AnkleL + a.Gait.PlaceL) * lz; a.LandY = py + st.GroundU;
        }

        a.Info = new ActiveRagdollInfo((ActiveMode)a.Mode, (GaitPhase)a.Gait.Phase, a.Gait.Steps, a.Gait.Time, (float)cx, (float)cy, (float)cz,
            (float)(px + o.CaptureF * fx + o.CaptureL * lx), (float)(pz + o.CaptureF * fz + o.CaptureL * lz),
            (float)(px + o.SupMinF * fx + o.SupMinL * lx), (float)(pz + o.SupMinF * fz + o.SupMinL * lz), (float)(px + o.SupMaxF * fx + o.SupMaxL * lx), (float)(pz + o.SupMaxF * fz + o.SupMaxL * lz), o.Grounded);
    }

    // The twist of a limb about its own axis, in the frame the limb's pitch and roll are measured in: the rotation left over once the pointing direction (rho, phi) is taken out.
    private static float TwistOf(QuaternionF delta, float rho, float phi, QuaternionF yaw)
    {
        var q = TargetDelta(yaw, rho, phi).Conjugated() * delta;
        if (q.W < 0f) { q = new QuaternionF(-q.X, -q.Y, -q.Z, -q.W); }
        return (float)(2.0 * Math64.Atan2(q.Y, q.W));
    }

    private static float WrapAct(float a)
    {
        const float Pi = 3.1415927f;
        while (a > Pi) { a -= 2f * Pi; }
        while (a < -Pi) { a += 2f * Pi; }
        return a;
    }

    private static void Own(ActiveState3D a, int part, QuaternionF rel)
    {
        a.Owned[part] = true;
        a.Rel[part] = rel;
    }

    // ------------------------------------------------------------------ snapshot section

    /// <summary>Id of the 3D active-ragdoll snapshot section ("ACT3").</summary>
    public const int ActiveSectionId = 0x33544341;

    internal void WriteActives(StateWriter w)
    {
        w.NamedInt32(1);   // section version
        int count = 0;
        for (int r = 0; r < RagHandles.HighWater; r++) { if (RagHandles.IsLive(r) && ActiveOf(r) != null) { count++; } }
        w.NamedInt32(count);
        for (int r = 0; r < RagHandles.HighWater; r++)
        {
            var a = RagHandles.IsLive(r) ? ActiveOf(r) : null;
            if (a == null) { continue; }
            w.DescribeEntity("ragdoll", r, RagHandles.GenerationOf(r));
            w.NamedInt32(r); w.NamedInt32(RagHandles.GenerationOf(r)); w.NamedInt32(a.Parts);
            a.WriteCommon(w);
            for (int i = 0; i < a.Parts; i++) { var q = a.Pose[i]; w.NamedSingle(q.X); w.NamedSingle(q.Y); w.NamedSingle(q.Z); w.NamedSingle(q.W); }
            for (int i = 0; i < a.Parts; i++) { var q = a.Cur[i]; w.NamedSingle(q.X); w.NamedSingle(q.Y); w.NamedSingle(q.Z); w.NamedSingle(q.W); }
            w.NamedDouble(a.Root.X); w.NamedDouble(a.Root.Y); w.NamedDouble(a.Root.Z);
            w.NamedSingle(a.RootRot.X); w.NamedSingle(a.RootRot.Y); w.NamedSingle(a.RootRot.Z); w.NamedSingle(a.RootRot.W);
        }
    }

    private static QuaternionF ReadUnit(StateReader rd, string what)
    {
        var q = new QuaternionF(rd.ReadSingle(), rd.ReadSingle(), rd.ReadSingle(), rd.ReadSingle());
        float l2 = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        if (!float.IsFinite(l2) || !(l2 > 1e-12f)) { throw rd.Error("invalid " + what); }
        return q;
    }

    private static ActiveState3D?[] ReadActives(StateReader rd, HandleTable ragHandles, RagdollState?[] ragdolls)
    {
        int version = rd.ReadInt32();
        if (version != 1) { throw rd.Error("unsupported 3D active ragdoll section version " + version.ToString(CultureInfo.InvariantCulture)); }
        int count = rd.ReadCount(4 + 4 + 4 + ActiveCommon.DefBytes, "active ragdolls");
        if (count < 1) { throw rd.Error("an active ragdoll section holding no ragdolls"); }
        var result = new ActiveState3D?[Math.Max(ragHandles.HighWater, 1)];
        int last = -1;
        for (int k = 0; k < count; k++)
        {
            int slot = rd.ReadInt32(), gen = rd.ReadInt32(), n = rd.ReadInt32();
            if (slot <= last || slot >= ragHandles.HighWater || !ragHandles.IsLive(slot) || ragHandles.GenerationOf(slot) != gen) { throw rd.Error("an active ragdoll that names no live ragdoll"); }
            last = slot;
            var rg = ragdolls[slot]!;
            if (n != rg.Parts) { throw rd.Error("an active ragdoll with " + n.ToString(CultureInfo.InvariantCulture) + " parts for a ragdoll of " + rg.Parts.ToString(CultureInfo.InvariantCulture)); }
            var a = new ActiveState3D();
            a.ReadCommon(rd, n);
            try { BipedSkeleton.Check(a.Def.Bones, i => rg.Def.Parts[i].Parent); }
            catch (ArgumentException e) { throw rd.Error("invalid active ragdoll bones: " + e.Message); }
            a.Pose = new QuaternionF[n]; a.Cur = new QuaternionF[n];
            for (int i = 0; i < n; i++) { a.Pose[i] = ReadUnit(rd, "active ragdoll pose"); }
            for (int i = 0; i < n; i++) { a.Cur[i] = ReadUnit(rd, "active ragdoll target"); }
            a.Root = new Vector3D(rd.ReadDouble(), rd.ReadDouble(), rd.ReadDouble());
            a.RootRot = new QuaternionF(rd.ReadSingle(), rd.ReadSingle(), rd.ReadSingle(), rd.ReadSingle());
            float rl = a.RootRot.X * a.RootRot.X + a.RootRot.Y * a.RootRot.Y + a.RootRot.Z * a.RootRot.Z + a.RootRot.W * a.RootRot.W;
            if (!float.IsFinite(rl) || (rl != 0f && !(rl > 1e-12f))) { throw rd.Error("invalid active ragdoll root rotation"); }
            if (!double.IsFinite(a.Root.X) || !double.IsFinite(a.Root.Y) || !double.IsFinite(a.Root.Z) || Math.Abs(a.Root.X) > 2097152.0 || Math.Abs(a.Root.Y) > 2097152.0 || Math.Abs(a.Root.Z) > 2097152.0) { throw rd.Error("invalid active ragdoll root"); }
            a.Derive(n);
            DeriveActiveGeometry(a, rg);
            result[slot] = a;
        }

        return result;
    }

    private void CommitActives(ActiveState3D?[]? block)
    {
        if (block == null) { Array.Clear(Actives); ActiveCount = 0; return; }
        Actives = block;
        ActiveCount = 0;
        foreach (var s in block) { if (s != null) { ActiveCount++; } }
    }

    private sealed class ActiveSection3D(Engine3D e) : ISnapshotSection
    {
        public int SectionId => ActiveSectionId;
        public bool IsOptional => true;
        public bool IsPresent => e.ActiveCount > 0;

        public void Reset()
        {
            if (e.ActiveCount == 0) { return; }
            e.CommitActives(null);
        }

        public void Write(StateWriter w) => e.WriteActives(w);

        public void Read(StateReader r) => e.CommitActives(ReadActives(r, e.RagHandles, e.Ragdolls));

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine3D)destination.GetSnapshotSection(Engine3D.Id);
            dst.Actives = new ActiveState3D?[e.Actives.Length];
            dst.ActiveCount = e.ActiveCount;
            for (int i = 0; i < dst.Actives.Length; i++) { dst.Actives[i] = e.Actives[i]?.Clone(); }
            return dst.ActiveSnapshotSection;
        }
    }
}
