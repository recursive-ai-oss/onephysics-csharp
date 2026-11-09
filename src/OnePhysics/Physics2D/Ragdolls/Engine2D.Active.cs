using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Active;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The saved and live state of one active ragdoll (2D): the common controller state plus the animation pose.</summary>
internal sealed class ActiveState2D : ActiveCommon
{
    public float[] Cur = Array.Empty<float>();               // the controller's rate-limited joint targets (what the drive actually chases)
    public float[] Pose = Array.Empty<float>();              // animation pose: one joint angle per part, like Ragdoll.DriveToPose
    public double RootX, RootY;                              // where the animation puts the ragdoll space's origin
    public float RootAngle;                                  // and its orientation

    // Derived (never saved): geometry of the legs, scratch for forward kinematics and the controller, the ground filter, the last readout.
    public float L1, L2, AnkleHeight, FootBack, FootFront, FootHalfWidth, TotalMass;
    public double[] TargetX = Array.Empty<double>(), TargetY = Array.Empty<double>();
    public float[] TargetAngle = Array.Empty<float>(), Delta = Array.Empty<float>(), Rel = Array.Empty<float>(), PartRho = Array.Empty<float>();
    public bool[] Owned = Array.Empty<bool>();
    public bool PinsReady;
    public Func<Shape, bool>? GroundFilter;
    public ActiveRagdollInfo Info;
    public float[] AnkleTorque = Array.Empty<float>();
    public bool[] TorqueMode = Array.Empty<bool>();
    public int SwingLeg = -1;                                // the leg the controller is swinging (0 or 1), or -1; not saved

    public void Allocate(int parts)
    {
        AllocateCommon(parts);
        Pose = new float[parts]; Cur = new float[parts];
        Derive(parts);
    }

    public void Derive(int parts)
    {
        TargetX = new double[parts]; TargetY = new double[parts]; TargetAngle = new float[parts]; Delta = new float[parts]; Rel = new float[parts]; PartRho = new float[parts]; Owned = new bool[parts]; AnkleTorque = new float[parts]; TorqueMode = new bool[parts];
        PinsReady = false; GroundFilter = null; Info = default;
    }

    public ActiveState2D Clone()
    {
        var c = new ActiveState2D();
        CloneCommonInto(c);
        c.Pose = (float[])Pose.Clone(); c.Cur = (float[])Cur.Clone(); c.RootX = RootX; c.RootY = RootY; c.RootAngle = RootAngle;
        c.Derive(Parts);
        c.CopyGeometry(this);
        c.Info = Info;
        return c;
    }

    public void CopyGeometry(ActiveState2D s) { L1 = s.L1; L2 = s.L2; AnkleHeight = s.AnkleHeight; FootBack = s.FootBack; FootFront = s.FootFront; FootHalfWidth = s.FootHalfWidth; TotalMass = s.TotalMass; }
}

internal sealed partial class Engine2D
{
    /// <summary>Active layers by ragdoll slot (null where a ragdoll has none).</summary>
    public ActiveState2D?[] Actives = new ActiveState2D?[4];

    public int ActiveCount;

    private ISnapshotSection? _activeSection;

    internal ISnapshotSection ActiveSnapshotSection => _activeSection ??= new ActiveSection2D(this);

    private const float TwoPiF = 6.2831855f;

    /// <summary>Drops a ragdoll's active layer (when the ragdoll goes away).</summary>
    internal void DropActive(int r)
    {
        if (r < Actives.Length && Actives[r] != null) { Actives[r] = null; ActiveCount--; }
    }

    internal ActiveState2D? ActiveOf(int r) => r < Actives.Length ? Actives[r] : null;

    /// <summary>Makes a ragdoll active: validates the settings and derives the leg geometry from the ragdoll's definition.</summary>
    internal ActiveState2D EnableActive(int r, ActiveRagdollDef def)
    {
        var rg = Ragdolls[r]!;
        var d = def.Clone();
        d.Validate(rg.Parts);
        var a = new ActiveState2D { Def = d };
        a.Allocate(rg.Parts);
        a.HasPose = false;
        DeriveActiveGeometry(a, rg);
        if (r >= Actives.Length) { Array.Resize(ref Actives, Math.Max(r + 1, Actives.Length * 2)); }
        if (Actives[r] == null) { ActiveCount++; }
        else { ReleaseActiveDrives(r); }
        Actives[r] = a;
        return a;
    }

    internal void ReleaseActiveDrives(int r) { Actives[r]?.Driven.AsSpan().Clear(); }

    private static void DeriveActiveGeometry(ActiveState2D a, RagdollState rg)
    {
        var parts = rg.Def.Parts;
        var b = a.Def.Bones;
        float Dist(RagdollPartDef p, RagdollPartDef q) { float dx = p.JointPosition.X - q.JointPosition.X, dy = p.JointPosition.Y - q.JointPosition.Y; return float.Sqrt(dx * dx + dy * dy); }
        var thigh = parts[b.LeftThigh]; var shin = parts[b.LeftShin]; var foot = parts[b.LeftFoot];
        a.L1 = Math.Max(Dist(thigh, shin), 0.05f); a.L2 = Math.Max(Dist(shin, foot), 0.05f);
        float hx = foot.ShapeHalfSize.X, hy = foot.ShapeHalfSize.Y;
        float extX = hx, extY = hy;
        if (foot.Shape == RagdollShapeKind.Capsule) { extX = hx + hy; }
        else if (foot.Shape == RagdollShapeKind.Circle) { extX = hx; extY = hx; }
        a.AnkleHeight = Math.Max(foot.JointPosition.Y - (foot.Position.Y - extY), 0.01f);
        a.FootBack = Math.Max(foot.JointPosition.X - (foot.Position.X - extX), 0.01f);
        a.FootFront = Math.Max((foot.Position.X + extX) - foot.JointPosition.X, 0.01f);
        a.FootHalfWidth = 0.05f;
        float m = 0f;
        for (int i = 0; i < parts.Length; i++) { m += parts[i].Mass; }
        a.TotalMass = m;
    }

    private float Gravity2D { get { float g = -Opt.Gravity.Y; return g > 0.1f ? g : 9.81f; } }

    // ---- sensing helpers (2D: position is the body origin, which is the centre of every ragdoll shape) ----

    private float BodyDelta(RagdollState rg, int i)
    {
        var r = Core.Bodies.RotationOf(rg.Body[i]);
        return WrapPi(Math32.Atan2(r[1], r[0]) - rg.Def.Parts[i].Angle);
    }

    private (double X, double Y) ActAnchor(int body, float lx, float ly)
    {
        var p = Core.Bodies.PositionOf(body); var r = Core.Bodies.RotationOf(body);
        return (p[0] + (double)(r[0] * lx - r[1] * ly), p[1] + (double)(r[1] * lx + r[0] * ly));
    }

    private bool GroundRay(ActiveState2D a, int r, double x, double yTop, float length, out double groundY, out float slope)
    {
        groundY = 0; slope = 0f;
        a.GroundFilter ??= sh => RagdollOfBody(sh.Body.RawHandle.Index) != r;
        var filter = new QueryFilter2D { Predicate = a.GroundFilter };
        if (!World!.Raycast(new Vector2D(x, yTop), new Vector2F(0f, -1f), out var hit, length, in filter)) { return false; }
        groundY = hit.Point.Y;
        var n = hit.Normal;
        slope = (n.X == 0f && n.Y == 0f) ? 0f : Math32.Atan2(-n.X, n.Y);
        return true;
    }

    // ---- the stage ----

    /// <summary>
    /// Active ragdolls, once per step before the solver, in ragdoll slot order: the controller (when a mode is on) decides joint targets; every bone then follows its target with a stable proportional-derivative torque
    /// (Tan, Liu and Turk 2011: the torque is computed against the position and velocity one step ahead, so any gain is stable), is pinned to the animation pose (weight 0), or is left free (weight 1).
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

    private void StepActiveOne(int r, ActiveState2D a, RagdollState rg, float dt)
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

        Array.Clear(a.Owned);
        Array.Clear(a.TorqueMode);
        a.SwingLeg = -1;
        if (a.Mode != (byte)ActiveMode.Pose) { RunController(r, a, rg, dt); }

        // Forward kinematics of the animation pose, for pinned bones and the root leash.
        var b0 = a.Def.Bones;
        bool anyPin = false;
        bool rootAnimated = a.HasPose && a.Def.RootMode == AnimationRootMode.Animated;
        for (int i = 0; i < parts; i++)
        {
            if (a.Owned[i] || !a.HasPose) { continue; }
            if (a.EffectiveWeight(i) == 0f && (i != 0 || rootAnimated) && (i == 0 || RagdollJointAlive(rg, i))) { anyPin = true; }
        }

        if (anyPin)
        {
            if (rootAnimated) { Fk(a, rg, a.Pose, a.RootX, a.RootY, a.RootAngle); }
            else
            {
                var rot = bodies.RotationOf(rg.Body[0]); var pos = bodies.PositionOf(rg.Body[0]);
                float ang0 = Math32.Atan2(rot[1], rot[0]) - rg.Def.Parts[0].Angle;
                var (rs, rc) = Math64.SinCos((double)ang0);
                double ox = rg.Def.Parts[0].Position.X, oy = rg.Def.Parts[0].Position.Y;
                var pose0 = a.Rel;   // scratch: the pose with the root entry zeroed
                Array.Copy(a.Pose, pose0, parts); pose0[0] = 0f;
                Fk(a, rg, pose0, pos[0] - (rc * ox - rs * oy), pos[1] - (rs * ox + rc * oy), ang0);
            }
        }

        for (int i = 0; i < parts; i++)
        {
            float w = a.EffectiveWeight(i);
            float strength = 1f - w;
            int b = rg.Body[i];
            bool owned = a.Owned[i];
            if (i == 0)
            {
                // The root: pinned (weight 0, animated root) or leashed towards the animation's centre of mass.
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
                float target = owned ? a.Rel[i] : (a.HasPose ? a.Pose[i] : 0f);
                if (owned)
                {
                    // Rate-limit the controller's target; start from where the joint is.
                    if (a.Driven[i] != 1) { a.Cur[i] = RagdollJointAngle(rg.Joint[i]); }
                    float step = a.Def.TargetSlew * dt;
                    float dlt = WrapPi(target - a.Cur[i]);
                    a.Cur[i] += dlt > step ? step : dlt < -step ? -step : dlt;
                    target = a.Cur[i];
                }

                bool leg = i == b0.LeftThigh || i == b0.LeftShin || i == b0.LeftFoot || i == b0.RightThigh || i == b0.RightShin || i == b0.RightFoot;
                if (a.TorqueMode[i]) { DriveTorque(rg, i, a.AnkleTorque[i], strength); }
                else
                {
                    float torqueScale = 1f;
                    if (a.SwingLeg >= 0 && (i == b0.Leg(a.SwingLeg, 0) || i == b0.Leg(a.SwingLeg, 1) || i == b0.Leg(a.SwingLeg, 2))) { torqueScale = a.Def.SwingTorqueScale; }
                    DriveMotor(a, rg, i, target, strength, leg ? a.Def.LegPdHertz : a.Def.PdHertz, torqueScale, dt);
                }
                a.Driven[i] = 1;
            }
            else { a.Driven[i] = 0; }
        }

    }

    // Pins a bone to its forward-kinematics target: the velocity that reaches it in one step now (so contacts see it move), the exact pose at the end of the step (EndActive).
    private void PinBody(ActiveState2D a, RagdollState rg, int i, float dt)
    {
        var bodies = Core.Bodies;
        int b = rg.Body[i];
        var p = bodies.PositionOf(b); var r = bodies.RotationOf(b);
        float inv = 1f / dt;
        var v = bodies.LinearVelocityOf(b);
        v[0] = (float)(a.TargetX[i] - p[0]) * inv; v[1] = (float)(a.TargetY[i] - p[1]) * inv;
        float da = WrapPi(a.TargetAngle[i] - Math32.Atan2(r[1], r[0]));
        bodies.AngularVelocityOf(b)[0] = da * inv;
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
                new Body(World!, new BodyHandle(World!.Id, rg.Body[i], rg.BodyGen[i])).SetTransform(new Vector2D(a.TargetX[i], a.TargetY[i]), a.TargetAngle[i]);
            }
        }
    }

    // Forward kinematics of the bind pose with each part turned by its pose entry: the same arithmetic as Ragdoll.SetPose, so pinned bones land on its bits.
    private static void Fk(ActiveState2D a, RagdollState rg, ReadOnlySpan<float> pose, double atX, double atY, float angle)
    {
        var parts = rg.Def.Parts;
        var delta = a.Delta;
        var (rs, rc) = Math64.SinCos((double)angle);
        double ox = parts[0].Position.X, oy = parts[0].Position.Y;
        a.TargetX[0] = atX + (rc * ox - rs * oy); a.TargetY[0] = atY + (rs * ox + rc * oy);
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
            a.TargetX[i] = a.TargetX[p] + (cp * tx - sp * ty) - (cc * fx - sc * fy);
            a.TargetY[i] = a.TargetY[p] + (sp * tx + cp * ty) - (sc * fx + cc * fy);
        }

        for (int i = 0; i < parts.Length; i++) { a.TargetAngle[i] = delta[i] + parts[i].Angle; }
    }

    // The root leash (CenterOfMassTracked, and a partly weighted animated root): a spring-damper on the whole figure's centre of mass towards the animation's, as an acceleration on every part (internal forces cancel).
    private void RootLeash(ActiveState2D a, RagdollState rg, float strength, float dt)
    {
        var parts = rg.Def.Parts;
        Fk(a, rg, a.Pose, a.RootX, a.RootY, a.RootAngle);
        double tx = 0, ty = 0, cx = 0, cy = 0, mt = 0, vx = 0, vy = 0;
        var bodies = Core.Bodies;
        for (int i = 0; i < parts.Length; i++)
        {
            float m = parts[i].Mass;
            var p = bodies.PositionOf(rg.Body[i]); var v = bodies.LinearVelocityOf(rg.Body[i]);
            tx += m * a.TargetX[i]; ty += m * a.TargetY[i]; cx += m * p[0]; cy += m * p[1]; vx += m * v[0]; vy += m * v[1]; mt += m;
        }

        float w = TwoPiF * a.Def.RootTrackHertz;
        float kp = w * w * strength, kd = 2f * w * strength;
        float den = 1f + dt * kd + dt * dt * kp;
        float ex = (float)((tx - cx) / mt), ey = (float)((ty - cy) / mt);
        float ax = (kp * (ex - dt * (float)(vx / mt)) - kd * (float)(vx / mt)) / den, ay = (kp * (ey - dt * (float)(vy / mt)) - kd * (float)(vy / mt)) / den;
        for (int i = 0; i < parts.Length; i++)
        {
            int b = rg.Body[i];
            float m = parts[i].Mass;
            Force[2 * b] += m * ax; Force[2 * b + 1] += m * ay;
        }
    }

    // The pose drive of one joint: a velocity-level proportional controller, solved inside the constraint solver. The motor asks for the joint speed 2 pi f * (angle error), capped by the part's MaxDriveSpeed, and the solver supplies
    // whatever impulse that takes up to the part's torque limit. Because the solver resolves it against the true masses at the end of the step (the argument Tan, Liu and Turk 2011 make for stable PD controllers: act on the next
    // state, not the current one), no frequency can make it overshoot or blow up, and the torque limit keeps a stiff drive from throwing the figure. Strength scales both the speed gain and the torque limit.
    private void DriveMotor(ActiveState2D a, RagdollState rg, int part, float relTarget, float strength, float hertz, float torqueScale, float dt)
    {
        int c = rg.Joint[part];
        float err = WrapPi(relTarget - RagdollJointAngle(c));
        var p = rg.Def.Parts[part];
        // The speed that closes the gap k * (error left after this step): speed = k (err - speed dt), solved for the speed. For a large k it tends to err / dt, never past it, so the loop cannot overshoot at any frequency.
        float k = TwoPiF * hertz * strength;
        float speed = k * err / (1f + k * dt);
        if (speed > p.MaxDriveSpeed) { speed = p.MaxDriveSpeed; } else if (speed < -p.MaxDriveSpeed) { speed = -p.MaxDriveSpeed; }
        ref var d = ref Joints[c];
        d.Flags |= JointData2D.FlagMotor;
        d.MotorSpeed = speed;
        d.MaxMotor = p.MaxTorque * a.Def.TorqueScale * strength * torqueScale;
    }

    // A planted ankle as a torque source: the motor is told to turn the joint at a speed it will not reach, with the torque as its limit, so the foot receives exactly that torque (up to the joint's limit).
    private void DriveTorque(RagdollState rg, int part, float torque, float strength)
    {
        int c = rg.Joint[part];
        ref var d = ref Joints[c];
        d.Flags |= JointData2D.FlagMotor;
        d.MotorSpeed = torque >= 0f ? 40f : -40f;
        d.MaxMotor = (torque >= 0f ? torque : -torque) * strength;
    }

    private void ReleaseJoint(RagdollState rg, int part)
    {
        int c = rg.Joint[part];
        if (c < 0 || !RagdollJointAlive(rg, part)) { return; }
        ref var d = ref Joints[c];
        d.Flags = (byte)(d.Flags & ~JointData2D.FlagMotor);
        d.MotorSpeed = 0f; d.MaxMotor = 0f;
    }

    // ---- the controller ----

    private void RunController(int r, ActiveState2D a, RagdollState rg, float dt)
    {
        var bodies = Core.Bodies;
        var bn = a.Def.Bones;
        var parts = rg.Def.Parts;
        int pelvis = bn.Pelvis;
        var pp = bodies.PositionOf(rg.Body[pelvis]);
        double px = pp[0], py = pp[1];
        float g = Gravity2D;

        // centre of mass and its velocity
        double cx = 0, cy = 0, vx = 0, vy = 0, mt = 0;
        for (int i = 0; i < parts.Length; i++)
        {
            float m = parts[i].Mass;
            var p = bodies.PositionOf(rg.Body[i]); var v = bodies.LinearVelocityOf(rg.Body[i]);
            cx += m * p[0]; cy += m * p[1]; vx += m * v[0]; vy += m * v[1]; mt += m;
            a.PartRho[i] = BodyDelta(rg, i);
        }

        cx /= mt; cy /= mt; vx /= mt; vy /= mt;

        var inp = new BipedIn { Dt = dt, Gravity = g, Planar = true, Walk = a.Mode == (byte)ActiveMode.Walk, TorsoLean = a.Def.Walker.TorsoLean };
        float cmd = Math.Min(Math.Max(a.CmdX, -a.Def.Walker.MaxSpeed), a.Def.Walker.MaxSpeed);
        inp.CmdF = cmd < 0f ? 0f : cmd;
        inp.ComF = (float)(cx - px); inp.ComU = (float)(cy - py); inp.VelF = (float)vx;
        inp.PelvisRho = a.PartRho[pelvis];
        inp.Geom = new BipedGeom { L1 = a.L1, L2 = a.L2, AnkleHeight = a.AnkleHeight, FootBack = a.FootBack, FootFront = a.FootFront, FootHalfWidth = a.FootHalfWidth, Mass = (float)mt };

        for (int k = 0; k < 2; k++)
        {
            int th = bn.Leg(k, 0), sh = bn.Leg(k, 1), ft = bn.Leg(k, 2);
            var tj = Joints[rg.Joint[th]]; var fj = Joints[rg.Joint[ft]];
            var hip = ActAnchor(rg.Body[th], tj.LBx, tj.LBy);
            var ankle = ActAnchor(rg.Body[ft], fj.LBx, fj.LBy);
            ref var lg = ref (k == 0 ? ref inp.Left : ref inp.Right);
            lg.HipF = (float)(hip.X - px); lg.HipU = (float)(hip.Y - py);
            lg.AnkleF = (float)(ankle.X - px); lg.AnkleU = (float)(ankle.Y - py);
            lg.ThighRho = a.PartRho[th]; lg.FootRho = a.PartRho[ft];
            bool hit = GroundRay(a, r, ankle.X, ankle.Y + 0.05, a.AnkleHeight + 0.45f, out double gy, out float slope);
            lg.Grounded = hit && ankle.Y - gy <= a.AnkleHeight + 0.04f;
            lg.GroundU = (float)((hit ? gy : ankle.Y - a.AnkleHeight) - py);
            lg.SlopeRho = hit ? slope : 0f;
        }

        // Where the last step chose to land: ground height and tilt there, for the swing foot.
        var gait = a.Gait;
        if (gait.Phase == (byte)GaitPhase.SwingLeft || gait.Phase == (byte)GaitPhase.SwingRight)
        {
            int swing = gait.Phase == (byte)GaitPhase.SwingLeft ? 0 : 1;
            ref var stance = ref (swing == 0 ? ref inp.Right : ref inp.Left);
            double lx = px + stance.AnkleF + gait.PlaceF;
            double ytop = Math.Max(py + inp.Left.AnkleU, py + inp.Right.AnkleU) + 0.3;
            if (GroundRay(a, r, lx, ytop, 1.6f, out double gy, out float sl))
            {
                float du = (float)(gy - (py + stance.GroundU));
                ref var sw = ref (swing == 0 ? ref inp.Left : ref inp.Right);
                sw.PlaceDU = du; sw.PlaceSlopeRho = sl;
            }
        }

        var o = new BipedOut();
        var ap = new ActiveParams(a.Def.Balance, a.Def.Walker);
        BipedGait.Step(ref a.Gait, in ap, in inp, ref o);
        a.SwingLeg = a.Gait.Phase == (byte)GaitPhase.SwingLeft ? 0 : a.Gait.Phase == (byte)GaitPhase.SwingRight ? 1 : -1;

        // Joint targets: world-frame limb targets minus the actual parent's world angle.
        Own(a, bn.Torso, o.TorsoRho - a.PartRho[pelvis]);
        if (bn.Head >= 0) { Own(a, bn.Head, o.TorsoRho - a.PartRho[bn.Torso]); }
        for (int k = 0; k < 2; k++)
        {
            ref var lo = ref (k == 0 ? ref o.Left : ref o.Right);
            int th = bn.Leg(k, 0), sh = bn.Leg(k, 1), ft = bn.Leg(k, 2);
            Own(a, th, lo.ThighRho - a.PartRho[pelvis]);
            Own(a, sh, -lo.KneeFlex);
            Own(a, ft, lo.FootRho - a.PartRho[sh]);
            if (lo.AnkleTorque) { a.TorqueMode[ft] = true; a.AnkleTorque[ft] = lo.TorqueRho; }
        }

        a.Info = new ActiveRagdollInfo((ActiveMode)a.Mode, (GaitPhase)a.Gait.Phase, a.Gait.Steps, a.Gait.Time, (float)cx, (float)cy, 0f,
            (float)(px + o.CaptureF), 0f, (float)(px + o.SupMinF), 0f, (float)(px + o.SupMaxF), 0f, o.Grounded);
    }

    private static void Own(ActiveState2D a, int part, float rel)
    {
        a.Owned[part] = true;
        a.Rel[part] = rel;
    }

    // ------------------------------------------------------------------ snapshot section

    /// <summary>Id of the 2D active-ragdoll snapshot section ("ACT2").</summary>
    public const int ActiveSectionId = 0x32544341;

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
            w.NamedSingles(a.Pose); w.NamedSingles(a.Cur); w.NamedDouble(a.RootX); w.NamedDouble(a.RootY); w.NamedSingle(a.RootAngle);
        }
    }

    private static ActiveState2D?[] ReadActives(StateReader rd, HandleTable ragHandles, RagdollState?[] ragdolls)
    {
        int version = rd.ReadInt32();
        if (version != 1) { throw rd.Error("unsupported 2D active ragdoll section version " + version.ToString(CultureInfo.InvariantCulture)); }
        int count = rd.ReadCount(4 + 4 + 4 + ActiveCommon.DefBytes, "active ragdolls");
        if (count < 1) { throw rd.Error("an active ragdoll section holding no ragdolls"); }
        var result = new ActiveState2D?[Math.Max(ragHandles.HighWater, 1)];
        int last = -1;
        for (int k = 0; k < count; k++)
        {
            int slot = rd.ReadInt32(), gen = rd.ReadInt32(), n = rd.ReadInt32();
            if (slot <= last || slot >= ragHandles.HighWater || !ragHandles.IsLive(slot) || ragHandles.GenerationOf(slot) != gen) { throw rd.Error("an active ragdoll that names no live ragdoll"); }
            last = slot;
            var rg = ragdolls[slot]!;
            if (n != rg.Parts) { throw rd.Error("an active ragdoll with " + n.ToString(CultureInfo.InvariantCulture) + " parts for a ragdoll of " + rg.Parts.ToString(CultureInfo.InvariantCulture)); }
            var a = new ActiveState2D();
            a.ReadCommon(rd, n);
            a.Pose = new float[n];
            rd.ReadSingles(a.Pose, true, "active ragdoll pose");
            for (int i = 0; i < n; i++) { if (Math.Abs(a.Pose[i]) > 1e6f) { throw rd.Error("invalid active ragdoll pose"); } }
            a.Cur = new float[n];
            rd.ReadSingles(a.Cur, true, "active ragdoll targets");
            for (int i = 0; i < n; i++) { if (Math.Abs(a.Cur[i]) > 1e6f) { throw rd.Error("invalid active ragdoll target"); } }
            a.RootX = rd.ReadDouble(); a.RootY = rd.ReadDouble(); a.RootAngle = rd.ReadSingle();
            if (!double.IsFinite(a.RootX) || !double.IsFinite(a.RootY) || Math.Abs(a.RootX) > 2097152.0 || Math.Abs(a.RootY) > 2097152.0 || !float.IsFinite(a.RootAngle) || Math.Abs(a.RootAngle) > 1e6f) { throw rd.Error("invalid active ragdoll root"); }
            a.Derive(n);
            DeriveActiveGeometry(a, rg);
            result[slot] = a;
        }

        return result;
    }

    private void CommitActives(ActiveState2D?[]? block)
    {
        if (block == null) { Array.Clear(Actives); ActiveCount = 0; return; }
        Actives = block;
        ActiveCount = 0;
        foreach (var s in block) { if (s != null) { ActiveCount++; } }
    }

    private sealed class ActiveSection2D(Engine2D e) : ISnapshotSection
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
            var dst = (Engine2D)destination.GetSnapshotSection(Engine2D.Id);
            dst.Actives = new ActiveState2D?[e.Actives.Length];
            dst.ActiveCount = e.ActiveCount;
            for (int i = 0; i < dst.Actives.Length; i++) { dst.Actives[i] = e.Actives[i]?.Clone(); }
            return dst.ActiveSnapshotSection;
        }
    }
}
