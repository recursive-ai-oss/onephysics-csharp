using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Active;

// The balance and walking controllers, written once for both dimensions. Everything here is plain float arithmetic on numbers in a "level frame":
// f is forward, l is left, u is up, with the origin at the pelvis. A 2D world feeds l = 0 and ignores the roll angles, so the planar biped runs the same
// code with the lateral terms switched off. Angles are measured the way a 2D world measures them: pitch rho counts counter-clockwise in the (f, u) plane
// (a hanging leg swung forward has rho > 0; a torso leaning forward has rho < 0), roll phi counts counter-clockwise in the (l, u) plane (a hanging leg
// swung to the left has phi > 0). Targets for limbs are world-frame ("the shin should point this way") except knees, which are relative; the engines turn
// them into joint targets with the actual parent's orientation, which is what makes a stance hip hold the pelvis level.
//
// Sources: Pratt et al. 2006 and Koolen et al. 2012 for the capture point; Yin, Loken and van de Panne 2007 (SIMBICON) and Coros, Beaudoin and van de Panne 2010
// for the finite-state gait, swing-foot placement from centre-of-mass feedback and the stance hip holding the torso; Macchietto, Zordan and Shelton 2009 for
// the centre-of-pressure objective of the ankle strategy (kept to the simple single-pendulum form).

/// <summary>One leg as the controller sees it, in the level frame relative to the pelvis.</summary>
internal struct LegSense
{
    public float HipF, HipL, HipU;
    public float AnkleF, AnkleL, AnkleU;
    public bool Grounded;
    public float GroundU;                 // height of the ground under the ankle (relative to the pelvis, so negative)
    public float SlopeRho, SlopePhi;      // ground tilt under the ankle
    public float ThighRho, ThighPhi;      // actual world orientation of the thigh's axis (hip to knee)
    public float FootRho, FootPhi;        // actual world pitch and roll of the foot
    public float PlaceDU, PlaceSlopeRho;  // ground height and tilt at the landing point chosen by the last step (relative to the stance ankle's ground)
}

/// <summary>What the controller decides for one leg: world-frame thigh orientation, knee flexion (relative) and foot orientation.</summary>
internal struct LegOut
{
    public float ThighRho, ThighPhi, KneeFlex, FootRho, FootPhi;
    public bool AnkleTorque;              // true: the ankle applies the torques below instead of holding the foot at FootRho/FootPhi (a planted foot balancing)
    public float TorqueRho, TorquePhi;    // torque on the foot about the lateral axis (counter-clockwise positive in the pitch plane) and the forward axis, N m
    public float Weight;                  // 0..1: how much of the figure's weight this leg carries (stance 1, swing 0); informs gains
}

/// <summary>The persistent part of the gait: this is what snapshots save.</summary>
internal struct GaitState
{
    public byte Phase;                    // GaitPhase
    public byte LastSwing;                // the leg that swung last (0 left, 1 right)
    public int Steps;
    public float Time;                    // seconds in the phase
    public float LiftF, LiftL, LiftU;     // swing ankle at lift-off, relative to the stance ankle
    public float PlaceF, PlaceL;          // landing point chosen on the last step, relative to the stance ankle (the engine raycasts there)
}

internal struct BipedGeom
{
    public float L1, L2;                  // thigh and shin lengths, m
    public float AnkleHeight;             // ankle joint above the sole, m
    public float FootBack, FootFront, FootHalfWidth;   // footprint relative to the ankle
    public float Mass;                    // whole figure, kg
}

internal struct BipedIn
{
    public float Dt, Gravity;
    public bool Planar, Walk;
    public float CmdF, CmdL;              // commanded centre-of-mass velocity in the level frame
    public float ComF, ComL, ComU, VelF, VelL;
    public float PelvisRho, PelvisPhi, YawError;
    public LegSense Left, Right;
    public BipedGeom Geom;
    public float TorsoLean;
}

internal struct BipedOut
{
    public LegOut Left, Right;
    public float TorsoRho, TorsoPhi;
    public float CaptureF, CaptureL;      // the capture point, level frame relative to the pelvis
    public float SupMinF, SupMaxF, SupMinL, SupMaxL;
    public int Grounded;
    public float ArmPhase;                // thigh swing of the stance-relative gait, for arm counter-swing (positive: left leg forward)
    public float StanceYaw;               // yaw offset of the legs' targets from the pelvis heading, radians
}

internal static class BipedGait
{
    internal const float Pi = 3.1415927f;

    internal static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;

    private static float Min(float a, float b) => a < b ? a : b;

    private static float Max(float a, float b) => a > b ? a : b;

    private static float Abs(float a) => a < 0f ? -a : a;

    private static float Smooth(float s) { s = Clamp(s, 0f, 1f); return s * s * (3f - 2f * s); }

    internal static ref LegSense Leg(ref BipedIn i, int k) { if (k == 0) { return ref i.Left; } return ref i.Right; }

    private static ref LegOut LegO(ref BipedOut o, int k) { if (k == 0) { return ref o.Left; } return ref o.Right; }

    // Two-link leg: the thigh angle and knee flexion that put the ankle at (df, dl, du) from the hip. Roll comes from the line to the ankle; the knee bends in the
    // pitch plane, forward. The reach is kept a hair short of straight so the knee never locks (a locked knee has no gradient for the drive).
    internal static void Ik(float l1, float l2, float df, float dl, float du, out float thighRho, out float thighPhi, out float knee)
    {
        float d2 = df * df + dl * dl + du * du;
        float d = float.Sqrt(d2);
        float maxReach = (l1 + l2) * 0.9995f;
        if (d > maxReach) { float k = maxReach / d; df *= k; dl *= k; du *= k; d = maxReach; }
        if (d < 0.05f) { d = 0.05f; }
        float lineRho = (float)Math64.Atan2(df, -du);
        float lineSin = Clamp(dl / d, -1f, 1f);
        thighPhi = (float)Math64.Asin(lineSin);
        float ca = Clamp((l1 * l1 + d * d - l2 * l2) / (2f * l1 * d), -1f, 1f);
        float alpha = (float)Math64.Acos(ca);
        thighRho = lineRho + alpha;
        float ck = Clamp((l1 * l1 + l2 * l2 - d * d) / (2f * l1 * l2), -1f, 1f);
        knee = Pi - (float)Math64.Acos(ck);
    }

    /// <summary>The capture point's frequency, 1/s: sqrt(g / h) for a pendulum of height h.</summary>
    internal static float Omega(float g, float h) => float.Sqrt(g / (h < 0.25f ? 0.25f : h));

    internal static void Step(ref GaitState s, in ActiveParams p, in BipedIn inp, ref BipedOut o)
    {
        var inn = inp;
        float dt = inn.Dt;
        s.Time += dt;
        float legLen = inn.Geom.L1 + inn.Geom.L2;
        var bal = p.Balance; var wk = p.Walker;
        bool planar = inn.Planar;

        // Support: bounding box of the grounded feet's footprints, in the level frame.
        int grounded = 0;
        float minF = float.MaxValue, maxF = float.MinValue, minL = float.MaxValue, maxL = float.MinValue;
        for (int k = 0; k < 2; k++)
        {
            ref var lg = ref Leg(ref inn, k);
            if (!lg.Grounded) { continue; }
            grounded++;
            minF = Min(minF, lg.AnkleF - inn.Geom.FootBack); maxF = Max(maxF, lg.AnkleF + inn.Geom.FootFront);
            minL = Min(minL, lg.AnkleL - inn.Geom.FootHalfWidth); maxL = Max(maxL, lg.AnkleL + inn.Geom.FootHalfWidth);
        }

        float groundU = inn.Left.GroundU < inn.Right.GroundU ? inn.Left.GroundU : inn.Right.GroundU;
        if (grounded == 1) { groundU = inn.Left.Grounded ? inn.Left.GroundU : inn.Right.GroundU; }
        else if (grounded == 2) { groundU = 0.5f * (inn.Left.GroundU + inn.Right.GroundU); }
        float w0 = Omega(inn.Gravity, inn.ComU - groundU);
        float capF = inn.ComF + inn.VelF / w0, capL = planar ? 0f : inn.ComL + inn.VelL / w0;
        o.CaptureF = capF; o.CaptureL = capL;
        if (grounded == 0) { minF = maxF = inn.ComF; minL = maxL = inn.ComL; }
        o.SupMinF = minF; o.SupMaxF = maxF; o.SupMinL = minL; o.SupMaxL = maxL; o.Grounded = grounded;

        float cmdSpeed = float.Sqrt(inn.CmdF * inn.CmdF + inn.CmdL * inn.CmdL);
        bool walking = inn.Walk && cmdSpeed > 0.02f;
        float trig = bal.StepTrigger;
        bool needStep = grounded == 2 && (capF > maxF + trig || capF < minF - trig || (!planar && (capL > maxL + trig || capL < minL - trig)));

        var phase = (GaitPhase)s.Phase;
        if (phase == GaitPhase.DoubleSupport)
        {
            if (grounded == 2 && (needStep || walking))
            {
                int swing = 1 - s.LastSwing;
                if (!planar && needStep)
                {
                    float centreL = 0.5f * (minL + maxL);
                    if (capL > maxL + trig) { swing = 0; } else if (capL < minL - trig) { swing = 1; } else if (!walking) { swing = capL > centreL ? 0 : 1; }
                }

                if (!planar && walking && !needStep)
                {
                    s.Phase = (byte)GaitPhase.Shift; s.LastSwing = (byte)(1 - swing); s.Time = 0f;
                }
                else { BeginSwing(ref s, in inn, swing); }
                phase = (GaitPhase)s.Phase;
            }
        }
        else if (phase == GaitPhase.Shift)
        {
            int swing = 1 - s.LastSwing;
            int stance = 1 - swing;
            float comOff = inn.ComL - Leg(ref inn, stance).AnkleL;
            if (!Leg(ref inn, stance).Grounded) { s.Phase = (byte)GaitPhase.DoubleSupport; s.Time = 0f; }
            else if (Abs(comOff) < 0.035f && Abs(inn.VelL) < 0.12f || s.Time >= wk.ShiftTime) { BeginSwing(ref s, in inn, swing); }
            phase = (GaitPhase)s.Phase;
        }

        // Which leg swings, and where it should land.
        int swingLeg = phase == GaitPhase.SwingLeft ? 0 : phase == GaitPhase.SwingRight ? 1 : phase == GaitPhase.Shift ? 1 - s.LastSwing : -1;
        float stepTime = walking || phase == GaitPhase.Shift ? wk.StepTime : bal.StepTime;
        float stepHeight = walking ? wk.StepHeight : bal.StepHeight;
        if (phase == GaitPhase.SwingLeft || phase == GaitPhase.SwingRight)
        {
            int st = 1 - swingLeg;
            ref var stance = ref Leg(ref inn, st);
            ref var swingL = ref Leg(ref inn, swingLeg);
            float sFrac = s.Time / stepTime;
            float tRem = Max(0f, stepTime - s.Time);
            float comRelF = inn.ComF - stance.AnkleF, comRelL = inn.ComL - stance.AnkleL;
            // Where the centre of mass will be, and how fast, when the swing ends: the linear inverted pendulum's closed form (Kajita et al. 2001), which sees the fall accelerate where a straight-line guess would not.
            float e1 = (float)Math64.Exp((double)(w0 * tRem)), e2 = 1f / e1;
            float ch = 0.5f * (e1 + e2), sh = 0.5f * (e1 - e2);
            float predF = comRelF * ch + inn.VelF / w0 * sh, vPredF = comRelF * w0 * sh + inn.VelF * ch;
            float predL = comRelL * ch + inn.VelL / w0 * sh, vPredL = comRelL * w0 * sh + inn.VelL * ch;
            float kc = walking ? wk.SpeedFeedback : 1f;
            float kl = walking ? wk.LateralFeedback : 1f;
            float half = 0.5f * stepTime;
            float pf;
            if (walking)
            {
                // Raibert's rule: the foot lands ahead of the centre of mass by the distance it will cover in half a stance, corrected by the speed error.
                pf = predF + vPredF * half + kc * (vPredF - inn.CmdF) / w0;
                if (pf < predF - 0.05f) { pf = predF - 0.05f; }
            }
            else { pf = predF + vPredF / w0; }   // capture point: the foot goes where the fall would stop
            float side = swingLeg == 0 ? 1f : -1f;
            float pl;
            if (planar) { pl = 0f; }
            else
            {
                float width = wk.StepWidth;
                pl = predL + vPredL * half * 0f + kl * (vPredL - inn.CmdL) / w0;
                // The foot lands outside the capture point on the swing side; the width term keeps the feet apart in steady walking.
                float centred = side * width * 0.5f;
                pl = side > 0f ? Max(pl + centred, width * 0.6f) : Min(pl + centred, -width * 0.6f);
                pl = Clamp(pl, -0.8f * legLen, 0.8f * legLen);
            }

            float reach = bal.MaxStepReach * legLen;
            float hd = float.Sqrt(pf * pf + pl * pl);
            if (hd > reach) { float k = reach / hd; pf *= k; pl *= k; }
            s.PlaceF = pf; s.PlaceL = pl;

            // Swing foot path: smooth interpolation from lift-off to landing with a clearance arch.
            float ss = Smooth(sFrac);
            float tf = s.LiftF + (pf - s.LiftF) * ss;
            float tl = s.LiftL + (pl - s.LiftL) * ss;
            float placeU = swingL.PlaceDU;
            float tu = s.LiftU + (placeU - s.LiftU) * ss + stepHeight * (float)Math64.Sin((double)(Pi * Clamp(sFrac, 0f, 1f)));

            float ax = stance.AnkleF + tf, al = stance.AnkleL + tl, au = stance.AnkleU + tu;
            ref var so = ref LegO(ref o, swingLeg);
            Ik(inn.Geom.L1, inn.Geom.L2, ax - swingL.HipF, planar ? 0f : al - swingL.HipL, au - swingL.HipU, out so.ThighRho, out so.ThighPhi, out so.KneeFlex);
            so.FootRho = swingL.PlaceSlopeRho * ss; so.FootPhi = 0f; so.Weight = 0f;
            if (planar) { so.ThighPhi = 0f; }

            bool landed = sFrac >= 0.5f && swingL.Grounded;
            if (landed || s.Time >= stepTime * wk.StepTimeout)
            {
                s.Steps++; s.LastSwing = (byte)swingLeg; s.Time = 0f;
                if (walking)
                {
                    BeginSwing(ref s, in inn, st);
                }
                else { s.Phase = (byte)GaitPhase.DoubleSupport; }
            }
        }
        else if (phase == GaitPhase.Shift)
        {
            // handled below as a stance leg with the lateral target at the hip
        }

        float pelvisRhoDes = 0f, pelvisPhiDes = 0f;
        float standH = (walking || phase != GaitPhase.DoubleSupport ? wk.HipHeight : bal.StandHeight) * legLen;

        // How the body's weight splits between two planted feet: by where the centre of mass sits between the ankles (the ground reaction follows the lever rule).
        float shareL = 0.5f, shareR = 0.5f;
        if (grounded == 2)
        {
            float span = inn.Right.AnkleF - inn.Left.AnkleF;
            if (Abs(span) > 0.02f) { float sr = Clamp((inn.ComF - inn.Left.AnkleF) / span, 0.1f, 0.9f); shareR = sr; shareL = 1f - sr; }
        }

        // Stance legs (both, in double support and shift).
        int swingNow = (GaitPhase)s.Phase is GaitPhase.SwingLeft ? 0 : (GaitPhase)s.Phase is GaitPhase.SwingRight ? 1 : -1;
        for (int k = 0; k < 2; k++)
        {
            if (k == swingNow) { continue; }
            ref var lg = ref Leg(ref inn, k);
            ref var lo = ref LegO(ref o, k);
            float weight = swingNow >= 0 ? 1f : (k == 0 ? shareL : shareR);
            lo.Weight = weight;
            // The pelvis is levelled by the legs that carry it, each in proportion to its share (a leg that carries little must not be pushed around by the whole correction).
            float gain = swingNow >= 0 ? 1f : 2f * weight;
            lo.ThighRho = lg.ThighRho + gain * (inn.PelvisRho - pelvisRhoDes);
            lo.ThighPhi = planar ? 0f : lg.ThighPhi + gain * (inn.PelvisPhi - pelvisPhiDes);
            float df = lg.AnkleF - lg.HipF, dl = lg.AnkleL - lg.HipL;
            float dd = float.Sqrt(standH * standH + df * df + (planar ? 0f : dl * dl));
            float maxD = legLen * 0.9995f;
            if (dd > maxD) { dd = maxD; }
            // Knee: the flexion that puts the hip at the standing height over this ankle.
            float ck = Clamp((inn.Geom.L1 * inn.Geom.L1 + inn.Geom.L2 * inn.Geom.L2 - dd * dd) / (2f * inn.Geom.L1 * inn.Geom.L2), -1f, 1f);
            lo.KneeFlex = Pi - (float)Math64.Acos(ck);
            lo.FootRho = lg.SlopeRho; lo.FootPhi = planar ? 0f : lg.SlopePhi;
        }

        // Shift: the stance leg's thigh goes vertical (sideways), which carries the pelvis over the planted foot.
        if ((GaitPhase)s.Phase == GaitPhase.Shift && !planar)
        {
            int stance = s.LastSwing == 0 ? 0 : 1;   // LastSwing was set to 1 - swing: the leg that will stay
            ref var lo = ref LegO(ref o, stance);
            lo.ThighPhi = 0f; lo.Weight = 1f;
        }

        // Ankle strategy (centre-of-pressure feedback).
        if (phase != GaitPhase.Shift && grounded > 0)
        {
            // The pressure point the body needs when it is balanced over its feet is where the weight-carrying ankles are, not the middle of the footprint.
            float centreF = shareL * inn.Left.AnkleF + shareR * inn.Right.AnkleF, centreL = shareL * inn.Left.AnkleL + shareR * inn.Right.AnkleL;
            if (grounded == 1) { centreF = inn.Left.Grounded ? inn.Left.AnkleF : inn.Right.AnkleF; centreL = inn.Left.Grounded ? inn.Left.AnkleL : inn.Right.AnkleL; }
            float pcF = capF + bal.CaptureGain * (capF - centreF);
            float pcL = capL + bal.CaptureGain * (capL - centreL);
            if (walking)
            {
                // Walking: the ankle regulates the speed instead of holding the position. The pressure point goes where the pendulum gives the wanted acceleration: a = (g / h) (x - p).
                float aDes = Clamp((inn.CmdF - inn.VelF) / 0.4f, -3f, 3f);
                float hh = inn.ComU - groundU;
                pcF = inn.ComF - aDes * hh / inn.Gravity;
                pcL = centreL;
            }

            float mg = inn.Geom.Mass * inn.Gravity;
            for (int k = 0; k < 2; k++)
            {
                ref var lg = ref Leg(ref inn, k);
                if (!lg.Grounded) { continue; }
                ref var lo = ref LegO(ref o, k);
                float share = grounded == 2 ? (k == 0 ? shareL : shareR) : 1f;
                float dF = Clamp(pcF - centreF, -0.85f * inn.Geom.FootBack, 0.85f * inn.Geom.FootFront);
                // The ground pushes up at the pressure point; the ankle must give the foot the opposite moment about itself.
                lo.AnkleTorque = true;
                lo.TorqueRho = -mg * share * dF;
                if (!planar)
                {
                    float dL = Clamp(pcL - centreL, -0.85f * inn.Geom.FootHalfWidth, 0.85f * inn.Geom.FootHalfWidth);
                    lo.TorquePhi = (Environment.GetEnvironmentVariable("LATSIGN") != null ? 1f : -1f) * mg * share * dL;
                }
            }
        }

        // Torso: upright, leaning a little into the walk.
        o.TorsoRho = walking ? -inn.TorsoLean * cmdSpeed * (inn.CmdF >= 0f ? 1f : -1f) : 0f;   // relative to the pelvis: the torso rides on it, and the stance hips level the pair
        o.TorsoPhi = 0f;
        o.ArmPhase = swingNow >= 0 ? (swingNow == 0 ? 1f : -1f) : 0f;
        o.StanceYaw = Clamp(inn.YawError, -0.5f, 0.5f);
    }

    private static void BeginSwing(ref GaitState s, in BipedIn inn, int swing)
    {
        s.Phase = (byte)(swing == 0 ? GaitPhase.SwingLeft : GaitPhase.SwingRight);
        s.Time = 0f;
        var a = swing == 0 ? inn.Left : inn.Right; var b = swing == 0 ? inn.Right : inn.Left;
        s.LiftF = a.AnkleF - b.AnkleF; s.LiftL = a.AnkleL - b.AnkleL; s.LiftU = a.AnkleU - b.AnkleU;
        s.PlaceF = s.LiftF; s.PlaceL = s.LiftL;
    }
}

/// <summary>The controller settings after validation, as plain values.</summary>
internal readonly struct ActiveParams
{
    public readonly BalanceDef Balance;
    public readonly WalkerDef Walker;
    public ActiveParams(BalanceDef b, WalkerDef w) { Balance = b; Walker = w; }
}
