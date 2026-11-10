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
    public float ShinRho, ShinPhi;        // actual world pitch and roll of the shin
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
    public float FVelF, FVelL;            // low-pass filtered centre-of-mass velocity (level frame)
    public float Height;                  // current hip height over the ankles as a fraction of the straight leg (eased towards the wanted one)
    public float BiasF, BiasL;            // integral correction of the commanded velocity the foot placement plans for (the model of the gait is never exact, so the speed would settle off the command)
    public float SumF, SumL, SumT;        // velocity accumulated over the current step
    public byte Primed;                   // the filter holds a measurement
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

/// <summary>Tuned constants of the walking controller (see the notes where each is used); fixed numbers, never read from the environment.</summary>
internal static class Tun
{
    internal const float Ds = 0.06f;          // short double support after each landing, s
    internal const float Lean = 0.1378f;      // how far ahead of the feet the hip leans before the first step, m
    internal const float LeanRate = 0.108f;   // how fast, m/s
    internal const float Launch = 0.1156f;    // the capture point must be this far ahead of the feet's middle before the first step
    internal const float Tau = 0.05f;         // look-ahead of the hip target, s
    internal const float LatTol = 0.04f;      // how far from the stance ankle the centre of mass may be sideways before the first step (3D), m
    internal const float Ki = 0.0202f;        // learning rate of the speed correction per step
    internal const float Over = 0.3f;         // fraction of the sideways weight shift before the first step (3D): the hip goes only part of the way over the staying foot, because a body that is already over the foot and moving has to be stopped there
    internal const float CrossLat = -0.1f;    // a landing wanted this far or less beyond the stance-side limit is crossed (3D); between that and the limit it is clamped to the limit, m
    internal const float CrossMax = 0.25f;    // the farthest a swing foot may cross past the stance foot, m (3D)
    internal const float TrimLat = 0.04f;     // sideways pressure-point trim limit (3D), m
    internal const float TrimMax = 0.1083f;   // forward pressure-point trim limits, m
    internal const float TrimMin = -0.0282f;
    internal const float MinStep = -0.0418f;  // the farthest behind the stance foot a step may land, m
    internal const float StopSpeed = 0.2f;    // a 3D figure told to stop keeps stepping until it is slower than this, m/s
    internal const float FinalPf = 0.1f;      // while stopping, the last steps land no farther than this ahead of the stance foot, m
    internal const float FinalSpeed = 0.5f;   // "the last steps" are those taken slower than this, m/s
    internal const float MinStep3 = -0.3f;    // the farthest behind the stance foot a step may land in 3D, m: a figure that is told to stop, or that is pushed backwards, has to be able to step back
    internal const float Filt = 0.0716f;      // time constant of the velocity filter, s
    internal const float Rate = 0.15f;        // how fast the hip height changes, fraction of a leg per second
}

internal static class BipedGait
{
    internal const float Pi = 3.1415927f;

    private static float TanSlope(float rho)
    {
        // Slopes are small angles in practice; the quotient of onemath's sine and cosine is the exact tangent.
        float t = (float)(Math64.Sin((double)rho) / Math64.Cos((double)rho));
        return Clamp(t, -1f, 1f);
    }

    /// <summary>
    /// Whether every measurement is finite and within a range a figure can really reach (1e7 m or m/s). A world whose parts have been thrown beyond that has no gait to plan; the engines skip the controller for that step,
    /// because planning on such numbers overflows into values the gait state could not be saved with.
    /// </summary>
    internal static bool Sane(in BipedIn i)
    {
        return Ok(i.CmdF) && Ok(i.CmdL) && Ok(i.ComF) && Ok(i.ComL) && Ok(i.ComU) && Ok(i.VelF) && Ok(i.VelL) && Ok(i.PelvisRho) && Ok(i.PelvisPhi) && Ok(i.YawError) && SaneLeg(in i.Left) && SaneLeg(in i.Right)
            && Ok(i.Geom.L1) && Ok(i.Geom.L2) && Ok(i.Geom.AnkleHeight) && Ok(i.Geom.FootBack) && Ok(i.Geom.FootFront) && Ok(i.Geom.FootHalfWidth) && Ok(i.Geom.Mass) && Ok(i.Gravity) && Ok(i.TorsoLean);
    }

    private static bool SaneLeg(in LegSense l)
    {
        return Ok(l.HipF) && Ok(l.HipL) && Ok(l.HipU) && Ok(l.AnkleF) && Ok(l.AnkleL) && Ok(l.AnkleU) && Ok(l.GroundU) && Ok(l.SlopeRho) && Ok(l.SlopePhi) && Ok(l.ShinRho) && Ok(l.ShinPhi) && Ok(l.ThighRho) && Ok(l.ThighPhi)
            && Ok(l.FootRho) && Ok(l.FootPhi) && Ok(l.PlaceDU) && Ok(l.PlaceSlopeRho);
    }

    private static bool Ok(float v) => Math.Abs(v) <= 1e7f;   // NaN fails

    internal static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;

    // Within [-limit, limit]; NaN becomes 0.
    private static float BoundTo(float v, float limit) => float.IsNaN(v) ? 0f : v < -limit ? -limit : v > limit ? limit : v;

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
        // The centre of mass velocity of a figure whose limbs are all being driven is noisy at the frame rate; the gait's decisions (where the foot lands) amplify velocity error by the pendulum's growth over a step, so they work on a smoothed value (time constant about 0.08 s).
        if (s.Primed == 0) { s.FVelF = inn.VelF; s.FVelL = inn.VelL; s.Primed = 1; }
        else { float al = dt / (Tun.Filt + dt); s.FVelF += al * (inn.VelF - s.FVelF); s.FVelL += al * (inn.VelL - s.FVelL); }
        if (inn.Walk) { inn.VelF = s.FVelF; inn.VelL = s.FVelL; }   // balance without walking reacts to the raw velocity: a push must be seen at once
        s.SumF += s.FVelF * dt; s.SumL += s.FVelL * dt; s.SumT += dt;
        // The gait plans for the commanded velocity plus a slowly learned correction.
        float planF = inn.CmdF + s.BiasF, planL = inn.CmdL + s.BiasL;
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
        // A figure told to stop while it moves keeps stepping, planning for zero speed, until it has slowed down and is between steps; only then does the standing balance take over.
        bool walking = inn.Walk && (cmdSpeed > 0.02f || (!planar && (s.Phase >= (byte)GaitPhase.SwingLeft || float.Sqrt(s.FVelF * s.FVelF + s.FVelL * s.FVelL) > Tun.StopSpeed)));
        float trig = bal.StepTrigger;
        bool needStep = !walking && grounded == 2 && (capF > maxF + trig || capF < minF - trig || (!planar && (capL > maxL + trig || capL < minL - trig)));

        var phase = (GaitPhase)s.Phase;
        if (phase == GaitPhase.DoubleSupport)
        {
            // Starting from rest the weight leans forward first (the ankles put the pressure point behind the centre of mass) until the capture point is well ahead; stepping at once would put the foot behind the body.
            bool launched = !walking;
            if (walking)
            {
                // The leg that will carry the first step is the one that stays (LastSwing); the weight must be over it, sideways too in 3D.
                bool fwdOk = capF >= 0.5f * (inn.Left.AnkleF + inn.Right.AnkleF) + Tun.Launch;
                bool latOk = planar || (Abs(inn.ComL - Leg(ref inn, s.LastSwing).AnkleL) < Tun.LatTol && Abs(inn.VelL) < 0.15f);
                launched = (fwdOk && latOk) || s.Time > 3f;
            }
            if (grounded == 2 && (needStep || (walking && launched)))
            {
                int swing = 1 - s.LastSwing;
                if (!planar && needStep)
                {
                    float centreL = 0.5f * (minL + maxL);
                    if (capL > maxL + trig) { swing = 0; } else if (capL < minL - trig) { swing = 1; } else if (!walking) { swing = capL > centreL ? 0 : 1; }
                }

                BeginSwing(ref s, in inn, swing);
                phase = (GaitPhase)s.Phase;
            }
        }
        else if (phase == GaitPhase.Shift)
        {
            // The short double support after a landing: the weight passes to the foot that landed before the other one lifts.
            int swing = 1 - s.LastSwing;
            if (!Leg(ref inn, s.LastSwing).Grounded) { s.Phase = (byte)GaitPhase.DoubleSupport; s.Time = 0f; }
            else if (!walking) { s.Phase = (byte)GaitPhase.DoubleSupport; s.Time = 0f; }
            else if (s.Time >= Tun.Ds) { BeginSwing(ref s, in inn, swing); }
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
            // Divergent component of motion (Englsberger et al. 2013 on the capture point): xi = x + v / w0 grows as e^(w0 t) about the stance foot, so where it will be when the swing ends is known; a foot that lands
            // at that point minus the offset a steady gait of the commanded speed keeps (b = speed * T / (e^(w0 T) - 1)) brings the next step's motion back onto the commanded gait.
            float e1 = (float)Math64.Exp((double)(w0 * tRem));
            float xiF = (comRelF + inn.VelF / w0) * e1, xiL = (comRelL + inn.VelL / w0) * e1;
            float tCycle = stepTime + Tun.Ds;   // one stance foot carries the body through the short double support and then the swing of the other leg
            float eT = (float)Math64.Exp((double)(w0 * tCycle));
            float side = swingLeg == 0 ? 1f : -1f;
            // Steady offsets of the capture point from the stance foot at touchdown: forward, the commanded speed's share; sideways, the feet alternate by the step width, so it points towards the next foot.
            float bF = walking ? planF * tCycle / (eT - 1f) : 0f;
            float bL = walking ? planL * tCycle / (eT - 1f) + side * wk.StepWidth / (eT + 1f) : 0f;
            float xiNomEnd = bF * eT;   // the capture point of a steady gait at the end of the swing
            float pf = walking ? (xiNomEnd + wk.PlacementGain * (xiF - xiNomEnd)) - bF : xiF;
            float minF0 = walking ? (planar ? Tun.MinStep : Tun.MinStep3) : -float.MaxValue;
            if (pf < minF0) { pf = minF0; }
            if (!planar && walking && cmdSpeed <= 0.02f && s.FVelF < Tun.FinalSpeed) { pf = Clamp(pf, minF0, Tun.FinalPf); }   // stopping: the swing foot is put down beside the stance foot, not ahead of it, so the figure ends with its feet together
            float pl;
            if (planar) { pl = 0f; }
            else
            {
                // The next stance foot's own offset points back towards the body's middle (mirror of the one now), so the foot goes to the capture point plus that offset.
                float bLnew = walking ? planL * tCycle / (eT - 1f) - side * wk.StepWidth / (eT + 1f) : 0f;
                float xiNomL = bL * eT;
                float minGap = 0.6f * wk.StepWidth;
                if (walking)
                {
                    pl = (xiNomL + wk.PlacementGain * (xiL - xiNomL)) - bLnew;
                    // A foot that the capture point wants inside the stance foot's side lands beside the stance foot; when the capture point has gone past the stance foot (a push towards it), the swing foot crosses it and lands on the
                    // far side, the only place the body can still be caught. Without crossing, the two-step answer (a close step, then a very wide one) sends the speed up and the figure falls.
                    float inward = side > 0f ? minGap - pl : pl + minGap;
                    if (inward > 0f)
                    {
                        float crossed = side > 0f ? -pl : pl;   // positive: wanted past the stance foot
                        if (crossed > Tun.CrossLat) { pl = side > 0f ? Max(pl, -Tun.CrossMax) : Min(pl, Tun.CrossMax); }
                        else { pl = side > 0f ? minGap : -minGap; }
                    }
                }
                else
                {
                    // A recovery step lands outside the capture point on the swing side.
                    pl = xiL + side * wk.StepWidth * 0.5f;
                    pl = side > 0f ? Max(pl, minGap) : Min(pl, -minGap);
                }

                pl = Clamp(pl, -0.8f * legLen, 0.8f * legLen);
            }

            float reach = bal.MaxStepReach * legLen;
            float hd = float.Sqrt(pf * pf + pl * pl);
            // A body flung at an absurd speed makes the capture point overflow; an infinite length would scale the target by zero and turn it into NaN, which the saved state then refuses.
            if (!float.IsFinite(hd)) { pf = BoundTo(pf, reach); pl = BoundTo(pl, reach); hd = float.Sqrt(pf * pf + pl * pl); }
            if (hd > reach) { float k = reach / hd; pf *= k; pl *= k; }
            s.PlaceF = pf; s.PlaceL = pl;

            // Swing foot path: smooth interpolation from lift-off to landing with a clearance arch.
            float ss = Smooth(sFrac);
            float tf = s.LiftF + (pf - s.LiftF) * ss;
            float ssL = ss;
            if (!planar && (side > 0f ? pl < 0f : pl > 0f)) { ssL = Smooth((sFrac - 0.3f) / 0.7f); }   // a crossing foot goes forward first and then sideways
            float tl = s.LiftL + (pl - s.LiftL) * ssL;
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
                if (walking && s.SumT > 0.2f)
                {
                    s.BiasF = Clamp(s.BiasF + Tun.Ki * (inn.CmdF - s.SumF / s.SumT), -0.5f * Max(1f, Abs(inn.CmdF)), 1f);
                    s.BiasL = planar ? 0f : Clamp(s.BiasL + Tun.Ki * (inn.CmdL - s.SumL / s.SumT), -0.5f, 0.5f);
                }

                s.SumF = 0f; s.SumL = 0f; s.SumT = 0f;
                if (walking) { s.Phase = (byte)GaitPhase.Shift; }
                else { s.Phase = (byte)GaitPhase.DoubleSupport; }
            }
        }

        float pelvisRhoDes = 0f, pelvisPhiDes = 0f;
        float wantH = walking ? wk.HipHeight : phase != GaitPhase.DoubleSupport ? Min(bal.StandHeight, 0.97f) : bal.StandHeight;   // a recovery step crouches a little
        if (s.Height <= 0f) { s.Height = Clamp(0.5f * ((inn.Left.HipU - inn.Left.AnkleU) + (inn.Right.HipU - inn.Right.AnkleU)) / legLen, 0.5f, 0.999f); }
        { float stepH = Tun.Rate * dt; s.Height += Clamp(wantH - s.Height, -stepH, stepH); }   // the knees bend or straighten over a few tenths of a second, so the feet are not thrown out
        float standH = s.Height * legLen;

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
            if (walking)
            {
                // Walking stance: the hip is moved the way a linear inverted pendulum with its pressure point at the ankle would move the centre of mass over the next instant (Kajita et al. 2001), by solving the leg for that hip position over
                // the planted ankle; the knee keeps the hip height. A leg that only holds its shape lets the foot slide as the knee works; one that is told where the hip goes does not.
                float tau = Tun.Tau;
                float xh, yh;   // where the hip should be, relative to this leg's ankle
                if (swingNow < 0 && (GaitPhase)s.Phase == GaitPhase.DoubleSupport)
                {
                    // Starting from rest: the hip is brought forward (and, in 3D, over the foot that will stay) a little more as time passes, until the capture point has left the feet and the first step can be taken.
                    float midF = 0.5f * (inn.Left.AnkleF + inn.Right.AnkleF), midL = 0.5f * (inn.Left.AnkleL + inn.Right.AnkleL);
                    // The hip moves by (where it should be relative to the ankles' middle) minus (where it is); this leg's hip target is its own hip moved by that.
                    xh = lg.HipF - lg.AnkleF + midF + Min(Tun.Lean, Tun.LeanRate * s.Time);
                    float over = Tun.Over * Clamp(s.Time / 0.5f, 0f, 1f);
                    yh = planar ? 0f : lg.HipL - lg.AnkleL + midL + over * (Leg(ref inn, s.LastSwing).AnkleL - midL);
                }
                else
                {
                    // The pendulum stands on the foot that carries the body (the one that stays through the swing, or the one that just landed). Its pressure point comes from the capture point's error against the one a steady gait of the
                    // commanded speed would have at this moment of the step: a body running ahead of it is braked by pressure forward of the ankle, a lagging one is pushed on by pressure behind it (Koolen et al. 2012).
                    int ns = swingNow >= 0 ? 1 - swingNow : s.LastSwing;
                    ref var nl = ref Leg(ref inn, ns);
                    float comRel = inn.ComF - nl.AnkleF;
                    float tStep = wk.StepTime + Tun.Ds;
                    float eNom = (float)Math64.Exp((double)(w0 * tStep));
                    float sinceTouch = swingNow >= 0 ? s.Time + Tun.Ds : s.Time;
                    float growth = (float)Math64.Exp((double)(w0 * Min(sinceTouch, tStep)));
                    float xiNom = (inn.CmdF + s.BiasF) * tStep / (eNom - 1f) * growth;
                    float trim = Clamp(wk.SpeedFeedback * ((comRel + inn.VelF / w0) - xiNom), Tun.TrimMin, Tun.TrimMax);
                    float acc = w0 * w0 * (comRel - trim);
                    float xhN = nl.HipF - nl.AnkleF + inn.VelF * tau + 0.5f * acc * tau * tau;
                    xh = xhN + nl.AnkleF - lg.AnkleF + lg.HipF - nl.HipF;
                    yh = 0f;
                    if (!planar)
                    {
                        float nextSide = swingNow >= 0 ? (swingNow == 0 ? 1f : -1f) : (s.LastSwing == 0 ? -1f : 1f);   // the side the next swing leg lies on
                        float comRelL = inn.ComL - nl.AnkleL;
                        float xiNomL = ((inn.CmdL + s.BiasL) * tStep / (eNom - 1f) + nextSide * wk.StepWidth / (eNom + 1f)) * growth;
                        float trimL = Clamp(wk.LateralFeedback * ((comRelL + inn.VelL / w0) - xiNomL), -Tun.TrimLat, Tun.TrimLat);
                        float accL = w0 * w0 * (comRelL - trimL);
                        float yhN = nl.HipL - nl.AnkleL + inn.VelL * tau + 0.5f * accL * tau * tau;
                        yh = yhN + nl.AnkleL - lg.AnkleL + lg.HipL - nl.HipL;
                    }

                }

                // On a slope the hip runs parallel to the ground, which leaves the pendulum's frequency unchanged when it is measured from the stance foot (a mass on a line parallel to the slope obeys x'' = g x / h).
                float tanS = TanSlope(lg.SlopeRho);
                Ik(inn.Geom.L1, inn.Geom.L2, -xh, planar ? 0f : -yh, -(standH + tanS * xh), out float th, out float thPhi, out float kn);
                lo.ThighRho = th + gain * (inn.PelvisRho - pelvisRhoDes); lo.KneeFlex = kn;
                if (!planar) { lo.ThighPhi = thPhi + gain * (inn.PelvisPhi - pelvisPhiDes); }
                // The ankle leans the shin to where the solved leg has it; the foot stays flat on the ground.
                lo.FootRho = lg.SlopeRho + (lg.ShinRho - (th - kn));
                lo.FootPhi = planar ? 0f : lg.SlopePhi + (lg.ShinPhi - thPhi);
            }
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
                // Walking: speed is the foot placement's job; the ankle only trims it, with a pressure point kept close to the ankle so the foot stays flat (a pressure point out at the toe lifts the heel and the foot rolls over).
                float trim = Clamp((inn.CmdF - inn.VelF) * 0.08f, -0.05f, 0.05f);
                pcF = centreF - trim;
                if (phase == GaitPhase.DoubleSupport) { pcF = centreF - 0.85f * inn.Geom.FootBack; }   // leaning into the start: pressure behind the centre of mass pushes it forward
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
                lo.AnkleTorque = !walking;
                lo.TorqueRho = -mg * share * dF;
                if (!planar)
                {
                    float dL = Clamp(pcL - centreL, -0.85f * inn.Geom.FootHalfWidth, 0.85f * inn.Geom.FootHalfWidth);
                    lo.TorquePhi = -mg * share * dL;
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
