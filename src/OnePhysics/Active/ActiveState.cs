using System;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Active;

/// <summary>The state an active ragdoll keeps beside its ragdoll, common to both dimensions: the settings, the controller mode and command, per-bone blend weights and hit reactions, and the gait.</summary>
internal abstract class ActiveCommon
{
    public ActiveRagdollDef Def = new();
    public byte Mode;                       // ActiveMode
    public float CmdX, CmdZ, Heading;       // commanded velocity in the world's horizontal axes (2D: CmdX only) and heading, radians (3D yaw about +y; 2D unused)
    public float[] Weight = Array.Empty<float>(), Hit = Array.Empty<float>();
    public byte[] Driven = Array.Empty<byte>();   // 0 free, 1 driven by a motor or drive, 2 pinned (a bone's last state, so a release is noticed)
    public bool HasPose;
    public GaitState Gait;
    public bool RootDriven;                 // the leash pushed the figure last step (so it is not pushed twice)

    public int Parts => Weight.Length;

    public void AllocateCommon(int parts)
    {
        Weight = new float[parts]; Hit = new float[parts]; Driven = new byte[parts];
    }

    public static void WriteDef(StateWriter w, ActiveRagdollDef d)
    {
        w.NamedSingle(d.PdHertz); w.NamedSingle(d.LegPdHertz); w.NamedSingle(d.LoadPdHertz); w.NamedSingle(d.TargetSlew); w.NamedSingle(d.SwingTorqueScale); w.NamedSingle(d.PdDampingRatio); w.NamedSingle(d.TorqueScale);
        w.NamedByte((byte)d.RootMode); w.NamedSingle(d.RootTrackHertz); w.NamedSingle(d.HitRecoveryTime); w.NamedSingle(d.HitStrength); w.NamedSingle(d.HitRadius);
        var b = d.Balance;
        w.NamedSingle(b.CaptureGain); w.NamedSingle(b.StepTrigger); w.NamedSingle(b.StepTime); w.NamedSingle(b.StepHeight); w.NamedSingle(b.StandHeight); w.NamedSingle(b.MaxStepReach);
        var k = d.Walker;
        w.NamedSingle(k.StepTime); w.NamedSingle(k.StepHeight); w.NamedSingle(k.StepWidth); w.NamedSingle(k.HipHeight); w.NamedSingle(k.MaxSpeed); w.NamedSingle(k.SpeedFeedback);
        w.NamedSingle(k.LateralFeedback); w.NamedSingle(k.PlacementGain); w.NamedSingle(k.SwingLegHertz); w.NamedSingle(k.UpperBodyHertz); w.NamedSingle(k.TorqueScale); w.NamedSingle(k.TargetSlew); w.NamedSingle(k.TorsoLean); w.NamedSingle(k.ArmSwing); w.NamedSingle(k.StepTimeout); w.NamedSingle(k.ShiftTime);
        var o = d.Bones;
        w.NamedInt32(o.Pelvis); w.NamedInt32(o.Torso); w.NamedInt32(o.Head); w.NamedInt32(o.LeftThigh); w.NamedInt32(o.LeftShin); w.NamedInt32(o.LeftFoot);
        w.NamedInt32(o.RightThigh); w.NamedInt32(o.RightShin); w.NamedInt32(o.RightFoot);
    }

    public const int DefBytes = 7 * 4 + 1 + 4 * 4 + 6 * 4 + 16 * 4 + 9 * 4;

    public static ActiveRagdollDef ReadDef(StateReader r, int parts)
    {
        var d = new ActiveRagdollDef { PdHertz = r.ReadSingle(), LegPdHertz = r.ReadSingle(), LoadPdHertz = r.ReadSingle(), TargetSlew = r.ReadSingle(), SwingTorqueScale = r.ReadSingle(), PdDampingRatio = r.ReadSingle(), TorqueScale = r.ReadSingle() };
        byte root = r.ReadByte();
        if (root > (byte)AnimationRootMode.CenterOfMassTracked) { throw r.Error("invalid active ragdoll root mode"); }
        d.RootMode = (AnimationRootMode)root; d.RootTrackHertz = r.ReadSingle(); d.HitRecoveryTime = r.ReadSingle(); d.HitStrength = r.ReadSingle(); d.HitRadius = r.ReadSingle();
        var b = new BalanceDef { CaptureGain = r.ReadSingle(), StepTrigger = r.ReadSingle(), StepTime = r.ReadSingle(), StepHeight = r.ReadSingle(), StandHeight = r.ReadSingle(), MaxStepReach = r.ReadSingle() };
        var k = new WalkerDef
        {
            StepTime = r.ReadSingle(), StepHeight = r.ReadSingle(), StepWidth = r.ReadSingle(), HipHeight = r.ReadSingle(), MaxSpeed = r.ReadSingle(), SpeedFeedback = r.ReadSingle(),
            LateralFeedback = r.ReadSingle(), PlacementGain = r.ReadSingle(), SwingLegHertz = r.ReadSingle(), UpperBodyHertz = r.ReadSingle(), TorqueScale = r.ReadSingle(), TargetSlew = r.ReadSingle(), TorsoLean = r.ReadSingle(), ArmSwing = r.ReadSingle(), StepTimeout = r.ReadSingle(), ShiftTime = r.ReadSingle(),
        };
        var o = new BipedBones
        {
            Pelvis = r.ReadInt32(), Torso = r.ReadInt32(), Head = r.ReadInt32(), LeftThigh = r.ReadInt32(), LeftShin = r.ReadInt32(), LeftFoot = r.ReadInt32(),
            RightThigh = r.ReadInt32(), RightShin = r.ReadInt32(), RightFoot = r.ReadInt32(),
        };
        d.Balance = b; d.Walker = k; d.Bones = o;
        try { d.Validate(parts); }
        catch (ArgumentException e) { throw r.Error("invalid active ragdoll settings: " + e.Message); }
        return d;
    }

    /// <summary>Everything except the pose arrays, which the dimension-specific class writes (their element type differs).</summary>
    public void WriteCommon(StateWriter w)
    {
        WriteDef(w, Def);
        w.NamedByte(Mode); w.NamedSingle(CmdX); w.NamedSingle(CmdZ); w.NamedSingle(Heading);
        w.NamedSingles(Weight); w.NamedSingles(Hit); w.NamedBytes(Driven);
        w.NamedBool(HasPose); w.NamedBool(RootDriven);
        w.NamedByte(Gait.Phase); w.NamedByte(Gait.LastSwing); w.NamedInt32(Gait.Steps); w.NamedSingle(Gait.Time);
        w.NamedSingle(Gait.LiftF); w.NamedSingle(Gait.LiftL); w.NamedSingle(Gait.LiftU); w.NamedSingle(Gait.PlaceF); w.NamedSingle(Gait.PlaceL);
        w.NamedSingle(Gait.FVelF); w.NamedSingle(Gait.FVelL); w.NamedSingle(Gait.Height); w.NamedSingle(Gait.BiasF); w.NamedSingle(Gait.BiasL); w.NamedSingle(Gait.SumF); w.NamedSingle(Gait.SumL); w.NamedSingle(Gait.SumT); w.NamedByte(Gait.Primed);
    }

    public void ReadCommon(StateReader r, int parts)
    {
        Def = ReadDef(r, parts);
        Mode = r.ReadByte();
        if (Mode > (byte)ActiveMode.Walk) { throw r.Error("invalid active ragdoll mode"); }
        CmdX = r.ReadSingle(); CmdZ = r.ReadSingle(); Heading = r.ReadSingle();
        if (!float.IsFinite(CmdX) || !float.IsFinite(CmdZ) || !float.IsFinite(Heading) || Math.Abs(Heading) > 1e6f) { throw r.Error("invalid active ragdoll command"); }
        AllocateCommon(parts);
        r.ReadSingles(Weight, true, "active ragdoll weights");
        r.ReadSingles(Hit, true, "active ragdoll hit reactions");
        r.ReadBytes(Driven);
        for (int i = 0; i < parts; i++)
        {
            if (!(Weight[i] >= 0f && Weight[i] <= 1f) || !(Hit[i] >= 0f && Hit[i] <= 1f) || Driven[i] > 2) { throw r.Error("invalid active ragdoll bone state"); }
        }

        HasPose = r.ReadBool(); RootDriven = r.ReadBool();
        Gait.Phase = r.ReadByte(); Gait.LastSwing = r.ReadByte(); Gait.Steps = r.ReadInt32(); Gait.Time = r.ReadSingle();
        Gait.LiftF = r.ReadSingle(); Gait.LiftL = r.ReadSingle(); Gait.LiftU = r.ReadSingle(); Gait.PlaceF = r.ReadSingle(); Gait.PlaceL = r.ReadSingle();
        Gait.FVelF = r.ReadSingle(); Gait.FVelL = r.ReadSingle(); Gait.Height = r.ReadSingle(); Gait.BiasF = r.ReadSingle(); Gait.BiasL = r.ReadSingle(); Gait.SumF = r.ReadSingle(); Gait.SumL = r.ReadSingle(); Gait.SumT = r.ReadSingle(); Gait.Primed = r.ReadByte();
        if (Gait.Phase > (byte)GaitPhase.SwingRight) { throw r.Error("invalid active ragdoll gait state: phase"); }
        if (Gait.LastSwing > 1 || Gait.Primed > 1) { throw r.Error("invalid active ragdoll gait state: flags"); }
        if (Gait.Steps < 0) { throw r.Error("invalid active ragdoll gait state: step count"); }
        if (!float.IsFinite(Gait.Time) || Gait.Time < 0f) { throw r.Error("invalid active ragdoll gait state: time"); }
        // Foot targets are offsets between ankles of one figure, so they stay far inside the world's coordinate range; a larger value could only come from damaged bytes and would overflow the swing interpolation.
        const float FootLimit = 33554432f;   // 2^25 m
        if (!(Math.Abs(Gait.LiftF) <= FootLimit) || !(Math.Abs(Gait.LiftL) <= FootLimit) || !(Math.Abs(Gait.LiftU) <= FootLimit) || !(Math.Abs(Gait.PlaceF) <= FootLimit) || !(Math.Abs(Gait.PlaceL) <= FootLimit)) { throw r.Error("invalid active ragdoll gait state: foot targets (" + Gait.LiftF.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + Gait.LiftL.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + Gait.LiftU.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + Gait.PlaceF.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + Gait.PlaceL.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ") beyond 2^25"); }
        if (!float.IsFinite(Gait.FVelF) || !float.IsFinite(Gait.FVelL) || !float.IsFinite(Gait.BiasF) || !float.IsFinite(Gait.BiasL) || !float.IsFinite(Gait.SumF) || !float.IsFinite(Gait.SumL)) { throw r.Error("invalid active ragdoll gait state: velocity filter"); }
        if (!(Gait.Height >= 0f && Gait.Height < 1f)) { throw r.Error("invalid active ragdoll gait state: height"); }
        if (!(Gait.SumT >= 0f) || !float.IsFinite(Gait.SumT)) { throw r.Error("invalid active ragdoll gait state: summed time"); }
    }

    public void CloneCommonInto(ActiveCommon c)
    {
        c.Def = Def.Clone(); c.Mode = Mode; c.CmdX = CmdX; c.CmdZ = CmdZ; c.Heading = Heading;
        c.Weight = (float[])Weight.Clone(); c.Hit = (float[])Hit.Clone(); c.Driven = (byte[])Driven.Clone();
        c.HasPose = HasPose; c.Gait = Gait; c.RootDriven = RootDriven;
    }

    /// <summary>The effective simulated weight of a bone: its set weight raised towards 1 by the current hit reaction.</summary>
    public float EffectiveWeight(int i) => Weight[i] + (1f - Weight[i]) * Hit[i];

    /// <summary>Decays every hit reaction by one step: exponential recovery with the definition's time constant, exact in a fixed-step world.</summary>
    public void DecayHits(float dt)
    {
        // exp(-dt / tau) from onemath, so the same bits come out everywhere.
        float k = (float)RecursiveAI.OneMath.Math64.Exp(-(double)dt / Def.HitRecoveryTime);
        for (int i = 0; i < Hit.Length; i++)
        {
            float h = Hit[i] * k;
            Hit[i] = h < 1e-4f ? 0f : h;
        }
    }

    public void AddHit(int part, float amount)
    {
        float h = Hit[part] + amount * Def.HitStrength * (1f - Hit[part]);
        Hit[part] = h > 1f ? 1f : h < 0f ? 0f : h;
    }
}
