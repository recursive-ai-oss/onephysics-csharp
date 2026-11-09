using System;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>Where a ragdoll's root (the pelvis in the humanoid presets) comes from while an animation pose is blended in.</summary>
[Experimental("OPHYS0003")]
public enum AnimationRootMode : byte
{
    /// <summary>The root follows the animation's root transform exactly while its blend weight is 0, and is simulated as it is weighted up. The default.</summary>
    Animated = 0,

    /// <summary>The root is always simulated and the animation's root transform is ignored; the other bones follow the animation relative to the simulated root.</summary>
    Physics = 1,

    /// <summary>The root is simulated but a soft spring-damper (<see cref="ActiveRagdollDef.RootTrackHertz"/>) pulls the whole figure's centre of mass towards the animation root's position sideways and vertically, scaled by (1 − the root's blend weight).</summary>
    CenterOfMassTracked = 2,
}

/// <summary>What an active ragdoll's controller is doing.</summary>
[Experimental("OPHYS0003")]
public enum ActiveMode : byte
{
    /// <summary>No locomotion controller: the bones follow the animation pose (or the bind pose) according to their blend weights.</summary>
    Pose = 0,

    /// <summary>Standing balance with push recovery: ankle torque limited to the support polygon, and a recovery step when the capture point leaves it.</summary>
    Stand = 1,

    /// <summary>A finite-state gait that follows the commanded velocity and heading.</summary>
    Walk = 2,
}

/// <summary>The gait's finite state, readable from <see cref="ActiveRagdollInfo"/>.</summary>
[Experimental("OPHYS0003")]
public enum GaitPhase : byte
{
    /// <summary>Both feet planted; the balance law runs.</summary>
    DoubleSupport = 0,

    /// <summary>Weight is moving over the stance foot before a step starts (3D only).</summary>
    Shift = 1,

    /// <summary>The left leg (the first leg in the skeleton) swings, the right leg carries the body.</summary>
    SwingLeft = 2,

    /// <summary>The right leg swings, the left leg carries the body.</summary>
    SwingRight = 3,
}

/// <summary>Which parts of a skeleton are the pelvis, torso, head, arms and legs, for the balance and walking controllers. The defaults are the layout of <c>RagdollDef.Humanoid</c> in both dimensions.</summary>
[Experimental("OPHYS0003")]
public sealed class BipedBones
{
    /// <summary>The root part. Default 0.</summary>
    public int Pelvis { get; set; }

    /// <summary>The torso, a child of the pelvis. Default 1.</summary>
    public int Torso { get; set; } = 1;

    /// <summary>The head, a child of the torso, or -1 for none. Default 2.</summary>
    public int Head { get; set; } = 2;

    /// <summary>Thigh of the first (left; +x in 3D) leg; its child is the shin and the shin's child the foot. Default 7.</summary>
    public int LeftThigh { get; set; } = 7;

    /// <summary>Shin of the first leg. Default 8.</summary>
    public int LeftShin { get; set; } = 8;

    /// <summary>Foot of the first leg. Default 9.</summary>
    public int LeftFoot { get; set; } = 9;

    /// <summary>Thigh of the second (right) leg. Default 10.</summary>
    public int RightThigh { get; set; } = 10;

    /// <summary>Shin of the second leg. Default 11.</summary>
    public int RightShin { get; set; } = 11;

    /// <summary>Foot of the second leg. Default 12.</summary>
    public int RightFoot { get; set; } = 12;

    internal BipedBones Clone() => (BipedBones)MemberwiseClone();

    internal int Leg(int leg, int joint) => leg == 0 ? (joint == 0 ? LeftThigh : joint == 1 ? LeftShin : LeftFoot) : (joint == 0 ? RightThigh : joint == 1 ? RightShin : RightFoot);
}

/// <summary>
/// Standing balance and push recovery. The law (Pratt et al. 2006; Koolen et al. 2012): the capture point ξ = x + v/ω₀ (centre of mass position plus velocity over the pendulum frequency) says where the feet must go to
/// stop the fall. While ξ stays inside the support polygon the ankles apply the torque that moves the centre of pressure towards ξ, never further than the foot's edge (a zero-moment-point limit); when ξ leaves the polygon
/// by more than <see cref="StepTrigger"/> the figure takes a step and puts the swing foot on ξ.
/// </summary>
[Experimental("OPHYS0003")]
public sealed class BalanceDef
{
    /// <summary>Centre-of-pressure feedback: the pressure point is put at ξ + gain · (ξ − the support centre). At least 0. Default 2.5.</summary>
    public float CaptureGain { get; set; } = 2.5f;

    /// <summary>How far beyond the support polygon's edge, m, the capture point may be before a recovery step starts. Negative values start a step while the capture point is still inside. Default -0.03.</summary>
    public float StepTrigger { get; set; } = -0.03f;

    /// <summary>Duration of a recovery step, s. Default 0.3.</summary>
    public float StepTime { get; set; } = 0.3f;

    /// <summary>Peak height of the swing foot above the straight line between its lift-off and landing, m. Default 0.07.</summary>
    public float StepHeight { get; set; } = 0.07f;

    /// <summary>Hip height as a fraction of the straight leg length while standing; below 1 keeps the knees slightly bent. Within (0.5, 1). Default 0.97.</summary>
    public float StandHeight { get; set; } = 0.995f;

    /// <summary>The farthest the swing foot may land from the stance foot, as a fraction of the leg length. Default 0.85.</summary>
    public float MaxStepReach { get; set; } = 0.85f;

    /// <summary>Standing balance: ankle strategy, a recovery step beyond a little over the foot's length. <c>BalanceDef.Stand</c> is a fresh copy each time.</summary>
    public static BalanceDef Stand => new();

    internal BalanceDef Clone() => (BalanceDef)MemberwiseClone();

    internal void Validate()
    {
        Check(CaptureGain, 0f, 100f, nameof(CaptureGain));
        Check(StepTrigger, -0.5f, 5f, nameof(StepTrigger));
        Check(StepTime, 0.05f, 5f, nameof(StepTime));
        Check(StepHeight, 0f, 2f, nameof(StepHeight));
        Check(StandHeight, 0.5f, 0.999f, nameof(StandHeight));
        Check(MaxStepReach, 0.1f, 1.5f, nameof(MaxStepReach));
    }

    internal static void Check(float v, float lo, float hi, string name)
    {
        if (!(v >= lo && v <= hi)) { throw Bad(name, v, name + " = " + v.ToString("R", CultureInfo.InvariantCulture) + " must be finite and within " + lo.ToString("R", CultureInfo.InvariantCulture) + ".." + hi.ToString("R", CultureInfo.InvariantCulture) + "."); }
    }

    internal static ArgumentOutOfRangeException Bad(string name, object value, string message) => new(name, value, message);
}

/// <summary>
/// A walking controller in the SIMBICON family (Yin, Loken and van de Panne 2007; Coros, Beaudoin and van de Panne 2010): a finite-state gait with one leg swinging while the other carries the body, the swing foot
/// placed by feedback on the centre of mass position and velocity (the capture point), the stance hip holding the pelvis level, the knee holding the hip height, and the ankle keeping the foot flat on the ground.
/// </summary>
[Experimental("OPHYS0003")]
public sealed class WalkerDef
{
    /// <summary>Duration of one step (one leg's swing), s. Default 0.42.</summary>
    public float StepTime { get; set; } = 0.47f;

    /// <summary>Peak swing-foot clearance above the straight line from lift-off to landing, m. Default 0.08.</summary>
    public float StepHeight { get; set; } = 0.0867f;

    /// <summary>Distance between the two feet sideways at touchdown, m (3D only; 2D ignores it). Default 0.2.</summary>
    public float StepWidth { get; set; } = 0.2f;

    /// <summary>Hip height as a fraction of the straight leg length. Within (0.5, 1). Default 0.95.</summary>
    public float HipHeight { get; set; } = 0.8785f;

    /// <summary>The fastest commanded speed, m/s; a command above it is clamped. Default 2.5.</summary>
    public float MaxSpeed { get; set; } = 2.5f;

    /// <summary>Speed feedback on the foot placement: the landing point moves by this times (speed − commanded speed) / ω₀ (1 is the capture point). At least 0. Default 1.2.</summary>
    public float SpeedFeedback { get; set; } = 0.625f;

    /// <summary>How much of the capture point's error at the end of the swing the foot placement corrects: 1 puts the foot where the fall would stop exactly (and so passes every measurement error on at the pendulum's growth rate), less is gentler. Within 0..2. Default 1.</summary>
    public float PlacementGain { get; set; } = 1f;

    /// <summary>Natural frequency, Hz, of the swinging leg's drive while walking (the leg that carries the body uses <see cref="ActiveRagdollDef.LoadPdHertz"/>). Within 0..1000. Default 10.5.</summary>
    public float SwingLegHertz { get; set; } = 10.5f;

    /// <summary>Natural frequency, Hz, of the torso, head and arms while walking. Within 0..1000. Default 8.</summary>
    public float UpperBodyHertz { get; set; } = 8f;

    /// <summary>The fastest a walking target may move, rad/s (replaces <see cref="ActiveRagdollDef.TargetSlew"/> while walking). Within 0.1..1000. Default 17.2.</summary>
    public float TargetSlew { get; set; } = 17.2f;

    /// <summary>Multiplier on every joint's torque limit while walking (on top of <see cref="ActiveRagdollDef.TorqueScale"/>): a walker needs more than a standing figure. Within 0..1000. Default 1.74.</summary>
    public float TorqueScale { get; set; } = 1.74f;

    /// <summary>Sideways feedback on the foot placement (3D), same scale. Default 1.3.</summary>
    public float LateralFeedback { get; set; } = 1.3f;

    /// <summary>Forward lean of the torso while walking, radians per m/s of commanded speed. Default 0.02.</summary>
    public float TorsoLean { get; set; } = -0.02f;

    /// <summary>Arm swing against the legs, as a fraction of the thigh swing (0 holds the arms where the pose puts them). Default 0.5.</summary>
    public float ArmSwing { get; set; } = 0.5f;

    /// <summary>The longest a step may take before it is cut short as a stumble, as a multiple of <see cref="StepTime"/>. Default 1.6.</summary>
    public float StepTimeout { get; set; } = 1.6f;

    /// <summary>Time allowed for the weight to move over the stance foot before the first step (3D), s. Default 0.6.</summary>
    public float ShiftTime { get; set; } = 0.6f;

    /// <summary>A walker for the thirteen-part humanoid presets: 0.42 s steps, 8 cm foot clearance, 20 cm step width, hip height 0.97 of the leg. A fresh copy each time.</summary>
    public static WalkerDef Humanoid => new();

    internal WalkerDef Clone() => (WalkerDef)MemberwiseClone();

    internal void Validate()
    {
        BalanceDef.Check(StepTime, 0.05f, 5f, nameof(StepTime));
        BalanceDef.Check(StepHeight, 0f, 2f, nameof(StepHeight));
        BalanceDef.Check(StepWidth, 0f, 2f, nameof(StepWidth));
        BalanceDef.Check(HipHeight, 0.5f, 0.999f, nameof(HipHeight));
        BalanceDef.Check(MaxSpeed, 0f, 50f, nameof(MaxSpeed));
        BalanceDef.Check(SpeedFeedback, 0f, 20f, nameof(SpeedFeedback));
        BalanceDef.Check(LateralFeedback, 0f, 20f, nameof(LateralFeedback));
        BalanceDef.Check(PlacementGain, 0f, 2f, nameof(PlacementGain));
        BalanceDef.Check(SwingLegHertz, 0f, 1000f, nameof(SwingLegHertz));
        BalanceDef.Check(TargetSlew, 0.1f, 1000f, nameof(TargetSlew));
        BalanceDef.Check(TorqueScale, 0f, 1000f, nameof(TorqueScale));
        BalanceDef.Check(UpperBodyHertz, 0f, 1000f, nameof(UpperBodyHertz));
        BalanceDef.Check(TorsoLean, -1f, 1f, nameof(TorsoLean));
        BalanceDef.Check(ArmSwing, 0f, 2f, nameof(ArmSwing));
        BalanceDef.Check(StepTimeout, 1f, 10f, nameof(StepTimeout));
        BalanceDef.Check(ShiftTime, 0f, 10f, nameof(ShiftTime));
    }
}

/// <summary>
/// What turns a ragdoll into an active one: the pose drive's gains (a stable proportional-derivative controller, Tan, Liu and Turk 2011, so any gain is safe at any time step), animation blending, hit reactions
/// and the optional balance and walking controllers. Copied when the ragdoll is made active.
/// </summary>
[Experimental("OPHYS0003")]
public sealed class ActiveRagdollDef
{
    /// <summary>
    /// Natural frequency, Hz, of the pose drive on the arms, torso and head: stiffness is the joint's effective inertia times (2π · this)². Stable at any value and any time step (the drive is integrated implicitly), so
    /// raise it until the pose is held tightly enough. Within 0..1000. Default 14.
    /// </summary>
    public float PdHertz { get; set; } = 14f;

    /// <summary>Natural frequency, Hz, of the leg joints' drive (hips, knees, ankles), which carry the body's weight. Within 0..1000. Default 22.</summary>
    public float LegPdHertz { get; set; } = 22f;

    /// <summary>Natural frequency, Hz, of the leg joints of a leg that stands on the ground, measured against the whole body they carry (so it is a stiffness for holding weight, not for moving a limb). Within 0..1000. Default 4.86.</summary>
    public float LoadPdHertz { get; set; } = 4.86f;

    /// <summary>The fastest a controller target may move, rad/s: targets are rate limited so a stiff drive is never asked for a jump. Within 0.1..1000. Default 10.</summary>
    public float TargetSlew { get; set; } = 10f;

    /// <summary>The torque limit of a swinging leg's joints as a fraction of their <c>MaxTorque</c>: a swing leg that could push as hard as a stance leg would spin the pelvis (every torque on the leg is felt as the opposite torque on the pelvis). Within 0..1. Default 0.2.</summary>
    public float SwingTorqueScale { get; set; } = 0.394f;

    /// <summary>Damping ratio of every drive: 1 is critical damping. Within 0..100. Default 1.</summary>
    public float PdDampingRatio { get; set; } = 1f;

    /// <summary>Multiplier on the ragdoll definition's <c>MaxTorque</c> of every joint. Within 0..1000. Default 1.</summary>
    public float TorqueScale { get; set; } = 1f;

    /// <summary>Where the root comes from while animation is blended in. Default <see cref="AnimationRootMode.Animated"/>.</summary>
    public AnimationRootMode RootMode { get; set; } = AnimationRootMode.Animated;

    /// <summary>Natural frequency, Hz, of the centre-of-mass leash of <see cref="AnimationRootMode.CenterOfMassTracked"/>. Default 3.</summary>
    public float RootTrackHertz { get; set; } = 3f;

    /// <summary>Time constant of a hit reaction's recovery, s: a bone's simulated weight returns to its set weight with this time constant (exponentially). Positive. Default 0.35.</summary>
    public float HitRecoveryTime { get; set; } = 0.35f;

    /// <summary>How far a hit of strength 1 pushes a bone towards fully simulated (weight 1), 0..1. Default 1.</summary>
    public float HitStrength { get; set; } = 1f;

    /// <summary>How far from the hit, m, neighbouring bones are affected; the effect falls off linearly to 0 at this distance. Positive. Default 0.45.</summary>
    public float HitRadius { get; set; } = 0.45f;

    /// <summary>Balance and recovery settings used while the mode is <see cref="ActiveMode.Stand"/> and between steps of a walk. Default <see cref="BalanceDef.Stand"/>.</summary>
    public BalanceDef Balance { get; set; } = BalanceDef.Stand;

    /// <summary>Walking settings. Default <see cref="WalkerDef.Humanoid"/>.</summary>
    public WalkerDef Walker { get; set; } = WalkerDef.Humanoid;

    /// <summary>Which parts are the pelvis, torso, legs and so on. Default: the humanoid layout.</summary>
    public BipedBones Bones { get; set; } = new();

    /// <summary>
    /// A person: 14 Hz arms, torso and head and 22 Hz legs at critical damping, the animated root, a 0.35 s hit recovery, <see cref="BalanceDef.Stand"/> and <see cref="WalkerDef.Humanoid"/>. A fresh copy each time.
    /// </summary>
    public static ActiveRagdollDef Humanoid => new();

    internal ActiveRagdollDef Clone() => new()
    {
        PdHertz = PdHertz, LegPdHertz = LegPdHertz, LoadPdHertz = LoadPdHertz, TargetSlew = TargetSlew, SwingTorqueScale = SwingTorqueScale, PdDampingRatio = PdDampingRatio, TorqueScale = TorqueScale, RootMode = RootMode, RootTrackHertz = RootTrackHertz, HitRecoveryTime = HitRecoveryTime, HitStrength = HitStrength, HitRadius = HitRadius,
        Balance = Balance?.Clone()!, Walker = Walker?.Clone()!, Bones = Bones?.Clone()!,
    };

    internal void Validate(int parts)
    {
        BalanceDef.Check(PdHertz, 0f, 1000f, nameof(PdHertz));
        BalanceDef.Check(LegPdHertz, 0f, 1000f, nameof(LegPdHertz));
        BalanceDef.Check(LoadPdHertz, 0f, 1000f, nameof(LoadPdHertz));
        BalanceDef.Check(TargetSlew, 0.1f, 1000f, nameof(TargetSlew));
        BalanceDef.Check(SwingTorqueScale, 0f, 1f, nameof(SwingTorqueScale));
        BalanceDef.Check(PdDampingRatio, 0f, 100f, nameof(PdDampingRatio));
        BalanceDef.Check(TorqueScale, 0f, 1000f, nameof(TorqueScale));
        if (!Enum.IsDefined(RootMode)) { throw BalanceDef.Bad(nameof(RootMode), RootMode, "Unknown root mode."); }
        BalanceDef.Check(RootTrackHertz, 0.01f, 100f, nameof(RootTrackHertz));
        BalanceDef.Check(HitRecoveryTime, 0.001f, 100f, nameof(HitRecoveryTime));
        BalanceDef.Check(HitStrength, 0f, 1f, nameof(HitStrength));
        BalanceDef.Check(HitRadius, 0.001f, 100f, nameof(HitRadius));
        if (Balance == null || Walker == null || Bones == null) { throw new ArgumentNullException(Balance == null ? nameof(Balance) : Walker == null ? nameof(Walker) : nameof(Bones), "Balance, Walker and Bones must be set."); }
        Balance.Validate();
        Walker.Validate();
        Span<int> idx = stackalloc int[10] { Bones.Pelvis, Bones.Torso, Bones.LeftThigh, Bones.LeftShin, Bones.LeftFoot, Bones.RightThigh, Bones.RightShin, Bones.RightFoot, Bones.Head, Bones.Head };
        string[] names = { "Pelvis", "Torso", "LeftThigh", "LeftShin", "LeftFoot", "RightThigh", "RightShin", "RightFoot", "Head", "Head" };
        for (int i = 0; i < 8; i++)
        {
            if ((uint)idx[i] >= (uint)parts) { throw BalanceDef.Bad("Bones." + names[i], idx[i], "Bones." + names[i] + " = " + idx[i].ToString(CultureInfo.InvariantCulture) + " is not a part of the ragdoll (0.." + (parts - 1).ToString(CultureInfo.InvariantCulture) + ")."); }
            for (int j = 0; j < i; j++) { if (idx[i] == idx[j]) { throw BalanceDef.Bad("Bones." + names[i], idx[i], "Bones." + names[i] + " and Bones." + names[j] + " are the same part."); } }
        }

        if (Bones.Head < -1 || Bones.Head >= parts) { throw BalanceDef.Bad("Bones.Head", Bones.Head, "Bones.Head must be a part index or -1."); }
    }
}

/// <summary>A read-only view of an active ragdoll's controller, from <c>Ragdoll.GetActiveInfo</c>: for hosts, tests and debug drawing.</summary>
[Experimental("OPHYS0003")]
public readonly struct ActiveRagdollInfo
{
    internal ActiveRagdollInfo(ActiveMode mode, GaitPhase phase, int stepCount, float time, float comX, float comY, float comZ, float captureX, float captureZ, float supportMinX, float supportMinZ, float supportMaxX, float supportMaxZ, int supportFeet)
    {
        Mode = mode; Phase = phase; StepCount = stepCount; PhaseTime = time;
        CenterOfMassX = comX; CenterOfMassHeight = comY; CenterOfMassZ = comZ; CapturePointX = captureX; CapturePointZ = captureZ;
        SupportMinX = supportMinX; SupportMinZ = supportMinZ; SupportMaxX = supportMaxX; SupportMaxZ = supportMaxZ; GroundedFeet = supportFeet;
    }

    /// <summary>The controller mode.</summary>
    public ActiveMode Mode { get; }

    /// <summary>The gait's finite state.</summary>
    public GaitPhase Phase { get; }

    /// <summary>Steps completed since the controller was last (re)started.</summary>
    public int StepCount { get; }

    /// <summary>Seconds in the current phase.</summary>
    public float PhaseTime { get; }

    /// <summary>Centre of mass of the whole figure at the end of the last step, world X (2D and 3D).</summary>
    public float CenterOfMassX { get; }

    /// <summary>Centre of mass height above the world origin (world Y).</summary>
    public float CenterOfMassHeight { get; }

    /// <summary>Centre of mass world Z (0 in 2D).</summary>
    public float CenterOfMassZ { get; }

    /// <summary>The capture point on the ground, world X.</summary>
    public float CapturePointX { get; }

    /// <summary>The capture point, world Z (0 in 2D).</summary>
    public float CapturePointZ { get; }

    /// <summary>Support polygon bounding box, world X minimum (the polygon is the convex hull of the grounded feet's footprints; the box is what the controller uses).</summary>
    public float SupportMinX { get; }

    /// <summary>Support box, world Z minimum.</summary>
    public float SupportMinZ { get; }

    /// <summary>Support box, world X maximum.</summary>
    public float SupportMaxX { get; }

    /// <summary>Support box, world Z maximum.</summary>
    public float SupportMaxZ { get; }

    /// <summary>How many feet were on the ground (0, 1 or 2).</summary>
    public int GroundedFeet { get; }
}
