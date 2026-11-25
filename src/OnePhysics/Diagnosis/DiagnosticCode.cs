namespace RecursiveAI.OnePhysics.Diagnosis;

/// <summary>
/// The stable code of a finding. Numbers are never reused or renumbered; a new check gets a new number. <see cref="Finding.Id"/> spells a code as <c>OPD</c> followed by the number
/// (<c>OPD1001</c>). The meaning, threshold and evidence of each code are in docs/spec/diagnosis.md.
/// </summary>
public enum DiagnosticCode
{
    /// <summary>Joints form a closed loop (or are otherwise stacked) that removes more degrees of freedom than the bodies have, and the joints end up fighting: their anchors do not stay together.</summary>
    JointsFighting = 1001,

    /// <summary>A joint connects two dynamic bodies whose mass ratio is beyond the range measured to be stable.</summary>
    JointMassRatio = 1002,

    /// <summary>Two dynamic bodies in contact, a heavy one resting on or pressed into a light one, with a mass ratio beyond the range measured to be stable.</summary>
    ContactMassRatio = 1003,

    /// <summary>A dynamic body is thinner than the speculative contact distance, or implausibly large, for the world's length unit.</summary>
    ImplausibleSize = 1004,

    /// <summary>A body moves at a speed near or beyond the engine's speed limit.</summary>
    ImplausibleSpeed = 1005,

    /// <summary>The gravity magnitude is implausible for the world's length unit.</summary>
    ImplausibleGravity = 1006,

    /// <summary>A body travels further in one step than it is thick and is not covered by continuous collision against the things it can meet.</summary>
    TunnellingRisk = 1007,

    /// <summary>A group of bodies stays awake for the whole observed time, or is held awake by its settings.</summary>
    NeverSleeps = 1008,

    /// <summary>Shapes of two bodies overlap deeply at the start of the observed time (they were created inside each other).</summary>
    ShapesOverlapping = 1009,

    /// <summary>A joint's anchor lies far from the centre of mass of the body it is attached to.</summary>
    AnchorFarFromBody = 1010,

    /// <summary>The two anchors of a point-like joint start far apart, so the first steps must pull the bodies together violently.</summary>
    AnchorsApart = 1011,

    /// <summary>A joint's position error stays large and does not fall over the observed time: the solve is not converging.</summary>
    JointNotConverging = 1012,

    /// <summary>The deepest contact penetration stays deep and does not fall over the observed time: the contact solve is not converging.</summary>
    PenetrationNotFalling = 1013,

    /// <summary>A joint motor is active for the observed time and never reaches its target speed: it works at its maximum force or torque.</summary>
    MotorSaturated = 1014,

    /// <summary>A joint limit is held active for the whole observed time while a motor of the same joint drives into it.</summary>
    LimitPinned = 1015,

    /// <summary>The probe could not keep stepping the copy of the world (a step threw), so nothing that needs time was judged past that point. The message names the exception.</summary>
    ProbeStopped = 1016,
}
