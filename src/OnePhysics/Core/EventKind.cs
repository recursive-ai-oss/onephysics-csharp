using System;
using System.Collections.Generic;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>The kinds of event a step can produce, in the order they sort in a delivery.</summary>
public enum EventKind : byte
{
    /// <summary>A contact constraint was created.</summary>
    ContactBegin = 0,

    /// <summary>A contact is still there at the end of a step (only for contacts flagged <see cref="ConstraintBits.ReportPersist"/>).</summary>
    ContactPersist = 1,

    /// <summary>A contact constraint was removed.</summary>
    ContactEnd = 2,

    /// <summary>A sensor overlap began.</summary>
    SensorEnter = 3,

    /// <summary>A sensor overlap ended.</summary>
    SensorExit = 4,

    /// <summary>A joint broke and was removed. <see cref="PhysicsEvent.Value"/> is the break value.</summary>
    JointBreak = 5,

    /// <summary>A body fell asleep.</summary>
    Sleep = 6,

    /// <summary>A body woke up.</summary>
    Wake = 7,

    /// <summary>A contact's approach speed at the start of the step was above the world's hit threshold; <see cref="PhysicsEvent.Value"/> is the approach speed. Opt-in through <see cref="EventMask.ContactHit"/>.</summary>
    ContactHit = 8,
}
