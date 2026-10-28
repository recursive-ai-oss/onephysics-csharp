using System;
using System.Globalization;
using RecursiveAI.OneMath;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Which categories of events a world records. Disabled kinds cost nothing.</summary>
[Flags]
public enum EventMask : ushort
{
    /// <summary>No events.</summary>
    None = 0,

    /// <summary>A contact constraint was created.</summary>
    ContactBegin = 1 << 0,

    /// <summary>A contact constraint exists at the end of a step and asked for persist events.</summary>
    ContactPersist = 1 << 1,

    /// <summary>A contact constraint was removed.</summary>
    ContactEnd = 1 << 2,

    /// <summary>A sensor overlap began.</summary>
    SensorEnter = 1 << 3,

    /// <summary>A sensor overlap ended.</summary>
    SensorExit = 1 << 4,

    /// <summary>A joint broke.</summary>
    JointBreak = 1 << 5,

    /// <summary>A body fell asleep.</summary>
    Sleep = 1 << 6,

    /// <summary>A body woke.</summary>
    Wake = 1 << 7,

    /// <summary>A contact approached faster than the engine's hit-speed threshold. Not part of <see cref="Default"/>.</summary>
    ContactHit = 1 << 8,

    /// <summary>Everything except per-step persist events and hit events.</summary>
    Default = 0xFF & ~ContactPersist,

    /// <summary>Every kind.</summary>
    All = 0x1FF,
}
