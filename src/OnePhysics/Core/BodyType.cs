using System;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>How a body takes part in the simulation.</summary>
public enum BodyType : byte
{
    /// <summary>Never moves, has no inverse mass, joins no island and no color.</summary>
    Static = 0,

    /// <summary>Moves under user control, has no inverse mass, never sleeps, joins no island and no color.</summary>
    Kinematic = 1,

    /// <summary>Moves under forces and constraints; belongs to an island.</summary>
    Dynamic = 2,
}
