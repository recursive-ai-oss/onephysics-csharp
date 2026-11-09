using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Destruction;

/// <summary>Bond-stress settings of a breakable compound. Copied when a compound is created; the defaults suit a stone-like material at metre scale.</summary>
public sealed class BreakableDef
{
    /// <summary>Tensile strength of a bond in pascals (newtons per square metre; per metre of length in 2D, for unit thickness). A bond breaks when its stress reaches this. Default 100,000. Must be finite and above 0.</summary>
    public float Strength { get; set; } = 1e5f;

    /// <summary>Weight of the shear stress against the tensile stress in the failure measure. Default 1. Finite, at least 0.</summary>
    public float ShearFactor { get; set; } = 1f;

    /// <summary>Consecutive steps a bond must stay at or above its strength before it breaks, 1 to 1,000,000. Default 1. A larger value ignores single-step impulse noise.</summary>
    public int OverloadSteps { get; set; } = 1;

    /// <summary>Indices of the parts held by the world (the supports of a static structure). Only used on a static body: loads flow towards these parts, and after a break a piece holding one stays static. Default none.</summary>
    public int[]? AnchorParts { get; set; }

    internal void Validate(int parts, string paramName = "def")
    {
        if (!(Strength > 0f) || !float.IsFinite(Strength)) throw new ArgumentOutOfRangeException(paramName, Strength, "Strength must be finite and above 0.");
        if (!(ShearFactor >= 0f) || !float.IsFinite(ShearFactor)) throw new ArgumentOutOfRangeException(paramName, ShearFactor, "ShearFactor must be finite and at least 0.");
        if (OverloadSteps < 1 || OverloadSteps > 1000000) throw new ArgumentOutOfRangeException(paramName, OverloadSteps, "OverloadSteps must be within 1..1000000.");
        if (AnchorParts != null) foreach (int a in AnchorParts) if ((uint)a >= (uint)parts) throw new ArgumentOutOfRangeException(paramName, a, "An anchor index is outside the asset's parts (0.." + (parts - 1).ToString(System.Globalization.CultureInfo.InvariantCulture) + ").");
    }
}

/// <summary>
/// How a world keeps the fragments that fracture, slicing and bond breaks create: a budget, a lifetime, and conversion of settled fragments to static bodies.
/// Read with <c>world.Debris</c> and set by assigning a copy. All values default to 0, which turns the rule off. Saved with the world.
/// </summary>
public sealed class DebrisOptions
{
    /// <summary>The most fragments the world keeps. When there are more, the smallest are removed first (by volume in 3D, area in 2D), then the oldest, until the budget holds; checked at the end of every step. 0 = no limit.</summary>
    public int MaxFragments { get; set; }

    /// <summary>A fragment that has been asleep for this many consecutive steps is replaced by a static body with the same shapes (its handle changes), so it stops costing solver time. 0 = never.</summary>
    public int SleepToStaticSteps { get; set; }

    /// <summary>A fragment is removed this many steps after it was created. 0 = never.</summary>
    public long LifetimeSteps { get; set; }

    internal void Validate(string paramName = "value")
    {
        if (MaxFragments < 0) throw new ArgumentOutOfRangeException(paramName, MaxFragments, "MaxFragments must be at least 0.");
        if (SleepToStaticSteps < 0) throw new ArgumentOutOfRangeException(paramName, SleepToStaticSteps, "SleepToStaticSteps must be at least 0.");
        if (LifetimeSteps < 0) throw new ArgumentOutOfRangeException(paramName, LifetimeSteps, "LifetimeSteps must be at least 0.");
    }
}

/// <summary>A body could not be fractured or sliced; the message says why (an unsupported shape, a body that belongs to a joint, character, vehicle or ragdoll, or a cut that leaves fewer than two pieces).</summary>
public sealed class FractureException : InvalidOperationException
{
    /// <summary>Creates the exception.</summary>
    public FractureException(string message) : base(message) { }
}
