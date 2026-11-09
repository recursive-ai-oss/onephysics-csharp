using System;

namespace RecursiveAI.OnePhysics;

/// <summary>A continuous query could not certify further progress. The world is unchanged; no collision is silently missed. SafeFraction is the last certified fraction before impact.</summary>
public sealed class QueryConvergenceException : PhysicsException
{
    /// <summary>Creates an error for a continuous query with a finite safe fraction in [0,1].</summary>
    /// <exception cref="ArgumentOutOfRangeException">The fraction is not finite or outside [0,1].</exception>
    public QueryConvergenceException(double safeFraction) : base("Conservative advancement could not certify progress at fraction " + safeFraction.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ". Increase the contact tolerance or shorten the sweep.")
    {
        if (!double.IsFinite(safeFraction) || safeFraction < 0 || safeFraction > 1) { throw new ArgumentOutOfRangeException(nameof(safeFraction)); }
        SafeFraction = safeFraction;
    }

    /// <summary>Last certified safe fraction, in [0,1]; not a claim that a collision occurs there.</summary>
    public double SafeFraction { get; }
}
