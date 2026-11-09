namespace RecursiveAI.OnePhysics;

/// <summary>Outcome of bounded iterative depenetration. A failure returns a tested partial correction, never a claim that the query is clear.</summary>
public enum DepenetrationStatus : byte
{
    /// <summary>The default result: no query was run and no correction is certified.</summary>
    NotRun,
    /// <summary>Every accepted obstacle has penetration no deeper than the requested tolerance.</summary>
    Cleared,
    /// <summary>The correction iteration bound was reached.</summary>
    IterationLimit,
    /// <summary>The sum of correction lengths exhausted the requested distance budget.</summary>
    DistanceLimit,
    /// <summary>A correction would leave the supported query coordinate range; the last valid pose is returned.</summary>
    CoordinateLimit,
    /// <summary>The penetration routine supplied no usable finite direction.</summary>
    DegenerateContact
}
