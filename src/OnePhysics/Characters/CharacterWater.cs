namespace RecursiveAI.OnePhysics;

/// <summary>Where a character is relative to water: standing clear of it, wading (walking with the water around its legs) or swimming (carried by the water, moved by swim strokes).</summary>
public enum CharacterWaterState : byte
{
    /// <summary>Not in water (less than 5% of the height under the surface; the character leaves this state below 2%).</summary>
    Dry = 0,

    /// <summary>Walking in water: the character is slowed in proportion to how deep it is and the current pushes it a little. The host's own walking and gravity still apply.</summary>
    Wading = 1,

    /// <summary>Swimming: the water carries the character (its current, buoyancy and drag) and the displacement given to <c>Move</c> is a swim intent relative to the water, not a walking step. Entered at 75% immersion, left below 60%.</summary>
    Swimming = 2,
}

/// <summary>What a <c>CharacterWaterEvent</c> reports.</summary>
public enum CharacterWaterEventKind : byte
{
    /// <summary>The character went from dry to wading or swimming.</summary>
    EnteredWater = 0,

    /// <summary>The character went from wading or swimming to dry.</summary>
    LeftWater = 1,

    /// <summary>The character started swimming.</summary>
    StartedSwimming = 2,

    /// <summary>The character stopped swimming (it is wading or dry now).</summary>
    StoppedSwimming = 3,
}

/// <summary>Which kind of water a character is in.</summary>
public enum CharacterWaterSource : byte
{
    /// <summary>No water.</summary>
    None = 0,

    /// <summary>A buoyancy volume (an infinite flat pool: <c>AddFluid</c> or a buoyancy field).</summary>
    BuoyancyVolume = 1,

    /// <summary>A shallow-water grid.</summary>
    ShallowWater = 2,

    /// <summary>Particle fluid.</summary>
    ParticleFluid = 3,
}
