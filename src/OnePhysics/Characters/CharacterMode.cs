namespace RecursiveAI.OnePhysics;

/// <summary>What a character is doing besides walking: sitting in a seat, hanging from a ledge, climbing a surface or holding a rope. Walking and falling are <see cref="Free"/>.</summary>
public enum CharacterMode : byte
{
    /// <summary>Walking, standing or falling under the character's own move calls.</summary>
    Free = 0,

    /// <summary>Mounted on a body at a fixed pose in that body's frame (a vehicle seat, a saddle, a deck chair). The body's motion carries the character; move calls are ignored.</summary>
    Seated = 1,

    /// <summary>Hanging by the hands from a ledge. Moves slide the character along the ledge; <c>ClimbUp</c> pulls it over; <c>Release</c> lets go.</summary>
    Hanging = 2,

    /// <summary>Climbing a surface marked climbable (a ladder, a vine wall). Moves go up, down and sideways along it.</summary>
    Climbing = 3,

    /// <summary>Holding a rod (a rope, a vine, a rail). The character's weight loads the rod, it swings with it, and moves climb along it.</summary>
    Roped = 4,
}
