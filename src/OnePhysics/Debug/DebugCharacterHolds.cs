using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>Where a character is held: a seat, a ledge, a climbed surface or a rope grip, as plain data. Planar worlds leave z at 0.</summary>
public readonly struct DebugCharacterHold
{
    /// <summary>Creates an entry.</summary>
    public DebugCharacterHold(BodyHandle character, byte mode, BodyHandle held, Vector3D point, Vector3F normal)
    { Character = character; Mode = (CharacterMode)mode; Held = held; Point = point; Normal = normal; }

    /// <summary>The character's body.</summary>
    public BodyHandle Character { get; }

    /// <summary>Seated, hanging, climbing or roped.</summary>
    public CharacterMode Mode { get; }

    /// <summary>The body that holds the character (the seat's body, the ledge's body); the null handle for a rope grip.</summary>
    public BodyHandle Held { get; }

    /// <summary>The seat position, the character centre while hanging or climbing, or the point of the rod gripped.</summary>
    public Vector3D Point { get; }

    /// <summary>The held surface's unit normal towards the character; zero for a seat or a rope.</summary>
    public Vector3F Normal { get; }
}

public sealed partial class DebugGeometry
{
    private DebugCharacterHold[] _holds = new DebugCharacterHold[4];
    private int _nHolds;

    /// <summary>One entry per character that is seated, hanging, climbing or holding a rope, in character slot order, valid until the next collection or Clear.</summary>
    public ReadOnlySpan<DebugCharacterHold> CharacterHolds => _holds.AsSpan(0, _nHolds);

    /// <summary>Appends a hold; owned storage grows when necessary.</summary>
    public void Add(in DebugCharacterHold h) => Push(ref _holds, ref _nHolds, in h);
}
