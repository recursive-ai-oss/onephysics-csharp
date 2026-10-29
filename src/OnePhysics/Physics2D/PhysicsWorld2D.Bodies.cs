using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>The number of live bodies, characters' bodies included. Allocates nothing.</summary>
    public int BodyCount => _core.Bodies.Count;

    /// <summary>
    /// The live bodies in slot order (the order they were created in, with destroyed slots reused lowest first), for <c>foreach (Body b in world.Bodies)</c>. Allocates nothing. Do not create or destroy bodies
    /// while enumerating: a body created during the loop may or may not be visited, and a destroyed one that is reached throws like any stale <see cref="Body"/>.
    /// </summary>
    public BodyEnumerable Bodies => new(this);

    /// <summary>The number of live characters. Allocates nothing.</summary>
    public int CharacterCount => _engine.CharHandles.Count;

    /// <summary>The live characters in slot order, for <c>foreach (Character c in world.Characters)</c>. Allocates nothing; the same rules as <see cref="Bodies"/> apply. <see cref="GetCharacters"/> returns an array instead.</summary>
    public CharacterEnumerable Characters => new(this);
}

/// <summary>The live bodies of a world, in slot order; see <see cref="PhysicsWorld2D.Bodies"/>.</summary>
public readonly struct BodyEnumerable
{
    private readonly PhysicsWorld2D _world;

    internal BodyEnumerable(PhysicsWorld2D world) => _world = world;

    /// <summary>Gets the enumerator.</summary>
    public BodyEnumerator GetEnumerator() => new(_world);
}

/// <summary>Walks the live bodies of a world in slot order. Allocates nothing.</summary>
public struct BodyEnumerator
{
    private readonly PhysicsWorld2D _world;
    private int _slot;

    internal BodyEnumerator(PhysicsWorld2D world)
    {
        _world = world;
        _slot = -1;
        Current = default;
    }

    /// <summary>The body the enumerator is at.</summary>
    public Body Current { get; private set; }

    /// <summary>Moves to the next live body; false at the end.</summary>
    public bool MoveNext()
    {
        var bodies = _world.Core.Bodies;
        while (++_slot < bodies.HighWater)
        {
            if (bodies.IsLive(_slot))
            {
                Current = new Body(_world, _world.Core.HandleOfBody(_slot));
                return true;
            }
        }

        return false;
    }
}

/// <summary>The live characters of a world, in slot order; see <see cref="PhysicsWorld2D.Characters"/>.</summary>
public readonly struct CharacterEnumerable
{
    private readonly PhysicsWorld2D _world;

    internal CharacterEnumerable(PhysicsWorld2D world) => _world = world;

    /// <summary>Gets the enumerator.</summary>
    public CharacterEnumerator GetEnumerator() => new(_world);
}

/// <summary>Walks the live characters of a world in slot order. Allocates nothing.</summary>
public struct CharacterEnumerator
{
    private readonly PhysicsWorld2D _world;
    private int _slot;

    internal CharacterEnumerator(PhysicsWorld2D world)
    {
        _world = world;
        _slot = -1;
        Current = default;
    }

    /// <summary>The character the enumerator is at.</summary>
    public Character Current { get; private set; }

    /// <summary>Moves to the next live character; false at the end.</summary>
    public bool MoveNext()
    {
        var handles = _world.Engine.CharHandles;
        while (++_slot < handles.HighWater)
        {
            if (handles.IsLive(_slot))
            {
                Current = new Character(_world, _slot, handles.GenerationOf(_slot));
                return true;
            }
        }

        return false;
    }
}
