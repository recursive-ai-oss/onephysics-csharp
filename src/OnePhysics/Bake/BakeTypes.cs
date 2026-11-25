using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>A body, soft body or rod as the world numbered it when the clip was baked: slot and generation.</summary>
public readonly struct BakeId : IEquatable<BakeId>
{
    /// <summary>Creates an id.</summary>
    public BakeId(int slot, int generation) { Slot = slot; Generation = generation; }

    /// <summary>The slot, or -1 for "no body" (a sleep event has no second body).</summary>
    public int Slot { get; }

    /// <summary>The slot's generation when the thing was created; 0 for "no body".</summary>
    public int Generation { get; }

    /// <summary>True for "no body".</summary>
    public bool IsNone => Slot < 0;

    /// <inheritdoc />
    public bool Equals(BakeId other) => Slot == other.Slot && Generation == other.Generation;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is BakeId b && Equals(b);

    /// <inheritdoc />
    public override int GetHashCode() => unchecked(Slot * 397 ^ Generation);

    /// <summary>Equality.</summary>
    public static bool operator ==(BakeId a, BakeId b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(BakeId a, BakeId b) => !a.Equals(b);

    /// <inheritdoc />
    public override string ToString() => IsNone ? "none" : "#" + Slot.ToString(System.Globalization.CultureInfo.InvariantCulture) + " gen " + Generation.ToString(System.Globalization.CultureInfo.InvariantCulture);
}

/// <summary>A body's pose in a baked clip.</summary>
public readonly struct BakePose
{
    internal BakePose(Vector3D position, QuaternionF rotation, float cos, float sin) { Position = position; Rotation = rotation; Cos = cos; Sin = sin; }

    /// <summary>Position of the body origin, metres (z is 0 in a 2D clip).</summary>
    public Vector3D Position { get; }

    /// <summary>Orientation in a 3D clip (unit length when sampled between frames; at a frame it is the stored value). Identity in a 2D clip.</summary>
    public QuaternionF Rotation { get; }

    /// <summary>Cosine of the angle in a 2D clip; 1 in a 3D clip.</summary>
    public float Cos { get; }

    /// <summary>Sine of the angle in a 2D clip; 0 in a 3D clip.</summary>
    public float Sin { get; }

    /// <summary>The angle in radians in a 2D clip, in (-pi, pi] (from <see cref="Cos"/> and <see cref="Sin"/>); 0 in a 3D clip.</summary>
    public float Angle => (float)Math64.Atan2(Sin, Cos);
}

/// <summary>An event of the baked world: what happened in which step, to which bodies.</summary>
public readonly struct BakeEvent
{
    internal BakeEvent(int step, int frame, EventKind kind, BakeId a, BakeId b, float value) { Step = step; Frame = frame; Kind = kind; BodyA = a; BodyB = b; Value = value; }

    /// <summary>The step (1 is the first step after the bake started) the event belongs to.</summary>
    public int Step { get; }

    /// <summary>The first frame at or after the step.</summary>
    public int Frame { get; }

    /// <summary>What happened.</summary>
    public EventKind Kind { get; }

    /// <summary>The first body.</summary>
    public BakeId BodyA { get; }

    /// <summary>The second body, or none.</summary>
    public BakeId BodyB { get; }

    /// <summary>The event's number (the world's <see cref="PhysicsEvent.Value"/>).</summary>
    public float Value { get; }
}

/// <summary>A fracture, slice or bond break during the bake.</summary>
public sealed class BakeFracture
{
    internal BakeFracture(int step, int frame, FractureKind kind, BakeId source, float impulse, Vector3D point, BakeId[] fragments)
    {
        Step = step; Frame = frame; Kind = kind; Source = source; Impulse = impulse; Point = point; Fragments = fragments;
    }

    /// <summary>The step it happened in (1 is the first step after the bake started).</summary>
    public int Step { get; }

    /// <summary>The first frame at or after the step: the fragments' rigid tracks begin here (if the stride skipped their whole life, they have no track).</summary>
    public int Frame { get; }

    /// <summary>Why it broke.</summary>
    public FractureKind Kind { get; }

    /// <summary>The body that broke (its track ends before <see cref="Frame"/>).</summary>
    public BakeId Source { get; }

    /// <summary>The impact impulse, kg·m/s (0 for the other kinds).</summary>
    public float Impulse { get; }

    /// <summary>Where it happened (z is 0 in a 2D clip).</summary>
    public Vector3D Point { get; }

    /// <summary>The bodies that replaced it, in the world's order.</summary>
    public BakeId[] Fragments { get; }
}

/// <summary>The shape of one fracture fragment, stored once, in the fragment's body frame: a vertex list and, in 3D, triangles.</summary>
public sealed class BakeFragmentGeometry
{
    internal BakeFragmentGeometry(BakeId body, Vector3F[] vertices, int[] triangles) { Body = body; Vertices = vertices; Triangles = triangles; }

    /// <summary>The fragment body; its pose is in the rigid track with the same id.</summary>
    public BakeId Body { get; }

    /// <summary>The corner points in the body's frame (z is 0 in a 2D clip, where they are the polygon in order).</summary>
    public Vector3F[] Vertices { get; }

    /// <summary>Triangle indices into <see cref="Vertices"/>, three per triangle, outward-facing; empty in a 2D clip.</summary>
    public int[] Triangles { get; }
}
