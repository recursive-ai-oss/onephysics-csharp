using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// What a client can see: a set of axis-aligned boxes and a set of named bodies. A dynamic or kinematic body is <b>inside</b> when its position (the centre of mass, inclusive of the box faces) is in a box, or when it is
/// named; a ragdoll is inside whole when one of its parts is. Static bodies are never inside because of a box (a level's ground has its origin anywhere); they reach a client as proxies when something inside touches them,
/// or when they are named.
/// </summary>
/// <remarks>A region has a dimension, fixed by its first box: 2 or 3. Boxes are in world units, relative to the world origin of the world they are used with, and must be finite, ordered (minimum not above maximum) and within ±2^21.</remarks>
public sealed class InterestRegion
{
    private readonly List<double[]> _min = new();
    private readonly List<double[]> _max = new();
    private readonly List<BodyHandle> _bodies = new();
    private int _dimension;

    /// <summary>0 while the region has no box, otherwise 2 or 3.</summary>
    public int Dimension => _dimension;

    /// <summary>Number of boxes.</summary>
    public int BoxCount => _min.Count;

    /// <summary>Number of named bodies.</summary>
    public int BodyCount => _bodies.Count;

    /// <summary>Adds a 3D box.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A lane is not finite, the minimum is above the maximum, or a lane is beyond ±2^21.</exception>
    /// <exception cref="InvalidOperationException">The region already holds 2D boxes.</exception>
    public InterestRegion AddBox(Vector3D minimum, Vector3D maximum) => Add(3, new[] { minimum.X, minimum.Y, minimum.Z }, new[] { maximum.X, maximum.Y, maximum.Z });

    /// <summary>Adds a 2D box.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A lane is not finite, the minimum is above the maximum, or a lane is beyond ±2^21.</exception>
    /// <exception cref="InvalidOperationException">The region already holds 3D boxes.</exception>
    public InterestRegion AddBox(Vector2D minimum, Vector2D maximum) => Add(2, new[] { minimum.X, minimum.Y }, new[] { maximum.X, maximum.Y });

    /// <summary>
    /// Names a body: it is inside whatever its position (and a static body named here is sent as an ordinary inside body). A handle that is not alive when a message is written, or that belongs to another world, is ignored.
    /// </summary>
    public InterestRegion AddBody(BodyHandle body)
    {
        _bodies.Add(body);
        return this;
    }

    /// <summary>Removes every box and every named body (the dimension is free again).</summary>
    public void Clear() { _min.Clear(); _max.Clear(); _bodies.Clear(); _dimension = 0; }

    private InterestRegion Add(int dimension, double[] min, double[] max)
    {
        if (_dimension != 0 && _dimension != dimension) { throw new InvalidOperationException("This region holds " + _dimension.ToString(CultureInfo.InvariantCulture) + "D boxes; a " + dimension.ToString(CultureInfo.InvariantCulture) + "D box cannot be added."); }
        for (int k = 0; k < dimension; k++)
        {
            if (!double.IsFinite(min[k]) || !double.IsFinite(max[k])) { throw new ArgumentOutOfRangeException(nameof(min), "Box lane " + k.ToString(CultureInfo.InvariantCulture) + " is not finite."); }
            if (min[k] > max[k]) { throw new ArgumentOutOfRangeException(nameof(min), "Box lane " + k.ToString(CultureInfo.InvariantCulture) + ": the minimum " + min[k].ToString("R", CultureInfo.InvariantCulture) + " is above the maximum " + max[k].ToString("R", CultureInfo.InvariantCulture) + "."); }
            if (Math.Abs(min[k]) > 2097152.0 || Math.Abs(max[k]) > 2097152.0) { throw new ArgumentOutOfRangeException(nameof(min), "Box lane " + k.ToString(CultureInfo.InvariantCulture) + " is beyond the ±2^21 range."); }
        }

        _dimension = dimension;
        _min.Add(min);
        _max.Add(max);
        return this;
    }

    internal bool Contains(ReadOnlySpan<double> p)
    {
        for (int i = 0; i < _min.Count; i++)
        {
            var lo = _min[i]; var hi = _max[i];
            bool inside = true;
            for (int k = 0; k < lo.Length; k++) { if (p[k] < lo[k] || p[k] > hi[k]) { inside = false; break; } }
            if (inside) { return true; }
        }

        return false;
    }

    internal IReadOnlyList<BodyHandle> Bodies => _bodies;
}

/// <summary>Options of an <see cref="InterestWriter"/>.</summary>
public sealed class InterestOptions
{
    /// <summary>
    /// True (the default) sends every body outside the region that a joint or a live contact connects to an inside dynamic body as a kinematic proxy (a static one stays static), so the inside bodies have something to stand on
    /// and hang from. False sends inside bodies only: joints to outside bodies and contacts with them are left out, and the client's bodies fall through anything that is not inside.
    /// </summary>
    public bool Proxies { get; set; } = true;
}

/// <summary>Which kind of message <see cref="InterestClient.Apply"/> took.</summary>
public enum InterestMessageKind
{
    /// <summary>The first message of a stream, or the first after <see cref="InterestWriter.Reset"/>: everything inside and every proxy arrives whole; the client forgets what it held before.</summary>
    Snapshot = 0,

    /// <summary>A change since the previous message: entities that entered or left, joints that appeared or went, and the state of every body that changed.</summary>
    Delta = 1,
}

/// <summary>What one applied message changed in the client's world.</summary>
public readonly struct InterestApplyResult
{
    internal InterestApplyResult(InterestMessageKind kind, long sequence, int entered, int left, int updated, int jointsAdded, int jointsRemoved)
    {
        Kind = kind; Sequence = sequence; Entered = entered; Left = left; Updated = updated; JointsAdded = jointsAdded; JointsRemoved = jointsRemoved;
    }

    /// <summary>Snapshot or delta.</summary>
    public InterestMessageKind Kind { get; }

    /// <summary>The message's sequence number: 0 for the first message of a stream, then 1, 2, ...</summary>
    public long Sequence { get; }

    /// <summary>Bodies created in the client's world.</summary>
    public int Entered { get; }

    /// <summary>Bodies destroyed in the client's world.</summary>
    public int Left { get; }

    /// <summary>Existing bodies whose state was overwritten.</summary>
    public int Updated { get; }

    /// <summary>Joints created that were not part of an entering packet.</summary>
    public int JointsAdded { get; }

    /// <summary>Joints destroyed while both their bodies stayed.</summary>
    public int JointsRemoved { get; }
}

/// <summary>Bytes that are not a valid interest message, or a message that does not follow the one before it.</summary>
public sealed class InterestStreamException : PhysicsException
{
    /// <summary>The field that was wrong, for example <c>states[3].rotation</c>.</summary>
    public string Field { get; }

    /// <summary>Byte offset where the problem was found (0 when it is about the stream rather than the bytes).</summary>
    public int Offset { get; }

    /// <summary>Creates the exception for a problem with <paramref name="field"/> at <paramref name="offset"/>.</summary>
    public InterestStreamException(string field, int offset, string message)
        : base("Interest message: " + field + ": " + message + " (at byte " + offset.ToString(CultureInfo.InvariantCulture) + ")")
    {
        Field = field;
        Offset = offset;
    }
}
