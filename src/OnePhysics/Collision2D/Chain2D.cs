using System;
using System.Runtime.Intrinsics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>A chain: a sequence of points whose consecutive pairs are segments, open or a closed loop, one- or two-sided.</summary>
public sealed class Chain2D
{
    private readonly Vec2[] _points;

    /// <summary>The chain's points (a copy).</summary>
    public ReadOnlySpan<Vec2> Points => _points;

    /// <summary>True when the last point connects to the first.</summary>
    public bool IsLoop { get; }

    /// <summary>True when segments collide from both sides.</summary>
    public bool IsTwoSided { get; }

    /// <summary>Number of segments: the point count for a loop, one less for an open chain.</summary>
    public int SegmentCount => IsLoop ? _points.Length : _points.Length - 1;

    /// <summary>
    /// Builds a chain. The solid side of a one-sided chain is to the right of the direction of travel, so a loop wound clockwise is solid
    /// on its inside and a loop wound counter-clockwise is solid on its outside.
    /// </summary>
    /// <param name="points">At least 2 points (3 for a loop), each coordinate finite and within ±2^20, consecutive points at least <see cref="Constants2D.MinShapeExtent"/> apart.</param>
    /// <param name="loop">Connect the last point to the first.</param>
    /// <param name="twoSided">Collide from both sides.</param>
    /// <exception cref="InvalidShapeException">Too few points, a coordinate that is not finite or out of range, or a zero-length segment.</exception>
    public Chain2D(ReadOnlySpan<Vec2> points, bool loop = false, bool twoSided = false)
    {
        int min = loop ? 3 : 2;
        if (points.Length < min)
        {
            throw new InvalidShapeException(ShapeError.TooFewPoints, "a " + (loop ? "loop" : "chain") + " needs at least " + min.ToString(CultureInfo.InvariantCulture) + " points, got " + points.Length.ToString(CultureInfo.InvariantCulture));
        }

        if (points.Length > (1 << 24))
        {
            throw new InvalidShapeException(ShapeError.TooManyPoints, "got " + points.Length.ToString(CultureInfo.InvariantCulture) + " points, the limit is 16777216");
        }

        _points = points.ToArray();
        for (int i = 0; i < _points.Length; i++)
        {
            Shape2D.CheckPoint(_points[i], "points[" + i.ToString(CultureInfo.InvariantCulture) + "]");
        }

        int segs = loop ? _points.Length : _points.Length - 1;
        for (int i = 0; i < segs; i++)
        {
            if (!((_points[(i + 1) % _points.Length] - _points[i]).Length >= Constants2D.MinShapeExtent))
            {
                throw new InvalidShapeException(ShapeError.Degenerate, "points[" + i.ToString(CultureInfo.InvariantCulture) + "] and the next point are closer than " + Constants2D.MinShapeExtent.ToString("R", CultureInfo.InvariantCulture));
            }
        }

        IsLoop = loop;
        IsTwoSided = twoSided;
    }

    /// <summary>
    /// The segment at <paramref name="index"/> as a shape, with its neighbours as ghost vertices. Use the index as the contact's sub-shape id.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is not in 0 to <see cref="SegmentCount"/> − 1.</exception>
    public Shape2D GetSegment(int index)
    {
        int n = _points.Length;
        if ((uint)index >= (uint)SegmentCount)
        {
            throw new ArgumentOutOfRangeException(nameof(index), index, "segment index must be in 0.." + (SegmentCount - 1).ToString(CultureInfo.InvariantCulture));
        }

        Vec2 p1 = _points[index];
        Vec2 p2 = _points[(index + 1) % n];
        Vec2? g1 = IsLoop ? _points[(index + n - 1) % n] : index > 0 ? _points[index - 1] : null;
        Vec2? g2 = IsLoop ? _points[(index + 2) % n] : index + 2 < n ? _points[index + 2] : null;
        return Shape2D.CreateChainSegment(g1, p1, p2, g2, IsTwoSided);
    }
}
