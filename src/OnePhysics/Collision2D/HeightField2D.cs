using System;

using RecursiveAI.OnePhysics.LowLevel;
namespace RecursiveAI.OnePhysics;

/// <summary>An immutable sampled, one-sided terrain surface. Each non-hole column interval is a chain segment, facing upward with neighbouring samples as ghost vertices. Zero area means zero mass. Snapshots of an attached field store its segments.</summary>
public sealed class HeightField2D
{
    /// <summary>Maximum number of samples, 2^20.</summary>
    public const int MaxSamples = 1 << 20;
    private readonly float[] _heights;
    private readonly bool[] _holes;
    private readonly float _spacing;

    /// <summary>Creates terrain with sample i at (i * spacing, heights[i]). Input is copied. A true hole removes that interval, not either neighbouring sample.</summary>
    /// <param name="heights">Between 2 and 2^20 finite heights, in metres within ±2^20.</param>
    /// <param name="spacing">Positive spacing in metres, at least 2e-5; the full width must fit within 2^20.</param>
    /// <param name="holes">Empty, or one flag per interval. Default: no holes.</param>
    /// <exception cref="InvalidShapeException">A coordinate is invalid or a segment is too short.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The sample count, spacing or hole mask length is invalid.</exception>
    public HeightField2D(ReadOnlySpan<float> heights, float spacing = 1f, ReadOnlySpan<bool> holes = default)
    {
        if (heights.Length < 2 || heights.Length > MaxSamples) { throw new ArgumentOutOfRangeException(nameof(heights), heights.Length, "Height field needs 2..1048576 samples."); }
        if (!float.IsFinite(spacing) || spacing < Constants2D.MinShapeExtent || (double)spacing * (heights.Length - 1) > Constants2D.MaxShapeCoordinate)
        {
            throw new ArgumentOutOfRangeException(nameof(spacing), spacing, "Spacing must be finite, at least 2e-5 metres, with total width at most 1048576 metres.");
        }
        if (holes.Length != 0 && holes.Length != heights.Length - 1) { throw new ArgumentOutOfRangeException(nameof(holes), holes.Length, "Hole mask must be empty or contain one flag per interval."); }
        for (int i = 0; i < heights.Length; i++) { Shape2D.CheckCoordinate(heights[i], "heights[" + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + "]"); }
        _heights = heights.ToArray(); _holes = holes.ToArray(); _spacing = spacing;
        for (int i = 0; i < IntervalCount; i++) { if (!IsHole(i)) { _ = GetSegment(i); } }
    }

    /// <summary>Number of column intervals, holes included.</summary>
    public int IntervalCount => _heights.Length - 1;

    /// <summary>The copied heights in metres.</summary>
    public ReadOnlySpan<float> Heights => _heights;

    /// <summary>Returns whether interval i is missing. Throws ArgumentOutOfRangeException outside [0, IntervalCount).</summary>
    public bool IsHole(int index)
    {
        CheckIndex(index);
        return _holes.Length != 0 && _holes[index];
    }

    /// <summary>Returns a one-sided segment, from its right sample to its left sample so its outward normal faces up. Ghosts stop at holes. Throws ArgumentOutOfRangeException for an invalid index or ArgumentException for a hole.</summary>
    public Shape2D GetSegment(int index)
    {
        if (IsHole(index)) { throw new ArgumentException("Height-field interval is a hole.", nameof(index)); }
        Vec2? before = index + 1 < IntervalCount && !IsHole(index + 1) ? Point(index + 2) : null;
        Vec2? after = index > 0 && !IsHole(index - 1) ? Point(index - 1) : null;
        return Shape2D.CreateChainSegment(before, Point(index + 1), Point(index), after, false);
    }

    private Vec2 Point(int i) => new(i * _spacing, _heights[i]);
    private void CheckIndex(int index)
    {
        if ((uint)index >= (uint)IntervalCount) { throw new ArgumentOutOfRangeException(nameof(index), index, "Interval index must be in [0, IntervalCount)."); }
    }
}
