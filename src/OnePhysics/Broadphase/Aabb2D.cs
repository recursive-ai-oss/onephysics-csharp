using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>A 2D axis-aligned box in world space (double precision). Its <see cref="Area"/> is the perimeter, the 2D analogue of surface area in the SAH.</summary>
[Experimental("OPHYS0001")]
public readonly struct Aabb2D : IBox<Aabb2D>, IEquatable<Aabb2D>
{
    /// <summary>Creates a box from its bounds. No validation happens here; the tree validates on entry.</summary>
    public Aabb2D(double minX, double minY, double maxX, double maxY)
    {
        MinX = minX;
        MinY = minY;
        MaxX = maxX;
        MaxY = maxY;
    }

    /// <summary>Lower x bound.</summary>
    public double MinX { get; }

    /// <summary>Lower y bound.</summary>
    public double MinY { get; }

    /// <summary>Upper x bound.</summary>
    public double MaxX { get; }

    /// <summary>Upper y bound.</summary>
    public double MaxY { get; }

    /// <inheritdoc/>
    public static int Axes => 2;

    /// <inheritdoc/>
    public double Area => 2.0 * ((MaxX - MinX) + (MaxY - MinY));

    /// <inheritdoc/>
    public static Aabb2D Union(in Aabb2D a, in Aabb2D b) => new(
        a.MinX < b.MinX ? a.MinX : b.MinX,
        a.MinY < b.MinY ? a.MinY : b.MinY,
        a.MaxX > b.MaxX ? a.MaxX : b.MaxX,
        a.MaxY > b.MaxY ? a.MaxY : b.MaxY);

    /// <inheritdoc/>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    public static bool Overlaps(in Aabb2D a, in Aabb2D b) =>
        a.MinX <= b.MaxX && b.MinX <= a.MaxX && a.MinY <= b.MaxY && b.MinY <= a.MaxY;

    /// <inheritdoc/>
    public static bool Contains(in Aabb2D outer, in Aabb2D inner) =>
        outer.MinX <= inner.MinX && outer.MinY <= inner.MinY && inner.MaxX <= outer.MaxX && inner.MaxY <= outer.MaxY;

    /// <inheritdoc/>
    public static Aabb2D Read(StateReader reader)
    {
        ArgumentNullException.ThrowIfNull(reader);
        Span<double> v = stackalloc double[4];
        reader.ReadDoubles(v, true, "2D box bounds");
        return new Aabb2D(v[0], v[1], v[2], v[3]);
    }

    /// <inheritdoc/>
    public double Min(int axis) => axis == 0 ? MinX : MinY;

    /// <inheritdoc/>
    public double Max(int axis) => axis == 0 ? MaxX : MaxY;

    /// <inheritdoc/>
    public Aabb2D Fatten(double margin, ReadOnlySpan<double> displacement, double multiplier)
    {
        double minX = MinX - margin, minY = MinY - margin, maxX = MaxX + margin, maxY = MaxY + margin;
        double dx = displacement.Length > 0 ? displacement[0] * multiplier : 0.0;
        double dy = displacement.Length > 1 ? displacement[1] * multiplier : 0.0;
        // The stretch is a hint for fast bodies; it stops at the coordinate limit instead of refusing a body that is still inside it.
        const double L = BoxLimits.MaxCoordinate;
        if (dx < 0.0) { minX = Math.Min(minX, Math.Max(minX + dx, -L)); } else { maxX = Math.Max(maxX, Math.Min(maxX + dx, L)); }
        if (dy < 0.0) { minY = Math.Min(minY, Math.Max(minY + dy, -L)); } else { maxY = Math.Max(maxY, Math.Min(maxY + dy, L)); }
        return new Aabb2D(minX, minY, maxX, maxY);
    }

    /// <inheritdoc/>
    public Aabb2D Translated(ReadOnlySpan<double> offset)
    {
        const double L = BoxLimits.MaxCoordinate;
        return new Aabb2D(Math.Clamp(MinX - offset[0], -L, L), Math.Clamp(MinY - offset[1], -L, L), Math.Clamp(MaxX - offset[0], -L, L), Math.Clamp(MaxY - offset[1], -L, L));
    }

    /// <inheritdoc/>
    public bool IsValid(out string reason) =>
        BoxLimits.CheckAxis(MinX, MaxX, "x", out reason) && BoxLimits.CheckAxis(MinY, MaxY, "y", out reason);

    /// <inheritdoc/>
    public void Write(StateWriter writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        writer.WriteDouble(MinX);
        writer.WriteDouble(MinY);
        writer.WriteDouble(MaxX);
        writer.WriteDouble(MaxY);
    }

    /// <summary>True when all four bounds are bit-identical.</summary>
    public bool Equals(Aabb2D other) =>
        BitConverter.DoubleToInt64Bits(MinX) == BitConverter.DoubleToInt64Bits(other.MinX)
        && BitConverter.DoubleToInt64Bits(MinY) == BitConverter.DoubleToInt64Bits(other.MinY)
        && BitConverter.DoubleToInt64Bits(MaxX) == BitConverter.DoubleToInt64Bits(other.MaxX)
        && BitConverter.DoubleToInt64Bits(MaxY) == BitConverter.DoubleToInt64Bits(other.MaxY);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Aabb2D b && Equals(b);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        ulong h = 14695981039346656037UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MinX)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MinY)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MaxX)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MaxY)) * 1099511628211UL;
        return (int)(h ^ (h >> 32));
    }

    /// <summary>Bitwise equality.</summary>
    public static bool operator ==(Aabb2D a, Aabb2D b) => a.Equals(b);

    /// <summary>Bitwise inequality.</summary>
    public static bool operator !=(Aabb2D a, Aabb2D b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => FormattableString.Invariant($"[{MinX}, {MinY}]-[{MaxX}, {MaxY}]");
}
