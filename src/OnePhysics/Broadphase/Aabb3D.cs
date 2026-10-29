using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>A 3D axis-aligned box in world space (double precision). Its <see cref="Area"/> is the surface area, the measure of the surface area heuristic.</summary>
[Experimental("OPHYS0001")]
public readonly struct Aabb3D : IBox<Aabb3D>, IEquatable<Aabb3D>
{
    /// <summary>Creates a box from its bounds. No validation happens here; the tree validates on entry.</summary>
    public Aabb3D(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        MinX = minX;
        MinY = minY;
        MinZ = minZ;
        MaxX = maxX;
        MaxY = maxY;
        MaxZ = maxZ;
    }

    /// <summary>Lower x bound.</summary>
    public double MinX { get; }

    /// <summary>Lower y bound.</summary>
    public double MinY { get; }

    /// <summary>Lower z bound.</summary>
    public double MinZ { get; }

    /// <summary>Upper x bound.</summary>
    public double MaxX { get; }

    /// <summary>Upper y bound.</summary>
    public double MaxY { get; }

    /// <summary>Upper z bound.</summary>
    public double MaxZ { get; }

    /// <inheritdoc/>
    public static int Axes => 3;

    /// <inheritdoc/>
    public double Area
    {
        get
        {
            double x = MaxX - MinX, y = MaxY - MinY, z = MaxZ - MinZ;
            return 2.0 * (((x * y) + (y * z)) + (z * x));
        }
    }

    /// <inheritdoc/>
    public static Aabb3D Union(in Aabb3D a, in Aabb3D b) => new(
        a.MinX < b.MinX ? a.MinX : b.MinX,
        a.MinY < b.MinY ? a.MinY : b.MinY,
        a.MinZ < b.MinZ ? a.MinZ : b.MinZ,
        a.MaxX > b.MaxX ? a.MaxX : b.MaxX,
        a.MaxY > b.MaxY ? a.MaxY : b.MaxY,
        a.MaxZ > b.MaxZ ? a.MaxZ : b.MaxZ);

    /// <inheritdoc/>
    public static bool Overlaps(in Aabb3D a, in Aabb3D b) =>
        a.MinX <= b.MaxX && b.MinX <= a.MaxX && a.MinY <= b.MaxY && b.MinY <= a.MaxY && a.MinZ <= b.MaxZ && b.MinZ <= a.MaxZ;

    /// <inheritdoc/>
    public static bool Contains(in Aabb3D outer, in Aabb3D inner) =>
        outer.MinX <= inner.MinX && outer.MinY <= inner.MinY && outer.MinZ <= inner.MinZ
        && inner.MaxX <= outer.MaxX && inner.MaxY <= outer.MaxY && inner.MaxZ <= outer.MaxZ;

    /// <inheritdoc/>
    public static Aabb3D Read(StateReader reader)
    {
        ArgumentNullException.ThrowIfNull(reader);
        Span<double> v = stackalloc double[6];
        reader.ReadDoubles(v, true, "3D box bounds");
        return new Aabb3D(v[0], v[1], v[2], v[3], v[4], v[5]);
    }

    /// <inheritdoc/>
    public double Min(int axis) => axis == 0 ? MinX : axis == 1 ? MinY : MinZ;

    /// <inheritdoc/>
    public double Max(int axis) => axis == 0 ? MaxX : axis == 1 ? MaxY : MaxZ;

    /// <inheritdoc/>
    public Aabb3D Fatten(double margin, ReadOnlySpan<double> displacement, double multiplier)
    {
        double minX = MinX - margin, minY = MinY - margin, minZ = MinZ - margin;
        double maxX = MaxX + margin, maxY = MaxY + margin, maxZ = MaxZ + margin;
        double dx = displacement.Length > 0 ? displacement[0] * multiplier : 0.0;
        double dy = displacement.Length > 1 ? displacement[1] * multiplier : 0.0;
        double dz = displacement.Length > 2 ? displacement[2] * multiplier : 0.0;
        // The stretch is a hint for fast bodies; it stops at the coordinate limit instead of refusing a body that is still inside it.
        const double L = BoxLimits.MaxCoordinate;
        if (dx < 0.0) { minX = Math.Min(minX, Math.Max(minX + dx, -L)); } else { maxX = Math.Max(maxX, Math.Min(maxX + dx, L)); }
        if (dy < 0.0) { minY = Math.Min(minY, Math.Max(minY + dy, -L)); } else { maxY = Math.Max(maxY, Math.Min(maxY + dy, L)); }
        if (dz < 0.0) { minZ = Math.Min(minZ, Math.Max(minZ + dz, -L)); } else { maxZ = Math.Max(maxZ, Math.Min(maxZ + dz, L)); }
        return new Aabb3D(minX, minY, minZ, maxX, maxY, maxZ);
    }

    /// <inheritdoc/>
    public Aabb3D Translated(ReadOnlySpan<double> offset)
    {
        const double L = BoxLimits.MaxCoordinate;
        return new Aabb3D(
            Math.Clamp(MinX - offset[0], -L, L), Math.Clamp(MinY - offset[1], -L, L), Math.Clamp(MinZ - offset[2], -L, L),
            Math.Clamp(MaxX - offset[0], -L, L), Math.Clamp(MaxY - offset[1], -L, L), Math.Clamp(MaxZ - offset[2], -L, L));
    }

    /// <inheritdoc/>
    public bool IsValid(out string reason) =>
        BoxLimits.CheckAxis(MinX, MaxX, "x", out reason) && BoxLimits.CheckAxis(MinY, MaxY, "y", out reason) && BoxLimits.CheckAxis(MinZ, MaxZ, "z", out reason);

    /// <inheritdoc/>
    public void Write(StateWriter writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        writer.WriteDouble(MinX);
        writer.WriteDouble(MinY);
        writer.WriteDouble(MinZ);
        writer.WriteDouble(MaxX);
        writer.WriteDouble(MaxY);
        writer.WriteDouble(MaxZ);
    }

    /// <summary>True when all six bounds are bit-identical.</summary>
    public bool Equals(Aabb3D other) =>
        BitConverter.DoubleToInt64Bits(MinX) == BitConverter.DoubleToInt64Bits(other.MinX)
        && BitConverter.DoubleToInt64Bits(MinY) == BitConverter.DoubleToInt64Bits(other.MinY)
        && BitConverter.DoubleToInt64Bits(MinZ) == BitConverter.DoubleToInt64Bits(other.MinZ)
        && BitConverter.DoubleToInt64Bits(MaxX) == BitConverter.DoubleToInt64Bits(other.MaxX)
        && BitConverter.DoubleToInt64Bits(MaxY) == BitConverter.DoubleToInt64Bits(other.MaxY)
        && BitConverter.DoubleToInt64Bits(MaxZ) == BitConverter.DoubleToInt64Bits(other.MaxZ);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Aabb3D b && Equals(b);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        ulong h = 14695981039346656037UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MinX)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MinY)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MinZ)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MaxX)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MaxY)) * 1099511628211UL;
        h = (h ^ (ulong)BitConverter.DoubleToInt64Bits(MaxZ)) * 1099511628211UL;
        return (int)(h ^ (h >> 32));
    }

    /// <summary>Bitwise equality.</summary>
    public static bool operator ==(Aabb3D a, Aabb3D b) => a.Equals(b);

    /// <summary>Bitwise inequality.</summary>
    public static bool operator !=(Aabb3D a, Aabb3D b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => FormattableString.Invariant($"[{MinX}, {MinY}, {MinZ}]-[{MaxX}, {MaxY}, {MaxZ}]");
}
