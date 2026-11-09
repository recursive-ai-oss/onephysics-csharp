using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Why a signed distance field could not be built.</summary>
public enum SdfBuildError
{
    /// <summary>A dimension, cell size, extent or sample count is outside the accepted range.</summary>
    OutOfRange,
    /// <summary>A coordinate, sample or parameter is NaN or infinite.</summary>
    NonFinite,
    /// <summary>The mesh is not a closed, consistently oriented surface (an edge is not shared by exactly two triangles walking it in opposite directions).</summary>
    NotClosed,
    /// <summary>A triangle has no area, or an index names no vertex.</summary>
    Degenerate,
    /// <summary>The mesh is empty.</summary>
    Empty,
}

/// <summary>Thrown when a signed distance field is built from input it cannot accept. The message names the input and what was expected.</summary>
public sealed class SdfBuildException : ArgumentException
{
    /// <summary>Creates the exception.</summary>
    public SdfBuildException(SdfBuildError error, string message, string? paramName = null) : base(message, paramName) { Error = error; }

    /// <summary>The kind of failure.</summary>
    public SdfBuildError Error { get; }
}

/// <summary>
/// An immutable signed distance field sampled on a regular grid: negative inside, positive outside, in the shape's own frame.
/// Between samples the field is trilinear, so its gradient is the gradient of that interpolant (piecewise linear in each
/// coordinate, continuous in value, not in direction across cell faces). Outside the grid box the field is the distance to the box plus the
/// clamped interior value floored at zero, so the solid ends at the box. Samples are x-fastest, then y, then z.
/// </summary>
[Experimental("OPHYS0001")]
public sealed partial class SdfGrid3D
{
    /// <summary>Largest sample count, 2^22; each dimension is 2..512.</summary>
    public const int MaxSamples = 1 << 22;
    private readonly float[] _values;
    private readonly float _lipschitz;

    /// <summary>Copies samples. Dimensions are 2..512 each with at most 2^22 samples in total, the cell size is finite and at least 1e-4, the origin finite, the total extent within 2^20 metres, and every sample finite. Throws SdfBuildException for any other input.</summary>
    public SdfGrid3D(int sizeX, int sizeY, int sizeZ, ReadOnlySpan<float> samples, float cellSize, Vector3F origin = default)
    {
        Validate(sizeX, sizeY, sizeZ, cellSize, origin);
        if (samples.Length != (long)sizeX * sizeY * sizeZ) throw new SdfBuildException(SdfBuildError.OutOfRange, "Sample count " + samples.Length + " must equal sizeX * sizeY * sizeZ = " + (long)sizeX * sizeY * sizeZ + ".", nameof(samples));
        for (int i = 0; i < samples.Length; i++) if (!float.IsFinite(samples[i])) throw new SdfBuildException(SdfBuildError.NonFinite, "Sample " + i + " is not finite.", nameof(samples));
        SizeX = sizeX; SizeY = sizeY; SizeZ = sizeZ; CellSize = cellSize; Origin = origin; _values = samples.ToArray();
        _lipschitz = MeasureLipschitz();
    }

    internal static void Validate(int x, int y, int z, float cell, Vector3F origin)
    {
        if (x < 2 || x > 512 || y < 2 || y > 512 || z < 2 || z > 512 || (long)x * y * z > MaxSamples) throw new SdfBuildException(SdfBuildError.OutOfRange, "Grid dimensions " + x + " x " + y + " x " + z + " must each be 2..512 with at most " + MaxSamples + " samples.");
        if (!float.IsFinite(cell) || cell < 1e-4f || (double)Math.Max(x, Math.Max(y, z)) * cell > 1048576) throw new SdfBuildException(SdfBuildError.OutOfRange, "Cell size " + cell + " must be finite, at least 1e-4, with total extent at most 2^20 metres.", nameof(cell));
        if (!float.IsFinite(origin.X) || !float.IsFinite(origin.Y) || !float.IsFinite(origin.Z) || Math.Abs(origin.X) > 1048576 || Math.Abs(origin.Y) > 1048576 || Math.Abs(origin.Z) > 1048576) throw new SdfBuildException(SdfBuildError.NonFinite, "Origin must be finite and within 2^20 metres.", nameof(origin));
    }

    /// <summary>Samples along X.</summary>
    public int SizeX { get; }
    /// <summary>Samples along Y.</summary>
    public int SizeY { get; }
    /// <summary>Samples along Z.</summary>
    public int SizeZ { get; }
    /// <summary>Distance between neighbouring samples, in metres.</summary>
    public float CellSize { get; }
    /// <summary>Position of sample (0,0,0) in the shape's frame.</summary>
    public Vector3F Origin { get; }
    /// <summary>The copied samples, x-fastest, then y, then z.</summary>
    public ReadOnlySpan<float> Samples => _values;
    /// <summary>The box the samples cover, in the shape's frame.</summary>
    public Bounds3F Bounds => new(Origin, Origin + new Vector3F((SizeX - 1) * CellSize, (SizeY - 1) * CellSize, (SizeZ - 1) * CellSize));

    /// <summary>The largest gradient magnitude of the interpolant (never below 1, the slope outside the box). Sphere tracing divides each step by it, so a step can never cross the zero set.</summary>
    public float Lipschitz => _lipschitz;

    private float At(int x, int y, int z) => _values[(z * SizeY + y) * SizeX + x];

    private float MeasureLipschitz()
    {
        // A trilinear interpolant's partial derivative is bilinear in the other two coordinates, so its extremes sit at cell corners
        // and equal the largest one-sided difference of a corner with its neighbours.
        double best = 1.0; double inv = 1.0 / CellSize;
        for (int z = 0; z < SizeZ; z++) for (int y = 0; y < SizeY; y++) for (int x = 0; x < SizeX; x++)
        {
            double dx = x + 1 < SizeX ? (double)At(x + 1, y, z) - At(x, y, z) : (x > 0 ? (double)At(x, y, z) - At(x - 1, y, z) : 0);
            double dy = y + 1 < SizeY ? (double)At(x, y + 1, z) - At(x, y, z) : (y > 0 ? (double)At(x, y, z) - At(x, y - 1, z) : 0);
            double dz = z + 1 < SizeZ ? (double)At(x, y, z + 1) - At(x, y, z) : (z > 0 ? (double)At(x, y, z) - At(x, y, z - 1) : 0);
            double g = Math.Sqrt(dx * dx + dy * dy + dz * dz) * inv;
            if (g > best) best = g;
        }
        // The gradient of the interpolant is the (bilinear) blend of the four edge differences along each axis; their norm is bounded by
        // the sum of the per-axis maxima, which the corner scan above underestimates by at most sqrt(3). Use the safe side.
        return (float)(best * 1.7320508075688772);
    }

    private void Cell(Vector3F p, out int ix, out int iy, out int iz, out float tx, out float ty, out float tz)
    {
        float inv = 1f / CellSize;
        float ux = (p.X - Origin.X) * inv, uy = (p.Y - Origin.Y) * inv, uz = (p.Z - Origin.Z) * inv;
        ix = Math.Clamp((int)MathF.Floor(ux), 0, SizeX - 2); iy = Math.Clamp((int)MathF.Floor(uy), 0, SizeY - 2); iz = Math.Clamp((int)MathF.Floor(uz), 0, SizeZ - 2);
        tx = Math.Clamp(ux - ix, 0f, 1f); ty = Math.Clamp(uy - iy, 0f, 1f); tz = Math.Clamp(uz - iz, 0f, 1f);
    }

    private Vector3F Clamped(Vector3F p)
    {
        var b = Bounds;
        return new Vector3F(Math.Clamp(p.X, b.Min.X, b.Max.X), Math.Clamp(p.Y, b.Min.Y, b.Max.Y), Math.Clamp(p.Z, b.Min.Z, b.Max.Z));
    }

    private float Interior(Vector3F p)
    {
        Cell(p, out int ix, out int iy, out int iz, out float tx, out float ty, out float tz);
        float c000 = At(ix, iy, iz), c100 = At(ix + 1, iy, iz), c010 = At(ix, iy + 1, iz), c110 = At(ix + 1, iy + 1, iz);
        float c001 = At(ix, iy, iz + 1), c101 = At(ix + 1, iy, iz + 1), c011 = At(ix, iy + 1, iz + 1), c111 = At(ix + 1, iy + 1, iz + 1);
        float x00 = c000 + (c100 - c000) * tx, x10 = c010 + (c110 - c010) * tx, x01 = c001 + (c101 - c001) * tx, x11 = c011 + (c111 - c011) * tx;
        float y0 = x00 + (x10 - x00) * ty, y1 = x01 + (x11 - x01) * ty;
        return y0 + (y1 - y0) * tz;
    }

    /// <summary>The field at a point in the shape's frame. Trilinear inside the grid box; outside it, the distance to the box plus the clamped value floored at zero. Throws ArgumentOutOfRangeException for a non-finite point.</summary>
    public float Value(Vector3F p)
    {
        CheckPoint(p);
        var q = Clamped(p);
        float inside = Interior(q);
        var d = p - q; float outside = MathF.Sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z);
        return outside == 0f ? inside : (inside > 0f ? inside : 0f) + outside;
    }

    /// <summary>The unit gradient of the interpolant at a point: the direction of steepest increase, away from the solid. Outside the box it points away from the box. A point with a vanishing gradient reports +Y. Throws ArgumentOutOfRangeException for a non-finite point.</summary>
    public Vector3F Gradient(Vector3F p)
    {
        CheckPoint(p);
        var q = Clamped(p); var d = p - q; float outside = MathF.Sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z);
        if (outside > 0f) return d * (1f / outside);
        Cell(q, out int ix, out int iy, out int iz, out float tx, out float ty, out float tz);
        float c000 = At(ix, iy, iz), c100 = At(ix + 1, iy, iz), c010 = At(ix, iy + 1, iz), c110 = At(ix + 1, iy + 1, iz);
        float c001 = At(ix, iy, iz + 1), c101 = At(ix + 1, iy, iz + 1), c011 = At(ix, iy + 1, iz + 1), c111 = At(ix + 1, iy + 1, iz + 1);
        float gx = ((c100 - c000) * (1 - ty) * (1 - tz) + (c110 - c010) * ty * (1 - tz) + (c101 - c001) * (1 - ty) * tz + (c111 - c011) * ty * tz);
        float gy = ((c010 - c000) * (1 - tx) * (1 - tz) + (c110 - c100) * tx * (1 - tz) + (c011 - c001) * (1 - tx) * tz + (c111 - c101) * tx * tz);
        float gz = ((c001 - c000) * (1 - tx) * (1 - ty) + (c101 - c100) * tx * (1 - ty) + (c011 - c010) * (1 - tx) * ty + (c111 - c110) * tx * ty);
        float len = MathF.Sqrt(gx * gx + gy * gy + gz * gz);
        return len > 1e-12f ? new Vector3F(gx / len, gy / len, gz / len) : new Vector3F(0, 1, 0);
    }

    private static void CheckPoint(Vector3F p)
    {
        if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z)) throw new ArgumentOutOfRangeException(nameof(p), p, "Point must be finite.");
    }

    /// <summary>
    /// Sphere tracing (Hart 1996) of a ray against the zero set of the interpolant. The direction is normalised and t is a distance in metres, in [0, maxT]. Each step is the field value divided by
    /// <see cref="Lipschitz"/>, which cannot cross the surface; the loop ends at the surface within 1e-4 of a cell, after 2048 steps, or past maxT. Returns false for a miss. A ray that starts inside a solid hits at t = 0 with <paramref name="startedInside"/> true.
    /// The normal is the interpolant's gradient at the hit. Throws ArgumentOutOfRangeException for a non-finite or zero direction or a negative or non-finite maxT.
    /// </summary>
    public bool Trace(Vector3F origin, Vector3F direction, float maxT, out float t, out Vector3F normal, out bool startedInside)
    {
        CheckPoint(origin); CheckPoint(direction);
        if (!(maxT >= 0f) || !float.IsFinite(maxT)) throw new ArgumentOutOfRangeException(nameof(maxT), maxT, "maxT must be finite and not negative.");
        float len = MathF.Sqrt(direction.X * direction.X + direction.Y * direction.Y + direction.Z * direction.Z);
        if (!(len > 0f)) throw new ArgumentOutOfRangeException(nameof(direction), direction, "Direction must be nonzero.");
        var d = direction * (1f / len); t = 0; normal = default; startedInside = false;
        float v0 = Value(origin);
        if (v0 <= 0f) { startedInside = true; normal = Gradient(origin); return true; }
        float tol = 1e-4f * CellSize, k = 1f / Math.Max(1f, _lipschitz), at = 0f;
        for (int i = 0; i < 2048; i++)
        {
            var p = origin + d * at; float v = Value(p);
            if (v <= tol) { t = at; normal = Gradient(p); return true; }
            at += Math.Max(v * k, tol * 0.25f);
            if (at > maxT) return false;
        }
        return false;
    }
}
