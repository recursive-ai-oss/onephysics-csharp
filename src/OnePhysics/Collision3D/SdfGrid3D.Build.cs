using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class SdfGrid3D
{
    /// <summary>Samples a function of position over a box. The grid has ceil(extent / cellSize) + 1 samples per axis starting at <paramref name="min"/>. The function should return the signed distance (negative inside); it is called once per sample in x-fastest order, so a deterministic function gives a deterministic grid. Throws SdfBuildException for an empty, inverted or oversized box, a bad cell size or a non-finite sample.</summary>
    public static SdfGrid3D FromFunction(Vector3F min, Vector3F max, float cellSize, Func<Vector3F, float> field)
    {
        ArgumentNullException.ThrowIfNull(field);
        if (!float.IsFinite(cellSize) || cellSize < 1e-4f) throw new SdfBuildException(SdfBuildError.OutOfRange, "Cell size must be finite and at least 1e-4.", nameof(cellSize));
        if (!float.IsFinite(min.X + min.Y + min.Z + max.X + max.Y + max.Z) || !(max.X > min.X) || !(max.Y > min.Y) || !(max.Z > min.Z)) throw new SdfBuildException(SdfBuildError.OutOfRange, "Box must be finite with max above min on every axis.");
        int nx = Count(max.X - min.X, cellSize), ny = Count(max.Y - min.Y, cellSize), nz = Count(max.Z - min.Z, cellSize);
        Validate(nx, ny, nz, cellSize, min);
        var samples = new float[nx * ny * nz]; int i = 0;
        for (int z = 0; z < nz; z++) for (int y = 0; y < ny; y++) for (int x = 0; x < nx; x++)
            samples[i++] = field(new Vector3F(min.X + x * cellSize, min.Y + y * cellSize, min.Z + z * cellSize));
        return new SdfGrid3D(nx, ny, nz, samples, cellSize, min);
    }
    private static int Count(float extent, float cell) { double n = Math.Ceiling((double)extent / cell) + 1; return n > 100000 ? 100000 : (int)n; }

    /// <summary>The exact distance field of a sphere.</summary>
    public static Func<Vector3F, float> Sphere(Vector3F center, float radius) => p => (p - center).Length() - radius;

    /// <summary>The exact distance field of an axis-aligned box (Quilez' formula: outside the length of the positive part of the corner offset, inside the largest negative offset).</summary>
    public static Func<Vector3F, float> Box(Vector3F center, Vector3F halfExtents) => p =>
    {
        float qx = MathF.Abs(p.X - center.X) - halfExtents.X, qy = MathF.Abs(p.Y - center.Y) - halfExtents.Y, qz = MathF.Abs(p.Z - center.Z) - halfExtents.Z;
        float ox = Math.Max(qx, 0f), oy = Math.Max(qy, 0f), oz = Math.Max(qz, 0f);
        return MathF.Sqrt(ox * ox + oy * oy + oz * oz) + Math.Min(Math.Max(qx, Math.Max(qy, qz)), 0f);
    };

    /// <summary>The exact distance field of a capsule: the points within <paramref name="radius"/> of the segment a-b.</summary>
    public static Func<Vector3F, float> Capsule(Vector3F a, Vector3F b, float radius) => p =>
    {
        var ab = b - a; float len2 = ab.X * ab.X + ab.Y * ab.Y + ab.Z * ab.Z; var ap = p - a;
        float t = len2 > 0f ? Math.Clamp((ap.X * ab.X + ap.Y * ab.Y + ap.Z * ab.Z) / len2, 0f, 1f) : 0f;
        return (ap - ab * t).Length() - radius;
    };

    /// <summary>The half space below a plane through <paramref name="point"/> with the given unit normal (the solid is behind the normal).</summary>
    public static Func<Vector3F, float> HalfSpace(Vector3F point, Vector3F normal) => p => (p - point).X * normal.X + (p - point).Y * normal.Y + (p - point).Z * normal.Z;

    /// <summary>The union of two fields (the smaller value). The result is a distance bound outside both shapes, exact on the boundary.</summary>
    public static Func<Vector3F, float> Union(Func<Vector3F, float> a, Func<Vector3F, float> b) => p => Math.Min(a(p), b(p));

    /// <summary>The intersection of two fields (the larger value).</summary>
    public static Func<Vector3F, float> Intersection(Func<Vector3F, float> a, Func<Vector3F, float> b) => p => Math.Max(a(p), b(p));

    /// <summary>The first field with the second cut away (the larger of a and -b).</summary>
    public static Func<Vector3F, float> Difference(Func<Vector3F, float> a, Func<Vector3F, float> b) => p => Math.Max(a(p), -b(p));

    /// <summary>
    /// Mass properties of the solid (the region where the field is negative) at unit density, in the shape's frame, as volume, centroid and inertia about the centroid.
    /// Cells entirely inside count as whole cubes; cells the surface crosses are split into <paramref name="subdivisions"/>^3 sub-cubes whose centres are tested against the
    /// interpolant, each counting as a cube. Accumulation is in double in cell order, so the result is deterministic. Throws ArgumentOutOfRangeException for subdivisions outside 1..8 and InvalidOperationException for a field with no inside.
    /// </summary>
    public MassPropertiesF ComputeMass(int subdivisions = 4)
    {
        if (subdivisions < 1 || subdivisions > 8) throw new ArgumentOutOfRangeException(nameof(subdivisions), subdivisions, "Subdivisions must be in 1..8.");
        double h = CellSize, vol = 0, mx = 0, my = 0, mz = 0, sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
        void Cube(double cx, double cy, double cz, double side)
        {
            double v = side * side * side, own = v * side * side / 12.0;
            vol += v; mx += v * cx; my += v * cy; mz += v * cz;
            sxx += v * cx * cx + own; syy += v * cy * cy + own; szz += v * cz * cz + own; sxy += v * cx * cy; sxz += v * cx * cz; syz += v * cy * cz;
        }
        for (int z = 0; z < SizeZ - 1; z++) for (int y = 0; y < SizeY - 1; y++) for (int x = 0; x < SizeX - 1; x++)
        {
            float lo = float.MaxValue, hi = float.MinValue;
            for (int k = 0; k < 8; k++) { float v = At(x + (k & 1), y + ((k >> 1) & 1), z + (k >> 2)); if (v < lo) lo = v; if (v > hi) hi = v; }
            if (lo >= 0f) continue;
            double ox = Origin.X + x * h, oy = Origin.Y + y * h, oz = Origin.Z + z * h;
            if (hi < 0f) { Cube(ox + 0.5 * h, oy + 0.5 * h, oz + 0.5 * h, h); continue; }
            double sub = h / subdivisions;
            for (int k = 0; k < subdivisions; k++) for (int j = 0; j < subdivisions; j++) for (int i = 0; i < subdivisions; i++)
            {
                double px = ox + (i + 0.5) * sub, py = oy + (j + 0.5) * sub, pz = oz + (k + 0.5) * sub;
                if (Interior(new Vector3F((float)px, (float)py, (float)pz)) < 0f) Cube(px, py, pz, sub);
            }
        }
        if (!(vol > 0)) throw new InvalidOperationException("The field has no inside, so it has no mass properties.");
        double cx0 = mx / vol, cy0 = my / vol, cz0 = mz / vol;
        sxx -= vol * cx0 * cx0; syy -= vol * cy0 * cy0; szz -= vol * cz0 * cz0; sxy -= vol * cx0 * cy0; sxz -= vol * cx0 * cz0; syz -= vol * cy0 * cz0;
        double tr = sxx + syy + szz;
        var inertia = new Matrix3x3F((float)(tr - sxx), (float)-sxy, (float)-sxz, (float)-sxy, (float)(tr - syy), (float)-syz, (float)-sxz, (float)-syz, (float)(tr - szz));
        return new MassPropertiesF((float)vol, new Vector3F((float)cx0, (float)cy0, (float)cz0), inertia);
    }
}
