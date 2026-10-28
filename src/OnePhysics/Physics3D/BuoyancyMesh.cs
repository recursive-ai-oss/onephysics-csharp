using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A closed convex polyhedron with outward, counter-clockwise faces, used to cut a shape by a fluid surface exactly.
/// Boxes and hulls are exact; round shapes (cylinders, capsules, tapered capsules) are tessellated into rings of <see cref="Segments"/> sides, with the ring radii scaled so each
/// ring's area equals the circle's, which keeps the volume error below about 1 % for the shapes' usual proportions (see docs/world3d.md, buoyancy).
/// </summary>
internal sealed class BuoyancyMesh
{
    public const int Segments = 24;
    private const int ArcRings = 8;

    public Vector3F[] Vertices = Array.Empty<Vector3F>();
    public int[] FaceStart = Array.Empty<int>();   // face f uses FaceIndex[FaceStart[f] .. FaceStart[f + 1])
    public int[] FaceIndex = Array.Empty<int>();
    public int FaceCount => FaceStart.Length - 1;

    /// <summary>Builds the polyhedron for a convex shape; null for a sphere, which is handled analytically.</summary>
    public static BuoyancyMesh? For(in ConvexShape3D shape)
    {
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                return null;
            case ConvexShapeKind3D.Box:
                {
                    float r = shape.ConvexRadius;
                    var h = shape.CoreSize + new Vector3F(r, r, r);
                    return Box(h);
                }

            case ConvexShapeKind3D.Cylinder:
                {
                    float r = shape.CoreSize.X + shape.ConvexRadius, hh = shape.CoreSize.Y + shape.ConvexRadius;
                    return Revolution([(0f, hh), (r, hh), (r, -hh), (0f, -hh)]);
                }

            case ConvexShapeKind3D.Capsule:
                return Tapered(shape.CoreSize.Y, shape.ConvexRadius, shape.ConvexRadius);
            case ConvexShapeKind3D.TaperedCapsule:
                return Tapered(shape.CoreSize.Z, shape.CoreSize.X, shape.CoreSize.Y);
            case ConvexShapeKind3D.Hull:
                return Hull(shape.Hull!);
            default:
                return null;
        }
    }

    private static BuoyancyMesh Box(Vector3F h)
    {
        var m = new BuoyancyMesh
        {
            Vertices =
            [
                new(-h.X, -h.Y, -h.Z), new(h.X, -h.Y, -h.Z), new(h.X, h.Y, -h.Z), new(-h.X, h.Y, -h.Z),
                new(-h.X, -h.Y, h.Z), new(h.X, -h.Y, h.Z), new(h.X, h.Y, h.Z), new(-h.X, h.Y, h.Z),
            ],
            FaceStart = [0, 4, 8, 12, 16, 20, 24],
            FaceIndex =
            [
                0, 3, 2, 1,   // -z
                4, 5, 6, 7,   // +z
                0, 4, 7, 3,   // -x
                1, 2, 6, 5,   // +x
                0, 1, 5, 4,   // -y
                3, 7, 6, 2,   // +y
            ],
        };
        return m;
    }

    private static BuoyancyMesh Hull(ConvexHull3D hull)
    {
        var faces = hull.Faces;
        var edges = hull.Edges;
        var start = new int[faces.Length + 1];
        int total = 0;
        for (int f = 0; f < faces.Length; f++) { start[f] = total; total += faces[f].EdgeCount; }
        start[faces.Length] = total;
        var idx = new int[total];
        for (int f = 0; f < faces.Length; f++)
        {
            int e = faces[f].FirstEdge;
            for (int k = 0; k < faces[f].EdgeCount; k++)
            {
                idx[start[f] + k] = edges[e].Origin;
                e = edges[e].Next;
            }
        }

        return new BuoyancyMesh { Vertices = hull.Vertices.ToArray(), FaceStart = start, FaceIndex = idx };
    }

    /// <summary>The hull of two spheres on the y axis (a capsule when the radii agree): arcs of latitude rings joined by the common tangent cone.</summary>
    private static BuoyancyMesh Tapered(float halfHeight, float topRadius, float bottomRadius)
    {
        float s = halfHeight > 0f ? (bottomRadius - topRadius) / (2f * halfHeight) : 0f;
        if (s > 0.999f) { s = 0.999f; }
        if (s < -0.999f) { s = -0.999f; }
        float cb = (float)Math.Sqrt(1.0 - (double)s * s);     // cos of the tangent cone's elevation
        float beta = (float)Math64.Atan2(s, cb);
        var profile = new (float R, float Y)[2 * ArcRings + 2];
        int n = 0;
        profile[n++] = (0f, halfHeight + topRadius);
        for (int j = 1; j <= ArcRings; j++)
        {
            // Top cap: elevation from the pole (90 degrees) down to the cone's tangent elevation.
            float t = (float)j / ArcRings;
            float el = 1.5707964f + (beta - 1.5707964f) * t;
            profile[n++] = (topRadius * Math32.Cos(el), halfHeight + topRadius * Math32.Sin(el));
        }

        for (int j = 0; j < ArcRings; j++)
        {
            // Bottom cap: from the tangent elevation down to the south pole (-90 degrees).
            float t = (float)j / ArcRings;
            float el = beta + (-1.5707964f - beta) * t;
            profile[n++] = (bottomRadius * Math32.Cos(el), -halfHeight + bottomRadius * Math32.Sin(el));
        }

        profile[n++] = (0f, -halfHeight - bottomRadius);
        return Revolution(profile.AsSpan(0, n));
    }

    /// <summary>A surface of revolution about y from a profile that runs from the top pole to the bottom pole (ring radius, height); a profile point with radius 0 is a single vertex.</summary>
    private static BuoyancyMesh Revolution(ReadOnlySpan<(float R, float Y)> profile)
    {
        int N = Segments;
        float step = 6.2831855f / N;
        float areaScale = (float)Math.Sqrt(6.2831855 / (N * (double)Math32.Sin(step)));
        var verts = new System.Collections.Generic.List<Vector3F>();
        var ringStart = new int[profile.Length];
        for (int i = 0; i < profile.Length; i++)
        {
            ringStart[i] = verts.Count;
            if (profile[i].R <= 0f) { verts.Add(new Vector3F(0f, profile[i].Y, 0f)); continue; }
            float r = profile[i].R * areaScale;
            for (int k = 0; k < N; k++) { verts.Add(new Vector3F(r * Math32.Cos(k * step), profile[i].Y, r * Math32.Sin(k * step))); }
        }

        var idx = new System.Collections.Generic.List<int>();
        var start = new System.Collections.Generic.List<int>();
        for (int i = 0; i + 1 < profile.Length; i++)
        {
            bool poleU = profile[i].R <= 0f, poleL = profile[i + 1].R <= 0f;
            for (int k = 0; k < N; k++)
            {
                int k1 = (k + 1) % N;
                if (poleU && poleL) { continue; }
                start.Add(idx.Count);
                if (poleU) { idx.Add(ringStart[i]); idx.Add(ringStart[i + 1] + k1); idx.Add(ringStart[i + 1] + k); }
                else if (poleL) { idx.Add(ringStart[i] + k); idx.Add(ringStart[i] + k1); idx.Add(ringStart[i + 1]); }
                else { idx.Add(ringStart[i] + k); idx.Add(ringStart[i] + k1); idx.Add(ringStart[i + 1] + k1); idx.Add(ringStart[i + 1] + k); }
            }
        }

        start.Add(idx.Count);
        return new BuoyancyMesh { Vertices = verts.ToArray(), FaceStart = start.ToArray(), FaceIndex = idx.ToArray() };
    }
}

/// <summary>What <see cref="BuoyancyCutter"/> measured: the volume below the surface, its centroid, and the area it presents to a flow.</summary>
internal struct CutResult
{
    public float Volume;
    public Vector3F Centroid;
    public float ProjectedArea;
}

/// <summary>Cuts convex polyhedra by a plane and measures what is below it.</summary>
/// <remarks>
/// Method (Ericson, Real-Time Collision Detection, ch. 8 on clipping a polygon against a plane; Catto, "Exact buoyancy for polyhedra", Game Programming Gems 6): clip each face to the
/// half-space below the surface, then add up signed tetrahedra from one apex to the clipped faces' fans (divergence theorem). When the apex lies on the surface plane the cap that closes the cut
/// contributes nothing, so it never has to be built. The projected area along a direction is half the sum of |area vector · direction| over the closed surface, cap included, and the cap's area vector
/// is minus the sum of the others.
/// </remarks>
internal sealed class BuoyancyCutter
{
    private Vector3F[] _pos = new Vector3F[64];
    private float[] _dist = new float[64];

    /// <summary>
    /// Measures the part of <paramref name="mesh"/> below the plane. Distances are signed, positive above the surface: <c>distance(p) = dot(normal, p) + planeOffset</c> in the frame the mesh pose is given in.
    /// </summary>
    public bool Cut(BuoyancyMesh mesh, in RigidTransformF pose, Vector3F normal, float planeOffset, Vector3F flowDirection, out CutResult result)
    {
        result = default;
        var verts = mesh.Vertices;
        if (_pos.Length < verts.Length)
        {
            int cap = Math.Max(verts.Length, _pos.Length * 2);
            _pos = new Vector3F[cap];
            _dist = new float[cap];
        }

        int below = 0;
        for (int i = 0; i < verts.Length; i++)
        {
            var p = pose.Rotation.Rotate(verts[i]) + pose.Translation;
            _pos[i] = p;
            float d = normal.X * p.X + normal.Y * p.Y + normal.Z * p.Z + planeOffset;
            _dist[i] = d;
            if (d <= 0f) { below++; }
        }

        if (below == 0) { return false; }
        bool whole = below == verts.Length;
        // The apex: on the surface when cut, the shape origin when whole (any point near the shape keeps the tetrahedra small).
        Vector3F apex = whole ? pose.Translation : new Vector3F(-planeOffset * normal.X, -planeOffset * normal.Y, -planeOffset * normal.Z);

        float vol6 = 0f, cx = 0f, cy = 0f, cz = 0f;
        float ax = 0f, ay = 0f, az = 0f, absSum = 0f;
        var start = mesh.FaceStart;
        var index = mesh.FaceIndex;
        for (int f = 0; f + 1 < start.Length; f++)
        {
            int first = start[f], last = start[f + 1];
            int prev = index[last - 1];
            bool prevIn = _dist[prev] <= 0f;
            int emitted = 0;
            Vector3F p0 = default, pl = default;
            float fx = 0f, fy = 0f, fz = 0f;   // this face's area vector (twice the area times the normal)
            for (int k = first; k < last; k++)
            {
                int cur = index[k];
                bool curIn = _dist[cur] <= 0f;
                if (curIn != prevIn)
                {
                    float t = _dist[prev] / (_dist[prev] - _dist[cur]);
                    Emit(_pos[prev] + (_pos[cur] - _pos[prev]) * t);
                }

                if (curIn) { Emit(_pos[cur]); }
                prev = cur;
                prevIn = curIn;
            }

            ax += fx; ay += fy; az += fz;
            absSum += Math.Abs(fx * flowDirection.X + fy * flowDirection.Y + fz * flowDirection.Z);

            void Emit(Vector3F q)
            {
                if (emitted == 0) { p0 = q; }
                else if (emitted >= 2)
                {
                    var a = p0 - apex; var b = pl - apex; var c = q - apex;
                    float v6 = a.X * (b.Y * c.Z - b.Z * c.Y) + a.Y * (b.Z * c.X - b.X * c.Z) + a.Z * (b.X * c.Y - b.Y * c.X);
                    vol6 += v6;
                    cx += v6 * (a.X + b.X + c.X); cy += v6 * (a.Y + b.Y + c.Y); cz += v6 * (a.Z + b.Z + c.Z);
                    var e1 = pl - p0; var e2 = q - p0;
                    fx += e1.Y * e2.Z - e1.Z * e2.Y; fy += e1.Z * e2.X - e1.X * e2.Z; fz += e1.X * e2.Y - e1.Y * e2.X;
                }

                pl = q;
                emitted++;
            }
        }

        float volume = vol6 / 6f;
        if (!(volume > 0f)) { return false; }
        float inv = 1f / (vol6 * 4f);   // centroid of the tetrahedra: sum(v6 * (a + b + c)) / (4 * sum(v6)) relative to the apex
        result.Volume = volume;
        result.Centroid = new Vector3F(apex.X + cx * inv, apex.Y + cy * inv, apex.Z + cz * inv);
        // The face sums hold twice the area vectors: half of the |.| sum, plus the cap's |-(sum)|, all halved again.
        float capAbs = Math.Abs(ax * flowDirection.X + ay * flowDirection.Y + az * flowDirection.Z);
        result.ProjectedArea = 0.25f * (absSum + capAbs);
        return true;
    }
}
