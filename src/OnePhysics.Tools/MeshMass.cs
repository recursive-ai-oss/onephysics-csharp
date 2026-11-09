using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.Tools;

/// <summary>
/// Mass properties of a closed triangle mesh: volume, centre of mass and inertia tensor, by the divergence theorem over the signed tetrahedra
/// (origin, a, b, c) of the outward-facing triangles (Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties", 1996; Eberly, "Polyhedral Mass Properties (Revisited)").
/// </summary>
public static class MeshMass
{
    /// <summary>Checks that the mesh is a closed, consistently wound surface: every directed edge appears once and its reverse appears once.</summary>
    /// <param name="vertices">Mesh vertices.</param>
    /// <param name="indices">Triangle corner indices, three per triangle.</param>
    /// <returns>True if the mesh is closed and manifold with consistent winding (the winding may still be inward; see <see cref="Compute"/>).</returns>
    /// <exception cref="GeometryToolException">The vertices or indices are invalid.</exception>
    public static bool IsClosed(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices)
    {
        MeshInput.Check(vertices, indices);
        return FindProblem(indices, out _, out _, out _) is null;
    }

    // Null when the surface is closed, manifold and consistently wound.
    private static GeometryToolError? FindProblem(ReadOnlySpan<int> indices, out int a, out int b, out int tri)
    {
        a = b = tri = -1;
        int n = indices.Length;
        var keys = new long[n];
        for (int t = 0; t < n; t += 3)
        {
            for (int k = 0; k < 3; k++)
            {
                int p = indices[t + k], q = indices[t + (k + 1) % 3];
                if (p == q) { a = p; b = q; tri = t / 3; return GeometryToolError.NonManifold; }
                keys[t + k] = ((long)p << 32) | (uint)q;
            }
        }

        Array.Sort(keys);
        for (int i = 1; i < n; i++)
        {
            if (keys[i] == keys[i - 1]) { a = (int)(keys[i] >> 32); b = (int)(keys[i] & 0xFFFFFFFF); return GeometryToolError.NonManifold; }
        }

        for (int i = 0; i < n; i++)
        {
            int p = (int)(keys[i] >> 32), q = (int)(keys[i] & 0xFFFFFFFF);
            long rev = ((long)q << 32) | (uint)p;
            if (Array.BinarySearch(keys, rev) < 0) { a = p; b = q; return GeometryToolError.OpenMesh; }
        }

        a = b = -1;
        return null;
    }

    /// <summary>Volume, centre of mass and inertia about the centre of mass for unit density, in double precision.</summary>
    /// <param name="vertices">Mesh vertices, finite, magnitude at most 2^21.</param>
    /// <param name="indices">Triangle corner indices, three per triangle, counter-clockwise seen from outside.</param>
    /// <returns><see cref="MassPropertiesD.Mass"/> is the volume (unit density).</returns>
    /// <exception cref="GeometryToolException">The input is invalid; the mesh has a hole (<see cref="GeometryToolError.OpenMesh"/>, naming the edge), an edge shared by more than two triangles or traversed twice in one direction (<see cref="GeometryToolError.NonManifold"/>), or encloses no or negative volume (<see cref="GeometryToolError.Degenerate"/>, <see cref="GeometryToolError.InvertedWinding"/>). <see cref="MeshCleaner"/> repairs most of these.</exception>
    public static MassPropertiesD ComputeDouble(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices)
    {
        MeshInput.Check(vertices, indices);
        var problem = FindProblem(indices, out int a, out int b, out int tri);
        if (problem is not null)
        {
            if (problem == GeometryToolError.OpenMesh)
            {
                throw new GeometryToolException(GeometryToolError.OpenMesh, "the mesh is open: edge " + MeshInput.Inv(a) + "-" + MeshInput.Inv(b) + " has no triangle on its other side", nameof(indices));
            }

            throw new GeometryToolException(GeometryToolError.NonManifold,
                tri >= 0 ? "triangle " + MeshInput.Inv(tri) + " repeats a vertex" : "directed edge " + MeshInput.Inv(a) + "-" + MeshInput.Inv(b) + " is used by more than one triangle", nameof(indices));
        }

        return Integrate(vertices, indices, out _);
    }

    /// <summary>Like <see cref="ComputeDouble"/>, rounded to single precision.</summary>
    /// <param name="vertices">Mesh vertices.</param>
    /// <param name="indices">Triangle corner indices, three per triangle.</param>
    /// <returns>The mass properties for unit density; <see cref="MassPropertiesF.Mass"/> is the volume.</returns>
    /// <exception cref="GeometryToolException">As for <see cref="ComputeDouble"/>.</exception>
    public static MassPropertiesF Compute(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices)
    {
        var d = ComputeDouble(vertices, indices);
        return new MassPropertiesF((float)d.Mass, d.Center.ToSingle(), d.Inertia.ToSingle());
    }

    /// <summary>The signed volume of the surface (positive for outward-facing triangles), without any closedness check. Useful for orienting a surface.</summary>
    /// <param name="vertices">Mesh vertices.</param>
    /// <param name="indices">Triangle corner indices, three per triangle.</param>
    /// <returns>The signed volume.</returns>
    /// <exception cref="GeometryToolException">The vertices or indices are invalid.</exception>
    public static double SignedVolume(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices)
    {
        MeshInput.Check(vertices, indices, requireTriangles: false);
        return SignedVolumeUnchecked(vertices, indices);
    }

    internal static double SignedVolumeUnchecked(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices)
    {
        double rx = 0, ry = 0, rz = 0;
        for (int i = 0; i < vertices.Length; i++) { rx += vertices[i].X; ry += vertices[i].Y; rz += vertices[i].Z; }
        if (vertices.Length > 0) { rx /= vertices.Length; ry /= vertices.Length; rz /= vertices.Length; }
        double vol = 0;
        for (int t = 0; t < indices.Length; t += 3)
        {
            Vector3F p = vertices[indices[t]], q = vertices[indices[t + 1]], r = vertices[indices[t + 2]];
            double ax = p.X - rx, ay = p.Y - ry, az = p.Z - rz;
            double bx = q.X - rx, by = q.Y - ry, bz = q.Z - rz;
            double cx = r.X - rx, cy = r.Y - ry, cz = r.Z - rz;
            vol += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6.0;
        }

        return vol;
    }

    internal static MassPropertiesD Integrate(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, out double rawVolume)
    {
        // Evaluated about the vertex mean so that large offsets do not cost precision. Moments of a simplex (Tonon 2004):
        // integral of x x^T = V/20 * (sum p_k p_k^T + (sum p_k)(sum p_k)^T).
        double rx = 0, ry = 0, rz = 0;
        for (int i = 0; i < vertices.Length; i++) { rx += vertices[i].X; ry += vertices[i].Y; rz += vertices[i].Z; }
        rx /= vertices.Length; ry /= vertices.Length; rz /= vertices.Length;

        double vol = 0, cx = 0, cy = 0, cz = 0;
        double sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
        for (int t = 0; t < indices.Length; t += 3)
        {
            Vector3F p = vertices[indices[t]], q = vertices[indices[t + 1]], r = vertices[indices[t + 2]];
            double ax = p.X - rx, ay = p.Y - ry, az = p.Z - rz;
            double bx = q.X - rx, by = q.Y - ry, bz = q.Z - rz;
            double qx = r.X - rx, qy = r.Y - ry, qz = r.Z - rz;
            double v = (ax * (by * qz - bz * qy) - ay * (bx * qz - bz * qx) + az * (bx * qy - by * qx)) / 6.0;
            vol += v;
            double ux = ax + bx + qx, uy = ay + by + qy, uz = az + bz + qz;
            cx += v * ux / 4.0; cy += v * uy / 4.0; cz += v * uz / 4.0;
            double f = v / 20.0;
            sxx += f * (ax * ax + bx * bx + qx * qx + ux * ux);
            syy += f * (ay * ay + by * by + qy * qy + uy * uy);
            szz += f * (az * az + bz * bz + qz * qz + uz * uz);
            sxy += f * (ax * ay + bx * by + qx * qy + ux * uy);
            sxz += f * (ax * az + bx * bz + qx * qz + ux * uz);
            syz += f * (ay * az + by * bz + qy * qz + uy * uz);
        }

        rawVolume = vol;
        if (!(vol > 0))
        {
            if (vol < 0 && -vol > 1e-12 * (MeshInput.Extent(vertices) * MeshInput.Extent(vertices) * MeshInput.Extent(vertices)))
            {
                throw new GeometryToolException(GeometryToolError.InvertedWinding, "the mesh encloses a negative volume (" + (-vol).ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "): its triangles wind clockwise seen from outside; MeshCleaner.Clean orients closed surfaces outward", nameof(indices));
            }

            throw new GeometryToolException(GeometryToolError.Degenerate, "the mesh encloses no volume", nameof(indices));
        }

        double mx = cx / vol, my = cy / vol, mz = cz / vol;
        sxx -= vol * mx * mx; syy -= vol * my * my; szz -= vol * mz * mz;
        sxy -= vol * mx * my; sxz -= vol * mx * mz; syz -= vol * my * mz;
        double tr = sxx + syy + szz;
        var inertia = new Matrix3x3D(tr - sxx, -sxy, -sxz, -sxy, tr - syy, -syz, -sxz, -syz, tr - szz);
        return new MassPropertiesD(vol, new Vector3D(mx + rx, my + ry, mz + rz), inertia);
    }
}
