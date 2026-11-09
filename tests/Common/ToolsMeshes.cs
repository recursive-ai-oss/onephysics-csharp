using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>A triangle mesh built by the generators below.</summary>
public sealed class TestMesh
{
    public List<Vector3F> Vertices = new();
    public List<int> Indices = new();

    public int Add(float x, float y, float z) { Vertices.Add(new Vector3F(x, y, z)); return Vertices.Count - 1; }

    public void Tri(int a, int b, int c) { Indices.Add(a); Indices.Add(b); Indices.Add(c); }

    public void Quad(int a, int b, int c, int d) { Tri(a, b, c); Tri(a, c, d); }

    public Vector3F[] V => Vertices.ToArray();
    public int[] I => Indices.ToArray();

    /// <summary>Appends another mesh (its own vertices) to this one.</summary>
    public void Append(TestMesh o)
    {
        int b = Vertices.Count;
        Vertices.AddRange(o.Vertices);
        foreach (int i in o.Indices) { Indices.Add(i + b); }
    }
}

/// <summary>Meshes generated for the tests; every one is closed and wound counter-clockwise seen from outside unless stated.</summary>
public static class TestMeshes
{
    public static TestMesh Box(float sx, float sy, float sz, float cx = 0, float cy = 0, float cz = 0)
    {
        var m = new TestMesh();
        float x = sx / 2, y = sy / 2, z = sz / 2;
        for (int i = 0; i < 8; i++) { m.Add(cx + ((i & 1) == 0 ? -x : x), cy + ((i & 2) == 0 ? -y : y), cz + ((i & 4) == 0 ? -z : z)); }
        // Corner index bits: x = 1, y = 2, z = 4.
        m.Quad(0, 4, 6, 2); // -x
        m.Quad(1, 3, 7, 5); // +x
        m.Quad(0, 1, 5, 4); // -y
        m.Quad(2, 6, 7, 3); // +y
        m.Quad(0, 2, 3, 1); // -z
        m.Quad(4, 5, 7, 6); // +z
        return m;
    }

    public static TestMesh Sphere(float r, int rings, int segments)
    {
        var m = new TestMesh();
        int top = m.Add(0, r, 0);
        for (int i = 1; i < rings; i++)
        {
            double phi = Math.PI * i / rings;
            for (int j = 0; j < segments; j++)
            {
                double th = 2 * Math.PI * j / segments;
                m.Add((float)(r * Math64.Sin(phi) * Math64.Cos(th)), (float)(r * Math64.Cos(phi)), (float)(r * Math64.Sin(phi) * Math64.Sin(th)));
            }
        }

        int bottom = m.Add(0, -r, 0);
        for (int j = 0; j < segments; j++) { m.Tri(top, 1 + (j + 1) % segments + 0 * j, 1 + j); }
        for (int i = 0; i < rings - 2; i++)
        {
            for (int j = 0; j < segments; j++)
            {
                int a = 1 + i * segments + j, b = 1 + i * segments + (j + 1) % segments, c = a + segments, d = b + segments;
                m.Quad(a, b, d, c);
            }
        }

        int last = 1 + (rings - 2) * segments;
        for (int j = 0; j < segments; j++) { m.Tri(bottom, last + j, last + (j + 1) % segments); }
        return m;
    }

    public static TestMesh Cylinder(float r, float halfHeight, int segments)
    {
        var m = new TestMesh();
        for (int j = 0; j < segments; j++)
        {
            double th = 2 * Math.PI * j / segments;
            m.Add((float)(r * Math64.Cos(th)), -halfHeight, (float)(r * Math64.Sin(th)));
            m.Add((float)(r * Math64.Cos(th)), halfHeight, (float)(r * Math64.Sin(th)));
        }

        int cb = m.Add(0, -halfHeight, 0), ct = m.Add(0, halfHeight, 0);
        for (int j = 0; j < segments; j++)
        {
            int j2 = (j + 1) % segments;
            int b0 = 2 * j, t0 = 2 * j + 1, b1 = 2 * j2, t1 = 2 * j2 + 1;
            m.Quad(b0, t0, t1, b1);
            m.Tri(ct, t1, t0);
            m.Tri(cb, b0, b1);
        }

        return m;
    }

    public static TestMesh Torus(float major, float minor, int ringSegments, int tubeSegments)
    {
        var m = new TestMesh();
        for (int i = 0; i < ringSegments; i++)
        {
            double u = 2 * Math.PI * i / ringSegments;
            for (int j = 0; j < tubeSegments; j++)
            {
                double v = 2 * Math.PI * j / tubeSegments;
                double rr = major + minor * Math64.Cos(v);
                m.Add((float)(rr * Math64.Cos(u)), (float)(minor * Math64.Sin(v)), (float)(rr * Math64.Sin(u)));
            }
        }

        for (int i = 0; i < ringSegments; i++)
        {
            for (int j = 0; j < tubeSegments; j++)
            {
                int i2 = (i + 1) % ringSegments, j2 = (j + 1) % tubeSegments;
                int a = i * tubeSegments + j, b = i2 * tubeSegments + j, c = i2 * tubeSegments + j2, d = i * tubeSegments + j2;
                m.Quad(a, d, c, b);
            }
        }

        return m;
    }

    /// <summary>A prism of a polygon in the xz plane between y = 0 and y = height, built from the polygon's convex decomposition.</summary>
    public static TestMesh Prism(Vector2F[] outline, float height)
    {
        var m = new TestMesh();
        var pieces = PolygonDecomposer.Decompose(outline, new PolygonDecompositionOptions { MaxVerticesPerPiece = 3 });
        foreach (var tri in pieces)
        {
            int a = m.Add(tri[0].X, height, tri[0].Y), b = m.Add(tri[1].X, height, tri[1].Y), c = m.Add(tri[2].X, height, tri[2].Y);
            // Counter-clockwise in (x, z) means clockwise seen from +y; flip the top so that it faces up.
            m.Tri(a, c, b);
            int a2 = m.Add(tri[0].X, 0, tri[0].Y), b2 = m.Add(tri[1].X, 0, tri[1].Y), c2 = m.Add(tri[2].X, 0, tri[2].Y);
            m.Tri(a2, b2, c2);
        }

        float area = 0;
        for (int i = 0; i < outline.Length; i++) { area += outline[i].X * outline[(i + 1) % outline.Length].Y - outline[(i + 1) % outline.Length].X * outline[i].Y; }
        bool ccw = area > 0;
        for (int i = 0; i < outline.Length; i++)
        {
            var p = outline[i];
            var q = outline[(i + 1) % outline.Length];
            int a = m.Add(p.X, 0, p.Y), b = m.Add(q.X, 0, q.Y), c = m.Add(q.X, height, q.Y), d = m.Add(p.X, height, p.Y);
            if (ccw) { m.Quad(a, d, c, b); } else { m.Quad(a, b, c, d); }
        }

        return m;
    }

    /// <summary>An L-shaped block: a 2 x 2 footprint with a 1 x 1 corner removed, one unit high.</summary>
    public static TestMesh LBlock() => Prism(new[] { new Vector2F(0, 0), new Vector2F(2, 0), new Vector2F(2, 1), new Vector2F(1, 1), new Vector2F(1, 2), new Vector2F(0, 2) }, 1f);

    /// <summary>A U-shaped block: 3 x 2 footprint with a 1 x 1 notch cut from the middle of one side.</summary>
    public static TestMesh UBlock() => Prism(new[] { new Vector2F(0, 0), new Vector2F(3, 0), new Vector2F(3, 2), new Vector2F(2, 2), new Vector2F(2, 1), new Vector2F(1, 1), new Vector2F(1, 2), new Vector2F(0, 2) }, 1f);

    /// <summary>A mug: an open cylinder cup (wall, floor and rim, closed surface) with a torus handle on one side that overlaps the wall.</summary>
    public static TestMesh Mug()
    {
        const int n = 32;
        float ro = 1.0f, ri = 0.85f, h = 2.0f, floor = 0.2f;
        var m = new TestMesh();
        // Profile around the y axis: outer bottom edge, outer top, inner top, inner floor.
        var profile = new[] { (0f, 0f), (ro, 0f), (ro, h), (ri, h), (ri, floor), (0f, floor) };
        int rings = profile.Length;
        for (int j = 0; j < n; j++)
        {
            double th = 2 * Math.PI * j / n;
            for (int p = 0; p < rings; p++)
            {
                m.Add((float)(profile[p].Item1 * Math64.Cos(th)), profile[p].Item2, (float)(profile[p].Item1 * Math64.Sin(th)));
            }
        }

        for (int j = 0; j < n; j++)
        {
            int j2 = (j + 1) % n;
            for (int p = 0; p < rings; p++)
            {
                int p2 = (p + 1) % rings;
                int a = j * rings + p, b = j * rings + p2, c = j2 * rings + p2, d = j2 * rings + p;
                // Skip the degenerate axis segments (profile points on the axis collapse).
                if (profile[p].Item1 == 0 && profile[p2].Item1 == 0) { continue; }
                if (profile[p].Item1 == 0) { m.Tri(a, b, c); continue; }
                if (profile[p2].Item1 == 0) { m.Tri(a, c, d); continue; }
                m.Quad(a, d, c, b);
            }
        }

        var handle = Torus(0.55f, 0.13f, 24, 10);
        // Stand the torus up in the xy plane, centred outside the wall, half of it inside the wall.
        for (int i = 0; i < handle.Vertices.Count; i++)
        {
            var v = handle.Vertices[i];
            handle.Vertices[i] = new Vector3F(v.X * 0.9f + 1.25f, v.Z * 0.9f + 1.0f, v.Y);
        }

        m.Append(handle);
        return m;
    }
}
