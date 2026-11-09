using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Convex shape against a signed distance field. The deepest points of a convex shape are among its support points, so the
    // shape is probed at the support points of 26 fixed directions (the corners of a box, the ends of a capsule, the rim of a cylinder) and
    // then, three times, at the support point opposite the field's gradient at the deepest probe (a local descent of the field over the shape,
    // as in Macklin et al., "Local Optimization for Robust Signed Distance Field Collision", 2020). A probe at core point c with convex radius r has
    // separation field(c) - r along the field's gradient there. At most four probes, spread over the largest area, become the manifold; its normal is
    // the gradient at the deepest one. Probes are in a fixed order, ties break on probe index, so the result is deterministic.
    private static readonly Vector3F[] ProbeDirections = BuildProbeDirections();

    private static Vector3F[] BuildProbeDirections()
    {
        var list = new System.Collections.Generic.List<Vector3F>();
        for (int z = -1; z <= 1; z++) for (int y = -1; y <= 1; y++) for (int x = -1; x <= 1; x++) if (x != 0 || y != 0 || z != 0) list.Add(new Vector3F(x, y, z));
        return list.ToArray();
    }

    internal struct SdfProbe { public Vector3F Core; public float Separation; public Vector3F Normal; public int Index; }

    /// <summary>Per-thread working storage for field contacts, so a step allocates nothing.</summary>
    internal sealed class SdfProbeBuffer
    {
        internal readonly SdfProbe[] Probes = new SdfProbe[1100];
        internal readonly ManifoldPoint3D[] Points = new ManifoldPoint3D[4];
        internal readonly int[] Pick = new int[4];
        internal int Count;
    }

    private static void AddProbe(SdfProbeBuffer b, SdfGrid3D grid, in ConvexShape3D shape, in RigidTransformF shapeInSoup, float radius, float spec, Vector3F direction, int index)
    {
        var cw = shapeInSoup.TransformPoint(shape.CoreSupport(direction));
        for (int i = 0; i < b.Count; i++) if (b.Probes[i].Core == cw) return;
        float sep = grid.Value(cw) - radius;
        if (sep > spec) return;
        b.Probes[b.Count++] = new SdfProbe { Core = cw, Separation = sep, Normal = grid.Gradient(cw), Index = index };
    }

    private static Vector3F surfacePoint(SdfSoup3D soup, int i) => soup.SurfacePoints[i].Point;

    private static void SdfVsConvex(NarrowScratch sc, in PairFrame f, SdfSoup3D soup, bool soupIsA, in ConvexShape3D shape, in RigidTransformF shapeLocal, bool hasChild,
        in RigidTransformF shapeInSoup, in RigidTransformF soupFromA, float spec, int child)
    {
        var grid = soup.Grid; float r = shape.ConvexRadius; var b = NarrowScratch.Sdf; b.Count = 0; var probes = b.Probes;
        for (int i = 0; i < ProbeDirections.Length; i++) AddProbe(b, grid, in shape, in shapeInSoup, r, spec, ProbeDirections[i], i);
        var inv = shapeInSoup.Rotation.Conjugated();
        for (int it = 0; it < 3 && b.Count > 0; it++)
        {
            int deepest = 0; for (int i = 1; i < b.Count; i++) if (probes[i].Separation < probes[deepest].Separation) deepest = i;
            AddProbe(b, grid, in shape, in shapeInSoup, r, spec, inv.Rotate(-probes[deepest].Normal), ProbeDirections.Length + it);
        }
        if (sc.SdfMoves)
        {
            // A moving field also offers its own surface points, tested against the convex shape (a point is a tiny sphere for the ordinary narrowphase): the shape may be much larger than the field.
            var gen = NarrowScratch.Gen; ShapeBounds3D.Compute(shape, shapeInSoup, out var lo, out var hi); lo -= new Vector3F(spec, spec, spec); hi += new Vector3F(spec, spec, spec);
            const float rho = 1e-3f; var probeShape = ConvexShape3D.Sphere(rho); var surface = soup.SurfacePoints;
            for (int i = 0; i < surface.Length && b.Count < probes.Length - 1; i++)
            {
                var sp = surface[i].Point;
                if (sp.X < lo.X || sp.Y < lo.Y || sp.Z < lo.Z || sp.X > hi.X || sp.Y > hi.Y || sp.Z > hi.Z) continue;
                var m0 = gen.Collide(probeShape, shape, new RigidTransformF(shapeInSoup.Rotation, shapeInSoup.Translation - sp), spec);
                if (m0.PointCount == 0) continue;
                float sep0 = m0[0].Separation + rho; if (sep0 > spec) continue;
                probes[b.Count++] = new SdfProbe { Core = sp + m0.Normal * (0.5f * sep0), Separation = sep0, Normal = m0.Normal, Index = 64 + i };
            }
        }
        int count = b.Count;
        if (count == 0) return;
        int d0 = 0; for (int i = 1; i < count; i++) if (probes[i].Separation < probes[d0].Separation || (probes[i].Separation == probes[d0].Separation && probes[i].Index < probes[d0].Index)) d0 = i;
        if (probes[d0].Index >= 64)
        {
            // The deepest probe is a sampled surface point: slide it over the field's surface to where the shape is closest (see RefineOnSurface).
            var gen = NarrowScratch.Gen; const float rho = 1e-3f; var probeShape = ConvexShape3D.Sphere(rho);
            var p = surfacePoint(soup, probes[d0].Index - 64); float sep = probes[d0].Separation; var nrm = probes[d0].Normal; float step = 0.5f * grid.CellSize;
            for (int it = 0; it < 10 && step > 0.01f * grid.CellSize; it++)
            {
                var ns = grid.Gradient(p); var tan = nrm - ns * Vector3F.Dot(nrm, ns); float len = tan.Length();
                if (len < 1e-6f) break;
                var q = p + tan * (step / len);
                for (int k = 0; k < 2; k++) q -= grid.Gradient(q) * grid.Value(q);
                var m1 = gen.Collide(probeShape, shape, new RigidTransformF(shapeInSoup.Rotation, shapeInSoup.Translation - q), spec);
                float s1 = m1.PointCount == 0 ? float.MaxValue : m1[0].Separation + rho;
                if (s1 < sep) { p = q; sep = s1; nrm = m1.Normal; } else { step *= 0.5f; }
            }
            probes[d0] = new SdfProbe { Core = p + nrm * (0.5f * sep), Separation = sep, Normal = nrm, Index = probes[d0].Index };
        }
        var n = probes[d0].Normal; int picked = PickSpread(b, count, d0, grid.CellSize); var pick = b.Pick;
        var pts = b.Points; int np = picked; var back = shapeInSoup.Inverted();
        for (int k = 0; k < np; k++)
        {
            var p = probes[pick[k]];
            // Midway between the shape's surface point (core - normal * r) and the field's surface point (core - normal * (separation + r)).
            var mid = p.Core - n * (r + 0.5f * p.Separation);
            uint shapeFeature = 0xC000u | (uint)(p.Index & 0x3FFF);
            uint id = soupIsA ? (0xC000u << 16) | shapeFeature : (shapeFeature << 16) | 0xC000u;
            pts[k] = new ManifoldPoint3D(soupIsA ? mid : back.TransformPoint(mid), p.Separation, id);
        }
        for (int a = 1; a < np; a++) { var t = pts[a]; int k = a - 1; while (k >= 0 && pts[k].Id > t.Id) { pts[k + 1] = pts[k]; k--; } pts[k + 1] = t; }
        var normal = soupIsA ? n : inv.Rotate(-n);
        var m = new Manifold3D(normal, np, pts[0], np > 1 ? pts[1] : default, np > 2 ? pts[2] : default, np > 3 ? pts[3] : default);
        bool map = hasChild && !soupIsA;
        int subA = soupIsA ? 0 : (child < 0 ? 0 : child), subB = soupIsA ? (child < 0 ? 0 : child) : 0;
        Emit(sc, in f, subA, subB, in m, shapeLocal, map, soupFromA, false, default(SoupTriangle3D), soupIsA, default);
    }



    // Spreads the manifold over the contact patch by its extremes along two fixed in-plane axes (stable from step to step: the probe points do not move in
    // their body, and a probe only displaces an earlier one when it is clearly farther out). The deepest probe is always kept. Returns the number picked in buf.Pick.
    private static int PickSpread(SdfProbeBuffer buf, int count, int d0, float cell)
    {
        var probes = buf.Probes; var pick = buf.Pick; int picked = 1; pick[0] = d0; var nm = probes[d0].Normal;
        var reference = Math.Abs(nm.X) < 0.7f ? new Vector3F(1, 0, 0) : new Vector3F(0, 1, 0);
        var u = Vector3F.Cross(nm, reference); u *= 1f / u.Length(); var w = Vector3F.Cross(nm, u);
        float band = probes[d0].Separation + 0.5f * cell, tol = 0.02f * cell;
        Span<float> bestDot = stackalloc float[4]; Span<int> bestAt = stackalloc int[4];
        for (int e = 0; e < 4; e++) { bestAt[e] = -1; bestDot[e] = float.MinValue; }
        for (int i = 0; i < count; i++)
        {
            if (probes[i].Separation > band) continue;
            var d = probes[i].Core - probes[d0].Core; float du = Vector3F.Dot(d, u), dw = Vector3F.Dot(d, w);
            if (du > bestDot[0] + tol) { bestDot[0] = du; bestAt[0] = i; }
            if (-du > bestDot[1] + tol) { bestDot[1] = -du; bestAt[1] = i; }
            if (dw > bestDot[2] + tol) { bestDot[2] = dw; bestAt[2] = i; }
            if (-dw > bestDot[3] + tol) { bestDot[3] = -dw; bestAt[3] = i; }
        }
        for (int e = 0; e < 4; e++)
        {
            int at = bestAt[e]; if (at < 0 || at == d0) continue;
            bool dup = false; for (int k = 0; k < picked; k++) if (pick[k] == at) dup = true;
            if (dup) continue;
            // Skip points that add no area: closer than a quarter cell to an already chosen point.
            bool near = false; for (int k = 0; k < picked; k++) if ((probes[pick[k]].Core - probes[at].Core).LengthSquared() < 0.0625f * cell * cell) near = true;
            if (!near && picked < 4) pick[picked++] = at;
        }
        return picked;
    }

    // Slides a surface point along its own surface towards where the other field is deepest (projected gradient descent with a halving step,
    // then two Newton steps back onto the surface). The sampled surface points are about a cell apart; this puts the contact where a curved body truly touches.
    private static Vector3F RefineOnSurface(SdfGrid3D own, SdfGrid3D other, in RigidTransformF ownToOther, Vector3F point, float cell, out float separation)
    {
        var toOther = ownToOther; var p = point; float sep = other.Value(toOther.TransformPoint(p)); float step = 0.5f * cell;
        for (int it = 0; it < 10 && step > 0.01f * cell; it++)
        {
            var g = toOther.Rotation.Conjugated().Rotate(other.Gradient(toOther.TransformPoint(p))); // other's gradient in own frame
            var n = own.Gradient(p); var tan = g - n * Vector3F.Dot(g, n); float len = tan.Length();
            if (len < 1e-6f) break;
            var q = p - tan * (step / len);
            for (int k = 0; k < 2; k++) q -= own.Gradient(q) * own.Value(q);
            float s = other.Value(toOther.TransformPoint(q));
            if (s < sep) { p = q; sep = s; } else { step *= 0.5f; }
        }
        separation = sep; return p;
    }

    // A field against a field: the surface points of each are tested against the other (Fuhrmann et al. 2003, Macklin et al. 2020), a point on a surface
    // with separation field_other(point) from the other surface. The normal of the deepest probe is the other field's gradient, pointing from A to B.
    private static void SdfVsSdf(NarrowScratch sc, in PairFrame f, SdfSoup3D a, SdfSoup3D b, in RigidTransformF bInA, float spec)
    {
        var buf = NarrowScratch.Sdf; buf.Count = 0; var probes = buf.Probes; var aInB = bInA.Inverted();
        var ga = a.Grid; var gb = b.Grid; var bp = b.SurfacePoints; var ap = a.SurfacePoints; int limit = probes.Length - 1;
        // Probes reuse SdfProbe: Core = the point of the contact in A's frame, Normal = unit normal from A to B, Index = a stable id.
        for (int i = 0; i < bp.Length && buf.Count < limit; i++)
        {
            var p = bInA.TransformPoint(bp[i].Point); float sep = ga.Value(p);
            if (sep > spec) continue;
            var n = ga.Gradient(p);
            probes[buf.Count++] = new SdfProbe { Core = p - n * (0.5f * sep), Separation = sep, Normal = n, Index = i };
        }
        for (int i = 0; i < ap.Length && buf.Count < limit; i++)
        {
            var q = aInB.TransformPoint(ap[i].Point); float sep = gb.Value(q);
            if (sep > spec) continue;
            var n = -bInA.Rotation.Rotate(gb.Gradient(q));
            probes[buf.Count++] = new SdfProbe { Core = ap[i].Point + n * (0.5f * sep), Separation = sep, Normal = n, Index = 1024 + i };
        }
        int count = buf.Count;
        if (count == 0) return;
        // The deepest probe (ties by index), then the farthest from it in the contact plane and the two probes spanning the largest area on either side.
        int d0 = 0; for (int i = 1; i < count; i++) if (probes[i].Separation < probes[d0].Separation || (probes[i].Separation == probes[d0].Separation && probes[i].Index < probes[d0].Index)) d0 = i;
        {
            // Refine the deepest probe onto the true contact point of the two surfaces.
            ref var d = ref probes[d0];
            if (d.Index >= 1024)
            {
                var q = RefineOnSurface(ga, gb, in aInB, ap[d.Index - 1024].Point, ga.CellSize, out float sep);
                var qb = aInB.TransformPoint(q); var nn = -bInA.Rotation.Rotate(gb.Gradient(qb));
                d = new SdfProbe { Core = q + nn * (0.5f * sep), Separation = sep, Normal = nn, Index = d.Index };
            }
            else
            {
                var pb = RefineOnSurface(gb, ga, in bInA, bp[d.Index].Point, gb.CellSize, out float sep);
                var pa = bInA.TransformPoint(pb); var nn = ga.Gradient(pa);
                d = new SdfProbe { Core = pa - nn * (0.5f * sep), Separation = sep, Normal = nn, Index = d.Index };
            }
        }
        var nm = probes[d0].Normal; int picked = PickSpread(buf, count, d0, Math.Max(ga.CellSize, gb.CellSize)); var pick = buf.Pick;
        var pts = buf.Points;
        for (int k = 0; k < picked; k++)
        {
            var p = probes[pick[k]];
            uint fromB = (uint)(0xC000 | (p.Index & 0x3FFF));
            uint id = p.Index >= 1024 ? (fromB << 16) | 0xC000u : (0xC000u << 16) | fromB;
            pts[k] = new ManifoldPoint3D(p.Core, p.Separation, id);
        }
        for (int x = 1; x < picked; x++) { var t = pts[x]; int k = x - 1; while (k >= 0 && pts[k].Id > t.Id) { pts[k + 1] = pts[k]; k--; } pts[k + 1] = t; }
        var m = new Manifold3D(nm, picked, pts[0], picked > 1 ? pts[1] : default, picked > 2 ? pts[2] : default, picked > 3 ? pts[3] : default);
        Emit(sc, in f, 0, 0, in m, default, false, default, false, default);
    }
}
