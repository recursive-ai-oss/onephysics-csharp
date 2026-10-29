using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>What a closed-form test could settle.</summary>
internal enum SweepOutcome : byte
{
    /// <summary>The sweep misses the shape.</summary>
    Miss,

    /// <summary>The sweep enters the shape from outside at the reported fraction.</summary>
    Hit,

    /// <summary>Not decided here (a shape kind without a closed form, a start inside or touching the shape, a grazing or edge-region hit): the general GJK route decides.</summary>
    Unsure,
}

/// <summary>
/// Closed-form ray and sphere-sweep tests against the common convex shapes, in the shape's own frame. A sphere of radius <c>r</c> swept along a segment is a ray against the shape grown by <c>r</c> (Minkowski sum;
/// Ericson, Real-Time Collision Detection, 5.3 and 5.5): spheres and capsules grow into spheres and capsules, boxes and hulls into their face planes moved out by <c>r</c> (Cyrus and Beck 1978, the slab method).
/// A decision is only returned when it is clear: anything starting inside or within a hair of the shape, grazing it, hitting a grown polytope's edge or corner region or involving another shape kind is left to
/// <see cref="Gjk3D"/>, so these tests never answer a case the general route would answer differently beyond its own tolerance. Arithmetic is IEEE double on float inputs: exact operations only, same bits everywhere.
/// </summary>
internal static class ConvexSweep3D
{
    private const double Margin = 1e-6;   // length slack: closer than this to a decision boundary goes to the general route
    private const double TMargin = 1e-9;   // fraction slack, the same idea along the segment

    /// <summary>
    /// Sweeps a sphere of radius <paramref name="moverRadius"/> (0 for a ray) from <paramref name="o"/> along <paramref name="d"/> (fractions 0 to 1) against <paramref name="shape"/>.
    /// On a hit, <paramref name="fraction"/> is the entry fraction and <paramref name="normal"/> the shape's outward unit normal at the contact.
    /// </summary>
    public static SweepOutcome Test(in ConvexShape3D shape, float moverRadius, Vector3F o, Vector3F d, out float fraction, out Vector3F normal)
    {
        fraction = 0f; normal = default;
        double r = moverRadius;
        switch (shape.Kind)
        {
            case ConvexShapeKind3D.Sphere:
                return Capsule(shape.ConvexRadius + r, 0.0, o, d, out fraction, out normal);
            case ConvexShapeKind3D.Capsule:
                return Capsule(shape.ConvexRadius + r, shape.CoreSize.Y, o, d, out fraction, out normal);
            case ConvexShapeKind3D.Box:
                return Polytope(shape, shape.ConvexRadius + r, o, d, out fraction, out normal);
            case ConvexShapeKind3D.Hull:
                return shape.Hull is null ? SweepOutcome.Unsure : Polytope(shape, shape.ConvexRadius + r, o, d, out fraction, out normal);
            default:
                return SweepOutcome.Unsure;
        }
    }

    // The segment from o along d against the capsule with core segment (0, -h, 0)..(0, h, 0) and radius R (a sphere for h = 0).
    private static SweepOutcome Capsule(double rad, double h, Vector3F o, Vector3F d, out float fraction, out Vector3F normal)
    {
        fraction = 0f; normal = default;
        double ox = o.X, oy = o.Y, oz = o.Z, dx = d.X, dy = d.Y, dz = d.Z;
        double dd = dx * dx + dy * dy + dz * dz;
        if (!(dd > 0.0)) { return SweepOutcome.Unsure; }

        // Starting inside, or within the margin of the surface, is the general route's: it reports the overlap semantics.
        double cy = Math.Min(Math.Max(oy, -h), h);
        double sy = oy - cy;
        double start2 = ox * ox + sy * sy + oz * oz;
        double lim = rad + Margin * (1.0 + rad);
        if (start2 <= lim * lim) { return SweepOutcome.Unsure; }

        double axy = dx * dx + dz * dz;
        if (h > 0.0 && axy != 0.0 && axy <= 1e-6 * dd) { return SweepOutcome.Unsure; } // nearly along the axis: the side and the caps trade places, leave it to the general route
        if (h > 0.0 && axy != 0.0)
        {
            // The infinite cylinder around the Y axis; the hit counts if it lies between the end caps.
            double b = ox * dx + oz * dz, c = ox * ox + oz * oz - rad * rad;
            double disc = b * b - axy * c;
            double tol = 1e-12 * (b * b + Math.Abs(axy * c));
            if (disc < -tol && c > 0.0)
            {
                return SweepOutcome.Miss; // the infinite cylinder is missed, so the capsule is too
            }

            if (disc >= -tol && disc <= tol) { return SweepOutcome.Unsure; } // grazing
            if (c > 0.0 && disc > 0.0 && b < 0.0)
            {
                double tc = (-b - Math.Sqrt(disc)) / axy;
                double yc = oy + tc * dy;
                if (yc >= -h && yc <= h)
                {
                    if (tc > 1.0 + TMargin) { return SweepOutcome.Miss; }
                    if (tc > 1.0 - TMargin) { return SweepOutcome.Unsure; }
                    double px = ox + tc * dx, pz = oz + tc * dz;
                    fraction = (float)tc;
                    double inv = 1.0 / rad;
                    normal = new Vector3F((float)(px * inv), 0f, (float)(pz * inv));
                    return tc > TMargin ? SweepOutcome.Hit : SweepOutcome.Unsure;
                }

                if (Math.Abs(yc) - h <= Margin * (1.0 + h)) { return SweepOutcome.Unsure; } // the cap seam
            }
            else if (c > 0.0) { return SweepOutcome.Miss; } // outside the cylinder and moving away from it (b >= 0) or no real roots
        }

        // The end caps: the ray enters the capsule through one of the two end spheres. Test both and keep the first entry.
        double best = double.PositiveInfinity;
        double bcy = 0.0;
        for (int k = 0; k < (h > 0.0 ? 2 : 1); k++)
        {
            double cyk = k == 0 ? h : -h;
            double qy = oy - cyk;
            double b = ox * dx + qy * dy + oz * dz;
            double c = ox * ox + qy * qy + oz * oz - rad * rad;
            double disc = b * b - dd * c;
            double tol = 1e-12 * (b * b + Math.Abs(dd * c));
            if (c <= 0.0) { continue; } // inside this sphere: the capsule is entered through the other part, checked by the start test above
            if (b >= 0.0 || disc < -tol) { continue; }
            if (disc <= tol) { return SweepOutcome.Unsure; }
            double ts = (-b - Math.Sqrt(disc)) / dd;
            // A sphere point only counts on its own side of the segment's end (y beyond the end), elsewhere the cylinder covers it.
            double yh = oy + ts * dy - cyk;
            if (h > 0.0 && (k == 0 ? yh < 0.0 : yh > 0.0))
            {
                if (Math.Abs(yh) <= Margin * (1.0 + h)) { return SweepOutcome.Unsure; }
                continue;
            }

            if (ts < best) { best = ts; bcy = cyk; }
        }

        if (double.IsPositiveInfinity(best))
        {
            return SweepOutcome.Miss;
        }

        if (best > 1.0 + TMargin) { return SweepOutcome.Miss; }
        if (best > 1.0 - TMargin || best <= TMargin) { return SweepOutcome.Unsure; }
        {
            double px = ox + best * dx, py = oy + best * dy - bcy, pz = oz + best * dz;
            double inv = 1.0 / rad;
            fraction = (float)best;
            normal = new Vector3F((float)(px * inv), (float)(py * inv), (float)(pz * inv));
            return SweepOutcome.Hit;
        }
    }

    // The face planes of a box or hull core, n . x <= offset, k = 0 .. count - 1.
    private static int PlaneCount(in ConvexShape3D shape) => shape.Kind == ConvexShapeKind3D.Box ? 6 : shape.Hull!.Faces.Length;

    private static void Plane(in ConvexShape3D shape, int k, out double nx, out double ny, out double nz, out double off)
    {
        if (shape.Kind == ConvexShapeKind3D.Box)
        {
            var h = shape.CoreSize;
            int axis = k >> 1; double s = (k & 1) == 0 ? 1.0 : -1.0;
            nx = axis == 0 ? s : 0.0; ny = axis == 1 ? s : 0.0; nz = axis == 2 ? s : 0.0;
            off = axis == 0 ? h.X : axis == 1 ? h.Y : h.Z;
            return;
        }

        ref readonly HullFace3D f = ref shape.Hull!.Faces[k];
        nx = f.Normal.X; ny = f.Normal.Y; nz = f.Normal.Z; off = f.Offset;
    }

    // Cyrus-Beck against the core's planes moved out by rad. Exact for a ray (rad = 0); for rad > 0 only when the contact is in a face region, which is checked.
    private static SweepOutcome Polytope(in ConvexShape3D shape, double rad, Vector3F o, Vector3F d, out float fraction, out Vector3F normal)
    {
        fraction = 0f; normal = default;
        double ox = o.X, oy = o.Y, oz = o.Z, dx = d.X, dy = d.Y, dz = d.Z;
        int count = PlaneCount(shape);
        double tEnter = double.NegativeInfinity, tExit = double.PositiveInfinity;
        int face = -1;
        double slack = Margin * (1.0 + rad + Math.Abs(ox) + Math.Abs(oy) + Math.Abs(oz));
        for (int k = 0; k < count; k++)
        {
            Plane(shape, k, out double nx, out double ny, out double nz, out double off);
            double nd = nx * dx + ny * dy + nz * dz;
            double dist = off + rad - (nx * ox + ny * oy + nz * oz); // positive inside this plane
            if (nd == 0.0)
            {
                if (dist < -slack) { return SweepOutcome.Miss; }
                if (dist <= slack) { return SweepOutcome.Unsure; }
                continue;
            }

            double t = dist / nd;
            if (nd < 0.0) { if (t > tEnter) { tEnter = t; face = k; } }
            else if (t < tExit) { tExit = t; }
        }

        if (tEnter <= TMargin) { return tExit < -TMargin ? SweepOutcome.Miss : SweepOutcome.Unsure; } // starts inside, touching, or the shape is behind the start
        if (tEnter > tExit + TMargin) { return SweepOutcome.Miss; }
        if (tEnter > tExit - TMargin) { return SweepOutcome.Unsure; }
        if (tEnter > 1.0 + TMargin) { return SweepOutcome.Miss; }
        if (tEnter > 1.0 - TMargin) { return SweepOutcome.Unsure; }

        Plane(shape, face, out double fx, out double fy, out double fz, out _);
        if (rad > 0.0)
        {
            // The grown polytope has sharp edges where the true shape is round: the contact is on a face only if the point of the core nearest the contact lies inside every other face.
            double qx = ox + tEnter * dx - fx * rad, qy = oy + tEnter * dy - fy * rad, qz = oz + tEnter * dz - fz * rad;
            double qs = Margin * (1.0 + Math.Abs(qx) + Math.Abs(qy) + Math.Abs(qz));
            for (int k = 0; k < count; k++)
            {
                if (k == face) { continue; }
                Plane(shape, k, out double nx, out double ny, out double nz, out double off);
                if (nx * qx + ny * qy + nz * qz > off - qs) { return SweepOutcome.Unsure; }
            }
        }

        fraction = (float)tEnter;
        normal = new Vector3F((float)fx, (float)fy, (float)fz);
        return SweepOutcome.Hit;
    }
}
