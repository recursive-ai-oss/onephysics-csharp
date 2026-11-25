using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Options for <see cref="PhysicsWorld3D.GetStabilityMargin"/>.</summary>
public sealed class StabilityOptions3D
{
    /// <summary>Number of horizontal directions sampled around the up axis, 4 to 360. Direction 0 is a fixed horizontal axis derived from gravity (see <see cref="StabilityMargin3D.Directions"/>). Default 16.</summary>
    public int DirectionCount { get; set; } = 16;
    /// <summary>A contact point supports the assembly when its separation is at most this many metres. Default 0.005.</summary>
    public float ContactTolerance { get; set; } = 0.005f;

    internal void Validate()
    {
        if (DirectionCount < 4 || DirectionCount > 360) throw new ArgumentOutOfRangeException(nameof(DirectionCount), DirectionCount, "DirectionCount must be within 4..360.");
        if (!(ContactTolerance >= 0f) || !float.IsFinite(ContactTolerance)) throw new ArgumentOutOfRangeException(nameof(ContactTolerance), ContactTolerance, "ContactTolerance must be finite and nonnegative.");
    }
}

/// <summary>
/// How hard it is to tip over a body or assembly that rests on other bodies (static equilibrium, no dynamics). The support polygon is the convex hull of the contact points between the assembly and everything outside it,
/// projected onto the plane perpendicular to gravity. The assembly tips about a hull edge when its centre of mass passes over it.
/// For a horizontal push of size F at the centre of mass in direction d, the torque about edge e (outward normal n) is F (d·n) H and the weight's restoring torque is m g w, with w the horizontal distance from the centre of mass to the edge and H its height above the pivot,
/// so the push that tips it is the smallest m g w / (H (d·n)) over edges with d·n &gt; 0. The same ratio is the tangent of the angle the ground can be tilted toward d before the assembly tips.
/// Friction is not in that estimate: <see cref="SlidingForce"/> says what a push has to beat to slide instead.
/// </summary>
public sealed class StabilityMargin3D
{
    internal StabilityMargin3D() { }

    /// <summary>True when at least one supporting contact was found. Without it nothing else is meaningful.</summary>
    public bool HasSupport { get; internal set; }
    /// <summary>True when the support polygon has area and the centre of mass projects strictly inside it.</summary>
    public bool IsStable { get; internal set; }
    /// <summary>Total mass of the assembly, kg.</summary>
    public float Mass { get; internal set; }
    /// <summary>Centre of mass of the assembly in world coordinates, metres.</summary>
    public Vector3D CenterOfMass { get; internal set; }
    /// <summary>The unit up direction, opposite to gravity.</summary>
    public Vector3F Up { get; internal set; }
    /// <summary>Corners of the support polygon (3D contact points), counter-clockwise seen from above, without repeated or collinear corners. One or two points when the support is a point or a line.</summary>
    public Vector3D[] SupportPolygon { get; internal set; } = [];
    /// <summary>Horizontal distance from the centre of mass to the nearest polygon edge, metres: positive inside, negative outside, zero for a degenerate polygon.</summary>
    public float Margin { get; internal set; }
    /// <summary>The horizontal sample directions, unit vectors.</summary>
    public Vector3F[] Directions { get; internal set; } = [];
    /// <summary>For each direction, the angle in radians the ground can be tilted toward it before the assembly tips (zero when unstable).</summary>
    public float[] TipAngles { get; internal set; } = [];
    /// <summary>For each direction, the smallest horizontal push in newtons at the centre of mass that tips it (zero when unstable).</summary>
    public float[] TippingForces { get; internal set; } = [];
    /// <summary>The smallest tip angle over the directions, radians.</summary>
    public float MinTipAngle { get; internal set; }
    /// <summary>The direction of <see cref="MinTipAngle"/> (the first, for equal angles).</summary>
    public Vector3F MinTipDirection { get; internal set; }
    /// <summary>The smallest tipping push over the directions, newtons.</summary>
    public float MinTippingForce { get; internal set; }
    /// <summary>Friction force a horizontal push has to exceed to slide the assembly: the mean contact friction (weighted by normal impulse) times the weight, newtons.</summary>
    public float SlidingForce { get; internal set; }
    /// <summary>True when the weakest tipping push is larger than <see cref="SlidingForce"/>, so the assembly slides before it tips.</summary>
    public bool SlidesBeforeTipping => HasSupport && MinTippingForce > SlidingForce;

    internal static StabilityMargin3D Compute(PhysicsWorld3D world, ReadOnlySpan<BodyHandle> handles, StabilityOptions3D options)
    {
        options.Validate();
        var core = world.Core;
        var slots = StabilitySupport.Slots(core, handles);
        var g = world.Options.Gravity;
        double gl = Math.Sqrt((double)g.X * g.X + (double)g.Y * g.Y + (double)g.Z * g.Z);
        if (!(gl > 0)) throw new InvalidOperationException("The world has no gravity, so there is no up direction to measure stability against.");
        double ux = -g.X / gl, uy = -g.Y / gl, uz = -g.Z / gl;
        double mass = 0, cx = 0, cy = 0, cz = 0;
        foreach (int s in slots)
        {
            double m = world.Engine.Mass[s]; var p = core.Bodies.PositionOf(s);
            mass += m; cx += m * p[0]; cy += m * p[1]; cz += m * p[2];
        }
        cx /= mass; cy /= mass; cz /= mass;
        var result = new StabilityMargin3D { Mass = (float)mass, CenterOfMass = new Vector3D(cx, cy, cz), Up = new Vector3F((float)ux, (float)uy, (float)uz) };
        // A horizontal basis from gravity: cross the up direction with the axis it is least aligned with.
        double ax = Math.Abs(ux), ay = Math.Abs(uy), az = Math.Abs(uz);
        double hx = 0, hy = 0, hz = 0;
        if (ax <= ay && ax <= az) hx = 1; else if (ay <= az) hy = 1; else hz = 1;
        double e1x = hy * uz - hz * uy, e1y = hz * ux - hx * uz, e1z = hx * uy - hy * ux;
        double l1 = Math.Sqrt(e1x * e1x + e1y * e1y + e1z * e1z); e1x /= l1; e1y /= l1; e1z /= l1;
        double e2x = uy * e1z - uz * e1y, e2y = uz * e1x - ux * e1z, e2z = ux * e1y - uy * e1x;
        int count = options.DirectionCount;
        var dirs = new Vector3F[count];
        double[] dc = new double[count], ds = new double[count];
        for (int k = 0; k < count; k++)
        {
            double phi = 2.0 * Math.PI * k / count;
            dc[k] = Math64.Cos(phi); ds[k] = Math64.Sin(phi);
            dirs[k] = new Vector3F((float)(dc[k] * e1x + ds[k] * e2x), (float)(dc[k] * e1y + ds[k] * e2y), (float)(dc[k] * e1z + ds[k] * e2z));
        }
        result.Directions = dirs; result.TipAngles = new float[count]; result.TippingForces = new float[count];
        var support = StabilitySupport.Gather(core, slots, options.ContactTolerance);
        if (support.Count == 0) return result;
        result.HasSupport = true;
        result.SlidingForce = (float)(StabilitySupport.EffectiveFriction(support) * mass * gl);

        // Project onto the horizontal plane (x, y in the basis) and keep the height.
        var px = new double[support.Count]; var py = new double[support.Count]; var ph = new double[support.Count];
        var order = new int[support.Count];
        for (int i = 0; i < support.Count; i++)
        {
            var s = support[i];
            px[i] = s.X * e1x + s.Y * e1y + s.Z * e1z; py[i] = s.X * e2x + s.Y * e2y + s.Z * e2z; ph[i] = s.X * ux + s.Y * uy + s.Z * uz; order[i] = i;
        }
        var hull = Hull(px, py, ph, order);
        var polygon = new Vector3D[hull.Length];
        for (int i = 0; i < hull.Length; i++)
        {
            int k = hull[i]; var s = support[k];
            polygon[i] = new Vector3D(s.X, s.Y, s.Z);
        }
        result.SupportPolygon = polygon;
        double qx = cx * e1x + cy * e1y + cz * e1z, qy = cx * e2x + cy * e2y + cz * e2z, qh = cx * ux + cy * uy + cz * uz;
        double area = 0;
        for (int i = 0; i < hull.Length; i++) { int a = hull[i], b = hull[(i + 1) % hull.Length]; area += px[a] * py[b] - px[b] * py[a]; }
        if (hull.Length < 3 || !(area > 1e-12)) { result.Margin = 0; Fill(result, 0f, 0f, count); return result; }
        int n = hull.Length;
        var nx = new double[n]; var ny = new double[n]; var w = new double[n]; var hgt = new double[n];
        double margin = double.PositiveInfinity;
        for (int i = 0; i < n; i++)
        {
            int a = hull[i], b = hull[(i + 1) % n];
            double dx = px[b] - px[a], dy = py[b] - py[a], len = Math.Sqrt(dx * dx + dy * dy);
            nx[i] = dy / len; ny[i] = -dx / len;
            w[i] = -(nx[i] * (qx - px[a]) + ny[i] * (qy - py[a]));
            double t = ((qx - px[a]) * dx + (qy - py[a]) * dy) / (len * len);
            t = t < 0 ? 0 : t > 1 ? 1 : t;
            hgt[i] = qh - (ph[a] + t * (ph[b] - ph[a]));
            if (w[i] < margin) margin = w[i];
        }
        result.Margin = (float)margin;
        result.IsStable = margin > 0;
        double minAngle = double.PositiveInfinity, minForce = double.PositiveInfinity; int minK = 0;
        for (int k = 0; k < count; k++)
        {
            double ratio = double.PositiveInfinity;
            for (int i = 0; i < n; i++)
            {
                double dn = dc[k] * nx[i] + ds[k] * ny[i];
                if (!(dn > 1e-9)) continue;
                double r = hgt[i] > 1e-12 ? w[i] / (hgt[i] * dn) : (w[i] > 0 ? double.PositiveInfinity : 0.0);
                if (r < ratio) ratio = r;
            }
            if (!result.IsStable || ratio < 0) ratio = 0;
            float angle = double.IsPositiveInfinity(ratio) ? (float)(Math.PI / 2) : (float)Math64.Atan(ratio);
            float force = double.IsPositiveInfinity(ratio) ? float.PositiveInfinity : (float)(ratio * mass * gl);
            result.TipAngles[k] = angle; result.TippingForces[k] = force;
            if (angle < minAngle) { minAngle = angle; minK = k; }
            if (force < minForce) { minForce = force; }
        }
        result.MinTipAngle = (float)minAngle; result.MinTipDirection = dirs[minK]; result.MinTippingForce = (float)minForce;
        return result;
    }

    private static void Fill(StabilityMargin3D r, float angle, float force, int count)
    {
        for (int k = 0; k < count; k++) { r.TipAngles[k] = angle; r.TippingForces[k] = force; }
        r.MinTipAngle = angle; r.MinTippingForce = force; r.MinTipDirection = r.Directions[0];
    }

    // Andrew's monotone chain on the projected points; ties in x break on y then on the lower height then the index, so the hull does not depend on the order contacts were found.
    private static int[] Hull(double[] x, double[] y, double[] h, int[] idx)
    {
        Array.Sort(idx, (a, b) =>
        {
            int c = x[a].CompareTo(x[b]); if (c != 0) return c;
            c = y[a].CompareTo(y[b]); if (c != 0) return c;
            c = h[a].CompareTo(h[b]); return c != 0 ? c : a.CompareTo(b);
        });
        var unique = new List<int>(idx.Length);
        foreach (int i in idx) { if (unique.Count == 0 || x[unique[^1]] != x[i] || y[unique[^1]] != y[i]) unique.Add(i); }
        if (unique.Count < 3) return unique.ToArray();
        var hull = new List<int>(unique.Count * 2);
        double Cross(int o, int a, int b) => (x[a] - x[o]) * (y[b] - y[o]) - (y[a] - y[o]) * (x[b] - x[o]);
        foreach (int p in unique) { while (hull.Count >= 2 && Cross(hull[^2], hull[^1], p) <= 0) hull.RemoveAt(hull.Count - 1); hull.Add(p); }
        int lower = hull.Count + 1;
        for (int i = unique.Count - 2; i >= 0; i--)
        {
            int p = unique[i];
            while (hull.Count >= lower && Cross(hull[^2], hull[^1], p) <= 0) hull.RemoveAt(hull.Count - 1);
            hull.Add(p);
        }
        hull.RemoveAt(hull.Count - 1);
        return hull.ToArray();
    }
}
