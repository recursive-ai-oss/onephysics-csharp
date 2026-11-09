using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>What one element of a track's plan view does.</summary>
public enum TrackElementKind : byte
{
    /// <summary>A straight of constant heading.</summary>
    Straight = 0,

    /// <summary>A circular arc: curvature constant.</summary>
    Arc = 1,

    /// <summary>A clothoid (Euler spiral) transition: curvature changes linearly with distance.</summary>
    Transition = 2,
}

/// <summary>One element of a track's plan view. Curvature is 1/radius in 1/m and positive turns to the right.</summary>
public readonly struct TrackElement : IEquatable<TrackElement>
{
    /// <summary>The kind of element.</summary>
    public TrackElementKind Kind { get; }

    /// <summary>Length along the track, m. Positive.</summary>
    public float Length { get; }

    /// <summary>Curvature at the start of the element, 1/m (0 for a straight).</summary>
    public float StartCurvature { get; }

    /// <summary>Curvature at the end of the element, 1/m (equal to the start for a straight or an arc).</summary>
    public float EndCurvature { get; }

    private TrackElement(TrackElementKind kind, float length, float k0, float k1) { Kind = kind; Length = length; StartCurvature = k0; EndCurvature = k1; }

    /// <summary>A straight.</summary>
    /// <param name="length">Length, m.</param>
    public static TrackElement Straight(float length) => new(TrackElementKind.Straight, length, 0f, 0f);

    /// <summary>A circular arc.</summary>
    /// <param name="length">Length along the arc, m.</param>
    /// <param name="radius">Radius, m, at least 10; positive turns right, negative left.</param>
    public static TrackElement Arc(float length, float radius) => new(TrackElementKind.Arc, length, 1f / radius, 1f / radius);

    /// <summary>A clothoid transition between two curvatures.</summary>
    /// <param name="length">Length, m.</param>
    /// <param name="startCurvature">Curvature at the start, 1/m (0 for the straight end of an easement).</param>
    /// <param name="endCurvature">Curvature at the end, 1/m.</param>
    public static TrackElement Transition(float length, float startCurvature, float endCurvature) => new(TrackElementKind.Transition, length, startCurvature, endCurvature);

    /// <inheritdoc/>
    public bool Equals(TrackElement other) => Kind == other.Kind && Length.Equals(other.Length) && StartCurvature.Equals(other.StartCurvature) && EndCurvature.Equals(other.EndCurvature);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is TrackElement e && Equals(e);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(Kind, Length, StartCurvature, EndCurvature);

    /// <summary>Equality of all fields.</summary>
    public static bool operator ==(TrackElement a, TrackElement b) => a.Equals(b);

    /// <summary>Inequality of any field.</summary>
    public static bool operator !=(TrackElement a, TrackElement b) => !a.Equals(b);
}

/// <summary>
/// A railway track as a curve: a plan view of straights, circular arcs and clothoid transitions, a vertical profile (heights along the track) and a cant profile (the rails' height difference), authored once and
/// used by both engines. In 3D the track is laid out from <see cref="Start"/> heading <see cref="StartHeading"/> in the horizontal plane (Y up; heading 0 points along +Z, positive turns towards +X), and
/// rail vehicles run on it with wheel-rail contact. In 2D only the vertical profile matters: the track runs along +X from <see cref="Start"/> and the plan view must be one straight.
/// Distances along the track ("chainage" s) are horizontal distances; the track's own height at s comes from <see cref="Elevation"/>.
/// </summary>
public sealed class TrackDef
{
    /// <summary>Where the track begins: the centre of the track at rail-head height. Default the origin. Each component within ±2^21 and (for exact origin shifts) a multiple of 2^-32.</summary>
    public Vector3D Start { get; set; }

    /// <summary>Heading of the track at its start, radians about +Y (3D only). Default 0.</summary>
    public float StartHeading { get; set; }

    /// <summary>The plan view, in order. Default one 100 m straight. At most 4096 elements and 20 km in all.</summary>
    public TrackElement[] Elements { get; set; } = { TrackElement.Straight(100f) };

    /// <summary>Heights of the track along its length as (chainage m, height m above <see cref="Start"/>) pairs with increasing chainage; the height is a smooth (cubic Hermite) curve through them and flat outside them. Null or empty: level.</summary>
    public Vector2F[]? Elevation { get; set; }

    /// <summary>Cant as (chainage m, cant m) pairs with increasing chainage, interpolated linearly: the right rail lies this much lower than the left one (negative for the other way). Null or empty: none.</summary>
    public Vector2F[]? Cant { get; set; }

    /// <summary>Distance between the inner faces of the rails, m. Default 1.435 (standard gauge). Within 0.3..3.</summary>
    public float Gauge { get; set; } = 1.435f;

    /// <summary>Friction coefficient between wheel and rail. Default 0.35 (dry steel). Within 0..2.</summary>
    public float Friction { get; set; } = 0.35f;

    /// <summary>When true (the default) a 3D track also builds static collision geometry: the ballast bed and the two rails, so that other bodies can rest on and hit them. Wheelsets pass through the rails (their contact is the wheel-rail model) but meet the ballast.</summary>
    public bool Collide { get; set; } = true;

    /// <summary>A copy.</summary>
    public TrackDef Clone()
    {
        var c = (TrackDef)MemberwiseClone();
        c.Elements = (TrackElement[])Elements.Clone();
        c.Elevation = (Vector2F[]?)Elevation?.Clone();
        c.Cant = (Vector2F[]?)Cant?.Clone();
        return c;
    }

    /// <summary>The length of the plan view, m.</summary>
    public float Length { get { float l = 0f; foreach (var e in Elements) { l += e.Length; } return l; } }

    /// <summary>A straight track of the given length, level, starting at the origin.</summary>
    public static TrackDef Straight(float length) => new() { Elements = new[] { TrackElement.Straight(length) } };

    /// <summary>
    /// A curve of <paramref name="arcLength"/> metres at <paramref name="radius"/> metres between two clothoid easements of <paramref name="transitionLength"/>, preceded and followed by a straight of
    /// <paramref name="lead"/> metres, with cant rising to <paramref name="cant"/> metres along each easement.
    /// </summary>
    public static TrackDef Curve(float radius, float arcLength, float transitionLength, float cant, float lead = 50f)
    {
        float k = 1f / radius;
        var d = new TrackDef
        {
            Elements = new[] { TrackElement.Straight(lead), TrackElement.Transition(transitionLength, 0f, k), TrackElement.Arc(arcLength, radius), TrackElement.Transition(transitionLength, k, 0f), TrackElement.Straight(lead) },
        };
        float s1 = lead, s2 = lead + transitionLength, s3 = s2 + arcLength, s4 = s3 + transitionLength;
        d.Cant = new[] { new Vector2F(0f, 0f), new Vector2F(s1, 0f), new Vector2F(s2, cant), new Vector2F(s3, cant), new Vector2F(s4, 0f) };
        return d;
    }

    internal const int MaxElements = 4096;
    internal const float MaxLength = 20000f;
    internal const int MaxProfilePoints = 4096;

    /// <summary>Checks every value; throws an <see cref="ArgumentOutOfRangeException"/> naming the field, the value and the bound.</summary>
    /// <param name="planar">True for a 2D track: the plan view must be a single straight.</param>
    public void Validate(bool planar = false)
    {
        if (Elements == null || Elements.Length == 0) { throw Bad(nameof(Elements), "A track needs at least one element."); }
        if (Elements.Length > MaxElements) { throw Bad(nameof(Elements), Elements.Length, "A track has at most " + MaxElements.ToString(CultureInfo.InvariantCulture) + " elements."); }
        double total = 0.0;
        for (int i = 0; i < Elements.Length; i++)
        {
            var e = Elements[i];
            string w = "Elements[" + i.ToString(CultureInfo.InvariantCulture) + "]";
            if (!(e.Length > 0f) || !float.IsFinite(e.Length) || e.Length > MaxLength) { throw Bad(w + ".Length", e.Length, w + ".Length must be finite, greater than 0 and at most " + MaxLength.ToString(CultureInfo.InvariantCulture) + " m."); }
            if (!(Math.Abs(e.StartCurvature) <= 0.1f) || !(Math.Abs(e.EndCurvature) <= 0.1f)) { throw Bad(w, "Curvature must be within ±0.1 1/m (radius at least 10 m) in " + w + "."); }
            if (e.Kind > TrackElementKind.Transition) { throw Bad(w + ".Kind", e.Kind, w + " has an unknown kind."); }
            if (e.Kind != TrackElementKind.Transition && e.StartCurvature != e.EndCurvature) { throw Bad(w, w + " is not a transition, so its curvature cannot change."); }
            if (e.Kind == TrackElementKind.Straight && e.StartCurvature != 0f) { throw Bad(w, w + " is a straight with curvature."); }
            if (planar && (e.Kind != TrackElementKind.Straight)) { throw Bad(w, "A 2D track is a vertical profile: its plan view must be one straight."); }
            total += e.Length;
        }

        if (planar && Elements.Length != 1) { throw Bad(nameof(Elements), Elements.Length, "A 2D track is a vertical profile: its plan view must be one straight."); }
        if (!(total <= MaxLength)) { throw Bad(nameof(Elements), total, "A track is at most " + MaxLength.ToString(CultureInfo.InvariantCulture) + " m long."); }
        CheckStart(Start.X, nameof(Start) + ".X"); CheckStart(Start.Y, nameof(Start) + ".Y"); CheckStart(Start.Z, nameof(Start) + ".Z");
        if (!float.IsFinite(StartHeading) || Math.Abs(StartHeading) > 1000f) { throw Bad(nameof(StartHeading), StartHeading, "StartHeading must be finite and within ±1000 rad."); }
        CheckProfile(Elevation, nameof(Elevation), 1e4f, total);
        CheckProfile(Cant, nameof(Cant), 0.5f * Gauge, total);
        if (!(Gauge >= 0.3f && Gauge <= 3f)) { throw Bad(nameof(Gauge), Gauge, "Gauge must be within 0.3..3 m."); }
        if (!(Friction >= 0f && Friction <= 2f)) { throw Bad(nameof(Friction), Friction, "Friction must be within 0..2."); }
    }

    // The parameter names here are the definition's properties, which the analyser does not know as parameters.
    private static ArgumentOutOfRangeException Bad(string name, string message) => new(name, message);

    private static ArgumentOutOfRangeException Bad(string name, object? value, string message) => new(name, value, message);

    private static void CheckStart(double v, string name)
    {
        if (!(Math.Abs(v) <= 2097152.0)) { throw Bad(name, v, name + " must be finite and within ±2097152 m."); }
        if (v * 4294967296.0 != Math.Round(v * 4294967296.0)) { throw Bad(name, v, name + " must be a multiple of 2^-32 m so that origin shifts stay exact."); }
    }

    private static void CheckProfile(Vector2F[]? p, string name, float limit, double total)
    {
        if (p == null) { return; }
        if (p.Length > MaxProfilePoints) { throw Bad(name, p.Length, name + " has at most " + MaxProfilePoints.ToString(CultureInfo.InvariantCulture) + " points."); }
        float prev = float.NegativeInfinity;
        for (int i = 0; i < p.Length; i++)
        {
            if (!float.IsFinite(p[i].X) || !float.IsFinite(p[i].Y) || Math.Abs(p[i].Y) > limit) { throw Bad(name, p[i], name + "[" + i.ToString(CultureInfo.InvariantCulture) + "] must be finite with a value within ±" + limit.ToString(CultureInfo.InvariantCulture) + "."); }
            if (!(p[i].X > prev)) { throw Bad(name, p[i].X, name + " chainages must increase."); }
            if (p[i].X < -1f || p[i].X > total + 1.0) { throw Bad(name, p[i].X, name + " chainage lies outside the track."); }
            prev = p[i].X;
        }
    }
}

/// <summary>The local frame of a track at one chainage: where the centreline is at rail-head height and the track's own axes.</summary>
public readonly struct TrackFrame
{
    /// <summary>Creates a frame.</summary>
    public TrackFrame(Vector3D position, Vector3F tangent, Vector3F right, Vector3F up, float curvature, float cantAngle)
    {
        Position = position; Tangent = tangent; Right = right; Up = up; Curvature = curvature; CantAngle = cantAngle;
    }

    /// <summary>The centre of the track at rail-head height.</summary>
    public Vector3D Position { get; }

    /// <summary>Unit vector along the track (including the grade).</summary>
    public Vector3F Tangent { get; }

    /// <summary>Unit vector across the track to the right, tilted by the cant.</summary>
    public Vector3F Right { get; }

    /// <summary>Unit vector normal to the track's surface (up, tilted by the cant and the grade).</summary>
    public Vector3F Up { get; }

    /// <summary>Plan curvature, 1/m, positive to the right.</summary>
    public float Curvature { get; }

    /// <summary>Roll of the track about its tangent, radians, positive when the right rail is lower.</summary>
    public float CantAngle { get; }
}

/// <summary>Evaluates a <see cref="TrackDef"/>: positions, headings and frames by chainage. Built once from the definition; everything is deterministic and exact under origin shifts (positions are rounded to the 2^-32 grid).</summary>
internal sealed class TrackGeometry
{
    private const double Grid = 4294967296.0;
    private const double PanelLength = 1.0;
    private readonly TrackDef _def;
    private readonly double[] _elementStart;   // chainage at the start of each element
    private readonly double[] _elementHeading; // heading at the start of each element
    // Panels: positions and headings at multiples of the panel length (also at every element start).
    private readonly double[] _panelS, _panelX, _panelZ, _panelHeading;
    private readonly int[] _panelElement;
    private readonly double _length;
    private readonly double _startHeading;

    // Gauss-Legendre nodes and weights on [-1, 1], three points: exact for the smooth integrands over a panel of a metre.
    private static readonly double[] GlX = { -0.7745966692414834, 0.0, 0.7745966692414834 };
    private static readonly double[] GlW = { 5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0 };

    internal TrackGeometry(TrackDef def)
    {
        _def = def;
        int n = def.Elements.Length;
        _elementStart = new double[n]; _elementHeading = new double[n];
        double s = 0.0, h = def.StartHeading;
        for (int i = 0; i < n; i++)
        {
            _elementStart[i] = s; _elementHeading[i] = h;
            var e = def.Elements[i];
            s += e.Length;
            h += 0.5 * ((double)e.StartCurvature + (double)e.EndCurvature) * e.Length;
        }

        _length = s; _startHeading = def.StartHeading;
        var ps = new List<double>(); var px = new List<double>(); var pz = new List<double>(); var ph = new List<double>(); var pe = new List<int>();
        double x = 0.0, z = 0.0;
        for (int i = 0; i < n; i++)
        {
            var e = def.Elements[i];
            double s0 = _elementStart[i], len = e.Length;
            int panels = (int)Math.Ceiling(len / PanelLength);
            for (int p = 0; p < panels; p++)
            {
                double a = s0 + p * PanelLength;
                ps.Add(a); px.Add(x); pz.Add(z); ph.Add(Heading(i, a)); pe.Add(i);
                double b = Math.Min(s0 + (p + 1) * PanelLength, s0 + len);
                Integrate(i, a, b, ref x, ref z);
            }
        }

        _panelS = ps.ToArray(); _panelX = px.ToArray(); _panelZ = pz.ToArray(); _panelHeading = ph.ToArray(); _panelElement = pe.ToArray();
    }

    /// <summary>Total plan length of the track, m.</summary>
    internal double Length => _length;

    private double Heading(int element, double s)
    {
        var e = _def.Elements[element];
        double u = s - _elementStart[element];
        double k0 = e.StartCurvature, k1 = e.EndCurvature;
        return _elementHeading[element] + k0 * u + (k1 - k0) * u * u / (2.0 * e.Length);
    }

    // Adds the displacement between chainages a and b of one element to (x, z): heading ψ moves the point by (sin ψ, cos ψ) per metre.
    private void Integrate(int element, double a, double b, ref double x, ref double z)
    {
        double mid = 0.5 * (a + b), half = 0.5 * (b - a);
        double sx = 0.0, sz = 0.0;
        for (int i = 0; i < 3; i++)
        {
            double psi = Heading(element, mid + half * GlX[i]);
            sx += GlW[i] * Math64.Sin(psi); sz += GlW[i] * Math64.Cos(psi);
        }

        x += half * sx; z += half * sz;
    }

    private static double Quantize(double v) => Math.Round(v * Grid) / Grid;

    /// <summary>Plan position (relative to the start, on the 2^-32 grid) and heading at chainage s; beyond either end the track continues straight.</summary>
    internal void Plan(double s, out double x, out double z, out double heading, out double curvature)
    {
        if (s <= 0.0)
        {
            double h0 = _startHeading;
            x = Quantize(s * Math64.Sin(h0)); z = Quantize(s * Math64.Cos(h0)); heading = h0; curvature = _def.Elements[0].StartCurvature;
            return;
        }

        if (s >= _length)
        {
            int last = _def.Elements.Length - 1;
            double hEnd = Heading(last, _elementStart[last] + _def.Elements[last].Length);
            // End position: integrate the last panel to the end.
            int pl = _panelS.Length - 1;
            double ex = _panelX[pl], ez = _panelZ[pl];
            Integrate(_panelElement[pl], _panelS[pl], _length, ref ex, ref ez);
            double d = s - _length;
            x = Quantize(ex + d * Math64.Sin(hEnd)); z = Quantize(ez + d * Math64.Cos(hEnd)); heading = hEnd; curvature = _def.Elements[last].EndCurvature;
            return;
        }

        int lo = 0, hi = _panelS.Length - 1;
        while (lo < hi) { int m = (lo + hi + 1) >> 1; if (_panelS[m] <= s) { lo = m; } else { hi = m - 1; } }
        double px = _panelX[lo], pz = _panelZ[lo];
        int el = _panelElement[lo];
        Integrate(el, _panelS[lo], s, ref px, ref pz);
        x = Quantize(px); z = Quantize(pz); heading = Heading(el, s);
        var e = _def.Elements[el];
        double u = s - _elementStart[el];
        curvature = e.StartCurvature + (e.EndCurvature - e.StartCurvature) * u / e.Length;
    }

    /// <summary>Height above the start (on the 2^-32 grid) and the grade (dz/ds) at chainage s.</summary>
    internal void Profile(double s, out double height, out double grade)
    {
        var p = _def.Elevation;
        if (p == null || p.Length == 0) { height = 0.0; grade = 0.0; return; }
        int n = p.Length;
        if (n == 1 || s <= p[0].X) { height = Quantize(p[0].Y); grade = 0.0; return; }
        if (s >= p[n - 1].X) { height = Quantize(p[n - 1].Y); grade = 0.0; return; }
        int lo = 0, hi = n - 1;
        while (hi - lo > 1) { int m = (lo + hi) >> 1; if (p[m].X <= s) { lo = m; } else { hi = m; } }
        double s0 = p[lo].X, s1 = p[hi].X, z0 = p[lo].Y, z1 = p[hi].Y;
        double dx = s1 - s0;
        double m0 = Slope(p, lo), m1 = Slope(p, hi);
        double t = (s - s0) / dx, t2 = t * t, t3 = t2 * t;
        double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t, h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
        height = Quantize(h00 * z0 + h10 * dx * m0 + h01 * z1 + h11 * dx * m1);
        double d00 = 6 * t2 - 6 * t, d10 = 3 * t2 - 4 * t + 1, d01 = -6 * t2 + 6 * t, d11 = 3 * t2 - 2 * t;
        grade = (d00 * z0 + d10 * dx * m0 + d01 * z1 + d11 * dx * m1) / dx;
    }

    private static double Slope(Vector2F[] p, int i)
    {
        int n = p.Length;
        if (i == 0) { return ((double)p[1].Y - p[0].Y) / ((double)p[1].X - p[0].X); }
        if (i == n - 1) { return ((double)p[n - 1].Y - p[n - 2].Y) / ((double)p[n - 1].X - p[n - 2].X); }
        return ((double)p[i + 1].Y - p[i - 1].Y) / ((double)p[i + 1].X - p[i - 1].X);
    }

    /// <summary>Cant (m, the right rail lower) at chainage s.</summary>
    internal double CantAt(double s)
    {
        var p = _def.Cant;
        if (p == null || p.Length == 0) { return 0.0; }
        int n = p.Length;
        if (s <= p[0].X) { return p[0].Y; }
        if (s >= p[n - 1].X) { return p[n - 1].Y; }
        int lo = 0, hi = n - 1;
        while (hi - lo > 1) { int m = (lo + hi) >> 1; if (p[m].X <= s) { lo = m; } else { hi = m; } }
        double t = (s - p[lo].X) / ((double)p[hi].X - p[lo].X);
        return p[lo].Y + t * ((double)p[hi].Y - p[lo].Y);
    }

    /// <summary>The frame at chainage s.</summary>
    internal TrackFrame FrameAt(double s)
    {
        Plan(s, out double x, out double z, out double psi, out double kappa);
        Profile(s, out double h, out double grade);
        var start = _def.Start;
        var pos = new Vector3D(start.X + x, start.Y + h, start.Z + z);
        float sp = (float)Math64.Sin(psi), cp = (float)Math64.Cos(psi);
        float gr = (float)grade;
        float inv = 1f / float.Sqrt(1f + gr * gr);
        var t = new Vector3F(sp * inv, gr * inv, cp * inv);
        // Right = up x tangent (horizontal), up0 = tangent x right.
        float rl = float.Sqrt(t.X * t.X + t.Z * t.Z);
        var r0 = new Vector3F(t.Z / rl, 0f, -t.X / rl);
        var u0 = Vector3F.Cross(t, r0);
        double cant = CantAt(s);
        float sinPhi = (float)Math.Max(-0.5, Math.Min(0.5, cant / _def.Gauge));
        float cosPhi = float.Sqrt(1f - sinPhi * sinPhi);
        var r = r0 * cosPhi - u0 * sinPhi;
        var u = u0 * cosPhi + r0 * sinPhi;
        return new TrackFrame(pos, t, r, u, (float)kappa, Math32.Asin(sinPhi));
    }
}
