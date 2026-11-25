using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Options for <see cref="PhysicsWorld2D.GetStabilityMargin"/>.</summary>
public sealed class StabilityOptions2D
{
    /// <summary>Number of horizontal directions sampled. A planar world has two (along the tangent and against it), so this is fixed at 2; it exists so both dimensions have the same options. Default 2.</summary>
    public int DirectionCount { get; set; } = 2;
    /// <summary>A contact point supports the assembly when its separation is at most this many metres. Default 0.005.</summary>
    public float ContactTolerance { get; set; } = 0.005f;

    internal void Validate()
    {
        if (DirectionCount != 2) throw new ArgumentOutOfRangeException(nameof(DirectionCount), DirectionCount, "A planar world has exactly two horizontal directions.");
        if (!(ContactTolerance >= 0f) || !float.IsFinite(ContactTolerance)) throw new ArgumentOutOfRangeException(nameof(ContactTolerance), ContactTolerance, "ContactTolerance must be finite and nonnegative.");
    }
}

/// <summary>
/// How hard it is to tip over a body or assembly that rests on other bodies (static equilibrium, no dynamics). The support is an interval along the tangent perpendicular to gravity, from the outermost contact points at each side;
/// the assembly tips about an end of it when its centre of mass passes over that end. For a horizontal push F at the centre of mass the torque about the pivot is F H and the weight restores m g w, with w the horizontal distance
/// from the centre of mass to the pivot and H its height above it, so the push that tips it is m g w / H and the ground can be tilted by atan(w / H) before it tips.
/// Friction is not in that estimate: <see cref="SlidingForce"/> says what a push has to beat to slide instead.
/// </summary>
public sealed class StabilityMargin2D
{
    internal StabilityMargin2D() { }

    /// <summary>True when at least one supporting contact was found. Without it nothing else is meaningful.</summary>
    public bool HasSupport { get; internal set; }
    /// <summary>True when the support interval has length and the centre of mass projects strictly inside it.</summary>
    public bool IsStable { get; internal set; }
    /// <summary>Total mass of the assembly, kg.</summary>
    public float Mass { get; internal set; }
    /// <summary>Centre of mass of the assembly in world coordinates, metres.</summary>
    public Vector2D CenterOfMass { get; internal set; }
    /// <summary>The unit up direction, opposite to gravity.</summary>
    public Vector2F Up { get; internal set; }
    /// <summary>The two ends of the support interval, first the end against the tangent, then the end along it (one point when the support is a single point), in world coordinates.</summary>
    public Vector2D[] SupportPolygon { get; internal set; } = [];
    /// <summary>Horizontal distance from the centre of mass to the nearer end of the support interval, metres: positive inside, negative outside, zero for a degenerate interval.</summary>
    public float Margin { get; internal set; }
    /// <summary>The two horizontal directions: along the tangent (up rotated a quarter turn clockwise) and against it.</summary>
    public Vector2F[] Directions { get; internal set; } = [];
    /// <summary>For each direction, the angle in radians the ground can be tilted toward it before the assembly tips (zero when unstable).</summary>
    public float[] TipAngles { get; internal set; } = [];
    /// <summary>For each direction, the horizontal push in newtons at the centre of mass that tips it (zero when unstable).</summary>
    public float[] TippingForces { get; internal set; } = [];
    /// <summary>The smaller tip angle, radians.</summary>
    public float MinTipAngle { get; internal set; }
    /// <summary>The direction of <see cref="MinTipAngle"/> (the first, for equal angles).</summary>
    public Vector2F MinTipDirection { get; internal set; }
    /// <summary>The smaller tipping push, newtons.</summary>
    public float MinTippingForce { get; internal set; }
    /// <summary>Friction force a horizontal push has to exceed to slide the assembly: the mean contact friction (weighted by normal impulse) times the weight, newtons.</summary>
    public float SlidingForce { get; internal set; }
    /// <summary>True when the weakest tipping push is larger than <see cref="SlidingForce"/>, so the assembly slides before it tips.</summary>
    public bool SlidesBeforeTipping => HasSupport && MinTippingForce > SlidingForce;

    internal static StabilityMargin2D Compute(PhysicsWorld2D world, ReadOnlySpan<BodyHandle> handles, StabilityOptions2D options)
    {
        options.Validate();
        var core = world.Core;
        var slots = StabilitySupport.Slots(core, handles);
        var g = world.Options.Gravity;
        double gl = Math.Sqrt((double)g.X * g.X + (double)g.Y * g.Y);
        if (!(gl > 0)) throw new InvalidOperationException("The world has no gravity, so there is no up direction to measure stability against.");
        double ux = -g.X / gl, uy = -g.Y / gl, tx = uy, ty = -ux;
        double mass = 0, cx = 0, cy = 0;
        foreach (int s in slots)
        {
            double m = world.Engine.Mass[s]; var p = core.Bodies.PositionOf(s);
            mass += m; cx += m * p[0]; cy += m * p[1];
        }
        cx /= mass; cy /= mass;
        var result = new StabilityMargin2D
        {
            Mass = (float)mass, CenterOfMass = new Vector2D(cx, cy), Up = new Vector2F((float)ux, (float)uy),
            Directions = [new Vector2F((float)tx, (float)ty), new Vector2F((float)-tx, (float)-ty)], TipAngles = new float[2], TippingForces = new float[2],
        };
        var support = StabilitySupport.Gather(core, slots, options.ContactTolerance);
        if (support.Count == 0) return result;
        result.HasSupport = true;
        result.SlidingForce = (float)(StabilitySupport.EffectiveFriction(support) * mass * gl);
        double sc = cx * tx + cy * ty, hc = cx * ux + cy * uy;
        // The extreme contact points along the tangent; equal ones keep the lower, then the earlier.
        int lo = 0, hi = 0;
        double Along(int i) => support[i].X * tx + support[i].Y * ty;
        double Height(int i) => support[i].X * ux + support[i].Y * uy;
        for (int i = 1; i < support.Count; i++)
        {
            double s = Along(i);
            if (s < Along(lo) || (s == Along(lo) && Height(i) < Height(lo))) lo = i;
            if (s > Along(hi) || (s == Along(hi) && Height(i) < Height(hi))) hi = i;
        }
        var left = new Vector2D(support[lo].X, support[lo].Y); var right = new Vector2D(support[hi].X, support[hi].Y);
        result.SupportPolygon = Along(hi) > Along(lo) ? [left, right] : [left];
        double wRight = Along(hi) - sc, wLeft = sc - Along(lo);
        double margin = Math.Min(wRight, wLeft);
        result.Margin = (float)(Along(hi) > Along(lo) ? margin : 0);
        result.IsStable = Along(hi) > Along(lo) && margin > 0;
        if (result.IsStable)
        {
            double[] ratios = [Ratio(wRight, hc - Height(hi)), Ratio(wLeft, hc - Height(lo))];
            for (int k = 0; k < 2; k++)
            {
                result.TipAngles[k] = double.IsPositiveInfinity(ratios[k]) ? (float)(Math.PI / 2) : (float)Math64.Atan(ratios[k]);
                result.TippingForces[k] = double.IsPositiveInfinity(ratios[k]) ? float.PositiveInfinity : (float)(ratios[k] * mass * gl);
            }
        }
        int minK = result.TipAngles[1] < result.TipAngles[0] ? 1 : 0;
        result.MinTipAngle = result.TipAngles[minK]; result.MinTipDirection = result.Directions[minK];
        result.MinTippingForce = Math.Min(result.TippingForces[0], result.TippingForces[1]);
        return result;
    }

    private static double Ratio(double w, double height) => height > 1e-12 ? w / height : double.PositiveInfinity;
}
