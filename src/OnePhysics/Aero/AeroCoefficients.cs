using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Aero;

/// <summary>The medium a lifting surface, thruster or foil works in.</summary>
public enum AeroMedium : byte
{
    /// <summary>The air of the world: density from the world's air setting, flow from the wind of the area fields (see the fields documentation).</summary>
    Air = 0,

    /// <summary>The water of the world: the first fluid or buoyancy field whose surface the point is below; density and flow come from it. The force fades in over the depth of the surface's own size as it goes under.</summary>
    Water = 1,
}

/// <summary>How a surface turns the relative flow into force.</summary>
public enum AeroSurfaceKind : byte
{
    /// <summary>A lifting surface (wing, plate, fin, rudder): lift and drag from the angle of attack in the plane of its chord and normal, from <see cref="AeroCoefficients"/>. Flow along the span makes no force.</summary>
    Lifting = 0,

    /// <summary>A bluff body: drag only, ½·ρ·C<sub>d</sub>·A·|v|·v with C<sub>d</sub> = <see cref="AeroCoefficients.MinDrag"/> and A the shape's silhouette seen along the flow (or the fixed area given).</summary>
    Bluff = 1,
}

/// <summary>
/// The lift and drag curves of a lifting surface as functions of the angle of attack α (the angle between the chord and the relative flow, positive when the flow hits the side the normal points away from).
/// Attached flow: C<sub>L</sub> = a·(α − α<sub>0</sub>) and C<sub>D</sub> = C<sub>D0</sub> + k·C<sub>L</sub>², with the finite-wing lift slope a and induced-drag factor k = 1/(π·e·AR) (Anderson, <i>Fundamentals of Aerodynamics</i>, thin-aerofoil and lifting-line results).
/// Separated flow past the stall: the flat-plate normal force C<sub>N</sub> = C<sub>D90</sub>·sin α, so C<sub>L</sub> = C<sub>D90</sub>·sin α·cos α and C<sub>D</sub> = C<sub>D0</sub> + C<sub>D90</sub>·sin²α (Hoerner, <i>Fluid-Dynamic Drag</i>, flat plate normal to the flow).
/// The two are blended with a smoothstep over <see cref="StallBlend"/> radians starting at <see cref="StallAngle"/> from the zero-lift angle, so the curves are continuous and the surface stalls the same way on both sides and in reversed flow.
/// </summary>
public struct AeroCoefficients
{
    /// <summary>Lift slope dC<sub>L</sub>/dα in 1/rad before the stall. Default 2π/(1 + 2/AR) with AR 6 (a finite wing).</summary>
    public float LiftSlope { get; set; }

    /// <summary>Zero-lift angle α<sub>0</sub>, radians (negative for a cambered aerofoil). Default 0.</summary>
    public float ZeroLiftAngle { get; set; }

    /// <summary>Half-width of the attached-flow range, radians: the curves start leaving the linear model at |α − α<sub>0</sub>| = StallAngle.</summary>
    public float StallAngle { get; set; }

    /// <summary>Angle over which the attached model hands over to the separated model, radians; above 0.</summary>
    public float StallBlend { get; set; }

    /// <summary>C<sub>D0</sub>: drag at zero lift (the whole drag coefficient of a <see cref="AeroSurfaceKind.Bluff"/> body).</summary>
    public float MinDrag { get; set; }

    /// <summary>k in C<sub>D</sub> = C<sub>D0</sub> + k·C<sub>L</sub>², the induced drag; 0 for none.</summary>
    public float InducedDrag { get; set; }

    /// <summary>C<sub>D90</sub>, the drag coefficient of the surface broadside to the flow.</summary>
    public float PlateDrag { get; set; }

    /// <summary>The largest values accepted for any coefficient.</summary>
    public const float MaxCoefficient = 1000f;

    /// <summary>The broadside drag of a flat plate of aspect ratio <paramref name="aspectRatio"/> (Hoerner: 1.1 + 0.018·AR up to the infinite plate's 1.98 at AR 20 and beyond).</summary>
    public static float PlateDragOf(float aspectRatio) => aspectRatio >= 20f ? 1.98f : 1.1f + 0.018f * aspectRatio;

    /// <summary>A finite wing of the given aspect ratio: lift slope 2π/(1 + 2/AR), induced drag 1/(π·0.85·AR), stall at 16° with a 17° blend, C<sub>D0</sub> 0.02.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The aspect ratio is not within 0.1 to 100.</exception>
    public static AeroCoefficients Wing(float aspectRatio = 6f)
    {
        CheckAspect(aspectRatio);
        return new AeroCoefficients
        {
            LiftSlope = 6.2831855f / (1f + 2f / aspectRatio), ZeroLiftAngle = 0f, StallAngle = 0.28f, StallBlend = 0.3f,
            MinDrag = 0.02f, InducedDrag = 1f / (3.1415927f * 0.85f * aspectRatio), PlateDrag = PlateDragOf(aspectRatio),
        };
    }

    /// <summary>A flat plate of the given aspect ratio: the same lift slope, a lower stall (10°) and a skin-friction C<sub>D0</sub> of 0.01.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The aspect ratio is not within 0.1 to 100.</exception>
    public static AeroCoefficients Plate(float aspectRatio = 2f)
    {
        CheckAspect(aspectRatio);
        return new AeroCoefficients
        {
            LiftSlope = 6.2831855f / (1f + 2f / aspectRatio), ZeroLiftAngle = 0f, StallAngle = 0.17f, StallBlend = 0.35f,
            MinDrag = 0.01f, InducedDrag = 1f / (3.1415927f * 0.9f * aspectRatio), PlateDrag = PlateDragOf(aspectRatio),
        };
    }

    /// <summary>A bluff body with the given drag coefficient (0.47 is a sphere, 1.05 a cube, 0.04 a streamlined body).</summary>
    public static AeroCoefficients Bluff(float dragCoefficient = 1f) => new() { LiftSlope = 0f, StallAngle = 0.28f, StallBlend = 0.3f, MinDrag = dragCoefficient, PlateDrag = dragCoefficient };

    private static void CheckAspect(float ar)
    {
        if (!(ar >= 0.1f && ar <= 100f)) { throw new ArgumentOutOfRangeException(nameof(ar), ar, "The aspect ratio must be within 0.1 to 100."); }
    }

    /// <summary>Checks every field.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A field is NaN, infinite or outside its range; the message names it.</exception>
    public readonly void Validate()
    {
        Check(LiftSlope, 0f, 100f, nameof(LiftSlope));
        if (!(ZeroLiftAngle >= -0.5f && ZeroLiftAngle <= 0.5f)) { throw Bad(nameof(ZeroLiftAngle), ZeroLiftAngle, "must be within ±0.5 rad"); }
        if (!(StallAngle >= 0.01f && StallAngle <= 1.2f)) { throw Bad(nameof(StallAngle), StallAngle, "must be within 0.01 to 1.2 rad"); }
        if (!(StallBlend >= 0.01f && StallBlend <= 0.7f)) { throw Bad(nameof(StallBlend), StallBlend, "must be within 0.01 to 0.7 rad"); }
        if (!(StallAngle + StallBlend + Math.Abs(ZeroLiftAngle) <= 1.5f)) { throw Bad(nameof(StallAngle), StallAngle, "plus StallBlend and |ZeroLiftAngle| must stay within 1.5 rad so the surface is fully stalled by a right angle"); }
        Check(MinDrag, 0f, MaxCoefficient, nameof(MinDrag));
        Check(InducedDrag, 0f, MaxCoefficient, nameof(InducedDrag));
        Check(PlateDrag, 0f, MaxCoefficient, nameof(PlateDrag));
    }

    private static void Check(float v, float lo, float hi, string name)
    {
        if (!(v >= lo && v <= hi)) { throw Bad(name, v, "must be within " + lo.ToString(CultureInfo.InvariantCulture) + " to " + hi.ToString(CultureInfo.InvariantCulture)); }
    }

    private static ArgumentOutOfRangeException Bad(string name, float v, string rule) =>
        new(name, v, "AeroCoefficients." + name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + " but " + rule + ".");

    /// <summary>The lift and drag coefficients at angle of attack <paramref name="alpha"/> (radians, any value; the model is periodic through the sine and cosine).</summary>
    public readonly void Evaluate(float alpha, out float cl, out float cd)
    {
        float x = alpha - ZeroLiftAngle;
        float clLin = LiftSlope * x;
        float cdLin = MinDrag + InducedDrag * clLin * clLin;
        // Past the stall the surface acts as a flat plate: normal force C_D90 sin(alpha) resolved into lift and drag.
        var (s, c) = Math32.SinCos(alpha);
        float clPlate = PlateDrag * s * c;
        float cdPlate = MinDrag + PlateDrag * s * s;
        float t = (Math.Abs(x) - StallAngle) / StallBlend;
        t = t < 0f ? 0f : t > 1f ? 1f : t;
        float w = t * t * (3f - 2f * t);
        cl = clLin + w * (clPlate - clLin);
        cd = cdLin + w * (cdPlate - cdLin);
    }

    /// <summary>
    /// The force on a surface of area <paramref name="area"/> in a flow of density <paramref name="density"/> whose velocity relative to the surface has components <paramref name="flowAlongChord"/> and <paramref name="flowAlongNormal"/>
    /// (the chord points from the leading edge to the trailing edge, so a flow along +chord is the normal one). Returns the components of the force along the chord and along the normal, newtons.
    /// F = q·S·(C<sub>D</sub>·û + C<sub>L</sub>·û<sup>⊥</sup>) with q = ½ρ(u<sub>c</sub>² + u<sub>n</sub>²), û the flow direction in the chord–normal plane and û<sup>⊥</sup> that direction turned a quarter turn towards the normal.
    /// </summary>
    public readonly void Force(float flowAlongChord, float flowAlongNormal, float area, float density, out float forceChord, out float forceNormal)
    {
        float v2 = flowAlongChord * flowAlongChord + flowAlongNormal * flowAlongNormal;
        if (!(v2 > 1e-12f)) { forceChord = 0f; forceNormal = 0f; return; }
        float alpha = Math32.Atan2(flowAlongNormal, flowAlongChord);
        Evaluate(alpha, out float cl, out float cd);
        float v = float.Sqrt(v2);
        float cosA = flowAlongChord / v, sinA = flowAlongNormal / v;
        float qs = 0.5f * density * v2 * area;
        forceChord = qs * (cd * cosA - cl * sinA);
        forceNormal = qs * (cd * sinA + cl * cosA);
    }
}
