using System;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.Rods;

/// <summary>Circular, homogeneous rod material in SI units. The default struct is invalid; use a preset or the constructor.</summary>
public readonly struct RodMaterial
{
    /// <summary>Young's modulus, Pa; finite, 1 to 10^12.</summary>
    public float YoungModulus { get; }
    /// <summary>Shear modulus, Pa; finite, 1 to 10^12.</summary>
    public float ShearModulus { get; }
    /// <summary>Cross-section radius, metres; finite, 10^-5 to 10.</summary>
    public float Radius { get; }
    /// <summary>Density, kg/m³; finite, 0.01 to 100000.</summary>
    public float Density { get; }
    // World units per metre the members are expressed in; 0 stands for 1 (a material written in SI). Validation checks the SI equivalent.
    private readonly float _units;

    internal float Units => _units == 0f ? 1f : _units;

    private RodMaterial(float youngModulus, float shearModulus, float radius, float density, float units)
    {
        YoungModulus = youngModulus; ShearModulus = shearModulus; Radius = radius; Density = density; _units = units;
    }

    /// <summary>
    /// This material, written in SI as the members above state (the presets <see cref="Rope"/>, <see cref="Cable"/> and <see cref="Hair"/>), expressed for a world whose length unit is <paramref name="unit"/>: the radius is multiplied by world units
    /// per metre, the moduli (pascals, force per area) divided by it, and the density (kilograms per cubic metre) divided by its cube. A rod added to a world of that unit takes it as it is.
    /// </summary>
    /// <param name="unit">The target length unit.</param>
    public RodMaterial InUnit(LengthUnit unit)
    {
        float k = unit.Scale;
        if (k == 1f) { return this; }
        return new RodMaterial(YoungModulus / k, ShearModulus / k, Radius * k, Density / (k * k * k), k);
    }

    // A material read from a snapshot, in the world's units: checked as its SI equivalent.
    internal static RodMaterial FromWorld(float youngModulus, float shearModulus, float radius, float density, float units)
    {
        var m = new RodMaterial(youngModulus, shearModulus, radius, density, units == 1f ? 0f : units);
        m.Validate();
        return m;
    }

    /// <summary>True for the default rod material: SI <see cref="Rope"/> exactly.</summary>
    internal bool IsDefaultRope => _units == 0f && YoungModulus == 1e7f && ShearModulus == 4e6f && Radius == 0.01f && Density == 1000f;

    /// <summary>Creates a material; throws ArgumentOutOfRangeException for non-finite or out-of-range fields.</summary>
    public RodMaterial(float youngModulus, float shearModulus, float radius, float density)
    {
        Check(youngModulus, 1, 1e12f, nameof(youngModulus));
        Check(shearModulus, 1, 1e12f, nameof(shearModulus));
        Check(radius, 1e-5f, 10, nameof(radius));
        Check(density, 0.01f, 100000, nameof(density));
        YoungModulus = youngModulus; ShearModulus = shearModulus; Radius = radius; Density = density;
    }
    /// <summary>A flexible rope: E=10 MPa, G=4 MPa, radius 1 cm, density 1000 kg/m³.</summary>
    public static RodMaterial Rope => new(1e7f, 4e6f, 0.01f, 1000);
    /// <summary>A stiff cable: E=200 GPa, G=80 GPa, radius 5 mm, density 7800 kg/m³.</summary>
    public static RodMaterial Cable => new(2e11f, 8e10f, 0.005f, 7800);
    /// <summary>A hair strand: E=4 GPa, G=1.5 GPa, radius 50 µm, density 1300 kg/m³.</summary>
    public static RodMaterial Hair => new(4e9f, 1.5e9f, 5e-5f, 1300);
    internal void Validate()
    {
        float k = Units;
        if (k == 1f) { _ = new RodMaterial(YoungModulus, ShearModulus, Radius, Density); return; }
        _ = new RodMaterial(YoungModulus * k, ShearModulus * k, Radius / k, Density * (k * k * k));
    }
    internal static void Check(float value, float minimum, float maximum, string name)
    {
        if (!float.IsFinite(value) || value < minimum || value > maximum)
            throw new ArgumentOutOfRangeException(name, value, $"Expected a finite value in [{minimum}, {maximum}].");
    }
}
