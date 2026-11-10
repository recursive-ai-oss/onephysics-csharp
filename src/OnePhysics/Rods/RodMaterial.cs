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
    internal void Validate() { _ = new RodMaterial(YoungModulus, ShearModulus, Radius, Density); }
    internal static void Check(float value, float minimum, float maximum, string name)
    {
        if (!float.IsFinite(value) || value < minimum || value > maximum)
            throw new ArgumentOutOfRangeException(name, value, $"Expected a finite value in [{minimum}, {maximum}].");
    }
}
