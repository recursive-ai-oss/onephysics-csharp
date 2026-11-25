using System.Globalization;
using RecursiveAI.OnePhysics;

namespace Bar;

using Scene = RecursiveAI.OnePhysics.Scene.Scene;

/// <summary>
/// Re-expresses a generated scene (written in metres) in another length unit, so the stability sweeps can run the standard scenes in millimetres or pixels: every length, speed, acceleration and anchor is multiplied
/// by <c>k</c> (world units per metre), densities are divided by <c>k</c> raised to the dimension so every mass stays the same, and torques by <c>k</c> squared. The scene gets the matching <see cref="LengthUnit"/>.
/// The drivers run the scaled scene and write their pose dumps converted back to metres, so the metrics and thresholds, which are written in metres, apply unchanged.
/// Set <c>BAR_UNITS_PER_METRE</c> to use it. Only what the stability scenes use is converted (rigid shapes, revolute, point and swing-twist joints); other scenes are refused.
/// </summary>
static class UnitScaling
{
    /// <summary>World units per metre from the environment, or 1 when not set.</summary>
    public static double FromEnvironment()
    {
        string? v = Environment.GetEnvironmentVariable("BAR_UNITS_PER_METRE");
        return string.IsNullOrEmpty(v) ? 1.0 : double.Parse(v, CultureInfo.InvariantCulture);
    }

    public static void Apply(Scene s, double k)
    {
        if (k == 1.0) return;
        int dim = s.Dim;
        double dens = dim == 3 ? 1.0 / (k * k * k) : 1.0 / (k * k);
        // BAR_UNITS_IGNORE=1 rescales the scene but leaves the world at one metre per unit: the negative control that shows what the unit changes.
        if (Environment.GetEnvironmentVariable("BAR_UNITS_IGNORE") != "1") s.LengthUnit = new LengthUnit(1.0 / k);
        s.Gx *= k; s.Gy *= k; s.Gz *= k;
        if (s.Softs.Count != 0 || s.Queries.Count != 0 || s.Params.ContainsKey("engines")) throw new NotSupportedException("unit scaling covers rigid-body stability scenes only");
        foreach (var b in s.Bodies)
        {
            if (b.Script != null || b.Vehicle != null) throw new NotSupportedException("unit scaling does not cover scripted characters or vehicles");
            b.X *= k; b.Y *= k; b.Z *= k; b.Vx *= k; b.Vy *= k; b.Vz *= k;
            foreach (var sh in b.Shapes)
            {
                sh.Radius *= k; sh.Density *= dens;
                var p = sh.P;
                switch (sh.Kind)
                {
                    case "circle": case "sphere": case "capsule": case "polygon": case "segment": case "chain": case "hull":
                        Mul(p, 0, p.Length, k); break;
                    case "box":
                        // 2D: hx hy cx cy angle; 3D: hx hy hz cx cy cz qx qy qz qw
                        Mul(p, 0, dim == 3 ? 6 : 4, k); break;
                    case "mesh":
                    {
                        int nv = (int)p[0];
                        Mul(p, 2, 3 * nv, k); break;
                    }
                    case "heightfield":
                    {
                        // nx nz cell ox oy oz h...
                        Mul(p, 2, p.Length - 2, k); break;
                    }
                    default: throw new NotSupportedException("unit scaling: shape " + sh.Kind);
                }
            }
        }
        foreach (var j in s.Joints)
        {
            if (j.Kind is not ("revolute" or "point" or "swingtwist")) throw new NotSupportedException("unit scaling: joint " + j.Kind);
            j.Ax *= k; j.Ay *= k; j.Az *= k; j.Bx *= k; j.By *= k; j.Bz *= k;
            j.MaxTorque *= k * k; j.DriveK *= k * k; j.DriveC *= k * k; j.DriveMax *= k * k;
        }
    }

    static void Mul(double[] p, int from, int count, double k) { for (int i = from; i < from + count; i++) p[i] *= k; }

    /// <summary>Converts a dumped frame back to metres: positions and linear velocities divide by k (2D rows are x y angle vx vy w asleep, 3D rows x y z qx qy qz qw vx vy vz wx wy wz asleep).</summary>
    public static void ToMetres(double[] d, int dim, double k)
    {
        if (k == 1.0) return;
        int stride = dim == 3 ? 14 : 7;
        for (int o = 0; o + stride <= d.Length; o += stride)
        {
            if (dim == 3) { d[o] /= k; d[o + 1] /= k; d[o + 2] /= k; d[o + 7] /= k; d[o + 8] /= k; d[o + 9] /= k; }
            else { d[o] /= k; d[o + 1] /= k; d[o + 3] /= k; d[o + 4] /= k; }
        }
    }
}
