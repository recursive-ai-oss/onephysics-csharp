using System;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>
/// The SPH kernels (Müller, Charypar, Gross 2003): poly6 for density and spiky for its gradient, with the normalisation of the dimension (3D, or the planar world's 2D).
/// Constants are computed once in double from IEEE operations and rounded to float, so every platform has the same bits.
/// </summary>
internal readonly struct FluidKernels
{
    public readonly float H, H2, Poly6, Spiky, Dq2, InvPoly6Dq;
    public readonly int Dim;

    public FluidKernels(int dim, float h)
    {
        Dim = dim; H = h; H2 = h * h;
        const double Pi = 3.14159265358979323846;
        double hd = h, h2 = hd * hd;
        if (dim == 3)
        {
            Poly6 = (float)(315.0 / (64.0 * Pi * h2 * h2 * h2 * h2 * hd));   // 315 / (64 π h^9)
            Spiky = (float)(45.0 / (Pi * h2 * h2 * h2));                      // |∇W| = 45 / (π h^6) (h - r)^2
        }
        else
        {
            Poly6 = (float)(4.0 / (Pi * h2 * h2 * h2 * h2));                  // 4 / (π h^8)
            Spiky = (float)(30.0 / (Pi * h2 * h2 * hd));                      // 30 / (π h^5)
        }

        // The artificial pressure of the tensile-instability correction compares W(r) with W at 0.2 h (Macklin and Müller 2013).
        float dq = 0.2f * h;
        Dq2 = dq * dq;
        float t = H2 - Dq2;
        InvPoly6Dq = 1f / (Poly6 * t * t * t);
    }

    /// <summary>Poly6 weight at squared distance r2 (zero beyond the radius).</summary>
    public float W(float r2)
    {
        if (!(r2 < H2)) { return 0f; }
        float t = H2 - r2;
        return Poly6 * t * t * t;
    }

    /// <summary>Sum of the poly6 weights of a cubic (square) lattice of the spacing around one of its points, itself included: the density a unit mass would see.</summary>
    public double LatticeSum(float spacing)
    {
        double s = spacing, h2 = (double)H * H, sum = 0.0;
        int n = (int)(H / spacing) + 1;
        double c = Poly6;
        for (int k = Dim == 3 ? -n : 0; k <= (Dim == 3 ? n : 0); k++)
        {
            for (int j = -n; j <= n; j++)
            {
                for (int i = -n; i <= n; i++)
                {
                    double r2 = ((double)i * i + (double)j * j + (double)k * k) * s * s;
                    if (!(r2 < h2)) { continue; }
                    double t = h2 - r2;
                    sum += c * t * t * t;
                }
            }
        }

        return sum;
    }
}
