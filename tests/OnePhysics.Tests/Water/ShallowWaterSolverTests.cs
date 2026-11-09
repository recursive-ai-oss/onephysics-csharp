using System;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Water;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Water;

/// <summary>The shallow-water scheme against closed-form answers: Stoker's dam break, a lake at rest over uneven ground, conservation, the wave speed.</summary>
public class ShallowWaterSolverTests
{
    private const float G = 9.81f;

    private static WaterGrid Planar(int nx, float dx, Func<double, double> bed, Func<double, double> level, WaterEdge west = WaterEdge.Wall, WaterEdge east = WaterEdge.Wall)
    {
        var g = new WaterGrid { Planar = true, Nx = nx, Nz = 1, Dx = dx, Cfl = 0.25f, MaxSubsteps = 64 };
        g.Edges[0] = west; g.Edges[1] = east;
        g.Allocate();
        for (int i = 0; i < nx; i++) { g.Bed[i] = Quantize(bed((i + 0.5) * dx)); }
        g.Derive();
        for (int i = 0; i < nx; i++) { g.H[i] = Math.Max((float)(level((i + 0.5) * dx) - g.Bed[i]), 0f); }
        return g;
    }

    // Bed heights on a binary grid, so that depth = level - bed and surface = depth + bed are exact in float and a lake at rest starts exactly level.
    private static float Quantize(double v) => (float)(Math.Round(v * 256.0) / 256.0);

    private static void Run(WaterGrid g, double seconds, float dt = 1f / 60f)
    {
        var ex = new TaskExecutor(1);
        int n = (int)Math.Round(seconds / dt);
        for (int i = 0; i < n; i++) { g.Advance(dt, G, ex); }
    }

    // Stoker: the plateau depth hm solves  u_m = 2 (sqrt(g hL) - sqrt(g hm)) = (hm - hR) sqrt(g (hm + hR) / (2 hm hR)).
    private static void Stoker(double hl, double hr, out double hm, out double um, out double shock)
    {
        double lo = Math.Sqrt(G * hr), hi = Math.Sqrt(G * hl);
        double Fn(double cm) { double h = cm * cm / G; return 2 * (Math.Sqrt(G * hl) - cm) - (h - hr) * Math.Sqrt(G * (h + hr) / (2 * h * hr)); }
        for (int i = 0; i < 200; i++) { double mid = 0.5 * (lo + hi); if (Fn(mid) > 0) { lo = mid; } else { hi = mid; } }
        double c = 0.5 * (lo + hi);
        hm = c * c / G; um = 2 * (Math.Sqrt(G * hl) - c); shock = hm * um / (hm - hr);
    }

    [Fact]
    public void DamBreakOnAWetBedMatchesStoker()
    {
        const double hl = 1.0, hr = 0.2, x0 = 50.0, t = 5.0;
        var g = Planar(2000, 0.05f, _ => 0.0, x => x < x0 ? hl : hr, WaterEdge.Open, WaterEdge.Open);
        Run(g, t);
        Stoker(hl, hr, out double hm, out double um, out double shock);
        double cm = Math.Sqrt(G * hm);
        // The plateau lies between the rarefaction's tail and the bore.
        double tail = x0 + (um - cm) * t, front = x0 + shock * t;
        int mid = (int)(0.5 * (tail + front) / 0.05);
        Assert.InRange(g.H[mid], hm * 0.95, hm * 1.05);
        // The bore: the position where the depth crosses halfway between the plateau and the dry-side depth.
        double target = 0.5 * (hm + hr);
        int i = mid;
        while (i < 1999 && g.H[i] > target) { i++; }
        double xFront = (i + 0.5) * 0.05;
        Assert.InRange(xFront - x0, (shock * t) * 0.95, (shock * t) * 1.05);
        // And the plateau velocity.
        Assert.InRange(g.Hu[mid] / g.H[mid], um * 0.95, um * 1.05);
    }

    [Fact]
    public void ALakeAtRestOverUnevenGroundStaysAtRest()
    {
        // A bumpy bed with a bump poking above the water: wet, dry and partly wet cells all at rest.
        double Bed(double x) => 0.3 * Math.Sin(x * 0.7) + 0.25 * Math.Cos(x * 2.3) + 0.3 + (x > 30 && x < 34 ? 1.2 : 0.0);
        var g = Planar(200, 0.25f, Bed, _ => 1.25);
        var ex = new TaskExecutor(1);
        float worst = 0f; int wc = -1, ws = -1;
        for (int i = 0; i < 6000; i++)
        {
            g.Advance(1f / 60f, G, ex);
            for (int c = 0; c < g.H.Length; c++)
            {
                if (g.H[c] > WaterGrid.DryDepth && Math.Abs(g.Hu[c]) / g.H[c] > worst) { worst = Math.Abs(g.Hu[c]) / g.H[c]; wc = c; ws = i; }
            }
        }

        Assert.True(worst < 1e-6f, "largest speed " + worst + " in cell " + wc + " (h " + (wc >= 0 ? g.H[wc] : 0) + ", bed " + (wc >= 0 ? g.BedOfCell(wc, 0) : 0) + ") at step " + ws);
    }

    [Fact]
    public void ALakeAtRestInTwoDimensionsStaysAtRest()
    {
        var g = new WaterGrid { Nx = 40, Nz = 32, Dx = 0.5f, Cfl = 0.25f, MaxSubsteps = 16 };
        g.Allocate();
        for (int k = 0; k < g.Nz; k++) { for (int i = 0; i < g.Nx; i++) { g.Bed[k * g.Nx + i] = Quantize(0.4 * Math.Sin(i * 0.5) * Math.Cos(k * 0.4) + 0.5 + (i > 10 && i < 14 && k > 8 && k < 12 ? 1.5 : 0.0)); } }
        g.Derive();
        g.Fill(1.5f);
        var ex = new TaskExecutor(1);
        float worst = 0f;
        for (int s = 0; s < 3000; s++)
        {
            g.Advance(1f / 60f, G, ex);
            for (int c = 0; c < g.H.Length; c++) { if (g.H[c] > WaterGrid.DryDepth) { worst = Math.Max(worst, Math.Max(Math.Abs(g.Hu[c]), Math.Abs(g.Hv[c])) / g.H[c]); } }
        }

        Assert.True(worst < 1e-6f, "largest speed " + worst);
    }

    [Fact]
    public void ClosedWaterKeepsItsVolume()
    {
        var g = Planar(400, 0.1f, x => 0.2 * Math.Sin(x), x => x < 15 ? 1.5 : 0.4);
        double v0 = g.Volume();
        Run(g, 30);
        Assert.InRange(g.Volume() / v0, 1 - 2e-5, 1 + 2e-5);
        Assert.Equal(0.0, g.LostVolume);
    }

    [Fact]
    public void ClosedWaterKeepsItsVolumeInTwoDimensions()
    {
        var g = new WaterGrid { Nx = 48, Nz = 40, Dx = 0.25f, Cfl = 0.25f, MaxSubsteps = 16 };
        g.Allocate(); g.Derive();
        for (int k = 0; k < g.Nz; k++) { for (int i = 0; i < g.Nx; i++) { g.H[k * g.Nx + i] = (i - 12) * (i - 12) + (k - 20) * (k - 20) < 36 ? 1.8f : 0.5f; } }
        double v0 = g.Volume();
        var ex = new TaskExecutor(1);
        for (int s = 0; s < 1200; s++) { g.Advance(1f / 60f, G, ex); }
        Assert.InRange(g.Volume() / v0, 1 - 2e-5, 1 + 2e-5);
    }

    [Fact]
    public void SmallWavesTravelAtRootGH()
    {
        const double depth = 2.0, x0 = 100.0, t = 8.0;
        var g = Planar(1600, 0.125f, _ => 0.0, x => depth + 0.01 * Math.Exp(-(x - x0) * (x - x0) / (2 * 1.5 * 1.5)), WaterEdge.Open, WaterEdge.Open);
        Run(g, t);
        double expect = Math.Sqrt(G * depth) * t;
        int best = 0; float peak = 0f;
        for (int i = (int)(x0 / 0.125) + 20; i < 1600; i++) { if (g.H[i] > peak) { peak = g.H[i]; best = i; } }
        double travelled = (best + 0.5) * 0.125 - x0;
        Assert.InRange(travelled, expect * 0.98, expect * 1.02);
    }

    [Fact]
    public void ADryBedStaysDryAndAFrontRunsOverIt()
    {
        // Water released on one side of a dry flat bed never goes negative and conserves its volume.
        var g = Planar(300, 0.1f, _ => 0.0, x => x < 5 ? 1.0 : 0.0);
        double v0 = g.Volume();
        Run(g, 12);
        foreach (float h in g.H) { Assert.True(h >= 0f); }
        Assert.InRange(g.Volume() / v0, 1 - 1e-5, 1 + 1e-5);
        Assert.True(g.H[250] > WaterGrid.DryDepth, "the front must have reached x = 25 m");
    }

    [Fact]
    public void ResultsDoNotDependOnTheThreadCount()
    {
        WaterGrid Make()
        {
            var g = new WaterGrid { Nx = 64, Nz = 64, Dx = 0.25f, Cfl = 0.25f, MaxSubsteps = 16 };
            g.Allocate();
            for (int k = 0; k < 64; k++) { for (int i = 0; i < 64; i++) { g.Bed[k * 64 + i] = 0.125f * (i % 7) * (k % 5 > 2 ? 1f : 0.5f); } }
            g.Derive();
            for (int k = 0; k < 64; k++) { for (int i = 0; i < 64; i++) { g.H[k * 64 + i] = (i - 20) * (i - 20) + (k - 30) * (k - 30) < 80 ? 2f : 1f; } }
            return g;
        }

        var a = Make(); var b = Make();
        var e1 = new TaskExecutor(1); var e8 = new TaskExecutor(8);
        for (int s = 0; s < 300; s++) { a.Advance(1f / 60f, G, e1); b.Advance(1f / 60f, G, e8); }
        for (int c = 0; c < a.H.Length; c++)
        {
            Assert.Equal(BitConverter.SingleToInt32Bits(a.H[c]), BitConverter.SingleToInt32Bits(b.H[c]));
            Assert.Equal(BitConverter.SingleToInt32Bits(a.Hu[c]), BitConverter.SingleToInt32Bits(b.Hu[c]));
            Assert.Equal(BitConverter.SingleToInt32Bits(a.Hv[c]), BitConverter.SingleToInt32Bits(b.Hv[c]));
        }
    }

    [Fact]
    public void InflowFillsTheGrid()
    {
        var g = Planar(100, 0.1f, _ => 0.0, _ => 0.0, WaterEdge.Inflow, WaterEdge.Wall);
        g.EdgeLevel[0] = 0.5f; g.EdgeSpeed[0] = 1f;
        Run(g, 4);
        Assert.True(g.Volume() > 0.5);
        Assert.True(g.H[0] > 0.3f);
    }
}
