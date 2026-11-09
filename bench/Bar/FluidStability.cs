#pragma warning disable OPHYS0002, OPHYS0003
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// Stability rows for the particle-fluid scenes (no reference engine runs them; the thresholds are ours, 1.5 times the worst of the x64 and arm64 values, rounded up). Over a fixed 600 steps, sampled every 20:
///  maxSpeed (m/s, any particle), densityOver95 (the 95th percentile of the compression of the density constraint, rho / rest - 1, worst sample after the first second), finalSpeed (fastest particle at the end),
///  escapes (particles outside the scene's bounds), nan, penetrating (particles a shape overlaps by more than a quarter of the contact radius) and the scene's own measures:
///  leaked (fluid-cloth-3d: particles below the sheet), floatDraftError and sunkHeight (fluid-boxes-3d: the light box's waterline error against its density ratio, m; the heavy box's bottom above the floor, m).
/// </summary>
internal static class FluidStability
{
    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    // maxSpeed, densityOver95, finalSpeed
    static readonly Dictionary<string, (double Speed, double Density, double Final)> Limits = new()
    {
        ["dam-break-2d"] = (6, 0.1, 1.5),
        ["dam-break-3d"] = (4, 0.02, 0.5),
        ["fluid-boxes-3d"] = (2, 0.04, 1),
        ["fluid-cloth-3d"] = (8, 0.03, 1),
        ["fluid-rope-2d"] = (12, 0.15, 4),
    };

    internal static int Run(Opts opts)
    {
        string only = opts.Get("scene", "");
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        float spacing = float.Parse(opts.Get("spacing", "0"), CultureInfo.InvariantCulture);
        int fails = 0;
        foreach (var name in FluidScenes.Names)
        {
            if (only.Length != 0 && only != name) continue;
            fails += RunScene(name, threads, spacing);
        }

        return fails;
    }

    static int RunScene(string name, int threads, float spacing)
    {
        bool d3 = FluidScenes.Is3D(name);
        var w3 = d3 ? FluidScenes.Build3D(name, threads, spacing) : null;
        var w2 = d3 ? null : FluidScenes.Build2D(name, threads, spacing);
        var ex = FluidScenes.ExtrasOf((object?)w3 ?? w2!);
        double maxSpeed = 0, dens = 0, finalSpeed = 0;
        int escapes = 0, nan = 0, pen = 0;
        double stepMs = 0;
        var sw = new Stopwatch();
        for (int i = 0; i < 600; i++)
        {
            sw.Restart();
            if (d3) w3!.Step(FluidScenes.Dt); else w2!.Step(FluidScenes.Dt);
            stepMs += sw.Elapsed.TotalMilliseconds;
            if (i % 20 != 19) continue;
            var m = d3 ? FluidScenes.Measure3D(w3!, name) : FluidScenes.Measure2D(w2!, name);
            maxSpeed = Math.Max(maxSpeed, m.MaxSpeed);
            if (i >= 59) dens = Math.Max(dens, m.DensityOver95);
            escapes = Math.Max(escapes, m.Escapes); nan = Math.Max(nan, m.NonFinite); pen = Math.Max(pen, m.Penetrating);
            if (i == 599) finalSpeed = m.MaxSpeed;
        }

        string extra = "";
        bool extraOk = true;
        if (name == "fluid-cloth-3d")
        {
            var cloth = ex.Cloth!.Value;
            double sheetMin = double.MaxValue;
            for (int k = 0; k < cloth.ParticleCount; k++) sheetMin = Math.Min(sheetMin, cloth.GetPosition(k).Y);
            var ps = new RecursiveAI.OnePhysics.Fluids.FluidParticleInfo[w3!.FluidParticleCount];
            w3.GetFluidParticles(ps);
            int leaked = ps.Count(p => p.Position.Y < sheetMin - 0.03);
            extra = $" leaked={leaked} sag={F(0.5 - sheetMin)}";
            extraOk = leaked == 0 && sheetMin > 0.2 && sheetMin < 0.5;
        }
        else if (name == "fluid-boxes-3d")
        {
            double lo = 0.2, hi = 0.9; float half = 0.5f * ex.Fluid.RestSphDensity;
            for (int k = 0; k < 30; k++) { double mid = 0.5 * (lo + hi); if (w3!.FluidDensityAt(new Vector3D(0.45, mid, 0.45)) > half) lo = mid; else hi = mid; }
            double draft = lo - (ex.Boxes[0].Position.Y - 0.125);
            double draftError = Math.Abs(draft - 0.075);
            double sunk = ex.Boxes[2].Position.Y - 0.125;
            extra = $" floatDraftError={F(draftError)} sunkHeight={F(sunk)}";
            extraOk = draftError < 0.05 && sunk < 0.1;
        }

        var lim = Limits[name];
        bool ok = nan == 0 && escapes == 0 && pen == 0 && maxSpeed <= lim.Speed && dens <= lim.Density && finalSpeed <= lim.Final && extraOk;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{name} steps=600 threads={threads} maxSpeed={F(maxSpeed)} densityOver95={F(dens)} finalSpeed={F(finalSpeed)} escapes={escapes} nan={nan} penetrating={pen}{extra} msPerStep={stepMs / 600:F2} {(ok ? "PASS" : "FAIL")}"));
        return ok ? 0 : 1;
    }

    /// <summary>
    /// Indicative step time of the dam break in a channel at a particle count (a cube of water of 37 particles a side by default, about 50 000): median and mean of 60 steps after 30 of warm-up, at the thread count.
    /// </summary>
    internal static int Time(Opts opts)
    {
        int threads = int.Parse(opts.Get("threads", "1"), CultureInfo.InvariantCulture);
        int side = int.Parse(opts.Get("side", "37"), CultureInfo.InvariantCulture);
        float spacing = 0.5f / (side + 0.2f);
        var w = FluidScenes.Build3D("dam-break-3d", threads, spacing);
        for (int i = 0; i < 30; i++) w.Step(FluidScenes.Dt);
        var times = new List<double>();
        var sw = new Stopwatch();
        for (int i = 0; i < 60; i++)
        {
            sw.Restart();
            w.Step(FluidScenes.Dt);
            times.Add(sw.Elapsed.TotalMilliseconds);
        }

        times.Sort();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"dam-break-3d particles={w.FluidParticleCount} threads={threads} medianMs={times[times.Count / 2]:F2} meanMs={times.Average():F2} minMs={times[0]:F2} maxMs={times[^1]:F2}"));
        return 0;
    }
}
