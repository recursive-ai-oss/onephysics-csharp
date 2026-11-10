using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;

namespace Bar;

internal static class RodRuns
{
    // Thresholds are ours (no managed engine runs rods): 1.5x the worst value measured on x64 and arm64, rounded up to a
    // 1/1.5/2/3/5/7 step with a penetration floor of 1 mm. Speed is the larger of particle and rigid body speed, stretch the
    // largest relative segment length error, penetration the deepest remaining rod contact after a step. Tunnelled counts
    // the per-scene definitions in RodScene.Tunnelled and must be zero.
    private static (double Speed,double Stretch,double Penetration) Limits(string scene)=>scene switch
    {
        "rope-whip-3d"=>(45,.005,.001),
        "rope-2d"=>(5,.05,.001),
        "rope-bridge-3d"=>(10,.05,.001),
        "hair-3d"=>(5,.15,.001),
        _=>(50,.25,.03)
    };
    internal static readonly string[] Names = ["rope-whip-3d", "rope-2d", "rope-bridge-3d", "hair-3d"];
    internal static int Run(Opts options)
    {
        string scene=options.Get("scene","rope-whip-3d");int threads=int.Parse(options.Get("threads","1"),CultureInfo.InvariantCulture);
        int steps=int.Parse(options.Get("steps","600"),CultureInfo.InvariantCulture),strands=int.Parse(options.Get("strands","1000"),CultureInfo.InvariantCulture);
        bool chain=options.Has("joint-chain"),timing=options.Has("indicative");
        Console.WriteLine($"scene={scene} model={(chain?"joint-chain":"rod")} threads={threads} steps={steps} strands={strands} runtime={RuntimeInformation.FrameworkDescription} os={RuntimeInformation.OSDescription}");
        var fixture=RodScene.Create(scene,threads,chain,strands);
        double speed=0,stretch=0,penetration=0;int tunnelled=0;
        var samples=new long[steps];
        for(int step=0;step<steps;step++)
        {
            long before=timing?Stopwatch.GetTimestamp():0;fixture.Step(step);if(timing)samples[step]=Stopwatch.GetTimestamp()-before;
            var stats=fixture.Statistics;speed=Math.Max(speed,Math.Max(stats.MaximumSpeed,fixture.BodyMaxSpeed()));stretch=Math.Max(stretch,chain?fixture.ChainStretch():stats.MaximumStretch);penetration=Math.Max(penetration,stats.MaximumPenetration);tunnelled=Math.Max(tunnelled,fixture.Tunnelled());
        }
        var limit=Limits(scene);
        bool stable=double.IsFinite(speed)&&double.IsFinite(stretch)&&speed<=limit.Speed&&stretch<=limit.Stretch&&penetration<=limit.Penetration&&tunnelled==0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"metrics maxSpeed={speed:R} maxStretch={stretch:R} maxPenetration={penetration:R} tunnelled={tunnelled} stable={stable} limits=(speed {limit.Speed:R}, stretch {limit.Stretch:R}, penetration {limit.Penetration:R}) hash={fixture.Hash():x16}"));
        if(timing)
        {
            var measured=samples.Skip(Math.Min(30,steps/2)).OrderBy(x=>x).ToArray();double ms=1000.0/Stopwatch.Frequency;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"indicative stepMedianMs={measured[measured.Length/2]*ms:R} stepMinMs={measured[0]*ms:R} managedMemoryBytes={GC.GetTotalMemory(false)} peakWorkingSetBytes={Process.GetCurrentProcess().PeakWorkingSet64}"));
        }
        return stable?0:1;
    }
}
