using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;

namespace Bar;

internal static class RodRuns
{
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
        bool stable=double.IsFinite(speed)&&double.IsFinite(stretch)&&speed<=50&&stretch<=.25&&penetration<=.03&&tunnelled==0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"metrics maxSpeed={speed:R} maxStretch={stretch:R} maxPenetration={penetration:R} tunnelled={tunnelled} stable={stable} hash={fixture.Hash():x16}"));
        if(timing)
        {
            var measured=samples.Skip(Math.Min(30,steps/2)).OrderBy(x=>x).ToArray();double ms=1000.0/Stopwatch.Frequency;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"indicative stepMedianMs={measured[measured.Length/2]*ms:R} stepMinMs={measured[0]*ms:R} managedMemoryBytes={GC.GetTotalMemory(false)} peakWorkingSetBytes={Process.GetCurrentProcess().PeakWorkingSet64}"));
        }
        return stable?0:1;
    }
}
