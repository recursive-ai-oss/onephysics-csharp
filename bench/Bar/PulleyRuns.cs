using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;

namespace Bar;

internal static class PulleyRuns
{
    // Thresholds are ours (no managed engine runs pulley systems): 1.5 times the worst value measured on x64 and arm64, rounded up
    // to a 1/1.5/2/3/5/7 step with floors of 1 mm stretch and 1 mm penetration. Speed is the fastest rigid body; stretch is the largest
    // overstretch (total path length minus rope) of a loaded system; length drift is the change of the rope's material length (conserved
    // exactly up to rounding); tunnelled counts bodies below the floor plus broken systems and must be zero.
    private static (double Speed,double Stretch,double Penetration) Limits(string scene)=>scene switch
    {
        "tackle-2d"=>(1.5,.001,.001),
        "belt-2d"=>(.5,.001,.001),
        "chain-drive-3d"=>(5,.001,.001),
        "crane-3d"=>(2,.007,.001),
        _=>(50,.05,.03)
    };
    internal static int Run(Opts options)
    {
        string scene=options.Get("scene","tackle-2d");int threads=int.Parse(options.Get("threads","1"),CultureInfo.InvariantCulture);
        int steps=int.Parse(options.Get("steps","600"),CultureInfo.InvariantCulture);bool timing=options.Has("indicative");
        Console.WriteLine($"scene={scene} threads={threads} steps={steps} runtime={RuntimeInformation.FrameworkDescription} os={RuntimeInformation.OSDescription}");
        var fixture=PulleyScene.Create(scene,threads);
        bool trace=options.Has("trace");
        double speed=0,stretch=0,penetration=0,drift=0;int tunnelled=0;
        var samples=new long[steps];
        for(int step=0;step<steps;step++)
        {
            long before=timing?Stopwatch.GetTimestamp():0;fixture.Step(step);if(timing)samples[step]=Stopwatch.GetTimestamp()-before;
            if(trace&&step<4)Console.WriteLine($"trace step {step}: {fixture.Trace()} hash={fixture.Hash():x16}");
            speed=Math.Max(speed,fixture.MaxSpeed());stretch=Math.Max(stretch,fixture.MaxLoadedStretch());penetration=Math.Max(penetration,fixture.Penetration());drift=Math.Max(drift,fixture.LengthDrift());tunnelled=Math.Max(tunnelled,fixture.Tunnelled());
        }
        var limit=Limits(scene);
        bool stable=double.IsFinite(speed)&&double.IsFinite(stretch)&&speed<=limit.Speed&&stretch<=limit.Stretch&&penetration<=limit.Penetration&&drift<=1e-9&&tunnelled==0;
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"metrics maxSpeed={speed:R} maxOverstretch={stretch:R} lengthDrift={drift:R} maxPenetration={penetration:R} tunnelled={tunnelled} stable={stable} limits=(speed {limit.Speed:R}, stretch {limit.Stretch:R}, penetration {limit.Penetration:R}, drift 1e-9) {fixture.Extra()} hash={fixture.Hash():x16}"));
        if(timing)
        {
            var measured=samples.Skip(Math.Min(30,steps/2)).OrderBy(x=>x).ToArray();double ms=1000.0/Stopwatch.Frequency;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"indicative stepMedianMs={measured[measured.Length/2]*ms:R} stepMinMs={measured[0]*ms:R} managedMemoryBytes={GC.GetTotalMemory(false)} peakWorkingSetBytes={Process.GetCurrentProcess().PeakWorkingSet64}"));
        }
        return stable?0:1;
    }
}
