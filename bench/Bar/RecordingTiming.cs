using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

// An indicative comparison, deliberately separate from published bar rows. The same source also builds against a baseline without recording APIs.
internal static class RecordingTiming
{
    private static string Load() => File.ReadAllText("/proc/loadavg").Split(' ')[0];
    internal static int Run(Opts options)
    {
        string variant=options.Get("variant","off");
        int repetitions=int.Parse(options.Get("reps","3"),CultureInfo.InvariantCulture);
        const int warmup=60,samples=90;
#if RECORDER_TIMING
        var prime2=new PhysicsWorld2D();prime2.AddCircle(0.5f,at:new(0,4));
        using(var r=prime2.StartRecording(Stream.Null))for(int i=0;i<4;i++)prime2.Step(1f/60);
        var prime3=new PhysicsWorld3D();prime3.AddSphere(0.5f,at:new(0,4,0));
        using(var r=prime3.StartRecording(Stream.Null))for(int i=0;i<4;i++)prime3.Step(1f/60);
#endif
        Console.WriteLine("variant,scene,dimension,threads,rep,warmup,samples,mean_ms,allocated_bytes,load_before,load_during,load_after,final_hash");
        for(int rep=0;rep<repetitions;rep++)
        {
            var planar=Gen.All().First(s=>s.Name=="large-pile");var spatial=Gen3.All().First(s=>s.Name=="large-pile-3d");
            var a=SceneBuilder2D.Build(planar,1).World;var b=SceneBuilder3D.Build(spatial,1).World;
            a.Warmup();b.Warmup();
            Measure2(a,"large-pile",rep);Measure3(b,"large-pile-3d",rep);
        }
        return 0;
        void Row(string scene,int dim,int rep,long ticks,long bytes,string before,string during,string after,ulong hash)
        {
            double ms=1000.0*ticks/Stopwatch.Frequency/samples;
            Console.WriteLine(FormattableString.Invariant($"{variant},{scene},{dim},1,{rep},{warmup},{samples},{ms:R},{bytes},{before},{during},{after},{hash:x16}"));
        }
        void Measure2(PhysicsWorld2D world,string scene,int rep)
        {
            string before=Load();for(int i=0;i<warmup;i++)world.Step(1f/60);
#if RECORDER_TIMING
            using var recorder=variant=="on"?world.StartRecording(Stream.Null):null;
#else
            if(variant=="on")throw new InvalidOperationException("Baseline has no recorder.");
#endif
            long ticks=0,bytes=0;string during=before;
            for(int i=0;i<samples;i++){long n=GC.GetAllocatedBytesForCurrentThread(),t=Stopwatch.GetTimestamp();world.Step(1f/60);ticks+=Stopwatch.GetTimestamp()-t;bytes+=GC.GetAllocatedBytesForCurrentThread()-n;if(i==samples/2)during=Load();}
            Row(scene,2,rep,ticks,bytes,before,during,Load(),world.ComputeHash());
        }
        void Measure3(PhysicsWorld3D world,string scene,int rep)
        {
            string before=Load();for(int i=0;i<warmup;i++)world.Step(1f/60);
#if RECORDER_TIMING
            using var recorder=variant=="on"?world.StartRecording(Stream.Null):null;
#else
            if(variant=="on")throw new InvalidOperationException("Baseline has no recorder.");
#endif
            long ticks=0,bytes=0;string during=before;
            for(int i=0;i<samples;i++){long n=GC.GetAllocatedBytesForCurrentThread(),t=Stopwatch.GetTimestamp();world.Step(1f/60);ticks+=Stopwatch.GetTimestamp()-t;bytes+=GC.GetAllocatedBytesForCurrentThread()-n;if(i==samples/2)during=Load();}
            Row(scene,3,rep,ticks,bytes,before,during,Load(),world.ComputeHash());
        }
    }
}
