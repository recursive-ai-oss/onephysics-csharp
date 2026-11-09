using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class FieldScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/fields",ctx=>
        {
            var w=FieldScenes.Planar(ctx.Threads);
            for(int i=0;i<180;i++)
            {
                if(i==40)w.Explode(new(0,1),0.1f,20);
                if(i==60){var s=w.Snapshot();w=w.Fork();w.Restore(s);ctx.Write(w.Snapshot());}
                if(i==120)w.ShiftOrigin(new(16,-8));
                w.Step(1f/60);ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/fields",ctx=>
        {
            var w=FieldScenes.Spatial(ctx.Threads);
            for(int i=0;i<180;i++)
            {
                if(i==40)w.Explode(new(0,1,0),0.001f,20);
                if(i==60){var s=w.Snapshot();w=w.Fork();w.Restore(s);ctx.Write(w.Snapshot());}
                if(i==120)w.ShiftOrigin(new(16,-8,2));
                w.Step(1f/60);ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
    }
}
