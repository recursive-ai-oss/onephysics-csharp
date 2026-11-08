using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    internal RegionSelection SelectRegion(double[] min, double[] max, RegionSnapshot? archive = null)
    {
        var s = SnapshotRegions.Seed(Core, min, max, archive);
        bool changed;
        do
        {
            changed = SnapshotRegions.CloseConstraints(Core, s);
            // Sensor overlaps live in the pair table rather than the constraint store. They need the same anchor closure.
            for (int p = 0; p < PairHigh; p++)
            {
                if (!Pairs[p].Live || Pairs[p].Cons != SensorOverlap) { continue; }
                int a = Core.Shapes.Owner[Pairs[p].A], b = Core.Shapes.Owner[Pairs[p].B];
                if ((s.Bodies[a] && Core.Bodies.IsDynamic(a)) || (s.Bodies[b] && Core.Bodies.IsDynamic(b)))
                { changed |= s.AddBody(a); changed |= s.AddBody(b); }
            }

            for (int i = 0; i < CharHandles.HighWater; i++)
            {
                if (!CharHandles.IsLive(i)) { continue; }
                if (s.Bodies[CharBody[i]] || s.Contains("character", i))
                { s.Add("character", i); changed |= s.AddBody(CharBody[i]); changed |= s.AddBody(CharGroundBody[i]); }
            }
        } while (changed);
        SnapshotRegions.Complete(Core, s);
        for (int i = 0; i < PairHigh; i++)
        { if (Pairs[i].Live && s.Contains("shape", Pairs[i].A) && s.Contains("shape", Pairs[i].B)) { s.Add("pair", i); } }
        return s;
    }

    internal void RefreshRegionProxies(RegionSelection selection)
    {
        for (int b = 0; b < selection.Bodies.Length; b++)
        {
            if (!selection.Bodies[b]) { continue; }
            for (int shape = ShapeHead[b]; shape >= 0; shape = NextShape[shape])
            {
                Bp.MoveProxy(Proxy[shape], TightBox(b, in Geom[shape]), 0.0, 0.0);
            }
        }
    }
}
