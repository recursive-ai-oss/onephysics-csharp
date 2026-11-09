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

            for (int i = 0; i < RagHandles.HighWater; i++)
            {
                if (!RagHandles.IsLive(i)) { continue; }
                var d = Ragdolls[i]!;
                bool selected = s.Contains("ragdoll", i);
                foreach (int b in d.Body) { selected |= s.Bodies[b]; }
                if (!selected) { continue; }
                s.Add("ragdoll", i);
                foreach (int b in d.Body) { changed |= s.AddBody(b); }
            }
            for (int i = 0; i < VehHandles.HighWater; i++)
            {
                if (!VehHandles.IsLive(i)) { continue; }
                var d = Vehicles[i]!;
                if (!s.Bodies[d.Body] && !s.Contains("vehicle", i)) { continue; }
                s.Add("vehicle", i); changed |= s.AddBody(d.Body);
                foreach (int b in d.GroundBody) { if (b >= 0) { changed |= s.AddBody(b); } }
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

    internal void CanonicalizeBroadphase()
    {
        // Validate all tight/fat boxes before discarding any tree state.
        ReadOnlySpan<double> zero = stackalloc double[2];
        CheckShift(zero);
        Bp.ResetForCanonicalization();
        for (int shape = 0; shape < Core.Shapes.HighWater; shape++)
        {
            if (!Core.Shapes.Handles.IsLive(shape)) { continue; }
            int b = Core.Shapes.Owner[shape];
            Proxy[shape] = Bp.CreateProxy(Core.Bodies.TypeOf(b), TightBox(b, in Geom[shape]), shape);
        }
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
