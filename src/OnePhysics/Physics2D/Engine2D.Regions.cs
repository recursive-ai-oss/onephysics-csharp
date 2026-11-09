using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    internal RegionSelection SelectRegion(double[] min, double[] max, RegionSnapshot? archive = null)
    {
        var s = SnapshotRegions.Seed(Core, min, max, archive);
        if (archive == null)
        {
            for (int i = 0; i < SoftHandles.HighWater; i++)
            {
                if (!SoftHandles.IsLive(i)) { continue; }
                var d = SoftData[i]!;
                for (int p = 0; p < d.N; p++)
                { if (SnapshotRegions.Inside(d.Pos.AsSpan(2 * p, 2), min, max)) { s.Add("softbody", i); break; } }
            }
        }

        bool changed;
        do
        {
            changed = SnapshotRegions.CloseConstraints(Core, s);
            // Conservative soft/rigid coupling closure. A selected soft body keeps every rigid shape touching its particle bounds;
            // a selected mass-bearing rigid body keeps touching soft bodies. Static anchors never fan out to unrelated soft bodies.
            for (int i = 0; i < SoftHandles.HighWater; i++)
            {
                if (!SoftHandles.IsLive(i)) { continue; }
                var d = SoftData[i]!;
                double minX = d.Pos[0], minY = d.Pos[1], maxX = minX, maxY = minY;
                for (int p = 1; p < d.N; p++)
                {
                    minX = Math.Min(minX, d.Pos[2 * p]); maxX = Math.Max(maxX, d.Pos[2 * p]);
                    minY = Math.Min(minY, d.Pos[2 * p + 1]); maxY = Math.Max(maxY, d.Pos[2 * p + 1]);
                }

                minX -= d.Radius; minY -= d.Radius; maxX += d.Radius; maxY += d.Radius;
                for (int sh = 0; sh < Core.Shapes.HighWater; sh++)
                {
                    if (!Core.Shapes.Handles.IsLive(sh)) { continue; }
                    int b = Core.Shapes.Owner[sh];
                    if (!s.Contains("softbody", i) && (!s.Bodies[b] || !Core.Bodies.IsDynamic(b))) { continue; }
                    var box = TightBox(b, in Geom[sh]);
                    if (box.MinX > maxX || box.MaxX < minX || box.MinY > maxY || box.MaxY < minY) { continue; }
                    if (s.Contains("softbody", i)) { changed |= s.AddBody(b); }
                    if (s.Bodies[b] && Core.Bodies.IsDynamic(b)) { changed |= s.Add("softbody", i); }
                }
            }

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
