using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
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
                { if (SnapshotRegions.Inside(d.Pos.AsSpan(3 * p, 3), min, max)) { s.Add("softbody", i); break; } }
            }
        }
        // Fluids are world fields: keep them all, even when their volume lies outside the seed box.
        for (int i = 0; i < FluidHandles.HighWater; i++) { if (FluidHandles.IsLive(i)) { s.Add("fluid", i); } }
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
                double minX = d.Pos[0], minY = d.Pos[1], minZ = d.Pos[2], maxX = minX, maxY = minY, maxZ = minZ;
                for (int p = 1; p < d.N; p++)
                {
                    minX = Math.Min(minX, d.Pos[3 * p]); maxX = Math.Max(maxX, d.Pos[3 * p]);
                    minY = Math.Min(minY, d.Pos[3 * p + 1]); maxY = Math.Max(maxY, d.Pos[3 * p + 1]);
                    minZ = Math.Min(minZ, d.Pos[3 * p + 2]); maxZ = Math.Max(maxZ, d.Pos[3 * p + 2]);
                }
                minX -= d.Radius; minY -= d.Radius; minZ -= d.Radius; maxX += d.Radius; maxY += d.Radius; maxZ += d.Radius;
                for (int sh = 0; sh < Core.Shapes.HighWater; sh++)
                {
                    if (!Core.Shapes.Handles.IsLive(sh)) { continue; }
                    int b = Core.Shapes.Owner[sh];
                    if (!s.Contains("softbody", i) && (!s.Bodies[b] || !Core.Bodies.IsDynamic(b))) { continue; }
                    var box = TightBox(b, sh);
                    if (box.MinX > maxX || box.MaxX < minX || box.MinY > maxY || box.MaxY < minY || box.MinZ > maxZ || box.MaxZ < minZ) { continue; }
                    if (s.Contains("softbody", i)) { changed |= s.AddBody(b); }
                    if (s.Bodies[b] && Core.Bodies.IsDynamic(b)) { changed |= s.Add("softbody", i); }
                }
            }

            // Sensor overlaps live in the pair table rather than the constraint store. They need the same anchor closure.
            for (int p = 0; p < PairHigh; p++)
            {
                if (!PairLive[p] || PairCons[p] != SensorOverlap) { continue; }
                int a = Core.Shapes.Owner[PairA[p]], b = Core.Shapes.Owner[PairB[p]];
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
                foreach (int b in d.GroundBody) { changed |= s.AddBody(b); }
            }
            for (int i = 0; i < CharHandles.HighWater; i++)
            {
                if (!CharHandles.IsLive(i)) { continue; }
                if (s.Bodies[CharBody[i]] || s.Contains("character", i))
                { s.Add("character", i); changed |= s.AddBody(CharBody[i]); changed |= s.AddBody(CharGroundBody[i]); }
            }
        } while (changed);
        SnapshotRegions.Complete(Core, s);
        for (int i = 0; i < Core.Shapes.HighWater; i++)
        { if (s.Contains("shape", i)) { s.Add("geometry", GeomOf[i]); } }
        for (int i = 0; i < PairHigh; i++)
        { if (PairLive[i] && s.Contains("shape", PairA[i]) && s.Contains("shape", PairB[i])) { s.Add("pair", i); } }
        return s;
    }

    internal void RefreshRegionProxies(RegionSelection selection)
    {
        for (int b = 0; b < selection.Bodies.Length; b++)
        {
            if (!selection.Bodies[b]) { continue; }
            for (int shape = ShapeHead[b]; shape >= 0; shape = NextShape[shape])
            {
                Bp.MoveProxy(Proxy[shape], TightBox(b, shape), 0.0, 0.0, 0.0);
            }
        }
    }
}
