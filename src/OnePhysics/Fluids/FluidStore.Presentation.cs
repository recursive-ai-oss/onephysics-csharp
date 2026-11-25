using System;

namespace RecursiveAI.OnePhysics.Fluids;

internal sealed partial class FluidStore
{
    /// <summary>
    /// Removes the oldest particles (lowest ids; ids grow with creation) until at most <paramref name="keep"/> remain, keeping the relative order of the others. Used by a presentation world whose particle budget was lowered.
    /// </summary>
    internal int CullOldest(int keep)
    {
        if (keep < 0) { keep = 0; }
        if (Count <= keep) { return 0; }
        int removed = Count - keep;
        if (keep == 0) { Count = 0; _qValid = false; Wake(); return removed; }
        var ids = new int[Count];
        Array.Copy(PId, ids, Count);
        Array.Sort(ids);
        int threshold = ids[Count - keep];
        int w = 0;
        for (int p = 0; p < Count; p++)
        {
            if (PId[p] < threshold) { continue; }
            if (w != p) { MoveParticle(p, w); }
            w++;
        }

        Count = w;
        _qValid = false;
        Wake();
        return removed;
    }
}
