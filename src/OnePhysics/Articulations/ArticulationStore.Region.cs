using System;

namespace RecursiveAI.OnePhysics.Articulations;

internal sealed partial class ArticulationStore
{
    /// <summary>Moves the bases of all articulations when the world origin shifts: a fixed base or a floating base position loses the offset (link bodies are shifted by the core).</summary>
    public void Shift(D3 offset)
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var m = States[i]!.Model;
            if (m.Kind[0] == JointKind.Fixed) { m.PlacePos[0] = m.PlacePos[0] - offset; }
            else { int po = m.PosOff[0]; m.Q[po] -= offset.X; m.Q[po + 1] -= offset.Y; m.Q[po + 2] -= offset.Z; }
            m.ForwardKinematics();
        }
    }

    /// <summary>An articulation is one unit: a region that holds any of its links holds all of them. Returns whether the selection grew.</summary>
    internal bool CloseRegion(RegionSelection s)
    {
        bool changed = false;
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var st = States[i]!;
            bool any = s.Contains("articulation", i);
            for (int k = 0; k < st.Body.Length && !any; k++) { any = (uint)st.Body[k] < (uint)s.Bodies.Length && s.Bodies[st.Body[k]]; }
            if (!any) { continue; }
            changed |= s.Add("articulation", i);
            for (int k = 0; k < st.Body.Length; k++) { changed |= s.AddBody(st.Body[k]); }
        }

        return changed;
    }

    internal void SelectRegion(RegionSelection s) { while (CloseRegion(s)) { } }
}
