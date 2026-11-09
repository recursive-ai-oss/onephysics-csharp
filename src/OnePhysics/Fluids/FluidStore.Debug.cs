using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Fluids;

internal sealed partial class FluidStore
{
    /// <summary>Emits the particles in storage order, then the emitters and drains by slot.</summary>
    public void AddDebug(DebugGeometry g)
    {
        for (int p = 0; p < Count; p++) { g.Add(new DebugFluidParticle(PId[p], PFluid[p], new Vector3D(PX[p], PY[p], PZ[p]), new Vector3F(PVx[p], PVy[p], PVz[p]))); }
        for (int e = 0; e < EmitterHandles.HighWater; e++)
        {
            if (!EmitterHandles.IsLive(e)) { continue; }
            var d = Emitters[e]!.Def;
            g.Add(new DebugFluidVolume(true, e, d.Kind, d.Position, d.Size, d.Direction, d.Enabled));
        }

        for (int k = 0; k < DrainHandles.HighWater; k++)
        {
            if (!DrainHandles.IsLive(k)) { continue; }
            var d = Drains[k]!.Def;
            g.Add(new DebugFluidVolume(false, k, d.Kind, d.Position, d.Size, default, d.Enabled));
        }
    }
}
