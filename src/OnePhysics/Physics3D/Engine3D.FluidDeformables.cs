using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    void IFluidHost.GatherDeformables(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, FluidDeformables into) { }

    void IFluidHost.ApplyDeformables(FluidDeformables from) { }
}
