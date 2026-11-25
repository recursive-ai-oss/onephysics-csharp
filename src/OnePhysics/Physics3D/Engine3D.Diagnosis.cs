namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    /// <summary>The smallest half-width of the body's shapes, computed the way the mass update does it. Static and kinematic bodies keep no stored value, so the diagnosis asks here.</summary>
    internal float ComputeMinExtent(int body)
    {
        ComputeExtents(body, default, out _, out float min);
        return min;
    }
}
