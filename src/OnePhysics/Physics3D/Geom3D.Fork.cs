using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Geom3D
{
    internal Geom3D CloneForFork()
    {
        var c = (Geom3D)MemberwiseClone();
        c.Recipe = (byte[])Recipe.Clone();
        c.FastHull = (float[]?)FastHull?.Clone();
        c.Buoyancy = null;
        return c;
    }
}
