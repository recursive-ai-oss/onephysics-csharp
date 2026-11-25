namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>The smallest half-width of the body's shapes, computed the way the mass update does it (a polygon's nearest side from its centroid plus the rounding radius, other shapes their radius). Static and kinematic bodies keep no stored value, so the diagnosis asks here.</summary>
    internal float ComputeMinExtent(int body)
    {
        float minExt = float.MaxValue;
        for (int s = ShapeHead[body]; s >= 0; s = NextShape[s])
        {
            ref readonly var g = ref Geom[s];
            float me = g.Radius;
            if (g.Kind == ShapeKind2D.Polygon)
            {
                var cv = g.Vertices;
                var nrm = g.Normals;
                me = float.MaxValue;
                for (int i = 0; i < cv.Length; i++)
                {
                    float d = Vec2.Dot(nrm[i], cv[i] - g.Centroid);
                    if (d < me) { me = d; }
                }

                me += g.Radius;
            }

            if (me < minExt) { minExt = me; }
        }

        return minExt;
    }
}
