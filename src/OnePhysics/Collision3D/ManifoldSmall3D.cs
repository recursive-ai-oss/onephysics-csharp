using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class ManifoldGenerator3D
{
    // Four or fewer candidates need no geometric reduction. Adjacent strict swaps are
    // stable even when feature ids coincide, matching the general insertion sort.
    private Manifold3D FinishSmall(Vector3F normal, int count)
    {
        OutPt a = _pts[0], b = default, c = default, d = default;
        if (count > 1)
        {
            b = _pts[1];
            if (a.Id > b.Id) { (a, b) = (b, a); }
        }
        if (count > 2)
        {
            c = _pts[2];
            if (b.Id > c.Id) { (b, c) = (c, b); }
            if (a.Id > b.Id) { (a, b) = (b, a); }
        }
        if (count > 3)
        {
            d = _pts[3];
            if (c.Id > d.Id) { (c, d) = (d, c); }
            if (b.Id > c.Id) { (b, c) = (c, b); }
            if (a.Id > b.Id) { (a, b) = (b, a); }
        }
        return new Manifold3D(UnitNormal(normal), count,
            new ManifoldPoint3D(a.P, a.Sep, a.Id),
            count > 1 ? new ManifoldPoint3D(b.P, b.Sep, b.Id) : default,
            count > 2 ? new ManifoldPoint3D(c.P, c.Sep, c.Id) : default,
            count > 3 ? new ManifoldPoint3D(d.P, d.Sep, d.Id) : default);
    }
}
