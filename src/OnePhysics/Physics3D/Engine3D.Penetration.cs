using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // For a nonconvex target there is no single convex MTV: report the deepest leaf,
    // with equal-depth ties on the lowest child/triangle id.
    private static bool PenetrationOf(Geom3D geometry, in ConvexShape3D mover, in RigidTransformF relative, QueryScratch scratch, out ConvexContact3D contact, out int sub)
    {
        var epa = scratch.PenetrationSolver ??= new Epa3D();
        contact = default; sub = 0;
        if (geometry.IsConvex)
        {
            contact = epa.Collide(geometry.Convex, mover, relative);
            return contact.Separation < 0f;
        }
        bool found = false;
        if (geometry.Compound is { } compound)
        {
            for (int i = 0; i < compound.SlotCount; i++)
            {
                if (!compound.IsLive(i)) { continue; }
                var child = compound.GetChild(i);
                var candidate = epa.Collide(child.Shape, mover, child.Local.Inverted() * relative);
                if (!(candidate.Separation < 0f) || (found && candidate.Separation >= contact.Separation)) { continue; }
                found = true; sub = i;
                contact = new ConvexContact3D(candidate.Separation, child.Local.Rotation.Rotate(candidate.Normal), child.Local.TransformPoint(candidate.PointA), child.Local.TransformPoint(candidate.PointB));
            }
        }
        else
        {
            var triangles = NarrowScratch.Tris;
            triangles.Clear();
            geometry.Soup!.GetCandidates(mover, relative, 0f, triangles, NarrowScratch.Ids);
            for (int i = 0; i < triangles.Count; i++)
            {
                var tri = triangles[i];
                var candidate = epa.Collide(NarrowScratch.TriBuf.Set(tri.V0, tri.V1, tri.V2), mover, relative);
                if (!(candidate.Separation < 0f) || (found && (candidate.Separation > contact.Separation || (candidate.Separation == contact.Separation && tri.Id >= sub)))) { continue; }
                found = true; sub = tri.Id; contact = candidate;
            }
        }
        return found;
    }
}
