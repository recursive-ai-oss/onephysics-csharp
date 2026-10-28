using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Raw-tier buoyancy geometry: how much of a convex shape lies below a plane. Spheres use the closed form for a spherical cap; boxes and hulls are cut exactly (their convex radius is ignored);
/// cylinders, capsules and tapered capsules are cut as 24-sided polyhedra with a volume within about 1 % of the true one. All arithmetic is float32 with IEEE-exact operations, so results are the same on every platform.
/// </summary>
public static class Buoyancy3D
{
    private const float Pi = 3.1415927f;

    /// <summary>Measures the part of a convex shape below a plane.</summary>
    /// <param name="shape">The shape.</param>
    /// <param name="pose">Where the shape is, in the frame the plane is given in.</param>
    /// <param name="surfaceNormal">Unit vector pointing out of the fluid (up, for water).</param>
    /// <param name="planeOffset">The plane is the set of points p with <c>dot(surfaceNormal, p) + planeOffset = 0</c>; points with a positive value are above the surface.</param>
    /// <param name="flowDirection">Optional unit direction; when given, <see cref="SubmergedVolume3D.ProjectedArea"/> is the submerged silhouette seen along it.</param>
    /// <returns>The submerged volume, its centroid and area; a zero volume when the shape is clear of the fluid. This overload builds the polyhedron on every call (it allocates): a world caches it for its own bodies.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The normal, offset, pose or direction has a NaN or infinite component, or the normal is not close to unit length.</exception>
    public static SubmergedVolume3D Submerged(in ConvexShape3D shape, in RigidTransformF pose, Vector3F surfaceNormal, float planeOffset, Vector3F flowDirection = default)
    {
        CheckFinite(surfaceNormal, nameof(surfaceNormal));
        CheckFinite(flowDirection, nameof(flowDirection));
        CheckFinite(pose.Translation, nameof(pose));
        if (!float.IsFinite(planeOffset)) { throw new ArgumentOutOfRangeException(nameof(planeOffset), planeOffset, "The plane offset must be finite."); }
        float l2 = surfaceNormal.X * surfaceNormal.X + surfaceNormal.Y * surfaceNormal.Y + surfaceNormal.Z * surfaceNormal.Z;
        if (!(l2 > 0.99f && l2 < 1.01f)) { throw new ArgumentOutOfRangeException(nameof(surfaceNormal), surfaceNormal, "The surface normal must be a unit vector."); }
        CutResult r;
        if (shape.Kind == ConvexShapeKind3D.Sphere)
        {
            if (!CutSphere(shape.ConvexRadius, pose.Translation, surfaceNormal, planeOffset, out r)) { return default; }
        }
        else
        {
            var mesh = BuoyancyMesh.For(shape);
            if (mesh == null || !new BuoyancyCutter().Cut(mesh, pose, surfaceNormal, planeOffset, flowDirection, out r)) { return default; }
        }

        return new SubmergedVolume3D(r.Volume, r.Centroid, r.ProjectedArea);
    }

    private static void CheckFinite(Vector3F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { throw new ArgumentOutOfRangeException(name, v, "Every component must be finite."); }
    }

    /// <summary>A sphere cut by a plane: the volume of the spherical cap below it, with the cap's centroid, and an estimate of the silhouette from the volume fraction.</summary>
    internal static bool CutSphere(float radius, Vector3F centre, Vector3F normal, float planeOffset, out CutResult result)
    {
        result = default;
        float s = normal.X * centre.X + normal.Y * centre.Y + normal.Z * centre.Z + planeOffset;   // signed height of the centre above the surface
        float h = radius - s;                                                                       // depth of the lowest point below the surface
        if (!(h > 0f)) { return false; }
        if (h > 2f * radius) { h = 2f * radius; }
        float full = 4f / 3f * Pi * radius * radius * radius;
        float vol = Pi * h * h * (3f * radius - h) / 3f;
        float t = 2f * radius - h;
        float down = 3f * t * t / (4f * (3f * radius - h));   // the cap's centroid lies this far below the sphere's centre, along the normal
        result.Volume = vol;
        result.Centroid = new Vector3F(centre.X - normal.X * down, centre.Y - normal.Y * down, centre.Z - normal.Z * down);
        // The silhouette of a cap is not a closed form in a direction-free way; (V / V_full)^(2/3) of the full disc is exact for a whole sphere and close for a half.
        float f = vol / full;
        float c = Math32.Cbrt(f);
        result.ProjectedArea = Pi * radius * radius * c * c;
        return true;
    }
}
