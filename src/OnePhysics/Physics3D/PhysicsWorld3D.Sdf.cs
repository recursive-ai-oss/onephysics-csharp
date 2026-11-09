using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Adds a body with a signed distance field shape. A static body (the default) is terrain: convex shapes, characters and particles collide with the field, rays are sphere-traced.
    /// A dynamic body takes its mass, centre of mass and inertia from the region where the field is negative, scaled by <paramref name="density"/>; it collides with convex shapes and static fields, not with
    /// meshes, height fields, voxel grids or other fields (those pairs are skipped, as mesh against mesh is). Positions are doubles within +-2^21; the default rotation is the identity.
    /// Throws ArgumentNullException for a null field and the usual argument errors for a bad pose or density; a dynamic field with no inside throws ArgumentOutOfRangeException.
    /// </summary>
    public Body AddSdf(SdfGrid3D grid, Vector3D at = default, QuaternionF rotation = default, BodyType type = BodyType.Static, Material3D? material = null, float density = 1f)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddSdf");
        ArgumentNullException.ThrowIfNull(grid);
        return AddOne(type, at, rotation, ShapeSpec3D.Sdf(grid), material, type == BodyType.Static ? 0f : density);
    }

    /// <summary>Attaches a signed distance field to a body with ShapeDef defaults; see <see cref="AddSdf"/>. A field on a moving body must have an inside to take mass from.</summary>
    public Shape CreateSdf(Body body, SdfGrid3D grid, ShapeDef? def = null)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.CreateSdf");
        ArgumentNullException.ThrowIfNull(grid);
        return AddShape(body, ShapeSpec3D.Sdf(grid).Recipe.ToArray(), def);
    }
}
