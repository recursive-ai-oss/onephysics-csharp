using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Creates a ghost-vertex chain with a material index per segment. Arrays are copied, indices are validated before creating shapes. Returns the first segment. Shape-wide Surface settings come from def; indexed materials must have default Surface settings.</summary>
    /// <exception cref="ArgumentException">Lengths disagree or an indexed material has shape-wide Surface settings.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A material index or shape parameter is invalid.</exception>
    public Shape CreateChain(Body body, ReadOnlySpan<Vector2F> points, ReadOnlySpan<Material2D> materials, ReadOnlySpan<int> materialIndices, bool loop = false, bool twoSided = true, ShapeDef? def = null)
    {
        using var mutation = RecordMutation("World.CreateMaterialChain");
        int count = loop ? points.Length : points.Length - 1;
        if (materialIndices.Length != count) throw new ArgumentException("Material index count must equal the chain segment count.", nameof(materialIndices));
        var table = new ContactMaterial[materials.Length];
        for (int i = 0; i < table.Length; i++)
        {
            if (!materials[i].Surface.IsDefault) throw new ArgumentException("Indexed scalar materials use shape-wide Surface properties.", nameof(materials));
            table[i] = new ContactMaterial(materials[i].Friction, materials[i].Restitution, materials[i].RollingResistance);
        }
        for (int i = 0; i < materialIndices.Length; i++) if ((uint)materialIndices[i] >= (uint)table.Length) throw new ArgumentOutOfRangeException(nameof(materialIndices), materialIndices[i], "Material index at segment " + i + " is outside the table.");
        var vertices = new Vec2[points.Length]; for (int i = 0; i < vertices.Length; i++) vertices[i] = new(points[i].X, points[i].Y);
        var chain = new Chain2D(vertices, loop, twoSided); Shape first = default;
        var index = new int[1];
        for (int i = 0; i < chain.SegmentCount; i++)
        {
            var shape = AddShape(body, chain.GetSegment(i), def); index[0] = materialIndices[i]; _core.SetContactMaterials(shape.RawHandle, table, index);
            if (i == 0) first = shape;
        }
        return first;
    }
}
