using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Optional scalar materials have one table index per interval, holes included; indices are validated before attachment. Attaches a sampled terrain surface as chain segments to a static body. Holes produce no shape. Returns the first segment, or a default handle for an entirely empty field. Throws ArgumentNullException for null, ArgumentException for a foreign body, StaticOnlyShapeException for a non-static body, and the usual shape-definition errors.</summary>
    public Shape CreateHeightField(Body body, HeightField2D field, ShapeDef? def = null, ReadOnlySpan<Material2D> materials = default, ReadOnlySpan<int> materialIndices = default)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.CreateHeightField");
        ArgumentNullException.ThrowIfNull(field);
        if (body.World != this) { throw new ArgumentException("The body belongs to another world.", nameof(body)); }
        if (body.Type != BodyType.Static) { throw new StaticOnlyShapeException("Height fields can only be attached to static bodies."); }
        var table=GridMaterials.Convert(materials);
        bool indexed=materials.Length!=0 || materialIndices.Length!=0;
        if(indexed)GridMaterials.Check(materialIndices,field.IntervalCount,table.Length);
        Shape first = default;
        var index=new int[1];
        bool found = false;
        for (int i = 0; i < field.IntervalCount; i++)
        {
            if (field.IsHole(i)) { continue; }
            var shape = AddShape(body, field.GetSegment(i), def);
            if(indexed){index[0]=0;_core.SetContactMaterials(shape.RawHandle,[table[materialIndices[i]]],index);}
            if (!found) { first = shape; found = true; }
        }
        return first;
    }

    /// <summary>Optional scalar materials have one table index per interval, holes included. Adds static, one-sided terrain. Heights and spacing are metres; heights contain 2..2^20 finite samples. Optional hole flags remove intervals. The field faces upward; ghosts prevent snagging at column joins. Throws the HeightField2D constructor's typed input errors.</summary>
    public Body AddHeightField(ReadOnlySpan<float> heights, float spacing = 1f, Vector2D at = default, float angleRadians = 0f, Material2D? material = null, ReadOnlySpan<bool> holes = default, ReadOnlySpan<Material2D> materials = default, ReadOnlySpan<int> materialIndices = default)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddHeightField");
        var field = new HeightField2D(heights, spacing, holes);
        var body = CreateBody(BodyType.Static, at, angleRadians);
        try { CreateHeightField(body, field, ShapeFor(material, 0f),materials,materialIndices); }
        catch (Exception) { body.Destroy(); throw; }
        return body;
    }
}
