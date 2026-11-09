using System;

namespace RecursiveAI.OnePhysics.Physics3D;

public readonly partial struct Shape
{
    /// <summary>Assigns a copied scalar material table and one index per triangle (or one for a segment). Empty spans remove the override. Shape-wide Surface settings remain separate. Invalid lengths, indices or per-material Surface settings throw ArgumentException; stale shapes throw InvalidHandleException.</summary>
    public void SetMaterials(ReadOnlySpan<Material3D> materials, ReadOnlySpan<int> indices)
    {
        using var mutation = _world?.RecordMutation("Shape.SetMaterials"); Resolve();
        var scalar = new ContactMaterial[materials.Length];
        for (int i = 0; i < scalar.Length; i++)
        {
            if (!materials[i].Surface.IsDefault) throw new ArgumentException("Indexed scalar materials use shape-wide Surface properties.", nameof(materials));
            scalar[i] = new ContactMaterial(materials[i].Friction, materials[i].Restitution, materials[i].RollingResistance);
        }
        _world!.Core.SetContactMaterials(_handle, scalar, indices);
    }
}
