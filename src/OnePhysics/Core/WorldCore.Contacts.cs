namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    internal readonly ContactModificationPipeline ContactModifications = new();
    internal SurfaceStore Surfaces { get; private set; } = null!;
    internal readonly ContactMaterialStore ContactMaterials = new();
    /// <summary>Assigns an immutable copied material table and one index per mesh/height-field triangle or segment. Empty spans remove the override. Throws ArgumentException for count mismatch, ArgumentOutOfRangeException for invalid indices, or InvalidHandleException for a stale shape.</summary>
    public void SetContactMaterials(ShapeHandle shape, System.ReadOnlySpan<ContactMaterial> materials, System.ReadOnlySpan<int> indices) { Resolve(shape); ContactMaterials.Set(shape, materials, indices); ContactModifications.RequestRefresh(); }
    /// <summary>Reads a material index, or -1 when no indexed override exists. Throws InvalidHandleException for a stale shape or ArgumentOutOfRangeException for a negative subshape id.</summary>
    public int GetContactMaterialIndex(ShapeHandle shape, int subshape) { Resolve(shape); System.ArgumentOutOfRangeException.ThrowIfNegative(subshape); return ContactMaterials.MaterialIndex(shape.Index, subshape); }
    internal readonly ContactMotionStore ContactMotion = new();
    /// <summary>Collect impact, sliding and rolling contact speeds for events and current contacts. Default false; enabling adds an optional snapshot section. Disabling clears collected speeds.</summary>
    public bool ContactMotionEnabled
    {
        get => ContactMotion.Enabled;
        set { if (value) ContactMotion.Enabled = true; else { ContactMotion.Reset(); Events.EnrichContactMotion(ContactMotion); } }
    }
    internal float ContactStepDt { get; private set; }

    /// <summary>Reads optional body-frame surface properties. Default is stationary, two-sided and isotropic. Throws InvalidHandleException for a stale shape.</summary>
    public SurfaceProperties GetSurfaceProperties(ShapeHandle shape) { int slot = Resolve(shape); return Surfaces.Get(slot); }

    /// <summary>Sets optional body-frame surface properties for the next step. Default removes the optional record. Throws ArgumentOutOfRangeException for invalid vectors or coefficients, or InvalidHandleException for a stale shape.</summary>
    public void SetSurfaceProperties(ShapeHandle shape, SurfaceProperties properties) { Resolve(shape); Surfaces.Set(shape, properties); ContactModifications.RequestRefresh(); }


    /// <summary>Returns allocation-free contact views for a live body in constraint-slot order. Read between mutations; sensors and disabled zero-point contacts are excluded. Throws InvalidHandleException for a stale body.</summary>
    public BodyContactEnumerable GetContacts(BodyHandle body) { Resolve(body); return new(this, body); }

    /// <summary>Raw-tier pre-solve manifold callback, null by default. The built-in engines invoke it serially after narrowphase and before solving, in shape-pair/subshape order. The view directly edits the constraint payload. Do not mutate other world storage from this callback. Not serialized or copied by Fork.</summary>
    public ContactModifier? ContactModifier
    {
        get => ContactModifications.Callback;
        set => ContactModifications.Callback = value;
    }
}
