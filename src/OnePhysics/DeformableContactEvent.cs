namespace RecursiveAI.OnePhysics;

/// <summary>What a <see cref="DeformableContactEvent"/> reports.</summary>
public enum DeformableContactEventKind : byte
{
    /// <summary>A deformable started touching a rigid body (any body type: static, kinematic or dynamic).</summary>
    Began = 0,

    /// <summary>A deformable stopped touching a rigid body.</summary>
    Ended = 1,
}

/// <summary>
/// A soft body, cloth or rod began or stopped touching a rigid body. Delivered after the step, ordered by deformable (soft bodies and cloth by slot, then rods by slot) and then by body slot, when the world's
/// <c>TrackDeformableContacts</c> is on. A deformable touches a body when one of its particles (for rods also the middle of each segment) is within 5 mm of a shape of the body that its layer, group, category
/// and mask let it collide with.
/// </summary>
public readonly struct DeformableContactEvent
{
    internal DeformableContactEvent(DeformableContactEventKind kind, DeformableKind deformable, int slot, int generation, BodyHandle body)
    {
        Kind = kind; Deformable = deformable; Slot = slot; Generation = generation; Body = body;
    }

    /// <summary>Whether the contact began or ended.</summary>
    public DeformableContactEventKind Kind { get; }

    /// <summary>The kind of deformable.</summary>
    public DeformableKind Deformable { get; }

    /// <summary>The soft body's or rod's slot (<c>GetSoftBody(slot)</c>, <c>GetRod(slot)</c>).</summary>
    public int Slot { get; }

    /// <summary>The generation of the soft body or rod at that slot.</summary>
    public int Generation { get; }

    /// <summary>The rigid body.</summary>
    public BodyHandle Body { get; }
}
