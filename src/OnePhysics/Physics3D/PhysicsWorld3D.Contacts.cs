using System;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Collect contact impact/sliding/rolling speeds for audio and haptics. Default false. Configuration and cached speeds are snapshotted and recorded only while enabled.</summary>
    public bool ContactMotionEnabled
    {
        get => _core.ContactMotionEnabled;
        set { using var mutation = RecordMutation("World.ContactMotionEnabled"); _core.ContactMotionEnabled = value; }
    }

    /// <summary>Read-only contacts for a body in constraint-slot order, without allocations. Includes speculative contacts, excludes sensors and disabled zero-point contacts. Throws InvalidHandleException for a stale body or a body from another world.</summary>
    public LowLevel.BodyContactEnumerable GetContacts(Body body)
    {
        if (!ReferenceEquals(body.World, this)) throw new InvalidHandleException("Body belongs to another world or is default.");
        return _core.GetContacts(body.RawHandle);
    }

    /// <summary>Optional pre-solve contact modifier, null by default. Runs once per non-sensor manifold on the calling thread in shape-pair/subshape order, including sleeping contacts. Geometry is recomputed while installed. Not serialized; reinstall on restore as with other delegates. Do not mutate the world inside it.</summary>
    public ContactModifier? ContactModifier
    {
        get => _engine.ContactModifications.Callback;
        set => _engine.ContactModifications.Callback = value;
    }

    /// <summary>Installs a struct-generic modifier. The adapter allocates once at installation, never per contact. Replaces the delegate; set ContactModifier to null to remove it.</summary>
    public void SetContactModifier<T>(T modifier) where T : struct, IContactModifier
    {
        var adapter = new ContactModifierAdapter<T>(modifier);
        ContactModifier = adapter.Modify;
    }

    private sealed class ContactModifierAdapter<T>(T modifier) where T : struct, IContactModifier
    {
        private T _modifier = modifier;
        public void Modify(ref ContactModification contact) => _modifier.Modify(ref contact);
    }
}
