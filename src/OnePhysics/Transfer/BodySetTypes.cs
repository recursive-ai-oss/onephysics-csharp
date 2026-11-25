using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePhysics;

/// <summary>What <c>ExtractBodies</c> does with a joint that connects a body of the set to a body outside it.</summary>
public enum BoundaryPolicy
{
    /// <summary>Refuse the extraction with a <see cref="BodySetTransferException"/> naming the joint and the outside body. The default.</summary>
    Refuse = 0,

    /// <summary>Leave the joint behind: it is destroyed with the set when bodies are moved, and simply not copied.</summary>
    Break = 1,

    /// <summary>
    /// Keep the joint and carry the outside body along as an <b>anchor</b>: a shapeless body at the outside body's pose with its velocity, so the set still hangs where it hung. The anchor is kinematic (it keeps moving
    /// at the velocity the outside body had), except that an outside body that is static gives a static anchor. It is created in the receiving world and listed in <see cref="BodySetHandleMap.Anchors"/>; the outside body itself stays where it is.
    /// </summary>
    HoldAsAnchor = 2,
}

/// <summary>Options of <c>ExtractBodies</c>. Every member has a default, so <c>new BodySetOptions()</c> moves the bodies and refuses a set that cuts through a joint.</summary>
public sealed class BodySetOptions
{
    /// <summary>
    /// True (the default) moves the bodies: the source world destroys them, with their shapes, joints and contacts, once the packet is built. False copies: the source world is not changed.
    /// </summary>
    public bool Move { get; set; } = true;

    /// <summary>What to do with a joint that connects the set to a body outside it. Default <see cref="BoundaryPolicy.Refuse"/>.</summary>
    public BoundaryPolicy Boundary { get; set; } = BoundaryPolicy.Refuse;
}

/// <summary>A set of bodies taken out of a world, ready to be inserted into another: shapes, the joints among them, the contacts among them with their warm-start state, and the boundary anchors. Immutable.</summary>
/// <remarks>The byte layout is specified in <c>docs/spec/transfer.md</c>. Packets are not snapshots and cannot be passed to <c>Restore</c>.</remarks>
public sealed class BodySetPacket
{
    private readonly byte[] _bytes;
    internal readonly PacketModel Model;

    internal BodySetPacket(byte[] bytes, PacketModel model) { _bytes = bytes; Model = model; }

    /// <summary>2 for a packet from a <c>PhysicsWorld2D</c>, 3 for one from a <c>PhysicsWorld3D</c>.</summary>
    public int Dimension => Model.Dimension;

    /// <summary>The length unit of the source world, in metres per world unit. A world accepts a packet only when its own unit is equal.</summary>
    public double MetresPerUnit => Model.MetresPerUnit;

    /// <summary>Bodies that were in the set (anchors not counted).</summary>
    public int BodyCount => Model.MemberCount;

    /// <summary>Anchors that were added by <see cref="BoundaryPolicy.HoldAsAnchor"/>.</summary>
    public int AnchorCount => Model.Bodies.Length - Model.MemberCount;

    /// <summary>Shapes carried.</summary>
    public int ShapeCount => Model.Shapes.Length;

    /// <summary>Joints carried.</summary>
    public int JointCount => Model.Joints.Length;

    /// <summary>Contact constraints carried (each with its solver warm-start state).</summary>
    public int ContactCount => Model.Contacts.Length;

    /// <summary>Size of <see cref="ToArray"/> in bytes.</summary>
    public int ByteCount => _bytes.Length;

    /// <summary>A copy of the packet's bytes (<c>docs/spec/transfer.md</c>), identical on every platform.</summary>
    public byte[] ToArray() => (byte[])_bytes.Clone();

    /// <summary>The packet's bytes without copying; do not modify.</summary>
    public ReadOnlySpan<byte> AsSpan() => _bytes;

    /// <summary>
    /// Reads and fully validates a packet: counts against the bytes that remain, every index, every number (finite, in range), geometry and joint records. Nothing is allocated before its size is checked.
    /// </summary>
    /// <param name="bytes">The packet bytes.</param>
    /// <param name="maxBytes">Largest packet accepted; default 256 MiB.</param>
    /// <exception cref="BodySetPacketException">The bytes are not a valid packet; the message names the field and its position.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxBytes"/> is negative.</exception>
    public static BodySetPacket Read(ReadOnlySpan<byte> bytes, int maxBytes = 256 * 1024 * 1024)
    {
        if (maxBytes < 0) { throw new ArgumentOutOfRangeException(nameof(maxBytes), maxBytes, "maxBytes must be at least 0."); }
        if (bytes.Length > maxBytes) { throw new BodySetPacketException("packet", 0, "the packet is " + bytes.Length.ToString(CultureInfo.InvariantCulture) + " bytes, above the limit of " + maxBytes.ToString(CultureInfo.InvariantCulture) + "."); }
        var copy = bytes.ToArray();
        var model = PacketIO.Read(copy);
        return new BodySetPacket(copy, model);
    }
}

/// <summary>
/// The old-to-new handles of an insertion, in packet order (bodies by ascending source slot, then anchors), and the same for shapes and joints. Contacts have no public handles and are not listed.
/// </summary>
public sealed class BodySetHandleMap
{
    private readonly BodyHandle[] _oldBodies, _newBodies;
    private readonly ShapeHandle[] _oldShapes, _newShapes;
    private readonly JointHandle[] _oldJoints, _newJoints;
    private readonly RagdollHandle[] _oldRagdolls, _newRagdolls;
    private readonly int _members;

    internal BodySetHandleMap(BodyHandle[] oldBodies, BodyHandle[] newBodies, int members, ShapeHandle[] oldShapes, ShapeHandle[] newShapes, JointHandle[] oldJoints, JointHandle[] newJoints, RagdollHandle[] oldRagdolls, RagdollHandle[] newRagdolls)
    {
        _oldBodies = oldBodies; _newBodies = newBodies; _members = members;
        _oldShapes = oldShapes; _newShapes = newShapes; _oldJoints = oldJoints; _newJoints = newJoints; _oldRagdolls = oldRagdolls; _newRagdolls = newRagdolls;
    }

    /// <summary>Handles the bodies had in the source world; for an anchor the outside body it stands for.</summary>
    public ReadOnlySpan<BodyHandle> OldBodies => _oldBodies;

    /// <summary>Handles the bodies have in the receiving world, parallel to <see cref="OldBodies"/>.</summary>
    public ReadOnlySpan<BodyHandle> NewBodies => _newBodies;

    /// <summary>How many of <see cref="NewBodies"/> are members of the set; the rest, at the end, are boundary anchors.</summary>
    public int MemberCount => _members;

    /// <summary>Shape handles in the source world.</summary>
    public ReadOnlySpan<ShapeHandle> OldShapes => _oldShapes;

    /// <summary>Shape handles in the receiving world, parallel to <see cref="OldShapes"/>.</summary>
    public ReadOnlySpan<ShapeHandle> NewShapes => _newShapes;

    /// <summary>Joint handles in the source world.</summary>
    public ReadOnlySpan<JointHandle> OldJoints => _oldJoints;

    /// <summary>Joint handles in the receiving world, parallel to <see cref="OldJoints"/>.</summary>
    public ReadOnlySpan<JointHandle> NewJoints => _newJoints;

    /// <summary>Ragdoll handles in the source world.</summary>
    public ReadOnlySpan<RagdollHandle> OldRagdolls => _oldRagdolls;

    /// <summary>Ragdoll handles in the receiving world, parallel to <see cref="OldRagdolls"/>.</summary>
    public ReadOnlySpan<RagdollHandle> NewRagdolls => _newRagdolls;

    /// <summary>The new handle of a body of the set. Returns false for a handle that was not in the set.</summary>
    public bool TryMap(BodyHandle old, out BodyHandle mapped)
    {
        for (int i = 0; i < _members; i++) { if (_oldBodies[i] == old) { mapped = _newBodies[i]; return true; } }
        mapped = default;
        return false;
    }

    /// <summary>The new handle of a shape of the set. Returns false for a handle that was not in the set.</summary>
    public bool TryMap(ShapeHandle old, out ShapeHandle mapped)
    {
        for (int i = 0; i < _oldShapes.Length; i++) { if (_oldShapes[i] == old) { mapped = _newShapes[i]; return true; } }
        mapped = default;
        return false;
    }

    /// <summary>The new handle of a joint of the set. Returns false for a handle that was not in the set.</summary>
    public bool TryMap(JointHandle old, out JointHandle mapped)
    {
        for (int i = 0; i < _oldJoints.Length; i++) { if (_oldJoints[i] == old) { mapped = _newJoints[i]; return true; } }
        mapped = default;
        return false;
    }

    /// <summary>The anchors that <see cref="BoundaryPolicy.HoldAsAnchor"/> created: the outside body's old handle and the anchor's new one.</summary>
    public IReadOnlyList<(BodyHandle Outside, BodyHandle Anchor)> Anchors
    {
        get
        {
            var list = new List<(BodyHandle, BodyHandle)>();
            for (int i = _members; i < _oldBodies.Length; i++) { list.Add((_oldBodies[i], _newBodies[i])); }
            return list;
        }
    }
}

/// <summary>A set of bodies cannot be extracted or a packet cannot be inserted, for a reason in the world's state: a joint cuts through the set and no boundary policy was chosen, a body belongs to a character, vehicle or other feature the packet does not carry, the length units differ. The world is unchanged (a moved set is removed only after the packet is built).</summary>
public sealed class BodySetTransferException : PhysicsException
{
    /// <summary>Creates the exception with a message naming what stopped the transfer.</summary>
    public BodySetTransferException(string message) : base(message) { }
}

/// <summary>Bytes that are not a valid body-set packet.</summary>
public sealed class BodySetPacketException : PhysicsException
{
    /// <summary>The field that was wrong, for example <c>bodies[3].rotation</c>.</summary>
    public string Field { get; }

    /// <summary>Byte offset where the problem was found.</summary>
    public int Offset { get; }

    /// <summary>Creates the exception for a problem with <paramref name="field"/> at <paramref name="offset"/>.</summary>
    public BodySetPacketException(string field, int offset, string message)
        : base("Body-set packet: " + field + ": " + message + " (at byte " + offset.ToString(CultureInfo.InvariantCulture) + ")")
    {
        Field = field;
        Offset = offset;
    }
}
