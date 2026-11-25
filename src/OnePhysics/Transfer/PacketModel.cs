using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>The decoded content of a packet; both dimensions use it, with lane counts taken from the dimension.</summary>
internal sealed class PacketModel
{
    public int Dimension;
    public double MetresPerUnit = 1.0;
    /// <summary>The id of the world the packet was extracted from; known only in memory (0 for a packet read from bytes), so the bytes depend on the set and nothing else.</summary>
    public int SourceWorld;
    public PacketBody[] Bodies = Array.Empty<PacketBody>();
    public int MemberCount;
    public PacketShape[] Shapes = Array.Empty<PacketShape>();
    public PacketJoint[] Joints = Array.Empty<PacketJoint>();
    public PacketContact[] Contacts = Array.Empty<PacketContact>();
    public PacketSupport[] Supports = Array.Empty<PacketSupport>();
    public PacketSupportContact[] SupportContacts = Array.Empty<PacketSupportContact>();
    public PacketRagdoll[] Ragdolls = Array.Empty<PacketRagdoll>();

    public int PositionLanes => Dimension;
    public int RotationLanes => Dimension == 3 ? 4 : 2;
    public int LinearLanes => Dimension;
    public int AngularLanes => Dimension == 3 ? 3 : 1;
    public int MaxManifoldPoints => Dimension == 3 ? 4 : 2;
    public int PayloadFloats => Dimension == 3 ? 104 : 40;

    /// <summary>Floats of engine data per body: linear damping, angular damping, gravity scale, last displacement, force and torque accumulators.</summary>
    public int ExtraFloats => 3 + 2 * LinearLanes + AngularLanes;
}

internal sealed class PacketBody
{
    public int Slot, Generation;
    public byte Type;
    public byte Bits;            // bit 0 bullet, bit 1 never sleeps
    public bool Anchor;
    public bool Asleep;
    public float SleepTime;
    public double[] Position = Array.Empty<double>();   // absolute: world origin plus stored position, of the centre of mass (of the origin for an anchor)
    public float[] Rotation = Array.Empty<float>();
    public float[] Linear = Array.Empty<float>();
    public float[] Angular = Array.Empty<float>();
    public byte EngineFlags;
    public float[] Extra = Array.Empty<float>();
    public bool HasControls;
    public BodyControlState Controls;
    public bool HasSleep;
    public BodySleepSettings Sleep;
}

internal sealed class PacketShape
{
    public int Slot, Generation;
    public int Body;             // index into PacketModel.Bodies
    public byte[] Blob = Array.Empty<byte>();
}

internal sealed class PacketJoint
{
    public int Slot, Generation;
    public int BodyA, BodyB;     // indices into PacketModel.Bodies
    public byte Flags;
    public byte Color;           // solve colour in the source world (255 = the serial overflow set)
    public float BreakValue;
    public byte[] Blob = Array.Empty<byte>();
    public float[] Payload = Array.Empty<float>();
}

internal sealed class PacketContact
{
    public int Slot, Generation;
    public int ShapeA, ShapeB;   // indices into PacketModel.Shapes, A below B
    public int SubA, SubB;
    public byte Flags;
    public byte Color;
    public byte Points;
    public uint[] Features = Array.Empty<uint>();
    public float BreakValue;
    public bool NewPair;         // first constraint of its shape pair
    public float[] Payload = Array.Empty<float>();
}

/// <summary>A static shape outside the set that a member rests on: enough to find the same shape (same geometry, same pose) in the receiving world.</summary>
internal sealed class PacketSupport
{
    public double[] Position = Array.Empty<double>();   // absolute position of the static body
    public float[] Rotation = Array.Empty<float>();
    public byte[] Blob = Array.Empty<byte>();
}

/// <summary>A contact between a member shape and a support shape, oriented as in the source world.</summary>
internal sealed class PacketSupportContact
{
    public int Member;           // index into PacketModel.Shapes
    public int Support;          // index into PacketModel.Supports
    public bool MemberIsA;       // the member shape was shape A of the constraint in the source world
    public int SubA, SubB;
    public byte Flags;
    public byte Color;
    public byte Points;
    public uint[] Features = Array.Empty<uint>();
    public float BreakValue;
    public bool NewPair;
    public float[] Payload = Array.Empty<float>();
}

/// <summary>A ragdoll whose parts are bodies of the packet: the record of its definition and drive state, with parts and joints named by packet index.</summary>
internal sealed class PacketRagdoll
{
    public int Slot, Generation;
    public byte[] Blob = Array.Empty<byte>();
}
