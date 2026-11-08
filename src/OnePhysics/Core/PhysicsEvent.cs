using System;
using System.Collections.Generic;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>An event. Contact and sensor events carry both bodies and shapes and the sub-shape ids; joint events carry the joint and its bodies; sleep and wake carry body A only.</summary>
public readonly struct PhysicsEvent
{
    /// <summary>What happened.</summary>
    public EventKind Kind { get; }

    /// <summary>First body.</summary>
    public BodyHandle BodyA { get; }

    /// <summary>Second body (null for sleep and wake).</summary>
    public BodyHandle BodyB { get; }

    /// <summary>Shape of A (null for joint, sleep and wake events).</summary>
    public ShapeHandle ShapeA { get; }

    /// <summary>Shape of B.</summary>
    public ShapeHandle ShapeB { get; }

    /// <summary>Sub-shape of A.</summary>
    public int SubShapeA { get; }

    /// <summary>Sub-shape of B.</summary>
    public int SubShapeB { get; }

    /// <summary>The joint for <see cref="EventKind.JointBreak"/>; null otherwise.</summary>
    public JointHandle Joint { get; }

    /// <summary>Kind-specific number (break value for joint breaks); zero otherwise.</summary>
    public float Value { get; }

    /// <summary>Optional audio/haptic speeds, zero unless ContactMotionEnabled is set on the world.</summary>
    public ContactMotion Motion { get; }
    /// <summary>Pre-solve approach speed at manifold birth, metres/second; zero if collection is disabled.</summary>
    public float ImpactSpeed => Motion.ImpactSpeed;
    /// <summary>Residual sliding speed at the end of this step, metres/second; zero if collection is disabled.</summary>
    public float SlidingSpeed => Motion.SlidingSpeed;
    /// <summary>Rolling surface speed at the end of this step, metres/second; zero if collection is disabled.</summary>
    public float RollingSpeed => Motion.RollingSpeed;
    internal PhysicsEvent WithMotion(ContactMotion motion) => new(Kind, BodyA, BodyB, ShapeA, ShapeB, SubShapeA, SubShapeB, Joint, Value, motion);

    /// <summary>Creates an event from all its parts.</summary>
    public PhysicsEvent(EventKind kind, BodyHandle a, BodyHandle b, ShapeHandle sa, ShapeHandle sb, int subA, int subB, JointHandle joint, float value)
        : this(kind, a, b, sa, sb, subA, subB, joint, value, default) { }

    internal PhysicsEvent(EventKind kind, BodyHandle a, BodyHandle b, ShapeHandle sa, ShapeHandle sb, int subA, int subB, JointHandle joint, float value, ContactMotion motion)
    {
        Motion = motion; Kind = kind; BodyA = a; BodyB = b; ShapeA = sa; ShapeB = sb; SubShapeA = subA; SubShapeB = subB; Joint = joint; Value = value;
    }

    /// <summary>
    /// The delivery order: kind, then body A, body B, shape A, shape B, sub-shape A, sub-shape B, joint (each by slot index, then generation), then the value's bit pattern.
    /// Total on every field, so events that compare equal are identical and the order does not depend on how they were collected.
    /// </summary>
    public static int Compare(in PhysicsEvent x, in PhysicsEvent y)
    {
        int c = ((int)x.Kind).CompareTo((int)y.Kind);
        if (c != 0) { return c; }
        c = x.BodyA.Index.CompareTo(y.BodyA.Index); if (c != 0) { return c; }
        c = x.BodyB.Index.CompareTo(y.BodyB.Index); if (c != 0) { return c; }
        c = x.ShapeA.Index.CompareTo(y.ShapeA.Index); if (c != 0) { return c; }
        c = x.ShapeB.Index.CompareTo(y.ShapeB.Index); if (c != 0) { return c; }
        c = x.SubShapeA.CompareTo(y.SubShapeA); if (c != 0) { return c; }
        c = x.SubShapeB.CompareTo(y.SubShapeB); if (c != 0) { return c; }
        c = x.Joint.Index.CompareTo(y.Joint.Index); if (c != 0) { return c; }
        c = x.BodyA.Generation.CompareTo(y.BodyA.Generation); if (c != 0) { return c; }
        c = x.BodyB.Generation.CompareTo(y.BodyB.Generation); if (c != 0) { return c; }
        c = x.ShapeA.Generation.CompareTo(y.ShapeA.Generation); if (c != 0) { return c; }
        c = x.ShapeB.Generation.CompareTo(y.ShapeB.Generation); if (c != 0) { return c; }
        c = x.Joint.Generation.CompareTo(y.Joint.Generation); if (c != 0) { return c; }
        c = BitConverter.SingleToUInt32Bits(x.Value).CompareTo(BitConverter.SingleToUInt32Bits(y.Value));
        if (c == 0) c = BitConverter.SingleToUInt32Bits(x.ImpactSpeed).CompareTo(BitConverter.SingleToUInt32Bits(y.ImpactSpeed));
        if (c == 0) c = BitConverter.SingleToUInt32Bits(x.SlidingSpeed).CompareTo(BitConverter.SingleToUInt32Bits(y.SlidingSpeed));
        return c != 0 ? c : BitConverter.SingleToUInt32Bits(x.RollingSpeed).CompareTo(BitConverter.SingleToUInt32Bits(y.RollingSpeed));
    }

    internal void Write(StateWriter w)
    {
        w.NamedByte((byte)Kind);
        w.NamedInt32(BodyA.World); w.NamedInt32(BodyA.Index); w.NamedInt32(BodyA.Generation);
        w.NamedInt32(BodyB.World); w.NamedInt32(BodyB.Index); w.NamedInt32(BodyB.Generation);
        w.NamedInt32(ShapeA.World); w.NamedInt32(ShapeA.Index); w.NamedInt32(ShapeA.Generation);
        w.NamedInt32(ShapeB.World); w.NamedInt32(ShapeB.Index); w.NamedInt32(ShapeB.Generation);
        w.NamedInt32(SubShapeA); w.NamedInt32(SubShapeB);
        w.NamedInt32(Joint.World); w.NamedInt32(Joint.Index); w.NamedInt32(Joint.Generation);
        w.NamedSingle(Value);
    }

    internal const int SnapshotBytes = 1 + 4 * 17 + 4;

    internal static PhysicsEvent Read(StateReader r)
    {
        byte k = r.ReadByte();
        if (k > (byte)EventKind.ContactHit) { throw r.Error("invalid event kind"); }
        var a = new BodyHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        var b = new BodyHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        var sa = new ShapeHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        var sb = new ShapeHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        int subA = r.ReadInt32(), subB = r.ReadInt32();
        var j = new JointHandle(r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
        float v = r.ReadSingle();
        return new PhysicsEvent((EventKind)k, a, b, sa, sb, subA, subB, j, v);
    }

    /// <inheritdoc/>
    public override string ToString() =>
        Kind + " " + BodyA + (BodyB.IsNull ? "" : " - " + BodyB) + (Joint.IsNull ? "" : " " + Joint) + (Value == 0f ? "" : " value " + Value.ToString(System.Globalization.CultureInfo.InvariantCulture));
}
