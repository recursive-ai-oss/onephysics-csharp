using System;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Whole-step joint lifetime, stored in reserved payload slots 32 and 33 as two exact unsigned 16-bit values. Zero means unlimited.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0001")]
public static class JointLifetime
{
    /// <summary>Reads the remaining steps of a live joint slot. Invalid slots or non-joints throw.</summary>
    public static int Get(ConstraintStore constraints, int slot)
    {
        Check(constraints, slot);
        if ((constraints.Flags[slot] & (byte)ConstraintBits.HasLifetime) == 0) { return 0; }
        var p = constraints.Payload(slot);
        return (int)p[32] | ((int)p[33] << 16);
    }
    /// <summary>Sets the remaining steps of a live joint slot. Zero disables expiry; negative values or invalid slots throw before mutation.</summary>
    public static void Set(ConstraintStore constraints, int slot, int steps)
    {
        if (steps < 0) { throw new ArgumentOutOfRangeException(nameof(steps), steps, "Lifetime steps must be at least zero."); }
        Check(constraints, slot);
        var p = constraints.Payload(slot); p[32] = steps & 65535; p[33] = steps >> 16;
        if (steps > 0) { constraints.Flags[slot] |= (byte)ConstraintBits.HasLifetime; }
        else { constraints.Flags[slot] &= unchecked((byte)~(byte)ConstraintBits.HasLifetime); }
    }
    private static void Check(ConstraintStore constraints, int slot)
    {
        ArgumentNullException.ThrowIfNull(constraints);
        if ((uint)slot >= (uint)constraints.HighWater || !constraints.IsLive(slot) || constraints.KindOf(slot) != ConstraintKind.Joint)
        { throw new InvalidJointException("Lifetime requires a live joint slot."); }
    }
}
