using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class RagdollPartDef
{
    internal RagdollPartDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        var h = ShapeHalfSize; s.D(ref h, new Vector2F(0.1f, 0.05f), 1); c.ShapeHalfSize = h;
        var p = Position; s.E(ref p, 1); c.Position = p;
        var j = JointPosition; s.E(ref j, 1); c.JointPosition = j;
        float t = MaxTorque; s.D(ref t, 100f, 2); c.MaxTorque = t;
        float bf = BreakForce; if (!float.IsPositiveInfinity(bf)) { s.E(ref bf, 1); c.BreakForce = bf; }
        float bt = BreakTorque; if (!float.IsPositiveInfinity(bt)) { s.E(ref bt, 2); c.BreakTorque = bt; }
        return c;
    }
}

public sealed partial class RagdollDef
{
    // World units per metre the members are expressed in; 1 for a definition written in metres.
    internal float Units = 1f;

    /// <summary>
    /// This definition with the members that were never set at their defaults for a world of the given unit (default part size, drive gains and torque limit). The world applies this itself when the ragdoll is added; a skeleton you
    /// laid out in metres (the <see cref="Humanoid"/> preset, <see cref="CreateHumanoid"/>) needs <see cref="InUnit"/>.
    /// </summary>
    /// <param name="unit">The world's length unit.</param>
    public RagdollDef WithDefaultsFor(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, false));

    /// <summary>
    /// This skeleton, laid out in metres and newtons, expressed in <paramref name="unit"/>: shape sizes, part and joint positions and break limits, and the drive gains and torque limits (newton-metres become kilogram world units squared per second squared).
    /// Masses, angles and rates stay.
    /// </summary>
    /// <param name="unit">The target length unit.</param>
    public RagdollDef InUnit(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, true));

    internal RagdollDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity || Parts == null) { return c; }
        c.Units = s.K;
        for (int i = 0; i < c.Parts.Length; i++) { c.Parts[i] = c.Parts[i]?.Scaled(s)!; }
        return c;
    }
}
