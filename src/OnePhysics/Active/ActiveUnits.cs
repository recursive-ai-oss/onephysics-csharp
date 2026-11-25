using System;

namespace RecursiveAI.OnePhysics;

public sealed partial class BalanceDef
{
    internal BalanceDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        float t = StepTrigger; s.D(ref t, -0.03f, 1); c.StepTrigger = t;
        float h = StepHeight; s.D(ref h, 0.07f, 1); c.StepHeight = h;
        return c;
    }
}

public sealed partial class WalkerDef
{
    internal WalkerDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        float h = StepHeight; s.D(ref h, 0.0867f, 1); c.StepHeight = h;
        float w = StepWidth; s.D(ref w, 0.2f, 1); c.StepWidth = w;
        float v = MaxSpeed; s.D(ref v, 2.5f, 1); c.MaxSpeed = v;
        return c;
    }
}

public sealed partial class ActiveRagdollDef
{
    // World units per metre the members are expressed in; 1 for a definition written in metres. Validation checks the metre equivalent.
    internal float Units = 1f;

    /// <summary>
    /// This definition with the lengths and speeds that were never set (the hit radius, a recovery step's trigger distance and clearance, a walker's step height and width and top speed) at their defaults for a world of the given unit.
    /// The world applies this itself when the active layer is enabled; use <see cref="InUnit"/> on <see cref="Humanoid3D"/>, whose step width is set.
    /// </summary>
    /// <param name="unit">The world's length unit.</param>
    public ActiveRagdollDef WithDefaultsFor(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, false));

    /// <summary>This definition, written in metres (for example <see cref="Humanoid3D"/>), expressed in <paramref name="unit"/>: hit radius, step trigger, clearance and width, and top walking speed. Gains in Hz, times and fractions stay.</summary>
    /// <param name="unit">The target length unit.</param>
    public ActiveRagdollDef InUnit(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, true));

    internal ActiveRagdollDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        c.Units = s.K;
        float r = HitRadius; s.D(ref r, 0.45f, 1); c.HitRadius = r;
        c.Balance = Balance?.Scaled(s)!; c.Walker = Walker?.Scaled(s)!;
        return c;
    }
}
