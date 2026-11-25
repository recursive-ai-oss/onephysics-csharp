using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// Moves the members of a definition that are written in metres (lengths, speeds, inertias, stiffnesses and the like) to a world's length unit.
/// A member has a <em>dimension</em>: the power of length in its SI unit once mass and time are left alone (a length is 1, an area 2, a speed 1, a torque or a moment of inertia 2, a force 1,
/// a pressure or modulus −1, a density per volume −3, a density per area −2, a compliance in m/N 0). A world unit is <c>Scale</c> times shorter than a metre, so a value of dimension <c>p</c> is multiplied
/// by <c>Scale</c> to the power <c>p</c> (divided, for a negative power). The factor is built from float products and quotients of the one float <c>Scale</c>, so every platform computes the same bits,
/// and at the default unit nothing is touched.
/// Two modes: <em>defaults only</em> (the world applies it when a definition is added, so a member still at its metre default follows the unit and a member the host set stays in world units)
/// and <em>everything</em> (<c>InUnit</c>: a definition written in metres, such as a preset, is converted whole).
/// </summary>
internal readonly struct UnitScaler
{
    private readonly float _k;
    private readonly bool _all;

    internal UnitScaler(float unitsPerMetre, bool all) { _k = unitsPerMetre; _all = all; }

    internal bool IsIdentity => _k == 1f;

    internal float K => _k;

    /// <summary>The factor for a quantity of dimension <paramref name="p"/>, as a number to multiply by (positive powers) or divide by (negative powers).</summary>
    internal float Apply(float v, int p)
    {
        switch (p)
        {
            case 0: return v;
            case 1: return v * _k;
            case 2: return v * (_k * _k);
            case 3: return v * (_k * _k * _k);
            case 4: return v * (_k * _k * _k * _k);
            case -1: return v / _k;
            case -2: return v / (_k * _k);
            case -3: return v / (_k * _k * _k);
            case -4: return v / (_k * _k * _k * _k);
            default: throw new ArgumentOutOfRangeException(nameof(p), p, "unsupported dimension");
        }
    }

    internal double Apply(double v, int p)
    {
        if (p == 0) { return v; }
        double f = Apply(1f, p > 0 ? p : -p);
        return p > 0 ? v * f : v / f;
    }

    /// <summary>A member with a metre default: it converts when it still equals that default (or always, when everything converts).</summary>
    internal void D(ref float v, float metreDefault, int p)
    {
        if (_k == 1f) { return; }
        if (!_all && BitConverter.SingleToInt32Bits(v) != BitConverter.SingleToInt32Bits(metreDefault)) { return; }
        v = Apply(v, p);
    }

    /// <summary>A member with no default worth following (an explicit position, a size): converts only when everything does.</summary>
    internal void E(ref float v, int p)
    {
        if (_k == 1f || !_all) { return; }
        v = Apply(v, p);
    }

    internal void E(ref Vector3F v, int p)
    {
        if (_k == 1f || !_all) { return; }
        v = new Vector3F(Apply(v.X, p), Apply(v.Y, p), Apply(v.Z, p));
    }

    internal void E(ref Vector2F v, int p)
    {
        if (_k == 1f || !_all) { return; }
        v = new Vector2F(Apply(v.X, p), Apply(v.Y, p));
    }

    internal void E(ref Vector3D v, int p)
    {
        if (_k == 1f || !_all) { return; }
        v = new Vector3D(Apply(v.X, p), Apply(v.Y, p), Apply(v.Z, p));
    }

    internal void E(ref Vector2D v, int p)
    {
        if (_k == 1f || !_all) { return; }
        v = new Vector2D(Apply(v.X, p), Apply(v.Y, p));
    }

    /// <summary>A vector member with a metre default (for example a gravity-like direction scaled in metres).</summary>
    internal void D(ref Vector3F v, Vector3F metreDefault, int p)
    {
        if (_k == 1f) { return; }
        if (!_all && !(v.X == metreDefault.X && v.Y == metreDefault.Y && v.Z == metreDefault.Z)) { return; }
        v = new Vector3F(Apply(v.X, p), Apply(v.Y, p), Apply(v.Z, p));
    }

    /// <summary>A 2D vector member with a metre default.</summary>
    internal void D(ref Vector2F v, Vector2F metreDefault, int p)
    {
        if (_k == 1f) { return; }
        if (!_all && !(v.X == metreDefault.X && v.Y == metreDefault.Y)) { return; }
        v = new Vector2F(Apply(v.X, p), Apply(v.Y, p));
    }
}
