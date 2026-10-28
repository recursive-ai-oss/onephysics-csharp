using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics;

// Definition structs must mean their documented defaults when they are the zero value (a class field, an array element, `default`).
// A field whose default is not zero is therefore stored as its bits XOR the default's bits: the stored zero reads back as the default,
// and every other value (NaN payloads and signed zeros included) round-trips exactly.
internal static class ZeroDefault
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Get(uint stored, float def) => BitConverter.UInt32BitsToSingle(stored ^ BitConverter.SingleToUInt32Bits(def));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static uint Set(float value, float def) => BitConverter.SingleToUInt32Bits(value) ^ BitConverter.SingleToUInt32Bits(def);
}

/// <summary>Three floats stored relative to a default so that the zero value reads as that default.</summary>
internal struct ZeroDefault3
{
    private uint _x, _y, _z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector3F Get(Vector3F def) => new(ZeroDefault.Get(_x, def.X), ZeroDefault.Get(_y, def.Y), ZeroDefault.Get(_z, def.Z));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Set(Vector3F value, Vector3F def)
    {
        _x = ZeroDefault.Set(value.X, def.X);
        _y = ZeroDefault.Set(value.Y, def.Y);
        _z = ZeroDefault.Set(value.Z, def.Z);
    }
}

/// <summary>Two floats stored relative to a default so that the zero value reads as that default.</summary>
internal struct ZeroDefault2
{
    private uint _x, _y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vector2F Get(Vector2F def) => new(ZeroDefault.Get(_x, def.X), ZeroDefault.Get(_y, def.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Set(Vector2F value, Vector2F def)
    {
        _x = ZeroDefault.Set(value.X, def.X);
        _y = ZeroDefault.Set(value.Y, def.Y);
    }
}

/// <summary>A quaternion stored relative to a default (the identity) so that the zero value reads as that default.</summary>
internal struct ZeroDefaultQuat
{
    private uint _x, _y, _z, _w;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly QuaternionF Get(QuaternionF def) => new(ZeroDefault.Get(_x, def.X), ZeroDefault.Get(_y, def.Y), ZeroDefault.Get(_z, def.Z), ZeroDefault.Get(_w, def.W));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Set(QuaternionF value, QuaternionF def)
    {
        _x = ZeroDefault.Set(value.X, def.X);
        _y = ZeroDefault.Set(value.Y, def.Y);
        _z = ZeroDefault.Set(value.Z, def.Z);
        _w = ZeroDefault.Set(value.W, def.W);
    }
}
