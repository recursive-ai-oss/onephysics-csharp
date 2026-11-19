using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The one place the solver kernels read and write float arrays without a bounds check. Every caller passes an offset that is in range by construction (a field index below the group's field count
/// plus a lane offset below eight, or a body row offset the preparation wrote); the <c>Checked</c> build configuration turns <see cref="Range"/> into a real check, and the tests and fuzzers run in it.
/// </summary>
internal static class Mem
{
    /// <summary>Throws unless <c>offset .. offset + count - 1</c> lies inside <paramref name="s"/>. Compiled out of every configuration except <c>Checked</c>.</summary>
    [Conditional("CHECKED")]
    public static void Range(ReadOnlySpan<float> s, int offset, int count)
    {
        if (offset < 0 || count < 0 || offset > s.Length - count) { throw new InvalidOperationException("kernel access " + offset + "+" + count + " outside " + s.Length); }
    }

    /// <summary>A reference to element <paramref name="offset"/> without a bounds check; the caller has established the range (see <see cref="Range"/>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref readonly float At(ReadOnlySpan<float> s, int offset)
    {
        Range(s, offset, 1);
        return ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);
    }

    /// <summary>A writable reference to element <paramref name="offset"/> without a bounds check; the caller has established the range (see <see cref="Range"/>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref float At(Span<float> s, int offset)
    {
        Range(s, offset, 1);
        return ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);
    }

    /// <summary>A reference to the first float of a group of <paramref name="count"/> floats; <see cref="Lane(ref float, int)"/> and <see cref="Field"/> then address it. The group's length is checked once here (and in the Checked build on every lane and field).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref float Base(Span<float> s, int count)
    {
        if (s.Length < count) { throw new InvalidOperationException("kernel group of " + count + " floats outside " + s.Length); }
        return ref MemoryMarshal.GetReference(s);
    }

    /// <summary>A reference to the first field of lane <paramref name="lane"/> of a wide group (fields sit eight floats apart); the caller keeps the lane below eight.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref float Lane(ref float group, int lane)
    {
        CheckSlot(0, lane);
        return ref Unsafe.Add(ref group, lane);
    }

    /// <summary>A reference to field <paramref name="field"/> of the lane that <paramref name="lane"/> was taken from (see <see cref="Lane(ref float, int)"/>); the caller keeps the field below the group's field count.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref float Field(ref float lane, int field)
    {
        CheckSlot(field, 0);
        return ref Unsafe.Add(ref lane, field * 8);
    }

    /// <summary>Loads the eight lanes of one field of a wide group.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector256<float> LoadLanes(ref float group, int field)
    {
        CheckSlot(field, 0);
        return Vector256.LoadUnsafe(ref group, (nuint)(field * 8));
    }

    /// <summary>Stores the eight lanes of one field of a wide group.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void StoreLanes(ref float group, int field, Vector256<float> value)
    {
        CheckSlot(field, 0);
        value.StoreUnsafe(ref group, (nuint)(field * 8));
    }

    /// <summary>A reference to element <paramref name="i"/> of an int table that starts at <paramref name="table"/>; the caller keeps the index inside the table's checked length.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref int Lane(ref int table, int i)
    {
        CheckItem(i);
        return ref Unsafe.Add(ref table, i);
    }

    /// <summary>Element <paramref name="offset"/> of an int array without a bounds check; the caller has established the range.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref readonly int At(ReadOnlySpan<int> s, int offset)
    {
        RangeIndex(offset, s.Length);
        return ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);
    }

    /// <summary>Element <paramref name="offset"/> of a byte array without a bounds check; the caller has established the range.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref readonly byte At(ReadOnlySpan<byte> s, int offset)
    {
        RangeIndex(offset, s.Length);
        return ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);
    }

    /// <summary>Element <paramref name="offset"/> of a double span without a bounds check; the caller has established the range (for FinalizeBodies' positions).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref readonly double At(ReadOnlySpan<double> s, int offset)
    {
        RangeIndex(offset, s.Length);
        return ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);
    }

    /// <summary>Element <paramref name="index"/> of an array without a bounds check; the caller has established the range (the narrowphase box phase: indices taken from the pair table and the update list, which were validated when the pairs were made).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref T At<T>(T[] a, int index)
    {
        RangeIndex(index, a.Length);
        return ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(a), index);
    }

    /// <summary>A reference to the first element of an array, the base for <see cref="Add"/> in a loop that hoists its arrays. No check at all: the caller owns every index it adds.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref T Ref<T>(T[] a) => ref MemoryMarshal.GetArrayDataReference(a);

    /// <summary>A reference to the first element of a span (see <see cref="Ref{T}(T[])"/>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref T Ref<T>(Span<T> s) => ref MemoryMarshal.GetReference(s);

    /// <summary>A reference to the first element of a read-only span (see <see cref="Ref{T}(T[])"/>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref T Ref<T>(ReadOnlySpan<T> s) => ref MemoryMarshal.GetReference(s);

    /// <summary>The element <paramref name="offset"/> places after <paramref name="r"/>, unchecked.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref T Add<T>(ref T r, int offset) => ref Unsafe.Add(ref r, offset);

    /// <summary>The same memory read as another type (a struct of floats as its first float, a row block as its first row); the caller knows the layout.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref TTo As<TFrom, TTo>(ref TFrom r) => ref Unsafe.As<TFrom, TTo>(ref r);

    /// <summary><paramref name="count"/> elements starting at <paramref name="r"/> as a read-only span, unchecked.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ReadOnlySpan<T> ReadOnly<T>(ref T r, int count) => MemoryMarshal.CreateReadOnlySpan(ref r, count);

    [Conditional("CHECKED")]
    private static void RangeIndex(int offset, int length)
    {
        if ((uint)offset >= (uint)length) { throw new InvalidOperationException("kernel access " + offset + " outside " + length); }
    }

    [Conditional("CHECKED")]
    private static void CheckSlot(int field, int lane)
    {
        if ((uint)field >= 40u || (uint)lane >= 8u) { throw new InvalidOperationException("kernel slot " + field + "/" + lane + " outside a group"); }
    }

    /// <summary>A reference to the first int of a span of at least <paramref name="count"/> ints; <see cref="Lane(ref int, int)"/> then indexes it.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref int Base(Span<int> s, int count)
    {
        if (s.Length < count) { throw new InvalidOperationException("kernel table of " + count + " ints outside " + s.Length); }
        return ref MemoryMarshal.GetReference(s);
    }

    [Conditional("CHECKED")]
    private static void CheckItem(int i)
    {
        if ((uint)i >= 32u) { throw new InvalidOperationException("kernel table index " + i + " outside a group"); }
    }

    /// <summary>A reference to row <paramref name="row"/> of a 3D wide group (rows of eight floats, one per lane) that starts at <paramref name="group"/>; the caller keeps the row below the group's row count, which the group's <see cref="Base(Span{float}, int)"/> covered.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref float Row(ref float group, int row)
    {
        CheckRow(row);
        return ref Unsafe.Add(ref group, row * 8);
    }

    /// <summary>Vector <paramref name="row"/> of a block of 256-bit vectors (a 3D kernel's gathered bodies, twenty vectors a side); the caller keeps the row inside the block.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector256<float> Row(ref Vector256<float> block, int row)
    {
        CheckItem(row);
        return Unsafe.Add(ref block, row);
    }

    /// <summary>A block of 256-bit lane vectors seen as its first vector (see <see cref="Row(ref Vector256{float}, int)"/>). <typeparamref name="TBlock"/> is a struct made of nothing but eight-lane vectors.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref Vector256<float> AsRows<TBlock>(ref TBlock block) where TBlock : struct => ref Unsafe.As<TBlock, Vector256<float>>(ref block);

    [Conditional("CHECKED")]
    private static void CheckRow(int row)
    {
        if ((uint)row >= 128u) { throw new InvalidOperationException("kernel row " + row + " outside a group"); }
    }

    /// <summary>Declares a scratch buffer without clearing it; the caller writes every element it reads.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void SkipInit<TBuffer>(out TBuffer buffer) => Unsafe.SkipInit(out buffer);

    /// <summary>Loads the eight floats of four consecutive points of <paramref name="points"/> starting at <paramref name="first"/> (0 or 4; x and y interleaved). Lanes of points past the shape's count hold whatever the buffer holds; the caller masks them.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector256<float> LoadFour(in Vec2x8 points, int first)
    {
        // first is 0 or 4 and the buffer holds eight points (16 floats), so the 8 floats read end at index 15 at most.
        return Vector256.LoadUnsafe(ref Unsafe.Add(ref Unsafe.As<Vec2x8, float>(ref Unsafe.AsRef(in points)), 2 * first));
    }
}
