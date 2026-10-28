using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using RecursiveAI.OnePhysics.Collision2D;

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
    public static ref readonly float At(ReadOnlySpan<float> s, int offset) => ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);

    /// <summary>A writable reference to element <paramref name="offset"/> without a bounds check; the caller has established the range (see <see cref="Range"/>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref float At(Span<float> s, int offset) => ref Unsafe.Add(ref MemoryMarshal.GetReference(s), offset);

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
