using System;

namespace RecursiveAI.OnePhysics.LowLevel;

internal static class ForkArrays
{
    internal static T[][] CopyRows<T>(T[][] source)
    {
        var copy = new T[source.Length][];
        for (int i = 0; i < copy.Length; i++) { copy[i] = source[i] == null ? null! : (T[])source[i].Clone(); }
        return copy;
    }

    /// <summary>
    /// Makes <paramref name="destination"/> an independent copy of <paramref name="source"/>. An array of the same length is overwritten in place, so copying a state into a world that already has the storage allocates nothing
    /// (a mark and a rollback rely on it); anything else gets a fresh clone, which is what a fork into a new world does.
    /// </summary>
    internal static void Into<T>(ref T[] destination, T[] source)
    {
        if (destination != null && destination.Length == source.Length) { Array.Copy(source, destination, source.Length); }
        else { destination = (T[])source.Clone(); }
    }

    /// <summary>The same as <see cref="Into{T}"/> for an array of rows (each row copied in place when the shapes agree).</summary>
    internal static void RowsInto<T>(ref T[][] destination, T[][] source)
    {
        if (destination == null || destination.Length != source.Length) { destination = CopyRows(source); return; }
        for (int i = 0; i < source.Length; i++)
        {
            if (source[i] == null) { destination[i] = null!; }
            else if (destination[i] == null || destination[i].Length != source[i].Length) { destination[i] = (T[])source[i].Clone(); }
            else { Array.Copy(source[i], destination[i], source[i].Length); }
        }
    }

    /// <summary>
    /// Like <see cref="Into{T}"/> for storage of which only the first <paramref name="count"/> elements mean anything (slots below a high-water mark): an array of the same length copies just that prefix,
    /// so the unused capacity, often most of a payload array, is never touched. What lies above the prefix in the destination is whatever the slots held before, as it is for any freed slot.
    /// </summary>
    internal static void Prefix<T>(ref T[] destination, T[] source, int count) => Prefix(ref destination, source, count, source.Length);

    /// <summary>The same, giving the destination <paramref name="length"/> elements (at least <paramref name="count"/>), for a copy that should not carry the source's spare capacity.</summary>
    internal static void Prefix<T>(ref T[] destination, T[] source, int count, int length)
    {
        if (destination == null || destination.Length != length) { destination = new T[length]; }
        Array.Copy(source, destination, count);
    }
}
