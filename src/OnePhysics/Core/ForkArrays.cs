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
}
