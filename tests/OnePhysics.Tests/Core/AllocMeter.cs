using System;

namespace RecursiveAI.OnePhysics.Tests.Core;

/// <summary>Counts what the calling thread allocates in a window.</summary>
internal static class AllocMeter
{
    /// <summary>Bytes allocated by <paramref name="action"/> on this thread.</summary>
    public static long Of(Action action)
    {
        var window = Start();
        action();
        return window.Bytes();
    }

    /// <summary>
    /// Collects garbage before a run of windows. Without it, a world set up a moment ago (large arrays, a fresh snapshot buffer) makes the runtime's allocator report a few KB on the calling
    /// thread in some window of its first hundred steps, when the machine is busy, whichever call that window is in and with the code under test allocating nothing: the same worlds,
    /// started after a collection or a pause, report zero. A library allocation is not hidden by this, since it repeats in every window.
    /// </summary>
    public static void Settle()
    {
        GC.Collect();
        GC.WaitForPendingFinalizers();
    }

    /// <summary>Starts a window that spans code which cannot be wrapped in a delegate.</summary>
    public static Window Start() => new(GC.GetAllocatedBytesForCurrentThread());

    /// <summary>An open measuring window.</summary>
    public readonly struct Window(long allocated)
    {
        /// <summary>Bytes allocated since <see cref="Start"/>.</summary>
        public long Bytes() => GC.GetAllocatedBytesForCurrentThread() - allocated;
    }
}
