using System;
using System.Runtime;

namespace RecursiveAI.OnePhysics.Tests.Core;

/// <summary>
/// Counts what the library allocates on the calling thread. When the runtime compiles a method on the thread in the middle of a window (the first run of a path that nothing exercised before),
/// the compiler's own allocations land in the same counter, whatever the code under test does; such a window is not counted. A library allocation repeats in every window, so it still shows in the others.
/// </summary>
internal static class AllocMeter
{
    /// <summary>Bytes allocated by <paramref name="action"/> on this thread, or 0 when a method was compiled on this thread while it ran.</summary>
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
    public static Window Start() => new(JitInfo.GetCompiledMethodCount(currentThread: true), GC.GetAllocatedBytesForCurrentThread());

    /// <summary>An open measuring window.</summary>
    public readonly struct Window(long compiled, long allocated)
    {
        /// <summary>Bytes allocated since <see cref="Start"/>, or 0 when a method was compiled on this thread meanwhile.</summary>
        public long Bytes()
        {
            long bytes = GC.GetAllocatedBytesForCurrentThread() - allocated;
            return JitInfo.GetCompiledMethodCount(currentThread: true) == compiled ? bytes : 0;
        }
    }
}
