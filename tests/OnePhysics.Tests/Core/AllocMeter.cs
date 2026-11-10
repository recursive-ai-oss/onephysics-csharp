using System;
using System.Runtime;
using System.Text;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

/// <summary>
/// Counts what the calling thread allocates in a window.
/// <para>
/// One thing besides the library can allocate on the calling thread: the runtime itself, when it compiles a method there. The first run of a path nothing exercised before, and the
/// on-stack replacement of a loop that crossed its hit count (the count is shared by every test in the process, so which test's thread pays for the compile depends on timing)
/// both compile on the thread that needs the code, and the compiler's own allocations land in that thread's counter whatever the code under test does. A library allocation, by contrast,
/// repeats in every window. So a window in which this thread compiled something is not judged: <see cref="Of"/> and <see cref="Window.Bytes"/> read it as zero, and a run of many windows (one per step) still
/// shows a library allocation in the others.
/// </para>
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
    public static Window Start() => new(GC.GetAllocatedBytesForCurrentThread(), JitInfo.GetCompiledMethodCount(currentThread: true));

    /// <summary>An open measuring window.</summary>
    public readonly struct Window(long allocated, long compiled)
    {
        /// <summary>Bytes allocated since <see cref="Start"/>, or 0 when a method was compiled on this thread meanwhile.</summary>
        public long Bytes() => Compiled() ? 0 : Raw();

        /// <summary>Bytes allocated since <see cref="Start"/>, compiler included.</summary>
        public long Raw() => GC.GetAllocatedBytesForCurrentThread() - allocated;

        /// <summary>True when the runtime compiled a method on this thread since <see cref="Start"/>, which puts the compiler's own allocations into <see cref="Raw"/>.</summary>
        public bool Compiled() => JitInfo.GetCompiledMethodCount(currentThread: true) != compiled;
    }

    /// <summary>
    /// Fails unless the code allocates nothing on this thread in steady state: after a collection it runs <paramref name="work"/> in up to <paramref name="attempts"/> windows and
    /// passes as soon as a window in which no method was compiled on this thread reads zero bytes. The failure message lists every window.
    /// </summary>
    public static void AssertAllocatesNothing(Action work, string what, int attempts = 8)
    {
        Settle();
        var log = new StringBuilder();
        long best = long.MaxValue;
        for (int attempt = 0; attempt < attempts; attempt++)
        {
            var window = Start();
            work();
            long bytes = window.Raw();
            bool compiled = window.Compiled();
            log.Append(" [").Append(bytes).Append(" B").Append(compiled ? ", compiled on this thread" : "").Append(']');
            if (compiled) { continue; }
            best = Math.Min(best, bytes);
            if (best == 0) { return; }
        }

        Assert.Fail(what + (best == long.MaxValue ? ": every window compiled code on the calling thread:" : ": the calling thread allocated in every window without a compile:") + log);
    }
}
