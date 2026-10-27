using System;
using System.Threading;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>One unit of parallel work, run with a task index. Tasks of one call must touch disjoint data (or only read shared data).</summary>
public interface IParallelBody
{
    /// <summary>Runs task number <paramref name="task"/>.</summary>
    void Run(int task);
}

/// <summary>
/// Runs independent tasks on up to <see cref="ThreadCount"/> threads. The task split is chosen by the caller from the data alone, never from the thread count,
/// and tasks write only their own slots, so results are the same for any thread count. This is the only place in the core that creates parallelism.
/// </summary>
/// <remarks>
/// Worker threads are shared by every executor in the process and stay alive for it (they spin briefly after a call, then sleep, so a world that is no longer stepped costs nothing).
/// A call that finds the workers busy with another world's step, or that runs on a platform without threads, simply runs its tasks on the calling thread: the answer is the same.
/// </remarks>
public sealed class TaskExecutor
{
    /// <summary>Creates an executor. <paramref name="threadCount"/> is at least 1; 1 never leaves the calling thread (always the case on single-threaded WASM).</summary>
    public TaskExecutor(int threadCount)
    {
        if (threadCount < 1) { throw new ArgumentOutOfRangeException(nameof(threadCount), threadCount, "threadCount must be at least 1."); }
        ThreadCount = threadCount;
    }

    /// <summary>Maximum number of threads working at once.</summary>
    public int ThreadCount { get; }

    /// <summary>Runs <paramref name="body"/> for every task in <c>0..taskCount-1</c> and returns when all are done. Order of execution is unspecified; the result must not depend on it.</summary>
    public void For<TBody>(int taskCount, TBody body) where TBody : IParallelBody
    {
        if (ThreadCount == 1 || taskCount <= 1 || !WorkerPool.TryRun(ThreadCount - 1, taskCount, in body))
        {
            for (int i = 0; i < taskCount; i++) { body.Run(i); }
        }
    }
}

/// <summary>A reusable holder that lets the shared workers run a body of any struct type without allocating per call.</summary>
internal abstract class PoolJob
{
    public abstract void Run(int task);
}

internal sealed class PoolJob<TBody> : PoolJob where TBody : IParallelBody
{
    public static readonly PoolJob<TBody> Instance = new();

    public TBody Body = default!;

    public override void Run(int task) => Body.Run(task);
}

/// <summary>
/// The shared worker threads. A call publishes a job and a claim word (call number in the high half, next task index in the low half); every thread, the caller included, claims task indices
/// with a compare-and-swap and runs them. The job fields are written only while the claim word is closed, and a claim succeeds only if the word is unchanged since the fields were read,
/// so a thread that was slow to wake can never run a task of a later call under an earlier call's description. Which thread runs which task never matters: tasks write disjoint data.
/// </summary>
internal static class WorkerPool
{
    private const int SpinRounds = 4000;     // about a few tens of microseconds of polling before a worker sleeps
    private const int MaxWorkers = 255;
    private const long Closed = 0x7FFFFFFF;  // low half of a closed claim word: no index below the task count is left

    private sealed class Worker
    {
        public int Index;
        public int Sleeping;
        public bool Signaled; // guarded by the worker object's monitor
    }

    private static readonly object CreateLock = new();
    private static Worker[] _workers = [];
    private static int _owner;
    private static long _claim = Closed;       // (call << 32) | next task index
    private static PoolJob? _job;
    private static int _taskCount;
    private static int _limit;                 // workers with an index below this take part
    private static int _completed;
    private static Exception? _error;

    /// <summary>Runs the tasks with up to <paramref name="workers"/> pool threads plus the caller; false when the pool is not available (the caller then runs them itself).</summary>
    public static bool TryRun<TBody>(int workers, int taskCount, in TBody body) where TBody : IParallelBody
    {
        if (OperatingSystem.IsBrowser()) { return false; }
        if (Interlocked.CompareExchange(ref _owner, 1, 0) != 0) { return false; }
        try
        {
            if (workers > MaxWorkers) { workers = MaxWorkers; }
            if (_workers.Length < workers && !Grow(workers)) { return false; }
            var job = PoolJob<TBody>.Instance;

            // Close the claim word, write the job, open it for the next call.
            long call = (Volatile.Read(ref _claim) >> 32) + 1;
            Volatile.Write(ref _claim, (call << 32) | Closed);
            job.Body = body;
            _job = job;
            _taskCount = taskCount;
            _limit = workers;
            _error = null;
            Volatile.Write(ref _completed, 0);
            Interlocked.Exchange(ref _claim, call << 32);

            var all = _workers;
            for (int i = 0; i < workers; i++)
            {
                if (Interlocked.CompareExchange(ref all[i].Sleeping, 0, 1) == 1) { Signal(all[i]); }
            }

            Work(-1);
            var wait = new SpinWait();
            while (Volatile.Read(ref _completed) < taskCount) { wait.SpinOnce(-1); }
            Volatile.Write(ref _claim, (call << 32) | Closed);
            job.Body = default!;
            _job = null;
            var error = _error;
            if (error != null) { throw new AggregateException(error); }
            return true;
        }
        finally
        {
            Volatile.Write(ref _owner, 0);
        }
    }

    private static void Signal(Worker w)
    {
        lock (w)
        {
            w.Signaled = true;
            Monitor.Pulse(w);
        }
    }

    private static void WaitSignal(Worker w)
    {
        lock (w)
        {
            while (!w.Signaled) { Monitor.Wait(w); }
            w.Signaled = false;
        }
    }

    private static bool Grow(int workers)
    {
        lock (CreateLock)
        {
            var current = _workers;
            if (current.Length >= workers) { return true; }
            var grown = new Worker[workers];
            Array.Copy(current, grown, current.Length);
            try
            {
                for (int i = current.Length; i < workers; i++)
                {
                    var w = new Worker { Index = i };
                    grown[i] = w;
                    new Thread(() => Loop(w)) { IsBackground = true, Name = "OnePhysics worker " + i }.Start();
                }
            }
            catch (Exception e) when (e is OutOfMemoryException or ThreadStartException or PlatformNotSupportedException)
            {
                // No threads here: keep what started and let the caller run alone.
                int started = 0;
                while (started < grown.Length && grown[started] != null) { started++; }
                Array.Resize(ref grown, started);
                _workers = grown;
                return false;
            }

            _workers = grown;
            return true;
        }
    }

    private static void Loop(Worker w)
    {
        long seen = Volatile.Read(ref _claim) >> 32;
        while (true)
        {
            int spins = 0;
            while ((Volatile.Read(ref _claim) >> 32) == seen && (Volatile.Read(ref _claim) & 0xFFFFFFFFL) == Closed)
            {
                if (++spins < SpinRounds) { Thread.SpinWait(4); continue; }
                Interlocked.Exchange(ref w.Sleeping, 1);
                long now = Volatile.Read(ref _claim);
                if ((now >> 32) != seen || (now & 0xFFFFFFFFL) != Closed)
                {
                    // A call arrived while going to sleep: undo, or absorb the wake-up the caller has already sent.
                    if (Interlocked.CompareExchange(ref w.Sleeping, 0, 1) != 1) { WaitSignal(w); }
                    break;
                }

                WaitSignal(w);
                spins = 0;
            }

            long v = Volatile.Read(ref _claim);
            seen = v >> 32;
            if ((v & 0xFFFFFFFFL) != Closed) { Work(w.Index); }
            else { Thread.SpinWait(4); }
        }
    }

    // Claims and runs tasks of the current call until none is left. A thread outside the call's worker limit does nothing.
    private static void Work(int index)
    {
        while (true)
        {
            long v = Volatile.Read(ref _claim);
            int next = (int)(v & 0xFFFFFFFFL);
            if (next == Closed) { return; }
            var job = _job;
            int count = _taskCount, limit = _limit;
            if (job == null || next >= count || (index >= limit)) { if (Volatile.Read(ref _claim) == v) { return; } continue; }
            if (Interlocked.CompareExchange(ref _claim, v + 1, v) != v) { continue; }
            try
            {
                job.Run(next);
            }
            catch (Exception e)
            {
                Interlocked.CompareExchange(ref _error, e, null);
            }

            Interlocked.Increment(ref _completed);
        }
    }
}
