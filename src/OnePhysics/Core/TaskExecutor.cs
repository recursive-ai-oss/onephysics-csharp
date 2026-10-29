using System;
using System.Threading;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>One unit of parallel work, run with a task index. Tasks of one call must touch disjoint data (or only read shared data).</summary>
[Experimental("OPHYS0001")]
public interface IParallelBody
{
    /// <summary>Runs task number <paramref name="task"/>.</summary>
    void Run(int task);
}

/// <summary>
/// A sequence of stages, each a set of independent tasks; every task of a stage finishes before any task of the next stage starts. <see cref="TaskCount"/> must depend only on data that does not
/// change during the call.
/// </summary>
[Experimental("OPHYS0001")]
public interface IStagedBody
{
    /// <summary>Number of tasks in stage <paramref name="stage"/> (zero skips the stage).</summary>
    int TaskCount(int stage);

    /// <summary>Runs task <paramref name="task"/> of stage <paramref name="stage"/>.</summary>
    void Run(int stage, int task);

    /// <summary>Runs tasks <c>first .. first + count - 1</c> of a stage in order, on one thread; used when a stage is not shared. The default calls <see cref="Run"/> for each; an implementation may decode the stage once.</summary>
    void RunRange(int stage, int first, int count)
    {
        for (int i = 0; i < count; i++) { Run(stage, first + i); }
    }
}

/// <summary>
/// Runs independent tasks on up to <see cref="ThreadCount"/> threads. The task split is chosen by the caller from the data alone, never from the thread count,
/// and tasks write only their own slots, so results are the same for any thread count. This is the only place in the core that creates parallelism.
/// </summary>
/// <remarks>
/// Worker threads are shared by every executor in the process and stay alive for it (they spin briefly after a call, then sleep, so a world that is no longer stepped costs nothing).
/// A call that finds the workers busy with another world's step, or that runs on a platform without threads, simply runs its tasks on the calling thread: the answer is the same.
/// </remarks>
[Experimental("OPHYS0001")]
public sealed class TaskExecutor
{
    /// <summary>Creates an executor. <paramref name="threadCount"/> is at least 1; 1 never leaves the calling thread (always the case on single-threaded WASM).</summary>
    public TaskExecutor(int threadCount)
    {
        if (threadCount < 1) { throw new ArgumentOutOfRangeException(nameof(threadCount), threadCount, "threadCount must be at least 1."); }
        ThreadCount = threadCount;
    }

    // A staged call needs at least this many tasks in some stage before waking the workers pays for itself.
    private const int MinParallelTasks = 4;

    // Set while a throwaway world primes the process (Warmup): every call then goes through the worker pool, however few tasks it has, so that the pool's per-body-type state is created
    // before the host's steps, not at the first step that happens to be big enough. Only which thread runs a task changes; results never do.
    [ThreadStatic] private static bool s_primeParallel;

    internal static void SetPrimeParallel(bool on) => s_primeParallel = on;

    /// <summary>Maximum number of threads working at once.</summary>
    public int ThreadCount { get; }

    /// <summary>Starts the pool threads this executor may use and sizes the per-stage counters for calls of up to <paramref name="stages"/> stages, so the first parallel step allocates nothing. Does nothing without threads.</summary>
    internal void Reserve(int stages)
    {
        if (ThreadCount > 1) { WorkerPool.Reserve(ThreadCount - 1, stages); }
    }

    /// <summary>Runs <paramref name="body"/> for every task in <c>0..taskCount-1</c> and returns when all are done. Order of execution is unspecified; the result must not depend on it.</summary>
    public void For<TBody>(int taskCount, TBody body) where TBody : IParallelBody
    {
        if (ThreadCount == 1 || taskCount <= 0 || (taskCount == 1 && !s_primeParallel) || !WorkerPool.TryRun(Math.Min(ThreadCount - 1, taskCount - 1), taskCount, in body))
        {
            for (int i = 0; i < taskCount; i++) { body.Run(i); }
        }
    }

    /// <summary>
    /// Runs <paramref name="stageCount"/> stages in order, the tasks of each stage in parallel, and returns when the last one is done. The threads stay inside the call from the first stage to the last and
    /// pass the stage boundaries by polling shared counters, which costs far less than one <see cref="For{TBody}"/> call per stage when stages are short. The result must not depend on which thread runs which task.
    /// </summary>
    /// <param name="stageCount">Number of stages.</param>
    /// <param name="body">The stages.</param>
    /// <param name="mostTasks">An upper bound on the task count of any one stage; when it is below four there is too little to share and the call stays on the calling thread.</param>
    public void ForStages<TBody>(int stageCount, TBody body, int mostTasks = int.MaxValue) where TBody : IStagedBody
    {
        if (stageCount <= 0) { return; }
        int workers = ThreadCount - 1;
        if (workers > 0 && mostTasks >= MinParallelTasks && !s_primeParallel)
        {
            // Every thread crosses every stage boundary, so threads beyond what the stages can keep busy only add waiting: take as many as there is one task each in the average stage.
            // This looks at the stage sizes alone; which thread runs which task never reaches a result.
            long total = 0;
            for (int s = 0; s < stageCount; s++) { total += body.TaskCount(s); }
            long useful = total / stageCount;
            if (useful - 1 < workers) { workers = (int)Math.Max(useful - 1, 0); }
        }

        if (workers == 0 || (mostTasks < MinParallelTasks && !s_primeParallel) || !WorkerPool.TryRunStages(workers, stageCount, in body))
        {
            for (int s = 0; s < stageCount; s++)
            {
                body.RunRange(s, 0, body.TaskCount(s));
            }
        }
    }
}

/// <summary>A reusable holder that lets the shared workers run a body of any struct type without allocating per call.</summary>
internal abstract class PoolJob
{
    public virtual bool Staged => false;

    public abstract void Run(int task);

    public virtual int Stages => 0;

    public virtual int TaskCount(int stage) => 0;

    public virtual void RunStage(int stage, int task) { }
}

internal sealed class StagedJob<TBody> : PoolJob where TBody : IStagedBody
{
    public static readonly StagedJob<TBody> Instance = new();

    public TBody Body = default!;
    public int StageCount;

    public override bool Staged => true;

    public override void Run(int task) { }

    public override int Stages => StageCount;

    public override int TaskCount(int stage) => Body.TaskCount(stage);

    public override void RunStage(int stage, int task) => Body.Run(stage, task);
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
        public int InitSeen;  // the last thread-init version this worker ran
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
    private const long ClosedBit = 0x80000000L;
    private static long _stagedState;          // (call << 32) | closed bit | number of threads inside the staged call
    private static long[] _stageCounters = new long[64 * 8]; // per stage and participant slot, one cache line: tasks claimed, then tasks finished
    private static int _slots;                 // participants of the current staged call: the caller (slot 0) and the workers (slot index + 1)

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

    internal static void Reserve(int workers, int stages)
    {
        if (OperatingSystem.IsBrowser()) { return; }
        if (workers > MaxWorkers) { workers = MaxWorkers; }
        // Only when no other world is stepping on the pool: the counters are shared.
        if (Interlocked.CompareExchange(ref _owner, 1, 0) != 0) { return; }
        try
        {
            if (_workers.Length < workers) { Grow(workers); }
            int needed = stages * (workers + 1) * CounterStride;
            if (_stageCounters.Length < needed) { _stageCounters = new long[needed]; }
        }
        finally
        {
            Volatile.Write(ref _owner, 0);
        }
    }

    /// <summary>Runs a staged body with up to <paramref name="workers"/> pool threads plus the caller; false when the pool is not available.</summary>
    public static bool TryRunStages<TBody>(int workers, int stages, in TBody body) where TBody : IStagedBody
    {
        if (OperatingSystem.IsBrowser()) { return false; }
        if (Interlocked.CompareExchange(ref _owner, 1, 0) != 0) { return false; }
        try
        {
            if (workers > MaxWorkers) { workers = MaxWorkers; }
            if (_workers.Length < workers && !Grow(workers)) { return false; }
            var job = StagedJob<TBody>.Instance;
            int slots = workers + 1;
            int needed = stages * slots * CounterStride;
            if (_stageCounters.Length < needed) { _stageCounters = new long[Math.Max(needed, _stageCounters.Length * 2)]; }
            Array.Clear(_stageCounters, 0, needed);
            _slots = slots;

            long call = (Volatile.Read(ref _claim) >> 32) + 1;
            Volatile.Write(ref _claim, (call << 32) | Closed);
            job.Body = body;
            job.StageCount = stages;
            _job = job;
            _taskCount = 0;
            _limit = workers;
            _error = null;
            Volatile.Write(ref _stagedState, (call << 32) | 1);
            Interlocked.Exchange(ref _claim, call << 32);

            var all = _workers;
            for (int i = 0; i < workers; i++)
            {
                if (Interlocked.CompareExchange(ref all[i].Sleeping, 0, 1) == 1) { Signal(all[i]); }
            }

            RunStageLoop(job, 0);

            // No thread may join from here on; wait for those inside to leave before the job and counters are reused.
            Volatile.Write(ref _claim, (call << 32) | Closed);
            while (true)
            {
                long st = Volatile.Read(ref _stagedState);
                if (Interlocked.CompareExchange(ref _stagedState, st | ClosedBit, st) == st) { break; }
            }

            var wait = new SpinWait();
            while ((Volatile.Read(ref _stagedState) & 0x7FFFFFFFL) > 1) { wait.SpinOnce(-1); }
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

    private const int CounterStride = 8; // longs per slot: one 64-byte cache line, so a thread working through its own tasks never shares a line with another

    // Every thread inside the call walks all stages in order. The tasks of a stage are cut into one contiguous block per participant: a thread runs its own block first (the same tasks, on the same thread,
    // in every pass of the solver, so the constraint groups stay in that core's cache) and then helps itself to what is left of the other blocks. It then waits for every block of the stage to finish.
    // A thread that joins late, or never, costs nothing but locality: the others take over its block. Which thread runs a task never matters, tasks write disjoint data.
    private static void RunStageLoop(PoolJob job, int slot)
    {
        var counters = _stageCounters;
        int stages = job.Stages;
        int slots = _slots;
        for (int s = 0; s < stages; s++)
        {
            int n = job.TaskCount(s);
            if (n <= 0) { continue; }
            int size = (n + slots - 1) / slots;
            int baseIndex = s * slots * CounterStride;
            for (int k = 0; k < slots; k++)
            {
                int owner = slot + k;
                if (owner >= slots) { owner -= slots; }
                int first = owner * size;
                int count = Math.Min(size, n - first);
                if (count <= 0) { continue; }
                int c = baseIndex + (owner * CounterStride);
                while (true)
                {
                    // A plain read first: a block that is already fully claimed costs a shared read instead of an atomic add on a line other threads are writing.
                    if (k != 0 && Volatile.Read(ref counters[c]) >= count) { break; }
                    long i = Interlocked.Increment(ref counters[c]) - 1;
                    if (i >= count) { break; }
                    try
                    {
                        job.RunStage(s, first + (int)i);
                    }
                    catch (Exception e)
                    {
                        Interlocked.CompareExchange(ref _error, e, null);
                    }

                    Interlocked.Increment(ref counters[c + 1]);
                }
            }

            for (int owner = 0; owner < slots; owner++)
            {
                int first = owner * size;
                int count = Math.Min(size, n - first);
                if (count <= 0) { break; }
                int c = baseIndex + (owner * CounterStride) + 1;
                if (Volatile.Read(ref counters[c]) < count)
                {
                    var wait = new SpinWait();
                    while (Volatile.Read(ref counters[c]) < count) { wait.SpinOnce(-1); }
                }
            }
        }
    }

    private static void JoinStaged(PoolJob job, long call, int index)
    {
        if (index >= _limit) { return; }
        while (true)
        {
            long st = Volatile.Read(ref _stagedState);
            if ((st >> 32) != call || (st & ClosedBit) != 0) { return; }
            if (Interlocked.CompareExchange(ref _stagedState, st + 1, st) == st) { break; }
        }

        try
        {
            RunStageLoop(job, index + 1);
        }
        finally
        {
            Interlocked.Decrement(ref _stagedState);
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

    private static Action? _threadInit;
    private static int _initVersion;

    /// <summary>
    /// Registers work every pool thread runs once, before it takes a task after the registration: an engine uses it to create the per-thread scratch its steps would otherwise create on first use
    /// (on a worker that is at an arbitrary step, which depends on timing). Returns after every worker started so far has run it, so threads created by an earlier world are covered as well.
    /// Registering the same action again does nothing.
    /// </summary>
    internal static void AddThreadInit(Action init)
    {
        int version;
        Worker[] all;
        lock (CreateLock)
        {
            if (_threadInit != null && Array.IndexOf(_threadInit.GetInvocationList(), init) >= 0) { return; }
            _threadInit = (Action?)Delegate.Combine(_threadInit, init);
            version = Interlocked.Increment(ref _initVersion);
            all = _workers;
        }

        // Workers asleep are woken to run it; workers polling see the new version at their next round.
        foreach (var w in all)
        {
            if (Interlocked.CompareExchange(ref w.Sleeping, 0, 1) == 1) { Signal(w); }
        }

        // A worker that cannot be scheduled for several seconds is not waited for; it runs the init when it next wakes.
        var wait = new SpinWait();
        foreach (var w in all)
        {
            for (int i = 0; i < 10000 && Volatile.Read(ref w.InitSeen) != version; i++) { wait.SpinOnce(); }
        }
    }

    private static void RunThreadInit(Worker w)
    {
        int version = Volatile.Read(ref _initVersion);
        if (w.InitSeen == version) { return; }
        Volatile.Read(ref _threadInit)?.Invoke();
        Volatile.Write(ref w.InitSeen, version);
    }

    private static void Loop(Worker w)
    {
        RunThreadInit(w);
        long seen = Volatile.Read(ref _claim) >> 32;
        while (true)
        {
            RunThreadInit(w);
            int spins = 0;
            while ((Volatile.Read(ref _claim) >> 32) == seen && (Volatile.Read(ref _claim) & 0xFFFFFFFFL) == Closed && w.InitSeen == Volatile.Read(ref _initVersion))
            {
                if (++spins < SpinRounds) { Thread.SpinWait(4); continue; }
                Interlocked.Exchange(ref w.Sleeping, 1);
                long now = Volatile.Read(ref _claim);
                if ((now >> 32) != seen || (now & 0xFFFFFFFFL) != Closed || w.InitSeen != Volatile.Read(ref _initVersion))
                {
                    // A call or an init arrived while going to sleep: undo, or absorb the wake-up the caller has already sent.
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
            if (job != null && job.Staged) { JoinStaged(job, v >> 32, index); return; }
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
