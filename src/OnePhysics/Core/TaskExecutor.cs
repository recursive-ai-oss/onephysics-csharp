using System;
using System.Threading.Tasks;

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
public sealed class TaskExecutor
{
    private readonly ParallelOptions _options;

    /// <summary>Creates an executor. <paramref name="threadCount"/> is at least 1; 1 never leaves the calling thread (always the case on single-threaded WASM).</summary>
    public TaskExecutor(int threadCount)
    {
        if (threadCount < 1) { throw new ArgumentOutOfRangeException(nameof(threadCount), threadCount, "threadCount must be at least 1."); }
        ThreadCount = threadCount;
        _options = new ParallelOptions { MaxDegreeOfParallelism = threadCount };
    }

    /// <summary>Maximum number of threads working at once.</summary>
    public int ThreadCount { get; }

    /// <summary>Runs <paramref name="body"/> for every task in <c>0..taskCount-1</c> and returns when all are done. Order of execution is unspecified; the result must not depend on it.</summary>
    public void For<TBody>(int taskCount, TBody body) where TBody : IParallelBody
    {
        if (ThreadCount == 1 || taskCount <= 1)
        {
            for (int i = 0; i < taskCount; i++) { body.Run(i); }
            return;
        }

        Parallel.For(0, taskCount, _options, i => body.Run(i));
    }
}
