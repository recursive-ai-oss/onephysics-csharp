using System;
using System.Threading;

namespace RecursiveAI.OnePhysics.Experiments.Batching;

/// <summary>A unit of parallel work: part <c>k</c> of <c>parts</c>.</summary>
internal interface IPartJob
{
    void Execute(int part);
}

/// <summary>
/// A small pool of spinning worker threads. The caller runs part 0 and workers run parts 1..Threads-1, so a fixed split
/// is dispatched with no allocation. Which thread runs which part never reaches the results: parts write disjoint data.
/// The thread count is a constructor argument, never read from the machine.
/// </summary>
internal sealed class SpinPool : IDisposable
{
    private readonly Thread[] _threads;
    private IPartJob? _job;
    private int _generation;
    private int _remaining;
    private volatile bool _stop;

    public SpinPool(int threads)
    {
        Threads = threads;
        if (threads > 1 && OperatingSystem.IsBrowser())
        {
            throw new PlatformNotSupportedException("Browser WebAssembly runs on one thread.");
        }
        _threads = new Thread[Math.Max(0, threads - 1)];
        for (int i = 0; i < _threads.Length; i++)
        {
            int part = i + 1;
            _threads[i] = new Thread(() => Worker(part)) { IsBackground = true };
            _threads[i].Start();
        }
    }

    public int Threads { get; }

    /// <summary>Runs <c>job.Execute(k)</c> for every k in 0..Threads-1 (every worker takes part in every run) and waits for all of them.</summary>
    public void Run(IPartJob job)
    {
        if (_threads.Length == 0)
        {
            job.Execute(0);
            return;
        }
        _job = job;
        Volatile.Write(ref _remaining, _threads.Length);
        Interlocked.Increment(ref _generation);
        job.Execute(0);
        var spin = new SpinWait();
        while (Volatile.Read(ref _remaining) != 0)
        {
            spin.SpinOnce(-1);
        }
    }

    private void Worker(int part)
    {
        int seen = 0;
        var spin = new SpinWait();
        while (!_stop)
        {
            int g = Volatile.Read(ref _generation);
            if (g == seen)
            {
                spin.SpinOnce(-1);
                continue;
            }
            seen = g;
            spin.Reset();
            _job!.Execute(part);
            Interlocked.Decrement(ref _remaining);
        }
    }

    public void Dispose()
    {
        _stop = true;
        foreach (Thread t in _threads)
        {
            t.Join();
        }
    }
}
