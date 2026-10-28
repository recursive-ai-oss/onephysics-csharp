using System.Threading;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

public class StagedExecutorTests
{
    // Stage s adds (s + 1) * (task + 1) to a shared sum, but only after checking that every task of the stage before has finished.
    private readonly struct Program : IStagedBody
    {
        private readonly long[] _done; private readonly int[] _tasks; private readonly long[] _sum; private readonly int[] _violations;

        public Program(long[] done, int[] tasks, long[] sum, int[] violations) { _done = done; _tasks = tasks; _sum = sum; _violations = violations; }

        public int TaskCount(int stage) => _tasks[stage];

        public void Run(int stage, int task)
        {
            if (stage > 0 && Volatile.Read(ref _done[stage - 1]) != _tasks[stage - 1]) { Interlocked.Increment(ref _violations[0]); }
            Interlocked.Add(ref _sum[0], (stage + 1) * (task + 1));
            Interlocked.Increment(ref _done[stage]);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(4)]
    [InlineData(8)]
    public void Stages_run_in_order_and_every_task_runs_once(int threads)
    {
        var exec = new TaskExecutor(threads);
        var tasks = new int[200];
        long expected = 0;
        for (int s = 0; s < tasks.Length; s++)
        {
            tasks[s] = (s * 7) % 11; // includes empty stages
            for (int t = 0; t < tasks[s]; t++) { expected += (s + 1) * (t + 1); }
        }

        for (int rep = 0; rep < 50; rep++)
        {
            var done = new long[tasks.Length];
            var sum = new long[1];
            var violations = new int[1];
            exec.ForStages(tasks.Length, new Program(done, tasks, sum, violations));
            Assert.Equal(expected, sum[0]);
            Assert.Equal(0, violations[0]);
            for (int s = 0; s < tasks.Length; s++) { Assert.Equal(tasks[s], done[s]); }
        }
    }

    private static readonly int[] TwoEach = { 2, 2, 2 };

    private readonly struct Throws : IStagedBody
    {
        public int TaskCount(int stage) => 3;

        public void Run(int stage, int task)
        {
            if (stage == 2 && task == 1) { throw new System.InvalidOperationException("boom"); }
        }
    }

    [Fact]
    public void A_task_that_throws_is_reported_after_the_call_and_the_pool_stays_usable()
    {
        var exec = new TaskExecutor(4);
        Assert.ThrowsAny<System.Exception>(() => exec.ForStages(5, new Throws()));
        var sum = new long[1];
        exec.ForStages(3, new Program(new long[3], TwoEach, sum, new int[1]));
        Assert.Equal((1 * 1) + (1 * 2) + (2 * 1) + (2 * 2) + (3 * 1) + (3 * 2), sum[0]);
    }
}
