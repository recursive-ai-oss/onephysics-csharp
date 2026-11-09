using System;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Mixed convex contacts keep their bytes and events across widths, thread counts, forks and restore.</summary>
public sealed class ConvexBatchReplayTests
{
    [Fact]
    public void WidthsForkAndRestoreKeepIdenticalStateAndEvents()
    {
        var scalar = World3DTests.BuildScene(1);
        var four = World3DTests.BuildScene(8);
        var eight = World3DTests.BuildScene(8);
        scalar.Engine.KernelWidth = 1; four.Engine.KernelWidth = 4; eight.Engine.KernelWidth = 8;
        for (int step = 0; step < 120; step++)
        {
            scalar.Step(1f / 60f); four.Step(1f / 60f); eight.Step(1f / 60f);
            Assert.Equal(scalar.ComputeHash(), four.ComputeHash()); Assert.Equal(scalar.ComputeHash(), eight.ComputeHash());
            Assert.Equal(scalar.Events.ToArray(), four.Events.ToArray()); Assert.Equal(scalar.Events.ToArray(), eight.Events.ToArray());
        }
        byte[] snapshot = scalar.Snapshot();
        var fork = scalar.Fork(); fork.Engine.KernelWidth = 8;
        four.Restore(snapshot);
        for (int step = 0; step < 60; step++)
        {
            scalar.Step(1f / 60f); four.Step(1f / 60f); fork.Step(1f / 60f);
            Assert.Equal(scalar.ComputeHash(), four.ComputeHash()); Assert.Equal(scalar.ComputeHash(), fork.ComputeHash());
        }
        Assert.Equal(scalar.Snapshot(), four.Snapshot()); Assert.Equal(scalar.Snapshot(), fork.Snapshot());
    }
}
