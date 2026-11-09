using System;
using System.Linq;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class TerrainEditSceneTests
{
    [Theory]
    [InlineData("sdf-pile-3d")]
    [InlineData("voxel-dig-3d")]
    [InlineData("carve-2d")]
    public void ScenesStayWithinTheirLimits(string name)
    {
        var s = TerrainEditScenes.Build(name, 1);
        for (int i = 0; i < s.Steps; i++) { s.Script(i); s.Step(); s.Observe(i); }
        foreach (var (metric, min, max) in TerrainEditScenes.Limits[name])
            Assert.True(s.Metrics.TryGetValue(metric, out double v) && v >= min && v <= max, name + " " + metric + "=" + (s.Metrics.TryGetValue(metric, out double w) ? w : double.NaN) + " outside [" + min + ", " + max + "]");
    }

    [Theory]
    [InlineData("sdf-pile-3d")]
    [InlineData("voxel-dig-3d")]
    [InlineData("carve-2d")]
    public void ScenesAgreeOnOneAndEightThreadsAndAfterARestore(string name)
    {
        ulong a = TerrainEditScenes.Run(name, 1, false, 120), b = TerrainEditScenes.Run(name, 8, false, 120), c = TerrainEditScenes.Run(name, 1, true, 120);
        Assert.Equal(a, b); Assert.Equal(a, c);
    }
}
