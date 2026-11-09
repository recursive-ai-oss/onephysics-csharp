using System;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

// The fabric scenes: stable on their own thresholds (1.5 times the worst of the x64 and arm64 values), no triangle crossings, and the same on any thread count and after a restore.
public class ClothScenes3DTests
{
    [Theory]
    [InlineData("cloth-flag-3d", 20.0, 0.0, 0.2)]
    [InlineData("cloth-character-3d", 7.0, 0.02, 0.09)]
    [InlineData("cloth-throw-3d", 40.0, 0.0, 0.11)]
    [InlineData("cloth-garment-3d", 3.5, 0.02, 0.06)]
    public void TheSceneIsStableAndDoesNotCrossItself(string name, double maxSpeed, double maxPenetration, double maxStrain)
    {
        var m = Cloth3DScene.Measure(name);
        Assert.Equal(0, m.NaN);
        Assert.Equal(0, m.Escapes);
        Assert.True(m.MaxSpeed <= maxSpeed, Cloth3DScene.Format(m));
        Assert.True(m.MaxPenetration <= maxPenetration, Cloth3DScene.Format(m));
        Assert.True(m.MaxStrain <= maxStrain, Cloth3DScene.Format(m));
        Assert.Equal(0, m.MaxIntersections);
        if (name == "cloth-garment-3d") { Assert.True(m.MaxSeamGap <= 0.09, Cloth3DScene.Format(m)); }
    }

    [Theory]
    [InlineData("cloth-flag-3d")]
    [InlineData("cloth-character-3d")]
    [InlineData("cloth-throw-3d")]
    [InlineData("cloth-garment-3d")]
    public void TheSceneIsTheSameOnAnyThreadCountAndAfterARestore(string name)
    {
        ulong a = Cloth3DScene.Run(name, 1, false, null, 120);
        Assert.Equal(a, Cloth3DScene.Run(name, 8, false, null, 120));
        Assert.Equal(a, Cloth3DScene.Run(name, 1, true, null, 120));
        Assert.Equal(a, Cloth3DScene.Run(name, 8, true, null, 120));
    }

    [Fact]
    public void StripSceneIsStableAndTheSameOnAnyThreadCountAndAfterARestore()
    {
        var m = Strip2DScene.Measure();
        Assert.Equal(0, m.NaN);
        Assert.Equal(0, m.Escapes);
        Assert.True(m.MaxSpeed <= 16.0, Strip2DScene.Format(m));
        Assert.True(m.MaxStrain <= 0.11, Strip2DScene.Format(m));
        ulong a = Strip2DScene.Run(1, false, null, 120);
        Assert.Equal(a, Strip2DScene.Run(8, false, null, 120));
        Assert.Equal(a, Strip2DScene.Run(1, true, null, 120));
    }
}
