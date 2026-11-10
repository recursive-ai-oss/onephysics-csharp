using System;
using RecursiveAI.OnePhysics;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Water;

public class SwimSceneTests
{
    [Fact]
    public void EverySwimmerOfTheThreeDimensionalSceneReachesItsGoal()
    {
        var s = SwimScenes.Build3D(1);
        int step = 0;
        for (; step < s.Steps && !SwimScenes.AllDone(s); step++) { SwimScenes.Drive3D(s, step); s.World.Step(SwimScenes.Dt); }
        Assert.True(SwimScenes.AllDone(s), "not every character arrived in " + step + " steps");
        Assert.True(s.ParticleCount > 1000);
        // Wading, swimming and diving all happened for the lake swimmers.
        for (int i = 0; i < 12; i++) { Assert.True(s.Swimmers[i].SwimSteps > 500 && s.Swimmers[i].DiveSteps > 100 && s.Swimmers[i].MinY < 0.8, "swimmer " + i); }
        // The pond's rope floats and the paddle made the pond's water move.
        double sum = 0;
        for (int i = 0; i < s.Rope.ParticleCount; i++) { sum += s.Rope.GetPosition(i).Y; }
        Assert.InRange(sum / s.Rope.ParticleCount, 1.85, 2.2);
    }

    [Fact]
    public void EverySwimmerOfTheSideOnSceneReachesItsGoal()
    {
        var s = SwimScenes.Build2D(1);
        int step = 0;
        for (; step < s.Steps && !SwimScenes.AllDone(s); step++) { SwimScenes.Drive2D(s, step); s.World.Step(SwimScenes.Dt); }
        Assert.True(SwimScenes.AllDone(s), "not every character arrived in " + step + " steps");
        Assert.True(s.ParticleCount > 300);
        for (int i = 0; i < 8; i++) { Assert.True(s.Swimmers[i].SwimSteps > 500 && s.Swimmers[i].DiveSteps > 100 && s.Swimmers[i].MinY < 0.8, "swimmer " + i); }
    }

    [Fact]
    public void TheSwimScenesAgreeAcrossThreadCountsAndARestore()
    {
        Assert.Equal(SwimScenes.Run3D(1, false), SwimScenes.Run3D(3, false));
        Assert.Equal(SwimScenes.Run3D(1, false), SwimScenes.Run3D(1, true));
        Assert.Equal(SwimScenes.Run2D(1, false), SwimScenes.Run2D(3, false));
        Assert.Equal(SwimScenes.Run2D(1, false), SwimScenes.Run2D(1, true));
    }
}
