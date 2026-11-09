using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Destruction;
using RecursiveAI.OnePhysics.Tests.Common;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class DestructionDeterminismTests
{
    [Fact]
    public void ThreadCountAndSnapshotRoundTripsDoNotChangeTheResult3D()
    {
        ulong one = DestructionScenes.Run3D(1, 200, false);
        Assert.Equal(one, DestructionScenes.Run3D(8, 200, false));
        Assert.Equal(one, DestructionScenes.Run3D(3, 200, true));
    }

    [Fact]
    public void ThreadCountAndSnapshotRoundTripsDoNotChangeTheResult2D()
    {
        ulong one = DestructionScenes.Run2D(1, 200, false);
        Assert.Equal(one, DestructionScenes.Run2D(8, 200, false));
        Assert.Equal(one, DestructionScenes.Run2D(3, 200, true));
    }

    [Fact]
    public void TheSceneActuallyBreaksThingsAndTracksDebris()
    {
        var (w, target) = DestructionScenes.Spatial(1);
        for (int i = 0; i < 200; i++) { if (i == 30) w.Slice(target, new Vector3D(-8, 12.8, 2), new Vector3F(1, 0.3f, 0.2f)); w.Step(1f / 60); }
        var c = w.DestructionCounters;
        Assert.True(c.Fractures >= 3, "fractures " + c.Fractures);
        Assert.True(c.Slices == 1);
        Assert.True(c.BondBreaks > 0);
        Assert.True(c.FragmentsAlive <= 70);
        Assert.True(w.Counters.Fragments > 0);
    }

    [Fact]
    public void SceneTextRoundTripKeepsDestructionState()
    {
        var (w, _) = DestructionScenes.Spatial(1);
        for (int i = 0; i < 90; i++) w.Step(1f / 60);
        string text = w.SaveScene();
        var copy = P3.PhysicsWorld3D.LoadScene(new StringReader(text));
        Assert.Equal(w.ComputeHash(), copy.ComputeHash());
        for (int i = 0; i < 40; i++) { w.Step(1f / 60); copy.Step(1f / 60); Assert.Equal(w.ComputeHash(), copy.ComputeHash()); }
    }

    [Fact]
    public void DebugGeometryReportsBondsAndTheirStress()
    {
        var world = new P3.PhysicsWorld3D();
        var cells = new System.Collections.Generic.List<Vector3F[]>();
        for (int i = 0; i < 3; i++) cells.Add([new(i, 0, 0), new(i + 1, 0, 0), new(i, 1, 0), new(i + 1, 1, 0), new(i, 0, 1), new(i + 1, 0, 1), new(i, 1, 1), new(i + 1, 1, 1)]);
        world.AddBreakable(P3.FractureAsset.FromCells(cells), new Vector3D(0, 5, 0), def: new BreakableDef { Strength = 1e9f }, type: BodyType.Static, density: 1000f);
        world.Step(1f / 60);
        var g = new DebugGeometry();
        world.GetDebugGeometry(DebugContent.Destruction, g);
        Assert.Equal(2, g.Bonds.Length);
        Assert.True(g.Bonds[0].Intact);
        Assert.Equal(1.0f, g.Bonds[0].Area, 4);
        Assert.Equal(1e9f, g.Bonds[0].Strength);
        Assert.Equal(1.0, Math.Abs(g.Bonds[0].Normal.X), 6);
        world.GetDebugGeometry(DebugContent.Shapes, g);
        Assert.Empty(g.Bonds.ToArray());
    }

    [Fact]
    public void StepsAndStressAllocateNothingOnceWarm()
    {
        var world = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        world.AddBox(new(40, 1, 40), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var cells = new System.Collections.Generic.List<Vector3F[]>();
        for (int i = 0; i < 6; i++) cells.Add([new(i, 0, 0), new(i + 1, 0, 0), new(i, 0.5f, 0), new(i + 1, 0.5f, 0), new(i, 0, 0.5f), new(i + 1, 0, 0.5f), new(i, 0.5f, 0.5f), new(i + 1, 0.5f, 0.5f)]);
        world.AddBreakable(P3.FractureAsset.FromCells(cells), new Vector3D(-3, 0.2, 0), def: new BreakableDef { Strength = 1e12f }, density: 500f);
        world.AddBox(new(1, 1, 1), new Vector3D(5, 0.5, 0)).SetFracturable(new P3.FractureDef { CellCount = 6, ImpulseThreshold = 1e6f });
        world.Warmup();
        for (int i = 0; i < 60; i++) world.Step(1f / 60);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 100; i++) world.Step(1f / 60);
        long used = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.True(used == 0, "allocated " + used + " bytes over 100 steps");
    }
}
