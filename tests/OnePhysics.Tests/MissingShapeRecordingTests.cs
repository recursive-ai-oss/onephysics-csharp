using System.IO;
using RecursiveAI.OnePhysics.Replay;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class MissingShapeRecordingTests
{
    [Fact]
    public void ConvexRecipesSurviveSceneAndRecordedCreation()
    {
        var w = new P3.PhysicsWorld3D(new() { WorldId = 991 });
        using var stream = new MemoryStream();
        using (var recording = w.StartRecording(stream))
        {
            w.AddTriangle(new(-4,0,-4), new(0,0,4), new(4,0,-4));
            w.AddTaperedCylinder(0.5f, 0.2f, 0.4f, at: new(0,2,0));
            w.Step(1f/60);
        }
        var copy = P3.PhysicsWorld3D.LoadScene(new StringReader(w.SaveScene()));
        Assert.Equal(w.Snapshot(), copy.Snapshot());
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var replay = new P3.PhysicsWorld3D(new() { WorldId = 991 });
        Assert.True(player.ReadNextStep(replay));
        Assert.Equal(w.ComputeHash(), replay.ComputeHash());
        Assert.False(player.ReadNextStep(replay));
    }

    [Fact]
    public void TerrainSegmentsSurviveSceneAndRecordedCreation()
    {
        var w = new P2.PhysicsWorld2D(new() { WorldId = 992 });
        using var stream = new MemoryStream();
        using (var recording = w.StartRecording(stream))
        {
            w.AddHeightField([0,0,1,1,0], holes: [false,false,true,false]);
            w.AddCircle(0.3f, at: new(1,2));
            w.Step(1f/60);
        }
        var copy = P2.PhysicsWorld2D.LoadScene(new StringReader(w.SaveScene()));
        Assert.Equal(w.Snapshot(), copy.Snapshot());
        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var replay = new P2.PhysicsWorld2D(new() { WorldId = 992 });
        Assert.True(player.ReadNextStep(replay));
        Assert.Equal(w.ComputeHash(), replay.ComputeHash());
        Assert.False(player.ReadNextStep(replay));
    }
}
