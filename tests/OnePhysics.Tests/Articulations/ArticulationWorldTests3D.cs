using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Tests.Articulations;

public sealed class ArticulationWorldTests3D
{
    private readonly ITestOutputHelper _out;
    public ArticulationWorldTests3D(ITestOutputHelper output) { _out = output; }

    // Writes a line to the file named by ARTI_TRACE (when set), so a passing run can be looked at.
    private static void Trace(string line)
    {
        string? path = Environment.GetEnvironmentVariable("ARTI_TRACE");
        if (path != null) { System.IO.File.AppendAllText(path, line + "\n"); }
    }

    [Fact]
    public void QuadrupedStandsOnAFloor()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var q = world.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(0, 0.57, 0));
        var torso = q.GetLink(0);
        double minFoot = 1e9;
        for (int i = 0; i < 300; i++)
        {
            world.Step(1f / 60f);
            if (i % 30 == 0) { Trace(i + ": torso " + torso.Position + " contacts " + q.ContactPointCount); }
            for (int l = 1; l < q.LinkCount; l++)
            {
                var b = q.GetLink(l);
                minFoot = Math.Min(minFoot, b.Position.Y);
            }
        }

        Trace("min link y " + minFoot);
        Assert.True(torso.Position.Y > 0.45, "torso at " + torso.Position);
    }
}
