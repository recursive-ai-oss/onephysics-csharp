using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Tests.Articulations;

/// <summary>Independent articulations simulate in parallel; the result must not depend on the thread count, also when two of them push the same body.</summary>
public sealed class ArticulationParallelTests
{
    private static PhysicsWorld3D Make(int threads)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 7000 + threads });
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 12; i++)
        {
            w.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(2.0 * (i % 4), 0.625, 2.0 * (i / 4)));
        }

        // A box between two arms: both push it.
        for (int i = 0; i < 2; i++)
        {
            var arm = w.AddArticulation(ArticulationDef.RobotArm(0.9f, 4, 6f), new Vector3D(10 + 1.0 * i, 0, 0));
            arm.SetJointTarget(2, 0.7f);
        }

        w.AddBox(new Vector3F(0.3f, 0.3f, 0.3f), at: new Vector3D(10.5, 0.15, 0.25), density: 300f);
        return w;
    }

    [Fact]
    public void HashesAreEqualAtOneThreeAndEightThreads()
    {
        var a = Make(1); var b = Make(3); var c = Make(8);
        for (int i = 0; i < 240; i++)
        {
            a.Step(1f / 60f); b.Step(1f / 60f); c.Step(1f / 60f);
            ulong h = a.ComputeHash();
            Assert.Equal(h, b.ComputeHash());
            Assert.Equal(h, c.ComputeHash());
        }
    }
}
