using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;


namespace RecursiveAI.OnePhysics.Tests.Articulations;

public sealed class ArticulationSelfCollisionTests
{
    private const double Radius = 0.05, Length = 0.5;

    private static double SegmentDistance(Vector3D p1, Vector3D q1, Vector3D p2, Vector3D q2)
    {
        // Closest distance between two segments by sampling then refining (test code: simple and obviously right).
        double best = double.MaxValue;
        const int n = 200;
        for (int i = 0; i <= n; i++)
        {
            var a = p1 + (q1 - p1) * (i / (double)n);
            double s = 0.0;
            var d = q2 - p2;
            double len2 = Vector3D.Dot(d, d);
            if (len2 > 0) { s = Math.Clamp(Vector3D.Dot(a - p2, d) / len2, 0.0, 1.0); }
            best = Math.Min(best, (a - (p2 + d * s)).Length());
        }

        return best;
    }

    private static (Vector3D A, Vector3D B) Segment(Articulation chain, int link)
    {
        var body = chain.GetLink(link);
        var r = body.Rotation.Rotate(new Vector3F((float)Length, 0f, 0f));
        return (body.Position, body.Position + new Vector3D(r.X, r.Y, r.Z));
    }

    private static double DeepestSelfPenetration(Articulation chain)
    {
        double deepest = 0.0;
        for (int i = 1; i < chain.LinkCount; i++)
        {
            for (int j = i + 2; j < chain.LinkCount; j++)
            {
                var (a0, a1) = Segment(chain, i); var (b0, b1) = Segment(chain, j);
                deepest = Math.Max(deepest, 2 * Radius - SegmentDistance(a0, a1, b0, b1));
            }
        }

        return deepest;
    }

    // Every joint is driven towards 1.4 rad: six of them add up to more than a full turn, so the chain winds up and must meet itself.
    private static void Fold(ArticulationDef def)
    {
        for (int i = 1; i < def.Links.Count; i++) { def.Links[i].Drive = new ArticulationDrive { Stiffness = 60f, Damping = 6f, TargetPosition = 1.4f }; }
    }

    private static void Fold(P2.ArticulationDef def)
    {
        for (int i = 1; i < def.Links.Count; i++) { def.Links[i].Drive = new ArticulationDrive { Stiffness = 60f, Damping = 6f, TargetPosition = 1.4f }; }
    }

    private static double Curl(bool selfCollision, int steps)
    {
        var world = new PhysicsWorld3D();
        var def = ArticulationDef.Chain(6, (float)Length, 1f, 1f);
        def.SelfCollision = selfCollision; def.GravityScale = 0f; Fold(def);
        var chain = world.AddArticulation(def, new Vector3D(0, 2, 0));
        double worst = 0.0;
        for (int i = 0; i < steps; i++)
        {
            world.Step(1f / 60f);
            worst = Math.Max(worst, DeepestSelfPenetration(chain));
        }

        return worst;
    }

    [Fact]
    public void ChainCurlingOntoItselfStaysSeparated()
    {
        double off = Curl(false, 180);
        double on = Curl(true, 180);
        Assert.True(off > 0.02, "without the flag the links should pass through each other, penetration " + off);
        Assert.True(on <= 0.005, "link penetration " + on);
    }

    [Fact]
    public void SelfContactsAreInTheSnapshotAndReplayTheSameHashes()
    {
        PhysicsWorld3D Make()
        {
            var w = new PhysicsWorld3D();
            var def = ArticulationDef.Chain(6, (float)Length, 1f, 1f);
            def.SelfCollision = true; def.GravityScale = 0f; Fold(def);
            var c = w.AddArticulation(def, new Vector3D(0, 2, 0));
            return w;
        }

        var a = Make();
        for (int i = 0; i < 40; i++) { a.Step(1f / 60f); }
        var bytes = a.Snapshot();
        var b = Make();
        b.Restore(bytes);
        for (int i = 0; i < 80; i++)
        {
            a.Step(1f / 60f); b.Step(1f / 60f);
            Assert.Equal(a.ComputeHash(), b.ComputeHash());
        }
    }

    [Fact]
    public void PlanarChainCurlingOntoItselfStaysSeparated()
    {
        double Run(bool self)
        {
            var world = new P2.PhysicsWorld2D();
            var def = P2.ArticulationDef.Chain(6, (float)Length, 1f, 1f);
            def.SelfCollision = self; def.GravityScale = 0f; Fold(def);
            var chain = world.AddArticulation(def, new Vector2D(0, 2));
            double worst = 0.0;
            for (int i = 0; i < 180; i++)
            {
                world.Step(1f / 60f);
                for (int a = 1; a < chain.LinkCount; a++)
                {
                    for (int b = a + 2; b < chain.LinkCount; b++)
                    {
                        var (a0, a1) = Segment2(chain, a); var (b0, b1) = Segment2(chain, b);
                        worst = Math.Max(worst, 2 * Radius - SegmentDistance(a0, a1, b0, b1));
                    }
                }
            }

            return worst;
        }

        Assert.True(Run(false) > 0.02);
        double on = Run(true);
        Assert.True(on <= 0.005, "link penetration " + on);
    }

    private static (Vector3D A, Vector3D B) Segment2(P2.Articulation chain, int link)
    {
        var body = chain.GetLink(link);
        double c = Math.Cos(body.Angle), s = Math.Sin(body.Angle);
        var p = new Vector3D(body.Position.X, body.Position.Y, 0);
        return (p, p + new Vector3D(Length * c, Length * s, 0));
    }
}
