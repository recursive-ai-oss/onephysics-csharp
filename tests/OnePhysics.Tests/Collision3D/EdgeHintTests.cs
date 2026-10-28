using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

public class EdgeHintTests
{
    private static RigidTransformF Pose(Vector3F t, Vector3F axis, float angle)
    {
        float h = 0.5f * angle;
        float s = MathF.Sin(h);
        return new RigidTransformF(new QuaternionF(axis.X * s, axis.Y * s, axis.Z * s, MathF.Cos(h)), t);
    }

    private static bool Same(in Manifold3D a, in Manifold3D b)
    {
        if (a.PointCount != b.PointCount || a.Normal != b.Normal) { return false; }
        for (int i = 0; i < a.PointCount; i++)
        {
            if (a[i].Point != b[i].Point || a[i].Separation != b[i].Separation || a[i].Id != b[i].Id) { return false; }
        }

        return true;
    }

    // A box resting on a box, nudged a little each step: while the hint says edges cannot win, the manifold with the hint must equal the one from a full query.
    [Fact]
    public void ManifoldWithHintEqualsFullQueryWhileRestingBoxesDrift()
    {
        var a = ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f));
        var b = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var rng = new Random(11);
        var full = new ManifoldGenerator3D();
        var hinted = new ManifoldGenerator3D();
        int skipped = 0, steps = 0;
        for (int trial = 0; trial < 200; trial++)
        {
            var axis = new Vector3F(0f, 1f, 0f);
            var p0 = Pose(new Vector3F(rng.NextSingle() - 0.5f, 0.99f, rng.NextSingle() - 0.5f), axis, rng.NextSingle() * 3f);
            full.Collide(a, b, p0);
            hinted.HintIn = default;
            hinted.Collide(a, b, p0);
            var hint = hinted.HintOut;
            Assert.True(hint.Gap > 0f);
            for (int k = 0; k < 20; k++)
            {
                var t = p0.Translation + new Vector3F((rng.NextSingle() - 0.5f) * 0.004f, (rng.NextSingle() - 0.5f) * 0.004f, (rng.NextSingle() - 0.5f) * 0.004f);
                var p = Pose(t, new Vector3F(0f, 1f, 0f), 0f);
                var q = new RigidTransformF(p0.Rotation * Pose(default, new Vector3F(0.2f, 0.9f, 0.1f).Normalized(), (rng.NextSingle() - 0.5f) * 0.01f).Rotation, t);
                var expected = full.Collide(a, b, q);
                hinted.HintIn = hint;
                var actual = hinted.Collide(a, b, q);
                if (hinted.HintOut.Gap == hint.Gap && hinted.HintOut.Tx == hint.Tx) { skipped++; }
                steps++;
                Assert.True(Same(expected, actual), $"trial {trial} step {k}");
                _ = p;
            }
        }

        Assert.True(skipped > steps / 2, $"the hint was used in {skipped} of {steps} steps");
    }

    // A large move must bring the full query back: a hint made when the boxes lay face to face says nothing about a pose a quarter turn away.
    [Fact]
    public void LargeMoveForcesFullQuery()
    {
        var a = ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f));
        var b = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var g = new ManifoldGenerator3D();
        var p0 = Pose(new Vector3F(0f, 0.99f, 0f), new Vector3F(0f, 1f, 0f), 0.3f);
        g.Collide(a, b, p0);
        var hint = g.HintOut;
        Assert.True(hint.Gap > 0f);
        var far = Pose(new Vector3F(0.2f, 0.9f, 0f), new Vector3F(1f, 0f, 0f), 0.8f);
        var full = new ManifoldGenerator3D();
        var expected = full.Collide(a, b, far);
        g.HintIn = hint;
        var actual = g.Collide(a, b, far);
        Assert.True(Same(expected, actual));
        Assert.False(g.HintOut.Gap == hint.Gap && g.HintOut.Tx == hint.Tx);
    }

    [Fact]
    public void HintDoesNotLeakIntoTheNextCall()
    {
        var a = ConvexShape3D.Box(new Vector3F(1f, 0.5f, 1f));
        var b = ConvexShape3D.Box(new Vector3F(0.5f, 0.5f, 0.5f));
        var g = new ManifoldGenerator3D();
        var p0 = Pose(new Vector3F(0f, 0.99f, 0f), new Vector3F(0f, 1f, 0f), 0.3f);
        g.Collide(a, b, p0);
        g.HintIn = g.HintOut;
        g.Collide(a, b, p0);
        Assert.Equal(0f, g.HintIn.Gap);
    }
}
