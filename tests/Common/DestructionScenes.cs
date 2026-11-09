using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Common;

/// <summary>Shared destruction scenes for the hash matrix, the smoke apps and the tests: crates that shatter on impact, a bonded wall hit by a projectile, a loaded static bridge, a slice and the debris rules.</summary>
internal static class DestructionScenes
{
    private const float Dt = 1f / 60f;

    internal static (P3.PhysicsWorld3D World, P3.Body SliceTarget) Spatial(int threads)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 71, ThreadCount = threads });
        w.AddBox(new Vector3F(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        for (int i = 0; i < 5; i++)
        {
            var c = w.AddBox(new Vector3F(0.8f, 0.8f, 0.8f), new Vector3D(-4 + i * 1.7, 2.5 + 0.7 * i, -3), new QuaternionF(0.1f * i, 0.2f, 0f, 1f).Normalized());
            c.SetFracturable(new P3.FractureDef { CellCount = 10, ImpulseThreshold = 3f, Seed = (ulong)(21 + i), Depth = 1, Pattern = i % 2 == 0 ? FracturePattern.Uniform : FracturePattern.Clustered });
        }
        var wall = P3.FractureAsset.FromHull([new(-2, 0, -0.3f), new(2, 0, -0.3f), new(-2, 3, -0.3f), new(2, 3, -0.3f), new(-2, 0, 0.3f), new(2, 0, 0.3f), new(-2, 3, 0.3f), new(2, 3, 0.3f)], new P3.FractureDef { CellCount = 14, Seed = 9 });
        w.AddBreakable(wall, new Vector3D(0, 0, 4), def: new BreakableDef { Strength = 2e5f }, density: 1500f);
        var ball = w.AddSphere(0.3f, new Vector3D(0, 1.5, -2), density: 3000f);
        ball.LinearVelocity = new Vector3F(0, 2, 22);
        var cells = new System.Collections.Generic.List<Vector3F[]>();
        for (int i = 0; i < 6; i++)
        {
            var p = new Vector3F[8];
            for (int k = 0; k < 8; k++) p[k] = new Vector3F(i + ((k & 1) != 0 ? 1 : 0), (k & 2) != 0 ? 0.25f : -0.25f, (k & 4) != 0 ? 0.5f : -0.5f);
            cells.Add(p);
        }
        var bridge = w.AddBreakable(P3.FractureAsset.FromCells(cells), new Vector3D(8, 3, 0), def: new BreakableDef { Strength = 6e5f, AnchorParts = [0, 5] }, type: BodyType.Static, density: 1500f);
        _ = bridge;
        w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), new Vector3D(11, 3.55, 0), density: 4000f);
        w.Debris = new DebrisOptions { MaxFragments = 70, SleepToStaticSteps = 60 };
        var target = w.AddBox(new Vector3F(1.2f, 0.6f, 0.9f), new Vector3D(-8, 14, 2), new QuaternionF(0.2f, 0.1f, 0.3f, 1f).Normalized());
        target.AngularVelocity = new Vector3F(1, 0.5f, 0);
        return (w, target);
    }

    internal static (P2.PhysicsWorld2D World, P2.Body SliceTarget) Planar(int threads)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 72, ThreadCount = threads });
        w.AddBox(new Vector2F(60, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 5; i++)
        {
            var c = w.AddBox(new Vector2F(0.8f, 0.8f), new Vector2D(-4 + i * 1.7, 2.5 + 0.7 * i), 0.1f * i);
            c.SetFracturable(new P2.FractureDef { CellCount = 8, ImpulseThreshold = 3f, Seed = (ulong)(31 + i), Depth = 1 });
        }
        var cells = new System.Collections.Generic.List<Vector2F[]>();
        for (int i = 0; i < 6; i++) cells.Add([new(i, -0.25f), new(i + 1, -0.25f), new(i + 1, 0.25f), new(i, 0.25f)]);
        w.AddBreakable(P2.FractureAsset.FromCells(cells), new Vector2D(8, 3), def: new BreakableDef { Strength = 3e5f, AnchorParts = [0, 5] }, type: BodyType.Static, density: 1500f);
        w.AddBox(new Vector2F(0.6f, 0.6f), new Vector2D(11, 3.55), density: 4000f);
        w.Debris = new DebrisOptions { MaxFragments = 60, SleepToStaticSteps = 60 };
        var target = w.AddBox(new Vector2F(1.2f, 0.6f), new Vector2D(-8, 14), 0.3f);
        target.AngularVelocity = 1f;
        return (w, target);
    }

    /// <summary>Runs a scene for <paramref name="steps"/> with a slice at step 30 and, when <paramref name="restore"/>, a snapshot round trip into a fork at step 50. Returns the final hash.</summary>
    internal static ulong Run3D(int threads, int steps, bool restore)
    {
        var (w, target) = Spatial(threads);
        for (int i = 0; i < steps; i++)
        {
            if (i == 30 && target.IsValid) w.Slice(target, new Vector3D(-8, 14 - 0.5 * 30.0 / 60 * 30.0 / 60 * 9.81, 2), new Vector3F(1, 0.3f, 0.2f));
            if (restore && i == 50) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); }
            w.Step(Dt);
        }
        return w.ComputeHash();
    }

    internal static ulong Run2D(int threads, int steps, bool restore)
    {
        var (w, target) = Planar(threads);
        for (int i = 0; i < steps; i++)
        {
            if (i == 30 && target.IsValid) w.Slice(target, new Vector2D(-8, 14 - 0.5 * 30.0 / 60 * 30.0 / 60 * 9.81), new Vector2F(1, 0.3f));
            if (restore && i == 50) { var s = w.Snapshot(); w = w.Fork(); w.Restore(s); }
            w.Step(Dt);
        }
        return w.ComputeHash();
    }
}
