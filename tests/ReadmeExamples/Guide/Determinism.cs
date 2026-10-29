using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/determinism.md.</summary>
public static class Determinism
{
    public static (double HostX, double Origin) Run()
    {
        var world = new PhysicsWorld2D();
        var crate = world.AddBox(new Vector2F(1, 1), at: new Vector2D(2_000_000, 0));
        world.Step(1f / 60);
        world.ShiftOrigin(new Vector2D(2_000_000, 0));          // between steps, by the same amount on every peer
        double hostX = world.Origin.X + crate.Position.X;       // the position in host coordinates is unchanged
        return (hostX, world.Origin.X);
    }
}
