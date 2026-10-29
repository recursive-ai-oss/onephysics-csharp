using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, events, exceptions
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, Body, Character

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/quick-start.md; each block's lines appear here verbatim and in order.</summary>
public static class QuickStart
{
    public static (Vector2D Crate, bool Grounded, ulong Hash) Run()
    {
        var world = new PhysicsWorld2D();
        var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var crate = world.AddBox(new Vector2F(1, 1), at: new Vector2D(0, 3), material: new Material2D(friction: 0.8f));
        var ball = world.AddCircle(0.5f, at: new Vector2D(2, 4));
        ball.LinearVelocity = new Vector2F(-1, 0);
        crate.OnCollisionEnter += c => Console.WriteLine($"crate hit {c.Other}");
        for (int i = 0; i < 120; i++) world.Step(1f / 60);
        Console.WriteLine($"{crate.Position} {crate.LinearVelocity} awake={crate.IsAwake}");

        var player = world.AddCharacter(at: new Vector2D(-3, 1));
        player.Move(new Vector2F(0.05f, -0.1f), 1f / 60);   // this frame's displacement, not a velocity
        world.Step(1f / 60);
        bool grounded = player.IsGrounded;
        if (world.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out var hit)) Console.WriteLine($"{hit.Body} at {hit.Point}, {hit.Distance} away");
        return (crate.Position, grounded, world.ComputeHash());
    }
}
