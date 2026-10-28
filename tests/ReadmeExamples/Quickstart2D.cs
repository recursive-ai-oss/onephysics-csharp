using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, events, exceptions
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, Body, Character

namespace ReadmeExamples;

/// <summary>The 2D quick start of README.md; the lines between the usings and the closing brace are the README's, verbatim.</summary>
public static class Quickstart2D
{
    public static (int Bytes, ulong Hash) Run()
    {
        var world = new PhysicsWorld2D();
        world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -1), type: BodyType.Static);
        var ball = world.AddCircle(0.5f, at: new Vector2D(0, 5));
        ball.OnCollisionEnter += c => Console.WriteLine($"hit {c.Other} along {c.Normal}");

        var player = world.AddCharacter(at: new Vector2D(3, 1));
        for (int i = 0; i < 600; i++)
        {
            player.Move(new Vector2F(0.05f, -0.1f), 1f / 60);
            world.Step(1f / 60);                        // fixed dt; events fire after the step, in a deterministic order
        }
        if (world.Raycast(new Vector2D(0, 10), new Vector2F(0, -1), out var hit)) Console.WriteLine(hit.Point);

        byte[] saved = world.Snapshot();                // byte-exact; Restore(saved) continues bit for bit
        ulong hash = world.ComputeHash();               // the same on every platform
        return (saved.Length, hash);
    }
}
