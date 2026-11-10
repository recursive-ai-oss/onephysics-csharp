using System;
using RecursiveAI.OneMath;               // Vector3F, Vector3D
using RecursiveAI.OnePhysics;            // BodyType, events, exceptions
using RecursiveAI.OnePhysics.Physics3D;  // PhysicsWorld3D, Body, Character

namespace ReadmeExamples;

/// <summary>The 3D quick start of README.md; the lines between the usings and the closing brace are the README's, verbatim.</summary>
public static class Quickstart3D
{
    public static (int Bytes, ulong Hash) Run()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -1, 0), type: BodyType.Static);
        var ball = world.AddSphere(0.5f, at: new Vector3D(0, 5, 0));
        ball.OnCollisionEnter += c => Console.WriteLine($"hit {c.Other} along {c.Normal}");

        for (int i = 0; i < 300; i++) world.Step(1f / 60);
        if (world.Raycast(new Vector3D(0, 10, 0), new Vector3F(0, -1, 0), out var hit)) Console.WriteLine(hit.Point);

        byte[] saved = world.Snapshot();
        ulong hash = world.ComputeHash();
        return (saved.Length, hash);
    }
}
