#pragma warning disable OPHYS0002
using System;
using RecursiveAI.OneMath;               // Vector3F, Vector3D
using RecursiveAI.OnePhysics;            // BodyType, events, exceptions
using RecursiveAI.OnePhysics.Physics3D;  // PhysicsWorld3D, Body, Character

namespace GuideExamples;

/// <summary>The 3D block of docs/guide/quick-start.md.</summary>
public static class QuickStart3D
{
    public static ulong Run()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20, 1, 20), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var crate = world.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 3, 0));
        var ball = world.AddSphere(0.5f, at: new Vector3D(2, 4, 0));
        for (int i = 0; i < 120; i++) world.Step(1f / 60);
        Console.WriteLine($"{crate.Position} {ball.Position}");
        return world.ComputeHash();
    }
}
