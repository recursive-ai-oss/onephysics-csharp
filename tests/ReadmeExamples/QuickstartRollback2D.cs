using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, EventKind
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, Body, HingeJoint

namespace ReadmeExamples;

/// <summary>The joint and rollback example of README.md; the lines between the usings and the closing brace are the README's, verbatim.</summary>
public static class QuickstartRollback2D
{
    public static (int Sounds, ulong Hash) Run()
    {
        var world = new PhysicsWorld2D();
        var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -1), type: BodyType.Static);
        var ball = world.AddCircle(0.5f, at: new Vector2D(0, 3));
        HingeJoint hinge = world.AddHinge(ground, ball, new Vector2D(0, 3));   // a typed view: only the members a hinge has
        hinge.EnableMotor = true;
        hinge.MaxMotorTorque = 5f;
        int sounds = 0;
        ball.OnCollisionEnter += c => sounds++;

        byte[] before = world.Snapshot();                // one per frame in a real host
        for (int i = 0; i < 120; i++) world.Step(1f / 60);
        world.Restore(before);                           // a late input arrived: roll back and resimulate
        using (world.SuppressCallbacks())                // no sounds or particles twice; polled events still work
        {
            for (int i = 0; i < 120; i++) world.Step(1f / 60);
        }
        return (sounds, world.ComputeHash());
    }
}
