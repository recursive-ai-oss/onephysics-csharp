using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, EventKind, EventMask
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, Body, ShapeDef

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/events.md.</summary>
public static class Events
{
    public static (int Enters, int Stays, int Hits, int Polled, int Triggers) Run()
    {
        var world = new PhysicsWorld2D(new WorldOptions2D
        {
            Events = EventMask.Default | EventMask.ContactPersist | EventMask.ContactHit,
        });
        var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var ball = world.AddCircle(0.5f, at: new Vector2D(0, 3));
        int enters = 0, stays = 0, hitCount = 0, triggers = 0;
        ball.OnCollisionEnter += c => enters++;
        ball.OnCollisionStay += c => stays++;
        ball.OnCollisionHit += c => { hitCount++; Console.WriteLine($"landed at {c.ApproachSpeed} m/s"); };

        var zone = world.CreateBody(new BodyDef { Type = BodyType.Static, Position = new Vector2D(0, 1.5) });
        world.CreateBox(zone, new Vector2F(4, 1), def: new ShapeDef { IsSensor = true });
        zone.OnTriggerEnter += t => { triggers++; Console.WriteLine($"{t.Other} entered the zone"); };

        int polled = 0;
        for (int i = 0; i < 120; i++)
        {
            world.Step(1f / 60);
            polled += world.GetEvents(EventKind.ContactBegin).Length;
            foreach (var e in world.Events)
            {
                if (e.Kind == EventKind.Sleep) Console.WriteLine($"{world.GetBody(e.BodyA)} fell asleep");
            }
        }

        return (enters, stays, hitCount, polled, triggers);
    }
}
