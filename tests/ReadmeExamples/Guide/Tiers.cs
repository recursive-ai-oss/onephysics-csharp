using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType, EventKind
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, BodyDef, ShapeDef, PhysicsQuery2D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/tiers.md.</summary>
public static class Tiers
{
    // A filter written as a struct, so the engine can specialize on it: no delegate call per pair.
    struct NotTheSameBody : IContactFilter
    {
        public bool ShouldCollide(Shape a, Shape b) => a.Body != b.Body;
    }

    public static (int Contacts, int Hits, ulong Hash) Run()
    {
        var world = new PhysicsWorld2D(new WorldOptions2D
        {
            Gravity = new Vector2F(0, -9.81f),
            Substeps = 4,                 // substeps per Step call
            VelocityIterations = 1,       // solve passes with bias per substep
            RelaxIterations = 1,          // relax passes without bias per substep
        });
        var floor = world.CreateBody(new BodyDef { Type = BodyType.Static, Position = new Vector2D(0, -0.5) });
        world.CreateBox(floor, new Vector2F(20, 1), def: new ShapeDef { Friction = 0.9f, Layer = 1 });
        var box = world.CreateBody(new BodyDef { Position = new Vector2D(0, 2), Angle = 0.3f });
        world.CreateBox(box, new Vector2F(1, 1), def: new ShapeDef { Density = 2f, Restitution = 0.1f, Layer = 2 });
        world.IgnoreLayerCollision(2, 3);

        world.SetContactFilter(new NotTheSameBody());

        int contacts = 0;
        for (int i = 0; i < 60; i++)
        {
            world.Step(1f / 60);
            foreach (var e in world.GetEvents(EventKind.ContactBegin))
            {
                contacts++;
                Console.WriteLine($"{world.GetBody(e.BodyA)} touched {world.GetBody(e.BodyB)}");
            }
        }

        var query = new PhysicsQuery2D(world) { Filter = QueryFilter2D.OnLayer(1) };
        var hits = new QueryHit2D[16];
        int n = query.RaycastAll(new Vector2D(-5, -0.25), new Vector2F(1, 0), hits);
        return (contacts, n, world.ComputeHash());
    }
}
