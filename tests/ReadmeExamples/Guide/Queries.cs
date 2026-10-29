using System;
using RecursiveAI.OneMath;               // Vector2F, Vector2D
using RecursiveAI.OnePhysics;            // BodyType
using RecursiveAI.OnePhysics.Physics2D;  // PhysicsWorld2D, QueryFilter2D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/queries-and-layers.md.</summary>
public static class Queries
{
    public static (bool Collide, int Overlaps, bool Cast) Run()
    {
        const int Ground = 1, Player = 2, Enemy = 3;
        var world = new PhysicsWorld2D();
        var ground = world.AddBox(new Vector2F(20, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var player = world.AddCircle(0.5f, at: new Vector2D(-2, 1));
        var enemy = world.AddCircle(0.5f, at: new Vector2D(2, 1));
        ground.Layer = Ground;
        player.Layer = Player;
        enemy.Layer = Enemy;
        world.IgnoreLayerCollision(Player, Enemy);               // layers 2 and 3 pass through each other
        bool collide = world.GetLayerCollision(Player, Ground);

        var enemiesOnly = QueryFilter2D.OnLayer(Enemy);
        var notThePlayer = new QueryFilter2D { Predicate = s => s.Body != player };
        var around = world.OverlapCircleAll(new Vector2D(2, 1), 1.5f, enemiesOnly);
        bool cast = world.CircleCast(new Vector2D(-5, 1), 0.25f, new Vector2F(1, 0), out var hit, 100f, notThePlayer);
        if (cast) Console.WriteLine($"first thing in the way: {hit.Body} after {hit.Distance} m");
        return (collide, around.Length, cast);
    }
}
