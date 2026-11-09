using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class ExtendedQueryScenarios
{
    [ModuleInitializer]
    internal static void Register()
    {
        Scenarios.Register("world2d/query-batches", Run2D);
        Scenarios.Register("world3d/query-batches", Run3D);
    }

    private static void Run2D(ScenarioContext context)
    {
        var world = new P2.PhysicsWorld2D(new P2.WorldOptions2D { ThreadCount = context.Threads, WorldId = 61, Gravity = default });
        for (int i = 0; i < 16; i++) { world.AddCircle(0.5f, new Vector2D(4 + i * 2, 0), BodyType.Static); }
        var shape = Shape2D.CreateCircle(default, 0.5f);
        var inputs = new P2.QueryInput2D[513];
        var hits = new P2.QueryHit2D[inputs.Length * 4];
        var counts = new int[inputs.Length];
        var any = new bool[inputs.Length];
        for (int step = 0; step < 8; step++)
        {
            for (int i = 0; i < inputs.Length; i++)
            {
                var at = new Vector2D(i % 7, step * 0.1);
                inputs[i] = (i % 5) switch
                {
                    0 => P2.QueryInput2D.Ray(at, new Vector2F(1, 0), 40f),
                    1 => P2.QueryInput2D.Cast(shape, at, 0f, new Vector2F(1, 0), 40f),
                    2 => P2.QueryInput2D.Overlap(shape, at, 0f),
                    3 => P2.QueryInput2D.Penetration(shape, at, 0f),
                    _ => P2.QueryInput2D.Closest(at, 40f)
                };
            }
            world.QueryAllBatch(inputs, hits, counts, 4);
            world.QueryAnyBatch(inputs, any);
            for (int i = 0; i < inputs.Length; i++)
            {
                context.Write(counts[i]); context.Write(any[i] ? 1 : 0);
                for (int j = 0; j < counts[i]; j++)
                {
                    var hit = hits[i * 4 + j];
                    context.Write(hit.Shape.RawHandle.Index);
                    context.Write(hit.Distance); context.Write(hit.Fraction);
                    context.Write(hit.Point.X); context.Write(hit.Point.Y);
                    context.Write(hit.Normal.X); context.Write(hit.Normal.Y);
                }
            }
            context.EndStep();
        }
    }

    private static void Run3D(ScenarioContext context)
    {
        var world = new P3.PhysicsWorld3D(new P3.WorldOptions3D { ThreadCount = context.Threads, WorldId = 61, Gravity = default });
        for (int i = 0; i < 16; i++) { world.AddSphere(0.5f, new Vector3D(4 + i * 2, 0, 0), BodyType.Static); }
        var shape = ConvexShape3D.Sphere(0.5f);
        var inputs = new P3.QueryInput3D[513];
        var hits = new P3.QueryHit3D[inputs.Length * 4];
        var counts = new int[inputs.Length];
        var any = new bool[inputs.Length];
        for (int step = 0; step < 8; step++)
        {
            for (int i = 0; i < inputs.Length; i++)
            {
                var at = new Vector3D(i % 7, step * 0.1, 0);
                inputs[i] = (i % 5) switch
                {
                    0 => P3.QueryInput3D.Ray(at, new Vector3F(1, 0, 0), 40f),
                    1 => P3.QueryInput3D.Cast(shape, at, default, new Vector3F(1, 0, 0), 40f),
                    2 => P3.QueryInput3D.Overlap(shape, at, default),
                    3 => P3.QueryInput3D.Penetration(shape, at, default),
                    _ => P3.QueryInput3D.Closest(at, 40f)
                };
            }
            world.QueryAllBatch(inputs, hits, counts, 4);
            world.QueryAnyBatch(inputs, any);
            for (int i = 0; i < inputs.Length; i++)
            {
                context.Write(counts[i]); context.Write(any[i] ? 1 : 0);
                for (int j = 0; j < counts[i]; j++)
                {
                    var hit = hits[i * 4 + j];
                    context.Write(hit.Shape.RawHandle.Index);
                    context.Write(hit.Distance); context.Write(hit.Fraction);
                    context.Write(hit.Point.X); context.Write(hit.Point.Y); context.Write(hit.Point.Z);
                    context.Write(hit.Normal.X); context.Write(hit.Normal.Y); context.Write(hit.Normal.Z);
                }
            }
            context.EndStep();
        }
    }
}
