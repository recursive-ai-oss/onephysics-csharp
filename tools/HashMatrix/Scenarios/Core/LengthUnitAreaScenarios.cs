using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Deformables and machines in a millimetre world: a jelly box, a rubber ball, a pinned cloth and a rope on a slab, and a car, a tank and a motorcycle driving on a plane. Each snapshots part way, restores into
/// a fresh world and carries on; the scenario fails if the two disagree at any step. The hash of every step goes into the manifest.
/// </summary>
internal static class LengthUnitAreaScenarios
{
    private const float Dt = 1f / 60f;
    private const double K = 1000.0;

    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world3d/length-unit-soft-millimetres", RunSoft);
        Scenarios.Register("world3d/length-unit-vehicles-millimetres", RunVehicles);
    }

    private static P3.PhysicsWorld3D New(int threads, int id) => new(new P3.WorldOptions3D { ThreadCount = threads, WorldId = id, LengthUnit = LengthUnit.Millimetres });

    private static void Run(ScenarioContext ctx, int id, int steps, int snapshotAt, Func<P3.PhysicsWorld3D> build)
    {
        var w = build();
        P3.PhysicsWorld3D? copy = null;
        for (int i = 0; i < steps; i++)
        {
            w.Step(Dt);
            ulong h = w.ComputeHash();
            ctx.Write((long)h);
            if (copy != null)
            {
                copy.Step(Dt);
                if (copy.ComputeHash() != h) { throw new InvalidOperationException("restored world diverged at step " + i); }
            }

            ctx.EndStep();
            if (i == snapshotAt)
            {
                copy = New(ctx.Threads, id);
                copy.Restore(w.Snapshot());
                if (copy.ComputeHash() != h) { throw new InvalidOperationException("restored world differs from the original"); }
            }
        }
    }

    private static void RunSoft(ScenarioContext ctx) => Run(ctx, 73, 120, 40, () =>
    {
        var w = New(ctx.Threads, 73);
        w.AddBox(new((float)(20 * K), (float)(1 * K), (float)(20 * K)), at: new(0, -0.5 * K, 0), type: BodyType.Static);
        w.AddSoftBox(new((float)(0.4 * K), (float)(0.4 * K), (float)(0.4 * K)), at: new(1.0 * K, 0.6 * K, 0), cells: 3);
        w.AddSoftBall((float)(0.25 * K), at: new(-1.0 * K, 0.6 * K, 0), segments: 3);
        w.AddCloth((float)(0.8 * K), (float)(0.8 * K), 8, at: new(0, 1.2 * K, 2 * K), pinCorners: true);
        w.AddRope(new(-1.5 * K, 2.0 * K, -2 * K), new(1.5 * K, 2.0 * K, -2 * K), 12, true, false);
        return w;
    });

    private static void RunVehicles(ScenarioContext ctx) => Run(ctx, 74, 150, 50, () =>
    {
        var w = New(ctx.Threads, 74);
        w.AddBox(new((float)(300 * K), (float)(1 * K), (float)(300 * K)), at: new(0, -0.5 * K, 0), type: BodyType.Static);
        var chassis = w.AddBox(new((float)(1.8 * K), (float)(0.6 * K), (float)(4.2 * K)), at: new(0, 0.95 * K, 0), density: (float)(1500.0 / (1.8 * 0.6 * 4.2) / (K * K * K)));
        var car = w.AddVehicle(chassis, P3.VehicleDef.Car.InUnit(LengthUnit.Millimetres));
        car.SetInput(throttle: 0.7f);
        var tank = w.AddTank(new(15 * K, 0, 0));
        tank.SetInput(throttle: 0.6f);
        var bike = w.AddMotorcycle(new(30 * K, 0, 0));
        bike.SetInput(throttle: 0.5f);
        return w;
    });
}
