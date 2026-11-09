using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class PulleyScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        // The four shared fixtures: tackle-2d, belt-2d, chain-drive-3d and crane-3d, 240 steps each.
        foreach (var name in PulleyScene.Names)
        {
            string n = name;
            Scenarios.Register("pulleys/" + n, ctx =>
            {
                var scene = PulleyScene.Create(n, ctx.Threads);
                for (int i = 0; i < 240; i++) { scene.Step(i); ctx.Write((long)scene.Hash()); ctx.EndStep(); }
            });
        }
        // Snapshot, fork, restore and origin shift with a rope over a friction pulley, a winch and a chain in one world.
        Scenarios.Register("world2d/pulleys", ctx =>
        {
            var w = new P2.PhysicsWorld2D(new() { WorldId = 885, ThreadCount = ctx.Threads, SleepEnabled = false });
            var a = w.AddBox(new(.4f, .4f), at: new(-.25, 3), density: 40); var b = w.AddBox(new(.4f, .4f), at: new(.25, 3), density: 20);
            w.AddRopeOverPulleys(P2.RopeEnd.On(a, default), [new P2.Pulley(new(0, 5), .25f, clockwise: true, friction: .3f, inertia: float.PositiveInfinity)], P2.RopeEnd.On(b, default));
            var load = w.AddBox(new(.4f, .4f), at: new(4, 1), density: 50);
            var winch = w.AddWinch(default, new(4.2, 6), .2f, P2.RopeEnd.On(load, default), drumInertia: .5f, maxTorque: 200);
            winch.SetMotor(0, -3, 200);
            for (int i = 0; i < 160; i++)
            {
                if (i == 40) { var snapshot = w.Snapshot(); w = w.Fork(); w.Restore(snapshot); ctx.Write(w.Snapshot()); }
                if (i == 80) w.ShiftOrigin(new(16, -8));
                w.Step(1f / 60); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/pulleys", ctx =>
        {
            var w = new P3.PhysicsWorld3D(new() { WorldId = 886, ThreadCount = ctx.Threads, SleepEnabled = false });
            var a = w.AddBox(new(.4f, .4f, .4f), at: new(-.25, 3, 0), density: 600); var b = w.AddBox(new(.4f, .4f, .4f), at: new(.25, 3, 0), density: 300);
            w.AddRopeOverPulleys(P3.RopeEnd.On(a, default), [new P3.Pulley(new(0, 5, 0), .25f, clockwise: true, friction: .3f, inertia: float.PositiveInfinity)], P3.RopeEnd.On(b, default));
            var load = w.AddBox(new(.4f, .4f, .4f), at: new(4, 1, 1), density: 800);
            var winch = w.AddWinch(default, new(4.2, 6, 1), .2f, P3.RopeEnd.On(load, default), drumInertia: .5f, maxTorque: 300);
            winch.SetMotor(0, -3, 300);
            for (int i = 0; i < 160; i++)
            {
                if (i == 40) { var snapshot = w.Snapshot(); w = w.Fork(); w.Restore(snapshot); ctx.Write(w.Snapshot()); }
                if (i == 80) w.ShiftOrigin(new(16, -8, 2));
                w.Step(1f / 60); ctx.Write((long)w.ComputeHash()); ctx.EndStep();
            }
        });
    }
}
