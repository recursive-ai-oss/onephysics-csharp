using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>A sensor never changes the result of a solid body: adding, moving or removing one leaves every dynamic body bit-identical.</summary>
public class SensorIndependenceTests
{
    private const float Dt = 1f / 60f;

    [Flags]
    public enum Kinds { None = 0, Static = 1, Kinematic = 2, Dynamic = 4, Late = 8, All = 15 }

    private sealed class Run
    {
        public readonly List<ulong> Fingerprints = new();
        public int Enters, Exits;
    }

    private static ulong Mix(ulong h, ulong v) => (h ^ v) * 0x100000001B3UL;

    private static Run Simulate(Kinds kinds, int threads, int steps = 900)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 5 });
        var ground = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(ground, new Vector2F(24f, 1f));
        var wall = w.CreateBody(BodyType.Static, new Vector2D(-12.5, 6));
        w.CreateBox(wall, new Vector2F(1f, 13f));
        wall = w.CreateBody(BodyType.Static, new Vector2D(12.5, 6));
        w.CreateBox(wall, new Vector2F(1f, 13f));

        // Solid bodies first, so that their slots are the same in every variant. A pile that keeps arriving: pairs are created, ended and their slots reused for the whole run.
        var solids = new List<Body>();
        uint seed = 12345;
        uint Next() { seed = seed * 1664525u + 1013904223u; return seed >> 8; }
        for (int i = 0; i < 120; i++)
        {
            double x = -10 + (Next() % 2000) / 100.0, y = 1 + i * 0.45;
            var b = w.CreateBody(BodyType.Dynamic, new Vector2D(x, y), (Next() % 628) / 100f);
            if (i % 3 == 0) { w.CreateCircle(b, 0.3f + (Next() % 30) / 100f); }
            else { w.CreateBox(b, new Vector2F(2f * (0.25f + (Next() % 40) / 100f), 2f * (0.25f + (Next() % 40) / 100f)), default, 0f, 0.05f); }
            solids.Add(b);
        }

        var kinematic = w.CreateBody(BodyType.Kinematic, new Vector2D(-11, 2));
        w.CreateBox(kinematic, new Vector2F(2f, 0.6f));
        kinematic.LinearVelocity = new Vector2F(1.2f, 0f);
        solids.Add(kinematic);

        var run = new Run();
        void Count(Body b) { b.OnTriggerEnter += _ => run.Enters++; b.OnTriggerExit += _ => run.Exits++; }
        Body late = default;
        Body dynamicSensor = default, kinematicSensor = default;
        if ((kinds & Kinds.Static) != 0)
        {
            var zone = w.CreateBody(BodyType.Static, new Vector2D(0, 2));
            w.CreateBox(zone, new Vector2F(8f, 4f), default, 0f, 0f, new ShapeDef { IsSensor = true });
            Count(zone);
            var zone2 = w.CreateBody(BodyType.Static, new Vector2D(-8, 5));
            w.CreateCircle(zone2, 2f, default, new ShapeDef { IsSensor = true });
        }

        if ((kinds & Kinds.Kinematic) != 0)
        {
            kinematicSensor = w.CreateBody(BodyType.Kinematic, new Vector2D(-11, 1));
            w.CreateCircle(kinematicSensor, 1.5f, default, new ShapeDef { IsSensor = true });
            kinematicSensor.LinearVelocity = new Vector2F(1.5f, 0.2f);
            Count(kinematicSensor);
        }

        if ((kinds & Kinds.Dynamic) != 0)
        {
            // A dynamic body that collides with nothing but whose sensor sees everything.
            dynamicSensor = w.CreateBody(BodyType.Dynamic, new Vector2D(6, 8));
            w.CreateCircle(dynamicSensor, 0.1f, default, new ShapeDef { CategoryBits = 2, MaskBits = 0 });
            w.CreateCircle(dynamicSensor, 2.5f, default, new ShapeDef { IsSensor = true, CategoryBits = 2, Density = 0f });
            Count(dynamicSensor);
        }

        for (int step = 1; step <= steps; step++)
        {
            if (step == 200 && (kinds & Kinds.Late) != 0)
            {
                late = w.CreateBody(BodyType.Static, new Vector2D(3, 1));
                w.CreateBox(late, new Vector2F(6f, 2f), default, 0f, 0f, new ShapeDef { IsSensor = true });
                Count(late);
            }

            if (step == 450 && (kinds & Kinds.Late) != 0) { late.Destroy(); }
            if (step == 600 && (kinds & Kinds.Kinematic) != 0) { kinematicSensor.Destroy(); }
            w.Step(Dt);
            if (step % 30 == 0)
            {
                ulong h = 0xCBF29CE484222325UL;
                foreach (var b in solids)
                {
                    var p = b.Position; var r = b.Rotation; var v = b.LinearVelocity;
                    h = Mix(h, (ulong)BitConverter.DoubleToInt64Bits(p.X)); h = Mix(h, (ulong)BitConverter.DoubleToInt64Bits(p.Y));
                    h = Mix(h, BitConverter.SingleToUInt32Bits(r.C)); h = Mix(h, BitConverter.SingleToUInt32Bits(r.S));
                    h = Mix(h, BitConverter.SingleToUInt32Bits(v.X)); h = Mix(h, BitConverter.SingleToUInt32Bits(v.Y));
                    h = Mix(h, BitConverter.SingleToUInt32Bits(b.AngularVelocity));
                }

                run.Fingerprints.Add(h);
            }
        }

        return run;
    }

    [Theory]
    [InlineData(Kinds.Static)]
    [InlineData(Kinds.Kinematic)]
    [InlineData(Kinds.Dynamic)]
    [InlineData(Kinds.Late)]
    [InlineData(Kinds.All)]
    public void ASensorLeavesEveryBodyBitIdentical(Kinds kinds)
    {
        var without = Simulate(Kinds.None, 1);
        var with = Simulate(kinds, 1);
        Assert.Equal(without.Fingerprints, with.Fingerprints);
        Assert.True(with.Enters > 0, "the sensors must see something");
    }

    [Fact]
    public void SensorsAndThreadsTogetherStayIdentical()
    {
        var one = Simulate(Kinds.All, 1);
        var many = Simulate(Kinds.All, 8);
        Assert.Equal(one.Fingerprints, many.Fingerprints);
        Assert.Equal(one.Enters, many.Enters);
        Assert.Equal(one.Exits, many.Exits);
    }
}
