using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>A sensor never changes the result of a solid body: adding, moving or removing one leaves every dynamic body bit-identical.</summary>
public class SensorIndependence3DTests
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

    private static Run Simulate(Kinds kinds, int threads, int steps = 600)
    {
        var w = World3DTests.NewWorld(threads, 9);
        w.AddBox(new Vector3F(30, 1, 30), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1, 10, 30), at: new Vector3D(-15.5, 4.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(1, 10, 30), at: new Vector3D(15.5, 4.5, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(30, 10, 1), at: new Vector3D(0, 4.5, -15.5), type: BodyType.Static);
        w.AddBox(new Vector3F(30, 10, 1), at: new Vector3D(0, 4.5, 15.5), type: BodyType.Static);

        // Solid bodies first, so that their slots are the same in every variant.
        var solids = new List<Body>();
        uint seed = 777;
        uint Next() { seed = seed * 1664525u + 1013904223u; return seed >> 8; }
        for (int i = 0; i < 90; i++)
        {
            var at = new Vector3D(-4 + (Next() % 800) / 100.0, 1 + i * 0.35, -4 + (Next() % 800) / 100.0);
            solids.Add(i % 3 == 0
                ? w.AddSphere(0.3f + (Next() % 25) / 100f, at)
                : w.AddBox(new Vector3F(0.4f + (Next() % 50) / 100f, 0.4f + (Next() % 50) / 100f, 0.4f + (Next() % 50) / 100f), at));
        }

        var kin = w.AddBox(new Vector3F(2, 0.5f, 2), at: new Vector3D(-12, 1.5, 0), type: BodyType.Kinematic);
        kin.LinearVelocity = new Vector3F(1.2f, 0, 0);
        solids.Add(kin);

        var run = new Run();
        void Count(Body b) { b.OnTriggerEnter += _ => run.Enters++; b.OnTriggerExit += _ => run.Exits++; }
        Body late = default, kinematicSensor = default;
        if ((kinds & Kinds.Static) != 0)
        {
            var zone = w.CreateBody(BodyType.Static, new Vector3D(0, 2, 0));
            w.CreateBox(zone, new Vector3F(4, 2, 4), 0f, new ShapeDef { IsSensor = true, Density = 0f });
            Count(zone);
            var zone2 = w.CreateBody(BodyType.Static, new Vector3D(-7, 4, 3));
            w.CreateSphere(zone2, 2.5f, new ShapeDef { IsSensor = true, Density = 0f });
        }

        if ((kinds & Kinds.Kinematic) != 0)
        {
            kinematicSensor = w.CreateBody(BodyType.Kinematic, new Vector3D(-12, 1, 0));
            w.CreateSphere(kinematicSensor, 1.5f, new ShapeDef { IsSensor = true, Density = 0f });
            kinematicSensor.LinearVelocity = new Vector3F(1.5f, 0.1f, 0);
            Count(kinematicSensor);
        }

        if ((kinds & Kinds.Dynamic) != 0)
        {
            // A dynamic body that collides with nothing but whose sensor sees everything.
            var d = w.CreateBody(BodyType.Dynamic, new Vector3D(5, 8, 0));
            w.CreateSphere(d, 0.1f, new ShapeDef { CategoryBits = 2, MaskBits = 0 });
            w.CreateSphere(d, 2.5f, new ShapeDef { IsSensor = true, CategoryBits = 2, Density = 0f });
            Count(d);
        }

        for (int step = 1; step <= steps; step++)
        {
            if (step == 150 && (kinds & Kinds.Late) != 0)
            {
                late = w.CreateBody(BodyType.Static, new Vector3D(3, 1, 0));
                w.CreateBox(late, new Vector3F(3, 1, 3), 0f, new ShapeDef { IsSensor = true, Density = 0f });
                Count(late);
            }

            if (step == 330 && (kinds & Kinds.Late) != 0) { late.Destroy(); }
            if (step == 450 && (kinds & Kinds.Kinematic) != 0) { kinematicSensor.Destroy(); }
            w.Step(Dt);
            if (step % 30 == 0)
            {
                ulong h = 0xCBF29CE484222325UL;
                foreach (var b in solids)
                {
                    var p = b.Position; var r = b.Rotation; var v = b.LinearVelocity; var a = b.AngularVelocity;
                    h = Mix(h, (ulong)BitConverter.DoubleToInt64Bits(p.X)); h = Mix(h, (ulong)BitConverter.DoubleToInt64Bits(p.Y)); h = Mix(h, (ulong)BitConverter.DoubleToInt64Bits(p.Z));
                    h = Mix(h, BitConverter.SingleToUInt32Bits(r.X)); h = Mix(h, BitConverter.SingleToUInt32Bits(r.Y)); h = Mix(h, BitConverter.SingleToUInt32Bits(r.Z)); h = Mix(h, BitConverter.SingleToUInt32Bits(r.W));
                    h = Mix(h, BitConverter.SingleToUInt32Bits(v.X)); h = Mix(h, BitConverter.SingleToUInt32Bits(v.Y)); h = Mix(h, BitConverter.SingleToUInt32Bits(v.Z));
                    h = Mix(h, BitConverter.SingleToUInt32Bits(a.X)); h = Mix(h, BitConverter.SingleToUInt32Bits(a.Y)); h = Mix(h, BitConverter.SingleToUInt32Bits(a.Z));
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
