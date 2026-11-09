#pragma warning disable OPHYS0002, OPHYS0003
using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Tools;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>Limp figures fitted from a skeleton, at the default swing limits, dropped in random orientations from 0.3 to 2.3 m: none may fly apart.</summary>
public class AutoRagdollDropTests
{
    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static float Unit(ref ulong s) => (Next(ref s) >> 40) * (1f / 16777216f);

    [Fact]
    public void FittedFiguresSettleAtDefaultLimits()
    {
        int bad = 0;
        for (int n = 0; n < 40; n++)
        {
            ulong seed = 777UL + (ulong)n * 7919UL;
            var def = AutoRagdollScenes.Make3D(n, 1.5f + 0.4f * Unit(ref seed), 50f + 40f * Unit(ref seed)).Definition;
            var o = new P3.WorldOptions3D { WorldId = 78 };
            P3.Ragdoll.ApplyRecommendedOptions(o);
            var w = new P3.PhysicsWorld3D(o);
            w.AddBox(new Vector3F(60f, 1f, 60f), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            var axis = new Vector3F(Unit(ref seed) - 0.5f, Unit(ref seed) - 0.5f, Unit(ref seed) - 0.5f);
            if (axis.Length() < 0.05f) { axis = new Vector3F(0, 1, 0); }
            var q = QuaternionF.CreateFromAxisAngle(axis.Normalized(), Unit(ref seed) * 6.2831855f);
            var rag = w.AddRagdoll(def, new Vector3D(0, 0.3 + 2.0 * Unit(ref seed) + 1.9, 0), q);
            double maxSpeed = 0, maxErr = 0;
            bool touched = false;
            for (int step = 0; step < 600; step++)
            {
                w.Step(1f / 60f);
                for (int i = 0; i < rag.PartCount; i++)
                {
                    var b = rag.GetBody(i);
                    if (b.Position.Y < 0.3) { touched = true; }
                    if (!touched) { continue; }
                    var v = b.LinearVelocity;
                    double sp = Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
                    if (!(sp <= maxSpeed)) { maxSpeed = sp; }
                }

                if (touched) { double e = AutoRagdollScenes.JointError(rag, def); if (!(e <= maxErr)) { maxErr = e; } }
            }

            if (!(maxSpeed <= 20) || !(maxErr <= 0.3)) { bad++; }
        }

        Assert.Equal(0, bad);
    }
}
