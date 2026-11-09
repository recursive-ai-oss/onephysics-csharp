#pragma warning disable OPHYS0003
using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Limp 2D humanoids dropped at random angles from 0.3 to 2.3 m must settle: no part flies off.</summary>
public class RagdollDropStress2DTests
{
    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static double Unit(ref ulong s) => (Next(ref s) >> 11) * (1.0 / 9007199254740992.0);

    [Fact]
    public void LimpHumanoidDropsSettle()
    {
        int bad = 0;
        for (int n = 0; n < 40; n++)
        {
            ulong s = 99UL + (ulong)n * 7919UL;
            var o = new WorldOptions2D { WorldId = 79 };
            Ragdoll.ApplyRecommendedOptions(o);
            var w = new PhysicsWorld2D(o);
            w.AddBox(new Vector2F(200f, 1f), new Vector2D(0, -0.5), type: BodyType.Static);
            var def = RagdollDef.Humanoid;
            var rag = w.AddRagdoll(def, new Vector2D(0, 0.3 + 2.0 * Unit(ref s) + 1.9), (float)(Unit(ref s) * 6.283185307));
            double maxSpeed = 0; bool touched = false;
            for (int step = 0; step < 600; step++)
            {
                w.Step(1f / 60f);
                for (int i = 0; i < rag.PartCount; i++)
                {
                    var b = rag.GetBody(i);
                    if (b.Position.Y < 0.3) { touched = true; }
                    if (!touched) { continue; }
                    var v = b.LinearVelocity;
                    double sp = Math.Sqrt(v.X * v.X + v.Y * v.Y);
                    if (!(sp <= maxSpeed)) { maxSpeed = sp; }
                }
            }

            if (!(maxSpeed <= 20)) { bad++; }
        }

        Assert.Equal(0, bad);
    }
}
