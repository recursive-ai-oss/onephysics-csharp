using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Limp swing-twist ragdolls dropped in random orientations must settle: no part flies off and the joints stay together.</summary>
public class RagdollDropStressTests
{
    private const float Dt = 1f / 60f;

    private static ulong Next(ref ulong s)
    {
        s += 0x9E3779B97F4A7C15UL;
        ulong z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    private static double Unit(ref ulong s) => (Next(ref s) >> 11) * (1.0 / 9007199254740992.0);

    private static QuaternionF RandomRotation(ref ulong s)
    {
        // Shoemake: a uniformly random rotation from three uniform numbers.
        double u1 = Unit(ref s), u2 = Unit(ref s) * 2.0 * Math.PI, u3 = Unit(ref s) * 2.0 * Math.PI;
        double a = Math.Sqrt(1 - u1), b = Math.Sqrt(u1);
        return new QuaternionF((float)(a * Math.Sin(u2)), (float)(a * Math.Cos(u2)), (float)(b * Math.Sin(u3)), (float)(b * Math.Cos(u3)));
    }

    /// <summary>Largest distance between a joint's anchor as seen from the parent body and from the child body.</summary>
    private static float JointError(Ragdoll rag, RagdollDef def)
    {
        float worst = 0f;
        for (int i = 1; i < def.Parts.Length; i++)
        {
            var part = def.Parts[i];
            int p = part.Parent;
            var parent = def.Parts[p];
            var bc = rag.GetBody(i); var bp = rag.GetBody(p);
            var offC = (bc.Rotation * part.Rotation.Conjugated()).Rotate(part.JointPosition - part.Position);
            var offP = (bp.Rotation * parent.Rotation.Conjugated()).Rotate(part.JointPosition - parent.Position);
            var ac = bc.Position + new Vector3D(offC.X, offC.Y, offC.Z);
            var ap = bp.Position + new Vector3D(offP.X, offP.Y, offP.Z);
            var d = ac - ap;
            float e = (float)Math.Sqrt(d.X * d.X + d.Y * d.Y + d.Z * d.Z);
            if (!(e <= worst)) { worst = e; }
        }

        return worst;
    }

    internal static (int bad, float worstSpeed, float worstError) Drops(Func<RagdollDef> make, int count, ulong seed, Action<string>? log = null)
    {
        int bad = 0; float worstSpeed = 0f, worstError = 0f;
        for (int n = 0; n < count; n++)
        {
            ulong s = seed + (ulong)n * 7919UL;
            var o = new WorldOptions3D { WorldId = 77 };
            Ragdoll.ApplyRecommendedOptions(o);
            var w = new PhysicsWorld3D(o);
            w.AddBox(new Vector3F(200f, 1f, 200f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(0.8f));
            var def = make();
            double h = 0.3 + 2.0 * Unit(ref s);
            var rag = w.AddRagdoll(def, new Vector3D(0, h + 1.9, 0), RandomRotation(ref s));
            float maxSpeed = 0f, maxErr = 0f;
            bool touched = false;
            for (int step = 0; step < 600; step++)
            {
                w.Step(Dt);
                bool low = false;
                for (int i = 0; i < rag.PartCount; i++)
                {
                    var b = rag.GetBody(i);
                    if (b.Position.Y < 0.3) { low = true; }
                }
                touched |= low;
                if (!touched) { continue; }
                for (int i = 0; i < rag.PartCount; i++)
                {
                    var v = rag.GetBody(i).LinearVelocity;
                    float sp = (float)Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
                    if (!(sp <= maxSpeed)) { maxSpeed = sp; }
                }
                float e = JointError(rag, def);
                if (!(e <= maxErr)) { maxErr = e; }
            }

            if (!(maxSpeed <= 20f) || !(maxErr <= 0.3f)) { bad++; log?.Invoke("drop " + n + ": speed " + maxSpeed + " error " + maxErr); }
            worstSpeed = Math.Max(worstSpeed, maxSpeed); worstError = Math.Max(worstError, maxErr);
        }

        return (bad, worstSpeed, worstError);
    }

    private static readonly int Count = int.Parse(Environment.GetEnvironmentVariable("ONEPHYSICS_DROP_COUNT") ?? "40");

    private readonly ITestOutputHelper _out;
    public RagdollDropStressTests(ITestOutputHelper output) { _out = output; }

    [Fact]
    public void LimpHumanoidDropsSettle()
    {
        var (bad, speed, err) = Drops(() => RagdollDef.Humanoid, Count, 12345UL, _out.WriteLine);
        _out.WriteLine("bad " + bad + " worstSpeed " + speed + " worstError " + err);
        Assert.Equal(0, bad);
    }
}
