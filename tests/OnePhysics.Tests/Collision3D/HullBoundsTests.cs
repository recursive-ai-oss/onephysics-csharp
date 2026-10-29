using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Collision3D;

/// <summary>The one-pass bounds of a hull give the bits of the six support searches they replace.</summary>
public class HullBoundsTests
{
    private static float Pad(float lo, float hi) => (Math.Abs(lo) + Math.Abs(hi) + (hi - lo)) * (1.0f / 65536f);

    [Fact]
    public void HullBoundsEqualSixSupportQueries()
    {
        var rng = new TestRng(2024);
        var hulls = Collision3DHash.RandomHulls(rng, 12);
        for (int i = 0; i < 20000; i++)
        {
            var shape = ConvexShape3D.FromHull(hulls[rng.Next(hulls.Length)], rng.Next(4) == 0 ? rng.Range(0f, 0.2f) : 0f);
            var pose = new RigidTransformF(rng.Rotation(), rng.InBall(rng.Range(0f, 50f)));
            ShapeBounds3D.Compute(shape, pose, out Vector3F min, out Vector3F max);
            Matrix3x3F m = pose.Rotation.ToMatrix();
            float[] lo = new float[3], hi = new float[3];
            for (int a = 0; a < 3; a++)
            {
                Vector3F row = m.Row(a);
                float t = a == 0 ? pose.Translation.X : a == 1 ? pose.Translation.Y : pose.Translation.Z;
                lo[a] = Vector3F.Dot(row, shape.Support(row * -1f)) + t;
                hi[a] = Vector3F.Dot(row, shape.Support(row * 1f)) + t;
            }

            for (int a = 0; a < 3; a++)
            {
                float p = Pad(lo[a], hi[a]);
                float expectMin = lo[a] - p, expectMax = hi[a] + p;
                float gotMin = a == 0 ? min.X : a == 1 ? min.Y : min.Z, gotMax = a == 0 ? max.X : a == 1 ? max.Y : max.Z;
                Assert.Equal(BitConverter.SingleToInt32Bits(expectMin), BitConverter.SingleToInt32Bits(gotMin));
                Assert.Equal(BitConverter.SingleToInt32Bits(expectMax), BitConverter.SingleToInt32Bits(gotMax));
            }
        }
    }
}
