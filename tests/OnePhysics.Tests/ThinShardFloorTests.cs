using System;
using RecursiveAI.OneMath;
using P3 = RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>
/// Long thin convex hulls tumbling into a floor must not sink into it. The far end of such a shape swings farther in one step than the speculative contacts reach, so the step's continuous
/// collision has to see the turn. The metric is geometric: the lowest hull vertex below the floor plane, over 600 steps, for every shard (not the contact solver's own separation).
/// </summary>
public class ThinShardFloorTests(ITestOutputHelper output)
{
    private sealed class Rng(ulong s)
    {
        private ulong _s = s;
        public double Next() { _s = unchecked(_s * 6364136223846793005UL + 1442695040888963407UL); return (_s >> 11) * (1.0 / 9007199254740992.0); }
        public float Range(float a, float b) => a + (b - a) * (float)Next();
    }

    private static P3.PhysicsWorld3D Floor(string kind)
    {
        var w = new P3.PhysicsWorld3D();
        switch (kind)
        {
            case "box":
                w.AddBox(new(80, 1, 80), at: new(0, -0.5, 0), type: BodyType.Static);
                break;
            case "heightfield":
                {
                    const int n = 41;
                    w.AddHeightField(new float[n * n], n, n, new(1, 1, 1), at: new(-20, 0, -20));
                    break;
                }
            default:
                {
                    const int cells = 40, samples = cells + 1;
                    var v = new Vector3F[samples * samples]; var idx = new int[cells * cells * 6];
                    for (int z = 0; z < samples; z++) for (int x = 0; x < samples; x++) v[z * samples + x] = new(x - 20, 0, z - 20);
                    int k = 0;
                    for (int z = 0; z < cells; z++) for (int x = 0; x < cells; x++)
                    {
                        int a = z * samples + x, b = a + 1, c = a + samples, d = c + 1;
                        idx[k++] = a; idx[k++] = c; idx[k++] = b; idx[k++] = b; idx[k++] = c; idx[k++] = d;
                    }
                    w.AddMesh(v, idx);
                    break;
                }
        }

        return w;
    }

    [Theory]
    [InlineData("box")]
    [InlineData("mesh")]
    [InlineData("heightfield")]
    public void SpinningThinShardsStayOnTheFloor(string floor)
    {
        var w = Floor(floor);
        var rng = new Rng(11);
        // 10:1 shards, slightly tapered so they are hulls and not boxes; dropped 0.6 to 3 m up at random attitudes, spinning 2 to 6 rad/s about random axes.
        Vector3F[] hull = [new(-0.5f, -0.05f, -0.05f), new(0.5f, -0.045f, -0.045f), new(-0.5f, 0.05f, -0.05f), new(0.5f, 0.045f, -0.045f),
                           new(-0.5f, -0.05f, 0.05f), new(0.5f, -0.045f, 0.045f), new(-0.5f, 0.05f, 0.05f), new(0.5f, 0.045f, 0.045f)];
        var bodies = new P3.Body[64];
        for (int i = 0; i < 64; i++)
        {
            var axis = new Vector3F(rng.Range(-1, 1), rng.Range(-1, 1), rng.Range(-1, 1)); if (axis.Length() < 0.1f) axis = new(1, 0, 0);
            axis = axis * (1f / axis.Length());
            var q = QuaternionF.CreateFromAxisAngle(axis, rng.Range(0, 6.28f));
            var at = new Vector3D(-10.5 + (i % 8) * 3, rng.Range(0.6f, 3f), -10.5 + (i / 8) * 3);
            var b = w.AddConvexHull(hull, at, q);
            var spin = new Vector3F(rng.Range(-1, 1), rng.Range(-1, 1), rng.Range(-1, 1)); if (spin.Length() < 0.1f) spin = new(0, 1, 0);
            b.AngularVelocity = spin * (rng.Range(2f, 6f) / spin.Length());
            bodies[i] = b;
        }

        double worst = 0;
        for (int step = 0; step < 600; step++)
        {
            w.Step(1f / 60);
            foreach (var b in bodies)
            {
                var p = b.WorldCenterOfMass; var q = b.Rotation;
                foreach (var h in hull) { worst = Math.Max(worst, -(p.Y + q.Rotate(h).Y)); }
            }
        }

        // Tunnelled: a shard whose centre ended below the floor plane (it can only have crossed it).
        int tunnelled = 0;
        foreach (var b in bodies) if (b.WorldCenterOfMass.Y < 0) tunnelled++;
        output.WriteLine($"{floor}: deepest vertex below the floor {worst:F4} m, tunnelled {tunnelled}");
        Assert.True(worst <= 0.05, $"{floor}: deepest vertex {worst} m below the floor");
        Assert.Equal(0, tunnelled);
    }

    // A crate landing at 11 m/s closes 18 cm in one step, far more than the speculative distance, so its contact has to be found from the approach speed, not from the gap.
    [Theory]
    [InlineData("box")]
    [InlineData("mesh")]
    [InlineData("heightfield")]
    public void CratesLandingFastDoNotSinkIn(string floor)
    {
        var w = Floor(floor);
        var crates = new P3.Body[16];
        for (int i = 0; i < crates.Length; i++) { crates[i] = w.AddBox(new(0.8f, 0.8f, 0.8f), new Vector3D(-6 + (i % 4) * 3, 6 + 0.5 * i, -4 + (i / 4) * 3)); }
        double worst = 0;
        for (int step = 0; step < 240; step++)
        {
            w.Step(1f / 60);
            foreach (var b in crates)
            {
                var p = b.WorldCenterOfMass; var q = b.Rotation;
                for (int k = 0; k < 8; k++)
                {
                    var corner = q.Rotate(new Vector3F((k & 1) != 0 ? 0.4f : -0.4f, (k & 2) != 0 ? 0.4f : -0.4f, (k & 4) != 0 ? 0.4f : -0.4f));
                    worst = Math.Max(worst, -(p.Y + corner.Y));
                }
            }
        }

        output.WriteLine($"{floor}: deepest crate corner below the floor {worst:F4} m");
        Assert.True(worst <= 0.05, $"{floor}: deepest crate corner {worst} m below the floor");
    }
}
