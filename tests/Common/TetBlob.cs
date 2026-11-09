using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>A closed, star-shaped, character-like surface: a sphere pulled out into a head, two arms and two legs by smooth bumps along fixed directions. Used to test tetrahedralization on a shape with thin features.</summary>
public static class TetBlob
{
    private static readonly (float X, float Y, float Z, float Height, float Width)[] Bumps =
    {
        (0f, 1f, 0f, 0.55f, 0.35f),          // head
        (1f, 0.35f, 0f, 0.9f, 0.28f),        // arms
        (-1f, 0.35f, 0f, 0.9f, 0.28f),
        (0.35f, -1f, 0f, 0.8f, 0.3f),        // legs
        (-0.35f, -1f, 0f, 0.8f, 0.3f),
    };

    /// <summary>The surface radius in direction (x, y, z), a unit vector, for a body of base radius 1.</summary>
    public static double Radius(double x, double y, double z)
    {
        double r = 1.0;
        foreach (var (bx, by, bz, h, w) in Bumps)
        {
            double len = Math.Sqrt(bx * bx + by * by + bz * bz);
            double dot = (x * bx + y * by + z * bz) / len;
            double angle = Math64.Acos(Math.Max(-1.0, Math.Min(1.0, dot)));
            r += h * Math64.Exp(-(angle * angle) / (w * w));
        }

        return r;
    }

    public static TestMesh Build(int rings, int segments, float scale = 1f)
    {
        var m = new TestMesh();
        Vector3F P(double phi, double th)
        {
            double sp = Math64.Sin(phi), x = sp * Math64.Cos(th), y = Math64.Cos(phi), z = sp * Math64.Sin(th);
            double r = Radius(x, y, z) * scale;
            return new Vector3F((float)(r * x), (float)(r * y), (float)(r * z));
        }

        var top = P(0, 0); m.Add(top.X, top.Y, top.Z);
        for (int i = 1; i < rings; i++)
        {
            double phi = Math.PI * i / rings;
            for (int j = 0; j < segments; j++) { var p = P(phi, 2 * Math.PI * j / segments); m.Add(p.X, p.Y, p.Z); }
        }

        var bot = P(Math.PI, 0); int bottom = m.Add(bot.X, bot.Y, bot.Z);
        for (int j = 0; j < segments; j++) { m.Tri(0, 1 + (j + 1) % segments, 1 + j); }
        for (int i = 0; i < rings - 2; i++)
        {
            for (int j = 0; j < segments; j++)
            {
                int a = 1 + i * segments + j, b = 1 + i * segments + (j + 1) % segments, c = a + segments, d = b + segments;
                m.Quad(a, b, d, c);
            }
        }

        int last = 1 + (rings - 2) * segments;
        for (int j = 0; j < segments; j++) { m.Tri(bottom, last + j, last + (j + 1) % segments); }
        return m;
    }
}
