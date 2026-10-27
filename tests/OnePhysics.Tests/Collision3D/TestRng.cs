using System;
using RecursiveAI.OneMath;

/// <summary>A small seeded generator (SplitMix64) so test inputs are identical on every platform.</summary>
internal sealed class TestRng
{
    private ulong _s;

    public TestRng(ulong seed) => _s = seed;

    public ulong NextU64()
    {
        ulong z = _s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    /// <summary>Uniform in [0, 1).</summary>
    public float NextFloat() => (NextU64() >> 40) * (1f / 16777216f);

    public float Range(float lo, float hi) => lo + ((hi - lo) * NextFloat());

    public int Next(int n) => (int)(NextU64() % (ulong)n);

    public Vector3F InCube(float h) => new(Range(-h, h), Range(-h, h), Range(-h, h));

    public Vector3F InBall(float r)
    {
        while (true)
        {
            Vector3F v = InCube(1f);
            if (v.LengthSquared() <= 1f)
            {
                return v * r;
            }
        }
    }

    public Vector3F OnSphere()
    {
        while (true)
        {
            Vector3F v = InCube(1f);
            float l = v.LengthSquared();
            if (l > 0.01f && l <= 1f)
            {
                return v.Normalized();
            }
        }
    }

    public QuaternionF Rotation()
    {
        while (true)
        {
            var q = new QuaternionF(Range(-1, 1), Range(-1, 1), Range(-1, 1), Range(-1, 1));
            float l = q.LengthSquared();
            if (l > 0.01f && l <= 1f)
            {
                return q.Normalized();
            }
        }
    }
}
