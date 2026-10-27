using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

// Usage: Fuzz <target> <file>...   runs the target once per file (a crash or an unexpected exception is a finding)
//        Fuzz <target> --random <seed> <count>   runs it over a seeded pseudo-random byte stream
internal static class Program
{
    private static readonly Dictionary<string, Action<byte[]>> Targets = new(StringComparer.Ordinal)
    {
        ["gravity"] = GravityTarget,
    };

    private static int Main(string[] args)
    {
        if (args.Length < 2 || !Targets.TryGetValue(args[0], out var target))
        {
            Console.Error.WriteLine("usage: Fuzz <target> <file>... | Fuzz <target> --random <seed> <count>; targets: " + string.Join(", ", Targets.Keys));
            return 2;
        }

        if (args[1] == "--random" && args.Length == 4)
        {
            var rng = new Random(int.Parse(args[2], System.Globalization.CultureInfo.InvariantCulture));
            int count = int.Parse(args[3], System.Globalization.CultureInfo.InvariantCulture);
            var buffer = new byte[64];
            for (int i = 0; i < count; i++)
            {
                rng.NextBytes(buffer);
                target(buffer);
            }

            Console.WriteLine("ran " + count);
            return 0;
        }

        for (int i = 1; i < args.Length; i++)
        {
            target(File.ReadAllBytes(args[i]));
        }

        Console.WriteLine("ran " + (args.Length - 1));
        return 0;
    }

    // Any three floats must give a strength that is NaN or non-negative, and never throw.
    private static void GravityTarget(byte[] data)
    {
        if (data.Length < 12)
        {
            return;
        }

        var v = new Vector3F(BitConverter.ToSingle(data, 0), BitConverter.ToSingle(data, 4), BitConverter.ToSingle(data, 8));
        float s = Gravity.Strength(v);
        if (s < 0f)
        {
            throw new InvalidOperationException("negative strength");
        }
    }
}
