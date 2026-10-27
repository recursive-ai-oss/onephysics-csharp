using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Core.Reference;

// Usage: Fuzz <target> <file>...   runs the target once per file (a crash or an unexpected exception is a finding)
//        Fuzz <target> --random <seed> <count>   runs it over a seeded pseudo-random byte stream
internal static class Program
{
    private static readonly Dictionary<string, Action<byte[]>> Targets = new(StringComparer.Ordinal)
    {
        ["snapshot"] = SnapshotTarget,
        ["snapshot-raw"] = SnapshotRawTarget,
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

            Console.WriteLine("ran " + count + " (restored and stepped: " + s_accepted + ")");
            return 0;
        }

        for (int i = 1; i < args.Length; i++)
        {
            target(File.ReadAllBytes(args[i]));
        }

        Console.WriteLine("ran " + (args.Length - 1));
        return 0;
    }

    private static byte[]? s_base;
    private static int s_accepted;

    private static byte[] Base()
    {
        if (s_base == null)
        {
            var scene = ParticleScene.CreatePile(24, 5, worldId: 1);
            scene.ReportPersist = true;
            for (int i = 0; i < 20; i++) { scene.Step(1f / 60f); }
            scene.World.DestroyBody(scene.World.HandleOfBody(2));
            s_base = scene.World.Snapshot();
        }

        return s_base;
    }

    // Bytes of any shape go straight to Restore: the only allowed outcome besides success is a SnapshotFormatException.
    private static void SnapshotRawTarget(byte[] data)
    {
        var w = new WorldCore(new WorldOptions { WorldId = 1 });
        try { w.Restore(data); }
        catch (SnapshotFormatException) { }
    }

    // The input drives mutations of a valid snapshot (pairs of position and xor value, plus truncation), so the fuzzer gets past the header.
    // A snapshot that restores must be a usable world that steps without error and snapshots to the same bytes it was restored from.
    private static void SnapshotTarget(byte[] data)
    {
        var bytes = (byte[])Base().Clone();
        int mutations = data.Length > 0 ? 1 + data[0] % 3 : 0;
        for (int i = 1, m = 0; i + 2 < data.Length && m < mutations; i += 3, m++)
        {
            int at = ((data[i] << 8) | data[i + 1]) % bytes.Length;
            bytes[at] ^= data[i + 2];
        }

        if (data.Length > 0 && (data[0] & 0x0F) == 0x0F) { Array.Resize(ref bytes, (data[^1] * bytes.Length) / 256); }

        var scene = new ParticleScene(new WorldOptions { WorldId = 1 });
        try { scene.World.Restore(bytes); }
        catch (SnapshotFormatException) { return; }
        s_accepted++;
        byte[] again = scene.World.Snapshot();
        if (!again.AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("accepted snapshot does not re-serialize to the same bytes"); }
        scene.Step(1f / 60f);
        scene.Step(1f / 60f);
    }
}
