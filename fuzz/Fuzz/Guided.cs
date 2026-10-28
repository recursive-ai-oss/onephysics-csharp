using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

// Coverage-guided mode. The library assembly is instrumented by the SharpFuzz tool (tools/fuzz.sh does that on a copy of the build output);
// every instrumented branch bumps a byte in an edge map, as AFL and libFuzzer do. This driver owns the map, keeps the inputs that reach a
// new (edge, hit-count bucket), mutates them, and records every input that makes a target throw, mismatch or hang.
// It runs in-process so that it needs no native fuzzer on the host. An uninstrumented build still runs; it just sees an empty map.
internal static unsafe class Guided
{
    private const int MapSize = 1 << 16;

    private static readonly byte[] BucketOf = BuildBuckets();
    private static byte* s_map;
    private static readonly byte[] s_virgin = new byte[MapSize];

    private static long s_execStart;       // Stopwatch ticks when the current input began, 0 while idle
    private static byte[] s_current = Array.Empty<byte>();
    private static string s_crashDir = ".";
    private static int s_hangSeconds = 30;

    private static byte[] BuildBuckets()
    {
        // AFL's hit-count classes: 1, 2, 3, 4-7, 8-15, 16-31, 32-127, 128+.
        var t = new byte[256];
        for (int i = 0; i < 256; i++)
        {
            t[i] = i == 0 ? (byte)0 : i == 1 ? (byte)1 : i == 2 ? (byte)2 : i == 3 ? (byte)4 : i < 8 ? (byte)8 : i < 16 ? (byte)16 : i < 32 ? (byte)32 : i < 128 ? (byte)64 : (byte)128;
        }

        return t;
    }

    internal static void InstallMap()
    {
        if (s_map == null)
        {
            s_map = (byte*)NativeMemory.AllocZeroed(MapSize);
            SharpFuzz.Common.Trace.SharedMem = s_map;
        }
    }

    // Runs one input and returns whether it reached a new (edge, bucket); a throwing input is reported through `failure`.
    private static bool Execute(Action<byte[]> target, byte[] input, out Exception? failure)
    {
        new Span<byte>(s_map, MapSize).Clear();
        s_current = input;
        Interlocked.Exchange(ref s_execStart, Stopwatch.GetTimestamp());
        failure = null;
        try { target(input); }
        catch (Exception e) { failure = e; }
        Interlocked.Exchange(ref s_execStart, 0);

        bool fresh = false;
        for (int i = 0; i < MapSize; i++)
        {
            byte hit = s_map[i];
            if (hit != 0)
            {
                byte b = BucketOf[hit];
                if ((s_virgin[i] & b) == 0) { s_virgin[i] |= b; fresh = true; }
            }
        }

        return fresh;
    }

    private static (int Edges, int Features) Coverage()
    {
        int edges = 0, features = 0;
        for (int i = 0; i < MapSize; i++)
        {
            if (s_virgin[i] != 0) { edges++; features += System.Numerics.BitOperations.PopCount(s_virgin[i]); }
        }

        return (edges, features);
    }

    // Two crashes are the same finding when their exception type and top frames agree; the message may carry input-dependent numbers.
    private static string Signature(Exception e)
    {
        var sb = new StringBuilder(e.GetType().Name);
        // The message with its numbers removed tells apart findings from one check that name different shapes or fields.
        sb.Append(':');
        foreach (char c in e.Message.Length > 90 ? e.Message[..90] : e.Message) { if (!char.IsDigit(c) && c != '.' && c != '-' && c != ',') { sb.Append(c); } }
        var trace = new StackTrace(e, false);
        for (int i = 0; i < trace.FrameCount && i < 4; i++)
        {
            var m = trace.GetFrame(i)?.GetMethod();
            sb.Append('|').Append(m?.DeclaringType?.Name).Append('.').Append(m?.Name);
        }

        return sb.ToString();
    }

    private static string Fnv(string s)
    {
        ulong h = 14695981039346656037UL;
        foreach (char c in s) { h = (h ^ c) * 1099511628211UL; }
        return h.ToString("x16", CultureInfo.InvariantCulture)[..10];
    }

    private static void Watchdog(Stopwatch clock, string target)
    {
        while (true)
        {
            Thread.Sleep(500);
            long start = Interlocked.Read(ref s_execStart);
            if (start != 0 && Stopwatch.GetElapsedTime(start).TotalSeconds > s_hangSeconds)
            {
                string path = Path.Combine(s_crashDir, "hang-" + target + "-" + Fnv(Convert.ToHexString(s_current)));
                File.WriteAllBytes(path, s_current);
                Console.WriteLine("HANG: input running for more than " + s_hangSeconds + " s saved to " + path);
                Environment.Exit(77);
            }
        }
    }

    private static readonly long[] Interesting =
    {
        0, 1, -1, 2, 3, 7, 8, 15, 16, 31, 32, 63, 64, 100, 127, 128, 255, 256, 511, 512, 1000, 1024, 4096, 32767, 32768, 65535, 65536,
        0x7FFFFFFF, 0x80000000L, 0xFFFFFFFFL, 0x3F800000, 0xBF800000, 0x7F800000, 0xFF800000, 0x7FC00000, 0x00800000, 0x00000001,
    };

    private static byte[] Mutate(Random rng, byte[] src, List<byte[]> corpus, int maxLen)
    {
        var d = new List<byte>(src);
        int rounds = 1 << rng.Next(0, 4);
        for (int r = 0; r < rounds; r++)
        {
            int n = d.Count;
            switch (rng.Next(0, 12))
            {
                case 0 when n > 0: d[rng.Next(n)] ^= (byte)(1 << rng.Next(8)); break;
                case 1 when n > 0: d[rng.Next(n)] = (byte)rng.Next(256); break;
                case 2 when n > 0: d[rng.Next(n)] += (byte)rng.Next(1, 36); break;
                case 3 when n > 0: d[rng.Next(n)] -= (byte)rng.Next(1, 36); break;
                case 4 when n > 1:
                    {
                        int at = rng.Next(n - 1);
                        long v = Interesting[rng.Next(Interesting.Length)];
                        int width = n - at >= 4 && rng.Next(2) == 0 ? 4 : 2;
                        for (int k = 0; k < width; k++) { d[at + k] = (byte)(v >> (8 * k)); }
                        break;
                    }

                case 5 when n < maxLen:
                    {
                        int at = rng.Next(n + 1), len = rng.Next(1, Math.Min(16, maxLen - n) + 1);
                        var ins = new byte[len];
                        rng.NextBytes(ins);
                        d.InsertRange(at, ins);
                        break;
                    }

                case 6 when n > 1:
                    {
                        int at = rng.Next(n), len = rng.Next(1, Math.Min(16, n - at) + 1);
                        d.RemoveRange(at, len);
                        break;
                    }

                case 7 when n > 0 && n < maxLen:
                    {
                        int at = rng.Next(n), len = rng.Next(1, Math.Min(32, n - at) + 1);
                        len = Math.Min(len, maxLen - n);
                        d.InsertRange(rng.Next(n + 1), d.GetRange(at, len));
                        break;
                    }

                case 8 when n > 0:
                    {
                        byte[] other = corpus[rng.Next(corpus.Count)];
                        if (other.Length > 0)
                        {
                            int from = rng.Next(other.Length), len = rng.Next(1, Math.Min(32, other.Length - from) + 1), at = rng.Next(n);
                            for (int k = 0; k < len && at + k < n; k++) { d[at + k] = other[from + k]; }
                        }

                        break;
                    }

                case 9:
                    {
                        byte[] other = corpus[rng.Next(corpus.Count)];
                        int cut = d.Count == 0 ? 0 : rng.Next(d.Count), from = other.Length == 0 ? 0 : rng.Next(other.Length);
                        d.RemoveRange(cut, d.Count - cut);
                        for (int k = from; k < other.Length && d.Count < maxLen; k++) { d.Add(other[k]); }
                        break;
                    }

                case 10 when n > 0:
                    {
                        int at = rng.Next(n), len = rng.Next(1, Math.Min(8, n - at) + 1);
                        for (int k = 0; k < len; k++) { d[at + k] = (byte)rng.Next(256); }
                        break;
                    }

                case 11 when n > 3:
                    {
                        int at = rng.Next(n - 3);
                        for (int k = 0; k < 4; k++) { d[at + k] = (byte)(rng.Next(2) == 0 ? 0 : 255); }
                        break;
                    }
            }
        }

        if (d.Count > maxLen) { d.RemoveRange(maxLen, d.Count - maxLen); }
        return d.ToArray();
    }

    private static string Arg(string[] args, string name, string fallback)
    {
        int i = Array.IndexOf(args, name);
        return i >= 0 && i + 1 < args.Length ? args[i + 1] : fallback;
    }

    // Fuzz <target> --guided <seconds> --corpus <dir> --crashes <dir> [--seed n] [--maxlen n] [--hang seconds] [--stats file]
    public static int Run(string name, Action<byte[]> target, string[] args)
    {
        double seconds = double.Parse(args[Array.IndexOf(args, "--guided") + 1], CultureInfo.InvariantCulture);
        string corpusDir = Arg(args, "--corpus", "corpus/" + name);
        s_crashDir = Arg(args, "--crashes", "crashes/" + name);
        int seed = int.Parse(Arg(args, "--seed", "1"), CultureInfo.InvariantCulture);
        int maxLen = int.Parse(Arg(args, "--maxlen", "4096"), CultureInfo.InvariantCulture);
        s_hangSeconds = int.Parse(Arg(args, "--hang", "30"), CultureInfo.InvariantCulture);
        string? statsFile = Arg(args, "--stats", "");
        Directory.CreateDirectory(corpusDir);
        Directory.CreateDirectory(s_crashDir);
        InstallMap();

        var clock = Stopwatch.StartNew();
        new Thread(() => Watchdog(clock, name)) { IsBackground = true }.Start();

        // The input of the run in flight is kept in a file so that a crash that kills the process (stack overflow, out of memory) leaves its input behind.
        using var currentHandle = File.OpenHandle(Path.Combine(s_crashDir, ".current"), FileMode.Create, FileAccess.Write, FileShare.ReadWrite);
        var rng = new Random(seed);
        var corpus = new List<byte[]>();
        var findings = new Dictionary<string, int>(StringComparer.Ordinal);
        long execs = 0;
        int crashes = 0;

        bool Try(byte[] input, bool store)
        {
            RandomAccess.SetLength(currentHandle, 0);
            RandomAccess.Write(currentHandle, input, 0);
            execs++;
            bool fresh = Execute(target, input, out Exception? failure);
            if (failure != null)
            {
                string sig = Signature(failure);
                string file = Path.Combine(s_crashDir, "crash-" + Fnv(sig));
                if (!findings.ContainsKey(sig))
                {
                    findings[sig] = 0;
                    crashes++;
                    Console.WriteLine("CRASH " + sig + " -> " + file + ": " + failure.Message);
                    File.WriteAllBytes(file, input);
                    File.WriteAllText(file + ".txt", sig + "\n" + failure + "\n");
                }
                else if (File.Exists(file) && new FileInfo(file).Length > input.Length)
                {
                    File.WriteAllBytes(file, input);
                }

                findings[sig]++;
                return false;
            }

            if (fresh && store)
            {
                corpus.Add(input);
                File.WriteAllBytes(Path.Combine(corpusDir, Fnv(Convert.ToHexString(input))), input);
            }

            return fresh;
        }

        foreach (string f in Directory.GetFiles(corpusDir).OrderBy(f => f, StringComparer.Ordinal))
        {
            byte[] bytes = File.ReadAllBytes(f);
            Try(bytes, store: false);
            corpus.Add(bytes);
        }

        if (corpus.Count == 0) { Try(new byte[16], store: true); if (corpus.Count == 0) { corpus.Add(new byte[16]); } }
        long loaded = execs;
        long nextReport = 30_000;
        while (clock.ElapsedMilliseconds < seconds * 1000)
        {
            // Half the picks come from the newest quarter of the corpus, which is where new coverage tends to be.
            int pick = rng.Next(2) == 0 ? rng.Next(Math.Max(0, corpus.Count - (corpus.Count / 4) - 1), corpus.Count) : rng.Next(corpus.Count);
            Try(Mutate(rng, corpus[pick], corpus, maxLen), store: true);
            if (clock.ElapsedMilliseconds >= nextReport)
            {
                nextReport += 30_000;
                var cv = Coverage();
                Console.WriteLine($"[{(int)clock.Elapsed.TotalSeconds} s] {name}: execs {execs} edges {cv.Edges} features {cv.Features} corpus {corpus.Count} crashes {crashes}");
            }
        }

        var cov = Coverage();
        string line = $"{name}\t{execs - loaded}\t{cov.Edges}\t{cov.Features}\t{corpus.Count}\t{crashes}\t{clock.Elapsed.TotalSeconds.ToString("F0", CultureInfo.InvariantCulture)}";
        Console.WriteLine("STATS target execs edges features corpus crashes seconds");
        Console.WriteLine("STATS " + line);
        if (statsFile.Length > 0) { File.AppendAllText(statsFile, line + "\n"); }
        return crashes == 0 ? 0 : 1;
    }

    // Fuzz <target> --minimize <input> <output>: shrinks an input while it keeps throwing the same finding.
    public static int Minimize(string name, Action<byte[]> target, string inputPath, string outputPath)
    {
        InstallMap();
        byte[] best = File.ReadAllBytes(inputPath);
        Execute(target, best, out Exception? first);
        if (first == null) { Console.Error.WriteLine("input does not fail"); return 2; }
        string sig = Signature(first);
        bool Fails(byte[] b) { Execute(target, b, out Exception? e); return e != null && Signature(e) == sig; }

        bool progress = true;
        while (progress)
        {
            progress = false;
            for (int chunk = Math.Max(1, best.Length / 2); chunk >= 1; chunk /= 2)
            {
                for (int at = 0; at + chunk <= best.Length;)
                {
                    var cand = new byte[best.Length - chunk];
                    Array.Copy(best, 0, cand, 0, at);
                    Array.Copy(best, at + chunk, cand, at, best.Length - at - chunk);
                    if (Fails(cand)) { best = cand; progress = true; } else { at += chunk; }
                }
            }

            for (int i = 0; i < best.Length; i++)
            {
                if (best[i] == 0) { continue; }
                byte keep = best[i];
                best[i] = 0;
                if (Fails(best)) { progress = true; } else { best[i] = keep; }
            }
        }

        File.WriteAllBytes(outputPath, best);
        Console.WriteLine($"minimized {name} {sig}: {new FileInfo(inputPath).Length} -> {best.Length} bytes");
        return 0;
    }
}
