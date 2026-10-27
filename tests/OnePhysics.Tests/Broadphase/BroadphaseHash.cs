using System;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Core;

/// <summary>
/// A fixed broadphase scenario in 2D and 3D (creates, moves, destroys, pair updates on 1 and several threads, rays, a snapshot round trip),
/// with every output bit folded into one FNV-1a hash. Every platform, vector width, runtime and thread count must produce the same hash;
/// the unit tests, the NativeAOT smoke app and the WASM smoke app all check it.
/// </summary>
internal static class BroadphaseHash
{
    /// <summary>The hash every platform must produce.</summary>
    public const ulong Expected = 4661559988502166513UL;

    private delegate TBox MakeBox<TBox>(ReadOnlySpan<double> min, ReadOnlySpan<double> max);

    private struct RayCollect : IRayCallback
    {
        public ulong Hash;
        public double Report(int proxy, int userId, double maxFraction)
        {
            Hash = Fold(Fold(Hash, (ulong)proxy), BitConverter.DoubleToUInt64Bits(maxFraction));
            return maxFraction * 0.75;
        }
    }

    private struct Even : IPairFilter
    {
        public bool ShouldCollide(int a, int b) => ((a + b) & 7) != 0;
    }

    private static ulong Fold(ulong h, ulong v) => (h ^ v) * 1099511628211UL;

    private static double Unit(TestRng rng) => (rng.NextU64() >> 11) * (1.0 / 9007199254740992.0);

    public static ulong Compute()
    {
        ulong h = 14695981039346656037UL;
        h = Run<Aabb2D>(h, 2, (mn, mx) => new Aabb2D(mn[0], mn[1], mx[0], mx[1]),
            bp => { var cb = new RayCollect { Hash = 7 }; double f = bp.RayCast(new Ray2D(-40, -35, 80, 71), 1.0, ref cb); return Fold(cb.Hash, BitConverter.DoubleToUInt64Bits(f)); });
        h = Run<Aabb3D>(h, 3, (mn, mx) => new Aabb3D(mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]),
            bp => { var cb = new RayCollect { Hash = 7 }; double f = bp.RayCast(new Ray3D(-40, -35, -30, 80, 71, 66), 1.0, ref cb); return Fold(cb.Hash, BitConverter.DoubleToUInt64Bits(f)); });
        return h;
    }

    private static ulong Run<TBox>(ulong h, int axes, MakeBox<TBox> make, Func<Broadphase<TBox>, ulong> ray) where TBox : struct, IBox<TBox>
    {
        var rng = new TestRng(2024 + (ulong)axes);
        var bp = new Broadphase<TBox>(sectionId: 77);
        var keys = new System.Collections.Generic.List<int>();
        var exec = new TaskExecutor(4);
        var mn = new double[3];
        var mx = new double[3];
        var disp = new double[3];

        TBox Random()
        {
            for (int a = 0; a < axes; a++)
            {
                double c = (Unit(rng) * 60.0) - 30.0, e = 0.2 + (Unit(rng) * 2.0);
                mn[a] = c - e;
                mx[a] = c + e;
            }

            return make(mn.AsSpan(0, axes), mx.AsSpan(0, axes));
        }

        for (int i = 0; i < 400; i++) { keys.Add(bp.CreateProxy((BodyType)(i % 5 == 0 ? 0 : (i % 7 == 0 ? 1 : 2)), Random(), i)); }
        bp.Rebuild(BodyTypeMask.Static);
        for (int round = 0; round < 24; round++)
        {
            for (int m = 0; m < 120; m++)
            {
                int key = keys[rng.Next(keys.Count)];
                if (bp.GetBodyType(key) == BodyType.Static) { continue; }
                TBox cur = bp.GetFatBounds(key);
                for (int a = 0; a < axes; a++)
                {
                    double d = (Unit(rng) * 2.0) - 1.0;
                    disp[a] = d;
                    mn[a] = cur.Min(a) + 0.1 + d;
                    mx[a] = cur.Max(a) - 0.1 + d;
                }

                bp.MoveProxy(key, make(mn.AsSpan(0, axes), mx.AsSpan(0, axes)), disp.AsSpan(0, axes));
            }

            if (round % 5 == 4)
            {
                int victim = rng.Next(keys.Count);
                bp.DestroyProxy(keys[victim]);
                keys[victim] = bp.CreateProxy(BodyType.Dynamic, Random(), 1000 + round);
            }

            ReadOnlySpan<ProxyPair> pairs = round % 2 == 0 ? bp.UpdatePairs(new AcceptAllPairs(), exec) : bp.UpdatePairs(new Even());
            h = Fold(h, (ulong)pairs.Length);
            foreach (ProxyPair p in pairs) { h = Fold(Fold(Fold(Fold(h, (ulong)p.ProxyA), (ulong)p.ProxyB), (ulong)p.UserA), (ulong)p.UserB); }
            if (round == 12) { bp.Rebuild(); }
        }

        h = Fold(h, ray(bp));
        var hw = StateWriter.ForHash();
        bp.Write(hw);
        h = Fold(h, hw.Hash);

        // A snapshot restored into a fresh broadphase must continue identically.
        var w = StateWriter.ToBuffer();
        bp.Write(w);
        var copy = new Broadphase<TBox>(sectionId: 77);
        copy.Read(new StateReader(w.ToArray()));
        var w2 = StateWriter.ToBuffer();
        copy.Write(w2);
        bool same = w2.ToArray().AsSpan().SequenceEqual(w.ToArray());
        h = Fold(h, same ? 1UL : 2UL);
        return h;
    }
}
