using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

// Fuzz target: the input bytes are a program of broadphase operations (create, destroy, move, touch, update pairs, queries, rebuild, snapshot round trip),
// run against a brute-force model. Any mismatch, broken invariant or exception other than the documented argument errors is a finding.
internal static class BroadphaseTarget
{
    private struct Info
    {
        public int Key;
        public BodyType Type;
        public int User;
    }

    private struct Collect : IOverlapCallback
    {
        public List<int> Keys;
        public bool Report(int proxy, int userId) { Keys.Add(proxy); return true; }
    }

    private struct RayCollect : IRayCallback
    {
        public List<int> Keys;
        public double Report(int proxy, int userId, double maxFraction) { Keys.Add(proxy); return maxFraction; }
    }

    private delegate TBox MakeBox<TBox>(double[] min, double[] max);

    public static void Run(byte[] data)
    {
        if (data.Length < 1) { return; }
        if ((data[0] & 1) == 0)
        {
            Run<Aabb2D>(data, 2, (mn, mx) => new Aabb2D(mn[0], mn[1], mx[0], mx[1]), (o, d) => new Ray2D(o[0], o[1], d[0], d[1]));
        }
        else
        {
            Run<Aabb3D>(data, 3, (mn, mx) => new Aabb3D(mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]), (o, d) => new Ray3D(o[0], o[1], o[2], d[0], d[1], d[2]));
        }
    }

    private static void Run<TBox>(byte[] data, int axes, MakeBox<TBox> make, Func<double[], double[], IRay<TBox>> ray) where TBox : struct, IBox<TBox>
    {
        var bp = new Broadphase<TBox>(new BroadphaseOptions { Margin = (data[0] >> 1 & 3) * 0.1, DisplacementMultiplier = (data[0] >> 3 & 3) }, 5);
        var infos = new List<Info>();
        var moved = new HashSet<int>();
        var exec = new TaskExecutor(2);
        var mn = new double[axes];
        var mx = new double[axes];
        var disp = new double[axes];

        TBox Box(byte a, byte b, byte c)
        {
            for (int k = 0; k < axes; k++)
            {
                byte src = k == 0 ? a : (k == 1 ? b : c);
                double lo = ((src & 31) - 16) * 0.75;
                double ext = (1 + (src >> 5)) * 0.5;
                mn[k] = lo - ext;
                mx[k] = lo + ext;
            }

            return make(mn, mx);
        }

        for (int pos = 1; pos + 3 < data.Length; pos += 4)
        {
            byte op = data[pos], a = data[pos + 1], b = data[pos + 2], c = data[pos + 3];
            switch (op % 12)
            {
                case 0:
                case 1:
                case 2:
                {
                    BodyType t = (BodyType)(a % 3);
                    int user = b;
                    int key = bp.CreateProxy(t, Box(a, b, c), user);
                    infos.Add(new Info { Key = key, Type = t, User = user });
                    moved.Add(key);
                    break;
                }

                case 3:
                    if (infos.Count > 0)
                    {
                        int i = a % infos.Count;
                        bp.DestroyProxy(infos[i].Key);
                        moved.Remove(infos[i].Key);
                        infos[i] = infos[^1];
                        infos.RemoveAt(infos.Count - 1);
                    }

                    break;
                case 4:
                case 5:
                    if (infos.Count > 0)
                    {
                        int i = a % infos.Count;
                        for (int k = 0; k < axes; k++) { disp[k] = ((k == 0 ? b : c) - 128) * 0.05; }
                        if (bp.MoveProxy(infos[i].Key, Box(b, c, a), disp)) { moved.Add(infos[i].Key); }
                    }

                    break;
                case 6:
                    if (infos.Count > 0)
                    {
                        int i = a % infos.Count;
                        bp.Touch(infos[i].Key);
                        moved.Add(infos[i].Key);
                    }

                    break;
                case 7:
                {
                    ProxyPair[] got = (a & 1) == 0 ? bp.UpdatePairs(new AcceptAllPairs()).ToArray() : bp.UpdatePairs(new AcceptAllPairs(), exec).ToArray();
                    var expect = new List<ProxyPair>();
                    for (int i = 0; i < infos.Count; i++)
                    {
                        for (int j = i + 1; j < infos.Count; j++)
                        {
                            Info x = infos[i], y = infos[j];
                            if (x.Type != BodyType.Dynamic && y.Type != BodyType.Dynamic) { continue; }
                            if (!moved.Contains(x.Key) && !moved.Contains(y.Key)) { continue; }
                            if (!TBox.Overlaps(bp.GetFatBounds(x.Key), bp.GetFatBounds(y.Key))) { continue; }
                            bool xFirst = x.User < y.User || (x.User == y.User && x.Key < y.Key);
                            expect.Add(xFirst ? new ProxyPair(x.Key, y.Key, x.User, y.User) : new ProxyPair(y.Key, x.Key, y.User, x.User));
                        }
                    }

                    expect.Sort((p, q) =>
                    {
                        int r = p.UserA.CompareTo(q.UserA);
                        if (r == 0) { r = p.UserB.CompareTo(q.UserB); }
                        if (r == 0) { r = p.ProxyA.CompareTo(q.ProxyA); }
                        return r == 0 ? p.ProxyB.CompareTo(q.ProxyB) : r;
                    });
                    if (!expect.ToArray().AsSpan().SequenceEqual(got)) { throw new InvalidOperationException("pairs differ from brute force"); }
                    moved.Clear();
                    break;
                }

                case 8:
                {
                    TBox q = Box(a, b, c);
                    var hits = new List<int>();
                    var cb = new Collect { Keys = hits };
                    bp.QueryAabb(q, ref cb);
                    int expected = 0;
                    foreach (Info x in infos) { if (TBox.Overlaps(bp.GetFatBounds(x.Key), q)) { expected++; } }
                    if (expected != hits.Count) { throw new InvalidOperationException("overlap query differs from brute force"); }
                    break;
                }

                case 9:
                {
                    var o = new double[axes];
                    var d = new double[axes];
                    for (int k = 0; k < axes; k++) { o[k] = (((k == 0 ? a : b) & 31) - 16) * 0.75; d[k] = ((k == 0 ? b : c) - 128) * 0.2; }
                    IRay<TBox> r = ray(o, d);
                    var hits = new List<int>();
                    var cb = new RayCollect { Keys = hits };
                    CastDyn(bp, r, ref cb);
                    int expected = 0;
                    foreach (Info x in infos) { if (r.Hit(bp.GetFatBounds(x.Key), 1.0, out _)) { expected++; } }
                    if (expected != hits.Count) { throw new InvalidOperationException("ray cast differs from brute force"); }
                    break;
                }

                case 10:
                    bp.Rebuild((BodyTypeMask)(a & 7));
                    break;
                default:
                {
                    var w = StateWriter.ToBuffer();
                    bp.Write(w);
                    byte[] bytes = w.ToArray();
                    var copy = new Broadphase<TBox>(bp.Options, 5);
                    copy.Read(new StateReader(bytes));
                    var w2 = StateWriter.ToBuffer();
                    copy.Write(w2);
                    if (!w2.ToArray().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("snapshot round trip changed the bytes"); }
                    if ((a & 1) == 0)
                    {
                        // Continue on the restored copy: it holds the same proxies and move buffer.
                        bp = copy;
                    }

                    break;
                }
            }

            bp.Validate();
        }
    }

    // The ray is held as an interface in this target; box it into a small struct so the generic cast can be used.
    private readonly struct Boxed<TBox> : IRay<TBox> where TBox : struct, IBox<TBox>
    {
        private readonly IRay<TBox> _inner;

        public Boxed(IRay<TBox> inner) => _inner = inner;

        public bool Hit(in TBox box, double maxFraction, out double enter) => _inner.Hit(box, maxFraction, out enter);
    }

    private static void CastDyn<TBox>(Broadphase<TBox> bp, IRay<TBox> ray, ref RayCollect cb) where TBox : struct, IBox<TBox>
    {
        bp.RayCast(new Boxed<TBox>(ray), 1.0, ref cb);
    }
}
