using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;

// Fuzz target "world2d-services": bytes are read as a short program over a small 2D world: queries with hostile numbers (NaN, infinity, huge, zero directions, negative sizes),
// filter and layer edits at any time (including out-of-range layers), sensors, struct and delegate contact filters, steps, destroying bodies and shapes (stale handles included), debug geometry,
// and snapshot round trips. Only argument errors and typed physics errors are allowed; an index or null error, a result that is not finite or not in the documented order, or a restored world that
// differs from the original is a finding.
internal static class Fuzz2DServices
{
    private static long s_queries, s_hits, s_refused, s_steps, s_edits, s_triggers;

    static Fuzz2DServices()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("world2d-services: queries " + s_queries + " (hits " + s_hits + "), refused " + s_refused + ", steps " + s_steps + ", filter edits " + s_edits + ", trigger callbacks " + s_triggers);
    }

    private readonly struct ByIndex : IContactFilter
    {
        private readonly int _m;

        public ByIndex(int m) => _m = m;

        public bool ShouldCollide(Shape a, Shape b) => ((a.RawHandle.Index ^ b.RawHandle.Index) % _m) != 0;
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;

        public Reader(byte[] d) { _d = d; }

        public bool More => _i < _d.Length;

        public byte Byte() => _i < _d.Length ? _d[_i++] : (byte)0;

        public float Float()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return float.NaN;
                case 251: return float.PositiveInfinity;
                case 252: return float.NegativeInfinity;
                case 253: return 3e7f;
                case 254: return 1e-9f;
                case 255: return -0f;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 256f;
        }

        public double Double()
        {
            byte b = Byte();
            switch (b)
            {
                case 250: return double.NaN;
                case 251: return double.PositiveInfinity;
                case 252: return double.NegativeInfinity;
                case 253: return 5e6;
                case 254: return 1e-300;
                case 255: return -0.0;
            }

            int v = (short)(b | (Byte() << 8));
            return v / 512.0;
        }

        public Vector2D V2D() => new(Double(), Double());

        public Vector2F V2F() => new(Float(), Float());
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void Check(in QueryHit2D h)
    {
        if (!h.IsHit) { throw new InvalidOperationException("a reported hit is not marked as a hit"); }
        if (!double.IsFinite(h.Point.X) || !double.IsFinite(h.Point.Y) || !float.IsFinite(h.Normal.X) || !float.IsFinite(h.Normal.Y) || !float.IsFinite(h.Fraction) || !float.IsFinite(h.Distance) || h.Distance < 0f)
        {
            throw new InvalidOperationException("a query result is not finite or negative: " + h.Point + " " + h.Normal + " " + h.Fraction + " " + h.Distance);
        }
    }

    private static void CheckSorted(QueryHit2D[] hits, int n)
    {
        for (int i = 0; i < n; i++)
        {
            Check(in hits[i]);
            if (i > 0 && hits[i - 1].Distance > hits[i].Distance) { throw new InvalidOperationException("results are not ordered by distance"); }
        }
    }

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = 13, Events = EventMask.All, Substeps = 1 + (r.Byte() & 3) });
        var bodies = new Body[12];
        int made = 0;
        w.AddBox(new Vector2F(40f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        for (int i = 0; i < 8; i++)
        {
            var b = w.CreateBody(i % 4 == 3 ? BodyType.Kinematic : BodyType.Dynamic, new Vector2D(-3 + i * 0.7, 0.5 + (i % 3) * 0.5));
            var def = new ShapeDef { Layer = r.Byte() % 4, IsSensor = (r.Byte() & 7) == 0, CategoryBits = (uint)(1 << (r.Byte() % 3)), MaskBits = r.Byte() < 40 ? 3u : uint.MaxValue };
            if (i % 2 == 0) { w.CreateCircle(b, 0.4f, default, def); } else { w.CreateBox(b, 0.4f, 0.3f, default, 0f, 0f, def); }
            b.OnTriggerEnter += _ => s_triggers++;
            b.OnTriggerExit += _ => s_triggers++;
            bodies[made++] = b;
        }

        var buffer = new QueryHit2D[4];
        var geometry = new DebugGeometry();
        byte[]? snap = null;
        ulong snapHash = 0;
        int ops = 0;
        while (r.More && ops++ < 48)
        {
            try
            {
                switch (r.Byte() % 20)
                {
                    case 0: { s_queries++; if (w.Raycast(r.V2D(), r.V2F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 1: { s_queries++; int n = w.RaycastAll(r.V2D(), r.V2F(), buffer, r.Float(), Filter(r)); s_hits += n; CheckSorted(buffer, n); break; }
                    case 2: { s_queries++; if (w.CircleCast(r.V2D(), r.Float(), r.V2F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 3: { s_queries++; if (w.BoxCast(r.V2D(), r.V2F(), r.Float(), r.V2F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 4: { s_queries++; if (w.CapsuleCast(r.V2D(), r.Float(), r.Float(), r.Float(), r.V2F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 5: { s_queries++; int n = w.OverlapCircleAll(r.V2D(), r.Float(), buffer, Filter(r)); s_hits += n; CheckSorted(buffer, n); break; }
                    case 6: { s_queries++; if (w.OverlapBox(r.V2D(), r.V2F(), r.Float(), out var h, Filter(r))) { s_hits++; Check(in h); } break; }
                    case 7: { s_queries++; if (w.OverlapPoint(r.V2D(), out var h, Filter(r))) { s_hits++; Check(in h); } break; }
                    case 8: { s_queries++; if (w.ClosestPoint(r.V2D(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 9: { s_queries++; int n = w.ClosestPointAll(r.V2D(), buffer, r.Float(), Filter(r)); s_hits += n; CheckSorted(buffer, n); break; }
                    case 10: { s_edits++; w.SetLayerCollision(r.Byte() % 36 - 2, r.Byte() % 36 - 2, (r.Byte() & 1) != 0); break; }
                    case 11: { s_edits++; var b = bodies[r.Byte() % made]; b.Layer = r.Byte() % 34 - 1; break; }
                    case 12: { s_edits++; var s = bodies[r.Byte() % made].GetShapes(); if (s.Length > 0) { s[0].MaskBits = r.Byte(); s[0].CategoryBits = r.Byte(); s[0].GroupIndex = (sbyte)r.Byte(); } break; }
                    case 13: { s_edits++; if ((r.Byte() & 1) == 0) { w.SetContactFilter(new ByIndex(1 + r.Byte() % 5)); } else { w.ClearContactFilter(); } break; }
                    case 14: { s_edits++; int m = r.Byte() % 4; w.Options.PairFilter = m == 0 ? null : (a, b) => ((a.RawHandle.Index + b.RawHandle.Index) % (m + 1)) != 0; break; }
                    case 15: { for (int k = 1 + r.Byte() % 6; k > 0; k--) { w.Step(1f / 60f); s_steps++; } break; }
                    case 16: { var b = bodies[r.Byte() % made]; if ((r.Byte() & 3) == 0) { b.Destroy(); } else { b.Wake(); } break; }
                    case 17: { w.GetDebugGeometry((DebugContent)r.Byte(), geometry); break; }
                    case 18: { snap = w.Snapshot(); snapHash = w.ComputeHash(); break; }
                    default:
                        if (snap != null)
                        {
                            var w2 = new PhysicsWorld2D(new WorldOptions2D { WorldId = 13, Events = EventMask.All });
                            w2.Restore(snap);
                            if (!w2.Snapshot().AsSpan().SequenceEqual(snap) || w2.ComputeHash() != snapHash) { throw new InvalidOperationException("restored world differs from the snapshotted one"); }
                        }

                        break;
                }
            }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }
    }

    private static QueryFilter2D Filter(Reader r)
    {
        byte k = r.Byte();
        return new QueryFilter2D { LayerMask = k < 30 ? 0u : (k < 60 ? 5u : uint.MaxValue), IncludeSensors = (k & 1) != 0 };
    }
}
