using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics3D;

// Fuzz target "world3d-services": bytes are read as a short program over a small 3D world: queries with hostile numbers (NaN, infinity, huge, zero directions, negative sizes),
// filter and layer edits at any time (including out-of-range layers), sensors, struct and delegate contact filters, steps, destroying bodies and shapes (stale handles included), debug geometry,
// and snapshot round trips. Only argument errors and typed physics errors are allowed; an index or null error, a result that is not finite or not in the documented order, or a restored world that
// differs from the original is a finding.
internal static class Fuzz3DServices
{
    private static long s_queries, s_hits, s_refused, s_steps, s_edits, s_triggers;

    static Fuzz3DServices()
    {
        AppDomain.CurrentDomain.ProcessExit += (_, _) => Console.WriteLine("world3d-services: queries " + s_queries + " (hits " + s_hits + "), refused " + s_refused + ", steps " + s_steps + ", filter edits " + s_edits + ", trigger callbacks " + s_triggers);
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

        public Vector3D V3D() => new(Double(), Double(), Double());

        public Vector3F V3F() => new(Float(), Float(), Float());
    }

    private static bool Allowed(Exception e) => e is ArgumentException || e is PhysicsException;

    private static void Check(in QueryHit3D h)
    {
        if (!h.IsHit) { throw new InvalidOperationException("a reported hit is not marked as a hit"); }
        if (!double.IsFinite(h.Point.X) || !double.IsFinite(h.Point.Y) || !double.IsFinite(h.Point.Z) || !float.IsFinite(h.Normal.Z) || !float.IsFinite(h.Normal.X) || !float.IsFinite(h.Normal.Y) || !float.IsFinite(h.Fraction) || !float.IsFinite(h.Distance) || h.Distance < 0f)
        {
            throw new InvalidOperationException("a query result is not finite or negative: " + h.Point + " " + h.Normal + " " + h.Fraction + " " + h.Distance);
        }
    }

    private static void CheckSorted(QueryHit3D[] hits, int n)
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
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 13, Events = EventMask.All, Substeps = 1 + (r.Byte() & 3) });
        var bodies = new Body[12];
        int made = 0;
        w.AddBox(new Vector3F(40f, 1f, 40f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        Vector3F[] verts = [new(0, 0, 0), new(2, 0.2f, 0), new(0, 0.1f, 2), new(2, 0, 2)];
        w.AddMesh(verts, [0, 2, 1, 1, 2, 3], new Vector3D(-1, 0.3, -1));
        for (int i = 0; i < 8; i++)
        {
            var b = w.CreateBody(i % 4 == 3 ? BodyType.Kinematic : BodyType.Dynamic, new Vector3D(-3 + i * 0.7, 0.5 + (i % 3) * 0.5, (i % 2) * 0.3));
            var def = new ShapeDef { Layer = r.Byte() % 4, IsSensor = (r.Byte() & 7) == 0, CategoryBits = (uint)(1 << (r.Byte() % 3)), MaskBits = r.Byte() < 40 ? 3u : uint.MaxValue };
            switch (i % 3)
            {
                case 0: w.CreateSphere(b, 0.4f, def); break;
                case 1: w.CreateBox(b, new Vector3F(0.4f, 0.3f, 0.35f), 0f, def); break;
                default: w.CreateCapsule(b, 0.3f, 0.25f, def); break;
            }

            b.OnTriggerEnter += _ => s_triggers++;
            b.OnTriggerExit += _ => s_triggers++;
            bodies[made++] = b;
        }

        var buffer = new QueryHit3D[4];
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
                    case 0: { s_queries++; if (w.Raycast(r.V3D(), r.V3F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 1: { s_queries++; int n = w.RaycastAll(r.V3D(), r.V3F(), buffer, r.Float(), Filter(r)); s_hits += n; CheckSorted(buffer, n); break; }
                    case 2: { s_queries++; if (w.SphereCast(r.V3D(), r.Float(), r.V3F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 3: { s_queries++; if (w.BoxCast(r.V3D(), r.V3F(), Rot(r), r.V3F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 4: { s_queries++; if (w.CapsuleCast(r.V3D(), r.Float(), r.Float(), Rot(r), r.V3F(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 5: { s_queries++; int n = w.OverlapSphereAll(r.V3D(), r.Float(), buffer, Filter(r)); s_hits += n; for (int k = 0; k < n; k++) { Check(in buffer[k]); } break; }
                    case 6: { s_queries++; if (w.OverlapBox(r.V3D(), r.V3F(), Rot(r), out var h, Filter(r))) { s_hits++; Check(in h); } break; }
                    case 7: { s_queries++; if (w.OverlapPoint(r.V3D(), out var h, Filter(r))) { s_hits++; Check(in h); } break; }
                    case 8: { s_queries++; if (w.ClosestPoint(r.V3D(), out var h, r.Float(), Filter(r))) { s_hits++; Check(in h); } break; }
                    case 9: { s_queries++; int n = w.ClosestPointAll(r.V3D(), buffer, r.Float(), Filter(r)); s_hits += n; for (int k = 0; k < n; k++) { Check(in buffer[k]); } break; }
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
                            var w2 = new PhysicsWorld3D(new WorldOptions3D { WorldId = 13, Events = EventMask.All });
                            w2.Restore(snap);
                            if (!w2.Snapshot().AsSpan().SequenceEqual(snap) || w2.ComputeHash() != snapHash) { throw new InvalidOperationException("restored world differs from the snapshotted one"); }
                        }

                        break;
                }
            }
            catch (Exception e) when (Allowed(e)) { s_refused++; }
        }
    }

    private static QuaternionF Rot(Reader r) => (r.Byte() & 3) == 0 ? new QuaternionF(r.Float(), r.Float(), r.Float(), r.Float()) : QuaternionF.Identity;

    private static QueryFilter3D Filter(Reader r)
    {
        byte k = r.Byte();
        return new QueryFilter3D { LayerMask = k < 30 ? 0u : (k < 60 ? 5u : uint.MaxValue), IncludeSensors = (k & 1) != 0 };
    }
}
