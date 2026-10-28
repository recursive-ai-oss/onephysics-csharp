using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

// Fuzz target: the input bytes are a program of compound operations (add, remove, move, scale, queries, optimise, static build, manifolds) with hostile numbers
// (NaN, infinities, huge and tiny values, stale handles). The documented argument errors are fine; any other exception, a broken tree invariant or a query that disagrees
// with iterating every child is a finding.
internal static class FuzzCompound3D
{
    private ref struct Reader
    {
        private readonly byte[] _d;
        private int _p;

        public Reader(byte[] d) { _d = d; _p = 0; }

        public bool More => _p < _d.Length;

        public byte B() => _p < _d.Length ? _d[_p++] : (byte)0;

        // A float: a quarter of the time raw bits (mostly hostile), otherwise a modest value.
        public float F(float scale = 4f)
        {
            byte b = B();
            if (b < 10)
            {
                return BitConverter.Int32BitsToSingle(B() | (B() << 8) | (B() << 16) | (B() << 24));
            }

            if (b < 18)
            {
                return b < 13 ? 0f : (b < 15 ? float.MaxValue : -1e-30f);
            }

            return (B() - 128) / 128f * scale;
        }

        // A positive float, hostile about one time in 25.
        public float P(float scale = 2f)
        {
            byte b = B();
            return b < 10 ? F(scale) : (B() + 1) / 256f * scale;
        }

        public Vector3F PV(float scale = 2f) => new(P(scale), P(scale), P(scale));

        public Vector3F V(float scale = 4f) => new(F(scale), F(scale), F(scale));

        public QuaternionF Q()
        {
            byte b = B();
            if (b < 8)
            {
                return new QuaternionF(F(), F(), F(), F());
            }

            return new QuaternionF(F(1f), F(1f), F(1f), F(1f)).Normalized();
        }
    }

    private static readonly ConvexHull3D[] Hulls = MakeHulls();

    private static ConvexHull3D[] MakeHulls()
    {
        var rng = new Random(5);
        var hulls = new ConvexHull3D[4];
        for (int i = 0; i < hulls.Length; i++)
        {
            var p = new Vector3F[12 + (i * 5)];
            for (int k = 0; k < p.Length; k++)
            {
                p[k] = new Vector3F((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f);
            }

            hulls[i] = ConvexHull3D.Build(p);
        }

        return hulls;
    }

    private static ConvexShape3D Shape(ref Reader r)
    {
        byte kind = r.B();
        ConvexShape3D s;
        switch (kind % 7)
        {
            case 0: s = ConvexShape3D.Sphere(r.P(2f)); break;
            case 1: s = ConvexShape3D.Box(r.PV(2f), r.P(0.3f)); break;
            case 2: s = ConvexShape3D.Capsule(r.P(2f), r.P(1f)); break;
            case 3: s = ConvexShape3D.Cylinder(r.P(2f), r.P(2f), r.P(0.3f)); break;
            case 4: s = ConvexShape3D.TaperedCapsule(r.P(2f), r.P(1f), r.P(1f)); break;
            case 5: s = ConvexShape3D.FromHull(Hulls[r.B() % Hulls.Length], r.P(0.3f)); break;
            default: s = ConvexShape3D.Point; break;
        }

        if ((kind & 0x80) != 0)
        {
            s = ScaledShape3D.Create(s, r.PV(2f));
        }

        return s;
    }

    private static CompoundChild3D Child(ref Reader r)
    {
        ConvexShape3D s = Shape(ref r);
        return new CompoundChild3D(s, new RigidTransformF(r.Q(), r.V(4f)), r.B());
    }

    private static CompoundChildHandle3D Handle(MutableCompound3D m, ref Reader r, List<CompoundChildHandle3D> issued)
    {
        byte b = r.B();
        if (b < 200 && issued.Count > 0)
        {
            return issued[b % issued.Count]; // may be stale
        }

        return m.SlotCount > 0 ? m.GetHandleOrDefault(b % m.SlotCount) : default;
    }

    private static CompoundChildHandle3D GetHandleOrDefault(this MutableCompound3D m, int slot) => m.IsLive(slot) ? m.GetHandle(slot) : default;

    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        var m = new MutableCompound3D();
        var issued = new List<CompoundChildHandle3D>();
        var ids = new List<int>();
        var gen = new ManifoldGenerator3D();
        var contacts = new List<ChildContact3D>();
        int ops = 0;
        while (r.More && ops++ < 64)
        {
            byte op = r.B();
            try
            {
                switch (op % 11)
                {
                    case 0:
                    case 1:
                        issued.Add(m.Add(Child(ref r)));
                        m.Validate();
                        break;
                    case 2:
                        m.Remove(Handle(m, ref r, issued));
                        m.Validate();
                        break;
                    case 3:
                        {
                            CompoundChildHandle3D h = Handle(m, ref r, issued);
                            m.Move(h, new RigidTransformF(r.Q(), r.V(4f)));
                            m.Validate();
                            break;
                        }

                    case 4:
                        {
                            Vector3F o = r.V(8f), d = r.V(8f);
                            float maxF = r.F(3f);
                            bool got = m.RayCast(o, d, maxF, out CompoundRayHit3D hit);
                            bool want = false;
                            float best = float.MaxValue;
                            for (int i = 0; i < m.SlotCount; i++)
                            {
                                if (m.IsLive(i))
                                {
                                    PlacedShape3D p = m.GetChild(i);
                                    if (Gjk3D.RayCast(p.Shape, p.Local, o, d, maxF, out ConvexCastHit3D h))
                                    {
                                        want = true;
                                        best = Math.Min(best, h.Fraction);
                                    }
                                }
                            }

                            if (got != want || (got && hit.Fraction != best))
                            {
                                throw new InvalidOperationException($"ray cast disagrees with brute force (compound {got}, children {want}): origin {o}, direction {d}, maxFraction {maxF}, {m.Count} children");
                            }

                            break;
                        }

                    case 5:
                        {
                            ConvexShape3D s = Shape(ref r);
                            var pose = new RigidTransformF(r.Q(), r.V(6f));
                            m.ShapeCast(s, pose, r.V(8f), out _);
                            CompoundPairs3D.FindChildren(m, s, pose, r.F(0.1f), ids);
                            break;
                        }

                    case 6:
                        {
                            Vector3F lo = r.V(6f), hi = r.V(6f);
                            m.OverlapBounds(lo, hi, ids);
                            var want = new List<int>();
                            for (int i = 0; i < m.SlotCount; i++)
                            {
                                if (!m.IsLive(i))
                                {
                                    continue;
                                }

                                m.GetChildBounds(i, out Vector3F a, out Vector3F b);
                                if (a.X <= hi.X && lo.X <= b.X && a.Y <= hi.Y && lo.Y <= b.Y && a.Z <= hi.Z && lo.Z <= b.Z)
                                {
                                    want.Add(i);
                                }
                            }

                            if (want.Count != ids.Count)
                            {
                                throw new InvalidOperationException("overlap disagrees with brute force");
                            }

                            break;
                        }

                    case 7:
                        m.Optimize();
                        m.Validate();
                        if (m.Count > 0)
                        {
                            m.GetMassProperties();
                        }

                        break;
                    case 8:
                        {
                            var kids = new CompoundChild3D[1 + (r.B() % 5)];
                            for (int i = 0; i < kids.Length; i++)
                            {
                                kids[i] = Child(ref r);
                            }

                            Compound3D c = Compound3D.Create(kids);
                            c.Validate();
                            c.GetMassProperties();
                            break;
                        }

                    case 9:
                        gen.CollideCompoundConvex(m, Shape(ref r), new RigidTransformF(r.Q(), r.V(3f)), contacts, r.F(0.1f));
                        break;
                    default:
                        {
                            var other = Compound3D.Create(new[] { Child(ref r) });
                            gen.CollideCompoundCompound(m, other, new RigidTransformF(r.Q(), r.V(3f)), contacts, r.F(0.1f));
                            break;
                        }
                }
            }
            catch (ArgumentException)
            {
                // The documented errors for hostile input: shape constructors, scaling, transforms, handles and queries.
            }
        }
    }
}
