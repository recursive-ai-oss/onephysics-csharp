using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The 2D engine's state and algorithms on top of <see cref="WorldCore"/>: per-body and per-shape data, the pair table, mass, the contact stage and the step stages. Internal: the public face is
/// <see cref="PhysicsWorld2D"/>. Everything saved in snapshots and covered by the world hash is written by <see cref="Write"/>; everything else is rebuilt every step.
/// </summary>
internal sealed partial class Engine2D : ISnapshotSection
{
    public const int Id = 0x32445747; // "GW2D"
    private const uint Version = 1;

    public readonly WorldCore Core;
    public readonly Broadphase2D Bp;
    public readonly WorldOptions2D Opt;

    /// <summary>The public world that owns this engine (used to hand out handles to callbacks).</summary>
    public PhysicsWorld2D? World;

    // Bodies (index = body slot).
    public float[] LocalCenter = new float[16];   // centre of mass in the body's frame, 2 per body
    public float[] Mass = new float[8];
    public float[] Inertia = new float[8];        // about the centre of mass
    public float[] MaxExtent = new float[8];      // farthest point of any shape from the centre of mass
    public float[] MinExtent = new float[8];      // smallest half-width of any shape; decides whether a body moves fast enough to need continuous collision
    public float[] LinearDamping = new float[8];
    public float[] AngularDamping = new float[8];
    public float[] GravityScale = new float[8];
    public float[] Force = new float[16];
    public float[] Torque = new float[8];
    public byte[] BodyFlags = new byte[8];        // bit 0: fixed rotation
    public int[] ShapeHead = new int[8];
    public int[] ShapeTail = new int[8];
    public float[] Dp = new float[16];            // displacement over the step (the last step's, between steps)

    // Transient per step.
    public double[] StartPos = new double[16];
    public float[] StartRot = new float[16];
    public float[] Dq = new float[16];            // accumulated rotation over the step as (cos, sin)
    public float[] Scratch = new float[ContactKernel2D.Stride * 8];

    // Shapes (index = shape slot).
    public Shape2D[] Geom = new Shape2D[8];
    public int[] Proxy = new int[8];
    public float[] Friction = new float[8];
    public float[] Restitution = new float[8];
    public float[] Rolling = new float[8];
    public float[] Density = new float[8];
    public uint[] Category = new uint[8];
    public uint[] Mask = new uint[8];
    public int[] Group = new int[8];
    public int[] NextShape = new int[8];

    // Pair table: one entry per pair of shapes whose broadphase boxes overlap, with the contact constraint while the shapes touch.
    public int[] PairA = new int[8];
    public int[] PairB = new int[8];
    public int[] PairCons = new int[8];
    public bool[] PairLive = new bool[8];
    public int PairHigh;
    public int[] PairFree = new int[8];
    public int PairFreeCount;
    public readonly Dictionary<long, int> PairMap = new();

    /// <summary>Lanes per kernel call: 8, 4 or 1. All give the same bits; the choice exists so tests can prove it.</summary>
    public int KernelWidth = 8;

    public Engine2D(WorldCore core, Broadphase2D bp, WorldOptions2D opt)
    {
        Core = core;
        Bp = bp;
        Opt = opt;
    }

    public int SectionId => Id;

    // ------------------------------------------------------------------ storage growth

    private static void Grow<T>(ref T[] a, int n)
    {
        if (a.Length < n) { Array.Resize(ref a, Math.Max(n, a.Length * 2)); }
    }

    public void EnsureBodies(int n)
    {
        Grow(ref LocalCenter, 2 * n); Grow(ref Mass, n); Grow(ref Inertia, n); Grow(ref MaxExtent, n); Grow(ref MinExtent, n);
        Grow(ref LinearDamping, n); Grow(ref AngularDamping, n); Grow(ref GravityScale, n); Grow(ref Force, 2 * n); Grow(ref Torque, n);
        Grow(ref BodyFlags, n); Grow(ref ShapeHead, n); Grow(ref ShapeTail, n); Grow(ref Dp, 2 * n);
        Grow(ref StartPos, 2 * n); Grow(ref StartRot, 2 * n); Grow(ref Dq, 2 * n);
    }

    public void EnsureShapes(int n)
    {
        Grow(ref Geom, n); Grow(ref Proxy, n); Grow(ref Friction, n); Grow(ref Restitution, n); Grow(ref Rolling, n); Grow(ref Density, n);
        Grow(ref Category, n); Grow(ref Mask, n); Grow(ref Group, n); Grow(ref NextShape, n);
    }

    // ------------------------------------------------------------------ bodies

    public BodyHandle CreateBody(in BodyDef d)
    {
        if (!double.IsFinite(d.Position.X) || !double.IsFinite(d.Position.Y)) { throw new ArgumentOutOfRangeException(nameof(d), "Body position must be finite."); }
        if (!float.IsFinite(d.Angle)) { throw new ArgumentOutOfRangeException(nameof(d), d.Angle, "Body angle must be finite."); }
        if (!float.IsFinite(d.AngularVelocity)) { throw new ArgumentOutOfRangeException(nameof(d), d.AngularVelocity, "Angular velocity must be finite."); }
        Nonnegative(d.LinearDamping, "LinearDamping");
        Nonnegative(d.AngularDamping, "AngularDamping");
        if (!float.IsFinite(d.GravityScale)) { throw new ArgumentOutOfRangeException(nameof(d), d.GravityScale, "GravityScale must be finite."); }
        var (s, c) = RecursiveAI.OneMath.Math64.SinCos(d.Angle);
        var flags = BodyBits.None;
        if (d.IsBullet) { flags |= BodyBits.Bullet; }
        if (!d.AllowSleep) { flags |= BodyBits.NoSleep; }
        var h = Core.CreateBody(d.Type, [d.Position.X, d.Position.Y], [(float)c, (float)s], [d.LinearVelocity.X, d.LinearVelocity.Y], [d.AngularVelocity], 1f, [1f], flags);
        int b = h.Index;
        EnsureBodies(b + 1);
        LocalCenter[2 * b] = 0f; LocalCenter[2 * b + 1] = 0f;
        Mass[b] = 0f; Inertia[b] = 0f; MaxExtent[b] = 0f; MinExtent[b] = float.MaxValue;
        LinearDamping[b] = d.LinearDamping; AngularDamping[b] = d.AngularDamping; GravityScale[b] = d.GravityScale;
        Force[2 * b] = 0f; Force[2 * b + 1] = 0f; Torque[b] = 0f;
        BodyFlags[b] = (byte)(d.FixedRotation ? 1 : 0);
        ShapeHead[b] = -1; ShapeTail[b] = -1;
        Dp[2 * b] = 0f; Dp[2 * b + 1] = 0f;
        if (d.Type == BodyType.Dynamic)
        {
            // Mass 1 and inertia 1 until shapes say otherwise.
            Mass[b] = 1f;
            Inertia[b] = 1f;
            Core.Bodies.InverseInertia[b] = d.FixedRotation ? 0f : 1f;
            if (d.StartAsleep) { Core.Sleep(h); }
        }

        return h;
    }

    private static void Nonnegative(float v, string name)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and at least 0."); }
    }

    public void DestroyBody(BodyHandle handle)
    {
        int b = Core.Resolve(handle);
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            DropShapePairs(s);
            Bp.DestroyProxy(Proxy[s]);
        }

        Core.DestroyBody(handle);
        ShapeHead[b] = -1; ShapeTail[b] = -1;
        Mass[b] = 0f; Inertia[b] = 0f;
    }

    // ------------------------------------------------------------------ shapes

    public ShapeHandle AddShape(BodyHandle body, in Shape2D geometry, in ShapeDef def)
    {
        Nonnegative(def.Density, "Density");
        Nonnegative(def.Friction, "Friction");
        Nonnegative(def.RollingResistance, "RollingResistance");
        if (!(def.Restitution >= 0f && def.Restitution <= 1f)) { throw new ArgumentOutOfRangeException(nameof(def), def.Restitution, "Restitution must be within 0..1."); }
        int b = Core.Resolve(body);
        var type = Core.Bodies.TypeOf(b);
        var tight = TightBox(b, in geometry);
        if (!tight.IsValid(out string reason)) { throw new ArgumentOutOfRangeException(nameof(geometry), "The shape's bounds are not valid: " + reason); }
        var sh = Core.CreateShape(body);
        int s = sh.Index;
        EnsureShapes(s + 1);
        Geom[s] = geometry;
        Friction[s] = def.Friction; Restitution[s] = def.Restitution; Rolling[s] = def.RollingResistance; Density[s] = def.Density;
        Category[s] = def.CategoryBits; Mask[s] = def.MaskBits; Group[s] = def.GroupIndex;
        NextShape[s] = -1;
        if (ShapeTail[b] >= 0) { NextShape[ShapeTail[b]] = s; } else { ShapeHead[b] = s; }
        ShapeTail[b] = s;
        Proxy[s] = Bp.CreateProxy(type, tight, s);
        if (type == BodyType.Dynamic) { UpdateMass(b); }
        return sh;
    }

    public void DestroyShape(ShapeHandle handle)
    {
        int s = Core.Resolve(handle);
        int b = Core.Shapes.Owner[s];
        DropShapePairs(s);
        Bp.DestroyProxy(Proxy[s]);
        // Unlink from the body's list.
        int prev = -1;
        for (int t = ShapeHead[b]; t >= 0; prev = t, t = NextShape[t])
        {
            if (t != s) { continue; }
            if (prev >= 0) { NextShape[prev] = NextShape[t]; } else { ShapeHead[b] = NextShape[t]; }
            if (ShapeTail[b] == t) { ShapeTail[b] = prev; }
            break;
        }

        Core.DestroyShape(handle);
        if (Core.Bodies.TypeOf(b) == BodyType.Dynamic) { UpdateMass(b); }
    }

    private readonly double[] _dp2 = new double[2];

    /// <summary>Fails before a step when a moving body's bounds are outside what the broadphase accepts, so a step never stops half way. Lowest slot first.</summary>
    public void CheckBounds()
    {
        if (FindOutOfBounds(false, out var h, out string message)) { throw new BodyOutOfBoundsException(h, message); }
    }

    /// <summary>Finds the first body (lowest slot) whose bounds, as the broadphase would store them, are not valid.</summary>
    public bool FindOutOfBounds(bool includeStatic, out BodyHandle handle, out string message)
    {
        var bodies = Core.Bodies;
        int nb = bodies.HighWater;
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b) || (!includeStatic && bodies.TypeOf(b) == BodyType.Static)) { continue; }
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                var box = TightBox(b, in Geom[s]);
                // The box the broadphase would store: margin plus the last step's displacement stretched by its default multiplier (2).
                _dp2[0] = Dp[2 * b]; _dp2[1] = Dp[2 * b + 1];
                var fat = box.Fatten(Opt.BroadphaseMargin, _dp2, 2.0);
                if (box.IsValid(out string reason) && fat.IsValid(out reason)) { continue; }
                handle = Core.HandleOfBody(b);
                var pos = bodies.PositionOf(b);
                var ci = System.Globalization.CultureInfo.InvariantCulture;
                message = "Body " + handle + " at (" + pos[0].ToString("R", ci) + ", " + pos[1].ToString("R", ci) + ") has left the supported range: " + reason + " Move it back, destroy it, or call ShiftOrigin.";
                return true;
            }
        }

        handle = default;
        message = "";
        return false;
    }

    /// <summary>The body's shape bounds in world space, tight (no margin).</summary>
    public Aabb2D TightBox(int b, in Shape2D shape)
    {
        var pos = Core.Bodies.PositionOf(b);
        var rot = Core.Bodies.RotationOf(b);
        var q = new Rot2(rot[0], rot[1]);
        var lc = new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]);
        var box = shape.ComputeAabb(new Transform2(-q.Rotate(lc), q));
        return new Aabb2D(pos[0] + (double)box.Lower.X, pos[1] + (double)box.Lower.Y, pos[0] + (double)box.Upper.X, pos[1] + (double)box.Upper.Y);
    }

    /// <summary>Recomputes mass, centre of mass, inertia and extents of a dynamic body from its shapes, moving the stored centre-of-mass position so the body origin stays put.</summary>
    public void UpdateMass(int b)
    {
        double m = 0, cx = 0, cy = 0;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (!(Density[s] > 0f)) { continue; }
            var md = Geom[s].ComputeMass();
            double dm = (double)Density[s] * md.Area;
            m += dm; cx += dm * md.CentroidX; cy += dm * md.CentroidY;
        }

        float oldX = LocalCenter[2 * b], oldY = LocalCenter[2 * b + 1];
        float newX = 0f, newY = 0f;
        double inertia = 0;
        if (m > 0)
        {
            newX = (float)(cx / m); newY = (float)(cy / m);
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                if (!(Density[s] > 0f)) { continue; }
                inertia += (double)Density[s] * Geom[s].ComputeMass().InertiaAbout(newX, newY);
            }
        }

        var bodies = Core.Bodies;
        float invMass, invI;
        if (m > 0 && float.IsFinite((float)m) && (float)m > 0f)
        {
            Mass[b] = (float)m;
            Inertia[b] = (float)inertia;
            invMass = (float)(1.0 / m);
            invI = inertia > 0 ? (float)(1.0 / inertia) : 0f;
        }
        else
        {
            newX = 0f; newY = 0f;
            Mass[b] = 1f; Inertia[b] = 1f;
            invMass = 1f; invI = 1f;
        }

        if ((BodyFlags[b] & 1) != 0) { invI = 0f; }
        bodies.InverseMass[b] = invMass;
        bodies.InverseInertia[b] = invI;

        // Keep the origin where it was: the centre of mass moves by the rotated change of the local centre.
        var rot = bodies.RotationOf(b);
        float dx = newX - oldX, dy = newY - oldY;
        var pos = bodies.PositionOf(b);
        pos[0] += (double)(rot[0] * dx - rot[1] * dy);
        pos[1] += (double)(rot[1] * dx + rot[0] * dy);
        LocalCenter[2 * b] = newX; LocalCenter[2 * b + 1] = newY;

        float maxExt = 0f, minExt = float.MaxValue;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            ref readonly var g = ref Geom[s];
            var cv = g.Vertices;
            float r = g.Radius;
            for (int i = 0; i < cv.Length; i++)
            {
                float ex = cv[i].X - newX, ey = cv[i].Y - newY;
                float e = float.Sqrt(ex * ex + ey * ey) + r;
                if (e > maxExt) { maxExt = e; }
            }

            float me = r;
            if (g.Kind == ShapeKind2D.Polygon)
            {
                var nrm = g.Normals;
                me = float.MaxValue;
                for (int i = 0; i < cv.Length; i++)
                {
                    float d = Vec2.Dot(nrm[i], cv[i] - g.Centroid);
                    if (d < me) { me = d; }
                }

                me += r;
            }

            if (me < minExt) { minExt = me; }
        }

        MaxExtent[b] = maxExt;
        MinExtent[b] = minExt;
    }

    // ------------------------------------------------------------------ pair table

    private static long PairKey(int a, int b) => ((long)a << 32) | (uint)b;

    private int AllocatePair(int a, int b)
    {
        int p;
        if (PairFreeCount > 0) { p = PairFree[--PairFreeCount]; }
        else
        {
            p = PairHigh++;
            Grow(ref PairA, PairHigh); Grow(ref PairB, PairHigh); Grow(ref PairCons, PairHigh); Grow(ref PairLive, PairHigh);
        }

        PairA[p] = a; PairB[p] = b; PairCons[p] = -1; PairLive[p] = true;
        PairMap.Add(PairKey(a, b), p);
        return p;
    }

    private void FreePair(int p)
    {
        PairMap.Remove(PairKey(PairA[p], PairB[p]));
        PairLive[p] = false;
        PairCons[p] = -1;
        Grow(ref PairFree, PairFreeCount + 1);
        PairFree[PairFreeCount++] = p;
    }

    /// <summary>Forgets every pair that involves the shape. The constraints go with the body or shape in the core; this only clears the table.</summary>
    private void DropShapePairs(int s)
    {
        for (int p = 0; p < PairHigh; p++)
        {
            if (PairLive[p] && (PairA[p] == s || PairB[p] == s)) { FreePair(p); }
        }
    }

    /// <summary>The live contact constraint of a pair of shapes, or -1.</summary>
    public int ContactOf(int shapeA, int shapeB)
    {
        if (shapeA > shapeB) { (shapeA, shapeB) = (shapeB, shapeA); }
        return PairMap.TryGetValue(PairKey(shapeA, shapeB), out int p) ? PairCons[p] : -1;
    }

    /// <summary>Category, mask and group rule (Catto's filter model): same non-zero group decides; otherwise both masks must include the other's category.</summary>
    public bool CanCollide(int sa, int sb)
    {
        if (Group[sa] == Group[sb] && Group[sa] != 0) { return Group[sa] > 0; }
        return (Mask[sa] & Category[sb]) != 0 && (Category[sa] & Mask[sb]) != 0;
    }

    // ------------------------------------------------------------------ origin shifting

    /// <summary>After the core shifted the origin: rebuild every proxy, since the broadphase keeps boxes in origin-relative coordinates. Shapes are visited in ascending slot order.</summary>
    public void RebuildProxies()
    {
        var shapes = Core.Shapes;
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s)) { continue; }
            int b = shapes.Owner[s];
            var type = Core.Bodies.TypeOf(b);
            Bp.DestroyProxy(Proxy[s]);
            Proxy[s] = Bp.CreateProxy(type, TightBox(b, in Geom[s]), s);
        }
    }

    // ------------------------------------------------------------------ snapshot section

    public void Write(StateWriter w)
    {
        uint version = SectionVersion();
        w.WriteUInt32(version);
        w.WriteSingle(Opt.ContactHertz); w.WriteSingle(Opt.ContactDampingRatio); w.WriteSingle(Opt.PushOutVelocity); w.WriteSingle(Opt.RestitutionThreshold);
        w.WriteSingle(Opt.MaxLinearSpeed); w.WriteSingle(Opt.MaxRotationPerSubstep); w.WriteBool(Opt.ContinuousEnabled); w.WriteSingle(Opt.ContactSlop);
        int nb = Core.Bodies.HighWater;
        w.WriteInt32(nb);
        w.WriteSingles(LocalCenter.AsSpan(0, 2 * nb));
        w.WriteSingles(Mass.AsSpan(0, nb)); w.WriteSingles(Inertia.AsSpan(0, nb));
        w.WriteSingles(MaxExtent.AsSpan(0, nb)); w.WriteSingles(MinExtent.AsSpan(0, nb));
        w.WriteSingles(LinearDamping.AsSpan(0, nb)); w.WriteSingles(AngularDamping.AsSpan(0, nb)); w.WriteSingles(GravityScale.AsSpan(0, nb));
        w.WriteSingles(Force.AsSpan(0, 2 * nb)); w.WriteSingles(Torque.AsSpan(0, nb));
        w.WriteBytes(BodyFlags.AsSpan(0, nb));
        w.WriteInt32s(ShapeHead.AsSpan(0, nb)); w.WriteInt32s(ShapeTail.AsSpan(0, nb));
        w.WriteSingles(Dp.AsSpan(0, 2 * nb));

        int ns = Core.Shapes.HighWater;
        w.WriteInt32(ns);
        for (int s = 0; s < ns; s++)
        {
            bool live = Core.Shapes.Handles.IsLive(s);
            w.WriteBool(live);
            if (!live) { continue; }
            WriteShape(w, in Geom[s]);
            w.WriteInt32(Proxy[s]);
            w.WriteSingle(Friction[s]); w.WriteSingle(Restitution[s]); w.WriteSingle(Rolling[s]); w.WriteSingle(Density[s]);
            w.WriteUInt32(Category[s]); w.WriteUInt32(Mask[s]); w.WriteInt32(Group[s]); w.WriteInt32(NextShape[s]);
        }

        w.WriteInt32(PairHigh);
        for (int p = 0; p < PairHigh; p++)
        {
            w.WriteBool(PairLive[p]);
            if (PairLive[p]) { w.WriteInt32(PairA[p]); w.WriteInt32(PairB[p]); w.WriteInt32(PairCons[p]); }
        }

        w.WriteInt32(PairFreeCount);
        w.WriteInt32s(PairFree.AsSpan(0, PairFreeCount));
        if (version == 2u) { WriteJoints(w); }
    }

    private static void WriteShape(StateWriter w, in Shape2D g)
    {
        w.WriteByte((byte)g._kind); w.WriteInt32(g._count); w.WriteSingle(g._radius); w.WriteByte(g._flags);
        w.WriteSingle(g._centroid.X); w.WriteSingle(g._centroid.Y);
        w.WriteSingle(g._ghost1.X); w.WriteSingle(g._ghost1.Y); w.WriteSingle(g._ghost2.X); w.WriteSingle(g._ghost2.Y);
        for (int i = 0; i < Constants2D.MaxPolygonVertices; i++)
        {
            w.WriteSingle(g._v[i].X); w.WriteSingle(g._v[i].Y); w.WriteSingle(g._n[i].X); w.WriteSingle(g._n[i].Y);
        }
    }

    private static Shape2D ReadShape(StateReader r)
    {
        var g = default(Shape2D);
        byte kind = r.ReadByte();
        if (kind < (byte)ShapeKind2D.Circle || kind > (byte)ShapeKind2D.ChainSegment) { throw r.Error("invalid 2D shape kind " + kind.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
        int count = r.ReadInt32();
        if (count < 1 || count > Constants2D.MaxPolygonVertices) { throw r.Error("invalid 2D shape point count"); }
        float radius = r.ReadSingle();
        if (!float.IsFinite(radius) || radius < 0f) { throw r.Error("invalid 2D shape radius"); }
        byte flags = r.ReadByte();
        g._kind = (ShapeKind2D)kind; g._count = count; g._radius = radius; g._flags = flags;
        g._centroid = ReadVec(r); g._ghost1 = ReadVec(r); g._ghost2 = ReadVec(r);
        for (int i = 0; i < Constants2D.MaxPolygonVertices; i++) { g._v[i] = ReadVec(r); g._n[i] = ReadVec(r); }
        return g;
    }

    private static Vec2 ReadVec(StateReader r)
    {
        float x = r.ReadSingle(), y = r.ReadSingle();
        if (!float.IsFinite(x) || !float.IsFinite(y)) { throw r.Error("non-finite coordinate in a 2D shape"); }
        return new Vec2(x, y);
    }

    public void Read(StateReader r)
    {
        uint v = r.ReadUInt32();
        if (v != Version && v != 2u) { throw r.Error("unsupported 2D engine section version " + v.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
        float hertz = r.ReadSingle(), zeta = r.ReadSingle(), push = r.ReadSingle(), thr = r.ReadSingle(), maxSpeed = r.ReadSingle(), maxRot = r.ReadSingle();
        bool continuous = r.ReadBool();
        float slop = r.ReadSingle();
        if (!float.IsFinite(hertz + zeta + push + thr + maxSpeed + maxRot + slop) || slop < 0f || hertz < 0f || zeta < 0f || push < 0f || thr < 0f || !(maxSpeed > 0f) || !(maxRot > 0f)) { throw r.Error("invalid 2D engine tunables"); }

        int nb = r.ReadCount(65, "2D bodies"); // 65 bytes of per-body arrays follow
        if (nb > 1 << 22) { throw r.Error("too many 2D bodies"); }
        var lc = new float[2 * nb]; var mass = new float[nb]; var inertia = new float[nb]; var maxE = new float[nb]; var minE = new float[nb];
        var ld = new float[nb]; var ad = new float[nb]; var gs = new float[nb]; var force = new float[2 * nb]; var torque = new float[nb];
        var bf = new byte[nb]; var head = new int[nb]; var tail = new int[nb]; var dp = new float[2 * nb];
        r.ReadSingles(lc, true, "local centres"); r.ReadSingles(mass, true, "masses"); r.ReadSingles(inertia, true, "inertias");
        r.ReadSingles(maxE, true, "extents"); r.ReadSingles(minE, false, "extents");
        r.ReadSingles(ld, true, "damping"); r.ReadSingles(ad, true, "damping"); r.ReadSingles(gs, true, "gravity scales");
        r.ReadSingles(force, true, "forces"); r.ReadSingles(torque, true, "torques");
        r.ReadBytes(bf); r.ReadInt32s(head); r.ReadInt32s(tail);
        r.ReadSingles(dp, true, "displacements");

        int ns = r.ReadCount(1, "2D shapes");
        if (ns > 1 << 22) { throw r.Error("too many 2D shapes"); }
        var geom = new Shape2D[ns]; var proxy = new int[ns]; var fr = new float[ns]; var re = new float[ns]; var ro = new float[ns]; var de = new float[ns];
        var cat = new uint[ns]; var mask = new uint[ns]; var grp = new int[ns]; var next = new int[ns]; var live = new bool[ns];
        for (int s = 0; s < ns; s++)
        {
            live[s] = r.ReadBool();
            if (!live[s]) { continue; }
            geom[s] = ReadShape(r);
            proxy[s] = r.ReadInt32();
            fr[s] = r.ReadSingle(); re[s] = r.ReadSingle(); ro[s] = r.ReadSingle(); de[s] = r.ReadSingle();
            cat[s] = r.ReadUInt32(); mask[s] = r.ReadUInt32(); grp[s] = r.ReadInt32(); next[s] = r.ReadInt32();
            if (!(fr[s] >= 0f && re[s] >= 0f && re[s] <= 1f && ro[s] >= 0f && de[s] >= 0f) || !float.IsFinite(fr[s] + ro[s] + de[s])) { throw r.Error("invalid shape material"); }
            if (next[s] < -1 || next[s] >= ns || !Bp.IsProxy(proxy[s])) { throw r.Error("invalid shape link or proxy"); }
        }

        for (int b = 0; b < nb; b++)
        {
            if (head[b] < -1 || head[b] >= ns || tail[b] < -1 || tail[b] >= ns) { throw r.Error("invalid body shape list"); }
        }

        int np = r.ReadCount(1, "pairs");
        var pa = new int[Math.Max(np, 1)]; var pb = new int[Math.Max(np, 1)]; var pc = new int[Math.Max(np, 1)]; var pl = new bool[Math.Max(np, 1)];
        for (int p = 0; p < np; p++)
        {
            pl[p] = r.ReadBool();
            if (!pl[p]) { continue; }
            pa[p] = r.ReadInt32(); pb[p] = r.ReadInt32(); pc[p] = r.ReadInt32();
            if (pa[p] < 0 || pb[p] <= pa[p] || pb[p] >= ns || !live[pa[p]] || !live[pb[p]]) { throw r.Error("invalid pair"); }
        }

        int nf = r.ReadCount(4, "free pairs");
        var free = new int[Math.Max(nf, 1)];
        r.ReadInt32s(free.AsSpan(0, nf));
        for (int i = 0; i < nf; i++)
        {
            if (free[i] < 0 || free[i] >= np || pl[free[i]]) { throw r.Error("invalid free pair"); }
        }

        JointData2D[] joints = new JointData2D[16];
        if (v == 2u) { ReadJoints(r, out joints); }

        // Commit.
        Opt.ContactHertz = hertz; Opt.ContactDampingRatio = zeta; Opt.PushOutVelocity = push; Opt.RestitutionThreshold = thr;
        Opt.MaxLinearSpeed = maxSpeed; Opt.MaxRotationPerSubstep = maxRot; Opt.ContinuousEnabled = continuous; Opt.ContactSlop = slop;
        LocalCenter = lc.Length > 0 ? lc : new float[2]; Mass = mass; Inertia = inertia; MaxExtent = maxE; MinExtent = minE;
        LinearDamping = ld; AngularDamping = ad; GravityScale = gs; Force = force; Torque = torque; BodyFlags = bf; ShapeHead = head; ShapeTail = tail; Dp = dp;
        EnsureBodies(Math.Max(nb, 1));
        Geom = geom; Proxy = proxy; Friction = fr; Restitution = re; Rolling = ro; Density = de; Category = cat; Mask = mask; Group = grp; NextShape = next;
        EnsureShapes(Math.Max(ns, 1));
        PairA = pa; PairB = pb; PairCons = pc; PairLive = pl; PairHigh = np; PairFree = free; PairFreeCount = nf;
        if (v == 2u) { Opt.JointHertz = _pendingJointHertz; Opt.JointDampingRatio = _pendingJointDamping; }
        Joints = joints;
        JointForce = new float[joints.Length]; JointTorque = new float[joints.Length];
        _needBind = true;
        PairMap.Clear();
        for (int p = 0; p < np; p++)
        {
            if (PairLive[p]) { PairMap[PairKey(PairA[p], PairB[p])] = p; }
        }
    }

    /// <summary>Checks the section against the core after a restore; false when the two disagree (a damaged snapshot that parsed).</summary>
    public bool IsConsistentWithCore()
    {
        var bodies = Core.Bodies;
        var shapes = Core.Shapes;
        if (ShapeHead.Length < bodies.HighWater || Geom.Length < shapes.HighWater) { return false; }
        var cons = Core.Constraints;
        if (!JointsConsistentWithCore()) { return false; }
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s)) { continue; }
            int owner = shapes.Owner[s];
            if (!bodies.IsLive(owner) || !Bp.IsProxy(Proxy[s]) || Bp.GetUserId(Proxy[s]) != s) { return false; }
        }

        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            int count = 0;
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                if (!shapes.Handles.IsLive(s) || shapes.Owner[s] != b || ++count > shapes.HighWater) { return false; }
            }
        }

        for (int p = 0; p < PairHigh; p++)
        {
            if (!PairLive[p]) { continue; }
            if (PairCons[p] >= 0 && (!cons.IsLive(PairCons[p]) || cons.ShapeA[PairCons[p]] != PairA[p] || cons.ShapeB[PairCons[p]] != PairB[p])) { return false; }
        }

        return true;
    }
}
