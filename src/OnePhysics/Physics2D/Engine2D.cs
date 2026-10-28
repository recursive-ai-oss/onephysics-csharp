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

    /// <summary>Solver state per body during a step, <see cref="BS2D.Stride"/> floats at row <c>body + 1</c> (row 0 is a zero row that unused lanes read): velocity and the motion accumulated over the step. Filled from the core at the start of the solve and written back by the finalize stage.</summary>
    public float[] BodyState = new float[BS2D.Stride * 9];

    /// <summary>Contact constraint data of the step in groups of eight lanes (<see cref="W2D"/>), rebuilt every step.</summary>
    public float[] Wide = new float[W2D.GroupFloats];
    public int[] WideIdx = new int[W2D.IdxPerGroup];
    public int[] WideCons = new int[W2D.Lanes];

    /// <summary>Per group: whether any contact in it has a positive restitution, so the restitution pass can skip a group that cannot change anything.</summary>
    public byte[] GroupRestitution = new byte[1];

    /// <summary>Per group: whether any contact in it has rolling resistance or a stored rolling impulse; without either the rolling step of the solver changes nothing.</summary>
    public byte[] GroupRolling = new byte[1];
    /// <summary>Per group: 1 when no contact in it has a second manifold point, else 2 (the kernel then skips the second point).</summary>
    public byte[] GroupPoints = new byte[1];

    /// <summary>Per constraint: its group times eight plus its lane, for the contacts of the current step.</summary>
    public int[] SlotOf = new int[8];
    /// <summary>Per step, on the first constraint of each batch: 1 when the batch is contacts only and 2 when it is wide revolute joints only (neither is the serial overflow set), so the wide kernels solve it directly.</summary>
    public byte[] BatchSimple = new byte[8];

    /// <summary>Plain revolute joints of the step in groups of eight lanes (<see cref="J2D"/>), their gather and scatter offsets (as <see cref="WideIdx"/>) and, per constraint, group times eight plus lane, or -1 for a joint solved by the scalar code.</summary>
    public float[] JWide = new float[J2D.GroupFloats];
    public int[] JIdx = new int[W2D.IdxPerGroup];
    public int[] JSlotOf = new int[8];

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
    // One record per pair so the narrowphase touches one or two cache lines per pair instead of one per field.
    public Pair2D[] Pairs = new Pair2D[8];
    public int PairHigh;
    public int[] PairFree = new int[8];
    public int PairFreeCount;
    // Pairs with a sensor shape take their slots from their own free stack, so that contact pairs get the same slots relative to each other whether or not a sensor is in the world
    // (the order of slots is the order contacts are added and colored). An overlapping sensor pair has Cons = SensorOverlap; it owns no constraint.
    public int[] SensorFree = new int[8];
    public int SensorFreeCount;
    public const int SensorOverlap = -2;
    public readonly PairTable PairMap = new();

    /// <summary>Lanes per kernel call: 8, 4 or 1. All give the same bits; the choice exists so tests can prove it.</summary>
    public int KernelWidth = KernelWidthChoice.Default;

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
        Grow(ref StartPos, 2 * n); Grow(ref StartRot, 2 * n);
    }

    public void EnsureShapes(int n)
    {
        Grow(ref Geom, n); Grow(ref Proxy, n); Grow(ref Friction, n); Grow(ref Restitution, n); Grow(ref Rolling, n); Grow(ref Density, n);
        Grow(ref Category, n); Grow(ref Mask, n); Grow(ref Group, n); Grow(ref NextShape, n);
        EnsureServiceShapes(n);
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

    private static void Coefficient(float v, string name)
    {
        if (!(v >= 0f && v <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(name, v, name + " must be within 0..1e15."); }
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

        DropCharacterOfBody(b);
        Core.DestroyBody(handle);
        ShapeHead[b] = -1; ShapeTail[b] = -1;
        Mass[b] = 0f; Inertia[b] = 0f;
        Dp[2 * b] = 0f; Dp[2 * b + 1] = 0f;
        ReconcileJoints();
    }

    // ------------------------------------------------------------------ shapes

    public ShapeHandle AddShape(BodyHandle body, in Shape2D geometry, in ShapeDef def)
    {
        Nonnegative(def.Density, "Density");
        Coefficient(def.Friction, "Friction");
        Coefficient(def.RollingResistance, "RollingResistance");
        CheckLayer(def.Layer, "Layer");
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
        ShapeFlags[s] = def.IsSensor ? FlagSensor : (byte)0; ShapeLayer[s] = (byte)def.Layer;
        NextShape[s] = -1;
        if (ShapeTail[b] >= 0) { NextShape[ShapeTail[b]] = s; } else { ShapeHead[b] = s; }
        ShapeTail[b] = s;
        Proxy[s] = Bp.CreateProxy(type, tight, s);
        if (type == BodyType.Dynamic)
        {
            try { UpdateMass(b, strict: true); }
            catch (ArgumentOutOfRangeException)
            {
                // The body keeps its previous mass properties (strict mode writes nothing before it throws); take the new shape back out.
                DestroyShape(sh);
                throw;
            }
        }

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

    private const double SafeBound = 524288.0;

    private bool VelocityFinite(int b)
    {
        var v = Core.Bodies.LinearVelocityOf(b);
        var w = Core.Bodies.AngularVelocityOf(b);
        var q = Core.Bodies.RotationOf(b);
        var p = Core.Bodies.PositionOf(b);
        return float.IsFinite(v[0] + v[1] + w[0] + q[0] + q[1]) && double.IsFinite(p[0] + p[1]);
    }

    /// <summary>Finds the first body (lowest slot) whose bounds, as the broadphase would store them, are not valid.</summary>
    public bool FindOutOfBounds(bool includeStatic, out BodyHandle handle, out string message)
    {
        var bodies = Core.Bodies;
        int nb = bodies.HighWater;
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b) || (!includeStatic && bodies.TypeOf(b) == BodyType.Static)) { continue; }

            // A body whose velocity went non-finite (an extreme joint or contact error in the previous step) would turn its position to NaN in this one; refuse before anything changes.
            if (bodies.TypeOf(b) == BodyType.Dynamic && !VelocityFinite(b))
            {
                handle = Core.HandleOfBody(b);
                var vp = bodies.PositionOf(b);
                var vci = System.Globalization.CultureInfo.InvariantCulture;
                message = "Body " + handle + " at (" + vp[0].ToString("R", vci) + ", " + vp[1].ToString("R", vci) + ") has a non-finite position, velocity or rotation, so the simulation diverged. Destroy it or reset its velocity.";
                return true;
            }

            // A dynamic body well inside the range whose shapes and motion are small cannot have an invalid box: its fat box reaches at most 5 * 2^19 < the limit of 2^22.
            // (Extents are measured from the centre of mass and include the radius; the box is stretched by at most twice the last displacement and widened by the margin.)
            // Anything not obviously inside, NaN included, takes the exact test below.
            // A restore (includeStatic) never takes the shortcut: the stored extents come from untrusted bytes.
            if (!includeStatic && bodies.TypeOf(b) == BodyType.Dynamic)
            {
                var p = bodies.PositionOf(b);
                if (Math.Abs(p[0]) < SafeBound && Math.Abs(p[1]) < SafeBound && MaxExtent[b] < SafeBound && Math.Abs(Dp[2 * b]) < SafeBound && Math.Abs(Dp[2 * b + 1]) < SafeBound && Opt.BroadphaseMargin < SafeBound) { continue; }
            }

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

    /// <summary>Farthest point of any shape of the body from the given centre of mass, radius included.</summary>
    private float ComputeMaxExtent(int b, float cx, float cy)
    {
        float maxExt = 0f;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            ref readonly var g = ref Geom[s];
            var cv = g.Vertices;
            float r = g.Radius;
            for (int i = 0; i < cv.Length; i++)
            {
                float ex = cv[i].X - cx, ey = cv[i].Y - cy;
                float e = float.Sqrt(ex * ex + ey * ey) + r;
                if (e > maxExt) { maxExt = e; }
            }
        }

        return maxExt;
    }

    /// <summary>Smallest mass or (non-zero) inertia a dynamic body may have; the inverse is then at most 1e18.</summary>
    public const double MinMassProperty = 1e-18;

    /// <summary>Largest mass or inertia a dynamic body may have.</summary>
    public const double MaxMassProperty = 1e30;

    /// <summary>Recomputes mass, centre of mass, inertia and extents of a dynamic body from its shapes, moving the stored centre-of-mass position so the body origin stays put.</summary>
    public void UpdateMass(int b, bool strict = false)
    {
        double m = 0, cx = 0, cy = 0;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (!(Density[s] > 0f) || IsSensor(s)) { continue; }
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
                if (!(Density[s] > 0f) || IsSensor(s)) { continue; }
                inertia += (double)Density[s] * Geom[s].ComputeMass().InertiaAbout(newX, newY);
            }
        }

        // Mass limits (documented on ShapeDef.Density): mass and inertia each within [MinMassProperty, MaxMassProperty], so every inverse is finite and
        // the solver's products of inverse inertia and squared lever arms cannot overflow float32.
        if (strict && m > 0 && !(m >= MinMassProperty && m <= MaxMassProperty && (inertia == 0 || (inertia >= MinMassProperty && inertia <= MaxMassProperty))))
        {
            throw new ArgumentOutOfRangeException(paramName: null, "The body's shapes give mass " + m.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " and inertia " + inertia.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "; each must lie within " + MinMassProperty.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ".." + MaxMassProperty.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " (raise the density or the shape size).");
        }

        var bodies = Core.Bodies;
        float invMass, invI;
        if (m >= MinMassProperty && m <= MaxMassProperty && (inertia == 0 || (inertia >= MinMassProperty && inertia <= MaxMassProperty)))
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

        float maxExt = ComputeMaxExtent(b, newX, newY), minExt = float.MaxValue;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            ref readonly var g = ref Geom[s];
            var cv = g.Vertices;
            float r = g.Radius;

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
        bool sensor = IsSensor(a) || IsSensor(b);
        if (sensor ? SensorFreeCount > 0 : PairFreeCount > 0) { p = sensor ? SensorFree[--SensorFreeCount] : PairFree[--PairFreeCount]; }
        else
        {
            p = PairHigh++;
            Grow(ref Pairs, PairHigh);
        }

        Pairs[p] = new Pair2D { A = a, B = b, Cons = -1, Live = true, BodyA = -1, BodyB = -1, MixEpoch = -1 };
        PairMap.Set(PairKey(a, b), p);
        return p;
    }

    private void FreePair(int p)
    {
        PairMap.Remove(PairKey(Pairs[p].A, Pairs[p].B));
        Pairs[p].Live = false;
        Pairs[p].Cons = -1;
        if (IsSensor(Pairs[p].A) || IsSensor(Pairs[p].B))
        {
            Grow(ref SensorFree, SensorFreeCount + 1);
            SensorFree[SensorFreeCount++] = p;
            return;
        }

        Grow(ref PairFree, PairFreeCount + 1);
        PairFree[PairFreeCount++] = p;
    }

    /// <summary>Ends a pair's constraint: removes a contact, or reports the exit of an overlapping sensor pair. The pair keeps its slot.</summary>
    private void EndPairConstraint(int p)
    {
        int c = Pairs[p].Cons;
        if (c >= 0) { Core.RemoveConstraint(c); }
        else if (c == SensorOverlap) { Core.EmitSensorEvent(false, Pairs[p].A, Pairs[p].B); }
        Pairs[p].Cons = -1;
    }

    /// <summary>Forgets every pair that involves the shape. The constraints go with the body or shape in the core; this only clears the table.</summary>
    private void DropShapePairs(int s)
    {
        for (int p = 0; p < PairHigh; p++)
        {
            if (Pairs[p].Live && (Pairs[p].A == s || Pairs[p].B == s))
            {
                // The core removes a destroyed body's contact constraints itself; an overlapping sensor pair has none, so its exit is reported here, while the handles are still valid.
                if (Pairs[p].Cons == SensorOverlap) { Core.EmitSensorEvent(false, Pairs[p].A, Pairs[p].B); }
                FreePair(p);
            }
        }
    }

    /// <summary>The live contact constraint of a pair of shapes, or -1.</summary>
    public int ContactOf(int shapeA, int shapeB)
    {
        if (shapeA > shapeB) { (shapeA, shapeB) = (shapeB, shapeA); }
        return PairMap.TryGetValue(PairKey(shapeA, shapeB), out int p) ? Pairs[p].Cons : -1;
    }

    // ------------------------------------------------------------------ origin shifting

    /// <summary>
    /// Fails, before anything changes, when shifting the origin by the offset would put a shape's bounds (or its broadphase box) outside what the broadphase accepts, or when one already is.
    /// Rebuilding the proxies after a shift cannot then fail half way.
    /// </summary>
    public void CheckShift(ReadOnlySpan<double> offset)
    {
        var bodies = Core.Bodies;
        var shapes = Core.Shapes;
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s)) { continue; }
            int b = shapes.Owner[s];
            var t = TightBox(b, in Geom[s]);
            var moved = new Aabb2D(t.MinX - offset[0], t.MinY - offset[1], t.MaxX - offset[0], t.MaxY - offset[1]);
            string reason = "";
            if (moved.IsValid(out reason) && moved.Fatten(Opt.BroadphaseMargin, ReadOnlySpan<double>.Empty, 2.0).IsValid(out reason)) { continue; }
            var handle = Core.HandleOfBody(b);
            throw new BodyOutOfBoundsException(handle, "Shifting the origin by (" + offset[0].ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + offset[1].ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ") would leave body " + handle + " outside the supported range: " + reason + " Nothing was changed.");
        }
    }

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
        bool services = HasServiceState();
        bool joints = LiveJointCount() > 0;
        bool chars = CharHandles.Count > 0;
        w.WriteUInt32(chars ? WithCharacters : joints ? Extended : services ? Services : Version);
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
            w.WriteBool(Pairs[p].Live);
            if (Pairs[p].Live) { w.WriteInt32(Pairs[p].A); w.WriteInt32(Pairs[p].B); w.WriteInt32(Pairs[p].Cons); }
        }

        // The contact stack bottom to top, then the sensor stack as complements (~slot); a world without sensor pairs writes the same bytes as before.
        w.WriteInt32(PairFreeCount + SensorFreeCount);
        w.WriteInt32s(PairFree.AsSpan(0, PairFreeCount));
        for (int i = 0; i < SensorFreeCount; i++) { w.WriteInt32(~SensorFree[i]); }
        if (chars)
        {
            // Version 4: a presence-flags word, then the flagged blocks in a fixed order.
            w.WriteUInt32((joints ? BlockJoints : 0u) | (services ? BlockServices : 0u) | BlockCharacters);
            if (joints) { WriteJoints(w); }
            if (services) { WriteServices(w, ns); }
            WriteCharacters(w);
        }
        else if (joints)
        {
            w.WriteBool(services);
            WriteJoints(w);
            if (services) { WriteServices(w, ns); }
        }
        else if (services) { WriteServices(w, ns); }
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
        if (v != Version && v != Services && v != Extended && v != WithCharacters) { throw r.Error("unsupported 2D engine section version " + v.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
        float hertz = r.ReadSingle(), zeta = r.ReadSingle(), push = r.ReadSingle(), thr = r.ReadSingle(), maxSpeed = r.ReadSingle(), maxRot = r.ReadSingle();
        bool continuous = r.ReadBool();
        float slop = r.ReadSingle();
        if (!float.IsFinite(hertz + zeta + push + thr + maxSpeed + maxRot + slop) || slop < 0f || hertz < 0f || zeta < 0f || push < 0f || thr < 0f || !(maxSpeed > 0f) || !(maxRot > 0f)) { throw r.Error("invalid 2D engine tunables"); }

        int nb = r.ReadCount(65, "2D bodies"); // 65 bytes of per-body arrays follow
        if (nb > 1 << 22) { throw r.Error("too many 2D bodies"); }
        // Everything is read into staging arrays that the engine keeps between restores and swapped in at the end, so a restore allocates nothing once they have grown.
        var lc = Fit(ref _sLc, 2 * nb); var mass = Fit(ref _sMass, nb); var inertia = Fit(ref _sInertia, nb); var maxE = Fit(ref _sMaxE, nb); var minE = Fit(ref _sMinE, nb);
        var ld = Fit(ref _sLd, nb); var ad = Fit(ref _sAd, nb); var gs = Fit(ref _sGs, nb); var force = Fit(ref _sForce, 2 * nb); var torque = Fit(ref _sTorque, nb);
        var bf = Fit(ref _sBf, nb); var head = Fit(ref _sHead, nb); var tail = Fit(ref _sTail, nb); var dp = Fit(ref _sDp, 2 * nb);
        r.ReadSingles(lc.AsSpan(0, 2 * nb), true, "local centres"); r.ReadSingles(mass.AsSpan(0, nb), true, "masses"); r.ReadSingles(inertia.AsSpan(0, nb), true, "inertias");
        r.ReadSingles(maxE.AsSpan(0, nb), true, "extents"); r.ReadSingles(minE.AsSpan(0, nb), false, "extents");
        r.ReadSingles(ld.AsSpan(0, nb), true, "damping"); r.ReadSingles(ad.AsSpan(0, nb), true, "damping"); r.ReadSingles(gs.AsSpan(0, nb), true, "gravity scales");
        r.ReadSingles(force.AsSpan(0, 2 * nb), true, "forces"); r.ReadSingles(torque.AsSpan(0, nb), true, "torques");
        r.ReadBytes(bf.AsSpan(0, nb)); r.ReadInt32s(head.AsSpan(0, nb)); r.ReadInt32s(tail.AsSpan(0, nb));
        r.ReadSingles(dp.AsSpan(0, 2 * nb), true, "displacements");

        int ns = r.ReadCount(1, "2D shapes");
        if (ns > 1 << 22) { throw r.Error("too many 2D shapes"); }
        var geom = Fit(ref _sGeom, ns); var proxy = Fit(ref _sProxy, ns); var fr = Fit(ref _sFr, ns); var re = Fit(ref _sRe, ns); var ro = Fit(ref _sRo, ns); var de = Fit(ref _sDe, ns);
        var cat = Fit(ref _sCat, ns); var mask = Fit(ref _sMask, ns); var grp = Fit(ref _sGrp, ns); var next = Fit(ref _sNext, ns); var live = Fit(ref _sLive, ns);
        for (int s = 0; s < ns; s++)
        {
            live[s] = r.ReadBool();
            if (!live[s]) { continue; }
            geom[s] = ReadShape(r);
            proxy[s] = r.ReadInt32();
            fr[s] = r.ReadSingle(); re[s] = r.ReadSingle(); ro[s] = r.ReadSingle(); de[s] = r.ReadSingle();
            cat[s] = r.ReadUInt32(); mask[s] = r.ReadUInt32(); grp[s] = r.ReadInt32(); next[s] = r.ReadInt32();
            if (!(fr[s] >= 0f && re[s] >= 0f && re[s] <= 1f && ro[s] >= 0f && de[s] >= 0f) || !(fr[s] <= MaterialLimits.MaxCoefficient && ro[s] <= MaterialLimits.MaxCoefficient) || !float.IsFinite(de[s])) { throw r.Error("invalid shape material"); }
            if (next[s] < -1 || next[s] >= ns || !Bp.IsProxy(proxy[s])) { throw r.Error("invalid shape link or proxy"); }
        }

        for (int b = 0; b < nb; b++)
        {
            if (head[b] < -1 || head[b] >= ns || tail[b] < -1 || tail[b] >= ns) { throw r.Error("invalid body shape list"); }
        }

        int np = r.ReadCount(1, "pairs");
        var pa = Fit(ref _sPa, np); var pb = Fit(ref _sPb, np); var pc = Fit(ref _sPc, np); var pl = Fit(ref _sPl, np);
        for (int p = 0; p < np; p++)
        {
            pl[p] = r.ReadBool();
            if (!pl[p]) { continue; }
            pa[p] = r.ReadInt32(); pb[p] = r.ReadInt32(); pc[p] = r.ReadInt32();
            if (pc[p] < SensorOverlap) { throw r.Error("invalid pair constraint"); }
            if (pa[p] < 0 || pb[p] <= pa[p] || pb[p] >= ns || !live[pa[p]] || !live[pb[p]]) { throw r.Error("invalid pair"); }
        }

        int nf = r.ReadCount(4, "free pairs");
        var free = Fit(ref _sFree, nf);
        r.ReadInt32s(free.AsSpan(0, nf));
        for (int i = 0; i < nf; i++)
        {
            int slot = free[i] < 0 ? ~free[i] : free[i];
            if (slot < 0 || slot >= np || pl[slot]) { throw r.Error("invalid free pair"); }
        }

        var joints = Fit(ref _sJoints, 16);
        Array.Clear(joints);
        ServiceBlock? svc = null;
        CharacterBlock? chars = null;
        bool jointsRead = false;
        if (v == Extended || v == WithCharacters)
        {
            bool hasJoints = true, hasServices, hasChars = false;
            if (v == Extended) { hasServices = r.ReadBool(); }
            else
            {
                uint flags = r.ReadUInt32();
                if ((flags & ~(BlockJoints | BlockServices | BlockCharacters)) != 0 || (flags & BlockCharacters) == 0) { throw r.Error("invalid 2D section block flags " + flags.ToString(System.Globalization.CultureInfo.InvariantCulture)); }
                hasJoints = (flags & BlockJoints) != 0; hasServices = (flags & BlockServices) != 0; hasChars = true;
            }

            if (hasJoints) { ReadJoints(r, ref _sJoints); joints = _sJoints; jointsRead = true; }
            if (hasServices)
            {
                svc = ReadServices(r, ns, live);
                if (svc.IsDefault()) { throw r.Error("2D service block holds only default state; the snapshot would not re-serialize to the same bytes"); }
            }

            if (hasChars)
            {
                chars = ReadCharacters(r, nb);
                if (chars.Handles.Count == 0) { throw r.Error("2D section version 4 holds no characters"); }
            }

            int liveJoints = 0;
            for (int c = 0; c < joints.Length; c++) { if (joints[c].Type != 0) { liveJoints++; } }
            if (hasJoints && liveJoints == 0) { throw r.Error("2D section holds a joints block with no joints"); }
        }
        else if (v == Services) { svc = ReadServices(r, ns, live); }

        // Commit: the staged arrays become the live ones and the old live ones become the next staging.
        Opt.ContactHertz = hertz; Opt.ContactDampingRatio = zeta; Opt.PushOutVelocity = push; Opt.RestitutionThreshold = thr;
        Opt.MaxLinearSpeed = maxSpeed; Opt.MaxRotationPerSubstep = maxRot; Opt.ContinuousEnabled = continuous; Opt.ContactSlop = slop;
        (LocalCenter, _sLc) = (_sLc, LocalCenter); (Mass, _sMass) = (_sMass, Mass); (Inertia, _sInertia) = (_sInertia, Inertia); (MaxExtent, _sMaxE) = (_sMaxE, MaxExtent); (MinExtent, _sMinE) = (_sMinE, MinExtent);
        (LinearDamping, _sLd) = (_sLd, LinearDamping); (AngularDamping, _sAd) = (_sAd, AngularDamping); (GravityScale, _sGs) = (_sGs, GravityScale); (Force, _sForce) = (_sForce, Force); (Torque, _sTorque) = (_sTorque, Torque);
        (BodyFlags, _sBf) = (_sBf, BodyFlags); (ShapeHead, _sHead) = (_sHead, ShapeHead); (ShapeTail, _sTail) = (_sTail, ShapeTail); (Dp, _sDp) = (_sDp, Dp);
        EnsureBodies(Math.Max(nb, 1));
        (Geom, _sGeom) = (_sGeom, Geom); (Proxy, _sProxy) = (_sProxy, Proxy); (Friction, _sFr) = (_sFr, Friction); (Restitution, _sRe) = (_sRe, Restitution); (Rolling, _sRo) = (_sRo, Rolling); (Density, _sDe) = (_sDe, Density);
        (Category, _sCat) = (_sCat, Category); (Mask, _sMask) = (_sMask, Mask); (Group, _sGrp) = (_sGrp, Group); (NextShape, _sNext) = (_sNext, NextShape);
        EnsureShapes(Math.Max(ns, 1));
        Grow(ref Pairs, Math.Max(np, 1));
        for (int p = 0; p < np; p++) { Pairs[p] = new Pair2D { A = pa[p], B = pb[p], Cons = pc[p], Live = pl[p], BodyA = -1, BodyB = -1, MixEpoch = -1 }; }
        PairHigh = np;
        PairFreeCount = 0; SensorFreeCount = 0;
        Grow(ref PairFree, nf); Grow(ref SensorFree, nf);
        for (int i = 0; i < nf; i++)
        {
            if (free[i] < 0) { SensorFree[SensorFreeCount++] = ~free[i]; } else { PairFree[PairFreeCount++] = free[i]; }
        }
        if (jointsRead) { Opt.JointHertz = _pendingJointHertz; Opt.JointDampingRatio = _pendingJointDamping; }
        (Joints, _sJoints) = (_sJoints, Joints);
        Fit(ref JointForce, Joints.Length); Fit(ref JointTorque, Joints.Length);
        Array.Clear(JointForce); Array.Clear(JointTorque);
        _needBind = true;
        PairMap.Clear();
        for (int p = 0; p < np; p++)
        {
            if (Pairs[p].Live) { PairMap.Set(PairKey(Pairs[p].A, Pairs[p].B), p); }
        }

        CommitServices(svc, ns);
        CommitCharacters(chars);
    }

    // Staging arrays of Read (see there).
    private float[] _sLc = new float[16], _sMass = new float[8], _sInertia = new float[8], _sMaxE = new float[8], _sMinE = new float[8], _sLd = new float[8], _sAd = new float[8], _sGs = new float[8];
    private float[] _sForce = new float[16], _sTorque = new float[8], _sDp = new float[16], _sFr = new float[8], _sRe = new float[8], _sRo = new float[8], _sDe = new float[8];
    private byte[] _sBf = new byte[8];
    private int[] _sHead = new int[8], _sTail = new int[8], _sProxy = new int[8], _sGrp = new int[8], _sNext = new int[8], _sPa = new int[8], _sPb = new int[8], _sPc = new int[8], _sFree = new int[8];
    private Shape2D[] _sGeom = new Shape2D[8];
    private uint[] _sCat = new uint[8], _sMask = new uint[8];
    private bool[] _sLive = new bool[8], _sPl = new bool[8];
    private JointData2D[] _sJoints = new JointData2D[16];

    private static T[] Fit<T>(ref T[] a, int n)
    {
        if (a.Length < n) { a = new T[Math.Max(n, 8)]; }
        return a;
    }

    /// <summary>Checks the section against the core after a restore; false when the two disagree (a damaged snapshot that parsed).</summary>
    /// <summary>Why the last <see cref="IsConsistentWithCore"/> failed.</summary>
    public string ConsistencyProblem = "";

    public bool IsConsistentWithCore()
    {
        ConsistencyProblem = "";
        var bodies = Core.Bodies;
        var shapes = Core.Shapes;
        if (ShapeHead.Length < bodies.HighWater || Geom.Length < shapes.HighWater) { return false; }
        if (!CharactersConsistent()) { return false; }
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

            // The bounds shortcut and the continuous-collision tests trust the stored extent, so it must cover the geometry.
            if (bodies.TypeOf(b) == BodyType.Dynamic && !(MaxExtent[b] >= ComputeMaxExtent(b, LocalCenter[2 * b], LocalCenter[2 * b + 1])))
            {
                ConsistencyProblem = "the stored maximum extent of body slot " + b + " does not cover its shapes";
                return false;
            }
        }

        for (int p = 0; p < PairHigh; p++)
        {
            if (!Pairs[p].Live) { continue; }
            if (Pairs[p].Cons < -1 && (Pairs[p].Cons != SensorOverlap || !(IsSensor(Pairs[p].A) || IsSensor(Pairs[p].B)))) { return false; }
            if (Pairs[p].Cons >= 0 && (!cons.IsLive(Pairs[p].Cons) || cons.ShapeA[Pairs[p].Cons] != Pairs[p].A || cons.ShapeB[Pairs[p].Cons] != Pairs[p].B)) { return false; }
        }

        return true;
    }
}
