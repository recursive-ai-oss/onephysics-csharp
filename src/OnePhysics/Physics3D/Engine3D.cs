using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// The 3D engine's state and algorithms on top of <see cref="WorldCore"/>: per-body and per-shape data, shared geometry, the pair table, mass, the contact stage and the step stages. Internal: the public face is
/// <see cref="PhysicsWorld3D"/>. Everything saved in snapshots and covered by the world hash is written by <see cref="Write"/>; everything else is rebuilt every step.
/// </summary>
internal sealed partial class Engine3D : ISnapshotSection
{
    public const int Id = 0x33445747; // "GW3D"
    private const uint Version = 1;

    public readonly WorldCore Core;
    public readonly Broadphase3D Bp;
    public readonly WorldOptions3D Opt;

    /// <summary>The public world that owns this engine (used to hand out handles to callbacks).</summary>
    public PhysicsWorld3D? World;

    // Bodies (index = body slot).
    public float[] LocalCenter = new float[24];   // centre of mass in the body's frame, 3 per body
    public float[] Mass = new float[8];
    public float[] Inertia = new float[48];       // about the centre of mass in the body's frame: xx xy xz yy yz zz
    public float[] MaxExtent = new float[8];      // farthest point of any shape from the centre of mass
    public float[] MinExtent = new float[8];      // smallest half-width of any shape; decides whether a body moves fast enough to need continuous collision
    public float[] LinearDamping = new float[8];
    public float[] AngularDamping = new float[8];
    public float[] GravityScale = new float[8];
    public float[] Force = new float[24];
    public float[] Torque = new float[24];
    public byte[] BodyFlags = new byte[8];        // bit 0: fixed rotation, bit 1: needs the gyroscopic term
    public int[] ShapeHead = new int[8];
    public int[] ShapeTail = new int[8];
    public float[] Dp = new float[24];            // displacement over the step (the last step's, between steps)

    // Transient per step.
    public double[] StartPos = new double[24];
    public float[] StartRot = new float[32];
    public float[] Dq = new float[32];            // accumulated rotation over the step as a quaternion (x, y, z, w)
    public float[] Pk = new float[16 * 8];        // per-step copy of Dp, Dq and WInvI side by side (16 floats a body) so the contact kernel reads one row per body; valid between the start of a step's contacts and its end
    public float[] WInvI = new float[48];         // inverse inertia in world axes, refreshed whenever a body turns
    public float[] Wide = Array.Empty<float>();   // contact solver groups, one per batch (see W3D)
    public int[] WideIdx = Array.Empty<int>();
    public float[] WideJ = Array.Empty<float>();   // point-joint solver groups, one per batch (see J3D)
    public int[] WideJIdx = Array.Empty<int>();
    public int[] SlotOf = Array.Empty<int>();     // constraint -> group * 8 + lane for this step, -1 when not in a group
    public int KernelWidth = KernelWidthChoice.Default; // lanes per kernel run; results are the same at every width

    // Shapes (index = shape slot).
    public int[] GeomOf = new int[8];
    public RigidTransformF[] Local = new RigidTransformF[8];
    public int[] Proxy = new int[8];
    public float[] Friction = new float[8];
    public float[] Restitution = new float[8];
    public float[] Rolling = new float[8];
    public float[] Density = new float[8];
    public uint[] Category = new uint[8];
    public uint[] Mask = new uint[8];
    public int[] Group = new int[8];
    public int[] NextShape = new int[8];

    // Shared geometry, deduplicated by recipe.
    public readonly List<Geom3D?> Geoms = new();
    private readonly Dictionary<RecipeKey, int> _geomIndex = new();

    // Pair table: one entry per pair of shapes whose broadphase boxes overlap, with the chain of contact constraints (one per sub-shape pair) while the shapes touch.
    public int[] PairA = new int[8];
    public int[] PairB = new int[8];
    public int[] PairCons = new int[8];           // first constraint of the chain, or -1
    public bool[] PairLive = new bool[8];
    public int PairHigh;
    public int[] PairFree = new int[8];
    public int PairFreeCount;
    public readonly Dictionary<long, int> PairMap = new();
    public int[] ConsNext = new int[8];           // next constraint in the pair's chain, indexed by constraint slot

    public Engine3D(WorldCore core, Broadphase3D bp, WorldOptions3D opt)
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
        Grow(ref LocalCenter, 3 * n); Grow(ref Mass, n); Grow(ref Inertia, 6 * n); Grow(ref MaxExtent, n); Grow(ref MinExtent, n);
        Grow(ref LinearDamping, n); Grow(ref AngularDamping, n); Grow(ref GravityScale, n); Grow(ref Force, 3 * n); Grow(ref Torque, 3 * n);
        Grow(ref BodyFlags, n); Grow(ref ShapeHead, n); Grow(ref ShapeTail, n); Grow(ref Dp, 3 * n);
        Grow(ref StartPos, 3 * n); Grow(ref StartRot, 4 * n); Grow(ref Dq, 4 * n); Grow(ref WInvI, 6 * n); Grow(ref Pk, 16 * n);
    }

    public void EnsureShapes(int n)
    {
        Grow(ref GeomOf, n); Grow(ref Local, n); Grow(ref Proxy, n); Grow(ref Friction, n); Grow(ref Restitution, n); Grow(ref Rolling, n); Grow(ref Density, n);
        Grow(ref Category, n); Grow(ref Mask, n); Grow(ref Group, n); Grow(ref NextShape, n); EnsureServiceShapes(n);
    }

    // ------------------------------------------------------------------ geometry table

    private readonly struct RecipeKey : IEquatable<RecipeKey>
    {
        private readonly byte[] _bytes;
        private readonly int _hash;

        public RecipeKey(byte[] bytes)
        {
            _bytes = bytes;
            uint h = 2166136261u;
            foreach (byte x in bytes) { h = (h ^ x) * 16777619u; }
            _hash = (int)h;
        }

        public bool Equals(RecipeKey other) => _hash == other._hash && _bytes.AsSpan().SequenceEqual(other._bytes);

        public override bool Equals(object? obj) => obj is RecipeKey k && Equals(k);

        // Only used to look recipes up; no result depends on it.
        public override int GetHashCode() => _hash;
    }

    /// <summary>Finds or builds the geometry for a recipe and takes a reference. Throws the shape factories' typed errors for bad numbers; nothing changes then.</summary>
    public int AcquireGeom(byte[] recipe)
    {
        var key = new RecipeKey(recipe);
        if (_geomIndex.TryGetValue(key, out int id))
        {
            Geoms[id]!.Refs++;
            return id;
        }

        var g = Geom3D.Build(recipe);
        g.Refs = 1;
        return InstallGeom(g);
    }

    private int InstallGeom(Geom3D g)
    {
        int id = Geoms.IndexOf(null);
        if (id < 0) { id = Geoms.Count; Geoms.Add(g); } else { Geoms[id] = g; }
        _geomIndex[new RecipeKey(g.Recipe)] = id;
        return id;
    }

    private void ReleaseGeom(int id)
    {
        var g = Geoms[id]!;
        if (--g.Refs > 0) { return; }
        _geomIndex.Remove(new RecipeKey(g.Recipe));
        Geoms[id] = null;
    }

    // ------------------------------------------------------------------ bodies

    public BodyHandle CreateBody(in BodyDef d)
    {
        if (!double.IsFinite(d.Position.X) || !double.IsFinite(d.Position.Y) || !double.IsFinite(d.Position.Z)) { throw new ArgumentOutOfRangeException(nameof(d), "Body position must be finite."); }
        var q = Normalize(d.Rotation, nameof(d));
        Finite3(d.LinearVelocity, "LinearVelocity");
        Finite3(d.AngularVelocity, "AngularVelocity");
        Nonnegative(d.LinearDamping, "LinearDamping");
        Nonnegative(d.AngularDamping, "AngularDamping");
        if (!float.IsFinite(d.GravityScale)) { throw new ArgumentOutOfRangeException(nameof(d), d.GravityScale, "GravityScale must be finite."); }
        var flags = BodyBits.None;
        if (d.IsBullet) { flags |= BodyBits.Bullet; }
        if (!d.AllowSleep) { flags |= BodyBits.NoSleep; }
        var h = Core.CreateBody(d.Type, [d.Position.X, d.Position.Y, d.Position.Z], [q.X, q.Y, q.Z, q.W], [d.LinearVelocity.X, d.LinearVelocity.Y, d.LinearVelocity.Z],
            [d.AngularVelocity.X, d.AngularVelocity.Y, d.AngularVelocity.Z], 1f, [1f, 0f, 0f, 1f, 0f, 1f], flags);
        int b = h.Index;
        EnsureBodies(b + 1);
        for (int k = 0; k < 3; k++) { LocalCenter[3 * b + k] = 0f; Force[3 * b + k] = 0f; Torque[3 * b + k] = 0f; Dp[3 * b + k] = 0f; }
        for (int k = 0; k < 6; k++) { Inertia[6 * b + k] = 0f; }
        Mass[b] = 0f; MaxExtent[b] = 0f; MinExtent[b] = float.MaxValue; ResetBuoyancy(b);
        LinearDamping[b] = d.LinearDamping; AngularDamping[b] = d.AngularDamping; GravityScale[b] = d.GravityScale;
        BodyFlags[b] = (byte)(d.FixedRotation ? 1 : 0);
        ShapeHead[b] = -1; ShapeTail[b] = -1;
        if (d.Type == BodyType.Dynamic)
        {
            // Mass 1 and a unit inertia until shapes say otherwise.
            Mass[b] = 1f;
            Inertia[6 * b] = 1f; Inertia[6 * b + 3] = 1f; Inertia[6 * b + 5] = 1f;
            if (d.FixedRotation) { Core.Bodies.InverseInertiaOf(b).Clear(); }
            if (d.StartAsleep) { Core.Sleep(h); }
        }

        return h;
    }

    /// <summary>Normalises a quaternion; zero or non-finite ones are rejected.</summary>
    public static QuaternionF Normalize(QuaternionF q, string param)
    {
        float l2 = q.LengthSquared();
        if (!float.IsFinite(l2) || !(l2 > 1e-30f)) { throw new ArgumentOutOfRangeException(param, q, "A rotation must be a finite, non-zero quaternion."); }
        float inv = 1f / float.Sqrt(l2);
        return new QuaternionF(q.X * inv, q.Y * inv, q.Z * inv, q.W * inv);
    }

    private static void Finite3(Vector3F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite."); }
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
            ReleaseGeom(GeomOf[s]);
        }

        DropCharacterOfBody(b);
        DropVehicleOfBody(b);
        DropRagdollOfBody(b);
        ResetBuoyancy(b);
        Core.DestroyBody(handle);
        ReconcileJoints();
        ShapeHead[b] = -1; ShapeTail[b] = -1;
        Mass[b] = 0f;
        for (int k = 0; k < 6; k++) { Inertia[6 * b + k] = 0f; }
    }

    // ------------------------------------------------------------------ shapes

    public ShapeHandle AddShape(BodyHandle body, byte[] recipe, in ShapeDef def)
    {
        Nonnegative(def.Density, "Density");
        Nonnegative(def.Friction, "Friction");
        Nonnegative(def.RollingResistance, "RollingResistance");
        if (!(def.Restitution >= 0f && def.Restitution <= 1f)) { throw new ArgumentOutOfRangeException(nameof(def), def.Restitution, "Restitution must be within 0..1."); }
        Finite3(def.LocalPosition, "LocalPosition");
        CheckLayer(def.Layer, "Layer");
        var rot = Normalize(def.LocalRotation, nameof(def));
        int b = Core.Resolve(body);
        var type = Core.Bodies.TypeOf(b);
        if (Math.Abs(def.LocalPosition.X) > PlacedShape3D.MaxLocalOffset || Math.Abs(def.LocalPosition.Y) > PlacedShape3D.MaxLocalOffset || Math.Abs(def.LocalPosition.Z) > PlacedShape3D.MaxLocalOffset)
        {
            throw new ArgumentOutOfRangeException(nameof(def), def.LocalPosition, "LocalPosition components must be at most " + PlacedShape3D.MaxLocalOffset.ToString(CultureInfo.InvariantCulture) + " in magnitude.");
        }

        int gid = AcquireGeom(recipe);
        var g = Geoms[gid]!;
        if (g.Soup != null && type != BodyType.Static)
        {
            ReleaseGeom(gid);
            throw new StaticOnlyShapeException("Triangle meshes and height fields can only be attached to static bodies.");
        }

        var sh = Core.CreateShape(body);
        int s = sh.Index;
        EnsureShapes(s + 1);
        GeomOf[s] = gid;
        Local[s] = new RigidTransformF(rot, def.LocalPosition);
        Friction[s] = def.Friction; Restitution[s] = def.Restitution; Rolling[s] = def.RollingResistance; Density[s] = def.Density;
        Category[s] = def.CategoryBits; Mask[s] = def.MaskBits; Group[s] = def.GroupIndex;
        ShapeFlags[s] = def.IsSensor ? FlagSensor : (byte)0; ShapeLayer[s] = (byte)def.Layer;
        NextShape[s] = -1;
        var tight = TightBox(b, s);
        if (!tight.IsValid(out string reason))
        {
            Core.DestroyShape(sh);
            ReleaseGeom(gid);
            throw new ArgumentOutOfRangeException(nameof(recipe), "The shape's bounds are not valid: " + reason);
        }

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
        int prev = -1;
        for (int t = ShapeHead[b]; t >= 0; prev = t, t = NextShape[t])
        {
            if (t != s) { continue; }
            if (prev >= 0) { NextShape[prev] = NextShape[t]; } else { ShapeHead[b] = NextShape[t]; }
            if (ShapeTail[b] == t) { ShapeTail[b] = prev; }
            break;
        }

        ReleaseGeom(GeomOf[s]);
        Core.DestroyShape(handle);
        if (Core.Bodies.TypeOf(b) == BodyType.Dynamic) { UpdateMass(b); }
    }

    /// <summary>Fails before a step when a moving body's bounds are outside what the broadphase accepts, so a step never stops half way. Lowest slot first.</summary>
    public void CheckBounds()
    {
        if (FindOutOfBounds(false, out var h, out string message, true)) { throw new BodyOutOfBoundsException(h, message); }
    }

    private readonly double[] _dp3 = new double[3];

    /// <summary>Finds the first body (lowest slot) whose bounds, as the broadphase would store them, are not valid. With <paramref name="movingOnly"/>, sleeping bodies are skipped.</summary>
    public bool FindOutOfBounds(bool includeStatic, out BodyHandle handle, out string message, bool movingOnly = false)
    {
        var bodies = Core.Bodies;
        int nb = bodies.HighWater;
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b) || (!includeStatic && bodies.TypeOf(b) == BodyType.Static)) { continue; }
            // A sleeping body has not moved since it was last checked (teleporting wakes it), and building the box of thousands of resting bodies every step is most of the cost of a sleeping world.
            if (movingOnly && bodies.IsDynamic(b) && !bodies.IsAwake(b)) { continue; }
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                var box = TightBox(b, s);
                // The box the broadphase would store: margin plus the last step's displacement stretched by its default multiplier (2).
                _dp3[0] = Dp[3 * b]; _dp3[1] = Dp[3 * b + 1]; _dp3[2] = Dp[3 * b + 2];
                var fat = box.Fatten(Opt.BroadphaseMargin, _dp3, 2.0);
                if (box.IsValid(out string reason) && fat.IsValid(out reason)) { continue; }
                handle = Core.HandleOfBody(b);
                var pos = bodies.PositionOf(b);
                var ci = CultureInfo.InvariantCulture;
                message = "Body " + handle + " at (" + pos[0].ToString("R", ci) + ", " + pos[1].ToString("R", ci) + ", " + pos[2].ToString("R", ci) + ") has left the supported range: " + reason + " Move it back, destroy it, or call ShiftOrigin.";
                return true;
            }
        }

        handle = default;
        message = "";
        return false;
    }

    public QuaternionF RotationOfBody(int b)
    {
        var r = Core.Bodies.RotationOf(b);
        return new QuaternionF(r[0], r[1], r[2], r[3]);
    }

    public Vector3F LocalCenterOf(int b) => new(LocalCenter[3 * b], LocalCenter[3 * b + 1], LocalCenter[3 * b + 2]);

    /// <summary>The pose of a shape's frame relative to its body's centre of mass, with world orientation: rotation <c>q·local</c>, translation <c>q·(local − lc)</c>.</summary>
    public RigidTransformF ShapePoseFromCom(int b, int s)
    {
        var q = RotationOfBody(b);
        var l = Local[s];
        return new RigidTransformF(q * l.Rotation, q.Rotate(l.Translation - LocalCenterOf(b)));
    }

    /// <summary>The body's shape bounds in world space, tight (no margin).</summary>
    public Aabb3D TightBox(int b, int s)
    {
        var pos = Core.Bodies.PositionOf(b);
        var pose = ShapePoseFromCom(b, s);
        var g = Geoms[GeomOf[s]]!;
        Vector3F mn, mx;
        if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, pose, out mn, out mx); }
        else { BoundsOfBox(g.LocalBounds, pose, out mn, out mx); }
        return new Aabb3D(pos[0] + (double)mn.X, pos[1] + (double)mn.Y, pos[2] + (double)mn.Z, pos[0] + (double)mx.X, pos[1] + (double)mx.Y, pos[2] + (double)mx.Z);
    }

    /// <summary>Bounds of a transformed box: centre moved, half sizes through |R|, padded like <see cref="ShapeBounds3D"/>.</summary>
    public static void BoundsOfBox(in Bounds3F box, in RigidTransformF pose, out Vector3F min, out Vector3F max)
    {
        var m = pose.Rotation.ToMatrix();
        var c = (box.Min + box.Max) * 0.5f;
        var e = (box.Max - box.Min) * 0.5f;
        var t = pose.Translation;
        float cx = m.M11 * c.X + m.M12 * c.Y + m.M13 * c.Z + t.X, cy = m.M21 * c.X + m.M22 * c.Y + m.M23 * c.Z + t.Y, cz = m.M31 * c.X + m.M32 * c.Y + m.M33 * c.Z + t.Z;
        float ex = Math.Abs(m.M11) * e.X + Math.Abs(m.M12) * e.Y + Math.Abs(m.M13) * e.Z;
        float ey = Math.Abs(m.M21) * e.X + Math.Abs(m.M22) * e.Y + Math.Abs(m.M23) * e.Z;
        float ez = Math.Abs(m.M31) * e.X + Math.Abs(m.M32) * e.Y + Math.Abs(m.M33) * e.Z;
        const float pad = 1.0f / 65536f;
        float px = (Math.Abs(cx - ex) + Math.Abs(cx + ex) + 2f * ex) * pad, py = (Math.Abs(cy - ey) + Math.Abs(cy + ey) + 2f * ey) * pad, pz = (Math.Abs(cz - ez) + Math.Abs(cz + ez) + 2f * ez) * pad;
        min = new Vector3F(cx - ex - px, cy - ey - py, cz - ez - pz);
        max = new Vector3F(cx + ex + px, cy + ey + py, cz + ez + pz);
    }

    private readonly List<MassPropertiesF> _parts = new();

    /// <summary>Recomputes mass, centre of mass, inertia and extents of a dynamic body from its shapes, moving the stored centre-of-mass position so the body origin stays put.</summary>
    public void UpdateMass(int b)
    {
        _parts.Clear();
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (!(Density[s] > 0f) || IsSensor(s)) { continue; }
            var g = Geoms[GeomOf[s]]!;
            if (g.Soup != null) { continue; }
            var m = g.UnitMass;
            float d = Density[s];
            var l = Local[s];
            _parts.Add(new MassPropertiesF(m.Mass * d, l.TransformPoint(m.Center), InertiaF.Rotate(m.Inertia, l.Rotation) * d));
        }

        var bodies = Core.Bodies;
        var oldLc = LocalCenterOf(b);
        var newLc = default(Vector3F);
        float mass = 1f;
        var tensor = Matrix3x3F.Identity;
        Matrix3x3F inv = Matrix3x3F.Identity;
        bool ok = false;
        if (_parts.Count > 0)
        {
            var total = InertiaF.Combine(System.Runtime.InteropServices.CollectionsMarshal.AsSpan(_parts));
            if (total.Mass > 0f && float.IsFinite(total.Mass) && total.Inertia.TryInvert(out var iv)
                && float.IsFinite(iv.M11 + iv.M12 + iv.M13 + iv.M22 + iv.M23 + iv.M33) && iv.M11 > 0f && iv.M22 > 0f && iv.M33 > 0f)
            {
                mass = total.Mass; newLc = total.Center; tensor = total.Inertia; inv = iv; ok = true;
            }
        }

        bool fixedRot = (BodyFlags[b] & 1) != 0;
        bodies.InverseMass[b] = 1f / mass;
        Mass[b] = mass;
        var ii = Inertia.AsSpan(6 * b, 6);
        ii[0] = tensor.M11; ii[1] = tensor.M12; ii[2] = tensor.M13; ii[3] = tensor.M22; ii[4] = tensor.M23; ii[5] = tensor.M33;
        var ci = bodies.InverseInertiaOf(b);
        if (fixedRot) { ci.Clear(); }
        else { ci[0] = inv.M11; ci[1] = inv.M12; ci[2] = inv.M13; ci[3] = inv.M22; ci[4] = inv.M23; ci[5] = inv.M33; }
        // The gyroscopic term vanishes for an isotropic tensor.
        bool iso = Math.Abs(tensor.M12) + Math.Abs(tensor.M13) + Math.Abs(tensor.M23) <= 1e-6f * tensor.M11 && Math.Abs(tensor.M11 - tensor.M22) <= 1e-6f * tensor.M11 && Math.Abs(tensor.M11 - tensor.M33) <= 1e-6f * tensor.M11;
        BodyFlags[b] = (byte)((fixedRot ? 1 : 0) | (ok && !iso && !fixedRot ? 2 : 0));

        // Keep the origin where it was: the centre of mass moves by the rotated change of the local centre.
        var q = RotationOfBody(b);
        var shift = q.Rotate(newLc - oldLc);
        var pos = bodies.PositionOf(b);
        pos[0] += (double)shift.X; pos[1] += (double)shift.Y; pos[2] += (double)shift.Z;
        LocalCenter[3 * b] = newLc.X; LocalCenter[3 * b + 1] = newLc.Y; LocalCenter[3 * b + 2] = newLc.Z;

        float maxExt = 0f, minExt = float.MaxValue;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            var g = Geoms[GeomOf[s]]!;
            var pose = Local[s];
            Vector3F mn, mx;
            if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, pose, out mn, out mx); }
            else { BoundsOfBox(g.LocalBounds, pose, out mn, out mx); }
            float dx = Math.Max(Math.Abs(mn.X - newLc.X), Math.Abs(mx.X - newLc.X)), dy = Math.Max(Math.Abs(mn.Y - newLc.Y), Math.Abs(mx.Y - newLc.Y)), dz = Math.Max(Math.Abs(mn.Z - newLc.Z), Math.Abs(mx.Z - newLc.Z));
            float ext = float.Sqrt(dx * dx + dy * dy + dz * dz);
            if (ext > maxExt) { maxExt = ext; }
            float me = 0.5f * Math.Min(mx.X - mn.X, Math.Min(mx.Y - mn.Y, mx.Z - mn.Z));
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
        if (p < _skip.Length) { _skip[p].Clear = 0f; }
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

    /// <summary>The first live contact constraint of a pair of shapes, or -1.</summary>
    public int ContactOf(int shapeA, int shapeB)
    {
        if (shapeA > shapeB) { (shapeA, shapeB) = (shapeB, shapeA); }
        return PairMap.TryGetValue(PairKey(shapeA, shapeB), out int p) ? PairCons[p] : -1;
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
            Proxy[s] = Bp.CreateProxy(type, TightBox(b, s), s);
        }
    }

    // ------------------------------------------------------------------ snapshot section

    public void Write(StateWriter w)
    {
        bool joints = LiveJointCount() > 0;
        bool services = HasServiceState();
        bool chars = HasCharacters;
        bool vehs = HasVehicles;
        bool rags = HasRagdolls;
        bool fluids = HasFluidState();
        w.WriteUInt32(joints || services || chars || vehs || rags || fluids ? VersionJoints : Version);
        w.WriteSingle(Opt.ContactHertz); w.WriteSingle(Opt.ContactDampingRatio); w.WriteSingle(Opt.PushOutVelocity); w.WriteSingle(Opt.RestitutionThreshold);
        w.WriteSingle(Opt.MaxLinearSpeed); w.WriteSingle(Opt.MaxRotationPerSubstep); w.WriteBool(Opt.ContinuousEnabled); w.WriteSingle(Opt.ContactSlop);
        w.WriteSingle(Opt.SpeculativeDistance);
        int nb = Core.Bodies.HighWater;
        w.WriteInt32(nb);
        w.WriteSingles(LocalCenter.AsSpan(0, 3 * nb));
        w.WriteSingles(Mass.AsSpan(0, nb)); w.WriteSingles(Inertia.AsSpan(0, 6 * nb));
        w.WriteSingles(MaxExtent.AsSpan(0, nb)); w.WriteSingles(MinExtent.AsSpan(0, nb));
        w.WriteSingles(LinearDamping.AsSpan(0, nb)); w.WriteSingles(AngularDamping.AsSpan(0, nb)); w.WriteSingles(GravityScale.AsSpan(0, nb));
        w.WriteSingles(Force.AsSpan(0, 3 * nb)); w.WriteSingles(Torque.AsSpan(0, 3 * nb));
        w.WriteBytes(BodyFlags.AsSpan(0, nb));
        w.WriteInt32s(ShapeHead.AsSpan(0, nb)); w.WriteInt32s(ShapeTail.AsSpan(0, nb));
        w.WriteSingles(Dp.AsSpan(0, 3 * nb));

        w.WriteInt32(Geoms.Count);
        foreach (var g in Geoms)
        {
            w.WriteBool(g != null);
            if (g == null) { continue; }
            w.WriteInt32(g.Recipe.Length);
            w.WriteBytes(g.Recipe);
        }

        int ns = Core.Shapes.HighWater;
        w.WriteInt32(ns);
        for (int s = 0; s < ns; s++)
        {
            bool live = Core.Shapes.Handles.IsLive(s);
            w.WriteBool(live);
            if (!live) { continue; }
            w.WriteInt32(GeomOf[s]);
            var l = Local[s];
            w.WriteSingle(l.Rotation.X); w.WriteSingle(l.Rotation.Y); w.WriteSingle(l.Rotation.Z); w.WriteSingle(l.Rotation.W);
            w.WriteSingle(l.Translation.X); w.WriteSingle(l.Translation.Y); w.WriteSingle(l.Translation.Z);
            w.WriteInt32(Proxy[s]);
            w.WriteSingle(Friction[s]); w.WriteSingle(Restitution[s]); w.WriteSingle(Rolling[s]); w.WriteSingle(Density[s]);
            w.WriteUInt32(Category[s]); w.WriteUInt32(Mask[s]); w.WriteInt32(Group[s]); w.WriteInt32(NextShape[s]);
        }

        w.WriteInt32(PairHigh);
        for (int p = 0; p < PairHigh; p++)
        {
            w.WriteBool(PairLive[p]);
            if (!PairLive[p]) { continue; }
            w.WriteInt32(PairA[p]); w.WriteInt32(PairB[p]);
            int n = 0;
            for (int c = PairCons[p]; c >= 0; c = ConsNext[c]) { n++; }
            w.WriteInt32(n);
            for (int c = PairCons[p]; c >= 0; c = ConsNext[c]) { w.WriteInt32(c); }
        }

        w.WriteInt32(PairFreeCount);
        w.WriteInt32s(PairFree.AsSpan(0, PairFreeCount));
        if (joints || services || chars || vehs || rags || fluids)
        {
            // Version 2: a presence-flags word, then the flagged blocks in a fixed order: joints, services (layers, filters, sensors), characters.
            w.WriteUInt32((joints ? BlockJoints : 0u) | (services ? BlockServices : 0u) | (chars ? BlockCharacters : 0u) | (vehs ? BlockVehicles : 0u) | (vehs && HasVehicleKinds() ? BlockVehicleKinds : 0u) | (rags ? BlockRagdolls : 0u) | (fluids ? BlockFluids : 0u));
            if (joints) { WriteJoints(w); }
            if (services) { WriteServices(w, ns); }
            if (chars) { WriteCharacters(w); }
            if (vehs) { WriteVehicles(w); }
            if (vehs && HasVehicleKinds()) { WriteVehicleKinds(w); }
            if (rags) { WriteRagdolls(w); }
            if (fluids) { WriteFluids(w); }
        }
    }

    public void Read(StateReader r)
    {
        uint v = r.ReadUInt32();
        if (v != Version && v != VersionJoints) { throw r.Error("unsupported 3D engine section version " + v.ToString(CultureInfo.InvariantCulture)); }
        float hertz = r.ReadSingle(), zeta = r.ReadSingle(), push = r.ReadSingle(), thr = r.ReadSingle(), maxSpeed = r.ReadSingle(), maxRot = r.ReadSingle();
        bool continuous = r.ReadBool();
        float slop = r.ReadSingle(), spec = r.ReadSingle();
        if (!float.IsFinite(hertz + zeta + push + thr + maxSpeed + maxRot + slop + spec) || slop < 0f || spec < 0f || hertz < 0f || zeta < 0f || push < 0f || thr < 0f || !(maxSpeed > 0f) || !(maxRot > 0f))
        {
            throw r.Error("invalid 3D engine tunables");
        }

        int nb = r.ReadCount(105, "3D bodies") /* bytes written per body: 12+4+24+4+4+4+4+4+12+12+1+4+4+12 */;
        if (nb > 1 << 22) { throw r.Error("too many 3D bodies"); }
        var lc = new float[3 * nb]; var mass = new float[nb]; var inertia = new float[6 * nb]; var maxE = new float[nb]; var minE = new float[nb];
        var ld = new float[nb]; var ad = new float[nb]; var gs = new float[nb]; var force = new float[3 * nb]; var torque = new float[3 * nb];
        var bf = new byte[nb]; var head = new int[nb]; var tail = new int[nb]; var dp = new float[3 * nb];
        r.ReadSingles(lc, true, "local centres"); r.ReadSingles(mass, true, "masses"); r.ReadSingles(inertia, true, "inertias");
        r.ReadSingles(maxE, true, "extents"); r.ReadSingles(minE, false, "extents");
        r.ReadSingles(ld, true, "damping"); r.ReadSingles(ad, true, "damping"); r.ReadSingles(gs, true, "gravity scales");
        r.ReadSingles(force, true, "forces"); r.ReadSingles(torque, true, "torques");
        r.ReadBytes(bf); r.ReadInt32s(head); r.ReadInt32s(tail);
        r.ReadSingles(dp, true, "displacements");

        int ng = r.ReadCount(1, "3D geometries");
        if (ng > 1 << 22) { throw r.Error("too many 3D geometries"); }
        var geoms = new List<Geom3D?>(ng);
        for (int i = 0; i < ng; i++)
        {
            if (!r.ReadBool()) { geoms.Add(null); continue; }
            int len = r.ReadCount(1, "shape recipe");
            var recipe = new byte[len];
            r.ReadBytes(recipe);
            try { geoms.Add(Geom3D.Build(recipe)); }
            catch (Exception e) when (e is ArgumentException || e is SnapshotFormatException || e is PhysicsException || e is NotSupportedException)
            {
                throw r.Error("invalid shape recipe: " + e.Message);
            }
        }

        int ns = r.ReadCount(1, "3D shapes");
        if (ns > 1 << 22) { throw r.Error("too many 3D shapes"); }
        var geomOf = new int[ns]; var local = new RigidTransformF[ns]; var proxy = new int[ns]; var fr = new float[ns]; var re = new float[ns]; var ro = new float[ns]; var de = new float[ns];
        var cat = new uint[ns]; var mask = new uint[ns]; var grp = new int[ns]; var next = new int[ns]; var live = new bool[ns];
        for (int s = 0; s < ns; s++)
        {
            live[s] = r.ReadBool();
            if (!live[s]) { continue; }
            geomOf[s] = r.ReadInt32();
            if (geomOf[s] < 0 || geomOf[s] >= ng || geoms[geomOf[s]] == null) { throw r.Error("shape refers to a missing geometry"); }
            geoms[geomOf[s]]!.Refs++;
            float qx = r.ReadSingle(), qy = r.ReadSingle(), qz = r.ReadSingle(), qw = r.ReadSingle(), tx = r.ReadSingle(), ty = r.ReadSingle(), tz = r.ReadSingle();
            float ql = qx * qx + qy * qy + qz * qz + qw * qw;
            if (!float.IsFinite(ql + tx + ty + tz) || !(ql >= 0.999f && ql <= 1.001f) || Math.Abs(tx) > PlacedShape3D.MaxLocalOffset || Math.Abs(ty) > PlacedShape3D.MaxLocalOffset || Math.Abs(tz) > PlacedShape3D.MaxLocalOffset)
            {
                throw r.Error("invalid shape placement");
            }

            local[s] = new RigidTransformF(new QuaternionF(qx, qy, qz, qw), new Vector3F(tx, ty, tz));
            proxy[s] = r.ReadInt32();
            fr[s] = r.ReadSingle(); re[s] = r.ReadSingle(); ro[s] = r.ReadSingle(); de[s] = r.ReadSingle();
            cat[s] = r.ReadUInt32(); mask[s] = r.ReadUInt32(); grp[s] = r.ReadInt32(); next[s] = r.ReadInt32();
            if (!(fr[s] >= 0f && re[s] >= 0f && re[s] <= 1f && ro[s] >= 0f && de[s] >= 0f) || !float.IsFinite(fr[s] + ro[s] + de[s])) { throw r.Error("invalid shape material"); }
            if (next[s] < -1 || next[s] >= ns || !Bp.IsProxy(proxy[s])) { throw r.Error("invalid shape link or proxy"); }
        }

        foreach (var g in geoms)
        {
            if (g != null && g.Refs == 0) { throw r.Error("geometry without a shape"); }
        }

        for (int b = 0; b < nb; b++)
        {
            if (head[b] < -1 || head[b] >= ns || tail[b] < -1 || tail[b] >= ns) { throw r.Error("invalid body shape list"); }
        }

        int np = r.ReadCount(1, "3D pairs");
        if (np > 1 << 24) { throw r.Error("too many 3D pairs"); }
        var pa = new int[np]; var pb = new int[np]; var pc = new int[np]; var pl = new bool[np];
        var chains = new List<int>();
        var chainLen = new int[np];
        int maxCons = Math.Max(Core.Constraints.HighWater, 8);
        for (int p = 0; p < np; p++)
        {
            pl[p] = r.ReadBool();
            if (!pl[p]) { continue; }
            pa[p] = r.ReadInt32(); pb[p] = r.ReadInt32();
            if (pa[p] < 0 || pb[p] <= pa[p] || pb[p] >= ns || !live[pa[p]] || !live[pb[p]]) { throw r.Error("invalid pair"); }
            int n = r.ReadCount(4, "pair constraints");
            chainLen[p] = n;
            for (int i = 0; i < n; i++)
            {
                int c = r.ReadInt32();
                if (c < 0 || c >= 1 << 26) { throw r.Error("invalid pair constraint"); }
                if (c >= maxCons) { maxCons = c + 1; }
                chains.Add(c);
            }
        }

        int nf = r.ReadCount(4, "free pairs");
        var free = new int[Math.Max(nf, 1)];
        r.ReadInt32s(free.AsSpan(0, nf));
        for (int i = 0; i < nf; i++)
        {
            if (free[i] < 0 || free[i] >= np || pl[free[i]]) { throw r.Error("invalid free pair"); }
        }

        JointData3D[]? joints = null;
        ServiceBlock? services = null;
        CharacterBlock? characters = null;
        VehicleBlock? vehicles = null;
        RagdollBlock? ragdolls = null;
        FluidBlock? fluidBlock = null;
        if (v == VersionJoints)
        {
            uint flags = r.ReadUInt32();
            if (flags == 0 || (flags & ~(BlockJoints | BlockServices | BlockCharacters | BlockVehicles | BlockVehicleKinds | BlockRagdolls | BlockFluids)) != 0) { throw r.Error("invalid 3D section block flags " + flags.ToString(CultureInfo.InvariantCulture)); }
            if ((flags & BlockJoints) != 0) { joints = ReadJoints(r); }
            if ((flags & BlockServices) != 0)
            {
                services = ReadServices(r, ns, live);
                if (services.IsDefault()) { throw r.Error("a services block holding only default state"); }
            }

            if ((flags & BlockCharacters) != 0)
            {
                characters = ReadCharacters(r, nb);
                if (characters.Handles.Count == 0) { throw r.Error("a characters block holding no characters"); }
            }

            if ((flags & BlockVehicles) != 0)
            {
                vehicles = ReadVehicles(r, nb);
                if (vehicles.Handles.Count == 0) { throw r.Error("a vehicles block holding no vehicles"); }
            }

            if ((flags & BlockVehicleKinds) != 0)
            {
                if (vehicles == null) { throw r.Error("a vehicle kinds block without a vehicles block"); }
                ReadVehicleKinds(r, vehicles);
            }

            if ((flags & BlockRagdolls) != 0)
            {
                ragdolls = ReadRagdolls(r, nb);
                if (ragdolls.Handles.Count == 0) { throw r.Error("a ragdolls block holding no ragdolls"); }
            }

            if ((flags & BlockFluids) != 0) { fluidBlock = ReadFluids(r, nb); }
        }

        // A constraint belongs to at most one chain, once (a repeated id would make a cycle that the writer and the solver would walk forever).
        var inChain = new bool[maxCons];
        foreach (int c in chains)
        {
            if (inChain[c]) { throw r.Error("a constraint appears twice in the pair chains"); }
            inChain[c] = true;
        }

        // Resolve chains: the first constraint heads the pair, the rest link through ConsNext. Constraint ids are checked against the core afterwards.
        var consNext = new int[maxCons];
        Array.Fill(consNext, -1);
        int cursor = 0;
        for (int p = 0; p < np; p++)
        {
            if (!pl[p]) { continue; }
            int n = chainLen[p];
            pc[p] = n > 0 ? chains[cursor] : -1;
            for (int i = 0; i + 1 < n; i++) { consNext[chains[cursor + i]] = chains[cursor + i + 1]; }
            cursor += n;
        }

        // Commit.
        Opt.ContactHertz = hertz; Opt.ContactDampingRatio = zeta; Opt.PushOutVelocity = push; Opt.RestitutionThreshold = thr;
        Opt.MaxLinearSpeed = maxSpeed; Opt.MaxRotationPerSubstep = maxRot; Opt.ContinuousEnabled = continuous; Opt.ContactSlop = slop; Opt.SpeculativeDistance = spec;
        LocalCenter = lc.Length > 0 ? lc : new float[3]; Mass = mass; Inertia = inertia.Length > 0 ? inertia : new float[6]; MaxExtent = maxE; MinExtent = minE;
        LinearDamping = ld; AngularDamping = ad; GravityScale = gs; Force = force.Length > 0 ? force : new float[3]; Torque = torque.Length > 0 ? torque : new float[3];
        BodyFlags = bf; ShapeHead = head; ShapeTail = tail; Dp = dp.Length > 0 ? dp : new float[3];
        EnsureBodies(Math.Max(nb, 1));
        Geoms.Clear();
        Geoms.AddRange(geoms);
        _geomIndex.Clear();
        for (int i = 0; i < Geoms.Count; i++)
        {
            if (Geoms[i] != null) { _geomIndex[new RecipeKey(Geoms[i]!.Recipe)] = i; }
        }

        GeomOf = geomOf; Local = local; Proxy = proxy; Friction = fr; Restitution = re; Rolling = ro; Density = de; Category = cat; Mask = mask; Group = grp; NextShape = next;
        EnsureShapes(Math.Max(ns, 1));
        PairA = pa; PairB = pb; PairCons = pc; PairLive = pl; PairHigh = np; PairFree = free; PairFreeCount = nf; ConsNext = consNext;
        CommitServices(services, ns);
        CommitCharacters(characters);
        CommitVehicles(vehicles);
        CommitRagdolls(ragdolls);
        CommitFluids(fluidBlock);
        if (joints != null) { Opt.JointHertz = _pendingJointHertz; Opt.JointDampingRatio = _pendingJointDamping; }
        Joints = joints ?? new JointData3D[16];
        JointForce = new float[Joints.Length]; JointTorque = new float[Joints.Length];
        _needBind = true;
        _skip = new SkipFrame3[8];
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
        if (ShapeHead.Length < bodies.HighWater || GeomOf.Length < shapes.HighWater) { return false; }
        var cons = Core.Constraints;
        if (!JointsConsistentWithCore() || !CharactersConsistent() || !VehiclesConsistent() || !RagdollsConsistent() || !FluidsConsistent()) { return false; }
        for (int s = 0; s < shapes.HighWater; s++)
        {
            if (!shapes.Handles.IsLive(s)) { continue; }
            int owner = shapes.Owner[s];
            if (!bodies.IsLive(owner) || !Bp.IsProxy(Proxy[s]) || Bp.GetUserId(Proxy[s]) != s) { return false; }
            if (Geoms[GeomOf[s]]!.Soup != null && bodies.TypeOf(owner) != BodyType.Static) { return false; }
        }

        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            // A body's rotation is a unit quaternion (the integrator keeps it so); anything else would break the narrowphase's input contract.
            var q = bodies.RotationOf(b);
            float ql = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (!(ql >= 0.999f && ql <= 1.001f)) { return false; }
            int count = 0;
            for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
            {
                if (!shapes.Handles.IsLive(s) || shapes.Owner[s] != b || ++count > shapes.HighWater) { return false; }
            }
        }

        // Every contact constraint belongs to exactly one pair chain, sorted by sub-shape pair.
        int seen = 0;
        for (int p = 0; p < PairHigh; p++)
        {
            if (!PairLive[p]) { continue; }
            int steps = 0, prevA = -1, prevB = -1;
            for (int c = PairCons[p]; c >= 0; c = c < ConsNext.Length ? ConsNext[c] : -1)
            {
                if (!cons.IsLive(c) || (cons.KindOf(c) != ConstraintKind.Contact && cons.KindOf(c) != ConstraintKind.Sensor) || cons.ShapeA[c] != PairA[p] || cons.ShapeB[c] != PairB[p] || ++steps > cons.HighWater) { return false; }
                int sa = cons.SubShapeA[c], sb = cons.SubShapeB[c];
                if (sa < prevA || (sa == prevA && sb <= prevB)) { return false; }
                prevA = sa; prevB = sb;
            }

            seen += steps;
        }

        int contacts = 0;
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (cons.IsLive(c) && (cons.KindOf(c) == ConstraintKind.Contact || cons.KindOf(c) == ConstraintKind.Sensor)) { contacts++; }
        }

        return contacts == seen;
    }
}
