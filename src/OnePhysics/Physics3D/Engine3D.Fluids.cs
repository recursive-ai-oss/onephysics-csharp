using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Polyhedra for one geometry, built the first time a fluid touches it (derived data: never saved, rebuilt after a restore).</summary>
internal sealed class BuoyancyCache
{
    public BuoyancyMesh?[] Meshes = Array.Empty<BuoyancyMesh?>();
    public bool[] Built = Array.Empty<bool>();
}

internal sealed partial class Engine3D
{
    private const uint BlockFluids = 32;
    private const int BuoyancyOneBits = 0x3F800000;   // the bits of 1f: a stored 0 means "factor 1"

    /// <summary>Slots of the fluids (handles with generations, lowest free slot first).</summary>
    public HandleTable FluidHandles = new();

    public FluidDef?[] Fluids = new FluidDef?[4];

    private int[] _buoyBits = new int[8];
    private readonly BuoyancyCutter _cutter = new();

    public float BuoyancyFactorOf(int b) => b < _buoyBits.Length ? BitConverter.Int32BitsToSingle(_buoyBits[b] ^ BuoyancyOneBits) : 1f;

    public void SetBuoyancyFactor(int b, float factor)
    {
        if (b >= _buoyBits.Length) { Array.Resize(ref _buoyBits, Math.Max(b + 1, _buoyBits.Length * 2)); }
        _buoyBits[b] = BitConverter.SingleToInt32Bits(factor) ^ BuoyancyOneBits;
    }

    /// <summary>Called when a body slot is created or destroyed: a recycled slot starts with factor 1.</summary>
    private void ResetBuoyancy(int b)
    {
        if (b < _buoyBits.Length) { _buoyBits[b] = 0; }
    }

    private bool HasFluidState()
    {
        if (FluidHandles.Count > 0) { return true; }
        int hw = Math.Min(Core.Bodies.HighWater, _buoyBits.Length);
        for (int b = 0; b < hw; b++)
        {
            if (_buoyBits[b] != 0 && Core.Bodies.IsLive(b)) { return true; }
        }

        return false;
    }

    /// <summary>Moves every fluid with the world origin: the surface plane and regions shrink by the shift.</summary>
    public void ShiftFluids(double dx, double dy, double dz)
    {
        for (int i = 0; i < FluidHandles.HighWater; i++)
        {
            if (!FluidHandles.IsLive(i)) { continue; }
            var d = Fluids[i]!;
            var n = d.SurfaceNormal;
            d.SurfaceOffset -= n.X * dx + n.Y * dy + n.Z * dz;
            d.RegionMin = new Vector3D(d.RegionMin.X - dx, d.RegionMin.Y - dy, d.RegionMin.Z - dz);
            d.RegionMax = new Vector3D(d.RegionMax.X - dx, d.RegionMax.Y - dy, d.RegionMax.Z - dz);
        }
    }

    public void WakeAllDynamic()
    {
        var bodies = Core.Bodies;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (bodies.IsLive(b) && bodies.IsDynamic(b)) { Core.WakeIsland(bodies.Island[b]); bodies.SleepTime[b] = 0f; }
        }
    }

    /// <summary>Validates a definition and returns the copy the world keeps: the surface normal made unit length (and the offset scaled with it).</summary>
    public static FluidDef Prepare(FluidDef def)
    {
        def.Validate();
        var d = def.Clone();
        var n = d.SurfaceNormal;
        double len = Math.Sqrt((double)n.X * n.X + (double)n.Y * n.Y + (double)n.Z * n.Z);
        d.SurfaceNormal = new Vector3F((float)(n.X / len), (float)(n.Y / len), (float)(n.Z / len));
        d.SurfaceOffset = d.SurfaceOffset / len;
        // The offset is stored for a unit normal, so a very short normal scales it up; the snapshot reader holds the stored value to the same bound.
        if (!double.IsFinite(d.SurfaceOffset) || Math.Abs(d.SurfaceOffset) > FluidDef.MaxCoordinate)
        {
            throw new ArgumentOutOfRangeException(nameof(def), def.SurfaceOffset, "SurfaceOffset " + def.SurfaceOffset.ToString("R", CultureInfo.InvariantCulture) + " divided by the length of SurfaceNormal " + len.ToString("R", CultureInfo.InvariantCulture) + " must be finite and within ±" + FluidDef.MaxCoordinate.ToString("R", CultureInfo.InvariantCulture) + ".");
        }

        return d;
    }

    public Fluid AddFluid(FluidDef def)
    {
        var d = Prepare(def);
        int i = FluidHandles.Allocate();
        if (i >= Fluids.Length) { Array.Resize(ref Fluids, Math.Max(i + 1, Fluids.Length * 2)); }
        Fluids[i] = d;
        WakeAllDynamic();
        return new Fluid(World!, i, FluidHandles.GenerationOf(i));
    }

    // ------------------------------------------------------------------ the pre-solve stage

    /// <summary>
    /// Adds each awake dynamic body's buoyancy and drag to its force and torque for the coming step. Bodies are visited in ascending slot order and, for each, fluids in ascending slot order, shapes in
    /// creation order and compound children in index order, so the sums are the same on every platform and thread count. Sleeping bodies are left alone (they are at rest in equilibrium or on the bed);
    /// changing a fluid wakes them.
    /// </summary>
    public void StepFluids(float dt)
    {
        if (FluidHandles.Count == 0) { return; }
        var bodies = Core.Bodies;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsLive(b) || !bodies.IsDynamic(b) || !bodies.IsAwake(b)) { continue; }
            for (int f = 0; f < FluidHandles.HighWater; f++)
            {
                if (FluidHandles.IsLive(f)) { ApplyFluidToBody(b, Fluids[f]!, dt); }
            }
        }
    }

    /// <summary>Adds one fluid's buoyancy and drag on one body to its force and torque; returns the submerged volume (0 when clear of the fluid).</summary>
    public float ApplyFluidToBody(int b, FluidDef fluid, float dt)
    {
        var bodies = Core.Bodies;
        var pos = bodies.PositionOf(b);
        var n = fluid.SurfaceNormal;
        double depth = n.X * pos[0] + n.Y * pos[1] + n.Z * pos[2] - fluid.SurfaceOffset;
        float h0 = (float)depth;
        float reach = MaxExtent[b];
        if (!(h0 < reach)) { return 0f; }
        if (fluid.LimitToRegion)
        {
            double r = reach;
            if (pos[0] + r < fluid.RegionMin.X || pos[0] - r > fluid.RegionMax.X || pos[1] + r < fluid.RegionMin.Y || pos[1] - r > fluid.RegionMax.Y
                || pos[2] + r < fluid.RegionMin.Z || pos[2] - r > fluid.RegionMax.Z) { return 0f; }
        }

        var lin = bodies.LinearVelocityOf(b);
        var rel = new Vector3F(lin[0] - fluid.Flow.X, lin[1] - fluid.Flow.Y, lin[2] - fluid.Flow.Z);
        float speed = float.Sqrt(rel.X * rel.X + rel.Y * rel.Y + rel.Z * rel.Z);
        var dir = speed > 1e-6f ? new Vector3F(rel.X / speed, rel.Y / speed, rel.Z / speed) : default;

        var q = RotationOfBody(b);
        var lc = LocalCenterOf(b);
        var g = Opt.Gravity;
        _fvolume = 0f; _farea = 0f; _fforce = default; _ftorque = default;
        // Buoyancy per unit volume is density * factor * (minus gravity).
        float k = fluid.Density * BuoyancyFactorOf(b);
        var up = new Vector3F(-g.X * k, -g.Y * k, -g.Z * k);

        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (IsSensor(s)) { continue; }
            var geom = Geoms[GeomOf[s]]!;
            if (geom.Kind != GeomKind.Convex && geom.Kind != GeomKind.Compound) { continue; }
            var l = Local[s];
            var pose = new RigidTransformF(q * l.Rotation, q.Rotate(l.Translation - lc));
            var cache = geom.Buoyancy ??= new BuoyancyCache();
            if (geom.Kind == GeomKind.Convex)
            {
                AddPiece(geom.Convex, cache, 0, 1, pose, n, h0, dir, up);
            }
            else
            {
                var comp = geom.Compound!;
                for (int i = 0; i < comp.SlotCount; i++)
                {
                    if (!comp.IsLive(i)) { continue; }
                    var child = comp.GetChild(i);
                    var cp = new RigidTransformF(pose.Rotation * child.Local.Rotation, pose.Rotation.Rotate(child.Local.Translation) + pose.Translation);
                    AddPiece(child.Shape, cache, i, comp.SlotCount, cp, n, h0, dir, up);
                }
            }
        }

        if (!(_fvolume > 0f)) { return 0f; }
        var force = _fforce;
        var torque = _ftorque;

        // Quadratic drag against the flow (Cd * 1/2 * density * A * |v|^2), through the centre of mass; never more than stops the body within the step.
        if (speed > 0f && (fluid.LinearDrag > 0f || fluid.ViscousDamping > 0f))
        {
            float mag = 0.5f * fluid.Density * fluid.LinearDrag * _farea * speed * speed + fluid.ViscousDamping * fluid.Density * _fvolume * speed;
            float cap = Mass[b] * speed / dt;
            if (mag > cap) { mag = cap; }
            force = new Vector3F(force.X - dir.X * mag, force.Y - dir.Y * mag, force.Z - dir.Z * mag);
        }

        var av = bodies.AngularVelocityOf(b);
        float w2 = av[0] * av[0] + av[1] * av[1] + av[2] * av[2];
        if (w2 > 0f && (fluid.AngularDrag > 0f || fluid.ViscousDamping > 0f))
        {
            float w = float.Sqrt(w2);
            float cb = Math32.Cbrt(_fvolume);
            float cb2 = cb * cb;
            float cb5 = cb2 * cb2 * cb;
            float mag = 0.5f * fluid.Density * fluid.AngularDrag * cb5 * w2 + fluid.ViscousDamping * fluid.Density * cb5 * w;
            var t = new Vector3F(-av[0] / w * mag, -av[1] / w * mag, -av[2] / w * mag);
            RefreshWorldInertia(b);
            var dw = ApplyWorldInertia(b, t);
            float dl = float.Sqrt(dw.X * dw.X + dw.Y * dw.Y + dw.Z * dw.Z) * dt;
            if (dl > w)
            {
                float scale = w / dl;
                t = new Vector3F(t.X * scale, t.Y * scale, t.Z * scale);
            }

            torque = new Vector3F(torque.X + t.X, torque.Y + t.Y, torque.Z + t.Z);
        }

        Force[3 * b] += force.X; Force[3 * b + 1] += force.Y; Force[3 * b + 2] += force.Z;
        Torque[3 * b] += torque.X; Torque[3 * b + 1] += torque.Y; Torque[3 * b + 2] += torque.Z;
        return _fvolume;
    }

    private float _fvolume, _farea;
    private Vector3F _fforce, _ftorque;

    /// <summary>Cuts one convex piece (a shape or a compound child) and adds its buoyancy, at its submerged centroid, to the running totals.</summary>
    private void AddPiece(in ConvexShape3D shape, BuoyancyCache cache, int slot, int slots, in RigidTransformF pose, Vector3F n, float h0, Vector3F dir, Vector3F up)
    {
        CutResult r;
        if (shape.Kind == ConvexShapeKind3D.Sphere)
        {
            if (!Buoyancy3D.CutSphere(shape.ConvexRadius, pose.Translation, n, h0, out r)) { return; }
        }
        else
        {
            if (cache.Meshes.Length < slots) { Array.Resize(ref cache.Meshes, slots); Array.Resize(ref cache.Built, slots); }
            if (!cache.Built[slot]) { cache.Meshes[slot] = BuoyancyMesh.For(shape); cache.Built[slot] = true; }
            var mesh = cache.Meshes[slot];
            if (mesh == null || !_cutter.Cut(mesh, pose, n, h0, dir, out r)) { return; }
        }

        _fvolume += r.Volume;
        _farea += r.ProjectedArea;
        var fb = new Vector3F(up.X * r.Volume, up.Y * r.Volume, up.Z * r.Volume);
        _fforce = new Vector3F(_fforce.X + fb.X, _fforce.Y + fb.Y, _fforce.Z + fb.Z);
        var c = r.Centroid;
        _ftorque = new Vector3F(_ftorque.X + (c.Y * fb.Z - c.Z * fb.Y), _ftorque.Y + (c.Z * fb.X - c.X * fb.Z), _ftorque.Z + (c.X * fb.Y - c.Y * fb.X));
    }

    // ------------------------------------------------------------------ snapshot block

    /// <summary>Bytes of one fluid's record: normal 12, offset 8, density 4, flow 12, drag, angular drag and damping 12, region flag 1, region corners 48.</summary>
    private const int FluidBytes = 12 + 8 + 4 + 12 + 12 + 1 + 48;

    private void WriteFluids(StateWriter w)
    {
        FluidHandles.Write(w);
        for (int i = 0; i < FluidHandles.HighWater; i++)
        {
            if (!FluidHandles.IsLive(i)) { continue; }
            var d = Fluids[i]!;
            w.WriteSingle(d.SurfaceNormal.X); w.WriteSingle(d.SurfaceNormal.Y); w.WriteSingle(d.SurfaceNormal.Z);
            w.WriteDouble(d.SurfaceOffset);
            w.WriteSingle(d.Density);
            w.WriteSingle(d.Flow.X); w.WriteSingle(d.Flow.Y); w.WriteSingle(d.Flow.Z);
            w.WriteSingle(d.LinearDrag); w.WriteSingle(d.AngularDrag); w.WriteSingle(d.ViscousDamping);
            w.WriteBool(d.LimitToRegion);
            w.WriteDouble(d.RegionMin.X); w.WriteDouble(d.RegionMin.Y); w.WriteDouble(d.RegionMin.Z);
            w.WriteDouble(d.RegionMax.X); w.WriteDouble(d.RegionMax.Y); w.WriteDouble(d.RegionMax.Z);
        }

        // Buoyancy factors that differ from 1, by ascending body slot.
        int hw = Math.Min(Core.Bodies.HighWater, _buoyBits.Length);
        int count = 0;
        for (int b = 0; b < hw; b++) { if (_buoyBits[b] != 0 && Core.Bodies.IsLive(b)) { count++; } }
        w.WriteInt32(count);
        for (int b = 0; b < hw; b++)
        {
            if (_buoyBits[b] == 0 || !Core.Bodies.IsLive(b)) { continue; }
            w.WriteInt32(b);
            w.WriteSingle(BuoyancyFactorOf(b));
        }
    }

    private sealed class FluidBlock
    {
        public HandleTable Handles = new();
        public FluidDef?[] Defs = Array.Empty<FluidDef?>();
        public int[] FactorBody = Array.Empty<int>();
        public int[] FactorBits = Array.Empty<int>();
    }

    private static FluidBlock ReadFluids(StateReader r, int bodyCount)
    {
        var blk = new FluidBlock { Handles = HandleTable.Read(r, "fluid") };
        blk.Defs = new FluidDef?[Math.Max(blk.Handles.HighWater, 1)];
        for (int i = 0; i < blk.Handles.HighWater; i++)
        {
            if (!blk.Handles.IsLive(i)) { continue; }
            if (r.Remaining < FluidBytes) { throw r.Error("truncated fluid record"); }
            var d = new FluidDef();
            d.SurfaceNormal = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            d.SurfaceOffset = r.ReadDouble();
            d.Density = r.ReadSingle();
            d.Flow = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            d.LinearDrag = r.ReadSingle(); d.AngularDrag = r.ReadSingle(); d.ViscousDamping = r.ReadSingle();
            d.LimitToRegion = r.ReadBool();
            d.RegionMin = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble());
            d.RegionMax = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble());
            try { d.Validate(); }
            catch (ArgumentOutOfRangeException e) { throw r.Error("invalid fluid: " + e.Message); }
            // The world stores unit normals; a different length would make the saved offset mean something else.
            double len2 = (double)d.SurfaceNormal.X * d.SurfaceNormal.X + (double)d.SurfaceNormal.Y * d.SurfaceNormal.Y + (double)d.SurfaceNormal.Z * d.SurfaceNormal.Z;
            if (!(len2 > 0.998 && len2 < 1.002)) { throw r.Error("a fluid surface normal that is not a unit vector"); }
            blk.Defs[i] = d;
        }

        int n = r.ReadCount(8, "buoyancy factors");
        blk.FactorBody = new int[n]; blk.FactorBits = new int[n];
        int prev = -1;
        for (int k = 0; k < n; k++)
        {
            int body = r.ReadInt32();
            float factor = r.ReadSingle();
            if (body <= prev || body >= bodyCount) { throw r.Error("buoyancy factors out of order or for a body that does not exist"); }
            if (!(factor >= 0f && factor <= 1e6f) || factor == 1f) { throw r.Error("invalid buoyancy factor " + factor.ToString("R", CultureInfo.InvariantCulture)); }
            prev = body;
            blk.FactorBody[k] = body;
            blk.FactorBits[k] = BitConverter.SingleToInt32Bits(factor) ^ BuoyancyOneBits;
        }

        if (blk.Handles.Count == 0 && n == 0) { throw r.Error("a fluids block holding nothing"); }
        return blk;
    }

    private void CommitFluids(FluidBlock? blk)
    {
        Array.Clear(_buoyBits);
        if (blk == null)
        {
            FluidHandles.Reset();
            Array.Clear(Fluids);
            return;
        }

        FluidHandles = blk.Handles;
        Fluids = blk.Defs;
        for (int k = 0; k < blk.FactorBody.Length; k++)
        {
            if (blk.FactorBody[k] >= _buoyBits.Length) { Array.Resize(ref _buoyBits, Math.Max(blk.FactorBody[k] + 1, _buoyBits.Length * 2)); }
            _buoyBits[blk.FactorBody[k]] = blk.FactorBits[k];
        }
    }

    /// <summary>After a restore: a factor belongs to a live body.</summary>
    private bool FluidsConsistent()
    {
        for (int b = 0; b < _buoyBits.Length; b++)
        {
            if (_buoyBits[b] != 0 && !(b < Core.Bodies.HighWater && Core.Bodies.IsLive(b))) { return false; }
        }

        return true;
    }
}
