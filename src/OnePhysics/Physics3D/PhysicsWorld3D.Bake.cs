using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Bake;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    private BakeRecorder? _bake;

    /// <summary>
    /// Starts recording the world into a bake cache: the first frame is the world as it is now, and every later <see cref="Step"/> adds frames at the options' stride.
    /// Call <see cref="BakeRecorder.Finish()"/> for the file. Only one bake can run on a world at a time; a replay recording can run beside it.
    /// Reading the world for a frame does not change it, its events or its hash.
    /// </summary>
    /// <param name="options">What to record and how coarsely; defaults when null. Copied.</param>
    /// <exception cref="ArgumentOutOfRangeException">An option is out of range, or the world holds a value the format cannot store.</exception>
    /// <exception cref="InvalidOperationException">A bake is already running on this world, or this is called inside a step.</exception>
    public BakeRecorder BeginBake(BakeOptions? options = null)
    {
        if (_bake != null) { throw new InvalidOperationException("This world is already being baked."); }
        if (_stepping) { throw new InvalidOperationException("BeginBake cannot run inside a step or a callback."); }
        _core.FlushPostStep();
        var recorder = new BakeRecorder(new BakeSource3D(this), options ?? new BakeOptions(), () => _bake = null);
        _bake = recorder;
        return recorder;
    }

    /// <summary>
    /// Raises the world's quality settings for an offline bake (see <see cref="OfflineQualityOptions"/>): more substeps and iterations, no sleeping, surface collision on every soft body, finer fluid steps.
    /// A setting is only ever raised, never lowered. The changes are ordinary option and body changes, so a replay recording sees them.
    /// </summary>
    /// <param name="options">The targets; the defaults of <see cref="OfflineQualityOptions"/> when null.</param>
    /// <exception cref="ArgumentOutOfRangeException">A target is out of range.</exception>
    public void ApplyOfflineQuality(OfflineQualityOptions? options = null)
    {
        var q = options ?? new OfflineQualityOptions();
        q.Validate();
        var o = Options;
        if (o.Substeps < q.Substeps) { o.Substeps = q.Substeps; }
        if (o.VelocityIterations < q.VelocityIterations) { o.VelocityIterations = q.VelocityIterations; }
        if (o.RelaxIterations < q.RelaxIterations) { o.RelaxIterations = q.RelaxIterations; }
        if (o.ContactSubsteps < q.ContactSubsteps) { o.ContactSubsteps = q.ContactSubsteps; }
        if (o.ContactVelocityIterations < q.VelocityIterations) { o.ContactVelocityIterations = q.VelocityIterations; }
        if (o.ContactRelaxIterations < q.RelaxIterations) { o.ContactRelaxIterations = q.RelaxIterations; }
        if (o.SoftSubsteps < q.SoftSubsteps) { o.SoftSubsteps = q.SoftSubsteps; }
        if (o.SoftIterations < q.SoftIterations) { o.SoftIterations = q.SoftIterations; }
        if (q.DisableSleep && o.SleepEnabled) { o.SleepEnabled = false; }
        if (q.Continuous && !o.ContinuousEnabled) { o.ContinuousEnabled = true; }
        var f = FluidSettings;
        if (f.Substeps < q.FluidSubsteps || f.Iterations < q.FluidIterations)
        {
            f.Substeps = Math.Max(f.Substeps, q.FluidSubsteps); f.Iterations = Math.Max(f.Iterations, q.FluidIterations);
            FluidSettings = f;
        }

        if (q.SoftSurfaceCollision)
        {
            for (int i = 0; i < _engine.SoftHandles.HighWater; i++)
            {
                if (!_engine.SoftHandles.IsLive(i)) { continue; }
                var s = GetSoftBody(i);
                if (!s.SurfaceCollision) { s.SurfaceCollision = true; }
            }
        }
    }

    // The shape of a body as triangles in its own frame (hulls and boxes), for the fracture track.
    internal GeometrySample? BakeGeometryOf(Body body)
    {
        if (!body.IsValid) { return null; }
        var h = body.RawHandle;
        int b = h.Index;
        var origin = body.Position;
        var inv = body.Rotation.Conjugated();
        var verts = new List<float>();
        var tris = new List<int>();
        int count = 0;
        for (int s = _engine.ShapeHead[b]; s >= 0; s = _engine.NextShape[s])
        {
            var g = new DebugGeometry();
            AddDebugShape(g, s, h, _core.HandleOfShape(s), false);
            foreach (var shape in g.Shapes)
            {
                if (shape.IndexCount == 0) { continue; }
                for (int i = 0; i < shape.PointCount; i++)
                {
                    var p = g.Points[shape.FirstPoint + i];
                    var l = inv.Rotate(new Vector3F((float)(p.X - origin.X), (float)(p.Y - origin.Y), (float)(p.Z - origin.Z)));
                    verts.Add(l.X); verts.Add(l.Y); verts.Add(l.Z);
                }

                for (int i = 0; i < shape.IndexCount; i++) { tris.Add(count + g.Indices[shape.FirstIndex + i]); }
                count += shape.PointCount;
            }
        }

        return new GeometrySample { Slot = h.Index, Gen = h.Generation, Vertices = verts.ToArray(), Triangles = tris.ToArray() };
    }

    private sealed class BakeSource3D(PhysicsWorld3D w) : IBakeSource
    {
        private Vector3D[] _tmp = Array.Empty<Vector3D>();

        public int Dimension => 3;
        public double MetresPerUnit => w.LengthUnit.MetresPerUnit;
        public long StepCount => w.StepCount;
        public ulong Hash() => w.ComputeHash();

        public void ReadBodies(List<PoseSample> into)
        {
            foreach (var b in w.Bodies)
            {
                var h = b.RawHandle;
                var p = b.Position;
                var q = b.Rotation;
                into.Add(new PoseSample { Slot = h.Index, Gen = h.Generation, X = p.X, Y = p.Y, Z = p.Z, R0 = q.X, R1 = q.Y, R2 = q.Z, R3 = q.W });
            }
        }

        public void ReadSoft(List<PointSet> into)
        {
            var e = w._engine;
            for (int i = 0; i < e.SoftHandles.HighWater; i++)
            {
                if (!e.SoftHandles.IsLive(i)) { continue; }
                var s = w.GetSoftBody(i);
                int n = s.ParticleCount;
                if (_tmp.Length < n) { _tmp = new Vector3D[n]; }
                s.CopyPositions(_tmp.AsSpan(0, n));
                var p = new double[3 * n];
                for (int k = 0; k < n; k++) { p[3 * k] = _tmp[k].X; p[3 * k + 1] = _tmp[k].Y; p[3 * k + 2] = _tmp[k].Z; }
                into.Add(new PointSet { Slot = i, Gen = e.SoftHandles.GenerationOf(i), Kind = e.SoftData[i]!.Def.Fabric != null ? 1 : 0, N = n, P = p });
            }
        }

        public int[] SoftTriangles(int slot) => w.GetSoftBody(slot).SurfaceTriangles.ToArray();

        public void ReadRods(List<PointSet> into)
        {
            var store = w._engine.Rods;
            for (int i = 0; i < store.Data.Length; i++)
            {
                if (store.Data[i] == null) { continue; }
                var rod = new Rod(store, i, store.Generations[i]);
                int n = rod.ParticleCount;
                var p = new double[3 * n];
                for (int k = 0; k < n; k++) { var v = rod.GetPosition(k); p[3 * k] = v.X; p[3 * k + 1] = v.Y; p[3 * k + 2] = v.Z; }
                into.Add(new PointSet { Slot = i, Gen = store.Generations[i], Kind = 0, N = n, P = p });
            }
        }

        public void ReadParticles(List<ParticleSample> into)
        {
            int n = w.FluidParticleCount;
            if (n == 0) { return; }
            var all = new FluidParticleInfo[n];
            int got = w.GetFluidParticles(all);
            for (int i = 0; i < got; i++) { var p = all[i]; into.Add(new ParticleSample { Id = p.Id, Fluid = p.Fluid, X = p.Position.X, Y = p.Position.Y, Z = p.Position.Z }); }
        }

        public void ReadEvents(List<EventSample> into)
        {
            foreach (var e in w.Events)
            {
                into.Add(new EventSample
                {
                    Kind = (byte)e.Kind,
                    ASlot = e.BodyA.IsNull ? 0 : e.BodyA.Index + 1, AGen = e.BodyA.IsNull ? 0 : e.BodyA.Generation,
                    BSlot = e.BodyB.IsNull ? 0 : e.BodyB.Index + 1, BGen = e.BodyB.IsNull ? 0 : e.BodyB.Generation,
                    ValueBits = BitConverter.SingleToUInt32Bits(float.IsFinite(e.Value) ? e.Value : 0f),
                });
            }
        }

        public void ReadFractures(List<FractureSample> into)
        {
            var events = w.FractureEvents;
            var frags = w.FractureFragments;
            foreach (var f in events)
            {
                var list = new (int, int)[f.FragmentCount];
                var geo = new List<GeometrySample>();
                for (int i = 0; i < f.FragmentCount; i++)
                {
                    var b = frags[f.FirstFragment + i];
                    list[i] = (b.RawHandle.Index, b.RawHandle.Generation);
                    var g = w.BakeGeometryOf(b);
                    if (g != null) { geo.Add(g); }
                }

                into.Add(new FractureSample
                {
                    Kind = (byte)f.Kind, SourceSlot = f.Source.Index, SourceGen = f.Source.Generation, Impulse = f.Impulse,
                    PX = f.Point.X, PY = f.Point.Y, PZ = f.Point.Z, Fragments = list, Geometry = geo.ToArray(),
                });
            }
        }
    }
}
