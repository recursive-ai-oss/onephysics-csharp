using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>
/// Records a stepping world into a bake cache (docs/spec/bake.md). Start one with <c>world.BeginBake(options)</c>; the first frame is the world as it
/// is then, and every later <c>Step</c> adds frames at the options' stride. <see cref="Finish()"/> returns the file. The bytes are a
/// function of the world's history and the options only: the same on every platform, width and thread count.
/// Recording is offline work: it allocates while it runs and is not meant for a game's frame budget.
/// </summary>
public sealed class BakeRecorder : IDisposable
{
    private readonly IBakeSource _source;
    private readonly BakeOptions _o;
    private readonly Action _detach;
    private readonly long _startStep;
    private long _lastStep;
    private bool _active = true, _faulted;
    private string? _fault;
    private int _frames;
    private readonly List<ulong> _hashes = new();
    private readonly double _posScale, _vertScale, _rotScale;
    private readonly int _dims;

    private readonly TrackBuilder _rigid = new(), _soft = new(), _rods = new(), _parts = new();
    private readonly List<(int Step, EventSample E)> _events = new();
    private readonly List<(int Step, FractureSample F)> _fractures = new();

    private readonly List<PoseSample> _poses = new();
    private readonly List<PointSet> _sets = new();
    private readonly List<ParticleSample> _particles = new();
    private readonly List<EventSample> _evScratch = new();
    private readonly List<FractureSample> _frScratch = new();
    private long[] _vec = new long[16];

    private sealed class TrackBuilder
    {
        internal readonly Dictionary<long, int> Lookup = new();
        internal readonly List<BakeEntityEncoder> All = new();
        internal List<BakeEntityEncoder> Active = new();
    }

    internal BakeRecorder(IBakeSource source, BakeOptions options, Action detach)
    {
        options.Validate();
        _source = source; _o = options.Copy(); _detach = detach;
        _dims = source.Dimension;
        _posScale = BakeFormat.Pow2(-_o.PositionExponent);
        _vertScale = BakeFormat.Pow2(-_o.VertexExponent);
        _rotScale = BakeFormat.Pow2(_o.RotationBits);
        _startStep = _lastStep = source.StepCount;
        CaptureFrame();
    }

    /// <summary>Frames recorded so far (the first frame is recorded when the bake starts).</summary>
    public int FrameCount => _frames;

    /// <summary>True until <see cref="Finish()"/> or <see cref="Dispose"/>.</summary>
    public bool IsActive => _active;

    /// <summary>The SHA-256 of a replay file (or any bytes), for <see cref="BakeOptions.ReplayDigest"/>.</summary>
    public static byte[] DigestOf(ReadOnlySpan<byte> bytes) => SHA256.HashData(bytes);

    internal void AfterStep(float dt)
    {
        if (!_active || _faulted) { return; }
        long step = _source.StepCount;
        if (dt != _o.Dt) { Fault("a step used dt " + dt.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + " but the bake was started for " + _o.Dt.ToString("R", System.Globalization.CultureInfo.InvariantCulture)); return; }
        if (step != _lastStep + 1) { Fault("the world's step count moved from " + _lastStep + " to " + step + " (a restore or a fork during a bake)"); return; }
        _lastStep = step;
        int rel = checked((int)(step - _startStep));
        if ((_o.Tracks & BakeTracks.Events) != 0)
        {
            _evScratch.Clear();
            _source.ReadEvents(_evScratch);
            foreach (var e in _evScratch) { if ((_o.EventKinds & (EventMask)(1 << e.Kind)) != 0) { _events.Add((rel, e)); } }
        }

        if ((_o.Tracks & BakeTracks.Fracture) != 0)
        {
            _frScratch.Clear();
            _source.ReadFractures(_frScratch);
            foreach (var f in _frScratch) { _fractures.Add((rel, f)); }
        }

        if (rel % _o.FrameStride == 0) { CaptureFrame(); }
    }

    private void Fault(string why) { _faulted = true; _fault = why; }

    private void CaptureFrame()
    {
        int f = _frames;
        if (f >= BakeFormat.MaxFrames) { Fault("more than " + BakeFormat.MaxFrames + " frames"); return; }
        if ((_o.Tracks & BakeTracks.Rigid) != 0) { CaptureRigid(f); }
        if ((_o.Tracks & BakeTracks.Soft) != 0) { _sets.Clear(); _source.ReadSoft(_sets); CapturePoints(_soft, f, soft: true); }
        if ((_o.Tracks & BakeTracks.Rods) != 0) { _sets.Clear(); _source.ReadRods(_sets); CapturePoints(_rods, f, soft: false); }
        if ((_o.Tracks & BakeTracks.Particles) != 0) { CaptureParticles(f); }
        _hashes.Add(_source.Hash());
        _frames = f + 1;
    }

    private Span<long> Vec(int n)
    {
        if (_vec.Length < n) { _vec = new long[Math.Max(n, _vec.Length * 2)]; }
        return _vec.AsSpan(0, n);
    }

    private BakeEntityEncoder Entity(TrackBuilder t, int key, int gen, int kind, int n, int frame, int len, int split)
    {
        long id = ((long)key << 32) | (uint)gen;
        if (t.Lookup.TryGetValue(id, out int ix))
        {
            var e = t.All[ix];
            // The same thing seen again after a gap is a new history.
            if (e.LastSeen == frame - 1 && e.N == n) { return e; }
        }

        var ne = new BakeEntityEncoder(key, gen, kind, n, frame, len, split, _o.BlockFrames);
        t.Lookup[id] = t.All.Count;
        t.All.Add(ne);
        t.Active.Add(ne);
        return ne;
    }

    private static void Retire(TrackBuilder t, int frame)
    {
        // Entities not seen this frame are finished; walking the list in creation order keeps the output order fixed.
        var keep = new List<BakeEntityEncoder>(t.Active.Count);
        foreach (var e in t.Active)
        {
            if (e.LastSeen == frame) { keep.Add(e); } else { e.Finish(); }
        }

        t.Active = keep;
    }

    private void CaptureRigid(int f)
    {
        _poses.Clear();
        _source.ReadBodies(_poses);
        int rc = _dims == 3 ? 4 : 2, dp = _dims;
        foreach (var p in _poses)
        {
            var e = Entity(_rigid, p.Slot, p.Gen, 0, 1, f, dp + rc, dp);
            var v = Vec(dp + rc);
            v[0] = BakeFormat.Quantize(p.X, _posScale, "body position x");
            v[1] = BakeFormat.Quantize(p.Y, _posScale, "body position y");
            if (_dims == 3) { v[2] = BakeFormat.Quantize(p.Z, _posScale, "body position z"); }
            float r0 = p.R0, r1 = p.R1, r2 = p.R2, r3 = p.R3;
            if (!(float.IsFinite(r0) && float.IsFinite(r1) && float.IsFinite(r2) && float.IsFinite(r3))) { throw new BakeRangeException("body rotation", double.NaN); }
            if (_dims == 3 && e.PrevRot != null)
            {
                // Keep q and -q (the same rotation) on the side nearest the last stored one, so frames interpolate the short way and residuals stay small.
                double d = (double)r0 * e.PrevRot[0] + (double)r1 * e.PrevRot[1] + (double)r2 * e.PrevRot[2] + (double)r3 * e.PrevRot[3];
                if (d < 0) { r0 = -r0; r1 = -r1; r2 = -r2; r3 = -r3; }
            }

            e.PrevRot ??= new float[4];
            e.PrevRot[0] = r0; e.PrevRot[1] = r1; e.PrevRot[2] = r2; e.PrevRot[3] = r3;
            v[dp] = QuantRot(r0); v[dp + 1] = QuantRot(r1);
            if (_dims == 3) { v[dp + 2] = QuantRot(r2); v[dp + 3] = QuantRot(r3); }
            e.Append(v);
            e.LastSeen = f;
        }

        Retire(_rigid, f);
    }

    private long QuantRot(float c) => _o.RotationBits == 0 ? BitConverter.SingleToInt32Bits(c) : BakeFormat.Quantize(c, _rotScale, "body rotation");

    private void CapturePoints(TrackBuilder t, int f, bool soft)
    {
        foreach (var s in _sets)
        {
            int len = s.N * _dims;
            if ((long)len * _o.BlockFrames > BakeClip.MaxDecodedIntegers) { Fault("a " + s.N + "-point body needs BlockFrames of at most " + (BakeClip.MaxDecodedIntegers / len)); return; }
            var e = Entity(t, s.Slot, s.Gen, s.Kind, s.N, f, len, 0);
            if (soft && e.Frames == 0) { e.Triangles = _source.SoftTriangles(s.Slot); }
            var v = Vec(len);
            for (int i = 0; i < len; i++) { v[i] = BakeFormat.Quantize(s.P[i], _vertScale, soft ? "soft body particle position" : "rod particle position"); }
            e.Append(v);
            e.LastSeen = f;
        }

        Retire(t, f);
    }

    private void CaptureParticles(int f)
    {
        _particles.Clear();
        _source.ReadParticles(_particles);
        _particles.Sort(static (a, b) => a.Id.CompareTo(b.Id));
        foreach (var p in _particles)
        {
            var e = Entity(_parts, p.Id, 0, p.Fluid, 1, f, _dims, 0);
            var v = Vec(_dims);
            v[0] = BakeFormat.Quantize(p.X, _vertScale, "fluid particle position x");
            v[1] = BakeFormat.Quantize(p.Y, _vertScale, "fluid particle position y");
            if (_dims == 3) { v[2] = BakeFormat.Quantize(p.Z, _vertScale, "fluid particle position z"); }
            e.Append(v);
            e.LastSeen = f;
        }

        Retire(_parts, f);
    }

    /// <summary>Ends the recording and returns the file. The recorder detaches from the world; the world is untouched.</summary>
    /// <exception cref="InvalidOperationException">The recording is faulted (a step with another dt, or a restore during it) or already finished.</exception>
    public byte[] Finish()
    {
        if (!_active) { throw new InvalidOperationException("The bake is already finished."); }
        Detach();
        if (_faulted) { throw new InvalidOperationException("The bake is faulted: " + _fault + "."); }
        return Serialize();
    }

    /// <summary>Ends the recording and writes the file to <paramref name="destination"/>, which stays open.</summary>
    /// <exception cref="InvalidOperationException">As <see cref="Finish()"/>.</exception>
    public void Finish(Stream destination)
    {
        ArgumentNullException.ThrowIfNull(destination);
        var bytes = Finish();
        destination.Write(bytes, 0, bytes.Length);
    }

    /// <summary>Stops recording and discards everything.</summary>
    public void Dispose() { if (_active) { Detach(); } }

    private void Detach() { _active = false; _detach(); }

    private byte[] Serialize()
    {
        var w = new ByteSink(1 << 16);
        w.U32(BakeFormat.Magic); w.U16(BakeFormat.Version); w.U16(0);
        w.U8(_dims); w.U8((byte)(sbyte)_o.PositionExponent); w.U8((byte)(sbyte)_o.VertexExponent); w.U8(_o.RotationBits);
        w.U32((uint)_frames); w.U32((uint)_o.FrameStride); w.U32((uint)_o.BlockFrames);
        w.F32(_o.Dt); w.U32(0);
        w.I64(_startStep);
        w.Bytes(_o.ReplayDigest ?? new byte[32]);
        w.Str(_o.SceneName);
        w.Str(_o.LibraryVersion ?? VersionText());

        var hs = new ByteSink(_frames * 8 + 8);
        foreach (var h in _hashes) { hs.U64(h); }
        Chunk(w, BakeFormat.TagHashes, hs);
        if ((_o.Tracks & BakeTracks.Rigid) != 0) { Chunk(w, BakeFormat.TagRigid, EntityChunk(_rigid, BakeFormat.TagRigid)); }
        if ((_o.Tracks & BakeTracks.Soft) != 0) { Chunk(w, BakeFormat.TagSoft, EntityChunk(_soft, BakeFormat.TagSoft)); }
        if ((_o.Tracks & BakeTracks.Rods) != 0) { Chunk(w, BakeFormat.TagRods, EntityChunk(_rods, BakeFormat.TagRods)); }
        if ((_o.Tracks & BakeTracks.Particles) != 0) { Chunk(w, BakeFormat.TagParticles, EntityChunk(_parts, BakeFormat.TagParticles)); }
        if ((_o.Tracks & BakeTracks.Fracture) != 0) { Chunk(w, BakeFormat.TagFracture, FractureChunk()); }
        if ((_o.Tracks & BakeTracks.Events) != 0) { Chunk(w, BakeFormat.TagEvents, EventChunk()); }
        ulong sum = BakeFormat.Checksum(w.Span);
        var end = new ByteSink(8);
        end.U64(sum);
        Chunk(w, BakeFormat.TagEnd, end);
        return w.ToArray();
    }

    private static string VersionText()
    {
        var v = typeof(BakeRecorder).Assembly.GetName().Version;
        return v == null ? "" : v.Major.ToString(System.Globalization.CultureInfo.InvariantCulture) + "." + v.Minor.ToString(System.Globalization.CultureInfo.InvariantCulture) + "." + Math.Max(v.Build, 0).ToString(System.Globalization.CultureInfo.InvariantCulture);
    }

    private static void Chunk(ByteSink w, uint tag, ByteSink payload)
    {
        if (payload.Length > int.MaxValue - 4096 || w.Length > int.MaxValue - payload.Length - 16) { throw new InvalidOperationException("The bake is larger than 2 GiB; raise the stride or the quanta."); }
        w.U32(tag); w.U32((uint)payload.Length); w.Bytes(payload.Span);
    }

    private static ByteSink EntityChunk(TrackBuilder t, uint tag)
    {
        foreach (var e in t.All) { e.Finish(); }
        var s = new ByteSink(1 << 14);
        s.Var((ulong)t.All.Count);
        foreach (var e in t.All)
        {
            s.Var((ulong)e.Key); s.Var((ulong)e.Gen); s.Var((ulong)e.FirstFrame); s.Var((ulong)e.Frames);
            if (tag == BakeFormat.TagSoft)
            {
                s.U8(e.Kind); s.Var((ulong)e.N);
                var tris = e.Triangles ?? Array.Empty<int>();
                s.Var((ulong)(tris.Length / 3));
                foreach (int i in tris) { s.Var((ulong)i); }
            }
            else if (tag == BakeFormat.TagRods) { s.Var((ulong)e.N); }
            else if (tag == BakeFormat.TagParticles) { s.Var((ulong)e.Kind); }
            s.Var((ulong)e.Blocks.Count);
            foreach (var b in e.Blocks) { s.Var((ulong)b.Length); }
            foreach (var b in e.Blocks) { s.Bytes(b); }
        }

        return s;
    }

    private ByteSink FractureChunk()
    {
        var s = new ByteSink(1 << 12);
        s.Var((ulong)_fractures.Count);
        var geometry = new List<GeometrySample>();
        foreach (var (step, f) in _fractures)
        {
            s.Var((ulong)step); s.U8(f.Kind); s.Var((ulong)f.SourceSlot); s.Var((ulong)f.SourceGen); s.F32(f.Impulse);
            s.F64(f.PX); s.F64(f.PY); s.F64(f.PZ);
            s.Var((ulong)f.Fragments.Length);
            foreach (var (slot, gen) in f.Fragments) { s.Var((ulong)slot); s.Var((ulong)gen); }
            geometry.AddRange(f.Geometry);
        }

        s.Var((ulong)geometry.Count);
        foreach (var g in geometry)
        {
            s.Var((ulong)g.Slot); s.Var((ulong)g.Gen);
            s.Var((ulong)(g.Vertices.Length / _dims));
            foreach (float x in g.Vertices) { s.F32(x); }
            s.Var((ulong)(g.Triangles.Length / 3));
            foreach (int i in g.Triangles) { s.Var((ulong)i); }
        }

        return s;
    }

    private ByteSink EventChunk()
    {
        var s = new ByteSink(1 << 12);
        s.Var((ulong)_events.Count);
        foreach (var (step, e) in _events)
        {
            s.Var((ulong)step); s.U8(e.Kind);
            s.Var((ulong)e.ASlot); s.Var((ulong)e.AGen); s.Var((ulong)e.BSlot); s.Var((ulong)e.BGen);
            s.U32(e.ValueBits);
        }

        return s;
    }
}
