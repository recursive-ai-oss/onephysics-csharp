using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>
/// One thing's track in a clip: a body's poses, a soft body's, rod's or particle's points. It exists for frames
/// <see cref="FirstFrame"/> to <see cref="LastFrame"/> and is sampled by time. Sampling decodes the entity's blocks on demand and
/// keeps two of them; an entity (and its clip) is not safe to sample from several threads at once.
/// </summary>
public sealed class BakeEntity
{
    private readonly BakeClip _clip;
    private readonly int[] _blockOffset, _blockLength; // offsets into the clip's bytes
    private readonly int _len, _split;
    private int _cacheA = -1, _cacheB = -1;
    private long[]? _dataA, _dataB;

    internal BakeEntity(BakeClip clip, BakeId id, int kind, int pointCount, int firstFrame, int frames, int vectorLength, int split, int[] offsets, int[] lengths, int[]? triangles, bool isRigid)
    {
        _clip = clip; Id = id; _kindRaw = kind; PointCount = pointCount; FirstFrame = firstFrame; FrameCount = frames;
        _len = vectorLength; _split = split; _blockOffset = offsets; _blockLength = lengths; SurfaceTriangles = triangles ?? Array.Empty<int>(); _isRigid = isRigid;
    }

    private int Off => _blockOffset.Length > 0 ? _blockOffset[0] : 0;
    private readonly int _kindRaw;
    private readonly bool _isRigid;

    /// <summary>The body, soft body or rod slot and generation, or for a fluid particle its id (slot) with generation 0.</summary>
    public BakeId Id { get; }

    /// <summary>The first frame it exists in.</summary>
    public int FirstFrame { get; }

    /// <summary>How many consecutive frames it exists for.</summary>
    public int FrameCount { get; }

    /// <summary>The last frame it exists in.</summary>
    public int LastFrame => FirstFrame + FrameCount - 1;

    /// <summary>Points per frame: 1 for a body or a particle; the particle count of a soft body or a rod.</summary>
    public int PointCount { get; }

    /// <summary>For a soft-body track: true when the body was cloth (it had fabric behaviour).</summary>
    public bool IsCloth => _kindRaw == 1;

    /// <summary>For a particle track: the slot of the fluid the particle belonged to.</summary>
    public int Fluid => _kindRaw;

    /// <summary>For a soft-body track in a 3D clip or a 2D triangle mesh: the surface triangles (indices into the points, three per triangle). Empty otherwise.</summary>
    public int[] SurfaceTriangles { get; }

    /// <summary>True when the entity exists at <paramref name="frame"/>.</summary>
    public bool ExistsAtFrame(int frame) => frame >= FirstFrame && frame <= LastFrame;

    /// <summary>
    /// Samples a body's pose at <paramref name="time"/> seconds from the start of the clip. At a frame time the stored value is returned as decoded;
    /// between frames the position is interpolated linearly and the rotation by normalised linear interpolation along the shorter arc (nlerp; its largest
    /// difference from slerp is about 1/10 of the angle turned in one frame squared, 0.0003 rad for a quarter-turn per second at 60 Hz).
    /// </summary>
    /// <returns>False when the body does not exist at that time (before its first frame, after its last, or outside the clip).</returns>
    /// <exception cref="InvalidOperationException">This is not a body track.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="time"/> is not finite.</exception>
    /// <exception cref="BakeFormatException">The file's data for this entity is malformed.</exception>
    public bool TrySampleRigid(double time, out BakePose pose)
    {
        if (!_isRigid) { throw new InvalidOperationException("This is not a rigid-body track."); }
        pose = default;
        if (!Resolve(time, out int f0, out int f1, out double a)) { return false; }
        int d = _clip.Dimension;
        double ps = BakeFormat.Pow2(_clip.PositionExponent);
        var p0 = Get(f0);
        double x = (double)p0[0] * ps, y = (double)p0[1] * ps, z = d == 3 ? (double)p0[2] * ps : 0.0;
        int rc = d == 3 ? 4 : 2;
        Span<float> r0 = stackalloc float[4];
        Span<float> r1 = stackalloc float[4];
        DecodeRot(p0, d, r0);
        if (a != 0.0)
        {
            var p1 = Get(f1);
            DecodeRot(p1, d, r1);
            x += ((double)p1[0] * ps - x) * a; y += ((double)p1[1] * ps - y) * a;
            if (d == 3) { z += ((double)p1[2] * ps - z) * a; }
            double dot = 0;
            for (int i = 0; i < rc; i++) { dot += (double)r0[i] * r1[i]; }
            double sgn = dot < 0 ? -1.0 : 1.0, ssq = 0;
            Span<double> m = stackalloc double[4];
            for (int i = 0; i < rc; i++) { m[i] = (double)r0[i] * (1.0 - a) + sgn * (double)r1[i] * a; ssq += m[i] * m[i]; }
            double inv = ssq > 0 ? 1.0 / Math.Sqrt(ssq) : 0.0;
            for (int i = 0; i < rc; i++) { r0[i] = ssq > 0 ? (float)(m[i] * inv) : r0[i]; }
        }

        pose = d == 3
            ? new BakePose(new Vector3D(x, y, z), new QuaternionF(r0[0], r0[1], r0[2], r0[3]), 1f, 0f)
            : new BakePose(new Vector3D(x, y, 0.0), QuaternionF.Identity, r0[0], r0[1]);
        return true;
    }

    /// <summary>Samples a soft body's, rod's or particle's points at <paramref name="time"/> seconds (linear interpolation between frames; z is 0 in a 2D clip).</summary>
    /// <param name="time">Seconds from the start of the clip.</param>
    /// <param name="destination">At least <see cref="PointCount"/> long.</param>
    /// <returns>False when the entity does not exist at that time.</returns>
    /// <exception cref="InvalidOperationException">This is a body track (use <see cref="TrySampleRigid"/>).</exception>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is too short.</exception>
    /// <exception cref="BakeFormatException">The file's data for this entity is malformed.</exception>
    public bool TrySamplePoints(double time, Span<Vector3D> destination)
    {
        if (_isRigid) { throw new InvalidOperationException("This is a rigid-body track; use TrySampleRigid."); }
        if (destination.Length < PointCount) { throw new ArgumentException("destination is shorter than PointCount.", nameof(destination)); }
        if (!Resolve(time, out int f0, out int f1, out double a)) { return false; }
        int d = _clip.Dimension;
        double s = BakeFormat.Pow2(_clip.VertexExponent);
        var p0 = Get(f0);
        ReadOnlySpan<long> p1 = default;
        if (a != 0.0) { p1 = Get(f1); }
        for (int i = 0; i < PointCount; i++)
        {
            double x = (double)p0[i * d] * s, y = (double)p0[i * d + 1] * s, z = d == 3 ? (double)p0[i * d + 2] * s : 0.0;
            if (a != 0.0)
            {
                x += ((double)p1[i * d] * s - x) * a; y += ((double)p1[i * d + 1] * s - y) * a;
                if (d == 3) { z += ((double)p1[i * d + 2] * s - z) * a; }
            }

            destination[i] = new Vector3D(x, y, z);
        }

        return true;
    }

    private bool Resolve(double time, out int f0, out int f1, out double a)
    {
        f0 = f1 = 0; a = 0;
        if (!double.IsFinite(time)) { throw new ArgumentOutOfRangeException(nameof(time), time, "time must be finite."); }
        double ft = time / _clip.FrameDuration;
        double r = Math.Round(ft);
        if (Math.Abs(ft - r) < 1e-9) { ft = r; }
        if (ft < FirstFrame || ft > LastFrame) { return false; }
        double fl = Math.Floor(ft);
        f0 = (int)fl; a = ft - fl;
        f1 = a == 0 ? f0 : f0 + 1;
        return f1 <= LastFrame;
    }

    private void DecodeRot(ReadOnlySpan<long> v, int d, Span<float> r)
    {
        int rc = d == 3 ? 4 : 2, at = d;
        int bits = _clip.RotationBits;
        double scale = bits == 0 ? 1.0 : BakeFormat.Pow2(-bits);
        for (int i = 0; i < rc; i++)
        {
            long q = v[at + i];
            if (bits == 0)
            {
                if (q < int.MinValue || q > int.MaxValue) { throw new BakeFormatException("rotation", Off, "an exact rotation component is not a 32-bit pattern"); }
                float f = BitConverter.Int32BitsToSingle((int)q);
                if (!float.IsFinite(f)) { throw new BakeFormatException("rotation", Off, "a rotation component is not finite"); }
                r[i] = f;
            }
            else { r[i] = (float)((double)q * scale); }
        }
    }

    private ReadOnlySpan<long> Get(int frame)
    {
        int rel = frame - FirstFrame;
        int bf = _clip.BlockFrames;
        int b = rel / bf;
        long[] data;
        if (_cacheA == b) { data = _dataA!; }
        else if (_cacheB == b) { data = _dataB!; }
        else
        {
            data = DecodeBlock(b);
            // Keep the two most recently decoded blocks: sampling walks forward through them.
            _cacheB = _cacheA; _dataB = _dataA;
            _cacheA = b; _dataA = data;
        }

        return data.AsSpan((rel - b * bf) * _len, _len);
    }

    private long[] DecodeBlock(int b)
    {
        int bf = _clip.BlockFrames;
        int n = Math.Min(bf, FrameCount - b * bf);
        long total = (long)n * _len;
        if (total > BakeClip.MaxDecodedIntegers) { throw new BakeFormatException("block", Off, "a block holds more than " + BakeClip.MaxDecodedIntegers + " numbers"); }
        var data = new long[total];
        var src = new ByteSource(_clip.Bytes.Span.Slice(_blockOffset[b], _blockLength[b]), _blockOffset[b]);
        int frame = 0;
        while (frame < n)
        {
            int tag = src.U8("block record tag");
            if (tag == 0)
            {
                int run = src.VarInt("zero-run length", n - frame);
                if (run == 0) { throw src.Fail("zero-run length", "a zero run of length 0"); }
                for (int k = 0; k < run; k++) { FillZero(data, frame++); }
            }
            else
            {
                int max = _split == 0 ? 1 : 3;
                if (tag > max) { throw src.Fail("block record tag", "tag " + tag + " is not valid for this track"); }
                Fill(data, frame++, ref src, tag);
            }
        }

        if (src.Remaining != 0) { throw src.Fail("block", "the block has bytes after its last frame"); }
        return data;
    }

    private void Fill(long[] data, int frame, ref ByteSource src, int mask)
    {
        int at = frame * _len;
        for (int i = 0; i < _len; i++)
        {
            long pred = frame == 0 ? 0 : frame == 1 ? data[at - _len + i] : unchecked(2 * data[at - _len + i] - data[at - 2 * _len + i]);
            bool inFirst = _split == 0 || i < _split;
            long res = (mask & (inFirst ? 1 : 2)) != 0 ? BakeFormat.UnZigZag(src.Var("residual")) : 0;
            data[at + i] = unchecked(pred + res);
        }
    }

    private void FillZero(long[] data, int frame)
    {
        int at = frame * _len;
        for (int i = 0; i < _len; i++)
        {
            long pred = frame == 0 ? 0 : frame == 1 ? data[at - _len + i] : unchecked(2 * data[at - _len + i] - data[at - 2 * _len + i]);
            data[at + i] = pred;
        }
    }
}
