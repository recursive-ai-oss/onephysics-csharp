using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;

namespace RecursiveAI.OnePhysics.Bake;

/// <summary>
/// A bake cache opened for playback. It needs no world: <see cref="Rigid"/>, <see cref="Soft"/>, <see cref="Rods"/> and <see cref="Particles"/> hold one
/// <see cref="BakeEntity"/> per thing, sampled by time with interpolation; <see cref="Fractures"/>, <see cref="FragmentGeometry"/> and <see cref="Events"/> are plain lists.
/// Opening validates the structure of the whole file; the numbers inside a block are validated when it is first sampled.
/// Every defect is a <see cref="BakeFormatException"/> naming the field and the byte offset. A clip is not safe for concurrent sampling.
/// </summary>
public sealed class BakeClip
{
    internal const int MaxDecodedIntegers = 1 << 24;

    private BakeClip(ReadOnlyMemory<byte> bytes) => Bytes = bytes;

    internal ReadOnlyMemory<byte> Bytes { get; }

    /// <summary>2 or 3.</summary>
    public int Dimension { get; private set; }

    /// <summary>Format version of the file (1).</summary>
    public int FormatVersion { get; private set; }

    /// <summary>Frames in the clip (at least 1).</summary>
    public int FrameCount { get; private set; }

    /// <summary>Steps between frames.</summary>
    public int FrameStride { get; private set; }

    /// <summary>The step in seconds the world used.</summary>
    public float Dt { get; private set; }

    /// <summary>Seconds between frames: <see cref="FrameStride"/> × <see cref="Dt"/> (as a double).</summary>
    public double FrameDuration { get; private set; }

    /// <summary>Seconds from the first frame to the last.</summary>
    public double Duration => FrameCount <= 1 ? 0.0 : (FrameCount - 1) * FrameDuration;

    /// <summary>The world's step count when the bake began (frame 0).</summary>
    public long StartStep { get; private set; }

    /// <summary>Body positions are multiples of 2^PositionExponent metres.</summary>
    public int PositionExponent { get; private set; }

    /// <summary>Soft, rod and particle positions are multiples of 2^VertexExponent metres.</summary>
    public int VertexExponent { get; private set; }

    /// <summary>Rotation components are multiples of 2^-RotationBits, or exact float32 values when 0.</summary>
    public int RotationBits { get; private set; }

    /// <summary>Frames per independently coded block.</summary>
    public int BlockFrames { get; private set; }

    /// <summary>The scene name from the header.</summary>
    public string SceneName { get; private set; } = "";

    /// <summary>The library version that wrote the file.</summary>
    public string LibraryVersion { get; private set; } = "";

    /// <summary>The SHA-256 of the replay that reproduces the bake (32 bytes; all zero when none was given).</summary>
    public ReadOnlySpan<byte> ReplayDigest => _digest;

    /// <summary>The tracks present in the file.</summary>
    public BakeTracks Tracks { get; private set; }

    /// <summary>The rigid-body tracks, in the order they first appear.</summary>
    public IReadOnlyList<BakeEntity> Rigid => _rigid;

    /// <summary>The soft-body and cloth tracks.</summary>
    public IReadOnlyList<BakeEntity> Soft => _soft;

    /// <summary>The rod, rope, cable and hair tracks.</summary>
    public IReadOnlyList<BakeEntity> Rods => _rods;

    /// <summary>The fluid and granular particle tracks (one per particle, by particle id).</summary>
    public IReadOnlyList<BakeEntity> Particles => _parts;

    /// <summary>Fractures in step order.</summary>
    public IReadOnlyList<BakeFracture> Fractures => _fractures;

    /// <summary>The shape of each fracture fragment, stored once.</summary>
    public IReadOnlyList<BakeFragmentGeometry> FragmentGeometry => _geometry;

    /// <summary>Events in step order, in the world's order within a step.</summary>
    public IReadOnlyList<BakeEvent> Events => _events;

    /// <summary>The world hash after frame <paramref name="frame"/> was recorded: re-simulating from the replay must reproduce it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The frame is outside the clip.</exception>
    public ulong GetFrameHash(int frame)
    {
        if ((uint)frame >= (uint)FrameCount) { throw new ArgumentOutOfRangeException(nameof(frame), frame, "frame is outside the clip."); }
        return _hashes != null ? _hashes[frame] : 0UL;
    }

    /// <summary>The seconds of frame <paramref name="frame"/>.</summary>
    public double TimeOfFrame(int frame) => frame * FrameDuration;

    /// <summary>The body track with this slot and generation, or null.</summary>
    public BakeEntity? FindRigid(int slot, int generation) => Find(_rigid, slot, generation);

    private static BakeEntity? Find(List<BakeEntity> list, int slot, int generation)
    {
        foreach (var e in list) { if (e.Id.Slot == slot && e.Id.Generation == generation) { return e; } }
        return null;
    }

    private byte[] _digest = new byte[32];
    private ulong[]? _hashes;
    private readonly List<BakeEntity> _rigid = new(), _soft = new(), _rods = new(), _parts = new();
    private readonly List<BakeFracture> _fractures = new();
    private readonly List<BakeFragmentGeometry> _geometry = new();
    private readonly List<BakeEvent> _events = new();

    /// <summary>Reads a clip from a stream (to its end).</summary>
    /// <param name="stream">Open, readable; left open.</param>
    /// <param name="verifyChecksum">Check the file's checksum (default true).</param>
    /// <exception cref="BakeFormatException">The file is malformed.</exception>
    public static BakeClip Read(Stream stream, bool verifyChecksum = true)
    {
        ArgumentNullException.ThrowIfNull(stream);
        using var ms = new MemoryStream();
        stream.CopyTo(ms);
        if (ms.Length > int.MaxValue - 4096) { throw new BakeFormatException("file", 0, "the file is larger than 2 GiB"); }
        return Parse(ms.ToArray(), verifyChecksum);
    }

    /// <summary>Opens a clip from bytes the clip then keeps referring to (do not change them).</summary>
    /// <param name="bytes">The file.</param>
    /// <param name="verifyChecksum">Check the file's checksum (default true).</param>
    /// <exception cref="BakeFormatException">The file is malformed.</exception>
    public static BakeClip Parse(ReadOnlyMemory<byte> bytes, bool verifyChecksum = true)
    {
        var clip = new BakeClip(bytes);
        clip.ParseAll(bytes.Span, verifyChecksum);
        return clip;
    }

    private void ParseAll(ReadOnlySpan<byte> file, bool verify)
    {
        var r = new ByteSource(file);
        if (r.U32("magic") != BakeFormat.Magic) { throw new BakeFormatException("magic", 0, "not a bake cache"); }
        FormatVersion = r.U16("format version");
        if (FormatVersion != BakeFormat.Version) { throw new BakeFormatException("format version", 4, "version " + FormatVersion.ToString(CultureInfo.InvariantCulture) + " is not understood (this reader reads version 1)"); }
        r.U16("flags");
        Dimension = r.U8("dimension");
        if (Dimension != 2 && Dimension != 3) { throw r.Fail("dimension", "must be 2 or 3"); }
        PositionExponent = (sbyte)r.U8("position exponent");
        VertexExponent = (sbyte)r.U8("vertex exponent");
        RotationBits = r.U8("rotation bits");
        if (PositionExponent < -40 || PositionExponent > 0) { throw r.Fail("position exponent", "must be -40 to 0"); }
        if (VertexExponent < -40 || VertexExponent > 0) { throw r.Fail("vertex exponent", "must be -40 to 0"); }
        if (RotationBits > 30) { throw r.Fail("rotation bits", "must be 0 to 30"); }
        uint frames = r.U32("frame count");
        if (frames < 1 || frames > BakeFormat.MaxFrames) { throw r.Fail("frame count", "must be 1 to " + BakeFormat.MaxFrames.ToString(CultureInfo.InvariantCulture)); }
        FrameCount = (int)frames;
        uint stride = r.U32("frame stride");
        if (stride < 1 || stride > 1024) { throw r.Fail("frame stride", "must be 1 to 1024"); }
        FrameStride = (int)stride;
        uint bf = r.U32("block frames");
        if (bf < 1 || bf > BakeFormat.MaxBlockFrames) { throw r.Fail("block frames", "must be 1 to " + BakeFormat.MaxBlockFrames.ToString(CultureInfo.InvariantCulture)); }
        BlockFrames = (int)bf;
        Dt = r.F32("dt");
        if (!(Dt > 0f) || !float.IsFinite(Dt)) { throw r.Fail("dt", "must be finite and positive"); }
        FrameDuration = (double)Dt * FrameStride;
        r.U32("reserved");
        StartStep = r.I64("start step");
        _digest = r.Slice(32, "replay digest").ToArray();
        SceneName = r.Str("scene name");
        LibraryVersion = r.Str("library version");

        var seen = new HashSet<uint>();
        int endAt = -1;
        while (true)
        {
            int chunkStart = r.Position;
            uint tag = r.U32("chunk tag");
            uint len = r.U32("chunk length");
            if (len > (uint)r.Remaining) { throw r.Fail("chunk length", "the chunk is longer than the file"); }
            int payloadAt = r.Offset;
            var payload = r.Slice((int)len, "chunk payload");
            if (!seen.Add(tag)) { throw new BakeFormatException("chunk tag", payloadAt - 8, "the chunk " + TagText(tag) + " appears twice"); }
            switch (tag)
            {
                case BakeFormat.TagHashes: ParseHashes(payload, payloadAt); break;
                case BakeFormat.TagRigid: ParseEntities(payload, payloadAt, _rigid, tag); Tracks |= BakeTracks.Rigid; break;
                case BakeFormat.TagSoft: ParseEntities(payload, payloadAt, _soft, tag); Tracks |= BakeTracks.Soft; break;
                case BakeFormat.TagRods: ParseEntities(payload, payloadAt, _rods, tag); Tracks |= BakeTracks.Rods; break;
                case BakeFormat.TagParticles: ParseEntities(payload, payloadAt, _parts, tag); Tracks |= BakeTracks.Particles; break;
                case BakeFormat.TagFracture: ParseFractures(payload, payloadAt); Tracks |= BakeTracks.Fracture; break;
                case BakeFormat.TagEvents: ParseEvents(payload, payloadAt); Tracks |= BakeTracks.Events; break;
                case BakeFormat.TagEnd:
                    if (len != 8) { throw new BakeFormatException("end chunk", payloadAt, "the end chunk must be 8 bytes"); }
                    endAt = chunkStart;
                    if (verify)
                    {
                        ulong want = System.Buffers.Binary.BinaryPrimitives.ReadUInt64LittleEndian(payload);
                        if (BakeFormat.Checksum(file.Slice(0, chunkStart)) != want) { throw new BakeFormatException("checksum", payloadAt, "the checksum does not match the file"); }
                    }

                    break;
                default: break; // a chunk from a later 1.x: skipped
            }

            if (endAt >= 0) { break; }
            if (r.Remaining == 0) { throw r.Fail("end chunk", "the file ends without an end chunk"); }
        }

        if (r.Remaining != 0) { throw r.Fail("end chunk", "bytes follow the end chunk"); }
    }

    private static string TagText(uint tag)
    {
        Span<char> c = stackalloc char[4];
        for (int i = 0; i < 4; i++) { int b = (int)((tag >> (8 * i)) & 0xFF); c[i] = b >= 32 && b < 127 ? (char)b : '?'; }
        return new string(c);
    }

    private void ParseHashes(ReadOnlySpan<byte> p, int at)
    {
        var r = new ByteSource(p, at);
        if (p.Length != (long)FrameCount * 8) { throw r.Fail("hash chunk", "must hold one 8-byte hash per frame"); }
        _hashes = new ulong[FrameCount];
        for (int i = 0; i < FrameCount; i++) { _hashes[i] = r.U64("frame hash"); }
    }

    private void ParseEntities(ReadOnlySpan<byte> p, int at, List<BakeEntity> into, uint tag)
    {
        var r = new ByteSource(p, at);
        int count = r.VarInt("entity count", Math.Max(0, p.Length));
        string what = TagText(tag);
        for (int n = 0; n < count; n++)
        {
            int key = r.VarInt(what + " key", int.MaxValue);
            int gen = r.VarInt(what + " generation", int.MaxValue);
            int first = r.VarInt(what + " first frame", FrameCount - 1);
            int frames = r.VarInt(what + " frame count", FrameCount);
            if (frames < 1 || (long)first + frames > FrameCount) { throw r.Fail(what + " frame range", "the entity's frames lie outside the clip"); }
            int kind = 0, points = 1;
            int[]? tris = null;
            if (tag == BakeFormat.TagSoft)
            {
                kind = r.U8("soft kind") & 1;
                points = r.VarInt("soft point count", BakeFormat.MaxPoints);
                if (points < 1) { throw r.Fail("soft point count", "must be at least 1"); }
                int triCount = r.VarInt("soft triangle count", r.Remaining / 3);
                tris = new int[triCount * 3];
                for (int i = 0; i < tris.Length; i++) { tris[i] = r.VarInt("soft triangle index", points - 1); }
            }
            else if (tag == BakeFormat.TagRods)
            {
                points = r.VarInt("rod point count", BakeFormat.MaxPoints);
                if (points < 1) { throw r.Fail("rod point count", "must be at least 1"); }
            }
            else if (tag == BakeFormat.TagParticles) { kind = r.VarInt("particle fluid", int.MaxValue); }

            bool rigid = tag == BakeFormat.TagRigid;
            int len = rigid ? Dimension + (Dimension == 3 ? 4 : 2) : points * Dimension;
            int split = rigid ? Dimension : 0;
            if ((long)len * BlockFrames > MaxDecodedIntegers) { throw r.Fail(what + " point count", "a block of this entity would hold more than " + MaxDecodedIntegers.ToString(CultureInfo.InvariantCulture) + " numbers"); }
            long blocksWanted = ((long)frames + BlockFrames - 1) / BlockFrames;
            int blocks = r.VarInt(what + " block count", int.MaxValue);
            if (blocks != blocksWanted) { throw r.Fail(what + " block count", "expected " + blocksWanted.ToString(CultureInfo.InvariantCulture) + " blocks for " + frames.ToString(CultureInfo.InvariantCulture) + " frames"); }
            var lens = new int[blocks];
            long sum = 0;
            for (int b = 0; b < blocks; b++)
            {
                lens[b] = r.VarInt(what + " block length", r.Remaining);
                sum += lens[b];
            }

            if (sum > r.Remaining) { throw r.Fail(what + " block data", "the blocks are longer than the chunk"); }
            var offs = new int[blocks];
            int pos = r.Offset;
            for (int b = 0; b < blocks; b++) { offs[b] = pos; pos += lens[b]; }
            r.Skip((int)sum, what + " block data");
            into.Add(new BakeEntity(this, new BakeId(key, gen), kind, points, first, frames, len, split, offs, lens, tris, rigid));
        }

        if (r.Remaining != 0) { throw r.Fail(what + " chunk", "bytes follow the last entity"); }
    }

    private void ParseFractures(ReadOnlySpan<byte> p, int at)
    {
        var r = new ByteSource(p, at);
        int count = r.VarInt("fracture count", r.Remaining / 30);
        for (int n = 0; n < count; n++)
        {
            int step = r.VarInt("fracture step", int.MaxValue);
            int kind = r.U8("fracture kind");
            if (kind > (int)FractureKind.Slice) { throw r.Fail("fracture kind", "unknown kind " + kind.ToString(CultureInfo.InvariantCulture)); }
            int slot = r.VarInt("fracture source slot", int.MaxValue);
            int gen = r.VarInt("fracture source generation", int.MaxValue);
            float impulse = r.F32("fracture impulse");
            double x = r.F64("fracture point"), y = r.F64("fracture point"), z = r.F64("fracture point");
            if (!float.IsFinite(impulse) || !double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z)) { throw r.Fail("fracture point", "a number is not finite"); }
            int fc = r.VarInt("fragment count", r.Remaining / 2);
            var frag = new BakeId[fc];
            for (int i = 0; i < fc; i++) { frag[i] = new BakeId(r.VarInt("fragment slot", int.MaxValue), r.VarInt("fragment generation", int.MaxValue)); }
            _fractures.Add(new BakeFracture(step, FrameOfStep(step), (FractureKind)kind, new BakeId(slot, gen), impulse, new Vector3D(x, y, z), frag));
        }

        int gc = r.VarInt("geometry count", r.Remaining / 3);
        for (int n = 0; n < gc; n++)
        {
            var id = new BakeId(r.VarInt("geometry slot", int.MaxValue), r.VarInt("geometry generation", int.MaxValue));
            int vc = r.VarInt("geometry vertex count", r.Remaining / (4 * Dimension));
            var v = new Vector3F[vc];
            for (int i = 0; i < vc; i++)
            {
                float vx = r.F32("geometry vertex"), vy = r.F32("geometry vertex"), vz = Dimension == 3 ? r.F32("geometry vertex") : 0f;
                if (!float.IsFinite(vx) || !float.IsFinite(vy) || !float.IsFinite(vz)) { throw r.Fail("geometry vertex", "a coordinate is not finite"); }
                v[i] = new Vector3F(vx, vy, vz);
            }

            int tc = r.VarInt("geometry triangle count", r.Remaining / 3);
            var t = new int[tc * 3];
            for (int i = 0; i < t.Length; i++) { t[i] = r.VarInt("geometry triangle index", Math.Max(vc - 1, 0)); if (vc == 0) { throw r.Fail("geometry triangle index", "there are no vertices to index"); } }
            _geometry.Add(new BakeFragmentGeometry(id, v, t));
        }

        if (r.Remaining != 0) { throw r.Fail("fracture chunk", "bytes follow the last record"); }
    }

    private void ParseEvents(ReadOnlySpan<byte> p, int at)
    {
        var r = new ByteSource(p, at);
        int count = r.VarInt("event count", r.Remaining / 10);
        for (int n = 0; n < count; n++)
        {
            int step = r.VarInt("event step", int.MaxValue);
            int kind = r.U8("event kind");
            if (kind > (int)EventKind.ContactHit) { throw r.Fail("event kind", "unknown kind " + kind.ToString(CultureInfo.InvariantCulture)); }
            int aSlot = r.VarInt("event body A slot", int.MaxValue), aGen = r.VarInt("event body A generation", int.MaxValue);
            int bSlot = r.VarInt("event body B slot", int.MaxValue), bGen = r.VarInt("event body B generation", int.MaxValue);
            uint bits = r.U32("event value");
            float value = BitConverter.UInt32BitsToSingle(bits);
            if (!float.IsFinite(value)) { throw r.Fail("event value", "the value is not finite"); }
            _events.Add(new BakeEvent(step, FrameOfStep(step), (EventKind)kind, new BakeId(aSlot - 1, aGen), new BakeId(bSlot - 1, bGen), value));
        }

        if (r.Remaining != 0) { throw r.Fail("event chunk", "bytes follow the last event"); }
    }

    private int FrameOfStep(int step) => (int)Math.Min(((long)step + FrameStride - 1) / FrameStride, int.MaxValue);
}
