using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Returns a delta against the supplied full snapshot bytes, covering every saved section. Does not retain the base.</summary>
    public byte[] SnapshotDelta(ReadOnlySpan<byte> baseSnapshot) => SnapshotEncoding.CreateDelta(baseSnapshot, Snapshot());

    /// <summary>Returns a version-1 literal or version-2 XOR delta. XOR is useful before compression when nearby floating-point bit patterns change.</summary>
    public byte[] SnapshotDelta(ReadOnlySpan<byte> baseSnapshot, bool xorPayload) => xorPayload ? SnapshotEncoding.CreateXorDelta(baseSnapshot, Snapshot()) : SnapshotDelta(baseSnapshot);

    /// <summary>Returns a compressed full snapshot with optional schema-independent byte-plane transposition (version 2). RestoreCompressed accepts both versions.</summary>
    public byte[] SnapshotCompressed(bool bytePlanes) => SnapshotEncoding.Compress(Snapshot(), bytePlanes);

    /// <summary>Returns a deterministic compressed full snapshot. Call RestoreCompressed to restore it.</summary>
    public byte[] SnapshotCompressed() => SnapshotEncoding.Compress(Snapshot());

    /// <summary>Applies a delta and restores its exact full snapshot. The world is unchanged on an invalid base, delta or snapshot.</summary>
    /// <exception cref="SnapshotEncodingException">Wrong base, invalid delta, or decoded length above the limit (default 256 MiB).</exception>
    /// <exception cref="SnapshotFormatException">Decoded bytes are not a valid snapshot for this world.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The output limit is negative.</exception>
    public OptionChanges RestoreDelta(ReadOnlySpan<byte> baseSnapshot, ReadOnlySpan<byte> delta, int maximumDecodedBytes = SnapshotEncoding.DefaultMaximumDecodedBytes)
        => Restore(SnapshotEncoding.ApplyDelta(baseSnapshot, delta, maximumDecodedBytes));

    /// <summary>Decompresses and restores a full snapshot. The world is unchanged on invalid input.</summary>
    /// <exception cref="SnapshotEncodingException">Invalid compression or decoded length above the limit (default 256 MiB).</exception>
    /// <exception cref="SnapshotFormatException">Decoded bytes are not a valid snapshot for this world.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The output limit is negative.</exception>
    public OptionChanges RestoreCompressed(ReadOnlySpan<byte> snapshot, int maximumDecodedBytes = SnapshotEncoding.DefaultMaximumDecodedBytes)
        => Restore(SnapshotEncoding.Decompress(snapshot, maximumDecodedBytes));
    /// <summary>Creates an independent structural copy of the complete simulation state, including pending events. Copies arrays directly, not through serialization. Cost and memory are linear in allocated storage. Subscriptions and callback delegates are not copied; the fork has the same numeric world id, but handles still belong to their original world object. Call between steps. Future copy-on-write optimizations do not change this contract.</summary>
    /// <exception cref="NotSupportedException">A custom registered section does not implement ISnapshotSection.Fork.</exception>
    public PhysicsWorld3D Fork()
    {
        var copy = _core.Fork();
        return ((Engine3D)copy.GetSnapshotSection(Engine3D.Id)).World!;
    }

    internal PhysicsWorld3D(WorldCore core, Engine3D engine)
    {
        _core = core; _engine = engine; engine.World = this;
        BuildPublicEvents();
    }

    /// <summary>Returns a readable desync report and typed differences, including events and all engine sections. Does not step or mutate either world. Allocates tooling buffers.</summary>
    /// <exception cref="ArgumentNullException">The other world is null.</exception>
    public SnapshotDiffReport FindDifferences(PhysicsWorld3D other)
    {
        ArgumentNullException.ThrowIfNull(other);
        return SnapshotDesync.Compare(_core.DescribeSnapshot(), other._core.DescribeSnapshot());
    }
    /// <summary>Captures live bodies whose origins lie in the inclusive box, then their complete dynamic constraint closure, anchors, shapes, islands, caches and associated services. Soft bodies are selected by any particle in the box and included whole. Costs tooling allocations. See docs/spec/snapshots.md for closure, globals and topology-compatible partial restore.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound is non-finite, unordered or outside ±2097152.</exception>
    public RegionSnapshot CaptureRegion(Vector3D minimum, Vector3D maximum)
    {
        double[] min = [minimum.X, minimum.Y, minimum.Z], max = [maximum.X, maximum.Y, maximum.Z];
        var selection = _engine.SelectRegion(min, max);
        var description = _core.DescribeSnapshot();
        return RegionSnapshot.Create(description, SnapshotRegions.Select(description, selection), min, max);
    }

    /// <summary>Returns the canonical region archive bytes. This is not full snapshot v1; restore with RestoreRegion, not Restore.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound is non-finite, unordered or outside ±2097152.</exception>
    public byte[] SnapshotRegion(Vector3D minimum, Vector3D maximum) => CaptureRegion(minimum, maximum).ToArray();

    /// <summary>Returns the region archive hash, including bounds, tick, origin and options. Uses the same selection as CaptureRegion. Allocates tooling buffers.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A bound is non-finite, unordered or outside ±2097152.</exception>
    public ulong ComputeRegionHash(Vector3D minimum, Vector3D maximum) => CaptureRegion(minimum, maximum).ComputeHash();

    /// <summary>Atomically resynchronizes existing entities at the same tick/origin/options. Requires matching selected handles, generations, field topology and constraint closure; otherwise use a full snapshot. Does not create/delete entities or change subscriptions. Broadphase proxies are refreshed, not imported. No events are dispatched.</summary>
    /// <exception cref="ArgumentNullException">The archive is null.</exception>
    /// <exception cref="SnapshotFormatException">Archive topology/context differs or its state is invalid. World unchanged.</exception>
    public OptionChanges RestoreRegion(RegionSnapshot archive)
    {
        ArgumentNullException.ThrowIfNull(archive);
        var selection = _engine.SelectRegion(archive.Minimum, archive.Maximum, archive);
        var description = _core.DescribeSnapshot();
        var patched = SnapshotRegions.Patch(description, SnapshotRegions.Select(description, selection), archive);
        var staging = Fork();
        staging.Restore(patched);
        staging._engine.RefreshRegionProxies(selection);
        return Restore(staging.Snapshot());
    }

    /// <summary>Reads and restores a region archive, enforcing default limits of 256 MiB input and 1,000,000 fields before allocating tooling state. Same topology/context requirements as the object overload.</summary>
    /// <exception cref="SnapshotFormatException">Malformed archive or incompatible state/topology/context. World unchanged.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A byte or field limit is negative.</exception>
    public OptionChanges RestoreRegion(ReadOnlySpan<byte> archive, int maximumBytes = SnapshotEncoding.DefaultMaximumDecodedBytes, int maximumFields = RegionSnapshot.DefaultMaximumFields) => RestoreRegion(RegionSnapshot.Read(archive, maximumBytes, maximumFields));
    /// <summary>Compares this world with a full snapshot using this world's section schema and construction options. Supports custom sections that implement structural copying. The current world is unchanged.</summary>
    /// <exception cref="SnapshotFormatException">The snapshot is invalid or has incompatible sections.</exception>
    /// <exception cref="NotSupportedException">A custom section does not implement structural copying.</exception>
    public SnapshotDiffReport FindDifferences(ReadOnlySpan<byte> snapshot)
    {
        var other = Fork(); other.Restore(snapshot); return FindDifferences(other);
    }
}
