using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Returns a delta against the supplied full snapshot bytes, covering every saved section. Does not retain the base.</summary>
    public byte[] SnapshotDelta(ReadOnlySpan<byte> baseSnapshot) => SnapshotEncoding.CreateDelta(baseSnapshot, Snapshot());

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
}
