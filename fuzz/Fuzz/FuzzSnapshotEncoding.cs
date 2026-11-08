using System;
using RecursiveAI.OnePhysics;

internal static class FuzzSnapshotEncoding
{
    internal static readonly byte[] Base = new byte[256];

    internal static void Desync(byte[] bytes)
    {
        if (bytes.Length > 65536) { return; }
        try { _ = SnapshotDesync.Inspect(bytes); }
        catch (SnapshotFormatException) { }
    }

    internal static void Region(byte[] bytes)
    {
        try
        {
            var region = RegionSnapshot.Read(bytes, 65536);
            if (!region.ToArray().AsSpan().SequenceEqual(bytes)) { throw new InvalidOperationException("Region bytes changed."); }
            _ = region.ComputeHash();
        }
        catch (SnapshotFormatException) { }
    }

    internal static void Delta(byte[] bytes)
    {
        try { SnapshotEncoding.ApplyDelta(Base, bytes, 65536); }
        catch (SnapshotEncodingException) { }
    }

    internal static void Compressed(byte[] bytes)
    {
        try { SnapshotEncoding.Decompress(bytes, 65536); }
        catch (SnapshotEncodingException) { }
    }
}
