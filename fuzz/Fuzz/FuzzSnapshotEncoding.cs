using System;
using RecursiveAI.OnePhysics;

internal static class FuzzSnapshotEncoding
{
    internal static readonly byte[] Base = new byte[256];

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
