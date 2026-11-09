using System;
using System.Globalization;
using System.IO;

// Seed corpora for the coverage-guided mode, produced by our own code so they can be regenerated: a few seeded pseudo-random inputs of
// growing length per target, plus hand-made inputs where a structure matters (a valid snapshot, a cube and a tetrahedron for the hull builder).
internal static class Seeds
{
    private static readonly int[] Lengths = { 8, 24, 64, 128, 256, 512 };

    public static int Write(string root)
    {
        foreach (string name in Program.Targets.Keys)
        {
            string dir = Path.Combine(root, name);
            Directory.CreateDirectory(dir);
            // A generator seeded from the target name (FNV-1a), so the same bytes come out on every machine.
            ulong h = 14695981039346656037UL;
            foreach (char c in name) { h = (h ^ c) * 1099511628211UL; }
            var rng = new Random((int)(h & 0x7FFFFFFF));
            foreach (int len in Lengths)
            {
                var b = new byte[len];
                rng.NextBytes(b);
                File.WriteAllBytes(Path.Combine(dir, "random-" + len.ToString(CultureInfo.InvariantCulture)), b);
            }
        }

        File.WriteAllBytes(Path.Combine(root, "snapshot-delta", "valid"), RecursiveAI.OnePhysics.SnapshotEncoding.CreateDelta(FuzzSnapshotEncoding.Base, new byte[512]));
        File.WriteAllBytes(Path.Combine(root, "snapshot-compressed", "valid"), RecursiveAI.OnePhysics.SnapshotEncoding.Compress(new byte[512]));
        var literals = new byte[256];
        for (int i = 0; i < literals.Length; i++) { literals[i] = (byte)i; }
        File.WriteAllBytes(Path.Combine(root, "snapshot-compressed", "literals"), RecursiveAI.OnePhysics.SnapshotEncoding.Compress(literals));

        File.WriteAllBytes(Path.Combine(root, "snapshot-delta", "xor"), RecursiveAI.OnePhysics.SnapshotEncoding.CreateXorDelta(FuzzSnapshotEncoding.Base, new byte[512]));
        File.WriteAllBytes(Path.Combine(root, "snapshot-compressed", "byteplanes"), RecursiveAI.OnePhysics.SnapshotEncoding.Compress(literals, true));
        var regionWorld = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D(new RecursiveAI.OnePhysics.Physics2D.WorldOptions2D { WorldId = 97 });
        regionWorld.AddCircle(0.5f, new RecursiveAI.OneMath.Vector2D(0, 0));
        File.WriteAllBytes(Path.Combine(root, "snapshot-region", "valid"), regionWorld.SnapshotRegion(new RecursiveAI.OneMath.Vector2D(-1, -1), new RecursiveAI.OneMath.Vector2D(1, 1)));

        File.WriteAllBytes(Path.Combine(root, "snapshot-desync", "valid"), regionWorld.Snapshot());
        File.WriteAllBytes(Path.Combine(root, "contact-state", "planar"), FuzzContactState.Seed(2));
        File.WriteAllBytes(Path.Combine(root, "contact-state", "spatial"), FuzzContactState.Seed(3));
        File.WriteAllBytes(Path.Combine(root, "scene", "surfaces"), System.Text.Encoding.UTF8.GetBytes("scene 1\ndim 2\nmaterial 0 0.5 0\nbody static 0 0 0 0 0 0 -\nshape box 0 0 0 2 0.1 0 0 0\nsurface 1 0 0 0 1 0 0 0 0 0 0 0\nmaterial_indices 0\n"));
        File.WriteAllBytes(Path.Combine(root, "scene", "authoring"), FuzzScene.Authoring());
        File.WriteAllBytes(Path.Combine(root, "scene", "state"), FuzzScene.State());
        File.WriteAllBytes(Path.Combine(root, "replay", "planar"), FuzzReplay.Seed(2));
        File.WriteAllBytes(Path.Combine(root, "replay", "spatial"), FuzzReplay.Seed(3));
        File.WriteAllBytes(Path.Combine(root, "replay", "recorded-planar"), FuzzReplay.CommandSeed(2));
        File.WriteAllBytes(Path.Combine(root, "replay", "recorded-spatial"), FuzzReplay.CommandSeed(3));

        // The raw snapshot reader starts from a real snapshot so that mutations get past the header.
        File.WriteAllBytes(Path.Combine(root, "snapshot-raw", "valid"), Program.BaseSnapshot());

        // Mode 1 of the hull decoder: signed bytes over 16, three per point.
        sbyte[][] cube = { new sbyte[] { -16, -16, -16 }, new sbyte[] { 16, -16, -16 }, new sbyte[] { -16, 16, -16 }, new sbyte[] { 16, 16, -16 }, new sbyte[] { -16, -16, 16 }, new sbyte[] { 16, -16, 16 }, new sbyte[] { -16, 16, 16 }, new sbyte[] { 16, 16, 16 } };
        var hull = new byte[4 + (cube.Length * 3)];
        hull[0] = 1;
        for (int i = 0; i < cube.Length; i++) { for (int k = 0; k < 3; k++) { hull[4 + (i * 3) + k] = (byte)cube[i][k]; } }
        File.WriteAllBytes(Path.Combine(root, "hull", "cube"), hull);
        return 0;
    }
}
