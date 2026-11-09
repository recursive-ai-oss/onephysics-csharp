using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Coplanar triangles describe one supporting plane. Keeping separate truncated patches at every diagonal changes the solver's leverage as a box crosses a seam.
    // Merge those patches, retaining four well-spread points, so inactive edges do not introduce an artificial support boundary.
    private static void MergeFlatPatches(NarrowScratch scratch, int first, bool soupIsA, QuaternionF inverseOther)
    {
        int end = scratch.OutCount;
        int firstFlat = first;
        while (firstFlat < end && !scratch.Out[firstFlat].FlatPatch) firstFlat++;
        if (firstFlat == end) return; // No shared supporting plane: keep ordinary triangle manifolds and their original feature ids.
        for (int i = firstFlat; i < end; i++)
        {
            var seed = scratch.Out[i];
            if (!seed.FlatPatch || seed.Count == 0) continue;
            int total = 0;
            for (int j = i; j < end; j++) if (SameFlatPatch(in seed, in scratch.Out[j], soupIsA)) { total += scratch.Out[j].Count; }
            if (total == 0) continue;
            if (total == 1)
            {
                // One point is already the full reduction; its canonical feature does not depend on local ordering.
                scratch.Out[i].Id[0] = 0xFC000000u;
                continue;
            }
            // Selection and the sorting network reuse each local ordering key. Compute it only once, and only for a multi-point patch.
            if (scratch.FlatLocalPoints.Length < end * 4) Array.Resize(ref scratch.FlatLocalPoints, scratch.Out.Length * 4);
            for (int j = i; j < end; j++)
            {
                ref var c = ref scratch.Out[j];
                if (!SameFlatPatch(in seed, in c, soupIsA)) continue;
                for (int k = 0; k < c.Count; k++)
                {
                    int p = k * 7 + (soupIsA ? 3 : 0);
                    var v = inverseOther.Rotate(new Vector3F(c.P[p], c.P[p + 1], c.P[p + 2]));
                    scratch.FlatLocalPoints[j * 4 + k] = new((float)Math.Round(v.X * 1e5f), (float)Math.Round(v.Y * 1e5f), (float)Math.Round(v.Z * 1e5f));
                }
            }
            var merged = seed;
            int s0 = -1, s1 = -1, s2 = -1, s3 = -1;
            for (int k = 0; k < Math.Min(total, 4); k++)
            {
                float best = -1; int chosen = -1;
                for (int j = i; j < end; j++)
                {
                    ref var candidate = ref scratch.Out[j]; if (!SameFlatPatch(in seed, in candidate, soupIsA)) continue;
                    for (int p = 0; p < candidate.Count; p++)
                    {
                        int key = j * 4 + p; var point = PatchPoint(scratch, key);
                        if (key == s0 || key == s1 || key == s2) continue;
                        float score;
                        if (k == 0)
                        {
                            bool better = chosen < 0 || CompareLocal(scratch, key, chosen, soupIsA, inverseOther) > 0;
                            if (better) chosen = key;
                            continue;
                        }
                        else if (k == 1) score = (point - PatchPoint(scratch, s0)).LengthSquared();
                        else if (k == 2) score = Vector3F.Cross(PatchPoint(scratch, s1) - PatchPoint(scratch, s0), point - PatchPoint(scratch, s0)).LengthSquared();
                        else
                        {
                            var p0 = PatchPoint(scratch, s0); var p1 = PatchPoint(scratch, s1); var p2 = PatchPoint(scratch, s2); var normal = new Vector3F(seed.Nx, seed.Ny, seed.Nz);
                            float orientation = Vector3F.Dot(normal, Vector3F.Cross(p1 - p0, p2 - p0)) >= 0f ? 1f : -1f;
                            score = Math.Max(0f, Math.Max(-orientation * Vector3F.Dot(normal, Vector3F.Cross(p1 - p0, point - p0)), Math.Max(-orientation * Vector3F.Dot(normal, Vector3F.Cross(p2 - p1, point - p1)), -orientation * Vector3F.Dot(normal, Vector3F.Cross(p0 - p2, point - p2)))));
                        }
                        if (chosen < 0 || score > best) { best = score; chosen = key; }
                    }
                }
                if (chosen < 0 || (k > 0 && best <= 1e-12f)) break;
                if (k == 0) s0 = chosen; else if (k == 1) s1 = chosen; else if (k == 2) s2 = chosen; else s3 = chosen;
            }
            SortLocal(scratch, ref s0, ref s1, soupIsA, inverseOther); SortLocal(scratch, ref s2, ref s3, soupIsA, inverseOther);
            SortLocal(scratch, ref s0, ref s2, soupIsA, inverseOther); SortLocal(scratch, ref s1, ref s3, soupIsA, inverseOther); SortLocal(scratch, ref s1, ref s2, soupIsA, inverseOther);
            merged.Count = 0;
            CopyPatchPoint(scratch, s0, ref merged); CopyPatchPoint(scratch, s1, ref merged); CopyPatchPoint(scratch, s2, ref merged); CopyPatchPoint(scratch, s3, ref merged);
            for (int j = i + 1; j < end; j++) if (SameFlatPatch(in seed, in scratch.Out[j], soupIsA)) scratch.Out[j].Count = 0;
            scratch.Out[i] = merged;
        }
        int write = first;
        for (int j = first; j < end; j++) if (scratch.Out[j].Count != 0) scratch.Out[write++] = scratch.Out[j];
        scratch.OutCount = write;
        // A plane/material partition can be represented by a triangle outside the current query. Its canonical id need not follow candidate triangle order.
        for (int j = first + 1; j < write; j++)
        {
            var value = scratch.Out[j]; int k = j - 1;
            while (k >= first && (scratch.Out[k].SubA > value.SubA || (scratch.Out[k].SubA == value.SubA && scratch.Out[k].SubB > value.SubB))) { scratch.Out[k + 1] = scratch.Out[k]; k--; }
            scratch.Out[k + 1] = value;
        }
    }
    private static int CompareLocal(NarrowScratch scratch, int a, int b, bool soupIsA, QuaternionF inverseOther)
    {
        var x = scratch.FlatLocalPoints[a]; var y = scratch.FlatLocalPoints[b]; int order = x.X.CompareTo(y.X); if (order == 0) order = x.Y.CompareTo(y.Y); if (order == 0) order = x.Z.CompareTo(y.Z); return order;
    }
    private static void SortLocal(NarrowScratch scratch, ref int a, ref int b, bool soupIsA, QuaternionF inverseOther)
    {
        if (b < 0) return;
        if (a < 0 || CompareLocal(scratch, a, b, soupIsA, inverseOther) > 0) { int t = a; a = b; b = t; }
    }
    private static bool SameFlatPatch(in ContactOut a, in ContactOut b, bool soupIsA) => b.Count != 0 && b.FlatPatch && a.FlatPlane == b.FlatPlane && a.Nx == b.Nx && a.Ny == b.Ny && a.Nz == b.Nz && a.SubA == b.SubA && a.SubB == b.SubB;
    private static Vector3F PatchPoint(NarrowScratch scratch, int key)
    {
        ref var c = ref scratch.Out[key / 4]; int p = key % 4 * 7; return new(c.P[p], c.P[p + 1], c.P[p + 2]);
    }
    private static void CopyPatchPoint(NarrowScratch scratch, int key, ref ContactOut destination)
    {
        if (key < 0) return;
        ref var source = ref scratch.Out[key / 4]; int p = key % 4;
        int k = destination.Count++;
        for (int f = 0; f < 7; f++) destination.P[k * 7 + f] = source.P[p * 7 + f];
        destination.Id[k] = 0xFC000000u + (uint)k;
    }
}
