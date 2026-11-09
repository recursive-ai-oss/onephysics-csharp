using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

internal sealed class RegionSelection
{
    internal readonly bool[] Bodies;
    internal readonly SortedDictionary<string, SortedSet<int>> Entities = new(StringComparer.Ordinal);
    internal RegionSelection(int bodies) => Bodies = new bool[bodies];
    internal bool Add(string kind, int index)
    {
        if (index < 0) { return false; }
        if (!Entities.TryGetValue(kind, out var set)) { set = new(); Entities.Add(kind, set); }
        return set.Add(index);
    }
    internal bool Contains(string kind, int index) => Entities.TryGetValue(kind, out var set) && set.Contains(index);
    internal bool AddBody(int index)
    {
        if ((uint)index >= (uint)Bodies.Length || Bodies[index]) { return false; }
        Bodies[index] = true; Add("body", index); return true;
    }
}

internal static class SnapshotRegions
{
    internal static bool Inside(ReadOnlySpan<double> p, ReadOnlySpan<double> min, ReadOnlySpan<double> max)
    { for (int i = 0; i < min.Length; i++) { if (p[i] < min[i] || p[i] > max[i]) { return false; } } return true; }

    internal static RegionSelection Seed(WorldCore core, double[] min, double[] max, RegionSnapshot? archive = null)
    {
        RegionSnapshot.ValidateBounds(min, max);
        var s = new RegionSelection(core.Bodies.HighWater);
        if (archive == null)
        {
            for (int b = 0; b < s.Bodies.Length; b++)
            { if (core.Bodies.IsLive(b) && Inside(core.Bodies.PositionOf(b), min, max)) { s.AddBody(b); } }
        }
        else
        {
            if (archive.Dimension != core.Layout.PositionLanes) { throw new SnapshotFormatException("region dimension differs from the world", 8); }
            foreach (var field in archive.Description.Fields)
            {
                if (field.Entity == "body")
                {
                    if ((uint)field.Index >= (uint)core.Bodies.HighWater || !core.Bodies.IsLive(field.Index) || field.Generation != core.Bodies.Handles.GenerationOf(field.Index))
                    { throw new SnapshotFormatException("region body handle is missing, dead or has a different generation; use a full snapshot", field.Offset); }
                    s.AddBody(field.Index);
                }
                if (field.Entity is "rod" or "cable" or "softbody" or "character" or "vehicle" or "ragdoll" or "fluid" or "thruster" or "boat" or "hovercraft" or "aero" or "bicycle" or "track" or "railvehicle") { s.Add(field.Entity, field.Index); }
            }
        }
        return s;
    }

    internal static bool CloseConstraints(WorldCore core, RegionSelection s)
    {
        bool changed = false; var cs = core.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c)) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            bool da = a >= 0 && s.Bodies[a] && core.Bodies.IsDynamic(a);
            bool db = b >= 0 && s.Bodies[b] && core.Bodies.IsDynamic(b);
            if (da || db) { changed |= s.AddBody(a); changed |= s.AddBody(b); }
        }
        return changed;
    }

    internal static void Complete(WorldCore core, RegionSelection s)
    {
        for (int b = 0; b < s.Bodies.Length; b++)
        { if (s.Bodies[b]) { s.Add("body", b); s.Add("island", core.Bodies.Island[b]); } }
        for (int sh = 0; sh < core.Shapes.HighWater; sh++)
        { if (core.Shapes.Handles.IsLive(sh) && s.Bodies[core.Shapes.Owner[sh]]) { s.Add("shape", sh); } }
        var cs = core.Constraints;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c)) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            if (a >= 0 && s.Bodies[a] && (b < 0 || s.Bodies[b]))
            { s.Add("constraint", c); s.Add(cs.KindOf(c) == ConstraintKind.Joint ? "joint" : "contact", c); }
        }
    }

    internal static List<SnapshotLocation> Select(SnapshotDescription d, RegionSelection s)
    {
        var selected = new List<SnapshotLocation>();
        var eventIndices = new Dictionary<(string, int), int>(); int delivered = 0, pending = 0;
        foreach (var f in d.Fields)
        {
            if (f.Entity == "shape" && f.Expression == "Proxy[s]") { continue; } // Local tree node identity is derived, not a regional handle.
            if (f.Section.StartsWith("broadphase", StringComparison.Ordinal)) { continue; } // Derived spatial index, rebuilt on regional restore.
            if (f.Entity is "event" or "pending event")
            {
                bool a = f.RelatedBodyA >= 0 && f.RelatedBodyA < s.Bodies.Length && s.Bodies[f.RelatedBodyA];
                bool b = f.RelatedBodyB >= 0 && f.RelatedBodyB < s.Bodies.Length && s.Bodies[f.RelatedBodyB];
                if (!a && !b) { continue; }
                var key = (f.Entity, f.Index);
                if (!eventIndices.TryGetValue(key, out int local)) { local = f.Entity == "event" ? delivered++ : pending++; eventIndices.Add(key, local); }
                selected.Add(f with { Index = local });
            }
            else if (s.Contains(f.Entity, f.Index)) { selected.Add(f); }
            else if (IsContext(f)) { selected.Add(f); }
        }
        return selected;
    }

    internal static bool IsContext(SnapshotLocation f) => f.Entity == "world" &&
        (f.Section is "options" or "world" || f.Section.StartsWith("section ", StringComparison.Ordinal) || f.Expression.StartsWith("Opt.", StringComparison.Ordinal) || f.Expression.StartsWith("LayerMatrix", StringComparison.Ordinal) || f.Expression == "FilterDirty");

    internal static byte[] Patch(SnapshotDescription target, List<SnapshotLocation> targetSelection, RegionSnapshot archive)
    {
        var a = SnapshotDesync.Keys(new List<SnapshotLocation>(archive.Description.Fields.FindAll(f => f.Section != "region")));
        var b = SnapshotDesync.Keys(targetSelection);
        if (a.Count != b.Count) { throw new SnapshotFormatException("region topology/context field set differs; use a full snapshot", 0); }
        var bytes = (byte[])target.Bytes.Clone();
        foreach (var pair in a)
        {
            var source = pair.Value;
            if (!b.TryGetValue(pair.Key, out var dest) || source.Type != dest.Type || source.Size != dest.Size)
            { throw new SnapshotFormatException("region field/handle topology differs: " + pair.Key + "; use a full snapshot", 0); }
            var value = archive.Description.Bytes.AsSpan(source.Offset, source.Size);
            if (IsContext(dest) && !value.SequenceEqual(target.Bytes.AsSpan(dest.Offset, dest.Size)))
            { throw new SnapshotFormatException("region tick/origin/options differ: " + dest.Field + "; use a full snapshot", 0); }
            value.CopyTo(bytes.AsSpan(dest.Offset, dest.Size));
        }
        return bytes;
    }
}
