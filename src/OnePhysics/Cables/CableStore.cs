using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Cables;

internal interface ICableHost
{
    WorldCore CableCore { get; }
    CableStore Cables { get; }
    int EngineSectionId { get; }
    bool Planar { get; }
    Vector3F Gravity { get; }
    float GravityScaleOf(int body);
    Vector3F LocalCenter(int body);
    /// <summary>Adds a world-space force (through the centre of mass) and torque that the next step integrates, then clears.</summary>
    /// <summary>The force and world-space torque already queued for the next step (user forces, fields), which the rigid step will integrate.</summary>
    void PendingLoad(int body, out Vector3F force, out Vector3F torque);
    void AddLoad(int body, Vector3F force, Vector3F torque);
    ReplayMutationScope CableMutation(string operation);
}

/// <summary>Every pulley system of a world: ropes over wheels, winches, belts and chains. Optional snapshot section, absent while the world holds none.</summary>
internal sealed partial class CableStore(ICableHost host) : ISnapshotSection
{
    internal const int Limit = 65536, NodeLimit = 256;
    internal readonly ICableHost Host = host;
    internal CableData?[] Data = [];
    internal int[] Generations = [];
    internal int Count;
    public int SectionId => 0x4341424C;
    public bool IsOptional => true;
    public bool IsPresent => Data.Length != 0;
    internal bool Alive(int slot, int generation) => (uint)slot < (uint)Data.Length && Data[slot] != null && Generations[slot] == generation;

    internal (int Slot, int Generation) Add(CableData data)
    {
        if (Data.Length >= Limit && Count == Data.Length) throw new InvalidOperationException("Pulley system limit 65536 reached.");
        using var mutation = Host.CableMutation("World.AddPulleySystem");
        int i = 0; while (i < Data.Length && Data[i] != null) i++;
        if (i == Data.Length) { Array.Resize(ref Data, i + 1); Array.Resize(ref Generations, i + 1); Generations[i] = 1; }
        Data[i] = data; Count++; return (i, Generations[i]);
    }

    internal void Remove(int slot)
    {
        if (Generations[slot] == int.MaxValue) throw new InvalidOperationException("Pulley system generation exhausted.");
        using var mutation = Host.CableMutation("PulleySystem.Destroy");
        Data[slot] = null; Generations[slot]++; Count--;
    }

    public void Reset() { Data = []; Generations = []; Count = 0; _scratch = null; }

    public ISnapshotSection Fork(WorldCore destination)
    {
        var copy = ((ICableHost)destination.GetSnapshotSection(Host.EngineSectionId)).Cables;
        copy.Data = new CableData?[Data.Length]; copy.Generations = (int[])Generations.Clone(); copy.Count = Count;
        for (int i = 0; i < Data.Length; i++) copy.Data[i] = Data[i]?.Copy();
        return copy;
    }

    /// <summary>Moves every world-anchored node by the origin shift; body-anchored nodes move with their bodies.</summary>
    internal void Shift(Vector3D offset)
    {
        foreach (var m in Data)
        {
            if (m == null) continue;
            foreach (var n in m.Nodes) if (n.Body < 0) n.Local -= offset;
        }
    }

    internal void CheckShift(Vector3D offset)
    {
        for (int s = 0; s < Data.Length; s++)
        {
            if (Data[s] is not { } m) continue;
            for (int i = 0; i < m.Nodes.Length; i++)
            {
                var n = m.Nodes[i]; if (n.Body >= 0) continue;
                var p = n.Local - offset;
                if (!(Math.Abs(p.X) <= 4194304 && Math.Abs(p.Y) <= 4194304 && Math.Abs(p.Z) <= 4194304))
                    throw new ArgumentOutOfRangeException(nameof(offset), offset, $"Shift would move node {i} of pulley system {s} outside ±4194304 metres.");
            }
        }
    }

    internal void SelectRegion(RegionSelection selection, double[] min, double[] max, RegionSnapshot? archive)
    {
        if (archive != null) return;
        for (int i = 0; i < Data.Length; i++)
        {
            if (Data[i] is not { } m) continue;
            foreach (var n in m.Nodes)
            {
                if (n.Body >= 0) continue;
                var p = n.Local;
                if (p.X >= min[0] && p.X <= max[0] && p.Y >= min[1] && p.Y <= max[1] && (Host.Planar || (p.Z >= min[2] && p.Z <= max[2]))) { selection.Add("cable", i); break; }
            }
        }
    }

    internal bool CloseRegion(RegionSelection selection)
    {
        bool changed = false;
        var bodies = Host.CableCore.Bodies;
        for (int s = 0; s < Data.Length; s++)
        {
            if (Data[s] is not { } m) continue;
            foreach (var n in m.Nodes)
            {
                if (n.Body < 0 || !bodies.Handles.IsAlive(n.Body, n.BodyGeneration)) continue;
                if (selection.Contains("cable", s)) changed |= selection.AddBody(n.Body);
                if (selection.Bodies[n.Body] && bodies.IsDynamic(n.Body)) changed |= selection.Add("cable", s);
            }
        }
        return changed;
    }
}
