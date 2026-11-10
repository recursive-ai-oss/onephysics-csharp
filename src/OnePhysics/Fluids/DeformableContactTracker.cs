using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Fluids;

/// <summary>One sphere a deformable touches the world with: a particle, or the middle of a rod segment.</summary>
internal struct ContactSample
{
    public double X, Y, Z;
    public float Radius;
}

/// <summary>One deformable's samples and the rules deciding what it may touch.</summary>
internal struct ContactGroup
{
    public DeformableKind Kind;
    public int Slot, Generation;
    public int Layer, Group;
    public uint Category, Mask;
    public int First, Count;   // into the sample list
}

/// <summary>What the tracker needs from an engine, besides the fluid host's shape routines: the samples to test.</summary>
internal interface IContactSamples
{
    /// <summary>Appends every soft body's, cloth's and rod's samples (soft bodies by slot, then rods by slot).</summary>
    void GatherContactSamples(List<ContactGroup> groups, List<ContactSample> samples);
}

/// <summary>
/// Finds which rigid bodies each soft body, cloth and rod touches, before and after a step, and reports the difference as events. Nothing here is state: the set before is found from the geometry as the step
/// begins, so a restored or forked world reports the same events as the original. Off unless a host turns it on; then it costs a shape query per deformable per step.
/// </summary>
internal sealed class DeformableContactTracker
{
    /// <summary>Contact distance, metres: a sample touches a shape when the gap between their surfaces is at most this.</summary>
    private const float Touch = 0.005f;

    public bool Enabled;
    public readonly List<DeformableContactRecord> Events = new();

    private readonly List<ContactGroup> _groups = new();
    private readonly List<ContactSample> _samples = new();
    private readonly List<int> _shapes = new();
    private readonly List<int> _bodies = new();
    private long[] _before = new long[16];
    private int _beforeCount;
    private long[] _after = new long[16];
    private int _afterCount;
    private readonly List<ContactGroup> _beforeGroups = new();

    // Key: deformable kind, slot, body slot: sorts soft bodies and cloth first (in slot order), then rods.
    private static long Key(DeformableKind kind, int slot, int body) => ((long)(kind == DeformableKind.Rod ? 1 : 0) << 62) | ((long)slot << 32) | (uint)body;

    public void Begin(IFluidHost host, IContactSamples source)
    {
        Events.Clear();
        if (!Enabled) { return; }
        Find(host, source, ref _before, out _beforeCount);
        _beforeGroups.Clear(); _beforeGroups.AddRange(_groups);
    }

    public void End(IFluidHost host, IContactSamples source)
    {
        if (!Enabled) { return; }
        Find(host, source, ref _after, out _afterCount);
        var bodies = host.Core.Bodies;
        int i = 0, j = 0;
        while (i < _beforeCount || j < _afterCount)
        {
            long a = i < _beforeCount ? _before[i] : long.MaxValue, b = j < _afterCount ? _after[j] : long.MaxValue;
            long key = Math.Min(a, b);
            bool inBefore = a == key, inAfter = b == key;
            if (inBefore) { i++; }
            if (inAfter) { j++; }
            if (inBefore == inAfter) { continue; }
            int slot = (int)((key >> 32) & 0x3FFFFFFF), body = (int)(key & 0xFFFFFFFF);
            bool rod = (key >> 62) != 0;
            if (!bodies.Handles.IsLive(body)) { continue; }
            var kind = DeformableKind.SoftBody;
            int generation = 0;
            // The kind and generation come from whichever of the two lists still names the deformable.
            if (!Lookup(_groups, rod, slot, out kind, out generation) && !Lookup(_beforeGroups, rod, slot, out kind, out generation)) { continue; }
            Events.Add(new DeformableContactRecord { Began = inAfter, Kind = kind, Slot = slot, Generation = generation, Body = body, BodyGeneration = bodies.Handles.GenerationOf(body) });
        }
    }

    private static bool Lookup(List<ContactGroup> groups, bool rod, int slot, out DeformableKind kind, out int generation)
    {
        for (int i = 0; i < groups.Count; i++)
        {
            var g = groups[i];
            if (g.Slot == slot && (g.Kind == DeformableKind.Rod) == rod) { kind = g.Kind; generation = g.Generation; return true; }
        }

        kind = default; generation = 0;
        return false;
    }

    private void Find(IFluidHost host, IContactSamples source, ref long[] keys, out int count)
    {
        _groups.Clear(); _samples.Clear();
        source.GatherContactSamples(_groups, _samples);
        count = 0;
        for (int g = 0; g < _groups.Count; g++)
        {
            var grp = _groups[g];
            if (grp.Count == 0) { continue; }
            double mnx = double.MaxValue, mny = double.MaxValue, mnz = double.MaxValue, mxx = -double.MaxValue, mxy = -double.MaxValue, mxz = -double.MaxValue;
            float rmax = 0f;
            for (int k = 0; k < grp.Count; k++)
            {
                var s = _samples[grp.First + k];
                if (s.X < mnx) { mnx = s.X; } if (s.X > mxx) { mxx = s.X; } if (s.Y < mny) { mny = s.Y; } if (s.Y > mxy) { mxy = s.Y; } if (s.Z < mnz) { mnz = s.Z; } if (s.Z > mxz) { mxz = s.Z; }
                if (s.Radius > rmax) { rmax = s.Radius; }
            }

            double pad = rmax + 0.05;
            _shapes.Clear();
            host.GatherShapes(mnx - pad, mny - pad, mnz - pad, mxx + pad, mxy + pad, mxz + pad, _shapes);
            _bodies.Clear();
            for (int si = 0; si < _shapes.Count; si++)
            {
                int shape = _shapes[si];
                if (!host.ShapeCollides(shape, grp.Layer, grp.Group, grp.Category, grp.Mask)) { continue; }
                int body = host.OwnerOf(shape);
                if (_bodies.Contains(body)) { continue; }
                host.LoadBody(body, out var pose);
                host.ShapeFrame(shape, in pose, out var frame);
                for (int k = 0; k < grp.Count; k++)
                {
                    var s = _samples[grp.First + k];
                    if (host.CollideSphere(shape, in frame, s.X, s.Y, s.Z, s.Radius, Touch, out var hit) && hit.Separation <= Touch) { _bodies.Add(body); break; }
                }
            }

            _bodies.Sort();
            if (count + _bodies.Count > keys.Length) { Array.Resize(ref keys, Math.Max(keys.Length * 2, count + _bodies.Count)); }
            for (int bi = 0; bi < _bodies.Count; bi++) { keys[count++] = Key(grp.Kind, grp.Slot, _bodies[bi]); }
        }

        // Soft bodies and cloth interleave by slot while their keys are the same class, so the list is sorted once at the end.
        Array.Sort(keys, 0, count);
    }
}

/// <summary>One contact event as the tracker collects it.</summary>
internal struct DeformableContactRecord
{
    public bool Began;
    public DeformableKind Kind;
    public int Slot, Generation, Body, BodyGeneration;
}
