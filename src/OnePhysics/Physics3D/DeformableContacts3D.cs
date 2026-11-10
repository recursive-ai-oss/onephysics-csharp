using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IContactSamples
{
    internal readonly DeformableContactTracker DeformableContacts = new();

    void IContactSamples.GatherContactSamples(List<ContactGroup> groups, List<ContactSample> samples)
    {
        for (int slot = 0; slot < SoftHandles.HighWater; slot++)
        {
            if (!SoftHandles.IsLive(slot)) { continue; }
            var d = SoftData[slot]!;
            if (d.N == 0) { continue; }
            groups.Add(new ContactGroup { Kind = SoftKindOf(d), Slot = slot, Generation = SoftHandles.GenerationOf(slot), Layer = d.Def.Layer, Group = d.Def.GroupIndex, Category = d.Def.CategoryBits, Mask = d.Def.MaskBits, First = samples.Count, Count = d.N });
            for (int p = 0; p < d.N; p++) { samples.Add(new ContactSample { X = d.Pos[3 * p], Y = d.Pos[3 * p + 1], Z = d.Pos[3 * p + 2], Radius = d.Radius }); }
        }

        var rods = _rods;
        if (rods == null) { return; }
        for (int slot = 0; slot < rods.Data.Length; slot++)
        {
            var r = rods.Data[slot];
            if (r == null) { continue; }
            var pp = r.PositionData;
            groups.Add(new ContactGroup { Kind = DeformableKind.Rod, Slot = slot, Generation = rods.Generations[slot], Layer = r.Layer, Group = r.Group, Category = r.Category, Mask = r.Mask, First = samples.Count, Count = 2 * pp.Length - 1 });
            float rad = r.Material.Radius;
            for (int i = 0; i < pp.Length; i++)
            {
                samples.Add(new ContactSample { X = pp[i].X, Y = pp[i].Y, Z = pp[i].Z, Radius = rad });
                if (i + 1 < pp.Length) { samples.Add(new ContactSample { X = 0.5 * (pp[i].X + pp[i + 1].X), Y = 0.5 * (pp[i].Y + pp[i + 1].Y), Z = 0.5 * (pp[i].Z + pp[i + 1].Z), Radius = rad }); }
            }
        }
    }
}

public sealed partial class PhysicsWorld3D
{
    private DeformableContactEvent[] _deformableContactScratch = Array.Empty<DeformableContactEvent>();

    /// <summary>
    /// Whether soft bodies, cloth and rods report when they begin or end touching rigid bodies (<see cref="DeformableContactEvents"/>, <see cref="DeformableContactChanged"/>). Default false: finding the contacts costs a shape query
    /// per deformable per step. This is a host setting, not world state: it is not saved or forked, so set it again on a restored or forked world.
    /// </summary>
    public bool TrackDeformableContacts { get => _engine.DeformableContacts.Enabled; set => _engine.DeformableContacts.Enabled = value; }

    /// <summary>The deformable contact events of the last step (empty unless <see cref="TrackDeformableContacts"/> is on); valid until the next step, rebuilt on each read.</summary>
    public ReadOnlySpan<DeformableContactEvent> DeformableContactEvents
    {
        get
        {
            var ev = _engine.DeformableContacts.Events;
            if (ev.Count == 0) { return default; }
            if (_deformableContactScratch.Length < ev.Count) { _deformableContactScratch = new DeformableContactEvent[Math.Max(ev.Count, 2 * _deformableContactScratch.Length)]; }
            for (int i = 0; i < ev.Count; i++)
            {
                var e = ev[i];
                _deformableContactScratch[i] = new DeformableContactEvent(e.Began ? DeformableContactEventKind.Began : DeformableContactEventKind.Ended, e.Kind, e.Slot, e.Generation, new BodyHandle(Id, e.Body, e.BodyGeneration));
            }

            return _deformableContactScratch.AsSpan(0, ev.Count);
        }
    }

    /// <summary>Raised after a step for each deformable contact event (unless callbacks are suppressed), in the order of <see cref="DeformableContactEvents"/>.</summary>
    public event Action<DeformableContactEvent>? DeformableContactChanged;

    private void DispatchDeformableContactEvents()
    {
        var h = DeformableContactChanged;
        if (h == null) { return; }
        var events = DeformableContactEvents.ToArray();
        for (int i = 0; i < events.Length; i++) { h(events[i]); }
    }
}
