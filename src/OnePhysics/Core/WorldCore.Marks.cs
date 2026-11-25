using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// One saved state of a world, kept for a rollback: the big stores copied array for array (no serialization, no validation, storage reused from the last time the slot was used),
/// and every registered section that holds state as the bytes its own writer produces, so a section that is absent in the marked state costs nothing.
/// </summary>
internal sealed class CoreMark
{
    internal bool InUse;
    internal int Generation;
    internal long Sequence;
    internal long StepCount;
    internal double[] Origin = Array.Empty<double>();
    internal WorldOptions? Options;
    internal BodyStore? Bodies;
    internal ShapeStore? Shapes;
    internal ConstraintStore? Constraints;
    internal IslandSet? Islands;
    internal EventCollector? Events;
    internal ConstraintColoring? Coloring;
    internal StateWriter Sections = StateWriter.ToBuffer();
    internal int[] Range = Array.Empty<int>();
    internal bool[] Present = Array.Empty<bool>();
    internal int SectionCount;
}

public sealed partial class WorldCore
{
    private readonly List<CoreMark> _marks = new();
    private long _markSequence;

    /// <summary>The number of marks that can still be rolled back to.</summary>
    internal int LiveMarkCount
    {
        get { int n = 0; foreach (var m in _marks) { if (m.InUse) { n++; } } return n; }
    }

    /// <summary>
    /// Saves the state a rollback returns to. Everything a snapshot holds is saved, so after <see cref="RollbackToMark"/> a snapshot, a hash and every later step are bit for bit those of the marked state.
    /// Cost is a copy of the used part of the big stores plus the written form of the sections that hold state; storage is pooled per slot, so marking again after a release allocates nothing while the world has not grown.
    /// </summary>
    internal void MarkState(out int slot, out int generation)
    {
        FlushPending();
        CheckStateFinite();
        slot = -1;
        for (int i = 0; i < _marks.Count; i++) { if (!_marks[i].InUse) { slot = i; break; } }
        if (slot < 0) { _marks.Add(new CoreMark()); slot = _marks.Count - 1; }
        var m = _marks[slot];
        m.InUse = true;
        m.Generation++;
        m.Sequence = ++_markSequence;
        generation = m.Generation;
        try { Capture(m); }
        catch { m.InUse = false; throw; }
    }

    private void Capture(CoreMark m)
    {
        m.StepCount = _stepCount;
        ForkArrays.Into(ref m.Origin, _origin);
        if (m.Options == null) { m.Options = _options.Clone(); } else { m.Options.CopySimulationValuesFrom(_options); }
        (m.Bodies ??= new BodyStore(Layout, 1)).CopyFrom(_bodies);
        (m.Shapes ??= new ShapeStore()).CopyFrom(_shapes);
        (m.Constraints ??= new ConstraintStore(Layout, 1)).CopyFrom(_constraints);
        (m.Islands ??= new IslandSet()).CopyFrom(_islands);
        (m.Events ??= new EventCollector(_options)).CopyFrom(_events);
        (m.Coloring ??= new ConstraintColoring()).CopyForFork(_coloring);

        int registered = _sections.Count;
        if (m.Range.Length < 2 * registered) { m.Range = new int[2 * registered]; }
        if (m.Present.Length < registered) { m.Present = new bool[registered]; }
        m.SectionCount = registered;
        var w = m.Sections;
        w.Reset();
        for (int i = 0; i < registered; i++)
        {
            var s = _sections[i];
            if (s.IsOptional && !s.IsPresent) { m.Present[i] = false; continue; }
            m.Present[i] = true;
            int start = w.Length;
            s.Write(w);
            m.Range[2 * i] = start;
            m.Range[2 * i + 1] = w.Length - start;
        }
    }

    /// <summary>The slot's mark is live and has the generation.</summary>
    internal bool IsMarkLive(int slot, int generation) => slot >= 0 && slot < _marks.Count && _marks[slot].InUse && _marks[slot].Generation == generation;

    /// <summary>
    /// Puts the world back to the marked state. Marks taken after it are released; it and the earlier ones stay. A section that fails to read what its own writer wrote is a defect, not bad input, so the world is not guarded against it.
    /// </summary>
    internal void RollbackToMark(int slot, int generation)
    {
        if (!IsMarkLive(slot, generation)) { throw new InvalidOperationException("The mark is not live."); }
        var m = _marks[slot];
        FlushPostStep();
        foreach (var other in _marks) { if (other.InUse && other.Sequence > m.Sequence) { other.InUse = false; } }

        _options.CopySimulationValuesFrom(m.Options!);
        _stepCount = m.StepCount;
        ForkArrays.Into(ref _origin, m.Origin);
        _bodies.CopyFrom(m.Bodies!);
        _shapes.CopyFrom(m.Shapes!);
        _constraints.CopyFrom(m.Constraints!);
        _islands.CopyFrom(m.Islands!);
        _events.CopyFrom(m.Events!);
        _coloring.CopyForFork(m.Coloring!);

        int registered = _sections.Count;
        if (m.SectionCount != registered) { throw new InvalidOperationException("The sections registered after the mark was taken."); }
        var sub = _subReader ??= new StateReader(Array.Empty<byte>());
        byte[] data = m.Sections.Storage;
        // The length unit comes first, as in a restore: the other sections fall back to defaults in the unit the state names.
        int unitIndex = -1;
        for (int i = 0; i < registered; i++) { if (_sections[i].SectionId == LengthUnitSection.Id) { unitIndex = i; break; } }
        for (int k = 0; k < registered; k++)
        {
            int i = unitIndex < 0 ? k : k == 0 ? unitIndex : k <= unitIndex ? k - 1 : k;
            var s = _sections[i];
            if (!m.Present[i]) { s.Reset(); continue; }
            sub.Reset(data, m.Range[2 * i], m.Range[2 * i + 1]);
            if (ReferenceEquals(s, _bodySleep)) { _bodySleep.Read(sub, _bodies); }
            else if (ReferenceEquals(s, Controls)) { Controls.Read(sub, _bodies); }
            else if (ReferenceEquals(s, ScaleStore)) { ScaleStore.Read(sub, _bodies); }
            else { s.Read(sub); }
            if (sub.Remaining != 0) { throw new SnapshotFormatException("section " + s.SectionId.ToString(CultureInfo.InvariantCulture) + " left unread bytes", m.Range[2 * i]); }
        }

        Surfaces.ValidateReferences(_shapes);
        ContactMaterials.ValidateReferences(_shapes);
        if (ContactMotion.Enabled) { _events.EnrichContactMotion(ContactMotion); }
        _awakeCount = 0;
        PendingDiscard?.Invoke();
    }

    /// <summary>Frees the mark's storage for the next mark (the arrays stay allocated). Returns false when it was not live.</summary>
    internal bool ReleaseMark(int slot, int generation)
    {
        if (!IsMarkLive(slot, generation)) { return false; }
        _marks[slot].InUse = false;
        return true;
    }

    /// <summary>Frees every live mark taken before the given one.</summary>
    internal void ReleaseMarksBefore(int slot, int generation)
    {
        if (!IsMarkLive(slot, generation)) { throw new InvalidOperationException("The mark is not live."); }
        long sequence = _marks[slot].Sequence;
        foreach (var other in _marks) { if (other.InUse && other.Sequence < sequence) { other.InUse = false; } }
    }

    /// <summary>Releases the oldest live marks until at most <paramref name="keep"/> remain; returns how many it released.</summary>
    internal int ReleaseOldMarks(int keep)
    {
        int released = 0;
        while (LiveMarkCount > keep)
        {
            CoreMark? oldest = null;
            foreach (var m in _marks) { if (m.InUse && (oldest == null || m.Sequence < oldest.Sequence)) { oldest = m; } }
            oldest!.InUse = false;
            released++;
        }

        return released;
    }

    /// <summary>Releases every mark and the storage behind them (the slots keep their generations, so old marks stay invalid).</summary>
    internal void TrimMarks()
    {
        foreach (var m in _marks)
        {
            m.InUse = false;
            m.Origin = Array.Empty<double>(); m.Options = null; m.Bodies = null; m.Shapes = null; m.Constraints = null; m.Islands = null; m.Events = null; m.Coloring = null;
            m.Sections = StateWriter.ToBuffer(); m.Range = Array.Empty<int>(); m.Present = Array.Empty<bool>();
        }
    }
}
