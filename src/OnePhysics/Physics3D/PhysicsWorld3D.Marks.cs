using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Saves the whole simulation state so <see cref="RollbackTo"/> can return to it: the state a snapshot holds (bodies, shapes, contacts and warm-start impulses, islands, sleep state, pending events and every feature's state), bit for bit.
    /// A rollback host marks once per frame and rolls back when a late input arrives. Several marks may be live; each keeps its own copy of the state, so a mark costs about as much memory as the world's used storage.
    /// Cost: a copy of the used part of the big stores plus the written form of whatever the optional features hold, with no validation and no parsing; storage is pooled, so once a mark has been released and the world has stopped growing,
    /// marking again allocates nothing. Release marks you no longer need with <see cref="ReleaseMark"/> or <see cref="ReleaseMarksBefore"/>.
    /// Handlers and callbacks are not part of the state (as with a snapshot). Not allowed to change the world; takes no part in a recording.
    /// </summary>
    /// <returns>The mark, valid until it is released or a rollback to an earlier mark passes it.</returns>
    /// <exception cref="NonFiniteStateException">A position or the origin is not finite (the world cannot be saved).</exception>
    public WorldMark Mark()
    {
        _remembersDestroyed = true;
        _core.MarkState(out int slot, out int generation);
        return new WorldMark(Id, slot, generation, _core.StepCount);
    }

    /// <summary>
    /// Returns the world to the state <paramref name="mark"/> saved. Afterwards a snapshot, <see cref="ComputeHash"/> and every later step are bit for bit those of the world at <see cref="Mark"/>, whatever happened since
    /// (steps, bodies created or destroyed, options and inputs changed, restores). The mark stays valid, so you can roll back to it again; every mark taken after it is released. Marks taken before it stay.
    /// Handles taken before the mark stay valid in the world when their bodies exist again, as after <see cref="Restore(ReadOnlySpan{byte})"/>, and callbacks of handles that exist again are given back.
    /// Cost: a copy of the used part of the big stores plus a read of the sections the mark saved; no validation. Allocates nothing once the world's storage has the marked size.
    /// A recording sees one mutation, as for a restore.
    /// </summary>
    /// <param name="mark">A mark taken from this world and still valid.</param>
    /// <returns>The saved options whose value the rollback changed; see <see cref="OptionChanges"/>.</returns>
    /// <exception cref="InvalidHandleException">The mark is null or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">The mark was released or passed by a rollback to an earlier mark.</exception>
    public OptionChanges RollbackTo(WorldMark mark)
    {
        CheckMark(mark);
        using var replayMutation = RecordMutation("PhysicsWorld3D.RollbackTo");
        var before = _engine.Opt;
        var oldGravity = before.Gravity;
        int oldSubsteps = before.Substeps, oldVelocity = before.VelocityIterations, oldRelax = before.RelaxIterations;
        int oldCSub = before.ContactSubsteps, oldCVel = before.ContactVelocityIterations, oldCRelax = before.ContactRelaxIterations;
        bool oldSleep = before.SleepEnabled;
        float oldTimeToSleep = before.TimeToSleep, oldThreshold = before.SleepThreshold;
        var oldEvents = before.Events;
        _core.RollbackToMark(mark.Slot, mark.Generation);
        LoadOptionsFromCore();
        var after = _engine.Opt;
        var changed = OptionChanges.None;
        if (after.Gravity != oldGravity) { changed |= OptionChanges.Gravity; }
        if (after.Substeps != oldSubsteps) { changed |= OptionChanges.Substeps; }
        if (after.VelocityIterations != oldVelocity) { changed |= OptionChanges.VelocityIterations; }
        if (after.RelaxIterations != oldRelax) { changed |= OptionChanges.RelaxIterations; }
        if (after.ContactSubsteps != oldCSub || after.ContactVelocityIterations != oldCVel || after.ContactRelaxIterations != oldCRelax) { changed |= OptionChanges.ContactSchedule; }
        if (after.SleepEnabled != oldSleep) { changed |= OptionChanges.SleepEnabled; }
        if (BitConverter.SingleToInt32Bits(after.TimeToSleep) != BitConverter.SingleToInt32Bits(oldTimeToSleep)) { changed |= OptionChanges.TimeToSleep; }
        if (BitConverter.SingleToInt32Bits(after.SleepThreshold) != BitConverter.SingleToInt32Bits(oldThreshold)) { changed |= OptionChanges.SleepThreshold; }
        if (after.Events != oldEvents) { changed |= OptionChanges.Events; }
        ReconcileHandlersAfterRestore();
        InvalidateQueryPad();
        _eventsPending = false;
        BuildPublicEvents();
        return changed;
    }

    /// <summary>Frees the storage behind <paramref name="mark"/> for the next <see cref="Mark"/>. Returns false when the mark was not valid (already released, passed by a rollback, null or from another world); never throws.</summary>
    public bool ReleaseMark(WorldMark mark) => !mark.IsNull && mark.World == Id && _core.ReleaseMark(mark.Slot, mark.Generation);

    /// <summary>Releases every live mark taken before <paramref name="mark"/>, which stays. The ring pattern: after marking frame n, release the marks older than frame n minus the ring size.</summary>
    /// <exception cref="InvalidHandleException">The mark is null or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">The mark is not valid.</exception>
    public void ReleaseMarksBefore(WorldMark mark)
    {
        CheckMark(mark);
        _core.ReleaseMarksBefore(mark.Slot, mark.Generation);
    }

    /// <summary>
    /// Releases the oldest live marks until at most <paramref name="keep"/> remain (the ring pattern: mark every frame, then keep the last N). Returns how many marks it released.
    /// Allocates nothing. Unlike <see cref="ReleaseMarksBefore"/> it needs no mark in hand, so it still works after a rollback has shortened the history.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="keep"/> is negative.</exception>
    public int ReleaseOldMarks(int keep)
    {
        if (keep < 0) { throw new ArgumentOutOfRangeException(nameof(keep), keep, "The number of marks to keep cannot be negative."); }
        return _core.ReleaseOldMarks(keep);
    }

    /// <summary>Releases every mark and the memory behind them. Marks taken before are invalid afterwards.</summary>
    public void ReleaseAllMarks() => _core.TrimMarks();

    /// <summary>The number of marks that are valid now.</summary>
    public int LiveMarkCount => _core.LiveMarkCount;

    /// <summary>True while <paramref name="mark"/> is valid in this world (never throws).</summary>
    public bool IsMarkValid(WorldMark mark) => !mark.IsNull && mark.World == Id && _core.IsMarkLive(mark.Slot, mark.Generation);

    private void CheckMark(WorldMark mark)
    {
        if (mark.IsNull) { throw new InvalidHandleException("The mark is the default (null) mark."); }
        if (mark.World != Id) { throw new InvalidHandleException("The mark was taken from world " + mark.World.ToString(System.Globalization.CultureInfo.InvariantCulture) + ", not world " + Id.ToString(System.Globalization.CultureInfo.InvariantCulture) + "."); }
        if (!_core.IsMarkLive(mark.Slot, mark.Generation)) { throw new StaleHandleException("The mark of step " + mark.Step.ToString(System.Globalization.CultureInfo.InvariantCulture) + " was released or passed by a rollback to an earlier mark."); }
    }
}
