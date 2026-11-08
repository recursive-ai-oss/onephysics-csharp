using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    /// <summary>Structurally copies this core and all registered sections without serialization. Arrays are independent; immutable geometry may be shared. Keeps numeric world id and options. Custom sections must implement ISnapshotSection.Fork. No callback subscriptions are copied by the built-in engines.</summary>
    /// <exception cref="NotSupportedException">A custom section cannot fork.</exception>
    public WorldCore Fork()
    {
        var copy = new WorldCore(_options);
        copy.CopyFrom(this);
        return copy;
    }

    /// <summary>Returns a registered section by stable id. Useful to bind dependencies while structurally copying a custom section. Registration order is preserved by Fork.</summary>
    /// <exception cref="System.Collections.Generic.KeyNotFoundException">The id has not been registered.</exception>
    public ISnapshotSection GetSnapshotSection(int id)
    {
        foreach (var section in _sections) { if (section.SectionId == id) { return section; } }
        throw new System.Collections.Generic.KeyNotFoundException("No snapshot section " + id.ToString(System.Globalization.CultureInfo.InvariantCulture) + " is registered.");
    }

    internal System.Collections.Generic.IReadOnlyList<ISnapshotSection> SnapshotSections => _sections;

    internal void CopyFrom(WorldCore s)
    {
        ContactModifications.CopyState(s.ContactModifications);
        Surfaces.CopyFrom(s.Surfaces);
        ContactMaterials.CopyFrom(s.ContactMaterials);
        ContactMotion.CopyFrom(s.ContactMotion);
        _options.CopySimulationValuesFrom(s._options);
        _origin = (double[])s._origin.Clone();
        _stepCount = s._stepCount;
        _awake = (int[])s._awake.Clone();
        _awakeCount = s._awakeCount;
        _liteOn = s._liteOn;
        _liteSubsteps = s._liteSubsteps;
        _liteVelocity = s._liteVelocity;
        _liteRelax = s._liteRelax;
        _islandFull = (byte[])s._islandFull.Clone();
        _islandJoints = (int[])s._islandJoints.Clone();
        _consLite = (byte[])s._consLite.Clone();
        _awakeFull = (int[])s._awakeFull.Clone();
        _awakeLite = (int[])s._awakeLite.Clone();
        _awakeFullCount = s._awakeFullCount;
        _awakeLiteCount = s._awakeLiteCount;
        _forcedFull = (int[])s._forcedFull.Clone();
        _forcedCount = s._forcedCount;
        _partTmp = (int[])s._partTmp.Clone();
        _awakeBlockCounts = (int[])s._awakeBlockCounts.Clone();
        s._tStart.CopyTo(_tStart, 0);
        s._tCount.CopyTo(_tCount, 0);
        s._progConstraints.CopyTo(_progConstraints, 0);
        _pbStart = (int[])s._pbStart.Clone();
        _pbCount = (int[])s._pbCount.Clone();
        _pbFirst = (int[])s._pbFirst.Clone();
        _pbProg = (int[])s._pbProg.Clone();
        _flipSweep = s._flipSweep;
        _stageKind = (int[])s._stageKind.Clone();
        _stageArg = (int[])s._stageArg.Clone();
        _stageTasks = (int[])s._stageTasks.Clone();
        _stageCount = s._stageCount;
        _stageMostTasks = s._stageMostTasks;
        _sleepRested = (int[])s._sleepRested.Clone();
        _inputBuffer = (byte[])s._inputBuffer.Clone();
        _inputLength = s._inputLength;
        _sectionRange = (int[])s._sectionRange.Clone();
        _bodies.CopyFrom(s._bodies);
        _shapes.CopyFrom(s._shapes);
        _constraints.CopyFrom(s._constraints);
        _islands.CopyFrom(s._islands);
        _events.CopyFrom(s._events);
        _coloring.CopyForFork(s._coloring);
        SplitWhenAnyBodyRests = s.SplitWhenAnyBodyRests;
        SymmetricSweep = s.SymmetricSweep;
        foreach (var section in s._sections)
        {
            bool present = false;
            foreach (var existing in _sections) { if (existing.SectionId == section.SectionId) { present = true; break; } }
            if (!present)
            {
                var copy = section.Fork(this);
                if (copy == null || ReferenceEquals(copy, section) || copy.SectionId != section.SectionId) { throw new InvalidOperationException("A custom section fork must return an independent section with the same id."); }
                AddSection(copy);
            }
        }
    }
}
