using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.LowLevel;

public partial class Broadphase<TBox> where TBox : struct, IBox<TBox>
{
    /// <summary>Creates the same kind of empty broadphase for structural copying. Subclasses with additional state must override Fork as well.</summary>
    protected virtual Broadphase<TBox> CreateForkInstance() => new(_options, SectionId);

    /// <summary>Structurally copies all tree nodes, proxies, pairs and pending moves. The options are immutable and may be shared. The destination parameter is for section dependencies; this section has none.</summary>
    public virtual ISnapshotSection Fork(WorldCore destination)
    {
        var copy = CreateForkInstance();
        copy.CopyFrom(this);
        return copy;
    }

    internal void CopyFrom(Broadphase<TBox> s)
    {
        _moved = (int[])s._moved.Clone();
        _movedCount = s._movedCount;
        _pairs = (ProxyPair[])s._pairs.Clone();
        _pairCount = s._pairCount;
        s._refitsSinceRebuild.CopyTo(_refitsSinceRebuild, 0);
        _sortKeys = (long[])s._sortKeys.Clone();
        for (int i = 0; i < _trees.Length; i++) { _trees[i].CopyFrom(s._trees[i]); }
        _moves.AddRange(s._moves);
        _moveStage.AddRange(s._moveStage);
    }
}
