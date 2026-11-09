using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

// The fixed-size shape record stays unchanged; only worlds using mass offsets carry this section.
internal sealed class ShapeMassOffsets2D(Engine2D engine) : ISnapshotSection
{
    private Vec2[] _staging = System.Array.Empty<Vec2>();
    public int SectionId => 0x324D4F43;
    public bool IsOptional => true;
    public bool IsPresent => Count() != 0;
    private int Count()
    {
        int count = 0;
        for (int s = 0; s < engine.Core.Shapes.HighWater; s++)
            if (engine.Core.Shapes.Handles.IsLive(s) && (engine.Geom[s]._massOffset.X != 0 || engine.Geom[s]._massOffset.Y != 0)) count++;
        return count;
    }
    public void Write(StateWriter w)
    {
        w.NamedInt32(1, "version"); w.NamedInt32(Count(), "count");
        for (int s = 0; s < engine.Core.Shapes.HighWater; s++)
        {
            var d = engine.Geom[s]._massOffset;
            if (!engine.Core.Shapes.Handles.IsLive(s) || (d.X == 0 && d.Y == 0)) continue;
            w.DescribeEntity("shape", s, engine.Core.Shapes.Handles.GenerationOf(s));
            w.NamedInt32(s, "slot"); w.NamedSingle(d.X); w.NamedSingle(d.Y);
        }
    }
    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) throw r.Error("unsupported planar mass-offset section version");
        int count = r.ReadCount(12, "planar shape mass offsets");
        if (count == 0 || count > engine.RestoredShapeCount) throw r.Error("invalid mass-offset count");
        if (_staging.Length < engine.RestoredShapeCount) System.Array.Resize(ref _staging, engine.RestoredShapeCount);
        System.Array.Clear(_staging);
        int previous = -1;
        for (int i = 0; i < count; i++)
        {
            int s = r.ReadInt32(); var d = new Vec2(r.ReadSingle(), r.ReadSingle());
            if (s <= previous || !engine.RestoredShapeIsLive(s)) throw r.Error("invalid or unordered mass-offset shape slot");
            if (!float.IsFinite(d.X) || !float.IsFinite(d.Y) || float.Abs(d.X) > Constants2D.MaxShapeCoordinate || float.Abs(d.Y) > Constants2D.MaxShapeCoordinate || (d.X == 0 && d.Y == 0)) throw r.Error("invalid planar shape mass offset");
            if (engine.Geom[s].ComputeMass().Area == 0) throw r.Error("mass offset on zero-area shape");
            _staging[s] = d; previous = s;
        }
        Reset();
        for (int s = 0; s < engine.RestoredShapeCount; s++) engine.Geom[s]._massOffset = _staging[s];
    }
    public void Reset()
    {
        for (int s = 0; s < engine.Geom.Length; s++) engine.Geom[s]._massOffset = default;
    }
    public ISnapshotSection Fork(WorldCore destination) => new ShapeMassOffsets2D((Engine2D)destination.GetSnapshotSection(engine.SectionId));
}
