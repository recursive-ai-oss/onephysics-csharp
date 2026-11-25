using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The speculative contact distance of a 2D world (<see cref="WorldOptions2D.SpeculativeDistance"/>), saved only when it is not the default so that worlds that never change it keep their snapshot bytes and hash.
/// A restore of a snapshot without the section puts the default back.
/// </summary>
internal sealed class SpeculativeDistance2D(Engine2D engine) : ISnapshotSection
{
    public int SectionId => 0x43455053;
    public bool IsOptional => true;
    public bool IsPresent => System.BitConverter.SingleToInt32Bits(engine.Opt.SpeculativeDistance) != System.BitConverter.SingleToInt32Bits(DefaultForUnit());

    private float DefaultForUnit() => WorldOptions2D.DefaultSpeculativeDistance * engine.Opt.UnitsPerMetre;


    public void Write(StateWriter w)
    {
        // Host configuration, not the state of any body, so a region snapshot leaves it alone.
        w.DescribeEntity("speculative", 0);
        w.NamedInt32(1, "version");
        w.NamedSingle(engine.Opt.SpeculativeDistance, "speculative distance");
    }

    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) { throw r.Error("unsupported speculative distance section version"); }
        float v = r.ReadSingle();
        // The unit section is read first, so the bound is the one of the snapshot's own unit.
        if (!(v >= 0f && v <= engine.Opt.SpeculativeLimit)) { throw r.Error("invalid speculative distance"); }
        engine.Opt.SpeculativeDistance = v;
    }

    public void Reset() => engine.Opt.SpeculativeDistance = DefaultForUnit();

    public ISnapshotSection Fork(WorldCore destination) => new SpeculativeDistance2D((Engine2D)destination.GetSnapshotSection(engine.SectionId));
}
