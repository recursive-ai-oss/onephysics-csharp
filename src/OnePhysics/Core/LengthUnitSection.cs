using System;
using System.Globalization;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// The world's <see cref="LengthUnit"/>, saved only when it is not one metre, so worlds that never set it keep their snapshot bytes and hash. A restore of a snapshot without the section puts one metre back.
/// Layout: int32 version (1), double metres per unit. The reader accepts only a value <see cref="LengthUnit.IsValid"/> accepts.
/// </summary>
internal sealed class LengthUnitSection : ISnapshotSection
{
    /// <summary>"UNIT" read as a little-endian int32.</summary>
    internal const int Id = 0x54494E55;

    private readonly Func<LengthUnit> _get;
    private readonly Action<LengthUnit> _set;
    private readonly Func<WorldCore, ISnapshotSection> _fork;

    // fork builds the destination world's own section from its engine, as the other optional sections do.
    internal LengthUnitSection(Func<LengthUnit> get, Action<LengthUnit> set, Func<WorldCore, ISnapshotSection> fork)
    {
        _get = get; _set = set; _fork = fork;
    }

    public int SectionId => Id;
    public bool IsOptional => true;
    public bool IsPresent => !_get().IsMetres;

    public void Write(StateWriter w)
    {
        // Host configuration, not the state of any body, so a region snapshot leaves it alone.
        w.DescribeEntity("length unit", 0);
        w.NamedInt32(1, "version");
        w.NamedDouble(_get().MetresPerUnit, "metres per unit");
    }

    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) { throw r.Error("unsupported length unit section version"); }
        double m = r.ReadDouble();
        if (!LengthUnit.IsValid(m)) { throw r.Error("invalid length unit " + m.ToString("R", CultureInfo.InvariantCulture) + " metres per unit"); }
        if (m == 1.0) { throw r.Error("length unit section holds one metre; the snapshot would not re-serialize to the same bytes"); }
        _set(new LengthUnit(m));
    }

    public void Reset() => _set(LengthUnit.Metres);

    public ISnapshotSection Fork(WorldCore destination) => _fork(destination);
}
