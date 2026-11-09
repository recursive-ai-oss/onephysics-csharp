using System;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Water;

internal sealed partial class WaterStore
{
    int ISnapshotSection.SectionId => Id;

    bool ISnapshotSection.IsOptional => true;

    bool ISnapshotSection.IsPresent => Handles.Count > 0;

    void ISnapshotSection.Reset()
    {
        Handles.Reset();
        Array.Clear(Grids);
        Events.Clear();
        _pendingCount = 0;
    }

    private const int SectionVersion = 1;

    /// <summary>Written while some grid holds water displaced by soft bodies, cloth or rods: each grid then ends with those per-cell volumes.</summary>
    private const int HoldSectionVersion = 2;

    private bool AnyHold()
    {
        for (int i = 0; i < Handles.HighWater; i++) { if (Handles.IsLive(i) && Grids[i]!.HoldCount > 0) { return true; } }
        return false;
    }

    void ISnapshotSection.Write(StateWriter w)
    {
        bool hold = AnyHold();
        w.NamedInt32(hold ? HoldSectionVersion : SectionVersion);
        w.DescribeTable("shallow water");
        Handles.Write(w);
        for (int i = 0; i < Handles.HighWater; i++)
        {
            w.DescribeEntity("shallow water", i, Handles.GenerationOf(i));
            if (!Handles.IsLive(i)) { continue; }
            var g = Grids[i]!;
            w.NamedBool(g.Planar); w.NamedInt32(g.Nx); w.NamedInt32(g.Nz); w.NamedSingle(g.Dx);
            w.NamedDouble(g.OriginX); w.NamedDouble(g.OriginZ); w.NamedDouble(g.BaseY);
            w.NamedSingle(g.Density); w.NamedSingle(g.LinearDrag); w.NamedSingle(g.AngularDrag); w.NamedSingle(g.ViscousDamping); w.NamedSingle(g.BottomFriction); w.NamedSingle(g.Damping); w.NamedSingle(g.Displacement);
            w.NamedSingle(g.Cfl); w.NamedInt32(g.MaxSubsteps);
            for (int e = 0; e < 4; e++) { w.NamedByte((byte)g.Edges[e]); w.NamedSingle(g.EdgeLevel[e]); w.NamedSingle(g.EdgeSpeed[e]); }
            w.NamedInt32(g.Sources.Length);
            foreach (var s in g.Sources) { w.NamedSingle(s.X); w.NamedSingle(s.Z); w.NamedSingle(s.Radius); w.NamedSingle(s.Rate); }
            w.NamedDouble(g.LostVolume);
            w.NamedSingles(g.Bed);
            w.NamedSingles(g.H);
            w.NamedSingles(g.Hu);
            w.NamedSingles(g.Hv);
            w.NamedInt32(g.RecordCount);
            for (int r = 0; r < g.RecordCount; r++)
            {
                ref readonly var rec = ref g.Records[r];
                w.DescribeEntity("body", rec.Slot, rec.Generation);
                w.NamedInt32(rec.Slot); w.NamedInt32(rec.Generation); w.NamedSingle(rec.Volume); w.NamedSingle(rec.Submerged);
                w.NamedSingle(rec.X0); w.NamedSingle(rec.X1); w.NamedSingle(rec.Z0); w.NamedSingle(rec.Z1);
            }

            if (hold)
            {
                w.NamedInt32(g.HoldCount);
                for (int k = 0; k < g.HoldCount; k++) { w.NamedInt32(g.HoldCells[k]); w.NamedSingle(g.HoldVolume[k]); }
            }
        }
    }

    private const int RecordBytes = 32;

    void ISnapshotSection.Read(StateReader r)
    {
        int version = r.ReadInt32();
        if (version != SectionVersion && version != HoldSectionVersion) { throw r.Error("unsupported shallow water section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var handles = HandleTable.Read(r, "shallow water");
        var grids = new WaterGrid?[Math.Max(handles.HighWater, 2)];
        bool planar = _host.Planar;
        for (int i = 0; i < handles.HighWater; i++)
        {
            if (!handles.IsLive(i)) { continue; }
            var g = new WaterGrid { Planar = r.ReadBool(), Nx = r.ReadInt32(), Nz = r.ReadInt32(), Dx = r.ReadSingle() };
            if (g.Planar != planar) { throw r.Error("shallow water grid " + i.ToString(CultureInfo.InvariantCulture) + " belongs to a " + (g.Planar ? "planar" : "spatial") + " world"); }
            if (g.Nx < 2 || g.Nx > 1 << 20 || (planar ? g.Nz != 1 : g.Nz < 2 || g.Nz > 1 << 20) || (long)g.Nx * g.Nz > WaterGrid.MaxCellsPerGrid) { throw r.Error("invalid shallow water grid size " + g.Nx.ToString(CultureInfo.InvariantCulture) + " x " + g.Nz.ToString(CultureInfo.InvariantCulture)); }
            if (!(g.Dx >= 0.001f && g.Dx <= 1000f)) { throw r.Error("invalid shallow water cell size " + g.Dx.ToString("R", CultureInfo.InvariantCulture)); }
            g.OriginX = r.ReadDouble(); g.OriginZ = r.ReadDouble(); g.BaseY = r.ReadDouble();
            if (!double.IsFinite(g.OriginX) || !double.IsFinite(g.OriginZ) || !double.IsFinite(g.BaseY) || Math.Abs(g.OriginX) > 4194304.0 || Math.Abs(g.OriginZ) > 4194304.0 || Math.Abs(g.BaseY) > 4194304.0) { throw r.Error("invalid shallow water origin"); }
            g.Density = r.ReadSingle(); g.LinearDrag = r.ReadSingle(); g.AngularDrag = r.ReadSingle(); g.ViscousDamping = r.ReadSingle(); g.BottomFriction = r.ReadSingle(); g.Damping = r.ReadSingle(); g.Displacement = r.ReadSingle();
            g.Cfl = r.ReadSingle(); g.MaxSubsteps = r.ReadInt32();
            if (!(g.Density > 0f && g.Density <= 1e6f) || !Within(g.LinearDrag) || !Within(g.AngularDrag) || !Within(g.ViscousDamping) || !Within(g.BottomFriction) || !Within(g.Damping) || !(g.Displacement >= 0f && g.Displacement <= 1f) || !(g.Cfl > 0f && g.Cfl <= 0.5f) || g.MaxSubsteps < 1 || g.MaxSubsteps > 256) { throw r.Error("invalid shallow water coefficients"); }
            for (int e = 0; e < 4; e++)
            {
                byte kind = r.ReadByte();
                if (kind > (byte)WaterEdge.Level) { throw r.Error("invalid shallow water boundary kind " + kind.ToString(CultureInfo.InvariantCulture)); }
                g.Edges[e] = (WaterEdge)kind; g.EdgeLevel[e] = r.ReadSingle(); g.EdgeSpeed[e] = r.ReadSingle();
                if (!float.IsFinite(g.EdgeLevel[e]) || Math.Abs(g.EdgeLevel[e]) > 4194304f || !(Math.Abs(g.EdgeSpeed[e]) <= 1000f)) { throw r.Error("invalid shallow water inflow"); }
            }

            int ns = r.ReadCount(16, "shallow water sources");
            if (ns > WaterGrid.MaxSources) { throw r.Error("too many shallow water sources"); }
            g.Sources = new WaterSourceSpec[ns];
            for (int k = 0; k < ns; k++)
            {
                g.Sources[k] = new WaterSourceSpec { X = r.ReadSingle(), Z = r.ReadSingle(), Radius = r.ReadSingle(), Rate = r.ReadSingle() };
                try { ValidateSource(g.Sources[k], "snapshot"); } catch (ArgumentOutOfRangeException) { throw r.Error("invalid shallow water source"); }
            }

            g.LostVolume = r.ReadDouble();
            if (!double.IsFinite(g.LostVolume) || g.LostVolume < 0) { throw r.Error("invalid lost volume"); }
            long cells = (long)g.Nx * g.Nz;
            if (4 * cells * 4 > r.Remaining) { throw r.Error("truncated shallow water grid"); }
            g.Allocate();
            r.ReadSingles(g.Bed, true, "shallow water bed");
            r.ReadSingles(g.H, true, "shallow water depth");
            r.ReadSingles(g.Hu, true, "shallow water momentum");
            r.ReadSingles(g.Hv, true, "shallow water momentum");
            for (int c = 0; c < g.H.Length; c++) { if (g.H[c] < 0f || g.H[c] > 4194304f) { throw r.Error("invalid shallow water depth"); } }
            int nr = r.ReadCount(RecordBytes, "shallow water body records");
            g.Records = new WaterBodyRecord[Math.Max(nr, 8)];
            int last = -1;
            for (int k = 0; k < nr; k++)
            {
                var rec = new WaterBodyRecord { Slot = r.ReadInt32(), Generation = r.ReadInt32(), Volume = r.ReadSingle(), Submerged = r.ReadSingle(), X0 = r.ReadSingle(), X1 = r.ReadSingle(), Z0 = r.ReadSingle(), Z1 = r.ReadSingle() };
                if (rec.Slot <= last || rec.Slot > 1 << 26 || rec.Generation < 1) { throw r.Error("shallow water body records out of order or for an invalid body"); }
                if (!(rec.Volume >= 0f && rec.Volume <= 1e12f) || !(rec.Submerged > 0f && rec.Submerged <= 1e12f) || !float.IsFinite(rec.X0) || !float.IsFinite(rec.X1) || !float.IsFinite(rec.Z0) || !float.IsFinite(rec.Z1) || rec.X1 < rec.X0 || rec.Z1 < rec.Z0) { throw r.Error("invalid shallow water body record"); }
                last = rec.Slot;
                g.Records[k] = rec;
            }

            g.RecordCount = nr;
            if (version == HoldSectionVersion)
            {
                int nh = r.ReadCount(8, "shallow water displaced volumes");
                if (nh > g.H.Length) { throw r.Error("more displaced-volume cells than the grid has"); }
                g.HoldCells = new int[nh]; g.HoldVolume = new float[nh]; g.HoldCount = nh;
                int lastCell = -1;
                for (int k = 0; k < nh; k++)
                {
                    int cell = r.ReadInt32(); float vol = r.ReadSingle();
                    if (cell <= lastCell || cell >= g.H.Length || !(vol > 0f && vol <= 1e12f)) { throw r.Error("invalid displaced volume in a shallow water cell"); }
                    lastCell = cell; g.HoldCells[k] = cell; g.HoldVolume[k] = vol;
                }
            }

            g.Derive();
            grids[i] = g;
        }

        // Commit.
        Handles = handles; Grids = grids; Events.Clear(); _pendingCount = 0;
    }

    private static bool Within(float v) => v >= 0f && v <= 1e6f;

    // ---------------------------------------------------------------- fork

    ISnapshotSection ISnapshotSection.Fork(WorldCore destination)
    {
        var dst = ((IWaterHost)destination.GetSnapshotSection(_host.EngineSectionId)).Water;
        dst.CopyFrom(this);
        return dst;
    }

    private void CopyFrom(WaterStore src)
    {
        Handles = new HandleTable(); Handles.CopyFrom(src.Handles);
        Grids = new WaterGrid?[src.Grids.Length];
        for (int i = 0; i < Grids.Length; i++) { Grids[i] = src.Grids[i]?.Clone(); }
        Events.Clear(); _pendingCount = 0;
    }
}
