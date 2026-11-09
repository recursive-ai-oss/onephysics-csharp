using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

internal sealed partial class FluidStore
{
    int ISnapshotSection.SectionId => Id;

    bool ISnapshotSection.IsOptional => true;

    bool ISnapshotSection.IsPresent => IsPresent;

    /// <summary>Whether the section is written: a fluid exists, or the settings were changed.</summary>
    public bool IsPresent => HasWork || !SettingsAreDefault;

    private bool SettingsAreDefault
    {
        get
        {
            var d = FluidSettings.Default; var s = Settings;
            return s.Substeps == d.Substeps && s.Iterations == d.Iterations && s.MaxParticles == d.MaxParticles
                && BitConverter.SingleToInt32Bits(s.SleepSpeed) == BitConverter.SingleToInt32Bits(d.SleepSpeed)
                && BitConverter.SingleToInt32Bits(s.TimeToSleep) == BitConverter.SingleToInt32Bits(d.TimeToSleep)
                && BitConverter.SingleToInt32Bits(s.WakeBodySpeed) == BitConverter.SingleToInt32Bits(d.WakeBodySpeed);
        }
    }

    void ISnapshotSection.Reset() { Clear(); }

    private void Clear()
    {
        Settings = FluidSettings.Default;
        FluidHandles.Reset(); EmitterHandles.Reset(); DrainHandles.Reset();
        Array.Clear(Fluids); Array.Clear(Emitters); Array.Clear(Drains);
        Spacing = 0f; KernelScale = 0f;
        Count = 0; NextId = 0; Asleep = false; QuietTime = 0f; StepCount = 0;
        WetCount = 0;
        Events.Clear(); Results.Clear();
        _qValid = false;
    }

    private const int SectionVersion = 1;

    private static void WriteDef(StateWriter w, ParticleFluidDef d)
    {
        w.NamedSingle(d.Spacing); w.NamedSingle(d.RestDensity); w.NamedSingle(d.KernelRadiusScale); w.NamedSingle(d.Viscosity); w.NamedSingle(d.Vorticity);
        w.NamedSingle(d.TensileCorrection); w.NamedSingle(d.Relaxation); w.NamedSingle(d.Friction); w.NamedSingle(d.MaxSpeed); w.NamedSingle(d.GravityScale);
        w.NamedInt32(d.Layer); w.NamedUInt32(d.CategoryBits); w.NamedUInt32(d.MaskBits); w.NamedInt32(d.GroupIndex);
        w.NamedBool(d.CollideWithSoftBodies); w.NamedBool(d.CollideWithRods);
    }

    private static ParticleFluidDef ReadDef(StateReader r)
    {
        var d = new ParticleFluidDef
        {
            Spacing = r.ReadSingle(), RestDensity = r.ReadSingle(), KernelRadiusScale = r.ReadSingle(), Viscosity = r.ReadSingle(), Vorticity = r.ReadSingle(),
            TensileCorrection = r.ReadSingle(), Relaxation = r.ReadSingle(), Friction = r.ReadSingle(), MaxSpeed = r.ReadSingle(), GravityScale = r.ReadSingle(),
            Layer = r.ReadInt32(), CategoryBits = r.ReadUInt32(), MaskBits = r.ReadUInt32(), GroupIndex = r.ReadInt32(),
            CollideWithSoftBodies = r.ReadBool(), CollideWithRods = r.ReadBool(),
        };
        try { d.Validate(); }
        catch (ArgumentOutOfRangeException e) { throw r.Error("invalid particle fluid: " + e.Message); }
        return d;
    }

    private const int DefBytes = 10 * 4 + 4 + 4 + 4 + 4 + 2;

    void ISnapshotSection.Write(StateWriter w)
    {
        w.NamedInt32(SectionVersion);
        w.NamedInt32(Settings.Substeps); w.NamedInt32(Settings.Iterations); w.NamedInt32(Settings.MaxParticles);
        w.NamedSingle(Settings.SleepSpeed); w.NamedSingle(Settings.TimeToSleep); w.NamedSingle(Settings.WakeBodySpeed);
        w.NamedSingle(Spacing); w.NamedSingle(KernelScale);
        w.NamedInt32(NextId); w.NamedBool(Asleep); w.NamedSingle(QuietTime); w.NamedInt64(StepCount);

        w.DescribeTable("particlefluid");
        FluidHandles.Write(w);
        for (int i = 0; i < FluidHandles.HighWater; i++)
        {
            w.DescribeEntity("particlefluid", i, FluidHandles.GenerationOf(i));
            if (!FluidHandles.IsLive(i)) { continue; }
            WriteDef(w, Fluids[i]!.Def);
        }

        w.DescribeTable("fluid emitter");
        EmitterHandles.Write(w);
        for (int i = 0; i < EmitterHandles.HighWater; i++)
        {
            w.DescribeEntity("fluid emitter", i, EmitterHandles.GenerationOf(i));
            if (!EmitterHandles.IsLive(i)) { continue; }
            var e = Emitters[i]!; var d = e.Def;
            w.NamedInt32(e.Fluid); w.NamedInt32(e.FluidGen);
            w.NamedDouble(d.Position.X); w.NamedDouble(d.Position.Y); w.NamedDouble(d.Position.Z);
            w.NamedSingle(d.Direction.X); w.NamedSingle(d.Direction.Y); w.NamedSingle(d.Direction.Z);
            w.NamedByte((byte)d.Kind);
            w.NamedSingle(d.Size.X); w.NamedSingle(d.Size.Y); w.NamedSingle(d.Size.Z);
            w.NamedSingle(d.Speed); w.NamedBool(d.Enabled); w.NamedInt32(d.MaxEmitted);
            w.NamedSingle(e.Phase); w.NamedInt32(e.Emitted);
        }

        w.DescribeTable("fluid drain");
        DrainHandles.Write(w);
        for (int i = 0; i < DrainHandles.HighWater; i++)
        {
            w.DescribeEntity("fluid drain", i, DrainHandles.GenerationOf(i));
            if (!DrainHandles.IsLive(i)) { continue; }
            var s = Drains[i]!; var d = s.Def;
            w.NamedInt32(s.Fluid); w.NamedInt32(s.FluidGen);
            w.NamedDouble(d.Position.X); w.NamedDouble(d.Position.Y); w.NamedDouble(d.Position.Z);
            w.NamedByte((byte)d.Kind);
            w.NamedSingle(d.Size.X); w.NamedSingle(d.Size.Y); w.NamedSingle(d.Size.Z);
            w.NamedBool(d.Enabled); w.NamedInt32(s.Removed);
        }

        w.DescribeEntity("world", -1);
        w.NamedInt32(Count);
        for (int p = 0; p < Count; p++)
        {
            w.DescribeEntity("fluidparticle", p);
            w.NamedInt32(PId[p]); w.NamedByte(PFluid[p]);
            w.NamedDouble(PX[p]); w.NamedDouble(PY[p]); w.NamedDouble(PZ[p]);
            w.NamedSingle(PVx[p]); w.NamedSingle(PVy[p]); w.NamedSingle(PVz[p]);
        }

        w.DescribeEntity("world", -1);
        w.NamedInt32(WetCount);
        for (int i = 0; i < WetCount; i++)
        {
            w.NamedInt32(Wet[i].Body); w.NamedInt32(Wet[i].BodyGen); w.NamedInt32(Wet[i].Fluid);
        }
    }

    void ISnapshotSection.Read(StateReader r)
    {
        int version = r.ReadInt32();
        if (version != SectionVersion) { throw r.Error("unsupported particle fluid section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var settings = new FluidSettings
        {
            Substeps = r.ReadInt32(), Iterations = r.ReadInt32(), MaxParticles = r.ReadInt32(),
            SleepSpeed = r.ReadSingle(), TimeToSleep = r.ReadSingle(), WakeBodySpeed = r.ReadSingle(),
        };
        try { settings.Validate(); }
        catch (ArgumentOutOfRangeException e) { throw r.Error("invalid fluid settings: " + e.Message); }
        float spacing = r.ReadSingle(), kscale = r.ReadSingle();
        int nextId = r.ReadInt32();
        bool asleep = r.ReadBool();
        float quiet = r.ReadSingle();
        long steps = r.ReadInt64();
        if (!(float.IsFinite(spacing) && float.IsFinite(kscale) && float.IsFinite(quiet) && quiet >= 0f) || nextId < 0 || steps < 0) { throw r.Error("invalid particle fluid header"); }

        var fh = HandleTable.Read(r, "particle fluid");
        var fluids = new FluidState?[Math.Max(fh.HighWater, 1)];
        if (fh.HighWater > 255 || fh.Count > MaxFluids) { throw r.Error("too many particle fluids"); }
        float firstSpacing = 0f;
        for (int i = 0; i < fh.HighWater; i++)
        {
            if (!fh.IsLive(i)) { continue; }
            var def = ReadDef(r);
            if (def.Spacing != spacing || def.KernelRadiusScale != kscale) { throw r.Error("particle fluids disagree on their spacing"); }
            firstSpacing = def.Spacing;
            fluids[i] = new FluidState { Def = def };
        }

        if (fh.Count > 0 && !(firstSpacing > 0f)) { throw r.Error("invalid particle spacing"); }

        var eh = HandleTable.Read(r, "fluid emitter");
        var emitters = new EmitterState?[Math.Max(eh.HighWater, 1)];
        for (int i = 0; i < eh.HighWater; i++)
        {
            if (!eh.IsLive(i)) { continue; }
            var e = new EmitterState { Fluid = r.ReadInt32(), FluidGen = r.ReadInt32() };
            if (!fh.IsAlive(e.Fluid, e.FluidGen)) { throw r.Error("emitter of a missing fluid"); }
            var d = new FluidEmitterDef();
            d.Position = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble());
            d.Direction = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            byte kind = r.ReadByte();
            if (kind > 1) { throw r.Error("invalid emitter kind"); }
            d.Kind = (FluidVolumeKind)kind;
            d.Size = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            d.Speed = r.ReadSingle(); d.Enabled = r.ReadBool(); d.MaxEmitted = r.ReadInt32();
            try { d.Validate(); }
            catch (ArgumentOutOfRangeException ex) { throw r.Error("invalid emitter: " + ex.Message); }
            e.Def = d; e.Phase = r.ReadSingle(); e.Emitted = r.ReadInt32();
            if (!(float.IsFinite(e.Phase) && e.Phase >= 0f) || e.Emitted < 0) { throw r.Error("invalid emitter state"); }
            emitters[i] = e;
        }

        var dh = HandleTable.Read(r, "fluid drain");
        var drains = new DrainState?[Math.Max(dh.HighWater, 1)];
        for (int i = 0; i < dh.HighWater; i++)
        {
            if (!dh.IsLive(i)) { continue; }
            var s = new DrainState { Fluid = r.ReadInt32(), FluidGen = r.ReadInt32() };
            if (s.Fluid >= 0 && !fh.IsAlive(s.Fluid, s.FluidGen)) { throw r.Error("drain of a missing fluid"); }
            var d = new FluidDrainDef { Position = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble()) };
            byte kind = r.ReadByte();
            if (kind > 1) { throw r.Error("invalid drain kind"); }
            d.Kind = (FluidVolumeKind)kind;
            d.Size = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            d.Enabled = r.ReadBool(); s.Removed = r.ReadInt32();
            try { d.Validate(); }
            catch (ArgumentOutOfRangeException ex) { throw r.Error("invalid drain: " + ex.Message); }
            if (s.Removed < 0) { throw r.Error("invalid drain count"); }
            s.Def = d;
            drains[i] = s;
        }

        int n = r.ReadCount(4 + 1 + 24 + 12, "fluid particles");
        var id = new int[Math.Max(n, 1)]; var fl = new byte[id.Length];
        var px = new double[id.Length]; var py = new double[id.Length]; var pz = new double[id.Length];
        var vx = new float[id.Length]; var vy = new float[id.Length]; var vz = new float[id.Length];
        bool planar = Dim == 2;
        for (int p = 0; p < n; p++)
        {
            id[p] = r.ReadInt32(); fl[p] = r.ReadByte();
            px[p] = r.ReadDouble(); py[p] = r.ReadDouble(); pz[p] = r.ReadDouble();
            vx[p] = r.ReadSingle(); vy[p] = r.ReadSingle(); vz[p] = r.ReadSingle();
            if (id[p] < 0 || id[p] >= nextId) { throw r.Error("invalid particle id"); }
            if (!fh.IsLive(fl[p])) { throw r.Error("particle of a missing fluid"); }
            if (!(Math.Abs(px[p]) <= 8388608.0 && Math.Abs(py[p]) <= 8388608.0 && Math.Abs(pz[p]) <= 8388608.0)) { throw r.Error("particle position is out of range or not finite"); }
            if (!(float.IsFinite(vx[p]) && float.IsFinite(vy[p]) && float.IsFinite(vz[p]))) { throw r.Error("particle velocity is not finite"); }
            if (planar && (pz[p] != 0.0 || vz[p] != 0f)) { throw r.Error("planar particle has a z component"); }
        }

        int nw = r.ReadCount(12, "wet records");
        var wet = new WetRecord[Math.Max(nw, 8)];
        for (int i = 0; i < nw; i++)
        {
            wet[i] = new WetRecord { Body = r.ReadInt32(), BodyGen = r.ReadInt32(), Fluid = r.ReadInt32(), Wet = true };
            if (wet[i].Body < 0 || wet[i].BodyGen < 1 || !fh.IsLive(wet[i].Fluid)) { throw r.Error("invalid wet record"); }
            if (i > 0 && (wet[i - 1].Body > wet[i].Body || (wet[i - 1].Body == wet[i].Body && wet[i - 1].Fluid >= wet[i].Fluid))) { throw r.Error("wet records out of order"); }
        }

        // Commit.
        Settings = settings;
        FluidHandles = fh; Fluids = fluids; EmitterHandles = eh; Emitters = emitters; DrainHandles = dh; Drains = drains;
        Spacing = fh.Count > 0 ? spacing : 0f; KernelScale = fh.Count > 0 ? kscale : 0f;
        for (int i = 0; i < fh.HighWater; i++) { if (fh.IsLive(i)) { Derive(Fluids[i]!); } }
        PId = id; PFluid = fl; PX = px; PY = py; PZ = pz; PVx = vx; PVy = vy; PVz = vz; Count = n;
        NextId = nextId; Asleep = asleep; QuietTime = quiet; StepCount = steps;
        Wet = wet; WetCount = nw;
        Events.Clear(); Results.Clear();
        _qValid = false;
    }

    // ------------------------------------------------------------------ fork

    ISnapshotSection ISnapshotSection.Fork(WorldCore destination)
    {
        var dst = ((IFluidHost)destination.GetSnapshotSection(Host.EngineSectionId)).ParticleFluids;
        dst.CopyFrom(this);
        return dst;
    }

    private void CopyFrom(FluidStore s)
    {
        Settings = s.Settings;
        FluidHandles = new HandleTable(); FluidHandles.CopyFrom(s.FluidHandles);
        EmitterHandles = new HandleTable(); EmitterHandles.CopyFrom(s.EmitterHandles);
        DrainHandles = new HandleTable(); DrainHandles.CopyFrom(s.DrainHandles);
        Fluids = new FluidState?[s.Fluids.Length];
        for (int i = 0; i < Fluids.Length; i++) { var f = s.Fluids[i]; if (f != null) { Fluids[i] = new FluidState { Def = f.Def.Clone(), Mass = f.Mass, N0 = f.N0, GradNorm = f.GradNorm, Count = f.Count }; } }
        Emitters = new EmitterState?[s.Emitters.Length];
        for (int i = 0; i < Emitters.Length; i++) { var e = s.Emitters[i]; if (e != null) { Emitters[i] = new EmitterState { Fluid = e.Fluid, FluidGen = e.FluidGen, Def = e.Def.Clone(), Phase = e.Phase, Emitted = e.Emitted }; } }
        Drains = new DrainState?[s.Drains.Length];
        for (int i = 0; i < Drains.Length; i++) { var d = s.Drains[i]; if (d != null) { Drains[i] = new DrainState { Fluid = d.Fluid, FluidGen = d.FluidGen, Def = d.Def.Clone(), Removed = d.Removed }; } }
        Spacing = s.Spacing; KernelScale = s.KernelScale;
        Count = s.Count; NextId = s.NextId; Asleep = s.Asleep; QuietTime = s.QuietTime; StepCount = s.StepCount;
        PId = (int[])s.PId.Clone(); PFluid = (byte[])s.PFluid.Clone();
        PX = (double[])s.PX.Clone(); PY = (double[])s.PY.Clone(); PZ = (double[])s.PZ.Clone();
        PVx = (float[])s.PVx.Clone(); PVy = (float[])s.PVy.Clone(); PVz = (float[])s.PVz.Clone();
        Wet = (WetRecord[])s.Wet.Clone(); WetCount = s.WetCount;
        Events = new System.Collections.Generic.List<FluidEvent>(s.Events);
        Results.Clear(); Results.AddRange(s.Results);
        _qValid = false;
    }

    // ------------------------------------------------------------------ regions and origin shifts

    /// <summary>A region takes the particles whose positions lie inside its box (an archive names them by index), the emitters, drains and fluids that cover them stay with the world.</summary>
    internal void SelectRegion(RegionSelection s, double[] min, double[] max, RegionSnapshot? archive)
    {
        if (archive == null)
        {
            for (int p = 0; p < Count; p++)
            {
                if (PX[p] >= min[0] && PX[p] <= max[0] && PY[p] >= min[1] && PY[p] <= max[1] && (Dim == 2 || (PZ[p] >= min[2] && PZ[p] <= max[2]))) { s.Add("fluidparticle", p); }
            }
        }

        for (int i = 0; i < FluidHandles.HighWater; i++) { if (FluidHandles.IsLive(i)) { s.Add("particlefluid", i); } }
        for (int i = 0; i < EmitterHandles.HighWater; i++) { if (EmitterHandles.IsLive(i)) { s.Add("fluid emitter", i); } }
        for (int i = 0; i < DrainHandles.HighWater; i++) { if (DrainHandles.IsLive(i)) { s.Add("fluid drain", i); } }
    }

    /// <summary>Moves everything by -offset (the world shifted its origin by offset). Positions of particles, emitters and drains stay on the grid the same way the bodies do.</summary>
    internal void ShiftOrigin(ReadOnlySpan<double> offset)
    {
        for (int p = 0; p < Count; p++)
        {
            PX[p] -= offset[0]; PY[p] -= offset[1];
            if (Dim == 3) { PZ[p] -= offset[2]; }
        }

        for (int e = 0; e < EmitterHandles.HighWater; e++)
        {
            if (!EmitterHandles.IsLive(e)) { continue; }
            var d = Emitters[e]!.Def;
            d.Position = new Vector3D(d.Position.X - offset[0], d.Position.Y - offset[1], Dim == 3 ? d.Position.Z - offset[2] : d.Position.Z);
        }

        for (int k = 0; k < DrainHandles.HighWater; k++)
        {
            if (!DrainHandles.IsLive(k)) { continue; }
            var d = Drains[k]!.Def;
            d.Position = new Vector3D(d.Position.X - offset[0], d.Position.Y - offset[1], Dim == 3 ? d.Position.Z - offset[2] : d.Position.Z);
        }

        _qValid = false;
    }

    /// <summary>Whether the shift keeps every particle within the world's range.</summary>
    internal bool CanShift(ReadOnlySpan<double> offset)
    {
        for (int p = 0; p < Count; p++)
        {
            if (!(Math.Abs(PX[p] - offset[0]) <= 8388608.0) || !(Math.Abs(PY[p] - offset[1]) <= 8388608.0) || (Dim == 3 && !(Math.Abs(PZ[p] - offset[2]) <= 8388608.0))) { return false; }
        }

        return true;
    }

    /// <summary>Whether the state is consistent with the world's bodies (used after a restore).</summary>
    internal bool Consistent()
    {
        for (int p = 0; p < Count; p++)
        {
            if (!FluidHandles.IsLive(PFluid[p])) { return false; }
            if (!double.IsFinite(PX[p]) || !double.IsFinite(PY[p]) || !double.IsFinite(PZ[p])) { return false; }
        }

        return true;
    }
}
