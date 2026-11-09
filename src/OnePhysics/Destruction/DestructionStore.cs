using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Destruction;

/// <summary>
/// Destruction state of a world: which bodies fracture on impact, the bond graphs of breakable compounds, the debris policy and its fragment table.
/// One implementation serves both dimensions through <see cref="IDestructionHost"/> (a 2D world embeds its plane at z = 0). It is an optional snapshot section, written only while it holds state,
/// so a world that never breaks anything keeps its bytes and hash. Fractures run at the end of <see cref="AfterStep"/>, after events are delivered, in ascending body slot order.
/// </summary>
internal sealed partial class DestructionStore(IDestructionHost host) : ISnapshotSection
{
    internal const int Id = 0x52545344; // "DSTR"
    internal readonly IDestructionHost Host = host;
    internal readonly List<Record> Records = new();
    internal readonly List<Fragment> Fragments = new();

    // Debris policy.
    internal int MaxFragments;
    internal int FreezeAfterSteps;
    internal long LifetimeSteps;

    private long _fractures, _slices, _bondBreaks, _created, _culled, _frozen, _expired, _dropped, _merged, _nextSequence;

    // Events of the last step (and any manual fractures since), cleared when the next step begins.
    internal readonly List<FractureRecord> Events = new();
    internal readonly List<BodyHandle> EventFragments = new();

    public int SectionId => Id;
    public bool IsOptional => true;
    public bool IsPresent => Records.Count != 0 || Fragments.Count != 0 || MaxFragments != 0 || FreezeAfterSteps != 0 || LifetimeSteps != 0 || _fractures != 0 || _slices != 0 || _bondBreaks != 0 || _created != 0 || _nextSequence != 0;

    /// <summary>True when a step has nothing to do here.</summary>
    internal bool Idle => Records.Count == 0 && Fragments.Count == 0;

    internal DestructionCounters Counters => new(_fractures, _slices, _bondBreaks, _created, Fragments.Count, _culled, _frozen, _expired, _dropped, _merged);

    internal bool BodyAlive(int body, int generation) => Host.DestructionCore.Bodies.Handles.IsAlive(body, generation);

    internal int FindRecord(int body)
    {
        int lo = 0, hi = Records.Count - 1;
        while (lo <= hi)
        {
            int mid = (lo + hi) >> 1;
            int v = Records[mid].Body;
            if (v == body) return mid;
            if (v < body) lo = mid + 1; else hi = mid - 1;
        }
        return ~lo;
    }

    internal Record GetOrAdd(int body)
    {
        var core = Host.DestructionCore;
        int generation = core.Bodies.Handles.GenerationOf(body);
        int i = FindRecord(body);
        if (i >= 0)
        {
            if (Records[i].Generation == generation) return Records[i];
            Records.RemoveAt(i); // a record left by a destroyed body in this slot
            i = FindRecord(body);
        }
        var r = new Record { Body = body, Generation = generation };
        Records.Insert(~i, r);
        return r;
    }

    internal Record? Find(int body)
    {
        int i = FindRecord(body);
        if (i < 0) return null;
        var r = Records[i];
        return BodyAlive(r.Body, r.Generation) ? r : null;
    }

    internal void Remove(int body)
    {
        int i = FindRecord(body);
        if (i >= 0) Records.RemoveAt(i);
    }

    // Removes records and fragments whose body is gone.
    internal void PruneDead()
    {
        for (int i = Records.Count - 1; i >= 0; i--)
            if (!BodyAlive(Records[i].Body, Records[i].Generation)) Records.RemoveAt(i);
        for (int i = Fragments.Count - 1; i >= 0; i--)
            if (!BodyAlive(Fragments[i].Body, Fragments[i].Generation)) Fragments.RemoveAt(i);
    }

    internal void BeginStep()
    {
        Events.Clear();
        EventFragments.Clear();
    }

    public void Reset()
    {
        Records.Clear(); Fragments.Clear();
        MaxFragments = 0; FreezeAfterSteps = 0; LifetimeSteps = 0;
        _fractures = _slices = _bondBreaks = _created = _culled = _frozen = _expired = _dropped = _merged = _nextSequence = 0;
        Events.Clear(); EventFragments.Clear();
    }

    public ISnapshotSection Fork(WorldCore destination)
    {
        var h = (IDestructionHost)destination.GetSnapshotSection(Host.EngineSectionId);
        var copy = h.Destruction;
        copy.Reset();
        foreach (var r in Records)
        {
            var c = new Record { Body = r.Body, Generation = r.Generation, Fracture = r.Fracture?.Clone(), Bonds = r.Bonds == null ? null : new BondSettings { Strength = r.Bonds.Strength, ShearFactor = r.Bonds.ShearFactor, OverloadSteps = r.Bonds.OverloadSteps } };
            c.Parts = new Part[r.Parts.Length];
            for (int i = 0; i < c.Parts.Length; i++) { var p = r.Parts[i]; c.Parts[i] = new Part { Shape = p.Shape, ShapeGeneration = p.ShapeGeneration, Anchored = p.Anchored, Measure = p.Measure, Mass = p.Mass, Centroid = p.Centroid }; }
            c.BondList = new Bond[r.BondList.Length];
            for (int i = 0; i < c.BondList.Length; i++) { var b = r.BondList[i]; c.BondList[i] = new Bond { A = b.A, B = b.B, Measure = b.Measure, Centroid = b.Centroid, Normal = b.Normal, Strength = b.Strength, Alive = b.Alive, Overload = b.Overload, Stress = b.Stress }; }
            copy.Records.Add(c);
        }
        foreach (var f in Fragments)
            copy.Fragments.Add(new Fragment { Body = f.Body, Generation = f.Generation, Sequence = f.Sequence, BirthStep = f.BirthStep, Measure = f.Measure, AsleepSteps = f.AsleepSteps, Frozen = f.Frozen });
        copy.MaxFragments = MaxFragments; copy.FreezeAfterSteps = FreezeAfterSteps; copy.LifetimeSteps = LifetimeSteps;
        copy._fractures = _fractures; copy._slices = _slices; copy._bondBreaks = _bondBreaks; copy._created = _created; copy._culled = _culled; copy._frozen = _frozen;
        copy._expired = _expired; copy._dropped = _dropped; copy._merged = _merged; copy._nextSequence = _nextSequence;
        return copy;
    }

    // ------------------------------------------------------------------ snapshot (version 1)

    public void Write(StateWriter w)
    {
        w.NamedInt32(1, "version");
        w.NamedInt32(MaxFragments, "maxFragments"); w.NamedInt32(FreezeAfterSteps, "freezeAfterSteps"); w.NamedInt64(LifetimeSteps, "lifetimeSteps");
        w.NamedInt64(_fractures, "fractures"); w.NamedInt64(_slices, "slices"); w.NamedInt64(_bondBreaks, "bondBreaks"); w.NamedInt64(_created, "fragmentsCreated");
        w.NamedInt64(_culled, "fragmentsCulled"); w.NamedInt64(_frozen, "fragmentsFrozen"); w.NamedInt64(_expired, "fragmentsExpired");
        w.NamedInt64(_dropped, "sliversDropped"); w.NamedInt64(_merged, "sliversMerged"); w.NamedInt64(_nextSequence, "nextSequence");
        int live = 0;
        foreach (var r in Records) if (BodyAlive(r.Body, r.Generation)) live++;
        w.NamedInt32(live, "records");
        foreach (var r in Records)
        {
            if (!BodyAlive(r.Body, r.Generation)) continue;
            w.DescribeEntity("breakable", r.Body, r.Generation);
            w.NamedInt32(r.Body, "body"); w.NamedInt32(r.Generation, "generation");
            w.NamedBool(r.Fracture != null, "fracturable");
            if (r.Fracture is { } f) WriteFracture(w, f);
            w.NamedBool(r.Bonds != null, "bonded");
            if (r.Bonds is { } bs)
            {
                w.NamedSingle(bs.Strength, "strength"); w.NamedSingle(bs.ShearFactor, "shearFactor"); w.NamedInt32(bs.OverloadSteps, "overloadSteps");
                w.NamedInt32(r.Parts.Length, "parts");
                foreach (var p in r.Parts)
                {
                    w.NamedInt32(p.Shape, "shape"); w.NamedInt32(p.ShapeGeneration, "shapeGeneration"); w.NamedBool(p.Anchored, "anchored"); w.NamedSingle(p.Measure, "measure"); w.NamedSingle(p.Mass, "mass");
                    w.NamedSingle(p.Centroid.X, "centroidX"); w.NamedSingle(p.Centroid.Y, "centroidY"); w.NamedSingle(p.Centroid.Z, "centroidZ");
                }
                w.NamedInt32(r.BondList.Length, "bonds");
                foreach (var b in r.BondList)
                {
                    w.NamedInt32(b.A, "a"); w.NamedInt32(b.B, "b"); w.NamedSingle(b.Measure, "measure");
                    w.NamedSingle(b.Centroid.X, "centroidX"); w.NamedSingle(b.Centroid.Y, "centroidY"); w.NamedSingle(b.Centroid.Z, "centroidZ");
                    w.NamedSingle(b.Normal.X, "normalX"); w.NamedSingle(b.Normal.Y, "normalY"); w.NamedSingle(b.Normal.Z, "normalZ");
                    w.NamedSingle(b.Strength, "strength"); w.NamedBool(b.Alive, "alive"); w.NamedInt32(b.Overload, "overload"); w.NamedSingle(b.Stress, "stress");
                }
            }
        }
        int frags = 0;
        foreach (var f in Fragments) if (BodyAlive(f.Body, f.Generation)) frags++;
        w.NamedInt32(frags, "fragments");
        foreach (var f in Fragments)
        {
            if (!BodyAlive(f.Body, f.Generation)) continue;
            w.DescribeEntity("debris", f.Body, f.Generation);
            w.NamedInt32(f.Body, "body"); w.NamedInt32(f.Generation, "generation"); w.NamedInt64(f.Sequence, "sequence"); w.NamedInt64(f.BirthStep, "birthStep");
            w.NamedSingle(f.Measure, "measure"); w.NamedInt32(f.AsleepSteps, "asleepSteps"); w.NamedBool(f.Frozen, "frozen");
        }
    }

    private static void WriteFracture(StateWriter w, FractureSettings f)
    {
        w.NamedInt32(f.CellCount, "cells"); w.NamedInt32((int)f.Pattern, "pattern"); w.NamedInt64((long)f.Seed, "seed");
        w.NamedInt32(f.Points.Length, "seedPoints");
        foreach (var p in f.Points) { w.NamedSingle(p.X, "x"); w.NamedSingle(p.Y, "y"); w.NamedSingle(p.Z, "z"); }
        w.NamedBool(f.ImpactPoint.HasValue, "hasImpactPoint");
        if (f.ImpactPoint is { } ip) { w.NamedDouble(ip.X, "x"); w.NamedDouble(ip.Y, "y"); w.NamedDouble(ip.Z, "z"); }
        w.NamedSingle(f.ImpactRadius, "impactRadius"); w.NamedSingle(f.ClusterPower, "clusterPower"); w.NamedSingle(f.ImpulseThreshold, "impulseThreshold");
        w.NamedInt32(f.Depth, "depth"); w.NamedInt32(f.ChildCellCount, "childCells"); w.NamedSingle(f.ChildThresholdScale, "childThresholdScale");
        w.NamedSingle(f.MinFragmentVolume, "minFragmentVolume"); w.NamedBool(f.MergeSmall, "mergeSmall");
    }

    public void Read(StateReader r)
    {
        if (r.ReadInt32() != 1) throw r.Error("unsupported destruction version");
        int maxFragments = r.ReadInt32(), freeze = r.ReadInt32();
        long lifetime = r.ReadInt64();
        if (maxFragments < 0 || freeze < 0 || lifetime < 0) throw r.Error("invalid debris options");
        long fractures = r.ReadInt64(), slices = r.ReadInt64(), bondBreaks = r.ReadInt64(), created = r.ReadInt64(), culled = r.ReadInt64(), frozen = r.ReadInt64(), expired = r.ReadInt64(), dropped = r.ReadInt64(), merged = r.ReadInt64(), next = r.ReadInt64();
        if (fractures < 0 || slices < 0 || bondBreaks < 0 || created < 0 || culled < 0 || frozen < 0 || expired < 0 || dropped < 0 || merged < 0 || next < 0) throw r.Error("invalid destruction counter");
        // Bodies the snapshot restores (the staged store while a restore is running): a record or fragment of a body that does not exist would not survive writing again.
        var staged = Host.DestructionCore.RestoringBodies ?? Host.DestructionCore.Bodies;
        int n = r.ReadCount(10, "breakable records");
        var records = new List<Record>(n);
        int lastBody = -1;
        for (int i = 0; i < n; i++)
        {
            var rec = new Record { Body = r.ReadInt32(), Generation = r.ReadInt32() };
            if (rec.Body <= lastBody || rec.Generation <= 0) throw r.Error("breakable records are not in body order");
            if (!staged.Handles.IsAlive(rec.Body, rec.Generation)) throw r.Error("breakable record refers to a body that does not exist");
            lastBody = rec.Body;
            if (r.ReadBool()) rec.Fracture = ReadFracture(r);
            if (r.ReadBool())
            {
                var bs = new BondSettings { Strength = r.ReadSingle(), ShearFactor = r.ReadSingle(), OverloadSteps = r.ReadInt32() };
                if (!(bs.Strength > 0f) || !float.IsFinite(bs.Strength) || !(bs.ShearFactor >= 0f) || !float.IsFinite(bs.ShearFactor) || bs.OverloadSteps < 1 || bs.OverloadSteps > 1000000) throw r.Error("invalid bond settings");
                rec.Bonds = bs;
                int np = r.ReadCount(29, "parts");
                rec.Parts = new Part[np];
                for (int k = 0; k < np; k++)
                {
                    var p = new Part { Shape = r.ReadInt32(), ShapeGeneration = r.ReadInt32(), Anchored = r.ReadBool(), Measure = r.ReadSingle(), Mass = r.ReadSingle(), Centroid = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()) };
                    if (p.Shape < 0 || p.ShapeGeneration <= 0 || !float.IsFinite(p.Measure) || !(p.Mass >= 0f) || !float.IsFinite(p.Mass) || !float.IsFinite(p.Centroid.X) || !float.IsFinite(p.Centroid.Y) || !float.IsFinite(p.Centroid.Z)) throw r.Error("invalid part");
                    rec.Parts[k] = p;
                }
                int nb = r.ReadCount(49, "bonds");
                rec.BondList = new Bond[nb];
                for (int k = 0; k < nb; k++)
                {
                    var b = new Bond { A = r.ReadInt32(), B = r.ReadInt32(), Measure = r.ReadSingle(), Centroid = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), Normal = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), Strength = r.ReadSingle(), Alive = r.ReadBool(), Overload = r.ReadInt32(), Stress = r.ReadSingle() };
                    if ((uint)b.A >= (uint)np || (uint)b.B >= (uint)np || b.A == b.B || !(b.Measure > 0f) || !float.IsFinite(b.Measure) || !(b.Strength >= 0f) || !float.IsFinite(b.Strength) || b.Overload < 0
                        || !float.IsFinite(b.Centroid.X) || !float.IsFinite(b.Centroid.Y) || !float.IsFinite(b.Centroid.Z) || !float.IsFinite(b.Normal.X) || !float.IsFinite(b.Normal.Y) || !float.IsFinite(b.Normal.Z) || !float.IsFinite(b.Stress))
                        throw r.Error("invalid bond");
                    rec.BondList[k] = b;
                }
            }
            if (rec.Fracture != null && rec.Bonds != null) throw r.Error("a body cannot be both fracturable and bonded");
            records.Add(rec);
        }
        int nf = r.ReadCount(33, "fragments");
        var fragments = new List<Fragment>(nf);
        long lastSeq = -1;
        for (int i = 0; i < nf; i++)
        {
            var f = new Fragment { Body = r.ReadInt32(), Generation = r.ReadInt32(), Sequence = r.ReadInt64(), BirthStep = r.ReadInt64(), Measure = r.ReadSingle(), AsleepSteps = r.ReadInt32(), Frozen = r.ReadBool() };
            if (!staged.Handles.IsAlive(f.Body, f.Generation)) throw r.Error("fragment refers to a body that does not exist");
            if (f.Body < 0 || f.Generation <= 0 || f.Sequence <= lastSeq || f.Sequence >= next || f.BirthStep < 0 || !float.IsFinite(f.Measure) || f.AsleepSteps < 0) throw r.Error("invalid fragment");
            lastSeq = f.Sequence;
            fragments.Add(f);
        }
        // Everything validated: replace the state.
        Reset();
        Records.AddRange(records); Fragments.AddRange(fragments);
        MaxFragments = maxFragments; FreezeAfterSteps = freeze; LifetimeSteps = lifetime;
        _fractures = fractures; _slices = slices; _bondBreaks = bondBreaks; _created = created; _culled = culled; _frozen = frozen; _expired = expired; _dropped = dropped; _merged = merged; _nextSequence = next;
    }

    private static FractureSettings ReadFracture(StateReader r)
    {
        var f = new FractureSettings { CellCount = r.ReadInt32(), Pattern = (FracturePattern)r.ReadInt32(), Seed = (ulong)r.ReadInt64() };
        int np = r.ReadCount(12, "seed points");
        f.Points = new Vector3F[np];
        for (int i = 0; i < np; i++) f.Points[i] = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        if (r.ReadBool()) f.ImpactPoint = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble());
        f.ImpactRadius = r.ReadSingle(); f.ClusterPower = r.ReadSingle(); f.ImpulseThreshold = r.ReadSingle();
        f.Depth = r.ReadInt32(); f.ChildCellCount = r.ReadInt32(); f.ChildThresholdScale = r.ReadSingle();
        f.MinFragmentVolume = r.ReadSingle(); f.MergeSmall = r.ReadBool();
        try { Validate(f); } catch (ArgumentException ex) { throw r.Error("invalid fracture settings: " + ex.Message); }
        return f;
    }

    /// <summary>Checks settings; throws ArgumentOutOfRangeException naming the field.</summary>
    internal static void Validate(FractureSettings f, string paramName = "def")
    {
        if (f.CellCount < 2 || f.CellCount > 512) throw new ArgumentOutOfRangeException(paramName, f.CellCount, "CellCount must be within 2..512.");
        if (!Enum.IsDefined(f.Pattern)) throw new ArgumentOutOfRangeException(paramName, f.Pattern, "Unknown fracture pattern.");
        if (f.Pattern == FracturePattern.Points && (f.Points.Length < 2 || f.Points.Length > 512)) throw new ArgumentOutOfRangeException(paramName, f.Points.Length, "A Points pattern needs 2..512 seed points.");
        foreach (var p in f.Points) if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z)) throw new ArgumentOutOfRangeException(paramName, p, "Seed points must be finite.");
        if (f.ImpactPoint is { } ip && !(double.IsFinite(ip.X) && double.IsFinite(ip.Y) && double.IsFinite(ip.Z))) throw new ArgumentOutOfRangeException(paramName, ip, "ImpactPoint must be finite.");
        if (!(f.ImpactRadius >= 0f) || !float.IsFinite(f.ImpactRadius)) throw new ArgumentOutOfRangeException(paramName, f.ImpactRadius, "ImpactRadius must be finite and at least 0.");
        if (!(f.ClusterPower >= 1f && f.ClusterPower <= 16f)) throw new ArgumentOutOfRangeException(paramName, f.ClusterPower, "ClusterPower must be within 1..16.");
        if (!(f.ImpulseThreshold >= 0f) || !float.IsFinite(f.ImpulseThreshold)) throw new ArgumentOutOfRangeException(paramName, f.ImpulseThreshold, "ImpulseThreshold must be finite and at least 0.");
        if (f.Depth < 0 || f.Depth > 8) throw new ArgumentOutOfRangeException(paramName, f.Depth, "Depth must be within 0..8.");
        if (f.ChildCellCount != 0 && (f.ChildCellCount < 2 || f.ChildCellCount > 512)) throw new ArgumentOutOfRangeException(paramName, f.ChildCellCount, "ChildCellCount must be 0 or within 2..512.");
        if (!(f.ChildThresholdScale > 0f) || !float.IsFinite(f.ChildThresholdScale)) throw new ArgumentOutOfRangeException(paramName, f.ChildThresholdScale, "ChildThresholdScale must be positive and finite.");
        if (!(f.MinFragmentVolume >= 0f && f.MinFragmentVolume < 1f)) throw new ArgumentOutOfRangeException(paramName, f.MinFragmentVolume, "MinFragmentVolume must be within [0, 1).");
    }

    // ------------------------------------------------------------------ regions

    internal void SelectRegion(RegionSelection s)
    {
        foreach (var r in Records) if ((uint)r.Body < (uint)s.Bodies.Length && s.Bodies[r.Body]) s.Add("breakable", r.Body);
        foreach (var f in Fragments) if ((uint)f.Body < (uint)s.Bodies.Length && s.Bodies[f.Body]) s.Add("debris", f.Body);
    }
}
