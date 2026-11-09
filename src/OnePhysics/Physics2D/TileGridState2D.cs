using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed class TileGridState2D(Engine2D engine) : ISnapshotSection
{
    internal sealed class Entry
    {
        internal int Body, Generation;
        internal TileGrid2D Grid = null!;
        internal ShapeDef Definition;
        internal ContactMaterial[] Materials=[];
        internal ShapeHandle[] Shapes = [];
        internal Entry Copy() => new() { Body=Body,Generation=Generation,Grid=Grid,Definition=Definition,Materials=Materials,Shapes=(ShapeHandle[])Shapes.Clone() };
    }
    internal readonly SortedDictionary<int,Entry> Entries=new();
    private Entry[] _ordered=[];
    internal void Set(Entry entry) { Entries[entry.Body]=entry; RefreshOrder(); }
    private void RefreshOrder()
    {
        _ordered=new Entry[Entries.Count];int i=0;
        foreach(var pair in Entries)_ordered[i++]=pair.Value;
    }
    public int SectionId => 0x54494C45;
    public bool IsOptional => true;
    public bool IsPresent { get { foreach(var e in _ordered) if(Live(e)) return true; return false; } }
    private bool Live(Entry e) => engine.Core.Bodies.Handles.IsAlive(e.Body,e.Generation);
    public void Write(StateWriter w)
    {
        int count=0; foreach(var e in _ordered) if(Live(e)) count++;
        int version=1;
        foreach(var e in _ordered)if(Live(e) && (e.Materials.Length!=0 || !e.Definition.Surface.IsDefault))version=2;
        w.NamedInt32(version,"version"); w.NamedInt32(count,"count");
        foreach(var e in _ordered)
        {
            if(!Live(e)) continue;
            w.DescribeEntity("body",e.Body,e.Generation);
            w.NamedInt32(e.Body,"body"); w.NamedInt32(e.Generation,"generation");
            w.NamedInt32(e.Grid.Width,"width"); w.NamedInt32(e.Grid.Height,"height"); w.NamedSingle(e.Grid.TileSize,"tileSize");
            foreach(var tile in e.Grid.Tiles) w.NamedByte((byte)tile,"tile");
            var d=e.Definition;
            w.NamedSingle(d.Density); w.NamedSingle(d.Friction); w.NamedSingle(d.Restitution); w.NamedSingle(d.RollingResistance);
            w.NamedUInt32(d.CategoryBits); w.NamedUInt32(d.MaskBits); w.NamedInt32(d.GroupIndex); w.NamedBool(d.IsSensor); w.NamedInt32(d.Layer);
            if(version==2)
            {
                WriteSurface(w,d.Surface);
                w.NamedInt32(e.Materials.Length,"materialCount");
                foreach(var material in e.Materials){w.NamedSingle(material.Friction);w.NamedSingle(material.Restitution);w.NamedSingle(material.RollingResistance);}
                w.NamedInt32(e.Grid.MaterialIndices.Length,"materialIndexCount");foreach(int index in e.Grid.MaterialIndices)w.NamedInt32(index,"materialIndex");
            }
            w.NamedInt32(e.Shapes.Length,"segmentCount");
            foreach(var shape in e.Shapes) { w.NamedInt32(shape.Index,"shape"); w.NamedInt32(shape.Generation,"shapeGeneration"); }
        }
    }
    internal static void ValidateDefinition(ShapeDef d)
    {
        d.Surface.Validate(2);
        if(!float.IsFinite(d.Density) || d.Density<0 || !float.IsFinite(d.Friction) || d.Friction<0 || d.Friction>1e15f || !float.IsFinite(d.RollingResistance) || d.RollingResistance<0 || d.RollingResistance>1e15f || !(d.Restitution>=0 && d.Restitution<=1) || d.Layer<0 || d.Layer>31) throw new ArgumentOutOfRangeException(nameof(d),"Tile-grid definition requires finite nonnegative density, friction and rolling resistance within 0..1e15, restitution within 0..1 and layer within 0..31.");
    }
    public void Read(StateReader r)
    {
        int version=r.ReadInt32();if(version!=1 && version!=2) throw r.Error("unsupported tile-grid section version");
        int count=r.ReadCount(57,"tile grids");
        if(count==0) throw r.Error("empty tile-grid section must be absent");
        var staging=new SortedDictionary<int,Entry>();
        int last=-1;bool hasExtras=false;
        var seen=new bool[engine.RestoredShapeCount];
        for(int i=0;i<count;i++)
        {
            int body=r.ReadInt32(), generation=r.ReadInt32(), width=r.ReadInt32(), height=r.ReadInt32(); float size=r.ReadSingle();
            long n=(long)width*height;
            if(body<=last || generation<1 || width<1 || width>1024 || height<1 || height>1024 || n>TileGrid2D.MaxTiles || n>r.Remaining) throw r.Error("invalid tile-grid slot, dimensions or truncated tiles");
            last=body;
            var tiles=new TileKind2D[(int)n]; for(int j=0;j<tiles.Length;j++) tiles[j]=(TileKind2D)r.ReadByte();
            TileGrid2D grid;
            try { grid=new(width,height,tiles,size); }
            catch(ArgumentException ex) { throw r.Error("invalid tile grid: "+ex.Message); }
            var d=new ShapeDef { Density=r.ReadSingle(),Friction=r.ReadSingle(),Restitution=r.ReadSingle(),RollingResistance=r.ReadSingle(),CategoryBits=r.ReadUInt32(),MaskBits=r.ReadUInt32(),GroupIndex=r.ReadInt32(),IsSensor=r.ReadBool(),Layer=r.ReadInt32() };
            ContactMaterial[] materials=[];
            if(version==2)
            {
                d.Surface=ReadSurface(r);
                int nm=r.ReadCount(12,"tile material table");materials=new ContactMaterial[nm];
                try { for(int j=0;j<nm;j++)materials[j]=new(r.ReadSingle(),r.ReadSingle(),r.ReadSingle()); }
                catch(ArgumentException ex){throw r.Error("invalid tile material: "+ex.Message);}
                int ni=r.ReadCount(4,"tile material indices");
                if(nm==0?ni!=0:ni!=tiles.Length)throw r.Error("tile material index count does not match occupancy");
                var indices=new int[ni];r.ReadInt32s(indices);
                try { if(nm!=0)GridMaterials.Check(indices,tiles.Length,nm);grid=grid.WithMaterialIndices(indices); }
                catch(ArgumentException ex){throw r.Error("invalid tile material indices: "+ex.Message);}
            }
            try { ValidateDefinition(d); }
            catch(ArgumentException ex) { throw r.Error("invalid tile-grid shape definition: "+ex.Message); }
            hasExtras|=materials.Length!=0 || !d.Surface.IsDefault;
            int ns=r.ReadCount(8,"tile segments"); if(ns!=grid.SegmentCount) throw r.Error("tile-grid segment count does not match its boundary");
            var shapes=new ShapeHandle[ns];
            for(int j=0;j<ns;j++)
            {
                int s=r.ReadInt32(), g=r.ReadInt32();
                if(!engine.RestoredShapeIsLive(s) || seen[s] || g<1 || s>=engine.Geom.Length || engine.Geom[s].Kind!=ShapeKind2D.ChainSegment || engine.Geom[s].Count!=2 || !engine.Geom[s].HasGhost1 || !engine.Geom[s].HasGhost2 || engine.Geom[s].IsTwoSided || TileGrid2D.SegmentComparer.Instance.Compare(engine.Geom[s],grid.GetSegment(j))!=0) throw r.Error("invalid tile-grid boundary shape");
                seen[s]=true; shapes[j]=new(engine.Core.Id,s,g);
            }
            staging.Add(body,new Entry { Body=body,Generation=generation,Grid=grid,Definition=d,Materials=materials,Shapes=shapes });
        }
        if(version==2 && !hasExtras)throw r.Error("tile-grid version 2 without material or surface state is noncanonical");
        Entries.Clear(); foreach(var pair in staging) Entries.Add(pair.Key,pair.Value); RefreshOrder();
    }
    internal bool Consistent()
    {
        foreach(var e in _ordered)
        {
            if(!Live(e) || engine.Core.Bodies.TypeOf(e.Body)!=BodyType.Static) return false;
            foreach(var shape in e.Shapes) if(!engine.Core.Shapes.Handles.IsAlive(shape.Index,shape.Generation) || engine.Core.Shapes.Owner[shape.Index]!=e.Body) return false;
        }
        return true;
    }
    private static void WriteSurface(StateWriter w,SurfaceProperties s)
    {
        WriteVector(w,s.Velocity);WriteVector(w,s.OneWayDirection);WriteVector(w,s.FrictionAxis);
        w.NamedSingle(s.LongitudinalFriction);w.NamedSingle(s.TransverseFriction);w.NamedSingle(s.SpinningFriction);
    }
    private static void WriteVector(StateWriter w,Vector3F v){w.NamedSingle(v.X);w.NamedSingle(v.Y);w.NamedSingle(v.Z);}
    private static Vector3F ReadVector(StateReader r)=>new(r.ReadSingle(),r.ReadSingle(),r.ReadSingle());
    private static SurfaceProperties ReadSurface(StateReader r)=>new(){Velocity=ReadVector(r),OneWayDirection=ReadVector(r),FrictionAxis=ReadVector(r),LongitudinalFriction=r.ReadSingle(),TransverseFriction=r.ReadSingle(),SpinningFriction=r.ReadSingle()};
    public void Reset() { Entries.Clear(); _ordered=[]; }
    public ISnapshotSection Fork(WorldCore destination)
    {
        var copy=new TileGridState2D((Engine2D)destination.GetSnapshotSection(engine.SectionId));
        foreach(var pair in Entries) if(Live(pair.Value))
        {
            var e=pair.Value.Copy();
            for(int i=0;i<e.Shapes.Length;i++) e.Shapes[i]=new(destination.Id,e.Shapes[i].Index,e.Shapes[i].Generation);
            copy.Entries.Add(pair.Key,e);
        }
        copy.RefreshOrder();copy.Bind(); return copy;
    }
    private void Bind() => engine.Tiles=this;
}

internal sealed partial class Engine2D
{
    private TileGridState2D? _tiles;
    internal TileGridState2D Tiles { get => _tiles??=new(this); set => _tiles=value; }
}
