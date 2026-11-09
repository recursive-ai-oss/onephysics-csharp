using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Adds editable static tile terrain. Origin is the bottom-left corner. Tiles are x-fastest; dimensions are 1..1024 with at most 65536 tiles. Size is at least 4e-5 metres, full extent at most 2^20. Throws ArgumentOutOfRangeException for invalid input. Default material is Material2D.Default; default angle is zero radians. Optional scalar materials are copied with one index per tile, empty cells included; indexed entries require default Surface properties.</summary>
    public Body AddTileGrid(int width,int height,ReadOnlySpan<TileKind2D> tiles,float tileSize=1f,Vector2D at=default,float angleRadians=0f,Material2D? material=null,ReadOnlySpan<Material2D> materials=default,ReadOnlySpan<int> materialIndices=default)
    {
        using var replayMutation=RecordMutation("PhysicsWorld2D.AddTileGrid");
        var grid=new TileGrid2D(width,height,tiles,tileSize,materialIndices);
        var body=CreateBody(BodyType.Static,at,angleRadians);
        try { CreateTileGrid(body,grid,ShapeFor(material,0f),materials); }
        catch(Exception) { body.Destroy(); throw; }
        return body;
    }
    /// <summary>Attaches one tile grid to a static body, returning its first segment (default for empty terrain). Throws ArgumentNullException for null, ArgumentException for a foreign body or duplicate grid, StaticOnlyShapeException for a moving body and standard shape-definition errors. Unchanged boundary segments retain handles across edits. Optional scalar materials are copied; the grid must contain one index per tile when a table is supplied.</summary>
    public Shape CreateTileGrid(Body body,TileGrid2D grid,ShapeDef? def=null,ReadOnlySpan<Material2D> materials=default)
    {
        using var replayMutation=RecordMutation("PhysicsWorld2D.CreateTileGrid");
        ArgumentNullException.ThrowIfNull(grid);
        if(body.World!=this) throw new ArgumentException("The body belongs to another world.",nameof(body));
        if(body.Type!=BodyType.Static) throw new StaticOnlyShapeException("Tile grids require a static body.");
        TileGridState2D.ValidateDefinition(def??new ShapeDef());
        var table=GridMaterials.Convert(materials);
        if(table.Length!=0 || grid.MaterialIndices.Length!=0)GridMaterials.Check(grid.MaterialIndices,grid.Tiles.Length,table.Length);
        int b=body.RawHandle.Index;
        if(_engine.Tiles.Entries.TryGetValue(b,out var old) && old.Generation==body.RawHandle.Generation) throw new ArgumentException("The body already has a tile grid.",nameof(body));
        var entry=new TileGridState2D.Entry { Body=b,Generation=body.RawHandle.Generation,Grid=new TileGrid2D(grid.Width,grid.Height,new TileKind2D[grid.Tiles.Length],grid.TileSize),Definition=def??new ShapeDef(),Materials=table };
        ReplaceTileBoundary(body,entry,grid);
        _engine.Tiles.Set(entry);
        return entry.Shapes.Length==0?default:new Shape(this,entry.Shapes[0]);
    }
    /// <summary>Returns immutable tile data for a body's grid. Throws ArgumentException for a foreign body or one without a grid; a stale body throws StaleHandleException.</summary>
    public TileGrid2D GetTileGrid(Body body) => TileEntry(body).Grid;
    /// <summary>Sets or clears one tile. Updates only changed collision segments; unchanged handles and contact caches survive. Throws ArgumentOutOfRangeException for invalid coordinates or kind and ArgumentException when the body has no grid. A stale body throws StaleHandleException.</summary>
    public void SetTile(Body body,int x,int y,TileKind2D kind) => SetTiles(body,x,y,1,1,kind);
    /// <summary>Fills a tile rectangle; Empty clears it. Zero size is a no-op. Mutation is recorded for replay. Input errors leave terrain unchanged. Throws ArgumentOutOfRangeException for an invalid rectangle or tile kind, ArgumentException for a foreign body or missing grid, and StaleHandleException for a stale body.</summary>
    public void SetTiles(Body body,int x,int y,int width,int height,TileKind2D kind)
    {
        using var replayMutation=RecordMutation("PhysicsWorld2D.SetTiles");
        var entry=TileEntry(body); var grid=entry.Grid.WithRectangle(x,y,width,height,kind);
        ReplaceTileBoundary(body,entry,grid);
    }
    /// <summary>Sets copied scalar materials and one table index per tile, empty tiles included. Empty spans remove overrides. Material boundaries split collinear segments while retaining ghost vertices. Invalid lengths, indices or material Surface settings throw ArgumentException before mutation; stale bodies throw StaleHandleException.</summary>
    public void SetTileMaterials(Body body,ReadOnlySpan<Material2D> materials,ReadOnlySpan<int> indices)
    {
        using var mutation=RecordMutation("PhysicsWorld2D.SetTileMaterials");
        var entry=TileEntry(body);var table=GridMaterials.Convert(materials);
        if(table.Length!=0 || indices.Length!=0)GridMaterials.Check(indices,entry.Grid.Tiles.Length,table.Length);
        var copy=entry.Copy();copy.Materials=table;
        ReplaceTileBoundary(body,copy,entry.Grid.WithMaterialIndices(indices));_engine.Tiles.Set(copy);
    }
    private TileGridState2D.Entry TileEntry(Body body)
    {
        if(body.World!=this) throw new ArgumentException("The body belongs to another world.",nameof(body));
        _=body.Type;
        if(!_engine.Tiles.Entries.TryGetValue(body.RawHandle.Index,out var entry) || entry.Generation!=body.RawHandle.Generation) throw new ArgumentException("The body has no tile grid.",nameof(body));
        return entry;
    }
    private void ReplaceTileBoundary(Body body,TileGridState2D.Entry entry,TileGrid2D grid)
    {
        var shapes=new ShapeHandle[grid.SegmentCount]; var added=new List<ShapeHandle>(); var removed=new List<ShapeHandle>();
        int a=0,b=0;
        try
        {
            while(a<entry.Shapes.Length || b<shapes.Length)
            {
                int comparison=a==entry.Shapes.Length?1:b==shapes.Length?-1:TileGrid2D.SegmentComparer.Instance.Compare(entry.Grid.GetSegment(a),grid.GetSegment(b));
                if(comparison==0)
                {
                    if(!_core.Shapes.Handles.IsAlive(entry.Shapes[a].Index,entry.Shapes[a].Generation)) throw new InvalidOperationException("A tile boundary shape was destroyed independently; destroy or edit the grid as a whole.");
                    shapes[b++]=entry.Shapes[a++];
                }
                else if(comparison<0) removed.Add(entry.Shapes[a++]);
                else { var shape=AddShape(body,grid.GetSegment(b),entry.Definition); shapes[b++]=shape.Handle; added.Add(shape.Handle); }
            }
        }
        catch(Exception) { foreach(var shape in added) _engine.DestroyShape(shape); throw; }
        var index=new int[1];
        for(int i=0;i<shapes.Length;i++)
        {
            if(entry.Materials.Length==0) _core.SetContactMaterials(shapes[i],[],[]);
            else {index[0]=0;_core.SetContactMaterials(shapes[i],[entry.Materials[grid.GetSegmentMaterialIndex(i)]],index);}
        }
        foreach(var shape in removed) if(_core.Shapes.Handles.IsAlive(shape.Index,shape.Generation)) _engine.DestroyShape(shape);
        entry.Grid=grid; entry.Shapes=shapes;
    }
}
