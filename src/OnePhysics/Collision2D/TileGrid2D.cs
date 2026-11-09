using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics;

/// <summary>Solid geometry within one tile; Y is up. Slopes rise from the bottom corners to the named top corner.</summary>
public enum TileKind2D : byte
{
    /// <summary>No geometry.</summary>
    Empty,
    /// <summary>Full square.</summary>
    Solid,
    /// <summary>Bottom half of a square.</summary>
    HalfBottom,
    /// <summary>Right triangle with a top-right corner.</summary>
    SlopeUpRight,
    /// <summary>Right triangle with a top-left corner.</summary>
    SlopeUpLeft,
}

/// <summary>Immutable static tile geometry. Shared edges cancel on a half-cell integer lattice, collinear outer edges merge, and each closed boundary supplies both ghost vertices. Origin is the bottom-left grid corner. A contour grid (see the constructor) treats each cell as a sample of a bitmask and extracts the boundary by marching squares instead.</summary>
public sealed partial class TileGrid2D
{
    /// <summary>Maximum tile count, 65536; dimensions must be in 1..1024.</summary>
    public const int MaxTiles = 65536;
    private readonly TileKind2D[] _tiles;
    private readonly int[] _materialIndices;
    private Seg[] _seg=[];

    /// <summary>Copies tiles in x-fastest order. Dimensions are 1..1024, count exactly width*height and at most 65536. Tile size must be finite, at least 4e-5 metres, and total extent at most 2^20 metres. Optional nonnegative material indices contain one entry per tile; empty means zero throughout. With <paramref name="contour"/> the grid is a bitmask: every nonempty cell is one solid sample at its centre and the boundary is the marching-squares contour of the samples (octilinear segments through the midpoints between neighbouring samples, with isolated corners cut off); only Empty and Solid cells and no material indices are allowed. Throws ArgumentOutOfRangeException for invalid dimensions, count, size, tile kind or indices.</summary>
    public TileGrid2D(int width, int height, ReadOnlySpan<TileKind2D> tiles, float tileSize = 1f, ReadOnlySpan<int> materialIndices = default, bool contour = false)
    {
        if(width<1 || width>1024) throw new ArgumentOutOfRangeException(nameof(width),width,"Tile dimension must be in 1..1024.");
        if(height<1 || height>1024) throw new ArgumentOutOfRangeException(nameof(height),height,"Tile dimension must be in 1..1024.");
        if((long)width*height>MaxTiles || tiles.Length != width*height) throw new ArgumentOutOfRangeException(nameof(tiles),tiles.Length,"Tile count must equal width * height and be at most 65536.");
        if(!float.IsFinite(tileSize) || tileSize<4e-5f || (double)Math.Max(width,height)*tileSize>1048576) throw new ArgumentOutOfRangeException(nameof(tileSize),tileSize,"Tile size must be finite, at least 4e-5 metres, with extent at most 1048576 metres.");
        for(int i=0;i<tiles.Length;i++) { CheckKind(tiles[i]); if(contour && tiles[i]>TileKind2D.Solid) throw new ArgumentOutOfRangeException(nameof(tiles),tiles[i],"A contour grid holds only Empty and Solid cells."); }
        ValidateIndices(materialIndices,tiles.Length);
        if(contour && materialIndices.Length!=0) throw new ArgumentOutOfRangeException(nameof(materialIndices),"A contour grid has no material indices.");
        _materialIndices=materialIndices.ToArray();
        Width=width; Height=height; TileSize=tileSize; _tiles=tiles.ToArray(); IsContour=contour;
        _seg=BuildGlobal();
    }
    /// <summary>Number of columns.</summary>
    public int Width { get; }
    /// <summary>Number of rows.</summary>
    public int Height { get; }
    /// <summary>Tile side length in metres.</summary>
    public float TileSize { get; }
    /// <summary>True for a bitmask grid whose boundary is a marching-squares contour of the cell centres.</summary>
    public bool IsContour { get; }
    /// <summary>Copied tiles in x-fastest order.</summary>
    public ReadOnlySpan<TileKind2D> Tiles => _tiles;
    /// <summary>Optional per-tile material indices, x-fastest; empty means index zero throughout. Indices are nonnegative; a world validates them against its material table.</summary>
    public ReadOnlySpan<int> MaterialIndices => _materialIndices;
    /// <summary>Returns the table index on a boundary segment; throws ArgumentOutOfRangeException outside the segment range.</summary>
    public int GetSegmentMaterialIndex(int index) { _=GetSegment(index);return _seg[index].Material; }
    /// <summary>Returns immutable geometry partitioned at material boundaries. Empty indices restore a single default region. Invalid length or negative indices throw ArgumentOutOfRangeException.</summary>
    public TileGrid2D WithMaterialIndices(ReadOnlySpan<int> indices)
    {
        ValidateIndices(indices,_tiles.Length);
        if(IsContour && indices.Length!=0) throw new ArgumentOutOfRangeException(nameof(indices),"A contour grid has no material indices.");
        var copy=new TileGrid2D(this,(TileKind2D[])_tiles.Clone(),indices.ToArray());
        copy._seg=copy.BuildGlobal();
        return copy;
    }
    private static void ValidateIndices(ReadOnlySpan<int> indices,int count)
    {
        if(indices.Length!=0 && indices.Length!=count)throw new ArgumentOutOfRangeException(nameof(indices),"Material indices must be empty or contain one index per tile.");
        foreach(int index in indices)if(index<0)throw new ArgumentOutOfRangeException(nameof(indices),index,"Material indices must be nonnegative.");
    }
    /// <summary>Number of merged exposed chain segments; zero for empty terrain.</summary>
    public int SegmentCount => _seg.Length;
    /// <summary>Returns a one-sided outer segment and its two ghosts. Throws ArgumentOutOfRangeException outside [0, SegmentCount).</summary>
    public Shape2D GetSegment(int index)
    {
        if((uint)index >= (uint)_seg.Length) throw new ArgumentOutOfRangeException(nameof(index),index,"Segment index must be in [0, SegmentCount).");
        return _seg[index].Shape;
    }
    /// <summary>Returns the tile at (x,y). Throws ArgumentOutOfRangeException outside the grid.</summary>
    public TileKind2D GetTile(int x,int y) { CheckRectangle(x,y,1,1); return _tiles[y*Width+x]; }
    /// <summary>Returns an edited immutable copy; a zero-sized rectangle is allowed inside the grid. Throws ArgumentOutOfRangeException for invalid kind, location or size. Original is unchanged.</summary>
    public TileGrid2D WithRectangle(int x,int y,int width,int height,TileKind2D kind)
    {
        CheckKind(kind); CheckRectangle(x,y,width,height);
        return WithCells(x,y,width,height,default,kind);
    }
    /// <summary>Returns an edited immutable copy with the cells of a rectangle set to <paramref name="kind"/> wherever <paramref name="mask"/> is true (x-fastest, width*height entries; an empty mask means every cell). Only the boundary near the changed cells is re-extracted; every other segment is carried over unchanged, so a destroyed-and-recreated world segment appears exactly where the boundary changed. Returns this instance when no cell changes. Throws ArgumentOutOfRangeException for an invalid kind, rectangle or mask length, or a kind other than Empty and Solid on a contour grid.</summary>
    public TileGrid2D WithCells(int x,int y,int width,int height,ReadOnlySpan<bool> mask,TileKind2D kind) => Edit(x,y,width,height,mask,kind,out _);

    /// <summary>The cells an edit changed: their number and inclusive bounds.</summary>
    internal readonly record struct Changed(int Count,int X0,int Y0,int X1,int Y1);

    internal TileGrid2D Edit(int x,int y,int width,int height,ReadOnlySpan<bool> mask,TileKind2D kind,out Changed changed)
    {
        changed=default;
        CheckKind(kind); CheckRectangle(x,y,width,height);
        if(IsContour && kind>TileKind2D.Solid) throw new ArgumentOutOfRangeException(nameof(kind),kind,"A contour grid holds only Empty and Solid cells.");
        if(mask.Length!=0 && mask.Length!=width*height) throw new ArgumentOutOfRangeException(nameof(mask),mask.Length,"Mask must be empty or contain width * height entries.");
        TileKind2D[]? tiles=null; int x0=int.MaxValue,y0=int.MaxValue,x1=-1,y1=-1,count=0;
        for(int j=y;j<y+height;j++) for(int i=x;i<x+width;i++)
        {
            if(mask.Length!=0 && !mask[(j-y)*width+(i-x)]) continue;
            int slot=j*Width+i;
            if(_tiles[slot]==kind) continue;
            tiles??=(TileKind2D[])_tiles.Clone();
            tiles[slot]=kind; count++; if(i<x0)x0=i; if(i>x1)x1=i; if(j<y0)y0=j; if(j>y1)y1=j;
        }
        if(tiles==null) return this;
        var grid=new TileGrid2D(this,tiles,_materialIndices);
        grid._seg=grid.BuildLocal(_seg,x0,y0,x1,y1);
        changed=new(count,x0,y0,x1,y1);
        return grid;
    }
    private static void CheckKind(TileKind2D kind) { if(kind>TileKind2D.SlopeUpLeft) throw new ArgumentOutOfRangeException(nameof(kind),kind,"Unknown tile kind; expected Empty, Solid, HalfBottom, SlopeUpRight or SlopeUpLeft."); }
    private void CheckRectangle(int x,int y,int width,int height)
    {
        if(x<0 || y<0 || width<0 || height<0 || (long)x+width>Width || (long)y+height>Height) throw new ArgumentOutOfRangeException(nameof(x),"Tile rectangle must lie within the grid.");
    }
    private TileGrid2D(TileGrid2D source,TileKind2D[] tiles,int[] materialIndices)
    {
        Width=source.Width; Height=source.Height; TileSize=source.TileSize; IsContour=source.IsContour; _tiles=tiles; _materialIndices=materialIndices;
    }
    private readonly record struct Point(int X,int Y) : IComparable<Point>
    {
        public int CompareTo(Point other) { int c=Y.CompareTo(other.Y); return c!=0?c:X.CompareTo(other.X); }
    }
    private readonly record struct Edge(Point A,Point B) : IComparable<Edge>
    {
        public int CompareTo(Edge other) { int c=A.CompareTo(other.A); return c!=0?c:B.CompareTo(other.B); }
    }
    /// <summary>One chain segment with its lattice corners: the ghost before, both ends, the ghost after.</summary>
    private readonly record struct Seg(Shape2D Shape,int Material,Point G1,Point A,Point B,Point G2);

    private bool Sample(int x,int y) => (uint)x<(uint)Width && (uint)y<(uint)Height && _tiles[y*Width+x]!=TileKind2D.Empty;
    private static void Unit(Edge[] buf,ref int n,Point from,Point to)
    {
        int steps=Math.Max(Math.Abs(to.X-from.X),Math.Abs(to.Y-from.Y)); int dx=(to.X-from.X)/steps, dy=(to.Y-from.Y)/steps;
        for(int i=0;i<steps;i++) { var p=new Point(from.X+i*dx,from.Y+i*dy); buf[n++]=new Edge(p,new Point(p.X+dx,p.Y+dy)); }
    }
    private static void Polygon(Edge[] buf,ref int n,ReadOnlySpan<Point> poly)
    {
        for(int i=0;i<poly.Length;i++) Unit(buf,ref n,poly[i],poly[(i+1)%poly.Length]);
    }
    /// <summary>Writes the counter-clockwise unit lattice edges of one cell: a tile in tile mode, a marching-squares dual cell (whose corners are four neighbouring samples) in contour mode. At most 12 edges.</summary>
    private int CellEdges(int cx,int cy,Edge[] buf)
    {
        int n=0;
        if(!IsContour)
        {
            var kind=_tiles[cy*Width+cx]; int xx=2*cx, yy=2*cy;
            Point a=new(xx,yy), b=new(xx+2,yy), c=new(xx+2,yy+2), d=new(xx,yy+2);
            switch(kind)
            {
                case TileKind2D.Solid: Polygon(buf,ref n,[a,b,c,d]); break;
                case TileKind2D.HalfBottom: Polygon(buf,ref n,[a,b,new Point(xx+2,yy+1),new Point(xx,yy+1)]); break;
                case TileKind2D.SlopeUpRight: Polygon(buf,ref n,[a,b,c]); break;
                case TileKind2D.SlopeUpLeft: Polygon(buf,ref n,[a,b,d]); break;
            }
            return n;
        }
        // Marching squares (Lorensen and Cline 1987, in 2D) over the cell-centre samples: sample (x,y) sits at lattice
        // point (2x+1,2y+1), so the midpoints between neighbouring samples fall on lattice points and the contour keeps
        // the integer lattice used for edge cancellation. Samples outside the grid are empty.
        int x0=2*cx-1, y0=2*cy-1;
        bool bl=Sample(cx-1,cy-1), br=Sample(cx,cy-1), tr=Sample(cx,cy), tl=Sample(cx-1,cy);
        Point pbl=new(x0,y0), pbr=new(x0+2,y0), ptr=new(x0+2,y0+2), ptl=new(x0,y0+2);
        Point mb=new(x0+1,y0), mr=new(x0+2,y0+1), mt=new(x0+1,y0+2), ml=new(x0,y0+1);
        if(bl && tr && !br && !tl) { Polygon(buf,ref n,[pbl,mb,ml]); Polygon(buf,ref n,[ptr,mt,mr]); return n; }
        if(br && tl && !bl && !tr) { Polygon(buf,ref n,[mb,pbr,mr]); Polygon(buf,ref n,[ptl,ml,mt]); return n; }
        var list=new Point[8]; int k=0;
        if(bl) list[k++]=pbl; if(bl!=br) list[k++]=mb;
        if(br) list[k++]=pbr; if(br!=tr) list[k++]=mr;
        if(tr) list[k++]=ptr; if(tr!=tl) list[k++]=mt;
        if(tl) list[k++]=ptl; if(tl!=bl) list[k++]=ml;
        if(k>=3) Polygon(buf,ref n,list.AsSpan(0,k));
        return n;
    }
    private int CellCountX => IsContour?Width+1:Width;
    private int CellCountY => IsContour?Height+1:Height;
    private static void Toggle(SortedSet<Edge> edges,Edge e) { if(!edges.Remove(new(e.B,e.A))) edges.Add(e); }
    private Seg[] BuildGlobal()
    {
        var edges=new SortedSet<Edge>(); var buf=new Edge[16]; var net=new int[8];
        for(int y=0;y<CellCountY;y++) for(int x=0;x<CellCountX;x++)
        {
            int n=CellEdges(x,y,buf); for(int i=0;i<n;i++) Toggle(edges,buf[i]);
        }
        var segments=new List<Seg>();
        while(edges.Count!=0)
        {
            Edge start=edges.Min, current=start;
            var loop=new List<Point>();
            do
            {
                edges.Remove(current); loop.Add(current.A);
                if(current.B==start.A) break;
                Net(current.B,buf,net);
                int pick=ChooseOut(net,Direction(current));
                Edge next=new(current.B,new Point(current.B.X+StepX[pick],current.B.Y+StepY[pick]));
                if(!edges.Contains(next)) throw new InvalidOperationException("Tile boundary is not closed.");
                current=next;
            } while(true);
            // Keep only corners, so a flat floor is one face regardless of its tile count.
            var corners=new List<Point>();
            for(int i=0;i<loop.Count;i++)
            {
                var a=loop[(i+loop.Count-1)%loop.Count]; var b=loop[i]; var c=loop[(i+1)%loop.Count];
                if((b.X-a.X)*(c.Y-b.Y)!=(b.Y-a.Y)*(c.X-b.X) || Material(new(a,b))!=Material(new(b,c))) corners.Add(b);
            }
            for(int i=0;i<corners.Count;i++)
                segments.Add(MakeSegment(corners[(i+corners.Count-1)%corners.Count],corners[i],corners[(i+1)%corners.Count],corners[(i+2)%corners.Count]));
        }
        segments.Sort(static(a,b)=>SegmentComparer.Instance.Compare(a.Shape,b.Shape));
        return segments.ToArray();
    }
    private Seg MakeSegment(Point g1,Point a,Point b,Point g2) => new(Shape2D.CreateChainSegment(Vec(g1),Vec(a),Vec(b),Vec(g2),false),Material(new(a,b)),g1,a,b,g2);
    private int Material(Edge edge)
    {
        if(_materialIndices.Length==0)return 0;
        // Sample just inside the solid (the left side) on the eighth-tile lattice.
        // Use only the first half-cell of a merged edge, which cannot cross a material boundary.
        int dx=Math.Sign(edge.B.X-edge.A.X),dy=Math.Sign(edge.B.Y-edge.A.Y);
        int x=(4*edge.A.X+2*dx-dy)/8,y=(4*edge.A.Y+2*dy+dx)/8;
        return _materialIndices[y*Width+x];
    }
    private Vec2 Vec(Point p) => new(p.X*(0.5f*TileSize),p.Y*(0.5f*TileSize));
    private static int Direction(Edge e)
    {
        int x=Math.Sign(e.B.X-e.A.X), y=Math.Sign(e.B.Y-e.A.Y);
        return y==0?(x>0?0:4):y>0?(x>0?1:x==0?2:3):(x<0?5:x==0?6:7);
    }
    internal sealed class SegmentComparer : IComparer<Shape2D>
    {
        internal static readonly SegmentComparer Instance=new();
        public int Compare(Shape2D a,Shape2D b)
        {
            int c=Compare(a.Vertices[0],b.Vertices[0]); if(c!=0)return c;
            c=Compare(a.Vertices[1],b.Vertices[1]); if(c!=0)return c;
            c=Compare(a.Ghost1,b.Ghost1); return c!=0?c:Compare(a.Ghost2,b.Ghost2);
        }
        private static int Compare(Vec2 a,Vec2 b) { int c=a.Y.CompareTo(b.Y); return c!=0?c:a.X.CompareTo(b.X); }
    }
}
