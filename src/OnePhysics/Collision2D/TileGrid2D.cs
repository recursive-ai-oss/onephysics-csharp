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

/// <summary>Immutable static tile geometry. Shared edges cancel on a half-cell integer lattice, collinear outer edges merge, and each closed boundary supplies both ghost vertices. Origin is the bottom-left grid corner.</summary>
public sealed class TileGrid2D
{
    /// <summary>Maximum tile count, 65536; dimensions must be in 1..1024.</summary>
    public const int MaxTiles = 65536;
    private readonly TileKind2D[] _tiles;
    private readonly int[] _materialIndices;
    private readonly int[] _segmentMaterials;
    private readonly Shape2D[] _segments;
    private readonly SortedSet<Edge> _edges;

    /// <summary>Copies tiles in x-fastest order. Dimensions are 1..1024, count exactly width*height and at most 65536. Tile size must be finite, at least 4e-5 metres, and total extent at most 2^20 metres. Optional nonnegative material indices contain one entry per tile; empty means zero throughout. Throws ArgumentOutOfRangeException for invalid dimensions, count, size, tile kind or indices.</summary>
    public TileGrid2D(int width, int height, ReadOnlySpan<TileKind2D> tiles, float tileSize = 1f, ReadOnlySpan<int> materialIndices = default)
    {
        if(width<1 || width>1024) throw new ArgumentOutOfRangeException(nameof(width),width,"Tile dimension must be in 1..1024.");
        if(height<1 || height>1024) throw new ArgumentOutOfRangeException(nameof(height),height,"Tile dimension must be in 1..1024.");
        if((long)width*height>MaxTiles || tiles.Length != width*height) throw new ArgumentOutOfRangeException(nameof(tiles),tiles.Length,"Tile count must equal width * height and be at most 65536.");
        if(!float.IsFinite(tileSize) || tileSize<4e-5f || (double)Math.Max(width,height)*tileSize>1048576) throw new ArgumentOutOfRangeException(nameof(tileSize),tileSize,"Tile size must be finite, at least 4e-5 metres, with extent at most 1048576 metres.");
        for(int i=0;i<tiles.Length;i++) CheckKind(tiles[i]);
        ValidateIndices(materialIndices,tiles.Length);
        _materialIndices=materialIndices.ToArray();
        Width=width; Height=height; TileSize=tileSize; _tiles=tiles.ToArray();
        _edges=new SortedSet<Edge>();
        for(int y=0;y<Height;y++) for(int x=0;x<Width;x++) ChangeEdges(_edges,x,y,_tiles[y*Width+x],false);
        (_segments,_segmentMaterials)=Build();
    }
    /// <summary>Number of columns.</summary>
    public int Width { get; }
    /// <summary>Number of rows.</summary>
    public int Height { get; }
    /// <summary>Tile side length in metres.</summary>
    public float TileSize { get; }
    /// <summary>Copied tiles in x-fastest order.</summary>
    public ReadOnlySpan<TileKind2D> Tiles => _tiles;
    /// <summary>Optional per-tile material indices, x-fastest; empty means index zero throughout. Indices are nonnegative; a world validates them against its material table.</summary>
    public ReadOnlySpan<int> MaterialIndices => _materialIndices;
    /// <summary>Returns the table index on a boundary segment; throws ArgumentOutOfRangeException outside the segment range.</summary>
    public int GetSegmentMaterialIndex(int index) { _=GetSegment(index);return _segmentMaterials[index]; }
    /// <summary>Returns immutable geometry partitioned at material boundaries. Empty indices restore a single default region. Invalid length or negative indices throw ArgumentOutOfRangeException.</summary>
    public TileGrid2D WithMaterialIndices(ReadOnlySpan<int> indices)
    {
        ValidateIndices(indices,_tiles.Length);
        return new TileGrid2D(this,(TileKind2D[])_tiles.Clone(),new SortedSet<Edge>(_edges),indices.ToArray());
    }
    private static void ValidateIndices(ReadOnlySpan<int> indices,int count)
    {
        if(indices.Length!=0 && indices.Length!=count)throw new ArgumentOutOfRangeException(nameof(indices),"Material indices must be empty or contain one index per tile.");
        foreach(int index in indices)if(index<0)throw new ArgumentOutOfRangeException(nameof(indices),index,"Material indices must be nonnegative.");
    }
    /// <summary>Number of merged exposed chain segments; zero for empty terrain.</summary>
    public int SegmentCount => _segments.Length;
    /// <summary>Returns a one-sided outer segment and its two ghosts. Throws ArgumentOutOfRangeException outside [0, SegmentCount).</summary>
    public Shape2D GetSegment(int index)
    {
        if((uint)index >= (uint)_segments.Length) throw new ArgumentOutOfRangeException(nameof(index),index,"Segment index must be in [0, SegmentCount).");
        return _segments[index];
    }
    /// <summary>Returns the tile at (x,y). Throws ArgumentOutOfRangeException outside the grid.</summary>
    public TileKind2D GetTile(int x,int y) { CheckRectangle(x,y,1,1); return _tiles[y*Width+x]; }
    /// <summary>Returns an edited immutable copy; a zero-sized rectangle is allowed inside the grid. Throws ArgumentOutOfRangeException for invalid kind, location or size. Original is unchanged.</summary>
    public TileGrid2D WithRectangle(int x,int y,int width,int height,TileKind2D kind)
    {
        CheckKind(kind); CheckRectangle(x,y,width,height);
        var tiles=(TileKind2D[])_tiles.Clone();
        var edges=new SortedSet<Edge>(_edges);
        for(int j=y;j<y+height;j++) for(int i=x;i<x+width;i++)
        {
            int slot=j*Width+i;
            if(tiles[slot]==kind) continue;
            ChangeEdges(edges,i,j,tiles[slot],true); ChangeEdges(edges,i,j,kind,false); tiles[slot]=kind;
        }
        return new TileGrid2D(this,tiles,edges,_materialIndices);
    }
    private static void CheckKind(TileKind2D kind) { if(kind>TileKind2D.SlopeUpLeft) throw new ArgumentOutOfRangeException(nameof(kind),kind,"Unknown tile kind; expected Empty, Solid, HalfBottom, SlopeUpRight or SlopeUpLeft."); }
    private void CheckRectangle(int x,int y,int width,int height)
    {
        if(x<0 || y<0 || width<0 || height<0 || (long)x+width>Width || (long)y+height>Height) throw new ArgumentOutOfRangeException(nameof(x),"Tile rectangle must lie within the grid.");
    }
    private readonly record struct Point(int X,int Y) : IComparable<Point>
    {
        public int CompareTo(Point other) { int c=Y.CompareTo(other.Y); return c!=0?c:X.CompareTo(other.X); }
    }
    private readonly record struct Edge(Point A,Point B) : IComparable<Edge>
    {
        public int CompareTo(Edge other) { int c=A.CompareTo(other.A); return c!=0?c:B.CompareTo(other.B); }
    }
    private TileGrid2D(TileGrid2D source,TileKind2D[] tiles,SortedSet<Edge> edges,int[] materialIndices)
    {
        Width=source.Width; Height=source.Height; TileSize=source.TileSize; _tiles=tiles; _edges=edges; _materialIndices=materialIndices; (_segments,_segmentMaterials)=Build();
    }
    private static void ChangeEdges(SortedSet<Edge> edges,int x,int y,TileKind2D kind,bool remove)
    {
        int xx=2*x, yy=2*y;
        Point a=new(xx,yy), b=new(xx+2,yy), c=new(xx+2,yy+2), d=new(xx,yy+2);
        switch(kind)
        {
            case TileKind2D.Solid: Add(a,b); Add(b,c); Add(c,d); Add(d,a); break;
            case TileKind2D.HalfBottom: c=new(xx+2,yy+1); d=new(xx,yy+1); Add(a,b); Add(b,c); Add(c,d); Add(d,a); break;
            case TileKind2D.SlopeUpRight: Add(a,b); Add(b,c); Add(c,a); break;
            case TileKind2D.SlopeUpLeft: Add(a,b); Add(b,d); Add(d,a); break;
        }
        void Add(Point from,Point to)
        {
            if(remove) (from,to)=(to,from);
            int n=Math.Max(Math.Abs(to.X-from.X),Math.Abs(to.Y-from.Y)); int dx=(to.X-from.X)/n, dy=(to.Y-from.Y)/n;
            for(int i=0;i<n;i++)
            {
                Point p=new(from.X+i*dx,from.Y+i*dy), q=new(p.X+dx,p.Y+dy);
                if(!edges.Remove(new(q,p))) edges.Add(new(p,q));
            }
        }
    }
    private (Shape2D[],int[]) Build()
    {
        var edges=new SortedSet<Edge>(_edges);
        var segments=new List<(Shape2D Shape,int Material)>();
        while(edges.Count!=0)
        {
            Edge start=edges.Min, current=start;
            var loop=new List<Point>();
            do
            {
                edges.Remove(current); loop.Add(current.A);
                if(current.B==start.A) break;
                var low=new Edge(current.B,new(int.MinValue,int.MinValue));
                var high=new Edge(current.B,new(int.MaxValue,int.MaxValue));
                Edge next=default; int best=9;
                foreach(var candidate in edges.GetViewBetween(low,high))
                {
                    int turn=(Direction(candidate)-Direction(current)+8)%8;
                    if(turn<best) { next=candidate; best=turn; }
                }
                if(best==9) throw new InvalidOperationException("Tile boundary is not closed.");
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
                segments.Add((Shape2D.CreateChainSegment(Vec(corners[(i+corners.Count-1)%corners.Count]),Vec(corners[i]),Vec(corners[(i+1)%corners.Count]),Vec(corners[(i+2)%corners.Count]),false),Material(new(corners[i],corners[(i+1)%corners.Count]))));
        }
        segments.Sort(static(a,b)=>SegmentComparer.Instance.Compare(a.Shape,b.Shape));
        var shapes=new Shape2D[segments.Count];var materials=new int[segments.Count];
        for(int i=0;i<shapes.Length;i++){shapes[i]=segments[i].Shape;materials[i]=segments[i].Material;}
        return(shapes,materials);
    }
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
