using System;
using System.Buffers;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>An immutable dense occupancy grid, static only. Cell (x,y,z) occupies [x,x+1] × [y,y+1] × [z,z+1] times CellSize. Only exposed faces collide; coplanar joins and face diagonals are inactive edges.</summary>
[Experimental("OPHYS0001")]
public sealed class VoxelGrid3D : TriangleSoup3D
{
    /// <summary>Maximum total cells, 2^20; each dimension must be in 1..1024.</summary>
    public const int MaxCells = 1 << 20;
    private readonly bool[] _solid;
    private readonly int _triangleCount;
    private readonly int[] _patches;
    private readonly Bounds3F _bounds;

    /// <summary>Copies occupancy in x-fastest, then y, then z order. Empty grids are allowed. Dimensions are 1..1024, total cells at most 2^20, cell size finite and at least 1e-5 metres, total extent at most 2^20 metres. Throws ArgumentOutOfRangeException for invalid dimensions, count or size.</summary>
    public VoxelGrid3D(int sizeX, int sizeY, int sizeZ, ReadOnlySpan<bool> occupancy, float cellSize = 1f)
    {
        Validate(sizeX, sizeY, sizeZ, occupancy.Length, cellSize);
        SizeX = sizeX; SizeY = sizeY; SizeZ = sizeZ; CellSize = cellSize;
        _solid = occupancy.ToArray();
        _bounds = new Bounds3F(default, new(sizeX * cellSize, sizeY * cellSize, sizeZ * cellSize));
        int count = 0;
        for (int z = 0; z < sizeZ; z++) for (int y = 0; y < sizeY; y++) for (int x = 0; x < sizeX; x++)
            if (Solid(x,y,z)) for (int f = 0; f < 6; f++) if (Exposed(x,y,z,f)) count += 2;
        _triangleCount = count;
        _patches=CoplanarPatchIds.Create(6*_solid.Length);
        for(int z=0;z<SizeZ;z++)for(int y=0;y<SizeY;y++)for(int x=0;x<SizeX;x++)
        {
            if(!Solid(x,y,z))continue;
            int cell=(z*SizeY+y)*SizeX+x;
            for(int f=0;f<6;f++)if(Exposed(x,y,z,f))
            {
                if(f/2!=0 && x>0 && Solid(x-1,y,z) && Exposed(x-1,y,z,f))CoplanarPatchIds.Join(_patches,cell*6+f,(cell-1)*6+f);
                if(f/2!=1 && y>0 && Solid(x,y-1,z) && Exposed(x,y-1,z,f))CoplanarPatchIds.Join(_patches,cell*6+f,(cell-SizeX)*6+f);
                if(f/2!=2 && z>0 && Solid(x,y,z-1) && Exposed(x,y,z-1,f))CoplanarPatchIds.Join(_patches,cell*6+f,(cell-SizeX*SizeY)*6+f);
            }
        }
        CoplanarPatchIds.Finish(_patches);
    }

    internal static void Validate(int x, int y, int z, int count, float size)
    {
        if (x < 1 || x > 1024) throw new ArgumentOutOfRangeException(nameof(x), x, "Voxel dimension must be in 1..1024.");
        if (y < 1 || y > 1024) throw new ArgumentOutOfRangeException(nameof(y), y, "Voxel dimension must be in 1..1024.");
        if (z < 1 || z > 1024) throw new ArgumentOutOfRangeException(nameof(z), z, "Voxel dimension must be in 1..1024.");
        long n = (long)x * y * z;
        if (n > MaxCells || count != n) throw new ArgumentOutOfRangeException(nameof(count), count, "Occupancy must contain exactly sizeX * sizeY * sizeZ entries, at most 1048576.");
        if (!float.IsFinite(size) || size < 1e-5f || (double)Math.Max(x,Math.Max(y,z)) * size > 1048576)
            throw new ArgumentOutOfRangeException(nameof(size), size, "Cell size must be finite, at least 1e-5 metres, and full extent at most 1048576 metres.");
    }

    /// <summary>Number of cells along X.</summary>
    public int SizeX { get; }
    /// <summary>Number of cells along Y.</summary>
    public int SizeY { get; }
    /// <summary>Number of cells along Z.</summary>
    public int SizeZ { get; }
    /// <summary>Cell side length, in metres.</summary>
    public float CellSize { get; }
    /// <summary>Copied occupancy, x-fastest then y then z.</summary>
    public ReadOnlySpan<bool> Occupancy => _solid;
    /// <summary>12 ids per cell, including hidden faces. Face order is -X,+X,-Y,+Y,-Z,+Z, two triangles per face.</summary>
    public override int TriangleIdCount => 12 * _solid.Length;
    /// <summary>Number of exposed triangles.</summary>
    public override int TriangleCount => _triangleCount;
    /// <summary>Full grid bounds in metres, including empty cells.</summary>
    public override Bounds3F Bounds => _bounds;
    /// <summary>Returns occupancy; throws ArgumentOutOfRangeException outside the grid.</summary>
    public bool IsSolid(int x, int y, int z)
    {
        if ((uint)x >= (uint)SizeX || (uint)y >= (uint)SizeY || (uint)z >= (uint)SizeZ) throw new ArgumentOutOfRangeException(nameof(x), "Voxel coordinates must lie within the grid.");
        return Solid(x,y,z);
    }
    private bool Solid(int x,int y,int z) => (uint)x < (uint)SizeX && (uint)y < (uint)SizeY && (uint)z < (uint)SizeZ && _solid[(z * SizeY + y) * SizeX + x];
    private bool Exposed(int x,int y,int z,int f) => !Solid(x + (f == 0 ? -1 : f == 1 ? 1 : 0), y + (f == 2 ? -1 : f == 3 ? 1 : 0), z + (f == 4 ? -1 : f == 5 ? 1 : 0));
    private bool EdgeActive(int x,int y,int z,int tangent,int step)
    {
        if (tangent == 0) x += step; else if (tangent == 1) y += step; else z += step;
        // A solid neighbour either extends this face or turns it into a concave
        // corner. Neither is a convex ridge that should generate an edge normal.
        return !Solid(x,y,z);
    }

    /// <summary>Returns an exposed triangle, or false for an out-of-range id, empty cell or hidden face. No allocation.</summary>
    public override bool TryGetTriangle(int id, out SoupTriangle3D triangle)
    {
        triangle = default;
        if ((uint)id >= (uint)TriangleIdCount) return false;
        int cell = id / 12, f = id % 12 / 2;
        int x = cell % SizeX, y = cell / SizeX % SizeY, z = cell / (SizeX * SizeY);
        if (!Solid(x,y,z)) return false;
        return TryBuild(id,cell,x,y,z,f,out triangle);
    }

    // Builds the triangle of a solid cell's face; hidden faces return false.
    private bool TryBuild(int id,int cell,int x,int y,int z,int f,out SoupTriangle3D triangle)
    {
        triangle = default;
        if (!Exposed(x,y,z,f)) return false;
        int axis = f / 2, u = (axis + 1) % 3, v = (axis + 2) % 3;
        // Evaluate each lattice vertex from its integer coordinates once. Adding CellSize
        // to a rounded neighbour can leave one-ulp cracks when CellSize is not a power of two.
        Vector3F Vertex(int alongU,int alongV)
        {
            int xx=x, yy=y, zz=z;
            int normal=(f&1)!=0?1:0;
            if(axis==0)xx+=normal;else if(axis==1)yy+=normal;else zz+=normal;
            if(u==0)xx+=alongU;else if(u==1)yy+=alongU;else zz+=alongU;
            if(v==0)xx+=alongV;else if(v==1)yy+=alongV;else zz+=alongV;
            return new(xx*CellSize,yy*CellSize,zz*CellSize);
        }
        // U × V points along the positive face normal; reverse U,V on negative faces.
        if ((f & 1) == 0) (u,v) = (v,u);
        var p=Vertex(0,0);var pu=Vertex(1,0);var puv=Vertex(1,1);var pv=Vertex(0,1);
        bool lowV = EdgeActive(x,y,z,v,-1), highU = EdgeActive(x,y,z,u,1);
        bool highV = EdgeActive(x,y,z,v,1), lowU = EdgeActive(x,y,z,u,-1);
        triangle = (id & 1) == 0
            ? new(id,p,pu,puv,(byte)((lowV?1:0)|(highU?2:0)),2*_patches[cell*6+f],true)
            : new(id,p,puv,pv,(byte)((highV?2:0)|(lowU?4:0)),2*_patches[cell*6+f],true);
        return true;
    }

    internal override void BoxWalk(in Bounds3F box, List<int> ids)
    {
        int x0 = Low(box.Min.X, SizeX), y0 = Low(box.Min.Y, SizeY), z0 = Low(box.Min.Z, SizeZ);
        int x1 = High(box.Max.X, SizeX), y1 = High(box.Max.Y, SizeY), z1 = High(box.Max.Z, SizeZ);
        for (int z = z0; z <= z1; z++) for (int y = y0; y <= y1; y++) for (int x = x0; x <= x1; x++)
        {
            if (!Solid(x,y,z)) continue;
            int first = ((z*SizeY+y)*SizeX+x)*12;
            int cell = first/12;
            for (int f = 0; f < 6; f++)
            {
                if (!Exposed(x,y,z,f)) continue;
                for (int h = 0; h < 2; h++)
                    if (TryBuild(first+2*f+h,cell,x,y,z,f,out var tri) && TriangleTests3D.BoxTriangle(box,tri.V0,tri.V1,tri.V2)) ids.Add(tri.Id);
            }
        }
    }
    private int Low(float p,int n)
    {
        int i=(int)Math.Clamp(Math.Floor((double)p/CellSize),0,n);
        // Compare the rounded lattice planes used by the triangles. A quotient
        // alone can round above an integer and lose the cell sharing that plane.
        while(i>0 && Plane(i)>=p)i--;
        while(i<n && Plane(i+1)<p)i++;
        return i;
    }
    private int High(float p,int n)
    {
        int i=(int)Math.Clamp(Math.Floor((double)p/CellSize),-1,n-1);
        while(i<n-1 && Plane(i+1)<=p)i++;
        while(i>=0 && Plane(i)>p)i--;
        return i;
    }

    /// <summary>Sweeps a convex shape through the grid. A swept cell range of at most 4096 cells (local casts) is enumerated directly with no traversal; longer casts use a dilated centre-ray traversal whose stencils are clipped to the swept cell range. Large cross-sections fall back to swept-box enumeration. Fractions and ties are those of TriangleSoup3D.CastTriangles. Invalid placement or nonfinite motion throws ArgumentOutOfRangeException; scratch is optional and cleared.</summary>
    public override bool ShapeCast(in ConvexShape3D shape,in RigidTransformF shapeInSoup,Vector3F motion,out SoupCastHit3D hit,List<int>? scratch=null)
    {
        CheckPlacement(shapeInSoup); CheckFinite(motion,nameof(motion)); hit=default;
        var start=ShapeBounds(shape,shapeInSoup);
        var half=(start.Max-start.Min)*0.5f;
        double rx=Math.Ceiling((double)half.X/CellSize)+1, ry=Math.Ceiling((double)half.Y/CellSize)+1, rz=Math.Ceiling((double)half.Z/CellSize)+1;
        // For broad movers, enumerating the finite grid is cheaper and avoids an unbounded stencil.
        if(motion==default || !double.IsFinite(rx+ry+rz) || rx>16 || ry>16 || rz>16 || (2*rx+1)*(2*ry+1)*(2*rz+1)>4096)
            return base.ShapeCast(shape,shapeInSoup,motion,out hit,scratch);
        int ix=(int)rx, iy=(int)ry, iz=(int)rz;
        var ids=scratch??new List<int>();ids.Clear();
        var sweep=Grow(Bounds3F.Union(start,new(start.Min+motion,start.Max+motion)));
        CheckFinite(sweep.Min,"sweep.Min");CheckFinite(sweep.Max,"sweep.Max");
        int x0=Low(sweep.Min.X,SizeX),x1=High(sweep.Max.X,SizeX);
        int y0=Low(sweep.Min.Y,SizeY),y1=High(sweep.Max.Y,SizeY);
        int z0=Low(sweep.Min.Z,SizeZ),z1=High(sweep.Max.Z,SizeZ);
        if(x0>x1 || y0>y1 || z0>z1)return false;
        if((long)(x1-x0+1)*(y1-y0+1)*(z1-z0+1)<=4096)
        {
            // Local sweeps (including character probes) are cheaper as one exact
            // cell range than as overlapping dilated DDA stencils and a bitset.
            Overlap(sweep,ids);return CastTriangles(shape,shapeInSoup,motion,ids,out hit);
        }
        var centre=(start.Min+start.Max)*0.5f;var ray=new RayD(centre,motion);
        if(!ray.HitBox(-ix*CellSize,-iy*CellSize,-iz*CellSize,(SizeX+ix)*CellSize,(SizeY+iy)*CellSize,(SizeZ+iz)*CellSize,1,out double enter)) return false;
        int x=(int)Math.Clamp(Math.Floor((ray.Ox+ray.Dx*enter)/CellSize),-ix,SizeX+ix);
        int y=(int)Math.Clamp(Math.Floor((ray.Oy+ray.Dy*enter)/CellSize),-iy,SizeY+iy);
        int z=(int)Math.Clamp(Math.Floor((ray.Oz+ray.Dz*enter)/CellSize),-iz,SizeZ+iz);
        int sx=Math.Sign(ray.Dx),sy=Math.Sign(ray.Dy),sz=Math.Sign(ray.Dz);
        int words=(_solid.Length+63)/64;var visited=ArrayPool<ulong>.Shared.Rent(words);visited.AsSpan(0,words).Clear();
        try
        {
            while(x>=-ix && x<=SizeX+ix && y>=-iy && y<=SizeY+iy && z>=-iz && z<=SizeZ+iz && enter<=1)
            {
                for(int zz=Math.Max(z0,z-iz);zz<=Math.Min(z1,z+iz);zz++)
                for(int yy=Math.Max(y0,y-iy);yy<=Math.Min(y1,y+iy);yy++)
                for(int xx=Math.Max(x0,x-ix);xx<=Math.Min(x1,x+ix);xx++)
                {
                    int cell=(zz*SizeY+yy)*SizeX+xx;ulong bit=1UL<<(cell&63);int word=cell>>6;
                    if((visited[word]&bit)!=0)continue;visited[word]|=bit;
                    if(!_solid[cell])continue;
                    for(int k=0;k<12;k++) if(TryGetTriangle(12*cell+k,out var tri) && TriangleTests3D.BoxTriangle(sweep,tri.V0,tri.V1,tri.V2))ids.Add(tri.Id);
                }
                double tx=Next(ray.Ox,ray.Dx,x),ty=Next(ray.Oy,ray.Dy,y),tz=Next(ray.Oz,ray.Dz,z);
                double next=Math.Min(tx,Math.Min(ty,tz));if(next>1 || !double.IsFinite(next))break;
                if(tx==next)x+=sx;if(ty==next)y+=sy;if(tz==next)z+=sz;enter=next;
            }
        }
        finally {ArrayPool<ulong>.Shared.Return(visited);}
        ids.Sort();return CastTriangles(shape,shapeInSoup,motion,ids,out hit);
    }

    internal override void RayWalk(in RayD ray, ref RayCollector collector)
    {
        if (!ray.HitBox(0,0,0,_bounds.Max.X,_bounds.Max.Y,_bounds.Max.Z,collector.Limit,out double enter)) return;
        // Amanatides and Woo (1987): tMax chooses the next grid plane. Recompute
        // its fraction from the rounded lattice coordinate so it agrees with the triangles.
        int x = Start(ray.Ox,ray.Dx,enter,SizeX), y = Start(ray.Oy,ray.Dy,enter,SizeY), z = Start(ray.Oz,ray.Dz,enter,SizeZ);
        int sx = Math.Sign(ray.Dx), sy = Math.Sign(ray.Dy), sz = Math.Sign(ray.Dz);
        double tx = Next(ray.Ox,ray.Dx,x), ty = Next(ray.Oy,ray.Dy,y), tz = Next(ray.Oz,ray.Dz,z);
        int stationary = (sx == 0 && OnPlane(ray.Ox) ? 1 : 0) | (sy == 0 && OnPlane(ray.Oy) ? 2 : 0) | (sz == 0 && OnPlane(ray.Oz) ? 4 : 0);
        while ((uint)x < (uint)SizeX && (uint)y < (uint)SizeY && (uint)z < (uint)SizeZ && enter <= collector.Limit)
        {
            VisitStationary(x,y,z,stationary,ray,ref collector);
            if (collector.Done) return;
            double next = Math.Min(tx,Math.Min(ty,tz));
            if (next > collector.Limit || !double.IsFinite(next)) return;
            int ties = (tx == next ? 1 : 0) | (ty == next ? 2 : 0) | (tz == next ? 4 : 0);
            // Supercover at a corner: test all side cells before the diagonal cell. No mutable ray stamps.
            for (int subset = 1; subset < 8; subset++) if ((subset & ties) == subset && subset != ties)
            {
                VisitStationary(x+((subset&1)!=0?sx:0),y+((subset&2)!=0?sy:0),z+((subset&4)!=0?sz:0),stationary,ray,ref collector);
                if (collector.Done) return;
            }
            if ((ties&1)!=0) { x+=sx; tx=Next(ray.Ox,ray.Dx,x); }
            if ((ties&2)!=0) { y+=sy; ty=Next(ray.Oy,ray.Dy,y); }
            if ((ties&4)!=0) { z+=sz; tz=Next(ray.Oz,ray.Dz,z); }
            enter=next;
        }
    }
    private float Plane(int index) => index*CellSize;
    private bool OnPlane(double p) => p==(float)(Math.Round(p/CellSize)*CellSize);
    private int Start(double o,double d,double t,int n)
    {
        double p=o+d*t;
        int i=(int)Math.Clamp(Math.Floor(p/CellSize),0,n-1);
        while(i>0 && p<Plane(i)) i--;
        while(i<n-1 && p>=Plane(i+1)) i++;
        if(d<0 && p==Plane(i) && i>0) i--;
        return i;
    }
    private double Next(double o,double d,int i) => d==0?double.PositiveInfinity:(Plane(d>0?i+1:i)-o)/d;
    private void VisitStationary(int x,int y,int z,int mask,in RayD ray,ref RayCollector c)
    {
        for(int subset=0;subset<8;subset++) if((subset&mask)==subset)
        {
            int xx=x-((subset&1)!=0?1:0), yy=y-((subset&2)!=0?1:0), zz=z-((subset&4)!=0?1:0);
            if(!Solid(xx,yy,zz)) continue;
            int first=((zz*SizeY+yy)*SizeX+xx)*12;
            for(int k=0;k<12;k++) if(TryGetTriangle(first+k,out var tri) && TriangleTests3D.RayTri(ray.Origin,ray.Direction,tri.V0.ToDouble(),tri.V1.ToDouble(),tri.V2.ToDouble(),c.CullBack,out double t,out bool back))
            { c.Offer(tri.Id,t,back); if(c.Done) return; }
        }
    }
}
