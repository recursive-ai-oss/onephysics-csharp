using System;
using System.Linq;
using RecursiveAI.OneMath;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Scene;

// Additive authoring records. The old records keep their existing construction path and bytes.
internal static class AdditionalSceneShapes
{
    internal static bool Recognizes(int dimension, string kind) => kind.StartsWith("offset-", StringComparison.Ordinal) || kind == "empty" ||
        (dimension == 3 ? kind is "plane" or "triangle" or "frustum" or "voxelgrid" : kind is "heightfield" or "tilegrid" or "bitmask");

    private static (string Kind, double[] P, Vector3F Offset) Parts(int dim, ShapeDef s)
    {
        if (!s.Kind.StartsWith("offset-", StringComparison.Ordinal)) return (s.Kind, s.P, default);
        if (s.P.Length < dim) throw new ArgumentException("Mass-offset record needs one offset component per dimension.");
        var p = s.P;
        return (s.Kind[7..], p[..^dim], new((float)p[^dim], (float)p[^(dim-1)], dim == 3 ? (float)p[^1] : 0));
    }

    private static void Length(double[] p, int n)
    {
        if (p.Length != n) throw new ArgumentException("Shape expects " + n + " parameters, found " + p.Length + ".");
    }

    internal static HeightField2D Field(double[] p)
    {
        // n, spacing, n heights, n-1 hole flags (0 or 1).
        if (p.Length < 5 || p[0] < 2 || p[0] > HeightField2D.MaxSamples || p[0] != Math.Truncate(p[0]) || p.Length != 1 + 2 * p[0]) throw new ArgumentException("Planar height field expects n, spacing, n heights and n-1 hole flags.");
        int n = (int)p[0]; var h = new float[n]; var holes = new bool[n-1];
        for (int i=0;i<n;i++) h[i]=(float)p[2+i];
        for (int i=0;i<n-1;i++) { if (p[2+n+i] != 0 && p[2+n+i] != 1) throw new ArgumentException("Hole flag must be 0 or 1."); holes[i]=p[2+n+i] == 1; }
        return new HeightField2D(h,(float)p[1],holes);
    }

    internal static TileGrid2D Bitmask(double[] p)
    {
        if(p.Length<3 || p[0]<1 || p[0]>1024 || p[1]<1 || p[1]>1024 || p[0]!=Math.Truncate(p[0]) || p[1]!=Math.Truncate(p[1]) || p[0]*p[1]>TileGrid2D.MaxTiles || p.Length!=3+p[0]*p[1]) throw new ArgumentException("Bitmask terrain expects width, height, cell size and width*height flags.");
        var cells=new TileKind2D[p.Length-3];
        for(int i=0;i<cells.Length;i++) { double t=p[i+3]; if(t!=0 && t!=1) throw new ArgumentException("Bitmask flag must be 0 or 1."); cells[i]=t==1?TileKind2D.Solid:TileKind2D.Empty; }
        return new TileGrid2D((int)p[0],(int)p[1],cells,(float)p[2],default,true);
    }
    internal static TileGrid2D Tiles(double[] p)
    {
        if(p.Length<3 || p[0]<1 || p[0]>1024 || p[1]<1 || p[1]>1024 || p[0]!=Math.Truncate(p[0]) || p[1]!=Math.Truncate(p[1]) || p[0]*p[1]>TileGrid2D.MaxTiles || p.Length!=3+p[0]*p[1]) throw new ArgumentException("Tile grid expects width, height, tile size and width*height tile kinds.");
        var tiles=new TileKind2D[p.Length-3];
        for(int i=0;i<tiles.Length;i++) { double t=p[i+3]; if(t<0 || t>4 || t!=Math.Truncate(t)) throw new ArgumentException("Tile kind must be an integer in 0..4."); tiles[i]=(TileKind2D)t; }
        return new TileGrid2D((int)p[0],(int)p[1],tiles,(float)p[2]);
    }
    internal static P3.ShapeSpec3D Voxels(double[] p)
    {
        if(p.Length<4 || p[0]<1 || p[0]>1024 || p[1]<1 || p[1]>1024 || p[2]<1 || p[2]>1024 || p[0]!=Math.Truncate(p[0]) || p[1]!=Math.Truncate(p[1]) || p[2]!=Math.Truncate(p[2]) || p[0]*p[1]*p[2]>LowLevel.VoxelGrid3D.MaxCells || p.Length!=4+p[0]*p[1]*p[2]) throw new ArgumentException("Voxel grid expects sizeX, sizeY, sizeZ, cell size and sizeX*sizeY*sizeZ occupancy flags.");
        var cells=new bool[p.Length-4];
        for(int i=0;i<cells.Length;i++) { double t=p[i+4]; if(t!=0 && t!=1) throw new ArgumentException("Voxel occupancy must be 0 or 1."); cells[i]=t==1; }
        return P3.ShapeSpec3D.VoxelGrid((int)p[0],(int)p[1],(int)p[2],cells,(float)p[3]);
    }

    private static Rot2 Rotation(float angle)
    {
        var (sn, cs) = Math64.SinCos(angle);
        return new Rot2((float)cs, (float)sn);
    }

    private static Shape2D Geometry2(string kind, double[] p, float r)
    {
        Vec2 V(int i) => new((float)p[i],(float)p[i+1]);
        switch (kind)
        {
            case "empty": Length(p,0); return Shape2D.Empty;
            case "circle": Length(p,2); return Shape2D.CreateCircle(V(0),r);
            case "capsule": Length(p,4); return Shape2D.CreateCapsule(V(0),V(2),r);
            case "segment": Length(p,4); return Shape2D.CreateSegment(V(0),V(2));
            case "box": Length(p,5); return Shape2D.CreateBox((float)p[0],(float)p[1],V(2),Rotation((float)p[4]),r);
            case "polygon":
                if (p.Length < 6 || p.Length % 2 != 0) throw new ArgumentException("Complete polygon vertices are required.");
                var v=new Vec2[p.Length/2];for(int i=0;i<v.Length;i++)v[i]=V(2*i);return Shape2D.CreatePolygon(v,r);
            default: throw new ArgumentException("Unsupported planar shape kind " + kind + ".");
        }
    }

    private static P3.ShapeSpec3D Geometry3(string kind, double[] p, float radius, ref P3.ShapeDef def)
    {
        Vector3F V(int i) => new((float)p[i],(float)p[i+1],(float)p[i+2]);
        switch(kind)
        {
            case "empty": Length(p,0); return P3.ShapeSpec3D.Empty;
            case "voxelgrid": return Voxels(p);
            case "plane": Length(p,4); return P3.ShapeSpec3D.Plane(V(0),(float)p[3]);
            case "triangle": Length(p,9); return P3.ShapeSpec3D.Triangle(V(0),V(3),V(6),radius);
            case "frustum": Length(p,3); return P3.ShapeSpec3D.TaperedCylinder((float)p[0],(float)p[1],(float)p[2]);
            case "sphere": Length(p,3); def.LocalPosition=V(0);return P3.ShapeSpec3D.Sphere(radius);
            case "box": Length(p,10);def.LocalPosition=V(3);def.LocalRotation=new((float)p[6],(float)p[7],(float)p[8],(float)p[9]);return P3.ShapeSpec3D.Box(V(0)*2);
            case "capsule": Length(p,6);def.LocalPosition=(V(0)+V(3))*0.5f;return P3.ShapeSpec3D.Capsule((V(3)-V(0)).Length()*0.5f,radius);
            case "hull":
                if(p.Length<12||p.Length%3!=0)throw new ArgumentException("Complete hull vertices required.");
                var v=new Vector3F[p.Length/3];for(int i=0;i<v.Length;i++)v[i]=V(3*i);return P3.ShapeSpec3D.ConvexHull(v,radius);
            case "mesh":
                if(p.Length<2 || p[0]<3 || p[1]<3 || p[0]!=Math.Truncate(p[0]) || p[1]!=Math.Truncate(p[1]) || 2+3*p[0]+p[1]!=p.Length || p[1]%3!=0)throw new ArgumentException("Mesh counts must match its payload.");
                int nv=(int)p[0],ni=(int)p[1];var vertices=new Vector3F[nv];var indices=new int[ni];
                for(int i=0;i<nv;i++)vertices[i]=V(2+3*i);
                for(int i=0;i<ni;i++){double index=p[2+3*nv+i];if(index<0||index>=nv||index!=Math.Truncate(index))throw new ArgumentException("Mesh index out of range.");indices[i]=(int)index;}
                return P3.ShapeSpec3D.Mesh(vertices,indices);
            case "heightfield":
                if(p.Length<6 || p[0]<2 || p[1]<2 || p[0]!=Math.Truncate(p[0]) || p[1]!=Math.Truncate(p[1]) || 6+p[0]*p[1]!=p.Length)throw new ArgumentException("Height-field counts must match its payload.");
                def.LocalPosition=V(3);return P3.ShapeSpec3D.HeightField(p.Skip(6).Select(x=>(float)x).ToArray(),(int)p[0],(int)p[1],new((float)p[2],1,(float)p[2]));
            default: throw new ArgumentException("Unsupported spatial shape kind " + kind + ".");
        }
    }

    internal static void Validate(int dim, ShapeDef s)
    {
        var (kind,p,offset)=Parts(dim,s);
        if(dim==2)
        {
            Shape2D.CheckPoint(new(offset.X,offset.Y),"mass offset");
            if(kind=="heightfield"){_=Field(p);return;}
            if(kind=="tilegrid"){_=Tiles(p);return;}
            if(kind=="bitmask"){_=Bitmask(p);return;}
            if(kind=="chain") { if(p.Length<4 || p.Length%2!=0)throw new ArgumentException("Complete chain vertices required.");for(int i=0;i+3<p.Length;i+=2)_=Shape2D.CreateSegment(new((float)p[i],(float)p[i+1]),new((float)p[i+2],(float)p[i+3]));return; }
            _=Geometry2(kind,p,(float)s.Radius).WithCenterOfMassOffset(new(offset.X,offset.Y));
        }
        else
        {
            var d=new P3.ShapeDef(); var spec=Geometry3(kind,p,(float)s.Radius,ref d).WithCenterOfMassOffset(offset);
            _=P3.Geom3D.Build(spec.Recipe.ToArray());
        }
    }

    internal static void Create(P2.PhysicsWorld2D world,P2.Body body,ShapeDef s,P2.ShapeDef def)
    {
        var(kind,p,offset)=Parts(2,s);
        if(kind=="heightfield"){world.CreateHeightField(body,Field(p),def);return;}
        if(kind=="tilegrid"){world.CreateTileGrid(body,Tiles(p),def);return;}
        if(kind=="bitmask"){world.CreateTileGrid(body,Bitmask(p),def);return;}
        if(kind=="chain"){for(int i=0;i+3<p.Length;i+=2)world.CreateSegment(body,new((float)p[i],(float)p[i+1]),new((float)p[i+2],(float)p[i+3]),def);return;}
        world.CreateShape(body,Geometry2(kind,p,(float)s.Radius).WithCenterOfMassOffset(new(offset.X,offset.Y)),def);
    }
    internal static void Create(P3.PhysicsWorld3D world,P3.Body body,ShapeDef s,P3.ShapeDef def)
    {
        var(kind,p,offset)=Parts(3,s);
        var spec=Geometry3(kind,p,(float)s.Radius,ref def);
        if(s.Kind.StartsWith("offset-",StringComparison.Ordinal))spec=spec.WithCenterOfMassOffset(offset);
        world.CreateShape(body,spec,def);
    }
}
