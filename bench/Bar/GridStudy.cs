using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

// The two representations use identical boundary geometry. Construction is outside the timed steps.
internal static class GridStudy
{
    internal static int Run(Opts options)
    {
        bool timing=options.Has("timing");
        bool pass=true;
        foreach(bool baseline in new[]{false,true})
        {
            var (w2,character2)=Planar(baseline); var (w3,character3)=Spatial(baseline);
            if(timing) for(int warm=0;warm<120;warm++)
            {
                character2.MoveVelocity(new(0,-2),1f/60);w2.Step(1f/60);
                character3.MoveVelocity(new(0,-2,0),1f/60);w3.Step(1f/60);
            }
            double max2=0,max3=0,penetration2=0,penetration3=0;int escapes2=0,escapes3=0;
            long ticks2=0,ticks3=0;
            for(int step=0;step<600;step++)
            {
                long start=timing?Stopwatch.GetTimestamp():0;
                character2.MoveVelocity(new(1.5f,-2),1f/60);w2.Step(1f/60);
                if(timing)ticks2+=Stopwatch.GetTimestamp()-start;
                start=timing?Stopwatch.GetTimestamp():0;
                character3.MoveVelocity(new(0.6f,-2,0),1f/60);w3.Step(1f/60);
                if(timing)ticks3+=Stopwatch.GetTimestamp()-start;
                for(int b=1;b<w2.Core.Bodies.HighWater;b++)
                {
                    var body=w2.GetBody(w2.Core.HandleOfBody(b)); var p=body.Position;var v=body.LinearVelocity;
                    max2=Math.Max(max2,Math.Sqrt(v.X*v.X+v.Y*v.Y));
                    if(!double.IsFinite(p.X+p.Y)||p.X< -1||p.X>33||p.Y<0||p.Y>10)escapes2++;
                    if(b<=20)
                    {
                        var r=w2.Core.Bodies.RotationOf(b);
                        penetration2=Math.Max(penetration2,0.5+0.15*(Math.Abs(r[0])+Math.Abs(r[1]))-p.Y);
                    }
                }
                for(int b=1;b<w3.Core.Bodies.HighWater;b++)
                {
                    var body=w3.GetBody(w3.Core.HandleOfBody(b));var p=body.Position;var v=body.LinearVelocity;
                    max3=Math.Max(max3,Math.Sqrt(v.X*v.X+v.Y*v.Y+v.Z*v.Z));
                    if(!double.IsFinite(p.X+p.Y+p.Z)||p.X< -1||p.X>9||p.Z< -1||p.Z>5||p.Y<0||p.Y>10)escapes3++;
                    if(b<=16)
                    {
                        var rotation=body.Rotation;
                        double half=b%2==0?0.12*Math.Abs(rotation.Rotate(Vector3F.UnitY).Y)+0.08:
                            0.075*(Math.Abs(rotation.Rotate(Vector3F.UnitX).Y)+Math.Abs(rotation.Rotate(Vector3F.UnitY).Y)+Math.Abs(rotation.Rotate(Vector3F.UnitZ).Y));
                        penetration3=Math.Max(penetration3,0.25+half-p.Y);
                    }
                }
            }
            string variant=baseline?"chain/mesh":"grid";
            double progress2=character2.Position.X-1,progress3=character3.Position.X-0.5;
            bool p2=max2<=20&&penetration2<=0.03&&escapes2==0&&progress2>=12;
            bool p3=max3<=20&&penetration3<=0.03&&escapes3==0&&progress3>=4;
            pass&=p2&&p3;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"tile-world-2d representation={variant} steps=600 maxSpeed={max2:R} floorPenetration={penetration2:R} escapes={escapes2} characterProgress={progress2:R} pass={p2} stepMicroseconds={(timing?ticks2*1e6/Stopwatch.Frequency/600:0):R}"));
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,$"voxel-world-3d representation={variant} steps=600 maxSpeed={max3:R} floorPenetration={penetration3:R} escapes={escapes3} characterProgress={progress3:R} pass={p3} stepMicroseconds={(timing?ticks3*1e6/Stopwatch.Frequency/600:0):R}"));
        }
        return pass?0:1;
    }
    private static (P2.PhysicsWorld2D,P2.Character) Planar(bool baseline)
    {
        var tiles=new TileKind2D[64*2];for(int x=0;x<64;x++)tiles[x]=TileKind2D.Solid;
        tiles[64+24]=TileKind2D.SlopeUpRight;for(int x=25;x<29;x++)tiles[64+x]=TileKind2D.Solid;tiles[64+29]=TileKind2D.SlopeUpLeft;
        var grid=new TileGrid2D(64,2,tiles,0.5f);var w=new P2.PhysicsWorld2D(new(){ThreadCount=1});
        if(!baseline)w.AddTileGrid(64,2,tiles,0.5f);
        else
        {
            var terrain=w.CreateBody(BodyType.Static,default);
            for(int i=0;i<grid.SegmentCount;i++)w.CreateShape(terrain,grid.GetSegment(i),new P2.ShapeDef {Density=0});
        }
        for(int i=0;i<20;i++)w.AddBox(new(0.3f,0.3f),new(18+i*0.4,1+(i%3)*0.4));
        var character=w.AddCharacter(new(1,1.5));
        return(w,character);
    }
    private static (P3.PhysicsWorld3D,P3.Character) Spatial(bool baseline)
    {
        const int nx=32,ny=5,nz=16;var cells=new bool[nx*ny*nz];
        for(int z=0;z<nz;z++)for(int x=0;x<nx;x++)
        {
            int top=x<12?0:x<16?x-11:x<24?4:x<28?27-x:0;
            for(int y=0;y<=top;y++)cells[(z*ny+y)*nx+x]=true;
        }
        var grid=new VoxelGrid3D(nx,ny,nz,cells,0.25f);var w=new P3.PhysicsWorld3D(new(){ThreadCount=1});
        if(!baseline)w.AddVoxelGrid(nx,ny,nz,cells,0.25f);
        else
        {
            var vertices=new List<Vector3F>();var indices=new List<int>();
            for(int id=0;id<grid.TriangleIdCount;id++)if(grid.TryGetTriangle(id,out var triangle))
            {
                indices.Add(vertices.Count);indices.Add(vertices.Count+1);indices.Add(vertices.Count+2);
                vertices.Add(triangle.V0);vertices.Add(triangle.V1);vertices.Add(triangle.V2);
            }
            w.AddMesh(vertices.ToArray(),indices.ToArray());
        }
        for(int i=0;i<16;i++)
        {
            Vector3D at=new(0.5+(i%4)*0.45,1+(i/4)*0.4,0.5+(i/4)*0.6);
            if(i%2==0)w.AddBox(new(0.15f,0.15f,0.15f),at);else w.AddCapsule(0.12f,0.08f,at);
        }
        var character=w.AddCharacter(new(0.5,1.3,3.2));return(w,character);
    }
}
