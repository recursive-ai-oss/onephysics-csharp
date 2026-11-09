using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using P2=RecursiveAI.OnePhysics.Physics2D;

internal static class FuzzGrids
{
    internal static void Tiles(byte[] input)
    {
        if(input.Length<12) return;
        int width=BitConverter.ToInt32(input,0),height=BitConverter.ToInt32(input,4); float size=BitConverter.ToSingle(input,8);
        if(input.Length>12 && (input[12]&1)==0) { width=1+(input[0]%16);height=1+(input[4]%16);size=1; }
        int n=width>=1&&width<=16&&height>=1&&height<=16?width*height:0;
        var tiles=new TileKind2D[n]; for(int i=0;i<n;i++) tiles[i]=(TileKind2D)(input[i%input.Length]%6);
        try
        {
            var grid=new TileGrid2D(width,height,tiles,size);
            for(int i=0;i<grid.SegmentCount;i++) _=grid.GetSegment(i);
            if(n>0) _=grid.WithRectangle(0,0,width,height,TileKind2D.Solid);
        }
        catch(Exception ex) when(ex is ArgumentException or InvalidShapeException) { }
    }
    internal static void Voxels(byte[] input)
    {
        if(input.Length<16) return;
        int x=BitConverter.ToInt32(input,0),y=BitConverter.ToInt32(input,4),z=BitConverter.ToInt32(input,8);float size=BitConverter.ToSingle(input,12);
        if(input.Length>16 && (input[16]&1)==0) { x=1+input[0]%8;y=1+input[4]%8;z=1+input[8]%8;size=1; }
        int n=x>=1&&x<=8&&y>=1&&y<=8&&z>=1&&z<=8?x*y*z:0;
        var cells=new bool[n]; for(int i=0;i<n;i++)cells[i]=(input[i%input.Length]&1)!=0;
        try
        {
            var grid=new VoxelGrid3D(x,y,z,cells,size);
            for(int i=0;i<grid.TriangleIdCount;i++) _=grid.TryGetTriangle(i,out _);
            _=grid.RayCast(new(-1,0,0),RecursiveAI.OneMath.Vector3F.UnitX,100,out _);
        }
        catch(ArgumentException) { }
    }
    private static byte[]? s_snapshot;
    internal static void Snapshot(byte[] input)
    {
        if(s_snapshot==null)
        {
            var w=new P2.PhysicsWorld2D(new(){WorldId=36});w.AddTileGrid(3,2,[TileKind2D.Solid,TileKind2D.Solid,TileKind2D.SlopeUpLeft,TileKind2D.Empty,TileKind2D.HalfBottom,TileKind2D.Empty]);s_snapshot=w.Snapshot();
        }
        var bytes=(byte[])s_snapshot.Clone();
        for(int i=0;i+2<input.Length;i+=3) { int offset=bytes.Length-1-(((input[i]<<8)|input[i+1])%Math.Min(500,bytes.Length));bytes[offset]^=input[i+2]; }
        var world=new P2.PhysicsWorld2D(new(){WorldId=36});var before=world.Snapshot();
        try {world.Restore(bytes);}
        catch(SnapshotFormatException) {if(!world.Snapshot().AsSpan().SequenceEqual(before))throw new InvalidOperationException("Rejected grid snapshot changed world.");return;}
        if(!world.Snapshot().AsSpan().SequenceEqual(bytes))throw new InvalidOperationException("Accepted grid snapshot changed bytes.");
    }
}
