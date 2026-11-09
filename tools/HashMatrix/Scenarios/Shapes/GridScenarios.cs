using System;
using System.Runtime.CompilerServices;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.HashMatrix;

internal static class GridScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("world2d/tilegrid",ctx=>
        {
            var w=new P2.PhysicsWorld2D(new(){WorldId=991,ThreadCount=ctx.Threads});
            var terrain=w.AddTileGrid(6,2,[TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Empty,TileKind2D.Empty,TileKind2D.SlopeUpRight,TileKind2D.Solid,TileKind2D.SlopeUpLeft,TileKind2D.Empty],materials:[new(0),new(1)],materialIndices:[0,0,0,1,1,1,0,0,0,1,1,1]);
            w.AddBox(new(0.7f,0.7f),new(1,4)); w.AddCircle(0.3f,new(4,5));
            w.AddHeightField([0,0,0],spacing:2,at:new(10,0),materials:[new(0),new(1)],materialIndices:[0,1]);w.AddCircle(0.3f,new(11,3));
            var character=w.AddCharacter(new(0.4,2),new(){SlopeLimitRadians=0.87266463f});
            for(int step=0;step<180;step++)
            {
                if(step==30)w.SetTile(terrain,3,1,TileKind2D.Empty);
                if(step==60) { var bytes=w.Snapshot();w=w.Fork();w.Restore(bytes);terrain=w.GetBody(w.Core.HandleOfBody(0));character=w.GetCharacter(new(w.Id,character.RawHandle.Index,character.RawHandle.Generation));ctx.Write(w.Snapshot()); }
                if(step==90){w.SetTiles(terrain,2,1,3,1,TileKind2D.HalfBottom);w.SetTileMaterials(terrain,[new(0.2f),new(0.8f)],[0,0,0,1,1,1,0,0,0,1,1,1]);}
                if(step==120)w.ShiftOrigin(new(8,-4));
                character.MoveVelocity(new(0.8f,character.IsGrounded?0:-2),1f/60);
                w.Step(1f/60);ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
        Scenarios.Register("world3d/voxelgrid",ctx=>
        {
            var cells=new bool[4*3*4];
            for(int z=0;z<4;z++)for(int y=0;y<3;y++)for(int x=0;x<4;x++)cells[(z*3+y)*4+x]=y==0 || x>=y+1;
            var materialIndices=new int[cells.Length];for(int i=0;i<materialIndices.Length;i++)materialIndices[i]=i%4<2?0:1;
            var w=new P3.PhysicsWorld3D(new(){WorldId=992,ThreadCount=ctx.Threads});w.AddVoxelGrid(4,3,4,cells,materials:[new(0.2f),new(0.8f)],materialIndices:materialIndices);
            w.AddBox(new(0.7f,0.7f,0.7f),new(1,4,1));w.AddCapsule(0.3f,0.2f,new(2,5,2));
            for(int step=0;step<180;step++)
            {
                if(step==60) { var bytes=w.Snapshot();w=w.Fork();w.Restore(bytes);ctx.Write(w.Snapshot()); }
                if(step==120)w.ShiftOrigin(new(8,-4,2));
                w.Step(1f/60);ctx.Write((long)w.ComputeHash());ctx.EndStep();
            }
        });
    }
}
