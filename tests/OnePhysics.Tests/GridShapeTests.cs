using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;
using Xunit;
using P2=RecursiveAI.OnePhysics.Physics2D;
using P3=RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class GridShapeTests
{
    [Fact]
    public void TileBoundariesMergeAndCancelInternalEdges()
    {
        var tiles=new TileKind2D[64]; Array.Fill(tiles,TileKind2D.Solid);
        var grid=new TileGrid2D(64,1,tiles);
        Assert.Equal(4,grid.SegmentCount);
        for(int i=0;i<grid.SegmentCount;i++) { var s=grid.GetSegment(i); Assert.True(s.HasGhost1); Assert.True(s.HasGhost2); }
        Assert.Equal(8,grid.WithRectangle(31,0,1,1,TileKind2D.Empty).SegmentCount);
        Assert.Equal(4,grid.SegmentCount);
        var diagonal=new TileGrid2D(2,2,[TileKind2D.Solid,TileKind2D.Empty,TileKind2D.Empty,TileKind2D.Solid]);
        Assert.Equal(8,diagonal.SegmentCount);
        var ramp=new TileGrid2D(2,1,[TileKind2D.SlopeUpRight,TileKind2D.Solid]);
        Assert.Equal(4,ramp.SegmentCount);
    }

    [Fact]
    public void AllSmallTileArrangementsProduceClosedNondegenerateBoundaries()
    {
        var tiles=new TileKind2D[4];
        for(int bits=0;bits<625;bits++)
        {
            int n=bits; for(int i=0;i<4;i++) { tiles[i]=(TileKind2D)(n%5); n/=5; }
            var grid=new TileGrid2D(2,2,tiles);
            for(int i=0;i<grid.SegmentCount;i++)
            {
                var segment=grid.GetSegment(i);
                Assert.True((segment.Vertices[1]-segment.Vertices[0]).Length>0);
                Assert.True(segment.HasGhost1 && segment.HasGhost2);
            }
        }
    }

    [Fact]
    public void TileQueriesEditsForkAndSnapshot()
    {
        var w=new P2.PhysicsWorld2D(); var terrain=w.AddTileGrid(2,1,[TileKind2D.Solid,TileKind2D.Solid]);
        Assert.True(w.Raycast(new(0.5,3),new(0,-1),out var ray,4)); Assert.Equal(2,ray.Distance,5);
        Assert.True(w.RaycastAny(new(0.5,3),new(0,-1),4)); Assert.Single(w.RaycastAll(new(0.5,3),new(0,-1),4));
        Assert.True(w.OverlapCircleAny(new(0.5,1.1),0.2f));
        var before=w.Snapshot(); var fork=w.Fork();
        w.SetTile(terrain,0,0,TileKind2D.Empty);
        Assert.False(w.Raycast(new(0.5,3),new(0,-1),out _,4));
        Assert.True(fork.Raycast(new(0.5,3),new(0,-1),out _,4));
        var fresh=new P2.PhysicsWorld2D(); fresh.Restore(w.Snapshot()); Assert.Equal(w.Snapshot(),fresh.Snapshot());
        w.Restore(before); Assert.Equal(TileKind2D.Solid,w.GetTileGrid(terrain).GetTile(0,0)); Assert.Equal(before,w.Snapshot());
    }

    [Fact]
    public void TileEditsReplayAndExactScene()
    {
        var w=new P2.PhysicsWorld2D(new(){WorldId=998}); using var stream=new MemoryStream();
        using(var recorder=w.StartRecording(stream))
        {
            var terrain=w.AddTileGrid(3,1,[TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid]);
            w.SetTiles(terrain,1,0,2,1,TileKind2D.Empty); w.Step(1f/60);
        }
        stream.Position=0; var player=new ReplayPlayer(stream); var replay=new P2.PhysicsWorld2D(new(){WorldId=998});
        Assert.True(player.ReadNextStep(replay)); Assert.Equal(w.Snapshot(),replay.Snapshot());
    }

    [Fact]
    public void GridScenesRegionsAndBoundaryOwnershipRoundTrip()
    {
        var w=new P2.PhysicsWorld2D();var terrain=w.AddTileGrid(2,1,[TileKind2D.Solid,TileKind2D.Solid]);w.AddCircle(0.2f,new(0.5,3));
        var scene=P2.PhysicsWorld2D.LoadScene(new StringReader(w.SaveScene()));Assert.Equal(w.Snapshot(),scene.Snapshot());
        var hash=w.ComputeRegionHash(new(-1,-1),new(1,1));var archive=w.CaptureRegion(new(-1,-1),new(1,1));
        w.RestoreRegion(archive);Assert.Equal(hash,w.ComputeRegionHash(new(-1,-1),new(1,1)));
        terrain.Destroy();w.Restore(w.Snapshot());
        var owner=w.CreateBody(BodyType.Static,default);var first=w.CreateTileGrid(owner,new TileGrid2D(1,1,[TileKind2D.Solid]));
        Assert.Throws<InvalidOperationException>(()=>first.Destroy());
        Assert.Throws<ArgumentOutOfRangeException>(()=>w.CreateTileGrid(w.CreateBody(BodyType.Static,default),new TileGrid2D(1,1,[TileKind2D.Empty]),new P2.ShapeDef {Friction=float.NaN}));
        var v=new P3.PhysicsWorld3D();v.AddVoxelGrid(1,1,1,[true]);
        Assert.Equal(v.Snapshot(),P3.PhysicsWorld3D.LoadScene(new StringReader(v.SaveScene())).Snapshot());
        var region=v.CaptureRegion(new(-1,-1,-1),new(1,1,1));var regionHash=region.ComputeHash();v.RestoreRegion(region);Assert.Equal(regionHash,v.ComputeRegionHash(new(-1,-1,-1),new(1,1,1)));
        using var stream=new MemoryStream();var recorded=new P3.PhysicsWorld3D(new(){WorldId=998});
        using(var recorder=recorded.StartRecording(stream)) {recorded.AddVoxelGrid(1,1,1,[true]);recorded.Step(1f/60);}
        stream.Position=0;var player=new ReplayPlayer(stream);var replay=new P3.PhysicsWorld3D(new(){WorldId=998});Assert.True(player.ReadNextStep(replay));Assert.Equal(recorded.Snapshot(),replay.Snapshot());
    }

    [Fact]
    public void TileContactsCoverEveryConvexCore()
    {
        var grid=new TileGrid2D(1,1,[TileKind2D.Solid],8);
        Shape2D[] shapes=[Shape2D.CreateCircle(default,0.3f),Shape2D.CreateCapsule(new(-0.2f,0),new(0.2f,0),0.3f),Shape2D.CreateBox(0.3f,0.3f),Shape2D.CreateBox(0.2f,0.2f,0.1f),Shape2D.CreatePolygon([new(-0.3f,-0.3f),new(0.3f,-0.3f),new(0,0.4f)])];
        foreach(var shape in shapes)
        {
            float low=float.PositiveInfinity;foreach(var v in shape.Vertices)low=Math.Min(low,v.Y-shape.Radius);
            bool found=false;
            for(int i=0;i<grid.SegmentCount;i++)
            {
                var m=Collide2D.Collide(grid.GetSegment(i),shape,new Transform2(new(4,8-low-0.02f),Rot2.Identity));
                if(m.PointCount==0)continue;found=true;Assert.InRange(m.Normal.Y,0.999f,1.001f);Assert.InRange(m[0].Separation,-0.0201f,-0.0199f);
            }
            Assert.True(found,shape.Kind.ToString());
        }
    }

    [Fact]
    public void VoxelContactsCoverEveryConvexCoreAndSoftParticles()
    {
        var grid=new VoxelGrid3D(1,1,1,[true],8);var generator=new ManifoldGenerator3D();var buffer=new TriangleShapeBuffer3D();
        ConvexShape3D[] shapes=[ConvexShape3D.Sphere(0.3f),ConvexShape3D.Box(new(0.3f,0.3f,0.3f)),ConvexShape3D.Capsule(0.2f,0.2f),ConvexShape3D.TaperedCapsule(0.3f,0.2f,0.3f),ConvexShape3D.Cylinder(0.3f,0.3f),ConvexShape3D.TaperedCylinder(0.3f,0.2f,0.3f),ConvexShape3D.TaperedCylinder(0.3f,0,0.3f),ScaledShape3D.Create(ConvexShape3D.Box(new(0.2f,0.3f,0.2f)),new Vector3F(1.5f,0.7f,2)),ConvexShape3D.Box(new(0.2f,0.2f,0.2f)).WithCenterOfMassOffset(new(0.5f,0,0)),ConvexShape3D.Triangle(new(-0.3f,0,-0.3f),new(0,0,0.3f),new(0.3f,0,-0.3f),0.1f),ConvexShape3D.FromHull(ConvexHull3D.Build([new(-0.3f,-0.3f,-0.3f),new(0.3f,-0.3f,-0.3f),new(0,-0.3f,0.3f),new(0,0.3f,0)]))];
        foreach(var shape in shapes)
        {
            var pose=new RigidTransformF(QuaternionF.Identity,new(4,8-shape.Support(-Vector3F.UnitY).Y-0.02f,4));
            var candidates=new List<SoupTriangle3D>();grid.GetCandidates(shape,pose,0.02f,candidates);
            bool found=false;
            foreach(var tri in candidates)
            {
                var m=generator.Collide(buffer.Set(tri.V0,tri.V1,tri.V2),shape,pose);
                if(m.PointCount==0)continue;found=true;Assert.InRange(m.Normal.Y,0.998f,1.002f);Assert.InRange(m.MinSeparation,-0.023f,-0.017f);
            }
            Assert.True(found,shape.Kind.ToString());
        }
        var w=new P3.PhysicsWorld3D();w.AddVoxelGrid(2,1,2,[true,true,true,true]);
        var compound=w.AddCompound([new(P3.ShapeSpec3D.Box(new(0.2f,0.2f,0.2f)),default)],at:new(1.3,2.2,1.3));
        var mesh=P3.SoftBodyMesh.FromTriangles([new(0,0,0),new(0.5f,0,0),new(0,0,0.5f)],[0,1,2]);
        var def=P3.SoftBodyDef.Cloth(mesh,new(0.2,2,0.2));def.ParticleRadius=0.02f;var cloth=w.AddSoftBody(def);
        for(int i=0;i<120;i++)w.Step(1f/60);
        for(int i=0;i<3;i++)Assert.InRange(cloth.GetPosition(i).Y,0.98,1.1);
        Assert.InRange(compound.Position.Y,1.09,1.11);
    }

    [Fact]
    public void VoxelExposedFacesHaveOutwardNormalsAndInactiveSeams()
    {
        var grid=new VoxelGrid3D(2,1,1,[true,true]); Assert.Equal(20,grid.TriangleCount);
        Assert.False(grid.TryGetTriangle(2,out _)); // +X of left voxel is hidden.
        Assert.False(grid.TryGetTriangle(12,out _)); // -X of right voxel is hidden.
        Assert.True(grid.TryGetTriangle(6,out var top)); Assert.Equal(Vector3F.UnitY,top.Normal);
        Assert.False(top.IsEdgeActive(2));
        for(int id=0;id<grid.TriangleIdCount;id++) if(grid.TryGetTriangle(id,out var t))
        {
            Vector3F expected=(id%12/2) switch {0=>-Vector3F.UnitX,1=>Vector3F.UnitX,2=>-Vector3F.UnitY,3=>Vector3F.UnitY,4=>-Vector3F.UnitZ,_=>Vector3F.UnitZ};
            Assert.Equal(expected,t.Normal);
        }
    }

    [Fact]
    public void VoxelQueriesMatchBruteTrianglesIncludingGridPlaneRays()
    {
        bool[] cells=new bool[27]; for(int i=0;i<cells.Length;i++) cells[i]=i%3!=1 || i/3%3==0;
        var grid=new VoxelGrid3D(3,3,3,cells);
        var all=new List<TriangleRayHit3D>(); var brute=new List<TriangleRayHit3D>();
        for(int axis=0;axis<3;axis++) for(int i=0;i<9;i++) for(int j=0;j<9;j++)
        {
            float a=i*0.5f-0.5f,b=j*0.5f-0.5f;
            var o=axis==0?new Vector3F(-1,a,b):axis==1?new Vector3F(a,-1,b):new Vector3F(a,b,-1);
            var d=axis==0?Vector3F.UnitX:axis==1?Vector3F.UnitY:Vector3F.UnitZ;
            all.Clear(); brute.Clear(); grid.RayCastAll(o,d,6,all);
            for(int id=0;id<grid.TriangleIdCount;id++) if(grid.TryGetTriangle(id,out var t) && TriangleTests3D.RayTriangle(o,d,t.V0,t.V1,t.V2,false,out double fraction,out bool back) && fraction<=6) brute.Add(new TriangleRayHit3D(id,(float)fraction,o+d*(float)fraction,t.Normal,back));
            brute.Sort((x,y)=> { int c=x.Fraction.CompareTo(y.Fraction); return c!=0?c:x.TriangleId.CompareTo(y.TriangleId); });
            Assert.Equal(brute.Count,all.Count);
            for(int k=0;k<all.Count;k++) { Assert.Equal(brute[k].TriangleId,all[k].TriangleId); Assert.Equal(brute[k].Fraction,all[k].Fraction); }
        }
    }

    [Theory]
    [InlineData(0.5f)]
    [InlineData(0.1f)]
    [InlineData(7.3f)]
    public void RandomVoxelRaysAndBoxesMatchBruteForce(float cellSize)
    {
        var random=new Random(108);
        bool[] cells=new bool[64];for(int i=0;i<cells.Length;i++)cells[i]=random.Next(2)==0;
        var grid=new VoxelGrid3D(4,4,4,cells,cellSize);
        var actual=new List<TriangleRayHit3D>();var expected=new List<TriangleRayHit3D>();
        var ids=new List<int>();var brute=new List<int>();
        for(int n=0;n<1000;n++)
        {
            Vector3F o=new((float)(random.NextDouble()*6-1)*cellSize,(float)(random.NextDouble()*6-1)*cellSize,(float)(random.NextDouble()*6-1)*cellSize);
            Vector3F d=new((float)(random.NextDouble()*2-1),(float)(random.NextDouble()*2-1),(float)(random.NextDouble()*2-1));
            actual.Clear();expected.Clear();grid.RayCastAll(o,d,10,actual);
            var bounds=new Bounds3F(o-new Vector3F(0.2f,0.2f,0.2f),o+new Vector3F(0.2f,0.2f,0.2f));
            ids.Clear();brute.Clear();grid.Overlap(bounds,ids);
            for(int id=0;id<grid.TriangleIdCount;id++) if(grid.TryGetTriangle(id,out var t))
            {
                if(TriangleTests3D.RayTriangle(o,d,t.V0,t.V1,t.V2,false,out double fraction,out bool back) && fraction<=10) expected.Add(new(id,(float)fraction,default,default,back));
                if(TriangleTests3D.BoxTriangle(bounds,t.V0,t.V1,t.V2)) brute.Add(id);
            }
            expected.Sort((x,y)=> {int c=x.Fraction.CompareTo(y.Fraction);return c!=0?c:x.TriangleId.CompareTo(y.TriangleId);});
            Assert.Equal(expected.Count,actual.Count);Assert.Equal(brute,ids);
            for(int i=0;i<actual.Count;i++) {Assert.Equal(expected[i].TriangleId,actual[i].TriangleId);Assert.Equal(expected[i].Fraction,actual[i].Fraction);}
        }
    }

    [Fact]
    public void GridCastsEqualBruteForceAndConcaveEdgesAreInactive()
    {
        var grid=new VoxelGrid3D(2,2,1,[true,true,true,false]);
        Assert.True(grid.TryGetTriangle(18,out var concave));Assert.False(concave.IsEdgeActive(0));
        var ids=new List<int>();for(int i=0;i<grid.TriangleIdCount;i++)if(grid.TryGetTriangle(i,out _))ids.Add(i);
        var random=new Random(108);
        for(int i=0;i<120;i++)
        {
            var shape=i%2==0?ConvexShape3D.Sphere(0.15f):ConvexShape3D.Box(new(0.2f,0.2f,0.2f));
            var start=new RigidTransformF(QuaternionF.Identity,new((float)(random.NextDouble()*4-1),(float)(random.NextDouble()*4-1),(float)(random.NextDouble()*3-1)));
            var motion=new Vector3F((float)(random.NextDouble()*4-2),(float)(random.NextDouble()*4-2),(float)(random.NextDouble()*4-2));
            bool expected=grid.CastTriangles(shape,start,motion,ids,out var brute);
            bool found=grid.ShapeCast(shape,start,motion,out var actual);
            Assert.Equal(expected,found);
            if(found) {Assert.Equal(brute.TriangleId,actual.TriangleId);Assert.Equal(brute.Fraction,actual.Fraction);}
        }
    }

    [Fact]
    public void VoxelClosedCellPlanesAndLongDilatedTraversalAreConservative()
    {
        bool[] cells=new bool[16];Array.Fill(cells,true);var grid=new VoxelGrid3D(16,1,1,cells,0.1f);
        var p=new Vector3F(12*0.1f,0.1f,0.05f);var bounds=new Bounds3F(p,p);
        var actual=new List<int>();var expected=new List<int>();grid.Overlap(bounds,actual);
        for(int id=0;id<grid.TriangleIdCount;id++)if(grid.TryGetTriangle(id,out var t)&&TriangleTests3D.BoxTriangle(bounds,t.V0,t.V1,t.V2))expected.Add(id);
        Assert.Equal(2,expected.Count);Assert.Equal(expected,actual);
        cells=new bool[128*128];Array.Fill(cells,true);var large=new VoxelGrid3D(128,1,128,cells);
        var pose=new RigidTransformF(QuaternionF.Identity,new(-1,2,-1));
        Assert.True(large.ShapeCast(ConvexShape3D.Sphere(0.2f),pose,new(130,-3,130),out var hit));
        Assert.InRange(hit.Fraction,0.2665f,0.2669f);Assert.InRange(hit.Normal.Y,0.999f,1.001f);
    }

    [Fact]
    public void IncrementalTileEditsEqualFreshConstruction()
    {
        var random=new Random(108);var tiles=new TileKind2D[64];var grid=new TileGrid2D(8,8,tiles);
        for(int n=0;n<300;n++)
        {
            int x=random.Next(8),y=random.Next(8);var kind=(TileKind2D)random.Next(5);tiles[y*8+x]=kind;
            grid=grid.WithRectangle(x,y,1,1,kind);var fresh=new TileGrid2D(8,8,tiles);
            Assert.Equal(fresh.SegmentCount,grid.SegmentCount);
            for(int i=0;i<grid.SegmentCount;i++)Assert.Equal(0,TileGrid2D.SegmentComparer.Instance.Compare(fresh.GetSegment(i),grid.GetSegment(i)));
        }
    }

    [Fact]
    public void VoxelWorldSnapshotForkAndQueries()
    {
        var w=new P3.PhysicsWorld3D(); w.AddVoxelGrid(2,1,2,[true,true,true,true]);
        Assert.True(w.Raycast(new(0.25,3,0.25),new(0,-1,0),out var ray,4)); Assert.InRange(ray.Distance,1.9999,2.0001);
        Assert.True(w.RaycastAny(new(0.25,3,0.25),new(0,-1,0),4));
        Assert.True(w.OverlapSphereAny(new(0.25,1.1,0.25),0.2f));
        var fresh=new P3.PhysicsWorld3D(); fresh.Restore(w.Snapshot()); Assert.Equal(w.Snapshot(),fresh.Snapshot()); Assert.Equal(w.Snapshot(),w.Fork().Snapshot());
    }

    [Fact]
    public void SlidingAcrossTileAndVoxelFloorsHasNoVerticalKicks()
    {
        var tiles=new TileKind2D[64]; Array.Fill(tiles,TileKind2D.Solid);
        var w2=new P2.PhysicsWorld2D(new(){SleepEnabled=false}); w2.AddTileGrid(64,1,tiles,material:new P2.Material2D(0));
        var box2=w2.AddBox(new(0.8f,1),new(1,1.5),material:new P2.Material2D(0)); 
        var cells=new bool[32*32]; Array.Fill(cells,true);
        var w3=new P3.PhysicsWorld3D(new(){SleepEnabled=false}); w3.AddVoxelGrid(32,1,32,cells,material:new P3.Material3D(0));
        var box3=w3.AddBox(new(0.8f,1,0.8f),new(1,1.5,1.25),material:new P3.Material3D(0)); 
        var vertices=new List<Vector3F>();var indices=new List<int>();var grid=new VoxelGrid3D(32,1,32,cells);
        for(int id=0;id<grid.TriangleIdCount;id++)if(grid.TryGetTriangle(id,out var tri))
        { indices.Add(vertices.Count);indices.Add(vertices.Count+1);indices.Add(vertices.Count+2);vertices.Add(tri.V0);vertices.Add(tri.V1);vertices.Add(tri.V2); }
        var wm=new P3.PhysicsWorld3D(new(){SleepEnabled=false});wm.AddMesh(vertices.ToArray(),indices.ToArray(),material:new P3.Material3D(0));
        var boxm=wm.AddBox(new(0.8f,1,0.8f),new(1,1.5,1.25),material:new P3.Material3D(0));
        for(int i=0;i<120;i++) { w2.Step(1f/60); w3.Step(1f/60); wm.Step(1f/60); }
        box2.LinearVelocity=new(6,0); box3.LinearVelocity=new(2,0,2);boxm.LinearVelocity=new(2,0,2);
        float max2=0,max3=0,maxm=0;
        for(int i=0;i<600;i++)
        {
            w2.Step(1f/60); w3.Step(1f/60);wm.Step(1f/60);
            max2=Math.Max(max2,float.Abs(box2.LinearVelocity.Y)); max3=Math.Max(max3,float.Abs(box3.LinearVelocity.Y));maxm=Math.Max(maxm,float.Abs(boxm.LinearVelocity.Y));
        }
        Console.WriteLine(FormattableString.Invariant($"flat-floor maxAbsVy tile={max2:R} voxel={max3:R} equivalentMesh={maxm:R}"));
        Assert.True(max2<=1e-4f && max3<=1e-4f,$"tile={max2:R}, voxel={max3:R}, equivalentMesh={maxm:R}");
        Assert.InRange(box2.Position.X,60,62); Assert.InRange(box3.Position.X,20,22);
    }

    [Fact]
    public void GridWorldsAreThreadCountIndependentAndRestoreContinuationIsExact()
    {
        var a=new P2.PhysicsWorld2D(new(){ThreadCount=1}); var b=new P2.PhysicsWorld2D(new(){ThreadCount=8});
        a.AddTileGrid(3,1,[TileKind2D.Solid,TileKind2D.Solid,TileKind2D.Solid]); b.Restore(a.Snapshot()); b.Options.ThreadCount=8;
        a.AddCircle(0.2f,new(1,3)); b.AddCircle(0.2f,new(1,3));
        var x=new P3.PhysicsWorld3D(new(){ThreadCount=1}); var y=new P3.PhysicsWorld3D(new(){ThreadCount=8});
        x.AddVoxelGrid(3,1,3,[true,true,true,true,true,true,true,true,true]); y.Restore(x.Snapshot()); y.Options.ThreadCount=8;
        x.AddSphere(0.2f,new(1,3,1)); y.AddSphere(0.2f,new(1,3,1));
        for(int i=0;i<120;i++) { a.Step(1f/60); b.Step(1f/60); x.Step(1f/60); y.Step(1f/60); Assert.Equal(a.ComputeHash(),b.ComputeHash()); Assert.Equal(x.ComputeHash(),y.ComputeHash()); }
        var s2=a.Snapshot();var s3=x.Snapshot();
        a.Step(1f/60); x.Step(1f/60); var after2=a.Snapshot();var after3=x.Snapshot();
        a.Restore(s2); x.Restore(s3); a.Step(1f/60); x.Step(1f/60); Assert.Equal(after2,a.Snapshot()); Assert.Equal(after3,x.Snapshot());
    }

    [Fact]
    public void IndexedHeightTileAndVoxelMaterialsResolveAndSurviveEdits()
    {
        P2.Material2D[] materials2=[new(0),new(1)];P3.Material3D[] materials3=[new(0),new(1)];
        var h=new P2.PhysicsWorld2D(new(){SleepEnabled=false});
        h.AddHeightField([0,0,0],spacing:2,materials:materials2,materialIndices:[0,1]);
        var ha=h.AddBox(new(0.5f,0.5f),new(1,0.25),material:new P2.Material2D(1));var hb=h.AddBox(new(0.5f,0.5f),new(3,0.25),material:new P2.Material2D(1));h.Step(1f/60);
        int count=0;foreach(var contact in ha.Contacts){count++;Assert.Equal(0,contact.Friction);}Assert.True(count>0);
        count=0;foreach(var contact in hb.Contacts){count++;Assert.Equal(1,contact.Friction);}Assert.True(count>0);
        var w=new P2.PhysicsWorld2D(new(){SleepEnabled=false});var terrain=w.AddTileGrid(2,1,[TileKind2D.Solid,TileKind2D.Solid],materials:materials2,materialIndices:[0,1]);
        Assert.Equal(6,w.GetTileGrid(terrain).SegmentCount);
        var a=w.AddBox(new(0.5f,0.5f),new(0.5,1.25),material:new P2.Material2D(1));var b=w.AddBox(new(0.5f,0.5f),new(1.5,1.25),material:new P2.Material2D(1));w.Step(1f/60);
        count=0;foreach(var contact in a.Contacts){count++;Assert.Equal(0,contact.Friction);}Assert.True(count>0);
        count=0;foreach(var contact in b.Contacts){count++;Assert.Equal(1,contact.Friction);}Assert.True(count>0);
        var snapshot=w.Snapshot();var fresh=new P2.PhysicsWorld2D();fresh.Restore(snapshot);Assert.Equal(snapshot,fresh.Snapshot());Assert.Equal(snapshot,w.Fork().Snapshot());
        w.SetTile(terrain,1,0,TileKind2D.Empty);w.SetTile(terrain,1,0,TileKind2D.Solid);Assert.Equal(1,w.GetTileGrid(terrain).MaterialIndices[1]);
        w.SetTileMaterials(terrain,materials2,[1,0]);w.Step(1f/60);foreach(var contact in a.Contacts)Assert.Equal(1,contact.Friction);
        var v=new P3.PhysicsWorld3D(new(){SleepEnabled=false});var vg=v.AddVoxelGrid(2,1,1,[true,true],materials:materials3,materialIndices:[0,1]);
        var va=v.AddBox(new(0.5f,0.5f,0.5f),new(0.5,1.25,0.5),material:new P3.Material3D(1));var vb=v.AddBox(new(0.5f,0.5f,0.5f),new(1.5,1.25,0.5),material:new P3.Material3D(1));v.Step(1f/60);
        count=0;foreach(var contact in va.Contacts){count++;Assert.Equal(0,contact.Friction);}Assert.True(count>0);
        count=0;foreach(var contact in vb.Contacts){count++;Assert.Equal(1,contact.Friction);}Assert.True(count>0);
        snapshot=v.Snapshot();var vf=new P3.PhysicsWorld3D();vf.Restore(snapshot);Assert.Equal(snapshot,vf.Snapshot());Assert.Equal(snapshot,v.Fork().Snapshot());
        vg.GetShapes()[0].SetVoxelMaterials(materials3,[1,0]);v.Step(1f/60);foreach(var contact in va.Contacts)Assert.Equal(1,contact.Friction);
        Assert.Throws<ArgumentOutOfRangeException>(()=>w.SetTileMaterials(terrain,materials2,[0,2]));
        Assert.Throws<ArgumentException>(()=>vg.GetShapes()[0].SetVoxelMaterials(materials3,[0]));
    }

    [Fact]
    public void GridSectionRejectsExcessRecordsBeforeAllocatingEntries()
    {
        var w=new P2.PhysicsWorld2D();w.Restore(w.Snapshot());
        var writer=StateWriter.ToBuffer();writer.WriteInt32(1);writer.WriteInt32(2);writer.WriteBytes(new byte[114]);
        var error=Assert.Throws<SnapshotFormatException>(()=>w.Core.GetSnapshotSection(0x54494C45).Read(new StateReader(writer.ToArray())));
        Assert.Contains("tile-grid count",error.Message,StringComparison.Ordinal);
    }

    [Fact]
    public void CharacterCrossesTileRampWithinItsSlopeLimit()
    {
        var tiles=new TileKind2D[128];for(int i=0;i<64;i++)tiles[i]=TileKind2D.Solid;
        tiles[88]=TileKind2D.SlopeUpRight;for(int i=25;i<29;i++)tiles[64+i]=TileKind2D.Solid;tiles[93]=TileKind2D.SlopeUpLeft;
        var w=new P2.PhysicsWorld2D();w.AddTileGrid(64,2,tiles,0.5f);
        var character=w.AddCharacter(new(1,1.5),new(){SlopeLimitRadians=0.87266463f});double highest=character.Position.Y;
        for(int i=0;i<600;i++)
        {
            character.MoveVelocity(new(1.5f,character.IsGrounded?0:-2),1f/60);w.Step(1f/60);highest=Math.Max(highest,character.Position.Y);
        }
        Assert.InRange(highest,1.85,2.05);Assert.InRange(character.Position.Y,1.39,1.41);
        Console.WriteLine(FormattableString.Invariant($"tile ramp progress={character.Position.X-1:R} finalHeight={character.Position.Y:R}"));
        Assert.True(character.Position.X>13,$"Ramp traversal stopped at {character.Position}.");
    }

    [Fact]
    public void HostileGridInputsAreTyped()
    {
        Assert.Throws<ArgumentOutOfRangeException>(()=>new TileGrid2D(0,1,[]));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new TileGrid2D(int.MaxValue,int.MaxValue,[]));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new TileGrid2D(1,1,[(TileKind2D)255]));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new TileGrid2D(1,1,[TileKind2D.Solid],float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VoxelGrid3D(int.MaxValue,int.MaxValue,int.MaxValue,[]));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VoxelGrid3D(1,1,1,[true],float.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(()=>new VoxelGrid3D(1024,1024,1024,[]));
    }
}
