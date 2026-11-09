using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class QueryShapeCoverageTests
{
    private static Body Target(PhysicsWorld2D w,string kind) => kind switch
    {
        "circle" => w.AddCircle(1,type:BodyType.Static),
        "box" => w.AddBox(new(2,2),type:BodyType.Static),
        "capsule" => w.AddCapsule(0.5f,0.5f,type:BodyType.Static),
        "polygon" => w.AddPolygon([new(-1,-1),new(1,-1),new(1,1),new(-1,1)],type:BodyType.Static),
        "rounded" => w.AddPolygon([new(-0.8f,-0.8f),new(0.8f,-0.8f),new(0.8f,0.8f),new(-0.8f,0.8f)],radius:0.2f,type:BodyType.Static),
        "segment" => w.AddSegment(new(-2,0),new(2,0)),
        "chain" => w.AddChain([new(-2,0),new(0,0),new(2,0)]),
        "heightfield" => w.AddHeightField(new float[5],at:new(-2,0)),
        "offset" => w.AddWithCenterOfMassOffset(Shape2D.CreateCircle(default,1),new(0.7f,0.5f),type:BodyType.Static),
        "empty" => w.AddEmpty(),
        _ => throw new ArgumentException(kind)
    };

    [Theory]
    [InlineData("circle",1)] [InlineData("box",1)] [InlineData("capsule",0.5f)] [InlineData("polygon",1)]
    [InlineData("rounded",1)] [InlineData("segment",0)] [InlineData("chain",0)] [InlineData("heightfield",0)]
    [InlineData("offset",1)] [InlineData("empty",0)]
    public void AllPlanarLeavesHaveAnalyticAnswersAndIdenticalBatches(string kind,float top)
    {
        var w=new PhysicsWorld2D(new WorldOptions2D { Gravity=default,ThreadCount=8 });
        var body=Target(w,kind); body.Layer=4;
        bool empty=kind=="empty";
        var from=new Vector2D(0,3); var down=new Vector2F(0,-1);
        Assert.Equal(!empty,body.Raycast(from,down,out var ray,5));
        Assert.Equal(!empty,body.ClosestPoint(from,out var closest,5));
        if(!empty)
        {
            Assert.InRange(ray.Distance,3-top-0.001f,3-top+0.001f);
            Assert.InRange(closest.Distance,3-top-0.001f,3-top+0.001f);
            Assert.True(ray.Shape.Raycast(from,down,out var selected,5)); Assert.Equal(ray.Distance,selected.Distance);
            Assert.False(body.Raycast(from,down,out _,5,QueryFilter2D.OnLayer(5)));
        }
        var mover=Shape2D.CreateCircle(default,0.25f);
        Assert.Equal(!empty,body.ShapeCast(mover,from,0,down,out var cast,5));
        if(!empty) Assert.InRange(cast.Distance,3-top-0.252f,3-top-0.248f);
        var overlapping=new Vector2D(0,top+0.1);
        Assert.Equal(!empty,body.OverlapShape(mover,overlapping,0,out _));
        Assert.Equal(!empty,w.Penetration(mover,overlapping,0,out var penetration));
        if(!empty) Assert.InRange(penetration.Distance,0.145f,0.155f);
        var correction=body.Depenetrate(mover,overlapping,0);
        Assert.True(correction.Converged); Assert.InRange(correction.RemainingDepth,0,0.0001f);
        var bounds=new Shape[16]; int n=w.QueryAabb(new(-0.5,-0.5),new(0.5,top+0.5),bounds);
        Assert.Contains(bounds.AsSpan(0,n).ToArray(),s=>s.Body==body);
        var inputs=new QueryInput2D[513];
        for(int i=0;i<inputs.Length;i++) inputs[i]=(i%5) switch
        {
            0=>QueryInput2D.Ray(from,down,5),1=>QueryInput2D.Cast(mover,from,0,down,5),
            2=>QueryInput2D.Overlap(mover,overlapping,0),3=>QueryInput2D.Closest(from,5),
            _=>QueryInput2D.Penetration(mover,overlapping,0)
        };
        var hits=new QueryHit2D[inputs.Length*4]; var counts=new int[inputs.Length]; var any=new bool[inputs.Length];
        w.QueryAllBatch(inputs,hits,counts,4); w.QueryAnyBatch(inputs,any);
        for(int i=0;i<inputs.Length;i++) { Assert.Equal(!empty,any[i]); Assert.Equal(!empty,counts[i]>0); }
        var serial=new PhysicsWorld2D(new WorldOptions2D { Gravity=default,ThreadCount=1 }); Target(serial,kind);
        var reference=new QueryHit2D[hits.Length]; var referenceCounts=new int[counts.Length]; serial.QueryAllBatch(inputs,reference,referenceCounts,4);
        Assert.Equal(referenceCounts,counts);
        for(int i=0;i<hits.Length;i++) { Assert.Equal(reference[i].Distance,hits[i].Distance); Assert.Equal(reference[i].Point,hits[i].Point); Assert.Equal(reference[i].Normal,hits[i].Normal); }
        var circle=w.AddCircle(0.25f,from);
        Assert.Equal(!empty,circle.Sweep(new Vector2F(0,-5),1f,out var angular));
        if(!empty) Assert.InRange(angular.Fraction,(3-top-0.252f)/5,(3-top-0.248f)/5);
        circle.Destroy();
        var obstacle=w.AddCircle(0.25f,new(0,5),BodyType.Static);
        Assert.Equal(!empty,body.Sweep(new Vector2F(0,5),0f,out var source));
        if(!empty) Assert.Equal(obstacle,source.Body);
        Assert.Equal(!empty,body.Sweep(new Vector2F(0,1),out var linear,5));
        if(!empty) Assert.InRange(linear.Distance,source.Distance-0.005f,source.Distance+0.005f);
    }

    [Fact]
    public void TerrainPenetrationIsAGeometricTwoSidedSurfaceQuery()
    {
        var w=new PhysicsWorld2D(); w.AddHeightField([0,0,0],at:new(-1,0));
        var mover=Shape2D.CreateCircle(default,0.25f);
        Assert.True(w.Penetration(mover,new(0,-0.1),0,out var hit));
        Assert.InRange(hit.Distance,0.149f,0.151f); Assert.True(hit.Normal.Y<0);
        Assert.True(w.Depenetrate(mover,new(0,-0.1),0).Converged);
    }

    [Fact]
    public void OffCentreMassRotatesGeometryAboutTheActualMassCentre()
    {
        var w=new PhysicsWorld2D(new WorldOptions2D { Gravity=default });
        var source=w.AddWithCenterOfMassOffset(Shape2D.CreateCircle(default,0.2f),new(1,0));
        var target=w.AddCircle(0.2f,new(1,-1),BodyType.Static);
        Assert.True(source.Sweep(default,3.1415927f,out var hit));
        Assert.Equal(target,hit.Body); Assert.InRange(hit.Fraction,0.36f,0.39f);
        Assert.Throws<ArgumentOutOfRangeException>(()=>AngularCast2D.Test(Shape2D.CreateCircle(default,1),Shape2D.CreateCircle(default,1),new Transform2(new(float.NaN,0),Rot2.Identity),default,default,0,out _,out _,out _));
        // The motion bound is deliberately pessimistic for spinning circles/spheres.
        // A bounded failure is valid; a false hit or silent skip of an impact is not.
        try { Assert.False(AngularCast2D.Test(Shape2D.CreateSegment(new(-1000,0),new(1000,0)),Shape2D.CreateCircle(default,100),new Transform2(new(0,100.02f),Rot2.Identity),new(0,100.02f),default,1,out _,out _,out _,0.000001f)); }
        catch(QueryConvergenceException failure) { Assert.InRange(failure.SafeFraction,0,0.999999); }
    }

    [Fact]
    public void BroadphaseSelectionMatchesAProxyBoxScanWithFiltersAndTruncation()
    {
        var w=new PhysicsWorld2D(new WorldOptions2D { Gravity=default });
        for(int i=0;i<60;i++) { var body=w.AddCircle(0.3f,new Vector2D(i%10-5,i/10-3),BodyType.Static); body.Layer=i%3; }
        var hits=new Shape[64]; var expected=new System.Collections.Generic.List<int>();
        for(int query=0;query<64;query++)
        {
            var min=new Vector2D((query%8)-5,(query/8)-4); var max=new Vector2D(min.X+2.2,min.Y+2.2);
            var bounds=new Aabb2D(min.X, min.Y, max.X, max.Y); var filter=QueryFilter2D.OnLayer(query%3);
            expected.Clear();
            for(int slot=0;slot<w.Core.Shapes.HighWater;slot++)
            {
                if(!w.Core.Shapes.Handles.IsLive(slot) || w.Engine.ShapeLayer[slot]!=query%3) continue;
                var box=w.Engine.Bp.GetFatBounds(w.Engine.Proxy[slot]);
                if(Aabb2D.Overlaps(box,bounds)) expected.Add(slot);
            }
            int count=w.QueryAabb(min,max,hits,in filter); Assert.Equal(expected.Count,count);
            for(int i=0;i<count;i++) Assert.Equal(expected[i],hits[i].RawHandle.Index);
            count=w.QueryAabb(min,max,hits.AsSpan(0,1),in filter);
            Assert.Equal(Math.Min(1,expected.Count),count);
            if(count!=0) Assert.Equal(expected[0],hits[0].RawHandle.Index);
        }
    }

    [Theory]
    [InlineData("chain")] [InlineData("heightfield")]
    public void RotatingSurfaceSourcesAgreeWithTheAnalyticMovingLine(string kind)
    {
        var w=new PhysicsWorld2D(new WorldOptions2D { Gravity=default }); var source=Target(w,kind);
        var target=w.AddCircle(0.25f,new(source.Position.X,5),BodyType.Static);
        Assert.True(source.Sweep(new Vector2F(0,5),1f,out var hit)); Assert.Equal(target,hit.Body);
        double lo=0,hi=1;
        for(int i=0;i<48;i++) { double t=(lo+hi)*0.5; if(5*(1-t)*Math.Cos(t)>0.25) lo=t; else hi=t; }
        Assert.InRange(hit.Fraction,lo-0.001,hi+0.0001);
    }
}
