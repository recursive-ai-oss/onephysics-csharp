using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class QueryShapeCoverageTests
{
    private static Body Target(PhysicsWorld3D w, string kind) => kind switch
    {
        "sphere" => w.AddSphere(1, type: BodyType.Static),
        "box" => w.AddBox(new(2, 2, 2), type: BodyType.Static),
        "capsule" => w.AddCapsule(0.5f, 0.5f, type: BodyType.Static),
        "cylinder" => w.AddCylinder(1, 1, type: BodyType.Static),
        "frustum" => w.AddTaperedCylinder(1, 0.6f, 1, type: BodyType.Static),
        "cone" => w.AddTaperedCylinder(1, 0, 1, type: BodyType.Static),
        "tapered-capsule" => MakeSpec(w, ShapeSpec3D.TaperedCapsule(0.5f, 0.5f, 0.3f)),
        "hull" => w.AddConvexHull([new(-1,-1,-1), new(-1,-1,1), new(-1,1,-1), new(-1,1,1), new(1,-1,-1), new(1,-1,1), new(1,1,-1), new(1,1,1)], type: BodyType.Static),
        "compound" => w.AddCompound([new(ShapeSpec3D.Empty, default), new(ShapeSpec3D.Box(new(2,2,2)), default), new(ShapeSpec3D.Sphere(0.5f), new(3,0,0))], type: BodyType.Static),
        "triangle" => w.AddTriangle(new(-2,0,-2), new(0,0,2), new(2,0,-2)),
        "mesh" => w.AddMesh([new(-2,0,-2), new(-2,0,2), new(2,0,2), new(2,0,-2)], [0,1,2,0,2,3]),
        "heightfield" => w.AddHeightField(new float[9], 3, 3, new(1,1,1), new(-1,0,-1)),
        "plane" => w.AddPlane(Vector3F.UnitY),
        "offset" => w.AddWithCenterOfMassOffset(ShapeSpec3D.Sphere(1), new(0.7f,0.5f,-0.2f), type: BodyType.Static),
        "empty" => w.AddEmpty(),
        _ => throw new ArgumentException(kind)
    };

    private static Body MakeSpec(PhysicsWorld3D w, ShapeSpec3D shape)
    {
        var b = w.CreateBody(BodyType.Static, default); w.CreateShape(b, shape); return b;
    }

    [Theory]
    [InlineData("sphere",1)] [InlineData("box",1)] [InlineData("capsule",1)] [InlineData("cylinder",1)]
    [InlineData("frustum",1)] [InlineData("cone",1)] [InlineData("tapered-capsule",1)] [InlineData("hull",1)]
    [InlineData("compound",1)] [InlineData("triangle",0)] [InlineData("mesh",0)] [InlineData("heightfield",0)]
    [InlineData("plane",0)] [InlineData("offset",1)] [InlineData("empty",0)]
    public void EveryShapeUsesTheSameLeavesForRestrictedBatchedPenetrationAndSweeps(string kind, float top)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { Gravity = default, ThreadCount = 8 });
        var body = Target(w, kind); body.Layer = 4;
        bool empty = kind == "empty";
        var from = new Vector3D(0, 3, 0); var down = new Vector3F(0,-1,0);
        Assert.Equal(!empty, body.Raycast(from, down, out var ray, 5));
        Assert.Equal(!empty, body.ClosestPoint(from, out var closest, 5));
        if (!empty)
        {
            Assert.InRange(ray.Distance, 3-top-0.001f, 3-top+0.001f);
            Assert.InRange(closest.Distance, 3-top-0.001f, 3-top+0.001f);
            Assert.True(ray.Shape.Raycast(from, down, out var selected, 5));
            Assert.Equal(ray.Distance, selected.Distance);
            Assert.False(body.Raycast(from, down, out _, 5, QueryFilter3D.OnLayer(5)));
        }
        var mover = ConvexShape3D.Sphere(0.25f);
        Assert.Equal(!empty, body.ShapeCast(mover, from, default, down, out var cast, 5));
        if (!empty) { Assert.InRange(cast.Distance, 3-top-0.252f, 3-top-0.248f); }
        var overlapping = new Vector3D(0, top+0.1, 0);
        Assert.Equal(!empty, body.OverlapShape(mover, overlapping, default, out _));
        Assert.Equal(!empty, w.Penetration(mover, overlapping, default, out var penetration));
        if (!empty) { Assert.InRange(penetration.Distance, 0.145f, 0.155f); }
        var correction = body.Depenetrate(mover, overlapping, default);
        Assert.True(correction.Converged);
        Assert.InRange(correction.RemainingDepth, 0, 0.0001f);
        var bounds = new Shape[16]; int n = w.QueryAabb(new(-0.5,-0.5,-0.5), new(0.5,top+0.5,0.5), bounds);
        Assert.Contains(bounds.AsSpan(0,n).ToArray(), s => s.Body == body);

        var inputs = new QueryInput3D[513];
        for (int i=0;i<inputs.Length;i++) inputs[i] = (i%5) switch
        {
            0 => QueryInput3D.Ray(from,down,5), 1 => QueryInput3D.Cast(mover,from,default,down,5),
            2 => QueryInput3D.Overlap(mover,overlapping,default), 3 => QueryInput3D.Closest(from,5),
            _ => QueryInput3D.Penetration(mover,overlapping,default)
        };
        var hits = new QueryHit3D[inputs.Length*4]; var counts = new int[inputs.Length]; var any = new bool[inputs.Length];
        w.QueryAllBatch(inputs,hits,counts,4); w.QueryAnyBatch(inputs,any);
        for(int i=0;i<inputs.Length;i++) { Assert.Equal(!empty,any[i]); Assert.Equal(empty?0:1,counts[i]); }
        var serial = new PhysicsWorld3D(new WorldOptions3D { Gravity=default,ThreadCount=1 }); Target(serial,kind);
        var reference = new QueryHit3D[hits.Length]; var referenceCounts=new int[counts.Length];
        serial.QueryAllBatch(inputs,reference,referenceCounts,4);
        Assert.Equal(referenceCounts,counts);
        for(int i=0;i<hits.Length;i++) { Assert.Equal(reference[i].Distance,hits[i].Distance); Assert.Equal(reference[i].Point,hits[i].Point); Assert.Equal(reference[i].Normal,hits[i].Normal); }

        var sphere = w.AddSphere(0.25f, from);
        Assert.Equal(!empty,sphere.Sweep(new Vector3F(0,-5,0),new Vector3F(0,1,0),out var angularHit));
        if(!empty) { Assert.InRange(angularHit.Fraction,(3-top-0.252f)/5,(3-top-0.248f)/5); }
        sphere.Destroy();
        var obstacle=w.AddSphere(0.25f,new(0,5,0),BodyType.Static);
        Assert.Equal(!empty,body.Sweep(new Vector3F(0,5,0),default(Vector3F),out var sourceHit));
        if(!empty) { Assert.Equal(obstacle,sourceHit.Body); }
        Assert.Equal(!empty,body.Sweep(Vector3F.UnitY,out var linear,5));
        if(!empty) { Assert.InRange(linear.Distance,sourceHit.Distance-0.005f,sourceHit.Distance+0.005f); }
    }

    [Fact]
    public void PlaneAndMassDecoratorsHaveExplicitHostileAndContinuousBehavior()
    {
        var w=new PhysicsWorld3D(new WorldOptions3D { Gravity=default });
        var plane=w.AddPlane(Vector3F.UnitY);
        Assert.False(plane.Sweep(default,new Vector3F(0,0,1),out _));
        var planeTarget=w.AddSphere(0.1f,new(0,1,0),BodyType.Static);
        Assert.True(plane.Sweep(default,new Vector3F(0,0,1.5707964f),out var planeHit));
        Assert.Equal(planeTarget,planeHit.Body); Assert.InRange(planeHit.Fraction,0.935f,0.937f);
        planeTarget.Destroy();
        Assert.Throws<ArgumentException>(()=>AngularCast3D.Test(ConvexShape3D.Plane(Vector3F.UnitY),ConvexShape3D.Plane(Vector3F.UnitX),RigidTransformF.Identity,default,default,default,out _,out _,out _));
        Assert.Throws<ArgumentOutOfRangeException>(()=>plane.Sweep(new Vector3F(float.PositiveInfinity,0,0),default(Vector3F),out _));
        var finite=ConvexShape3D.Sphere(0.5f);
        Assert.True(PhysicsWorld3D.ComputePenetration(ConvexShape3D.Plane(Vector3F.UnitY), new(0,2097152,0), default, finite, new(0,-2097152,0),default,out _,out float depth,out _));
        Assert.True(depth>4194304f);
        plane.Destroy();
        var source=w.AddWithCenterOfMassOffset(ShapeSpec3D.Sphere(0.2f),new(1,0,0));
        var target=w.AddSphere(0.2f,new(1,-1,0),BodyType.Static);
        Assert.True(source.Sweep(default,new Vector3F(0,0,3.1415927f),out var hit));
        Assert.Equal(target,hit.Body); Assert.InRange(hit.Fraction,0.36f,0.39f);
        Assert.Throws<ArgumentOutOfRangeException>(()=>AngularCast3D.Test(finite,finite,new RigidTransformF(QuaternionF.Identity,new(float.MaxValue,0,0)),default,default,default,out _,out _,out _));
        // The motion bound is deliberately pessimistic for spinning circles/spheres.
        // A bounded failure is valid; a false hit or silent skip of an impact is not.
        try { Assert.False(AngularCast3D.Test(ConvexShape3D.Plane(Vector3F.UnitY),ConvexShape3D.Sphere(100),new RigidTransformF(QuaternionF.Identity,new(0,100.02f,0)),new(0,100.02f,0),default,Vector3F.UnitY,out _,out _,out _,0.000001f)); }
        catch(QueryConvergenceException failure) { Assert.InRange(failure.SafeFraction,0,0.999999); }
    }

    [Fact]
    public void ScaledConvexMoversUseSupportGeometryInBothCastForms()
    {
        var w=new PhysicsWorld3D(new WorldOptions3D { Gravity=default }); w.AddPlane(Vector3F.UnitY);
        ConvexShape3D[] shapes=[ScaledShape3D.Create(ConvexShape3D.Sphere(1),2),ScaledShape3D.Create(ConvexShape3D.Box(new(1,1,1)),new Vector3F(2,3,2))];
        float[] tops=[2,3];
        for(int i=0;i<shapes.Length;i++)
        {
            Assert.True(w.ShapeCast(shapes[i],new(0,5,0),default,new(0,-1,0),out var linear,10));
            Assert.InRange(linear.Distance,5-tops[i]-0.002f,5-tops[i]+0.002f);
            Assert.True(AngularCast3D.Test(ConvexShape3D.Plane(Vector3F.UnitY),shapes[i],new RigidTransformF(QuaternionF.Identity,new(0,5,0)),new(0,5,0),new(0,-10,0),Vector3F.UnitY,out float fraction,out _,out _));
            Assert.InRange(fraction,(5-tops[i]-0.002f)/10,(5-tops[i]+0.002f)/10);
        }
    }

    [Fact]
    public void BroadphaseSelectionMatchesAProxyBoxScanWithFiltersAndTruncation()
    {
        var w=new PhysicsWorld3D(new WorldOptions3D { Gravity=default });
        for(int i=0;i<60;i++) { var body=w.AddSphere(0.3f,new Vector3D(i%10-5,i/10-3,0),BodyType.Static); body.Layer=i%3; }
        var hits=new Shape[64]; var expected=new System.Collections.Generic.List<int>();
        for(int query=0;query<64;query++)
        {
            var min=new Vector3D((query%8)-5,(query/8)-4,-1); var max=new Vector3D(min.X+2.2,min.Y+2.2,1);
            var bounds=new Aabb3D(min.X, min.Y, min.Z, max.X, max.Y, max.Z); var filter=QueryFilter3D.OnLayer(query%3);
            expected.Clear();
            for(int slot=0;slot<w.Core.Shapes.HighWater;slot++)
            {
                if(!w.Core.Shapes.Handles.IsLive(slot) || w.Engine.ShapeLayer[slot]!=query%3) continue;
                var box=w.Engine.Bp.GetFatBounds(w.Engine.Proxy[slot]);
                if(Aabb3D.Overlaps(box,bounds)) expected.Add(slot);
            }
            int count=w.QueryAabb(min,max,hits,in filter); Assert.Equal(expected.Count,count);
            for(int i=0;i<count;i++) Assert.Equal(expected[i],hits[i].RawHandle.Index);
            count=w.QueryAabb(min,max,hits.AsSpan(0,1),in filter);
            Assert.Equal(Math.Min(1,expected.Count),count);
            if(count!=0) Assert.Equal(expected[0],hits[0].RawHandle.Index);
        }
    }

    [Fact]
    public void NearlyTangentialLargeTranslationDoesNotRoundClosingVelocityToZero()
    {
        var normal=new Vector3F(1,-1,0).Normalized();
        var start=normal*0.1f;
        Assert.True(AngularCast3D.Test(ConvexShape3D.Plane(normal),ConvexShape3D.Point,new RigidTransformF(QuaternionF.Identity,start),start,new(-4194304,-4194303.75f,0),default,out float fraction,out _,out _));
        Assert.InRange(fraction,0,0.6f);
    }

    [Theory]
    [InlineData("compound")] [InlineData("mesh")] [InlineData("heightfield")]
    public void InfiniteQueryMoversDoNotUseFiniteReachOrProxyRejection(string kind)
    {
        var w=new PhysicsWorld3D(new WorldOptions3D { Gravity=default }); var target=Target(w,kind); target.Position=new(-2097152,0,0);
        var plane=ConvexShape3D.Plane(Vector3F.UnitX); var position=new Vector3D(4194304,0,0);
        Assert.True(w.OverlapShape(plane,position,default,out var overlap)); Assert.Equal(target,overlap.Body);
        Assert.True(w.Penetration(plane,position,default,out var penetration)); Assert.True(penetration.Distance>=6291456);
        Assert.True(w.ShapeCast(plane,position,default,Vector3F.UnitX,out var cast,1)); Assert.Equal(0f,cast.Fraction);
        float localMinimum=kind=="compound"?-1:kind=="mesh"?-2:0;
        Assert.Equal(-2097152.0+localMinimum,cast.Point.X);
        var correction=w.Depenetrate(plane,position,default,maxDistance:PhysicsWorld3D.MaxQueryDistance);
        Assert.True(correction.Converged);
    }

    [Theory]
    [InlineData("triangle")] [InlineData("mesh")] [InlineData("heightfield")]
    public void RotatingStaticSurfaceSourcesKeepTheirLeafPlacements(string kind)
    {
        var w=new PhysicsWorld3D(new WorldOptions3D { Gravity=default }); var source=Target(w,kind);
        var target=w.AddSphere(0.25f,new(source.Position.X,5,source.Position.Z),BodyType.Static);
        Assert.True(source.Sweep(new Vector3F(0,5,0),Vector3F.UnitY,out var hit));
        Assert.Equal(target,hit.Body); Assert.InRange(hit.Fraction,0.949f,0.951f);
    }
}
