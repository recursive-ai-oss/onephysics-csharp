using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class ContinuousQueryTests
{
    private static PhysicsWorld3D World() => new(new WorldOptions3D { Gravity = default, ThreadCount = 1 });

    [Theory]
    [InlineData(1.5707963267948966)]
    [InlineData(6.283185307179586)]
    [InlineData(64.0)]
    public void RotatingBarFindsFirstImpactEvenWhenEndpointsAreClear(double angle)
    {
        var w = World();
        var bar = w.AddBox(new Vector3F(4, 0.1f, 0.1f), at: default);
        var target = w.AddSphere(0.1f, new Vector3D(0, 1.5, 0), BodyType.Static);
        ulong hash = w.ComputeHash();
        Assert.True(bar.Sweep(default, new Vector3F(0, 0, (float)angle), out var hit));
        double expected = 1.4706289056333368 / angle;
        Assert.InRange(hit.Fraction, expected - 0.001, expected + 0.0001);
        Assert.Equal(target, hit.Body);
        Assert.Equal(0f, hit.Distance);
        Assert.Equal(hash, w.ComputeHash());
    }

    [Fact]
    public void AngularSweepValidationIncludesEmptyAndStaleSources()
    {
        var w = World(); var empty = w.AddEmpty();
        Assert.False(empty.Sweep(default, new Vector3F(0, 0, 1), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.Sweep(default, new Vector3F(float.NaN, 0, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.Sweep(default, new Vector3F(65, 0, 0), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.Sweep(default, default(Vector3F), out _, tolerance: 0));
        empty.Destroy();
        Assert.Throws<StaleHandleException>(() => empty.Sweep(default, default(Vector3F), out _));
    }

    [Fact]
    public void IterativeDepenetrationClearsACornerAndReportsUnsolvableEnclosure()
    {
        var w = World();
        w.AddBox(new Vector3F(2, 20, 20), at: new Vector3D(-1, 0, 0), type: BodyType.Static);
        w.AddBox(new Vector3F(20, 2, 20), at: new Vector3D(0, -1, 0), type: BodyType.Static);
        var shape = ConvexShape3D.Sphere(0.5f);
        ulong hash = w.ComputeHash();
        var result = w.Depenetrate(shape, new Vector3D(0.1, 0.1, 0), default);
        Assert.True(result.Converged);
        Assert.InRange(result.RemainingDepth, 0f, 0.0001f);
        Assert.InRange(result.Translation.X, 0.399, 0.402);
        Assert.InRange(result.Translation.Y, 0.399, 0.402);
        Assert.Equal(hash, w.ComputeHash());
        Assert.False(w.Penetration(shape, new Vector3D(0.1 + result.Translation.X, 0.1 + result.Translation.Y, 0), default, out _));
        var partial = w.Depenetrate(shape, new Vector3D(0.1, 0.1, 0), default, maxDistance: 0.01f);
        Assert.False(partial.Converged); Assert.Equal(DepenetrationStatus.DistanceLimit, partial.Status);
        w.AddBox(new Vector3F(2, 20, 20), at: new Vector3D(1.5, 0, 0), type: BodyType.Static);
        var impossible = w.Depenetrate(shape, new Vector3D(0.25, 1, 0), default, maxIterations: 8);
        Assert.False(impossible.Converged); Assert.Equal(DepenetrationStatus.IterationLimit, impossible.Status);
        Assert.True(impossible.RemainingDepth > 0.1f);
        Assert.True(w.Penetration(shape,new Vector3D(0.25+impossible.Translation.X,1+impossible.Translation.Y,0),default,out var remaining));
        Assert.Equal(remaining.Distance,impossible.RemainingDepth);
    }

    [Fact]
    public void DepenetrationValidatesLimitsAndHandlesEmptyGeometry()
    {
        var w = World(); var empty = ConvexShape3D.Empty;
        var result = w.Depenetrate(empty, default, default);
        Assert.True(result.Converged); Assert.Equal(0, result.Iterations);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Depenetrate(empty, default, default, maxIterations: 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Depenetrate(empty, default, default, tolerance: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Depenetrate(empty, default, default, maxDistance: float.PositiveInfinity));
    }

    [Fact]
    public void NonconvexSelectionDepenetrationRechecksEveryLeafAndRespectsCoordinateLimits()
    {
        var w=World(); var obstacle=w.AddCompound([new(ShapeSpec3D.Box(new(2,20,20)),new(-1,0,0)),new(ShapeSpec3D.Box(new(20,2,20)),new(0,-1,0))],type:BodyType.Static);
        var shape=ConvexShape3D.Sphere(0.5f); var position=new Vector3D(0.1,0.1,0);
        var worldResult=w.Depenetrate(shape,position,default);
        var selected=obstacle.Depenetrate(shape,position,default);
        Assert.True(worldResult.Converged); Assert.True(selected.Converged);
        Assert.Equal(worldResult.Translation,selected.Translation);
        Assert.True(selected.Iterations>=2);
        Assert.False(default(DepenetrationResult3D).Converged);
        var limitWorld=World(); limitWorld.AddPlane(Vector3F.UnitY,2097152);
        var limited=limitWorld.Depenetrate(ConvexShape3D.Sphere(2097152),new(0,4194303,0),default);
        Assert.False(limited.Converged); Assert.Equal(DepenetrationStatus.CoordinateLimit,limited.Status);
        Assert.Equal(default,limited.Translation);
        obstacle.Destroy();
        Assert.Throws<StaleHandleException>(()=>obstacle.Depenetrate(shape,position,default));
    }

    [Fact]
    public void AngularCompoundSweepVisitsOffsetLeavesAndHonorsFiltersAndJoints()
    {
        var w=World(); var source=w.AddCompound([new(ShapeSpec3D.Sphere(0.1f),new(-2,0,0)),new(ShapeSpec3D.Sphere(0.1f),new(2,0,0)),new(ShapeSpec3D.Empty,default)]);
        var target=w.AddSphere(0.1f,new Vector3D(0,2,0),BodyType.Static);
        Assert.True(source.Sweep(default,new Vector3F(0,0,1.5707964f),out var hit));
        Assert.Equal(target,hit.Body); Assert.InRange(hit.Fraction,0.93f,0.94f);
        Assert.False(source.Sweep(default,new Vector3F(0,0,1.5707964f),out _,filter:QueryFilter3D.OnLayer(31)));
        w.AddDistance(source,target,default,new Vector3D(0,2,0));
        Assert.False(source.Sweep(default,new Vector3F(0,0,1.5707964f),out _,excludeJointed:true));
    }

    private static double Next(ref uint seed)
    { seed=unchecked(seed*1664525u+1013904223u); return (seed>>8)*(1.0/16777216.0); }

    [Fact]
    public void ConservativeAdvancementAgreesWithAnalyticBoxCircleOracleOnSeededMotions()
    {
        uint seed=109;
        for(int scene=0;scene<64;scene++)
        {
            double cx=Next(ref seed)*4-2, cy=Next(ref seed)*4-2;
            double vx=Next(ref seed)*2-1, vy=Next(ref seed)*2-1;
            double initial=Next(ref seed)*6.28, angular=Next(ref seed)*6+0.1;
            double Distance(double fraction)
            {
                double angle=initial+angular*fraction, c=Math.Cos(angle), sn=Math.Sin(angle);
                double x=cx+vx*fraction,y=cy+vy*fraction;
                double dx=Math.Max(0,Math.Abs(c*x+sn*y)-1.5),dy=Math.Max(0,Math.Abs(-sn*x+c*y)-0.05);
                return Math.Sqrt(dx*dx+dy*dy)-0.2;
            }
            int first=-1;
            for(int sample=0;sample<=512;sample++) { if(Distance(sample/512.0)<=0) { first=sample; break; } }
            var pose=new RigidTransformF(new QuaternionF(0,0,(float)Math.Sin(initial/2),(float)Math.Cos(initial/2)),new((float)cx,(float)cy,0));
            bool hit=AngularCast3D.Test(ConvexShape3D.Sphere(0.2f),ConvexShape3D.Box(new(1.5f,0.05f,0.05f)),pose,pose.Translation,new((float)vx,(float)vy,0),new(0,0,(float)angular),out float fraction,out _,out _);
            if(first>=0)
            {
                Assert.True(hit,$"scene {scene} missed analytic overlap at sample {first}");
                Assert.True(fraction<=first/512.0+0.0001,$"scene {scene} reported late impact");
            }
            if(hit) Assert.True(Distance(fraction)<=0.00025,$"scene {scene} returned a distant conservative contact: {Distance(fraction)}");
        }
    }

    [Fact]
    public void ContinuousAndIterativeQueriesAllocateNothingAfterScratchWarmup()
    {
        var w=World(); w.AddBox(new Vector3F(2,2,2),type:BodyType.Static);
        var source=w.AddSphere(0.25f,new Vector3D(0,3,0)); var shape=ConvexShape3D.Sphere(0.25f);
        var motion=new Vector3F(0,-5,0); var angular=new Vector3F(0,0,1); var penetrating=new Vector3D(0,1.1,0);
        for(int i=0;i<10;i++) { source.Sweep(motion,angular,out _); w.Depenetrate(shape,penetrating,default); }
        long before=GC.GetAllocatedBytesForCurrentThread();
        for(int i=0;i<100;i++) { source.Sweep(motion,angular,out _); w.Depenetrate(shape,penetrating,default); }
        Assert.Equal(0,GC.GetAllocatedBytesForCurrentThread()-before);
    }
}
