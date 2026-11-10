using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class ContinuousQueryTests
{
    private static PhysicsWorld2D World() => new(new WorldOptions2D { Gravity = default, ThreadCount = 1 });

    [Theory]
    [InlineData(1.5707963267948966)]
    [InlineData(6.283185307179586)]
    [InlineData(64.0)]
    public void RotatingBarFindsFirstImpactEvenWhenEndpointsAreClear(double angle)
    {
        var w = World();
        var bar = w.AddBox(new Vector2F(4, 0.1f), at: default);
        var target = w.AddCircle(0.1f, new Vector2D(0, 1.5), BodyType.Static);
        ulong hash = w.ComputeHash();
        Assert.True(bar.Sweep(default, (float)angle, out var hit));
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
        Assert.False(empty.Sweep(default, 1f, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.Sweep(default, float.NaN, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.Sweep(default, 65f, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => empty.Sweep(default, 0f, out _, tolerance: 0));
        empty.Destroy();
        Assert.Throws<StaleHandleException>(() => empty.Sweep(default, 0f, out _));
    }

    [Fact]
    public void IterativeDepenetrationClearsACornerAndReportsUnsolvableEnclosure()
    {
        var w = World();
        w.AddBox(new Vector2F(2, 20), at: new Vector2D(-1, 0), type: BodyType.Static);
        w.AddBox(new Vector2F(20, 2), at: new Vector2D(0, -1), type: BodyType.Static);
        var shape = Shape2D.CreateCircle(default, 0.5f);
        ulong hash = w.ComputeHash();
        var result = w.Depenetrate(shape, new Vector2D(0.1, 0.1), 0f);
        Assert.True(result.Converged);
        Assert.InRange(result.RemainingDepth, 0f, 0.0001f);
        Assert.InRange(result.Translation.X, 0.399, 0.402);
        Assert.InRange(result.Translation.Y, 0.399, 0.402);
        Assert.Equal(hash, w.ComputeHash());
        Assert.False(w.Penetration(shape, new Vector2D(0.1 + result.Translation.X, 0.1 + result.Translation.Y), 0f, out _));
        var partial = w.Depenetrate(shape, new Vector2D(0.1, 0.1), 0f, maxDistance: 0.01f);
        Assert.False(partial.Converged); Assert.Equal(DepenetrationStatus.DistanceLimit, partial.Status);
        w.AddBox(new Vector2F(2, 20), at: new Vector2D(1.5, 0), type: BodyType.Static);
        var impossible = w.Depenetrate(shape, new Vector2D(0.25, 1), 0f, maxIterations: 8);
        Assert.False(impossible.Converged); Assert.Equal(DepenetrationStatus.IterationLimit, impossible.Status);
        Assert.True(impossible.RemainingDepth > 0.1f);
        Assert.True(w.Penetration(shape,new Vector2D(0.25+impossible.Translation.X,1+impossible.Translation.Y),0f,out var remaining));
        Assert.Equal(remaining.Distance,impossible.RemainingDepth);
    }

    [Fact]
    public void DepenetrationValidatesLimitsAndHandlesEmptyGeometry()
    {
        var w = World(); var empty = Shape2D.Empty;
        var result = w.Depenetrate(empty, default, 0f);
        Assert.True(result.Converged); Assert.Equal(0, result.Iterations);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Depenetrate(empty, default, 0f, maxIterations: 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Depenetrate(empty, default, 0f, tolerance: float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.Depenetrate(empty, default, 0f, maxDistance: float.PositiveInfinity));
    }

    [Fact]
    public void NonconvexSelectionDepenetrationRechecksEveryLeafAndRespectsCoordinateLimits()
    {
        var w=World(); var obstacle=w.CreateBody(BodyType.Static,default);
        w.CreateBox(obstacle,new(2,20),new(-1,0)); w.CreateBox(obstacle,new(20,2),new(0,-1));
        var shape=Shape2D.CreateCircle(default,0.5f); var position=new Vector2D(0.1,0.1);
        var worldResult=w.Depenetrate(shape,position,0f);
        var selected=obstacle.Depenetrate(shape,position,0f);
        Assert.True(worldResult.Converged); Assert.True(selected.Converged);
        Assert.Equal(worldResult.Translation,selected.Translation);
        Assert.True(selected.Iterations>=2);
        Assert.False(default(DepenetrationResult2D).Converged);
        var limitWorld=World(); limitWorld.AddBox(new(2097152,2097152),new(0,2097152),type:BodyType.Static);
        var limited=limitWorld.Depenetrate(Shape2D.CreateCircle(default,1048576),new(0,4194303),0);
        Assert.False(limited.Converged); Assert.Equal(DepenetrationStatus.CoordinateLimit,limited.Status);
        Assert.Equal(default,limited.Translation);
        obstacle.Destroy();
        Assert.Throws<StaleHandleException>(()=>obstacle.Depenetrate(shape,position,0f));
    }

    [Fact]
    public void AngularCompoundSweepVisitsOffsetLeavesAndHonorsFiltersAndJoints()
    {
        var w=World(); var source=w.CreateBody(BodyType.Dynamic,default); w.CreateCircle(source,0.1f,new(-2,0)); w.CreateCircle(source,0.1f,new(2,0)); w.CreateEmpty(source);
        var target=w.AddCircle(0.1f,new Vector2D(0,2),BodyType.Static);
        Assert.True(source.Sweep(default,1.5707964f,out var hit));
        Assert.Equal(target,hit.Body); Assert.InRange(hit.Fraction,0.93f,0.94f);
        Assert.False(source.Sweep(default,1.5707964f,out _,filter:QueryFilter2D.OnLayer(31)));
        w.AddDistance(source,target,default,new Vector2D(0,2));
        Assert.False(source.Sweep(default,1.5707964f,out _,excludeJointed:true));
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
            var pose=new Transform2(new((float)cx,(float)cy),new Rot2((float)Math.Cos(initial),(float)Math.Sin(initial)));
            bool hit=AngularCast2D.Test(Shape2D.CreateCircle(default,0.2f),Shape2D.CreateBox(1.5f,0.05f),pose,pose.P,new((float)vx,(float)vy),(float)angular,out float fraction,out _,out _);
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
        var w=World(); w.AddBox(new Vector2F(2,2),type:BodyType.Static);
        var source=w.AddCircle(0.25f,new Vector2D(0,3)); var shape=Shape2D.CreateCircle(default,0.25f);
        var motion=new Vector2F(0,-5); var angular=1f; var penetrating=new Vector2D(0,1.1);
        for(int i=0;i<10;i++) { source.Sweep(motion,angular,out _); w.Depenetrate(shape,penetrating,0f); }
        Core.AllocMeter.AssertAllocatesNothing(() => { for(int i=0;i<100;i++) { source.Sweep(motion,angular,out _); w.Depenetrate(shape,penetrating,0f); } }, "sweeps and depenetration");
    }
}
