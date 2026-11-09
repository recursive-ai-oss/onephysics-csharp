using RecursiveAI.OneMath;
using Xunit;
using Physics3D=RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

// The rod contact search caches its candidates and no-contact proofs between passes and substeps. The cache must never change a result,
// and neither may the thread count: sweeps run in parallel and their effects are applied in rod and pair order.
public sealed class RodContactSearchTests
{
    private static ulong Run(string scene,int threads,bool fresh,int steps,int strands=1000)
    {
        var fixture=RodScene.Create(scene,threads,false,strands);
        var rods=fixture.Spatial?.Engine.Rods??fixture.Planar!.Engine.Rods;rods.FreshSearch=fresh;
        for(int step=0;step<steps;step++)fixture.Step(step);
        return fixture.Hash();
    }
    [Theory]
    [InlineData("rope-whip-3d",120,1000)]
    [InlineData("rope-2d",240,1000)]
    [InlineData("rope-bridge-3d",240,1000)]
    [InlineData("hair-3d",90,60)]
    public void CachedSearchGivesTheBitsOfARediscoveredOne(string scene,int steps,int strands)
    {
        Assert.Equal(Run(scene,1,true,steps,strands),Run(scene,1,false,steps,strands));
    }
    [Theory]
    [InlineData("rope-whip-3d",120,1000)]
    [InlineData("rope-2d",240,1000)]
    [InlineData("hair-3d",90,60)]
    public void ThreadCountDoesNotChangeTheBits(string scene,int steps,int strands)
    {
        ulong one=Run(scene,1,false,steps,strands);
        Assert.Equal(one,Run(scene,3,false,steps,strands));
        Assert.Equal(one,Run(scene,8,false,steps,strands));
    }
    // A light sphere that the rope pushes and a box that lands beside it: bodies the solve moves, so the cache is checked against pushed poses too.
    [Fact]
    public void PushedBodiesDoNotBreakTheCache()
    {
        ulong Hash(bool fresh,int threads,out int gathers)
        {
            var world=new Physics3D.PhysicsWorld3D(new(){WorldId=811,ThreadCount=threads,SleepEnabled=false});
            world.AddBox(new(10,1,10),at:new(0,-.5,0),type:BodyType.Static);
            var points=new Vector3D[21];for(int i=0;i<=20;i++)points[i]=new(-1+.1*i,.6,0);
            var rope=world.AddRod(new(){Points=points,PinnedParticles=[0,20],Substeps=4,Iterations=3});
            world.AddSphere(.12f,at:new(.05,1.4,.02),density:.3f);world.AddBox(new(.2f,.2f,.2f),at:new(-.45,1.1,0),density:.5f);
            world.Engine.Rods.FreshSearch=fresh;
            for(int step=0;step<240;step++)world.Step(1f/60);
            gathers=world.Engine.Rods.GatherCount;return world.ComputeHash();
        }
        ulong cached=Hash(false,1,out int cachedGathers);
        Assert.Equal(Hash(true,1,out _),cached);
        Assert.Equal(cached,Hash(false,5,out _));
        Assert.True(cachedGathers>=240*4,$"{cachedGathers} gathers");
    }
}
