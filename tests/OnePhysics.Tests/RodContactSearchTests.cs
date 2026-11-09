using Xunit;

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
}
