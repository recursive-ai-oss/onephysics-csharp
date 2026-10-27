using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

public class GravityTests
{
    [Fact]
    public void StandardGravityHasExpectedStrength()
    {
        Assert.Equal(9.81f, Gravity.Strength(Gravity.Standard));
    }

    [Fact]
    public void StrengthOfThreeFourFiveIsFive()
    {
        Assert.Equal(5f, Gravity.Strength(new Vector3F(3f, 4f, 0f)));
    }
}
