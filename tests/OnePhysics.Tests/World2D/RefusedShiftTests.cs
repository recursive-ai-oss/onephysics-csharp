using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

// A shift that would push a body out of range is refused before anything changes: same hash, same origin, and the world still steps and answers queries.
public class RefusedShiftTests
{
    [Fact]
    public void Refused2DShiftLeavesTheWorldUnchanged()
    {
        var w = new PhysicsWorld2D();
        w.AddBox(new Vector2F(40, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var near = w.AddCircle(0.5f, at: new Vector2D(0, 2));
        w.AddCircle(0.2f, at: new Vector2D(2.0e6, 0));
        ulong h0 = w.ComputeHash();
        var e = Assert.Throws<Physics2D.BodyOutOfBoundsException>(() => w.ShiftOrigin(new Vector2D(-2.2e6, 0)));
        Assert.Contains("body", e.Message);
        Assert.Equal(h0, w.ComputeHash());
        Assert.Equal(0.0, w.Origin.X);
        for (int i = 0; i < 60; i++) { w.Step(1f / 60); }
        Assert.True(w.OverlapCircleAll(near.Position, 1, new QueryHit2D[8]) > 0);
    }

    [Fact]
    public void Refused3DShiftLeavesTheWorldUnchanged()
    {
        var w = new PhysicsWorld3D();
        w.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var near = w.AddSphere(0.5f, at: new Vector3D(0, 2, 0));
        w.AddSphere(0.2f, at: new Vector3D(2.0e6, 0, 0));
        ulong h0 = w.ComputeHash();
        var e = Assert.Throws<Physics3D.BodyOutOfBoundsException>(() => w.ShiftOrigin(new Vector3D(-2.2e6, 0, 0)));
        Assert.Contains("body", e.Message);
        Assert.Equal(h0, w.ComputeHash());
        Assert.Equal(0.0, w.Origin.X);
        for (int i = 0; i < 60; i++) { w.Step(1f / 60); }
        Assert.True(w.OverlapSphereAll(near.Position, 1, new QueryHit3D[8]) > 0);
    }
}
