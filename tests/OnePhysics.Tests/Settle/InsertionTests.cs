using System;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Settle;

public sealed class InsertionTests
{
    // A square hole 0.22 m wide and 0.5 m deep in a block; a 0.2 m peg 0.6 m above its floor gives 0.01 m of clearance on every side.
    private static (P3.PhysicsWorld3D World, P3.Body Floor) Hole3()
    {
        var w = new P3.PhysicsWorld3D();
        var floor = w.AddBox(new(2, 1, 2), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddBox(new(0.2f, 0.5f, 0.62f), new Vector3D(0.21, 0.25, 0), type: BodyType.Static);
        w.AddBox(new(0.2f, 0.5f, 0.62f), new Vector3D(-0.21, 0.25, 0), type: BodyType.Static);
        w.AddBox(new(0.22f, 0.5f, 0.2f), new Vector3D(0, 0.25, 0.21), type: BodyType.Static);
        w.AddBox(new(0.22f, 0.5f, 0.2f), new Vector3D(0, 0.25, -0.21), type: BodyType.Static);
        return (w, floor);
    }

    private static (P2.PhysicsWorld2D World, P2.Body Floor) Slot2()
    {
        var w = new P2.PhysicsWorld2D();
        var floor = w.AddBox(new(2, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        w.AddBox(new(0.2f, 0.5f), new Vector2D(0.21, 0.25), type: BodyType.Static);
        w.AddBox(new(0.2f, 0.5f), new Vector2D(-0.21, 0.25), type: BodyType.Static);
        return (w, floor);
    }

    [Fact]
    public void APegSeatsInAHoleAndReportsItsClearance()
    {
        var (w, floor) = Hole3();
        var peg = w.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0, 0.8, 0));
        var r = peg.CheckInsertion(new Vector3F(0, -1, 0), 0.6f);
        Assert.True(r.Seats);
        Assert.False(r.IsBlocked);
        Assert.InRange(r.MinClearance, 0.0099f, 0.0101f);
        Assert.True(r.MinClearanceDistance >= 0.1f - 1e-3f, r.MinClearanceDistance.ToString());
        Assert.Equal(17, r.ClearanceProfile.Length);
        Assert.Equal(0.05f, r.ClearanceProfile[0]);
        Assert.Equal(0.6f, r.TravelDistance, 3);
        // Beyond the floor it is blocked, and the first contact is the floor at 0.6 m.
        var deep = peg.CheckInsertion(new Vector3F(0, -1, 0), 0.7f);
        Assert.False(deep.Seats);
        Assert.True(deep.IsBlocked);
        Assert.InRange(deep.FirstContactDistance, 0.6f - 5e-4f, 0.6f + 5e-4f);
        Assert.Equal(floor.RawHandle, deep.Blocker);
        Assert.InRange(deep.FirstContactNormal.Y, 0.99f, 1.01f);
        Assert.InRange(deep.FirstContactPoint.Y, -1e-3, 1e-3);
        // The check moved nothing.
        Assert.Equal(0.8, peg.Position.Y, 6);
    }

    [Fact]
    public void APegThatIsTooWideOrOffsetStopsAtTheRimWithTheRightDepth()
    {
        var (w, _) = Hole3();
        var offset = w.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0.02, 0.8, 0));
        var r = offset.CheckInsertion(new Vector3F(0, -1, 0), 0.6f);
        Assert.False(r.Seats);
        Assert.True(r.IsBlocked);
        Assert.InRange(r.FirstContactDistance, 0.1f - 5e-4f, 0.1f + 5e-4f);
        var (w2, _) = Hole3();
        var fat = w2.AddBox(new(0.24f, 0.4f, 0.24f), new Vector3D(0, 0.8, 0));
        var f = fat.CheckInsertion(new Vector3F(0, -1, 0), 0.6f);
        Assert.True(f.IsBlocked);
        Assert.InRange(f.FirstContactDistance, 0.1f - 5e-4f, 0.1f + 5e-4f);
        var (w3, _) = Hole3();
        var tight = w3.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0, 0.8, 0));
        Assert.False(tight.CheckInsertion(new Vector3F(0, -1, 0), 0.6f, new P3.InsertionOptions3D { RequiredClearance = 0.02f }).Seats);
    }

    [Fact]
    public void ASetOfBodiesMovesTogetherAndIgnoresItself()
    {
        var (w, _) = Hole3();
        var peg = w.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0, 0.8, 0));
        var flange = w.AddBox(new(0.8f, 0.05f, 0.8f), new Vector3D(0, 1.025, 0));
        var r = w.CheckInsertion([peg, flange], new Vector3F(0, -1, 0), 0.6f);
        Assert.True(r.IsBlocked);
        // The flange's underside (y = 1.0) meets the rim (y = 0.5): half a metre of travel.
        Assert.InRange(r.FirstContactDistance, 0.5f - 5e-4f, 0.5f + 5e-4f);
        // The peg and the flange touch each other; that contact must not read as zero clearance at the start.
        Assert.Equal(0.05f, r.ClearanceProfile[0]);
        Assert.Throws<ArgumentException>(() => w.CheckInsertion([], new Vector3F(0, -1, 0), 1f));
    }

    [Fact]
    public void AnAngledPathStillFindsTheSameClearance()
    {
        var (w, _) = Hole3();
        var peg = w.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0, 0.8, 0));
        var r = peg.CheckInsertion(new Vector3F(0, -2, 0), 0.6f, new P3.InsertionOptions3D { Stations = 4, ProbeDirections = 16 });
        Assert.True(r.Seats);
        Assert.InRange(r.MinClearance, 0.0099f, 0.0101f);
        Assert.Equal(new Vector3F(0, -1, 0), r.Axis);
    }

    [Fact]
    public void ASlotInTwoDimensionsReportsTheSameThings()
    {
        var (w, floor) = Slot2();
        var peg = w.AddBox(new(0.2f, 0.4f), new Vector2D(0, 0.8));
        var r = peg.CheckInsertion(new Vector2F(0, -1), 0.6f);
        Assert.True(r.Seats);
        Assert.InRange(r.MinClearance, 0.0099f, 0.0101f);
        Assert.Equal(0.05f, r.ClearanceProfile[0]);
        var deep = peg.CheckInsertion(new Vector2F(0, -1), 0.7f);
        Assert.True(deep.IsBlocked);
        Assert.InRange(deep.FirstContactDistance, 0.6f - 5e-4f, 0.6f + 5e-4f);
        Assert.Equal(floor.RawHandle, deep.Blocker);
        var (w2, _) = Slot2();
        var offset = w2.AddBox(new(0.2f, 0.4f), new Vector2D(0.02, 0.8));
        var o = offset.CheckInsertion(new Vector2F(0, -1), 0.6f);
        Assert.True(o.IsBlocked);
        Assert.InRange(o.FirstContactDistance, 0.1f - 5e-4f, 0.1f + 5e-4f);
    }

    [Fact]
    public void BadInputIsRejectedByName()
    {
        var (w, _) = Hole3();
        var peg = w.AddBox(new(0.2f, 0.4f, 0.2f), new Vector3D(0, 0.8, 0));
        Assert.Equal("axis", Assert.Throws<ArgumentOutOfRangeException>(() => peg.CheckInsertion(default, 1f)).ParamName);
        Assert.Equal("distance", Assert.Throws<ArgumentOutOfRangeException>(() => peg.CheckInsertion(new Vector3F(0, 1, 0), -1f)).ParamName);
        Assert.Equal("Stations", Assert.Throws<ArgumentOutOfRangeException>(() => peg.CheckInsertion(new Vector3F(0, 1, 0), 1f, new P3.InsertionOptions3D { Stations = 0 })).ParamName);
        Assert.Throws<ArgumentException>(() => w.CheckInsertion([w.AddBox(new(1, 1, 1), type: BodyType.Static)], new Vector3F(0, 1, 0), 1f));
        peg.Destroy();
        Assert.Throws<StaleHandleException>(() => peg.CheckInsertion(new Vector3F(0, 1, 0), 1f));
    }
}
