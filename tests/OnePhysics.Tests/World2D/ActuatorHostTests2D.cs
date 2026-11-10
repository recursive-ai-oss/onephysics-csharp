using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Boats, hovercraft, thrusters and aerodynamic surfaces in the plane through the host hooks.</summary>
public class ActuatorHostTests2D
{
    private const float Dt = 1f / 60f;

    private static Body B(PhysicsWorld2D w, int slot) => w.GetBody(new BodyHandle(w.Id, slot, 1));

    private static PhysicsWorld2D World(int id = 78)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { WorldId = id });
        w.AddBuoyancyEffector(0.0, 1000f);
        w.AddWind(new Vector2F(1f, 0.5f));
        w.AddBox(new Vector2F(200f, 1f), new Vector2D(0, -3.5), type: BodyType.Static);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector2D(0, 0.2));
        w.CreateBox(hull, new Vector2F(3f, 0.6f), default, 0f, 0f, new ShapeDef { Density = 150f / 1.8f, Aero = AeroSurface.Body(0.5f) });
        var def = BoatDef.Dinghy;
        foreach (var f in def.Foils) { f.Position = new Vector2F(f.Position.X, -0.45f); }
        foreach (var p in def.Propellers) { p.Position = new Vector2F(p.Position.X, -0.45f); }
        w.AddBoat(hull, def);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector2D(40, 1));
        w.CreateBox(craft, new Vector2F(2f, 0.3f), default, 0f, 0f, new ShapeDef { Density = 100f });
        w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        var plane = w.CreateBody(BodyType.Dynamic, new Vector2D(80, 30));
        w.CreateBox(plane, new Vector2F(1f, 0.1f), default, 0f, 0f, new ShapeDef { Density = 100f, Aero = AeroSurface.Wing(0.5f) });
        plane.AddThruster(ThrusterDef.Rocket(5f, direction: new Vector2F(1f, 0f)));
        return w;
    }

    private static void Command(PhysicsWorld2D w, int i)
    {
        Assert.True(w.TryGetBoat(B(w, 1), out var boat));
        boat.Throttle = 0.8f; boat.Steer = i < 30 ? 0.2f : -0.1f;
        Assert.True(w.TryGetHovercraft(B(w, 2), out var h));
        h.Lift = 0.9f; h.Throttle = 0.4f;
        foreach (var t in w.GetThrusters()) { t.Throttle = 1f; }
    }

    [Fact]
    public void RecordedActuatorsReplayWithEveryHashEqual()
    {
        var w = World();
        using var stream = new MemoryStream();
        var expected = new List<ulong>();
        using (var recorder = w.StartRecording(stream))
        {
            for (int i = 0; i < 90; i++) { Command(w, i); w.Step(Dt); expected.Add(w.ComputeHash()); }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = World();
        int step = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(expected[step++], copy.ComputeHash()); }
        Assert.Equal(90, step);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void SceneFileDesyncRegionBoundAndShiftWorkWithTheActuators()
    {
        var w = World();
        for (int i = 0; i < 40; i++) { Command(w, i); w.Step(Dt); }
        var loaded = PhysicsWorld2D.LoadScene(new StringReader(w.SaveScene()));
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
        for (int i = 0; i < 40; i++) { Command(w, i); Command(loaded, i); w.Step(Dt); loaded.Step(Dt); }
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());

        var b = w.Fork();
        Assert.Equal(0, w.FindDifferences(b).Count);
        Assert.True(b.TryGetBoat(B(b, 1), out var boat));
        boat.Throttle = -0.3f;
        Assert.Contains(w.FindDifferences(b).Differences, d => d.Entity == "boat");

        var hull = B(w, 1).Position;
        var min = new Vector2D(hull.X - 4, hull.Y - 4); var max = new Vector2D(hull.X + 4, hull.Y + 4);
        ulong h = w.ComputeRegionHash(min, max);
        Assert.True(w.TryGetHovercraft(B(w, 2), out var hc));
        hc.Lift = 0.5f;
        Assert.Equal(h, w.ComputeRegionHash(min, max));
        Assert.True(w.TryGetBoat(B(w, 1), out var boat2));
        boat2.Throttle = 0.1f;
        Assert.NotEqual(h, w.ComputeRegionHash(min, max));
        Assert.True(w.Snapshot().Length <= w.GetSnapshotSizeBound());

        double x = B(w, 1).Position.X;
        w.ShiftOrigin(new Vector2D(16, 0));
        Assert.Equal(x - 16, B(w, 1).Position.X, 1e-9);
        for (int i = 0; i < 60; i++) { Command(w, i); w.Step(Dt); }
        Assert.InRange(B(w, 1).Position.Y, 0.1, 0.5);
    }

    [Fact]
    public void DebugGeometryHasTheSurfacesAndTheForceArrows()
    {
        var w = World();
        for (int i = 0; i < 40; i++) { Command(w, i); w.Step(Dt); }
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.Aero, g);
        Assert.NotEmpty(g.AeroSurfaces.ToArray());
        Assert.Contains(g.AeroForces.ToArray(), f => f.Kind == DebugAeroKind.Thruster);
        Assert.Contains(g.AeroForces.ToArray(), f => f.Kind == DebugAeroKind.Hull);
        Assert.Contains(g.AeroForces.ToArray(), f => f.Kind == DebugAeroKind.Fan);
        foreach (var f in g.AeroForces) { Assert.Equal(0.0, f.Point.Z); Assert.Equal(0f, f.Force.Z); }
    }
}
