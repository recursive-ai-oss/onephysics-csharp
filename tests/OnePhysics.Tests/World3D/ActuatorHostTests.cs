#pragma warning disable OPHYS0002, OPHYS0003
using System;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

/// <summary>Boats, hovercraft, thrusters and aerodynamic surfaces through the host hooks: recording and replay, scene files, desync labels, regions, snapshot bounds, origin shift, handles and the debug geometry.</summary>
public class ActuatorHostTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D World(int id = 77)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = id });
        w.AddWater(0.0, 1000f, default, 0f);
        w.AddWind(new Vector3F(1f, 0f, 2f));
        w.AddBox(new Vector3F(200f, 1f, 200f), new Vector3D(0, -3.5, 0), type: BodyType.Static);
        var hull = w.CreateBody(BodyType.Dynamic, new Vector3D(0, 0.2, 0));
        w.CreateBox(hull, new Vector3F(1.2f, 0.6f, 3f), 0f, new ShapeDef { Density = 150f / (1.2f * 0.6f * 3f), Aero = AeroSurface.Body(0.5f) });
        var def = BoatDef.Dinghy;
        foreach (var f in def.Foils) { f.Position = new Vector3F(f.Position.X, -0.45f, f.Position.Z); }
        foreach (var p in def.Propellers) { p.Position = new Vector3F(p.Position.X, -0.45f, p.Position.Z); }
        w.AddBoat(hull, def);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector3D(40, 1, 0));
        w.CreateBox(craft, new Vector3F(2f, 0.3f, 3f), 0f, new ShapeDef { Density = 100f });
        w.AddHovercraft(craft, HovercraftDef.ForMass(craft.Mass));
        var plane = w.CreateBody(BodyType.Dynamic, new Vector3D(80, 30, 0));
        w.CreateBox(plane, new Vector3F(0.2f, 0.1f, 1f), 0f, new ShapeDef { Density = 100f, Aero = AeroSurface.Wing(0.5f) });
        plane.AddThruster(ThrusterDef.Rocket(5f, direction: new Vector3F(0f, 0f, 1f)));
        return w;
    }

    private static void Command(PhysicsWorld3D w, int i)
    {
        TryBoat(w, 0, out var boat);
        boat.Throttle = 0.8f; boat.Steer = i < 30 ? 0.2f : -0.1f;
        w.TryGetHovercraft(B(w, 2), out var h);
        h.Lift = 0.9f; h.Throttle = 0.4f;
        foreach (var t in w.GetThrusters()) { t.Throttle = 1f; }
    }

    private static Body B(PhysicsWorld3D w, int slot) => w.GetBody(new BodyHandle(w.Id, slot, 1));

    private static void TryBoat(PhysicsWorld3D w, int unused, out Boat boat) { _ = unused; Assert.True(w.TryGetBoat(B(w, 1), out boat)); }

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
    public void SceneFileKeepsTheActuators()
    {
        var w = World();
        for (int i = 0; i < 40; i++) { Command(w, i); w.Step(Dt); }
        string text = w.SaveScene();
        var loaded = PhysicsWorld3D.LoadScene(new StringReader(text));
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
        Assert.Single(loaded.GetThrusters());
        for (int i = 0; i < 40; i++) { Command(w, i); Command(loaded, i); w.Step(Dt); loaded.Step(Dt); }
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
    }

    [Fact]
    public void DesyncFinderNamesTheActuatorField()
    {
        var a = World();
        for (int i = 0; i < 20; i++) { Command(a, i); a.Step(Dt); }
        var b = a.Fork();
        Assert.Equal(0, a.FindDifferences(b).Count);
        TryBoat(b, 0, out var boat);
        boat.Throttle = -0.3f;
        var report = a.FindDifferences(b);
        Assert.True(report.Count > 0);
        Assert.Contains(report.Differences, d => d.Entity == "boat");
        TryBoat(b, 0, out boat);
        boat.Throttle = 0.8f;
        Assert.Equal(0, a.FindDifferences(b).Count);
        b.GetThrusters()[0].Throttle = 0.25f;
        Assert.Contains(a.FindDifferences(b).Differences, d => d.Entity == "thruster");
    }

    [Fact]
    public void RegionTakesTheActuatorsOfItsBodiesAndNoOthers()
    {
        var w = World();
        for (int i = 0; i < 30; i++) { Command(w, i); w.Step(Dt); }
        var hull = B(w, 1).Position;
        var min = new Vector3D(hull.X - 4, hull.Y - 4, hull.Z - 4); var max = new Vector3D(hull.X + 4, hull.Y + 4, hull.Z + 4);
        ulong h = w.ComputeRegionHash(min, max);
        // The hovercraft is 40 m away: changing its lift leaves the boat's region hash alone; changing the boat's throttle moves it.
        w.TryGetHovercraft(B(w, 2), out var hc);
        hc.Lift = 0.5f;
        Assert.Equal(h, w.ComputeRegionHash(min, max));
        TryBoat(w, 0, out var boat);
        boat.Throttle = 0.1f;
        Assert.NotEqual(h, w.ComputeRegionHash(min, max));
        var archive = w.CaptureRegion(min, max);
        Assert.Contains(archive.Fields, f => f.Entity == "boat");
        Assert.DoesNotContain(archive.Fields, f => f.Entity == "hovercraft");
    }

    [Fact]
    public void SnapshotBoundCoversTheActuators()
    {
        var w = World();
        for (int i = 0; i < 20; i++) { Command(w, i); w.Step(Dt); }
        Assert.True(w.Snapshot().Length <= w.GetSnapshotSizeBound());
    }

    [Fact]
    public void OriginShiftCarriesTheBoatAndItsWaterAlong()
    {
        var w = World();
        for (int i = 0; i < 60; i++) { Command(w, i); w.Step(Dt); }
        var hull = B(w, 1);
        var p = hull.Position;
        w.ShiftOrigin(new Vector3D(16, 0, -8));
        Assert.Equal(p.X - 16, hull.Position.X, 1e-9);
        for (int i = 0; i < 60; i++) { Command(w, i); w.Step(Dt); }
        // Still afloat at the draft: no drop into the void, no jump out of the water.
        Assert.InRange(hull.Position.Y, 0.15, 0.4);
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
        Assert.Contains(g.AeroSurfaces.ToArray(), s => s.Kind == DebugAeroKind.Foil && s.IsLifting);
        int n = g.AeroForces.Length;
        g.Clear();
        Assert.Equal(0, g.AeroForces.Length);
        w.GetDebugGeometry(DebugContent.All, g);
        Assert.Equal(n, g.AeroForces.Length);
    }
}
