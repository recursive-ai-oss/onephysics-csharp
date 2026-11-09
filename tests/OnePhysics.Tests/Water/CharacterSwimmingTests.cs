using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;
using Def2 = RecursiveAI.OnePhysics.Physics2D.CharacterDef;
using Def3 = RecursiveAI.OnePhysics.Physics3D.CharacterDef;

namespace RecursiveAI.OnePhysics.Tests.Water;

/// <summary>Swimming: wading, floating at the waterline a density gives, terminal speeds from the drag law, drifting with a current, events and snapshots (both dimensions).</summary>
public class CharacterSwimmingTests
{
    private const float Dt = 1f / 60f;
    private const float G = 9.81f;

    private static PhysicsWorld3D World3(int id = 331) => new(new WorldOptions3D { ThreadCount = 1, WorldId = id });

    private static PhysicsWorld2D World2(int id = 332) => new(new WorldOptions2D { ThreadCount = 1, WorldId = id });

    // Fraction of an upright capsule under a surface `depth` above its lowest point, by numeric integration of its cross-sections (3D: discs; 2D: chords), independent of the engine's closed form.
    private static double Wet(bool planar, double r, double h, double depth)
    {
        int n = 200000;
        double total = 0, wet = 0, l = h - 2 * r;
        for (int i = 0; i < n; i++)
        {
            double y = (i + 0.5) / n * h;
            double rho = y < r ? Math.Sqrt(Math.Max(r * r - (r - y) * (r - y), 0)) : y > r + l ? Math.Sqrt(Math.Max(r * r - (y - r - l) * (y - r - l), 0)) : r;
            double w = planar ? 2 * rho : Math.PI * rho * rho;
            total += w;
            if (y < depth) { wet += w; }
        }

        return wet / total;
    }

    private static double EquilibriumDepth(bool planar, double r, double h, double ratio)
    {
        double lo = 0, hi = h;
        for (int i = 0; i < 60; i++)
        {
            double mid = 0.5 * (lo + hi);
            if (Wet(planar, r, h, mid) < ratio) { lo = mid; } else { hi = mid; }
        }

        return 0.5 * (lo + hi);
    }

    [Theory]
    [InlineData(700f)]
    [InlineData(300f)]
    public void ASwimmerFloatsAtTheWaterlineItsDensityGives3D(float density)
    {
        var w = World3();
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector3D(0, -0.4, 0), new Def3 { SwimDensity = density });
        for (int i = 0; i < 900; i++) { c.Move(default, Dt); w.Step(Dt); }
        double depth = EquilibriumDepth(false, 0.3, 1.8, density / 1000.0);
        double expected = -depth + 0.9;
        Assert.InRange(c.Position.Y, expected - 0.02, expected + 0.02);
        Assert.Equal(CharacterWaterState.Swimming, c.WaterState);
    }

    [Fact]
    public void ASwimmerFloatsAtTheWaterlineItsDensityGives2D()
    {
        var w = World2();
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector2D(0, -0.4), new Def2 { SwimDensity = 600f });
        for (int i = 0; i < 900; i++) { c.Move(default, Dt); w.Step(Dt); }
        double depth = EquilibriumDepth(true, 0.3, 1.8, 0.6);
        Assert.InRange(c.Position.Y, -depth + 0.9 - 0.02, -depth + 0.9 + 0.02);
    }

    [Fact]
    public void AHeavySwimmerSinksAtTheTerminalSpeedOfTheDragLaw()
    {
        var w = World3();
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector3D(0, -3.0, 0), new Def3 { SwimDensity = 1500f, SwimDamping = 4f });
        for (int i = 0; i < 150; i++) { c.Move(default, Dt); w.Step(Dt); }
        // dv/dt = g (rho_w / rho_c - 1) - k v with the body fully under: v = g (2/3 - 1) / k.
        float expected = G * (1000f / 1500f - 1f) / 4f;
        Assert.InRange(c.Velocity.Y, expected * 1.05f, expected * 0.95f);
    }

    [Fact]
    public void ALightSwimmerRisesAtTheTerminalSpeedOfTheDragLaw2D()
    {
        var w = World2();
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector2D(0, -12.0), new Def2 { SwimDensity = 500f, SwimDamping = 4f });
        for (int i = 0; i < 120; i++) { c.Move(default, Dt); w.Step(Dt); }
        float expected = G * (1000f / 500f - 1f) / 4f;
        Assert.InRange(c.Velocity.Y, expected * 0.95f, expected * 1.05f);
    }

    [Fact]
    public void AnIdleSwimmerDriftsWithTheCurrent()
    {
        var w = World3();
        w.AddWater(0.0, flow: new Vector3F(1.2f, 0f, -0.5f));
        var c = w.AddCharacter(new Vector3D(0, -3.0, 0), new Def3 { SwimDensity = 1000f });
        for (int i = 0; i < 300; i++) { c.Move(default, Dt); w.Step(Dt); }
        Assert.InRange(c.Velocity.X, 1.2f * 0.95f, 1.2f * 1.05f);
        Assert.InRange(c.Velocity.Z, -0.5f * 1.05f, -0.5f * 0.95f);
        Assert.InRange(c.Velocity.Y, -0.05f, 0.05f);
    }

    [Fact]
    public void AStrokeAddsToTheCurrent()
    {
        var w = World2();
        w.AddWater(0.0, flow: new Vector2F(1f, 0f));
        var c = w.AddCharacter(new Vector2D(0, -3.0), new Def2 { SwimDensity = 1000f });
        for (int i = 0; i < 300; i++) { c.Move(new Vector2F(1.5f * Dt, 0f), Dt); w.Step(Dt); }
        Assert.InRange(c.Velocity.X, 2.5f * 0.97f, 2.5f * 1.03f);
    }

    [Fact]
    public void WadingSlowsWalkingAndSwimmingReplacesIt()
    {
        var w = World3();
        w.AddBox(new Vector3F(100f, 1f, 100f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddWater(0.6);
        var c = w.AddCharacter(new Vector3D(0, 0.91, 0));
        float moved = 0f;
        for (int i = 0; i < 30; i++)
        {
            double x0 = c.Position.X;
            c.Move(new Vector3F(2f * Dt, -0.05f, 0f), Dt);
            w.Step(Dt);
            moved = (float)(c.Position.X - x0);
        }

        Assert.Equal(CharacterWaterState.Wading, c.WaterState);
        Assert.InRange(c.Immersion, 0.3f, 0.36f);
        // Walking at 2 m/s over 0.33 immersion: 1 - 0.5 * 0.33 / 0.75 = 0.78 of the speed.
        Assert.InRange(moved / Dt, 1.45f, 1.7f);
    }

    [Fact]
    public void EventsReportEnteringAndSwimmingInOrderAfterTheStep()
    {
        var w = World3();
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector3D(0, 3.0, 0));
        var seen = new System.Collections.Generic.List<CharacterWaterEventKind>();
        w.CharacterWaterChanged += e => { Assert.Equal(c, e.Character); seen.Add(e.Kind); };
        float vy = 0f;
        for (int i = 0; i < 200; i++)
        {
            vy = c.IsSwimming ? 0f : vy - G * Dt;
            c.Move(new Vector3F(0f, vy * Dt, 0f), Dt);
            w.Step(Dt);
        }

        Assert.Contains(CharacterWaterEventKind.EnteredWater, seen);
        Assert.True(seen.IndexOf(CharacterWaterEventKind.EnteredWater) < seen.IndexOf(CharacterWaterEventKind.StartedSwimming));
        Assert.Equal(CharacterWaterState.Swimming, c.WaterState);
    }

    [Fact]
    public void ACharacterWhoCannotSwimIsLeftAlone()
    {
        var w = World3();
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector3D(0, -1.0, 0), new Def3 { CanSwim = false });
        for (int i = 0; i < 60; i++) { c.Move(new Vector3F(0f, -0.1f * Dt, 0f), Dt); w.Step(Dt); }
        Assert.Equal(CharacterWaterState.Dry, c.WaterState);
        Assert.InRange(c.Velocity.Y, -0.11f, -0.09f);
    }

    [Fact]
    public void SwimStateSurvivesSnapshotForkAndReplayHashes()
    {
        var w = World3(341);
        w.AddWater(0.0, flow: new Vector3F(0.5f, 0f, 0f));
        var a = w.AddCharacter(new Vector3D(0, -1.0, 0), new Def3 { SwimDensity = 800f });
        var b = w.AddCharacter(new Vector3D(3, 0.6, 0));
        for (int i = 0; i < 90; i++) { a.Move(new Vector3F(0.5f * Dt, 0f, 0f), Dt); b.Move(new Vector3F(0f, -0.2f * Dt, 0f), Dt); w.Step(Dt); }
        var snap = w.Snapshot();
        var fork = w.Fork();
        var hashes = new ulong[60];
        for (int i = 0; i < 60; i++) { a.Move(new Vector3F(0f, 0.3f * Dt, 0f), Dt); b.Move(new Vector3F(0f, -0.2f * Dt, 0f), Dt); w.Step(Dt); hashes[i] = w.ComputeHash(); }
        var fa = fork.GetCharacter(a.RawHandle); var fb = fork.GetCharacter(b.RawHandle);
        for (int i = 0; i < 60; i++) { fa.Move(new Vector3F(0f, 0.3f * Dt, 0f), Dt); fb.Move(new Vector3F(0f, -0.2f * Dt, 0f), Dt); fork.Step(Dt); Assert.Equal(hashes[i], fork.ComputeHash()); }
        w.Restore(snap);
        for (int i = 0; i < 60; i++) { a.Move(new Vector3F(0f, 0.3f * Dt, 0f), Dt); b.Move(new Vector3F(0f, -0.2f * Dt, 0f), Dt); w.Step(Dt); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.NotEmpty(snap);
    }

    [Fact]
    public void AWorldWithoutSwimmersKeepsItsSnapshotBytes()
    {
        var w = World3(342);
        w.AddWater(0.0);
        var c = w.AddCharacter(new Vector3D(0, 5.0, 0));
        w.Step(Dt);
        var before = w.Snapshot();
        c.Move(new Vector3F(0f, -0.01f, 0f), Dt);
        w.Step(Dt);
        var fresh = World3(342);
        fresh.AddWater(0.0);
        fresh.AddCharacter(new Vector3D(0, 5.0, 0));
        fresh.Step(Dt);
        Assert.Equal(before, fresh.Snapshot());
    }
}
