using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Scene;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class LengthUnitTests
{
    [Fact]
    public void DefaultIsOneMetreAndPresetsAreExact()
    {
        Assert.True(default(LengthUnit).IsMetres);
        Assert.Equal(LengthUnit.Metres, new LengthUnit(1.0));
        Assert.Equal(0.001, LengthUnit.Millimetres.MetresPerUnit);
        Assert.Equal(0.01, LengthUnit.Centimetres.MetresPerUnit);
        Assert.Equal(1000.0, LengthUnit.Millimetres.UnitsPerMetre);
        Assert.Equal(32.0, LengthUnit.Pixels(32).UnitsPerMetre);
        Assert.Equal(100.0, LengthUnit.Pixels(100).UnitsPerMetre);
        Assert.Equal(2.5, LengthUnit.Millimetres.ToMetres(2500));
        Assert.Equal(2500.0, LengthUnit.Millimetres.FromMetres(2.5));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-1.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    [InlineData(5e-324)]
    [InlineData(1e-9)]
    [InlineData(1e9)]
    public void HostileUnitsAreRefused(double metresPerUnit)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new LengthUnit(metresPerUnit));
        Assert.False(LengthUnit.IsValid(metresPerUnit));
    }

    [Fact]
    public void DefaultsFollowTheUnitAndExplicitValuesStay()
    {
        var o3 = new P3.WorldOptions3D { LengthUnit = LengthUnit.Millimetres };
        Assert.Equal(-9.81f * 1000f, o3.Gravity.Y);
        Assert.Equal(0.05f * 1000f, o3.SleepThreshold);
        Assert.Equal(0.02f * 1000f, o3.SpeculativeDistance);
        Assert.Equal(3f * 1000f, o3.PushOutVelocity);
        Assert.Equal(400f * 1000f, o3.MaxLinearSpeed);
        Assert.Equal(0.03 * 1000f, o3.BroadphaseMargin);
        o3.Validate();

        var o2 = new P2.WorldOptions2D { SpeculativeDistance = 7f };
        o2.LengthUnit = LengthUnit.Pixels(32);
        Assert.Equal(7f, o2.SpeculativeDistance);                       // the host set it: it stays in world units
        Assert.Equal(-9.81f * 32f, o2.Gravity.Y);                       // never set: it follows the unit
        o2.LengthUnit = LengthUnit.Metres;
        Assert.Equal(-9.81f, o2.Gravity.Y);                             // and follows it back
        Assert.Equal(7f, o2.SpeculativeDistance);

        // A world at the default unit keeps every default exactly.
        var d = new P3.WorldOptions3D();
        Assert.Equal(0.02f, d.SpeculativeDistance);
        Assert.Equal(-9.81f, d.Gravity.Y);
        Assert.Equal(0.03, d.BroadphaseMargin);
    }

    [Fact]
    public void UnitOneAddsNoSectionAndOtherUnitsDo()
    {
        var plain = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 11 });
        var metres = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 11, LengthUnit = LengthUnit.Metres });
        Assert.Equal(plain.Snapshot(), metres.Snapshot());
        Assert.Equal(plain.ComputeHash(), metres.ComputeHash());
        var mm = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 11, LengthUnit = LengthUnit.Millimetres });
        Assert.True(mm.Snapshot().Length > plain.Snapshot().Length);
        Assert.NotEqual(plain.ComputeHash(), mm.ComputeHash());
        Assert.Equal(LengthUnit.Millimetres, mm.Options.LengthUnit);

        var plain2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 12 });
        var px = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 12, LengthUnit = LengthUnit.Pixels(32) });
        Assert.True(px.Snapshot().Length > plain2.Snapshot().Length);
        Assert.NotEqual(plain2.ComputeHash(), px.ComputeHash());
    }

    private static P3.PhysicsWorld3D PileOfBoxes3D(LengthUnit unit, int id, double k = 1.0)
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = id, LengthUnit = unit, Gravity = new(0f, (float)(-9.81 * k), 0f) });
        w.AddBox(new((float)(40 * k), (float)(1 * k), (float)(40 * k)), at: new(0, -0.5 * k, 0), type: BodyType.Static);
        for (int i = 0; i < 4; i++) { w.AddBox(new((float)(1 * k), (float)(1 * k), (float)(1 * k)), at: new(0.02 * k * i, (0.5 + 1.0 * i) * k, 0.0)); }
        return w;
    }

    // A scene scaled by a power of two in a world whose unit is that power of two behaves like the metre scene: the same arithmetic on exactly scaled numbers. Any metre-tuned constant that is
    // missed from the derivation shows up here as a difference far above the 2^-32 position grid.
    [Fact]
    public void ScaledWorldMatchesTheMetreWorld3D()
    {
        const double k = 1024.0;
        var a = PileOfBoxes3D(LengthUnit.Metres, 21);
        var b = PileOfBoxes3D(new LengthUnit(1.0 / k), 22, k);
        for (int s = 0; s < 120; s++) { a.Step(1f / 60); b.Step(1f / 60); }
        var ib = new System.Collections.Generic.List<P3.Body>(); foreach (var y in b.Bodies) { ib.Add(y); }
        int n = 0;
        foreach (var x in a.Bodies)
        {
            var y = ib[n++];
            Assert.InRange(Math.Abs(x.Position.X - y.Position.X / k), 0, 1e-6);
            Assert.InRange(Math.Abs(x.Position.Y - y.Position.Y / k), 0, 1e-6);
            Assert.InRange(Math.Abs(x.LinearVelocity.Y - y.LinearVelocity.Y / k), 0, 1e-4);
        }
    }

    [Fact]
    public void ScaledWorldMatchesTheMetreWorld2D()
    {
        const double k = 1024.0;
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 31 });
        var b = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 32, LengthUnit = new LengthUnit(1.0 / k), Gravity = new(0f, (float)(-9.81 * k)) });
        foreach (var (w, s) in new[] { (a, 1.0), (b, k) })
        {
            w.AddBox(new((float)(40 * s), (float)(1 * s)), at: new(0, -0.5 * s), type: BodyType.Static);
            for (int i = 0; i < 4; i++) { w.AddBox(new((float)(1 * s), (float)(1 * s)), at: new(0.02 * s * i, (0.5 + 1.0 * i) * s)); }
        }

        for (int s = 0; s < 120; s++) { a.Step(1f / 60); b.Step(1f / 60); }
        var ib = new System.Collections.Generic.List<P2.Body>(); foreach (var y in b.Bodies) { ib.Add(y); }
        int n = 0;
        foreach (var x in a.Bodies)
        {
            var y = ib[n++];
            Assert.InRange(Math.Abs(x.Position.X - y.Position.X / k), 0, 1e-6);
            Assert.InRange(Math.Abs(x.Position.Y - y.Position.Y / k), 0, 1e-6);
        }
    }

    [Fact]
    public void UnitSurvivesSnapshotRestoreForkAndSceneText()
    {
        var w = PileOfBoxes3D(LengthUnit.Millimetres, 41, 1000.0);
        for (int i = 0; i < 20; i++) { w.Step(1f / 60); }
        var bytes = w.Snapshot();

        // A world created in metres (with the creation options a restore needs: the broadphase margin) takes the snapshot's unit and then continues exactly as the original does.
        var fresh = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 41, BroadphaseMargin = w.Options.BroadphaseMargin, Gravity = w.Options.Gravity });
        Assert.True(fresh.Options.LengthUnit.IsMetres);
        fresh.Restore(bytes);
        Assert.Equal(LengthUnit.Millimetres, fresh.Options.LengthUnit);
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        for (int i = 0; i < 10; i++) { w.Step(1f / 60); fresh.Step(1f / 60); }
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());

        // Restoring a snapshot without the section puts the metre back.
        var metres = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 41 });
        var plain = metres.Snapshot();
        var back = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 41 });
        back.Options.LengthUnit = LengthUnit.Millimetres;
        back.Restore(plain);
        Assert.True(back.Options.LengthUnit.IsMetres);

        var fork = w.Fork();
        Assert.Equal(LengthUnit.Millimetres, fork.Options.LengthUnit);
        for (int i = 0; i < 10; i++) { w.Step(1f / 60); fork.Step(1f / 60); }
        Assert.Equal(w.ComputeHash(), fork.ComputeHash());

        // Exact-state scene text (version 2) carries the unit and the options derived from it.
        var text = w.SaveScene();
        var loaded = P3.PhysicsWorld3D.LoadScene(new StringReader(text));
        Assert.Equal(LengthUnit.Millimetres, loaded.Options.LengthUnit);
        Assert.Equal(w.Options.SpeculativeDistance, loaded.Options.SpeculativeDistance);
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
        for (int i = 0; i < 10; i++) { w.Step(1f / 60); loaded.Step(1f / 60); }
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
    }

    [Fact]
    public void UnitSurvivesSnapshotRestoreAndSceneText2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 51, LengthUnit = LengthUnit.Pixels(100) });
        w.AddBox(new(4000, 100), at: new(0, -50), type: BodyType.Static);
        w.AddBox(new(100, 100), at: new(0, 60));
        for (int i = 0; i < 30; i++) { w.Step(1f / 60); }
        var text = w.SaveScene();
        var loaded = P2.PhysicsWorld2D.LoadScene(new StringReader(text));
        Assert.Equal(LengthUnit.Pixels(100), loaded.Options.LengthUnit);
        Assert.Equal(w.Options.SpeculativeDistance, loaded.Options.SpeculativeDistance);
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
        for (int i = 0; i < 10; i++) { w.Step(1f / 60); loaded.Step(1f / 60); }
        Assert.Equal(w.ComputeHash(), loaded.ComputeHash());
        var fork = w.Fork();
        Assert.Equal(LengthUnit.Pixels(100), fork.Options.LengthUnit);
    }

    [Fact]
    public void ReplayCarriesTheUnit()
    {
        var w = PileOfBoxes3D(LengthUnit.Centimetres, 61, 100.0);
        using var stream = new MemoryStream();
        var hashes = new ulong[24];
        using (w.StartRecording(stream))
        {
            for (int i = 0; i < hashes.Length; i++) { w.Step(1f / 60); hashes[i] = w.ComputeHash(); }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld3D();
        Assert.Equal(LengthUnit.Centimetres, copy.Options.LengthUnit);
        int n = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(hashes[n++], copy.ComputeHash()); }
        Assert.Equal(hashes.Length, n);

        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 62, LengthUnit = LengthUnit.Millimetres });
        w2.AddBox(new(20000, 1000), at: new(0, -500), type: BodyType.Static);
        w2.AddBox(new(500, 500), at: new(0, 700));
        using var stream2 = new MemoryStream();
        var hashes2 = new ulong[24];
        using (w2.StartRecording(stream2))
        {
            for (int i = 0; i < hashes2.Length; i++) { w2.Step(1f / 60); hashes2[i] = w2.ComputeHash(); }
        }

        stream2.Position = 0;
        var player2 = new ReplayPlayer(stream2);
        var copy2 = player2.CreateWorld2D();
        Assert.Equal(LengthUnit.Millimetres, copy2.Options.LengthUnit);
        n = 0;
        while (player2.ReadNextStep(copy2)) { Assert.Equal(hashes2[n++], copy2.ComputeHash()); }
        Assert.Equal(hashes2.Length, n);
    }

    [Fact]
    public void AuthoringSceneUnitRecordRoundTripsAndOldScenesStayUnitOne()
    {
        const string text = "scene 1\nname unit\ndim 2\nunit 0.001\nmaterial 0 0.6 0\ngravity 0 -9810\nbody static 0 -500 0 0 0 0 -\nshape box 0 1 0 5000 500 0 0 0\nbody dynamic 0 600 0 0 0 0 -\nshape box 0 0.000001 0 500 500 0 0 0\n";
        var scene = Scene.Scene.Read(new StringReader(text), "unit");
        Assert.Equal(LengthUnit.Millimetres, scene.LengthUnit);
        using var writer = new StringWriter(); scene.Write(writer);
        Assert.Contains("unit 0.001\n", writer.ToString());
        var world = scene.CreateWorld2D();
        Assert.Equal(LengthUnit.Millimetres, world.Options.LengthUnit);
        Assert.Equal(-9810f, world.Options.Gravity.Y);
        P2.Body box = default; int bi = 0; foreach (var b in world.Bodies) { if (bi++ == 1) box = b; }
        for (int i = 0; i < 120; i++) { world.Step(1f / 60); }
        Assert.InRange(box.Position.Y, 499.0, 501.0);   // resting on the slab, in millimetres

        var old = Scene.Scene.Read(new StringReader("scene 1\ndim 2\nmaterial 0 0.6 0\nbody dynamic 0 1 0 0 0 0 -\nshape box 0 1 0 0.5 0.5 0 0 0\n"), "old");
        Assert.True(old.LengthUnit.IsMetres);
        using var w2 = new StringWriter(); old.Write(w2);
        Assert.DoesNotContain("unit", w2.ToString());

        Assert.Throws<SceneFormatException>(() => Scene.Scene.Read(new StringReader("scene 1\ndim 2\nunit 0\n"), "bad"));
        Assert.Throws<SceneFormatException>(() => Scene.Scene.Read(new StringReader("scene 1\ndim 2\nunit nan\n"), "bad"));
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(-0.001)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(5e-324)]
    [InlineData(1e300)]
    public void HostileUnitInASnapshotIsRefusedAndTheWorldIsUnchanged(double metresPerUnit)
    {
        var w = PileOfBoxes3D(LengthUnit.Millimetres, 71, 1000.0);
        w.Step(1f / 60);
        var bytes = w.Snapshot();
        var pattern = BitConverter.GetBytes(0.001);
        int at = -1;
        for (int i = bytes.Length - 8; i >= 0; i--)
        {
            if (bytes.AsSpan(i, 8).SequenceEqual(pattern)) { at = i; break; }
        }

        Assert.True(at > 0);
        BitConverter.GetBytes(metresPerUnit).CopyTo(bytes, at);
        ulong before = w.ComputeHash();
        Assert.Throws<SnapshotFormatException>(() => w.Restore(bytes));
        Assert.Equal(before, w.ComputeHash());
        Assert.Equal(LengthUnit.Millimetres, w.Options.LengthUnit);
    }

    [Fact]
    public void CharactersTakeTheirDefaultSizeFromTheUnit()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 81, LengthUnit = LengthUnit.Millimetres });
        w.AddBox(new(20000, 1000, 20000), at: new(0, -500, 0), type: BodyType.Static);
        var c = w.AddCharacter(new(0, 1000, 0));
        Assert.Equal(300f, c.Radius);
        Assert.Equal(1800f, c.Height);
        for (int i = 0; i < 120; i++) { c.Move(new(0, -10f, 0), 1f / 60); w.Step(1f / 60); }
        Assert.True(c.IsGrounded);
        Assert.InRange(c.Body.Position.Y, 890.0, 920.0);   // standing: half the height above the floor, give or take the skin

        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 82, LengthUnit = LengthUnit.Pixels(32) });
        var c2 = w2.AddCharacter(new(0, 40));
        Assert.Equal(0.3f * 32f, c2.Radius);
        Assert.Equal(1.8f * 32f, c2.Height);

        // A preset written in metres is converted explicitly.
        var child = P3.CharacterDef.Child.InUnit(LengthUnit.Millimetres);
        Assert.Equal(250f, child.Radius);
        Assert.Equal(1200f, child.Height);
        var c3 = w.AddCharacter(new(5000, 1000, 0), child);
        Assert.Equal(250f, c3.Radius);
    }
}
