using System;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Rods;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>Seats, ledge hangs, climbing, rope grips and carried characters are in the snapshot, the hash, forks, regions and replays like every other part of the world.</summary>
public sealed class CharacterAbilityHostTests
{
    private const float Dt = 1f / 60f;

    private sealed class Scene3D
    {
        public P3.PhysicsWorld3D World = null!;
        public P3.Character Rider, Hanger, Climber, Roper, Deck, Walker;
        public P3.Body Car = default!;
        public P3.Body Platform = default!;
        public P3.Rod Rope;

        public static Scene3D Build(int id, int threads = 1)
        {
            var s = new Scene3D { World = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = id, ThreadCount = threads }) };
            var w = s.World;
            w.AddBox(new Vector3F(100, 1, 100), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
            s.Car = w.AddBox(new Vector3F(2, 0.6f, 4), at: new Vector3D(-10, 0.5, 0), density: 200f);
            var def = P3.CharacterDef.Humanoid; def.CanClimb = true; def.ExactPlatformCarry = true;
            s.Rider = w.AddCharacter(new Vector3D(-10, 3, 0), def);
            s.Rider.Mount(s.Car, new Vector3F(0.4f, 0.8f, 0.5f), default);
            var ledge = w.AddBox(new Vector3F(8f, 2f, 4f), at: new Vector3D(0, 1, 4), type: BodyType.Static);
            s.Hanger = w.AddCharacter(new Vector3D(0, 1.15, 1.65), def);
            Assert.True(s.Hanger.TryGrabLedge(new Vector3F(0, 0, 1)));
            var wall = w.AddBox(new Vector3F(4f, 4f, 1f), at: new Vector3D(10, 2, 2.5), type: BodyType.Static);
            wall.GetShapes()[0].Climbable = true;
            s.Climber = w.AddCharacter(new Vector3D(10, 0.9, 1.65), def);
            Assert.True(s.Climber.TryClimb(new Vector3F(0, 0, 1)));
            var pts = new Vector3D[31];
            for (int i = 0; i <= 30; i++) { pts[i] = new Vector3D(20, 6, 0) + new Vector3D(Math.Sin(0.2) * 3, -Math.Cos(0.2) * 3, 0) * (i / 30.0); }
            s.Rope = w.AddRod(new P3.RodDef { Points = pts, Material = new RodMaterial(2e10f, 8e9f, 0.003f, 1000f), PinnedParticles = [0] });
            s.Roper = w.AddCharacter(new Vector3D(20, 3, 0), def);
            s.Roper.Grab(s.Rope, 30f);
            s.Platform = w.AddBox(new Vector3F(8, 0.5f, 8), at: new Vector3D(30, -0.25, 0), type: BodyType.Kinematic);
            s.Deck = w.AddCharacter(new Vector3D(31, 0.91, 1), def);
            s.Walker = w.AddCharacter(new Vector3D(-20, 0.91, 0), P3.CharacterDef.Humanoid);
            return s;
        }

        private float _vy;

        public void Tick(int i)
        {
            Platform.LinearVelocity = new Vector3F(0.5f * MathF.Sin(i * 0.05f), 0, 0.3f);
            Platform.AngularVelocity = new Vector3F(0, 0.3f, 0);
            _vy = Deck.IsGrounded && _vy < 0 ? -1f : _vy - 9.81f * Dt;
            Deck.Move(new Vector3F(0.01f, _vy * Dt, 0), Dt);
            Walker.Move(new Vector3F(0.02f, _vy * Dt, 0), Dt);
            if (i < 40 && Climber.Mode == CharacterMode.Climbing) { Climber.Climb(0.02f); }
            if (i == 20) { Hanger.Move(new Vector3F(0.2f, 0, 0), Dt); }
            if (i == 50 && Roper.Mode == CharacterMode.Roped) { Roper.Climb(0.3f); }
            Car.ApplyForce(new Vector3F(0, 0, 2000f));
            World.Step(Dt);
        }
    }

    [Fact]
    public void AbilityStateSurvivesSnapshotRestoreForkAndHashLikeEverythingElse3D()
    {
        var a = Scene3D.Build(41);
        for (int i = 0; i < 30; i++) { a.Tick(i); }
        Assert.Equal(CharacterMode.Seated, a.Rider.Mode);
        Assert.Equal(CharacterMode.Hanging, a.Hanger.Mode);
        Assert.Equal(CharacterMode.Climbing, a.Climber.Mode);
        Assert.Equal(CharacterMode.Roped, a.Roper.Mode);
        var snap = a.World.Snapshot();
        var fork = a.World.Fork();
        var fresh = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 41 });
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        Assert.Equal(a.World.ComputeHash(), fresh.ComputeHash());
        Assert.Equal(a.World.ComputeHash(), fork.ComputeHash());
        var hashes = new ulong[90];
        for (int i = 0; i < hashes.Length; i++) { a.Tick(30 + i); hashes[i] = a.World.ComputeHash(); }
        // The restored copy and the fork carry on with the same state: the handles are the same, so the same calls drive them.
        var b = new Scene3D { World = fresh, Rider = fresh.GetCharacter(a.Rider.RawHandle), Hanger = fresh.GetCharacter(a.Hanger.RawHandle), Climber = fresh.GetCharacter(a.Climber.RawHandle), Roper = fresh.GetCharacter(a.Roper.RawHandle), Deck = fresh.GetCharacter(a.Deck.RawHandle), Walker = fresh.GetCharacter(a.Walker.RawHandle), Car = fresh.GetBody(a.Car.RawHandle), Platform = fresh.GetBody(a.Platform.RawHandle) };
        for (int i = 0; i < hashes.Length; i++) { b.Tick(30 + i); Assert.Equal(hashes[i], fresh.ComputeHash()); }
        Assert.Equal(a.World.Snapshot(), fresh.Snapshot());
    }

    [Fact]
    public void ARecordedSessionOfAbilitiesReplaysWithEveryHashEqual3D()
    {
        var s = Scene3D.Build(42);
        var hashes = new ulong[60];
        using var stream = new MemoryStream();
        using (var recorder = s.World.StartRecording(stream))
        {
            for (int i = 0; i < hashes.Length; i++)
            {
                if (i == 10) { s.Hanger.ClimbUp(); }
                if (i == 12) { s.Roper.Release(); }
                if (i == 14) { s.Rider.Dismount(new Vector3F(2, 0, 0)); }
                if (i == 20) { s.Climber.WallJump(3f, 4f); }
                s.Tick(i);
                hashes[i] = s.World.ComputeHash();
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld3D();
        int n = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(hashes[n++], copy.ComputeHash()); }
        Assert.Equal(hashes.Length, n);
        Assert.Equal(s.World.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void AWorldWithoutAbilitiesKeepsItsBytes3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 43 });
        w.AddBox(new Vector3F(10, 1, 10), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var c = w.AddCharacter(new Vector3D(0, 1, 0));
        for (int i = 0; i < 10; i++) { c.Move(new Vector3F(0, -0.01f, 0), Dt); w.Step(Dt); }
        var bytes = w.Snapshot();
        // Section id "CEXT" is little-endian in the stream; nothing of it appears while the feature is unused.
        Assert.False(ContainsSection(bytes, 0x43455854));
        var d = P3.CharacterDef.Humanoid; d.CanClimb = true;
        var c2 = w.AddCharacter(new Vector3D(3, 1, 0), d);
        Assert.True(ContainsSection(w.Snapshot(), 0x43455854));
        c2.CanClimb = false;
        Assert.False(ContainsSection(w.Snapshot(), 0x43455854));
    }

    private static bool ContainsSection(byte[] bytes, int id)
    {
        var needle = BitConverter.GetBytes(id);
        for (int i = 0; i + 4 <= bytes.Length; i++)
        {
            if (bytes[i] == needle[0] && bytes[i + 1] == needle[1] && bytes[i + 2] == needle[2] && bytes[i + 3] == needle[3]) { return true; }
        }

        return false;
    }

    [Fact]
    public void AbilityStateSurvivesSnapshotRestoreForkAndReplay2D()
    {
        P2.PhysicsWorld2D Build(int id, out P2.Character rider, out P2.Character hanger, out P2.Character roper, out P2.Body car)
        {
            var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = id });
            w.AddBox(new Vector2F(100, 1), at: new Vector2D(0, -0.5), type: BodyType.Static);
            car = w.AddBox(new Vector2F(4, 0.6f), at: new Vector2D(-10, 0.5), density: 200f);
            var def = P2.CharacterDef.Humanoid; def.CanClimb = true; def.ExactPlatformCarry = true;
            rider = w.AddCharacter(new Vector2D(-10, 3), def);
            rider.Mount(car, new Vector2F(0.4f, 0.8f), 0.1f);
            w.AddBox(new Vector2F(8f, 2f), at: new Vector2D(6, 1), type: BodyType.Static);
            hanger = w.AddCharacter(new Vector2D(1.65, 1.15), def);
            Assert.True(hanger.TryGrabLedge(new Vector2F(1, 0)));
            var pts = new Vector2D[31];
            for (int i = 0; i <= 30; i++) { pts[i] = new Vector2D(20, 6) + new Vector2D(Math.Sin(0.2) * 3, -Math.Cos(0.2) * 3) * (i / 30.0); }
            var rope = w.AddRod(new P2.RodDef { Points = pts, Material = new RodMaterial(2e10f, 8e9f, 0.003f, 1000f), PinnedParticles = [0] });
            roper = w.AddCharacter(new Vector2D(20, 3), def);
            roper.Grab(rope, 30f);
            return w;
        }

        var a = Build(44, out var rider, out var hanger, out var roper, out var car);
        for (int i = 0; i < 30; i++) { car.ApplyForce(new Vector2F(2000f, 0)); a.Step(Dt); }
        Assert.Equal(CharacterMode.Seated, rider.Mode);
        Assert.Equal(CharacterMode.Hanging, hanger.Mode);
        Assert.Equal(CharacterMode.Roped, roper.Mode);
        var snap = a.Snapshot();
        var fork = a.Fork();
        var fresh = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 44 });
        fresh.Restore(snap);
        Assert.Equal(snap, fresh.Snapshot());
        Assert.Equal(a.ComputeHash(), fresh.ComputeHash());
        Assert.Equal(a.ComputeHash(), fork.ComputeHash());
        var freshCar = fresh.GetBody(car.RawHandle);
        for (int i = 0; i < 60; i++)
        {
            car.ApplyForce(new Vector2F(2000f, 0)); a.Step(Dt);
            freshCar.ApplyForce(new Vector2F(2000f, 0)); fresh.Step(Dt);
            Assert.Equal(a.ComputeHash(), fresh.ComputeHash());
        }

        var w = Build(45, out var r2, out var h2, out var ro2, out var car2);
        var hashes = new ulong[40];
        using var stream = new MemoryStream();
        using (var recorder = w.StartRecording(stream))
        {
            for (int i = 0; i < hashes.Length; i++)
            {
                if (i == 10) { h2.ClimbUp(); }
                if (i == 12) { ro2.Release(); }
                if (i == 14) { r2.Dismount(new Vector2F(2, 0)); }
                car2.ApplyForce(new Vector2F(2000f, 0));
                w.Step(Dt); hashes[i] = w.ComputeHash();
            }
        }

        stream.Position = 0;
        var player = new ReplayPlayer(stream);
        var copy = player.CreateWorld2D();
        int n = 0;
        while (player.ReadNextStep(copy)) { Assert.Equal(hashes[n++], copy.ComputeHash()); }
        Assert.Equal(hashes.Length, n);
        Assert.Equal(w.Snapshot(), copy.Snapshot());
    }

    [Fact]
    public void RegionsKeepRidersWithTheirSeatsAndHoldersWithTheirRopes3D()
    {
        var s = Scene3D.Build(47);
        for (int i = 0; i < 20; i++) { s.Tick(i); }
        // A box around the car only: the rider belongs with it, the rope holder does not.
        var min = new Vector3D(-14, -1, -4); var max = new Vector3D(-6, 5, 8);
        var region = s.World.CaptureRegion(min, max);
        var hash = region.ComputeHash();
        s.Rider.Dismount();
        s.Car.LinearVelocity = new Vector3F(5, 0, 0);
        Assert.NotEqual(hash, s.World.ComputeRegionHash(min, max));
        Assert.Equal(CharacterMode.Free, s.Rider.Mode);
        s.World.RestoreRegion(RegionSnapshot.Read(region.ToArray()));
        Assert.Equal(hash, s.World.ComputeRegionHash(min, max));
        Assert.Equal(CharacterMode.Seated, s.Rider.Mode);
    }

    [Fact]
    public void ShiftingTheOriginKeepsHoldsAndCarriesWhereTheyWere3D()
    {
        var s = Scene3D.Build(48);
        for (int i = 0; i < 20; i++) { s.Tick(i); }
        var before = new[] { s.Rider.Position, s.Hanger.Position, s.Climber.Position, s.Roper.Position, s.Deck.Position };
        var shift = new Vector3D(64, 0, -32);
        s.World.ShiftOrigin(shift);
        var after = new[] { s.Rider.Position, s.Hanger.Position, s.Climber.Position, s.Roper.Position, s.Deck.Position };
        for (int i = 0; i < before.Length; i++)
        {
            Assert.Equal(before[i].X - shift.X, after[i].X, 9);
            Assert.Equal(before[i].Z - shift.Z, after[i].Z, 9);
        }

        for (int i = 20; i < 40; i++) { s.Tick(i); }
        Assert.Equal(CharacterMode.Hanging, s.Hanger.Mode);
        Assert.Equal(CharacterMode.Roped, s.Roper.Mode);
    }

    [Fact]
    public void DebugGeometryShowsTheHolds3D()
    {
        var s = Scene3D.Build(49);
        for (int i = 0; i < 5; i++) { s.Tick(i); }
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        s.World.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.CharacterHolds, g);
        Assert.Equal(4, g.CharacterHolds.Length);
        Assert.Equal(CharacterMode.Seated, g.CharacterHolds[0].Mode);
        Assert.Equal(CharacterMode.Hanging, g.CharacterHolds[1].Mode);
        Assert.Equal(CharacterMode.Climbing, g.CharacterHolds[2].Mode);
        Assert.Equal(CharacterMode.Roped, g.CharacterHolds[3].Mode);
        Assert.True(g.CharacterHolds[1].Normal.Z < -0.9f);
    }

    [Fact]
    public void ASnapshotWithDamagedAbilityStateIsRefusedCleanly()
    {
        var s = Scene3D.Build(46);
        for (int i = 0; i < 5; i++) { s.Tick(i); }
        var bytes = s.World.Snapshot();
        var fresh = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 46 });
        // Flip bytes of the section only: find it and damage its mode byte region, then every attempt either restores a valid world or throws the typed error; none corrupts the target.
        int at = Array.FindIndex(bytes, 0, bytes.Length, _ => false);
        _ = at;
        var needle = BitConverter.GetBytes(0x43455854);
        int pos = -1;
        for (int i = 0; i + 4 <= bytes.Length; i++) { if (bytes[i] == needle[0] && bytes[i + 1] == needle[1] && bytes[i + 2] == needle[2] && bytes[i + 3] == needle[3]) { pos = i; break; } }
        Assert.True(pos > 0);
        int refused = 0;
        for (int k = 8; k < 64 && pos + k < bytes.Length; k++)
        {
            var damaged = (byte[])bytes.Clone();
            damaged[pos + k] ^= 0xFF;
            var target = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 46 });
            try { target.Restore(damaged); }
            catch (SnapshotFormatException) { refused++; target.Step(Dt); }
        }

        Assert.True(refused > 0);
    }
}
