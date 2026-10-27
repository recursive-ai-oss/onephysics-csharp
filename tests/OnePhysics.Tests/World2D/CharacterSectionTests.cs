using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

/// <summary>Characters together with joints, layers, sensors and debug geometry: the 2D snapshot section combinations, and crowds of characters.</summary>
public class CharacterSectionTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(int id = 91)
        => new(new WorldOptions2D { Gravity = new Vector2F(0f, -9.81f), SleepEnabled = false, WorldId = id, ThreadCount = 1 });

    private static PhysicsWorld2D Make(bool joints, bool services, bool characters)
    {
        var w = NewWorld();
        var floor = w.CreateBody(BodyType.Static, new Vector2D(0, -0.5));
        w.CreateBox(floor, 30f, 0.5f, default, 0f, 0f, new ShapeDef { Layer = services ? 2 : 0 });
        for (int i = 0; i < 4; i++)
        {
            var b = w.CreateBody(BodyType.Dynamic, new Vector2D(i * 1.2 - 3, 1.0 + 0.3 * i));
            w.CreateCircle(b, 0.5f, default, new ShapeDef { Layer = services ? 1 : 0 });
        }

        if (services)
        {
            w.IgnoreLayerCollision(1, 3);
            var s = w.CreateBody(BodyType.Static, new Vector2D(0, 1));
            w.CreateBox(s, 1f, 1f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        }

        if (joints)
        {
            var pivot = w.AddCircle(0.1f, new Vector2D(5, 8), BodyType.Static);
            var bob = w.AddCircle(0.3f, new Vector2D(7, 8));
            w.AddHinge(pivot, bob, new Vector2D(5, 8));
        }

        if (characters)
        {
            w.AddCharacter(new Vector2D(-8, 1), Character.Humanoid);
            w.AddCharacter(new Vector2D(8, 1), Character.Child);
        }

        return w;
    }

    private static void Drive(PhysicsWorld2D w, int steps, List<ulong>? hashes = null)
    {
        var chars = w.GetCharacters();
        for (int s = 0; s < steps; s++)
        {
            for (int i = 0; i < chars.Length; i++)
            {
                chars[i].Move(new Vector2F(1.5f * Dt, (chars[i].IsGrounded ? -0.5f : -4f) * Dt), Dt);
            }

            w.Step(Dt);
            hashes?.Add(w.ComputeHash());
        }
    }

    private static int SectionStart(byte[] snapshot, PhysicsWorld2D w)
    {
        var hertz = BitConverter.GetBytes(w.Options.ContactHertz);
        for (int i = 4; i + 8 <= snapshot.Length; i++)
        {
            if (snapshot[i + 4] == hertz[0] && snapshot[i + 5] == hertz[1] && snapshot[i + 6] == hertz[2] && snapshot[i + 7] == hertz[3]
                && snapshot[i + 1] == 0 && snapshot[i + 2] == 0 && snapshot[i + 3] == 0 && snapshot[i] >= 1 && snapshot[i] <= 4) { return i; }
        }

        return -1;
    }

    [Theory]
    [InlineData(false, false, false, 1u)]
    [InlineData(false, true, false, 2u)]
    [InlineData(true, false, false, 3u)]
    [InlineData(true, true, false, 3u)]
    [InlineData(false, false, true, 4u)]
    [InlineData(false, true, true, 4u)]
    [InlineData(true, false, true, 4u)]
    [InlineData(true, true, true, 4u)]
    public void EveryCombinationRoundTripsByteExactlyAndReplays(bool joints, bool services, bool characters, uint version)
    {
        var w = Make(joints, services, characters);
        Drive(w, 50);
        var snap = w.Snapshot();
        Assert.Equal(version, BitConverter.ToUInt32(snap, SectionStart(snap, w)));
        var fresh = NewWorld();
        fresh.Restore(snap);
        Assert.True(snap.AsSpan().SequenceEqual(fresh.Snapshot()));
        Assert.Equal(w.ComputeHash(), fresh.ComputeHash());
        Assert.Equal(characters ? 2 : 0, fresh.GetCharacters().Length);
        var a = new List<ulong>(); var b = new List<ulong>();
        Drive(w, 50, a); Drive(fresh, 50, b);
        Assert.Equal(a, b);
        Assert.True(w.Snapshot().AsSpan().SequenceEqual(fresh.Snapshot()));
    }

    [Fact]
    public void MalformedCombinationsAreRejectedWithTypedErrorsAndLeaveTheWorldIntact()
    {
        var w = Make(true, true, true);
        Drive(w, 5);
        var snap = w.Snapshot();
        int at = SectionStart(snap, w);
        Assert.True(at > 0);
        foreach (uint other in new uint[] { 0, 1, 2, 3, 5, 99 })
        {
            var bad = (byte[])snap.Clone();
            BitConverter.TryWriteBytes(bad.AsSpan(at), other);
            var target = NewWorld();
            try
            {
                target.Restore(bad);
                Assert.False(bad.AsSpan().SequenceEqual(target.Snapshot()));
            }
            catch (PhysicsException) { }
        }

        // The flags word follows the free-pair stack; find it from the tail: it is the only u32 equal to 7 followed by the joint tunables.
        int flagsAt = -1;
        for (int i = at; i + 12 <= snap.Length; i++)
        {
            if (BitConverter.ToUInt32(snap, i) == 7u && BitConverter.ToSingle(snap, i + 4) == w.Options.JointHertz && BitConverter.ToSingle(snap, i + 8) == w.Options.JointDampingRatio) { flagsAt = i; break; }
        }

        Assert.True(flagsAt > 0);
        foreach (uint flags in new uint[] { 0, 1, 2, 3, 5, 6, 8, 0xFFFFFFFFu })
        {
            var bad = (byte[])snap.Clone();
            BitConverter.TryWriteBytes(bad.AsSpan(flagsAt), flags);
            var target = NewWorld();
            ulong before = target.ComputeHash();
            Assert.Throws<SnapshotFormatException>(() => target.Restore(bad));
            Assert.Equal(before, target.ComputeHash());
        }

        for (int cut = at; cut < snap.Length; cut += Math.Max(1, snap.Length / 97))
        {
            var target = Make(true, true, true);
            Drive(target, 5);
            ulong before = target.ComputeHash();
            Assert.Throws<SnapshotFormatException>(() => target.Restore(snap.AsSpan(0, cut).ToArray()));
            Assert.Equal(before, target.ComputeHash());
        }
    }

    [Fact]
    public void ACharacterStandingOnAJointedPlatformRidesIt()
    {
        var w = NewWorld(92);
        // A plank hinged at its centre to a fixed pivot, turning slowly; gravity gives it no torque.
        var pivot = w.AddCircle(0.1f, new Vector2D(0, 5), BodyType.Static);
        var plank = w.AddBox(new Vector2F(8f, 0.3f), at: new Vector2D(0, 5));
        var joint = w.AddHinge(pivot, plank, new Vector2D(0, 5));
        Assert.True(joint.IsValid);
        plank.AngularVelocity = 0.25f;
        var c = w.AddCharacter(new Vector2D(1.5, 6.1), Character.Humanoid);
        int grounded = 0;
        for (int i = 0; i < 90; i++)
        {
            c.Move(new Vector2F(0f, (c.IsGrounded ? -0.5f : -4f) * Dt), Dt);
            w.Step(Dt);
            if (i > 30 && c.IsGrounded && c.GroundBody == plank) { grounded++; }
        }

        Assert.True(grounded >= 55, "grounded on the plank for " + grounded);
        // The capsule rides on the plank's surface (a kinematic body on a dynamic one pushes it, so the plank may tilt either way).
        double surface = plank.Position.Y + 0.15 / Math.Cos(plank.Angle) + (c.Position.X - plank.Position.X) * Math.Tan(plank.Angle);
        Assert.InRange(c.Position.Y - surface, 0.85, 1.0);
    }

    [Fact]
    public void ACharacterWalksThroughASensorAndTriggersIt()
    {
        var w = NewWorld(96);
        w.AddBox(new Vector2F(60f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        // A trigger zone is a weightless dynamic body: a sensor never detects a static body, and two non-dynamic bodies never pair.
        var zdef = new BodyDef { Type = BodyType.Dynamic, Position = new Vector2D(3, 1), GravityScale = 0f };
        var zone = w.CreateBody(in zdef);
        w.CreateBox(zone, 0.05f, 0.05f, new Vector2F(0f, 5f), 0f, 1f, new ShapeDef { Layer = 5 });
        w.CreateBox(zone, 0.5f, 1f, default, 0f, 0f, new ShapeDef { IsSensor = true });
        w.IgnoreLayerCollision(5, 0);
        int enters = 0;
        var c = w.AddCharacter(new Vector2D(0, 0.91), Character.Humanoid);
        zone.OnTriggerEnter += t => { if (t.Other == c.Body) { enters++; } };
        for (int s = 0; s < 240; s++) { c.Move(new Vector2F(2f * Dt, -0.5f * Dt), Dt); w.Step(Dt); }
        Assert.Equal(1, enters);
        Assert.True(c.Position.X > 5.0, "blocked by the sensor at " + c.Position.X);
    }

    [Fact]
    public void DebugGeometryListsCharacterCapsulesAndGroundNormals()
    {
        var w = NewWorld(93);
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var c = w.AddCharacter(new Vector2D(0, 2), Character.Humanoid);
        for (int i = 0; i < 90; i++) { c.Move(new Vector2F(0f, (c.IsGrounded ? -0.5f : -4f) * Dt), Dt); w.Step(Dt); }
        Assert.True(c.IsGrounded);
        var g = new DebugGeometry();
        w.GetDebugGeometry(DebugContent.Characters, g);
        Assert.Equal(1, g.Characters.Length);
        var dc = g.Characters[0];
        Assert.True(dc.IsGrounded);
        Assert.Equal(1f, dc.GroundNormal.Y, 3);
        Assert.Equal(c.Position.Y, dc.Center.Y, 6);
        Assert.Equal(1u, (uint)dc.GroundBody.Generation);
        Assert.Equal(0.3f, dc.Radius, 6);
    }

    [Fact]
    public void ACrowdOnOneLayerStaysStillAndNeverRidesItsNeighbours()
    {
        var w = NewWorld(94);
        w.AddBox(new Vector2F(60f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        // Forty characters standing closer together than their width (overlapping by design).
        var chars = new Character[40];
        for (int i = 0; i < chars.Length; i++) { chars[i] = w.AddCharacter(new Vector2D(-10 + i * 0.25, 0.91), Character.Humanoid); }
        var start = new Vector2D[chars.Length];
        for (int i = 0; i < chars.Length; i++) { start[i] = chars[i].Position; }
        double maxSpeed = 0;
        for (int s = 0; s < 600; s++)
        {
            foreach (var c in chars) { c.Move(new Vector2F(0f, -0.5f * Dt), Dt); }
            w.Step(Dt);
            foreach (var c in chars)
            {
                if (s > 10) { maxSpeed = Math.Max(maxSpeed, Math.Abs(c.Velocity.X) + Math.Abs(c.Velocity.Y)); }
                Assert.True(c.IsGrounded);
                Assert.True(c.GroundBody.Type == BodyType.Static);
            }
        }

        Assert.True(maxSpeed < 0.5, "crowd speed " + maxSpeed);
        for (int i = 0; i < chars.Length; i++)
        {
            Assert.True(Math.Abs(chars[i].Position.X - start[i].X) < 0.05 && Math.Abs(chars[i].Position.Y - start[i].Y) < 0.05, "character " + i + " crept");
        }
    }

    [Fact]
    public void TwoCharactersWalkingTowardEachOtherStopAtTheirRadii()
    {
        var w = NewWorld(95);
        w.AddBox(new Vector2F(60f, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);
        var a = w.AddCharacter(new Vector2D(-3, 0.95), Character.Humanoid);
        var b = w.AddCharacter(new Vector2D(3, 0.95), Character.Humanoid);
        for (int s = 0; s < 400; s++)
        {
            a.Move(new Vector2F(2f * Dt, -0.5f * Dt), Dt);
            b.Move(new Vector2F(-2f * Dt, -0.5f * Dt), Dt);
            w.Step(Dt);
        }

        double gap = b.Position.X - a.Position.X;
        Assert.InRange(gap, 0.59, 0.65);
        Assert.True(a.IsGrounded && b.IsGrounded);
    }
}
