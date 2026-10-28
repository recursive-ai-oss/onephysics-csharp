using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public class CharacterTests
{
    private const float Dt = 1f / 60f;
    private const float G = 9.81f;

    private static PhysicsWorld2D NewWorld(int threads = 1, int id = 61) => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    /// <summary>Static floor with its top at y = 0.</summary>
    private static Body Floor(PhysicsWorld2D w, double x = 0, double halfWidth = 50) => w.AddBox(new Vector2F((float)(2 * halfWidth), 1f), at: new Vector2D(x, -0.5), type: BodyType.Static);

    /// <summary>Drives a character the way a game would: horizontal speed plus gravity integrated into a vertical speed that resets on the ground.</summary>
    private sealed class Driver
    {
        public readonly Character C;
        public float Vx, Vy;
        public CharacterContact Last;

        public Driver(Character c) { C = c; }

        public void Tick(PhysicsWorld2D w)
        {
            Vy = C.IsGrounded && Vy < 0f ? -1f : Vy - G * Dt;
            Last = C.Move(new Vector2F(Vx * Dt, Vy * Dt), Dt);
            w.Step(Dt);
        }
    }

    private static Driver Spawn(PhysicsWorld2D w, double x, double y, CharacterDef? def = null) => new(w.AddCharacter(new Vector2D(x, y), def ?? CharacterDef.Humanoid));

    private static void Run(PhysicsWorld2D w, Driver d, int steps) { for (int i = 0; i < steps; i++) { d.Tick(w); } }

    [Fact]
    public void ACharacterLandsOnTheFloorAndStandsStill()
    {
        var w = NewWorld();
        Floor(w);
        var d = Spawn(w, 0, 3);
        Run(w, d, 120);
        Assert.True(d.C.IsGrounded);
        Assert.Equal(0f, d.C.GroundNormal.X, 4);
        Assert.Equal(1f, d.C.GroundNormal.Y, 4);
        Assert.InRange(d.C.Position.Y, 0.9 - 1e-4, 0.9 + 0.002);
        double y0 = d.C.Position.Y, x0 = d.C.Position.X;
        double worst = 0;
        for (int i = 0; i < 600; i++)
        {
            var before = d.C.Position;
            d.Tick(w);
            var after = d.C.Position;
            worst = Math.Max(worst, Math.Max(Math.Abs(after.X - before.X), Math.Abs(after.Y - before.Y)));
        }

        Assert.True(worst <= 1e-5, "standing character moved by " + worst);
        Assert.Equal(y0, d.C.Position.Y, 4);
        Assert.Equal(x0, d.C.Position.X, 4);
        Assert.True((d.Last & CharacterContact.Below) != 0);
    }

    [Fact]
    public void ACharacterStopsAgainstAWallWithoutJitter()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector2F(1f, 10f), at: new Vector2D(6, 5), type: BodyType.Static); // face at x = 5.5
        var d = Spawn(w, 0, 0.95);
        d.Vx = 3f;
        Run(w, d, 200);
        // Stops a radius plus the skin short of the wall.
        Assert.InRange(d.C.Position.X, 5.5 - 0.3 - 0.01 - 0.002, 5.5 - 0.3 - 0.01 + 0.002);
        Assert.True((d.Last & CharacterContact.Sides) != 0);
        double worst = 0;
        for (int i = 0; i < 600; i++)
        {
            var before = d.C.Position;
            d.Tick(w);
            var after = d.C.Position;
            worst = Math.Max(worst, Math.Max(Math.Abs(after.X - before.X), Math.Abs(after.Y - before.Y)));
        }

        Assert.True(worst <= 1e-5, "character pressed against a wall moved by " + worst + " in one step");
        Assert.True(d.C.IsGrounded);
    }

    [Fact]
    public void ACharacterSlidesIntoTheCornerOfAWallAndTheFloor()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector2F(1f, 10f), at: new Vector2D(6, 5), type: BodyType.Static);
        var c = w.AddCharacter(new Vector2D(2, 6), CharacterDef.Humanoid);
        // Diagonal movement down and into the wall: it slides down the wall, then along the floor, and rests in the corner touching neither.
        for (int i = 0; i < 400; i++)
        {
            c.Move(new Vector2F(0.04f, -0.04f), Dt);
            w.Step(Dt);
        }

        Assert.InRange(c.Position.X, 5.19 - 0.002, 5.19 + 0.002);
        Assert.InRange(c.Position.Y, 0.9, 0.9 + 0.002);
        Assert.True(c.IsGrounded);
    }

    [Fact]
    public void ACharacterWalksUpAStepNoTallerThanTheStepOffsetAndRefusesATallerOne()
    {
        double Walk(float stepHeight, out bool grounded)
        {
            var w = NewWorld();
            Floor(w);
            w.AddBox(new Vector2F(20f, stepHeight), at: new Vector2D(13, stepHeight / 2), type: BodyType.Static); // face at x = 3
            var d = Spawn(w, 0, 0.95);
            d.Vx = 2f;
            Run(w, d, 240);
            grounded = d.C.IsGrounded;
            return d.C.Position.Y;
        }

        // Step offset 0.3: a 0.25 step is climbed (the character ends up on top, 0.25 higher and well past the face).
        double up = Walk(0.25f, out bool g1);
        Assert.InRange(up, 0.9 + 0.25 - 0.003, 0.9 + 0.25 + 0.003);
        Assert.True(g1);

        // A 0.4 step is a wall.
        double blocked = Walk(0.4f, out bool g2);
        Assert.InRange(blocked, 0.9, 0.9 + 0.002);
        Assert.True(g2);
    }

    [Fact]
    public void ThePositionAfterClimbingIsPastTheStepFace()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector2F(20f, 0.25f), at: new Vector2D(13, 0.125), type: BodyType.Static);
        var d = Spawn(w, 0, 0.95);
        d.Vx = 2f;
        Run(w, d, 240);
        Assert.True(d.C.Position.X > 6.0, "x = " + d.C.Position.X);
    }

    [Fact]
    public void ACharacterStandsOnAGentleSlopeAndSlidesDownASteepOne()
    {
        double Drift(double degrees, out bool grounded, out Vector2D end)
        {
            var w = NewWorld();
            Floor(w);
            double t = Math.Tan(degrees * Math.PI / 180.0);
            w.AddPolygon([new Vector2F(0, 0), new Vector2F(20, 0), new Vector2F(20, (float)(20 * t))], at: default, type: BodyType.Static);
            var d = Spawn(w, 10, 10 * t + 0.9 / Math.Cos(degrees * Math.PI / 180.0) + 0.2);
            Run(w, d, 60);
            var p0 = d.C.Position;
            Run(w, d, 120);
            end = d.C.Position;
            grounded = d.C.IsGrounded;
            return end.X - p0.X;
        }

        double drift30 = Drift(30, out bool g30, out _);
        Assert.True(g30);
        Assert.InRange(drift30, -0.005, 0.005);

        double drift60 = Drift(60, out bool g60, out var end60);
        Assert.True(drift60 < -1.0, "steep slope drift " + drift60);
        _ = g60;
        Assert.True(end60.X < 10);
    }

    [Fact]
    public void ACharacterRidesAMovingPlatform()
    {
        var w = NewWorld();
        var def = new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(0, -0.25), LinearVelocity = new Vector2F(2f, 0f) };
        var platform = w.CreateBody(in def);
        w.CreateBox(platform, new Vector2F(10f, 0.5f));
        var d = Spawn(w, 0, 0.95);
        Run(w, d, 30);
        Assert.True(d.C.IsGrounded);
        double x0 = d.C.Position.X;
        Run(w, d, 120);
        Assert.True(d.C.IsGrounded);
        Assert.Equal(platform, d.C.GroundBody);
        Assert.InRange(d.C.Position.X - x0, 4.0 - 0.05, 4.0 + 0.05);
        Assert.InRange(d.C.Position.Y, 0.9, 0.9 + 0.002);
        Assert.Equal(2f, d.C.GroundVelocity.X, 3);
    }

    [Fact]
    public void ACharacterRidesAPlatformMovingUp()
    {
        var w = NewWorld();
        var def = new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(0, -0.25), LinearVelocity = new Vector2F(0f, 1.5f) };
        var platform = w.CreateBody(in def);
        w.CreateBox(platform, new Vector2F(10f, 0.5f));
        var d = Spawn(w, 0, 0.95);
        Run(w, d, 30);
        double y0 = d.C.Position.Y;
        Run(w, d, 60);
        Assert.True(d.C.IsGrounded);
        Assert.InRange(d.C.Position.Y - y0, 1.5 - 0.05, 1.5 + 0.05);
    }

    [Fact]
    public void ACharacterPushesADynamicBoxOnlyWhenAskedTo()
    {
        double Push(float strength, out double boxX, out double boxV)
        {
            var w = NewWorld();
            Floor(w);
            var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(3, 0.5));
            var def = CharacterDef.Humanoid;
            def.PushStrength = strength;
            var d = Spawn(w, 0, 0.95, def);
            d.Vx = 2f;
            Run(w, d, 180);
            boxX = box.Position.X;
            boxV = box.LinearVelocity.X;
            return d.C.Position.X;
        }

        double cx = Push(1f, out double bx, out double bv);
        Assert.True(bx > 5.0, "box at " + bx);
        Assert.True(cx > 4.0, "character at " + cx);
        Assert.InRange(bv, 1.0, 2.5);

        double cx0 = Push(0f, out double bx0, out _);
        Assert.InRange(bx0, 2.99, 3.05);
        Assert.True(cx0 < 3 - 0.5 - 0.3 + 0.02, "blocked at " + cx0);
    }

    [Fact]
    public void ASteepSlopeIsReportedAndTheCharacterSlidesDownIt()
    {
        var w = NewWorld();
        Floor(w);
        // A ramp rising to the right at 54 degrees, steeper than the 45 degree limit; the character drops onto its face and cannot stand there.
        w.AddBox(new Vector2F(20f, 1f), at: new Vector2D(0, 3), angleRadians: 0.9425f, type: BodyType.Static);
        var d = Spawn(w, -0.5, 8);
        bool sawSteep = false;
        for (int i = 0; i < 240; i++)
        {
            d.Tick(w);
            sawSteep |= d.C.Ground == GroundState.OnSteepGround;
            Assert.Equal(d.C.Ground == GroundState.OnGround, d.C.IsGrounded);
        }

        Assert.True(sawSteep);
    }

    [Fact]
    public void FlatGroundIsOnGroundAndTheAirIsInAir()
    {
        var w = NewWorld();
        Floor(w);
        var d = Spawn(w, 0, 3);
        Assert.Equal(GroundState.InAir, d.C.Ground);
        Run(w, d, 120);
        Assert.Equal(GroundState.OnGround, d.C.Ground);
        Assert.Equal(1f, d.C.GroundNormal.Y, 4);
    }

    [Fact]
    public void MassAndMaxPushForceHaveTheirDefaultsPresetsAndLimits()
    {
        var c = NewWorld().AddCharacter(default);
        Assert.Equal(70f, c.Mass);
        Assert.Equal(500f, c.MaxPushForce);
        Assert.Equal(30f, CharacterDef.Child.Mass);
        Assert.Equal(400f, CharacterDef.Giant.Mass);
        Assert.Equal(2000f, CharacterDef.Giant.MaxPushForce);
        c.MaxPushForce = 0f;
        Assert.Equal(0f, c.MaxPushForce);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.MaxPushForce = -1f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.MaxPushForce = float.NaN);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.MaxPushForce = 2e9f);
        var w = NewWorld(id: 62);
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Mass = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Mass = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { MaxPushForce = -1f }));
    }

    [Fact]
    public void MaxPushForceLimitsHowHardACharacterPushes()
    {
        double BoxSpeed(float maxForce)
        {
            var w = NewWorld();
            Floor(w);
            var box = w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(1.2, 0.5), density: 20f); // 20 kg
            var def = CharacterDef.Humanoid;
            def.PushStrength = 1f;
            def.MaxPushForce = maxForce;
            var d = Spawn(w, 0, 0.95, def);
            d.Vx = 2f;
            Run(w, d, 30);
            return box.LinearVelocity.X;
        }

        double strong = BoxSpeed(1e6f), weak = BoxSpeed(10f), none = BoxSpeed(0f);
        Assert.True(strong > 1.0, "strong " + strong);
        Assert.True(weak < 0.5 * strong, "weak " + weak + " strong " + strong);
        Assert.True(none < weak, "none " + none + " weak " + weak);
    }

    [Fact]
    public void StandingOnADynamicBodyPressesItDownWithTheCharactersWeight()
    {
        var w = NewWorld();
        var plank = w.AddBox(new Vector2F(4f, 0.4f), at: new Vector2D(0, -0.2), density: 100f); // 160 kg
        var def = CharacterDef.Humanoid;
        def.Mass = 80f;
        var c = w.AddCharacter(new Vector2D(0, 0.9 + 0.001), def);
        c.Move(new Vector2F(0f, -0.001f), Dt); // first move: finds the ground and presses on it
        Assert.True(c.IsGrounded);
        // Impulse 80 kg * 9.81 * dt on a 160 kg plank.
        Assert.Equal(-80f * 9.81f * Dt / 160f, plank.LinearVelocity.Y, 4);
    }

    [Fact]
    public void TheHitCallbackReportsTheWallOnceWithItsNormal()
    {
        var w = NewWorld();
        Floor(w);
        var wall = w.AddBox(new Vector2F(1f, 10f), at: new Vector2D(6, 5), type: BodyType.Static);
        var d = Spawn(w, 5.15, 0.95);
        var hits = new List<ControllerColliderHit>();
        d.C.OnControllerColliderHit += h => hits.Add(h);
        d.Vx = 3f;
        d.Tick(w);
        var hit = Assert.Single(hits.FindAll(h => h.Body == wall));
        Assert.Equal(d.C, hit.Character);
        Assert.Equal(-1f, hit.Normal.X, 3);
        Assert.Equal(0f, hit.Normal.Y, 3);
        Assert.Equal(1f, hit.MoveDirection.X, 2);
    }

    [Fact]
    public void TheLayerMaskLetsTheCharacterWalkThroughAWall()
    {
        var w = NewWorld();
        Floor(w);
        var wall = w.AddBox(new Vector2F(1f, 10f), at: new Vector2D(6, 5), type: BodyType.Static);
        wall.Layer = 3;
        var def = CharacterDef.Humanoid;
        def.LayerMask = ~(1u << 3);
        var d = Spawn(w, 5, 0.95, def);
        d.Vx = 3f;
        Run(w, d, 100);
        Assert.True(d.C.Position.X > 8, "x = " + d.C.Position.X);
    }

    [Fact]
    public void SolvePlanesMatchesKnownAnswers()
    {
        static CollisionPlane2D P(float nx, float ny, float sep, float limit = 100f) => new() { Normal = new Vector2F(nx, ny), Separation = sep, PushLimit = limit };

        // A floor: the vertical part is removed, the sliding part kept.
        var floor = new[] { P(0, 1, 0) };
        var r = Mover2D.SolvePlanes(floor, new Vector2F(1, -1));
        Assert.Equal(1f, r.X, 5); Assert.Equal(0f, r.Y, 5);
        Assert.Equal(1f, floor[0].Push, 5);

        // A gap of 0.5 above the floor: only the part beyond the gap is removed.
        r = Mover2D.SolvePlanes(new[] { P(0, 1, 0.5f) }, new Vector2F(0, -1));
        Assert.Equal(-0.5f, r.Y, 5);

        // The inside corner of a floor and a wall: movement into both stops.
        r = Mover2D.SolvePlanes(new[] { P(0, 1, 0), P(-1, 0, 0) }, new Vector2F(1, -1));
        Assert.Equal(0f, r.X, 5); Assert.Equal(0f, r.Y, 5);

        // Movement away from the corner is untouched.
        r = Mover2D.SolvePlanes(new[] { P(0, 1, 0), P(-1, 0, 0) }, new Vector2F(-1, 1));
        Assert.Equal(-1f, r.X, 5); Assert.Equal(1f, r.Y, 5);

        // Two planes 60 degrees apart (a V-shaped notch): sliding down one is stopped by the other; the result satisfies both.
        float s = MathF.Sqrt(3f) / 2f;
        var v = new[] { P(0.5f, s, 0), P(-0.5f, s, 0) };
        r = Mover2D.SolvePlanes(v, new Vector2F(0.3f, -1f));
        Assert.True(r.X * 0.5f + r.Y * s >= -1e-4f && r.X * -0.5f + r.Y * s >= -1e-4f);
        Assert.True(MathF.Abs(r.X) < 1e-3f && MathF.Abs(r.Y) < 2e-3f, "notch result " + r.X + ", " + r.Y);

        // Two walls meeting at 90 degrees seen from the side (normals (1,1)/sqrt2 and (-1,1)/sqrt2), moving along the first wall's surface into the second.
        float h = 0.70710678f;
        r = Mover2D.SolvePlanes(new[] { P(h, h, 0), P(-h, h, 0) }, new Vector2F(1f, 0f));
        // First plane: d.n = 0.7071 >= 0 fine; second: d.n = -0.7071 < 0 -> pushed along (-h, h): d = (0.5, 0.5); then first: 0.7071 ok.
        Assert.Equal(0.5f, r.X, 4); Assert.Equal(0.5f, r.Y, 4);

        // An overlap is pushed out by at most the push limit; a zero limit never pushes but still blocks.
        r = Mover2D.SolvePlanes(new[] { P(0, 1, -0.3f, 0.1f) }, new Vector2F(0, 0));
        Assert.Equal(0.1f, r.Y, 5);
        r = Mover2D.SolvePlanes(new[] { P(0, 1, -0.3f, 0f) }, new Vector2F(1, -1));
        Assert.Equal(1f, r.X, 5); Assert.Equal(0f, r.Y, 5);
        r = Mover2D.SolvePlanes(new[] { P(0, 1, -0.3f, 0f) }, new Vector2F(1, 1));
        Assert.Equal(1f, r.Y, 5);

        // Velocity clipping keeps the speed along the surface.
        var clipped = Mover2D.ClipVelocity(new[] { P(0, 1, 0), P(-1, 0, 0.5f) }, new Vector2F(4, -3));
        Assert.Equal(4f, clipped.X, 5); Assert.Equal(0f, clipped.Y, 5);
    }

    [Fact]
    public void HostileInputsGiveTypedErrors()
    {
        var w = NewWorld();
        Floor(w);
        var c = w.AddCharacter(new Vector2D(0, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector2F(float.NaN, 0), Dt));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector2F(0, float.PositiveInfinity), Dt));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector2F(1e6f, 0), Dt));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector2F(1, 0), 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector2F(1, 0), float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.SlopeLimit = 91f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.StepOffset = -1f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.PushStrength = float.NaN);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Position = new Vector2D(double.NaN, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Position = new Vector2D(3e6, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(new Vector2D(0, 0), new CharacterDef { Radius = float.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(new Vector2D(0, 0), new CharacterDef { Radius = 1f, Height = 1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(new Vector2D(0, 0), new CharacterDef { Skin = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(new Vector2D(0, 0), new CharacterDef { Layer = 40 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(new Vector2D(double.PositiveInfinity, 0)));
        Assert.Single(w.GetCharacters()); // nothing half-created
        Assert.Throws<ArgumentOutOfRangeException>(() => Mover2D.SolvePlanes(new[] { new CollisionPlane2D { Normal = new Vector2F(2, 0) } }, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => Mover2D.SolvePlanes(new[] { new CollisionPlane2D { Normal = new Vector2F(0, 1), PushLimit = -1f } }, default));
        Assert.Throws<ArgumentOutOfRangeException>(() => Mover2D.SolvePlanes(Array.Empty<CollisionPlane2D>(), new Vector2F(float.NaN, 0)));
        var state = new MoverState2D { Position = new Vector2D(0, 1) };
        Assert.Throws<ArgumentNullException>(() => Mover2D.Move(null!, new CharacterDef(), ref state, default, Dt));
        state.Position = new Vector2D(5e6, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => Mover2D.Move(w, new CharacterDef(), ref state, default, Dt));
    }

    [Fact]
    public void StaleAndDefaultHandlesFailClearly()
    {
        var w = NewWorld();
        var c = w.AddCharacter(new Vector2D(0, 1));
        var copy = c;
        c.Destroy();
        Assert.False(copy.IsValid);
        Assert.Throws<StaleHandleException>(() => copy.Move(new Vector2F(1, 0), Dt));
        Assert.Throws<StaleHandleException>(() => _ = copy.IsGrounded);
        Assert.Throws<StaleHandleException>(() => copy.OnControllerColliderHit += _ => { });
        Assert.Throws<StaleHandleException>(() => copy.Destroy());
        Assert.Throws<InvalidHandleException>(() => default(Character).Move(new Vector2F(1, 0), Dt));

        // Destroying the body behind the host's back ends the character too, and a reused slot has a new generation.
        var c2 = w.AddCharacter(new Vector2D(0, 1));
        c2.Body.Destroy();
        Assert.False(c2.IsValid);
        Assert.Throws<StaleHandleException>(() => c2.Move(new Vector2F(1, 0), Dt));
        var c3 = w.AddCharacter(new Vector2D(0, 1));
        Assert.NotEqual(c2, c3);
        Assert.Throws<StaleHandleException>(() => w.GetCharacter(c2.RawHandle));
    }

    [Fact]
    public void CharactersSurviveASnapshotRoundTripByteForByte()
    {
        PhysicsWorld2D Build(int id)
        {
            var w = NewWorld(1, id);
            Floor(w);
            w.AddBox(new Vector2F(1f, 10f), at: new Vector2D(6, 5), type: BodyType.Static);
            var pdef = new BodyDef { Type = BodyType.Kinematic, Position = new Vector2D(-8, -0.25), LinearVelocity = new Vector2F(1f, 0f) };
            var pb = w.CreateBody(in pdef);
            w.CreateBox(pb, new Vector2F(6f, 0.5f));
            w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(3, 0.5));
            for (int i = 0; i < 4; i++)
            {
                var def = CharacterDef.Humanoid;
                def.PushStrength = i % 2;
                w.AddCharacter(new Vector2D(-8 + i * 2, 1.5), def);
            }

            return w;
        }

        void Drive(PhysicsWorld2D w, int from, int steps, List<ulong> hashes)
        {
            var chars = w.GetCharacters();
            for (int s = from; s < from + steps; s++)
            {
                for (int i = 0; i < chars.Length; i++)
                {
                    float vx = (s / 60 + i) % 2 == 0 ? 2.5f : -1.5f;
                    float vy = chars[i].IsGrounded ? (s % 90 == 0 ? 5f : -1f) : chars[i].Velocity.Y / 1f - G * Dt;
                    chars[i].Move(new Vector2F(vx * Dt, vy * Dt), Dt);
                }

                w.Step(Dt);
                hashes.Add(w.ComputeHash());
            }
        }

        var a = Build(71);
        var ha = new List<ulong>();
        Drive(a, 0, 90, ha);
        var snap = a.Snapshot();
        var hashAfter = a.ComputeHash();
        var b = NewWorld(1, 71);
        b.Restore(snap);
        Assert.Equal(hashAfter, b.ComputeHash());
        Assert.Equal(snap, b.Snapshot());
        var more = new List<ulong>(); var moreB = new List<ulong>();
        Drive(a, 90, 120, more);
        Drive(b, 90, 120, moreB);
        Assert.Equal(more, moreB);
        Assert.Equal(a.Snapshot(), b.Snapshot());

        // A world without characters writes the same bytes as before the feature existed: its section version stays 1.
        var plain = NewWorld(1, 72);
        Floor(plain);
        Assert.Equal(1u, BitConverter.ToUInt32(SectionBytes(plain.Snapshot()), 0));
    }

    private static byte[] SectionBytes(byte[] snapshot)
    {
        // The engine section is the last one; its first four bytes are the version. Find it by scanning for the section id 0x32445747 written as a little-endian i32.
        byte[] id = BitConverter.GetBytes(0x32445747);
        for (int i = snapshot.Length - 8; i >= 0; i--)
        {
            if (snapshot[i] == id[0] && snapshot[i + 1] == id[1] && snapshot[i + 2] == id[2] && snapshot[i + 3] == id[3])
            {
                // id, length, then the section bytes
                return snapshot.AsSpan(i + 8).ToArray();
            }
        }

        throw new InvalidOperationException("2D section not found");
    }

    [Fact]
    public void ManyCharactersGiveTheSameHashOnEveryThreadCount()
    {
        ulong Run(int threads)
        {
            var w = NewWorld(threads, 81);
            Floor(w);
            w.AddBox(new Vector2F(1f, 10f), at: new Vector2D(20, 5), type: BodyType.Static);
            for (int i = 0; i < 12; i++) { w.AddBox(new Vector2F(1f, 1f), at: new Vector2D(2 + i * 1.3, 0.5 + (i % 3) * 1.1)); }
            var chars = new Character[20];
            for (int i = 0; i < chars.Length; i++)
            {
                var def = CharacterDef.Humanoid;
                def.PushStrength = 0.5f;
                chars[i] = w.AddCharacter(new Vector2D(-10 + i * 0.7, 1), def);
            }

            ulong h = 0;
            for (int s = 0; s < 240; s++)
            {
                for (int i = 0; i < chars.Length; i++) { chars[i].Move(new Vector2F((2f + i * 0.1f) * Dt, (chars[i].IsGrounded ? -0.5f : -3f) * Dt), Dt); }
                w.Step(Dt);
                h = h * 31 + w.ComputeHash();
            }

            return h;
        }

        ulong one = Run(1);
        Assert.Equal(one, Run(4));
        Assert.Equal(one, Run(8));
    }

    [Fact]
    public void TheMiddleTierMovesACapsuleWithoutAHandle()
    {
        var w = NewWorld();
        Floor(w);
        var state = new MoverState2D { Position = new Vector2D(0, 0.95) };
        var def = new CharacterDef();
        var hits = new MoverHit2D[4];
        for (int i = 0; i < 60; i++)
        {
            var r = Mover2D.Move(w, in def, ref state, new Vector2F(0.05f, -0.02f), Dt, default, hits);
            Assert.True(r.HitCount <= hits.Length);
        }

        Assert.True(state.IsGrounded);
        Assert.InRange(state.Position.X, 2.9, 3.1);
        Assert.InRange(state.Position.Y, 0.9, 0.902);
    }
}
