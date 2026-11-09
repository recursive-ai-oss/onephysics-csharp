using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;
using RecursiveAI.OnePhysics.Rods;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

public sealed class CharacterAbilities2DTests(ITestOutputHelper output)
{
    private const float Dt = 1f / 60f;
    private const float G = 9.81f;

    private void Note(string line)
    {
        output.WriteLine(line);
        string? path = Environment.GetEnvironmentVariable("ONEPHYSICS_TEST_NOTES");
        if (path != null) { System.IO.File.AppendAllText(path, line + "\n"); }
    }

    private static PhysicsWorld2D NewWorld(int threads = 1, int id = 61) => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    private static Body Floor(PhysicsWorld2D w, float half = 50f) => w.AddBox(new Vector2F(2 * half, 1f), at: new Vector2D(0, -0.5), type: BodyType.Static);

    private sealed class Driver(Character c)
    {
        public readonly Character C = c;
        public float Vx { get; set; }
        public float Vy { get; set; }

        public void Prepare()
        {
            Vy = C.IsGrounded && Vy < 0f ? -1f : Vy - G * Dt;
            C.Move(new Vector2F(Vx * Dt, Vy * Dt), Dt);
        }
    }

    private static CharacterDef Climber()
    {
        var def = CharacterDef.Humanoid;
        def.CanClimb = true;
        return def;
    }

    private static readonly Vector2F Ahead = new(1, 0);

    [Fact]
    public void ASeatedRiderStaysAtItsSeatThroughHardAccelerationAndATurn()
    {
        var w = NewWorld();
        Floor(w);
        var car = w.AddBox(new Vector2F(4f, 0.6f), at: new Vector2D(0, 0.5), density: 200f);
        var rider = w.AddCharacter(new Vector2D(0, 3), CharacterDef.Humanoid);
        var seat = new Vector2F(0.5f, 0.8f);
        rider.Mount(car, seat, 0.2f);
        Assert.Equal(CharacterMode.Seated, rider.Mode);
        double worst = 0;
        for (int i = 0; i < 600; i++)
        {
            car.ApplyForce(new Vector2F(i < 200 ? 20000f : 0f, 0));
            car.ApplyTorque(i >= 200 && i < 420 ? 4000f : 0f);
            rider.Move(new Vector2F(1, 0), Dt);
            w.Step(Dt);
            var p = car.Position;
            float a = car.Angle;
            var (s, c) = Math32.SinCos(a);
            double wx = p.X + c * seat.X - s * seat.Y, wy = p.Y + s * seat.X + c * seat.Y;
            var got = rider.Position;
            worst = Math.Max(worst, Math.Sqrt((wx - got.X) * (wx - got.X) + (wy - got.Y) * (wy - got.Y)));
        }

        Note("2d worst seat pose error " + worst.ToString("R"));
        Assert.True(worst <= 1e-4, "seat pose error " + worst);
        Assert.True(Math.Abs(car.AngularVelocity) > 0.3f, "the car should be turning");
        var v = rider.Release();
        Assert.Equal(CharacterMode.Free, rider.Mode);
        Assert.True((rider.Velocity - v).Length() < 1e-6f);
    }

    [Fact]
    public void ADeckCarriesAStandingCharacterWithoutDrift()
    {
        double Drift(bool exact)
        {
            var w = NewWorld();
            var deck = w.AddBox(new Vector2F(12f, 0.5f), at: new Vector2D(0, -0.25), type: BodyType.Kinematic);
            var def = CharacterDef.Humanoid;
            def.ExactPlatformCarry = exact;
            var ch = w.AddCharacter(new Vector2D(3, 0.91), def);
            var d = new Driver(ch);
            for (int i = 0; i < 30; i++) { d.Prepare(); w.Step(Dt); }
            Assert.True(ch.IsGrounded);
            double Local(Vector2D p)
            {
                var (s, c) = Math32.SinCos(-deck.Angle);
                double dx = p.X - deck.Position.X, dy = p.Y - deck.Position.Y;
                return c * dx - s * dy;
            }

            double l0 = Local(ch.Position), worst = 0;
            for (int i = 0; i < 600; i++)
            {
                float t = i * Dt;
                deck.LinearVelocity = new Vector2F(0.8f * MathF.Sin(t * 0.7f), 0.15f * MathF.Cos(t * 1.3f));
                deck.AngularVelocity = 0.15f * MathF.Cos(t * 0.9f);
                d.Prepare();
                w.Step(Dt);
                worst = Math.Max(worst, Math.Abs(Local(ch.Position) - l0));
            }

            return worst;
        }

        double byVelocity = Drift(false), byPose = Drift(true);
        Note("2d deck drift by velocity " + byVelocity.ToString("R") + " by pose " + byPose.ToString("R"));
        Assert.True(byPose <= 1e-3, "pose-carried drift " + byPose);
        Assert.True(byPose < byVelocity);
    }

    private static SoftBody Bar(PhysicsWorld2D w, out float pinY)
    {
        const int Nx = 30, Ny = 2;
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(3.0f, 0.2f), Nx, Ny), new Vector2D(0, 1.0));
        def.ParticleRadius = 0.09f; def.Mass = 400f; def.EdgeCompliance = 0f; def.LinearDamping = 0.5f;
        var pins = new System.Collections.Generic.List<int>();
        for (int j = 0; j <= Ny; j++) { pins.Add(j * (Nx + 1)); pins.Add(j * (Nx + 1) + Nx); }
        def.PinnedParticles = pins;
        var body = w.AddSoftBody(def);
        for (int i = 0; i < body.ParticleCount; i++)
        {
            var p = body.GetPosition(i);
            body.SetPosition(i, new Vector2D(p.X * 0.9, p.Y));
        }

        pinY = 1.0f;
        return body;
    }

    private static double Sag(SoftBody b, double pinY)
    {
        double low = double.MaxValue;
        for (int i = 0; i < b.ParticleCount; i++)
        {
            var p = b.GetPosition(i);
            if (Math.Abs(p.X) < 0.31) { low = Math.Min(low, p.Y); }
        }

        return pinY - low;
    }

    [Fact]
    public void ACharacterOnASoftBarLoadsItLikeARigidBodyOfTheSameMass()
    {
        double Loaded(bool character)
        {
            var w = NewWorld();
            var bar = Bar(w, out float pinY);
            for (int i = 0; i < 240; i++) { w.Step(Dt); }
            if (character)
            {
                var ch = w.AddCharacter(new Vector2D(0, 1.0 - Sag(bar, pinY) + 0.9 + 0.2), CharacterDef.Humanoid);
                var d = new Driver(ch);
                for (int i = 0; i < 600; i++) { d.Prepare(); w.Step(Dt); }
                Assert.True(ch.IsGrounded);
            }
            else
            {
                w.AddBox(new Vector2F(0.4f, 0.4f), at: new Vector2D(0, 1.0 - Sag(bar, pinY) + 0.1 + 0.2 + 0.05), density: 70f / 0.16f);
                for (int i = 0; i < 600; i++) { w.Step(Dt); }
            }

            return Sag(bar, pinY);
        }

        double byChar = Loaded(true), byBox = Loaded(false);
        Note("2d bar sag by a character " + byChar + " by a 70 kg box " + byBox);
        Assert.InRange(byChar / byBox, 0.8, 1.25);
    }

    private static RodDef StiffRope(Vector2D from, Vector2D to, int segments)
    {
        var pts = new Vector2D[segments + 1];
        for (int i = 0; i <= segments; i++) { pts[i] = from + (to - from) * ((double)i / segments); }
        return new RodDef { Points = pts, Material = new RodMaterial(2e10f, 8e9f, 0.003f, 1000f), PinnedParticles = [0] };
    }

    [Fact]
    public void ACharacterHangingFromARopeSwingsAsTheCombinedPendulumAndReleasesWithItsVelocity()
    {
        var w = NewWorld();
        var pivot = new Vector2D(0, 6);
        double theta0 = 10.0 * Math.PI / 180.0;
        const float L = 3f;
        var rope = w.AddRod(StiffRope(pivot, new Vector2D(Math.Sin(theta0) * L, 6 - Math.Cos(theta0) * L), 30));
        var ch = w.AddCharacter(new Vector2D(0, 3), CharacterDef.Humanoid);
        ch.Grab(rope, 30f);
        Assert.Equal(CharacterMode.Roped, ch.Mode);
        var crossings = new System.Collections.Generic.List<double>();
        double prev = double.NaN;
        for (int i = 0; i < 60 * 14; i++)
        {
            w.Step(Dt);
            double x = rope.GetPosition(30).X - pivot.X;
            if (!double.IsNaN(prev) && prev < 0 && x >= 0) { crossings.Add((i - 1 + (-prev) / (x - prev)) * (double)Dt); }
            prev = x;
        }

        Assert.True(crossings.Count >= 3);
        double period = (crossings[^1] - crossings[0]) / (crossings.Count - 1);
        var gp = rope.GetPosition(30);
        double length = Math.Sqrt((gp.X - pivot.X) * (gp.X - pivot.X) + (gp.Y - pivot.Y) * (gp.Y - pivot.Y));
        double rodMass = 1000.0 * Math.PI * 0.003 * 0.003 * L;
        double predicted = 2 * Math.PI * Math.Sqrt(length * (70 + rodMass / 3) / (G * (70 + rodMass / 2))) * (1 + theta0 * theta0 / 16.0);
        Note("2d pendulum period " + period + " predicted " + predicted);
        Assert.InRange(period / predicted, 0.98, 1.02);
        var grip = rope.GetVelocity(30);
        var v = ch.Release();
        Assert.Equal(CharacterMode.Free, ch.Mode);
        Assert.True((v - new Vector2F(grip.X, grip.Y)).Length() < 1e-4f);
        Assert.True(rope.Particles.InverseMass[30] > 1f);
    }

    [Fact]
    public void ACharacterWalksAlongARopeBridge()
    {
        var w = NewWorld();
        var from = new Vector2D(-6, 3); var to = new Vector2D(6, 3);
        var pts = new Vector2D[49];
        for (int i = 0; i <= 48; i++) { pts[i] = from + (to - from) * (i / 48.0); }
        var rope = w.AddRod(new RodDef { Points = pts, Material = new RodMaterial(2e9f, 8e8f, 0.01f, 1000f), PinnedParticles = [0, 48], Damping = 2f });
        var ch = w.AddCharacter(new Vector2D(-4, 3 + 0.9 + 0.02), CharacterDef.Humanoid);
        var d = new Driver(ch) { Vx = 0.8f };
        for (int i = 0; i < 240; i++) { d.Prepare(); w.Step(Dt); }
        Assert.True(ch.IsGrounded);
        double worstGap = 0;
        for (int i = 0; i < 360; i++)
        {
            d.Prepare(); w.Step(Dt);
            var p = ch.Position;
            int k = Math.Clamp((int)Math.Round((p.X + 6) / 12.0 * 48), 0, 48);
            worstGap = Math.Max(worstGap, Math.Abs(p.Y - 0.9 - (rope.GetPosition(k).Y + 0.01)));
            Assert.True(ch.IsGrounded, "fell off the rope at step " + i);
        }

        Note("2d rope bridge worst gap " + worstGap);
        Assert.True(ch.Position.X > -4 + 0.8 * 6 * 0.9);
        Assert.True(worstGap < 0.12);
    }

    public enum LedgeKind { Box, Tiles, HeightField }

    private static void AddLedge(PhysicsWorld2D w, LedgeKind kind, double top)
    {
        switch (kind)
        {
            case LedgeKind.Box:
                w.AddBox(new Vector2F(8f, (float)top), at: new Vector2D(6, top / 2), type: BodyType.Static);
                break;
            case LedgeKind.Tiles:
            {
                int ny = (int)Math.Round(top / 0.5);
                var tiles = new TileKind2D[24 * ny];
                for (int y = 0; y < ny; y++) { for (int x = 4; x < 24; x++) { tiles[y * 24 + x] = TileKind2D.Solid; } }
                w.AddTileGrid(24, ny, tiles, 0.5f, new Vector2D(0, 0));
                break;
            }
            case LedgeKind.HeightField:
            {
                var h = new float[121];
                for (int i = 0; i < h.Length; i++) { h[i] = i >= 20 ? (float)top : 0f; }
                w.AddHeightField(h, 0.1f, new Vector2D(0, 0));
                break;
            }
        }
    }

    private static Character AtLedge(PhysicsWorld2D w, double top, CharacterDef? def = null) => w.AddCharacter(new Vector2D(1.65, top - 0.8), def ?? Climber());

    [Theory]
    [InlineData(LedgeKind.Box)]
    [InlineData(LedgeKind.Tiles)]
    [InlineData(LedgeKind.HeightField)]
    public void ALedgeIsGrabbedHungAndClimbedOnEveryKindOfGeometry(LedgeKind kind)
    {
        var w = NewWorld();
        Floor(w);
        AddLedge(w, kind, 2.0);
        var ch = AtLedge(w, 2.0 - 0.05);
        Assert.True(ch.TryGrabLedge(Ahead), "no ledge found on " + kind);
        Assert.Equal(CharacterMode.Hanging, ch.Mode);
        var p = ch.Position;
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        var q = ch.Position;
        Assert.True(Math.Abs(p.Y - q.Y) < 1e-6 && Math.Abs(p.X - q.X) < 1e-6);
        Assert.InRange(q.X, 2 - 0.3 - 0.04, 2 - 0.3);
        Assert.True(ch.ClimbUp(), "climb up failed on " + kind);
        Assert.Equal(CharacterMode.Free, ch.Mode);
        var d = new Driver(ch);
        for (int i = 0; i < 60; i++) { d.Prepare(); w.Step(Dt); }
        Assert.True(ch.IsGrounded);
        Assert.InRange(ch.Position.Y, 2.89, 2.95);
        Assert.True(ch.Position.X > 2.0);
    }

    [Fact]
    public void ThingsThatAreNotLedgesAreNotGrabbed()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector2F(8f, 6f), at: new Vector2D(6, 3), type: BodyType.Static);
        Assert.False(AtLedge(w, 2.0).TryGrabLedge(Ahead));
        var w2 = NewWorld(1, 62);
        Floor(w2);
        w2.AddBox(new Vector2F(8f, 1.0f), at: new Vector2D(6, 0.5), type: BodyType.Static);
        Assert.False(AtLedge(w2, 2.0).TryGrabLedge(Ahead));
        var w3 = NewWorld(1, 63);
        Floor(w3);
        Assert.False(AtLedge(w3, 2.0).TryGrabLedge(Ahead));
        var w4 = NewWorld(1, 64);
        Floor(w4);
        w4.AddChain([new Vector2D(2, 0), new Vector2D(8, 3)], false, true);
        Assert.False(AtLedge(w4, 1.0).TryGrabLedge(Ahead), "a ramp is not a ledge");
        var w5 = NewWorld(1, 65);
        Floor(w5);
        AddLedge(w5, LedgeKind.Box, 2.0);
        Assert.False(AtLedge(w5, 1.95, CharacterDef.Humanoid).TryGrabLedge(Ahead));
    }

    [Fact]
    public void AHangerOnAMovingPlatformGoesWithItAndCanWallJump()
    {
        var w = NewWorld();
        var platform = w.AddBox(new Vector2F(8f, 2f), at: new Vector2D(6, 1), type: BodyType.Kinematic);
        var ch = AtLedge(w, 1.95);
        Assert.True(ch.TryGrabLedge(Ahead));
        var q0 = ch.Position - platform.Position;
        for (int i = 0; i < 240; i++)
        {
            platform.LinearVelocity = new Vector2F(0.5f, 0.2f * MathF.Sin(i * 0.05f));
            platform.AngularVelocity = 0.05f;
            w.Step(Dt);
        }

        var (s, c) = Math32.SinCos(-platform.Angle);
        var rel = ch.Position - platform.Position;
        double lx = c * rel.X - s * rel.Y, ly = s * rel.X + c * rel.Y;
        Assert.InRange(Math.Sqrt(lx * lx + ly * ly), Math.Sqrt(q0.X * q0.X + q0.Y * q0.Y) - 1e-3, Math.Sqrt(q0.X * q0.X + q0.Y * q0.Y) + 1e-3);
        var v = ch.WallJump(4f, 5f);
        Assert.Equal(CharacterMode.Free, ch.Mode);
        Assert.True(v.Y >= 4.0f);
    }

    [Fact]
    public void AClimbableLadderIsClimbedAndOverTheTop()
    {
        var w = NewWorld();
        Floor(w);
        var wall = w.AddBox(new Vector2F(1f, 4f), at: new Vector2D(2.5, 2), type: BodyType.Static);
        var ch = w.AddCharacter(new Vector2D(1.65, 0.9), Climber());
        Assert.False(ch.TryClimb(Ahead));
        var shape = wall.GetShapes()[0];
        shape.Climbable = true;
        Assert.True(shape.Climbable);
        Assert.True(ch.TryClimb(Ahead));
        Assert.Equal(CharacterMode.Climbing, ch.Mode);
        double y0 = ch.Position.Y;
        for (int i = 0; i < 60; i++) { ch.Climb(0.02f); w.Step(Dt); }
        Assert.InRange(ch.Position.Y - y0, 1.15, 1.25);
        Assert.InRange(ch.Position.X, 1.65, 1.72);
        for (int i = 0; i < 400 && ch.Mode == CharacterMode.Climbing; i++) { ch.Climb(0.02f); w.Step(Dt); }
        Assert.Equal(CharacterMode.Free, ch.Mode);
        var d = new Driver(ch);
        for (int i = 0; i < 60; i++) { d.Prepare(); w.Step(Dt); }
        Assert.True(ch.IsGrounded);
        Assert.InRange(ch.Position.Y, 4.89, 4.96);
        var ch2 = w.AddCharacter(new Vector2D(1.65, 1.5), Climber());
        Assert.True(ch2.TryClimb(Ahead));
        for (int i = 0; i < 200 && ch2.Mode == CharacterMode.Climbing; i++) { ch2.Climb(-0.02f); w.Step(Dt); }
        Assert.Equal(CharacterMode.Free, ch2.Mode);
    }
}
