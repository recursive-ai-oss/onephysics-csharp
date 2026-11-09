using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Rods;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public sealed class CharacterAbilities3DTests(ITestOutputHelper output)
{
    /// <summary>Measured values go to the test output and, when ONEPHYSICS_TEST_NOTES names a file, are appended there.</summary>
    private void Note(string line)
    {
        output.WriteLine(line);
        string? path = Environment.GetEnvironmentVariable("ONEPHYSICS_TEST_NOTES");
        if (path != null) { System.IO.File.AppendAllText(path, line + "\n"); }
    }

    private const float Dt = 1f / 60f;
    private const float G = 9.81f;

    private static PhysicsWorld3D NewWorld(int threads = 1, int id = 91) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    private static Body Floor(PhysicsWorld3D w, float half = 50f) => w.AddBox(new Vector3F(2 * half, 1f, 2 * half), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);

    /// <summary>A character driven like a game would: gravity into a vertical speed that resets on the ground.</summary>
    private sealed class Driver(Character c)
    {
        public readonly Character C = c;
        public float Vx { get; set; }
        public float Vy { get; set; }
        public float Vz { get; set; }

        public void Prepare()
        {
            Vy = C.IsGrounded && Vy < 0f ? -1f : Vy - G * Dt;
            C.Move(new Vector3F(Vx * Dt, Vy * Dt, Vz * Dt), Dt);
        }
    }

    private static Vector3D SeatWorld(Body b, Vector3F local)
    {
        var r = b.Rotation.Rotate(local);
        var p = b.Position;
        return new Vector3D(p.X + r.X, p.Y + r.Y, p.Z + r.Z);
    }

    [Fact]
    public void ASeatedRiderStaysAtItsSeatThroughHardAccelerationAndATurn()
    {
        var w = NewWorld();
        Floor(w);
        var car = w.AddBox(new Vector3F(2f, 0.6f, 4f), at: new Vector3D(0, 0.5, 0), density: 200f);
        var rider = w.AddCharacter(new Vector3D(0, 3, 0), CharacterDef.Humanoid);
        var seat = new Vector3F(0.4f, 0.8f, 0.5f);
        rider.Mount(car, seat);
        Assert.Equal(CharacterMode.Seated, rider.Mode);
        double worst = 0;
        for (int i = 0; i < 600; i++)
        {
            // Hard acceleration forward, then a spin through more than a full turn.
            car.ApplyForce(new Vector3F(0, 0, i < 200 ? 20000f : 0f));
            car.ApplyTorque(new Vector3F(0, i >= 200 && i < 420 ? 6000f : 0f, 0));
            rider.Move(new Vector3F(1, 0, 0), Dt);
            w.Step(Dt);
            var want = SeatWorld(car, seat);
            var got = rider.Position;
            worst = Math.Max(worst, Math.Sqrt((want.X - got.X) * (want.X - got.X) + (want.Y - got.Y) * (want.Y - got.Y) + (want.Z - got.Z) * (want.Z - got.Z)));
        }

        Note("worst seat pose error " + worst.ToString("R"));
        Assert.True(worst <= 1e-4, "seat pose error " + worst);
        float spin = Math.Abs(car.AngularVelocity.Y);
        Assert.True(spin > 0.5f, "the car should be turning: " + spin);
        var v = rider.Release();
        Assert.Equal(CharacterMode.Free, rider.Mode);
        var cv = car.LinearVelocity;
        Assert.True(v.Length() > 1f);
        Assert.True((rider.Velocity - v).Length() < 1e-6f);
        _ = cv;
    }

    [Fact]
    public void ADeckCarriesAStandingCharacterWithoutDrift()
    {
        double Drift(bool exact)
        {
            var w = NewWorld();
            var deck = w.AddBox(new Vector3F(12f, 0.5f, 12f), at: new Vector3D(0, -0.25, 0), type: BodyType.Kinematic);
            var def = CharacterDef.Humanoid;
            def.ExactPlatformCarry = exact;
            var ch = w.AddCharacter(new Vector3D(3, 0.91, 2), def);
            var d = new Driver(ch);
            for (int i = 0; i < 30; i++) { d.Prepare(); w.Step(Dt); }
            Assert.True(ch.IsGrounded);
            // The deck yaws, surges and heaves like a boat in a swell.
            double maxDrift = 0;
            var local0 = DeckLocal(deck, ch.Position);
            for (int i = 0; i < 600; i++)
            {
                float t = i * Dt;
                deck.LinearVelocity = new Vector3F(0.8f * MathF.Sin(t * 0.7f), 0.15f * MathF.Cos(t * 1.3f), 0.5f * MathF.Cos(t * 0.5f));
                deck.AngularVelocity = new Vector3F(0, 0.4f, 0.03f * MathF.Sin(t));
                d.Prepare();
                w.Step(Dt);
                var l = DeckLocal(deck, ch.Position);
                maxDrift = Math.Max(maxDrift, Math.Sqrt((l.X - local0.X) * (l.X - local0.X) + (l.Z - local0.Z) * (l.Z - local0.Z)));
            }

            return maxDrift;
        }

        double byVelocity = Drift(false), byPose = Drift(true);
        Note("deck drift by velocity " + byVelocity.ToString("R") + " by pose " + byPose.ToString("R"));
        Assert.True(byPose <= 1e-3, "pose-carried drift " + byPose);
        Assert.True(byPose < byVelocity, "the pose carry should be no worse than the velocity carry");
    }

    /// <summary>A strip of cloth pinned along both short ends, with slack: cells along x, a few across z.</summary>
    private static SoftBody Hammock(PhysicsWorld3D w, float restLength, float span, float depth, out float sagPrediction, out float pinY)
    {
        int nx = 30, nz = 8;
        var def = SoftBodyDef.Cloth(SoftBodyMesh.Cloth(restLength, depth, nx, nz), new Vector3D(0, 1.0, 0));
        def.ParticleRadius = 0.09f; def.Mass = 400f; def.EdgeCompliance = 0f; def.BendCompliance = 1e-2f; def.LinearDamping = 0.5f;
        var pins = new System.Collections.Generic.List<int>();
        for (int j = 0; j <= nz; j++) { pins.Add(j * (nx + 1)); pins.Add(j * (nx + 1) + nx); }
        def.PinnedParticles = pins;
        var body = w.AddSoftBody(def);
        // Pull the ends in so that the strip has slack.
        float shrink = span / restLength;
        for (int j = 0; j <= nz; j++)
        {
            for (int i = 0; i <= nx; i++)
            {
                var p = body.GetPosition(j * (nx + 1) + i);
                body.SetPosition(j * (nx + 1) + i, new Vector3D(p.X * shrink, p.Y, p.Z));
            }
        }

        sagPrediction = MathF.Sqrt(0.25f * restLength * restLength - 0.25f * span * span);
        pinY = 1.0f;
        return body;
    }

    private static double CenterSag(SoftBody b, double pinY)
    {
        double low = double.MaxValue;
        for (int i = 0; i < b.ParticleCount; i++)
        {
            var p = b.GetPosition(i);
            if (Math.Abs(p.X) < 0.31 && Math.Abs(p.Z) < 0.05) { low = Math.Min(low, p.Y); }
        }

        return pinY - low;
    }

    [Fact]
    public void ACharacterOnAHammockLoadsItLikeARigidBodyOfTheSameMass()
    {
        double Loaded(bool character)
        {
            var w = NewWorld();
            var cloth = Hammock(w, 3.0f, 2.7f, 1.2f, out _, out float pinY);
            for (int i = 0; i < 240; i++) { w.Step(Dt); }
            if (character)
            {
                var ch = w.AddCharacter(new Vector3D(0, 1.0 + 0.9 + 0.09 + 0.05, 0), CharacterDef.Humanoid);
                var d = new Driver(ch);
                for (int i = 0; i < 600; i++) { d.Prepare(); w.Step(Dt); }
                Assert.True(ch.IsGrounded);
            }
            else
            {
                // A box of 70 kg set on the sagged strip.
                var box = w.AddBox(new Vector3F(0.4f, 0.4f, 0.4f), at: new Vector3D(0, 1.0 - CenterSag(cloth, pinY) + 0.09 + 0.2 + 0.02, 0), density: 70f / 0.064f);
                for (int i = 0; i < 600; i++) { w.Step(Dt); }
                _ = box;
            }

            return CenterSag(cloth, pinY);
        }

        double withChar = Loaded(true), withBox = Loaded(false);
        Note("hammock sag with a character " + withChar + " with a 70 kg box " + withBox);
        Assert.InRange(withChar / withBox, 0.85, 1.15);
    }

    /// <summary>Sag of a rope of length l pinned at both ends a span apart under a point load w at its middle: (l/2 (1 + T/EA))^2 = (span/2)^2 + h^2 with T = w / (2 sin(theta)).</summary>
    private static double TautRopeSag(double length, double span, double load, double ea)
    {
        double lo = 0, hi = length;
        for (int i = 0; i < 200; i++)
        {
            double h = 0.5 * (lo + hi);
            double half = Math.Sqrt(0.25 * span * span + h * h);
            double sin = h / half, tension = load / (2 * sin);
            double stretched = 0.5 * length * (1 + tension / ea);
            if (half > stretched) { hi = h; } else { lo = h; }
        }

        return 0.5 * (lo + hi);
    }

    [Fact]
    public void ACharacterOnATautRopeLoadsItWithItsWeight()
    {
        const double Length = 10.0;
        double Sag(bool character)
        {
            var w = NewWorld();
            var from = new Vector3D(-5, 3, 0); var to = new Vector3D(5, 3, 0);
            var pts = new Vector3D[41];
            for (int i = 0; i <= 40; i++) { pts[i] = from + (to - from) * (i / 40.0); }
            var mat = new RecursiveAI.OnePhysics.Rods.RodMaterial(2e9f, 8e8f, 0.01f, 1000f);
            var rope = w.AddRod(new RodDef { Points = pts, Material = mat, PinnedParticles = [0, 40], Damping = 2f });
            if (character)
            {
                var ch = w.AddCharacter(new Vector3D(0, 3 + 0.9 + 0.02, 0), CharacterDef.Humanoid);
                var d = new Driver(ch);
                for (int i = 0; i < 1200; i++) { d.Prepare(); w.Step(Dt); }
                Assert.True(ch.IsGrounded);
                var gap = ch.Position.Y - 0.9 - (rope.GetPosition(20).Y + 0.01);
                Assert.InRange(gap, -0.02, 0.05);
            }
            else
            {
                // A 70 kg ball hung from the middle of the rope.
                var ball = w.AddSphere(0.2f, new Vector3D(0, 3, 0), density: 70f / (4f / 3f * MathF.PI * 0.008f));
                rope.Attach(20, ball);
                for (int i = 0; i < 1200; i++) { w.Step(Dt); }
            }

            return 3.0 - rope.GetPosition(20).Y;
        }

        double byCharacter = Sag(true), byBox = Sag(false);
        double ea = 2e9 * Math.PI * 0.01 * 0.01;
        double predicted = TautRopeSag(Length, Length, 70.0 * G, ea);
        Note("rope sag by a character " + byCharacter + " by a 70 kg ball hung at the middle " + byBox + " cube-root law " + predicted);
        // The rope's solver carries a rigid 70 kg load about 11 percent below the closed form at this tension (a rigidly attached 70 kg body gives the same), so the closed form is the loose bound
        // and the ball hung on the rope, loaded through the solver's own coupling, the tight one.
        Assert.InRange(byCharacter / byBox, 0.9, 1.1);
        Assert.InRange(byCharacter / predicted, 0.85, 1.25);
    }

    [Fact]
    public void ACharacterWalksAlongARopeBridge()
    {
        var w = NewWorld();
        var from = new Vector3D(-6, 3, 0); var to = new Vector3D(6, 3, 0);
        var pts = new Vector3D[49];
        for (int i = 0; i <= 48; i++) { pts[i] = from + (to - from) * (i / 48.0); }
        var mat = new RecursiveAI.OnePhysics.Rods.RodMaterial(2e9f, 8e8f, 0.01f, 1000f);
        var rope = w.AddRod(new RodDef { Points = pts, Material = mat, PinnedParticles = [0, 48], Damping = 2f });
        var ch = w.AddCharacter(new Vector3D(-4, 3 + 0.9 + 0.02, 0), CharacterDef.Humanoid);
        var d = new Driver(ch) { Vx = 0.8f };
        for (int i = 0; i < 240; i++) { d.Prepare(); w.Step(Dt); }
        Assert.True(ch.IsGrounded);
        double worstGap = 0, worstOff = 0;
        for (int i = 0; i < 360; i++)
        {
            d.Prepare(); w.Step(Dt);
            var p = ch.Position;
            int k = Math.Clamp((int)Math.Round((p.X + 6) / 12.0 * 48), 0, 48);
            double ropeY = rope.GetPosition(k).Y + 0.01;
            worstGap = Math.Max(worstGap, Math.Abs(p.Y - 0.9 - ropeY));
            worstOff = Math.Max(worstOff, Math.Abs(p.Z));
            Assert.True(ch.IsGrounded, "fell off the rope at step " + i);
        }

        Note("rope bridge walk: worst gap " + worstGap + " lateral " + worstOff + " x " + ch.Position.X);
        Assert.True(ch.Position.X > -4 + 0.8 * 6 * 0.9);
        Assert.True(worstGap < 0.12);
    }

    private static RodDef StiffRope(Vector3D from, Vector3D to, int segments)
    {
        var pts = new Vector3D[segments + 1];
        for (int i = 0; i <= segments; i++) { pts[i] = from + (to - from) * ((double)i / segments); }
        return new RodDef { Points = pts, Material = new RecursiveAI.OnePhysics.Rods.RodMaterial(2e10f, 8e9f, 0.003f, 1000f), PinnedParticles = [0] };
    }

    [Fact]
    public void ACharacterHangingFromARopeSwingsAsTheCombinedPendulum()
    {
        var w = NewWorld();
        var pivot = new Vector3D(0, 6, 0);
        double theta0 = 10.0 * Math.PI / 180.0;
        const float L = 3f;
        var tip = new Vector3D(Math.Sin(theta0) * L, 6 - Math.Cos(theta0) * L, 0);
        var rope = w.AddRod(StiffRope(pivot, tip, 30));
        var ch = w.AddCharacter(new Vector3D(0, 3, 0), CharacterDef.Humanoid);
        ch.Grab(rope, 30f);
        Assert.Equal(CharacterMode.Roped, ch.Mode);
        double dt = Dt;
        var xs = new System.Collections.Generic.List<double>();
        var crossings = new System.Collections.Generic.List<double>();
        double prev = double.NaN;
        for (int i = 0; i < 60 * 14; i++)
        {
            w.Step(Dt);
            var g = rope.GetPosition(30);
            double x = g.X - pivot.X;
            if (!double.IsNaN(prev) && prev < 0 && x >= 0) { crossings.Add((i - 1 + (-prev) / (x - prev)) * dt); }
            prev = x;
            xs.Add(x);
        }

        Assert.True(crossings.Count >= 3, "crossings " + crossings.Count);
        double period = (crossings[crossings.Count - 1] - crossings[0]) / (crossings.Count - 1);
        var gp = rope.GetPosition(30);
        double length = Math.Sqrt((gp.X - pivot.X) * (gp.X - pivot.X) + (gp.Y - pivot.Y) * (gp.Y - pivot.Y));
        double m = 70.0, mr = rope.Particles.Positions.Length * 0.0; _ = mr;
        double rodMass = 1000.0 * Math.PI * 0.003 * 0.003 * L;
        double t0 = 2 * Math.PI * Math.Sqrt(length * (m + rodMass / 3) / (G * (m + rodMass / 2)));
        double predicted = t0 * (1 + theta0 * theta0 / 16.0);
        Note("pendulum period " + period + " predicted " + predicted + " length " + length);
        Assert.InRange(period / predicted, 0.98, 1.02);
    }

    [Fact]
    public void LettingGoOfARopeKeepsTheSwingVelocity()
    {
        var w = NewWorld();
        var pivot = new Vector3D(0, 6, 0);
        var rope = w.AddRod(StiffRope(pivot, new Vector3D(Math.Sin(0.5) * 3, 6 - Math.Cos(0.5) * 3, 0), 30));
        var ch = w.AddCharacter(new Vector3D(0, 3, 0), CharacterDef.Humanoid);
        ch.Grab(rope, 30f);
        for (int i = 0; i < 70; i++) { w.Step(Dt); }
        var grip = rope.GetVelocity(30);
        var v = ch.Release();
        Note("release velocity " + v + " grip " + grip);
        Assert.Equal(CharacterMode.Free, ch.Mode);
        Assert.True((v - grip).Length() < 1e-4f);
        Assert.True(v.Length() > 1f);
        // Energy: the speed at the lowest part of the swing is sqrt(2 g L (1 - cos theta)) at most.
        Assert.True(v.Length() <= MathF.Sqrt(2 * G * 3f * (1f - MathF.Cos(0.5f))) * 1.05f);
        // The rod lets go of the weight: it swings freely afterwards, lighter at the tip.
        Assert.True(rope.Particles.InverseMass[30] > 1f / 1f);
    }

    private static double TopY(SoftBody pad)
    {
        double best = double.MinValue;
        for (int i = 0; i < pad.ParticleCount; i++) { best = Math.Max(best, pad.GetPosition(i).Y); }
        return best;
    }

    private static CharacterDef Climber()
    {
        var def = CharacterDef.Humanoid;
        def.CanClimb = true;
        return def;
    }

    private static readonly Vector3F Ahead = new(0, 0, 1);

    /// <summary>A character whose hands are exactly at height <paramref name="top"/>, standing 0.35 m from a wall at z = 2 (the capsule centre is 0.8 m below the hands).</summary>
    private static Character AtLedge(PhysicsWorld3D w, double top, CharacterDef? def = null) => w.AddCharacter(new Vector3D(0, top - 0.8, 1.65), def ?? Climber());

    public enum LedgeKind { Box, Mesh, HeightField, Voxel }

    private static void AddLedge(PhysicsWorld3D w, LedgeKind kind, double top)
    {
        switch (kind)
        {
            case LedgeKind.Box:
                w.AddBox(new Vector3F(8f, (float)top, 4f), at: new Vector3D(0, top / 2, 4), type: BodyType.Static);
                break;
            case LedgeKind.Mesh:
            {
                float t = (float)top;
                Vector3F[] v = [new(-4, 0, 2), new(4, 0, 2), new(4, 0, 6), new(-4, 0, 6), new(-4, t, 2), new(4, t, 2), new(4, t, 6), new(-4, t, 6)];
                int[] ix = [0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3, 3, 7, 4, 3, 4, 0, 4, 7, 6, 4, 6, 5, 0, 1, 2, 0, 2, 3];
                w.AddMesh(v, ix);
                break;
            }
            case LedgeKind.HeightField:
            {
                const int N = 81;
                var h = new float[N * N];
                for (int j = 0; j < N; j++) { for (int i = 0; i < N; i++) { h[j * N + i] = j >= 20 ? (float)top : 0f; } }
                w.AddHeightField(h, N, N, new Vector3F(0.1f, 1f, 0.1f), new Vector3D(-4, 0, 0));
                break;
            }
            case LedgeKind.Voxel:
            {
                int ny = (int)Math.Round(top / 0.5);
                var occ = new bool[16 * ny * 8];
                Array.Fill(occ, true);
                w.AddVoxelGrid(16, ny, 8, occ, 0.5f, new Vector3D(-4, 0, 2));
                break;
            }
        }
    }

    [Theory]
    [InlineData(LedgeKind.Box)]
    [InlineData(LedgeKind.Mesh)]
    [InlineData(LedgeKind.HeightField)]
    [InlineData(LedgeKind.Voxel)]
    public void ALedgeIsGrabbedHungAndClimbedOnEveryKindOfGeometry(LedgeKind kind)
    {
        var w = NewWorld();
        Floor(w);
        AddLedge(w, kind, 2.0);
        var ch = AtLedge(w, 2.0 - 0.05);
        Assert.True(ch.TryGrabLedge(Ahead), "no ledge found on " + kind);
        Assert.Equal(CharacterMode.Hanging, ch.Mode);
        // It hangs against the wall with its hands at the ledge, and stays there.
        var p = ch.Position;
        for (int i = 0; i < 60; i++) { w.Step(Dt); }
        var q = ch.Position;
        Assert.True(Math.Abs(p.Y - q.Y) < 1e-6 && Math.Abs(p.Z - q.Z) < 1e-6);
        Assert.InRange(q.Z, 2 - 0.3 - 0.04, 2 - 0.3 - 0.0);
        Assert.True(ch.ClimbUp(), "climb up failed on " + kind);
        Assert.Equal(CharacterMode.Free, ch.Mode);
        var d = new Driver(ch);
        for (int i = 0; i < 60; i++) { d.Prepare(); w.Step(Dt); }
        Assert.True(ch.IsGrounded);
        Assert.InRange(ch.Position.Y, 2.89, 2.95);
        Assert.True(ch.Position.Z > 2.0);
    }

    [Fact]
    public void ThingsThatAreNotLedgesAreNotGrabbed()
    {
        var w = NewWorld();
        Floor(w);
        // A wall far taller than the hands reach, a wall that ends below the hands, a ramp, and open air.
        w.AddBox(new Vector3F(8f, 6f, 4f), at: new Vector3D(0, 3, 4), type: BodyType.Static);
        var tall = AtLedge(w, 2.0);
        Assert.False(tall.TryGrabLedge(Ahead));
        Assert.Equal(CharacterMode.Free, tall.Mode);
        var w2 = NewWorld(1, 92);
        Floor(w2);
        w2.AddBox(new Vector3F(8f, 1.0f, 4f), at: new Vector3D(0, 0.5, 4), type: BodyType.Static);
        Assert.False(AtLedge(w2, 2.0).TryGrabLedge(Ahead));
        var w3 = NewWorld(1, 93);
        Floor(w3);
        Assert.False(AtLedge(w3, 2.0).TryGrabLedge(Ahead));
        var w4 = NewWorld(1, 94);
        Floor(w4);
        Vector3F[] v = [new(-4, 0, 2), new(4, 0, 2), new(-4, 3, 8), new(4, 3, 8)];
        w4.AddMesh(v, [0, 2, 1, 1, 2, 3]);
        Assert.False(AtLedge(w4, 1.0).TryGrabLedge(Ahead), "a ramp is not a ledge");
        // Without the permission nothing is grabbed.
        var w5 = NewWorld(1, 95);
        Floor(w5);
        AddLedge(w5, LedgeKind.Box, 2.0);
        Assert.False(AtLedge(w5, 1.95, CharacterDef.Humanoid).TryGrabLedge(Ahead));
    }

    [Fact]
    public void AHangingCharacterShimmiesAlongTheLedgeAndStopsAtItsEnd()
    {
        var w = NewWorld();
        Floor(w);
        AddLedge(w, LedgeKind.Box, 2.0);
        var ch = AtLedge(w, 1.95);
        Assert.True(ch.TryGrabLedge(Ahead));
        double x0 = ch.Position.X;
        for (int i = 0; i < 60; i++) { ch.Move(new Vector3F(0.02f, 0, 0), Dt); w.Step(Dt); }
        Assert.InRange(ch.Position.X - x0, 1.1, 1.3);
        Assert.Equal(CharacterMode.Hanging, ch.Mode);
        for (int i = 0; i < 400; i++) { ch.Move(new Vector3F(0.02f, 0, 0), Dt); w.Step(Dt); }
        // The ledge ends at x = 4: the capsule's centre, with its hands, does not go beyond.
        Assert.InRange(ch.Position.X, 3.5, 4.01);
        Assert.Equal(CharacterMode.Hanging, ch.Mode);
        // A wall jump lets go and heads away from the wall.
        var v = ch.WallJump(4f, 5f);
        Assert.Equal(CharacterMode.Free, ch.Mode);
        Assert.True(v.Z < -3.9f && v.Y == 5f);
    }

    [Fact]
    public void AHangerOnAMovingPlatformGoesWithIt()
    {
        var w = NewWorld();
        var platform = w.AddBox(new Vector3F(8f, 2f, 4f), at: new Vector3D(0, 1, 4), type: BodyType.Kinematic);
        var ch = AtLedge(w, 1.95);
        Assert.True(ch.TryGrabLedge(Ahead));
        var local0 = ch.Position - platform.Position;
        for (int i = 0; i < 240; i++)
        {
            platform.LinearVelocity = new Vector3F(0.5f, 0.2f * MathF.Sin(i * 0.05f), 0.3f);
            platform.AngularVelocity = new Vector3F(0, 0.3f, 0);
            w.Step(Dt);
        }

        // In the platform's frame the hands have not moved.
        var q = platform.Rotation.Conjugated();
        var rel = ch.Position - platform.Position;
        var l1 = q.Rotate(new Vector3F((float)rel.X, (float)rel.Y, (float)rel.Z));
        double turned = Math.Sqrt((double)(l1.X * l1.X + l1.Z * l1.Z));
        double was = Math.Sqrt(local0.X * local0.X + local0.Z * local0.Z);
        Assert.InRange(turned, was - 1e-3, was + 1e-3);
        Assert.Equal(CharacterMode.Hanging, ch.Mode);
    }

    [Fact]
    public void AClimbableWallIsClimbedAndOverTheTop()
    {
        var w = NewWorld();
        Floor(w);
        var wall = w.AddBox(new Vector3F(4f, 4f, 1f), at: new Vector3D(0, 2, 2.5), type: BodyType.Static);
        var ch = w.AddCharacter(new Vector3D(0, 0.9, 1.65), Climber());
        // Not climbable yet.
        Assert.False(ch.TryClimb(Ahead));
        var shape = wall.GetShapes()[0];
        shape.Climbable = true;
        Assert.True(shape.Climbable);
        Assert.True(ch.TryClimb(Ahead));
        Assert.Equal(CharacterMode.Climbing, ch.Mode);
        double y0 = ch.Position.Y;
        for (int i = 0; i < 60; i++) { ch.Climb(0.02f); w.Step(Dt); }
        Assert.InRange(ch.Position.Y - y0, 1.15, 1.25);
        Assert.InRange(ch.Position.Z, 1.65, 1.72);
        // Sideways along the wall.
        double x0 = ch.Position.X;
        for (int i = 0; i < 30; i++) { ch.Move(new Vector3F(0.02f, 0, 0), Dt); w.Step(Dt); }
        Assert.InRange(ch.Position.X - x0, 0.55, 0.65);
        // Up and over the top (the wall is 4 m high).
        for (int i = 0; i < 400 && ch.Mode == CharacterMode.Climbing; i++) { ch.Climb(0.02f); w.Step(Dt); }
        Assert.Equal(CharacterMode.Free, ch.Mode);
        var d = new Driver(ch);
        for (int i = 0; i < 60; i++) { d.Prepare(); w.Step(Dt); }
        Assert.True(ch.IsGrounded);
        Assert.InRange(ch.Position.Y, 4.89, 4.96);
        // Climbing down to the ground lets go.
        var ch2 = w.AddCharacter(new Vector3D(1.5, 1.5, 1.65), Climber());
        Assert.True(ch2.TryClimb(Ahead));
        for (int i = 0; i < 200 && ch2.Mode == CharacterMode.Climbing; i++) { ch2.Climb(-0.02f); w.Step(Dt); }
        Assert.Equal(CharacterMode.Free, ch2.Mode);
    }

    private static Vector3D DeckLocal(Body deck, Vector3D p)
    {
        var q = deck.Rotation.Conjugated();
        var o = deck.Position;
        var l = q.Rotate(new Vector3F((float)(p.X - o.X), (float)(p.Y - o.Y), (float)(p.Z - o.Z)));
        return new Vector3D(l.X, l.Y, l.Z);
    }
}
