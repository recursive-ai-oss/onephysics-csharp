using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class Character3DTests
{
    private const float Dt = 1f / 60f;
    private const float G = 9.81f;

    private static PhysicsWorld3D NewWorld(int threads = 1, int id = 71) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    /// <summary>Static floor with its top at y = 0.</summary>
    private static Body Floor(PhysicsWorld3D w, float half = 50f) => w.AddBox(new Vector3F(2 * half, 1f, 2 * half), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);

    /// <summary>Drives a character the way a game would: horizontal velocity plus gravity integrated into a vertical speed that resets on the ground.</summary>
    private sealed class Driver
    {
        public readonly Character C;
        public float Vx, Vy, Vz;
        public CharacterContact Last;

        public Driver(Character c) { C = c; }

        public void Tick(PhysicsWorld3D w)
        {
            Prepare();
            w.Step(Dt);
        }

        /// <summary>The move without the step, for scenes with several characters and one world step.</summary>
        public void Prepare()
        {
            Vy = C.IsGrounded && Vy < 0f ? -1f : Vy - G * Dt;
            Last = C.Move(new Vector3F(Vx * Dt, Vy * Dt, Vz * Dt), Dt);
        }
    }

    private static Driver Spawn(PhysicsWorld3D w, double x, double y, double z, CharacterDef? def = null) => new(w.AddCharacter(new Vector3D(x, y, z), def ?? Character.Humanoid));

    private static void Run(PhysicsWorld3D w, Driver d, int steps) { for (int i = 0; i < steps; i++) { d.Tick(w); } }

    /// <summary>A ramp rising along +z over a floor: <paramref name="angleTan"/> is rise over run, 20 m wide, starting at z = 2.</summary>
    private static Body Ramp(PhysicsWorld3D w, float angleTan, float run = 12f)
    {
        float h = angleTan * run;
        Vector3F[] v = [new(-10, 0, 2), new(10, 0, 2), new(-10, h, 2 + run), new(10, h, 2 + run)];
        return w.AddMesh(v, [0, 2, 1, 1, 2, 3]);
    }

    [Fact]
    public void ACharacterLandsOnTheFloorAndStandsStill()
    {
        var w = NewWorld();
        Floor(w);
        var d = Spawn(w, 0, 3, 0);
        Run(w, d, 120);
        Assert.True(d.C.IsGrounded);
        Assert.Equal(GroundState.OnGround, d.C.Ground);
        Assert.Equal(1f, d.C.GroundNormal.Y, 4);
        Assert.InRange(d.C.Position.Y, 0.9 - 1e-4, 0.9 + 0.002);
        var p0 = d.C.Position;
        double worst = 0;
        for (int i = 0; i < 300; i++)
        {
            var before = d.C.Position;
            d.Tick(w);
            var after = d.C.Position;
            worst = Math.Max(worst, Math.Max(Math.Abs(after.X - before.X), Math.Max(Math.Abs(after.Y - before.Y), Math.Abs(after.Z - before.Z))));
        }

        Assert.True(worst <= 1e-5, "standing character moved by " + worst);
        Assert.Equal(p0.Y, d.C.Position.Y, 4);
        Assert.True((d.Last & CharacterContact.Below) != 0);
    }

    [Fact]
    public void SlidingAlongAWallKeepsTheTangentialSpeed()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector3F(1f, 10f, 40f), at: new Vector3D(6, 5, 0), type: BodyType.Static); // face at x = 5.5
        var d = Spawn(w, 0, 0.95, 0);
        d.Vx = 3f; d.Vz = 2f;
        Run(w, d, 120); // reaches the wall and slides
        Assert.InRange(d.C.Position.X, 5.5 - 0.3 - 0.01 - 0.003, 5.5 - 0.3 - 0.01 + 0.003);
        double z0 = d.C.Position.Z;
        Run(w, d, 60);
        double speedZ = (d.C.Position.Z - z0) / 1.0;
        Assert.InRange(speedZ, 2.0 * 0.995, 2.0 * 1.005);
        Assert.True((d.Last & CharacterContact.Sides) != 0);
    }

    [Theory]
    [InlineData(0.25f, true)]
    [InlineData(0.35f, false)]
    public void AStepAtTheOffsetIsClimbedAndOneAboveIsNot(float height, bool climbs)
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector3F(20f, height, 4f), at: new Vector3D(0, height / 2, 5), type: BodyType.Static); // step face at z = 3
        var d = Spawn(w, 0, 0.95, 0);
        d.Vz = 1.5f;
        Run(w, d, 200);
        if (climbs)
        {
            Assert.True(d.C.Position.Z > 4.5);
            Assert.InRange(d.C.Position.Y, height + 0.9 - 0.01, height + 0.9 + 0.01);
            Assert.True(d.C.IsGrounded);
        }
        else
        {
            Assert.InRange(d.C.Position.Z, 3 - 0.3 - 0.01 - 0.01, 3 - 0.3 - 0.01 + 0.01);
            Assert.InRange(d.C.Position.Y, 0.9 - 0.01, 0.9 + 0.01);
        }
    }

    [Theory]
    [InlineData(44f, true)]
    [InlineData(46f, false)]
    public void TheSlopeLimitHoldsToAboutADegree(float degrees, bool climbs)
    {
        var w = NewWorld();
        Floor(w);
        float tan = (float)(Math.Sin(degrees * Math.PI / 180) / Math.Cos(degrees * Math.PI / 180));
        Ramp(w, tan);
        var d = Spawn(w, 0, 0.95, -2);
        d.Vz = 2f;
        Run(w, d, 240);
        if (climbs)
        {
            Assert.True(d.C.Position.Y > 2.5, "climbed to " + d.C.Position.Y);
            Assert.True(d.C.IsGrounded);
        }
        else
        {
            Assert.True(d.C.Position.Y < 0.9 + 0.35, "reached " + d.C.Position.Y);
            Assert.False(d.C.IsGrounded && d.C.Position.Y > 1.2);
        }
    }

    [Fact]
    public void ASteepSlopeIsReportedAndTheCharacterSlidesDownIt()
    {
        var w = NewWorld();
        Floor(w);
        float tan = 1.4f; // 54 degrees
        Ramp(w, tan, 6f);
        // Drop the character on the ramp: it cannot stand and slides down towards the floor.
        var d = Spawn(w, 0, 6.0, 4.0);
        bool sawSteep = false;
        for (int i = 0; i < 200; i++)
        {
            d.Tick(w);
            sawSteep |= d.C.Ground == GroundState.OnSteepGround;
        }

        Assert.True(sawSteep);
        Assert.True(d.C.Position.Z < 3.0, "slid to z " + d.C.Position.Z);
    }

    [Fact]
    public void LandingOnAMovingPlatformCarriesTheCharacter()
    {
        var w = NewWorld();
        Floor(w);
        var platform = w.AddBox(new Vector3F(8f, 0.5f, 8f), at: new Vector3D(0, 0.75, 0), type: BodyType.Kinematic); // top at 1.0
        platform.LinearVelocity = new Vector3F(1f, 0f, 0f);
        var d = Spawn(w, 0, 3, 0);
        Run(w, d, 90);
        Assert.True(d.C.IsGrounded);
        Assert.Equal(platform, d.C.GroundBody);
        Assert.True(Math.Abs(d.C.GroundVelocity.X - 1f) < 1e-4f);
        double x0 = d.C.Position.X, px0 = platform.Position.X;
        Run(w, d, 60);
        Assert.InRange(d.C.Position.X - x0, 0.98, 1.02);
        Assert.InRange(platform.Position.X - px0, 0.98, 1.02);
        Assert.InRange(d.C.Position.Y, 1.9 - 0.01, 1.9 + 0.01);
    }

    [Fact]
    public void ARotatingPlatformCarriesTheCharacterAround()
    {
        var w = NewWorld();
        Floor(w);
        var platform = w.AddCylinder(0.25f, 6f, new Vector3D(0, 0.75, 0), type: BodyType.Kinematic); // top at 1.0
        platform.AngularVelocity = new Vector3F(0, 0.5f, 0);
        var d = Spawn(w, 3, 1.95, 0);
        Run(w, d, 5);
        double a0 = Math.Atan2(d.C.Position.Z, d.C.Position.X);
        Run(w, d, 120);
        double a1 = Math.Atan2(d.C.Position.Z, d.C.Position.X);
        // 2 s at 0.5 rad/s about +y turns x towards -z: the angle falls by 1 rad.
        Assert.InRange(a0 - a1, 0.9, 1.1);
        Assert.True(d.C.IsGrounded);
        double r = Math.Sqrt(d.C.Position.X * d.C.Position.X + d.C.Position.Z * d.C.Position.Z);
        Assert.InRange(r, 2.9, 3.4);
    }

    [Fact]
    public void PushingACrateMovesItAndAHeavyCrateIsNotMovedThrough()
    {
        var w = NewWorld();
        Floor(w);
        var light = w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(0, 0.3, 3), density: 100f); // 21.6 kg
        var heavy = w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(10, 0.3, 3), density: 1e6f); // 216 t
        var def = Character.Humanoid;
        def.PushStrength = 1f;
        var a = Spawn(w, 0, 0.95, 0, def);
        var b = Spawn(w, 10, 0.95, 0, def);
        a.Vz = 1.0f; b.Vz = 1.0f;
        for (int i = 0; i < 300; i++) { a.Prepare(); b.Prepare(); w.Step(Dt); }
        Assert.True(light.Position.Z > 4.5, "light crate at " + light.Position.Z);
        Assert.InRange(heavy.Position.Z, 2.99, 3.05);
        // The character stands in front of the heavy crate's face (z = 2.7), a radius and a skin short of it.
        Assert.InRange(b.C.Position.Z, 2.7 - 0.3 - 0.02, 2.7 - 0.3 + 0.002);
    }

    [Fact]
    public void ACharacterIsPushedOutOfACrateDroppedOnIt()
    {
        var w = NewWorld();
        Floor(w);
        var d = Spawn(w, 0, 0.95, 0);
        Run(w, d, 30);
        w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(0.35, 0.3, 0), density: 100f);
        Run(w, d, 60);
        var p = d.C.Position;
        // No overlap remains between the capsule axis and the crate's footprint.
        Assert.True(Math.Abs(p.X - 0.35) >= 0.3 + 0.3 - 0.03 || Math.Abs(p.Z) >= 0.3 + 0.3 - 0.03 || p.Y > 1.0, "still inside the crate at " + p);
    }

    [Fact]
    public void ACharacterWalksAcrossMeshSeamsWithoutSnagging()
    {
        var w = NewWorld();
        // A flat mesh floor cut into 1 m cells (many internal edges), top at y = 0.
        var verts = new List<Vector3F>();
        var idx = new List<int>();
        const int n = 40;
        for (int j = 0; j <= n; j++) { for (int i = 0; i <= n; i++) { verts.Add(new Vector3F(i - n / 2, 0, j - n / 2)); } }
        for (int j = 0; j < n; j++)
        {
            for (int i = 0; i < n; i++)
            {
                int a = j * (n + 1) + i, b = a + 1, c = a + n + 1, dd = c + 1;
                if (((i + j) & 1) == 0) { idx.AddRange([a, c, b, b, c, dd]); } else { idx.AddRange([a, c, dd, a, dd, b]); }
            }
        }

        w.AddMesh(verts.ToArray(), idx.ToArray());
        var d = Spawn(w, -15, 0.95, -15);
        d.Vx = 3f; d.Vz = 1.7f;
        Run(w, d, 20);
        var last = d.C.Position;
        double worstDeviation = 0;
        for (int i = 0; i < 300; i++)
        {
            d.Tick(w);
            var p = d.C.Position;
            double dx = p.X - last.X, dz = p.Z - last.Z;
            worstDeviation = Math.Max(worstDeviation, Math.Max(Math.Abs(dx - 3.0 * Dt), Math.Abs(dz - 1.7 * Dt)));
            Assert.True(Math.Abs(p.Y - 0.9) < 0.004, "height " + p.Y + " at step " + i);
            Assert.True(d.C.IsGrounded);
            last = p;
        }

        Assert.True(worstDeviation < 2e-4, "seam deviation " + worstDeviation);
    }

    [Fact]
    public void ABlockStepFieldOfVoxelChunksIsWalkable()
    {
        var w = NewWorld();
        // 1 m blocks whose tops rise 0.25 m per column: a mesh of top faces and side walls, like a voxel chunk.
        var verts = new List<Vector3F>();
        var idx = new List<int>();
        const int cols = 12, rowsZ = 6;
        void Quad(Vector3F a, Vector3F b, Vector3F c, Vector3F d2)
        {
            int k = verts.Count;
            verts.AddRange([a, b, c, d2]);
            idx.AddRange([k, k + 1, k + 2, k, k + 2, k + 3]);
        }

        for (int i = 0; i < cols; i++)
        {
            float h = 0.25f * (i + 1), x0 = i, x1 = i + 1;
            for (int j = 0; j < rowsZ; j++)
            {
                float z0 = j, z1 = j + 1;
                Quad(new(x0, h, z0), new(x0, h, z1), new(x1, h, z1), new(x1, h, z0)); // top, normal up
                if (j == 0) { Quad(new(x0, 0, z0), new(x0, h, z0), new(x1, h, z0), new(x1, 0, z0)); }
                if (i == 0 || true) { Quad(new(x0, 0, z0), new(x0, 0, z1), new(x0, h, z1), new(x0, h, z0)); } // wall at x0 facing -x
            }
        }

        w.AddMesh(verts.ToArray(), idx.ToArray());
        // A floor under the first column so the character starts level.
        w.AddBox(new Vector3F(8f, 1f, 8f), at: new Vector3D(-4, -0.5, 3), type: BodyType.Static);
        var d = Spawn(w, -1, 0.95, 3);
        d.Vx = 1.5f;
        for (int i = 0; i < 700 && d.C.Position.X < cols - 0.6; i++) { d.Tick(w); }
        Assert.True(d.C.Position.X > cols - 0.7, "reached x " + d.C.Position.X);
        Assert.InRange(d.C.Position.Y, 0.25 * cols + 0.9 - 0.02, 0.25 * cols + 0.9 + 0.02);
        Assert.True(d.C.IsGrounded);
    }

    [Fact]
    public void ACharacterClimbsAndDescendsStairsAndStaysOnThem()
    {
        var w = NewWorld();
        Floor(w);
        for (int i = 0; i < 20; i++)
        {
            w.AddBox(new Vector3F(10f, 0.2f * (i + 1), 0.3f), at: new Vector3D(0, 0.1 * (i + 1), 3 + 0.3 * i + 0.15), type: BodyType.Static);
        }

        w.AddBox(new Vector3F(10f, 4f, 10f), at: new Vector3D(0, 2.0, 3 + 0.3 * 20 + 5), type: BodyType.Static); // the top landing
        var d = Spawn(w, 0, 0.95, 0);
        d.Vz = 1.2f;
        Run(w, d, 400);
        Assert.True(d.C.Position.Z > 3 + 0.3 * 19, "top reached? z = " + d.C.Position.Z);
        Assert.InRange(d.C.Position.Y, 4.0 + 0.9 - 0.02, 4.0 + 0.9 + 0.02);
        // Back down: the character keeps its feet on the stairs (grounded on almost every step).
        d.Vz = -1.2f;
        int airborne = 0;
        for (int i = 0; i < 1100; i++)
        {
            d.Tick(w);
            if (d.C.Ground == GroundState.InAir) { airborne++; }
        }

        Assert.True(d.C.Position.Y < 1.0, "back down at y " + d.C.Position.Y);
        Assert.True(airborne <= 60, "airborne " + airborne + " steps while descending");
    }

    [Fact]
    public void ACeilingStopsAJump()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector3F(10f, 1f, 10f), at: new Vector3D(0, 3.0, 0), type: BodyType.Static); // underside at 2.5
        var d = Spawn(w, 0, 0.95, 0);
        Run(w, d, 20);
        d.Vy = 6f;
        bool above = false;
        for (int i = 0; i < 40; i++)
        {
            d.C.Move(new Vector3F(0, d.Vy * Dt, 0), Dt);
            w.Step(Dt);
            above |= (d.C.Position.Y > 1.0) && d.C.Position.Y <= 2.5 - 0.9 + 1e-3;
            Assert.True(d.C.Position.Y <= 2.5 - 0.9 + 1e-3, "head through the ceiling at " + d.C.Position.Y);
            d.Vy -= G * Dt;
        }

        Assert.True(above);
    }

    [Fact]
    public void CharactersAreObstaclesToEachOther()
    {
        var w = NewWorld();
        Floor(w);
        var a = Spawn(w, 0, 0.95, 0);
        var b = Spawn(w, 0, 0.95, 3);
        a.Vz = 2f;
        for (int i = 0; i < 180; i++) { a.Prepare(); b.Prepare(); w.Step(Dt); }
        Assert.InRange(a.C.Position.Z, 3 - 0.6 - 0.02, 3 - 0.6 + 0.001);
        Assert.InRange(b.C.Position.Z, 3 - 1e-4, 3 + 1e-4);
    }

    [Fact]
    public void AnUpDirectionOtherThanYStandsOnAFloorAlongThatAxis()
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { WorldId = 72, Gravity = new Vector3F(0, 0, -9.81f) });
        w.AddBox(new Vector3F(100f, 100f, 1f), at: new Vector3D(0, 0, -0.5), type: BodyType.Static);
        var def = Character.Humanoid;
        def.Up = new Vector3F(0, 0, 1);
        var c = w.AddCharacter(new Vector3D(0, 0, 3), def);
        float vz = 0f;
        for (int i = 0; i < 150; i++)
        {
            vz = c.IsGrounded && vz < 0f ? -1f : vz - G * Dt;
            c.Move(new Vector3F(1f * Dt, 0, vz * Dt), Dt);
            w.Step(Dt);
        }

        Assert.True(c.IsGrounded);
        Assert.Equal(0.9, c.Position.Z, 2);
        Assert.Equal(1f, c.GroundNormal.Z, 3);
        Assert.True(c.Position.X > 1.0);
        Assert.Equal(1f, c.Up.Z, 5);
    }

    [Fact]
    public void ARigidCharacterWalksStandsAndRespectsTheSlopeLimit()
    {
        var w = NewWorld();
        Floor(w);
        var c = w.AddCharacter(new Vector3D(0, 1.0, -4), Character.RigidHumanoid);
        Assert.Equal(CharacterKind.Rigid, c.Kind);
        Assert.InRange(c.Body.Mass, 69f, 71f);
        for (int i = 0; i < 90; i++) { c.Move(default, Dt); w.Step(Dt); }
        Assert.True(c.IsGrounded);
        Assert.InRange(c.Position.Y, 0.88, 0.93);
        var p0 = c.Position;
        for (int i = 0; i < 120; i++) { c.Move(new Vector3F(0, 0, 2f * Dt), Dt); w.Step(Dt); }
        Assert.InRange(c.Position.Z - p0.Z, 3.8, 4.1);
        // A 50 degree ramp cannot be walked up.
        Ramp(w, 1.19f, 6f);
        for (int i = 0; i < 240; i++) { c.Move(new Vector3F(0, 0, 2f * Dt), Dt); w.Step(Dt); }
        Assert.True(c.Position.Y < 1.3, "climbed to " + c.Position.Y);
    }

    [Fact]
    public void ARigidCharacterClimbsAGentleSlopeAndHoldsStillOnIt()
    {
        var w = NewWorld();
        Floor(w);
        Ramp(w, 0.5f, 10f); // 26.6 degrees
        var c = w.AddCharacter(new Vector3D(0, 1.0, 0), Character.RigidHumanoid);
        for (int i = 0; i < 60; i++) { c.Move(default, Dt); w.Step(Dt); }
        for (int i = 0; i < 240; i++) { c.Move(new Vector3F(0, 0, 1.5f * Dt), Dt); w.Step(Dt); }
        Assert.True(c.Position.Y > 2.5, "climbed to " + c.Position.Y);
        Assert.True(c.IsGrounded);
        var held = c.Position;
        for (int i = 0; i < 120; i++) { c.Move(default, Dt); w.Step(Dt); }
        Assert.True((c.Position.Y - held.Y) > -0.01 && Math.Abs(c.Position.Z - held.Z) < 0.01, "slid by " + (c.Position.Z - held.Z));
    }

    // ------------------------------------------------------------------ determinism and snapshots

    private static (PhysicsWorld3D World, Driver[] Drivers) Scene(int threads, int id = 73)
    {
        var w = NewWorld(threads, id);
        Floor(w);
        Ramp(w, 0.5f);
        w.AddBox(new Vector3F(20f, 0.25f, 3f), at: new Vector3D(0, 0.125, -6), type: BodyType.Static);
        var platform = w.AddBox(new Vector3F(5f, 0.5f, 5f), at: new Vector3D(12, 0.25, 0), type: BodyType.Kinematic);
        platform.LinearVelocity = new Vector3F(0, 0, 0.5f);
        for (int i = 0; i < 6; i++) { w.AddBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(-3 + i * 1.5, 0.3, 8), density: 50f); }
        var drivers = new Driver[8];
        for (int i = 0; i < drivers.Length; i++)
        {
            var def = Character.Humanoid;
            def.PushStrength = i % 2 == 0 ? 0.8f : 0f;
            drivers[i] = new Driver(w.AddCharacter(new Vector3D(-4 + i * 1.4, 1.5 + 0.2 * i, -8), def)) { Vz = 0.8f + 0.2f * (i % 4), Vx = (i - 3.5f) * 0.1f };
        }

        drivers[7] = new Driver(w.AddCharacter(new Vector3D(12, 1.9, 0), Character.RigidHumanoid)) { Vz = 0.5f };
        return (w, drivers);
    }

    private static void TickAll(PhysicsWorld3D w, Driver[] ds)
    {
        foreach (var d in ds)
        {
            if (d.C.Kind == CharacterKind.Rigid) { d.Last = d.C.Move(new Vector3F(d.Vx * Dt, 0, d.Vz * Dt), Dt); }
            else
            {
                d.Vy = d.C.IsGrounded && d.Vy < 0f ? -1f : d.Vy - G * Dt;
                d.Last = d.C.Move(new Vector3F(d.Vx * Dt, d.Vy * Dt, d.Vz * Dt), Dt);
            }
        }

        w.Step(Dt);
    }

    [Fact]
    public void OneThreadAndManyThreadsGiveTheSameWorld()
    {
        var (w1, d1) = Scene(1);
        var (w4, d4) = Scene(4);
        for (int i = 0; i < 240; i++)
        {
            TickAll(w1, d1); TickAll(w4, d4);
            Assert.Equal(w1.ComputeHash(), w4.ComputeHash());
        }
    }

    [Fact]
    public void SnapshotMidMoveRestoresByteExactly()
    {
        var (w, ds) = Scene(1);
        for (int i = 0; i < 100; i++) { TickAll(w, ds); }
        var snap = w.Snapshot();
        var hashes = new List<ulong>();
        for (int i = 0; i < 120; i++) { TickAll(w, ds); hashes.Add(w.ComputeHash()); }
        var finalBytes = w.Snapshot();
        w.Restore(snap);
        Assert.True(w.Snapshot().AsSpan().SequenceEqual(snap));
        for (int i = 0; i < 120; i++) { TickAll(w, ds); Assert.Equal(hashes[i], w.ComputeHash()); }
        Assert.True(w.Snapshot().AsSpan().SequenceEqual(finalBytes));

        // A fresh world of the same layout takes the snapshot and continues identically (the handles keep working).
        var (fresh, fd) = Scene(1);
        fresh.Restore(snap);
        Assert.True(fresh.Snapshot().AsSpan().SequenceEqual(snap));
        for (int i = 0; i < 120; i++) { TickAll(fresh, fd); Assert.Equal(hashes[i], fresh.ComputeHash()); }
    }

    [Fact]
    public void DamagedCharacterBlocksAreRefusedAndLeaveTheWorldAlone()
    {
        var (w, ds) = Scene(1);
        for (int i = 0; i < 30; i++) { TickAll(w, ds); }
        var snap = w.Snapshot();
        ulong before = w.ComputeHash();
        int bad = 0;
        // Flip single bytes in the tail of the snapshot, where the characters block lives; the world must either refuse or hold a different valid state, never throw anything else.
        for (int i = snap.Length - 1; i > snap.Length - 3000; i -= 7)
        {
            var copy = (byte[])snap.Clone();
            copy[i] ^= 0x5A;
            try { w.Restore(copy); w.Restore(snap); }
            catch (SnapshotFormatException) { bad++; Assert.Equal(before, w.ComputeHash()); }
        }

        Assert.True(bad > 0);
        w.Restore(snap);
        Assert.Equal(before, w.ComputeHash());
    }

    [Fact]
    public void CharactersDoNotChangeAWorldWithoutThem()
    {
        var a = NewWorld(1, 74);
        var b = NewWorld(1, 74);
        Floor(a); Floor(b);
        a.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(0, 3, 0));
        b.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(0, 3, 0));
        for (int i = 0; i < 60; i++) { a.Step(Dt); b.Step(Dt); }
        Assert.Equal(a.ComputeHash(), b.ComputeHash());
        Assert.True(a.Snapshot().AsSpan().SequenceEqual(b.Snapshot()));
    }

    // ------------------------------------------------------------------ handles and hostile input

    [Fact]
    public void ADestroyedCharacterFailsClearlyAndItsBodyGoesWithIt()
    {
        var w = NewWorld();
        Floor(w);
        var c = w.AddCharacter(new Vector3D(0, 1, 0));
        var body = c.Body;
        Assert.True(w.TryGetCharacter(body, out var again));
        Assert.Equal(c, again);
        c.Destroy();
        Assert.False(c.IsValid);
        Assert.False(body.IsValid);
        Assert.Throws<StaleHandleException>(() => c.Move(default, Dt));
        Assert.Throws<StaleHandleException>(() => _ = c.Position);
        Assert.Throws<InvalidHandleException>(() => _ = default(Character).Radius);
        Assert.Empty(w.GetCharacters());
        w.Step(Dt);
    }

    [Fact]
    public void HostileCharacterInputGivesTypedErrors()
    {
        var w = NewWorld();
        Floor(w);
        var c = w.AddCharacter(new Vector3D(0, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector3F(float.NaN, 0, 0), Dt));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector3F(0, float.PositiveInfinity, 0), Dt));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(new Vector3F(1e6f, 0, 0), Dt));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(default, 0f));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(default, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Move(default, -1f));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.SlopeLimit = 91f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.StepOffset = 100f);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Position = new Vector3D(double.NaN, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(new Vector3D(1e12, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Radius = -1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Height = 0.1f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Up = default }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Up = new Vector3F(float.NaN, 1, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Mass = 0f }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Kind = (CharacterKind)7 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.AddCharacter(default, new CharacterDef { Layer = 40 }));
        Assert.Throws<ArgumentNullException>(() => Mover3D.Move(null!, Character.Humanoid, ref StateRef(), default, Dt));
        Assert.Single(w.GetCharacters());
        w.Step(Dt);
        Assert.True(float.IsFinite((float)c.Position.Y));
    }

    private static MoverState3D s_state;

    private static ref MoverState3D StateRef() => ref s_state;

    [Fact]
    public void ARigidCharacterIsDrivenWithoutAStructCallbackAndTheHitHandlerFires()
    {
        var w = NewWorld();
        Floor(w);
        w.AddBox(new Vector3F(1f, 10f, 40f), at: new Vector3D(3, 5, 0), type: BodyType.Static);
        var c = w.AddCharacter(new Vector3D(0, 0.95, 0));
        int hits = 0;
        Action<ControllerColliderHit> handler = h => { if (h.Normal.X < -0.9f) { hits++; } Assert.Equal(c, h.Character); };
        c.OnControllerColliderHit += handler;
        for (int i = 0; i < 120; i++) { c.MoveVelocity(new Vector3F(2f, -1f, 0), Dt); w.Step(Dt); }
        Assert.True(hits > 0);
        int seen = hits;
        c.OnControllerColliderHit -= handler;
        for (int i = 0; i < 10; i++) { c.MoveVelocity(new Vector3F(2f, -1f, 0), Dt); w.Step(Dt); }
        Assert.Equal(seen, hits);
    }

    [Fact]
    public void TheDebugGeometryListsCharacters()
    {
        var w = NewWorld();
        Floor(w);
        var c = w.AddCharacter(new Vector3D(0, 1, 0));
        for (int i = 0; i < 60; i++) { c.Move(new Vector3F(0, -1f * Dt, 0), Dt); w.Step(Dt); }
        var g = new RecursiveAI.OnePhysics.Debug.DebugGeometry();
        w.GetDebugGeometry(RecursiveAI.OnePhysics.Debug.DebugContent.Characters, g);
        Assert.Single(g.Characters.ToArray());
        Assert.True(g.Characters[0].IsGrounded);
    }

    [Fact]
    public void CharacterAndDefinitionExposeTheSameMembersAs2DExceptTheListedOnes()
    {
        static HashSet<string> Names(Type t) => t.GetMembers(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.DeclaredOnly)
            .Where(m => m.MemberType != System.Reflection.MemberTypes.Constructor).Select(m => m.Name.StartsWith("get_", StringComparison.Ordinal) || m.Name.StartsWith("set_", StringComparison.Ordinal) ? m.Name[4..] : m.Name).ToHashSet();
        // Dimension-intrinsic, 3D only: the up direction and the rigid kind (2D is always upright in y and has no rigid variant here).
        string[] only3 = ["Up", "Kind", "RigidHumanoid"];
        foreach (var (two, three) in new[] { (typeof(RecursiveAI.OnePhysics.Physics2D.Character), typeof(Character)), (typeof(RecursiveAI.OnePhysics.Physics2D.CharacterDef), typeof(CharacterDef)) })
        {
            var a = Names(two); var b = Names(three);
            Assert.Empty(a.Except(b));
            Assert.Empty(b.Except(a).Except(only3));
        }
    }

    [Fact]
    public void TheSharedCharactersSceneIsThreadAndRestoreIndependent()
    {
        // The scene the hash matrix and smoke apps run: every path must end in the same hash.
        ulong a = Characters3DScene.Run(1, false), b = Characters3DScene.Run(4, false), c = Characters3DScene.Run(1, true);
        Assert.Equal(a, b);
        Assert.Equal(a, c);
    }
}
