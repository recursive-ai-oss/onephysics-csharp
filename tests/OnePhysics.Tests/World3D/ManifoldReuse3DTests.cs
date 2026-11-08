using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World3D;

public class ManifoldReuse3DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld3D Scene(bool reuse, int threads = 1)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 91 });
        w.Engine.RecycleManifolds = reuse;
        w.AddBox(new Vector3F(40f, 1f, 40f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        // A small tilted stack and a few balls: face contacts, an edge contact while settling, and ball contacts whose normals belong to no face.
        for (int i = 0; i < 4; i++) { w.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(0.03 * i, 0.5 + 1.01 * i, 0.02 * i)); }
        w.AddBox(new Vector3F(1f, 0.5f, 1f), at: new Vector3D(3, 0.7, 0), rotation: new QuaternionF(0.1f, 0.2f, 0.05f, 0.97f));
        for (int i = 0; i < 3; i++) { w.AddSphere(0.4f, at: new Vector3D(-3 + 0.7 * i, 0.4 + 1.2 * i, 1)); }
        return w;
    }

    [Fact]
    public void IncreasingSpeculativeDistanceInvalidatesEmptyPairClearance()
    {
        PhysicsWorld3D Empty() => new(new WorldOptions3D { WorldId = 91, Gravity = default, SleepEnabled = false });
        var w = Empty();
        var q = new QuaternionF(0f, 0f, (float)Math.Sin(Math.PI / 8), (float)Math.Cos(Math.PI / 8));
        var a = w.AddBox(new Vector3F(0.2f, 4f, 0.2f), default, q);
        var normal = a.Rotation.Rotate(Vector3F.UnitX);
        w.AddBox(new Vector3F(0.2f, 4f, 0.2f), new Vector3D(normal.X * 0.24f, normal.Y * 0.24f, 0), q);
        w.Step(Dt); w.Step(Dt);
        w.Options.SpeculativeDistance = 0.06f;
        var restored = Empty(); restored.Restore(w.Snapshot());
        for (int i = 0; i < 4; i++)
        {
            w.Step(Dt); restored.Step(Dt);
            Assert.Equal(restored.ComputeHash(), w.ComputeHash());
        }
        Assert.Equal(restored.Snapshot(), w.Snapshot());
    }

    [Fact]
    public void AddingAnOffsetMassInvalidatesTheOldSeparatingPlane()
    {
        PhysicsWorld3D Empty() => new(new WorldOptions3D { WorldId = 91, Gravity = default, SleepEnabled = false });
        var w = Empty();
        var q = new QuaternionF(0f, 0f, (float)Math.Sin(Math.PI / 8), (float)Math.Cos(Math.PI / 8));
        var a = w.AddBox(new Vector3F(0.2f, 4f, 0.2f), default, q);
        var normal = a.Rotation.Rotate(Vector3F.UnitX);
        var b = w.AddBox(new Vector3F(0.2f, 4f, 0.2f), new Vector3D(normal.X * 0.24f, normal.Y * 0.24f, 0), q);
        w.Step(Dt); w.Step(Dt); // Keep an empty pair whose oriented boxes are separated inside overlapping AABBs.
        w.CreateSphere(a, 0.1f, new ShapeDef { LocalPosition = new Vector3F(-3, 0, 0), Density = 1000f });
        b.LinearVelocity = normal * -0.2f;
        var restored = Empty();
        restored.Restore(w.Snapshot()); // Restoring clears all transient empty-pair hints.
        for (int i = 0; i < 120; i++)
        {
            w.Step(Dt); restored.Step(Dt);
            Assert.Equal(restored.ComputeHash(), w.ComputeHash());
        }
        Assert.Equal(restored.Snapshot(), w.Snapshot());
    }

    [Fact]
    public void SeparatingPlaneBudgetIgnoresTangentialMotionButRejectsClosingMotion()
    {
        var frame = new Engine3D.SkipFrame3 { X = 1.06f, Qw = 1f, Clear = 0.04f, Reach = 0.867f, Nx = 1f, AxisLength = 1f };
        Assert.True(Engine3D.StillApart(in frame, QuaternionF.Identity, QuaternionF.Identity, 1.06f, 2f, 0f));
        Assert.True(Engine3D.StillApart(in frame, QuaternionF.Identity, QuaternionF.Identity, 3f, 0f, 0f));
        Assert.False(Engine3D.StillApart(in frame, QuaternionF.Identity, QuaternionF.Identity, 1.01f, 0f, 0f));
        frame.Nx = 0f; frame.AxisLength = 0f; // Without an axis, the Euclidean motion budget is the conservative fallback.
        Assert.False(Engine3D.StillApart(in frame, QuaternionF.Identity, QuaternionF.Identity, 1.06f, 2f, 0f));
    }

    [Fact]
    public void SeparatingPlaneBudgetIsConservativeAgainstBoxVertexProjection()
    {
        var random = new Random(9721);
        var normal = new Vector3F(0.6f, 0.8f, 0f);
        var start = normal * 1.46f; // Both half-unit boxes reach 0.7 along this axis; gap 0.06.
        var frame = new Engine3D.SkipFrame3 { X = start.X, Y = start.Y, Qw = 1f, Clear = 0.04f, Reach = 0.867f, Nx = normal.X, Ny = normal.Y, AxisLength = 1f };
        int accepted = 0;
        for (int i = 0; i < 2000; i++)
        {
            float angle = (float)(random.NextDouble() * 0.3 - 0.15);
            var q = new QuaternionF(0f, 0f, (float)Math.Sin(angle * 0.5), (float)Math.Cos(angle * 0.5));
            if ((i & 1) == 0) { q = new QuaternionF(-q.X, -q.Y, -q.Z, -q.W); }
            var delta = new Vector3F((float)(random.NextDouble() * 0.4 - 0.2), (float)(random.NextDouble() * 0.4 - 0.2), (float)(random.NextDouble() * 2 - 1));
            var at = start + delta;
            if (!Engine3D.StillApart(in frame, QuaternionF.Identity, q, at.X, at.Y, at.Z)) { continue; }
            accepted++;
            float min = float.PositiveInfinity;
            for (int v = 0; v < 8; v++)
            {
                var corner = new Vector3F((v & 1) == 0 ? -0.5f : 0.5f, (v & 2) == 0 ? -0.5f : 0.5f, (v & 4) == 0 ? -0.5f : 0.5f);
                float projection = Vector3F.Dot(at + q.Rotate(corner), normal);
                if (projection < min) { min = projection; }
            }

            Assert.True(min - 0.7f > 0.02f, "An accepted cached plane must leave the boxes beyond the speculative distance.");
        }

        Assert.True(accepted > 100);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(8)]
    public void ReusedContactsMatchSerialMaterialWrites(int threads)
    {
        PhysicsWorld3D Build(bool serial)
        {
            var w = new PhysicsWorld3D(new WorldOptions3D { ThreadCount = threads, WorldId = 91, Events = EventMask.All });
            if (serial)
            {
                // The equivalent user mixer keeps all payload writes on the serial matching path.
                w.Options.MaterialMixer = (a, b) => new Material3D(
                    float.Sqrt(a.Friction * b.Friction),
                    a.Restitution > b.Restitution ? a.Restitution : b.Restitution,
                    a.RollingResistance > b.RollingResistance ? a.RollingResistance : b.RollingResistance);
            }

            w.AddBox(new Vector3F(60, 1, 60), new Vector3D(0, -0.5, 0), type: BodyType.Static);
            // More than 64 simultaneous pairs exercise parallel payload writes, including four-point
            // box contacts and one-point sphere contacts. Both worlds use the same sleep settings.
            for (int i = 0; i < 100; i++)
            {
                var at = new Vector3D((i % 10) * 2 - 9, 0.5, (i / 10) * 2 - 9);
                if (i % 2 == 0) { w.AddBox(new Vector3F(1, 1, 1), at); }
                else { w.AddSphere(0.5f, at); }
            }

            return w;
        }

        var parallel = Build(false);
        var serial = Build(true);
        for (int i = 0; i < 120; i++)
        {
            parallel.Step(Dt); serial.Step(Dt);
            Assert.Equal(serial.ComputeHash(), parallel.ComputeHash());
        }

        Assert.Equal(serial.Snapshot(), parallel.Snapshot());
    }

    [Fact]
    public void ReusedManifoldsRestWhereFreshOnesDo()
    {
        var a = Scene(true);
        var b = Scene(false);
        for (int i = 0; i < 240; i++) { a.Step(Dt); b.Step(Dt); }
        // Every body ends where it would have with a fresh manifold every step, to well within the penetration slop, and the world is quiet.
        var ea = a.Engine.Core.Bodies; var eb = b.Engine.Core.Bodies;
        for (int k = 0; k < ea.HighWater; k++)
        {
            if (!ea.IsLive(k)) { continue; }
            var pa = ea.PositionOf(k); var pb = eb.PositionOf(k);
            for (int c = 0; c < 3; c++) { Assert.InRange(pa[c] - pb[c], -0.02, 0.02); }
        }
    }

    [Fact]
    public void ReusedManifoldsSurviveSnapshotAndRestoreBitForBit()
    {
        var a = Scene(true);
        for (int i = 0; i < 60; i++) { a.Step(Dt); }
        var snap = a.Snapshot();
        var hashes = new ulong[40];
        for (int i = 0; i < 40; i++) { a.Step(Dt); hashes[i] = a.ComputeHash(); }
        var b = Scene(true);
        b.Restore(snap);
        for (int i = 0; i < 40; i++) { b.Step(Dt); Assert.Equal(hashes[i], b.ComputeHash()); }
    }

    [Fact]
    public void ReusedManifoldsGiveTheSameBitsAtAnyThreadCount()
    {
        var a = Scene(true, 1);
        var b = Scene(true, 4);
        for (int i = 0; i < 120; i++) { a.Step(Dt); b.Step(Dt); Assert.Equal(a.ComputeHash(), b.ComputeHash()); }
    }
}
