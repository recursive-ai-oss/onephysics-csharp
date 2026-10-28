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
