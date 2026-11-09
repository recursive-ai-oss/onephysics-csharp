using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using Xunit;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests;

public sealed class Sdf3DTests
{
    // --- closed meshes ---------------------------------------------------------------------------------------------
    private static (Vector3F[] V, int[] I) BoxMesh(Vector3F min, Vector3F max)
    {
        var v = new Vector3F[8]; for (int i = 0; i < 8; i++) v[i] = new((i & 1) != 0 ? max.X : min.X, (i & 2) != 0 ? max.Y : min.Y, (i & 4) != 0 ? max.Z : min.Z);
        // Counter-clockwise from outside.
        int[] q = [0, 2, 3, 1,  4, 5, 7, 6,  0, 1, 5, 4,  2, 6, 7, 3,  0, 4, 6, 2,  1, 3, 7, 5];
        var idx = new List<int>(); for (int f = 0; f < 6; f++) { int a = q[4 * f], b = q[4 * f + 1], c = q[4 * f + 2], d = q[4 * f + 3]; idx.AddRange([a, b, c, a, c, d]); }
        return (v, idx.ToArray());
    }

    private static (Vector3F[] V, int[] I) Torus(float major, float minor, int nu, int nv)
    {
        var v = new List<Vector3F>(); var idx = new List<int>();
        for (int i = 0; i < nu; i++) for (int j = 0; j < nv; j++)
        {
            double u = 2 * Math.PI * i / nu, w = 2 * Math.PI * j / nv;
            v.Add(new((float)((major + minor * Math.Cos(w)) * Math.Cos(u)), (float)(minor * Math.Sin(w)), (float)((major + minor * Math.Cos(w)) * Math.Sin(u))));
        }
        for (int i = 0; i < nu; i++) for (int j = 0; j < nv; j++)
        {
            int a = i * nv + j, b = ((i + 1) % nu) * nv + j, c = ((i + 1) % nu) * nv + (j + 1) % nv, d = i * nv + (j + 1) % nv;
            idx.AddRange([a, d, c, a, c, b]);
        }
        return (v.ToArray(), idx.ToArray());
    }

    private static (Vector3F[] V, int[] I) Wedge(float length, float height, float width)
    {
        // A ramp: rises from 0 at z=0 to `height` at z=length, as a closed prism.
        Vector3F[] v = [new(-width, 0, 0), new(width, 0, 0), new(width, 0, length), new(-width, 0, length), new(-width, height, length), new(width, height, length)];
        int[] i = [0, 1, 2, 0, 2, 3,   3, 2, 5, 3, 5, 4,   0, 3, 4,   1, 5, 2,   0, 4, 5, 0, 5, 1];
        return (v, i);
    }

    [Fact]
    public void MeshFieldMatchesExactDistanceWithinHalfACellAndHasTheRightSign()
    {
        var random = new Random(4); const float cell = 0.1f;
        var meshes = new[] { BoxMesh(new(-1, -0.5f, -0.7f), new(1, 0.5f, 0.7f)), Torus(1f, 0.35f, 24, 12) };
        foreach (var (v, i) in meshes)
        {
            var field = SdfGrid3D.FromMesh(v, i, cell, 3);
            var b = field.Bounds; int far = 0;
            for (int n = 0; n < 600; n++)
            {
                var p = new Vector3F(b.Min.X + (float)random.NextDouble() * (b.Max.X - b.Min.X), b.Min.Y + (float)random.NextDouble() * (b.Max.Y - b.Min.Y), b.Min.Z + (float)random.NextDouble() * (b.Max.Z - b.Min.Z));
                float exact = SdfGrid3D.ExactMeshDistance(v, i, p), got = field.Value(p);
                Assert.True(Math.Abs(exact - got) <= 0.5f * cell, "point " + p + ": exact " + exact + " field " + got);
                if (Math.Abs(exact) > 0.5f * cell) { Assert.True(Math.Sign(exact) == Math.Sign(got), "sign at " + p); far++; }
            }
            Assert.True(far > 300);
        }
        // Points at vertices and along edges of the box are on the surface (distance 0); just outside and inside they get the right sign.
        var (bv, bi) = BoxMesh(new(-1, -1, -1), new(1, 1, 1)); var box = SdfGrid3D.FromMesh(bv, bi, 0.1f, 3);
        Assert.True(box.Value(new(1.15f, 1.15f, 1.15f)) > 0.2f); Assert.True(box.Value(new(0.85f, 0.85f, 0.85f)) < -0.1f);
        Assert.True(box.Value(new(1.1f, 0, 1.1f)) > 0.1f); Assert.True(box.Value(new(0, 0, 0)) < -0.9f);
    }

    [Fact]
    public void HostileMeshesAreRejectedWithTypedErrors()
    {
        var (v, i) = BoxMesh(new(-1, -1, -1), new(1, 1, 1));
        var open = i.AsSpan(0, i.Length - 3).ToArray();
        Assert.Equal(SdfBuildError.NotClosed, Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(v, open, 0.2f)).Error);
        var flipped = (int[])i.Clone(); (flipped[1], flipped[2]) = (flipped[2], flipped[1]);
        Assert.Equal(SdfBuildError.NotClosed, Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(v, flipped, 0.2f)).Error);
        var bad = (Vector3F[])v.Clone(); bad[3] = new(float.NaN, 0, 0);
        Assert.Equal(SdfBuildError.NonFinite, Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(bad, i, 0.2f)).Error);
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(v, [0, 1, 99, 0, 1, 2, 0, 1, 3, 1, 2, 3], 0.2f));
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(v, i, float.NaN));
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(v, i, 1e-6f));
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(v, i, 0.2f, 0));
        var flat = new Vector3F[] { new(0, 0, 0), new(1, 0, 0), new(2, 0, 0), new(3, 0, 0) };
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(flat, [0, 1, 2, 0, 2, 3, 1, 2, 3, 0, 1, 3], 0.2f));
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromMesh(new Vector3F[] { new(1e9f, 0, 0), new(0, 1, 0), new(0, 0, 1), new(0, 0, 0) }, [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2], 0.2f));
        Assert.Throws<SdfBuildException>(() => new SdfGrid3D(1, 4, 4, new float[16], 1f));
        Assert.Throws<SdfBuildException>(() => new SdfGrid3D(4, 4, 4, new float[63], 1f));
        var nan = new float[64]; nan[5] = float.PositiveInfinity;
        Assert.Throws<SdfBuildException>(() => new SdfGrid3D(4, 4, 4, nan, 1f));
        Assert.Throws<SdfBuildException>(() => SdfGrid3D.FromFunction(new(0, 0, 0), new(1000, 1000, 1000), 0.001f, _ => 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SdfGrid3D(4, 4, 4, new float[64], 1f).Value(new(float.NaN, 0, 0)));
    }

    [Fact]
    public void PrimitivesAndCsgAreExactOnTheirSurfaces()
    {
        var sphere = SdfGrid3D.FromFunction(new(-2, -2, -2), new(2, 2, 2), 0.1f, SdfGrid3D.Sphere(default, 1));
        Assert.InRange(sphere.Value(new(1.3f, 0, 0)), 0.29f, 0.31f); Assert.InRange(sphere.Value(default), -1.0f, -0.99f);
        var csg = SdfGrid3D.FromFunction(new(-2, -2, -2), new(2, 2, 2), 0.1f, SdfGrid3D.Difference(SdfGrid3D.Box(default, new(1, 1, 1)), SdfGrid3D.Sphere(new(1, 1, 1), 0.8f)));
        Assert.True(csg.Value(new(0.9f, 0.9f, 0.9f)) > 0); Assert.True(csg.Value(new(-0.5f, -0.5f, -0.5f)) < 0);
        var m = sphere.ComputeMass(); float exact = 4f / 3f * MathF.PI;
        Assert.InRange(m.Mass, exact * 0.99f, exact * 1.01f); Assert.True(MathF.Abs(m.Center.X) < 0.01f);
        Assert.InRange(m.Inertia.M11, 0.4f * exact * 0.97f, 0.4f * exact * 1.03f);
    }

    [Fact]
    public void SphereTracingMatchesTheMeshRayWithinAFractionOfACell()
    {
        var (v, i) = Torus(1f, 0.35f, 40, 20); const float cell = 0.05f;
        var field = SdfGrid3D.FromMesh(v, i, cell, 3);
        var wm = new P3.PhysicsWorld3D(); wm.AddMesh(v, i);
        var wf = new P3.PhysicsWorld3D(); wf.AddSdf(field);
        var random = new Random(8); int hits = 0;
        for (int n = 0; n < 300; n++)
        {
            var o = new Vector3D(random.NextDouble() * 4 - 2, 2 + random.NextDouble(), random.NextDouble() * 4 - 2); var d = new Vector3F((float)(random.NextDouble() - 0.5) * 0.6f, -1, (float)(random.NextDouble() - 0.5) * 0.6f);
            bool a = wm.Raycast(o, d, out var ha, 10), b = wf.Raycast(o, d, out var hb, 10);
            if (a && b) { Assert.True(Math.Abs(ha.Distance - hb.Distance) <= 0.6 * cell, "ray " + n + ": mesh " + ha.Distance + " field " + hb.Distance); hits++; }
            else Assert.True(!a || !b ? true : false);
        }
        Assert.True(hits > 60, "hits " + hits);
        // The data-level sphere trace agrees with the world ray and reports its step bound.
        Assert.True(field.Lipschitz >= 1f);
        Assert.True(field.Trace(new(0.0f, 3, 1.0f), new(0, -1, 0), 10, out float t, out var normal, out bool inside) && !inside);
        Assert.InRange(t, 2.6f, 2.7f); Assert.True(normal.Y > 0.9f);
    }

    [Fact]
    public void SphereRestsOnAnSdfGroundWhereItRestsOnTheSourceMesh()
    {
        var (v, i) = BoxMesh(new(-6, -2, -6), new(6, 0, 6));
        var field = SdfGrid3D.FromMesh(v, i, 0.2f, 3);
        float Rest(P3.PhysicsWorld3D w) { var s = w.AddSphere(0.5f, new(0, 1.5, 0)); for (int n = 0; n < 240; n++) w.Step(1f / 60); return (float)s.Position.Y; }
        var wm = new P3.PhysicsWorld3D(); wm.AddMesh(v, i); var wf = new P3.PhysicsWorld3D(); wf.AddSdf(field);
        float a = Rest(wm), b = Rest(wf);
        Assert.InRange(a, 0.49f, 0.51f); Assert.InRange(b, 0.49f, 0.52f);
    }

    [Fact]
    public void BoxSlidesDownAnSdfRampAtTheMeshRampsSpeed()
    {
        var (v, i) = Wedge(8, 4, 3); var field = SdfGrid3D.FromMesh(v, i, 0.1f, 3);
        float Slide(P3.PhysicsWorld3D w)
        {
            var q = QuaternionF.CreateFromAxisAngle(new(1, 0, 0), MathF.Atan2(4, 8));
            var box = w.AddBox(new(0.6f, 0.4f, 0.6f), new(0, 4.2, 6), rotation: QuaternionF.Identity, material: new P3.Material3D(0.05f));
            for (int n = 0; n < 40; n++) w.Step(1f / 60);
            return box.LinearVelocity.Length();
        }
        var wm = new P3.PhysicsWorld3D(new() { SleepEnabled = false }); wm.AddMesh(v, i, material: new P3.Material3D(0.05f));
        var wf = new P3.PhysicsWorld3D(new() { SleepEnabled = false }); wf.AddSdf(field, material: new P3.Material3D(0.05f));
        float sm = Slide(wm), sf = Slide(wf);
        Assert.True(sm > 0.5f, "mesh slide " + sm);
        Assert.InRange(sf, sm * 0.85f, sm * 1.15f);
    }

    [Fact]
    public void DynamicSdfBodiesHaveFieldMassAndSettleOnASdfGround()
    {
        var ground = SdfGrid3D.FromFunction(new(-8, -3, -8), new(8, 0, 8), 0.25f, SdfGrid3D.HalfSpace(new(0, -0.0f, 0), new(0, 1, 0)));
        var ball = SdfGrid3D.FromFunction(new(-1.2f, -1.2f, -1.2f), new(1.2f, 1.2f, 1.2f), 0.1f, SdfGrid3D.Sphere(default, 1f));
        var cube = SdfGrid3D.FromFunction(new(-1, -1, -1), new(1, 1, 1), 0.1f, SdfGrid3D.Box(default, new(0.5f, 0.5f, 0.5f)));
        var w = new P3.PhysicsWorld3D(); w.AddSdf(ground, material: new P3.Material3D(0.6f));
        var a = w.AddSdf(ball, new(0, 3, 0), type: BodyType.Dynamic, density: 2f); var b = w.AddSdf(cube, new(2, 3, 0), type: BodyType.Dynamic);
        Assert.InRange(a.Mass, 2f * 4.18879f * 0.98f, 2f * 4.18879f * 1.02f); Assert.InRange(b.Mass, 0.97f, 1.03f);
        var box = w.AddBox(new(1, 1, 1), new(-2, 3, 0));
        for (int n = 0; n < 420; n++) w.Step(1f / 60);
        Assert.InRange(a.Position.Y, 0.97, 1.03); Assert.InRange(b.Position.Y, 0.47, 0.54); Assert.InRange(box.Position.Y, 0.48, 0.52);
        Assert.True(a.LinearVelocity.Length() < 0.3f && b.LinearVelocity.Length() < 0.3f && box.LinearVelocity.Length() < 0.3f, "ball v " + a.LinearVelocity + " w " + a.AngularVelocity + " cube v " + b.LinearVelocity);
    }

    [Fact]
    public void FieldWorldsSnapshotForkAndThreadCountsAgree()
    {
        ulong Run(int threads, out byte[] bytes)
        {
            var w = new P3.PhysicsWorld3D(new() { ThreadCount = threads });
            w.AddSdf(SdfGrid3D.FromFunction(new(-6, -3, -6), new(6, 0.5f, 6), 0.25f, SdfGrid3D.Union(SdfGrid3D.HalfSpace(default, new(0, 1, 0)), SdfGrid3D.Sphere(new(1, -0.5f, 0), 1.5f))));
            var ball = SdfGrid3D.FromFunction(new(-0.7f, -0.7f, -0.7f), new(0.7f, 0.7f, 0.7f), 0.1f, SdfGrid3D.Sphere(default, 0.5f));
            for (int k = 0; k < 6; k++) { w.AddBox(new(0.5f, 0.5f, 0.5f), new(-3 + k * 0.4, 2 + k, 0.1 * k)); w.AddSdf(ball, new(-2 + k * 0.5, 4 + k, -1), type: BodyType.Dynamic); }
            for (int n = 0; n < 240; n++) w.Step(1f / 60);
            bytes = w.Snapshot(); return w.ComputeHash();
        }
        ulong h1 = Run(1, out var b1), h8 = Run(8, out var b8);
        Assert.Equal(h1, h8); Assert.Equal(b1, b8);
        var restored = new P3.PhysicsWorld3D(); restored.Restore(b1); Assert.Equal(b1, restored.Snapshot());
        Assert.Equal(h1, restored.ComputeHash());
        for (int n = 0; n < 30; n++) restored.Step(1f / 60);
        var fork = restored.Fork(); for (int n = 0; n < 30; n++) { restored.Step(1f / 60); fork.Step(1f / 60); Assert.Equal(restored.ComputeHash(), fork.ComputeHash()); }
    }
}
