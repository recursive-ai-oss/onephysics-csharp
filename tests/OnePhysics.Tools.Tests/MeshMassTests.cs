using System;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class MeshMassTests
{
    [Fact]
    public void BoxMatchesAnalytic()
    {
        var m = TestMeshes.Box(2, 3, 4, 10, -5, 7);
        var mp = MeshMass.ComputeDouble(m.V, m.I);
        Assert.Equal(24.0, mp.Mass, 9);
        Assert.Equal(10.0, mp.Center.X, 9); Assert.Equal(-5.0, mp.Center.Y, 9); Assert.Equal(7.0, mp.Center.Z, 9);
        // Inertia of a box of unit density about its centre: m/12 (b^2 + c^2) per axis.
        Assert.Equal(24.0 / 12 * (9 + 16), mp.Inertia.M11, 7);
        Assert.Equal(24.0 / 12 * (4 + 16), mp.Inertia.M22, 7);
        Assert.Equal(24.0 / 12 * (4 + 9), mp.Inertia.M33, 7);
        Assert.Equal(0.0, mp.Inertia.M12, 7);
    }

    [Fact]
    public void SphereMatchesAnalyticWithinTessellation()
    {
        var m = TestMeshes.Sphere(1.5f, 48, 96);
        var mp = MeshMass.ComputeDouble(m.V, m.I);
        double vol = 4.0 / 3 * Math.PI * 1.5 * 1.5 * 1.5;
        Assert.InRange(mp.Mass / vol, 0.995, 1.0);
        Assert.InRange(mp.Inertia.M11 / (0.4 * vol * 1.5 * 1.5), 0.99, 1.0);
        Assert.InRange(mp.Inertia.M22 / (0.4 * vol * 1.5 * 1.5), 0.99, 1.0);
        Assert.Equal(0.0, mp.Center.Length(), 6);
    }

    [Fact]
    public void CylinderMatchesAnalyticWithinTessellation()
    {
        var m = TestMeshes.Cylinder(1f, 2f, 128);
        var mp = MeshMass.ComputeDouble(m.V, m.I);
        double vol = Math.PI * 1 * 4;
        Assert.InRange(mp.Mass / vol, 0.999, 1.0);
        // Axis y: I = m r^2 / 2; transverse: m (3 r^2 + h^2) / 12 with h = 4.
        Assert.InRange(mp.Inertia.M22 / (vol * 0.5), 0.995, 1.0);
        Assert.InRange(mp.Inertia.M11 / (vol * (3 + 16) / 12.0), 0.995, 1.0);
    }

    [Fact]
    public void MatchesTheEngineHullForConvexInput()
    {
        var m = TestMeshes.Box(1.5f, 0.5f, 2.5f, 0.25f, 0.5f, -1f);
        var hull = ConvexHull3D.Build(m.V);
        var a = MeshMass.Compute(m.V, m.I);
        var b = hull.MassProperties;
        Assert.Equal(b.Mass, a.Mass, 5);
        Assert.True(Vector3F.Distance(a.Center, b.Center) < 1e-5f);
        Assert.Equal(b.Inertia.M11, a.Inertia.M11, 4);
        Assert.Equal(b.Inertia.M22, a.Inertia.M22, 4);
        Assert.Equal(b.Inertia.M33, a.Inertia.M33, 4);

        var s = TestMeshes.Sphere(1f, 16, 24);
        var sh = ConvexHull3D.Build(s.V);
        Assert.Equal(sh.MassProperties.Mass, MeshMass.Compute(s.V, s.I).Mass, 4);
    }

    [Fact]
    public void OpenMeshIsRejectedNamingTheEdge()
    {
        var m = TestMeshes.Box(1, 1, 1);
        var idx = m.I[..^3];
        var e = Assert.Throws<GeometryToolException>(() => MeshMass.Compute(m.V, idx));
        Assert.Equal(GeometryToolError.OpenMesh, e.Error);
        Assert.Contains("edge", e.Message);
        Assert.False(MeshMass.IsClosed(m.V, idx));
        Assert.True(MeshMass.IsClosed(m.V, m.I));
    }

    [Fact]
    public void InvertedAndNonManifoldMeshesAreRejected()
    {
        var m = TestMeshes.Box(1, 1, 1);
        var idx = m.I;
        for (int t = 0; t < idx.Length; t += 3) { (idx[t + 1], idx[t + 2]) = (idx[t + 2], idx[t + 1]); }
        Assert.Equal(GeometryToolError.InvertedWinding, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(m.V, idx)).Error);

        var dup = new int[m.I.Length + 3];
        m.I.CopyTo(dup, 0);
        m.I.AsSpan(0, 3).CopyTo(dup.AsSpan(m.I.Length));
        Assert.Equal(GeometryToolError.NonManifold, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(m.V, dup)).Error);
    }

    [Fact]
    public void HostileInputsGetTypedErrors()
    {
        var v = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0) };
        Assert.Equal(GeometryToolError.BadIndex, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(v, new[] { 0, 1, 3 })).Error);
        Assert.Equal(GeometryToolError.BadIndex, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(v, new[] { 0, 1 })).Error);
        Assert.Equal(GeometryToolError.Empty, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(v, ReadOnlySpan<int>.Empty)).Error);
        Assert.Equal(GeometryToolError.Empty, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(ReadOnlySpan<Vector3F>.Empty, new[] { 0, 0, 0 })).Error);
        var nan = new[] { new Vector3F(float.NaN, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0) };
        Assert.Equal(GeometryToolError.NonFinite, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(nan, new[] { 0, 1, 2 })).Error);
        var big = new[] { new Vector3F(1e9f, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0) };
        Assert.Equal(GeometryToolError.OutOfRange, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(big, new[] { 0, 1, 2 })).Error);
        Assert.Equal(GeometryToolError.NonManifold, Assert.Throws<GeometryToolException>(() => MeshMass.Compute(v, new[] { 0, 0, 1 })).Error);
    }
}
