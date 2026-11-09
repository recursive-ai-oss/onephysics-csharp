using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Fields;
using RecursiveAI.OnePhysics.Physics2D;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.World2D;

// 2D strips: anisotropy, the strain limit, wind and the host hooks.
public class Strip2DTests
{
    private const float Dt = 1f / 60f;

    private static PhysicsWorld2D NewWorld(Action<WorldOptions2D>? tweak = null)
    {
        var o = new WorldOptions2D { ThreadCount = 1, WorldId = 43 };
        tweak?.Invoke(o);
        return new PhysicsWorld2D(o);
    }

    private static void Run(PhysicsWorld2D w, int steps) { for (int i = 0; i < steps; i++) { w.Step(Dt); } }

    private static double MaxStrain(SoftBody b)
    {
        var pos = b.RawPositions;
        var mesh = b.Mesh;
        var tris = mesh.Triangles;
        double worst = 0.0;
        for (int t = 0; t < tris.Length / 3; t++)
        {
            for (int k = 0; k < 3; k++)
            {
                int a = tris[3 * t + k], c = tris[3 * t + (k + 1) % 3];
                double rx = (double)mesh.Vertices[a].X - mesh.Vertices[c].X, ry = (double)mesh.Vertices[a].Y - mesh.Vertices[c].Y;
                double dx = pos[2 * a] - pos[2 * c], dy = pos[2 * a + 1] - pos[2 * c + 1];
                worst = Math.Max(worst, Math.Sqrt(dx * dx + dy * dy) / Math.Sqrt(rx * rx + ry * ry) - 1.0);
            }
        }

        return worst;
    }

    private static SoftBody Hanging(PhysicsWorld2D w, float limit, float along)
    {
        var mesh = SoftBodyMesh.Box(new Vector2F(1f, 0.05f), 20, 1);
        var def = SoftBodyDef.Jelly(mesh, new Vector2D(0.5, 3));
        def.Mass = 20f; def.EdgeCompliance = 5e-3f; def.AreaCompliance = 1e-3f; def.LinearDamping = 0.1f; def.EdgeDamping = 0f; def.AreaDamping = 0f;
        def.Angle = MathF.PI / 2f;
        def.Strip = new StripParams { AlongCompliance = along, AcrossCompliance = 5e-3f, ShearCompliance = 5e-3f, StrainLimit = limit };
        def.PinnedParticles = new[] { 20, 41 };
        return w.AddSoftBody(def);
    }

    [Fact]
    public void StrainLimitHoldsUnderAHeavyHangingLoad()
    {
        var wa = NewWorld(); var a = Hanging(wa, 0f, 5e-3f); Run(wa, 300);
        var wb = NewWorld(); var b = Hanging(wb, 0.05f, 5e-3f); Run(wb, 300);
        Assert.True(MaxStrain(a) > 0.15, "unlimited strip stretches " + MaxStrain(a));
        Assert.True(MaxStrain(b) <= 0.06, "limited strip " + MaxStrain(b));
    }

    [Fact]
    public void ASofterAlongCompliancePullsLonger()
    {
        double Length(float along)
        {
            var w = NewWorld(); var s = Hanging(w, 0f, along); Run(w, 300);
            return Math.Abs(s.GetPosition(0).Y - s.GetPosition(20).Y);
        }

        Assert.True(Length(2e-2f) > Length(1e-4f) + 0.05);
    }

    [Fact]
    public void AStripInWindIsBlownDownwindAndStaysFinite()
    {
        var w = NewWorld();
        w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-50, -50, -1), Maximum = new Vector3D(50, 50, 1), Flow = new Vector3F(0f, 8f, 0f) });
        var s = w.AddStrip(1f, 0.05f, 16, new Vector2D(0.5, 3), 0.05f, pinStart: true);
        var s2 = w.AddStrip(1f, 0.05f, 16, new Vector2D(3.5, 3), 0.05f, pinStart: true);
        s2.Definition.Strip!.Drag = 0f;
        double fastest = 0.0;
        for (int i = 0; i < 300; i++)
        {
            w.Step(Dt);
            for (int k = 0; k < s.ParticleCount; k++) { var v = s.GetVelocity(k); fastest = Math.Max(fastest, Math.Sqrt(v.X * v.X + v.Y * v.Y)); }
        }

        Assert.True(fastest < 40.0, "fastest " + fastest);
        // the free end is lifted by the upward wind against gravity
        Assert.True(s.GetPosition(16).Y > 3.0 - 0.3, "tip " + s.GetPosition(16).Y);
    }

    [Fact]
    public void StripSnapshotRestoresToTheSameBytesAndFuture()
    {
        PhysicsWorld2D Make()
        {
            var w = NewWorld();
            w.AddField(new FieldDefinition { Kind = FieldKind.Area, Direction = default, Minimum = new Vector3D(-50, -50, -1), Maximum = new Vector3D(50, 50, 1), Flow = new Vector3F(3f, 0f, 0f) });
            w.AddStrip(1f, 0.05f, 12, new Vector2D(0.5, 3), 0.1f, pinStart: true);
            return w;
        }

        var a = Make(); Run(a, 30);
        var snap = a.Snapshot();
        var b = Make(); b.Restore(snap);
        Assert.Equal(snap, b.Snapshot());
        for (int i = 0; i < 30; i++) { a.Step(Dt); b.Step(Dt); Assert.Equal(a.ComputeHash(), b.ComputeHash()); }
    }

    [Fact]
    public void StripParametersAreValidated()
    {
        var w = NewWorld();
        var def = SoftBodyDef.Jelly(SoftBodyMesh.Box(new Vector2F(1f, 0.1f), 4, 1));
        def.Strip = new StripParams { StrainLimit = float.NaN };
        Assert.Contains("StrainLimit", Assert.Throws<SoftBodyException>(() => w.AddSoftBody(def)).Message);
        def.Strip = new StripParams { AlongAxis = default };
        Assert.Throws<SoftBodyException>(() => w.AddSoftBody(def));
    }
}
