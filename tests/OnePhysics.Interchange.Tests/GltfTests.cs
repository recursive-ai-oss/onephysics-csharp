using System;
using System.Globalization;
using System.Linq;
using System.Text;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;
using SceneFormat = RecursiveAI.OnePhysics.Scene;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Interchange.Tests;

public sealed class GltfTests
{
    private static string D(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    private static SceneFormat.Scene HandBuilt()
    {
        var s = new SceneFormat.Scene { Dim = 3, Gx = 0, Gy = -9.81, Gz = 0 };
        s.Materials.Add((0.6, 0.0)); s.Materials.Add((0.5, 0.1));
        s.Bodies.Add(new SceneFormat.BodyDef { Type = "static", Y = -0.5, Shapes = { new SceneFormat.ShapeDef { Kind = "box", Material = 1, Density = 1000, P = new[] { 20.0, 0.5, 20.0, 0, 0, 0, 0, 0, 0, 1 } } } });
        for (int i = 1; i < 20; i++)
        {
            double x = (i % 5) * 0.9 - 1.8, y = 0.4 + (i / 5) * 0.8, z = (i % 3) * 0.7 - 0.7;
            var b = new SceneFormat.BodyDef { Type = "dynamic", X = x, Y = y, Z = z, MassOverride = (float)(1 + 0.25 * i) };
            b.Shapes.Add(i <= 9 ? new SceneFormat.ShapeDef { Kind = "box", Material = 1, Density = 1000, P = new[] { 0.25, 0.25, 0.25, 0, 0, 0, 0, 0, 0, 1 } }
                : i <= 14 ? new SceneFormat.ShapeDef { Kind = "sphere", Material = 1, Density = 1000, Radius = 0.3, P = new[] { 0.0, 0, 0 } }
                : new SceneFormat.ShapeDef { Kind = "capsule", Material = 1, Density = 1000, Radius = 0.2, P = new[] { 0.0, -0.3, 0, 0, 0.3, 0 } });
            s.Bodies.Add(b);
        }

        s.Joints.Add(new SceneFormat.JointDef { Kind = "point", A = 8, B = 9 });
        return s;
    }

    private static ulong Hash(SceneFormat.Scene s, int steps)
    {
        var w = s.CreateWorld3D(1);
        for (int i = 0; i < steps; i++) { w.Step(1f / 60f); }
        return w.ComputeHash();
    }

    [Fact]
    public void ImportedSceneEqualsTheHandBuiltOne()
    {
        var model = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(InterchangeSamples.Twenty()));
        Assert.Empty(model.Warnings);
        Assert.Equal(20, model.Scene.Bodies.Count);
        Assert.Single(model.Scene.Joints);
        Assert.Equal(Hash(HandBuilt(), 240), Hash(model.Scene, 240));
        // Different shapes make the check meaningful.
        var other = HandBuilt(); other.Bodies[3].Shapes[0].P[0] = 0.3;
        Assert.NotEqual(Hash(other, 240), Hash(model.Scene, 240));
    }

    [Fact]
    public void SameBytesGiveTheSameScene()
    {
        var bytes = Encoding.UTF8.GetBytes(InterchangeSamples.Twenty());
        Assert.Equal(Hash(GltfPhysicsImporter.Import(bytes).Scene, 60), Hash(GltfPhysicsImporter.Import(bytes).Scene, 60));
    }
}
