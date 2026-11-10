#pragma warning disable OPHYS0001, OPHYS0002, OPHYS0003
using System;
using System.IO;
using RecursiveAI.OneMath;                       // Vector3D
using RecursiveAI.OnePhysics.Interchange;        // the importers
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/interchange.md (compiled, not run: they read files).</summary>
public static class Interchange
{
    private static InterchangeMesh? LoadMesh(string name) => null;

    public static P3.PhysicsWorld3D Import(P3.PhysicsWorld3D world, P2.PhysicsWorld2D world2)
    {
        // A robot: URDF or MJCF to an articulation.
        var robot = UrdfImporter.Import(File.ReadAllText("arm.urdf"), new UrdfOptions { Meshes = name => LoadMesh(name) });
        var arm = world.AddArticulation(robot.Definition, at: new Vector3D(0, 0, 0));
        arm.SetJointTarget(robot.FindLink("elbow"), 0.5f);
        foreach (var w in robot.Warnings) { Console.WriteLine(w.Code + ": " + w.Message); }

        // A glTF scene with physics extensions to the public scene format, then a world.
        var gltf = GltfPhysicsImporter.Import(File.ReadAllBytes("level.glb"));
        var world3 = gltf.Scene.CreateWorld3D();

        // A Tiled map or an LDtk level to tile grids and shapes.
        var map = TiledImporter.Import(File.ReadAllText("level.tmj"));   // or LdtkImporter.Import(json, "Level_0")
        var instance = map.Apply(world2);
        return world3;
    }
}
