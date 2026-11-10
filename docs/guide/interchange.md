# Importing robots, scenes and maps

`RecursiveAI.OnePhysics.Interchange` turns files other tools write into this library's definitions. Nothing in it touches the file system: you read the file, the importer returns definitions and a list of warnings for what it could not express.

```csharp
using RecursiveAI.OnePhysics.Interchange;

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
```

A file the importer cannot read throws `InterchangeException`, which names the format, the place in the file and what was expected. Ask for nothing else: hostile files are refused with that exception and no other.

The rules each importer follows, the unit conversions, which attributes map where, and what is left out are in [the importer specification](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/interchange.md). The conventions for maps (which layers are collision layers, the `kind`, `oneway` and `body` properties) are there too.
