# RecursiveAI.OnePhysics.Interchange

Importers for [RecursiveAI.OnePhysics](https://github.com/recursive-ai-oss/onephysics-csharp): robots, scenes and maps from the formats other tools write, as plain definitions. The same bytes give the same definitions on every platform, and a hostile file is refused with a typed error that names where it went wrong.

```csharp
using RecursiveAI.OnePhysics.Interchange;

var robot = UrdfImporter.Import(urdfText);                       // or MjcfImporter.Import(mjcfText)
var arm = world.AddArticulation(robot.Definition);

var gltf = GltfPhysicsImporter.Import(glbBytes);                 // KHR_physics_rigid_bodies, OMI_physics_body
var world3 = gltf.Scene.CreateWorld3D();

var map = TiledImporter.Import(tmjText);                         // or LdtkImporter.Import(json, "Level_0")
map.Apply(world2);                                               // tile grids (slopes, one-way platforms) and shapes
```

Every importer returns what it could not express as warnings. The specification (unit conversions, mappings, conventions) is in [docs/spec/interchange.md](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/interchange.md).

Licence `MIT OR Apache-2.0`. Copyright (c) Recursive AI, LLC.
