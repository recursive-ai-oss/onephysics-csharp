# RecursiveAI.OnePhysics.Tools

Authoring-time geometry for [RecursiveAI.OnePhysics](https://github.com/recursive-ai-oss/onephysics-csharp): turn meshes and sprites into shapes a physics world can use. Nothing here is needed at runtime. Deterministic: the same input gives the same bits on every platform and at every thread count.

```csharp
using RecursiveAI.OnePhysics.Tools;

// Concave mesh -> compound of convex hulls.
var result = ConvexDecomposer.Decompose(vertices, indices, new ConvexDecompositionOptions { MaxHulls = 16 });
var body = world.AddCompound(result.ToCompoundParts(), at: new Vector3D(0, 5, 0));

// Clean a mesh, then get its mass properties.
var clean = MeshCleaner.Clean(vertices, indices);
var mass = MeshMass.Compute(clean.Vertices, clean.Indices);

// Polygon (with holes) and sprite -> convex pieces for polygon shapes.
var pieces = PolygonDecomposer.Decompose(outline, holes);
var sprite = SpriteOutliner.ToConvexPieces(alpha, width, height);
```

The tools: `ConvexDecomposer`, `HullSimplifier`, `MeshCleaner`, `MeshMass`, `PolygonDecomposer`, `SpriteOutliner`. Bad input raises `GeometryToolException` with an error kind and a message that names the value. The specification is in [docs/spec/tools.md](https://github.com/recursive-ai-oss/onephysics-csharp/blob/main/docs/spec/tools.md).

Licence `MIT OR Apache-2.0`. Copyright (c) Recursive AI, LLC.
