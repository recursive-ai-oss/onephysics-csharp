# Editable and sampled terrain

Terrain you can change while the world runs, and terrain described by a distance field. Everything here is deterministic, goes into snapshots and hashes, is recorded by replays and is carried by forks. The normative descriptions are in [collision2d](../spec/collision2d.md) and [collision3d](../spec/collision3d.md).

## Carve a 2D bitmask terrain

<!-- example: Guide/Terrain2D.cs -->
```csharp
var world = new PhysicsWorld2D();
var solid = new bool[128 * 64];                       // x-fastest; true = ground
for (int y = 0; y < 32; y++) for (int x = 0; x < 128; x++) solid[y * 128 + x] = true;
Body ground = world.AddBitmaskTerrain(128, 64, solid, cellSize: 0.25f);

ground.CarveCircle(new Vector2D(10, 8), 2.5);         // a crater; bodies resting on it wake and fall
ground.FillPolygon([new(4, 6), new(8, 6), new(8, 8)]);
Body[] pieces = ground.DetachIslands();               // chunks cut loose become dynamic bodies
```

Each nonempty cell is one sample at its centre and the boundary is the marching-squares contour of the samples, so craters are smoother than cell staircases. An edit rebuilds the boundary only near the changed cells: segments elsewhere keep their shape handles and cached contacts, and bodies far from the edit are not woken. `DetachIslands` keeps pieces that touch the bottom row (`TerrainAnchors`), turns the rest into bodies covered by boxes and returns them in piece order.

## Dig a 3D voxel grid

<!-- example: Guide/Terrain3D.cs -->
```csharp
var world = new PhysicsWorld3D();
Body ground = world.AddVoxelGrid(64, 8, 64, occupancy, cellSize: 0.5f);
Shape voxels = ground.Voxels;

voxels.DigSphere(new Vector3D(10, 2, 10), 1.8);       // cells whose centres are inside
voxels.SetVoxels(14, 0, 14, 4, 4, 4, solid: true);    // a block of cells, in cell coordinates
```

Triangle ids are `12 * cell + 2 * face + half`, so faces nobody touched keep their ids; characters and bodies elsewhere keep their state bit for bit. Bodies near the edit wake and re-find the changed surface.

## Signed distance fields

<!-- example: Guide/Terrain3D.cs -->
```csharp
SdfGrid3D hill = SdfGrid3D.FromFunction(new(-8, -3, -8), new(8, 2, 8), 0.25f,
    SdfGrid3D.Union(SdfGrid3D.HalfSpace(default, new(0, 1, 0)), SdfGrid3D.Sphere(new(0, 0, 0), 2f)));
world.AddSdf(hill);                                   // static terrain

SdfGrid3D ball = SdfGrid3D.FromMesh(vertices, indices, cellSize: 0.1f);   // a closed, consistently oriented mesh
Body body = world.AddSdf(ball, at: new Vector3D(0, 5, 0), type: BodyType.Dynamic);    // mass from the inside volume
```

The field is trilinear between samples; contacts use its gradient. A dynamic field collides with convex shapes and other fields; characters, particles and rods see the field's iso-surface as triangles; rays are sphere-traced. A field is at most 2^22 samples and is copied into every snapshot, so keep the grid as coarse as the contact quality allows. `FromMesh` rejects open, inconsistently wound or degenerate meshes with an `SdfBuildException` naming the problem. In 2D, `SdfGrid2D` and `PhysicsWorld2D.AddSdf` make static terrain from the field's zero contour, or (with `type: BodyType.Dynamic`) a moving body covered by convex polygons that tile the contour, with its mass from the inside area; 2D contacts use those polygons rather than field samples.
