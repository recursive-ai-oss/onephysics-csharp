# Big worlds: streamed terrain, level of detail and slow motion

```csharp
var world = new PhysicsWorld3D();

// Terrain in tiles: the host builds each tile's geometry, the world builds the bodies at the start of the next step.
world.LoadTile(new TileKey(3, 7), tileDef);          // a TileDef3D: height fields, meshes, boxes, voxel grids around an origin
world.UnloadTile(new TileKey(0, 7));                  // waits until nothing rests on it (or pass TileUnloadPolicy.Immediate)

// Far islands step less often: within 30 m every step, to 80 m every second, to 150 m every fourth, then every eighth.
world.SetSimulationLod([player], [new LodBand(30, 1), new LodBand(80, 2), new LodBand(150, 4)], farPeriod: 8);
world.SetLodFocus([player]);                         // each frame, as the player moves

// Slow motion for one body and for a region.
body.TimeScale = 0.25f;
world.SetTimeScaleRegions([new TimeScaleRegion3D(min, max, 0.5f)]);
```

- **Seams.** Give each height-field tile the heights its neighbours would have just outside its edge (`TileHeightField3D.Edges`, in 2D `HeightBefore` and `HeightAfter`) and a body rolls across the border as over one field. Without them every border edge is active and shows as a bump.
- **Order.** Calls within a step may come in any order: tiles are built in key order, so every peer gets the same world. Bodies you create after a `LoadTile` come after that tile's bodies.
- **What an unload does to a resting body.** By default nothing: the tile stays until the contact ends. A body that sleeps on a tile keeps it loaded; `Immediate` drops the support.
- **Level of detail** applies to rigid bodies, contacts and joints. Islands that touch a kinematic body or belong to a vehicle or machine always step every step; characters, soft bodies, ropes and fluids run at the full rate. A body entering a slow region joins its rhythm at the next due step (its clock can move by up to period − 1 steps), keeping its velocity.
- **Time scale** applies per island, at the smallest scale of its bodies: a fast body that hits a slow one runs with it for as long as they touch, and momentum is conserved. Scales go from 1/1024 to 4.
- Everything here is part of the snapshot, the hash, `Fork`, replays and scenes; a region snapshot leaves the receiving world's tiles and regions alone.

The normative description is in `docs/spec/worldscale.md`.
