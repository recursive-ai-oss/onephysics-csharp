# Shallow water

A body of shallow water is a grid that simulates waves, currents and wet and dry shores, with bodies that float in it, push it and ride its waves. Use it where you want a lake, a harbour, a river or a wave tank that reacts; use `AddWater` for an infinite flat pool that never moves. The numbers and the scheme are in [../spec/water.md](../spec/water.md).

<!-- example: Guide/Water.cs -->
```csharp
var world = new PhysicsWorld3D();
world.AddBox(new Vector3F(60f, 1f, 60f), at: new Vector3D(10, -0.5, 10), type: BodyType.Static);   // the bed is whatever static shapes lie under the water
var water = world.AddShallowWater(minX: 0, minZ: 0, maxX: 20, maxZ: 20, level: 2.0, cellSize: 0.5f, floor: -1.0);

var crate = world.AddBox(new Vector3F(1f, 1f, 1f), at: new Vector3D(10, 3, 10), density: 500f);   // floats at half depth
world.BodyWaterChanged += e => events++;                                                          // a body entered or left the water

for (int i = 0; i < 600; i++)
{
    if (i == 200) water.AddVolume(0, 0, 2, 20, 20f);   // a wave maker: 20 m³ at the west end
    world.Step(1f / 60f);
}

if (world.QueryWater(new Vector3D(10, 0, 10), out var sample)) { surface = sample.Surface; depth = sample.Depth; }
if (world.RaycastWater(new Vector3D(10, 10, 10), new Vector3F(0, -1, 0), out var hit)) { hitDistance = hit.Distance; }
```

The 2D world is the same along one axis: `world.AddShallowWater(minX, maxX, level, cellSize, floor)` makes a side-view channel in which volumes are areas.

## What to know

- **The bed is sampled once, from the static shapes under each cell's centre, when the water is created** (or given with `ShallowWaterDef.Bed`). Make the terrain first; `ResampleBed()` samples again after you change it. Where nothing is hit the bed is the floor, which is also the lowest the water can go.
- **Cell size sets what you see.** Waves shorter than a few cells are lost; a step takes as many substeps as the fastest wave needs (up to `MaxSubsteps`, 16 by default), so a fine grid over deep water costs more. `docs/spec/water.md` says how time is split.
- **Sides** are walls unless you set `West`, `East`, `South`, `North` on the definition (or `water.SetBoundary`): `Open` lets waves leave, `HeldLevel` is the sea, `Inflow` feeds a fixed discharge. **Sources and drains** add or take water at points.
- **Boats and everything that reads water** (`AddBoat`, thrusters, hovercraft fans, aerodynamic surfaces) see the local surface and current where a shallow water covers them, and the flat water elsewhere.
- **Bodies float and displace.** Awake dynamic bodies get buoyancy and drag from the water around them; sleeping bodies wake when the water moves them. `Displacement` (default 0.5) is how much of a body's volume is added to the water; 1 is physically complete but can make a long hull in a narrow channel ring.
- **Damping** (default 0.1 per second) lets a splash die out; set it to 0 for water that rings for ever, and `BottomFriction` for drag from the bed.
- **Not yet in it:** kinematic bodies, soft bodies, rods and characters do not touch the water; read it with `QueryWater` and apply your own force.
- Water is part of the snapshot, the hash, `Fork`, region snapshots and `ShiftOrigin`, and is in `DebugContent.ShallowWater`.
