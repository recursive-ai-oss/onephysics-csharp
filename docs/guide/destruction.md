# Destruction

Breaking things, in 2D and 3D. Everything here is deterministic and part of the snapshot and the world hash; the exact rules are in [../spec/destruction.md](../spec/destruction.md).

## Happy path

- `body.Fracture()` replaces a convex body (a box, a convex hull, a 2D polygon) by Voronoi cells and returns the new bodies. Pass a `FractureDef` for the number of cells (`CellCount`, default 16), where the seeds go (`Pattern`: `Uniform`, `Clustered` around `ImpactPoint`, or your own `Points`) and the `Seed` of the generator. The same seed gives the same cells on every platform. The fragments inherit the body's velocity field, so mass and momentum are conserved.
- `body.SetFracturable(new FractureDef { ImpulseThreshold = 5f })` makes a body break by itself at the end of the step in which a contact impulse (kg m/s) reaches the threshold. `Depth` lets the fragments break again, with `ChildThresholdScale` raising their threshold.
- `body.Slice(point, normal)` (or `world.Slice`) cuts a body by a plane, a line in 2D, into two bodies; it returns them as `Positive` and `Negative`.
- `world.FractureEvents` lists what broke in the last step (kind, the destroyed body's handle, its fragments in `world.FractureFragments`); `world.OnFracture` calls you after the step. The destroyed body's handle is stale.

## Breakable structures

`FractureAsset.FromHull(points, def)` (2D: `FromPolygon`) fractures a shape ahead of time into cells and bonds the cells that touch; `FromCells` takes cells you made yourself, and the `RecursiveAI.OnePhysics.Tools` package's `FractureAuthoring.Prefracture` decomposes a whole mesh or outline first. `world.AddBreakable(asset, at, rotation, new BreakableDef { Strength = 2e5f })` turns it into one body with one shape per cell.

Each step the bonds are loaded by what the solver pushes through them: contact forces, weight and, for a moving body, the inertia of its motion. A bond whose stress (pascals: tension plus shear plus bending) reaches its `Strength` breaks, and the body is replaced by one body per piece that is still connected. On a static body, mark the parts the world holds with `BreakableDef.AnchorParts`: loads flow towards them and a piece that keeps one stays static, so a bridge loses only what is cut free. `world.GetBonds(body)` reports each bond's area and last stress; `DebugContent.Destruction` puts them in the debug geometry.

## Keeping the debris in check

`world.Debris = new DebrisOptions { MaxFragments = 200, SleepToStaticSteps = 120, LifetimeSteps = 1800 }` caps how many fragments the world keeps (the smallest, then the oldest, go first), turns fragments that have slept for two seconds into static bodies, and removes them after thirty. All three default to off. `world.DestructionCounters` and `world.Counters.Fractures` report what happened.

## What can break

Bodies made of sharp convex hulls or boxes (3D) or polygons (2D) with no rounding. Spheres, capsules, meshes and rounded shapes, and bodies that belong to a joint, vehicle, character or ragdoll, are refused with a `FractureException` that says why (a body set to break by itself is simply left whole). Fragment bodies have new handles; handlers subscribed to the old body are gone with it.
