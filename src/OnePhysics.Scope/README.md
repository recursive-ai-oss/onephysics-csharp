# RecursiveAI.OnePhysics.Scope

The OnePhysics module for the OneScope viewer. It opens a OnePhysics replay (checkpoint or command, 2D or 3D), a world snapshot or a
scene, and lets the viewer play it, seek, fork, poke a fork, diff two runs and inspect bodies and joints. Stepping, snapshots, restore and
the world hash are the library's own, so the viewer's history gives exactly the hashes the replay records.

```csharp
var module = new OnePhysicsModule();
IScopeSource source = module.Open(File.ReadAllBytes("run.oppr"), "run.oppr");
var history = new History(source);   // OneScope.Sdk
history.Seek(120);
ulong hash = history.StateHash;      // OnePhysics' world hash at step 120
```

## What it supports

- **Sources:** replays (version 1 and 2), world snapshots, scene text (authored or exact-state). The first state builds the world; later recorded
  state changes are replayed as written. Live-option records after the first one are not applied.
- **Pokes on a fork:** `impulse`, `set_velocity`, `set_position` and `wake`, on a body entity.
- **Entities:** the world, every body and every joint, with inspector rows from the library's per-body and per-joint inspection.
- **Overlays:** shapes (coloured by body state), bounds, contacts, joints and centres of mass. Soft bodies, fluids, rods and the other
  deformable stores are not drawn yet.
- **Timeline:** awake bodies and contact points as series; contact begin, sleep and wake and joint break as events.
- **Units and axes:** metres; the time axis only.
- **Writing:** `WriteReplay` writes a OnePhysics replay (version 2) that the library's own player accepts.

## Building

The module is built against the OneScope SDK package, which is not on nuget.org yet, so this project and its tests are not part of the
solution and CI does not build them. Point the restore at the folder or feed that holds the SDK package:

```
ONESCOPE_SDK_FEED=<the OneScope SDK feed> dotnet build src/OnePhysics.Scope
ONESCOPE_SDK_FEED=<the OneScope SDK feed> dotnet test --project tests/OnePhysics.Scope.Tests
dotnet pack src/OnePhysics.Scope -c Release
```

Licence: MIT OR Apache-2.0, © Recursive AI, LLC.
