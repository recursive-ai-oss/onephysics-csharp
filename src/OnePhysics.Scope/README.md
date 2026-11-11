# RecursiveAI.OnePhysics.Scope

The OnePhysics module for the OneScope viewer, on OneScope SDK 1.0. It opens a OnePhysics replay (checkpoint or command, 2D or 3D), a world snapshot or a
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

- **Sources:** replays (version 1 and 2), world snapshots, scene text (authored or exact-state). The first state builds the world and is
  the recording's start step; later recorded state changes are held in the step inputs and replayed as written. Live-option records after
  the first one are not applied. The replay's recorded per-step hashes are passed to the history, which raises `replay.hash-mismatch` at
  the first step that does not reproduce.
- **Live:** `OnePhysicsModule` is an `ILiveModule`. `LiveFeedStream` is the host side: a write-only `Stream` that turns the library's
  recorder output into the whole records the viewer applies. Create one for the host's world, hand it to the recorder, and give the probe
  its snapshot and record calls:

  ```csharp
  var feed = new LiveFeedStream(world);          // PhysicsWorld2D or PhysicsWorld3D
  world.StartRecording(feed);                    // the library's recorder writes into the feed
  // wire the probe: feed.WriteSnapshot is its snapshot source, and feed.Record is called with each record
  feed.Record = record => probe.Record(record);  // OneScope.Probe; a record is a world mutation or one whole step
  ```

  The feed forwards a record only after the host has applied it, which is when the probe wants it, and drops records while `Record` is
  null. The recorder's own header, initial snapshot and option records are not forwarded; the snapshot the probe takes carries that
  state. The viewer then re-simulates the host's steps from the attach point, with the host's step numbers and hashes.
- **Pokes on a fork:** `impulse`, `set_velocity`, `set_position` and `wake`, on a body entity; the first and third are drag-to-point.
- **Entities:** the world, every body and every joint, with inspector rows from the library's per-body and per-joint inspection.
- **Overlays:** shapes (coloured by body state), bounds, contact normals (longer for harder contacts, and sized by the typical body so they read at any scale), joints, joint limits (3D:
  hinge arcs, slider ranges, cone and swing cones) and centres of mass. Every item names its body or joint, so picking works. The hinge
  arc shows the limit span from a fixed direction: the debug geometry has no zero direction for the angle. 2D limits are not exposed. Soft bodies, fluids, rods and the other
  deformable stores are not drawn yet.
- **Timeline:** awake bodies and contact points as series; contact begin, sleep and wake and joint break as events.
- **Units and axes:** metres; the time axis only.
- **Writing:** `WriteReplay` writes a OnePhysics replay (version 2) that the library's own player accepts.

## Building

```
dotnet build src/OnePhysics.Scope
dotnet test --project tests/OnePhysics.Scope.Tests
dotnet pack src/OnePhysics.Scope -c Release
```

The module and its tests are not in the solution, so CI does not build them; the release workflow builds, tests and packs them on a
`scope-v<version>` tag. To restore the OneScope SDK from a folder or feed instead of nuget.org, set `ONESCOPE_SDK_FEED`.

Licence: MIT OR Apache-2.0, © Recursive AI, LLC.
