# Changelog

What changed for hosts in each preview, newest first. "Moved results" lists the changes that alter a world's state after the same inputs (and so its snapshot bytes and world hashes).

**0.x previews are not bit-compatible with each other: snapshots, saved games and replays from one preview do not carry to another.**

## Unreleased

- **Moved results: sensors.** A sensor overlap is no longer a constraint. It is a mark on the pair, so a sensor never takes a constraint slot, a pair slot or a color that a contact could have had. Adding, moving or removing a sensor of any body type (static, kinematic, dynamic) leaves every solid body bit-identical. Worlds that contain sensors give different results and different snapshot bytes than preview.2 (in preview.2 sensors paired with kinematic bodies and characters reordered contact solving). Worlds without sensors are unchanged by this change.
- Sensor enter and exit events are unchanged in kind and order; an exit is now also reported when a body or shape that was inside a sensor is destroyed.
- Snapshot layout: the free-pair list of both engines now also carries the slots freed by sensor pairs (as complements, after the contact slots), and a 3D sensor overlap is written as a one-entry pair chain holding -2. Worlds without sensor pairs write the same bytes as before.
- New capacity options on `WorldOptions2D` and `WorldOptions3D` (`PairCapacity`, `ContactCapacity`, `IslandCapacity`, `QueryScratchCapacity`) and `Warmup()` on both worlds, so a host can reach an allocation-free steady state from the first step. Capacity options never change a result and are not saved in snapshots.
- `ComputeHash()` streams over the world state in snapshot order and allocates nothing (as since preview.2); this changelog now records it.
- Allocation: a deep-contact path in the 3D narrowphase allocated 96 bytes per call and no longer does; a restore of a larger world now allocates nothing after `Warmup()`.
- **Moved results: speed rounds and fixes since preview.2.** These change the state after the same inputs, for the scenes named: the 2D pair stage (refit between tree rebuilds) moves 2D worlds with many moving bodies; the 3D solver kernels move worlds with joints and ragdolls; the character and services fixes for stale displacement move 2D worlds with characters or services.

## 0.1.0-preview.2

- Host hooks: `TrySnapshot`, `GetSnapshotSize` and `Restore(ReadOnlySpan<byte>)` without per-call copies; allocation-free `ComputeHash`; handle liveness queries; character index and generation; typed errors for velocity and bullet settings on static bodies.
- **Moved results: sensors for kinematic bodies and characters.** Sensors now pair with kinematic bodies and characters, so those worlds' events, constraint order and results differ from preview.1.
- Speed rounds between preview.1 and preview.2 (pair stage, solver glue, 3D contact preparation) left the world hashes of the project's scenes unchanged; the sensor change above is what moved results in preview.2.

## 0.1.0-preview.1

First preview: the 2D engine, the shared core (handles, deterministic scheduler and coloring, islands and sleeping, layers and filters, events and sensors, queries, materials, snapshot and restore, world hashes, debug geometry), and the 3D rigid core with characters, vehicles, ragdolls and buoyancy in progress.
