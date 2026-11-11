# Changelog of RecursiveAI.OnePhysics.Scope

The viewer module is released on its own, with its own version and tag (`scope-v<version>`), separately from the library packages in the
repository's top-level `CHANGELOG.md`. Each release has a `## [<version>]` heading; the text under it becomes the release notes.

## [Unreleased]

## [1.1.0]

The first release of the OnePhysics module for the OneScope viewer, on OneScope SDK 1.0 and RecursiveAI.OnePhysics 1.0.0. It follows the 1.1.0-preview builds, which were never published to nuget.org; the module's version number follows the RecursiveAI.OnePhysics 1.x line.

- **Files:** opens OnePhysics replays (versions 1 and 2, checkpoint or command, 2D or 3D), world snapshots and scene text. Seeking to any
  step through the viewer's keyframes gives the replay's own per-step hashes; the first step that does not reproduce raises
  `replay.hash-mismatch`.
- **Live:** `OnePhysicsModule` is a live module; `LiveFeedStream` is the host side that regroups the recorder's output into whole records
  for the probe, so the viewer re-simulates a running world with the host's step numbers and hashes.
- **Forks and pokes:** `impulse`, `set_velocity`, `set_position` and `wake` on a body, applied to a fork only; `WriteReplay` saves a fork
  as a replay the library's player accepts.
- **Diff and inspection:** differences name the body or joint first; the world, bodies and joints appear as entities with inspector rows
  from the library's own inspection.
- **Drawing:** shapes coloured by body state, bounds, contact normals (sized by the typical body, longer for harder contacts), joints,
  3D joint limits (hinge arcs, slider ranges, cone and swing cones) and centres of mass. Every item names its body or joint, so picking
  works.
- **Timeline:** awake bodies and contact points as series; contact begin, sleep, wake and joint break as events.
