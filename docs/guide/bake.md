# Baking a simulation

A cinematic or an animated model stage simulates once, offline, and plays the result back with no world. `BeginBake` records a stepping world into a compact cache; `BakeClip` opens the cache and samples it by time. The same members exist on `PhysicsWorld2D` (with `Vector2D` and so on). The bytes and every rule are in [the spec](../spec/bake.md).

<!-- example: Guide/Bake.cs -->
```csharp
var world = new PhysicsWorld3D();
world.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
var crate = world.AddBox(new Vector3F(1, 1, 1), at: new Vector3D(0, 4, 0));
world.AddCloth(1.2f, 1.2f, 8, new Vector3D(3, 2, 0));
world.ApplyOfflineQuality();                                  // more substeps and iterations, no sleep, surface collision on

using BakeRecorder bake = world.BeginBake(new BakeOptions { SceneName = "crate-drop", FrameStride = 2 });
for (int i = 0; i < 120; i++) world.Step(1f / 60);
byte[] file = bake.Finish();                                 // the same bytes on every platform

BakeClip clip = BakeClip.Parse(file);                        // no world needed from here on
BakeEntity box = clip.FindRigid(crate.RawHandle.Index, crate.RawHandle.Generation)!;
box.TrySampleRigid(0.5, out BakePose top);                   // half a second in; positions are linear, rotations nlerp
var cloth = new Vector3D[clip.Soft[0].PointCount];
clip.Soft[0].TrySamplePoints(0.5 + clip.FrameDuration / 2, cloth);   // between two frames
```

## What is recorded

`BakeOptions.Tracks` picks any subset of:

- **Rigid:** the origin position and rotation of every body, every frame. Bodies that appear later (fracture fragments) start at their own frame; destroyed bodies end.
- **Soft:** every particle of every soft body and cloth, plus its surface triangles once; `BakeEntity.IsCloth` tells fabric from other soft bodies.
- **Rods:** every particle of every rod, rope, cable and hair strand.
- **Particles:** every fluid and granular particle by id.
- **Fracture:** when a body broke, which body, which fragments, and the fragments' shape once (corners and triangles in their own frame), so a player can build them when their track begins.
- **Events:** the world's events by step (all steps, whatever the frame stride).

Frame 0 is the world when `BeginBake` is called, so call it after building the scene. `FrameStride` keeps every Nth step's frame. The step must stay the options' `Dt` (default 1/60); a different dt, a restore or a fork during the bake faults it and `Finish` throws.

## Size and accuracy

Positions are stored as multiples of 2^-16 m (rigid) and 2^-14 m (soft, rods, particles), rotation components as multiples of 2^-16, predicted from the two frames before and stored as the difference: a resting thing costs two bytes per run of frames and a steadily moving one little. A 1000-body pile for 600 frames is about 5 MB for its rigid track (8.5 bytes per body per frame); 2025 cloth particles for 600 frames about 3.8 MB. The errors are at most half a quantum (7.6 µm and 31 µm, below 0.001° of rotation). Choose `PositionExponent = -40` and `RotationBits = 0` for poses that read back to the double and float the world held (to 5e-13 m).

Sampling at a frame time returns the stored value; between frames positions are linear and rotations are normalised linear interpolation along the shorter arc. `TrySampleRigid` and `TrySamplePoints` return false when the thing does not exist at that time. A clip is not safe to sample from several threads; open one per thread.

## Re-simulating a frame

The header holds the scene name, the library version and the SHA-256 of a replay (`BakeOptions.ReplayDigest`, from `BakeRecorder.DigestOf(replayBytes)`), and `clip.GetFrameHash(k)` the world hash after frame `k`. To check or extend a baked frame, play the replay up to `StartStep + k * FrameStride` steps and compare `world.ComputeHash()`.

## The offline quality preset

`world.ApplyOfflineQuality()` raises substeps (4 to 8), solver passes (1 to 2), soft and fluid steps, turns sleep off and surface collision on for existing soft bodies. Settings are only raised. A 10-box stack with a 10× heavier top swayed 0.29 m with the defaults and 0.018 m with it, at about twice the step cost; a cotton drape over a sphere stretched 5.3 % at most instead of 6.6 %. See the table in the spec.

## Reading a cache that is not yours

`BakeClip.Parse` and `Read` check the structure of the whole file and raise `BakeFormatException` (the field, the byte offset and what was wrong) for anything malformed; a block's numbers are checked when it is first sampled. Chunks it does not know are skipped, so a cache written by a later 1.x opens.
