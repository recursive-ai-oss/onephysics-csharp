# Soft bodies

Cloth, rubber and jelly in the 3D world (a preview, like the rest of `Physics3D`: `#pragma warning disable OPHYS0002`). A soft body is a mesh of particles held together by edge, bend and volume constraints (position-based dynamics, XPBD). Particles are small spheres that collide with every rigid shape (static, kinematic and dynamic; compounds, meshes and height fields included), with each other, and with friction. They push dynamic rigid bodies and are pushed by them.

## Happy path

<!-- example: Guide/SoftBodies.cs -->
```csharp
SoftBody cloth = world.AddCloth(width: 2f, depth: 2f, segments: 16, at: new Vector3D(0, 1.8, 0));
SoftBody cube = world.AddSoftBox(new Vector3F(0.6f, 0.6f, 0.6f), at: new Vector3D(2, 1, 0), SoftMaterial.Jelly);
SoftBody ball = world.AddSoftBall(0.3f, at: new Vector3D(-2, 1, 0), SoftMaterial.Rubber);
Vector3D middle = cloth.GetPosition(8 * 17 + 8);
```

- `SoftBody` is a small struct handle like `Body`: it allocates nothing and throws `StaleHandleException` after `Destroy()`. There is no callback anywhere on it.
- Sizes are full extents, positions are `double`, everything else `float`, as for rigid bodies.
- `GetPosition(i)` reads one particle without allocating. Read particles with `GetPosition(i)`, `GetVelocity(i)`, `CopyPositions(span)`, or the raw spans (`RawPositions`, `RawVelocities`, `RawEdgeA`, ...); `Volume`, `RestVolume`, `Bounds`, `CenterOfMass`, `Momentum` and `IsAsleep` are cheap.
- `SetPinned(i, true)` fixes a particle where it is; `SetPosition` teleports one; `ApplyImpulse` and `AddVelocity` push.
- Presets: `SoftMaterial.Cloth` (inextensible, bends easily, self-collision on; use with an open surface such as `SoftBodyMesh.Cloth`), `Rubber` (stiff, springs back) and `Jelly` (wobbly). The meshes `SoftBodyMesh.Cloth`, `Box` and `Sphere` are generated; `FromTriangles` and `FromTetrahedra` take your own.

## Middle tier

`SoftBodyDef` holds every parameter with a default: mass, particle radius, the compliance (metres per newton; 0 is as stiff as it gets) and damping of the edge, bend and volume constraints, `InternalPressure` for a closed surface, friction, damping, gravity scale, self-collision, pins, and the layer, category/mask and group used by the rules in [queries and layers](queries-and-layers.md). `SoftBodyDef.Cloth`, `Rubber` and `Jelly` start from the presets. The world options `SoftSubsteps` (default 4) and `SoftIterations` (default 3) trade cost for stiffness: many small substeps beat many iterations (Macklin et al. 2019).

## Skinning to a skeleton

A soft body can follow an animated skeleton: clothing on a character, flesh on a bone. `SoftSkin.Create` takes the skeleton (parents before children, as in a ragdoll definition), the joints' bind positions in the mesh's coordinates, four joint numbers and weights per particle (linear blend skinning) and a **maximum distance** per particle. Each step the particle is held within that distance of its skinned position and never beyond; a distance of 0 makes the particle kinematic. Optional **back-stop** spheres (a distance behind the skinned surface along its normal, and a radius) keep a cloth from being pushed into the body it covers.

<!-- example: Guide/SoftBodies.cs -->
```csharp
var sheet = SoftBodyMesh.Cloth(1f, 1f, 4, 4);
int n = sheet.VertexCount;
var joints = new int[4 * n];        // four joint numbers per particle (all joint 0 here)
var weights = new float[4 * n];     // and four weights
var maxDistance = new float[n];
for (int i = 0; i < n; i++) { weights[4 * i] = 1f; maxDistance[i] = i < 5 ? 0f : 0.05f; }
var skin = SoftSkin.Create(sheet, [-1], [new Vector3F(0, 0, 0)], joints, weights, maxDistance);
SoftBody banner = world.AddSoftBody(new SoftBodyDef { Mesh = sheet, Skin = skin, Position = new Vector3D(0, 4, 3) });
banner.SetSkeletonPose([QuaternionF.Identity], at: new Vector3D(0.5, 4, 3));   // move the bone; the targets follow over the next step
```

- Pose the skeleton before `world.Step` with `SetSkeletonPose` (one rotation per joint, relative to the parent in bind-pose axes, as ragdoll poses are) or with `SetSkeletonTransforms` (a world position and rotation per joint, for an animation system that has them). The targets move linearly to the new pose over the step.
- `teleport: true` places the particles on their skinned positions at once with zero velocity; use it to spawn or reset a body.
- A body with a moving pose does not sleep; a changed pose wakes it. `GetSkinnedPosition(i)` and `CopySkinnedPositions` read the targets, and the debug geometry carries them.
- The limits hold after every step to within float32 rounding, even against contacts: where a contact and a limit disagree the limit wins, so keep the skin where the cloth can reach the shapes it should collide with.

## What to expect

- Stiff, heavy rigid bodies on light soft ones sink: a 5 kg box on an 8 kg cushion is fine; 300 kg on a 5 kg block is not, because the soft body is solved with a few passes. Add iterations or mass.
- Thin shapes are not tunnelled at the speeds of the scenes (particles that move more than half their radius in a substep are swept).
- Soft bodies sleep like rigid bodies and wake when an awake rigid body comes near.
- Snapshots, hashes and rollback include soft bodies (an optional section: worlds without soft bodies keep the bytes and hashes they had). Soft bodies are not solved in parallel; the thread count does not change a bit.
- Not yet: tearing, and solving separate bodies in parallel.

Details and the order of a step are in [the specification](../spec/softbody.md).
