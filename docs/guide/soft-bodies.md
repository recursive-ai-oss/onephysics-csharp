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

## What to expect

- Stiff, heavy rigid bodies on light soft ones sink: a 5 kg box on an 8 kg cushion is fine; 300 kg on a 5 kg block is not, because the soft body is solved with a few passes. Add iterations or mass.
- Thin shapes are not tunnelled at the speeds of the scenes (particles that move more than half their radius in a substep are swept).
- Soft bodies sleep like rigid bodies and wake when an awake rigid body comes near.
- Snapshots, hashes and rollback include soft bodies (an optional section: worlds without soft bodies keep the bytes and hashes they had). Soft bodies are not solved in parallel; the thread count does not change a bit.
- Not yet: skinning to a skeleton (per-particle targets with a maximum distance), tearing, and solving separate bodies in parallel.

Details and the order of a step are in [the specification](../spec/softbody.md).
