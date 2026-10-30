# Soft bodies (3D)

Normative for `PhysicsWorld3D` soft bodies: what a body is, the order of a step, the constraints, the contact and coupling rules, the snapshot section and the scenes that measure them. The 3D surface is `[Experimental("OPHYS0002")]`.

## What a soft body is

A mesh of particles (`SoftBodyMesh`: vertices with either a closed or open triangle surface, or tetrahedra) with a mass spread over the particles in proportion to the rest volume (tetrahedra) or area (triangles) they carry. Each particle is a sphere of radius `ParticleRadius` for collision (default a quarter of the mean rest edge length). Particles may be pinned (inverse mass 0). Meshes are validated on construction: counts, finite coordinates inside ±2^21, indices, degenerate and non-manifold elements, unused vertices; errors are `SoftBodyException` with a `SoftBodyError` code naming the field or element.

**Storage.** Soft bodies do not live in the rigid body store. Particles have their own struct-of-arrays store (positions as double, velocities and inverse masses as float), solved serially in a fixed order after the rigid step. The rigid islands, colouring, contact stage and solver passes do not see them, so a world with soft bodies steps its rigid part exactly as one without (the rigid state and hashes are identical until a rigid body touches a soft one). Reasons: the rigid solve is a velocity-level soft-step loop on 104-float contact payloads, and XPBD is position-level, so particles would not share a pass with it; a payload per particle contact would cost far more than the contact needs; and a serial, index-ordered Gauss–Seidel is deterministic at any thread count and vector width by construction. Solving different soft bodies in parallel (a colouring over soft bodies, combined in index order) is a follow-up.

## A step

`Step(dt)` runs the rigid step, then the soft step (`StepSoft`), when any soft body is awake:

1. **Awake set.** Soft bodies in slot order. A sleeping body wakes when an awake rigid body's bounds, grown by the particle radius and the speculative distance, reach its bounds. Candidate rigid shapes are those whose bounds, swept over the step, meet an awake body's bounds (layer matrix, group and mask rules as for shape pairs; sensors never collide). A sleeping rigid body is static for soft bodies.
2. **Substeps.** `SoftSubsteps` (default 4) substeps of `h = dt / SoftSubsteps`, each with `SoftIterations` (default 3) Gauss–Seidel passes (Macklin et al. 2019, small steps; Müller et al. 2007; XPBD, Macklin, Müller and Chentanez 2016).
3. **Rigid bodies in the substep.** Each shape's pose at the end of substep k is the linear interpolation (position) and normalised linear interpolation (rotation, shortest arc) between its poses before and after the rigid step at t = (k+1)/SoftSubsteps, plus the displacement that soft reactions have already given the body.
4. **Predict.** `v += g · GravityScale · h`, `v /= 1 + LinearDamping · h`, `x* = x + v h`; pinned particles stay.
5. **Detect contacts.** Each particle sphere against every candidate shape (sphere–convex manifold routine; compounds per child; triangle soups per candidate triangle; a particle that moved more than half its radius is swept against the shape's motion). Contacts are made for a distance of the speculative distance plus twice the particle's motion in the substep. At most four contacts per particle: the deepest are kept, ties keep the earlier. A particle further than 2·10^6 m from a shape makes no contact (the next step reports it out of bounds).
6. **Particle pairs.** Particles of different bodies, and of one body when `SelfCollision` is set, collide as spheres, except pairs that share an edge or a bend hinge or rest closer than their spheres. Pairs are found with a sorted cell grid (cell size 6 times the largest radius; keys = hashed cell << 24 | particle number, sorted ascending), so the pair order is fixed.
7. **Soften push-out.** A contact whose particle starts the substep inside the surface raises its constant term so that at most 1 m/s · h of the overlap is removed per substep (split-impulse style); the removed amount is excluded from the particle's velocity afterwards, for contacts not coupled to a dynamic rigid body.
8. **Iterate.** Per pass: for each awake soft body in slot order its constraints (edges, then bends, then volume), then all shape contacts in the order detected, then all particle pairs in key order.
9. **Finish.** `v = (x - x_prev) / h`, clamped to `MaxLinearSpeed`; approach velocity against surfaces not coupled to a dynamic body is removed (restitution 0).
10. **After the last substep.** Reactions on rigid bodies are applied in ascending body index: the accumulated velocity impulse (linear and angular, through the world inverse inertia) and the accumulated pose displacement, rounded to the position grid; the body's sleep timer is reset. Then soft sleeping is updated.

## Constraints

XPBD with compliance α̃ = α / h² and damping γ = β / h per constraint family (`EdgeCompliance`, `BendCompliance`, `VolumeCompliance`, units m/N, and the matching `*Damping` in seconds); multipliers are zeroed at the start of each substep.

* **Edge:** distance to the rest length, one per distinct edge of the triangles and tetrahedra, edges sorted by (low, high) particle number.
* **Bend:** isometric bending (Bergou et al. 2006; Bender et al. 2014): for each pair of triangles sharing an edge, the quadratic bending energy of the rest shape, evaluated through the four-particle gradient. Distance across the hinge was tried first and rejected: it locks shear and exploded on a draped cloth.
* **Volume:** per tetrahedron, the signed volume (6V form) against its rest value; or, for a closed triangle surface with `InternalPressure` > 0, one constraint on the enclosed volume against `InternalPressure` times the rest volume (Müller, Heidelberger, Teschner, Gross 2005, pressure).

## Contacts, friction, coupling

* **Normal:** a position correction along the shape normal, shared between the particle and a coupled rigid body by inverse effective mass; the body's share is `wb = 1/m + (r×n)ᵀ I⁻¹ (r×n)` at the contact point. Later passes add the body's displacement (`wb` times the multiplier applied so far) to the gap, so more than one pass does not over-correct.
* **Friction:** Coulomb, as a position correction (Müller et al. 2007; Macklin et al. 2014): the tangential displacement relative to the surface is removed up to μ times the normal multiplier. μ is the geometric mean of the body's `Friction` and the shape's.
* **Two-way coupling:** a rigid body is *coupled* when it is dynamic, awake and has non-zero inverse mass; it receives the reaction. Kinematic and static bodies are moving or fixed obstacles. Momentum is conserved up to float rounding (tests: a ball into a jelly with no gravity within 2 %).

## Snapshot section

Section id `0x534F4654` ("SOFT"), **optional**: written, hashed and counted only once the world has created a soft body, so worlds without soft bodies keep their bytes and hashes (the core's section rule: a snapshot lacking an optional section resets it; a world that does not register the id rejects a snapshot that has it with a `SnapshotFormatException` naming the section count). Layout: i32 version (1); i32 SoftSubsteps; i32 SoftIterations; the handle table; then per live body in slot order: bool tetrahedra, i32 vertex count, vertices (3 × f32), i32 element count, indices (i32 each), the definition (position 3 × f64, rotation 4 × f32, scale, velocity 3 × f32, mass, radius, edge/bend/volume compliance and damping, pressure, friction, linear damping, gravity scale, bools self-collision and allow-sleep, category, mask, group, layer), bool asleep, f32 sleep time, then positions (f64), previous positions (f64), velocities (f32), pinned bytes. The constraints are rebuilt from the mesh on load. Restore validates counts, finiteness and ranges before touching the world.

## Out of bounds

A particle outside ±2^22 m or not finite raises `SoftBodyOutOfBoundsException` at the start of `Step`, world unchanged. `ShiftOrigin` shifts soft particles by the same offset.

## Not built (skinning hook)

Skinning to a skeleton is the next piece. The hook: per-particle skin targets (a position, a maximum distance and a back-stop radius, as in the published descriptions of skinned soft bodies) that act as extra one-sided distance constraints solved in the same pass as the edge constraints, pinned particles being the degenerate case with distance 0.

## Scenes and thresholds

See `docs/spec/scenes.md` ("Soft bodies"). Reference: Jitter2's soft bodies (vertices as rigid bodies, springs, triangle and tetrahedron shapes); Jolt's are not driven.
