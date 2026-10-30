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

Section id `0x534F4654` ("SOFT"), **optional**: written, hashed and counted only once the world has created a soft body, so worlds without soft bodies keep their bytes and hashes (the core's section rule: a snapshot lacking an optional section resets it; a world that does not register the id rejects a snapshot that has it with a `SnapshotFormatException` naming the section count). Layout: i32 version (1, or 2 when a body is skinned; see Skinning); i32 SoftSubsteps; i32 SoftIterations; the handle table; then per live body in slot order: bool tetrahedra, i32 vertex count, vertices (3 × f32), i32 element count, indices (i32 each), the definition (position 3 × f64, rotation 4 × f32, scale, velocity 3 × f32, mass, radius, edge/bend/volume compliance and damping, pressure, friction, linear damping, gravity scale, bools self-collision and allow-sleep, category, mask, group, layer), bool asleep, f32 sleep time, then positions (f64), previous positions (f64), velocities (f32), pinned bytes. The constraints are rebuilt from the mesh on load. Restore validates counts, finiteness and ranges before touching the world.

## Out of bounds

A particle outside ±2^22 m or not finite raises `SoftBodyOutOfBoundsException` at the start of `Step`, world unchanged. `ShiftOrigin` shifts soft particles by the same offset.

## Skinning

A body made with `SoftBodyDef.Skin` follows a skeleton. Data (`SoftSkin`, immutable): J joints (1 to 256) with parents before children and joint 0 the only root; the bind position of each joint in mesh coordinates (scaled by the definition's `Scale` together with the vertices); per particle four joint numbers and four weights (non-negative, normalised to sum to 1 in float32 by division; a weight of 0 skips its influence); a maximum distance D, a back-stop distance b and a back-stop radius R (0 to 10^6 m; R at most b); and a rest normal in mesh coordinates (given, or the area-weighted vertex normal of the mesh's triangles, normalised; none for a tetrahedral mesh; R > 0 needs one). Errors are `SoftBodyException` naming the array and entry.

**Pose.** State per joint: a world position (double) and a unit rotation (float32 x y z w), current and as of the end of the previous step. A body starts in its bind pose placed at `Position` and `Rotation`: joint j at `Position + Rotation * bind_j`, rotated by `Rotation`; the particles start exactly on their skinned positions. `SetSkeletonTransforms` stores the given positions and normalised rotations. `SetSkeletonPose(pose, at, rotation)` takes one rotation D_j per joint, relative to the parent in bind-pose axes (the ragdoll convention): the root's frame is qRoot = `rotation` (identity when zero); joint 0: rot = qRoot * D_0, position = at + qRoot * bind_0; joint j with parent p: rot_j = normalize(rot_p * D_j), position = pos_p + rot_p * (bind_j - bind_p). A pose that differs from the stored one wakes the body; `teleport` puts the particles on the targets (positions and previous positions), zeroes their velocities and makes the previous pose the current one.

**Linear blend skinning.** For particle i with influences (j_k, w_k), k = 0..3 in order, skipping w_k = 0: target = sum over k, accumulated in double in that order, of (double)w_k * (pos[j_k] + (double)(rot[j_k] * (rest_i - bind[j_k]))) with the rotation and the difference in float32; normal = the float32 sum of w_k * (rot[j_k] * restNormal_i), divided by its length (zero if the length is at most 1e-6 or not finite). Targets are computed twice at the start of a step, from the previous and the current pose; a substep k (of S) uses the target at fraction t = (k+1)/S, previous + (current - previous) * t in double, and exactly the current target at the last substep. The skinned normal always comes from the current pose.

**In the step.** The skin adds three things to the steps above:

1. *After Predict* (step 4), each kinematic particle (D = 0, not pinned) is set to its target at the end of the substep. Kinematic particles and pinned ones have effective inverse mass 0 (as for pins, they never collide with shapes and take no velocity). Bodies' bounds, for candidate shapes, are widened to the current targets.
2. *At the end of every pass* (after step 8's pair contacts) each free particle is limited, with d = position - target in float32 and c = -normal * b: if R > 0 and a normal exists, and |d - c| < R, d is moved radially out to |d - c| = R (to c + normal * R when d = c); then if |d| > D, d is scaled to length D; if that put d back inside the back-stop (and b > 0), d is replaced by the nearest point of the circle where the two spheres meet, at height a = (R^2 - D^2 - b^2) / (2b) along the normal and radius sqrt(D^2 - a^2) (0 if negative), taking the direction from the normal's perpendicular part of d (any perpendicular if on the axis). The position becomes target + d only when a limit acted. Because the limits come last in the last pass, the position at the end of the substep satisfies both to float32 rounding (a few ulp of D; the tests allow 1e-4 relative).
3. *At the end of the step* the current pose becomes the previous one. A body whose current pose differs from the previous one this step does not accumulate sleep time.

Contacts lose against the limits: a particle pushed by a shape beyond D comes back. The limits act as the long-range attachments of Kim, Chentanez and Müller (2012) and the attachments of Müller et al. (2007), with the back-stop as an exclusion sphere; the sphere definition is this library's own, written from the published description (a distance behind the skinned surface along its normal, and a radius).

**Snapshot.** The section version is 2 when any body is skinned and 1 otherwise, so worlds without skinned bodies keep their bytes and hashes; a reader accepts both. In version 2 each body has a bool after the definition's `Layer` field; when set, the skin follows: i32 J; parents (i32 x J); bind positions (3 x f32 x J); joint numbers (i32 x 4N); weights (f32 x 4N); maximum distances, back-stop distances and radii (f32 x N each); normals (3 x f32 x N). After the pin flags of such a body: current joint positions (3 x f64 x J), rotations (4 x f32 x J), then the same for the previous pose. The reader checks parents, indices, weights (non-negative, sum within 1e-3 of 1), limits, normals (zero or unit within 1e-3), rotations (unit within 1e-3) and positions (within 2^22) and does not renormalise anything, so a restored body writes the same bytes back. The world hash covers all of it, being the snapshot's state in the same order. `ShiftOrigin` shifts the joint positions with the particles.

**Debug.** `DebugSoftBody.FirstTargetPoint` indexes the N skinned targets (for the current pose) appended to the points after the particles; -1 for an unskinned body.

## Scenes and thresholds

See `docs/spec/scenes.md` ("Soft bodies"). Reference: Jitter2's soft bodies (vertices as rigid bodies, springs, triangle and tetrahedron shapes); Jolt's are not driven.
