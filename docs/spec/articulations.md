# Articulations (reduced-coordinate rigid-body trees)

Normative for both dimensions. An articulation is a tree of rigid links whose relative motion is described by joint coordinates, so its joints cannot separate: the constraint is satisfied by construction, not by a solver. Maximal-coordinate joints (`joints.md`) and ragdolls stay as they are.

References: Featherstone, *Rigid Body Dynamics Algorithms* (2008) and "The calculation of robot dynamics using articulated-body inertias" (1983) for ABA, RNEA and CRBA; Mirtich, *Impulse-based dynamic simulation of multibody systems* (1996); Kokkevis, "Practical physics for articulated characters" (GDC 2004) for contacts in joint space; Baraff and Witkin, "Large steps in cloth simulation" (1998) for the implicit diagonal; Wampler (1986) and Buss and Kim (2005) for damped least squares; Catto (GDC 2006) for the contact bias.

## Definition

`ArticulationDef` (3D `Physics3D.ArticulationDef`, 2D `Physics2D.ArticulationDef`) is a list of `ArticulationLinkDef`, parents first, plus world options. It is shaped so that URDF and MJCF map onto it one to one:

| Member | Meaning | URDF / MJCF |
|---|---|---|
| `Links[i].Parent` | index of an earlier link; the root has -1 | `joint/parent` / body nesting |
| `Joint` | `Fixed`, `Revolute`, `Prismatic`, `Spherical` (3D only) | fixed, revolute and continuous, prismatic, ball |
| `Axis` | joint axis in the link's own frame (normalised on creation) | `axis` / `axis` |
| `Origin`, `OriginRotation` | the link frame (= the joint frame) in the parent's frame at joint position 0 | `origin xyz rpy` / `pos`, `quat` |
| `Mass`, `CenterOfMass`, `InertiaTensor` | inertial properties in the link frame (3×3 about the centre of mass); mass 0 derives them from the shapes | `inertial` |
| `HasLimits`, `LowerLimit`, `UpperLimit` | position limits (radians or metres) | `limit lower upper` / `range` |
| `SwingLimit` | cone half-angle of a spherical joint | MJCF ball `range` |
| `Drive` (`Stiffness`, `Damping`, `MaxForce`, `TargetPosition`, `TargetVelocity`) | PD drive; `MaxForce` is the actuator effort limit (0 = none) | `limit effort`; position actuator `kp`, `kv`, `forcerange` |
| `MaxVelocity` | joint speed limit (0 = none) | `limit velocity` |
| `Friction`, `Damping` | dry friction (force) and viscous damping of the free joint | `dynamics friction damping` / `frictionloss`, `damping` |
| `Armature` | rotor inertia added to the joint's diagonal | `armature` |
| `InitialPosition`, `InitialRotation` | joint coordinate at creation | `ref` / `qpos0` |
| `Shapes` | collision shapes (`ShapeSpec` with `ShapeDef`) in the link frame | `collision` / `geom` |
| `FixedBase`, `Position`, `Rotation` | root fixed to the world at a pose, or free (6 dof in 3D, 3 in 2D) | `world` fixed joint or floating base |
| `Substeps`, `Iterations`, `SelfCollision` (events only, see Contacts), `GravityCompensation`, `GravityScale` | per-articulation options | — |

Mimic joints and closed loops are not part of 1.0 (an interchange importer may express a mimic as a drive). Presets: 3D `RobotArm(reach, joints, mass)`, `Quadruped(scale)`, `Chain(links, linkLength, linkMass, endMass)`; 2D `Arm(reach, joints, mass)`, `Chain(...)`.

Validation (typed `ArgumentException`/`ArgumentOutOfRangeException` naming link and member): 1 to 2048 links; parents earlier; finite values within stated bounds; inertia symmetric positive semi-definite; axis non-zero; planar articulations: revolute axis z, prismatic axis in the plane, no spherical joint, origins with z = 0, rotations about z. The base position is within ±2^21. A definition whose joint-space mass matrix is singular is refused (a link with no inertia about its own joint axis).

## State and coordinates

Joint coordinates `q` (per link in order: nothing for fixed; one value for revolute and prismatic; a unit quaternion x, y, z, w for spherical; for a free root, position then quaternion; 2D free root: position x, y, 0 and a z-rotation quaternion) and speeds `qd` (free root: linear velocity then angular velocity in the world frame; spherical: angular velocity in the parent frame; 2D free root: vx, vy, ωz). Everything inside the model is **double precision**; link bodies receive float32 velocities and rotations and positions on the 2^-32 grid. A floating root's translation is rounded to that grid at the end of every step.

The links are **kinematic bodies** of the world (so shapes, queries, filters, events, debug geometry and contact lists work unchanged). After each step every link body gets the pose and twist that forward kinematics gives; a user move of a link body is ignored. `Articulation.GetLink(i)` returns the body. Destroying the articulation does not destroy the bodies; `Articulation.Destroy()` does both.

## Dynamics

Per world step, S substeps of length h = dt/S, in articulation slot order (articulations do not interact with each other except through contacts, which treat the other articulation's links as kinematic surfaces):

1. **Substeps.** `Substeps = 0` (default) is automatic: S = clamp(ceil(dt·ω), 4, 64) where ω is `StiffnessFrequency`, the largest √(k/m)-style frequency of the joint drives and the gravity stiffness estimate. It depends only on the articulation's own state, never on threads or width. A fixed 4 substeps blows up a 64-link chain with a 1000:1 end mass; the automatic rule is what keeps it stable.
2. **Drift** by h/2 (leapfrog: positions by the velocity, quaternions by the exact rotation of the angular velocity), forward kinematics.
3. **Drives.** PD forces per joint with the force limit, joint damping, friction handled as a constraint row, gravity compensation (`GravityCompensation`: the RNEA forces that hold the present pose against gravity, added to the drive force), armature. Stiffness and damping are integrated **implicitly**: their derivative enters the diagonal of the mass matrix (Baraff and Witkin), so any stiffness is stable at any step.
4. **Forward dynamics** by the articulated-body algorithm with the diagonal term, then two fixed-point passes of the implicit midpoint rule for the velocity-dependent (Coriolis, centrifugal) terms: one pass (explicit midpoint) amplifies the pure rotations of a whipping chain by 1 + (hω)⁴/8 per substep and diverges; two make it 1 − (hω)⁴/8.
5. **Kick**: qd += h·qdd.
6. **Constraint solve** (dense projected Gauss-Seidel in joint space, `Iterations` passes, default 8): rows for joint friction, joint limits (position error biased at 0.2 of the depth per substep, speculative when not yet reached), `MaxVelocity`, and one normal plus two friction rows per contact point of a link. Effective mass is the joint-space mass matrix M(q) plus the implicit diagonal, Cholesky-factored once per substep; W = M⁻¹Jᵀ is kept for every row. The other body of a contact is updated in the same loop with its own inverse mass and inertia (a dynamic partner is pushed out of penetration by the rigid solver, not here). Contacts come from the engine's manifolds of the step (see Contacts); impulses are warm-started from the previous substep. A body squeezed between a link and static geometry is held by the geometry through *support rows* made from that body's contacts with static bodies.
7. **Drift** by h/2.

At the end of the step link bodies are placed from forward kinematics (positions rounded to the 2^-32 grid) and their velocities set from the link twist (so the rigid solver, events and the next step's speculative margins see the true motion).

## Contacts

Link bodies are kinematic. The narrowphase pairs them with every other body: kinematic-versus-static pairs, which the broadphase never produces, are added by `AddLinkPairs` (queries of the static and kinematic trees with each link's shapes, in link and shape order). Pairs between two links of one articulation are blocked unless `SelfCollision` is set (then non-adjacent links form pairs, which raise events and appear in contact lists, but the articulation solve skips them: **self-collision does not push links apart in 1.0**). Links of different articulations collide as kinematic surfaces. Contact material combine rules, filters, sensors, modification and events are the engine's (`contacts.md`). The articulation solve reads each pair's manifold in constraint order, converts it to joint-space rows and writes nothing back to the contact impulse store.

## Task-space targets

`SetTaskTarget(link, localPoint, target, gain, damping, maxStep)` runs one damped-least-squares step per world step: the point Jacobian J (3 × dof) of the point on the link, the displacement δq = Jᵀ(JJᵀ + λ²I)⁻¹·e with e the clamped error (at most `maxStep` metres) and λ = `damping`, applied as new position-drive targets of the joints that have a position drive (stiffness above 0), within their limits. `SetTaskOrientation` adds a rotation error. It does not move joints without a drive. This is a joint-space feedback controller on the drives, not a separate dynamics, so it respects contacts and limits.

## Inverse dynamics and raw access

`ComputeInverseDynamics(qdd, tau)` (RNEA) gives the joint forces for given accelerations at the present q, qd, including gravity; `ComputeGravityCompensation` is RNEA with qd = qdd = 0; `ComputeMassMatrix` is CRBA (dof × dof, row-major, symmetric); `ComputePointJacobian(link, localPoint)` the linear Jacobian. `GetPositions/SetPositions/GetVelocities/SetVelocities` read and write the joint coordinates (all references are doubles). Identities checked by tests: τ = M(q)·qdd + C(q,qd) + g(q) to 1e-5 relative; CRBA symmetric positive definite; Jacobian against finite differences.

## Host hooks

- **Snapshot.** Optional section "ARTI" (`0x49545241`, version 1), written only while an articulation exists, registered **last** (after every other section of that world). Per articulation slot (handle table, then for each live slot): link count n (int32), flag byte (1 fixed base, 2 planar, 4 self collision, 8 gravity compensation, 16 task target, 32 task target has orientation, 64 external forces present), substeps (int32), iterations (int32), gravity scale (double); per link: parent (int32), joint kind (byte), axis (3 doubles), placement rotation (4), placement position (3), mass, centre of mass (3), rotational inertia about the centre of mass (A B C E F I: 6 doubles), body index and generation (2 int32), lower, upper (doubles), has-limit (bool), drive stiffness, damping, max force, target position, target velocity, feed-forward, friction, armature, joint damping, max velocity, cone limit (doubles), target rotation (4 doubles); then q (named double array), qd, the task target (link int32, local point, position, rotation, gain, damping, max step), and the external joint forces when flagged. Contact impulses of the articulation solve are not saved (they are rebuilt from the manifolds each step, warm start is within the step). The reader validates ranges, finiteness, parent order, joint kinds for the dimension, unit quaternions and that every link body exists, is kinematic and belongs to no other articulation; a snapshot that fails is refused with the world unchanged.
- **World hash:** the same fields in the same order.
- **Fork, restore, region:** a fork copies the store; a region that holds any link holds the whole articulation (`CloseRegion`), its bodies and their joints/contacts closure as for any body.
- **Origin shift:** root translation (or fixed-base placement) loses the offset exactly, link bodies shift with the core. Because the model's contact arithmetic is double relative to absolute positions, a shifted world follows the unshifted one to rounding (about 1e-6 over seconds of motion), not bit for bit as rigid bodies do; two worlds shifted alike by the same offset at the same step stay bit-identical to each other, which is the contract.
- **Replay:** every public mutator is recorded as a complete-state mutation (`Articulation.*`, `PhysicsWorld*.AddArticulation`).
- **Debug geometry:** `DebugContent.Articulations` (1048576; `All` = 2097151): link frames, joint axes and limits, task targets, contact points the solve used.
- **Sleeping:** articulations never sleep; their links are always active and keep touching bodies awake.

## Determinism

All arithmetic is IEEE `+ − × ÷ sqrt` in double; quaternion rotations of angular velocity use onemath's sine and cosine. Articulations run on the caller thread in slot order (per-articulation parallelism is a follow-up; independent articulations could be solved in parallel and combined in slot order because they share no state except bodies they push, which they take in link order). Contacts, rows and bodies are visited in the constraint-table order of the step. Matrix rows `world3d/articulation-arm-3d`, `world3d/quadruped-3d`, `world2d/articulation-chain-2d` are equal on x64 jit, threads8, scalar, no-AVX2, tiering off, NativeAOT, arm64 and WASM.

## Scenes and thresholds

See `scenes.md` → Articulation scenes.

## Limits and known gaps

- Self-collision contacts are not solved.
- Closed loops and mimic joints are not modelled (a parallel gripper or a four-bar needs maximal-coordinate joints).
- Linearly implicit Coriolis terms would allow fewer substeps on whipping chains (follow-up); until then a long light chain with a heavy end takes up to 64 substeps.
- Contact solving is dense in joint space: cost grows with dof² per contact row and dof³ per substep factorisation; a 12-dof quadruped is cheap, a 2000-link tree is not meant for many contacts.
- Articulation links are kinematic for the rigid solver: a dynamic body is never pushed by an articulation except through the impulse the articulation's own solve gives it, so a heavy dynamic body on a weak articulation is held up only as far as the drives and limits allow.
