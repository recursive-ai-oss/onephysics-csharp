# Coming from Unity

OnePhysics uses Unity's vocabulary for the happy path where it can, with these differences: there are no GameObjects, so the `Body` is the thing you hold; the world is an object you create and step (there is no global `Physics2D`); angles are radians; box sizes are full sizes; and positions are `double`. The 3D names are the same as the 2D names listed here, on `PhysicsWorld3D` and its `Body`.

| Unity | OnePhysics | Note |
|---|---|---|
| `Physics2D.Simulate(dt)`, `FixedUpdate` | `world.Step(dt)` | Fixed dt; the world has no clock. |
| `Time.fixedDeltaTime` | the `dt` you pass | You choose it, the same on every peer. |
| `Physics2D.gravity` | `world.Options.Gravity` | Takes effect at the next step. |
| `Rigidbody2D` | `Body` | A struct handle; allocates nothing. |
| `rb.bodyType` (Dynamic, Kinematic, Static) | `body.Type`, `BodyType.Dynamic/Kinematic/Static` | Fixed at creation (`type:` argument or `BodyDef.Type`). |
| `rb.isKinematic` | `body.IsKinematic` | |
| `rb.velocity` | `body.LinearVelocity` | |
| `rb.angularVelocity` (degrees/s) | `body.AngularVelocity` | Radians/s. |
| `rb.position`, `rb.rotation` (degrees) | `body.Position`, `body.Angle` (radians), `body.Rotation` | `SetTransform(position, angleRadians)` teleports. |
| `rb.mass` | `body.Mass` | From shape density. |
| `rb.AddForce(f)` | `body.ApplyForce(f)` | Cleared each step. |
| `rb.AddForce(f, ForceMode2D.Impulse)` | `body.ApplyImpulse(i)` | |
| `rb.AddForceAtPosition(f, p)` | `body.ApplyForceAt(f, p)`, `ApplyImpulseAt` | |
| `rb.AddTorque(t)` | `body.ApplyTorque(t)` | |
| `rb.IsSleeping()`, `Sleep()`, `WakeUp()` | `!body.IsAwake`, `body.Sleep()`, `body.Wake()` | |
| `rb.gravityScale`, `drag`, `angularDrag` | `BodyDef.GravityScale`, `LinearDamping`, `AngularDamping` | |
| freeze rotation constraint | `BodyDef.FixedRotation` | |
| `CollisionDetectionMode2D.Continuous` | `BodyDef.IsBullet` | Static geometry is always swept; bullets also sweep against other moving bodies. |
| `Destroy(gameObject)` | `body.Destroy()` | A destroyed handle throws `StaleHandleException`. |
| `BoxCollider2D`, `CircleCollider2D`, `CapsuleCollider2D`, `PolygonCollider2D`, `EdgeCollider2D` | `AddBox`, `AddCircle`, `AddCapsule`, `AddPolygon`, `AddSegment`/`AddChain`; or `CreateBox` etc. on an existing body | A body can have many shapes. |
| `collider.isTrigger` | `ShapeDef.IsSensor` | |
| `PhysicsMaterial2D` (friction, bounciness) | `Material2D(friction, restitution, rollingResistance)`, or `ShapeDef` fields | |
| `gameObject.layer` | `body.Layer` (0 to 31) | |
| `Physics2D.IgnoreLayerCollision(a, b)` | `world.IgnoreLayerCollision(a, b)` | |
| `OnCollisionEnter2D`, `Stay`, `Exit` | `body.OnCollisionEnter`, `OnCollisionStay`, `OnCollisionExit` | Stay needs `EventMask.ContactPersist` in the world options. |
| `collision.gameObject`, `collision.collider` | `c.Other`, `c.OtherShape` | |
| `collision.GetContact(0).point` | `c.Point`, `c.GetPoint(i)` | |
| `collision.relativeVelocity` | `c.ApproachSpeed` | Approach speed along the normal. |
| `OnTriggerEnter2D`, `OnTriggerExit2D` | `body.OnTriggerEnter`, `OnTriggerExit` | `Trigger.Other`. |
| `Physics2D.Raycast`, `RaycastAll` | `world.Raycast`, `RaycastAll` | Positions are `double`; the direction is a `float` vector. |
| `Physics2D.CircleCast`, `BoxCast`, `CapsuleCast` | `world.CircleCast`, `BoxCast`, `CapsuleCast` | |
| `Physics2D.OverlapCircle`, `OverlapBox`, `OverlapPoint` | `world.OverlapCircle`, `OverlapBox`, `OverlapPoint` (and `...All`, `...Any`) | |
| `Physics2D.OverlapCircleNonAlloc` | `world.OverlapCircleAll(center, radius, span)` | Fills your span. |
| `Collider2D.ClosestPoint`, `Physics2D.Distance` | `world.ClosestPoint` | |
| `ContactFilter2D` (layer mask, trigger toggle) | `QueryFilter2D` (`LayerMask`, `IncludeSensors`, `Predicate`) | |
| `HingeJoint2D` | `world.AddHinge(a, b, anchor)` returning `HingeJoint` | Radians. |
| `SliderJoint2D`, `FixedJoint2D`, `WheelJoint2D` | `AddSlider`, `AddFixed`, `AddWheel` | |
| `DistanceJoint2D`, `SpringJoint2D` | `AddDistance`, `AddSpring` | |
| `TargetJoint2D`, `RelativeJoint2D` | `AddTarget`, `AddRelative` | |
| `CharacterController.Move(motion)` | `character.Move(displacement, dt)` | Displacement for this frame. |
| `CharacterController.SimpleMove(v)` | `character.MoveVelocity(velocity, dt)` | |
| `isGrounded`, `slopeLimit` (degrees), `stepOffset` | `IsGrounded`, `SlopeLimitRadians`, `StepOffset` | Radians. |
| `OnControllerColliderHit` | `character.OnControllerColliderHit` | |
| `Physics.Raycast`, `Rigidbody`, ... (3D) | the same names on `PhysicsWorld3D` and its `Body` | The 3D engine is a preview. |

The joint one-liner to definition to view table is in [names and conventions](api-names.md). If a Unity feature you rely on is missing, the middle tier ([the tiers](tiers.md)) usually has it under the engine's own name.
