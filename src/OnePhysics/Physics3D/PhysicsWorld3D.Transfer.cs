using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Takes a set of bodies out of this world into a <see cref="BodySetPacket"/>: their shapes, the joints among them, the contacts among them with their warm-start state, and what a joint across the boundary needs
    /// (see <see cref="BodySetOptions.Boundary"/>). By default the bodies are moved: they are destroyed here once the packet is built. Contacts with static shapes outside the set are carried when the receiving world holds the same shape at the same pose; other contacts with outside bodies are not, and the receiving world finds them again. A ragdoll goes whole: naming one of its parts takes all of them. Characters and vehicles travel whole with their bodies (list a driver with her car); `docs/spec/transfer.md` says what each carries.
    /// </summary>
    /// <param name="bodies">Live bodies of this world, each listed once.</param>
    /// <param name="options">Move or copy, and the boundary policy; the defaults move the set and refuse a joint that crosses its boundary.</param>
    /// <exception cref="ArgumentException">No bodies, or a body listed twice.</exception>
    /// <exception cref="InvalidHandleException">A body is the default value, destroyed (<see cref="StaleHandleException"/>) or from another world.</exception>
    /// <exception cref="BodySetTransferException">A joint cuts through the set and the policy is <see cref="BoundaryPolicy.Refuse"/>, or a body takes part in something a packet does not carry (a bicycle, rail vehicle, articulation, rod, cable, actuator, cloth link, breakable or streamed tile, an active ragdoll), or a character holds a rope or a body that is not in the set, or a shape or joint kind is not carried. The world is unchanged.</exception>
    public BodySetPacket ExtractBodies(ReadOnlySpan<Body> bodies, BodySetOptions? options = null)
    {
        var handles = new BodyHandle[bodies.Length];
        for (int i = 0; i < handles.Length; i++) { handles[i] = bodies[i].Handle; }
        options ??= new BodySetOptions();
        _core.FlushPostStep();
        using var replayMutation = options.Move ? RecordMutation("PhysicsWorld3D.ExtractBodies") : default;
        return BodySetTransfer.Extract(_engine, handles, options, "PhysicsWorld3D.ExtractBodies");
    }

    /// <summary>
    /// Puts the bodies of a packet into this world, optionally through a rigid transform (a portal): positions go to <c>rotation * p + at</c>, velocities, spins and orientations are rotated with them, and warm-start
    /// impulses along the contact normals are kept. Bodies, shapes and joints take the lowest free slots in packet order (bodies by their old slot, then anchors; shapes, joints by their old slot), so two worlds in the same state
    /// that insert the same packet are in the same state afterwards. Nothing is simulated: the new bodies take part from the next step.
    /// </summary>
    /// <param name="packet">A packet made by <see cref="ExtractBodies"/> or read with <see cref="BodySetPacket.Read"/>.</param>
    /// <param name="at">Translation of the transform, in world units; default none.</param>
    /// <param name="rotation">Rotation of the transform about the world origin; default none.</param>
    /// <returns>The old and new handles of every body, shape and joint.</returns>
    /// <exception cref="BodySetTransferException">The packet is 2D, its length unit differs from this world's, or a body would land beyond +-2^21 m. The world is unchanged.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The translation or rotation is not finite or the rotation is zero.</exception>
    public BodySetHandleMap InsertBodies(BodySetPacket packet, Vector3D at = default, QuaternionF? rotation = null)
    {
        ArgumentNullException.ThrowIfNull(packet);
        var transform = TransferTransform.Create3D(at, rotation);
        _core.FlushPostStep();
        using var replayMutation = RecordMutation("PhysicsWorld3D.InsertBodies");
        return BodySetTransfer.Insert(_engine, packet, transform, "PhysicsWorld3D.InsertBodies");
    }
}
