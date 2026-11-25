using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Takes a set of bodies out of this world into a <see cref="BodySetPacket"/>: their shapes, the joints among them, the contacts among them with their warm-start state, and what a joint across the boundary needs
    /// (see <see cref="BodySetOptions.Boundary"/>). By default the bodies are moved: they are destroyed here once the packet is built. Contacts with bodies outside the set are not carried; the receiving world finds them again.
    /// </summary>
    /// <param name="bodies">Live bodies of this world, each listed once.</param>
    /// <param name="options">Move or copy, and the boundary policy; the defaults move the set and refuse a joint that crosses its boundary.</param>
    /// <exception cref="ArgumentException">No bodies, or a body listed twice.</exception>
    /// <exception cref="InvalidHandleException">A body is the default value, destroyed (<see cref="StaleHandleException"/>) or from another world.</exception>
    /// <exception cref="BodySetTransferException">A joint cuts through the set and the policy is <see cref="BoundaryPolicy.Refuse"/>, or a body takes part in a character, vehicle, ragdoll, articulation, rod, cable, actuator, breakable or streamed tile (a packet carries rigid bodies, shapes, joints and contacts), or a shape or joint kind is not carried. The world is unchanged.</exception>
    public BodySetPacket ExtractBodies(ReadOnlySpan<Body> bodies, BodySetOptions? options = null)
    {
        var handles = new BodyHandle[bodies.Length];
        for (int i = 0; i < handles.Length; i++) { handles[i] = bodies[i].Handle; }
        options ??= new BodySetOptions();
        _core.FlushPostStep();
        using var replayMutation = options.Move ? RecordMutation("PhysicsWorld2D.ExtractBodies") : default;
        return BodySetTransfer.Extract(_engine, handles, options, "PhysicsWorld2D.ExtractBodies");
    }

    /// <summary>
    /// Puts the bodies of a packet into this world, optionally through a rigid transform (a portal): positions go to <c>R(angle) p + at</c>, velocities, spins and orientations are rotated with them, and warm-start
    /// impulses along the contact normals are kept. Bodies, shapes and joints take the lowest free slots in packet order (bodies by their old slot, then anchors; shapes, joints by their old slot), so two worlds in the same state
    /// that insert the same packet are in the same state afterwards. Nothing is simulated: the new bodies take part from the next step.
    /// </summary>
    /// <param name="packet">A packet made by <see cref="ExtractBodies"/> or read with <see cref="BodySetPacket.Read"/>.</param>
    /// <param name="at">Translation of the transform, in world units; default none.</param>
    /// <param name="angle">Rotation of the transform about the world origin, in radians; default none.</param>
    /// <returns>The old and new handles of every body, shape and joint.</returns>
    /// <exception cref="BodySetTransferException">The packet is 2D, its length unit differs from this world's, or a body would land beyond +-2^21 m. The world is unchanged.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The translation or angle is not finite.</exception>
    public BodySetHandleMap InsertBodies(BodySetPacket packet, Vector2D at = default, float angle = 0f)
    {
        ArgumentNullException.ThrowIfNull(packet);
        var transform = TransferTransform.Create2D(at, angle);
        _core.FlushPostStep();
        using var replayMutation = RecordMutation("PhysicsWorld2D.InsertBodies");
        return BodySetTransfer.Insert(_engine, packet, transform, "PhysicsWorld2D.InsertBodies");
    }
}
