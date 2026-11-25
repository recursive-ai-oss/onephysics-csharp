using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Creates the authority's side of an interest stream for one client: <see cref="InterestWriter.Write"/> gives a snapshot of what lies inside an <see cref="InterestRegion"/> and then deltas, with kinematic proxies for the bodies
    /// outside that the inside bodies touch or hang from. The writer reads this world and never changes its simulation state.
    /// </summary>
    /// <param name="options">Whether proxies are sent; default yes.</param>
    public InterestWriter CreateInterestWriter(InterestOptions? options = null) => new(_engine, "PhysicsWorld3D.InterestWriter", options);

    /// <summary>
    /// Creates the client's side of an interest stream on this world: <see cref="InterestClient.Apply"/> builds and updates bodies here from the authority's messages. The world may hold other bodies and may step between messages.
    /// </summary>
    public InterestClient CreateInterestClient() => new(_engine, "PhysicsWorld3D.ApplyInterest", op => RecordMutation(op));

    /// <summary>
    /// The hash of the bodies inside <paramref name="region"/> right now, in the form a client's <see cref="InterestClient.ComputeSharedHash"/> computes over the same bodies: slot, generation, type, position, rotation, linear and angular
    /// velocity of each inside body in ascending slot order. Proxies, joints, contacts and sleeping are not in it.
    /// </summary>
    public ulong ComputeInterestHash(InterestRegion region) => CreateInterestWriter().ComputeSharedHash(region);

    /// <summary>One-shot interest snapshot: everything inside <paramref name="region"/> (with proxies by default) as one message that <see cref="RestoreInterest"/> or any <see cref="InterestClient"/> can apply to a world of any other content.</summary>
    public byte[] SnapshotInterest(InterestRegion region, InterestOptions? options = null) => CreateInterestWriter(options).Write(region);

    /// <summary>
    /// Builds the bodies of an interest snapshot in this world, next to whatever it already holds (different bodies, different slots), and returns the client, which can go on applying the authority's deltas. Where the region
    /// is isolated, stepping this world continues the region exactly as the source world does.
    /// </summary>
    /// <exception cref="InterestStreamException">The bytes are not a snapshot message for this world.</exception>
    public InterestClient RestoreInterest(ReadOnlySpan<byte> snapshot)
    {
        var client = CreateInterestClient();
        client.Apply(snapshot);
        return client;
    }
}
