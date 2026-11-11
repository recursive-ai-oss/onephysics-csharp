using OneScope.Sdk;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// Entity ids: the kind in the top byte, then the handle's generation (24 bits) and slot (32 bits), so an id is never reused
/// for another body or joint and needs no table. The world itself is id 1.
/// </summary>
internal static class EntityIds
{
    public static readonly EntityId World = new(1);

    private const ulong BodyKind = 1UL << 56;
    private const ulong JointKind = 2UL << 56;

    public static EntityId Of(BodyHandle h) => new(BodyKind | Pack(h.Generation, h.Index));

    public static EntityId Of(JointHandle h) => new(JointKind | Pack(h.Generation, h.Index));

    public static bool TryBody(ulong id, int worldId, out BodyHandle handle)
    {
        handle = default;
        if ((id >> 56) != 1)
        {
            return false;
        }

        handle = new BodyHandle(worldId, (int)(uint)id, (int)((id >> 32) & 0xFFFFFF));
        return true;
    }

    public static bool TryJoint(ulong id, int worldId, out JointHandle handle)
    {
        handle = default;
        if ((id >> 56) != 2)
        {
            return false;
        }

        handle = new JointHandle(worldId, (int)(uint)id, (int)((id >> 32) & 0xFFFFFF));
        return true;
    }

    private static ulong Pack(int generation, int index) => ((ulong)(uint)generation & 0xFFFFFF) << 32 | (uint)index;
}
