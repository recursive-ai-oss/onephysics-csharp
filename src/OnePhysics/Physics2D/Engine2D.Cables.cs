using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : ICableHost
{
    private CableStore? _cables;
    internal CableStore Cables => _cables ??= new CableStore(this);
    CableStore ICableHost.Cables => Cables;
    WorldCore ICableHost.CableCore => Core;
    int ICableHost.EngineSectionId => Id;
    bool ICableHost.Planar => true;
    Vector3F ICableHost.Gravity => new(Opt.Gravity.X, Opt.Gravity.Y, 0);
    float ICableHost.GravityScaleOf(int b) => GravityScale[b];
    Vector3F ICableHost.LocalCenter(int b) => new(LocalCenter[2 * b], LocalCenter[2 * b + 1], 0);
    void ICableHost.PendingLoad(int b, out Vector3F force, out Vector3F torque)
    {
        force = new(Force[2 * b], Force[2 * b + 1], 0); torque = new(0, 0, Torque[b]);
    }
    void ICableHost.AddLoad(int b, Vector3F force, Vector3F torque)
    {
        Force[2 * b] += force.X; Force[2 * b + 1] += force.Y; Torque[b] += torque.Z;
    }
    ReplayMutationScope ICableHost.CableMutation(string name) => World!.RecordMutation(name);
}
