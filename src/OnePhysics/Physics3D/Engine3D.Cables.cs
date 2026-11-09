using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Cables;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : ICableHost
{
    private CableStore? _cables;
    internal CableStore Cables => _cables ??= new CableStore(this);
    CableStore ICableHost.Cables => Cables;
    WorldCore ICableHost.CableCore => Core;
    int ICableHost.EngineSectionId => Id;
    bool ICableHost.Planar => false;
    Vector3F ICableHost.Gravity => new(Opt.Gravity.X, Opt.Gravity.Y, Opt.Gravity.Z);
    float ICableHost.GravityScaleOf(int b) => GravityScale[b];
    Vector3F ICableHost.LocalCenter(int b) => LocalCenterOf(b);
    void ICableHost.AddLoad(int b, Vector3F force, Vector3F torque)
    {
        Force[3 * b] += force.X; Force[3 * b + 1] += force.Y; Force[3 * b + 2] += force.Z;
        Torque[3 * b] += torque.X; Torque[3 * b + 1] += torque.Y; Torque[3 * b + 2] += torque.Z;
    }
    ReplayMutationScope ICableHost.CableMutation(string name) => World!.RecordMutation(name);
}
