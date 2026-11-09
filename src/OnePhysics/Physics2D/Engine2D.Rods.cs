using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IRodHost
{
    private RodStore? _rods;
    internal RodStore Rods => _rods ??= new RodStore(this);
    RodStore IRodHost.Rods => Rods;
    WorldCore IRodHost.RodCore => Core;
    int IRodHost.EngineSectionId => Id;
    bool IRodHost.Planar => true;
    Vector3F IRodHost.Gravity => new(Opt.Gravity.X,Opt.Gravity.Y,0);
    ReplayMutationScope IRodHost.RodMutation(string name) => World!.RecordMutation(name);
}
