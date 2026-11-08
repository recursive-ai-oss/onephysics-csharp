using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class RagdollState
{
    internal RagdollState CloneForFork()
    {
        var c = (RagdollState)MemberwiseClone();
        c.Body = (int[])Body.Clone();
        c.BodyGen = (int[])BodyGen.Clone();
        c.Joint = (int[])Joint.Clone();
        c.JointGen = (int[])JointGen.Clone();
        c.BindRot = (QuaternionF[])BindRot.Clone();
        c.JointRot = (QuaternionF[])JointRot.Clone();
        c.TargetRot = (QuaternionF[])TargetRot.Clone();
        c.TargetPos = (Vector3D[])TargetPos.Clone();
        c.Def = Def.Clone();
        return c;
    }
}
