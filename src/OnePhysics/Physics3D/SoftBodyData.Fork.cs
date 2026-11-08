using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class SoftBodyData
{
    internal SoftBodyData CloneForFork()
    {
        var c = (SoftBodyData)MemberwiseClone();
        c.Pos = (double[])Pos.Clone();
        c.Prev = (double[])Prev.Clone();
        c.Vel = (float[])Vel.Clone();
        c.InvMass0 = (float[])InvMass0.Clone();
        c.InvMass = (float[])InvMass.Clone();
        c.Pinned = (byte[])Pinned.Clone();
        c.Rest = (float[])Rest.Clone();
        c.EdgeA = (int[])EdgeA.Clone();
        c.EdgeB = (int[])EdgeB.Clone();
        c.EdgeRest = (float[])EdgeRest.Clone();
        c.EdgeKey = (long[])EdgeKey.Clone();
        c.BendIdx = (int[])BendIdx.Clone();
        c.BendK = (float[])BendK.Clone();
        c.BendS = (float[])BendS.Clone();
        c.BendM0 = (float[])BendM0.Clone();
        c.HingeKey = (long[])HingeKey.Clone();
        c.Tets = (int[])Tets.Clone();
        c.TetRest = (float[])TetRest.Clone();
        c.Tris = (int[])Tris.Clone();
        c.Lambda = (float[])Lambda.Clone();
        c.X = (float[])X.Clone();
        c.XP = (float[])XP.Clone();
        c.GradP = (float[])GradP.Clone();
        c.Bind = (float[])Bind.Clone();
        c.JointPos = (double[])JointPos.Clone();
        c.JointRot = (float[])JointRot.Clone();
        c.JointPosPrev = (double[])JointPosPrev.Clone();
        c.JointRotPrev = (float[])JointRotPrev.Clone();
        c.TgtPrev = (double[])TgtPrev.Clone();
        c.TgtCur = (double[])TgtCur.Clone();
        c.NrmCur = (float[])NrmCur.Clone();
        c.SEdgeA = (int[])SEdgeA.Clone();
        c.SEdgeB = (int[])SEdgeB.Clone();
        c.SEdgeRest = (float[])SEdgeRest.Clone();
        c.SBendIdx = (int[])SBendIdx.Clone();
        c.SBendK = (float[])SBendK.Clone();
        c.SBendS = (float[])SBendS.Clone();
        c.SBendM0 = (float[])SBendM0.Clone();
        c.STets = (int[])STets.Clone();
        c.STetRest = (float[])STetRest.Clone();
        c.BlockOf = (int[])BlockOf.Clone();
        c.StepStart = (int[])StepStart.Clone();
        c.StepKind = (byte[])StepKind.Clone();
        c.StepFirst = (int[])StepFirst.Clone();
        c.StepQuadEnd = (int[])StepQuadEnd.Clone();
        c.StepEnd = (int[])StepEnd.Clone();
        c.NbrStart = (int[])NbrStart.Clone();
        c.NbrList = (int[])NbrList.Clone();
        c.Def = Def.Clone();
        return c;
    }
}
