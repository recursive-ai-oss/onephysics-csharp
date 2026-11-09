using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Catto 2014: axis-restricted inverse mass and inertia participate in J M^-1 J^T.
    private void PrepareControlledContacts(ReadOnlySpan<int> constraints)
    {
        var cs = Core.Constraints;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) != ConstraintKind.Contact) continue;
            int slot = SlotOf[c], lane = slot & 7; var w = Wide.AsSpan((slot >> 3)*ContactGroupFloats, ContactGroupFloats);
            int a = cs.BodyA[c], b = cs.BodyB[c]; var ca = Core.Controls.Get(a).Settings; var cb = Core.Controls.Get(b).Settings;
            float scalarA = Core.Bodies.InverseMass[a], scalarB = Core.Bodies.InverseMass[b];
            w[W3D.MassAY*8+lane] = scalarA; w[W3D.MassAZ*8+lane] = scalarA; w[W3D.MassBY*8+lane] = scalarB; w[W3D.MassBZ*8+lane] = scalarB;
            w[W3D.PushLimit*8+lane] = Math.Min(Opt.PushOutVelocity, Math.Min(ca.MaxDepenetrationSpeed, cb.MaxDepenetrationSpeed));
            if ((ca.Constraints | cb.Constraints) == 0) continue;
            var ma = new Vector3F(Core.AxisMass(a,0), Core.AxisMass(a,1), Core.AxisMass(a,2)); var mb = new Vector3F(Core.AxisMass(b,0), Core.AxisMass(b,1), Core.AxisMass(b,2));
            w[W3D.InvMassA*8+lane] = ma.X; w[W3D.MassAY*8+lane] = ma.Y; w[W3D.MassAZ*8+lane] = ma.Z;
            w[W3D.InvMassB*8+lane] = mb.X; w[W3D.MassBY*8+lane] = mb.Y; w[W3D.MassBZ*8+lane] = mb.Z;
            if ((((int)ca.Constraints | (int)cb.Constraints) & 56) != 0)
            {
                var ia=WInvI.AsSpan(6*a,6);var ib=WInvI.AsSpan(6*b,6);
                var eigen=SymmetricEigenF.Decompose(new Matrix3x3F(ia[0]+ib[0],ia[1]+ib[1],ia[2]+ib[2],ia[1]+ib[1],ia[3]+ib[3],ia[4]+ib[4],ia[2]+ib[2],ia[4]+ib[4],ia[5]+ib[5]));
                var values=eigen.Values;float floor=Math.Max(values.Z,0)*1e-6f;
                var inverse=InertiaF.Rotate(new Matrix3x3F(values.X>floor ? 1/values.X : 0,0,0,0,values.Y>floor ? 1/values.Y : 0,0,0,0,values.Z>floor ? 1/values.Z : 0),eigen.Vectors);
                w[W3D.RollMass*8+lane]=inverse.M11;w[(W3D.RollMass+1)*8+lane]=inverse.M12;w[(W3D.RollMass+2)*8+lane]=inverse.M13;w[(W3D.RollMass+3)*8+lane]=inverse.M22;w[(W3D.RollMass+4)*8+lane]=inverse.M23;w[(W3D.RollMass+5)*8+lane]=inverse.M33;
            }
            for (int i = 0; i < cs.PointCount[c]; i++)
            {
                int o = W3D.Point0+i*W3D.PointStride;
                var ra = new Vector3F(w[(o+W3D.ArmA)*8+lane], w[(o+W3D.ArmA+1)*8+lane], w[(o+W3D.ArmA+2)*8+lane]);
                var rb = new Vector3F(w[(o+W3D.ArmB)*8+lane], w[(o+W3D.ArmB+1)*8+lane], w[(o+W3D.ArmB+2)*8+lane]);
                for (int j = 0; j < 3; j++)
                {
                    int f = j == 0 ? W3D.N : j == 1 ? W3D.T1 : W3D.T2;
                    var dir = new Vector3F(w[f*8+lane], w[(f+1)*8+lane], w[(f+2)*8+lane]);
                    var aa = Vector3F.Cross(ra, dir); var ab = Vector3F.Cross(rb, dir);
                    float k = (ma.X+mb.X)*dir.X*dir.X + (ma.Y+mb.Y)*dir.Y*dir.Y + (ma.Z+mb.Z)*dir.Z*dir.Z + Vector3F.Dot(aa, ApplyWorldInertia(a,aa)) + Vector3F.Dot(ab,ApplyWorldInertia(b,ab));
                    w[(o+(j == 0 ? W3D.NormalMass : j == 1 ? W3D.Tangent1Mass : W3D.Tangent2Mass))*8+lane] = k > 0 ? 1/k : 0;
                }
            }
        }
    }
}
