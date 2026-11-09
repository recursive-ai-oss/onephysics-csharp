using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // Catto, Understanding Constraints (2014): K = J M^-1 J^T.
    // A locked world axis has zero inverse mass in both K and the impulse response.
    private void PrepareControlledContacts(ReadOnlySpan<int> constraints)
    {
        var cs = Core.Constraints;
        foreach (int c in constraints)
        {
            if (cs.KindOf(c) != ConstraintKind.Contact) continue;
            int slot = SlotOf[c], lane = slot & 7; var w = Wide.AsSpan((slot >> 3) * ContactGroupFloats, ContactGroupFloats);
            int a = cs.BodyA[c], b = cs.BodyB[c]; var ca = Core.Controls.Get(a).Settings; var cb = Core.Controls.Get(b).Settings;
            w[W2D.MassAY*8+lane] = Core.Bodies.InverseMass[a]; w[W2D.MassBY*8+lane] = Core.Bodies.InverseMass[b];
            w[W2D.PushLimit*8+lane] = Math.Min(Opt.PushOutVelocity, Math.Min(ca.MaxDepenetrationSpeed, cb.MaxDepenetrationSpeed));
            if ((ca.Constraints | cb.Constraints) == 0) continue;
            float ax = Core.AxisMass(a,0), ay = Core.AxisMass(a,1), bx = Core.AxisMass(b,0), by = Core.AxisMass(b,1);
            float ia = ((int)ca.Constraints & 32) == 0 ? Core.Bodies.InverseInertia[a] : 0, ib = ((int)cb.Constraints & 32) == 0 ? Core.Bodies.InverseInertia[b] : 0;
            w[W2D.InvMassA*8+lane] = ax; w[W2D.MassAY*8+lane] = ay; w[W2D.InvMassB*8+lane] = bx; w[W2D.MassBY*8+lane] = by;
            w[W2D.InvInertiaA*8+lane] = ia; w[W2D.InvInertiaB*8+lane] = ib; w[W2D.RollingMass*8+lane] = ia + ib > 0 ? 1/(ia+ib) : 0;
            float nx = w[W2D.NormalX*8+lane], ny = w[W2D.NormalY*8+lane];
            for (int i = 0; i < cs.PointCount[c]; i++)
            {
                int o = W2D.Point0 + i*W2D.PointStride;
                float rax = w[(o+W2D.AnchorAX)*8+lane], ray = w[(o+W2D.AnchorAY)*8+lane], rbx = w[(o+W2D.AnchorBX)*8+lane], rby = w[(o+W2D.AnchorBY)*8+lane];
                float an = rax*ny-ray*nx, bn = rbx*ny-rby*nx, at = -rax*nx-ray*ny, bt = -rbx*nx-rby*ny;
                float kn = (ax+bx)*nx*nx + (ay+by)*ny*ny + ia*an*an + ib*bn*bn;
                float kt = (ax+bx)*ny*ny + (ay+by)*nx*nx + ia*at*at + ib*bt*bt;
                w[(o+W2D.NormalMass)*8+lane] = kn > 0 ? 1/kn : 0; w[(o+W2D.TangentMass)*8+lane] = kt > 0 ? 1/kt : 0;
            }
        }
    }
}
