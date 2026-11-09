using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Catto, Understanding Constraints (2014): solve the point's coupled effective mass,
    // accumulate impulses, then project the accumulated vector onto the force/torque ball.
    private static void CappedBlock(ref JS3 j, Span<float> p, int slot, JRowBuf rows, Vector3F error, Soft soft, float maxImpulse, bool warm, JointRowBits role = JointRowBits.Motor)
    {
        for (int i = 0; i < 3; i++) { j.Row(slot + i, rows[i], i == 0 ? error.X : i == 1 ? error.Y : error.Z, role | JointRowBits.Active); }
        if (warm) { for (int i = 0; i < 3; i++) { j.Apply(rows[i], p[slot + i]); } return; }
        float k00 = j.K(rows[0], rows[0]), k01 = j.K(rows[0], rows[1]), k02 = j.K(rows[0], rows[2]);
        float k11 = j.K(rows[1], rows[1]), k12 = j.K(rows[1], rows[2]), k22 = j.K(rows[2], rows[2]);
        float c00 = k11 * k22 - k12 * k12, c01 = k02 * k12 - k01 * k22, c02 = k01 * k12 - k02 * k11;
        float det = k00 * c00 + k01 * c01 + k02 * c02;
        if (det != 0f) { det = 1f / det; }
        float c11 = k00 * k22 - k02 * k02, c12 = k01 * k02 - k00 * k12, c22 = k00 * k11 - k01 * k01;
        float r0 = soft.Ms * j.Cdot(rows[0]) + soft.Kk * error.X;
        float r1 = soft.Ms * j.Cdot(rows[1]) + soft.Kk * error.Y;
        float r2 = soft.Ms * j.Cdot(rows[2]) + soft.Kk * error.Z;
        var old = new Vector3F(p[slot], p[slot + 1], p[slot + 2]);
        var next = new Vector3F(old.X - det * (c00 * r0 + c01 * r1 + c02 * r2) - soft.Isc * old.X,
            old.Y - det * (c01 * r0 + c11 * r1 + c12 * r2) - soft.Isc * old.Y,
            old.Z - det * (c02 * r0 + c12 * r1 + c22 * r2) - soft.Isc * old.Z);
        float len = next.Length();
        if (len > maxImpulse) { next *= maxImpulse / len; }
        var delta = next - old;
        p[slot] = next.X; p[slot + 1] = next.Y; p[slot + 2] = next.Z;
        j.Apply(rows[0], delta.X); j.Apply(rows[1], delta.Y); j.Apply(rows[2], delta.Z);
    }

    private static void SolveMotor(ref JS3 j, in JGeo g, in JointData3D d, Span<float> p, bool warm, float invH)
    {
        var q = Relative(g);
        var soft = new Soft(1f, d.Length * invH, 0f);
        JRowBuf rows = default;
        rows[0] = JointRow.Ang(g.AxA); rows[1] = JointRow.Ang(g.AyA); rows[2] = JointRow.Ang(g.AzA);
        CappedBlock(ref j, p, 3, rows, new(2f * q.X, 2f * q.Y, 2f * q.Z), soft, d.Ratio / invH, warm);
        rows[0] = JointRow.Lin(UX, g.RA, g.RB); rows[1] = JointRow.Lin(UY, g.RA, g.RB); rows[2] = JointRow.Lin(UZ, g.RA, g.RB);
        CappedBlock(ref j, p, 0, rows, g.C, soft, d.MaxMotor / invH, warm);
    }

    private static void SolveMouse(ref JS3 j, in JGeo g, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias)
    {
        if (!warm && !useBias) { return; }
        JRowBuf rows = default;
        rows[0] = new JointRow { LB = UX, AB = Vector3F.Cross(g.RB, UX) };
        rows[1] = new JointRow { LB = UY, AB = Vector3F.Cross(g.RB, UY) };
        rows[2] = new JointRow { LB = UZ, AB = Vector3F.Cross(g.RB, UZ) };
        CappedBlock(ref j, p, 0, rows, g.DpB + g.RB - V(s, JRow3.PulleyA), new Soft(s[JRow3.SMass], s[JRow3.SBias], s[JRow3.SImp]), s[JRow3.MaxMotorImp], warm, JointRowBits.Spring);
    }

    private static void SolveWheel(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        var suspension = JointRow.Lin(g.AxA, g.E, g.RB);
        float travel = Vector3F.Dot(g.AxA, g.C);
        if (d.Spring && d.Hertz > 0f) { SpringRow(ref j, p, JP.Spring, suspension, travel, s[JRow3.SBias], s[JRow3.SMass], s[JRow3.SImp], useBias, warm); }
        else { p[JP.Spring] = 0f; }
        if (d.Motor) { MotorRow(ref j, p, JP.Motor, JointRow.Ang(g.AyA), d.MotorSpeed, s[JRow3.MaxMotorImp], warm); }
        if (d.Limit)
        {
            LimitRow(ref j, p, JP.Lower, suspension, travel - d.Lower, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm);
            LimitRow(ref j, p, JP.Upper, suspension.Neg(), d.Upper - travel, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm);
        }
        JRowBuf rows = default; JF3 errors = default;
        var spinError = Vector3F.Cross(g.AyA, g.FB.Rotate(UY));
        rows[0] = JointRow.Ang(g.AxA); rows[1] = JointRow.Ang(g.AzA);
        errors[0] = Vector3F.Dot(spinError, g.AxA); errors[1] = Vector3F.Dot(spinError, g.AzA);
        Block(ref j, p, 3, 2, rows, errors, rigid, warm);
        rows[0] = JointRow.Lin(g.AyA, g.E, g.RB); rows[1] = JointRow.Lin(g.AzA, g.E, g.RB);
        errors[0] = Vector3F.Dot(g.AyA, g.C); errors[1] = Vector3F.Dot(g.AzA, g.C);
        Block(ref j, p, 0, 2, rows, errors, rigid, warm);
    }
}
