using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private readonly struct MachineRow(float lax, float lay, float aa, float lbx, float lby, float ab)
    {
        public readonly float LAx = lax, LAy = lay, AA = aa, LBx = lbx, LBy = lby, AB = ab;
        public float Velocity(in JS j) => ((LAx * j.VAx + LAy * j.VAy) + AA * j.WA) + ((LBx * j.VBx + LBy * j.VBy) + AB * j.WB);
        public float Mass(in JS j)
        {
            float k = j.MA * (LAx * LAx + LAy * LAy) + j.IA * AA * AA + j.MB * (LBx * LBx + LBy * LBy) + j.IB * AB * AB;
            return k > 0f ? 1f / k : 0f;
        }
        public void Apply(ref JS j, float impulse)
        {
            if (j.Readback.Enabled) { j.Readback.Add(j.Constraint, new(LAx, LAy, 0), new(0, 0, AA), new(LBx, LBy, 0), new(0, 0, AB), impulse); }
            j.VAx += j.MA * LAx * impulse; j.VAy += j.MA * LAy * impulse; j.WA += j.IA * AA * impulse;
            j.VBx += j.MB * LBx * impulse; j.VBy += j.MB * LBy * impulse; j.WB += j.IB * AB * impulse;
        }
        public JointRowBits Units => LAx == 0f && LAy == 0f && LBx == 0f && LBy == 0f ? JointRowBits.Angular : 0;
        public MachineRow Neg() => new(-LAx, -LAy, -AA, -LBx, -LBy, -AB);
    }
    private static MachineRow MovingAxis(in JS j, float x, float y) => new(-x, -y, -((j.Dx + j.RBx) * y - (j.Dy + j.RBy) * x), x, y, j.RBx * y - j.RBy * x);
    private static void MachineSolve(ref JS j, MachineRow r, Span<float> p, int slot, float error, float ms, float kk, float isc, bool warm, JointRowBits bits = JointRowBits.Active)
    {
        if (j.Readback.Enabled) { j.Readback.Row(j.Constraint, slot, (bits & JointRowBits.Velocity) != 0 ? r.Velocity(j) : error, bits | r.Units); }
        if (warm) { r.Apply(ref j, p[slot]); return; }
        float delta = -r.Mass(j) * (ms * r.Velocity(j) + kk * error) - isc * p[slot];
        p[slot] += delta; r.Apply(ref j, delta);
    }
    private static void MachineLimit(ref JS j, MachineRow r, Span<float> p, int slot, float error, ReadOnlySpan<float> s, float invH, bool warm, bool bias)
    {
        if (j.Readback.Enabled) { j.Readback.Row(j.Constraint, slot, error, JointRowBits.Limit | r.Units | (error <= 0f || p[slot] > 0f ? JointRowBits.Active : 0)); }
        if (warm) { r.Apply(ref j, p[slot]); return; }
        float impulse = LimitImpulse(bias, error, r.Velocity(j), r.Mass(j), ref p[slot], s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
        r.Apply(ref j, impulse);
        if (j.Readback.Enabled) { j.Readback.Row(j.Constraint, slot, error, JointRowBits.Limit | r.Units | (error <= 0f || p[slot] > 0f ? JointRowBits.Active : 0)); }
    }
    private static void MachineMotor(ref JS j, MachineRow r, Span<float> p, int slot, float speed, float cap, bool warm)
    {
        if (j.Readback.Enabled) { j.Readback.Row(j.Constraint, slot, r.Velocity(j) - speed, JointRowBits.Motor | JointRowBits.Velocity | JointRowBits.Active | r.Units); }
        if (warm) { r.Apply(ref j, p[slot]); return; }
        float next = Clamp(p[slot] - r.Mass(j) * (r.Velocity(j) - speed), cap);
        float delta = next - p[slot]; p[slot] = next; r.Apply(ref j, delta);
    }
    private void SolveMachine(int c, ref JS j, in JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, bool bias, float invH)
    {
        var cs = Core.Constraints; int a = cs.BodyA[c], b = cs.BodyB[c];
        float ms = bias ? s[JRow.JMass] : 1f, kk = bias ? s[JRow.JMass] * s[JRow.JBias] : 0f, isc = bias ? s[JRow.JImp] : 0f;
        if (d.Type == (byte)JointType2D.Gear)
        {
            MachineSolve(ref j, new(0, 0, 1, 0, 0, d.OffAngle), p, 0, 0, 1, 0, 0, warm, JointRowBits.Velocity | JointRowBits.Active); return;
        }
        if (d.Type == (byte)JointType2D.RackAndPinion)
        {
            var qb = Core.Bodies.RotationOf(b);
            float co = j.DBc * qb[0] - j.DBs * qb[1], si = j.DBs * qb[0] + j.DBc * qb[1];
            float x = co * d.AxX - si * d.AxY, y = si * d.AxX + co * d.AxY;
            MachineSolve(ref j, new(0, 0, d.OffAngle, -x, -y, 0), p, 0, 0, 1, 0, 0, warm, JointRowBits.Velocity | JointRowBits.Active); return;
        }
        var m = Machines[c]!;
        if (d.Type == (byte)JointType2D.Pulley)
        {
            var pa = Core.Bodies.PositionOf(a); var pb = Core.Bodies.PositionOf(b);
            int oa = (a + 1) * BS2D.Stride, ob = (b + 1) * BS2D.Stride;
            var ua = new Vector2F((float)(pa[0] - m.FixedA.X) + BodyState[oa + BS2D.Dpx] + j.RAx, (float)(pa[1] - m.FixedA.Y) + BodyState[oa + BS2D.Dpy] + j.RAy);
            var ub = new Vector2F((float)(pb[0] - m.FixedB.X) + BodyState[ob + BS2D.Dpx] + j.RBx, (float)(pb[1] - m.FixedB.Y) + BodyState[ob + BS2D.Dpy] + j.RBy);
            float la = ua.Length(), lb = ub.Length(); var na = la > 1e-4f ? ua / la : default; var nb = lb > 1e-4f ? ub / lb : default;
            var r = new MachineRow(na.X, na.Y, j.RAx * na.Y - j.RAy * na.X, nb.X * d.OffAngle, nb.Y * d.OffAngle, (j.RBx * nb.Y - j.RBy * nb.X) * d.OffAngle);
            float length = la + d.OffAngle * lb;
            if (d.Lower == d.Upper) { MachineSolve(ref j, r, p, 0, length - d.Lower, ms, kk, isc, warm); }
            else
            {
                MachineLimit(ref j, r, p, 1, length - d.Lower, s, invH, warm, bias);
                MachineLimit(ref j, r.Neg(), p, 2, d.Upper - length, s, invH, warm, bias);
            }
            return;
        }
        var qa = Core.Bodies.RotationOf(a);
        float ac = j.DAc * qa[0] - j.DAs * qa[1], asi = j.DAs * qa[0] + j.DAc * qa[1];
        if (d.Type == (byte)JointType2D.Path)
        {
            float ex = j.Dx + j.RBx, ey = j.Dy + j.RBy;
            var query = new Vector2F(ac * ex + asi * ey + LocalCenter[2 * a], -asi * ex + ac * ey + LocalCenter[2 * a + 1]);
            PathSpline2D.Closest(m.Points, m.Loop, query, out float t, out var point, out var tangent);
            float len = tangent.Length(); tangent = len > 1e-8f ? tangent / len : new(1, 0);
            float tx = ac * tangent.X - asi * tangent.Y, ty = asi * tangent.X + ac * tangent.Y;
            point -= new Vector2F(LocalCenter[2 * a], LocalCenter[2 * a + 1]);
            float dx = ex - (ac * point.X - asi * point.Y), dy = ey - (asi * point.X + ac * point.Y);
            var along = MovingAxis(j, tx, ty);
            if (d.Motor) { MachineMotor(ref j, along, p, 1, d.MotorSpeed, d.MaxMotor / invH, warm); }
            if (!m.Loop)
            {
                if (t <= 0f) { MachineLimit(ref j, along, p, 2, dx * tx + dy * ty, s, invH, warm, bias); }
                else if (t >= m.Points.Length - 1) { MachineLimit(ref j, along.Neg(), p, 2, -(dx * tx + dy * ty), s, invH, warm, bias); }
                else if (warm) { along.Apply(ref j, p[2]); }
                else { along.Apply(ref j, -p[2]); p[2] = 0f; }
            }
            MachineSolve(ref j, MovingAxis(j, -ty, tx), p, 0, -dx * ty + dy * tx, ms, kk, isc, warm);
            if (m.Align)
            {
                var qb = Core.Bodies.RotationOf(b); float bc = j.DBc * qb[0] - j.DBs * qb[1], bs = j.DBs * qb[0] + j.DBc * qb[1];
                float error = (float)Math64.Atan2(tx * bs - ty * bc, tx * bc + ty * bs);
                MachineSolve(ref j, new(0, 0, -1, 0, 0, 1), p, 3, error, ms, kk, isc, warm);
            }
            return;
        }
        var (sn, coFrame) = Math64.SinCos(d.OffAngle);
        float ax = ac * (float)coFrame - asi * (float)sn, ay = asi * (float)coFrame + ac * (float)sn;
        float cx = j.Dx + j.RBx - j.RAx, cy = j.Dy + j.RBy - j.RAy;
        SolveConfigurableAxis(ref j, MovingAxis(j, ax, ay), p, 0, cx * ax + cy * ay, m.X, s, invH, warm, bias);
        SolveConfigurableAxis(ref j, MovingAxis(j, -ay, ax), p, 5, -cx * ay + cy * ax, m.Y, s, invH, warm, bias);
        SolveConfigurableAxis(ref j, new(0, 0, -1, 0, 0, 1), p, 10, ExactAngle(j, s), m.Angle, s, invH, warm, bias);
    }
    private static void SolveConfigurableAxis(ref JS j, MachineRow r, Span<float> p, int slot, float error, JointAxisSettings a, ReadOnlySpan<float> s, float invH, bool warm, bool bias)
    {
        if (a.EnableSpring && a.SpringHertz > 0f)
        {
            MakeSpring(a.SpringHertz, a.SpringDampingRatio, 1f / invH, out float k, out float mass, out float impulse);
            if (warm || bias) { MachineSolve(ref j, r, p, slot + 1, error, mass, k, impulse, warm, JointRowBits.Spring | JointRowBits.Active); }
        }
        if (a.EnableMotor) { MachineMotor(ref j, r, p, slot + 2, a.MotorSpeed, a.MaxMotor / invH, warm); }
        if (a.Mode == JointAxisMode.Limited)
        {
            MachineLimit(ref j, r, p, slot + 3, error - a.Lower, s, invH, warm, bias);
            MachineLimit(ref j, r.Neg(), p, slot + 4, a.Upper - error, s, invH, warm, bias);
        }
        if (a.Mode == JointAxisMode.Locked) { MachineSolve(ref j, r, p, slot, error, bias ? s[JRow.JMass] : 1, bias ? s[JRow.JMass] * s[JRow.JBias] : 0, bias ? s[JRow.JImp] : 0, warm); }
    }
}
