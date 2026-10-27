using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The 2D joint solver: soft-step rows (Catto, "Soft Step Solver", GDC 2024, and "Solver2D"), solved scalar, one joint at a time. A batch holds joints of one color, which share no dynamic body, so
/// joints in a batch never touch the same data and the order inside it cannot matter. Position errors come from the body motion accumulated in the step (Δp, Δq) so substeps see fresh errors without
/// recomputing poses. Row order inside a joint: spring, motor, limits, then the rigid rows last, so the rigid rows win a disagreement.
/// </summary>
internal sealed partial class Engine2D
{
    /// <summary>Velocities, masses, rotated anchors and motion of one joint's two bodies, loaded once per solve call.</summary>
    private struct JS
    {
        public float VAx, VAy, WA, VBx, VBy, WB, MA, IA, MB, IB;
        public float RAx, RAy, RBx, RBy;   // anchors from the centres of mass at the current rotation
        public float Dx, Dy;               // current centre of mass of B minus that of A
        public float DAc, DAs, DBc, DBs;   // rotation accumulated this step

        /// <summary>Linear impulse (px, py) at the anchors: pushes B along it and A against it.</summary>
        public void Lin(float px, float py)
        {
            VAx -= MA * px; VAy -= MA * py; WA -= IA * (RAx * py - RAy * px);
            VBx += MB * px; VBy += MB * py; WB += IB * (RBx * py - RBy * px);
        }

        /// <summary>Angular impulse: pushes B's rotation up and A's down.</summary>
        public void Ang(float l) { WA -= IA * l; WB += IB * l; }

        /// <summary>Impulse <paramref name="imp"/> along a unit direction with torque arms a1 (A) and a2 (B).</summary>
        public void Axial(float imp, float ax, float ay, float a1, float a2)
        {
            float px = imp * ax, py = imp * ay;
            VAx -= MA * px; VAy -= MA * py; WA -= IA * a1 * imp;
            VBx += MB * px; VBy += MB * py; WB += IB * a2 * imp;
        }

        public readonly float AxialVelocity(float ax, float ay, float a1, float a2) => (ax * (VBx - VAx) + ay * (VBy - VAy)) + a2 * WB - a1 * WA;

        public readonly void RelativePointVelocity(out float x, out float y)
        {
            x = (VBx - WB * RBy) - (VAx - WA * RAy);
            y = (VBy + WB * RBx) - (VAy + WA * RAx);
        }
    }

    private const float Pi = 3.1415927f;

    private void LoadJoint(int a, int b, ReadOnlySpan<float> s, out JS j)
    {
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        j = default;
        j.VAx = lin[2 * a]; j.VAy = lin[2 * a + 1]; j.WA = ang[a];
        j.VBx = lin[2 * b]; j.VBy = lin[2 * b + 1]; j.WB = ang[b];
        j.MA = s[JRow.MA]; j.IA = s[JRow.IA]; j.MB = s[JRow.MB]; j.IB = s[JRow.IB];
        float dAc = Dq[2 * a], dAs = Dq[2 * a + 1], dBc = Dq[2 * b], dBs = Dq[2 * b + 1];
        j.DAc = dAc; j.DAs = dAs; j.DBc = dBc; j.DBs = dBs;
        float rx = s[JRow.RAx], ry = s[JRow.RAy];
        j.RAx = dAc * rx - dAs * ry; j.RAy = dAs * rx + dAc * ry;
        rx = s[JRow.RBx]; ry = s[JRow.RBy];
        j.RBx = dBc * rx - dBs * ry; j.RBy = dBs * rx + dBc * ry;
        j.Dx = s[JRow.Dx] + (Dp[2 * b] - Dp[2 * a]);
        j.Dy = s[JRow.Dy] + (Dp[2 * b + 1] - Dp[2 * a + 1]);
    }

    private void StoreJoint(int a, int b, in JS j)
    {
        var bodies = Core.Bodies;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<byte> type = bodies.Type;
        if (type[a] == (byte)BodyType.Dynamic) { lin[2 * a] = j.VAx; lin[2 * a + 1] = j.VAy; ang[a] = j.WA; }
        if (type[b] == (byte)BodyType.Dynamic) { lin[2 * b] = j.VBx; lin[2 * b + 1] = j.VBy; ang[b] = j.WB; }
    }

    /// <summary>Relative rotation of B in A's frame now (reference removed), as (cos, sin): ΔqB · conj(ΔqA) · q0.</summary>
    private static void RelativeRotation(in JS j, ReadOnlySpan<float> s, out float c, out float sn)
    {
        float tc = j.DBc * j.DAc + j.DBs * j.DAs, ts = j.DBs * j.DAc - j.DBc * j.DAs;
        float q0c = s[JRow.Q0C], q0s = s[JRow.Q0S];
        c = tc * q0c - ts * q0s;
        sn = ts * q0c + tc * q0s;
    }

    /// <summary>The joint angle in radians within -pi..pi (onemath's correctly rounded atan2): used where the value matters, such as limits and springs.</summary>
    private static float ExactAngle(in JS j, ReadOnlySpan<float> s)
    {
        RelativeRotation(in j, s, out float c, out float sn);
        return (float)Math64.Atan2(sn, c);
    }

    /// <summary>
    /// A cheap angle for rigid rows that only need an error to drive to zero: 2 sin(θ) / (1 + cos(θ)) = 2 tan(θ/2), equal to θ to third order and monotonic up to a half turn, with no transcendental call.
    /// </summary>
    private static float SmallAngle(in JS j, ReadOnlySpan<float> s)
    {
        RelativeRotation(in j, s, out float c, out float sn);
        float d = 1f + c;
        if (d < 1e-4f) { d = 1e-4f; }
        float a = 2f * sn / d;
        if (a > Pi) { a = Pi; } else if (a < -Pi) { a = -Pi; }
        return a;
    }

    /// <summary>Soft inequality row (a limit): speculative when the limit is not yet reached, soft when violated.</summary>
    private static float LimitImpulse(bool useBias, float c, float cdot, float mass, ref float lambda, float jb, float jm, float ji, float invH)
    {
        float bias = 0f, ms = 1f, isc = 0f;
        if (c > 0f) { bias = c * invH; }
        else if (useBias) { bias = jb * c; ms = jm; isc = ji; }
        float imp = -mass * ms * (cdot + bias) - isc * lambda;
        float nl = lambda + imp;
        if (nl < 0f) { nl = 0f; }
        imp = nl - lambda;
        lambda = nl;
        return imp;
    }

    /// <summary>Solves K x = r for the symmetric 2×2 matrix [[k11, k12], [k12, k22]]; zero when singular.</summary>
    private static void Solve22(float k11, float k12, float k22, float rx, float ry, out float x, out float y)
    {
        float det = k11 * k22 - k12 * k12;
        if (det != 0f) { det = 1f / det; }
        x = det * (k22 * rx - k12 * ry);
        y = det * (k11 * ry - k12 * rx);
    }

    /// <summary>The two-row point constraint with soft coefficients: impulse = -K^-1 (ms * dv + k * C) - isc * accumulated; accumulates into (lx, ly). A rigid row passes k = ms * biasRate.</summary>
    private static void PointRow(ref JS j, float cx, float cy, float k, float ms, float isc, ref float lx, ref float ly)
    {
        j.RelativePointVelocity(out float dvx, out float dvy);
        float k11 = j.MA + j.MB + j.IA * j.RAy * j.RAy + j.IB * j.RBy * j.RBy;
        float k12 = -j.IA * j.RAx * j.RAy - j.IB * j.RBx * j.RBy;
        float k22 = j.MA + j.MB + j.IA * j.RAx * j.RAx + j.IB * j.RBx * j.RBx;
        Solve22(k11, k12, k22, ms * dvx + k * cx, ms * dvy + k * cy, out float bx, out float by);
        float ix = -bx - isc * lx, iy = -by - isc * ly;
        lx += ix; ly += iy;
        j.Lin(ix, iy);
    }

    private static float Clamp(float v, float lim) => v > lim ? lim : (v < -lim ? -lim : v);

    public void SolveJoint(int c, SolvePhase phase, in StepContext ctx)
    {
        if (phase == SolvePhase.Restitution) { return; }
        var cs = Core.Constraints;
        int a = cs.BodyA[c], b = cs.BodyB[c];
        ref JointData2D d = ref Joints[c];
        var s = JScratch.AsSpan(c * JRow.Stride, JRow.Stride);
        var p = cs.Payload(c);
        LoadJoint(a, b, s, out JS j);
        bool warm = phase == SolvePhase.WarmStart, useBias = phase == SolvePhase.Solve;
        float invH = ctx.InverseSubstepDt;
        switch ((JointType2D)d.Type)
        {
            case JointType2D.Revolute: SolveRevolute(ref j, ref d, s, p, warm, useBias, invH); break;
            case JointType2D.Prismatic: SolvePrismatic(ref j, ref d, s, p, warm, useBias, invH); break;
            case JointType2D.Weld: SolveWeld(ref j, ref d, s, p, warm, useBias); break;
            case JointType2D.Wheel: SolveWheel(ref j, ref d, s, p, warm, useBias, invH); break;
            case JointType2D.Distance: SolveDistance(ref j, ref d, s, p, warm, useBias, invH); break;
            case JointType2D.Motor: SolveMotor(ref j, ref d, s, p, warm, invH); break;
            case JointType2D.Mouse: SolveMouse(ref j, s, p, warm, useBias, b); break;
        }

        StoreJoint(a, b, in j);
    }

    // ------------------------------------------------------------------ revolute

    private static void SolveRevolute(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias, float invH)
    {
        if (warm)
        {
            j.Lin(p[JPay.P0], p[JPay.P1]);
            j.Ang(p[JPay.Spring] + p[JPay.Motor] + p[JPay.Lower] - p[JPay.Upper]);
            return;
        }

        float angMass = s[JRow.AngMass];
        bool springOn = d.Spring && d.Hertz > 0f;
        float theta = (springOn && useBias) || d.Limit ? ExactAngle(in j, s) : 0f;
        if (springOn && useBias)
        {
            float cdot = j.WB - j.WA;
            float imp = -angMass * (s[JRow.SMass] * cdot + s[JRow.SBias] * theta) - s[JRow.SImp] * p[JPay.Spring];
            p[JPay.Spring] += imp;
            j.Ang(imp);
        }

        if (d.Motor)
        {
            float cdot = j.WB - j.WA - d.MotorSpeed;
            float imp = -angMass * cdot;
            float lim = s[JRow.MaxMotorImp];
            float nl = Clamp(p[JPay.Motor] + imp, lim);
            imp = nl - p[JPay.Motor];
            p[JPay.Motor] = nl;
            j.Ang(imp);
        }

        if (d.Limit)
        {
            float lo = p[JPay.Lower], up = p[JPay.Upper];
            float imp = LimitImpulse(useBias, theta - d.Lower, j.WB - j.WA, angMass, ref lo, s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
            p[JPay.Lower] = lo; j.Ang(imp);
            imp = LimitImpulse(useBias, d.Upper - theta, j.WA - j.WB, angMass, ref up, s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
            p[JPay.Upper] = up; j.Ang(-imp);
        }

        float cx = j.Dx + j.RBx - j.RAx, cy = j.Dy + j.RBy - j.RAy;
        float lx = p[JPay.P0], ly = p[JPay.P1];
        if (useBias) { PointRow(ref j, cx, cy, s[JRow.JMass] * s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], ref lx, ref ly); }
        else { PointRow(ref j, cx, cy, 0f, 1f, 0f, ref lx, ref ly); }
        p[JPay.P0] = lx; p[JPay.P1] = ly;
    }

    // ------------------------------------------------------------------ prismatic and wheel

    /// <summary>The current axis, its normal and the torque arms of the axial and perpendicular rows.</summary>
    private struct Axes
    {
        public float Ax, Ay, Nx, Ny, A1, A2, S1, S2, Translation;
    }

    private static Axes ComputeAxes(in JS j, ReadOnlySpan<float> s)
    {
        float x0 = s[JRow.AxX], y0 = s[JRow.AxY];
        var r = default(Axes);
        r.Ax = j.DAc * x0 - j.DAs * y0; r.Ay = j.DAs * x0 + j.DAc * y0;
        r.Nx = -r.Ay; r.Ny = r.Ax;
        float ex = j.Dx + j.RBx, ey = j.Dy + j.RBy; // from A's centre of mass to B's anchor
        r.A1 = ex * r.Ay - ey * r.Ax; r.A2 = j.RBx * r.Ay - j.RBy * r.Ax;
        r.S1 = ex * r.Ny - ey * r.Nx; r.S2 = j.RBx * r.Ny - j.RBy * r.Nx;
        float dx = ex - j.RAx, dy = ey - j.RAy; // anchor to anchor
        r.Translation = r.Ax * dx + r.Ay * dy;
        return r;
    }

    private static void AxialRows(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, in Axes ax, bool useBias, float invH, bool spring, bool limits)
    {
        float k = j.MA + j.MB + j.IA * ax.A1 * ax.A1 + j.IB * ax.A2 * ax.A2;
        float mass = k > 0f ? 1f / k : 0f;
        if (spring && useBias)
        {
            float cdot = j.AxialVelocity(ax.Ax, ax.Ay, ax.A1, ax.A2);
            float imp = -mass * (s[JRow.SMass] * cdot + s[JRow.SBias] * ax.Translation) - s[JRow.SImp] * p[JPay.Spring];
            p[JPay.Spring] += imp;
            j.Axial(imp, ax.Ax, ax.Ay, ax.A1, ax.A2);
        }

        if (d.Motor && d.Type == (byte)JointType2D.Prismatic)
        {
            float cdot = j.AxialVelocity(ax.Ax, ax.Ay, ax.A1, ax.A2) - d.MotorSpeed;
            float imp = -mass * cdot;
            float nl = Clamp(p[JPay.Motor] + imp, s[JRow.MaxMotorImp]);
            imp = nl - p[JPay.Motor];
            p[JPay.Motor] = nl;
            j.Axial(imp, ax.Ax, ax.Ay, ax.A1, ax.A2);
        }

        if (limits)
        {
            float lo = p[JPay.Lower], up = p[JPay.Upper];
            float cdot = j.AxialVelocity(ax.Ax, ax.Ay, ax.A1, ax.A2);
            float imp = LimitImpulse(useBias, ax.Translation - d.Lower, cdot, mass, ref lo, s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
            p[JPay.Lower] = lo; j.Axial(imp, ax.Ax, ax.Ay, ax.A1, ax.A2);
            cdot = j.AxialVelocity(ax.Ax, ax.Ay, ax.A1, ax.A2);
            imp = LimitImpulse(useBias, d.Upper - ax.Translation, -cdot, mass, ref up, s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
            p[JPay.Upper] = up; j.Axial(-imp, ax.Ax, ax.Ay, ax.A1, ax.A2);
        }
    }

    private static void SolvePrismatic(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias, float invH)
    {
        var ax = ComputeAxes(in j, s);
        if (warm)
        {
            float perp = p[JPay.P0], angImp = p[JPay.P1];
            j.Axial(p[JPay.Spring] + p[JPay.Motor] + p[JPay.Lower] - p[JPay.Upper], ax.Ax, ax.Ay, ax.A1, ax.A2);
            j.Axial(perp, ax.Nx, ax.Ny, ax.S1, ax.S2);
            j.Ang(angImp);
            return;
        }

        AxialRows(ref j, ref d, s, p, in ax, useBias, invH, d.Spring && d.Hertz > 0f, d.Limit);

        // Perpendicular row and angle row together as a 2x2 block.
        float cPerp = ax.Nx * (j.Dx + j.RBx - j.RAx) + ax.Ny * (j.Dy + j.RBy - j.RAy);
        float cAng = SmallAngle(in j, s);
        float dPerp = j.AxialVelocity(ax.Nx, ax.Ny, ax.S1, ax.S2), dAng = j.WB - j.WA;
        float k11 = j.MA + j.MB + j.IA * ax.S1 * ax.S1 + j.IB * ax.S2 * ax.S2;
        float k12 = j.IA * ax.S1 + j.IB * ax.S2;
        float k22 = j.IA + j.IB;
        if (k22 == 0f) { k22 = 1f; }
        float bias = 0f, ms = 1f, isc = 0f;
        if (useBias) { bias = s[JRow.JBias]; ms = s[JRow.JMass]; isc = s[JRow.JImp]; }
        Solve22(k11, k12, k22, dPerp + bias * cPerp, dAng + bias * cAng, out float bx, out float by);
        float i1 = -ms * bx - isc * p[JPay.P0], i2 = -ms * by - isc * p[JPay.P1];
        p[JPay.P0] += i1; p[JPay.P1] += i2;
        j.Axial(i1, ax.Nx, ax.Ny, ax.S1, ax.S2);
        j.Ang(i2);
    }

    private static void SolveWheel(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias, float invH)
    {
        var ax = ComputeAxes(in j, s);
        if (warm)
        {
            j.Axial(p[JPay.Spring] + p[JPay.Lower] - p[JPay.Upper], ax.Ax, ax.Ay, ax.A1, ax.A2);
            j.Axial(p[JPay.P0], ax.Nx, ax.Ny, ax.S1, ax.S2);
            j.Ang(p[JPay.Motor]);
            return;
        }

        if (d.Motor)
        {
            float cdot = j.WB - j.WA - d.MotorSpeed;
            float imp = -s[JRow.AngMass] * cdot;
            float nl = Clamp(p[JPay.Motor] + imp, s[JRow.MaxMotorImp]);
            imp = nl - p[JPay.Motor];
            p[JPay.Motor] = nl;
            j.Ang(imp);
        }

        AxialRows(ref j, ref d, s, p, in ax, useBias, invH, d.Spring && d.Hertz > 0f, d.Limit);

        // The line constraint: B's anchor stays on the axis through A's anchor.
        float cPerp = ax.Nx * (j.Dx + j.RBx - j.RAx) + ax.Ny * (j.Dy + j.RBy - j.RAy);
        float k = j.MA + j.MB + j.IA * ax.S1 * ax.S1 + j.IB * ax.S2 * ax.S2;
        float mass = k > 0f ? 1f / k : 0f;
        float bias = 0f, ms = 1f, isc = 0f;
        if (useBias) { bias = s[JRow.JBias]; ms = s[JRow.JMass]; isc = s[JRow.JImp]; }
        float cdotP = j.AxialVelocity(ax.Nx, ax.Ny, ax.S1, ax.S2);
        float impP = -mass * ms * (cdotP + bias * cPerp) - isc * p[JPay.P0];
        p[JPay.P0] += impP;
        j.Axial(impP, ax.Nx, ax.Ny, ax.S1, ax.S2);
    }

    // ------------------------------------------------------------------ weld

    private static void SolveWeld(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias)
    {
        if (warm)
        {
            j.Lin(p[JPay.P0], p[JPay.P1]);
            j.Ang(p[JPay.P2]);
            return;
        }

        // Angle first, then position. A rigid part follows the soft-step bias rule; a part with a spring frequency is always a spring.
        float cAng = SmallAngle(in j, s);
        float k, ms, isc;
        if (d.Hertz2 == 0f) { ms = useBias ? s[JRow.JMass] : 1f; k = useBias ? ms * s[JRow.JBias] : 0f; isc = useBias ? s[JRow.JImp] : 0f; }
        else { ms = s[JRow.AMass]; k = s[JRow.ABias]; isc = s[JRow.AImp]; }
        if (d.Hertz2 == 0f || useBias)
        {
            float imp = -s[JRow.AngMass] * (ms * (j.WB - j.WA) + k * cAng) - isc * p[JPay.P2];
            p[JPay.P2] += imp;
            j.Ang(imp);
        }

        float cx = j.Dx + j.RBx - j.RAx, cy = j.Dy + j.RBy - j.RAy;
        if (d.Hertz == 0f) { ms = useBias ? s[JRow.JMass] : 1f; k = useBias ? ms * s[JRow.JBias] : 0f; isc = useBias ? s[JRow.JImp] : 0f; }
        else if (useBias) { ms = s[JRow.SMass]; k = s[JRow.SBias]; isc = s[JRow.SImp]; }
        else { return; }
        float lx = p[JPay.P0], ly = p[JPay.P1];
        PointRow(ref j, cx, cy, k, ms, isc, ref lx, ref ly);
        p[JPay.P0] = lx; p[JPay.P1] = ly;
    }

    // ------------------------------------------------------------------ distance

    private static void SolveDistance(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias, float invH)
    {
        float dx = j.Dx + j.RBx - j.RAx, dy = j.Dy + j.RBy - j.RAy;
        float len = float.Sqrt(dx * dx + dy * dy);
        float ax = 0f, ay = 0f;
        if (len > 0.005f) { float inv = 1f / len; ax = dx * inv; ay = dy * inv; }
        float a1 = j.RAx * ay - j.RAy * ax, a2 = j.RBx * ay - j.RBy * ax;
        if (warm)
        {
            j.Axial(p[JPay.P0] + p[JPay.Spring] + p[JPay.Motor] + p[JPay.Lower] - p[JPay.Upper], ax, ay, a1, a2);
            return;
        }

        float k = j.MA + j.MB + j.IA * a1 * a1 + j.IB * a2 * a2;
        float mass = k > 0f ? 1f / k : 0f;
        bool range = d.Lower < d.Upper;
        if (d.Spring && range)
        {
            if (d.Hertz > 0f && useBias)
            {
                float cdot = j.AxialVelocity(ax, ay, a1, a2);
                float imp = -mass * (s[JRow.SMass] * cdot + s[JRow.SBias] * (len - d.Length)) - s[JRow.SImp] * p[JPay.Spring];
                p[JPay.Spring] += imp;
                j.Axial(imp, ax, ay, a1, a2);
            }

            if (d.Motor)
            {
                float cdot = j.AxialVelocity(ax, ay, a1, a2) - d.MotorSpeed;
                float imp = -mass * cdot;
                float nl = Clamp(p[JPay.Motor] + imp, s[JRow.MaxMotorImp]);
                imp = nl - p[JPay.Motor];
                p[JPay.Motor] = nl;
                j.Axial(imp, ax, ay, a1, a2);
            }
        }
        else
        {
            float cdot = j.AxialVelocity(ax, ay, a1, a2);
            float bias = useBias ? s[JRow.JBias] * (len - d.Length) : 0f;
            float ms = useBias ? s[JRow.JMass] : 1f, isc = useBias ? s[JRow.JImp] : 0f;
            float imp = -mass * ms * (cdot + bias) - isc * p[JPay.P0];
            p[JPay.P0] += imp;
            j.Axial(imp, ax, ay, a1, a2);
        }

        if (d.Limit && range)
        {
            float lo = p[JPay.Lower], up = p[JPay.Upper];
            float cdot = j.AxialVelocity(ax, ay, a1, a2);
            float imp = LimitImpulse(useBias, len - d.Lower, cdot, mass, ref lo, s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
            p[JPay.Lower] = lo; j.Axial(imp, ax, ay, a1, a2);
            cdot = j.AxialVelocity(ax, ay, a1, a2);
            imp = LimitImpulse(useBias, d.Upper - len, -cdot, mass, ref up, s[JRow.JBias], s[JRow.JMass], s[JRow.JImp], invH);
            p[JPay.Upper] = up; j.Axial(-imp, ax, ay, a1, a2);
        }
    }

    // ------------------------------------------------------------------ motor (relative pose)

    private static void SolveMotor(ref JS j, ref JointData2D d, ReadOnlySpan<float> s, Span<float> p, bool warm, float invH)
    {
        if (warm)
        {
            j.Lin(p[JPay.P0], p[JPay.P1]);
            j.Ang(p[JPay.P2]);
            return;
        }

        // Catto, "Modeling and Solving Constraints", GDC 2009: a velocity target proportional to the pose error, with the impulse capped.
        float corr = d.Correction * invH;
        float angErr = SmallAngle(in j, s); // the reference rotation of a motor joint is its angular offset
        float cdot = (j.WB - j.WA) + corr * angErr;
        float imp = -s[JRow.AngMass] * cdot;
        float nl = Clamp(p[JPay.P2] + imp, s[JRow.MaxMotorImp2]);
        imp = nl - p[JPay.P2];
        p[JPay.P2] = nl;
        j.Ang(imp);

        float ox = j.DAc * s[JRow.TgtX] - j.DAs * s[JRow.TgtY], oy = j.DAs * s[JRow.TgtX] + j.DAc * s[JRow.TgtY];
        float ex = j.Dx + j.RBx - j.RAx - ox, ey = j.Dy + j.RBy - j.RAy - oy;
        j.RelativePointVelocity(out float dvx, out float dvy);
        float k11 = j.MA + j.MB + j.IA * j.RAy * j.RAy + j.IB * j.RBy * j.RBy;
        float k12 = -j.IA * j.RAx * j.RAy - j.IB * j.RBx * j.RBy;
        float k22 = j.MA + j.MB + j.IA * j.RAx * j.RAx + j.IB * j.RBx * j.RBx;
        Solve22(k11, k12, k22, dvx + corr * ex, dvy + corr * ey, out float bx, out float by);
        float nx = p[JPay.P0] - bx, ny = p[JPay.P1] - by;
        float max = s[JRow.MaxMotorImp];
        float m2 = nx * nx + ny * ny;
        if (m2 > max * max) { float sc = max / float.Sqrt(m2); nx *= sc; ny *= sc; }
        float ix = nx - p[JPay.P0], iy = ny - p[JPay.P1];
        p[JPay.P0] = nx; p[JPay.P1] = ny;
        j.Lin(ix, iy);
    }

    // ------------------------------------------------------------------ mouse

    private void SolveMouse(ref JS j, ReadOnlySpan<float> s, Span<float> p, bool warm, bool useBias, int b)
    {
        // Only B moves: A is in the constraint graph to give the joint a place, nothing more.
        j.MA = 0f; j.IA = 0f;
        if (warm)
        {
            j.Lin(p[JPay.P0], p[JPay.P1]);
            return;
        }

        if (!useBias) { return; } // a spring acts once per substep, in the biased pass
        float cx = Dp[2 * b] + j.RBx - s[JRow.TgtX], cy = Dp[2 * b + 1] + j.RBy - s[JRow.TgtY];
        j.RelativePointVelocity(out float dvx, out float dvy);
        float k11 = j.MB + j.IB * j.RBy * j.RBy;
        float k12 = -j.IB * j.RBx * j.RBy;
        float k22 = j.MB + j.IB * j.RBx * j.RBx;
        Solve22(k11, k12, k22, s[JRow.SMass] * dvx + s[JRow.SBias] * cx, s[JRow.SMass] * dvy + s[JRow.SBias] * cy, out float bx, out float by);
        float nx = p[JPay.P0] - bx - s[JRow.SImp] * p[JPay.P0];
        float ny = p[JPay.P1] - by - s[JRow.SImp] * p[JPay.P1];
        float max = s[JRow.MaxMotorImp];
        float m2 = nx * nx + ny * ny;
        if (m2 > max * max) { float sc = max / float.Sqrt(m2); nx *= sc; ny *= sc; }
        float ix = nx - p[JPay.P0], iy = ny - p[JPay.P1];
        p[JPay.P0] = nx; p[JPay.P1] = ny;
        j.Lin(ix, iy);
    }
}
