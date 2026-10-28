using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Slots of a joint's persistent payload (accumulated impulses). Which slots a type uses is listed in docs/world3d.md.</summary>
internal static class JP
{
    public const int Spring = 10, Motor = 11, Lower = 12, Upper = 13, Swing = 14;
    public const int SixAxis = 6;   // six-dof: per axis four slots (motor, spring, upper, lower); the locked axes' block impulses are compact in slots 0..2 (translation) and 3..5 (rotation)
}

[InlineArray(3)]
internal struct JRowBuf
{
    private JointRow _e;
}

[InlineArray(3)]
internal struct JF3
{
    private float _e;
}

/// <summary>
/// One scalar constraint row in velocity space: <c>Cdot = LA·vA + AA·ωA + LB·vB + AB·ωB</c>. An impulse λ along it changes the velocities by <c>mA·λ·LA</c>, <c>IA·λ·AA</c> and so on.
/// Linear and angular rows of every joint are built from this one shape, so blocks of up to three rows (a point constraint, a locked rotation) share one solver.
/// </summary>
internal struct JointRow
{
    public Vector3F LA, AA, LB, AB;

    public readonly JointRow Neg() => new() { LA = -LA, AA = -AA, LB = -LB, AB = -AB };

    /// <summary>A row along the unit direction u at the arms armA (on A) and armB (on B): the relative velocity of the two points along u.</summary>
    public static JointRow Lin(Vector3F u, Vector3F armA, Vector3F armB) => new() { LA = -u, AA = -Vector3F.Cross(armA, u), LB = u, AB = Vector3F.Cross(armB, u) };

    /// <summary>A row along an axis of relative angular velocity.</summary>
    public static JointRow Ang(Vector3F u) => new() { AA = -u, AB = u };
}

internal sealed partial class Engine3D
{
    /// <summary>Velocities and masses of one joint's two bodies, loaded once per solve call and written back at the end.</summary>
    private struct JS3
    {
        public Vector3F VA, WA, VB, WB;
        public float MA, MB;
        public Sym IA, IB;

        public readonly float Cdot(in JointRow r) => Vector3F.Dot(r.LA, VA) + Vector3F.Dot(r.AA, WA) + Vector3F.Dot(r.LB, VB) + Vector3F.Dot(r.AB, WB);

        public void Apply(in JointRow r, float l)
        {
            VA += r.LA * (MA * l); WA += IA.Mul(r.AA) * l;
            VB += r.LB * (MB * l); WB += IB.Mul(r.AB) * l;
        }

        public readonly float K(in JointRow r, in JointRow s) =>
            MA * Vector3F.Dot(r.LA, s.LA) + Vector3F.Dot(r.AA, IA.Mul(s.AA)) + MB * Vector3F.Dot(r.LB, s.LB) + Vector3F.Dot(r.AB, IB.Mul(s.AB));

        public readonly float Mass(in JointRow r)
        {
            float k = K(r, r);
            return k > 0f ? 1f / k : 0f;
        }
    }

    /// <summary>A symmetric 3×3 matrix stored as xx xy xz yy yz zz.</summary>
    private struct Sym
    {
        public float XX, XY, XZ, YY, YZ, ZZ;

        public readonly Vector3F Mul(Vector3F v) => new(XX * v.X + XY * v.Y + XZ * v.Z, XY * v.X + YY * v.Y + YZ * v.Z, XZ * v.X + YZ * v.Y + ZZ * v.Z);
    }

    /// <summary>Soft coefficients of a row: impulse = -K⁻¹ (Ms·Cdot + Kk·C) - Isc·accumulated.</summary>
    private readonly struct Soft
    {
        public readonly float Ms, Kk, Isc;

        public Soft(float ms, float kk, float isc) { Ms = ms; Kk = kk; Isc = isc; }
    }

    /// <summary>The current geometry of a joint: anchors and frames with the rotation accumulated this step applied.</summary>
    private struct JGeo
    {
        public Vector3F RA, RB;       // anchors from the centres of mass
        public Vector3F D;            // centre of mass of B minus that of A
        public Vector3F DpA, DpB;     // displacement of each centre of mass this step
        public Vector3F E;            // from A's centre of mass to B's anchor
        public Vector3F C;            // B's anchor minus A's anchor
        public QuaternionF FA, FB;    // frames in world axes
        public QuaternionF DqA, DqB;
        public Vector3F AxA, AyA, AzA; // frame A's axes
    }

    private const float JPi = 3.1415927f;

    private void LoadJoint(int a, int b, ReadOnlySpan<float> s, out JS3 j, out JGeo g)
    {
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity, invMass = bodies.InverseMass;
        j = default;
        j.VA = V(lin, 3 * a); j.WA = V(ang, 3 * a); j.VB = V(lin, 3 * b); j.WB = V(ang, 3 * b);
        j.MA = invMass[a]; j.MB = invMass[b];
        var wa = WInvI.AsSpan(6 * a, 6); var wb = WInvI.AsSpan(6 * b, 6);
        j.IA = new Sym { XX = wa[0], XY = wa[1], XZ = wa[2], YY = wa[3], YZ = wa[4], ZZ = wa[5] };
        j.IB = new Sym { XX = wb[0], XY = wb[1], XZ = wb[2], YY = wb[3], YZ = wb[4], ZZ = wb[5] };
        g = default;
        g.DqA = new QuaternionF(Dq[4 * a], Dq[4 * a + 1], Dq[4 * a + 2], Dq[4 * a + 3]);
        g.DqB = new QuaternionF(Dq[4 * b], Dq[4 * b + 1], Dq[4 * b + 2], Dq[4 * b + 3]);
        g.RA = g.DqA.Rotate(V(s, JRow3.RA)); g.RB = g.DqB.Rotate(V(s, JRow3.RB));
        g.DpA = V(Dp, 3 * a); g.DpB = V(Dp, 3 * b);
        g.D = V(s, JRow3.D) + (V(Dp, 3 * b) - V(Dp, 3 * a));
        g.E = g.D + g.RB;
        g.C = g.E - g.RA;
        g.FA = g.DqA * new QuaternionF(s[JRow3.FA], s[JRow3.FA + 1], s[JRow3.FA + 2], s[JRow3.FA + 3]);
        g.FB = g.DqB * new QuaternionF(s[JRow3.FB], s[JRow3.FB + 1], s[JRow3.FB + 2], s[JRow3.FB + 3]);
        g.AxA = g.FA.Rotate(new Vector3F(1f, 0f, 0f)); g.AyA = g.FA.Rotate(new Vector3F(0f, 1f, 0f)); g.AzA = g.FA.Rotate(new Vector3F(0f, 0f, 1f));
    }

    private void StoreJoint(int a, int b, in JS3 j)
    {
        var bodies = Core.Bodies;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        if (j.MA > 0f) { Put(lin, 3 * a, j.VA); Put(ang, 3 * a, j.WA); }
        if (j.MB > 0f) { Put(lin, 3 * b, j.VB); Put(ang, 3 * b, j.WB); }
    }

    /// <summary>B's rotation relative to A in A's frame, with its sign chosen so the scalar part is not negative: the rotation vector is then about twice its vector part for small angles.</summary>
    private static QuaternionF Relative(in JGeo g)
    {
        var q = g.FA.Conjugate() * g.FB;
        return q.W < 0f ? new QuaternionF(-q.X, -q.Y, -q.Z, -q.W) : q;
    }

    /// <summary>Twist about x and swing (y, z) of a relative rotation: q = swing · twist with the twist about the x axis. Returns the twist angle in -pi..pi (onemath's correctly rounded atan2) and the swing quaternion.</summary>
    private static float SwingTwist(QuaternionF q, out QuaternionF swing)
    {
        float n = float.Sqrt(q.X * q.X + q.W * q.W);
        QuaternionF twist = n > 1e-12f ? new QuaternionF(q.X / n, 0f, 0f, q.W / n) : QuaternionF.Identity;
        swing = q * twist.Conjugate();
        return (float)(2.0 * Math64.Atan2(twist.X, twist.W));
    }

    // ------------------------------------------------------------------ row primitives

    private static float Clamp(float v, float lim) => v > lim ? lim : (v < -lim ? -lim : v);

    /// <summary>
    /// A block of up to three rows solved together (the exact solution of the small system, so coupled rows such as a point constraint do not fight): impulse = -K⁻¹ (Ms·Cdot + Kk·C) - Isc·accumulated.
    /// Accumulated impulses live in p[slot..slot+n). When <paramref name="warm"/> the stored impulses are applied instead.
    /// </summary>
    private static void Block(ref JS3 j, Span<float> p, int slot, int n, in JRowBuf rows, in JF3 c, Soft k, bool warm)
    {
        ReadOnlySpan<JointRow> r = rows;
        ReadOnlySpan<float> cv = c;
        if (warm)
        {
            for (int i = 0; i < n; i++) { j.Apply(r[i], p[slot + i]); }
            return;
        }

        JF3 rhs = default;
        for (int i = 0; i < n; i++) { rhs[i] = k.Ms * j.Cdot(r[i]) + k.Kk * cv[i]; }
        float x0, x1 = 0f, x2 = 0f;
        float k00 = j.K(r[0], r[0]);
        if (n == 1)
        {
            x0 = k00 > 0f ? rhs[0] / k00 : 0f;
        }
        else if (n == 2)
        {
            float k01 = j.K(r[0], r[1]), k11 = j.K(r[1], r[1]);
            float det = k00 * k11 - k01 * k01;
            if (det != 0f) { det = 1f / det; }
            x0 = det * (k11 * rhs[0] - k01 * rhs[1]);
            x1 = det * (k00 * rhs[1] - k01 * rhs[0]);
        }
        else
        {
            float k01 = j.K(r[0], r[1]), k02 = j.K(r[0], r[2]), k11 = j.K(r[1], r[1]), k12 = j.K(r[1], r[2]), k22 = j.K(r[2], r[2]);
            float c00 = k11 * k22 - k12 * k12, c01 = k02 * k12 - k01 * k22, c02 = k01 * k12 - k02 * k11;
            float det = k00 * c00 + k01 * c01 + k02 * c02;
            if (det != 0f) { det = 1f / det; }
            float c11 = k00 * k22 - k02 * k02, c12 = k01 * k02 - k00 * k12, c22 = k00 * k11 - k01 * k01;
            x0 = det * (c00 * rhs[0] + c01 * rhs[1] + c02 * rhs[2]);
            x1 = det * (c01 * rhs[0] + c11 * rhs[1] + c12 * rhs[2]);
            x2 = det * (c02 * rhs[0] + c12 * rhs[1] + c22 * rhs[2]);
        }

        JF3 imp = default;
        imp[0] = -x0 - k.Isc * p[slot];
        if (n > 1) { imp[1] = -x1 - k.Isc * p[slot + 1]; }
        if (n > 2) { imp[2] = -x2 - k.Isc * p[slot + 2]; }
        for (int i = 0; i < n; i++)
        {
            p[slot + i] += imp[i];
            j.Apply(r[i], imp[i]);
        }
    }

    /// <summary>A one-sided row (a limit): speculative when the limit is not reached (C &gt; 0), soft when violated; the accumulated impulse stays at least zero.</summary>
    private static void LimitRow(ref JS3 j, Span<float> p, int slot, in JointRow r, float c, bool useBias, float jb, float jm, float ji, float invH, bool warm)
    {
        if (warm) { j.Apply(r, p[slot]); return; }
        float mass = j.Mass(r);
        float bias = 0f, ms = 1f, isc = 0f;
        if (c > 0f) { bias = c * invH; }
        else if (useBias) { bias = jb * c; ms = jm; isc = ji; }
        float lambda = p[slot];
        float imp = -mass * ms * (j.Cdot(r) + bias) - isc * lambda;
        float nl = lambda + imp;
        if (nl < 0f) { nl = 0f; }
        imp = nl - lambda;
        p[slot] = nl;
        j.Apply(r, imp);
    }

    /// <summary>A motor row: drives Cdot to <paramref name="speed"/>, accumulated impulse capped at ±<paramref name="maxImpulse"/>.</summary>
    private static void MotorRow(ref JS3 j, Span<float> p, int slot, in JointRow r, float speed, float maxImpulse, bool warm)
    {
        if (warm) { j.Apply(r, p[slot]); return; }
        float imp = -j.Mass(r) * (j.Cdot(r) - speed);
        float nl = Clamp(p[slot] + imp, maxImpulse);
        imp = nl - p[slot];
        p[slot] = nl;
        j.Apply(r, imp);
    }

    /// <summary>A user spring row on C with the coefficients of <c>MakeSpring</c>; acts once per substep, in the biased pass.</summary>
    private static void SpringRow(ref JS3 j, Span<float> p, int slot, in JointRow r, float c, float stiffness, float massScale, float impulseScale, bool useBias, bool warm)
    {
        if (warm) { j.Apply(r, p[slot]); return; }
        if (!useBias) { return; }
        float imp = -j.Mass(r) * (massScale * j.Cdot(r) + stiffness * c) - impulseScale * p[slot];
        p[slot] += imp;
        j.Apply(r, imp);
    }

    // ------------------------------------------------------------------ dispatch

    public void SolveJoint(int c, SolvePhase phase, in StepContext ctx)
    {
        if (phase == SolvePhase.Restitution) { return; }
        var cs = Core.Constraints;
        int a = cs.BodyA[c], b = cs.BodyB[c];
        ref JointData3D d = ref Joints[c];
        var s = JScratch.AsSpan(c * JRow3.Stride, JRow3.Stride);
        var p = cs.Payload(c);
        LoadJoint(a, b, s, out JS3 j, out JGeo g);
        bool warm = phase == SolvePhase.WarmStart, useBias = phase == SolvePhase.Solve;
        float invH = ctx.InverseSubstepDt;
        var rigid = useBias ? new Soft(s[JRow3.JMass], s[JRow3.JMass] * s[JRow3.JBias], s[JRow3.JImp]) : new Soft(1f, 0f, 0f);
        switch ((JointType3D)d.Type)
        {
            case JointType3D.Fixed: SolveFixed(ref j, in g, p, rigid, warm); break;
            case JointType3D.Point: PointBlock(ref j, in g, p, rigid, warm); break;
            case JointType3D.Distance: SolveDistance(ref j, in g, ref d, s, p, useBias, invH, warm); break;
            case JointType3D.Hinge: SolveHinge(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
            case JointType3D.Slider: SolveSlider(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
            case JointType3D.Cone: SolveCone(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
            case JointType3D.SwingTwist: SolveSwingTwist(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
            case JointType3D.SixDof: SolveSixDof(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
            case JointType3D.Path: SolvePath(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
            case JointType3D.Gear: SolveGear(ref j, in g, ref d, p, warm); break;
            case JointType3D.RackAndPinion: SolveRack(ref j, in g, ref d, p, warm); break;
            case JointType3D.Pulley: SolvePulley(ref j, in g, ref d, s, p, rigid, useBias, invH, warm); break;
        }

        StoreJoint(a, b, in j);
    }

    private static readonly Vector3F UX = new(1f, 0f, 0f), UY = new(0f, 1f, 0f), UZ = new(0f, 0f, 1f);

    /// <summary>The three-row point constraint: the two anchors coincide. Slots 0..2.</summary>
    private static void PointBlock(ref JS3 j, in JGeo g, Span<float> p, Soft rigid, bool warm)
    {
        JRowBuf rows = default; JF3 c = default;
        rows[0] = JointRow.Lin(UX, g.RA, g.RB); rows[1] = JointRow.Lin(UY, g.RA, g.RB); rows[2] = JointRow.Lin(UZ, g.RA, g.RB);
        c[0] = g.C.X; c[1] = g.C.Y; c[2] = g.C.Z;
        Block(ref j, p, 0, 3, rows, c, rigid, warm);
    }

    /// <summary>Locks the relative rotation of the two frames with a block along frame A's axes. The error of axis i is twice the vector part of the relative rotation.</summary>
    private static void LockRotation(ref JS3 j, in JGeo g, Span<float> p, int slot, Soft rigid, bool warm)
    {
        var q = Relative(in g);
        JRowBuf rows = default; JF3 c = default;
        rows[0] = JointRow.Ang(g.AxA); rows[1] = JointRow.Ang(g.AyA); rows[2] = JointRow.Ang(g.AzA);
        c[0] = 2f * q.X; c[1] = 2f * q.Y; c[2] = 2f * q.Z;
        Block(ref j, p, slot, 3, rows, c, rigid, warm);
    }

    // ------------------------------------------------------------------ fixed, point

    private static void SolveFixed(ref JS3 j, in JGeo g, Span<float> p, Soft rigid, bool warm)
    {
        LockRotation(ref j, in g, p, 3, rigid, warm);
        PointBlock(ref j, in g, p, rigid, warm);
    }

    // ------------------------------------------------------------------ distance

    private static void SolveDistance(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, bool useBias, float invH, bool warm)
    {
        float len = g.C.Length();
        var axis = len > 0.005f ? g.C * (1f / len) : default;
        var row = JointRow.Lin(axis, g.RA, g.RB);
        float jb = s[JRow3.JBias], jm = s[JRow3.JMass], ji = s[JRow3.JImp];
        bool range = d.Lower < d.Upper;
        if (d.Spring && range)
        {
            SpringRow(ref j, p, JP.Spring, row, len - d.Length, s[JRow3.SBias], s[JRow3.SMass], s[JRow3.SImp], useBias, warm);
        }
        else if (warm) { j.Apply(row, p[0]); }
        else
        {
            float bias = useBias ? jb * (len - d.Length) : 0f;
            float ms = useBias ? jm : 1f, isc = useBias ? ji : 0f;
            float imp = -j.Mass(row) * ms * (j.Cdot(row) + bias) - isc * p[0];
            p[0] += imp;
            j.Apply(row, imp);
        }

        if (d.Limit && range)
        {
            LimitRow(ref j, p, JP.Lower, row, len - d.Lower, useBias, jb, jm, ji, invH, warm);
            LimitRow(ref j, p, JP.Upper, row.Neg(), d.Upper - len, useBias, jb, jm, ji, invH, warm);
        }
    }

    // ------------------------------------------------------------------ hinge

    private static void SolveHinge(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        var aB = g.FB.Rotate(UX);
        var axis = g.AxA;
        var axisRow = JointRow.Ang(axis);
        bool needAngle = (d.Spring && d.Hertz > 0f) || d.Limit;
        float theta = needAngle && !warm ? SwingTwist(Relative(in g), out _) : 0f;
        float jb = s[JRow3.JBias], jm = s[JRow3.JMass], ji = s[JRow3.JImp];
        if (d.Spring && d.Hertz > 0f) { SpringRow(ref j, p, JP.Spring, axisRow, theta, s[JRow3.SBias], s[JRow3.SMass], s[JRow3.SImp], useBias, warm); }
        else if (warm) { j.Apply(axisRow, p[JP.Spring]); }
        if (d.Motor) { MotorRow(ref j, p, JP.Motor, axisRow, d.MotorSpeed, s[JRow3.MaxMotorImp], warm); }
        if (d.Limit)
        {
            LimitRow(ref j, p, JP.Lower, axisRow, theta - d.Lower, useBias, jb, jm, ji, invH, warm);
            LimitRow(ref j, p, JP.Upper, axisRow.Neg(), d.Upper - theta, useBias, jb, jm, ji, invH, warm);
        }

        // The two rows that keep the axes together: the part of the rotation that is not about the axis.
        var e = Vector3F.Cross(axis, aB);
        JRowBuf rows = default; JF3 c = default;
        rows[0] = JointRow.Ang(g.AyA); rows[1] = JointRow.Ang(g.AzA);
        c[0] = Vector3F.Dot(e, g.AyA); c[1] = Vector3F.Dot(e, g.AzA);
        Block(ref j, p, 3, 2, rows, c, rigid, warm);
        PointBlock(ref j, in g, p, rigid, warm);
    }

    // ------------------------------------------------------------------ slider

    private static void SolveSlider(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        var axisRow = JointRow.Lin(g.AxA, g.E, g.RB);
        float t = Vector3F.Dot(g.AxA, g.C);
        float jb = s[JRow3.JBias], jm = s[JRow3.JMass], ji = s[JRow3.JImp];
        if (d.Spring && d.Hertz > 0f) { SpringRow(ref j, p, JP.Spring, axisRow, t, s[JRow3.SBias], s[JRow3.SMass], s[JRow3.SImp], useBias, warm); }
        else if (warm) { j.Apply(axisRow, p[JP.Spring]); }
        if (d.Motor) { MotorRow(ref j, p, JP.Motor, axisRow, d.MotorSpeed, s[JRow3.MaxMotorImp], warm); }
        if (d.Limit)
        {
            LimitRow(ref j, p, JP.Lower, axisRow, t - d.Lower, useBias, jb, jm, ji, invH, warm);
            LimitRow(ref j, p, JP.Upper, axisRow.Neg(), d.Upper - t, useBias, jb, jm, ji, invH, warm);
        }

        LockRotation(ref j, in g, p, 3, rigid, warm);
        JRowBuf rows = default; JF3 c = default;
        rows[0] = JointRow.Lin(g.AyA, g.E, g.RB); rows[1] = JointRow.Lin(g.AzA, g.E, g.RB);
        c[0] = Vector3F.Dot(g.AyA, g.C); c[1] = Vector3F.Dot(g.AzA, g.C);
        Block(ref j, p, 0, 2, rows, c, rigid, warm);
    }

    // ------------------------------------------------------------------ cone, swing-twist

    private static void SolveCone(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        var aA = g.AxA;
        var aB = g.FB.Rotate(UX);
        var cross = Vector3F.Cross(aA, aB);
        float sin = cross.Length(), cos = Vector3F.Dot(aA, aB);
        Vector3F n;
        float angle;
        if (sin > 1e-6f) { n = cross * (1f / sin); angle = (float)Math64.Atan2(sin, cos); }
        else if (cos > 0f) { n = g.AyA; angle = 0f; }
        else { n = g.AyA; angle = JPi; }
        // Rotating B about n by a positive angle opens the cone, so the row for "half angle minus angle >= 0" is the negated angular row.
        LimitRow(ref j, p, JP.Lower, JointRow.Ang(n).Neg(), d.Swing1 - angle, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm);
        PointBlock(ref j, in g, p, rigid, warm);
    }

    private static void SolveSwingTwist(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        float jb = s[JRow3.JBias], jm = s[JRow3.JMass], ji = s[JRow3.JImp];
        float twist = 0f;
        QuaternionF swing = QuaternionF.Identity;
        if (!warm) { twist = SwingTwist(Relative(in g), out swing); }
        var twistRow = JointRow.Ang(g.AxA);
        LimitRow(ref j, p, JP.Lower, twistRow, twist - d.Lower, useBias, jb, jm, ji, invH, warm);
        LimitRow(ref j, p, JP.Upper, twistRow.Neg(), d.Upper - twist, useBias, jb, jm, ji, invH, warm);

        // Swing: the quaternion's (y, z) components bound an ellipse (y / sin(hy/2))² + (z / sin(hz/2))² <= 1 (Rouwé's talks on ragdoll constraints, swing-twist decomposition of Dobrowolski).
        // Outside it the row pushes B back about the ellipse's normal at the swing's direction.
        if (warm) { LimitRowWarm(ref j, in g, p); }
        else
        {
            float ly = s[JRow3.SwingLimit], lz = s[JRow3.SwingLimit + 1];
            float sy = swing.Y, sz = swing.Z;
            float rho = float.Sqrt((sy / ly) * (sy / ly) + (sz / lz) * (sz / lz));
            JointRow row = default;
            float cval = 1f;
            if (rho > 1e-6f)
            {
                float mag = 2f * float.Sqrt(sy * sy + sz * sz);
                cval = mag * (1f - rho) / rho;
                float gy = sy / (ly * ly), gz = sz / (lz * lz);
                float gl = float.Sqrt(gy * gy + gz * gz);
                var axisA = g.AyA * (gy / gl) + g.AzA * (gz / gl);
                row = JointRow.Ang(axisA).Neg();
                LimitRow(ref j, p, JP.Swing, row, cval, useBias, jb, jm, ji, invH, false);
            }
            else { p[JP.Swing] = 0f; }
        }

        PointBlock(ref j, in g, p, rigid, warm);
    }

    /// <summary>Warm start of the swing row: the stored impulse acts about the axis of the swing at the start of the solve call (re-derived from the current pose).</summary>
    private static void LimitRowWarm(ref JS3 j, in JGeo g, Span<float> p)
    {
        float lam = p[JP.Swing];
        if (lam == 0f) { return; }
        SwingTwist(Relative(in g), out var swing);
        float sy = swing.Y, sz = swing.Z;
        float l = float.Sqrt(sy * sy + sz * sz);
        if (!(l > 1e-9f)) { return; }
        var axisA = g.AyA * (sy / l) + g.AzA * (sz / l);
        j.Apply(JointRow.Ang(axisA).Neg(), lam);
    }

    // ------------------------------------------------------------------ six degrees of freedom

    private static void SolveSixDof(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        float jb = s[JRow3.JBias], jm = s[JRow3.JMass], ji = s[JRow3.JImp];
        // Value and row of every axis. Translation rows act at the anchors (arms E and RB); rotation rows are angular.
        var q = Relative(in g);
        float twist = SwingTwist(q, out var swing);
        float swingY = 2f * swing.Y, swingZ = 2f * swing.Z;
        float v0 = Vector3F.Dot(g.AxA, g.C), v1 = Vector3F.Dot(g.AyA, g.C), v2 = Vector3F.Dot(g.AzA, g.C);
        for (int i = 0; i < 6; i++)
        {
            byte mode = d.Six.Mode[i];
            bool motor = (d.Six.MotorMask & (1 << i)) != 0, spring = (d.Six.SpringMask & (1 << i)) != 0 && d.Six.Hertz[i] > 0f;
            if (mode == 0 && !motor && !spring) { continue; }
            JointRow row = AxisRow(in g, i);
            float v = AxisValue(i, v0, v1, v2, twist, swingY, swingZ);
            int o = JP.SixAxis + 4 * i;
            if (spring) { SpringRow(ref j, p, o + 1, row, v, s[JRow3.SixSpring + 3 * i], s[JRow3.SixSpring + 3 * i + 1], s[JRow3.SixSpring + 3 * i + 2], useBias, warm); }
            else if (warm) { j.Apply(row, p[o + 1]); }
            if (motor) { MotorRow(ref j, p, o, row, d.Six.MotorSpeed[i], s[JRow3.SixMaxMotor + i], warm); }
            if (mode == 1)
            {
                LimitRow(ref j, p, o + 3, row, v - d.Six.Lo[i], useBias, jb, jm, ji, invH, warm);
                LimitRow(ref j, p, o + 2, row.Neg(), d.Six.Hi[i] - v, useBias, jb, jm, ji, invH, warm);
            }
        }

        // Locked axes last, as two blocks (translation, rotation) whose rows are compacted into the first slots.
        JRowBuf rows = default; JF3 c = default;
        int n = 0;
        for (int i = 0; i < 3; i++)
        {
            if (d.Six.Mode[i] != 0) { continue; }
            rows[n] = AxisRow(in g, i); c[n] = AxisValue(i, v0, v1, v2, twist, swingY, swingZ); n++;
        }

        if (n > 0) { Block(ref j, p, 0, n, rows, c, rigid, warm); }
        rows = default; c = default; n = 0;
        for (int i = 3; i < 6; i++)
        {
            if (d.Six.Mode[i] != 0) { continue; }
            rows[n] = AxisRow(in g, i); c[n] = AxisValue(i, v0, v1, v2, twist, swingY, swingZ); n++;
        }

        if (n > 0) { Block(ref j, p, 3, n, rows, c, rigid, warm); }
    }

    private static JointRow AxisRow(in JGeo g, int i) => i switch
    {
        0 => JointRow.Lin(g.AxA, g.E, g.RB),
        1 => JointRow.Lin(g.AyA, g.E, g.RB),
        2 => JointRow.Lin(g.AzA, g.E, g.RB),
        3 => JointRow.Ang(g.AxA),
        4 => JointRow.Ang(g.AyA),
        _ => JointRow.Ang(g.AzA),
    };

    private static float AxisValue(int i, float v0, float v1, float v2, float twist, float swingY, float swingZ) => i switch
    {
        0 => v0, 1 => v1, 2 => v2, 3 => twist, 4 => swingY, _ => swingZ,
    };

    // ------------------------------------------------------------------ path

    private static void SolvePath(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        var t = g.DqA.Rotate(V(s, JRow3.PathT));
        var nrm = g.DqA.Rotate(V(s, JRow3.PathN));
        var bin = g.DqA.Rotate(V(s, JRow3.PathB));
        var off = g.E - g.DqA.Rotate(V(s, JRow3.PathP)); // from the path point to the follower's anchor
        var along = JointRow.Lin(t, g.E, g.RB);
        if (d.Motor) { MotorRow(ref j, p, JP.Motor, along, d.MotorSpeed, s[JRow3.MaxMotorImp], warm); }
        int end = (int)s[JRow3.PathEnd];
        if (end != 0)
        {
            float x = Vector3F.Dot(off, t);
            if (end < 0) { LimitRow(ref j, p, JP.Lower, along, x, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm); }
            else { LimitRow(ref j, p, JP.Lower, along.Neg(), -x, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm); }
        }
        else if (warm) { j.Apply(along, p[JP.Lower]); }

        var fb = g.FB;
        if (d.Aux == 1)
        {
            var fwd = fb.Rotate(UX);
            var e = Vector3F.Cross(t, fwd);
            JRowBuf rr = default; JF3 cc = default;
            rr[0] = JointRow.Ang(nrm); rr[1] = JointRow.Ang(bin);
            cc[0] = Vector3F.Dot(e, nrm); cc[1] = Vector3F.Dot(e, bin);
            Block(ref j, p, 3, 2, rr, cc, rigid, warm);
        }
        else if (d.Aux == 2)
        {
            var pf = PathSpline3D.FromBasis(t, nrm, bin);
            var q = pf.Conjugate() * fb;
            if (q.W < 0f) { q = new QuaternionF(-q.X, -q.Y, -q.Z, -q.W); }
            JRowBuf rr = default; JF3 cc = default;
            rr[0] = JointRow.Ang(t); rr[1] = JointRow.Ang(nrm); rr[2] = JointRow.Ang(bin);
            cc[0] = 2f * q.X; cc[1] = 2f * q.Y; cc[2] = 2f * q.Z;
            Block(ref j, p, 3, 3, rr, cc, rigid, warm);
        }

        JRowBuf rows = default; JF3 c = default;
        rows[0] = JointRow.Lin(nrm, g.E, g.RB); rows[1] = JointRow.Lin(bin, g.E, g.RB);
        c[0] = Vector3F.Dot(off, nrm); c[1] = Vector3F.Dot(off, bin);
        Block(ref j, p, 0, 2, rows, c, rigid, warm);
    }

    // ------------------------------------------------------------------ gear, rack and pinion, pulley

    private static void SolveGear(ref JS3 j, in JGeo g, ref JointData3D d, Span<float> p, bool warm)
    {
        // Velocity constraint ωA·axisA + ratio · ωB·axisB = 0, always rigid. Both axes are the frames' x axes.
        var aA = g.AxA; var aB = g.FB.Rotate(UX);
        var row = new JointRow { AA = aA, AB = aB * d.Ratio };
        JRowBuf rows = default; JF3 c = default;
        rows[0] = row;
        Block(ref j, p, 0, 1, rows, c, new Soft(1f, 0f, 0f), warm);
    }

    private static void SolveRack(ref JS3 j, in JGeo g, ref JointData3D d, Span<float> p, bool warm)
    {
        // ratio · ωA·axisA = vB·axisB: the pinion turns about its axis, the rack moves along its own.
        var aA = g.AxA; var aB = g.FB.Rotate(UX);
        var row = new JointRow { AA = aA * d.Ratio, LB = -aB };
        JRowBuf rows = default; JF3 c = default;
        rows[0] = row;
        Block(ref j, p, 0, 1, rows, c, new Soft(1f, 0f, 0f), warm);
    }

    private static void SolvePulley(ref JS3 j, in JGeo g, ref JointData3D d, ReadOnlySpan<float> s, Span<float> p, Soft rigid, bool useBias, float invH, bool warm)
    {
        // The rope runs from each fixed point to the body's anchor: C = lengthA + ratio * lengthB. Each anchor is its start-of-step offset from the fixed point plus the body's motion this step.
        var uA = V(s, JRow3.PulleyA) + g.DpA + g.RA;
        var uB = V(s, JRow3.PulleyB) + g.DpB + g.RB;
        float la = uA.Length(), lb = uB.Length();
        var nA = la > 1e-4f ? uA * (1f / la) : default;
        var nB = lb > 1e-4f ? uB * (1f / lb) : default;
        var row = new JointRow { LA = nA, AA = Vector3F.Cross(g.RA, nA), LB = nB * d.Ratio, AB = Vector3F.Cross(g.RB, nB) * d.Ratio };
        float len = la + d.Ratio * lb;
        JRowBuf rows = default; JF3 c = default;
        rows[0] = row;
        c[0] = len - d.Lower;
        bool rigidRope = d.Lower == d.Upper;
        if (rigidRope || warm) { Block(ref j, p, 0, 1, rows, c, rigid, warm); }
        if (!rigidRope)
        {
            LimitRow(ref j, p, JP.Lower, row, len - d.Lower, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm);
            LimitRow(ref j, p, JP.Upper, row.Neg(), d.Upper - len, useBias, s[JRow3.JBias], s[JRow3.JMass], s[JRow3.JImp], invH, warm);
        }
    }
}
