using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Layout of the per-constraint scratch rows filled by Prepare and read by the solver.</summary>
internal static class Stride3D
{
    public const int Constraint = 72;

    // Header.
    public const int N = 0, T1 = 3, T2 = 6, Friction = 9, Restitution = 10, Rolling = 11, InvMassA = 12, InvMassB = 13, BiasRate = 14, MassScale = 15, ImpulseScale = 16, RollMass = 17;

    // Points.
    public const int Point0 = 24, PointStride = 12;
    public const int ArmA = 0, ArmB = 3, AdjustedSeparation = 6, NormalMass = 7, Tangent1Mass = 8, Tangent2Mass = 9, RelativeVelocity = 10;
}

internal sealed partial class Engine3D
{
    // ------------------------------------------------------------------ small math

    /// <summary>Applies a symmetric tensor stored as xx xy xz yy yz zz to a vector.</summary>
    private static Vector3F Apply(ReadOnlySpan<float> m, Vector3F v) => new(
        m[0] * v.X + m[1] * v.Y + m[2] * v.Z,
        m[1] * v.X + m[3] * v.Y + m[4] * v.Z,
        m[2] * v.X + m[4] * v.Y + m[5] * v.Z);

    private static Vector3F V(ReadOnlySpan<float> a, int i) => new(a[i], a[i + 1], a[i + 2]);

    private static void Put(Span<float> a, int i, Vector3F v) { a[i] = v.X; a[i + 1] = v.Y; a[i + 2] = v.Z; }

    /// <summary>Refreshes one body's world-axes inverse inertia from the current rotation (for impulses applied between steps).</summary>
    public void RefreshWorldInertia(int b)
    {
        EnsureBodies(b + 1);
        var bodies = Core.Bodies;
        RefreshWorldInertia(b, bodies.InverseMass, bodies.InverseInertia, bodies.Rotation);
    }

    /// <summary>Multiplies a vector by the body's world-axes inverse inertia (as last refreshed).</summary>
    public Vector3F ApplyWorldInertia(int b, Vector3F v) => Apply(WInvI.AsSpan(6 * b, 6), v);

    /// <summary>Inverse inertia in world axes, R · I⁻¹ · Rᵀ, from the body-frame tensor and the body's rotation. Static and kinematic bodies get zeros.</summary>
    private void RefreshWorldInertia(int b, ReadOnlySpan<float> invMass, ReadOnlySpan<float> invI, ReadOnlySpan<float> rot)
    {
        var w = WInvI.AsSpan(6 * b, 6);
        if (!(invMass[b] > 0f)) { w.Clear(); return; }
        float x = rot[4 * b], y = rot[4 * b + 1], z = rot[4 * b + 2], s = rot[4 * b + 3];
        float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, sx = s * x, sy = s * y, sz = s * z;
        float r00 = 1f - 2f * (yy + zz), r01 = 2f * (xy - sz), r02 = 2f * (xz + sy);
        float r10 = 2f * (xy + sz), r11 = 1f - 2f * (xx + zz), r12 = 2f * (yz - sx);
        float r20 = 2f * (xz - sy), r21 = 2f * (yz + sx), r22 = 1f - 2f * (xx + yy);
        float ia = invI[6 * b], ib = invI[6 * b + 1], ic = invI[6 * b + 2], id = invI[6 * b + 3], ie = invI[6 * b + 4], ifz = invI[6 * b + 5];
        float t00 = r00 * ia + r01 * ib + r02 * ic, t01 = r00 * ib + r01 * id + r02 * ie, t02 = r00 * ic + r01 * ie + r02 * ifz;
        float t10 = r10 * ia + r11 * ib + r12 * ic, t11 = r10 * ib + r11 * id + r12 * ie, t12 = r10 * ic + r11 * ie + r12 * ifz;
        float t20 = r20 * ia + r21 * ib + r22 * ic, t21 = r20 * ib + r21 * id + r22 * ie, t22 = r20 * ic + r21 * ie + r22 * ifz;
        w[0] = t00 * r00 + t01 * r01 + t02 * r02;
        w[1] = t00 * r10 + t01 * r11 + t02 * r12;
        w[2] = t00 * r20 + t01 * r21 + t02 * r22;
        w[3] = t10 * r10 + t11 * r11 + t12 * r12;
        w[4] = t10 * r20 + t11 * r21 + t12 * r22;
        w[5] = t20 * r20 + t21 * r21 + t22 * r22;
    }

    /// <summary>Two unit vectors perpendicular to a unit normal and to each other, chosen by a fixed rule so the same normal always gives the same pair.</summary>
    private static void Tangents(Vector3F n, out Vector3F t1, out Vector3F t2)
    {
        // Pick the axis the normal is least aligned with; cross products and square roots only.
        Vector3F t;
        if (Math.Abs(n.X) >= 0.57735f) { t = new Vector3F(n.Y, -n.X, 0f); }
        else { t = new Vector3F(0f, n.Z, -n.Y); }
        float len = float.Sqrt(t.X * t.X + t.Y * t.Y + t.Z * t.Z);
        t1 = len > 0f ? t * (1f / len) : new Vector3F(1f, 0f, 0f);
        t2 = Vector3F.Cross(n, t1);
    }

    // ------------------------------------------------------------------ soft constraint coefficients

    /// <summary>
    /// Soft constraint coefficients (Catto, "Soft Step Solver", GDC 2024; "Solver2D", 2024): a spring of frequency <paramref name="hertz"/> and damping ratio <paramref name="zeta"/>
    /// integrated implicitly over <paramref name="h"/>, expressed as a bias rate and scales for the constraint mass and the accumulated impulse.
    /// </summary>
    private static void MakeSoft(float hertz, float zeta, float h, out float biasRate, out float massScale, out float impulseScale)
    {
        if (hertz == 0f) { biasRate = 0f; massScale = 1f; impulseScale = 0f; return; }
        float omega = 6.2831855f * hertz;
        float a1 = 2f * zeta + h * omega;
        float a2 = h * omega * a1;
        float a3 = 1f / (1f + a2);
        biasRate = omega / a1;
        massScale = a2 * a3;
        impulseScale = a3;
    }

    // ------------------------------------------------------------------ prepare

    /// <summary>Once per step: effective masses, adjusted separations, relative velocities for restitution and soft coefficients, from the manifolds stored in the payloads.</summary>
    public void Prepare(ReadOnlySpan<int> constraints, in StepContext ctx)
    {
        var core = Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        ReadOnlySpan<float> invMass = bodies.InverseMass;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        float hertz = Opt.ContactHertz;
        float cap = 0.25f * ctx.InverseSubstepDt;
        if (hertz > cap) { hertz = cap; }
        MakeSoft(hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out float dynBias, out float dynMass, out float dynImpulse);
        MakeSoft(2f * hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out float stBias, out float stMass, out float stImpulse);

        foreach (int c in constraints)
        {
            var pl = cs.Payload(c);
            int a = bodyA[c], b = bodyB[c];
            float mA = invMass[a], mB = invMass[b];
            ReadOnlySpan<float> iA = WInvI.AsSpan(6 * a, 6), iB = WInvI.AsSpan(6 * b, 6);
            var n = V(pl, ManifoldLayout.Normal);
            Tangents(n, out var t1, out var t2);
            var s = Scratch.AsSpan(c * Stride3D.Constraint, Stride3D.Constraint);
            s.Clear();
            Put(s, Stride3D.N, n); Put(s, Stride3D.T1, t1); Put(s, Stride3D.T2, t2);
            s[Stride3D.Friction] = pl[ManifoldLayout.Friction];
            s[Stride3D.Restitution] = pl[ManifoldLayout.Restitution];
            s[Stride3D.Rolling] = pl[ManifoldLayout.RollingResistance];
            s[Stride3D.InvMassA] = mA; s[Stride3D.InvMassB] = mB;
            if (mA == 0f || mB == 0f) { s[Stride3D.BiasRate] = stBias; s[Stride3D.MassScale] = stMass; s[Stride3D.ImpulseScale] = stImpulse; }
            else { s[Stride3D.BiasRate] = dynBias; s[Stride3D.MassScale] = dynMass; s[Stride3D.ImpulseScale] = dynImpulse; }
            if (s[Stride3D.Rolling] > 0f) { RollingMass(iA, iB, s.Slice(Stride3D.RollMass, 6)); }

            Vector3F vA = V(lin, 3 * a), wA = V(ang, 3 * a), vB = V(lin, 3 * b), wB = V(ang, 3 * b);
            int count = cs.PointCount[c];
            for (int i = 0; i < count; i++)
            {
                var rA = V(pl, ManifoldLayout.AnchorA(i));
                var rB = V(pl, ManifoldLayout.AnchorB(i));
                float sep = pl[ManifoldLayout.Separation(i)];
                int o = Stride3D.Point0 + i * Stride3D.PointStride;
                Put(s, o + Stride3D.ArmA, rA); Put(s, o + Stride3D.ArmB, rB);
                s[o + Stride3D.AdjustedSeparation] = sep - Vector3F.Dot(rB - rA, n);
                s[o + Stride3D.NormalMass] = EffectiveMass(mA, mB, iA, iB, rA, rB, n);
                s[o + Stride3D.Tangent1Mass] = EffectiveMass(mA, mB, iA, iB, rA, rB, t1);
                s[o + Stride3D.Tangent2Mass] = EffectiveMass(mA, mB, iA, iB, rA, rB, t2);
                var dv = (vB + Vector3F.Cross(wB, rB)) - (vA + Vector3F.Cross(wA, rA));
                s[o + Stride3D.RelativeVelocity] = Vector3F.Dot(dv, n);
                pl[ManifoldLayout.MaxNormalImpulse(i)] = 0f;
            }
        }
    }

    private static float EffectiveMass(float mA, float mB, ReadOnlySpan<float> iA, ReadOnlySpan<float> iB, Vector3F rA, Vector3F rB, Vector3F d)
    {
        var rnA = Vector3F.Cross(rA, d);
        var rnB = Vector3F.Cross(rB, d);
        float k = mA + mB + Vector3F.Dot(rnA, Apply(iA, rnA)) + Vector3F.Dot(rnB, Apply(iB, rnB));
        return k > 0f ? 1f / k : 0f;
    }

    /// <summary>Inverse of (IA + IB), the mass seen by a relative angular velocity; zero when the sum is singular.</summary>
    private static void RollingMass(ReadOnlySpan<float> iA, ReadOnlySpan<float> iB, Span<float> dest)
    {
        float a = iA[0] + iB[0], b = iA[1] + iB[1], c = iA[2] + iB[2], d = iA[3] + iB[3], e = iA[4] + iB[4], f = iA[5] + iB[5];
        float c00 = d * f - e * e, c01 = c * e - b * f, c02 = b * e - c * d;
        float det = a * c00 + b * c01 + c * c02;
        if (!(det > 0f) || !float.IsFinite(det)) { dest.Clear(); return; }
        float inv = 1f / det;
        dest[0] = c00 * inv; dest[1] = c01 * inv; dest[2] = c02 * inv;
        dest[3] = (a * f - c * c) * inv; dest[4] = (b * c - a * e) * inv; dest[5] = (a * d - b * b) * inv;
    }

    // ------------------------------------------------------------------ solve

    private static Vector3F Rotate(float x, float y, float z, float w, Vector3F v)
    {
        // q v q⁻¹ for a unit quaternion: v + w·t + u × t with t = 2 u × v.
        float tx = 2f * (y * v.Z - z * v.Y), ty = 2f * (z * v.X - x * v.Z), tz = 2f * (x * v.Y - y * v.X);
        return new Vector3F(v.X + w * tx + (y * tz - z * ty), v.Y + w * ty + (z * tx - x * tz), v.Z + w * tz + (x * ty - y * tx));
    }

    /// <summary>
    /// Solves contacts one at a time (scalar). A colored batch has no body in common between constraints, and the overflow set is solved in list order, so the order inside a batch cannot matter.
    /// Each constraint does the same operations in the same order as any other width of the 2D kernel's scalar twin would.
    /// </summary>
    public void SolveBatch(ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext ctx)
    {
        if (constraints.Length == 0) { return; }
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        ReadOnlySpan<int> bodyA = cs.BodyA, bodyB = cs.BodyB;
        ReadOnlySpan<byte> pointCount = cs.PointCount;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        Span<float> payload = cs.PayloadAll;
        int pf = cs.PayloadFloats;
        float invH = ctx.InverseSubstepDt, slop = Opt.ContactSlop, negPush = -Opt.PushOutVelocity, threshold = Opt.RestitutionThreshold;
        bool useBias = phase == SolvePhase.Solve;

        foreach (int c in constraints)
        {
            int a = bodyA[c], b = bodyB[c];
            var s = Scratch.AsSpan(c * Stride3D.Constraint, Stride3D.Constraint);
            var pl = payload.Slice(c * pf, pf);
            var n = V(s, Stride3D.N); var t1 = V(s, Stride3D.T1); var t2 = V(s, Stride3D.T2);
            float mA = s[Stride3D.InvMassA], mB = s[Stride3D.InvMassB];
            ReadOnlySpan<float> iA = WInvI.AsSpan(6 * a, 6), iB = WInvI.AsSpan(6 * b, 6);
            Vector3F vA = V(lin, 3 * a), wA = V(ang, 3 * a), vB = V(lin, 3 * b), wB = V(ang, 3 * b);
            int count = pointCount[c];

            if (phase == SolvePhase.WarmStart)
            {
                for (int i = 0; i < count; i++)
                {
                    int o = Stride3D.Point0 + i * Stride3D.PointStride;
                    var rA = V(s, o + Stride3D.ArmA); var rB = V(s, o + Stride3D.ArmB);
                    var p = n * pl[ManifoldLayout.NormalImpulse(i)] + t1 * pl[ManifoldLayout.Tangent1Impulse(i)] + t2 * pl[ManifoldLayout.Tangent2Impulse(i)];
                    vA -= p * mA; wA -= Apply(iA, Vector3F.Cross(rA, p));
                    vB += p * mB; wB += Apply(iB, Vector3F.Cross(rB, p));
                }

                var roll = V(pl, ManifoldLayout.RollingImpulse);
                wA -= Apply(iA, roll); wB += Apply(iB, roll);
            }
            else if (phase == SolvePhase.Restitution)
            {
                float e = s[Stride3D.Restitution];
                if (e == 0f) { continue; }
                for (int i = 0; i < count; i++)
                {
                    int o = Stride3D.Point0 + i * Stride3D.PointStride;
                    float relVel = s[o + Stride3D.RelativeVelocity];
                    if (!(relVel <= -threshold) || !(pl[ManifoldLayout.MaxNormalImpulse(i)] > 0f)) { continue; }
                    var rA = V(s, o + Stride3D.ArmA); var rB = V(s, o + Stride3D.ArmB);
                    var dv = (vB + Vector3F.Cross(wB, rB)) - (vA + Vector3F.Cross(wA, rA));
                    float vn = Vector3F.Dot(dv, n);
                    float lambda = -s[o + Stride3D.NormalMass] * (vn + e * relVel);
                    float old = pl[ManifoldLayout.NormalImpulse(i)];
                    float nw = old + lambda; if (!(nw > 0f)) { nw = 0f; }
                    lambda = nw - old;
                    pl[ManifoldLayout.NormalImpulse(i)] = nw;
                    if (nw > pl[ManifoldLayout.MaxNormalImpulse(i)]) { pl[ManifoldLayout.MaxNormalImpulse(i)] = nw; }
                    var p = n * lambda;
                    vA -= p * mA; wA -= Apply(iA, Vector3F.Cross(rA, p));
                    vB += p * mB; wB += Apply(iB, Vector3F.Cross(rB, p));
                }
            }
            else
            {
                float biasRate = s[Stride3D.BiasRate], massScale = s[Stride3D.MassScale], impulseScale = s[Stride3D.ImpulseScale];
                float friction = s[Stride3D.Friction], rolling = s[Stride3D.Rolling];
                Vector3F dpA = V(Dp, 3 * a), dpB = V(Dp, 3 * b);
                int qa = 4 * a, qb = 4 * b;

                // Rolling resistance first, then friction, then non-penetration last: the last constraint solved wins the argument, and keeping bodies apart matters most.
                if (rolling > 0f)
                {
                    float total = 0f;
                    for (int i = 0; i < count; i++) { total += pl[ManifoldLayout.NormalImpulse(i)]; }
                    float maxRoll = rolling * total;
                    var dw = wB - wA;
                    dw -= n * Vector3F.Dot(dw, n);
                    var rm = s.Slice(Stride3D.RollMass, 6);
                    var lam = -Apply(rm, dw);
                    lam -= n * Vector3F.Dot(lam, n);
                    var oldR = V(pl, ManifoldLayout.RollingImpulse);
                    var newR = oldR + lam;
                    float len2 = Vector3F.Dot(newR, newR);
                    if (len2 > maxRoll * maxRoll) { newR = newR * (maxRoll / float.Sqrt(len2)); }
                    lam = newR - oldR;
                    Put(pl, ManifoldLayout.RollingImpulse, newR);
                    wA -= Apply(iA, lam); wB += Apply(iB, lam);
                }

                for (int i = 0; i < count; i++)
                {
                    int o = Stride3D.Point0 + i * Stride3D.PointStride;
                    var rA = V(s, o + Stride3D.ArmA); var rB = V(s, o + Stride3D.ArmB);
                    var dv = (vB + Vector3F.Cross(wB, rB)) - (vA + Vector3F.Cross(wA, rA));
                    float l1 = pl[ManifoldLayout.Tangent1Impulse(i)], l2 = pl[ManifoldLayout.Tangent2Impulse(i)];
                    float n1 = l1 - s[o + Stride3D.Tangent1Mass] * Vector3F.Dot(dv, t1);
                    float n2 = l2 - s[o + Stride3D.Tangent2Mass] * Vector3F.Dot(dv, t2);
                    float maxF = friction * pl[ManifoldLayout.NormalImpulse(i)];
                    float m2 = n1 * n1 + n2 * n2;
                    if (m2 > maxF * maxF)
                    {
                        float k = maxF / float.Sqrt(m2);
                        n1 *= k; n2 *= k;
                    }

                    pl[ManifoldLayout.Tangent1Impulse(i)] = n1; pl[ManifoldLayout.Tangent2Impulse(i)] = n2;
                    var p = t1 * (n1 - l1) + t2 * (n2 - l2);
                    vA -= p * mA; wA -= Apply(iA, Vector3F.Cross(rA, p));
                    vB += p * mB; wB += Apply(iB, Vector3F.Cross(rB, p));
                }

                for (int i = 0; i < count; i++)
                {
                    int o = Stride3D.Point0 + i * Stride3D.PointStride;
                    var rA = V(s, o + Stride3D.ArmA); var rB = V(s, o + Stride3D.ArmB);
                    // Current separation: the anchors move with their bodies' accumulated translation and rotation.
                    var sepVec = (dpB - dpA) + (Rotate(Dq[qb], Dq[qb + 1], Dq[qb + 2], Dq[qb + 3], rB) - Rotate(Dq[qa], Dq[qa + 1], Dq[qa + 2], Dq[qa + 3], rA));
                    float sep = Vector3F.Dot(sepVec, n) + s[o + Stride3D.AdjustedSeparation];
                    float bias, ms, isc;
                    if (sep > 0f) { bias = sep * invH; ms = 1f; isc = 0f; }
                    else if (useBias)
                    {
                        float x = sep + slop; if (x > 0f) { x = 0f; }
                        bias = biasRate * x; if (bias < negPush) { bias = negPush; }
                        ms = massScale; isc = impulseScale;
                    }
                    else { bias = 0f; ms = 1f; isc = 0f; }

                    var dv = (vB + Vector3F.Cross(wB, rB)) - (vA + Vector3F.Cross(wA, rA));
                    float vn = Vector3F.Dot(dv, n);
                    float old = pl[ManifoldLayout.NormalImpulse(i)];
                    float lambda = -s[o + Stride3D.NormalMass] * ms * (vn + bias) - isc * old;
                    float nw = old + lambda; if (!(nw > 0f)) { nw = 0f; }
                    lambda = nw - old;
                    pl[ManifoldLayout.NormalImpulse(i)] = nw;
                    if (nw > pl[ManifoldLayout.MaxNormalImpulse(i)]) { pl[ManifoldLayout.MaxNormalImpulse(i)] = nw; }
                    var p = n * lambda;
                    vA -= p * mA; wA -= Apply(iA, Vector3F.Cross(rA, p));
                    vB += p * mB; wB += Apply(iB, Vector3F.Cross(rB, p));
                }
            }

            // Only dynamic bodies are written (a static or kinematic body has zero inverse mass).
            if (mA > 0f) { Put(lin, 3 * a, vA); Put(ang, 3 * a, wA); }
            if (mB > 0f) { Put(lin, 3 * b, vB); Put(ang, 3 * b, wB); }
        }
    }

    // ------------------------------------------------------------------ integration

    public void IntegrateVelocities(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> invMass = bodies.InverseMass, rot = bodies.Rotation;
        ReadOnlySpan<byte> type = bodies.Type;
        float h = ctx.SubstepDt;
        var g = Core.Options.Gravity;
        float maxSpeed = Opt.MaxLinearSpeed, maxSpeed2 = maxSpeed * maxSpeed;
        float maxSpin = Opt.MaxRotationPerSubstep / h;
        foreach (int b in list)
        {
            if (type[b] != (byte)BodyType.Dynamic) { continue; }
            float gs = GravityScale[b], im = invMass[b];
            float vx = lin[3 * b] + h * (gs * g.X + im * Force[3 * b]);
            float vy = lin[3 * b + 1] + h * (gs * g.Y + im * Force[3 * b + 1]);
            float vz = lin[3 * b + 2] + h * (gs * g.Z + im * Force[3 * b + 2]);
            var w = V(ang, 3 * b);
            var tq = V(Torque, 3 * b);
            if (tq.X != 0f || tq.Y != 0f || tq.Z != 0f) { w += Apply(WInvI.AsSpan(6 * b, 6), tq) * h; }
            if ((BodyFlags[b] & 2) != 0) { w = Gyroscopic(b, w, h, rot); }
            float dl = 1f / (1f + h * LinearDamping[b]);
            float da = 1f / (1f + h * AngularDamping[b]);
            vx *= dl; vy *= dl; vz *= dl; w *= da;
            float v2 = vx * vx + vy * vy + vz * vz;
            if (v2 > maxSpeed2)
            {
                float scale = maxSpeed / float.Sqrt(v2);
                vx *= scale; vy *= scale; vz *= scale;
            }

            float w2 = w.X * w.X + w.Y * w.Y + w.Z * w.Z;
            if (w2 > maxSpin * maxSpin) { w *= maxSpin / float.Sqrt(w2); }
            lin[3 * b] = vx; lin[3 * b + 1] = vy; lin[3 * b + 2] = vz;
            Put(ang, 3 * b, w);
        }
    }

    /// <summary>
    /// The gyroscopic term ω × Iω as one implicit step (Catto, "Numerical Methods", GDC 2015): in the body frame, one Newton iteration on f(ω) = I(ω − ω₀) + h ω × Iω with the exact Jacobian
    /// J = I + h (ω̃ I − (Iω)~). It conserves angular momentum much better than an explicit step and, unlike a full solve, never blows up for a fast spin because the step is clamped by the speed limit.
    /// </summary>
    private Vector3F Gyroscopic(int b, Vector3F w, float h, ReadOnlySpan<float> rot)
    {
        float qx = rot[4 * b], qy = rot[4 * b + 1], qz = rot[4 * b + 2], qw = rot[4 * b + 3];
        var wl = Rotate(-qx, -qy, -qz, qw, w);
        ReadOnlySpan<float> I = Inertia.AsSpan(6 * b, 6);
        var iw = Apply(I, wl);
        var f = Vector3F.Cross(wl, iw) * h;
        // J = I + h (skew(wl) I − skew(iw)); rows below.
        float a = I[0], bb = I[1], c = I[2], d = I[3], e = I[4], ff = I[5];
        float x = wl.X, y = wl.Y, z = wl.Z;
        // skew(wl) · I: rows (−z·I[1,:] + y·I[2,:]), (z·I[0,:] − x·I[2,:]), (−y·I[0,:] + x·I[1,:])
        float m00 = a + h * (-z * bb + y * c), m01 = bb + h * (-z * d + y * e), m02 = c + h * (-z * e + y * ff);
        float m10 = bb + h * (z * a - x * c), m11 = d + h * (z * bb - x * e), m12 = e + h * (z * c - x * ff);
        float m20 = c + h * (-y * a + x * bb), m21 = e + h * (-y * bb + x * d), m22 = ff + h * (-y * c + x * e);
        // − skew(iw)
        m01 -= h * (-iw.Z); m02 -= h * iw.Y; m10 -= h * iw.Z; m12 -= h * (-iw.X); m20 -= h * (-iw.Y); m21 -= h * iw.X;
        float c00 = m11 * m22 - m12 * m21, c01 = m12 * m20 - m10 * m22, c02 = m10 * m21 - m11 * m20;
        float det = m00 * c00 + m01 * c01 + m02 * c02;
        if (!(Math.Abs(det) > 1e-30f) || !float.IsFinite(det)) { return w; }
        float inv = 1f / det;
        float dx = ((m11 * m22 - m12 * m21) * f.X + (m02 * m21 - m01 * m22) * f.Y + (m01 * m12 - m02 * m11) * f.Z) * inv;
        float dy = (c01 * f.X + (m00 * m22 - m02 * m20) * f.Y + (m02 * m10 - m00 * m12) * f.Z) * inv;
        float dz = (c02 * f.X + (m01 * m20 - m00 * m21) * f.Y + (m00 * m11 - m01 * m10) * f.Z) * inv;
        var nl = new Vector3F(wl.X - dx, wl.Y - dy, wl.Z - dz);
        if (!float.IsFinite(nl.X + nl.Y + nl.Z)) { return w; }
        return Rotate(qx, qy, qz, qw, nl);
    }

    private static void IntegrateRotation(Span<float> q, int o, Vector3F w, float h)
    {
        // Baraff: q' = q + h/2 · (w, 0) · q, renormalised. The product (w, 0)·q is written out.
        float qx = q[o], qy = q[o + 1], qz = q[o + 2], qw = q[o + 3];
        float dx = w.X * qw + w.Y * qz - w.Z * qy;
        float dy = w.Y * qw + w.Z * qx - w.X * qz;
        float dz = w.Z * qw + w.X * qy - w.Y * qx;
        float dw = -(w.X * qx + w.Y * qy + w.Z * qz);
        float hh = 0.5f * h;
        float nx = qx + hh * dx, ny = qy + hh * dy, nz = qz + hh * dz, nw = qw + hh * dw;
        float mag = float.Sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        if (!(mag > 0f) || !float.IsFinite(mag)) { return; } // a runaway state (damaged or extreme input) keeps the last valid orientation instead of becoming a zero quaternion
        float inv = 1f / mag;
        q[o] = nx * inv; q[o + 1] = ny * inv; q[o + 2] = nz * inv; q[o + 3] = nw * inv;
    }

    public void IntegratePositions(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        Span<double> pos = bodies.Position;
        Span<float> rot = bodies.Rotation;
        float h = ctx.SubstepDt;
        foreach (int b in list)
        {
            float dx = h * lin[3 * b], dy = h * lin[3 * b + 1], dz = h * lin[3 * b + 2];
            Dp[3 * b] += dx; Dp[3 * b + 1] += dy; Dp[3 * b + 2] += dz;
            pos[3 * b] += (double)dx; pos[3 * b + 1] += (double)dy; pos[3 * b + 2] += (double)dz;
            var w = V(ang, 3 * b);
            if (w.X != 0f || w.Y != 0f || w.Z != 0f)
            {
                IntegrateRotation(Dq, 4 * b, w, h);
                IntegrateRotation(rot, 4 * b, w, h);
                RefreshWorldInertia(b, invMass, invI, rot);
            }
        }
    }

    // ------------------------------------------------------------------ finalize: forces and continuous collision

    public void FinalizeBodies(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        foreach (int b in list)
        {
            if (bodies.TypeOf(b) != BodyType.Dynamic) { continue; }
            Force[3 * b] = 0f; Force[3 * b + 1] = 0f; Force[3 * b + 2] = 0f; Torque[3 * b] = 0f; Torque[3 * b + 1] = 0f; Torque[3 * b + 2] = 0f;
            if (Opt.ContinuousEnabled) { Continuous(b); }
        }
    }

    private struct ToiQuery : IOverlapCallback
    {
        public Engine3D E;
        public int MoverShape;
        public ConvexShape3D Shape;       // the moving convex shape (or compound child)
        public RigidTransformF ShapePose; // its frame at the start: world rotation, translation relative to the start centre of mass (float)
        public double Cx, Cy, Cz;         // start centre of mass
        public Vector3F Motion;           // centre of mass motion over the step, world axes
        public float DepthTolerance;      // hits that would end the step shallower than this are left to the contact solver
        public float Best;                // smallest fraction so far
        public bool Hit;

        public bool Report(int key, int userId)
        {
            var e = E;
            if (!e.CanCollide(MoverShape, userId)) { return true; }
            var core = e.Core;
            int sb = core.Shapes.Owner[userId];
            var bodies = core.Bodies;
            var p = bodies.PositionOf(sb);
            var other = e.Geoms[e.GeomOf[userId]]!;
            // The static shape's frame in world axes and the mover's shape frame in it.
            var pose = e.ShapePoseFromCom(sb, userId);
            var rs = pose.Rotation;
            var inv = rs.Conjugate();
            var off = new Vector3F(
                (float)((Cx + (double)ShapePose.Translation.X) - (p[0] + (double)pose.Translation.X)),
                (float)((Cy + (double)ShapePose.Translation.Y) - (p[1] + (double)pose.Translation.Y)),
                (float)((Cz + (double)ShapePose.Translation.Z) - (p[2] + (double)pose.Translation.Z)));
            var rel = new RigidTransformF(inv * ShapePose.Rotation, inv.Rotate(off));
            var motion = inv.Rotate(Motion);
            // The hit only matters if the body would end the step well inside the surface: shallower overlaps are the contact solver's job, and clamping them would pin a body that is resting on,
            // or rolling over, a surface (its centre of mass moves along the surface while the shape, held at its start orientation, brushes it).
            float t, approach;
            if (other.IsConvex)
            {
                if (!Gjk3D.ShapeCast(other.Convex, Shape, rel, -motion, out var hit) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = -Vector3F.Dot(motion, hit.Normal);
            }
            else if (other.Compound != null)
            {
                if (!other.Compound.ShapeCast(Shape, rel, motion, out var hit) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = Vector3F.Dot(motion, hit.Normal);
            }
            else
            {
                if (!other.Soup!.ShapeCast(Shape, rel, motion, out var hit) || hit.StartsOverlapping) { return true; }
                t = hit.Fraction; approach = -Vector3F.Dot(motion, hit.Normal);
            }

            if ((1f - t) * approach < DepthTolerance) { return true; }
            // A surface already within speculative range has a contact of its own; clamping against it would only pin the body at a place where no contact exists yet.
            if (t * float.Sqrt(Motion.X * Motion.X + Motion.Y * Motion.Y + Motion.Z * Motion.Z) < 2f * e.Opt.SpeculativeDistance) { return true; }
            if (t > 0f && t < Best) { Best = t; Hit = true; }
            return true;
        }
    }

    /// <summary>
    /// Continuous collision against static geometry for a body that moved far compared with its size this step: a linear shape cast of each shape along the centre-of-mass motion, with the shape held at its
    /// start orientation, finds the first touch; the body is moved back to just before it (a gap of 5 mm along the motion) with its rotation interpolated. Velocity is unchanged: the next step finds a
    /// speculative contact. Rotation during the step is not part of the cast; it only counts towards deciding that the body moved far enough to need one.
    /// </summary>
    private void Continuous(int b)
    {
        var bodies = Core.Bodies;
        var pos = bodies.PositionOf(b);
        var rot = bodies.RotationOf(b);
        double cx = StartPos[3 * b], cy = StartPos[3 * b + 1], cz = StartPos[3 * b + 2];
        var m = new Vector3F((float)(pos[0] - cx), (float)(pos[1] - cy), (float)(pos[2] - cz));
        var q1 = new QuaternionF(StartRot[4 * b], StartRot[4 * b + 1], StartRot[4 * b + 2], StartRot[4 * b + 3]);
        var q2 = new QuaternionF(rot[0], rot[1], rot[2], rot[3]);
        var rel = q1.Conjugate() * q2;
        float spin = 2f * float.Sqrt(rel.X * rel.X + rel.Y * rel.Y + rel.Z * rel.Z) * MaxExtent[b];
        float len = float.Sqrt(m.X * m.X + m.Y * m.Y + m.Z * m.Z);
        if (!(len + spin > 0.5f * MinExtent[b]) || !(len > 0f)) { return; }

        var lc = LocalCenterOf(b);
        float best = 1f;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            var g = Geoms[GeomOf[s]]!;
            if (g.Soup != null) { continue; }
            var l = Local[s];
            // Box of the whole sweep: the shape's bounds at the start and at the end.
            var startPose = new RigidTransformF(q1 * l.Rotation, q1.Rotate(l.Translation - lc));
            var endPose = new RigidTransformF(q2 * l.Rotation, q2.Rotate(l.Translation - lc));
            Vector3F a0, a1, c0, c1;
            if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, startPose, out a0, out a1); ShapeBounds3D.Compute(g.Convex, endPose, out c0, out c1); }
            else { BoundsOfBox(g.LocalBounds, startPose, out a0, out a1); BoundsOfBox(g.LocalBounds, endPose, out c0, out c1); }
            var box = new Aabb3D(
                cx + (double)Math.Min(a0.X, c0.X + m.X), cy + (double)Math.Min(a0.Y, c0.Y + m.Y), cz + (double)Math.Min(a0.Z, c0.Z + m.Z),
                cx + (double)Math.Max(a1.X, c1.X + m.X), cy + (double)Math.Max(a1.Y, c1.Y + m.Y), cz + (double)Math.Max(a1.Z, c1.Z + m.Z));

            if (g.IsConvex)
            {
                var q = new ToiQuery { E = this, MoverShape = s, Shape = g.Convex, ShapePose = startPose, Cx = cx, Cy = cy, Cz = cz, Motion = m, Best = best, DepthTolerance = 0.25f * MinExtent[b] };
                Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
                if (q.Hit && q.Best < best) { best = q.Best; }
            }
            else
            {
                var comp = g.Compound!;
                for (int j = 0; j < comp.SlotCount; j++)
                {
                    if (!comp.IsLive(j)) { continue; }
                    var child = comp.GetChild(j);
                    var childPose = new RigidTransformF(startPose.Rotation * child.Local.Rotation, startPose.Rotation.Rotate(child.Local.Translation) + startPose.Translation);
                    var q = new ToiQuery { E = this, MoverShape = s, Shape = child.Shape, ShapePose = childPose, Cx = cx, Cy = cy, Cz = cz, Motion = m, Best = best, DepthTolerance = 0.25f * MinExtent[b] };
                    Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
                    if (q.Hit && q.Best < best) { best = q.Best; }
                }
            }
        }

        if (best >= 1f) { return; }
        float t = best - 0.005f / len;
        if (t < 0f) { t = 0f; }
        pos[0] = cx + (double)(t * m.X); pos[1] = cy + (double)(t * m.Y); pos[2] = cz + (double)(t * m.Z);
        // Normalised lerp along the shorter arc.
        float dot = q1.X * q2.X + q1.Y * q2.Y + q1.Z * q2.Z + q1.W * q2.W;
        float sgn = dot < 0f ? -1f : 1f;
        float nx = (1f - t) * q1.X + t * sgn * q2.X, ny = (1f - t) * q1.Y + t * sgn * q2.Y, nz = (1f - t) * q1.Z + t * sgn * q2.Z, nw = (1f - t) * q1.W + t * sgn * q2.W;
        float mag = float.Sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
        if (mag > 0f) { rot[0] = nx / mag; rot[1] = ny / mag; rot[2] = nz / mag; rot[3] = nw / mag; }
        Dp[3 * b] = (float)(pos[0] - cx); Dp[3 * b + 1] = (float)(pos[1] - cy); Dp[3 * b + 2] = (float)(pos[2] - cz);
    }

    // ------------------------------------------------------------------ sleeping

    public float BodyMotion(int b)
    {
        var bodies = Core.Bodies;
        // A body touching a moving kinematic body is not at rest.
        var cs = Core.Constraints;
        for (int e = bodies.FirstEdge(b); e >= 0; e = cs.NextEdge(e))
        {
            int c = e >> 1;
            int other = (e & 1) == 0 ? cs.BodyB[c] : cs.BodyA[c];
            if (bodies.TypeOf(other) == BodyType.Kinematic && cs.PointCount[c] > 0 && IsActive(bodies, other)) { return float.MaxValue; }
        }

        var v = bodies.LinearVelocityOf(b);
        var w = bodies.AngularVelocityOf(b);
        return float.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) + float.Sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * MaxExtent[b];
    }
}
