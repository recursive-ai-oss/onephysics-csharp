using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // ------------------------------------------------------------------ soft constraint coefficients

    /// <summary>
    /// Soft constraint coefficients (Catto, "Soft Step Solver", GDC 2024; "Solver2D"): a spring of frequency <paramref name="hertz"/> and damping ratio <paramref name="zeta"/>
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
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        float hertz = Opt.ContactHertz;
        float cap = 0.25f * ctx.InverseSubstepDt;
        if (hertz > cap) { hertz = cap; }
        MakeSoft(hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out float dynBias, out float dynMass, out float dynImpulse);
        MakeSoft(2f * hertz, Opt.ContactDampingRatio, ctx.SubstepDt, out float stBias, out float stMass, out float stImpulse);

        foreach (int c in constraints)
        {
            if (cs.KindOf(c) == ConstraintKind.Joint) { PrepareJoint(c, in ctx); continue; }
            var pl = cs.Payload(c);
            int a = bodyA[c], b = bodyB[c];
            float mA = invMass[a], iA = invI[a], mB = invMass[b], iB = invI[b];
            float nx = pl[ManifoldLayout.Normal], ny = pl[ManifoldLayout.Normal + 1];
            float tx = ny, ty = -nx;
            var s = Scratch.AsSpan(c * ContactKernel2D.Stride, ContactKernel2D.Stride);
            s.Clear();
            s[Row2D.NormalX] = nx; s[Row2D.NormalY] = ny;
            s[Row2D.Friction] = pl[ManifoldLayout.Friction];
            s[Row2D.Restitution] = pl[ManifoldLayout.Restitution];
            s[Row2D.Rolling] = pl[ManifoldLayout.RollingResistance];
            float k = iA + iB;
            s[Row2D.RollingMass] = k > 0f ? 1f / k : 0f;
            s[Row2D.InvMassA] = mA; s[Row2D.InvInertiaA] = iA; s[Row2D.InvMassB] = mB; s[Row2D.InvInertiaB] = iB;
            if (mA == 0f || mB == 0f) { s[Row2D.BiasRate] = stBias; s[Row2D.MassScale] = stMass; s[Row2D.ImpulseScale] = stImpulse; }
            else { s[Row2D.BiasRate] = dynBias; s[Row2D.MassScale] = dynMass; s[Row2D.ImpulseScale] = dynImpulse; }

            float vAx = lin[2 * a], vAy = lin[2 * a + 1], wA = ang[a], vBx = lin[2 * b], vBy = lin[2 * b + 1], wB = ang[b];
            int count = cs.PointCount[c];
            for (int i = 0; i < count; i++)
            {
                float rAx = pl[ManifoldLayout.AnchorA(i)], rAy = pl[ManifoldLayout.AnchorA(i) + 1];
                float rBx = pl[ManifoldLayout.AnchorB(i)], rBy = pl[ManifoldLayout.AnchorB(i) + 1];
                float sep = pl[ManifoldLayout.Separation(i)];
                int o = Row2D.Point0 + i * Row2D.PointStride;
                s[o + Row2D.AnchorAX] = rAx; s[o + Row2D.AnchorAY] = rAy; s[o + Row2D.AnchorBX] = rBx; s[o + Row2D.AnchorBY] = rBy;
                s[o + Row2D.AdjustedSeparation] = sep - ((rBx - rAx) * nx + (rBy - rAy) * ny);
                float rnA = rAx * ny - rAy * nx, rnB = rBx * ny - rBy * nx;
                float kn = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
                s[o + Row2D.NormalMass] = kn > 0f ? 1f / kn : 0f;
                float rtA = rAx * ty - rAy * tx, rtB = rBx * ty - rBy * tx;
                float kt = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
                s[o + Row2D.TangentMass] = kt > 0f ? 1f / kt : 0f;
                float dvx = (vBx - wB * rBy) - (vAx - wA * rAy);
                float dvy = (vBy + wB * rBx) - (vAy + wA * rAx);
                s[o + Row2D.RelativeVelocity] = dvx * nx + dvy * ny;
                pl[ManifoldLayout.MaxNormalImpulse(i)] = 0f;
            }
        }
    }

    // ------------------------------------------------------------------ solve

    public void SolveBatch(ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext ctx)
    {
        var kp = new KernelParams(ctx.InverseSubstepDt, Opt.PushOutVelocity, Opt.ContactSlop, Opt.RestitutionThreshold);
        if (constraints.Length == 0) { return; }
        var cons = Core.Constraints;
        bool overflow = cons.Color[constraints[0]] == ConstraintStore.Overflow;
        int joints = 0;
        for (int i = 0; i < constraints.Length; i++) { if (cons.KindOf(constraints[i]) == ConstraintKind.Joint) { joints++; } }
        if (joints > 0)
        {
            SolveMixed(constraints, joints, overflow, phase, ctx, in kp);
            return;
        }

        if (overflow || KernelWidth == 1)
        {
            // The overflow set may share bodies inside a group, so it is solved one constraint at a time, in list order.
            for (int i = 0; i < constraints.Length; i++) { ContactKernel2D.Run<Lanes1>(this, constraints.Slice(i, 1), phase, in kp); }
        }
        else if (KernelWidth == 4)
        {
            for (int i = 0; i < constraints.Length; i += 4) { ContactKernel2D.Run<Lanes128>(this, constraints.Slice(i, Math.Min(4, constraints.Length - i)), phase, in kp); }
        }
        else
        {
            ContactKernel2D.Run<Lanes256>(this, constraints, phase, in kp);
        }
    }

    // ------------------------------------------------------------------ integration

    private static void IntegrateRotation(ref float c, ref float s, float angle)
    {
        // Catto: advance the rotation by a small angle linearly, then renormalize.
        float nc = c - angle * s;
        float ns = s + angle * c;
        float mag = float.Sqrt(nc * nc + ns * ns);
        float inv = mag > 0f ? 1f / mag : 0f;
        c = nc * inv;
        s = ns * inv;
    }

    public void IntegrateVelocities(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        Span<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        ReadOnlySpan<float> invMass = bodies.InverseMass, invI = bodies.InverseInertia;
        ReadOnlySpan<byte> type = bodies.Type;
        float h = ctx.SubstepDt;
        var g = Core.Options.Gravity;
        float maxSpeed = Opt.MaxLinearSpeed, maxSpeed2 = maxSpeed * maxSpeed;
        float maxSpin = Opt.MaxRotationPerSubstep / h;
        foreach (int b in list)
        {
            if (type[b] != (byte)BodyType.Dynamic) { continue; }
            float gs = GravityScale[b];
            float vx = lin[2 * b] + h * (gs * g.X + invMass[b] * Force[2 * b]);
            float vy = lin[2 * b + 1] + h * (gs * g.Y + invMass[b] * Force[2 * b + 1]);
            float w = ang[b] + h * (invI[b] * Torque[b]);
            float dl = 1f / (1f + h * LinearDamping[b]);
            float da = 1f / (1f + h * AngularDamping[b]);
            vx *= dl; vy *= dl; w *= da;
            float v2 = vx * vx + vy * vy;
            if (v2 > maxSpeed2)
            {
                float scale = maxSpeed / float.Sqrt(v2);
                vx *= scale; vy *= scale;
            }

            if (w > maxSpin) { w = maxSpin; }
            else if (w < -maxSpin) { w = -maxSpin; }
            lin[2 * b] = vx; lin[2 * b + 1] = vy; ang[b] = w;
        }
    }

    public void IntegratePositions(ReadOnlySpan<int> list, in StepContext ctx)
    {
        var bodies = Core.Bodies;
        ReadOnlySpan<float> lin = bodies.LinearVelocity, ang = bodies.AngularVelocity;
        Span<double> pos = bodies.Position;
        Span<float> rot = bodies.Rotation;
        float h = ctx.SubstepDt;
        foreach (int b in list)
        {
            float dx = h * lin[2 * b], dy = h * lin[2 * b + 1];
            Dp[2 * b] += dx; Dp[2 * b + 1] += dy;
            pos[2 * b] += (double)dx; pos[2 * b + 1] += (double)dy;
            float w = ang[b];
            if (w != 0f)
            {
                float a = h * w;
                IntegrateRotation(ref Dq[2 * b], ref Dq[2 * b + 1], a);
                IntegrateRotation(ref rot[2 * b], ref rot[2 * b + 1], a);
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
            Force[2 * b] = 0f; Force[2 * b + 1] = 0f; Torque[b] = 0f;
            // Bullets are swept after every body has finished (RunBullets), against static and moving shapes alike.
            if (Opt.ContinuousEnabled && !IsBullet(b)) { Continuous(b); }
        }
    }

    private struct ToiQuery : IOverlapCallback
    {
        public Engine2D E;
        public int Shape;      // the fast shape
        public Sweep2D Sweep;  // its body's sweep, relative to the start position
        public double Cx, Cy;  // start position of the fast body's centre of mass
        public Vec2 End;       // end position of the fast body's centre of mass, relative to its start
        public float MinExtent;
        public float Best;     // smallest fraction so far
        public bool Hit;

        public bool Report(int key, int userId)
        {
            var e = E;
            var core = e.Core;
            int other = userId;
            if (e.IsSensor(Shape) || e.IsSensor(other) || !e.CanCollide(Shape, other)) { return true; }
            int sb = core.Shapes.Owner[other];
            var bodies = core.Bodies;
            var p = bodies.PositionOf(sb);
            var r = bodies.RotationOf(sb);
            var q = new Rot2(r[0], r[1]);
            var c = new Vec2((float)(p[0] - Cx), (float)(p[1] - Cy));
            ref readonly var g = ref e.Geom[other];
            if (g.Kind == ShapeKind2D.Segment || g.Kind == ShapeKind2D.ChainSegment)
            {
                // Catto, continuous collision: against a segment only count a body whose centre really moves towards it. This keeps a body sliding
                // along a rough surface (it stays within a few millimetres of it) from being pulled back to its start pose every step.
                var v = g.Vertices;
                Vec2 edge = v[1] - v[0];
                float len = edge.Length;
                if (!(len > 0f)) { return true; }
                Vec2 l0 = q.InverseRotate(new Vec2(-c.X, -c.Y)) - v[0];
                Vec2 l1 = q.InverseRotate(new Vec2(End.X - c.X, End.Y - c.Y)) - v[0];
                float d0 = (edge.X * l0.Y - edge.Y * l0.X) / len; // positive on the left of v0 -> v1
                float d1 = (edge.X * l1.Y - edge.Y * l1.X) / len;
                if (g.Kind == ShapeKind2D.ChainSegment && !g.IsTwoSided)
                {
                    // Solid on the right only: a mover that started behind passes.
                    if (d0 > 0f) { return true; }
                    d0 = -d0; d1 = -d1;
                }
                else if (d0 < 0f) { d0 = -d0; d1 = -d1; }

                if (d0 - d1 < 0.25f * MinExtent) { return true; }
            }

            var sweepStatic = new Sweep2D(default, c, c, q, q);
            var toi = Distance2D.TimeOfImpact(in g, sweepStatic, in e.Geom[Shape], Sweep, Best, Constants2D.LinearSlop);
            if ((toi.State == ToiState.Hit || toi.State == ToiState.Failed) && toi.Fraction > 0f && toi.Fraction < Best)
            {
                Best = toi.Fraction;
                Hit = true;
            }

            return true;
        }
    }

    /// <summary>
    /// Continuous collision against static shapes (Catto, GDC 2013, conservative advancement): a body that moved more than half its smallest dimension this step is swept
    /// from its start pose against every static shape its path touches; if it hit one it is pulled back to the first time of impact, so the next step sees a speculative contact
    /// instead of a body on the far side of the wall.
    /// </summary>
    private void Continuous(int b)
    {
        var bodies = Core.Bodies;
        var pos = bodies.PositionOf(b);
        var rot = bodies.RotationOf(b);
        double c1x = StartPos[2 * b], c1y = StartPos[2 * b + 1];
        float mx = (float)(pos[0] - c1x), my = (float)(pos[1] - c1y);
        float q1c = StartRot[2 * b], q1s = StartRot[2 * b + 1], q2c = rot[0], q2s = rot[1];
        float spin = Math.Abs(q1c * q2s - q1s * q2c) * MaxExtent[b];
        float moved = float.Sqrt(mx * mx + my * my) + spin;
        if (!(moved > 0.5f * MinExtent[b])) { return; }

        var lc = new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]);
        var sweep = new Sweep2D(lc, default, new Vec2(mx, my), new Rot2(q1c, q1s), new Rot2(q2c, q2s));
        float best = 1f;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            // The box of the whole sweep: the shape's bounds at the start and at the end.
            var xf1 = new Transform2(-new Rot2(q1c, q1s).Rotate(lc), new Rot2(q1c, q1s));
            var xf2 = new Transform2(new Vec2(mx, my) - new Rot2(q2c, q2s).Rotate(lc), new Rot2(q2c, q2s));
            var a1 = Geom[s].ComputeAabb(xf1);
            var a2 = Geom[s].ComputeAabb(xf2);
            var box = new Aabb2D(
                c1x + (double)Math.Min(a1.Lower.X, a2.Lower.X), c1y + (double)Math.Min(a1.Lower.Y, a2.Lower.Y),
                c1x + (double)Math.Max(a1.Upper.X, a2.Upper.X), c1y + (double)Math.Max(a1.Upper.Y, a2.Upper.Y));
            var q = new ToiQuery { E = this, Shape = s, Sweep = sweep, Cx = c1x, Cy = c1y, End = new Vec2(mx, my), MinExtent = MinExtent[b], Best = best };
            Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
            if (q.Hit && q.Best < best) { best = q.Best; }
        }

        if (best >= 1f) { return; }
        float t = best;
        pos[0] = c1x + (double)(t * mx);
        pos[1] = c1y + (double)(t * my);
        float nc = (1f - t) * q1c + t * q2c, ns = (1f - t) * q1s + t * q2s;
        float mag = float.Sqrt(nc * nc + ns * ns);
        if (mag > 0f) { rot[0] = nc / mag; rot[1] = ns / mag; }
        Dp[2 * b] = (float)(pos[0] - c1x); Dp[2 * b + 1] = (float)(pos[1] - c1y);
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
        return float.Sqrt(v[0] * v[0] + v[1] * v[1]) + Math.Abs(bodies.AngularVelocityOf(b)[0]) * MaxExtent[b];
    }
}
