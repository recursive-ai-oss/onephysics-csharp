using System;

namespace RecursiveAI.OnePhysics.Articulations;

/// <summary>The kinds of joint an articulation is built from. Free and FreePlanar are the floating base only.</summary>
internal enum JointKind : byte
{
    Fixed = 0,
    Revolute = 1,
    Prismatic = 2,
    Spherical = 3,
    Free = 4,
    FreePlanar = 5,
}

/// <summary>What the model needs to know about one link: where it hangs, how its joint moves, and its mass.</summary>
internal struct LinkSpec
{
    public int Parent;
    public JointKind Kind;
    /// <summary>Joint axis in the child link's frame (revolute and prismatic); unit.</summary>
    public D3 Axis;
    /// <summary>Placement of the link frame at joint coordinate zero, in the parent link's frame (for the root of a fixed base: in the world).</summary>
    public Q4 PlacementRot;
    public D3 PlacementPos;
    public double Mass;
    /// <summary>Centre of mass in the link frame.</summary>
    public D3 Com;
    /// <summary>Inertia tensor about the centre of mass, in link axes.</summary>
    public M3 Inertia;
}

/// <summary>
/// A tree of rigid links in reduced (joint) coordinates, after Featherstone, <i>Rigid Body Dynamics Algorithms</i> (2008): forward kinematics, recursive Newton–Euler inverse dynamics (table 5.1), the composite-rigid-body
/// mass matrix (table 6.2), the articulated-body forward dynamics (table 7.1) with implicit joint damping and stiffness, and point Jacobians. Everything is double precision and IEEE-exact; no operation depends on anything but the state.
/// Spatial vectors are (angular, linear) in link coordinates; the velocity of a free joint is the body-frame twist.
/// Velocity coordinates ("dofs") are laid out link by link; position coordinates ("pos") likewise (a revolute or prismatic joint 1, a spherical joint a quaternion of 4, a free joint a position and a quaternion of 7).
/// </summary>
internal sealed class ArticulationModel
{
    public readonly int N;
    public readonly int[] Parent, Dof, VelOff, PosOff, PosLen;
    public readonly JointKind[] Kind;
    public readonly int TotalDof, TotalPos;
    public readonly D3[] Axis;
    public readonly Q4[] PlaceRot;
    public readonly D3[] PlacePos;
    public readonly double[] Mass;
    public readonly D3[] Com;
    public readonly M3[] Icom;
    public readonly double[] Smat;           // 36 per link: 6 × dof, row-major with stride dof
    public readonly double[] Ispat;          // 36 per link: spatial inertia about the link origin
    public readonly double[] RootAcc = new double[6];
    /// <summary>The specification the model was built from (the structure never changes afterwards).</summary>
    public readonly LinkSpec[] Specs;

    // State.
    public readonly double[] Q, Qd;

    // Joint parameters, per link (a spherical joint applies its scalars to all three of its dofs).
    public readonly double[] Lower, Upper, Stiff, Damp, MaxForce, Target, TargetVel, FeedFwd, Friction, Armature, JointDamping, MaxVel;
    public readonly bool[] HasLimit;
    public readonly Q4[] TargetRot;         // spherical drive target, per link
    public readonly double[] ConeLimit;     // spherical joints: largest rotation angle from zero, radians (0 = none)

    // Derived by ForwardKinematics.
    public readonly Q4[] Ql, Qw;
    public readonly D3[] Pl, Pw;
    public readonly M3[] Rl, Rw;
    public readonly double[] X;             // 36 per link: motion transform parent → link

    // Scratch.
    private readonly double[] _v, _a, _c, _f, _pa, _ia, _u, _dinv, _uu, _ic, _tmp1, _tmp2, _tmp3;

    public ArticulationModel(ReadOnlySpan<LinkSpec> links)
    {
        N = links.Length;
        Specs = links.ToArray();
        Parent = new int[N]; Dof = new int[N]; VelOff = new int[N]; PosOff = new int[N]; PosLen = new int[N]; Kind = new JointKind[N];
        Axis = new D3[N]; PlaceRot = new Q4[N]; PlacePos = new D3[N]; Mass = new double[N]; Com = new D3[N]; Icom = new M3[N];
        Smat = new double[36 * N]; Ispat = new double[36 * N];
        int vo = 0, po = 0;
        for (int i = 0; i < N; i++)
        {
            ref readonly var l = ref links[i];
            Parent[i] = l.Parent; Kind[i] = l.Kind; Axis[i] = l.Axis; PlaceRot[i] = l.PlacementRot; PlacePos[i] = l.PlacementPos;
            Mass[i] = l.Mass; Com[i] = l.Com; Icom[i] = l.Inertia;
            int dof, pl;
            switch (l.Kind)
            {
                case JointKind.Fixed: dof = 0; pl = 0; break;
                case JointKind.Revolute: case JointKind.Prismatic: dof = 1; pl = 1; break;
                case JointKind.Spherical: dof = 3; pl = 4; break;
                case JointKind.Free: dof = 6; pl = 7; break;
                default: dof = 3; pl = 7; break;
            }

            Dof[i] = dof; PosLen[i] = pl; VelOff[i] = vo; PosOff[i] = po; vo += dof; po += pl;
            BuildS(i, l);
            Dense.BuildInertia(l.Mass, l.Com, l.Inertia, Ispat.AsSpan(36 * i, 36));
        }

        TotalDof = vo; TotalPos = po;
        Q = new double[po]; Qd = new double[vo];
        Lower = new double[N]; Upper = new double[N]; Stiff = new double[N]; Damp = new double[N]; MaxForce = new double[N]; Target = new double[N]; TargetVel = new double[N];
        FeedFwd = new double[N]; Friction = new double[N]; Armature = new double[N]; JointDamping = new double[N]; MaxVel = new double[N]; HasLimit = new bool[N];
        TargetRot = new Q4[N]; ConeLimit = new double[N];
        for (int i = 0; i < N; i++)
        {
            TargetRot[i] = Q4.Identity;
            if (Kind[i] == JointKind.Spherical) { Q[PosOff[i] + 3] = 1.0; }
            else if (Kind[i] == JointKind.Free || Kind[i] == JointKind.FreePlanar) { Q[PosOff[i] + 6] = 1.0; }
        }

        Ql = new Q4[N]; Qw = new Q4[N]; Pl = new D3[N]; Pw = new D3[N]; Rl = new M3[N]; Rw = new M3[N]; X = new double[36 * N];
        _v = new double[6 * N]; _a = new double[6 * N]; _c = new double[6 * N]; _f = new double[6 * N]; _pa = new double[6 * N]; _ia = new double[36 * N]; _u = new double[36 * N];
        _dinv = new double[36 * N]; _uu = new double[6 * N]; _ic = new double[36 * N]; _tmp1 = new double[36]; _tmp2 = new double[36]; _tmp3 = new double[36];
        ForwardKinematics();
    }

    private void BuildS(int i, in LinkSpec l)
    {
        var s = Smat.AsSpan(36 * i, 36);
        s.Clear();
        int dof = Dof[i];
        switch (l.Kind)
        {
            case JointKind.Revolute: s[0 * dof] = l.Axis.X; s[1 * dof] = l.Axis.Y; s[2 * dof] = l.Axis.Z; break;
            case JointKind.Prismatic: s[3 * dof] = l.Axis.X; s[4 * dof] = l.Axis.Y; s[5 * dof] = l.Axis.Z; break;
            case JointKind.Spherical: s[0 * 3 + 0] = 1; s[1 * 3 + 1] = 1; s[2 * 3 + 2] = 1; break;
            case JointKind.Free: for (int k = 0; k < 6; k++) { s[k * 6 + k] = 1; } break;
            case JointKind.FreePlanar: s[2 * 3 + 0] = 1; s[3 * 3 + 1] = 1; s[4 * 3 + 2] = 1; break;
        }
    }

    // ------------------------------------------------------------------ kinematics

    /// <summary>Sets the base pose of a fixed-base articulation (the placement of the root link in the world).</summary>
    public void SetBasePose(D3 p, Q4 q) { PlacePos[0] = p; PlaceRot[0] = q.Normalized(); }

    /// <summary>Recomputes every link's pose and motion transform from the joint positions.</summary>
    public void ForwardKinematics()
    {
        for (int i = 0; i < N; i++)
        {
            Q4 ql; D3 pl;
            int po = PosOff[i];
            switch (Kind[i])
            {
                case JointKind.Revolute: ql = PlaceRot[i] * Q4.FromAxisAngle(Axis[i], Q[po]); pl = PlacePos[i]; break;
                case JointKind.Prismatic: ql = PlaceRot[i]; pl = PlacePos[i] + PlaceRot[i].Rotate(Axis[i] * Q[po]); break;
                case JointKind.Spherical: ql = PlaceRot[i] * new Q4(Q[po], Q[po + 1], Q[po + 2], Q[po + 3]); pl = PlacePos[i]; break;
                case JointKind.Free: case JointKind.FreePlanar: ql = new Q4(Q[po + 3], Q[po + 4], Q[po + 5], Q[po + 6]); pl = new D3(Q[po], Q[po + 1], Q[po + 2]); break;
                default: ql = PlaceRot[i]; pl = PlacePos[i]; break;
            }

            Ql[i] = ql; Pl[i] = pl;
            var rl = ql.ToMatrix();
            Rl[i] = rl;
            Dense.BuildX(rl.Transposed(), pl, X.AsSpan(36 * i, 36));
            int p = Parent[i];
            if (p < 0) { Qw[i] = ql; Pw[i] = pl; Rw[i] = rl; }
            else
            {
                var qw = (Qw[p] * ql).Normalized();
                Qw[i] = qw; Rw[i] = qw.ToMatrix(); Pw[i] = Pw[p] + Rw[p] * pl;
            }
        }
    }

    /// <summary>World position of a point fixed in link <paramref name="link"/> (link coordinates).</summary>
    public D3 WorldPoint(int link, D3 local) => Pw[link] + Rw[link] * local;

    /// <summary>World velocity of a point fixed in a link.</summary>
    public D3 PointVelocity(int link, D3 local)
    {
        ComputeVelocities(_v);
        var v = _v.AsSpan(6 * link, 6);
        var w = Rw[link] * new D3(v[0], v[1], v[2]);
        var vo = Rw[link] * new D3(v[3], v[4], v[5]);
        return vo + D3.Cross(w, Rw[link] * local);
    }

    private void ComputeVelocities(Span<double> v)
    {
        for (int i = 0; i < N; i++)
        {
            var vi = v.Slice(6 * i, 6);
            if (Parent[i] >= 0) { Dense.Mul6(X.AsSpan(36 * i, 36), v.Slice(6 * Parent[i], 6), vi); } else { vi.Clear(); }
            int dof = Dof[i];
            for (int k = 0; k < dof; k++)
            {
                double qd = Qd[VelOff[i] + k];
                if (qd == 0.0) { continue; }
                for (int r = 0; r < 6; r++) { vi[r] += Smat[36 * i + r * dof + k] * qd; }
            }
        }
    }

    /// <summary>Spatial velocity of link <paramref name="link"/> in world axes: angular velocity, and the velocity of the link frame's origin.</summary>
    public void LinkTwist(int link, out D3 omega, out D3 vOrigin)
    {
        ComputeVelocities(_v);
        var v = _v.AsSpan(6 * link, 6);
        omega = Rw[link] * new D3(v[0], v[1], v[2]);
        vOrigin = Rw[link] * new D3(v[3], v[4], v[5]);
    }

    /// <summary>Twists of every link at once: (ω, v origin) in world axes, six doubles per link.</summary>
    public void AllTwists(Span<double> o)
    {
        ComputeVelocities(_v);
        for (int i = 0; i < N; i++)
        {
            var v = _v.AsSpan(6 * i, 6);
            var w = Rw[i] * new D3(v[0], v[1], v[2]);
            var vo = Rw[i] * new D3(v[3], v[4], v[5]);
            o[6 * i] = w.X; o[6 * i + 1] = w.Y; o[6 * i + 2] = w.Z; o[6 * i + 3] = vo.X; o[6 * i + 4] = vo.Y; o[6 * i + 5] = vo.Z;
        }
    }

    // ------------------------------------------------------------------ integration

    /// <summary>Advances the joint positions by <paramref name="h"/> with the present velocities (semi-implicit Euler: call after the velocities were updated).</summary>
    public void IntegratePositions(double h)
    {
        for (int i = 0; i < N; i++)
        {
            int po = PosOff[i], vo = VelOff[i];
            switch (Kind[i])
            {
                case JointKind.Revolute: case JointKind.Prismatic: Q[po] += h * Qd[vo]; break;
                case JointKind.Spherical:
                {
                    var q = new Q4(Q[po], Q[po + 1], Q[po + 2], Q[po + 3]);
                    q = (q * Q4.Exp(new D3(Qd[vo] * h, Qd[vo + 1] * h, Qd[vo + 2] * h))).Normalized();
                    Q[po] = q.X; Q[po + 1] = q.Y; Q[po + 2] = q.Z; Q[po + 3] = q.W;
                    break;
                }

                case JointKind.Free: case JointKind.FreePlanar:
                {
                    D3 w, v;
                    if (Kind[i] == JointKind.Free) { w = new D3(Qd[vo], Qd[vo + 1], Qd[vo + 2]); v = new D3(Qd[vo + 3], Qd[vo + 4], Qd[vo + 5]); }
                    else { w = new D3(0, 0, Qd[vo]); v = new D3(Qd[vo + 1], Qd[vo + 2], 0); }
                    var q = new Q4(Q[po + 3], Q[po + 4], Q[po + 5], Q[po + 6]);
                    var dp = q.Rotate(v) * h;
                    Q[po] += dp.X; Q[po + 1] += dp.Y; Q[po + 2] += dp.Z;
                    q = (q * Q4.Exp(w * h)).Normalized();
                    Q[po + 3] = q.X; Q[po + 4] = q.Y; Q[po + 5] = q.Z; Q[po + 6] = q.W;
                    break;
                }
            }
        }
    }

    // ------------------------------------------------------------------ inverse dynamics

    /// <summary>
    /// Recursive Newton–Euler: the joint forces that produce <paramref name="qdd"/> at the present positions and velocities, under gravity <paramref name="gravity"/> (world axes, acceleration) and the optional external spatial forces
    /// <paramref name="external"/> (six per link: torque and force at the link origin, link axes; empty for none). tau = M(q) qdd + C(q, qd) + g(q).
    /// </summary>
    public void InverseDynamics(ReadOnlySpan<double> qdd, D3 gravity, ReadOnlySpan<double> external, Span<double> tau)
    {
        RootAcc[0] = 0; RootAcc[1] = 0; RootAcc[2] = 0; RootAcc[3] = -gravity.X; RootAcc[4] = -gravity.Y; RootAcc[5] = -gravity.Z;
        Span<double> vj = stackalloc double[6];
        Span<double> sq = stackalloc double[6];
        Span<double> iv = stackalloc double[6];
        Span<double> ia = stackalloc double[6];
        Span<double> bias = stackalloc double[6];
        Span<double> fp = stackalloc double[6];
        for (int i = 0; i < N; i++)
        {
            var vi = _v.AsSpan(6 * i, 6); var ai = _a.AsSpan(6 * i, 6); var ci = _c.AsSpan(6 * i, 6);
            var xi = X.AsSpan(36 * i, 36);
            int dof = Dof[i], vo = VelOff[i];
            vj.Clear(); sq.Clear();
            for (int k = 0; k < dof; k++)
            {
                for (int r = 0; r < 6; r++)
                {
                    double s = Smat[36 * i + r * dof + k];
                    vj[r] += s * Qd[vo + k];
                    sq[r] += s * qdd[vo + k];
                }
            }

            if (Parent[i] >= 0)
            {
                Dense.Mul6(xi, _v.AsSpan(6 * Parent[i], 6), vi);
                Dense.Mul6(xi, _a.AsSpan(6 * Parent[i], 6), ai);
            }
            else
            {
                vi.Clear();
                Dense.Mul6(xi, RootAcc, ai);
            }

            for (int r = 0; r < 6; r++) { vi[r] += vj[r]; }
            Dense.CrossMotion(vi, vj, ci);
            for (int r = 0; r < 6; r++) { ai[r] += sq[r] + ci[r]; }
            var ii = Ispat.AsSpan(36 * i, 36);
            Dense.Mul6(ii, vi, iv);
            Dense.Mul6(ii, ai, ia);
            Dense.CrossForce(vi, iv, bias);
            var fi = _f.AsSpan(6 * i, 6);
            for (int r = 0; r < 6; r++) { fi[r] = ia[r] + bias[r] - (external.Length == 0 ? 0.0 : external[6 * i + r]); }
        }

        for (int i = N - 1; i >= 0; i--)
        {
            var fi = _f.AsSpan(6 * i, 6);
            int dof = Dof[i], vo = VelOff[i];
            for (int k = 0; k < dof; k++)
            {
                double s = 0;
                for (int r = 0; r < 6; r++) { s += Smat[36 * i + r * dof + k] * fi[r]; }
                tau[vo + k] = s;
            }

            if (Parent[i] >= 0)
            {
                Dense.MulT6(X.AsSpan(36 * i, 36), fi, fp);
                var fl = _f.AsSpan(6 * Parent[i], 6);
                for (int r = 0; r < 6; r++) { fl[r] += fp[r]; }
            }
        }
    }

    // ------------------------------------------------------------------ mass matrix

    /// <summary>Composite-rigid-body algorithm: the joint-space mass matrix (TotalDof × TotalDof, row-major, symmetric).</summary>
    public void MassMatrix(Span<double> m)
    {
        int n = TotalDof;
        m.Slice(0, n * n).Clear();
        Array.Copy(Ispat, _ic, 36 * N);
        for (int i = N - 1; i >= 0; i--)
        {
            int p = Parent[i];
            if (p < 0) { continue; }
            Dense.Mul66(_ic.AsSpan(36 * i, 36), X.AsSpan(36 * i, 36), _tmp1);
            Dense.MulT66(X.AsSpan(36 * i, 36), _tmp1, _tmp2);
            var icp = _ic.AsSpan(36 * p, 36);
            for (int k = 0; k < 36; k++) { icp[k] += _tmp2[k]; }
        }

        Span<double> f = stackalloc double[36];
        Span<double> g = stackalloc double[36];
        for (int i = 0; i < N; i++)
        {
            int dof = Dof[i];
            if (dof == 0) { continue; }
            // F = IC_i S_i, 6 × dof (row-major, stride dof)
            for (int r = 0; r < 6; r++)
            {
                for (int k = 0; k < dof; k++)
                {
                    double s = 0;
                    for (int t = 0; t < 6; t++) { s += _ic[36 * i + r * 6 + t] * Smat[36 * i + t * dof + k]; }
                    f[r * dof + k] = s;
                }
            }

            int j = i;
            while (true)
            {
                int dj = Dof[j];
                if (dj > 0)
                {
                    for (int a = 0; a < dj; a++)
                    {
                        for (int b = 0; b < dof; b++)
                        {
                            double s = 0;
                            for (int r = 0; r < 6; r++) { s += Smat[36 * j + r * dj + a] * f[r * dof + b]; }
                            m[(VelOff[j] + a) * n + VelOff[i] + b] = s;
                            m[(VelOff[i] + b) * n + VelOff[j] + a] = s;
                        }
                    }
                }

                int pj = Parent[j];
                if (pj < 0) { break; }
                // F ← X_jᵀ F
                for (int r = 0; r < 6; r++)
                {
                    for (int k = 0; k < dof; k++)
                    {
                        double s = 0;
                        for (int t = 0; t < 6; t++) { s += X[36 * j + t * 6 + r] * f[t * dof + k]; }
                        g[r * dof + k] = s;
                    }
                }

                g.Slice(0, 6 * dof).CopyTo(f);
                j = pj;
            }
        }
    }

    // ------------------------------------------------------------------ forward dynamics

    /// <summary>
    /// Articulated-body algorithm: the joint accelerations under joint forces <paramref name="tau"/>, an added diagonal <paramref name="dAdd"/> on the joint-space mass matrix (the implicit part of damping and stiffness, rotor inertia),
    /// gravity and external spatial forces. Returns false when a joint-space inertia block is not positive definite (a massless link).
    /// </summary>
    public bool ForwardDynamics(ReadOnlySpan<double> tau, ReadOnlySpan<double> dAdd, D3 gravity, ReadOnlySpan<double> external, Span<double> qdd)
    {
        RootAcc[0] = 0; RootAcc[1] = 0; RootAcc[2] = 0; RootAcc[3] = -gravity.X; RootAcc[4] = -gravity.Y; RootAcc[5] = -gravity.Z;
        ComputeVelocities(_v);
        Span<double> vj = stackalloc double[6];
        Span<double> t6 = stackalloc double[6];
        for (int i = 0; i < N; i++)
        {
            int dof = Dof[i], vo = VelOff[i];
            vj.Clear();
            for (int k = 0; k < dof; k++) { for (int r = 0; r < 6; r++) { vj[r] += Smat[36 * i + r * dof + k] * Qd[vo + k]; } }
            var vi = _v.AsSpan(6 * i, 6);
            Dense.CrossMotion(vi, vj, _c.AsSpan(6 * i, 6));
            Array.Copy(Ispat, 36 * i, _ia, 36 * i, 36);
            Dense.Mul6(Ispat.AsSpan(36 * i, 36), vi, t6);
            var pa = _pa.AsSpan(6 * i, 6);
            Dense.CrossForce(vi, t6, pa);
            if (external.Length != 0) { for (int r = 0; r < 6; r++) { pa[r] -= external[6 * i + r]; } }
        }

        Span<double> ucol = stackalloc double[6];
        Span<double> d = stackalloc double[36];
        Span<double> pa2 = stackalloc double[6];
        for (int i = N - 1; i >= 0; i--)
        {
            int dof = Dof[i], vo = VelOff[i];
            var iaI = _ia.AsSpan(36 * i, 36);
            var pai = _pa.AsSpan(6 * i, 6);
            if (dof == 0)
            {
                // A fixed joint: the link is part of its parent.
                int pf = Parent[i];
                if (pf >= 0) { PropagateToParent(i, iaI, pai, _c.AsSpan(6 * i, 6), pf); }
                continue;
            }

            // U = IA S (6 × dof), D = Sᵀ U + dAdd, u = tau − Sᵀ pA
            var u6 = _u.AsSpan(36 * i, 36);
            for (int r = 0; r < 6; r++)
            {
                for (int k = 0; k < dof; k++)
                {
                    double s = 0;
                    for (int t = 0; t < 6; t++) { s += iaI[r * 6 + t] * Smat[36 * i + t * dof + k]; }
                    u6[r * dof + k] = s;
                }
            }

            for (int a = 0; a < dof; a++)
            {
                for (int b = 0; b < dof; b++)
                {
                    double s = 0;
                    for (int r = 0; r < 6; r++) { s += Smat[36 * i + r * dof + a] * u6[r * dof + b]; }
                    d[a * dof + b] = s;
                }

                d[a * dof + a] += dAdd[vo + a];
            }

            if (!Dense.InvertSpd(d.Slice(0, dof * dof), dof)) { return false; }
            d.Slice(0, dof * dof).CopyTo(_dinv.AsSpan(36 * i, dof * dof));
            for (int a = 0; a < dof; a++)
            {
                double s = 0;
                for (int r = 0; r < 6; r++) { s += Smat[36 * i + r * dof + a] * pai[r]; }
                _uu[6 * i + a] = tau[vo + a] - s;
            }

            int p = Parent[i];
            if (p < 0) { continue; }
            // Ia = IA − U D⁻¹ Uᵀ ; pa = pA + Ia c + U D⁻¹ u
            var ia2 = _tmp3;
            for (int r = 0; r < 6; r++)
            {
                for (int s2 = 0; s2 < 6; s2++)
                {
                    double acc = 0;
                    for (int a = 0; a < dof; a++)
                    {
                        double ud = 0;
                        for (int b = 0; b < dof; b++) { ud += u6[r * dof + b] * d[b * dof + a]; }
                        acc += ud * u6[s2 * dof + a];
                    }

                    ia2[r * 6 + s2] = iaI[r * 6 + s2] - acc;
                }
            }

            Dense.Mul6(ia2, _c.AsSpan(6 * i, 6), pa2);
            for (int r = 0; r < 6; r++)
            {
                double s = pai[r] + pa2[r];
                for (int a = 0; a < dof; a++)
                {
                    double dinvU = 0;
                    for (int b = 0; b < dof; b++) { dinvU += d[a * dof + b] * _uu[6 * i + b]; }
                    s += u6[r * dof + a] * dinvU;
                }

                ucol[r] = s;
            }

            PropagateInertia(i, ia2, ucol, p);
        }

        for (int i = 0; i < N; i++)
        {
            int dof = Dof[i], vo = VelOff[i];
            var ai = _a.AsSpan(6 * i, 6);
            if (Parent[i] >= 0) { Dense.Mul6(X.AsSpan(36 * i, 36), _a.AsSpan(6 * Parent[i], 6), ai); } else { Dense.Mul6(X.AsSpan(36 * i, 36), RootAcc, ai); }
            for (int r = 0; r < 6; r++) { ai[r] += _c[6 * i + r]; }
            if (dof == 0) { continue; }
            var u6 = _u.AsSpan(36 * i, 36);
            for (int a = 0; a < dof; a++)
            {
                double ut = 0;
                for (int r = 0; r < 6; r++) { ut += u6[r * dof + a] * ai[r]; }
                _tmp1[a] = _uu[6 * i + a] - ut;
            }

            for (int a = 0; a < dof; a++)
            {
                double s = 0;
                for (int b = 0; b < dof; b++) { s += _dinv[36 * i + a * dof + b] * _tmp1[b]; }
                qdd[vo + a] = s;
            }

            for (int k = 0; k < dof; k++) { for (int r = 0; r < 6; r++) { ai[r] += Smat[36 * i + r * dof + k] * qdd[vo + k]; } }
        }

        return true;
    }

    // A fixed joint adds its whole articulated inertia and bias to the parent (no joint-space reduction).
    private void PropagateToParent(int i, ReadOnlySpan<double> iaI, ReadOnlySpan<double> pai, ReadOnlySpan<double> c, int p)
    {
        Span<double> pa2 = stackalloc double[6];
        Dense.Mul6(iaI, c, pa2);
        for (int r = 0; r < 6; r++) { pa2[r] += pai[r]; }
        PropagateInertia(i, iaI, pa2, p);
    }

    private void PropagateInertia(int i, ReadOnlySpan<double> iaChild, ReadOnlySpan<double> paChild, int p)
    {
        Dense.Mul66(iaChild, X.AsSpan(36 * i, 36), _tmp1);
        Dense.MulT66(X.AsSpan(36 * i, 36), _tmp1, _tmp2);
        var ip = _ia.AsSpan(36 * p, 36);
        for (int k = 0; k < 36; k++) { ip[k] += _tmp2[k]; }
        Span<double> t = stackalloc double[6];
        Dense.MulT6(X.AsSpan(36 * i, 36), paChild, t);
        var pp = _pa.AsSpan(6 * p, 6);
        for (int r = 0; r < 6; r++) { pp[r] += t[r]; }
    }

    // ------------------------------------------------------------------ stability

    private double[]? _stiff, _subtreeMass;

    /// <summary>
    /// An estimate of the highest frequency (rad/s) the explicit part of the step has to resolve, valid right after <see cref="ForwardDynamics"/>: for every joint the geometric stiffness gravity gives it
    /// (the weight hanging beyond it times the lever it acts on, K) over the inertia the joint sees with all the other joints free (the articulated-body inertia D from the last pass), ω² = K/D.
    /// A taut chain with a heavy end is the stiff case: bending one joint is resisted by the whole tension, while only one link's inertia moves.
    /// </summary>
    public double StiffnessFrequency(double gravity)
    {
        _stiff ??= new double[N]; _subtreeMass ??= new double[N];
        for (int i = 0; i < N; i++) { _subtreeMass[i] = Mass[i]; _stiff[i] = Mass[i] * Com[i].Length; }
        for (int i = N - 1; i > 0; i--) { _subtreeMass[Parent[i]] += _subtreeMass[i]; }
        for (int i = 1; i < N; i++) { _stiff[Parent[i]] += _subtreeMass[i] * PlacePos[i].Length; }
        double w2 = 0;
        for (int j = 0; j < N; j++)
        {
            int dof = Dof[j];
            if (dof == 0 || Kind[j] == JointKind.Free || Kind[j] == JointKind.FreePlanar) { continue; }
            double dinv = 0;
            for (int a = 0; a < dof; a++) { dinv = Math.Max(dinv, _dinv[36 * j + a * dof + a]); }
            double v = gravity * _stiff[j] * dinv;
            if (v > w2) { w2 = v; }
        }

        return Math.Sqrt(w2);
    }

    // ------------------------------------------------------------------ Jacobians

    /// <summary>
    /// The Jacobian of a point fixed in link <paramref name="link"/> (link coordinates): <paramref name="linear"/> (3 × TotalDof, row-major) gives the world velocity of the point per unit joint velocity,
    /// <paramref name="angular"/> (3 × TotalDof, or empty) the angular velocity of the link in world axes. Columns of joints off the path to the link are zero.
    /// </summary>
    public void PointJacobian(int link, D3 local, Span<double> linear, Span<double> angular)
    {
        int n = TotalDof;
        linear.Slice(0, 3 * n).Clear();
        if (angular.Length != 0) { angular.Slice(0, 3 * n).Clear(); }
        var point = Pw[link] + Rw[link] * local;
        for (int j = link; j >= 0; j = Parent[j])
        {
            int vo = VelOff[j];
            var o = Pw[j];
            var r = point - o;
            switch (Kind[j])
            {
                case JointKind.Revolute: { var a = Rw[j] * Axis[j]; Put(linear, angular, n, vo, D3.Cross(a, r), a); break; }
                case JointKind.Prismatic: { var a = Rw[j] * Axis[j]; Put(linear, angular, n, vo, a, D3.Zero); break; }
                case JointKind.Spherical:
                {
                    Put(linear, angular, n, vo, D3.Cross(new D3(Rw[j].A, Rw[j].D, Rw[j].G), r), new D3(Rw[j].A, Rw[j].D, Rw[j].G));
                    Put(linear, angular, n, vo + 1, D3.Cross(new D3(Rw[j].B, Rw[j].E, Rw[j].H), r), new D3(Rw[j].B, Rw[j].E, Rw[j].H));
                    Put(linear, angular, n, vo + 2, D3.Cross(new D3(Rw[j].C, Rw[j].F, Rw[j].I), r), new D3(Rw[j].C, Rw[j].F, Rw[j].I));
                    break;
                }

                case JointKind.Free:
                {
                    var c0 = new D3(Rw[j].A, Rw[j].D, Rw[j].G); var c1 = new D3(Rw[j].B, Rw[j].E, Rw[j].H); var c2 = new D3(Rw[j].C, Rw[j].F, Rw[j].I);
                    Put(linear, angular, n, vo, D3.Cross(c0, r), c0); Put(linear, angular, n, vo + 1, D3.Cross(c1, r), c1); Put(linear, angular, n, vo + 2, D3.Cross(c2, r), c2);
                    Put(linear, angular, n, vo + 3, c0, D3.Zero); Put(linear, angular, n, vo + 4, c1, D3.Zero); Put(linear, angular, n, vo + 5, c2, D3.Zero);
                    break;
                }

                case JointKind.FreePlanar:
                {
                    var c0 = new D3(Rw[j].A, Rw[j].D, Rw[j].G); var c1 = new D3(Rw[j].B, Rw[j].E, Rw[j].H); var c2 = new D3(Rw[j].C, Rw[j].F, Rw[j].I);
                    Put(linear, angular, n, vo, D3.Cross(c2, r), c2);
                    Put(linear, angular, n, vo + 1, c0, D3.Zero); Put(linear, angular, n, vo + 2, c1, D3.Zero);
                    break;
                }
            }
        }
    }

    private static void Put(Span<double> lin, Span<double> ang, int n, int col, D3 v, D3 w)
    {
        lin[col] = v.X; lin[n + col] = v.Y; lin[2 * n + col] = v.Z;
        if (ang.Length != 0) { ang[col] = w.X; ang[n + col] = w.Y; ang[2 * n + col] = w.Z; }
    }

    // ------------------------------------------------------------------ energy and centre of mass

    /// <summary>Kinetic energy ½ Σ vᵀ I v of all links.</summary>
    public double KineticEnergy()
    {
        ComputeVelocities(_v);
        double e = 0;
        Span<double> iv = stackalloc double[6];
        for (int i = 0; i < N; i++)
        {
            Dense.Mul6(Ispat.AsSpan(36 * i, 36), _v.AsSpan(6 * i, 6), iv);
            double s = 0;
            for (int r = 0; r < 6; r++) { s += _v[6 * i + r] * iv[r]; }
            e += 0.5 * s;
        }

        return e;
    }

    /// <summary>Potential energy −Σ m g·x of the centres of mass under gravity <paramref name="gravity"/> (acceleration).</summary>
    public double PotentialEnergy(D3 gravity)
    {
        double e = 0;
        for (int i = 0; i < N; i++) { e -= Mass[i] * D3.Dot(gravity, WorldPoint(i, Com[i])); }
        return e;
    }
}
