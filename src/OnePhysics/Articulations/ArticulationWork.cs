using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Articulations;

/// <summary>
/// The scratch space and the step of one articulation. An articulation's step reads the engine (contacts, other bodies) only in <see cref="Prepare"/>, which runs serially, and writes only its own model and this scratch in
/// <see cref="Simulate"/>, so independent articulations can simulate in parallel; everything the engine receives is written by <see cref="Publish"/>, serially, in slot order.
/// </summary>
internal sealed class ArticulationWork
{
    /// <summary>Contact pushes the link out of penetration at this fraction of the depth per substep (Baumgarte; Catto, GDC 2006).</summary>
    private const double Baumgarte = 0.2;
    private const double ArmatureFloor = 1e-9;

    private enum RowKind : byte { JointFriction, Limit, Normal, Friction1, Friction2 }

    private struct Row
    {
        public RowKind Kind;
        public int Other;              // index into _others, or -1
        public int Contact;            // index into _cp for contact rows, or -1
        public int Support;            // index into _sc for support rows (a pushed body against the ground), or -1
        public int NormalRow;          // for friction rows: the row of the normal impulse that bounds it
        public double InvD, Target, Lo, Hi, Lambda, Mu;
        public D3 Dir;                 // direction the row pushes the other body
        public D3 Arm;                 // from the other body's centre of mass to the point
    }

    private struct ContactPt
    {
        public int Link, Other, Link2;     // Link2 is the second link of a contact inside the articulation (Other is then -1), else -1
        public D3 Local, N, T1, T2, P0, Arm, Local2, P02;
        public double S0, Mu, Ln, Lt1, Lt2;
    }

    // A contact of a body the articulation pushes with the static ground or wall: the articulation's solve keeps that body from being pressed through it.
    private struct SupportPt
    {
        public int Other;
        public D3 N, T1, T2, Arm;
        public double S0, Mu, Ln, Lt1, Lt2;
    }

    private struct OtherBody
    {
        public int Body;
        public D3 V, W, V0, W0;
        public double InvM;
        public M3 InvI;
    }

    private ContactPt[] _cp = new ContactPt[16];
    private int _cpCount;
    private OtherBody[] _others = new OtherBody[8];
    private int _otherCount;
    private SupportPt[] _sc = new SupportPt[16];
    private int _scCount;
    private readonly List<HostContact> _support = new();
    private readonly ArticulationStore _store;
    private readonly IArticulationHost _host;
    private ArticulationModel _m = null!;
    private double _slop, _pushCap;
    private double[] _twist = Array.Empty<double>();

    public ArticulationWork(ArticulationStore store, IArticulationHost host) { _store = store; _host = host; }
    private Row[] _rows = new Row[32];
    private double[] _jlin2 = Array.Empty<double>();
    private double[] _jRows = Array.Empty<double>(), _wRows = Array.Empty<double>(), _mDense = Array.Empty<double>(), _tau = Array.Empty<double>(), _dAdd = Array.Empty<double>(), _qdd = Array.Empty<double>(), _tauG = Array.Empty<double>(), _jlin = Array.Empty<double>(), _zero = Array.Empty<double>(), _col = Array.Empty<double>();
    private double[] _ikJ = Array.Empty<double>();

    internal static void Fit(ref double[] a, int n)
    {
        if (a.Length < n) { a = new double[Math.Max(n, Math.Max(16, a.Length * 2))]; }
    }

    internal static void Fit<T>(ref T[] a, int n)
    {
        if (a.Length < n) { Array.Resize(ref a, Math.Max(n, Math.Max(16, a.Length * 2))); }
    }

    // ------------------------------------------------------------------ the step

    /// <summary>The serial part: forward kinematics and the contacts of this step (read from the engine, bucketed by articulation in <paramref name="sorted"/>[<paramref name="first"/>..<paramref name="last"/>)).</summary>
    public void Prepare(ArticulationState s, List<HostContact> all, int[] sorted, int first, int last, int art)
    {
        _first = first; _last = last; _all = all; _sorted = sorted; _art = art;
        s.Model.BeginRelative();
        s.Model.ForwardKinematics();
        GatherContactPoints(s);
    }

    private List<HostContact> _all = null!;
    private int[] _sorted = Array.Empty<int>();
    private int _first, _last, _art;

    /// <summary>The parallel part: touches only the model, this scratch and read-only world options.</summary>
    public void Simulate(ArticulationState s, float dt, D3 gravity, double slop, double pushCap)
    {
        try { SimulateCore(s, dt, gravity, slop, pushCap); }
        finally { s.Model.EndRelative(); }
    }

    private void SimulateCore(ArticulationState s, float dt, D3 gravity, double slop, double pushCap)
    {
        var m = s.Model;
        _m = m; _slop = slop; _pushCap = pushCap;
        int n = m.TotalDof;
        D3 g = gravity * s.GravityScale;
        Fit(ref _tau, n); Fit(ref _dAdd, n); Fit(ref _qdd, n); Fit(ref _tauG, n); Fit(ref _zero, n); Fit(ref _col, n);
        if (s.HasTarget) { UpdateTarget(s); }
        ReadOnlySpan<double> external = s.AnyExternal ? s.External : ReadOnlySpan<double>.Empty;
        int substeps = s.Substeps;
        bool rk4 = false;
        if (substeps == 0)
        {
            // Automatic: at least 4, more when the highest frequency of the step (see StiffnessFrequency) would make the explicit part unstable (h ω ≤ 1).
            ComputeDrives(s, dt * 0.25, g);
            double w = m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, _qdd.AsSpan(0, n)) ? m.StiffnessFrequency(g.Length) : 0.0;
            // A fast spin needs short steps for accuracy even when nothing is stiff: the energy error of a step grows with (hω)² (second order) or (hω)⁴ (the fourth-order step of passive chains), so h ω is kept near 0.2 or 0.25.
            // A passive chain whose stiffness estimate allows it takes the fourth-order step. An explicit method needs hω below about 2.8 and the estimate can be a factor of two low (a 64-link chain with a 1000:1 end mass), so it takes 2.5 times the substeps the estimate alone asks for.
            rk4 = _cpCount == 0 && IsPassive(s) && dt * w <= 10.0;
            double spin = SpinRate(m, _qdd, dt);
            w = Math.Max(rk4 ? w * 2.5 : w, spin / (rk4 ? 0.25 : 0.2));
            substeps = (int)Math.Min(64.0, Math.Max(4.0, Math.Ceiling(dt * w)));
        }

        double h = (double)dt / substeps;
        for (int sub = 0; sub < substeps; sub++)
        {
            if (rk4)
            {
                ComputeDrives(s, h, g);
                Rk4Substep(h, g, external, n);
                SolveConstraints(s, h);
                ClampVelocities(s);
                continue;
            }

            // Drift, kick, drift (leapfrog): positions and velocities both land on whole steps, second order in the step.
            m.IntegratePositions(0.5 * h);
            m.ForwardKinematics();
            ComputeDrives(s, h, g);
            if (!m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, _qdd.AsSpan(0, n))) { Array.Clear(_qdd, 0, n); }
            // The velocity-dependent terms (Coriolis and centrifugal) are taken at the velocity half way through the kick, found by two fixed-point iterations of the implicit midpoint rule
            // (qd_mid = qd + h/2 · qdd(qd_mid)). One iteration (explicit midpoint) amplifies the pure rotations of a whipping chain by 1 + (hω)⁴/8 a substep and blows it up; two make the
            // amplification 1 − (hω)⁴/8 + …, stable up to hω = 2 and second order.
            Array.Copy(m.Qd, _col, n);
            for (int pass = 0; pass < 2; pass++)
            {
                for (int d = 0; d < n; d++) { m.Qd[d] = _col[d] + 0.5 * h * _qdd[d]; }
                if (!m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, _qdd.AsSpan(0, n))) { Array.Clear(_qdd, 0, n); }
            }

            Array.Copy(_col, m.Qd, n);
            for (int d = 0; d < n; d++) { m.Qd[d] += h * _qdd[d]; }
            SolveConstraints(s, h);
            ClampVelocities(s);
            m.IntegratePositions(0.5 * h);
        }

        m.ForwardKinematics();
        Fit(ref _twist, 6 * m.N);
        for (int k = 0; k < m.N; k++)
        {
            m.LinkTwist(k, out var w, out var v);
            _twist[6 * k] = w.X; _twist[6 * k + 1] = w.Y; _twist[6 * k + 2] = w.Z; _twist[6 * k + 3] = v.X; _twist[6 * k + 4] = v.Y; _twist[6 * k + 5] = v.Z;
        }

        s.ContactPoints = _cpCount;
        double deepest = 0.0;
        for (int c = 0; c < _cpCount; c++)
        {
            double sepNow = Separation(ref _cp[c]);
            if (-sepNow > deepest) { deepest = -sepNow; }
        }

        s.DeepestPenetration = deepest;
        if (s.LastContacts.Length < _cpCount) { s.LastContacts = new Debug.DebugArticulationContact[Math.Max(_cpCount, 8)]; }
        for (int c = 0; c < _cpCount; c++)
        {
            ref var cp = ref _cp[c];
            s.LastContacts[c] = new Debug.DebugArticulationContact(0, cp.Link, new Vector3D(cp.P0.X + m.Anchor.X, cp.P0.Y + m.Anchor.Y, cp.P0.Z + m.Anchor.Z), new Vector3F((float)cp.N.X, (float)cp.N.Y, (float)cp.N.Z), (float)cp.S0, (float)cp.Ln);
        }

        s.LastContactCount = _cpCount;
        // A floating base lands on the 2^-32 grid like every body position; it is rounded relative to the anchor, which is itself on the grid, so a shift of the origin by whole grid steps is exact.
        if (m.Kind[0] != JointKind.Fixed) { int po = m.PosOff[0]; var gr = ArticulationStore.Grid(new D3(m.Q[po], m.Q[po + 1], m.Q[po + 2])); m.Q[po] = gr.X; m.Q[po + 1] = gr.Y; m.Q[po + 2] = gr.Z; }
        if (s.AnyExternal) { Array.Clear(s.External); s.AnyExternal = false; }
    }

    /// <summary>The serial part after the simulation: link velocities for the rigid step and the velocity changes of the bodies the links pushed, written to the engine in slot order.</summary>
    public void Publish(ArticulationState s)
    {
        for (int k = 0; k < s.Model.N; k++)
        {
            _host.SetLinkVelocity(s.Body[k], new D3(_twist[6 * k], _twist[6 * k + 1], _twist[6 * k + 2]), new D3(_twist[6 * k + 3], _twist[6 * k + 4], _twist[6 * k + 5]));
        }

        for (int o = 0; o < _otherCount; o++)
        {
            ref var ob = ref _others[o];
            if (ob.InvM == 0.0) { continue; }
            var dv = ob.V - ob.V0; var dw = ob.W - ob.W0;
            if (dv.X != 0.0 || dv.Y != 0.0 || dv.Z != 0.0 || dw.X != 0.0 || dw.Y != 0.0 || dw.Z != 0.0) { _host.AddBodyVelocity(ob.Body, dv, dw); }
        }
    }

    /// <summary>The fastest joint rotation, rad/s, at the present state and at the end of the step if the present accelerations lasted.</summary>
    private static double SpinRate(ArticulationModel m, double[] qdd, double dt)
    {
        double best = 0.0;
        for (int j = 0; j < m.N; j++)
        {
            int vo = m.VelOff[j];
            double v;
            switch (m.Kind[j])
            {
                case JointKind.Revolute: v = Math.Max(Math.Abs(m.Qd[vo]), Math.Abs(m.Qd[vo] + dt * qdd[vo])); break;
                case JointKind.Spherical: case JointKind.Free: v = Math.Max(new D3(m.Qd[vo], m.Qd[vo + 1], m.Qd[vo + 2]).Length, new D3(m.Qd[vo] + dt * qdd[vo], m.Qd[vo + 1] + dt * qdd[vo + 1], m.Qd[vo + 2] + dt * qdd[vo + 2]).Length); break;
                case JointKind.FreePlanar: v = Math.Max(Math.Abs(m.Qd[vo]), Math.Abs(m.Qd[vo] + dt * qdd[vo])); break;
                default: continue;
            }

            if (v > best) { best = v; }
        }

        return best;
    }

    /// <summary>A fixed-base chain of revolute and prismatic joints with no drives, no joint damping and no limits: nothing stiff, so an explicit fourth-order step is stable and accurate (whips, ropes of links, hanging chains).</summary>
    private double[] _below = Array.Empty<double>();

    private bool IsPassive(ArticulationState s)
    {
        var m = s.Model;
        if (m.Kind[0] != JointKind.Fixed || s.GravityCompensation || s.HasTarget) { return false; }
        for (int j = 1; j < m.N; j++)
        {
            if (m.Kind[j] != JointKind.Revolute && m.Kind[j] != JointKind.Prismatic && m.Kind[j] != JointKind.Fixed) { return false; }
            if (m.Stiff[j] > 0.0 || m.Damp[j] > 0.0 || m.JointDamping[j] > 0.0 || m.HasLimit[j]) { return false; }
        }

        // The stiffness estimate counts gravity only, but a link carrying a much heavier subtree is held by a tension that grows with the motion (a 1000:1 end mass swinging down), and an explicit step then goes unstable
        // beyond the estimate. Chains where no link carries more than 16 times its own mass are far from that.
        Fit(ref _below, m.N);
        var below = _below;
        for (int j = 0; j < m.N; j++) { below[j] = m.Mass[j]; }
        for (int j = m.N - 1; j > 0; j--) { below[m.Parent[j]] += below[j]; }
        for (int j = 1; j < m.N; j++) { if (below[j] > 16.0 * m.Mass[j]) { return false; } }
        return true;
    }

    private double[] _q0 = Array.Empty<double>(), _v0 = Array.Empty<double>(), _a1 = Array.Empty<double>(), _a2 = Array.Empty<double>(), _a3 = Array.Empty<double>(), _v2 = Array.Empty<double>(), _v3 = Array.Empty<double>(), _v4 = Array.Empty<double>();

    /// <summary>
    /// One classical fourth-order Runge–Kutta step of the free motion, positions and velocities together (Kutta 1901; the accelerations are the articulated-body sweep's). Used where nothing in the system is stiff, so the
    /// explicit method is stable, and the energy error of a fast whip falls with the fourth power of the step instead of the second.
    /// </summary>
    private void Rk4Substep(double h, D3 g, ReadOnlySpan<double> external, int n)
    {
        var m = _m;
        Fit(ref _q0, m.TotalPos); Fit(ref _v0, n); Fit(ref _a1, n); Fit(ref _a2, n); Fit(ref _a3, n); Fit(ref _v2, n); Fit(ref _v3, n); Fit(ref _v4, n); Fit(ref _qdd, n);
        Array.Copy(m.Q, _q0, m.TotalPos); Array.Copy(m.Qd, _v0, n);
        var tau = _tau.AsSpan(0, n); var dAdd = _dAdd.AsSpan(0, n);
        m.ForwardKinematics();
        if (!m.ForwardDynamics(tau, dAdd, g, external, _a1.AsSpan(0, n))) { Array.Clear(_a1, 0, n); }
        // Stage 2: half a step along the start velocity and acceleration.
        for (int d = 0; d < n; d++) { _v2[d] = _v0[d] + 0.5 * h * _a1[d]; }
        Stage(h * 0.5, _v0, _v2, g, external, n, _a2);
        // Stage 3: half a step along stage 2.
        for (int d = 0; d < n; d++) { _v3[d] = _v0[d] + 0.5 * h * _a2[d]; }
        Stage(h * 0.5, _v2, _v3, g, external, n, _a3);
        // Stage 4: a whole step along stage 3.
        for (int d = 0; d < n; d++) { _v4[d] = _v0[d] + h * _a3[d]; }
        Stage(h, _v3, _v4, g, external, n, _qdd);
        // Combine.
        Array.Copy(_q0, m.Q, m.TotalPos);
        for (int d = 0; d < n; d++) { m.Qd[d] = (_v0[d] + 2.0 * _v2[d] + 2.0 * _v3[d] + _v4[d]) / 6.0; }
        m.IntegratePositions(h);
        for (int d = 0; d < n; d++) { m.Qd[d] = _v0[d] + h / 6.0 * (_a1[d] + 2.0 * _a2[d] + 2.0 * _a3[d] + _qdd[d]); }
        m.ForwardKinematics();
    }

    // One stage: positions at the start plus dt along the velocity that drives them (move), then the acceleration at the stage's own velocity.
    private void Stage(double dt, double[] drive, double[] velocity, D3 g, ReadOnlySpan<double> external, int n, double[] acc)
    {
        var m = _m;
        Array.Copy(_q0, m.Q, m.TotalPos);
        Array.Copy(drive, m.Qd, n);
        m.IntegratePositions(dt);
        Array.Copy(velocity, m.Qd, n);
        m.ForwardKinematics();
        if (!m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, acc.AsSpan(0, n))) { Array.Clear(acc, 0, n); }
    }

    /// <summary>The gap of a contact now: along its normal, between the point on the link and the point it touches (a fixed point on the other link, or the starting point on a body outside).</summary>
    private double Separation(ref ContactPt cp)
    {
        var m = _m;
        var p1 = m.Pw[cp.Link] + m.Rw[cp.Link] * cp.Local;
        if (cp.Link2 < 0) { return cp.S0 - D3.Dot(cp.N, p1 - cp.P0); }
        var p2 = m.Pw[cp.Link2] + m.Rw[cp.Link2] * cp.Local2;
        return cp.S0 + D3.Dot(cp.N, (p2 - cp.P02) - (p1 - cp.P0));
    }

    private static void ClampVelocities(ArticulationState s)
    {
        var m = s.Model;
        for (int j = 0; j < m.N; j++)
        {
            double mv = m.MaxVel[j];
            if (!(mv > 0.0)) { continue; }
            int vo = m.VelOff[j];
            for (int k = 0; k < m.Dof[j]; k++)
            {
                if (m.Kind[j] == JointKind.Free || m.Kind[j] == JointKind.FreePlanar) { break; }
                if (m.Qd[vo + k] > mv) { m.Qd[vo + k] = mv; } else if (m.Qd[vo + k] < -mv) { m.Qd[vo + k] = -mv; }
            }
        }
    }

    // ------------------------------------------------------------------ drives

    /// <summary>
    /// Joint forces for the coming kick and the implicit part of damping and stiffness (Baraff and Witkin, large steps: the force is linear in the new velocity, so its derivative goes into the mass).
    /// A drive whose force is at its limit contributes a constant and no implicit term.
    /// </summary>
    private void ComputeDrives(ArticulationState s, double h, D3 g)
    {
        var m = s.Model;
        int n = m.TotalDof;
        Array.Clear(_tau, 0, n);
        for (int d = 0; d < n; d++) { _dAdd[d] = ArmatureFloor; }
        if (s.GravityCompensation)
        {
            // g(q): the forces that hold the present pose against gravity alone.
            Array.Copy(m.Qd, _col, n);
            Array.Clear(m.Qd);
            m.InverseDynamics(_zero.AsSpan(0, n), g, ReadOnlySpan<double>.Empty, _tauG.AsSpan(0, n));
            Array.Copy(_col, m.Qd, n);
            for (int j = 0; j < m.N; j++)
            {
                if (m.Kind[j] == JointKind.Free || m.Kind[j] == JointKind.FreePlanar) { continue; }
                for (int k = 0; k < m.Dof[j]; k++) { _tau[m.VelOff[j] + k] += _tauG[m.VelOff[j] + k]; }
            }
        }

        for (int j = 0; j < m.N; j++)
        {
            int vo = m.VelOff[j], po = m.PosOff[j];
            switch (m.Kind[j])
            {
                case JointKind.Revolute:
                case JointKind.Prismatic:
                {
                    double q = m.Q[po], qd = m.Qd[vo];
                    _tau[vo] += m.FeedFwd[j] - m.JointDamping[j] * qd;
                    _dAdd[vo] += h * m.JointDamping[j] + m.Armature[j];
                    if (m.Stiff[j] > 0.0 || m.Damp[j] > 0.0)
                    {
                        double pred = q + h * qd;
                        double td = -m.Stiff[j] * (pred - m.Target[j]) - m.Damp[j] * (qd - m.TargetVel[j]);
                        double lim = m.MaxForce[j];
                        if (lim > 0.0 && td > lim) { _tau[vo] += lim; }
                        else if (lim > 0.0 && td < -lim) { _tau[vo] -= lim; }
                        else { _tau[vo] += td; _dAdd[vo] += h * h * m.Stiff[j] + h * m.Damp[j]; }
                    }

                    break;
                }

                case JointKind.Spherical:
                {
                    var w = new D3(m.Qd[vo], m.Qd[vo + 1], m.Qd[vo + 2]);
                    for (int k = 0; k < 3; k++) { _tau[vo + k] += m.JointDamping[j] * m.Qd[vo + k]; _dAdd[vo + k] += h * m.JointDamping[j] + m.Armature[j]; }
                    if (m.Stiff[j] > 0.0 || m.Damp[j] > 0.0)
                    {
                        var q = new Q4(m.Q[po], m.Q[po + 1], m.Q[po + 2], m.Q[po + 3]);
                        var e = (q.Conjugated() * m.TargetRot[j]).Log();
                        var td = (e - w * h) * m.Stiff[j] - w * m.Damp[j];
                        double lim = m.MaxForce[j], len = td.Length;
                        if (lim > 0.0 && len > lim)
                        {
                            td = td * (lim / len);
                            for (int k = 0; k < 3; k++) { _tau[vo + k] += k == 0 ? td.X : k == 1 ? td.Y : td.Z; }
                        }
                        else
                        {
                            for (int k = 0; k < 3; k++) { _tau[vo + k] += k == 0 ? td.X : k == 1 ? td.Y : td.Z; _dAdd[vo + k] += h * h * m.Stiff[j] + h * m.Damp[j]; }
                        }
                    }

                    break;
                }
            }
        }
    }

    // ------------------------------------------------------------------ task-space target

    /// <summary>
    /// One step of damped least squares (Wampler 1986; Buss and Kim 2005): the joint displacement that moves the target point of a link towards its goal, turned into drive targets for the joints that have a position drive.
    /// </summary>
    private void UpdateTarget(ArticulationState s)
    {
        var m = s.Model;
        int n = m.TotalDof;
        m.ForwardKinematics();
        int rows = s.TargetOrientation ? 6 : 3;
        Fit(ref _ikJ, 6 * n + 6 * n);
        var lin = _ikJ.AsSpan(0, 3 * n); var ang = _ikJ.AsSpan(3 * n, 3 * n);
        m.PointJacobian(s.TargetLink, s.TargetLocal, lin, s.TargetOrientation ? ang : Span<double>.Empty);
        var cur = m.WorldPoint(s.TargetLink, s.TargetLocal);
        var e3 = (s.TargetPosition - m.Anchor) - cur;
        Span<double> err = stackalloc double[6];
        err[0] = e3.X; err[1] = e3.Y; err[2] = e3.Z;
        if (s.TargetOrientation)
        {
            var dq = (s.TargetRotation * m.Qw[s.TargetLink].Conjugated()).Log();
            err[3] = dq.X; err[4] = dq.Y; err[5] = dq.Z;
        }

        // Columns that may move: joints with a position drive.
        Span<double> jj = stackalloc double[36];
        jj.Clear();
        for (int j = 0; j < m.N; j++)
        {
            if (!(m.Kind[j] == JointKind.Revolute || m.Kind[j] == JointKind.Prismatic) || !(m.Stiff[j] > 0.0)) { continue; }
            int c = m.VelOff[j];
            for (int r = 0; r < rows; r++)
            {
                double jr = r < 3 ? lin[r * n + c] : ang[(r - 3) * n + c];
                for (int r2 = 0; r2 < rows; r2++)
                {
                    double j2 = r2 < 3 ? lin[r2 * n + c] : ang[(r2 - 3) * n + c];
                    jj[r * rows + r2] += jr * j2;
                }
            }
        }

        double lam2 = s.IkDamping * s.IkDamping;
        for (int r = 0; r < rows; r++) { jj[r * rows + r] += lam2; }
        if (!Dense.InvertSpd(jj.Slice(0, rows * rows), rows)) { return; }
        Span<double> y = stackalloc double[6];
        for (int r = 0; r < rows; r++)
        {
            double sum = 0;
            for (int r2 = 0; r2 < rows; r2++) { sum += jj[r * rows + r2] * err[r2]; }
            y[r] = sum;
        }

        double biggest = 0;
        for (int j = 0; j < m.N; j++)
        {
            if (!(m.Kind[j] == JointKind.Revolute || m.Kind[j] == JointKind.Prismatic) || !(m.Stiff[j] > 0.0)) { continue; }
            double v = Step1(m, lin, ang, n, rows, y, m.VelOff[j]);
            double a = Math.Abs(v);
            if (a > biggest) { biggest = a; }
        }

        double scale = biggest > s.IkMaxStep ? s.IkMaxStep / biggest : 1.0;
        for (int j = 0; j < m.N; j++)
        {
            if (!(m.Kind[j] == JointKind.Revolute || m.Kind[j] == JointKind.Prismatic) || !(m.Stiff[j] > 0.0)) { continue; }
            double v = Step1(m, lin, ang, n, rows, y, m.VelOff[j]) * scale * s.IkGain;
            double t = m.Q[m.PosOff[j]] + v;
            if (m.HasLimit[j]) { t = t < m.Lower[j] ? m.Lower[j] : t > m.Upper[j] ? m.Upper[j] : t; }
            m.Target[j] = t;
        }
    }

    private static double Step1(ArticulationModel m, ReadOnlySpan<double> lin, ReadOnlySpan<double> ang, int n, int rows, ReadOnlySpan<double> y, int col)
    {
        _ = m;
        double sum = 0;
        for (int r = 0; r < rows; r++) { sum += (r < 3 ? lin[r * n + col] : ang[(r - 3) * n + col]) * y[r]; }
        return sum;
    }

    // ------------------------------------------------------------------ contacts

    private int OtherIndex(int body)
    {
        for (int o = 0; o < _otherCount; o++) { if (_others[o].Body == body) { return o; } }
        Fit(ref _others, _otherCount + 1);
        ref var ob = ref _others[_otherCount];
        ob.Body = body;
        _host.BodyMotion(body, out _, out ob.V, out ob.W);
        _host.BodyInverse(body, out ob.InvM, out ob.InvI);
        ob.V0 = ob.V; ob.W0 = ob.W;
        return _otherCount++;
    }

    private static void Tangents(D3 n, out D3 t1, out D3 t2)
    {
        D3 t = Math.Abs(n.X) >= 0.57735 ? new D3(n.Y, -n.X, 0) : new D3(0, n.Z, -n.Y);
        double len = t.Length;
        t1 = len > 0 ? t * (1.0 / len) : new D3(1, 0, 0);
        t2 = D3.Cross(n, t1);
    }

    private void GatherContactPoints(ArticulationState s)
    {
        var m = s.Model;
        _cpCount = 0; _otherCount = 0;
        int art = _art;
        for (int i = _first; i < _last; i++)
        {
            var c = _all[_sorted[i]];
            bool self = _store.ArticulationOfBody(c.Other) == art;
            // A contact inside one articulation is listed once from each side; the side with the lower body index makes the row.
            if (self && (!s.SelfCollision || c.Other < c.LinkBody)) { continue; }
            int k = _store.LinkOfBody(c.LinkBody);
            Fit(ref _cp, _cpCount + 1);
            ref var p = ref _cp[_cpCount++];
            var p0 = (c.LinkPosition - m.Anchor) + c.PointArm;
            p.P0 = p0;
            p.Link = k; p.Link2 = -1; p.Other = -1; p.Arm = default; p.Local2 = default; p.P02 = default;
            if (self)
            {
                int k2 = _store.LinkOfBody(c.Other);
                p.Link2 = k2;
                p.P02 = p0 + c.Normal * c.Separation;
                p.Local2 = m.Rw[k2].Transposed() * (p.P02 - m.Pw[k2]);
            }
            else { p.Other = OtherIndex(c.Other); p.Arm = c.OtherArm; }
            p.N = c.Normal; p.S0 = c.Separation; p.Mu = c.Friction;
            Tangents(c.Normal, out p.T1, out p.T2);
            p.Local = m.Rw[k].Transposed() * (p.P0 - m.Pw[k]);
            p.Ln = 0; p.Lt1 = 0; p.Lt2 = 0;
        }

        // The bodies the links touch: their contacts with static bodies join the solve, so a body squeezed between an arm and the floor is held by the floor instead of being pushed through it.
        _scCount = 0;
        int touched = _otherCount;
        for (int o = 0; o < touched; o++)
        {
            if (_others[o].InvM == 0.0) { continue; }
            _support.Clear();
            _host.GatherSupportContacts(_others[o].Body, _support);
            foreach (var c in _support)
            {
                Fit(ref _sc, _scCount + 1);
                ref var q = ref _sc[_scCount++];
                q.Other = o; q.N = c.Normal; q.Arm = c.OtherArm; q.S0 = c.Separation; q.Mu = c.Friction;
                Tangents(c.Normal, out q.T1, out q.T2);
                q.Ln = 0; q.Lt1 = 0; q.Lt2 = 0;
            }
        }
    }

    // ------------------------------------------------------------------ the constraint solve

    private static double TargetSpeed(double c, double h, double slop) => c > 0.0 ? -c / h : (c + slop < 0.0 ? -Baumgarte * (c + slop) / h : 0.0);

    private void SolveConstraints(ArticulationState s, double h)
    {
        var m = s.Model;
        int n = m.TotalDof;
        double slop = _slop, pushCap = _pushCap;
        int rowCount = 0;
        int maxRows = 6 * m.N + 3 * _cpCount + 3 * _scCount + 4;
        Fit(ref _rows, maxRows);
        Fit(ref _jRows, maxRows * n); Fit(ref _wRows, maxRows * n);
        // Joint friction and limits.
        for (int j = 0; j < m.N; j++)
        {
            int vo = m.VelOff[j], po = m.PosOff[j];
            switch (m.Kind[j])
            {
                case JointKind.Revolute:
                case JointKind.Prismatic:
                {
                    if (m.Friction[j] > 0.0) { AddUnit(ref rowCount, n, vo, 1.0, RowKind.JointFriction, 0.0, -m.Friction[j] * h, m.Friction[j] * h); }
                    if (m.HasLimit[j])
                    {
                        double q = m.Q[po];
                        double cLo = q - m.Lower[j], cHi = m.Upper[j] - q;
                        double qd = m.Qd[vo];
                        if (cLo < 0.05 + 2.0 * h * Math.Max(0.0, -qd)) { AddUnit(ref rowCount, n, vo, 1.0, RowKind.Limit, TargetSpeed(cLo, h, 0.0), 0.0, double.PositiveInfinity); }
                        if (cHi < 0.05 + 2.0 * h * Math.Max(0.0, qd)) { AddUnit(ref rowCount, n, vo, -1.0, RowKind.Limit, TargetSpeed(cHi, h, 0.0), 0.0, double.PositiveInfinity); }
                    }

                    break;
                }

                case JointKind.Spherical:
                {
                    if (m.Friction[j] > 0.0) { for (int k = 0; k < 3; k++) { AddUnit(ref rowCount, n, vo + k, 1.0, RowKind.JointFriction, 0.0, -m.Friction[j] * h, m.Friction[j] * h); } }
                    if (m.ConeLimit[j] > 0.0)
                    {
                        var q = new Q4(m.Q[po], m.Q[po + 1], m.Q[po + 2], m.Q[po + 3]);
                        var r = q.Log();
                        double theta = r.Length;
                        double c = m.ConeLimit[j] - theta;
                        if (theta > 1e-9 && c < 0.05)
                        {
                            var a = r * (1.0 / theta);
                            ref var row = ref _rows[rowCount];
                            var jr = _jRows.AsSpan(rowCount * n, n);
                            jr.Clear();
                            jr[vo] = -a.X; jr[vo + 1] = -a.Y; jr[vo + 2] = -a.Z;
                            row = default;
                            row.Kind = RowKind.Limit; row.Other = -1; row.Target = TargetSpeed(c, h, 0.0); row.Lo = 0.0; row.Hi = double.PositiveInfinity;
                            rowCount++;
                        }
                    }

                    break;
                }
            }
        }

        // Contacts: tangent rows first (their bounds follow the normal impulse of the previous pass), then the normal.
        Fit(ref _jlin, 3 * n);
        for (int c = 0; c < _cpCount; c++)
        {
            ref var cp = ref _cp[c];
            m.PointJacobian(cp.Link, cp.Local, _jlin.AsSpan(0, 3 * n), Span<double>.Empty);
            if (cp.Link2 >= 0) { Fit(ref _jlin2, 3 * n); m.PointJacobian(cp.Link2, cp.Local2, _jlin2.AsSpan(0, 3 * n), Span<double>.Empty); }
            double sep = Separation(ref cp);
            bool dynamicOther = cp.Other >= 0 && _others[cp.Other].InvM > 0.0;
            int first = rowCount;
            // Friction rows are added before the normal row but refer to it by index.
            AddContactRow(ref rowCount, n, c, cp.T1, RowKind.Friction1, 0.0, cp.Mu, first + 2);
            AddContactRow(ref rowCount, n, c, cp.T2, RowKind.Friction2, 0.0, cp.Mu, first + 2);
            double speed = TargetSpeed(sep, h, slop);
            // A dynamic partner is moved out of penetration by the rigid solver; a push here would be a second one.
            if (dynamicOther && sep < 0.0) { speed = 0.0; }
            if (speed > pushCap) { speed = pushCap; }
            AddContactRow(ref rowCount, n, c, cp.N, RowKind.Normal, speed, 0.0, -1);
            _rows[first].Lambda = cp.Lt1; _rows[first + 1].Lambda = cp.Lt2; _rows[first + 2].Lambda = cp.Ln;
        }

        // Support rows: a pushed body against the ground. They have no joint part; they only limit what the links can do to the body.
        for (int c = 0; c < _scCount; c++)
        {
            ref var sp = ref _sc[c];
            int first = rowCount;
            AddSupportRow(ref rowCount, n, c, sp.T1, RowKind.Friction1, 0.0, sp.Mu, first + 2);
            AddSupportRow(ref rowCount, n, c, sp.T2, RowKind.Friction2, 0.0, sp.Mu, first + 2);
            double speed = sp.S0 > 0.0 ? -sp.S0 / h : 0.0;   // the rigid solver moves a body out of penetration; here it only may not be pushed deeper
            AddSupportRow(ref rowCount, n, c, sp.N, RowKind.Normal, speed, 0.0, -1);
            _rows[first].Lambda = sp.Lt1; _rows[first + 1].Lambda = sp.Lt2; _rows[first + 2].Lambda = sp.Ln;
        }

        if (rowCount == 0) { return; }

        // Dense effective mass: M(q) + the implicit diagonal, factored once; W = M⁻¹ Jᵀ for every row.
        Fit(ref _mDense, n * n);
        m.MassMatrix(_mDense.AsSpan(0, n * n));
        for (int d = 0; d < n; d++) { _mDense[d * n + d] += _dAdd[d]; }
        if (!Dense.Cholesky(_mDense.AsSpan(0, n * n), n)) { return; }
        for (int r = 0; r < rowCount; r++)
        {
            var w = _wRows.AsSpan(r * n, n);
            _jRows.AsSpan(r * n, n).CopyTo(w);
            Dense.CholeskySolve(_mDense.AsSpan(0, n * n), n, w);
            double d = 0;
            var jr = _jRows.AsSpan(r * n, n);
            for (int k = 0; k < n; k++) { d += jr[k] * w[k]; }
            ref var row = ref _rows[r];
            if (row.Other >= 0)
            {
                ref var ob = ref _others[row.Other];
                if (ob.InvM > 0.0)
                {
                    var rxd = D3.Cross(row.Arm, row.Dir);
                    d += ob.InvM + D3.Dot(rxd, ob.InvI * rxd);
                }
            }

            row.InvD = d > 0.0 ? 1.0 / d : 0.0;
        }

        // Warm start from the previous substep's contact impulses.
        for (int r = 0; r < rowCount; r++)
        {
            ref var row = ref _rows[r];
            if (row.Lambda != 0.0) { Apply(r, row.Lambda, n); }
        }

        int iterations = s.Iterations;
        for (int it = 0; it < iterations; it++)
        {
            for (int r = 0; r < rowCount; r++)
            {
                ref var row = ref _rows[r];
                if (row.InvD == 0.0) { continue; }
                double v = 0;
                var jr = _jRows.AsSpan(r * n, n);
                for (int k = 0; k < n; k++) { v += jr[k] * m.Qd[k]; }
                if (row.Other >= 0)
                {
                    ref var ob = ref _others[row.Other];
                    v += D3.Dot(row.Dir, ob.V + D3.Cross(ob.W, row.Arm));
                }

                double lo = row.Lo, hi = row.Hi;
                if (row.Kind == RowKind.Friction1 || row.Kind == RowKind.Friction2) { double b = row.Mu * _rows[row.NormalRow].Lambda; lo = -b; hi = b; }
                double nl = row.Lambda - (v - row.Target) * row.InvD;
                nl = nl < lo ? lo : nl > hi ? hi : nl;
                double dl = nl - row.Lambda;
                row.Lambda = nl;
                if (dl != 0.0) { Apply(r, dl, n); }
            }
        }

        for (int r = 0; r < rowCount; r++)
        {
            ref var row = ref _rows[r];
            if (row.Kind < RowKind.Normal) { continue; }
            if (row.Contact >= 0)
            {
                ref var cp = ref _cp[row.Contact];
                if (row.Kind == RowKind.Normal) { cp.Ln = row.Lambda; } else if (row.Kind == RowKind.Friction1) { cp.Lt1 = row.Lambda; } else { cp.Lt2 = row.Lambda; }
            }
            else if (row.Support >= 0)
            {
                ref var sp = ref _sc[row.Support];
                if (row.Kind == RowKind.Normal) { sp.Ln = row.Lambda; } else if (row.Kind == RowKind.Friction1) { sp.Lt1 = row.Lambda; } else { sp.Lt2 = row.Lambda; }
            }
        }
    }

    private void Apply(int r, double dl, int n)
    {
        var m = _m;
        var w = _wRows.AsSpan(r * n, n);
        for (int k = 0; k < n; k++) { m.Qd[k] += w[k] * dl; }
        ref var row = ref _rows[r];
        if (row.Other >= 0)
        {
            ref var ob = ref _others[row.Other];
            if (ob.InvM > 0.0)
            {
                ob.V = ob.V + row.Dir * (ob.InvM * dl);
                ob.W = ob.W + ob.InvI * D3.Cross(row.Arm, row.Dir) * dl;
            }
        }
    }

    private void AddUnit(ref int rowCount, int n, int dof, double sign, RowKind kind, double target, double lo, double hi)
    {
        ref var row = ref _rows[rowCount];
        row = default;
        row.Kind = kind; row.Other = -1; row.Contact = -1; row.Support = -1; row.NormalRow = -1; row.Target = target; row.Lo = lo; row.Hi = hi;
        var jr = _jRows.AsSpan(rowCount * n, n);
        jr.Clear();
        jr[dof] = sign;
        rowCount++;
    }

    private void AddContactRow(ref int rowCount, int n, int contact, D3 dir, RowKind kind, double target, double mu, int normalRow)
    {
        ref var row = ref _rows[rowCount];
        row = default;
        ref var cp = ref _cp[contact];
        row.Kind = kind; row.Other = cp.Other; row.Contact = contact; row.Support = -1; row.NormalRow = normalRow; row.Target = target; row.Mu = mu; row.Dir = dir; row.Arm = cp.Arm;
        row.Lo = kind == RowKind.Normal ? 0.0 : double.NegativeInfinity; row.Hi = double.PositiveInfinity;
        var jr = _jRows.AsSpan(rowCount * n, n);
        for (int k = 0; k < n; k++) { jr[k] = -(dir.X * _jlin[k] + dir.Y * _jlin[n + k] + dir.Z * _jlin[2 * n + k]); }
        if (cp.Link2 >= 0) { for (int k = 0; k < n; k++) { jr[k] += dir.X * _jlin2[k] + dir.Y * _jlin2[n + k] + dir.Z * _jlin2[2 * n + k]; } }
        rowCount++;
    }

    private void AddSupportRow(ref int rowCount, int n, int support, D3 dir, RowKind kind, double target, double mu, int normalRow)
    {
        ref var row = ref _rows[rowCount];
        row = default;
        ref var sp = ref _sc[support];
        row.Kind = kind; row.Other = sp.Other; row.Contact = -1; row.Support = support; row.NormalRow = normalRow; row.Target = target; row.Mu = mu; row.Dir = dir; row.Arm = sp.Arm;
        row.Lo = kind == RowKind.Normal ? 0.0 : double.NegativeInfinity; row.Hi = double.PositiveInfinity;
        _jRows.AsSpan(rowCount * n, n).Clear();
        rowCount++;
    }
}
