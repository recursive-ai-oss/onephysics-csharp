using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Articulations;

internal sealed partial class ArticulationStore
{
    /// <summary>Contact pushes the link out of penetration at this fraction of the depth per substep (Baumgarte; Catto, GDC 2006).</summary>
    private const double Baumgarte = 0.2;
    private const double ArmatureFloor = 1e-9;

    private enum RowKind : byte { JointFriction, Limit, Normal, Friction1, Friction2 }

    private struct Row
    {
        public RowKind Kind;
        public int Other;              // index into _others, or -1
        public int Contact;            // index into _cp for contact rows
        public int NormalRow;          // for friction rows: the row of the normal impulse that bounds it
        public double InvD, Target, Lo, Hi, Lambda, Mu;
        public D3 Dir;                 // direction the row pushes the other body
    }

    private struct ContactPt
    {
        public int Link, Other;
        public D3 Local, N, T1, T2, P0, Arm;
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
    private Row[] _rows = new Row[32];
    private double[] _jRows = Array.Empty<double>(), _wRows = Array.Empty<double>(), _mDense = Array.Empty<double>(), _tau = Array.Empty<double>(), _dAdd = Array.Empty<double>(), _qdd = Array.Empty<double>(), _tauG = Array.Empty<double>(), _jlin = Array.Empty<double>(), _zero = Array.Empty<double>(), _col = Array.Empty<double>();
    private double[] _ikJ = Array.Empty<double>();

    private static void Fit(ref double[] a, int n)
    {
        if (a.Length < n) { a = new double[Math.Max(n, Math.Max(16, a.Length * 2))]; }
    }

    private static void Fit<T>(ref T[] a, int n)
    {
        if (a.Length < n) { Array.Resize(ref a, Math.Max(n, Math.Max(16, a.Length * 2))); }
    }

    // ------------------------------------------------------------------ the stage

    /// <summary>Runs every articulation for one step, in slot order. Called once per world step after the contact stage has made this step's manifolds.</summary>
    public void Step(float dt)
    {
        if (Handles.Count == 0) { return; }
        // One scan of the constraint table, then the contacts are bucketed by articulation (counting sort, so each keeps its constraint order).
        _all.Clear();
        _host.GatherLinkContacts(_all);
        int slots = Handles.HighWater;
        Fit(ref _start, slots + 2);
        Array.Clear(_start, 0, slots + 2);
        foreach (var c in _all) { _start[ArticulationOfBody(c.LinkBody) + 1]++; }
        for (int i = 1; i <= slots; i++) { _start[i] += _start[i - 1]; }
        Fit(ref _sorted, _all.Count);
        Fit(ref _fill, slots + 1);
        Array.Copy(_start, _fill, slots + 1);
        for (int k = 0; k < _all.Count; k++) { _sorted[_fill[ArticulationOfBody(_all[k].LinkBody)]++] = k; }
        for (int i = 0; i < slots; i++)
        {
            if (Handles.IsLive(i)) { _first = _start[i]; _last = _start[i + 1]; StepOne(States[i]!, dt); }
        }
    }

    private readonly List<HostContact> _all = new();
    private int[] _start = new int[8], _sorted = new int[8], _fill = new int[8];
    private int _first, _last;

    /// <summary>Puts every link body at the pose its joint coordinates give, at the end of the step.</summary>
    public void EndStep()
    {
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            Place(i);
        }
    }

    // Positions of bodies sit on a 2^-32 grid (see the engine's integration), so a shifted world steps like an unshifted one.
    private const double GridScale = 4294967296.0;

    private static D3 Grid(D3 p) => new(Math.Floor(p.X * GridScale + 0.5) / GridScale, Math.Floor(p.Y * GridScale + 0.5) / GridScale, Math.Floor(p.Z * GridScale + 0.5) / GridScale);

    private void StepOne(ArticulationState s, float dt)
    {
        var m = s.Model;
        int n = m.TotalDof;
        D3 g = _host.Gravity * s.GravityScale;
        Fit(ref _tau, n); Fit(ref _dAdd, n); Fit(ref _qdd, n); Fit(ref _tauG, n); Fit(ref _zero, n); Fit(ref _col, n);
        if (s.HasTarget) { UpdateTarget(s); }
        m.ForwardKinematics();
        GatherContactPoints(s);
        ReadOnlySpan<double> external = s.AnyExternal ? s.External : ReadOnlySpan<double>.Empty;
        int substeps = s.Substeps;
        if (substeps == 0)
        {
            // Automatic: at least 4, more when the highest frequency of the step (see StiffnessFrequency) would make the explicit part unstable (h ω ≤ 1).
            ComputeDrives(s, dt * 0.25, g);
            double w = m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, _qdd.AsSpan(0, n)) ? m.StiffnessFrequency(g.Length) : 0.0;
            substeps = (int)Math.Min(64.0, Math.Max(4.0, Math.Ceiling(dt * w)));
        }

        double h = (double)dt / substeps;
        for (int sub = 0; sub < substeps; sub++)
        {
            // Drift, kick, drift (leapfrog): positions and velocities both land on whole steps, second order in the step.
            m.IntegratePositions(0.5 * h);
            m.ForwardKinematics();
            ComputeDrives(s, h, g);
            if (!m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, _qdd.AsSpan(0, n))) { Array.Clear(_qdd, 0, n); }
            // The velocity-dependent terms (Coriolis and centrifugal) are taken at the velocity half way through the kick (explicit midpoint on the velocity), which keeps the step second order.
            Array.Copy(m.Qd, _col, n);
            for (int d = 0; d < n; d++) { m.Qd[d] += 0.5 * h * _qdd[d]; }
            if (!m.ForwardDynamics(_tau.AsSpan(0, n), _dAdd.AsSpan(0, n), g, external, _qdd.AsSpan(0, n))) { Array.Clear(_qdd, 0, n); }
            Array.Copy(_col, m.Qd, n);
            for (int d = 0; d < n; d++) { m.Qd[d] += h * _qdd[d]; }
            SolveConstraints(s, h);
            ClampVelocities(s);
            m.IntegratePositions(0.5 * h);
        }

        m.ForwardKinematics();
        // Velocities of the link bodies for the rigid step (the narrowphase has already run) and the velocity changes of the bodies the links pushed.
        for (int k = 0; k < m.N; k++)
        {
            m.LinkTwist(k, out var w, out var v);
            _host.SetLinkVelocity(s.Body[k], w, v);
        }

        for (int o = 0; o < _otherCount; o++)
        {
            ref var ob = ref _others[o];
            if (ob.InvM == 0.0) { continue; }
            var dv = ob.V - ob.V0; var dw = ob.W - ob.W0;
            if (dv.X != 0.0 || dv.Y != 0.0 || dv.Z != 0.0 || dw.X != 0.0 || dw.Y != 0.0 || dw.Z != 0.0) { _host.AddBodyVelocity(ob.Body, dv, dw); }
        }

        if (s.AnyExternal) { Array.Clear(s.External); s.AnyExternal = false; }
        s.ContactPoints = _cpCount;
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
        var e3 = s.TargetPosition - cur;
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
        int art = ArticulationOfBody(s.Body[0]);
        for (int i = _first; i < _last; i++)
        {
            var c = _all[_sorted[i]];
            if (ArticulationOfBody(c.Other) == art) { continue; }
            int k = LinkOfBody(c.LinkBody);
            Fit(ref _cp, _cpCount + 1);
            ref var p = ref _cp[_cpCount++];
            p.Link = k; p.Other = OtherIndex(c.Other);
            p.P0 = c.Point; p.N = c.Normal; p.Arm = c.OtherArm; p.S0 = c.Separation; p.Mu = c.Friction;
            Tangents(c.Normal, out p.T1, out p.T2);
            p.Local = m.Rw[k].Transposed() * (c.Point - m.Pw[k]);
            p.Ln = 0; p.Lt1 = 0; p.Lt2 = 0;
        }
    }

    // ------------------------------------------------------------------ the constraint solve

    private static double TargetSpeed(double c, double h, double slop) => c > 0.0 ? -c / h : (c + slop < 0.0 ? -Baumgarte * (c + slop) / h : 0.0);

    private void SolveConstraints(ArticulationState s, double h)
    {
        var m = s.Model;
        int n = m.TotalDof;
        double slop = _host.ContactSlop, pushCap = _host.PushOutVelocity;
        _applyModel = m;
        int rowCount = 0;
        int maxRows = 6 * m.N + 3 * _cpCount + 4;
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
            var pNow = m.Pw[cp.Link] + m.Rw[cp.Link] * cp.Local;
            double sep = cp.S0 - D3.Dot(cp.N, pNow - cp.P0);
            ref var ob = ref _others[cp.Other];
            bool dynamicOther = ob.InvM > 0.0;
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
                    var rxd = D3.Cross(_cp[row.Contact].Arm, row.Dir);
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
                    v += D3.Dot(row.Dir, ob.V + D3.Cross(ob.W, _cp[row.Contact].Arm));
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
            if (row.Contact < 0 || row.Kind < RowKind.Normal) { continue; }
            ref var cp = ref _cp[row.Contact];
            if (row.Kind == RowKind.Normal) { cp.Ln = row.Lambda; } else if (row.Kind == RowKind.Friction1) { cp.Lt1 = row.Lambda; } else { cp.Lt2 = row.Lambda; }
        }
    }

    private void Apply(int r, double dl, int n)
    {
        var m = _applyModel!;
        var w = _wRows.AsSpan(r * n, n);
        for (int k = 0; k < n; k++) { m.Qd[k] += w[k] * dl; }
        ref var row = ref _rows[r];
        if (row.Other >= 0)
        {
            ref var ob = ref _others[row.Other];
            if (ob.InvM > 0.0)
            {
                ob.V = ob.V + row.Dir * (ob.InvM * dl);
                ob.W = ob.W + ob.InvI * D3.Cross(_cp[row.Contact].Arm, row.Dir) * dl;
            }
        }
    }

    private ArticulationModel? _applyModel;

    private void AddUnit(ref int rowCount, int n, int dof, double sign, RowKind kind, double target, double lo, double hi)
    {
        ref var row = ref _rows[rowCount];
        row = default;
        row.Kind = kind; row.Other = -1; row.Contact = -1; row.NormalRow = -1; row.Target = target; row.Lo = lo; row.Hi = hi;
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
        row.Kind = kind; row.Other = cp.Other; row.Contact = contact; row.NormalRow = normalRow; row.Target = target; row.Mu = mu; row.Dir = dir;
        row.Lo = kind == RowKind.Normal ? 0.0 : double.NegativeInfinity; row.Hi = double.PositiveInfinity;
        var jr = _jRows.AsSpan(rowCount * n, n);
        for (int k = 0; k < n; k++) { jr[k] = -(dir.X * _jlin[k] + dir.Y * _jlin[n + k] + dir.Z * _jlin[2 * n + k]); }
        rowCount++;
    }
}
