using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Cables;

internal sealed partial class CableStore
{
    // ---------------------------------------------------------------- rows

    private static Term MakeTerm(Scratch sc, CableData m, int ni, Vector3D pt, Vector3D dir, ref double cst)
    {
        var t = new Term { Be = -1, Se = -1 };
        var node = m.Nodes[ni]; int e = sc.NodeEnt[ni];
        if (!node.IsWheel)
        {
            if (e < 0) return t;
            ref var b = ref sc.B[e];
            if (b.Dyn) { t.Be = e; t.Lin = dir; t.Ang = Vector3D.Cross(pt - b.X, dir); b.InSolver = true; }
            else cst += Vector3D.Dot(dir, b.V + Vector3D.Cross(b.W, pt - b.X));
            return t;
        }
        var n = sc.AxisW[ni];
        double c = Vector3D.Dot(dir, Vector3D.Cross(n, pt - sc.Center[ni]));
        if (node.Mode == WheelSpin.Rotor)
        {
            t.Se = ni; t.Sc = c;
            if (e >= 0)
            {
                ref var b = ref sc.B[e];
                if (b.Dyn && !node.FixedAxle) { t.Be = e; t.Lin = dir; t.Ang = Vector3D.Cross(pt - b.X, dir) - n * c; b.InSolver = true; }
                else cst += Vector3D.Dot(dir, b.V + Vector3D.Cross(b.W, pt - b.X)) - c * Vector3D.Dot(b.W, n);
            }
        }
        else if (e >= 0)
        {
            ref var b = ref sc.B[e];
            if (b.Dyn && !node.FixedAxle) { t.Be = e; t.Lin = dir; t.Ang = Vector3D.Cross(pt - b.X, dir); b.InSolver = true; }
            else if (b.Dyn) { t.Se = ni; t.Sc = c; cst += Vector3D.Dot(dir, b.V + Vector3D.Cross(b.W, pt - b.X)) - c * Vector3D.Dot(b.W, n); }
            else cst += Vector3D.Dot(dir, b.V + Vector3D.Cross(b.W, pt - b.X));
        }
        return t;
    }

    private void BuildRows(Scratch sc, CableData m, float dt)
    {
        int n = m.Nodes.Length, spans = m.Spans;
        for (int e = 0; e < sc.NB; e++) sc.B[e].InSolver = false;
        // Spin entities.
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i]; ref var s = ref sc.Spin[i]; s = default; s.Carrier = -1;
            if (!node.IsWheel) continue;
            int e = sc.NodeEnt[i];
            if (node.Mode == WheelSpin.Rotor)
            {
                s.Active = true; s.S = node.Spin / (1.0 + (double)node.Bearing * dt);
                s.InvI = float.IsPositiveInfinity(node.Inertia) ? 0 : 1.0 / node.Inertia;
                if (e >= 0 && sc.B[e].Dyn && !node.FixedAxle) s.Carrier = e;
            }
            else if (node.FixedAxle && e >= 0 && sc.B[e].Dyn)
            {
                var ax = sc.AxisW[i];
                s.Active = true; s.S = Vector3D.Dot(sc.B[e].W, ax); s.InvI = Vector3D.Dot(ax, sc.B[e].Inertia(Host.Planar, ax));
            }
        }
        int r = 0;
        for (int k = 0; k < spans; k++, r++)
        {
            ref var row = ref sc.Rows[r]; row = default; row.Node = -1;
            row.Valid = sc.Valid[k];
            if (!row.Valid) { sc.Lambda[r] = 0; continue; }
            int j = (k + 1) % n; var d = sc.Dir[k];
            double cst = 0;
            row.T0 = MakeTerm(sc, m, k, sc.P[k], -d, ref cst);
            row.T1 = MakeTerm(sc, m, j, sc.Q[k], d, ref cst);
            row.Const = cst;
            double c = m.Geometric[k] - m.Material[k];
            m.Stretch[k] = c;
            // Within a deadband (ten micrometres; a millimetre for a chain, whose sprockets' integrated angles drift faster) the stretch is rounding noise (body angles are float32); acting on it would turn into a velocity error of c/dt, so the span just holds its velocity.
            double band = m.Chain ? 1e-3 : Deadband, e = c > band ? c - band : c < -band ? c + band : 0;
            row.Bias = e > 0 ? Beta * e / dt : e / dt;
        }
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i];
            if (!node.IsWheel || node.Mode != WheelSpin.Rotor) continue;
            for (int kind = 1; kind <= 2; kind++)
            {
                if ((kind == 1 ? node.MotorTorque : node.BrakeTorque) <= 0) continue;
                ref var row = ref sc.Rows[r]; row = default; row.Valid = true; row.Node = i; row.Kind = kind;
                var ax = sc.AxisW[i]; int e = sc.NodeEnt[i];
                row.T0 = new Term { Be = -1, Se = i, Sc = 1 }; row.T1 = new Term { Be = -1, Se = -1 };
                if (e >= 0)
                {
                    ref var b = ref sc.B[e];
                    if (b.Dyn && !node.FixedAxle) { row.T0.Be = e; row.T0.Ang = -ax; b.InSolver = true; }
                    else row.Const = -Vector3D.Dot(b.W, ax);
                }
                sc.Lambda[r] = 0; r++;
            }
        }
        sc.RowCount = r;
    }

    private static double JV(Scratch sc, ref Row row)
    {
        double v = row.Const;
        v += TermV(sc, ref row.T0) + TermV(sc, ref row.T1);
        return v;
    }

    private static double TermV(Scratch sc, ref Term t)
    {
        double v = 0;
        if (t.Be >= 0) { ref var b = ref sc.B[t.Be]; v += Vector3D.Dot(t.Lin, b.V) + Vector3D.Dot(t.Ang, b.W); }
        if (t.Se >= 0) v += t.Sc * sc.Spin[t.Se].S;
        return v;
    }

    private void ApplyRow(Scratch sc, ref Row row, double scale)
    {
        ApplyTerm(sc, ref row.T0, scale); ApplyTerm(sc, ref row.T1, scale);
    }

    private void ApplyTerm(Scratch sc, ref Term t, double scale)
    {
        if (t.Be >= 0)
        {
            ref var b = ref sc.B[t.Be];
            b.V = b.V + t.Lin * (b.InvM * scale); b.W = b.W + b.Inertia(Host.Planar, t.Ang) * scale;
        }
        if (t.Se >= 0) sc.Spin[t.Se].S += sc.Spin[t.Se].InvI * t.Sc * scale;
    }

    private double Kdot(Scratch sc, ref Row a, ref Row b) => TermDot(sc, ref a.T0, ref b.T0) + TermDot(sc, ref a.T0, ref b.T1) + TermDot(sc, ref a.T1, ref b.T0) + TermDot(sc, ref a.T1, ref b.T1);

    private double TermDot(Scratch sc, ref Term a, ref Term b)
    {
        double k = 0;
        if (a.Be >= 0 && a.Be == b.Be)
        {
            ref var e = ref sc.B[a.Be];
            k += Vector3D.Dot(a.Lin, b.Lin) * e.InvM + Vector3D.Dot(a.Ang, e.Inertia(Host.Planar, b.Ang));
        }
        if (a.Se >= 0 && a.Se == b.Se) k += a.Sc * b.Sc * sc.Spin[a.Se].InvI;
        return k;
    }

    // ---------------------------------------------------------------- runs

    private static bool Merges(Scratch sc, CableData m, int node)
    {
        var nd = m.Nodes[node]; int n = m.Nodes.Length;
        if (!nd.IsWheel || nd.Kind != CableNodeKind.Wheel || nd.NoSlip || nd.SlipState == 0) return false;
        if (!m.Closed && (node == 0 || node == n - 1)) return false;
        return sc.Valid[(node + n - 1) % n] && sc.Valid[node];
    }

    private void BuildRuns(Scratch sc, CableData m)
    {
        int n = m.Nodes.Length, spans = m.Spans;
        int first = 0;
        if (m.Closed)
        {
            first = -1;
            for (int i = 0; i < n; i++) if (!Merges(sc, m, i)) { first = i; break; }
            if (first < 0) { m.Nodes[0].SlipState = 0; first = 0; }
        }
        sc.Runs = 0; int processed = 0, k = first;
        while (processed < spans)
        {
            int start = k, count = 1; sc.RunRatio[start] = 1;
            while (processed + count < spans && Merges(sc, m, (start + count) % n))
            {
                int w = (start + count) % n; var node = m.Nodes[w];
                double e = sc.E[w]; double ratio = node.SlipState == 2 ? 1 / e : e;
                sc.RunRatio[(start + count) % spans] = sc.RunRatio[(start + count - 1) % spans] * ratio;
                count++;
            }
            sc.RunStart[sc.Runs] = start; sc.RunCount[sc.Runs] = count;
            // Bring the run's multipliers into the ratios of its wheels.
            double k0 = 0;
            if (sc.Valid[start]) k0 = sc.Lambda[start];
            double kk = 0, ka = 0;
            for (int a = 0; a < count; a++)
            {
                int sa = (start + a) % spans; if (!sc.Valid[sa]) continue;
                double want = sc.RunRatio[sa] * k0, have = sc.Lambda[sa];
                if (want != have) { ApplyRow(sc, ref sc.Rows[sa], -(want - have)); sc.Lambda[sa] = want; }
                for (int b2 = 0; b2 < count; b2++) { int sb = (start + b2) % spans; double kd = sc.RunRatio[sb] * Kdot(sc, ref sc.Rows[sa], ref sc.Rows[sb]); kk += kd; ka += Math.Abs(kd); }
            }
            sc.RunK[sc.Runs] = kk; sc.RunScale[sc.Runs] = ka;
            sc.Runs++; processed += count; k = (start + count) % spans;
        }
    }

    private void UpdateRun(Scratch sc, CableData m, int run, double dt)
    {
        int spans = m.Spans, start = sc.RunStart[run], count = sc.RunCount[run];
        if (!sc.Valid[start]) return;
        double rho = 0, floor = 0, pre = (double)m.Preload * dt;
        for (int a = 0; a < count; a++)
        {
            int sa = (start + a) % spans;
            rho += JV(sc, ref sc.Rows[sa]) + sc.Rows[sa].Bias;
            double f = pre / sc.RunRatio[sa]; if (f > floor) floor = f;
        }
        double l0 = sc.Lambda[start], nl;
        // A run whose tension changes nothing it can feel (a belt slipping over a wheel that cannot move) has no determined tension level: a belt tensioner holds it at its preload.
        if (!(sc.RunK[run] > 1e-9 * sc.RunScale[run]) || !(sc.RunK[run] > 1e-14)) nl = floor;
        else nl = l0 + rho / sc.RunK[run];
        if (nl < floor) nl = floor;
        double delta = nl - l0;
        if (delta == 0) return;
        for (int a = 0; a < count; a++)
        {
            int sa = (start + a) % spans;
            ApplyRow(sc, ref sc.Rows[sa], -sc.RunRatio[sa] * delta); sc.Lambda[sa] += sc.RunRatio[sa] * delta;
        }
    }

    private void UpdateDrive(Scratch sc, CableData m, int r, double dt)
    {
        ref var row = ref sc.Rows[r]; var node = m.Nodes[row.Node];
        double k = Kdot(sc, ref row, ref row);
        if (!(k > 1e-14)) return;
        double target = row.Kind == 1 ? node.MotorSpeed : 0, limit = (row.Kind == 1 ? node.MotorTorque : node.BrakeTorque) * dt;
        double rho = JV(sc, ref row) - target, old = sc.Lambda[r], nj = old - rho / k;
        if (nj > limit) nj = limit; else if (nj < -limit) nj = -limit;
        double d = nj - old; if (d == 0) return;
        ApplyRow(sc, ref row, d); sc.Lambda[r] = nj;
    }

    private static bool Transitions(Scratch sc, CableData m)
    {
        int n = m.Nodes.Length, spans = m.Spans; bool changed = false;
        for (int run = 0; run < sc.Runs; run++)
        {
            int start = sc.RunStart[run], count = sc.RunCount[run]; double sigma = 0;
            for (int a = 0; a < count - 1; a++)
            {
                int sa = (start + a) % spans;
                sigma -= JV(sc, ref sc.Rows[sa]) + sc.Rows[sa].Bias;
                var node = m.Nodes[(start + a + 1) % n];
                if (node.Friction > 0 && ((node.SlipState == 1 && sigma < -1e-5) || (node.SlipState == 2 && sigma > 1e-5))) { node.SlipState = 0; changed = true; }
            }
        }
        int merged = 0;
        for (int i = 0; i < n; i++) if (Merges(sc, m, i)) merged++;
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i];
            if (Merges(sc, m, i) || !node.IsWheel || node.Kind != CableNodeKind.Wheel || node.NoSlip || !(node.Friction > 0)) continue;
            if (!m.Closed && (i == 0 || i == n - 1)) continue;
            int pin = (i + n - 1) % n, pout = i;
            if (!sc.Valid[pin] || !sc.Valid[pout]) continue;
            double e = sc.E[i], lp = sc.Lambda[pin], lk = sc.Lambda[pout], tol = 1e-9 + 1e-6 * (lp + lk);
            byte next = 0;
            if (lk > e * lp + tol) next = 1; else if (lp > e * lk + tol) next = 2;
            if (next == 0) continue;
            if (m.Closed && merged + 1 >= n) continue;
            node.SlipState = next; merged++; changed = true;
        }
        return changed;
    }

    private void SolveRows(Scratch sc, CableData m, float dt)
    {
        int spans = m.Spans, iters = m.Iterations;
        for (int round = 0; round < 6; round++)
        {
            BuildRuns(sc, m);
            int count = round == 0 ? iters : Math.Max(4, iters / 2);
            for (int it = 0; it < count; it++)
            {
                for (int run = 0; run < sc.Runs; run++) UpdateRun(sc, m, run, dt);
                for (int r = spans; r < sc.RowCount; r++) UpdateDrive(sc, m, r, dt);
            }
            if (!Transitions(sc, m)) break;
        }
        BuildRuns(sc, m);
    }

    // ---------------------------------------------------------------- results

    private static void Load(Scratch sc, CableData m, int node, Vector3D pt, Vector3D dir, double tension)
    {
        int e = sc.NodeEnt[node]; if (e < 0) return;
        ref var b = ref sc.B[e]; if (!b.Dyn) return;
        var f = dir * -tension;
        var torque = Vector3D.Cross(pt - b.X, f);
        var nd = m.Nodes[node];
        if (nd.IsWheel && nd.Mode == WheelSpin.Rotor)
        {
            var ax = sc.AxisW[node]; double c = Vector3D.Dot(dir, Vector3D.Cross(ax, pt - sc.Center[node]));
            torque = torque + ax * (tension * c);
        }
        b.Force = b.Force + f; b.Torque = b.Torque + torque;
    }

    private void Finish(Scratch sc, CableData m, float dt)
    {
        int n = m.Nodes.Length, spans = m.Spans;
        // Material flow through slipping wheels, with the run's flow summing to exactly zero so the rope's length is conserved.
        for (int run = 0; run < sc.Runs; run++)
        {
            int start = sc.RunStart[run], count = sc.RunCount[run];
            if (count < 2) continue;
            // Rope slides through the run's wheels, so each span but the last takes the material it needs to stay exactly taut (its rate of change plus its whole stretch in one step), and the last span
            // gives back what they took: the run's total never changes.
            double sum = 0;
            for (int a = 0; a < count - 1; a++)
            {
                int sa = (start + a) % spans; double flow = JV(sc, ref sc.Rows[sa]) + (m.Geometric[sa] - m.Material[sa]) / dt;
                m.Material[sa] += flow * dt; sum += flow;
            }
            m.Material[(start + count - 1) % spans] -= sum * dt;
        }
        if (m.Chain && m.Closed)
        {
            // A chain never slips on its teeth, so the loaded span's stretch can only be the rotation integrator's rounding of the two sprockets' angles. Handing it to the neighbouring span keeps the chain's
            // total length exactly and lets the speed ratio stay the tooth ratio instead of being bent to hide the drift.
            for (int k = 0; k < spans; k++)
            {
                if (!sc.Valid[k] || !(sc.Lambda[k] > 0)) continue;
                double c = m.Geometric[k] - m.Material[k];
                if (Math.Abs(c) > 1e-3) continue;
                m.Material[k] += c; m.Material[(k + 1) % spans] -= c;
            }
        }
        double tmax = 0;
        for (int k = 0; k < spans; k++)
        {
            double l = sc.Valid[k] ? sc.Lambda[k] : 0; m.Impulse[k] = l;
            double t = l / dt; if (t > tmax) tmax = t;
        }
        if (tmax > m.BreakForce) { Break(m); return; }
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i]; if (!node.IsWheel) continue;
            if (node.Mode == WheelSpin.Rotor) node.Spin = (float)sc.Spin[i].S;
            node.MotorImpulse = 0; node.BrakeImpulse = 0;
        }
        for (int r = spans; r < sc.RowCount; r++)
        {
            ref var row = ref sc.Rows[r]; var node = m.Nodes[row.Node];
            if (row.Kind == 1) node.MotorImpulse = (float)sc.Lambda[r]; else node.BrakeImpulse = (float)sc.Lambda[r];
            // The motor's reaction on the body that carries the wheel.
            int e = sc.NodeEnt[row.Node]; if (e >= 0 && sc.B[e].Dyn) sc.B[e].Torque = sc.B[e].Torque - sc.AxisW[row.Node] * (sc.Lambda[r] / dt);
        }
        m.LastDt = dt; m.AdvanceDt = dt;
        for (int k = 0; k < spans; k++)
        {
            if (!sc.Valid[k] || !(sc.Lambda[k] > 0)) continue;
            double t = sc.Lambda[k] / dt; int j = (k + 1) % n;
            Load(sc, m, k, sc.P[k], -sc.Dir[k], t); Load(sc, m, j, sc.Q[k], sc.Dir[k], t);
        }
        ApplyLoads(sc, dt);
    }

    private void ApplyLoads(Scratch sc, float dt)
    {
        var core = Host.CableCore; var bodies = core.Bodies;
        double threshold = core.Options.SleepThreshold;
        var g = Host.Gravity; bool anyAwake = false;
        for (int e = 0; e < sc.NB; e++) if (sc.B[e].Dyn && sc.B[e].Awake) anyAwake = true;
        for (int pass = 0; pass < 2; pass++)
        {
            for (int e = 0; e < sc.NB; e++)
            {
                ref var b = ref sc.B[e];
                if (!b.Dyn || b.Awake) continue;
                bool wake;
                if (pass == 0 && b.InSolver)
                {
                    var dv = b.Force * (b.InvM * (double)dt) + new Vector3D(g.X, g.Y, Host.Planar ? 0 : g.Z) * ((double)b.GravityScale * dt);
                    var dw = b.Inertia(Host.Planar, b.Torque) * (double)dt;
                    wake = dv.Length() > threshold || dw.Length() > threshold;
                }
                else if (pass == 1 && !b.InSolver) wake = anyAwake && (b.Force.LengthSquared() != 0 || b.Torque.LengthSquared() != 0);
                else continue;
                if (!wake) continue;
                core.WakeIsland(bodies.Island[b.Slot]); bodies.SleepTime[b.Slot] = 0f; b.Awake = true; anyAwake = true;
            }
        }
        for (int e = 0; e < sc.NB; e++)
        {
            ref var b = ref sc.B[e];
            if (!b.Dyn || !b.Awake) continue;
            if (b.Force.LengthSquared() != 0 || b.Torque.LengthSquared() != 0)
                Host.AddLoad(b.Slot, new Vector3F((float)b.Force.X, (float)b.Force.Y, (float)b.Force.Z), new Vector3F((float)b.Torque.X, (float)b.Torque.Y, (float)b.Torque.Z));
            core.KeepJointSchedule(b.Slot);
        }
    }
}
