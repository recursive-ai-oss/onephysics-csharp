using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Cables;

// The pulley-system solver. One step per world step, before the rigid solver, like the vehicle stage: it reads the bodies' velocities, predicts them one step ahead under gravity, and solves for the
// rope tensions that keep every span at its material length (sequential impulses over a velocity-level model that includes each wheel's own spin), then hands the resulting forces to the
// rigid step as constant loads. Because the loads are applied in every substep, a load hanging in equilibrium stays exactly at rest.
internal sealed partial class CableStore
{
    private const double Beta = 0.25, Deadband = 1e-5;

    private struct Frame
    {
        internal Vector3D C0, C1, C2;
        internal Vector3D Rotate(Vector3D v) => C0 * v.X + C1 * v.Y + C2 * v.Z;
        internal Vector3D RotateT(Vector3D v) => new(Vector3D.Dot(C0, v), Vector3D.Dot(C1, v), Vector3D.Dot(C2, v));
        internal static Frame Identity => new() { C0 = new(1, 0, 0), C1 = new(0, 1, 0), C2 = new(0, 0, 1) };
    }

    private struct Ent
    {
        internal int Slot;
        internal bool Dyn, Awake, InSolver;
        internal Vector3D X, V, W, Force, Torque;
        internal Frame R;
        internal double InvM, I0, I1, I2, I3, I4, I5;
        internal float GravityScale;
        internal Vector3D Inertia(bool planar, Vector3D v)
        {
            if (InvM == 0) return default;
            if (planar) return new(0, 0, v.Z * I0);
            var t = R.RotateT(v);
            return R.Rotate(new(I0 * t.X + I1 * t.Y + I2 * t.Z, I1 * t.X + I3 * t.Y + I4 * t.Z, I2 * t.X + I4 * t.Y + I5 * t.Z));
        }
    }

    private struct SpinEnt { internal bool Active; internal double S, InvI; internal int Carrier; }
    private struct Term { internal int Be, Se; internal Vector3D Lin, Ang; internal double Sc; }
    private struct Row
    {
        internal Term T0, T1;
        internal double Const, Bias;
        internal bool Valid;
        internal int Node;       // for motor rows: the wheel; -1 for spans
        internal int Kind;       // 0 span, 1 motor, 2 brake
    }

    private sealed class Scratch
    {
        internal Ent[] B = new Ent[16]; internal int NB;
        internal SpinEnt[] Spin = new SpinEnt[16];
        internal int[] NodeEnt = new int[16];
        internal Vector3D[] Center = new Vector3D[16], AxisW = new Vector3D[16], E1 = new Vector3D[16], E2 = new Vector3D[16], P = new Vector3D[16], Q = new Vector3D[16], Dir = new Vector3D[16];
        internal double[] Len = new double[16], RunRatio = new double[16], Lambda = new double[48], Bias = new double[16], Theta = new double[16], E = new double[16], Stretch = new double[16];
        internal bool[] Valid = new bool[16];
        internal Row[] Rows = new Row[48];
        internal int[] RunStart = new int[16], RunCount = new int[16];
        internal double[] RunK = new double[16], RunScale = new double[16];
        internal int Runs;
        internal int RowCount;
        internal void Ensure(int nodes)
        {
            if (Spin.Length >= nodes + 1) return;
            int n = Math.Max(nodes + 1, Spin.Length * 2);
            Spin = new SpinEnt[n]; NodeEnt = new int[n]; Center = new Vector3D[n]; AxisW = new Vector3D[n]; E1 = new Vector3D[n]; E2 = new Vector3D[n]; P = new Vector3D[n]; Q = new Vector3D[n]; Dir = new Vector3D[n];
            Len = new double[n]; RunRatio = new double[n]; Lambda = new double[3 * n]; Bias = new double[n]; Theta = new double[n]; E = new double[n]; Stretch = new double[n];
            Valid = new bool[n]; Rows = new Row[3 * n]; RunStart = new int[n]; RunCount = new int[n]; RunK = new double[n]; RunScale = new double[n];
        }
    }

    private Scratch? _scratch;
    private Scratch Sc => _scratch ??= new Scratch();

    // ---------------------------------------------------------------- bodies and node frames

    private int EntOf(Scratch sc, int slot)
    {
        if (slot < 0) return -1;
        for (int i = 0; i < sc.NB; i++) if (sc.B[i].Slot == slot) return i;
        if (sc.NB == sc.B.Length) Array.Resize(ref sc.B, sc.NB * 2);
        var bodies = Host.CableCore.Bodies;
        ref var e = ref sc.B[sc.NB];
        e = default; e.Slot = slot;
        var p = bodies.PositionOf(slot);
        e.X = new(p[0], p[1], Host.Planar ? 0 : p[2]);
        var r = bodies.RotationOf(slot);
        if (Host.Planar) e.R = new Frame { C0 = new(r[0], r[1], 0), C1 = new(-r[1], r[0], 0), C2 = new(0, 0, 1) };
        else
        {
            double x = r[0], y = r[1], z = r[2], w = r[3];
            e.R = new Frame
            {
                C0 = new(1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)),
                C1 = new(2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)),
                C2 = new(2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)),
            };
        }
        var v = bodies.LinearVelocityOf(slot); var a = bodies.AngularVelocityOf(slot);
        e.V = new(v[0], v[1], Host.Planar ? 0 : v[2]);
        e.W = Host.Planar ? new(0, 0, a[0]) : new(a[0], a[1], a[2]);
        e.Dyn = bodies.IsDynamic(slot); e.Awake = bodies.IsAwake(slot);
        e.GravityScale = Host.GravityScaleOf(slot);
        if (e.Dyn)
        {
            e.InvM = bodies.InverseMass[slot];
            var ii = bodies.InverseInertiaOf(slot);
            e.I0 = ii[0]; if (!Host.Planar) { e.I1 = ii[1]; e.I2 = ii[2]; e.I3 = ii[3]; e.I4 = ii[4]; e.I5 = ii[5]; }
        }
        return sc.NB++;
    }

    private Vector3D WorldPoint(Scratch sc, CableNode n, int ent)
    {
        if (ent < 0) return n.Local;
        ref var b = ref sc.B[ent];
        var lc = Host.LocalCenter(b.Slot);
        return b.X + b.R.Rotate(new(n.Local.X - lc.X, n.Local.Y - lc.Y, n.Local.Z - lc.Z));
    }

    private bool LoadFrames(Scratch sc, CableData m)
    {
        var bodies = Host.CableCore.Bodies;
        int n = m.Nodes.Length;
        sc.Ensure(n); sc.NB = 0;
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i];
            if (node.Body >= 0 && (!bodies.Handles.IsAlive(node.Body, node.BodyGeneration) || !bodies.IsLive(node.Body))) return false;
            int e = EntOf(sc, node.Body); sc.NodeEnt[i] = e;
            sc.Center[i] = WorldPoint(sc, node, e);
            if (node.IsWheel)
            {
                Vector3D a = new(node.Axis.X, node.Axis.Y, node.Axis.Z), r1 = new(node.Ref1.X, node.Ref1.Y, node.Ref1.Z), r2 = new(node.Ref2.X, node.Ref2.Y, node.Ref2.Z);
                if (e >= 0) { var f = sc.B[e].R; a = f.Rotate(a); r1 = f.Rotate(r1); r2 = f.Rotate(r2); }
                sc.AxisW[i] = a; sc.E1[i] = r1; sc.E2[i] = r2;
            }
        }
        return true;
    }

    // ---------------------------------------------------------------- geometry

    private static bool ComputeSpans(Scratch sc, CableData m)
    {
        int spans = m.Spans, n = m.Nodes.Length; bool all = true;
        for (int k = 0; k < spans; k++)
        {
            int i = k, j = (k + 1) % n; var a = m.Nodes[i]; var b = m.Nodes[j];
            bool ok;
            Vector3D p = default, q = default;
            if (!a.IsWheel && !b.IsWheel) { p = sc.Center[i]; q = sc.Center[j]; ok = true; }
            else if (!a.IsWheel) { p = sc.Center[i]; ok = CableGeometry.PointToWheel(p, sc.Center[j], sc.AxisW[j], b.Radius, b.Side, true, out q); }
            else if (!b.IsWheel) { q = sc.Center[j]; ok = CableGeometry.PointToWheel(q, sc.Center[i], sc.AxisW[i], a.Radius, a.Side, false, out p); }
            else ok = CableGeometry.WheelToWheel(sc.Center[i], sc.AxisW[i], a.Radius, a.Side, sc.Center[j], sc.AxisW[j], b.Radius, b.Side, out p, out q);
            double len = ok ? (q - p).Length() : 0;
            if (!(len > 1e-9)) ok = false;
            sc.Valid[k] = ok; all &= ok;
            if (ok) { sc.P[k] = p; sc.Q[k] = q; sc.Len[k] = len; sc.Dir[k] = (q - p) / len; }
        }
        return all;
    }

    private static double Unwrap(double raw, double previous) => raw + 2 * Math.PI * Math.Floor((previous - raw) / (2 * Math.PI) + 0.5);

    private static double RawAngle(Scratch sc, int node, Vector3D point)
    {
        var u = point - sc.Center[node];
        return Math64.Atan2(Vector3D.Dot(u, sc.E2[node]), Vector3D.Dot(u, sc.E1[node]));
    }

    private static void UpdateAngles(Scratch sc, CableData m)
    {
        int n = m.Nodes.Length, spans = m.Spans;
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i]; if (!node.IsWheel) continue;
            int outSpan = (!m.Closed && i == n - 1) ? -1 : i, inSpan = (!m.Closed && i == 0) ? -1 : (i + n - 1) % n;
            if (outSpan >= 0 && sc.Valid[outSpan]) node.AlphaOut = Unwrap(RawAngle(sc, i, sc.P[outSpan]), node.AlphaOut);
            if (inSpan >= 0 && sc.Valid[inSpan]) node.AlphaIn = Unwrap(RawAngle(sc, i, sc.Q[inSpan]), node.AlphaIn);
        }
    }

    private static double WheelAngle(CableNode node, double carrierFrameAngle) => carrierFrameAngle - (node.Mode == WheelSpin.Rotor ? node.RotorAngle : 0);

    /// <summary>The span's material length as the geometry sees it: the straight part plus the arcs from the wheels' reference points to the tangent points.</summary>
    private static double SurfaceLength(CableData m, Scratch sc, int k)
    {
        int n = m.Nodes.Length, i = k, j = (k + 1) % n;
        var a = m.Nodes[i]; var b = m.Nodes[j];
        double h = sc.Len[k];
        if (a.IsWheel) h += a.Side * (double)a.Radius * (WheelAngle(a, a.AlphaOut) - a.Reference);
        if (b.IsWheel) h += b.Side * (double)b.Radius * (b.Reference - WheelAngle(b, b.AlphaIn));
        return h;
    }

    private static double Exponent(CableData m, Scratch sc, int i)
    {
        var node = m.Nodes[i];
        double theta = node.Side * (node.AlphaOut - node.AlphaIn);
        if (!(theta > 0)) theta = 0;
        sc.Theta[i] = theta;
        return Math64.Exp((double)node.Friction * theta);
    }

    // ---------------------------------------------------------------- creation

    /// <summary>Computes the initial wrap angles, reference points and material lengths of a freshly built system. Throws ArgumentException when the geometry has no taut rope (a point inside a wheel, overlapping wheels).</summary>
    internal void Initialize(CableData m, double length, double slack, int[] turns)
    {
        var sc = Sc;
        if (!LoadFrames(sc, m)) throw new ArgumentException("A node's body is not alive.");
        if (!ComputeSpans(sc, m)) throw new ArgumentException("The rope has no tangent path: a rope end lies inside a wheel, or two wheels overlap too much for the rope to leave one and reach the other.");
        int n = m.Nodes.Length, spans = m.Spans;
        for (int i = 0; i < n; i++)
        {
            var node = m.Nodes[i]; if (!node.IsWheel) continue;
            bool hasIn = m.Closed || i > 0, hasOut = m.Closed || i < n - 1;
            double aIn = hasIn ? RawAngle(sc, i, sc.Q[(i + n - 1) % n]) : 0, aOut = hasOut ? RawAngle(sc, i, sc.P[i]) : 0;
            node.RotorAngle = 0; node.Spin = 0;
            if (hasIn && hasOut)
            {
                double theta = node.Side * (aOut - aIn);
                theta -= 2 * Math.PI * Math.Floor(theta / (2 * Math.PI));
                if (theta > 2 * Math.PI - 1e-9) theta -= 2 * Math.PI;
                node.AlphaIn = aIn; node.AlphaOut = aIn + node.Side * (theta + 2 * Math.PI * turns[i]);
                node.Reference = aIn;
            }
            else if (hasOut) { node.AlphaOut = aOut; node.AlphaIn = aOut; node.Reference = aOut; }
            else { node.AlphaIn = aIn; node.AlphaOut = aIn; node.Reference = aIn; }
            node.SlipState = (byte)(node.IsWheel && node.Kind == CableNodeKind.Wheel && !node.NoSlip && node.Friction == 0 && hasIn && hasOut ? 1 : 0);
        }
        var material = new double[spans]; double total = 0;
        for (int k = 0; k < spans; k++) { material[k] = SurfaceLength(m, sc, k); total += material[k]; }
        if (length <= 0 && slack > 0) length = total + slack;
        if (length > 0)
        {
            if (length < total * (1 - 1e-9)) throw new ArgumentOutOfRangeException(nameof(length), length, $"Rope length {length} is shorter than the {total} the path needs.");
            double scale = length / total; for (int k = 0; k < spans; k++) material[k] *= scale;
        }
        m.Material = material; m.Impulse = new double[spans]; m.Stretch = new double[spans]; m.Geometric = new double[spans]; m.Free = new double[spans]; m.Wrap = new double[n];
        for (int k = 0; k < spans; k++) { m.Geometric[k] = SurfaceLength(m, sc, k); m.Free[k] = sc.Len[k]; }
        RecordWrap(m, sc);
    }

    private static void RecordWrap(CableData m, Scratch sc)
    {
        for (int i = 0; i < m.Nodes.Length; i++)
        {
            var node = m.Nodes[i];
            if (!node.IsWheel) { m.Wrap[i] = 0; continue; }
            double theta = node.Side * (node.AlphaOut - node.AlphaIn); m.Wrap[i] = theta > 0 ? theta : 0;
        }
    }

    // ---------------------------------------------------------------- the step

    internal void Step(float dt)
    {
        for (int s = 0; s < Data.Length; s++)
            if (Data[s] is { } m) StepOne(m, dt);
    }

    private static bool Break(CableData m)
    {
        m.Broken = true; Array.Clear(m.Impulse); Array.Clear(m.Stretch);
        foreach (var n in m.Nodes) { n.MotorImpulse = 0; n.BrakeImpulse = 0; }
        return false;
    }

    private void StepOne(CableData m, float dt)
    {
        if (m.Broken) return;
        var sc = Sc;
        if (!LoadFrames(sc, m)) { Break(m); return; }
        int n = m.Nodes.Length, spans = m.Spans;

        // Is anything moving or able to move? A system whose bodies all sleep and whose wheels stand still costs nothing and changes nothing.
        bool active = false;
        for (int e = 0; e < sc.NB && !active; e++)
        {
            ref var b = ref sc.B[e];
            if (b.Dyn ? b.Awake : (b.V.LengthSquared() != 0 || b.W.LengthSquared() != 0)) active = true;
        }
        for (int i = 0; i < n && !active; i++)
        {
            var node = m.Nodes[i];
            if (node.IsWheel && node.Mode == WheelSpin.Rotor && (node.Spin != 0 || (node.MotorTorque > 0 && node.MotorSpeed != 0))) active = true;
        }
        if (!active) { m.AdvanceDt = 0; return; }

        // Rotor angles advance with the spin the last solve found, relative to the carrier's turning about the axle.
        if (m.AdvanceDt > 0)
        {
            for (int i = 0; i < n; i++)
            {
                var node = m.Nodes[i];
                if (!node.IsWheel || node.Mode != WheelSpin.Rotor) continue;
                int e = sc.NodeEnt[i]; double carrier = e >= 0 ? Vector3D.Dot(sc.B[e].W, sc.AxisW[i]) : 0;
                node.RotorAngle += (node.Spin - carrier) * (double)m.AdvanceDt;
            }
        }

        bool allValid = ComputeSpans(sc, m);
        UpdateAngles(sc, m);
        double h = dt;
        for (int k = 0; k < spans; k++)
        {
            if (!sc.Valid[k]) { m.Impulse[k] = 0; m.Stretch[k] = 0; continue; }
            m.Geometric[k] = SurfaceLength(m, sc, k); m.Free[k] = sc.Len[k];
        }
        RecordWrap(m, sc);

        // Wheels' friction factors.
        for (int i = 0; i < n; i++) { var node = m.Nodes[i]; sc.Theta[i] = 0; sc.E[i] = node.IsWheel && node.Kind == CableNodeKind.Wheel ? Exponent(m, sc, i) : 1; }

        // Predicted velocities: gravity for every body that can move.
        var g = Host.Gravity;
        for (int e = 0; e < sc.NB; e++)
        {
            ref var b = ref sc.B[e];
            if (b.Dyn)
            {
                // What the rigid step will add on its own: gravity, and any force or torque already queued (user forces, fields).
                Host.PendingLoad(b.Slot, out var pf, out var pt);
                b.V = b.V + new Vector3D(g.X, g.Y, Host.Planar ? 0 : g.Z) * ((double)b.GravityScale * h) + new Vector3D(pf.X, pf.Y, pf.Z) * (b.InvM * h);
                b.W = b.W + b.Inertia(Host.Planar, new Vector3D(pt.X, pt.Y, pt.Z)) * h;
            }
            b.Force = default; b.Torque = default;
        }

        BuildRows(sc, m, dt);
        // Warm start.
        for (int r = 0; r < sc.RowCount; r++)
        {
            ref var row = ref sc.Rows[r];
            if (!row.Valid) continue;
            if (row.Kind == 0) { sc.Lambda[r] = m.Impulse[r]; ApplyRow(sc, ref row, -sc.Lambda[r]); }
            else { double j = row.Kind == 1 ? m.Nodes[row.Node].MotorImpulse : m.Nodes[row.Node].BrakeImpulse; sc.Lambda[r] = j; ApplyRow(sc, ref row, j); }
        }
        SolveRows(sc, m, dt);
        Finish(sc, m, dt);
    }
}
