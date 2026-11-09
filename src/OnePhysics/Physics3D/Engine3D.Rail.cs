using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A switch (points) on a track: at chainage <see cref="S"/> a branch track leaves; while <see cref="Diverging"/> is set, wheelsets running forward past the point follow the branch.</summary>
internal struct SwitchState
{
    public double S;
    public int Branch, BranchGen;
    public bool Diverging;
}

/// <summary>A track: its definition (copied), the evaluator built from it, the static bodies that make its collision geometry and where it branches from.</summary>
internal sealed class TrackState
{
    public TrackDef Def = new();
    public TrackGeometry Geo = null!;
    public int Ballast = -1, BallastGen, Rails = -1, RailsGen;
    public int Parent = -1, ParentGen;
    public double ParentS;
    public SwitchState[] Switches = Array.Empty<SwitchState>();

    public TrackState CloneForFork()
    {
        var c = (TrackState)MemberwiseClone();
        c.Def = Def.Clone();
        c.Switches = (SwitchState[])Switches.Clone();
        return c;   // the geometry is immutable and shared
    }
}

/// <summary>A rail vehicle: a car with bogies and wheelsets, where each wheelset is on its track at a chainage, spinning, with the forces of its last contacts.</summary>
internal sealed class RailState
{
    public RailDef Def = new();
    public int Car = -1, CarGen;
    public int[] Bogie = Array.Empty<int>(), BogieGen = Array.Empty<int>();
    public int[] Ws = Array.Empty<int>(), WsGen = Array.Empty<int>(), Trk = Array.Empty<int>(), TrkGen = Array.Empty<int>();
    public double[] S = Array.Empty<double>();
    public float[] Spin = Array.Empty<float>();
    // Last step's contact readback per wheelset: normal and creep forces (left wheel, right wheel), flange normal forces, the larger lateral-to-vertical force ratio, contact flags.
    public float[] NormalL = Array.Empty<float>(), NormalR = Array.Empty<float>(), FxL = Array.Empty<float>(), FxR = Array.Empty<float>(), FyL = Array.Empty<float>(), FyR = Array.Empty<float>();
    public float[] FlangeL = Array.Empty<float>(), FlangeR = Array.Empty<float>(), Climb = Array.Empty<float>();
    // All wheel-rail force on each wheel (tread and flange, normal and creep) along the track's up and right axes: the wheel's vertical load Q and lateral force Y.
    public float[] QL = Array.Empty<float>(), QR = Array.Empty<float>(), YL = Array.Empty<float>(), YR = Array.Empty<float>();
    public byte[] Flags = Array.Empty<byte>();
    public float Throttle, Brake;
    // Couplers: index 0 the front (+Z) end, 1 the rear end.
    public int[] CoupleVeh = { -1, -1 }, CoupleGen = new int[2], CoupleEnd = new int[2];
    public byte[] Broken = new byte[2];
    public float[] CouplerForce = new float[2];

    public int Wheelsets => Ws.Length;

    public void Allocate(int n)
    {
        Ws = new int[n]; WsGen = new int[n]; Trk = new int[n]; TrkGen = new int[n]; S = new double[n]; Spin = new float[n];
        NormalL = new float[n]; NormalR = new float[n]; FxL = new float[n]; FxR = new float[n]; FyL = new float[n]; FyR = new float[n];
        FlangeL = new float[n]; FlangeR = new float[n]; Climb = new float[n]; Flags = new byte[n]; QL = new float[n]; QR = new float[n]; YL = new float[n]; YR = new float[n];
        Array.Fill(Trk, -1);
    }

    public RailState CloneForFork()
    {
        var c = (RailState)MemberwiseClone();
        c.Def = Def.Clone();
        c.Bogie = (int[])Bogie.Clone(); c.BogieGen = (int[])BogieGen.Clone(); c.Ws = (int[])Ws.Clone(); c.WsGen = (int[])WsGen.Clone(); c.Trk = (int[])Trk.Clone(); c.TrkGen = (int[])TrkGen.Clone();
        c.S = (double[])S.Clone(); c.Spin = (float[])Spin.Clone();
        c.NormalL = (float[])NormalL.Clone(); c.NormalR = (float[])NormalR.Clone(); c.FxL = (float[])FxL.Clone(); c.FxR = (float[])FxR.Clone(); c.FyL = (float[])FyL.Clone(); c.FyR = (float[])FyR.Clone();
        c.FlangeL = (float[])FlangeL.Clone(); c.FlangeR = (float[])FlangeR.Clone(); c.Climb = (float[])Climb.Clone(); c.Flags = (byte[])Flags.Clone(); c.QL = (float[])QL.Clone(); c.QR = (float[])QR.Clone(); c.YL = (float[])YL.Clone(); c.YR = (float[])YR.Clone();
        c.CoupleVeh = (int[])CoupleVeh.Clone(); c.CoupleGen = (int[])CoupleGen.Clone(); c.CoupleEnd = (int[])CoupleEnd.Clone(); c.Broken = (byte[])Broken.Clone(); c.CouplerForce = (float[])CouplerForce.Clone();
        return c;
    }
}

internal sealed partial class Engine3D
{
    /// <summary>Half the width of a rail head, m.</summary>
    internal const float RailHeadWidth = 0.07f;

    /// <summary>Slots of the tracks and of the rail vehicles (handles with generations, lowest free slot first).</summary>
    public HandleTable TrackHandles = new(), RailHandles = new();

    public TrackState?[] Tracks = new TrackState?[2];
    public RailState?[] Rails = new RailState?[2];

    public bool HasRail => TrackHandles.Count > 0 || RailHandles.Count > 0;

    public int CreateTrack(TrackState t)
    {
        int i = TrackHandles.Allocate();
        if (i >= Tracks.Length) { Array.Resize(ref Tracks, Math.Max(i + 1, Tracks.Length * 2)); }
        Tracks[i] = t;
        return i;
    }

    public int CreateRailVehicle(RailState r)
    {
        int i = RailHandles.Allocate();
        if (i >= Rails.Length) { Array.Resize(ref Rails, Math.Max(i + 1, Rails.Length * 2)); }
        Rails[i] = r;
        return i;
    }

    /// <summary>Registers a vehicle's wheelsets for the per-substep wheel-rail forces (call once its wheelset bodies are known).</summary>
    public void RegisterRailVehicle(int slot) => IndexWheelsets(slot, Rails[slot]!);

    public void DestroyRailVehicle(int i)
    {
        var r = Rails[i]!;
        for (int e = 0; e < 2; e++)
        {
            int o = r.CoupleVeh[e];
            if (o >= 0 && RailHandles.IsAlive(o, r.CoupleGen[e])) { var os = Rails[o]!; os.CoupleVeh[r.CoupleEnd[e]] = -1; os.CouplerForce[r.CoupleEnd[e]] = 0f; }
        }

        UnindexWheelsets(i, r);
        Rails[i] = null;
        RailHandles.Free(i);
    }

    public void DestroyTrack(int i)
    {
        Tracks[i] = null;
        TrackHandles.Free(i);
    }

    /// <summary>Called when a body is destroyed: a rail vehicle whose car (or only wheelset) it is goes with it; a track whose static bodies it is goes too. The other bodies stay.</summary>
    public void DropRailOfBody(int body)
    {
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            if (r.Car == body || (r.Car < 0 && r.Ws.Length > 0 && r.Ws[0] == body)) { DestroyRailVehicle(i); }
        }

        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            if (t.Ballast == body) { t.Ballast = -1; }
            if (t.Rails == body) { t.Rails = -1; }
        }
    }

    private struct RailContact { public int Side; public bool Flange; public Vector3F Arm, Normal; public float Load; }

    private readonly RailContact[] _rc = new RailContact[4];

    private float EffInvRail(int b, Vector3F arm, Vector3F u, Vector3F axle, float iAx)
    {
        var bodies = Core.Bodies;
        float m = bodies.InverseMass[b];
        if (!(m > 0f)) { return 0f; }
        var rxu = Vector3F.Cross(arm, u);
        float ax = Vector3F.Dot(rxu, axle);
        var t = rxu - axle * ax;
        return m + Vector3F.Dot(t, ApplyWorldInertia(b, t)) + ax * ax / iAx;
    }

    // An impulse at the contact: the body takes the linear part and the torque with the axle's component removed, which goes to the wheelset's spin instead (the body is the wheelset's non-spinning frame).
    private void ApplyRailImpulse(int b, Vector3F arm, Vector3F j, Vector3F axle, float iAx, ref float spin)
    {
        var bodies = Core.Bodies;
        float m = bodies.InverseMass[b];
        if (!(m > 0f)) { return; }
        var v = bodies.LinearVelocityOf(b);
        v[0] += m * j.X; v[1] += m * j.Y; v[2] += m * j.Z;
        var tq = Vector3F.Cross(arm, j);
        float ax = Vector3F.Dot(tq, axle);
        var dw = ApplyWorldInertia(b, tq - axle * ax);
        var w = bodies.AngularVelocityOf(b);
        w[0] += dw.X; w[1] += dw.Y; w[2] += dw.Z;
        spin += ax / iAx;
    }

    private Vector3F RailPointVelocity(int b, Vector3F arm, Vector3F axle, float spin)
    {
        var bodies = Core.Bodies;
        var v = bodies.LinearVelocityOf(b); var w = bodies.AngularVelocityOf(b);
        return new Vector3F(v[0], v[1], v[2]) + Vector3F.Cross(new Vector3F(w[0], w[1], w[2]), arm) + Vector3F.Cross(axle, arm) * spin;
    }

    // Kalker coefficient scale: G a b C, with G = 8.1e10 Pa and the contact patch a b = (c N^(1/3))^2 with c = 1.49e-4 (a 100 kN load gives a patch of 4.8e-5 m²), C11 = 4.1, C22 = 3.7, C23 = 1.5 for a/b near 1 (Kalker 1990, table of coefficients).
    private const float KalkerG = 8.1e10f, PatchC = 1.49e-4f, C11 = 4.12f, C22 = 3.72f, C23 = 1.47f;

    // Shen, Hedrick and Elkins saturation: the linear force F of magnitude u * mu N is scaled by 1 - u/3 + u^2/27 up to u = 3 and capped at mu N beyond.
    private static float Saturate(float u) => u < 3f ? 1f - u / 3f + u * u / 27f : 1f / u;

    private float SoftNormal(int b, Vector3F arm, Vector3F n, float pen, float load, float hertz, float zeta, float h, float gravityLead, Vector3F axle, float iAx, ref float spin, Vector3F vContact)
    {
        float invN = EffInvRail(b, arm, n, axle, iAx);
        float meffN = invN > 0f ? 1f / invN : 0f;
        float closing = -Vector3F.Dot(vContact - Opt.Gravity * (gravityLead * GravityScale[b]), n);
        float w = TwoPi * hertz;
        float k = load * w * w, c = 2f * zeta * load * w;
        float spring = h * k * pen;
        if (meffN > 0f) { spring = Math.Min(spring, meffN * pen / h); }
        float damp = h * c * closing / (1f + h * c * invN);
        float pn = spring + damp;
        if (pn < 0f) { pn = 0f; }
        if (pn > 0f) { ApplyRailImpulse(b, arm, n * pn, axle, iAx, ref spin); }
        return pn;
    }

    /// <summary>Projects a point on a track by three fixed-point iterations on the chainage, starting from <paramref name="s"/>.</summary>
    private static TrackFrame Project(TrackState tr, double px, double py, double pz, ref double s)
    {
        TrackFrame f = default;
        for (int it = 0; it < 3; it++)
        {
            f = tr.Geo.FrameAt(s);
            s += (px - f.Position.X) * f.Tangent.X + (py - f.Position.Y) * f.Tangent.Y + (pz - f.Position.Z) * f.Tangent.Z;
        }

        return tr.Geo.FrameAt(s);
    }

    /// <summary>
    /// Before the vehicles run, once per step: every rail vehicle in slot order runs its wheelsets (track following with the switches, wheel-rail normal, creep and flange forces, traction and brakes, gyroscopic
    /// coupling), then the couplers act between vehicles in slot order. All impulses are applied one after another from the velocities at that moment, so the result does not depend on the thread count.
    /// </summary>
    public void StepRail(float h)
    {
        if (RailHandles.Count == 0) { return; }
        for (int v = 0; v < RailHandles.HighWater; v++)
        {
            if (RailHandles.IsLive(v)) { StepRailVehicle(Rails[v]!, h); }
        }

        for (int v = 0; v < RailHandles.HighWater; v++)
        {
            if (RailHandles.IsLive(v)) { StepCouplers(v, Rails[v]!, h); }
        }
    }

    private bool RailLive(RailState r)
    {
        var bodies = Core.Bodies;
        if (r.Car >= 0 && !bodies.Handles.IsAlive(r.Car, r.CarGen)) { return false; }
        for (int k = 0; k < r.Ws.Length; k++) { if (!bodies.Handles.IsAlive(r.Ws[k], r.WsGen[k])) { return false; } }
        return true;
    }

    // Once per step, before the solver: wakes a sleeping vehicle that is driven, keeps its island on the world's schedule, and applies the drive and brake torques to the wheelsets' spin.
    private void StepRailVehicle(RailState r, float h)
    {
        if (!RailLive(r)) { return; }
        var bodies = Core.Bodies;
        int anchor = r.Car >= 0 ? r.Car : r.Ws[0];
        bool active = r.Throttle != 0f || r.Brake > 0f;
        if (!bodies.IsAwake(anchor))
        {
            if (!active) { return; }
            WakeBodyIsland(anchor);
        }
        else if (active) { bodies.SleepTime[anchor] = 0f; }

        Core.KeepJointSchedule(anchor);
        var def = r.Def;
        int n = r.Wheelsets;
        float rw = def.WheelRadius;
        int driven = 0;
        for (int k = 0; k < n; k++) { if ((def.DrivenWheelsets >> k & 1u) != 0u) { driven++; } }
        float iAx = def.WheelsetSpinInertia;
        for (int k = 0; k < n; k++)
        {
            float spin = r.Spin[k];
            float vSpeed = Math.Abs(spin) * rw;
            if ((def.DrivenWheelsets >> k & 1u) != 0u && r.Throttle != 0f && driven > 0)
            {
                float f = def.MaxTractionForce / driven;
                if (def.MaxPower > 0f) { f = Math.Min(f, def.MaxPower / driven / Math.Max(vSpeed, 1f)); }
                spin += h * r.Throttle * f * rw / iAx;
            }

            if (r.Brake > 0f && def.MaxBrakeForce > 0f)
            {
                float dwb = Math.Min(h * r.Brake * def.MaxBrakeForce / n * rw / iAx, Math.Abs(spin));
                spin = spin > 0f ? spin - dwb : spin + dwb;
            }

            r.Spin[k] = spin;
        }
    }

    /// <summary>Applies the wheel-rail forces of every rail wheelset in the list for one substep of length <paramref name="h"/>, after gravity and before the constraints: each wheelset touches only its own body and its own records, so chunks may run concurrently.</summary>
    internal void RailSubstep(ReadOnlySpan<int> list, float h)
    {
        for (int i = 0; i < list.Length; i++)
        {
            int b = list[i];
            if ((uint)b >= (uint)_wsOfBody.Length) { continue; }
            int code = _wsOfBody[b] - 1;
            if (code < 0) { continue; }
            int slot = code >> 3;
            if (!RailHandles.IsLive(slot)) { continue; }
            var r = Rails[slot]!;
            int k = code & 7;
            if (k < r.Wheelsets && r.Ws[k] == b) { WheelsetSubstep(r, k, h); }
        }
    }

    private int[] _wsOfBody = Array.Empty<int>();

    private void IndexWheelsets(int slot, RailState r)
    {
        for (int k = 0; k < r.Wheelsets; k++)
        {
            int b = r.Ws[k];
            if (b >= _wsOfBody.Length) { int old = _wsOfBody.Length; Array.Resize(ref _wsOfBody, Math.Max(b + 1, old * 2)); }
            _wsOfBody[b] = (slot << 3 | k) + 1;
        }
    }

    private void UnindexWheelsets(int slot, RailState r)
    {
        for (int k = 0; k < r.Wheelsets; k++) { if (r.Ws[k] < _wsOfBody.Length && _wsOfBody[r.Ws[k]] == (slot << 3 | k) + 1) { _wsOfBody[r.Ws[k]] = 0; } }
    }

    private void WheelsetSubstep(RailState r, int k, float h)
    {
        var bodies = Core.Bodies;
        int b = r.Ws[k];
        if (!bodies.Handles.IsAlive(b, r.WsGen[k]) || bodies.TypeOf(b) != BodyType.Dynamic) { return; }
        var def = r.Def;
        int n = r.Wheelsets;
        float rw = def.WheelRadius;
        float carried = (def.CarMass + (def.Bogies == 2 ? def.Bogies * def.BogieMass : 0f)) / n + def.WheelsetMass;
        float iAx = def.WheelsetSpinInertia;
        float gamma = def.Conicity;
        float sinA = Math32.Sin(def.FlangeAngle), cosA = Math32.Cos(def.FlangeAngle);
        float invRoot = 1f / float.Sqrt(1f + gamma * gamma);
        float gravityLead = 0f;   // gravity of this substep is already in the velocities
        Span<RailContact> rcs = stackalloc RailContact[4];   // at most a tread and a flange on each side
        {
            var q = RotationOfBody(b);
            var pp = bodies.PositionOf(b);
            double px = pp[0], py = pp[1], pz = pp[2];
            var axle = q.Rotate(new Vector3F(1f, 0f, 0f));
            float spin = r.Spin[k];

            r.QL[k] = 0f; r.QR[k] = 0f; r.YL[k] = 0f; r.YR[k] = 0f; r.NormalL[k] = 0f; r.NormalR[k] = 0f; r.FxL[k] = 0f; r.FxR[k] = 0f; r.FyL[k] = 0f; r.FyR[k] = 0f; r.FlangeL[k] = 0f; r.FlangeR[k] = 0f; r.Climb[k] = 0f; r.Flags[k] = 0;
            int ti = r.Trk[k];
            if (ti < 0 || !TrackHandles.IsAlive(ti, r.TrkGen[k])) { r.Spin[k] = spin; return; }
            var tr = Tracks[ti]!;
            double s = r.S[k], sPrev = s;
            var f0 = Project(tr, px, py, pz, ref s);

            // Points: forward through a diverging switch onto the branch, and a trailing run off the start of a branch back onto the parent.
            for (int w = 0; w < tr.Switches.Length; w++)
            {
                var sw = tr.Switches[w];
                if (sw.Diverging && sPrev < sw.S && s >= sw.S && sw.Branch >= 0 && TrackHandles.IsAlive(sw.Branch, sw.BranchGen))
                {
                    ti = sw.Branch; tr = Tracks[ti]!; s = Math.Max(0.0, s - sw.S); f0 = Project(tr, px, py, pz, ref s);
                    r.Trk[k] = ti; r.TrkGen[k] = sw.BranchGen;
                    break;
                }
            }

            if (s < 0.0 && tr.Parent >= 0 && TrackHandles.IsAlive(tr.Parent, tr.ParentGen))
            {
                double ps = tr.ParentS + s;
                ti = tr.Parent; tr = Tracks[ti]!; s = ps; f0 = Project(tr, px, py, pz, ref s);
                r.Trk[k] = ti; r.TrkGen[k] = TrackHandles.GenerationOf(ti);
            }

            r.S[k] = s;
            var fr = f0;
            var rel = new Vector3F((float)(px - fr.Position.X), (float)(py - fr.Position.Y), (float)(pz - fr.Position.Z));
            var e = fr.Right; var u = fr.Up; var t = fr.Tangent;
            float y = Vector3F.Dot(rel, e), z = Vector3F.Dot(rel, u);
            float ae = Vector3F.Dot(axle, e), au = Vector3F.Dot(axle, u);
            if (ae < 0f) { ae = -ae; au = -au; }
            float ln = float.Sqrt(ae * ae + au * au);
            if (!(ln > 0.5f)) { r.Spin[k] = spin; return; }   // the axle is across the track no more: no wheel-rail model applies
            float cosP = ae / ln, sinP = au / ln;
            var a3 = e * cosP + u * sinP;
            var w3 = e * -sinP + u * cosP;
            float bw = 0.5f * def.TreadSpan, br = 0.5f * (tr.Def.Gauge + RailHeadWidth);
            float mu = tr.Def.Friction;
            float sprungWheel = 0.5f * carried;
            var vBody = new Vector3F(bodies.LinearVelocityOf(b)[0], bodies.LinearVelocityOf(b)[1], bodies.LinearVelocityOf(b)[2]);
            float vAlong = Vector3F.Dot(vBody, t);
            float vRef = Math.Max(Math.Abs(vAlong), 1f);
            float flangePlay = def.FlangeClearance + (br - bw);
            // Phase 1: both treads' and both flanges' normal forces, applied together, so that the roll each one causes is balanced by the other before any slip is measured.
            int nc = 0;
            for (int side = 0; side < 2; side++)
            {
                float sg = side == 0 ? -1f : 1f;
                // Tread: the point of the cone that meets the rail head's centre line (cone radius r0 - gamma xi towards the outer edge).
                float R = rw, xi = 0f;
                for (int it = 0; it < 3; it++) { xi = (br - sg * y - sg * R * sinP) / cosP - bw; R = rw - gamma * xi; }
                float zs = z + sg * (bw + xi) * sinP - R * cosP;
                float pen = -zs;
                if (pen > 0f && xi > -0.04f && xi < 0.08f)
                {
                    var nrm = (w3 - a3 * (sg * gamma)) * invRoot;
                    // The contact point on the wheel surface (at the cone's radius R from the axle), which is what moves at the wheel's rolling speed; it lies pen below the rail head.
                    var arm = a3 * (sg * (bw + xi)) - w3 * R;
                    var vP = RailPointVelocity(b, arm, axle, spin);
                    float pn = SoftNormal(b, arm, nrm, pen, sprungWheel, def.ContactHertz, 1f, h, gravityLead, axle, iAx, ref spin, vP);
                    rcs[nc++] = new RailContact { Side = side, Flange = false, Arm = arm, Normal = nrm, Load = pn / h };
                    if (side == 0) { r.NormalL[k] = pn / h; } else { r.NormalR[k] = pn / h; }
                    r.Flags[k] |= (byte)(1 << side);
                }

                // Flange: when this side has used up the lateral play the flange meets the rail's gauge face.
                float d = sg * y - flangePlay;
                if (d > 0f)
                {
                    var nf = e * (-sg * sinA) + u * cosA;
                    var arm = rel * -1f + e * (sg * (br - 0.04f)) - u * 0.012f;
                    var vP = RailPointVelocity(b, arm, axle, spin);
                    float pn = SoftNormal(b, arm, nf, d * sinA, sprungWheel, def.ContactHertz, 1f, h, gravityLead, axle, iAx, ref spin, vP);
                    rcs[nc++] = new RailContact { Side = side, Flange = true, Arm = arm, Normal = nf, Load = pn / h };
                    if (side == 0) { r.FlangeL[k] = pn / h; } else { r.FlangeR[k] = pn / h; }
                    r.Flags[k] |= (byte)(4 << side);
                }
            }

            // Phase 2: the tangential forces of every contact, from the slip velocities after all the normal forces have acted.
            float qLeft = 0f, qRight = 0f, yLeft = 0f, yRight = 0f;   // vertical and lateral force on each wheel (track axes) for the climb ratio
            for (int c = 0; c < nc; c++)
            {
                var rc = rcs[c];
                int side = rc.Side;
                var arm = rc.Arm; var nrm = rc.Normal; float load = rc.Load;
                float fe = load * Vector3F.Dot(nrm, e), fu = load * Vector3F.Dot(nrm, u);
                if (load > 0f)
                {
                    var l = t - nrm * Vector3F.Dot(t, nrm);
                    float ll = l.LengthSquared();
                    if (ll > 1e-8f)
                    {
                        l = l * (1f / float.Sqrt(ll));
                        var vP = RailPointVelocity(b, arm, axle, spin);
                        if (!rc.Flange)
                        {
                            // Contact patch tangent frame: rolling direction and lateral direction.
                            var lat = Vector3F.Cross(nrm, l);
                            var vw = vP - nrm * Vector3F.Dot(vP, nrm);
                            var bodyW = bodies.AngularVelocityOf(b);
                            float spinCreep = (Vector3F.Dot(new Vector3F(bodyW[0], bodyW[1], bodyW[2]), nrm) + spin * Vector3F.Dot(axle, nrm)) / vRef;
                            float xx = Vector3F.Dot(vw, l) / vRef, eta = Vector3F.Dot(vw, lat) / vRef;
                            float cN = PatchC * Math32.Cbrt(load);
                            float ab = cN * cN;
                            float f11 = def.CreepScale * KalkerG * ab * C11, f22 = def.CreepScale * KalkerG * ab * C22, f23 = def.CreepScale * KalkerG * ab * cN * C23;
                            float invL = EffInvRail(b, arm, l, axle, iAx), invT = EffInvRail(b, arm, lat, axle, iAx);
                            // Linear creep force, made implicit in the slip it acts on (a damper of coefficient f / vRef): stable for any stiffness and never reversing the slip within the step.
                            float fx = -f11 * xx / (1f + h * f11 * invL / vRef);
                            float fy = -(f22 * eta + f23 * spinCreep) / (1f + h * f22 * invT / vRef);
                            float mag = float.Sqrt(fx * fx + fy * fy);
                            float lim = mu * load;
                            if (!(lim > 0f)) { fx = 0f; fy = 0f; }
                            else if (mag > 1e-6f) { float sc = Saturate(mag / lim); fx *= sc; fy *= sc; }
                            ApplyRailImpulse(b, arm, l * (h * fx) + lat * (h * fy), axle, iAx, ref spin);
                            if (side == 0) { r.FxL[k] = fx; r.FyL[k] = fy; } else { r.FxR[k] = fx; r.FyR[k] = fy; }
                            var ft = l * fx + lat * fy;
                            fe += Vector3F.Dot(ft, e); fu += Vector3F.Dot(ft, u);
                        }
                        else
                        {
                            // Friction at the flange: along the track and up the face (Coulomb, regularised); the up-the-face part is what resists climbing (Nadal 1908).
                            var up = Vector3F.Cross(nrm, l);
                            float vl = Vector3F.Dot(vP, l), vu = Vector3F.Dot(vP, up);
                            float vs = float.Sqrt(vl * vl + vu * vu);
                            const float Eps = 0.01f;
                            float reg = float.Sqrt(vs * vs + Eps * Eps);
                            float fl = -mu * load * vl / reg, fuu = -mu * load * vu / reg;
                            float invL = EffInvRail(b, arm, l, axle, iAx), invU = EffInvRail(b, arm, up, axle, iAx);
                            if (invL > 0f) { float cap = Math.Abs(vl) / (h * invL); if (Math.Abs(fl) > cap) { fl = fl < 0f ? -cap : cap; } }
                            if (invU > 0f) { float cap = Math.Abs(vu) / (h * invU); if (Math.Abs(fuu) > cap) { fuu = fuu < 0f ? -cap : cap; } }
                            ApplyRailImpulse(b, arm, l * (h * fl) + up * (h * fuu), axle, iAx, ref spin);
                            var ft = l * fl + up * fuu;
                            fe += Vector3F.Dot(ft, e); fu += Vector3F.Dot(ft, u);
                        }
                    }
                }

                if (side == 0) { yLeft += fe; qLeft += fu; } else { yRight += fe; qRight += fu; }
            }

            float ratioL = (r.Flags[k] & 4) != 0 && qLeft > 1f ? Math.Abs(yLeft) / qLeft : 0f;
            float ratioR = (r.Flags[k] & 8) != 0 && qRight > 1f ? Math.Abs(yRight) / qRight : 0f;
            r.Climb[k] = Math.Max(ratioL, ratioR);
            r.QL[k] = qLeft; r.QR[k] = qRight; r.YL[k] = yLeft; r.YR[k] = yRight;

            // Gyroscopic coupling of the spinning wheelset with its frame.
            if (spin != 0f)
            {
                var hv = axle * (iAx * spin);
                var wv = bodies.AngularVelocityOf(b);
                AddAngularImpulse(b, Vector3F.Cross(new Vector3F(wv[0], wv[1], wv[2]), hv) * -h);
            }

            r.Spin[k] = float.IsNaN(spin) ? 0f : spin > 1e9f ? 1e9f : spin < -1e9f ? -1e9f : spin;
        }
    }

    // A coupler between this vehicle's end e and another vehicle's end, handled once from the lower slot: buffers push, the draw gear pulls after its slack and breaks at its rating.
    private void StepCouplers(int slot, RailState r, float h)
    {
        if (r.Car < 0) { return; }
        var bodies = Core.Bodies;
        for (int e = 0; e < 2; e++)
        {
            int o = r.CoupleVeh[e];
            if (o < 0 || o <= slot || r.Broken[e] != 0 || !RailHandles.IsAlive(o, r.CoupleGen[e])) { continue; }
            var os = Rails[o]!;
            int oe = r.CoupleEnd[e];
            if (os.Car < 0 || !RailLive(r) || !RailLive(os)) { continue; }
            int a = r.Car, b = os.Car;
            if (!bodies.IsAwake(a) && !bodies.IsAwake(b)) { continue; }
            if (!bodies.IsAwake(a)) { WakeBodyIsland(a); }
            if (!bodies.IsAwake(b)) { WakeBodyIsland(b); }
            RefreshWorldInertia(a); RefreshWorldInertia(b);
            var da = r.Def; var db = os.Def;
            var qa = RotationOfBody(a); var qb = RotationOfBody(b);
            var armA = qa.Rotate(new Vector3F(0f, da.CouplerHeight - da.CarCenterHeight, e == 0 ? da.CouplerReach : -da.CouplerReach));
            var armB = qb.Rotate(new Vector3F(0f, db.CouplerHeight - db.CarCenterHeight, oe == 0 ? db.CouplerReach : -db.CouplerReach));
            var pa = bodies.PositionOf(a); var pb = bodies.PositionOf(b);
            var delta = new Vector3F((float)(pb[0] - pa[0]), (float)(pb[1] - pa[1]), (float)(pb[2] - pa[2])) + armB - armA;
            float dist = float.Sqrt(delta.LengthSquared());
            if (!(dist > 1e-4f)) { continue; }
            var n = delta * (1f / dist);
            float x = dist - da.CouplerFreeLength;
            var va = PointVelocity(a, armA); var vb = PointVelocity(b, armB);
            float vrel = Vector3F.Dot(vb - va, n);   // positive while the couplers separate
            float force = 0f;   // positive pulls the vehicles together (tension), negative pushes them apart
            float invPair = bodies.InverseMass[a] + bodies.InverseMass[b];
            if (x < 0f)
            {
                float f = da.BufferStiffness * -x + da.CouplerDamping * Math.Max(-vrel, 0f);
                if (invPair > 0f) { f = Math.Min(f, 0.5f * -x / (h * h * invPair)); }
                force = -f;
            }
            else if (x > da.CouplerSlack)
            {
                float ext = x - da.CouplerSlack;
                float f = da.DrawStiffness * ext + da.CouplerDamping * Math.Max(vrel, 0f);
                if (invPair > 0f) { f = Math.Min(f, 0.5f * ext / (h * h * invPair)); }
                if (f > da.CouplerBreakForce) { r.Broken[e] = 1; os.Broken[oe] = 1; r.CouplerForce[e] = 0f; os.CouplerForce[oe] = 0f; continue; }
                force = f;
            }

            r.CouplerForce[e] = force; os.CouplerForce[oe] = force;
            if (force == 0f) { continue; }
            var j = n * (h * force);   // towards the other vehicle on this one, away on the other
            ApplyImpulseAtArm(a, armA, j);
            ApplyImpulseAtArm(b, armB, j * -1f);
        }
    }

    // ------------------------------------------------------------------ origin shift

    /// <summary>After the core shifted the origin: tracks are world geometry and move with it, exactly (their start points and the geometry's positions are on the 2^-32 grid).</summary>
    public void ShiftRail(Vector3D offset)
    {
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            t.Def.Start = new Vector3D(t.Def.Start.X - offset.X, t.Def.Start.Y - offset.Y, t.Def.Start.Z - offset.Z);
            t.Geo = new TrackGeometry(t.Def);
        }
    }

    /// <summary>Checks that every track's start stays within the supported range after a shift by <paramref name="offset"/>.</summary>
    public void CheckRailShift(Vector3D offset)
    {
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var s = Tracks[i]!.Def.Start;
            if (!(Math.Abs(s.X - offset.X) <= 4194304.0 && Math.Abs(s.Y - offset.Y) <= 4194304.0 && Math.Abs(s.Z - offset.Z) <= 4194304.0))
            { throw new ArgumentOutOfRangeException(nameof(offset), offset, "Shifting by this offset moves a track beyond ±4194304 m."); }
        }
    }

    // ------------------------------------------------------------------ debug

    /// <summary>Emits every track's centreline and every wheelset's contacts and creep forces.</summary>
    public void AddDebugRail(Debug.DebugGeometry g)
    {
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            double len = t.Geo.Length;
            int n = (int)Math.Min(2048.0, Math.Ceiling(len / 2.0)) + 1;
            var pts = new Vector3D[n];
            for (int k = 0; k < n; k++) { pts[k] = t.Geo.FrameAt(len * k / (n - 1)).Position; }
            g.Add(new Debug.DebugTrack(i, pts, t.Def.Gauge));
        }

        for (int v = 0; v < RailHandles.HighWater; v++)
        {
            if (!RailHandles.IsLive(v)) { continue; }
            var r = Rails[v]!;
            for (int k = 0; k < r.Wheelsets; k++)
            {
                int ti = r.Trk[k];
                if (ti < 0 || !TrackHandles.IsAlive(ti, r.TrkGen[k]) || !Core.Bodies.Handles.IsAlive(r.Ws[k], r.WsGen[k])) { continue; }
                var f = Tracks[ti]!.Geo.FrameAt(r.S[k]);
                float br = 0.5f * (Tracks[ti]!.Def.Gauge + RailHeadWidth);
                for (int side = 0; side < 2; side++)
                {
                    float sg = side == 0 ? -1f : 1f;
                    if ((r.Flags[k] & (1 << side)) == 0 && (r.Flags[k] & (4 << side)) == 0) { continue; }
                    var p = new Vector3D(f.Position.X + sg * br * f.Right.X, f.Position.Y + sg * br * f.Right.Y, f.Position.Z + sg * br * f.Right.Z);
                    float fx = side == 0 ? r.FxL[k] : r.FxR[k], fy = side == 0 ? r.FyL[k] : r.FyR[k];
                    float normal = (side == 0 ? r.NormalL[k] : r.NormalR[k]) + (side == 0 ? r.FlangeL[k] : r.FlangeR[k]);
                    var creep = f.Tangent * fx + f.Right * fy;
                    g.Add(new Debug.DebugRailContact(Core.HandleOfBody(r.Ws[k]), k, p, f.Up, normal, creep, (r.Flags[k] & (4 << side)) != 0));
                }
            }
        }
    }

    // ------------------------------------------------------------------ snapshot

    /// <summary>Id of the rail snapshot section ("RAIL"), shared with the 2D engine (a world is one or the other).</summary>
    public const int RailSectionId = 0x4C494152;

    private static void WriteVec3(StateWriter w, Vector3F v) { w.NamedSingle(v.X); w.NamedSingle(v.Y); w.NamedSingle(v.Z); }

    private static Vector3F ReadVec3(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    private static void WriteRailDef(StateWriter w, RailDef d)
    {
        w.NamedSingle(d.CarMass); WriteVec3(w, d.CarSize); w.NamedSingle(d.CarCenterHeight); w.NamedSingle(d.CarFloorHeight); w.NamedInt32(d.Bogies); w.NamedInt32(d.AxlesPerBogie); w.NamedSingle(d.BogieSpacing); w.NamedSingle(d.AxleSpacing);
        w.NamedSingle(d.BogieMass); WriteVec3(w, d.BogieInertia);
        w.NamedSingle(d.WheelRadius); w.NamedSingle(d.TreadSpan); w.NamedSingle(d.Conicity); w.NamedSingle(d.FlangeClearance); w.NamedSingle(d.FlangeAngle);
        w.NamedSingle(d.WheelsetMass); w.NamedSingle(d.WheelsetSpinInertia); w.NamedSingle(d.WheelsetTransverseInertia); w.NamedSingle(d.ContactHertz); w.NamedSingle(d.CreepScale);
        WriteVec3(w, d.PrimaryHertz); WriteVec3(w, d.PrimaryAngularHertz); w.NamedSingle(d.PrimaryDamping); WriteVec3(w, d.SecondaryHertz); WriteVec3(w, d.SecondaryAngularHertz); w.NamedSingle(d.SecondaryDamping);
        w.NamedSingle(d.MaxTractionForce); w.NamedSingle(d.MaxPower); w.NamedUInt32(d.DrivenWheelsets); w.NamedSingle(d.MaxBrakeForce);
        w.NamedSingle(d.CouplerReach); w.NamedSingle(d.CouplerHeight); w.NamedSingle(d.CouplerFreeLength); w.NamedSingle(d.CouplerSlack);
        w.NamedSingle(d.BufferStiffness); w.NamedSingle(d.DrawStiffness); w.NamedSingle(d.CouplerDamping); w.NamedSingle(d.CouplerBreakForce);
    }

    private static RailDef ReadRailDef(StateReader r)
    {
        var d = new RailDef
        {
            CarMass = r.ReadSingle(), CarSize = ReadVec3(r), CarCenterHeight = r.ReadSingle(), CarFloorHeight = r.ReadSingle(), Bogies = r.ReadInt32(), AxlesPerBogie = r.ReadInt32(), BogieSpacing = r.ReadSingle(), AxleSpacing = r.ReadSingle(),
            BogieMass = r.ReadSingle(), BogieInertia = ReadVec3(r),
            WheelRadius = r.ReadSingle(), TreadSpan = r.ReadSingle(), Conicity = r.ReadSingle(), FlangeClearance = r.ReadSingle(), FlangeAngle = r.ReadSingle(),
            WheelsetMass = r.ReadSingle(), WheelsetSpinInertia = r.ReadSingle(), WheelsetTransverseInertia = r.ReadSingle(), ContactHertz = r.ReadSingle(), CreepScale = r.ReadSingle(),
            PrimaryHertz = ReadVec3(r), PrimaryAngularHertz = ReadVec3(r), PrimaryDamping = r.ReadSingle(), SecondaryHertz = ReadVec3(r), SecondaryAngularHertz = ReadVec3(r), SecondaryDamping = r.ReadSingle(),
            MaxTractionForce = r.ReadSingle(), MaxPower = r.ReadSingle(), DrivenWheelsets = r.ReadUInt32(), MaxBrakeForce = r.ReadSingle(),
            CouplerReach = r.ReadSingle(), CouplerHeight = r.ReadSingle(), CouplerFreeLength = r.ReadSingle(), CouplerSlack = r.ReadSingle(),
            BufferStiffness = r.ReadSingle(), DrawStiffness = r.ReadSingle(), CouplerDamping = r.ReadSingle(), CouplerBreakForce = r.ReadSingle(),
        };
        try { d.Validate(); }
        catch (ArgumentException e) { throw r.Error("invalid rail vehicle definition: " + e.Message); }
        return d;
    }

    internal void WriteRail(StateWriter w)
    {
        w.NamedInt32(1);   // section version
        w.DescribeTable("track");
        TrackHandles.Write(w);
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            w.DescribeEntity("track", i, TrackHandles.GenerationOf(i));
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            TrackDefIo.Write(w, t.Def);
            w.NamedInt32(t.Ballast); w.NamedInt32(t.BallastGen); w.NamedInt32(t.Rails); w.NamedInt32(t.RailsGen);
            w.NamedInt32(t.Parent); w.NamedInt32(t.ParentGen); w.NamedDouble(t.ParentS);
            w.NamedInt32(t.Switches.Length);
            foreach (var sw in t.Switches) { w.NamedDouble(sw.S); w.NamedInt32(sw.Branch); w.NamedInt32(sw.BranchGen); w.NamedBool(sw.Diverging); }
        }

        w.DescribeTable("railvehicle");
        RailHandles.Write(w);
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            w.DescribeEntity("railvehicle", i, RailHandles.GenerationOf(i));
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            WriteRailDef(w, r.Def);
            w.NamedInt32(r.Car); w.NamedInt32(r.CarGen);
            w.NamedInt32(r.Bogie.Length);
            for (int k = 0; k < r.Bogie.Length; k++) { w.NamedInt32(r.Bogie[k]); w.NamedInt32(r.BogieGen[k]); }
            w.NamedInt32(r.Wheelsets);
            for (int k = 0; k < r.Wheelsets; k++)
            {
                w.NamedInt32(r.Ws[k]); w.NamedInt32(r.WsGen[k]); w.NamedInt32(r.Trk[k]); w.NamedInt32(r.TrkGen[k]); w.NamedDouble(r.S[k]); w.NamedSingle(r.Spin[k]);
                w.NamedSingle(r.NormalL[k]); w.NamedSingle(r.NormalR[k]); w.NamedSingle(r.FxL[k]); w.NamedSingle(r.FxR[k]); w.NamedSingle(r.FyL[k]); w.NamedSingle(r.FyR[k]);
                w.NamedSingle(r.FlangeL[k]); w.NamedSingle(r.FlangeR[k]); w.NamedSingle(r.Climb[k]); w.NamedByte(r.Flags[k]);
                w.NamedSingle(r.QL[k]); w.NamedSingle(r.QR[k]); w.NamedSingle(r.YL[k]); w.NamedSingle(r.YR[k]);
            }

            w.NamedSingle(r.Throttle); w.NamedSingle(r.Brake);
            for (int e = 0; e < 2; e++) { w.NamedInt32(r.CoupleVeh[e]); w.NamedInt32(r.CoupleGen[e]); w.NamedInt32(r.CoupleEnd[e]); w.NamedByte(r.Broken[e]); w.NamedSingle(r.CouplerForce[e]); }
        }
    }

    private sealed class RailBlock
    {
        public HandleTable Tracks = new(), Vehicles = new();
        public TrackState?[] TrackStates = Array.Empty<TrackState?>();
        public RailState?[] VehicleStates = Array.Empty<RailState?>();
    }

    private static RailBlock ReadRail(StateReader r, int bodyCount)
    {
        int version = r.ReadInt32();
        if (version != 1) { throw r.Error("unsupported rail section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var b = new RailBlock { Tracks = HandleTable.Read(r, "track") };
        b.TrackStates = new TrackState?[Math.Max(b.Tracks.HighWater, 1)];
        for (int i = 0; i < b.Tracks.HighWater; i++)
        {
            if (!b.Tracks.IsLive(i)) { continue; }
            var t = new TrackState { Def = TrackDefIo.Read(r) };
            t.Ballast = r.ReadInt32(); t.BallastGen = r.ReadInt32(); t.Rails = r.ReadInt32(); t.RailsGen = r.ReadInt32();
            t.Parent = r.ReadInt32(); t.ParentGen = r.ReadInt32(); t.ParentS = r.ReadDouble();
            if (t.Ballast >= bodyCount || t.Rails >= bodyCount || t.Ballast < -1 || t.Rails < -1) { throw r.Error("invalid track body"); }
            if (t.Parent < -1 || t.Parent >= b.Tracks.HighWater || !double.IsFinite(t.ParentS)) { throw r.Error("invalid track parent"); }
            int ns = r.ReadCount(17, "track switches");
            if (ns > 256) { throw r.Error("a track with more than 256 switches"); }
            t.Switches = new SwitchState[ns];
            for (int k = 0; k < ns; k++)
            {
                var sw = new SwitchState { S = r.ReadDouble(), Branch = r.ReadInt32(), BranchGen = r.ReadInt32(), Diverging = r.ReadBool() };
                if (!double.IsFinite(sw.S) || sw.Branch < -1 || sw.Branch >= b.Tracks.HighWater) { throw r.Error("invalid switch"); }
                t.Switches[k] = sw;
            }

            t.Geo = new TrackGeometry(t.Def);
            b.TrackStates[i] = t;
        }

        b.Vehicles = HandleTable.Read(r, "railvehicle");
        b.VehicleStates = new RailState?[Math.Max(b.Vehicles.HighWater, 1)];
        for (int i = 0; i < b.Vehicles.HighWater; i++)
        {
            if (!b.Vehicles.IsLive(i)) { continue; }
            var s = new RailState { Def = ReadRailDef(r) };
            s.Car = r.ReadInt32(); s.CarGen = r.ReadInt32();
            if (s.Car >= bodyCount || s.Car < -1) { throw r.Error("invalid rail vehicle car"); }
            int nb = r.ReadCount(8, "bogies");
            if (nb != (s.Def.Bogies == 0 ? 0 : s.Def.Bogies)) { throw r.Error("rail vehicle bogie count disagrees with its definition"); }
            s.Bogie = new int[nb]; s.BogieGen = new int[nb];
            for (int k = 0; k < nb; k++) { s.Bogie[k] = r.ReadInt32(); s.BogieGen[k] = r.ReadInt32(); if (s.Bogie[k] < 0 || s.Bogie[k] >= bodyCount) { throw r.Error("invalid bogie"); } }
            int nw = r.ReadCount(86, "wheelsets");
            if (nw != s.Def.WheelsetCount) { throw r.Error("rail vehicle wheelset count disagrees with its definition"); }
            s.Allocate(nw);
            for (int k = 0; k < nw; k++)
            {
                s.Ws[k] = r.ReadInt32(); s.WsGen[k] = r.ReadInt32(); s.Trk[k] = r.ReadInt32(); s.TrkGen[k] = r.ReadInt32(); s.S[k] = r.ReadDouble(); s.Spin[k] = r.ReadSingle();
                s.NormalL[k] = r.ReadSingle(); s.NormalR[k] = r.ReadSingle(); s.FxL[k] = r.ReadSingle(); s.FxR[k] = r.ReadSingle(); s.FyL[k] = r.ReadSingle(); s.FyR[k] = r.ReadSingle();
                s.FlangeL[k] = r.ReadSingle(); s.FlangeR[k] = r.ReadSingle(); s.Climb[k] = r.ReadSingle(); s.Flags[k] = r.ReadByte();
                s.QL[k] = r.ReadSingle(); s.QR[k] = r.ReadSingle(); s.YL[k] = r.ReadSingle(); s.YR[k] = r.ReadSingle();
                if (s.Ws[k] < 0 || s.Ws[k] >= bodyCount || s.WsGen[k] < 1) { throw r.Error("invalid wheelset body"); }
                if (s.Trk[k] < -1 || s.Trk[k] >= b.Tracks.HighWater) { throw r.Error("invalid wheelset track"); }
                float sum = s.Spin[k] + s.NormalL[k] + s.NormalR[k] + s.FxL[k] + s.FxR[k] + s.FyL[k] + s.FyR[k] + s.FlangeL[k] + s.FlangeR[k] + s.Climb[k] + s.QL[k] + s.QR[k] + s.YL[k] + s.YR[k];
                if (!float.IsFinite(sum) || !double.IsFinite(s.S[k]) || !(Math.Abs(s.Spin[k]) <= 1e9f) || s.Flags[k] > 15) { throw r.Error("invalid wheelset state"); }
            }

            s.Throttle = r.ReadSingle(); s.Brake = r.ReadSingle();
            if (!(Math.Abs(s.Throttle) <= 1f) || !(s.Brake >= 0f && s.Brake <= 1f)) { throw r.Error("invalid rail vehicle inputs"); }
            for (int e = 0; e < 2; e++)
            {
                s.CoupleVeh[e] = r.ReadInt32(); s.CoupleGen[e] = r.ReadInt32(); s.CoupleEnd[e] = r.ReadInt32(); s.Broken[e] = r.ReadByte(); s.CouplerForce[e] = r.ReadSingle();
                if (s.CoupleVeh[e] < -1 || s.CoupleVeh[e] >= b.Vehicles.HighWater || s.CoupleEnd[e] < 0 || s.CoupleEnd[e] > 1 || s.Broken[e] > 1 || !float.IsFinite(s.CouplerForce[e])) { throw r.Error("invalid coupler"); }
            }

            b.VehicleStates[i] = s;
        }

        return b;
    }

    private void CommitRail(RailBlock? b)
    {
        if (b == null)
        {
            TrackHandles.Reset(); RailHandles.Reset();
            Array.Clear(Tracks); Array.Clear(Rails); Array.Clear(_wsOfBody);
            return;
        }

        TrackHandles = b.Tracks; RailHandles = b.Vehicles;
        Tracks = b.TrackStates; Rails = b.VehicleStates;
        RebuildWheelsetIndex();
    }

    private void RebuildWheelsetIndex()
    {
        Array.Clear(_wsOfBody);
        for (int i = 0; i < RailHandles.HighWater; i++) { if (RailHandles.IsLive(i)) { IndexWheelsets(i, Rails[i]!); } }
    }

    /// <summary>After a restore: every rail vehicle's bodies exist as the generations it names and its tracks and coupler partners are live.</summary>
    internal bool RailConsistent()
    {
        var bodies = Core.Bodies;
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            if (!RailLive(r)) { return false; }
            for (int k = 0; k < r.Bogie.Length; k++) { if (!bodies.Handles.IsAlive(r.Bogie[k], r.BogieGen[k])) { return false; } }
            for (int k = 0; k < r.Wheelsets; k++)
            {
                if (r.Trk[k] >= 0 && !TrackHandles.IsAlive(r.Trk[k], r.TrkGen[k])) { return false; }
                if (bodies.TypeOf(r.Ws[k]) != BodyType.Dynamic) { return false; }
            }

            for (int e = 0; e < 2; e++)
            {
                int o = r.CoupleVeh[e];
                if (o >= 0 && (!RailHandles.IsAlive(o, r.CoupleGen[e]) || Rails[o]!.CoupleVeh[r.CoupleEnd[e]] != i)) { return false; }
            }
        }

        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            if (t.Ballast >= 0 && !bodies.Handles.IsAlive(t.Ballast, t.BallastGen)) { return false; }
            if (t.Rails >= 0 && !bodies.Handles.IsAlive(t.Rails, t.RailsGen)) { return false; }
            if (t.Parent >= 0 && !TrackHandles.IsAlive(t.Parent, t.ParentGen)) { return false; }
            foreach (var sw in t.Switches) { if (sw.Branch >= 0 && !TrackHandles.IsAlive(sw.Branch, sw.BranchGen)) { return false; } }
        }

        return true;
    }

    private RailSection? _railSection;

    internal ISnapshotSection RailSnapshotSection => _railSection ??= new RailSection(this);

    /// <summary>The optional snapshot section that carries tracks and rail vehicles. Written only while one exists, so worlds without rail keep their bytes and hashes.</summary>
    private sealed class RailSection(Engine3D e) : ISnapshotSection
    {
        public int SectionId => RailSectionId;
        public bool IsOptional => true;
        public bool IsPresent => e.TrackHandles.Count > 0 || e.RailHandles.Count > 0;

        public void Reset()
        {
            if (e.TrackHandles.HighWater == 0 && e.RailHandles.HighWater == 0) { return; }
            e.CommitRail(null);
        }

        public void Write(StateWriter w) => e.WriteRail(w);

        public void Read(StateReader r) => e.CommitRail(ReadRail(r, e.RestoredBodyCount));

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine3D)destination.GetSnapshotSection(Engine3D.Id);
            dst.TrackHandles = new HandleTable(); dst.TrackHandles.CopyFrom(e.TrackHandles);
            dst.RailHandles = new HandleTable(); dst.RailHandles.CopyFrom(e.RailHandles);
            dst.Tracks = new TrackState?[e.Tracks.Length];
            for (int i = 0; i < dst.Tracks.Length; i++) { dst.Tracks[i] = e.Tracks[i]?.CloneForFork(); }
            dst.Rails = new RailState?[e.Rails.Length];
            for (int i = 0; i < dst.Rails.Length; i++) { dst.Rails[i] = e.Rails[i]?.CloneForFork(); }
            dst._wsOfBody = (int[])e._wsOfBody.Clone();
            return dst.RailSnapshotSection;
        }
    }
}
