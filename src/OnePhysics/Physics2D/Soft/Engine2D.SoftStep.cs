using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>One particle-against-shape contact of a substep: a line the particle must stay on the outer side of, and the rigid body (if any) that takes the reaction.</summary>
internal struct SoftContact
{
    public SoftBodyData Body;
    public int Particle;
    public int Rigid;          // rigid body slot whose surface moves with the contact, or -1 for a static surface
    public bool Coupled;       // the rigid body is awake and dynamic: it takes the reaction
    public float Nx, Ny;       // outward normal of the shape (towards the particle)
    public float C0;           // separation of the particle surface from the shape at detection (negative when overlapping)
    public float X, Y;         // particle centre at detection, relative to the body origin of the step
    public float Mu;
    public float Applied;      // sum of the contact multipliers applied so far in this substep
    public float Rx, Ry;       // contact point relative to the rigid body's centre of mass
}

internal sealed partial class Engine2D
{
    // ---- pooled scratch of the soft step (grown, never shrunk; nothing here is state)
    private double[] _spPos0 = Array.Empty<double>();      // body positions at the start of the step, 2 per body slot
    private float[] _spRot0 = Array.Empty<float>();        // body rotations (cos, sin) at the start of the step, 2 per body slot
    private int[] _spAwakeStamp = Array.Empty<int>();      // per body slot: the step stamp when the body is in the awake list
    private SoftBodyData[] _awakeSoft = new SoftBodyData[4];
    private int _awakeSoftCount;
    // A sleeping body is woken by a touching soft body whose smoothed net speed is above this multiple of the sleep threshold.
    private const float WakeK = 8f;
    // Weight of the latest step in the smoothed net speed that decides rest.
    private const float SmoothK = 0.1f;
    private int _pairSoftCount;        // the awake bodies, then the sleeping bodies touched by them (immovable obstacles for the step)

    private bool SoftTouch(SoftBodyData d, SoftBodyData o)
    {
        double pad = d.Radius + o.Radius + Constants2D.SpeculativeDistance + 0.02;
        if (d.MinX - pad > o.MaxX || d.MaxX + pad < o.MinX || d.MinY - pad > o.MaxY || d.MaxY + pad < o.MinY) { return false; }
        return SoftPairAllowed(d.Def, o.Def);
    }
    private byte[] _wakeFlag = new byte[4];

    private int[] _cShape = new int[16], _cBody = new int[16];
    private bool[] _cMoving = new bool[16];
    private double[] _cPos = new double[32], _cPos0 = new double[32];       // shape frame origin at the end / start of the substep
    private float[] _cRot = new float[32];                                  // shape frame rotation (cos, sin) at the end of the substep
    private double[] _cBox = new double[64];                                // swept box of the shape over the substep: min xy, max xy
    private double[] _cBoxEnd = new double[64];                             // box at the end of the previous substep
    private double[] _cBoxNow = new double[64];
    private int _candCount;

    private SoftContact[] _sc = new SoftContact[64];
    private int _scCount;

    // Per rigid body slot: reaction impulses and virtual displacement, valid when the stamp matches.
    private int[] _rbStamp = Array.Empty<int>();
    private float[] _rbJ = Array.Empty<float>(), _rbL = Array.Empty<float>(), _rbDp = Array.Empty<float>(), _rbDr = Array.Empty<float>();
    private int[] _rbTouched = new int[16];
    private int _rbTouchedCount;
    private int _softStamp;

    private int[] _pairA = new int[64], _pairB = new int[64];
    private int _pairCount;
    private long[] _gridKeys = new long[64];
    private SoftBodyData[] _gData = new SoftBodyData[64];
    private int[] _gPart = new int[64];
    private int[] _gridStart = new int[64], _gridTable = new int[64];
    private long[] _gridCell = new long[64];

    private static void GrowTo<T>(ref T[] a, int n)
    {
        if (a.Length < n) { Array.Resize(ref a, Math.Max(n, a.Length * 2)); }
    }

    /// <summary>Remembers where every body is before the rigid step, so the soft step can follow the bodies' motion through it.</summary>
    public void BeginSoftStep()
    {
        if (SoftHandles.Count == 0) { return; }
        var bodies = Core.Bodies;
        var pos = bodies.Position;
        var rot = bodies.Rotation;
        GrowTo(ref _spPos0, pos.Length);
        GrowTo(ref _spRot0, rot.Length);
        pos.CopyTo(_spPos0);
        rot.CopyTo(_spRot0);
    }

    // ------------------------------------------------------------------ the soft step

    /// <summary>
    /// Advances every awake soft body by dt after the rigid step. Rigid bodies are followed through the step by interpolating between their poses before and after it, so a body that moved far
    /// is met where it was at each substep; the reaction on a rigid body is applied to its velocity at the end, as one impulse (momentum is conserved).
    /// </summary>
    public void StepSoft(float dt)
    {
        _softTears.Clear();
        if (SoftHandles.Count == 0) { return; }
        Core.Profile.BeginSoft();
        Core.Profile.Mark(StepStage.SoftSetup);
        var bodies = Core.Bodies;
        int substeps = Opt.SoftSubsteps, iterations = Opt.SoftIterations;
        float h = dt / substeps, invH = 1f / h;
        _softStamp++;
        if (_softStamp == int.MaxValue) { _softStamp = 1; Array.Clear(_rbStamp); Array.Clear(_spAwakeStamp); }
        GrowTo(ref _rbStamp, bodies.HighWater); GrowTo(ref _spAwakeStamp, bodies.HighWater);
        GrowTo(ref _rbJ, 2 * bodies.HighWater); GrowTo(ref _rbL, bodies.HighWater); GrowTo(ref _rbDp, 2 * bodies.HighWater); GrowTo(ref _rbDr, bodies.HighWater);
        _rbTouchedCount = 0;
        foreach (int b in Core.AwakeBodies) { _spAwakeStamp[b] = _softStamp; }

        // Which bodies are awake: a sleeping one wakes when an awake rigid body comes near it, or an awake soft body touches it.
        _awakeSoftCount = 0;
        float maxTravel = 0f;
        GrowTo(ref _wakeFlag, SoftHandles.HighWater);
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            _wakeFlag[i] = 0;
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            ComputeSoftBounds(d);
            if (d.Asleep && !RigidNear(d)) { _wakeFlag[i] = 1; continue; }
            _wakeFlag[i] = 2;
        }

        // A sleeping soft body that an awake one touches while moving (its smoothed net speed above eight times the sleep threshold) wakes; one that is only touched by an awake body that is nearly at rest stays asleep
        // and is an immovable obstacle for the step (passive), so an awake body never passes through it and a settling pile is not woken by the noise of its neighbours.
        float wakeSpeed = WakeK * Opt.SleepThreshold;
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (_wakeFlag[i] != 1) { continue; }
            var d = SoftData[i]!;
            for (int j = 0; j < SoftHandles.HighWater; j++)
            {
                if (_wakeFlag[j] != 2 || !(SoftData[j]!.StepSpeed2 > wakeSpeed * wakeSpeed)) { continue; }
                if (SoftTouch(d, SoftData[j]!)) { _wakeFlag[i] = 4; break; }
            }
        }

        for (int i = 0; i < SoftHandles.HighWater; i++) { if (_wakeFlag[i] == 4) { _wakeFlag[i] = 2; SoftData[i]!.Asleep = false; SoftData[i]!.SleepTime = 0f; } }
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (_wakeFlag[i] != 1) { continue; }
            var d = SoftData[i]!;
            for (int j = 0; j < SoftHandles.HighWater; j++)
            {
                if (_wakeFlag[j] == 2 && SoftTouch(d, SoftData[j]!)) { _wakeFlag[i] = 3; break; }
            }
        }

        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (_wakeFlag[i] != 2) { continue; }
            var d = SoftData[i]!;
            if (d.Asleep) { d.Asleep = false; d.SleepTime = 0f; }
            GrowTo(ref _awakeSoft, _awakeSoftCount + 1);
            _awakeSoft[_awakeSoftCount++] = d;
            if (d.MaxSpeed * dt > maxTravel) { maxTravel = d.MaxSpeed * dt; }
        }

        _pairSoftCount = _awakeSoftCount;
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (_wakeFlag[i] != 3) { continue; }
            GrowTo(ref _awakeSoft, _pairSoftCount + 1);
            _awakeSoft[_pairSoftCount++] = SoftData[i]!;
        }

        if (_awakeSoftCount == 0) { _pairSoftCount = 0; Core.Profile.Mark(StepStage.SoftEnd); return; }
        for (int a = 0; a < _pairSoftCount; a++) { BeginSoftBody(_awakeSoft[a]); }
        for (int a = _awakeSoftCount; a < _pairSoftCount; a++) { _awakeSoft[a].MakePassive(); }
        var g = Opt.Gravity;
        float gmag = float.Sqrt(g.X * g.X + g.Y * g.Y);
        float margin = maxTravel + 0.5f * gmag * dt * dt + Constants2D.SpeculativeDistance;
        GatherCandidates(margin);

        int total = 0;
        for (int a = 0; a < _pairSoftCount; a++) { _awakeSoft[a].Base = total; total += _awakeSoft[a].N; }
        Core.Counters.SoftParticles = total;
        for (int a = 0; a < _awakeSoftCount; a++) { Core.Counters.SoftBlocks += _awakeSoft[a].BlockCount; }
        GrowTo(ref _gData, total); GrowTo(ref _gPart, total); GrowTo(ref _gridKeys, total); GrowTo(ref _gridStart, total + 1);
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            for (int p = 0; p < d.N; p++) { _gData[d.Base + p] = d; _gPart[d.Base + p] = p; }
        }

        // The candidate shapes start the step at their poses before the rigid step.
        for (int c = 0; c < _candCount; c++) { CandidatePose(c, 0f); }
        Array.Copy(_cBoxNow, _cBoxEnd, 4 * _candCount);

        Core.Profile.Mark(StepStage.SoftPairs);
        BuildDetectTasks();
        BuildPairs(margin);
        BuildSurfacePairs(margin);
        for (int k = 0; k < substeps; k++)
        {
            float t1 = (float)(k + 1) / substeps;
            float tTarget = k == substeps - 1 ? 1f : t1;
            for (int c = 0; c < _candCount; c++)
            {
                Array.Copy(_cPos, 2 * c, _cPos0, 2 * c, 2);
                CandidatePose(c, t1);
            }

            Core.Profile.Mark(StepStage.SoftPredict);
            for (int a = 0; a < _awakeSoftCount; a++)
            {
                PredictSoft(_awakeSoft[a], h, g);
                if (_awakeSoft[a].HasTargets) { PredictTargets(_awakeSoft[a], tTarget); }
            }

            Core.Profile.Mark(StepStage.SoftDetect);
            DetectAll();
            Core.Profile.Mark(StepStage.SoftPairs);
            SoftenDepenetration(h);
            for (int a = 0; a < _awakeSoftCount; a++) { Array.Clear(_awakeSoft[a].Lambda); Array.Clear(_awakeSoft[a].MatchMu); }
            _softSubstep = k;
            SolveSubstep(h, invH, iterations);

            Core.Profile.Mark(StepStage.SoftFinish);
            ExcludeDepenetration();
            for (int a = 0; a < _awakeSoftCount; a++) { FinishSubstep(_awakeSoft[a], invH); }
            RemoveApproachVelocity();
            Array.Copy(_cBoxNow, _cBoxEnd, 4 * _candCount);
        }

        for (int a = 0; a < _awakeSoftCount; a++) { NetSpeed(_awakeSoft[a], 1f / dt); }
        DetectSoftTears(h);
        for (int a = 0; a < _awakeSoftCount; a++) { EndSoftBody(_awakeSoft[a]); }
        ApplyRigidReactions();
        ApplySoftTears();
        for (int a = _awakeSoftCount; a < _pairSoftCount; a++) { _awakeSoft[a].RestorePassive(); }
        UpdateSoftSleep(dt);
        Core.Profile.Mark(StepStage.SoftEnd);
        for (int a = 0; a < _awakeSoftCount; a++) { if (_awakeSoft[a].HasTargets) { EndTargets(_awakeSoft[a]); } }
    }

    // The step's float32 positions are relative to the middle of the body's bounds, rounded by the double subtraction alone, so they depend on the body's state only.
    private static void BeginSoftBody(SoftBodyData d)
    {
        d.Ox = 0.5 * (d.MinX + d.MaxX); d.Oy = 0.5 * (d.MinY + d.MaxY);
        var pos = d.Pos; var x = d.X; var xp = d.XP;
        for (int i = 0; i < d.N; i++)
        {
            int k = 2 * i;
            x[k] = (float)(pos[k] - d.Ox); x[k + 1] = (float)(pos[k + 1] - d.Oy);
            xp[k] = x[k]; xp[k + 1] = x[k + 1];
            d.X0[k] = x[k]; d.X0[k + 1] = x[k + 1];
        }
    }

    // The motion of the step as a whole: the substeps' last velocity can swing back and forth around a position that barely moves, so rest is judged from the net displacement of the step.
    private static void NetSpeed(SoftBodyData d, float invDt)
    {
        double ke = 0.0, mass = 0.0;
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            float dx = (d.X[2 * i] - d.X0[2 * i]) * invDt, dy = (d.X[2 * i + 1] - d.X0[2 * i + 1]) * invDt;
            double mi = 1.0 / d.InvMass[i];
            ke += mi * (dx * dx + dy * dy); mass += mi;
        }

        float now = mass > 0.0 ? (float)(ke / mass) : 0f;
        d.StepSpeed2 += (now - d.StepSpeed2) * SmoothK;
    }

    // Writes the step's result back to the double state between steps.
    private static void EndSoftBody(SoftBodyData d)
    {
        var pos = d.Pos; var prev = d.Prev; var x = d.X; var xp = d.XP;
        for (int i = 0; i < d.N; i++)
        {
            int k = 2 * i;
            // A pinned particle never moves in the step; keeping its double position as given stops the rounding to float from walking it.
            if (d.Pinned[i] != 0) { prev[k] = pos[k]; prev[k + 1] = pos[k + 1]; continue; }
            // A kinematic particle ends the step exactly on its target.
            if (d.IsTargetKinematic(i)) { pos[k] = d.TargetPos[k]; pos[k + 1] = d.TargetPos[k + 1]; prev[k] = d.Ox + xp[k]; prev[k + 1] = d.Oy + xp[k + 1]; continue; }
            pos[k] = d.Ox + x[k]; pos[k + 1] = d.Oy + x[k + 1];
            prev[k] = d.Ox + xp[k]; prev[k + 1] = d.Oy + xp[k + 1];
        }
    }

    private static void ComputeSoftBounds(SoftBodyData d)
    {
        double minx = double.MaxValue, miny = double.MaxValue, maxx = double.MinValue, maxy = double.MinValue;
        float vmax2 = 0f;
        for (int i = 0; i < d.N; i++)
        {
            double x = d.Pos[2 * i], y = d.Pos[2 * i + 1];
            if (x < minx) { minx = x; } if (x > maxx) { maxx = x; }
            if (y < miny) { miny = y; } if (y > maxy) { maxy = y; }
            float vx = d.Vel[2 * i], vy = d.Vel[2 * i + 1];
            float v2 = vx * vx + vy * vy;
            if (v2 > vmax2) { vmax2 = v2; }
            if (d.HasTargets && d.TargetOn[i] != 0)
            {
                double tx = d.TargetPos[2 * i], ty = d.TargetPos[2 * i + 1];
                if (tx < minx) { minx = tx; } if (tx > maxx) { maxx = tx; }
                if (ty < miny) { miny = ty; } if (ty > maxy) { maxy = ty; }
            }
        }

        d.MinX = minx; d.MinY = miny; d.MaxX = maxx; d.MaxY = maxy;
        d.MaxSpeed = float.Sqrt(vmax2);
    }

    // Whether an awake rigid body is within reach of a sleeping soft body.
    private bool RigidNear(SoftBodyData d)
    {
        var bodies = Core.Bodies;
        double pad = d.Radius + Constants2D.SpeculativeDistance + 0.02;
        foreach (int b in Core.AwakeBodies)
        {
            var p = bodies.PositionOf(b);
            double reach = MaxExtent[b] + pad;
            double x0 = _spPos0[2 * b], y0 = _spPos0[2 * b + 1];
            if (Math.Min(p[0], x0) - reach > d.MaxX || Math.Max(p[0], x0) + reach < d.MinX
                || Math.Min(p[1], y0) - reach > d.MaxY || Math.Max(p[1], y0) + reach < d.MinY) { continue; }
            if (!CanCollideSoftBody(d, b)) { continue; }
            return true;
        }

        return false;
    }

    private bool CanCollideSoftBody(SoftBodyData d, int b)
    {
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            if (CanCollideSoft(d, s)) { return true; }
        }

        return false;
    }

    /// <summary>The same rule as shape pairs: sensors never collide, then the layer matrix, then group, then category and mask.</summary>
    private bool CanCollideSoft(SoftBodyData d, int s)
    {
        if ((ShapeFlags[s] & FlagSensor) != 0) { return false; }
        var def = d.Def;
        if (((LayerMatrix[def.Layer] >> ShapeLayer[s]) & 1u) == 0) { return false; }
        if (Group[s] == def.GroupIndex && def.GroupIndex != 0) { return def.GroupIndex > 0; }
        return (def.MaskBits & Category[s]) != 0 && (def.CategoryBits & Mask[s]) != 0;
    }

    // ------------------------------------------------------------------ candidate shapes

    private struct StaticQuery : IOverlapCallback
    {
        public Engine2D E;
        public SoftBodyData D;

        public bool Report(int proxy, int userId)
        {
            int s = userId;
            int b = E.Core.Shapes.Owner[s];
            // Bodies in the awake list (awake dynamic, moving kinematic) are found through it: their poses move during the step.
            if (E._spAwakeStamp[b] == E._softStamp) { return true; }
            if (!E.CanCollideSoft(D, s)) { return true; }
            E.AddCandidate(s, b, false);
            return true;
        }
    }

    private void AddCandidate(int s, int b, bool moving)
    {
        int n = _candCount;
        GrowTo(ref _cShape, n + 1); GrowTo(ref _cBody, n + 1); GrowTo(ref _cMoving, n + 1);
        GrowTo(ref _cPos, 2 * (n + 1)); GrowTo(ref _cPos0, 2 * (n + 1)); GrowTo(ref _cRot, 2 * (n + 1));
        GrowTo(ref _cBox, 4 * (n + 1)); GrowTo(ref _cBoxEnd, 4 * (n + 1)); GrowTo(ref _cBoxNow, 4 * (n + 1));
        _cShape[n] = s; _cBody[n] = b; _cMoving[n] = moving;
        _candCount = n + 1;
    }

    private void GatherCandidates(float margin)
    {
        _candCount = 0;
        var bodies = Core.Bodies;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            d.CandStart = _candCount;
            double pad = d.Radius + margin + 0.05;
            var box = new Aabb2D(d.MinX - pad, d.MinY - pad, d.MaxX + pad, d.MaxY + pad);
            var q = new StaticQuery { E = this, D = d };
            Bp.QueryAabb(in box, ref q, BodyTypeMask.All);
            foreach (int b in Core.AwakeBodies)
            {
                var p = bodies.PositionOf(b);
                double reach = MaxExtent[b] + 0.05;
                double x0 = _spPos0[2 * b], y0 = _spPos0[2 * b + 1];
                if (Math.Min(p[0], x0) - reach > box.MaxX || Math.Max(p[0], x0) + reach < box.MinX
                    || Math.Min(p[1], y0) - reach > box.MaxY || Math.Max(p[1], y0) + reach < box.MinY) { continue; }
                for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
                {
                    if (CanCollideSoft(d, s)) { AddCandidate(s, b, true); }
                }
            }

            d.CandCount = _candCount - d.CandStart;
        }
    }

    // The pose of body b's centre of mass and rotation at the fraction t of the step (the motion through the rigid step interpolated, plus the displacement the soft reactions have given it so far).
    private void BodyPoseAt(int b, float t, out double px, out double py, out float c, out float s)
    {
        var bodies = Core.Bodies;
        var p1 = bodies.PositionOf(b);
        var r1 = bodies.RotationOf(b);
        double x0 = _spPos0[2 * b], y0 = _spPos0[2 * b + 1];
        px = x0 + (p1[0] - x0) * t; py = y0 + (p1[1] - y0) * t;
        float c0 = _spRot0[2 * b], s0 = _spRot0[2 * b + 1];
        float c1 = r1[0], s1 = r1[1];
        float w0 = 1f - t;
        float qc = w0 * c0 + t * c1, qs = w0 * s0 + t * s1;
        if (_rbStamp[b] == _softStamp)
        {
            px += _rbDp[2 * b]; py += _rbDp[2 * b + 1];
            float dr = _rbDr[b];
            // q += (virtual angle) * perp(q)
            float nc = qc - dr * qs, ns = qs + dr * qc;
            qc = nc; qs = ns;
        }

        float len = float.Sqrt(qc * qc + qs * qs);
        c = qc / len; s = qs / len;
    }

    private void CandidatePose(int c, float t)
    {
        int s = _cShape[c], b = _cBody[c];
        double px, py;
        float qc, qs;
        if (_cMoving[c]) { BodyPoseAt(b, t, out px, out py, out qc, out qs); }
        else
        {
            var p = Core.Bodies.PositionOf(b);
            var r = Core.Bodies.RotationOf(b);
            px = p[0]; py = p[1]; qc = r[0]; qs = r[1];
        }

        var q = new Rot2(qc, qs);
        var arm = q.Rotate(new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]));
        // The shape's frame is the body's frame: the centre of mass minus the rotated local centre.
        double sx = px - arm.X, sy = py - arm.Y;
        _cPos[2 * c] = sx; _cPos[2 * c + 1] = sy;
        _cRot[2 * c] = qc; _cRot[2 * c + 1] = qs;
        var bb = Geom[s].ComputeAabb(new Transform2(default, q));
        double n0 = sx + bb.Lower.X, n1 = sy + bb.Lower.Y, x0 = sx + bb.Upper.X, x1 = sy + bb.Upper.Y;
        int o = 4 * c;
        bool first = t == 0f;
        _cBox[o] = first ? n0 : Math.Min(n0, _cBoxEnd[o]); _cBox[o + 1] = first ? n1 : Math.Min(n1, _cBoxEnd[o + 1]);
        _cBox[o + 2] = first ? x0 : Math.Max(x0, _cBoxEnd[o + 2]); _cBox[o + 3] = first ? x1 : Math.Max(x1, _cBoxEnd[o + 3]);
        _cBoxNow[o] = n0; _cBoxNow[o + 1] = n1; _cBoxNow[o + 2] = x0; _cBoxNow[o + 3] = x1;
    }
}
