using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>One particle-against-shape contact of a substep: a plane the particle must stay on the outer side of, and the rigid body (if any) that takes the reaction.</summary>
internal struct SoftContact
{
    public SoftBodyData Body;
    public int Particle;
    public int Rigid;          // rigid body slot whose surface moves with the contact, or -1 for a static surface
    public bool Coupled;       // the rigid body is awake and dynamic: it takes the reaction
    public float Nx, Ny, Nz;   // outward normal of the shape (towards the particle)
    public float C0;           // separation of the particle surface from the shape at detection (negative when overlapping)
    public double X, Y, Z;     // particle centre at detection
    public float Mu;
    public float Rx, Ry, Rz;   // contact point relative to the rigid body's centre of mass
}

internal sealed partial class Engine3D
{
    // ---- pooled scratch of the soft step (grown, never shrunk; nothing here is state)
    private double[] _spPos0 = Array.Empty<double>();      // body positions at the start of the step, 3 per body slot
    private float[] _spRot0 = Array.Empty<float>();        // body rotations at the start of the step, 4 per body slot
    private SoftBodyData[] _awakeSoft = new SoftBodyData[4];
    private int _awakeSoftCount;

    private int[] _cShape = new int[16], _cBody = new int[16];
    private bool[] _cMoving = new bool[16];
    private double[] _cPos = new double[48], _cPos0 = new double[48];       // shape frame position at the end / start of the substep
    private float[] _cRot = new float[64];                                  // shape frame rotation (x y z w) at the end of the substep
    private double[] _cBox = new double[96];                                // swept box of the shape over the substep: min xyz, max xyz
    private double[] _cBoxEnd = new double[96];                             // box at the end of the previous substep
    private int _candCount;

    private SoftContact[] _sc = new SoftContact[64];
    private int _scCount;

    // Per rigid body slot: reaction impulses, virtual displacement and world inverse inertia, valid when the stamp matches.
    private int[] _rbStamp = Array.Empty<int>();
    private float[] _rbJ = Array.Empty<float>(), _rbL = Array.Empty<float>(), _rbDp = Array.Empty<float>(), _rbDr = Array.Empty<float>(), _rbIw = Array.Empty<float>();
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
    private float[] _gradP = Array.Empty<float>();
    private readonly System.Collections.Generic.List<int> _softIds = new();
    private readonly System.Collections.Generic.List<SoupTriangle3D> _softTris = new();

    private static void GrowTo<T>(ref T[] a, int n)
    {
        if (a.Length < n) { Array.Resize(ref a, Math.Max(n, a.Length * 2)); }
    }

    /// <summary>Remembers where every body is before the rigid step, so the soft step can follow the bodies' motion through it.</summary>
    public void BeginSoftStep()
    {
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
        if (SoftHandles.Count == 0) { return; }
        var bodies = Core.Bodies;
        int substeps = Opt.SoftSubsteps, iterations = Opt.SoftIterations;
        float h = dt / substeps, invH = 1f / h;
        _softStamp++;
        if (_softStamp == int.MaxValue) { _softStamp = 1; Array.Clear(_rbStamp); }
        GrowTo(ref _rbStamp, bodies.HighWater);
        GrowTo(ref _rbJ, 3 * bodies.HighWater); GrowTo(ref _rbL, 3 * bodies.HighWater); GrowTo(ref _rbDp, 3 * bodies.HighWater); GrowTo(ref _rbDr, 3 * bodies.HighWater); GrowTo(ref _rbIw, 6 * bodies.HighWater);
        _rbTouchedCount = 0;

        // Which bodies are awake: a sleeping one wakes when an awake rigid body comes near it.
        _awakeSoftCount = 0;
        float maxTravel = 0f;
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            ComputeSoftBounds(d);
            if (d.Asleep)
            {
                if (!RigidNear(d)) { continue; }
                d.Asleep = false; d.SleepTime = 0f;
            }

            GrowTo(ref _awakeSoft, _awakeSoftCount + 1);
            _awakeSoft[_awakeSoftCount++] = d;
            if (d.MaxSpeed * dt > maxTravel) { maxTravel = d.MaxSpeed * dt; }
        }

        if (_awakeSoftCount == 0) { return; }
        var g = Opt.Gravity;
        float gmag = float.Sqrt(g.X * g.X + g.Y * g.Y + g.Z * g.Z);
        float margin = maxTravel + 0.5f * gmag * dt * dt + Opt.SpeculativeDistance;
        GatherCandidates(margin, dt);

        int total = 0;
        for (int a = 0; a < _awakeSoftCount; a++) { _awakeSoft[a].Base = total; total += _awakeSoft[a].N; }
        GrowTo(ref _gData, total); GrowTo(ref _gPart, total); GrowTo(ref _gridKeys, total); GrowTo(ref _gridStart, total + 1);
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            for (int p = 0; p < d.N; p++) { _gData[d.Base + p] = d; _gPart[d.Base + p] = p; }
        }

        // The candidate shapes start the step at their poses before the rigid step.
        for (int c = 0; c < _candCount; c++) { CandidatePose(c, 0f); }
        Array.Copy(_cBoxNow, _cBoxEnd, 6 * _candCount);

        for (int k = 0; k < substeps; k++)
        {
            float t0 = (float)k / substeps, t1 = (float)(k + 1) / substeps;
            for (int c = 0; c < _candCount; c++)
            {
                Array.Copy(_cPos, 3 * c, _cPos0, 3 * c, 3);
                CandidatePose(c, t1);
            }

            for (int a = 0; a < _awakeSoftCount; a++) { PredictSoft(_awakeSoft[a], h, g); }
            _scCount = 0;
            for (int a = 0; a < _awakeSoftCount; a++) { DetectContacts(_awakeSoft[a], h); }
            BuildPairs();
            for (int a = 0; a < _awakeSoftCount; a++) { Array.Clear(_awakeSoft[a].Lambda); }
            for (int it = 0; it < iterations; it++)
            {
                for (int a = 0; a < _awakeSoftCount; a++) { SolveConstraints(_awakeSoft[a], h); }
                SolveContacts(h, invH);
                SolvePairs();
            }

            ExcludeDepenetration();
            for (int a = 0; a < _awakeSoftCount; a++) { FinishSubstep(_awakeSoft[a], invH); }
            RemoveApproachVelocity();
            Array.Copy(_cBoxNow, _cBoxEnd, 6 * _candCount);
            _ = t0;
        }

        ApplyRigidReactions();
        UpdateSoftSleep(dt);
    }

    private static void ComputeSoftBounds(SoftBodyData d)
    {
        double minx = double.MaxValue, miny = double.MaxValue, minz = double.MaxValue, maxx = double.MinValue, maxy = double.MinValue, maxz = double.MinValue;
        float vmax2 = 0f;
        for (int i = 0; i < d.N; i++)
        {
            double x = d.Pos[3 * i], y = d.Pos[3 * i + 1], z = d.Pos[3 * i + 2];
            if (x < minx) { minx = x; } if (x > maxx) { maxx = x; }
            if (y < miny) { miny = y; } if (y > maxy) { maxy = y; }
            if (z < minz) { minz = z; } if (z > maxz) { maxz = z; }
            float vx = d.Vel[3 * i], vy = d.Vel[3 * i + 1], vz = d.Vel[3 * i + 2];
            float v2 = vx * vx + vy * vy + vz * vz;
            if (v2 > vmax2) { vmax2 = v2; }
        }

        d.MinX = minx; d.MinY = miny; d.MinZ = minz; d.MaxX = maxx; d.MaxY = maxy; d.MaxZ = maxz;
        d.MaxSpeed = float.Sqrt(vmax2);
    }

    // Whether an awake rigid body is within reach of a sleeping soft body.
    private bool RigidNear(SoftBodyData d)
    {
        var bodies = Core.Bodies;
        double pad = d.Radius + Opt.SpeculativeDistance + 0.02;
        foreach (int b in Core.AwakeBodies)
        {
            var p = bodies.PositionOf(b);
            double reach = MaxExtent[b] + pad;
            double x0 = _spPos0[3 * b], y0 = _spPos0[3 * b + 1], z0 = _spPos0[3 * b + 2];
            if (Math.Min(p[0], x0) - reach > d.MaxX || Math.Max(p[0], x0) + reach < d.MinX
                || Math.Min(p[1], y0) - reach > d.MaxY || Math.Max(p[1], y0) + reach < d.MinY
                || Math.Min(p[2], z0) - reach > d.MaxZ || Math.Max(p[2], z0) + reach < d.MinZ) { continue; }
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
        public Engine3D E;
        public SoftBodyData D;

        public bool Report(int proxy, int userId)
        {
            int s = userId;
            var bodies = E.Core.Bodies;
            int b = E.Core.Shapes.Owner[s];
            // Awake dynamic and all kinematic bodies are found through the awake list (their poses move during the step).
            if (bodies.TypeOf(b) == BodyType.Kinematic || (bodies.TypeOf(b) == BodyType.Dynamic && bodies.IsAwake(b))) { return true; }
            if (!E.CanCollideSoft(D, s)) { return true; }
            E.AddCandidate(s, b, false);
            return true;
        }
    }

    private void AddCandidate(int s, int b, bool moving)
    {
        int n = _candCount;
        GrowTo(ref _cShape, n + 1); GrowTo(ref _cBody, n + 1); GrowTo(ref _cMoving, n + 1);
        GrowTo(ref _cPos, 3 * (n + 1)); GrowTo(ref _cPos0, 3 * (n + 1)); GrowTo(ref _cRot, 4 * (n + 1));
        GrowTo(ref _cBox, 6 * (n + 1)); GrowTo(ref _cBoxEnd, 6 * (n + 1)); GrowTo(ref _cBoxNow, 6 * (n + 1));
        _cShape[n] = s; _cBody[n] = b; _cMoving[n] = moving;
        _candCount = n + 1;
    }

    private void GatherCandidates(float margin, float dt)
    {
        _candCount = 0;
        var bodies = Core.Bodies;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            d.CandStart = _candCount;
            double pad = d.Radius + margin + 0.05;
            var box = new Aabb3D(d.MinX - pad, d.MinY - pad, d.MinZ - pad, d.MaxX + pad, d.MaxY + pad, d.MaxZ + pad);
            var q = new StaticQuery { E = this, D = d };
            Bp.QueryAabb(box, ref q, BodyTypeMask.Static | BodyTypeMask.Dynamic);
            foreach (int b in Core.AwakeBodies)
            {
                var p = bodies.PositionOf(b);
                double reach = MaxExtent[b] + 0.05;
                double x0 = _spPos0[3 * b], y0 = _spPos0[3 * b + 1], z0 = _spPos0[3 * b + 2];
                if (Math.Min(p[0], x0) - reach > box.MaxX || Math.Max(p[0], x0) + reach < box.MinX
                    || Math.Min(p[1], y0) - reach > box.MaxY || Math.Max(p[1], y0) + reach < box.MinY
                    || Math.Min(p[2], z0) - reach > box.MaxZ || Math.Max(p[2], z0) + reach < box.MinZ) { continue; }
                for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
                {
                    if (CanCollideSoft(d, s)) { AddCandidate(s, b, true); }
                }
            }

            d.CandCount = _candCount - d.CandStart;
        }

        _ = dt;
    }

    // The pose of body b at the fraction t of the step (the motion through the rigid step interpolated, plus the displacement the soft reactions have given it so far).
    private void BodyPoseAt(int b, float t, out double px, out double py, out double pz, out QuaternionF q)
    {
        var bodies = Core.Bodies;
        var p1 = bodies.PositionOf(b);
        var r1 = bodies.RotationOf(b);
        double x0 = _spPos0[3 * b], y0 = _spPos0[3 * b + 1], z0 = _spPos0[3 * b + 2];
        px = x0 + (p1[0] - x0) * t; py = y0 + (p1[1] - y0) * t; pz = z0 + (p1[2] - z0) * t;
        float ax = _spRot0[4 * b], ay = _spRot0[4 * b + 1], az = _spRot0[4 * b + 2], aw = _spRot0[4 * b + 3];
        float bx = r1[0], by = r1[1], bz = r1[2], bw = r1[3];
        float dot = ax * bx + ay * by + az * bz + aw * bw;
        if (dot < 0f) { bx = -bx; by = -by; bz = -bz; bw = -bw; }
        float s0 = 1f - t;
        float qx = s0 * ax + t * bx, qy = s0 * ay + t * by, qz = s0 * az + t * bz, qw = s0 * aw + t * bw;
        if (_rbStamp[b] == _softStamp)
        {
            px += _rbDp[3 * b]; py += _rbDp[3 * b + 1]; pz += _rbDp[3 * b + 2];
            float dx = _rbDr[3 * b], dy = _rbDr[3 * b + 1], dz = _rbDr[3 * b + 2];
            // q += ½ (ω_virtual, 0) ⊗ q
            float nx = qw * dx + (dy * qz - dz * qy), ny = qw * dy + (dz * qx - dx * qz), nz = qw * dz + (dx * qy - dy * qx), nw = -(dx * qx + dy * qy + dz * qz);
            qx += 0.5f * nx; qy += 0.5f * ny; qz += 0.5f * nz; qw += 0.5f * nw;
        }

        float len = float.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        q = new QuaternionF(qx / len, qy / len, qz / len, qw / len);
    }

    private void CandidatePose(int c, float t)
    {
        int s = _cShape[c], b = _cBody[c];
        double px, py, pz;
        QuaternionF q;
        if (_cMoving[c]) { BodyPoseAt(b, t, out px, out py, out pz, out q); }
        else
        {
            var p = Core.Bodies.PositionOf(b);
            px = p[0]; py = p[1]; pz = p[2];
            q = RotationOfBody(b);
        }

        var l = Local[s];
        var arm = q.Rotate(l.Translation - LocalCenterOf(b));
        var rw = q * l.Rotation;
        double sx = px + arm.X, sy = py + arm.Y, sz = pz + arm.Z;
        _cPos[3 * c] = sx; _cPos[3 * c + 1] = sy; _cPos[3 * c + 2] = sz;
        _cRot[4 * c] = rw.X; _cRot[4 * c + 1] = rw.Y; _cRot[4 * c + 2] = rw.Z; _cRot[4 * c + 3] = rw.W;
        // Box of the shape at this pose, and its union with the box at the start of the substep.
        var g = Geoms[GeomOf[s]]!;
        Vector3F mn, mx;
        if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, new RigidTransformF(rw, default), out mn, out mx); }
        else
        {
            RotatedBox(g.LocalBounds, rw, out mn, out mx);
        }

        double n0 = sx + mn.X, n1 = sy + mn.Y, n2 = sz + mn.Z, x0 = sx + mx.X, x1 = sy + mx.Y, x2 = sz + mx.Z;
        int o = 6 * c;
        double e0 = _cBoxEnd[o], e1 = _cBoxEnd[o + 1], e2 = _cBoxEnd[o + 2], e3 = _cBoxEnd[o + 3], e4 = _cBoxEnd[o + 4], e5 = _cBoxEnd[o + 5];
        bool first = t == 0f;
        _cBox[o] = first ? n0 : Math.Min(n0, e0); _cBox[o + 1] = first ? n1 : Math.Min(n1, e1); _cBox[o + 2] = first ? n2 : Math.Min(n2, e2);
        _cBox[o + 3] = first ? x0 : Math.Max(x0, e3); _cBox[o + 4] = first ? x1 : Math.Max(x1, e4); _cBox[o + 5] = first ? x2 : Math.Max(x2, e5);
        _cBoxNow[o] = n0; _cBoxNow[o + 1] = n1; _cBoxNow[o + 2] = n2; _cBoxNow[o + 3] = x0; _cBoxNow[o + 4] = x1; _cBoxNow[o + 5] = x2;
    }

    private double[] _cBoxNow = new double[96];

    private static void RotatedBox(Bounds3F lb, QuaternionF q, out Vector3F mn, out Vector3F mx)
    {
        float lox = float.MaxValue, loy = float.MaxValue, loz = float.MaxValue, hix = float.MinValue, hiy = float.MinValue, hiz = float.MinValue;
        for (int i = 0; i < 8; i++)
        {
            var c = q.Rotate(new Vector3F((i & 1) == 0 ? lb.Min.X : lb.Max.X, (i & 2) == 0 ? lb.Min.Y : lb.Max.Y, (i & 4) == 0 ? lb.Min.Z : lb.Max.Z));
            if (c.X < lox) { lox = c.X; } if (c.X > hix) { hix = c.X; }
            if (c.Y < loy) { loy = c.Y; } if (c.Y > hiy) { hiy = c.Y; }
            if (c.Z < loz) { loz = c.Z; } if (c.Z > hiz) { hiz = c.Z; }
        }

        mn = new Vector3F(lox, loy, loz); mx = new Vector3F(hix, hiy, hiz);
    }
}
