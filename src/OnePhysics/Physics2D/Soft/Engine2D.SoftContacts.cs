using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    // Contacts, friction, particle pairs and sleep of the soft step. The constraint kernels are in Engine2D.SoftConstraints.cs; the step works on float32 positions relative to a per-body origin.

    // ------------------------------------------------------------------ contacts with shapes

    private static bool BoxesOverlap(double[] box, int o, double x0, double y0, double x1, double y1) =>
        !(x0 > box[o + 2] || x1 < box[o] || y0 > box[o + 3] || y1 < box[o + 1]);

    private void DetectRange(SoftBodyData d, int from, int to, SoftTaskBuf buf)
    {
        float r = d.Radius;
        float spec = Opt.SpeculativeDistance;
        float fastSq = 0.25f * r * r;
        var px2 = d.X; var pp2 = d.XP;
        double ox = d.Ox, oy = d.Oy;
        for (int i = from; i < to; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            int k = 2 * i;
            double x = ox + px2[k], y = oy + px2[k + 1], px = ox + pp2[k], py = oy + pp2[k + 1];
            float mx = (float)(x - px), my = (float)(y - py);
            float m2 = mx * mx + my * my;
            bool fast = m2 > fastSq;
            // A particle that moves far in a substep may be pushed much farther by the constraints, so contacts are made speculatively for twice the distance it moved: a line that is not reached stays inactive.
            float specP = spec + 2f * float.Sqrt(m2);
            double reach = r + specP;
            double x0 = Math.Min(x, px) - reach, y0 = Math.Min(y, py) - reach;
            double x1 = Math.Max(x, px) + reach, y1 = Math.Max(y, py) + reach;
            int first = buf.Count;
            for (int c = d.CandStart; c < d.CandStart + d.CandCount; c++)
            {
                if (!BoxesOverlap(_cBox, 4 * c, x0, y0, x1, y1)) { continue; }
                CollideParticle(d, i, c, buf, first, fast, specP, x, y, px, py);
            }
        }
    }

    private static void AddContact(SoftTaskBuf buf, int first, in SoftContact sc)
    {
        if (buf.Count - first < 4)
        {
            GrowTo(ref buf.C, buf.Count + 1);
            buf.C[buf.Count++] = sc;
            return;
        }

        // Keep the four deepest contacts of the particle; ties keep the earlier one.
        var cs = buf.C;
        int worst = first;
        for (int j = first + 1; j < buf.Count; j++) { if (cs[j].C0 > cs[worst].C0) { worst = j; } }
        if (sc.C0 < cs[worst].C0) { cs[worst] = sc; }
    }

    private void TouchSoftBody(int b)
    {
        if (_rbStamp[b] == _softStamp) { return; }
        _rbStamp[b] = _softStamp;
        _rbJ[2 * b] = 0f; _rbJ[2 * b + 1] = 0f; _rbL[b] = 0f;
        _rbDp[2 * b] = 0f; _rbDp[2 * b + 1] = 0f; _rbDr[b] = 0f;
        GrowTo(ref _rbTouched, _rbTouchedCount + 1);
        _rbTouched[_rbTouchedCount++] = b;
    }

    // Records a contact from a normal pointing from the shape to the particle and the separation of the surfaces.
    private void RecordContact(SoftBodyData d, int i, int c, SoftTaskBuf buf, int first, float nx, float ny, float sep, double cx, double cy, double qx, double qy)
    {
        int s = _cShape[c], b = _cBody[c];
        var bodies = Core.Bodies;
        var sc = new SoftContact { Body = d, Particle = i, Rigid = -1, Nx = nx, Ny = ny, C0 = sep, X = (float)(cx - d.Ox), Y = (float)(cy - d.Oy), Mu = float.Sqrt(d.Def.Friction * Friction[s]) };
        if (_cMoving[c])
        {
            sc.Rigid = b;
            buf.Moving = true;
            sc.Coupled = bodies.IsDynamic(b) && bodies.IsAwake(b) && bodies.InverseMass[b] > 0f;
            // Contact point on the shape surface relative to the body's centre of mass at the end of the substep.
            BodyPoseAt(b, 1f, out double bx, out double by, out _, out _);
            sc.Rx = (float)(qx - bx); sc.Ry = (float)(qy - by);
        }

        AddContact(buf, first, sc);
    }

    private void CollideParticle(SoftBodyData d, int i, int c, SoftTaskBuf buf, int first, bool fast, float spec, double x, double y, double px, double py)
    {
        int s = _cShape[c];
        ref readonly var g = ref Geom[s];
        var q = new Rot2(_cRot[2 * c], _cRot[2 * c + 1]);
        double sx = _cPos[2 * c], sy = _cPos[2 * c + 1];
        if (!(Math.Abs(x - sx) < 2e6 && Math.Abs(y - sy) < 2e6 && Math.Abs(px - sx) < 2e6 && Math.Abs(py - sy) < 2e6)) { return; }

        if (fast)
        {
            // Continuous: sweep the particle from where it started the substep, relative to the shape's own motion in the substep, against the shape's pose at the end.
            double ex = sx - _cPos0[2 * c], ey = sy - _cPos0[2 * c + 1];
            double s0x = px - ex, s0y = py - ey;
            var motion = new Vec2((float)(x - s0x), (float)(y - s0y));
            var rel0 = q.InverseRotate(new Vec2((float)(s0x - sx), (float)(s0y - sy)));
            var hit = Distance2D.ShapeCast(in g, in d.ParticleShape, new Transform2(rel0, Rot2.Identity), q.InverseRotate(motion), 1f);
            // A zero normal means the particle already overlaps the shape at the start; the discrete test below handles that.
            if (hit.Hit && hit.Fraction < 1f && (hit.Normal.X != 0f || hit.Normal.Y != 0f))
            {
                var n = q.Rotate(hit.Normal);
                double hx = s0x + motion.X * hit.Fraction, hy = s0y + motion.Y * hit.Fraction;
                // The disc touches the surface at the hit, so the surface point is a radius back along the normal.
                double r = d.Radius;
                RecordContact(d, i, c, buf, first, n.X, n.Y, 0f, hx, hy, hx - n.X * r, hy - n.Y * r);
                return;
            }
        }

        var rel = q.InverseRotate(new Vec2((float)(x - sx), (float)(y - sy)));
        // A particle flung far away by a degenerate body (this step's constraints can overshoot) makes no contact; the next step reports it as out of bounds.
        if (!(Math.Abs(rel.X) < 4e6f && Math.Abs(rel.Y) < 4e6f)) { return; }
        var m = Collide2D.Collide(in g, in d.ParticleShape, new Transform2(rel, Rot2.Identity), spec);
        if (m.PointCount == 0) { return; }
        int best = 0;
        if (m.PointCount > 1 && m.Point1.Separation < m.Point0.Separation) { best = 1; }
        var p = best == 0 ? m.Point0 : m.Point1;
        var nw = q.Rotate(m.Normal);
        var pt = q.Rotate(p.Point);
        RecordContact(d, i, c, buf, first, nw.X, nw.Y, p.Separation, x, y, sx + pt.X, sy + pt.Y);
    }

    // Solves the contacts of one task (see BuildContactTasks), in the order listed.
    private void SolveContactTask(int task, float h, float invH)
    {
        var bodies = Core.Bodies;
        for (int ii = _ctStart[task]; ii < _ctStart[task + 1]; ii++)
        {
            ref SoftContact c = ref _sc[_ctIdx[ii]];
            var d = c.Body;
            int k = 2 * c.Particle;
            float wp = d.InvMass[c.Particle];
            float nx = c.Nx, ny = c.Ny;
            float wb = 0f, invM = 0f, invI = 0f, rn = 0f;
            int b = c.Rigid;
            if (c.Coupled)
            {
                invM = bodies.InverseMass[b];
                invI = bodies.InverseInertia[b];
                rn = c.Rx * ny - c.Ry * nx;
                wb = invM + invI * rn * rn;
            }

            float dx = d.X[k] - c.X, dy = d.X[k + 1] - c.Y;
            // The body has already moved away by wb times what earlier passes applied (c.Applied), which opens the gap as well.
            float cv = c.C0 + c.Nx * dx + c.Ny * dy + wb * c.Applied;
            if (!(cv < 0f)) { continue; }
            float wsum = wp + wb;
            if (!(wsum > 0f)) { continue; }
            float lam = -cv / wsum;
            c.Applied += lam;
            float sp = wp * lam;
            d.X[k] += nx * sp; d.X[k + 1] += ny * sp;
            if (c.Coupled)
            {
                float li = lam * invH;
                // The body is pushed the other way: momentum -n·lam/h, with the matching virtual displacement for the rest of the step.
                _rbJ[2 * b] -= nx * li; _rbJ[2 * b + 1] -= ny * li;
                _rbL[b] -= rn * li;
                float sb = lam * invM;
                _rbDp[2 * b] -= nx * sb; _rbDp[2 * b + 1] -= ny * sb;
                _rbDr[b] -= invI * rn * lam;
            }

            if (c.Mu > 0f) { Friction2(ref c, d, k, wp, lam, h, invH); }
        }
    }

    // Split-impulse style push-out: a particle that starts a substep inside a surface (a body placed resting on a floor) is only brought out at SoftPushOutSpeed, so the
    // neighbours it is connected to are not dragged along. The contact's constant term is raised by the depth that is not allowed to be removed this substep.
    private const float SoftPushOutSpeed = 1f;

    private void SoftenDepenetration(float h)
    {
        float allowed = SoftPushOutSpeed * h;
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            if (c.Coupled) { continue; }
            var d = c.Body;
            int k = 2 * c.Particle;
            if (d.InvMass[c.Particle] == 0f) { continue; }
            float dx = d.XP[k] - c.X, dy = d.XP[k + 1] - c.Y;
            float cp = c.C0 + c.Nx * dx + c.Ny * dy;
            if (cp < -allowed) { c.C0 += -cp - allowed; }
        }
    }

    // A particle that started the substep inside a surface is pushed out by the contact; that push is a correction of position, not a motion, so it must not become velocity.
    // The start of the substep is moved out along the normal by the depth it was in, for contacts not coupled to a dynamic body.
    private void ExcludeDepenetration()
    {
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            if (c.Coupled) { continue; }
            var d = c.Body;
            int k = 2 * c.Particle;
            if (d.InvMass[c.Particle] == 0f) { continue; }
            float dx = d.XP[k] - c.X, dy = d.XP[k + 1] - c.Y;
            float cp = c.C0 + c.Nx * dx + c.Ny * dy;
            if (!(cp < 0f)) { continue; }
            d.XP[k] -= c.Nx * cp; d.XP[k + 1] -= c.Ny * cp;
        }
    }

    // Restitution zero against surfaces that are not coupled to a dynamic body: a particle that rests on a contact keeps no velocity into it. (Against a coupled body the exchange of momentum
    // in the position solve already decides both velocities, and changing the particle's here would break the balance.)
    private void RemoveApproachVelocity()
    {
        var bodies = Core.Bodies;
        for (int ci = 0; ci < _scCount; ci++)
        {
            ref SoftContact c = ref _sc[ci];
            if (c.Coupled) { continue; }
            var d = c.Body;
            int k = 2 * c.Particle;
            if (d.InvMass[c.Particle] == 0f) { continue; }
            float dx = d.X[k] - c.X, dy = d.X[k + 1] - c.Y;
            float cv = c.C0 + c.Nx * dx + c.Ny * dy;
            if (!(cv < 0.1f * d.Radius)) { continue; }
            float svx = 0f, svy = 0f;
            if (c.Rigid >= 0)
            {
                var v = bodies.LinearVelocityOf(c.Rigid);
                float w = bodies.AngularVelocityOf(c.Rigid)[0];
                svx = v[0] - w * c.Ry; svy = v[1] + w * c.Rx;
            }

            float vn = (d.Vel[k] - svx) * c.Nx + (d.Vel[k + 1] - svy) * c.Ny;
            if (vn < 0f) { d.Vel[k] -= c.Nx * vn; d.Vel[k + 1] -= c.Ny * vn; }
        }
    }

    // Coulomb friction as a position correction (Müller et al. 2007 / Macklin et al. 2014): the particle's displacement relative to the surface it touches is removed up to mu times the normal correction.
    private void Friction2(ref SoftContact c, SoftBodyData d, int k, float wp, float normalLambda, float h, float invH)
    {
        var bodies = Core.Bodies;
        float nx = c.Nx, ny = c.Ny;
        float tx = -ny, ty = nx;
        float ux = d.X[k] - d.XP[k], uy = d.X[k + 1] - d.XP[k + 1];
        if (c.Rigid >= 0)
        {
            var v = bodies.LinearVelocityOf(c.Rigid);
            float w = bodies.AngularVelocityOf(c.Rigid)[0];
            float sx = v[0] - w * c.Ry, sy = v[1] + w * c.Rx;
            ux -= sx * h; uy -= sy * h;
        }

        float ut = ux * tx + uy * ty;
        float len = float.Abs(ut);
        if (!(len > 1e-12f)) { return; }
        float dir = ut > 0f ? 1f : -1f;
        float wt = wp, invM = 0f, invI = 0f, rt = 0f;
        if (c.Coupled)
        {
            int b = c.Rigid;
            invM = bodies.InverseMass[b];
            invI = bodies.InverseInertia[b];
            rt = c.Rx * ty - c.Ry * tx;
            wt += invM + invI * rt * rt;
        }

        if (!(wt > 0f)) { return; }
        float lt = len / wt;
        float cap = c.Mu * normalLambda;
        if (lt > cap) { lt = cap; }
        float sp = wp * lt * dir;
        d.X[k] -= tx * sp; d.X[k + 1] -= ty * sp;
        if (c.Coupled)
        {
            int b = c.Rigid;
            float li = lt * dir * invH;
            _rbJ[2 * b] += tx * li; _rbJ[2 * b + 1] += ty * li;
            _rbL[b] += rt * li;
            float sb = lt * dir * invM;
            _rbDp[2 * b] += tx * sb; _rbDp[2 * b + 1] += ty * sb;
            _rbDr[b] += invI * rt * lt * dir;
        }
    }

    private void ApplyRigidReactions()
    {
        if (_rbTouchedCount == 0) { return; }
        var bodies = Core.Bodies;
        Array.Sort(_rbTouched, 0, _rbTouchedCount);
        for (int t = 0; t < _rbTouchedCount; t++)
        {
            int b = _rbTouched[t];
            float invM = bodies.InverseMass[b];
            float invI = bodies.InverseInertia[b];
            var v = bodies.LinearVelocityOf(b);
            var w = bodies.AngularVelocityOf(b);
            v[0] += _rbJ[2 * b] * invM; v[1] += _rbJ[2 * b + 1] * invM;
            w[0] += invI * _rbL[b];
            bodies.SleepTime[b] = 0f;
            // The body ends the step where the particles met it: the displacement the reactions gave it is applied to its pose as well, so that no overlap is left to be pushed out next step
            // (an overlap resolved there would turn into velocity and give the body energy).
            var pos = bodies.PositionOf(b);
            pos[0] += WorldCore.Grid(_rbDp[2 * b]); pos[1] += WorldCore.Grid(_rbDp[2 * b + 1]);
            var rot = bodies.RotationOf(b);
            float dr = _rbDr[b];
            float qc = rot[0] - dr * rot[1], qs = rot[1] + dr * rot[0];
            float len = float.Sqrt(qc * qc + qs * qs);
            // A body with a damaged inverse inertia can overflow here; its rotation is then kept, so the pose stays a unit rotation and the next step reports the body.
            if (len > 0f && float.IsFinite(len)) { rot[0] = qc / len; rot[1] = qs / len; }
        }
    }

    // ------------------------------------------------------------------ particle against particle

    private bool SoftPairAllowed(SoftBodyDef a, SoftBodyDef b)
    {
        if (((LayerMatrix[a.Layer] >> b.Layer) & 1u) == 0) { return false; }
        if (a.GroupIndex == b.GroupIndex && a.GroupIndex != 0) { return a.GroupIndex > 0; }
        return (a.MaskBits & b.CategoryBits) != 0 && (a.CategoryBits & b.MaskBits) != 0;
    }

    private static long CellHash(long cx, long cy)
    {
        ulong h = (ulong)cx * 0x9E3779B97F4A7C15UL ^ (ulong)cy * 0xC2B2AE3D27D4EB4FUL;
        h ^= h >> 31; h *= 0x94D049BB133111EBUL; h ^= h >> 29;
        return (long)(h & ((1UL << 39) - 1));
    }

    // Finds the particle pairs that may touch during the step, once per step: particles are at their start-of-step positions, and the search reach is the contact distance plus the most either of them
    // can travel in the step (the same margin as the candidate shapes), so a pair that comes into reach in a later substep is already listed. Each substep tests the listed pairs again by their actual distance.
    private void BuildPairs(float travel)
    {
        _pairCount = 0;
        int total = 0;
        float rmax = 0f;
        bool any = _pairSoftCount > 1;
        for (int a = 0; a < _pairSoftCount; a++)
        {
            var d = _awakeSoft[a];
            total += d.N;
            if (d.Radius > rmax) { rmax = d.Radius; }
            if (d.Def.SelfCollision) { any = true; }
        }

        if (!any || total < 2) { return; }
        double reachExtra = 2.0 * travel;
        double cs = 6.0 * rmax + 2.0 * reachExtra, icsz = 1.0 / cs;
        var keys = _gridKeys;
        for (int g = 0; g < total; g++)
        {
            var d = _gData[g];
            int k = 2 * _gPart[g];
            long cx = (long)Math.Floor((d.Ox + d.X[k]) * icsz), cy = (long)Math.Floor((d.Oy + d.X[k + 1]) * icsz);
            keys[g] = (CellHash(cx, cy) << 24) | (long)g;
        }

        Array.Sort(keys, 0, total);
        // Runs of equal cell hash and a table from hash to run.
        int runs = 0;
        GrowTo(ref _gridCell, total + 1);
        for (int e = 0; e < total; e++)
        {
            long hsh = keys[e] >> 24;
            if (e == 0 || hsh != (keys[e - 1] >> 24)) { _gridCell[runs] = hsh; _gridStart[runs] = e; runs++; }
        }

        _gridStart[runs] = total;
        int size = 16;
        while (size < 2 * runs) { size <<= 1; }
        GrowTo(ref _gridTable, size);
        Array.Fill(_gridTable, -1, 0, size);
        int mask = size - 1;
        for (int u = 0; u < runs; u++)
        {
            int slot = (int)(_gridCell[u] & mask);
            while (_gridTable[slot] >= 0) { slot = (slot + 1) & mask; }
            _gridTable[slot] = u;
        }

        _pgIcsz = icsz; _pgMask = mask; _pgReach = (float)reachExtra; _pgTotal = total;
        int tasks = (total + PairChunk - 1) / PairChunk;
        EnsureTaskBufs(tasks);
        for (int t = 0; t < tasks; t++) { _tb[t].PairCount = 0; }
        Core.Executor.For(tasks, new PairProgram(this));
        // The chunks cover the particles in order, so joining them in task order lists the pairs in the order one pass over all particles would.
        for (int t = 0; t < tasks; t++)
        {
            var tb = _tb[t];
            GrowTo(ref _pairA, _pairCount + tb.PairCount); GrowTo(ref _pairB, _pairCount + tb.PairCount);
            for (int i = 0; i < tb.PairCount; i++) { _pairA[_pairCount] = (int)(tb.Pairs[i] >> 32); _pairB[_pairCount] = (int)(tb.Pairs[i] & 0xFFFFFFFFL); _pairCount++; }
        }
    }

    private double _pgIcsz;
    private int _pgMask, _pgTotal;
    private float _pgReach;
    internal static int PairChunk = 32;

    // Pairs for the particles g0..g1-1 against the grid built by BuildPairs: the particle's nearest 2 x 2 cells, other particles with a larger number only (each pair once).
    private void FindPairs(int g0, int g1, SoftTaskBuf buf)
    {
        var keys = _gridKeys;
        double icsz = _pgIcsz;
        int mask = _pgMask;
        float reachExtra = _pgReach;
        Span<long> seen = stackalloc long[4];
        for (int g = g0; g < g1; g++)
        {
            var di = _gData[g];
            int pi = _gPart[g];
            int k = 2 * pi;
            double fx = (di.Ox + di.X[k]) * icsz, fy = (di.Oy + di.X[k + 1]) * icsz;
            long cx = (long)Math.Floor(fx), cy = (long)Math.Floor(fy);
            int sx = fx - cx < 0.5 ? -1 : 1, sy = fy - cy < 0.5 ? -1 : 1;
            int nseen = 0;
            for (int q = 0; q < 4; q++)
            {
                long hsh = CellHash(cx + ((q & 1) != 0 ? sx : 0), cy + ((q & 2) != 0 ? sy : 0));
                bool dup = false;
                for (int u = 0; u < nseen; u++) { if (seen[u] == hsh) { dup = true; break; } }
                if (dup) { continue; }
                seen[nseen++] = hsh;
                int slot = (int)(hsh & mask);
                int run = -1;
                while (_gridTable[slot] >= 0)
                {
                    if (_gridCell[_gridTable[slot]] == hsh) { run = _gridTable[slot]; break; }
                    slot = (slot + 1) & mask;
                }

                if (run < 0) { continue; }
                for (int e = _gridStart[run]; e < _gridStart[run + 1]; e++)
                {
                    int j = (int)(keys[e] & 0xFFFFFF);
                    if (j <= g) { continue; }
                    var dj = _gData[j];
                    int pj = _gPart[j];
                    // The distance test comes first: it rejects most candidates and costs far less than the filters.
                    int kj = 2 * pj;
                    float ddx = SoftDelta(di, dj, k, kj, 0), ddy = SoftDelta(di, dj, k, kj, 1);
                    float rr = (di.Radius + dj.Radius) * 1.5f + reachExtra;
                    if (ddx * ddx + ddy * ddy >= rr * rr) { continue; }
                    if (di.InvMass[pi] == 0f && dj.InvMass[pj] == 0f) { continue; }
                    if (di == dj)
                    {
                        if (!di.Def.SelfCollision || Neighbours(di, pi, pj)) { continue; }
                    }
                    else if (!SoftPairAllowed(di.Def, dj.Def)) { continue; }

                    GrowTo(ref buf.Pairs, buf.PairCount + 1);
                    buf.Pairs[buf.PairCount++] = ((long)g << 32) | (long)j;
                }
            }
        }
    }

    // Component c of the vector from particle kj of dj to particle ki of di. Within one body the origins agree and the float difference is used; between bodies the difference of the double positions.
    private static float SoftDelta(SoftBodyData di, SoftBodyData dj, int ki, int kj, int c)
    {
        if (di == dj) { return di.X[ki + c] - di.X[kj + c]; }
        double oi = c == 0 ? di.Ox : di.Oy, oj = c == 0 ? dj.Ox : dj.Oy;
        return (float)((oi + di.X[ki + c]) - (oj + dj.X[kj + c]));
    }

    // Particles that share an edge or a hinge, or rest closer than their discs, never collide with each other.
    private static bool Neighbours(SoftBodyData d, int a, int b)
    {
        // The shorter of the two neighbour lists is scanned.
        int la = d.NbrStart[a + 1] - d.NbrStart[a], lb = d.NbrStart[b + 1] - d.NbrStart[b];
        int p = la <= lb ? a : b, q = la <= lb ? b : a;
        var nl = d.NbrList;
        for (int i = d.NbrStart[p], e = d.NbrStart[p + 1]; i < e; i++) { if (nl[i] == q) { return true; } }
        float dx = d.Rest[2 * a] - d.Rest[2 * b], dy = d.Rest[2 * a + 1] - d.Rest[2 * b + 1];
        float rr = 2f * d.Radius * 1.05f;
        return dx * dx + dy * dy < rr * rr;
    }

    private void SolvePairs()
    {
        SolveDiscPairs();
        if (_seCount > 0) { SolveSurfacePairs(); }
    }

    private void SolveDiscPairs()
    {
        for (int p = 0; p < _pairCount; p++)
        {
            int gi = _pairA[p], gj = _pairB[p];
            var di = _gData[gi]; var dj = _gData[gj];
            int pi = _gPart[gi], pj = _gPart[gj];
            float wi = di.InvMass[pi], wj = dj.InvMass[pj];
            float wsum = wi + wj;
            if (wsum == 0f) { continue; }
            int ki = 2 * pi, kj = 2 * pj;
            float dx = SoftDelta(di, dj, ki, kj, 0), dy = SoftDelta(di, dj, ki, kj, 1);
            float d2 = dx * dx + dy * dy;
            float rr = di.Radius + dj.Radius;
            if (!(d2 < rr * rr)) { continue; }
            float len = float.Sqrt(d2);
            float nx, ny;
            if (len > 1e-9f) { float il = 1f / len; nx = dx * il; ny = dy * il; }
            else { nx = 0f; ny = 1f; }
            float lam = (rr - len) / wsum;
            float si = wi * lam, sj = wj * lam;
            di.X[ki] += nx * si; di.X[ki + 1] += ny * si;
            dj.X[kj] -= nx * sj; dj.X[kj + 1] -= ny * sj;
        }
    }

    // ------------------------------------------------------------------ sleep

    private void UpdateSoftSleep(float dt)
    {
        float thr2 = Opt.SleepThreshold * Opt.SleepThreshold;
        for (int a = 0; a < _awakeSoftCount; a++)
        {
            var d = _awakeSoft[a];
            float vmax2 = d.StepSpeed2;

            if (!Opt.SleepEnabled || !d.Def.AllowSleep || d.HadMovingContact || d.TargetsMoved || !(vmax2 < thr2)) { d.SleepTime = 0f; continue; }
            d.SleepTime += dt;
            if (d.SleepTime >= Opt.TimeToSleep)
            {
                d.Asleep = true;
                Array.Clear(d.Vel);
            }
        }
    }
}
