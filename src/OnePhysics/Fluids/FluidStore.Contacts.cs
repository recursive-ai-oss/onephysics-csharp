using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Fluids;

internal sealed partial class FluidStore
{
    private const int MaxContactsPerParticle = 6;

    private readonly List<int> _shapeList = new();
    private int _candCount;
    private int[] _cShape = new int[16], _cBody = new int[16];
    private ulong[] _cAllow = new ulong[16];
    private FluidFrame[] _cFrame = new FluidFrame[16], _cFrame0 = new FluidFrame[16];
    private double[] _cBox = new double[96], _cBoxNow = new double[96];
    private FluidBody[] _bodies = new FluidBody[8];
    private int _bodyCount;
    private int[] _bodyMap = Array.Empty<int>();
    private FluidContact[] _cs = new FluidContact[64];
    private int _csCount;
    private int[] _coupled = new int[64];
    private int _coupledCount;
    private readonly FluidDeformables _def = new();
    private bool _lastSubstep;

    // grid of the deformables' particles (hashed buckets), in local coordinates
    private const int DefBuckets = 1 << 14;
    private int[] _defStart = Array.Empty<int>(), _defList = new int[16];
    private float[] _dfx = new float[16], _dfy = new float[16], _dfz = new float[16];
    private float _defCell, _defMinX, _defMinY, _defMinZ;
    private int[] _defFill = Array.Empty<int>();

    private float _spec;

    // ------------------------------------------------------------------ candidates

    private void PrepareCandidates(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        _shapeList.Clear();
        _candCount = 0;
        _bodyCount = 0;
        int hw = Host.BodyHighWater;
        if (_bodyMap.Length < hw) { _bodyMap = new int[Math.Max(hw, _bodyMap.Length * 2)]; }
        Host.GatherShapes(minX, minY, minZ, maxX, maxY, maxZ, _shapeList);
        for (int q = 0; q < _shapeList.Count; q++)
        {
            int s = _shapeList[q];
            ulong allow = 0;
            for (int f = 0; f < FluidHandles.HighWater; f++)
            {
                if (!FluidHandles.IsLive(f)) { continue; }
                var d = Fluids[f]!.Def;
                if (Host.ShapeCollides(s, d.Layer, d.GroupIndex, d.CategoryBits, d.MaskBits)) { allow |= 1UL << f; }
            }

            if (allow == 0) { continue; }
            int b = Host.OwnerOf(s);
            Host.LoadBody(b, out var pose);
            int bi = -1;
            if (pose.Dynamic || pose.Moves)
            {
                if (_bodyMap[b] == 0)
                {
                    if (_bodyCount == _bodies.Length) { Array.Resize(ref _bodies, _bodyCount * 2); }
                    pose.Body = b;
                    pose.StartX = pose.X; pose.StartY = pose.Y; pose.StartZ = pose.Z; pose.StartQx = pose.Qx; pose.StartQy = pose.Qy; pose.StartQz = pose.Qz; pose.StartQw = pose.Qw;
                    _bodies[_bodyCount++] = pose;
                    _bodyMap[b] = _bodyCount;
                }

                bi = _bodyMap[b] - 1;
            }

            int c = _candCount++;
            if (c == _cShape.Length)
            {
                int n = c * 2;
                Array.Resize(ref _cShape, n); Array.Resize(ref _cBody, n); Array.Resize(ref _cAllow, n); Array.Resize(ref _cFrame, n); Array.Resize(ref _cFrame0, n);
                Array.Resize(ref _cBox, 6 * n); Array.Resize(ref _cBoxNow, 6 * n);
            }

            _cShape[c] = s; _cBody[c] = bi; _cAllow[c] = allow;
            Host.ShapeFrame(s, in pose, out _cFrame[c]);
            _cFrame0[c] = _cFrame[c];
            Host.ShapeBox(s, in _cFrame[c], out _cBox[6 * c], out _cBox[6 * c + 1], out _cBox[6 * c + 2], out _cBox[6 * c + 3], out _cBox[6 * c + 4], out _cBox[6 * c + 5]);
            Array.Copy(_cBox, 6 * c, _cBoxNow, 6 * c, 6);
        }

        for (int i = 0; i < _bodyCount; i++) { _bodyMap[_bodies[i].Body] = 0; }
        // The map is cleared again at once: the candidate list holds the table index, so it is not needed after this.
    }

    /// <summary>Advances the virtual poses of the bodies by one substep (gravity, then the pose by the velocity) and recomputes the candidates' frames and boxes, the box being the union with the previous substep's.</summary>
    private void AdvanceBodies()
    {
        float h = _h;
        for (int i = 0; i < _bodyCount; i++)
        {
            ref var b = ref _bodies[i];
            if (!b.Moves) { continue; }
            // The body falls under gravity during the substep as it will in the rigid step; the reaction of the fluid is what holds it (the real body gets only that reaction).
            if (b.Dynamic) { b.Vx += b.Gx * h; b.Vy += b.Gy * h; b.Vz += b.Gz * h; }
            b.X += (double)b.Vx * h; b.Y += (double)b.Vy * h; b.Z += (double)b.Vz * h;
            float wx = b.Wx * h * 0.5f, wy = b.Wy * h * 0.5f, wz = b.Wz * h * 0.5f;
            float qx = b.Qx, qy = b.Qy, qz = b.Qz, qw = b.Qw;
            // q += 1/2 (w, 0) x q
            float nx = qw * wx + (wy * qz - wz * qy), ny = qw * wy + (wz * qx - wx * qz), nz = qw * wz + (wx * qy - wy * qx), nw = -(wx * qx + wy * qy + wz * qz);
            qx += nx; qy += ny; qz += nz; qw += nw;
            float len = float.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
            b.Qx = qx / len; b.Qy = qy / len; b.Qz = qz / len; b.Qw = qw / len;
        }

        for (int c = 0; c < _candCount; c++)
        {
            int bi = _cBody[c];
            _cFrame0[c] = _cFrame[c];
            if (bi >= 0 && _bodies[bi].Moves)
            {
                Host.ShapeFrame(_cShape[c], in _bodies[bi], out _cFrame[c]);
                Host.ShapeBox(_cShape[c], in _cFrame[c], out double a0, out double a1, out double a2, out double a3, out double a4, out double a5);
                int o = 6 * c;
                _cBox[o] = Math.Min(a0, _cBoxNow[o]); _cBox[o + 1] = Math.Min(a1, _cBoxNow[o + 1]); _cBox[o + 2] = Math.Min(a2, _cBoxNow[o + 2]);
                _cBox[o + 3] = Math.Max(a3, _cBoxNow[o + 3]); _cBox[o + 4] = Math.Max(a4, _cBoxNow[o + 4]); _cBox[o + 5] = Math.Max(a5, _cBoxNow[o + 5]);
                _cBoxNow[o] = a0; _cBoxNow[o + 1] = a1; _cBoxNow[o + 2] = a2; _cBoxNow[o + 3] = a3; _cBoxNow[o + 4] = a4; _cBoxNow[o + 5] = a5;
            }
        }
    }

    // ------------------------------------------------------------------ deformables

    private void BuildDeformableGrid()
    {
        int n = _def.Count;
        if (_dfx.Length < n) { _dfx = new float[Math.Max(n, _dfx.Length * 2)]; _dfy = new float[_dfx.Length]; _dfz = new float[_dfx.Length]; _defList = new int[_dfx.Length]; }
        float maxR = 0f;
        float mnx = float.MaxValue, mny = float.MaxValue, mnz = float.MaxValue;
        for (int i = 0; i < n; i++)
        {
            _dfx[i] = (float)(_def.X[i] - _ox); _dfy[i] = (float)(_def.Y[i] - _oy); _dfz[i] = (float)(_def.Z[i] - _oz);
            if (_def.Radius[i] > maxR) { maxR = _def.Radius[i]; }
            if (_dfx[i] < mnx) { mnx = _dfx[i]; } if (_dfy[i] < mny) { mny = _dfy[i]; } if (_dfz[i] < mnz) { mnz = _dfz[i]; }
        }

        _defCell = Math.Max(_k.H * 0.5f, 2f * maxR + Spacing);
        _defMinX = mnx; _defMinY = mny; _defMinZ = mnz;
        if (_defStart.Length == 0) { _defStart = new int[DefBuckets + 1]; _defFill = new int[DefBuckets]; }
        Array.Clear(_defStart);
        var bucket = _cursor.Length >= n ? _cursor : _cursor = new int[Math.Max(n, _cursor.Length * 2)];
        for (int i = 0; i < n; i++) { int b = DefBucket(_dfx[i], _dfy[i], _dfz[i]); bucket[i] = b; _defStart[b + 1]++; }
        for (int b = 0; b < DefBuckets; b++) { _defStart[b + 1] += _defStart[b]; }
        var fill = _defFill; Array.Clear(fill);
        for (int i = 0; i < n; i++) { int b = bucket[i]; _defList[_defStart[b] + fill[b]++] = i; }
    }

    private int DefCoord(float v, float min) => (int)Math.Min((long)Math.Floor((v - min) / _defCell), int.MaxValue / 2);

    private int DefBucket(float x, float y, float z) => DefBucketOf(DefCoord(x, _defMinX), DefCoord(y, _defMinY), DefCoord(z, _defMinZ));

    private static int DefBucketOf(int ix, int iy, int iz)
    {
        unchecked
        {
            uint h = (uint)ix * 73856093u ^ (uint)iy * 19349663u ^ (uint)iz * 83492791u;
            h ^= h >> 15; h *= 2246822519u; h ^= h >> 13;
            return (int)(h & (DefBuckets - 1));
        }
    }

    // ------------------------------------------------------------------ detection

    private struct DetectTask : IParallelBody
    {
        public FluidStore S;
        public void Run(int task)
        {
            var s = S;
            s.DetectRange(task, task * Chunk, Math.Min(s.Count, task * Chunk + Chunk));
        }
    }

    private static void AddContact(FluidTaskBuf buf, int first, in FluidContact c)
    {
        if (buf.Count - first < MaxContactsPerParticle)
        {
            if (buf.Count == buf.C.Length) { Array.Resize(ref buf.C, buf.Count * 2); }
            buf.C[buf.Count++] = c;
            return;
        }

        // Keep the deepest contacts; ties keep the earlier one.
        var cs = buf.C;
        int worst = first;
        for (int j = first + 1; j < buf.Count; j++) { if (cs[j].C0 > cs[worst].C0) { worst = j; } }
        if (c.C0 < cs[worst].C0) { cs[worst] = c; }
    }

    private void DetectRange(int task, int from, int to)
    {
        var buf = _tb[task];
        buf.Count = 0;
        float rc = 0.5f * Spacing;
        bool haveDef = _def.Count > 0;
        for (int i = from; i < to; i++)
        {
            _cFirst[i] = -1; _cCount[i] = 0;
            int f = PFluid[i];
            var fl = Fluids[f]!.Def;
            float x = _x[i], y = _y[i], z = _z[i];
            float mx = x - _x0[i], my = y - _y0[i], mz = z - _z0[i];
            float m2 = mx * mx + my * my + mz * mz;
            float motion = float.Sqrt(m2);
            // A particle that moves far in a substep may be pushed farther by the constraints, so contacts are made speculatively for twice the distance it moved.
            float spec = _spec + 2f * motion;
            double reach = rc + spec;
            double px = _ox + x, py = _oy + y, pz = _oz + z;
            // The box covers where the particle started the substep as well as where it is.
            double bx0 = px - reach - (mx > 0f ? mx : 0.0), by0 = py - reach - (my > 0f ? my : 0.0), bz0 = pz - reach - (mz > 0f ? mz : 0.0);
            double bx1 = px + reach + (mx < 0f ? -mx : 0.0), by1 = py + reach + (my < 0f ? -my : 0.0), bz1 = pz + reach + (mz < 0f ? -mz : 0.0);
            int first = buf.Count;
            bool fast = motion > 0.5f * rc;
            for (int c = 0; c < _candCount; c++)
            {
                if (((_cAllow[c] >> f) & 1UL) == 0) { continue; }
                int o = 6 * c;
                if (bx0 > _cBox[o + 3] || bx1 < _cBox[o] || by0 > _cBox[o + 4] || by1 < _cBox[o + 1] || bz0 > _cBox[o + 5] || bz1 < _cBox[o + 2]) { continue; }
                CollideWith(c, i, buf, first, fast, spec, rc, px, py, pz, mx, my, mz, fl.Friction, x, y, z);
            }

            if (haveDef && (fl.CollideWithSoftBodies || fl.CollideWithRods)) { CollideDeformables(i, buf, first, fl, rc, spec, x, y, z); }
        }
    }

    private void CollideWith(int c, int i, FluidTaskBuf buf, int first, bool fast, float spec, float rc, double px, double py, double pz, float mx, float my, float mz, float fluidFriction, float lx, float ly, float lz)
    {
        int s = _cShape[c];
        ref readonly var fr = ref _cFrame[c];
        FluidHit hit;
        if (fast)
        {
            ref readonly var f0 = ref _cFrame0[c];
            double ex = fr.X - f0.X, ey = fr.Y - f0.Y, ez = fr.Z - f0.Z;
            // Sweep from where the particle started the substep, relative to the shape's own motion, against the shape at the end.
            double s0x = px - mx - ex, s0y = py - my - ey, s0z = pz - mz - ez;
            if (Host.SweepSphere(s, in fr, s0x, s0y, s0z, (float)(px - s0x), (float)(py - s0y), (float)(pz - s0z), rc, out hit))
            {
                var sc = Make(c, i, hit.Nx, hit.Ny, hit.Nz, 0f, (float)(hit.Px - _ox), (float)(hit.Py - _oy), (float)(hit.Pz - _oz), hit.Px - hit.Nx * rc, hit.Py - hit.Ny * rc, hit.Pz - hit.Nz * rc, fluidFriction * hit.Friction);
                AddContact(buf, first, in sc);
                return;
            }
        }

        if (Host.CollideSphere(s, in fr, px, py, pz, rc, spec, out hit))
        {
            var sc = Make(c, i, hit.Nx, hit.Ny, hit.Nz, hit.Separation, lx, ly, lz, hit.Px, hit.Py, hit.Pz, fluidFriction * hit.Friction);
            AddContact(buf, first, in sc);
        }
    }

    private FluidContact Make(int c, int i, float nx, float ny, float nz, float sep, float cx, float cy, float cz, double qx, double qy, double qz, float mu2)
    {
        var k = new FluidContact { P = i, Body = _cBody[c], Def = -1, Nx = nx, Ny = ny, Nz = nz, C0 = sep, X = cx, Y = cy, Z = cz, Mu = float.Sqrt(mu2) };
        if (k.Body >= 0)
        {
            ref readonly var b = ref _bodies[k.Body];
            k.Coupled = b.Dynamic;
            // The contact point on the shape, relative to the body's centre of mass at the end of the substep.
            float rx = (float)(qx - b.X), ry = (float)(qy - b.Y), rz = (float)(qz - b.Z);
            k.Rx = rx; k.Ry = ry; k.Rz = rz;
            // Velocity of the surface there.
            k.Sx = b.Vx + (b.Wy * rz - b.Wz * ry); k.Sy = b.Vy + (b.Wz * rx - b.Wx * rz); k.Sz = b.Vz + (b.Wx * ry - b.Wy * rx);
        }

        return k;
    }

    private void CollideDeformables(int i, FluidTaskBuf buf, int first, ParticleFluidDef fl, float rc, float spec, float x, float y, float z)
    {
        int ix = DefCoord(x, _defMinX), iy = DefCoord(y, _defMinY), iz = DefCoord(z, _defMinZ);
        Span<int> seen = stackalloc int[27];
        int ns = 0;
        for (int cz = Dim == 3 ? -1 : 0; cz <= (Dim == 3 ? 1 : 0); cz++)
        {
            for (int cy = -1; cy <= 1; cy++)
            {
                for (int cx = -1; cx <= 1; cx++)
                {
                    int bk = DefBucketOf(ix + cx, iy + cy, iz + cz);
                    bool dup = false;
                    for (int q = 0; q < ns; q++) { if (seen[q] == bk) { dup = true; break; } }
                    if (dup) { continue; }
                    seen[ns++] = bk;
                    for (int a = _defStart[bk]; a < _defStart[bk + 1]; a++)
                    {
                        int d = _defList[a];
                        bool rod = (_def.Owner[d] >> 24) == 1;
                        if (rod ? !fl.CollideWithRods : !fl.CollideWithSoftBodies) { continue; }
                        float dx = x - _dfx[d], dy = y - _dfy[d], dz = z - _dfz[d];
                        float rr = rc + _def.Radius[d];
                        float reach = rr + spec;
                        float d2 = dx * dx + dy * dy + dz * dz;
                        if (!(d2 < reach * reach)) { continue; }
                        float dist = float.Sqrt(d2);
                        float nx, ny, nz;
                        if (dist > 1e-9f) { float inv = 1f / dist; nx = dx * inv; ny = dy * inv; nz = dz * inv; }
                        else { nx = 0f; ny = 1f; nz = 0f; }
                        var k = new FluidContact
                        {
                            P = i, Body = -1, Def = d, Nx = nx, Ny = ny, Nz = Dim == 3 ? nz : 0f, C0 = dist - rr, X = x, Y = y, Z = z, Mu = float.Sqrt(fl.Friction * 0.5f),
                            Coupled = _def.InvMass[d] > 0f, Sx = _def.Vx[d], Sy = _def.Vy[d], Sz = _def.Vz[d],
                        };
                        AddContact(buf, first, in k);
                    }
                }
            }
        }
    }

    /// <summary>Joins the task buffers in task order, so the list is the one a single pass would make.</summary>
    private void JoinContacts()
    {
        _csCount = 0; _coupledCount = 0;
        for (int t = 0; t < _taskCount; t++)
        {
            var tb = _tb[t];
            if (_cs.Length < _csCount + tb.Count) { Array.Resize(ref _cs, Math.Max(_csCount + tb.Count, _cs.Length * 2)); }
            for (int q = 0; q < tb.Count; q++)
            {
                int idx = _csCount++;
                _cs[idx] = tb.C[q];
                int p = tb.C[q].P;
                if (_cFirst[p] < 0) { _cFirst[p] = idx; }
                _cCount[p]++;
                if (tb.C[q].Coupled)
                {
                    if (_coupledCount == _coupled.Length) { Array.Resize(ref _coupled, _coupledCount * 2); }
                    _coupled[_coupledCount++] = idx;
                }
            }
        }

        if (_lastSubstep)
        {
            float area = Dim == 3 ? Spacing * Spacing : Spacing;
            for (int q = 0; q < _csCount; q++)
            {
                int bi = _cs[q].Body;
                if (bi >= 0) { _bodies[bi].WetArea += area; }
            }
        }
    }

    // ------------------------------------------------------------------ solving

    // A contact with something that takes no reaction (static and kinematic surfaces, sleeping bodies treated as static): the particle is pushed out; Coulomb friction removes its displacement along the surface up to mu times the push
    // (Müller et al. 2007; Macklin et al. 2014).
    private void SolveStatic(int i, int first, int count)
    {
        float h = _h;
        for (int q = first; q < first + count; q++)
        {
            ref FluidContact c = ref _cs[q];
            if (c.Coupled) { continue; }
            float cv = c.C0 + c.Nx * (_x[i] - c.X) + c.Ny * (_y[i] - c.Y) + c.Nz * (_z[i] - c.Z);
            if (!(cv < 0f)) { continue; }
            float corr = Math.Min(-cv, 0.5f * _k.H);
            _x[i] += c.Nx * corr; _y[i] += c.Ny * corr; _z[i] += c.Nz * corr;
            if (c.Mu > 0f)
            {
                float ux = _x[i] - _x0[i] - c.Sx * h, uy = _y[i] - _y0[i] - c.Sy * h, uz = _z[i] - _z0[i] - c.Sz * h;
                float un = ux * c.Nx + uy * c.Ny + uz * c.Nz;
                float tx = ux - c.Nx * un, ty = uy - c.Ny * un, tz = uz - c.Nz * un;
                float tl = float.Sqrt(tx * tx + ty * ty + tz * tz);
                if (tl > 1e-12f)
                {
                    float lim = c.Mu * corr;
                    float k = tl <= lim ? 1f : lim / tl;
                    _x[i] -= tx * k; _y[i] -= ty * k; _z[i] -= tz * k;
                }
            }
        }
    }

    // Contacts that give a reaction, solved serially in list order (ascending particle) so the momentum each body receives sums in a fixed order. The same two-body position constraint as XPBD (Macklin et al. 2016)
    // and the unified-particle contact (Macklin et al. 2014): the particle moves by w_p * lambda, the body takes the impulse lambda / h at the contact point, which keeps momentum exactly.
    private void SolveCoupled()
    {
        float h = _h, invH = 1f / h;
        for (int qq = 0; qq < _coupledCount; qq++)
        {
            ref FluidContact c = ref _cs[_coupled[qq]];
            int i = c.P;
            float wp = _invMass[PFluid[i]];
            float nx = c.Nx, ny = c.Ny, nz = c.Nz;
            float wb, rnx = 0f, rny = 0f, rnz = 0f, iwx = 0f, iwy = 0f, iwz = 0f;
            if (c.Def < 0)
            {
                ref var b = ref _bodies[c.Body];
                rnx = c.Ry * nz - c.Rz * ny; rny = c.Rz * nx - c.Rx * nz; rnz = c.Rx * ny - c.Ry * nx;
                iwx = b.Iw0 * rnx + b.Iw1 * rny + b.Iw2 * rnz; iwy = b.Iw1 * rnx + b.Iw3 * rny + b.Iw4 * rnz; iwz = b.Iw2 * rnx + b.Iw4 * rny + b.Iw5 * rnz;
                wb = b.InvMass + rnx * iwx + rny * iwy + rnz * iwz;
            }
            else { wb = _def.InvMass[c.Def]; }

            // The other side has already moved away by wb times what earlier passes applied, which opens the gap as well.
            float cv = c.C0 + nx * (_x[i] - c.X) + ny * (_y[i] - c.Y) + nz * (_z[i] - c.Z) + wb * c.Applied;
            if (!(cv < 0f)) { continue; }
            float wsum = wp + wb;
            if (!(wsum > 0f)) { continue; }
            float lam = -cv / wsum;
            // Limit one pass's correction to half a kernel radius, as for static contacts.
            float maxLam = 0.5f * _k.H / wp;
            if (lam > maxLam) { lam = maxLam; }
            c.Applied += lam;
            float sp = wp * lam;
            _x[i] += nx * sp; _y[i] += ny * sp; _z[i] += nz * sp;
            float li = lam * invH;
            if (c.Def < 0)
            {
                ref var b = ref _bodies[c.Body];
                b.Jx -= nx * li; b.Jy -= ny * li; b.Jz -= nz * li;
                b.Lx -= rnx * li; b.Ly -= rny * li; b.Lz -= rnz * li;
            }
            else
            {
                int d = c.Def;
                _def.Jx[d] -= nx * li; _def.Jy[d] -= ny * li; _def.Jz[d] -= nz * li;
            }

            if (c.Mu > 0f) { FrictionCoupled(ref c, i, wp, wb, lam, h, invH, rnx, rny, rnz); }
        }
    }

    private void FrictionCoupled(ref FluidContact c, int i, float wp, float wb, float normalLambda, float h, float invH, float rnx, float rny, float rnz)
    {
        float nx = c.Nx, ny = c.Ny, nz = c.Nz;
        float ux = _x[i] - _x0[i] - c.Sx * h, uy = _y[i] - _y0[i] - c.Sy * h, uz = _z[i] - _z0[i] - c.Sz * h;
        float un = ux * nx + uy * ny + uz * nz;
        float tx = ux - nx * un, ty = uy - ny * un, tz = uz - nz * un;
        float tl = float.Sqrt(tx * tx + ty * ty + tz * tz);
        if (!(tl > 1e-12f)) { return; }
        float dx = tx / tl, dy = ty / tl, dz = tz / tl;
        // Effective inverse mass along the tangent.
        float wbt;
        float qx = 0f, qy = 0f, qz = 0f;
        if (c.Def < 0)
        {
            ref var b = ref _bodies[c.Body];
            qx = c.Ry * dz - c.Rz * dy; qy = c.Rz * dx - c.Rx * dz; qz = c.Rx * dy - c.Ry * dx;
            float ix2 = b.Iw0 * qx + b.Iw1 * qy + b.Iw2 * qz, iy2 = b.Iw1 * qx + b.Iw3 * qy + b.Iw4 * qz, iz2 = b.Iw2 * qx + b.Iw4 * qy + b.Iw5 * qz;
            wbt = b.InvMass + qx * ix2 + qy * iy2 + qz * iz2;
        }
        else { wbt = c.Def >= 0 ? _def.InvMass[c.Def] : 0f; }

        float lim = c.Mu * (normalLambda * wp);
        float want = tl;
        float lamT = want / (wp + wbt);
        float maxLam = lim / wp;
        if (lamT > maxLam) { lamT = maxLam; }
        if (!(lamT > 0f)) { return; }
        _x[i] -= dx * wp * lamT; _y[i] -= dy * wp * lamT; _z[i] -= dz * wp * lamT;
        float li = lamT * invH;
        if (c.Def < 0)
        {
            ref var b = ref _bodies[c.Body];
            b.Jx += dx * li; b.Jy += dy * li; b.Jz += dz * li;
            b.Lx += qx * li; b.Ly += qy * li; b.Lz += qz * li;
        }
        else
        {
            int d = c.Def;
            _def.Jx[d] += dx * li; _def.Jy[d] += dy * li; _def.Jz[d] += dz * li;
        }
    }

    /// <summary>Gives the bodies the momentum of the substep's contacts: their virtual velocities change now (the next substep sees them), and the whole step's sum is applied to the real bodies at the end.</summary>
    private void UpdateBodies()
    {
        for (int i = 0; i < _bodyCount; i++)
        {
            ref var b = ref _bodies[i];
            if (b.Jx == 0f && b.Jy == 0f && b.Jz == 0f && b.Lx == 0f && b.Ly == 0f && b.Lz == 0f) { continue; }
            if (b.Dynamic)
            {
                b.Vx += b.Jx * b.InvMass; b.Vy += b.Jy * b.InvMass; b.Vz += b.Jz * b.InvMass;
                b.Wx += b.Iw0 * b.Lx + b.Iw1 * b.Ly + b.Iw2 * b.Lz; b.Wy += b.Iw1 * b.Lx + b.Iw3 * b.Ly + b.Iw4 * b.Lz; b.Wz += b.Iw2 * b.Lx + b.Iw4 * b.Ly + b.Iw5 * b.Lz;
            }

            b.Tx += b.Jx; b.Ty += b.Jy; b.Tz += b.Jz; b.Ux += b.Lx; b.Uy += b.Ly; b.Uz += b.Lz;
            b.Jx = 0f; b.Jy = 0f; b.Jz = 0f; b.Lx = 0f; b.Ly = 0f; b.Lz = 0f;
        }
    }
}
