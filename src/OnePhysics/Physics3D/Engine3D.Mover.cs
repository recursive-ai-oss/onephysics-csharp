using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    private const double MoverBoundary = 2097152.0; // 2^21
    private const float MoverMinMove = 1e-6f;
    private const int MoverSlideIterations = 5;

    private struct PlaneMeta
    {
        public int Slot;
        public double Px, Py, Pz;
    }

    private struct MoverCtx
    {
        public ConvexShape3D Capsule;
        public QuaternionF Rot;         // capsule orientation: local y axis to the up direction
        public Vector3F Up;
        public float Radius, HalfLength, CosSlope, Step, Skin, Range, MaxPush;
        public uint Mask;
        public int IgnoreBody;
    }

    private struct MoverHitRec
    {
        public int Slot;
        public double Px, Py, Pz;
        public float Nx, Ny, Nz;
    }

    private int[] _mvCand = new int[32];
    private int _mvCandCount;
    private CollisionPlane3D[] _mvPlanes = new CollisionPlane3D[32];
    private PlaneMeta[] _mvMeta = new PlaneMeta[32];
    private int _mvPlaneCount;
    private MoverHitRec[] _mvHits = new MoverHitRec[16];
    private int _mvHitCount;
    private bool _mvWall, _mvCeiling;
    private readonly NarrowScratch _mvScratch = new();
    private readonly QueryScratch _mvQuery = new();

    private static Vector3D MvAdd(Vector3D p, Vector3F v) => new(p.X + (double)v.X, p.Y + (double)v.Y, p.Z + (double)v.Z);

    private static Vector3D MvAdd(Vector3D p, Vector3F v, float s) => new(p.X + (double)(s * v.X), p.Y + (double)(s * v.Y), p.Z + (double)(s * v.Z));

    private static Vector3F MvDiff(Vector3D a, Vector3D b) => new((float)(a.X - b.X), (float)(a.Y - b.Y), (float)(a.Z - b.Z));

    /// <summary>The rotation that takes the local y axis to the unit vector <paramref name="u"/> (shortest arc; half-vector form, so no trigonometry).</summary>
    internal static QuaternionF AlignY(Vector3F u)
    {
        float w = 1f + u.Y;
        if (w < 1e-6f) { return new QuaternionF(1f, 0f, 0f, 0f); }
        return new QuaternionF(u.Z, 0f, -u.X, w).Normalized();
    }

    internal static Vector3F UnitUp(Vector3F up) => up.Normalized();

    // ------------------------------------------------------------------ candidates and planes

    private struct CandidateCollector : IOverlapCallback
    {
        public Engine3D E;
        public uint Mask;
        public int IgnoreBody;

        public bool Report(int key, int userId)
        {
            int s = userId;
            if (E.Core.Shapes.Owner[s] == IgnoreBody || E.IsSensor(s) || ((Mask >> E.ShapeLayer[s]) & 1u) == 0) { return true; }
            if (E._mvCandCount == E._mvCand.Length) { Array.Resize(ref E._mvCand, E._mvCandCount * 2); }
            E._mvCand[E._mvCandCount++] = s;
            return true;
        }
    }

    /// <summary>Collects the shapes whose boxes overlap <paramref name="box"/> (moving shapes' stored boxes may lag behind their bodies, so those are looked up with the query pad), ascending by slot.</summary>
    private void MvGather(in Aabb3D box, in MoverCtx c)
    {
        _mvCandCount = 0;
        var col = new CandidateCollector { E = this, Mask = c.Mask, IgnoreBody = c.IgnoreBody };
        Bp.QueryAabb(in box, ref col, BodyTypeMask.Static);
        double pad = QueryPad();
        var grown = new Aabb3D(box.MinX - pad, box.MinY - pad, box.MinZ - pad, box.MaxX + pad, box.MaxY + pad, box.MaxZ + pad);
        Bp.QueryAabb(in grown, ref col, BodyTypeMask.Kinematic | BodyTypeMask.Dynamic);
        Array.Sort(_mvCand, 0, _mvCandCount);
    }

    private static Aabb3D MvBox(Vector3D p, Vector3F d, in MoverCtx c, float grow)
    {
        double ex = (double)(Math.Abs(c.Up.X) * c.HalfLength + c.Radius + grow);
        double ey = (double)(Math.Abs(c.Up.Y) * c.HalfLength + c.Radius + grow);
        double ez = (double)(Math.Abs(c.Up.Z) * c.HalfLength + c.Radius + grow);
        return new Aabb3D(p.X - ex + (double)Math.Min(0f, d.X), p.Y - ey + (double)Math.Min(0f, d.Y), p.Z - ez + (double)Math.Min(0f, d.Z),
            p.X + ex + (double)Math.Max(0f, d.X), p.Y + ey + (double)Math.Max(0f, d.Y), p.Z + ez + (double)Math.Max(0f, d.Z));
    }

    private void MvAddPlane(Vector3F n, float sep, double px, double py, double pz, int slot, in MoverCtx c)
    {
        int body = Core.Shapes.Owner[slot];
        bool other = IsCharacterBody(body);
        bool walkable = Vector3F.Dot(n, c.Up) >= c.CosSlope && !other;
        float separation = walkable ? sep : sep - c.Skin;
        // Overlaps are undone gradually (at most half a radius per move), so a character spawned inside geometry or pushed by a moving obstacle is never thrown across a thin wall.
        // Another character is an obstacle that never pushes and is never pushed out of: two characters that overlap stay put instead of shoving each other away.
        float push = other ? 0f : c.MaxPush;
        // One plane per direction: a mesh gives a plane per triangle, and parallel planes are implied by the nearest one.
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (_mvPlanes[i].PushLimit != push || Vector3F.Dot(_mvPlanes[i].Normal, n) <= 0.99995f) { continue; }
            if (separation < _mvPlanes[i].Separation)
            {
                _mvPlanes[i].Separation = separation;
                _mvMeta[i] = new PlaneMeta { Slot = slot, Px = px, Py = py, Pz = pz };
            }

            return;
        }

        if (_mvPlaneCount == _mvPlanes.Length)
        {
            Array.Resize(ref _mvPlanes, _mvPlaneCount * 2);
            Array.Resize(ref _mvMeta, _mvPlaneCount * 2);
        }

        int k = _mvPlaneCount++;
        _mvPlanes[k] = new CollisionPlane3D { Normal = n, Separation = separation, PushLimit = push };
        _mvMeta[k] = new PlaneMeta { Slot = slot, Px = px, Py = py, Pz = pz };
    }

    private void MvManifoldPlane(in Manifold3D m, double ox, double oy, double oz, QuaternionF rs, int slot, in MoverCtx c)
    {
        if (m.PointCount == 0) { return; }
        float sep = m[0].Separation;
        Vector3F pt = m[0].Point;
        for (int k = 1; k < m.PointCount; k++)
        {
            if (m[k].Separation < sep) { sep = m[k].Separation; pt = m[k].Point; }
        }

        Vector3F n = rs.Rotate(m.Normal);
        Vector3F wp = rs.Rotate(pt);
        MvAddPlane(n, sep, ox + (double)wp.X, oy + (double)wp.Y, oz + (double)wp.Z, slot, in c);
    }

    /// <summary>One plane per direction around the capsule: the contact normal and smallest separation of every gathered shape (every triangle, every compound child). Planes on ground within the slope limit keep no skin (the character rests on them); all others hold the skin distance.</summary>
    private void MvPlanes(Vector3D p, in MoverCtx c)
    {
        _mvPlaneCount = 0;
        var gen = NarrowScratch.Gen;
        for (int i = 0; i < _mvCandCount; i++)
        {
            int s = _mvCand[i];
            ShapeFrame(s, out double ox, out double oy, out double oz, out QuaternionF rs);
            var inv = rs.Conjugate();
            var lo = inv.Rotate(new Vector3F((float)(p.X - ox), (float)(p.Y - oy), (float)(p.Z - oz)));
            var rel = new RigidTransformF(inv * c.Rot, lo);
            var g = Geoms[GeomOf[s]]!;
            if (g.IsConvex)
            {
                var m = gen.Collide(g.Convex, c.Capsule, rel, c.Range);
                MvManifoldPlane(in m, ox, oy, oz, rs, s, in c);
            }
            else if (g.Compound != null)
            {
                NarrowScratch.Children.Clear();
                gen.CollideCompoundConvex(g.Compound, c.Capsule, rel, NarrowScratch.Children, c.Range);
                foreach (var cc in NarrowScratch.Children)
                {
                    var m = cc.Manifold;
                    MvManifoldPlane(in m, ox, oy, oz, rs, s, in c);
                }
            }
            else
            {
                NarrowScratch.Tris.Clear();
                g.Soup!.GetCandidates(c.Capsule, rel, c.Range, NarrowScratch.Tris, NarrowScratch.Ids);
                for (int t = 0; t < NarrowScratch.Tris.Count; t++)
                {
                    var tri = NarrowScratch.Tris[t];
                    var tshape = NarrowScratch.TriBuf.Set(tri.V0, tri.V1, tri.V2);
                    var m = gen.Collide(tshape, c.Capsule, rel, c.Range);
                    if (m.PointCount == 0) { continue; }
                    // The same internal-edge rule as the contact stage: a contact on a seam between flat neighbours whose normal leans away from the face is the neighbour's job.
                    float d = Vector3F.Dot(m.Normal, tri.Normal);
                    if (d >= -0.1f && d < GhostCosine)
                    {
                        bool ghost = false;
                        for (int k = 0; k < m.PointCount && !ghost; k++) { ghost = IsGhostRegion(m[k].Point, in tri); }
                        if (ghost) { continue; }
                    }

                    MvManifoldPlane(in m, ox, oy, oz, rs, s, in c);
                }
            }
        }
    }

    // ------------------------------------------------------------------ casts

    /// <summary>
    /// Sweeps the capsule by <paramref name="d"/> and returns the nearest shape it would touch. A shape that is already touching and not approached by at least 1% of the sweep does not count (the planes
    /// handle touching shapes); ties between equal fractions go to the lower shape slot.
    /// </summary>
    private bool MvCast(Vector3D p, Vector3F d, in MoverCtx c, out float fraction, out Vector3F normal, out int slot)
    {
        fraction = 1f; normal = default; slot = -1;
        float len = d.Length();
        if (!(len > MoverMinMove)) { return false; }
        var box = MvBox(p, d, in c, 1e-3f);
        MvGather(in box, in c);
        bool any = false;
        var q = new QueryParams { Kind = QueryKind.Cast, X = p.X, Y = p.Y, Z = p.Z, Tx = d.X, Ty = d.Y, Tz = d.Z, Mover = c.Capsule, MoverRot = c.Rot };
        for (int i = 0; i < _mvCandCount; i++)
        {
            int s = _mvCand[i];
            if (!Narrow(in q, QueryMode.Closest, s, _mvQuery, out RawHit h)) { continue; }
            var wn = new Vector3F(h.Nx, h.Ny, h.Nz);
            if (wn.X == 0f && wn.Y == 0f && wn.Z == 0f) { continue; }
            if (-Vector3F.Dot(d, wn) < 0.01f * len) { continue; }
            if (any && !(h.Fraction < fraction)) { continue; }
            any = true;
            fraction = h.Fraction;
            normal = wn;
            slot = s;
        }

        return any;
    }

    // ------------------------------------------------------------------ slide

    private void MvRecordHits(in MoverCtx c)
    {
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (!(_mvPlanes[i].Push > 1e-5f)) { continue; }
            float nu = Vector3F.Dot(_mvPlanes[i].Normal, c.Up);
            if (nu < c.CosSlope)
            {
                if (nu > -c.CosSlope) { _mvWall = true; } else { _mvCeiling = true; }
            }

            int slot = _mvMeta[i].Slot;
            bool seen = false;
            for (int h = 0; h < _mvHitCount; h++) { if (_mvHits[h].Slot == slot) { seen = true; break; } }
            if (seen) { continue; }
            if (_mvHitCount == _mvHits.Length) { Array.Resize(ref _mvHits, _mvHitCount * 2); }
            var n = _mvPlanes[i].Normal;
            _mvHits[_mvHitCount++] = new MoverHitRec { Slot = slot, Px = _mvMeta[i].Px, Py = _mvMeta[i].Py, Pz = _mvMeta[i].Pz, Nx = n.X, Ny = n.Y, Nz = n.Z };
        }
    }

    /// <summary>
    /// Collide and slide: gather the planes around the capsule, clip the wanted movement against them, sweep along the clipped movement, and if something new stops the sweep early repeat from there
    /// with the rest of the movement (at most five rounds).
    /// </summary>
    private void MvSlide(ref Vector3D p, Vector3F d, in MoverCtx c)
    {
        var r = d;
        for (int it = 0; it < MoverSlideIterations; it++)
        {
            if (!(r.Length() > MoverMinMove)) { break; }
            var box = MvBox(p, r, in c, c.Range + 1e-3f);
            MvGather(in box, in c);
            MvPlanes(p, in c);
            var v = Mover3D.Solve(_mvPlanes.AsSpan(0, _mvPlaneCount), r, 50, 1e-5f);
            MvRecordHits(in c);
            float len = v.Length();
            if (!(len > MoverMinMove)) { break; }
            if (!MvCast(p, v, in c, out float f, out _, out _))
            {
                p = MvAdd(p, v);
                break;
            }

            p = MvAdd(p, v, f);
            if (f * len <= MoverMinMove) { break; }
            r = v * (1f - f);
        }
    }

    // ------------------------------------------------------------------ the move

    private bool MvStepAttempt(Vector3D p0, Vector3F dir, float forward, float progressNow, in MoverCtx c, out Vector3D landedAt)
    {
        landedAt = p0;
        // Lift as far as the step offset allows (a ceiling stops it), move forward, then drop onto whatever is below: walkable ground no higher than the step offset above where we started.
        float raise = c.Step;
        if (MvCast(p0, c.Up * c.Step, in c, out float fu, out _, out _)) { raise = fu * c.Step; }
        if (raise < 1e-3f) { return false; }
        var p = MvAdd(p0, c.Up, raise);
        int hits = _mvHitCount;
        MvSlide(ref p, dir * forward, in c);
        var delta = MvDiff(p, p0);
        float progress = Vector3F.Dot(delta, dir);
        if (!(progress > progressNow + 1e-3f)) { _mvHitCount = hits; return false; }
        // The slide may have ridden up a rounded edge; the drop must end no higher than the step offset above where we started.
        float drop = Vector3F.Dot(delta, c.Up) + 2e-3f;
        if (!(drop > 0f) || !MvCast(p, c.Up * -drop, in c, out float fd, out Vector3F n, out int dropSlot) || Vector3F.Dot(n, c.Up) < c.CosSlope || IsCharacterBody(Core.Shapes.Owner[dropSlot]))
        {
            _mvHitCount = hits;
            return false;
        }

        var landed = MvAdd(p, c.Up, -(fd * drop));
        // A step goes up: landing at the height we started from is just a slide past the end of a wall.
        float gain = Vector3F.Dot(MvDiff(landed, p0), c.Up);
        if (gain > c.Step + 1e-3f || gain < 0.5f * c.Skin) { _mvHitCount = hits; return false; }
        landedAt = landed;
        return true;
    }

    private void MvHorizontal(ref Vector3D p, Vector3F h, bool tryStep, in MoverCtx c)
    {
        var p0 = p;
        bool wallBefore = _mvWall;
        _mvWall = false;
        int hitsBefore = _mvHitCount;
        MvSlide(ref p, h, in c);
        bool wall = _mvWall;
        _mvWall = wallBefore | wall;
        float want = h.Length();
        var dir = h / want;
        float progress = Vector3F.Dot(MvDiff(p, p0), dir);
        if (!tryStep || !wall || c.Step <= 0f || !(progress < want - 1e-4f)) { return; }
        int keep = _mvHitCount;
        _mvHitCount = hitsBefore;
        // First try the movement as asked; a short move leaves the capsule's rounded bottom too far behind the step's edge to rest on it (the contact there leans over the slope limit),
        // so a second try moves far enough forward that the bottom sits on the edge at half the slope limit's lean.
        float sinSlope = float.Sqrt(Math.Max(0f, 1f - c.CosSlope * c.CosSlope));
        float reach = c.Radius + c.Skin - 0.5f * c.Radius * sinSlope;
        if (MvStepAttempt(p0, dir, want, progress, in c, out var np) || (want < reach && MvStepAttempt(p0, dir, reach, progress, in c, out np)))
        {
            p = np;
            _mvWall = wallBefore;
            return;
        }

        // No step: keep the plain slide's result (its hits were dropped when the attempt started; restore them).
        _mvHitCount = keep;
    }

    private void MvVertical(ref Vector3D p, float v, bool wasGrounded, bool userUp, in MoverCtx c)
    {
        if (v > 0f)
        {
            MvSlide(ref p, c.Up * v, in c);
            return;
        }

        float down = -v;
        // Resolve overlaps first and see whether we already stand on walkable ground: then the downward movement is simply absorbed (no sliding down a gentle slope, no sinking).
        MvGather(MvBox(p, default, in c, c.Range + 1e-3f), in c);
        MvPlanes(p, in c);
        var r = Mover3D.Solve(_mvPlanes.AsSpan(0, _mvPlaneCount), default, 50, 1e-5f);
        bool resting = false;
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            var n = _mvPlanes[i].Normal;
            if (Vector3F.Dot(n, c.Up) >= c.CosSlope && !IsCharacterBody(Core.Shapes.Owner[_mvMeta[i].Slot]) && _mvPlanes[i].Separation + Vector3F.Dot(r, n) <= 1e-3f)
            {
                resting = true;
                // Pressing into the ground counts as a hit with it.
                if (down > MoverMinMove && _mvPlanes[i].Push <= 1e-5f) { _mvPlanes[i].Push = 1e-3f; }
            }
        }

        MvRecordHits(in c);
        p = MvAdd(p, r);
        if (resting) { return; }

        float castLen = wasGrounded && !userUp && c.Step > down ? c.Step : down;
        if (!(castLen > MoverMinMove)) { return; }
        if (!MvCast(p, c.Up * -castLen, in c, out float f, out Vector3F hn, out _))
        {
            MvSlide(ref p, c.Up * -down, in c);
            return;
        }

        float dist = f * castLen;
        if (Vector3F.Dot(hn, c.Up) >= c.CosSlope)
        {
            p = MvAdd(p, c.Up, -dist);
            return;
        }

        if (dist >= down)
        {
            p = MvAdd(p, c.Up, -down);
            return;
        }

        // A steep surface within reach of this move: go down to it and slide along it with what is left.
        p = MvAdd(p, c.Up, -dist);
        MvSlide(ref p, c.Up * -(down - dist), in c);
    }

    private static MoverCtx MakeMoverCtx(in CharacterDef def, int ignoreBody)
    {
        var up = UnitUp(def.Up);
        float halfLength = 0.5f * def.Height - def.Radius;
        double cosDeg = Math64.SinCos((double)def.SlopeLimit * (Math.PI / 180.0)).Item2;
        return new MoverCtx
        {
            Capsule = ConvexShape3D.Capsule(halfLength, def.Radius), Rot = AlignY(up), Up = up,
            Radius = def.Radius, HalfLength = halfLength, CosSlope = (float)cosDeg, Step = def.StepOffset, Skin = def.Skin, Range = 2f * def.Skin, MaxPush = 0.5f * def.Radius,
            Mask = def.LayerMask, IgnoreBody = ignoreBody,
        };
    }

    /// <summary>Finds what the capsule at <paramref name="p"/> stands on: the planes within 3 skins, then the best walkable one (largest cosine, lowest plane on ties) or else the best steep one that leans up towards the capsule.</summary>
    private GroundState MvDetectGround(Vector3D p, Vector3F settle, ref MoverCtx c, out Vector3F normal, out int slot, out double gx, out double gy, out double gz, out int planesUsed)
    {
        c.Range = 3f * c.Skin;
        MvGather(MvBox(p, default, in c, c.Range + 1e-3f), in c);
        MvPlanes(p, in c);
        planesUsed = _mvPlaneCount;
        normal = default; slot = -1; gx = gy = gz = 0.0;
        var state = GroundState.InAir;
        float best = c.CosSlope, bestSteep = 0f;
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            var n = _mvPlanes[i].Normal;
            float nu = Vector3F.Dot(n, c.Up);
            float gap = _mvPlanes[i].Separation + Vector3F.Dot(settle, n);
            bool isChar = IsCharacterBody(Core.Shapes.Owner[_mvMeta[i].Slot]);
            if (nu >= c.CosSlope && !isChar)
            {
                if (gap > 2f * c.Skin) { continue; }
                if (state != GroundState.OnGround || nu > best) { best = nu; state = GroundState.OnGround; normal = n; slot = _mvMeta[i].Slot; gx = _mvMeta[i].Px; gy = _mvMeta[i].Py; gz = _mvMeta[i].Pz; }
            }
            else if (state != GroundState.OnGround && nu > 0.01f && nu < c.CosSlope && !isChar)
            {
                // Steep planes hold the skin distance: their separation already has it taken off.
                if (gap + c.Skin > 2f * c.Skin) { continue; }
                if (state != GroundState.OnSteepGround || nu > bestSteep) { bestSteep = nu; state = GroundState.OnSteepGround; normal = n; slot = _mvMeta[i].Slot; gx = _mvMeta[i].Px; gy = _mvMeta[i].Py; gz = _mvMeta[i].Pz; }
            }
        }

        return state;
    }

    public MoveResult3D MoveCapsule(PhysicsWorld3D world, in CharacterDef def, ref MoverState3D state, Vector3F displacement, float dt, Body ignore, Span<MoverHit3D> hits)
    {
        if (!float.IsFinite(displacement.X) || !float.IsFinite(displacement.Y) || !float.IsFinite(displacement.Z) || displacement.LengthSquared() > 1e10f || !float.IsFinite(displacement.LengthSquared()))
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The displacement must be finite and at most 100000 long.");
        }

        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        var pos = state.Position;
        if (!double.IsFinite(pos.X) || !double.IsFinite(pos.Y) || !double.IsFinite(pos.Z) || Math.Abs(pos.X) > MoverBoundary || Math.Abs(pos.Y) > MoverBoundary || Math.Abs(pos.Z) > MoverBoundary)
        {
            throw new ArgumentOutOfRangeException(nameof(state), pos, "The mover position " + pos + " must be finite and within +-2^21 of the world origin.");
        }

        int ignoreBody = -1;
        if (ignore.World != null) { ignoreBody = Core.Resolve(ignore.RawHandle); }
        var c = MakeMoverCtx(in def, ignoreBody);

        bool wasGrounded = state.Ground == GroundState.OnGround;
        // Ride the ground: the body underneath moves in this step, and so does the character with it.
        Vector3F carry = default;
        if (wasGrounded && state.GroundBody.World != null && state.GroundBody.IsValid)
        {
            carry = BodyVelocityAt(state.GroundBody.RawHandle.Index, pos.X, pos.Y, pos.Z) * dt;
        }

        var total = displacement + carry;
        float up = Vector3F.Dot(total, c.Up);
        var horizontal = total - c.Up * up;
        bool userUp = Vector3F.Dot(displacement, c.Up) > 0f;
        var p = pos;
        _mvHitCount = 0; _mvWall = false; _mvCeiling = false;
        if (horizontal.LengthSquared() > 0f) { MvHorizontal(ref p, horizontal, wasGrounded, in c); }
        MvVertical(ref p, up, wasGrounded, userUp, in c);

        if (Math.Abs(p.X) > MoverBoundary || Math.Abs(p.Y) > MoverBoundary || Math.Abs(p.Z) > MoverBoundary)
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The move would take the character beyond +-2^21 from the world origin.");
        }

        // Settle: undo any overlap with the skin zone the last movement ended in (and report those surfaces as hits), then find the ground: the best surface within the slope limit just under the capsule.
        c.Range = 3f * c.Skin;
        MvGather(MvBox(p, default, in c, c.Range + 1e-3f), in c);
        MvPlanes(p, in c);
        var settle = Mover3D.Solve(_mvPlanes.AsSpan(0, _mvPlaneCount), default, 50, 1e-5f);
        MvRecordHits(in c);
        p = MvAdd(p, settle);
        var ground = GroundState.InAir;
        Vector3F gn = default;
        int gslot = -1;
        if (!userUp)
        {
            // The planes from the settle stage are still current; the detection looks at them again with the settle shift applied to their gaps.
            ground = MvDetectGround(p, default, ref c, out gn, out gslot, out _, out _, out _, out _);
        }

        var flags = CharacterContact.None;
        if (_mvWall) { flags |= CharacterContact.Sides; }
        if (_mvCeiling) { flags |= CharacterContact.Above; }
        if (ground == GroundState.OnGround) { flags |= CharacterContact.Below; }

        // Pushes: a dynamic body the character walked into is brought up to the walking speed along the push direction, with a bounded force.
        int n = _mvHitCount;
        var bodies = Core.Bodies;
        if (def.PushStrength > 0f && def.MaxPushForce > 0f)
        {
            for (int i = 0; i < n; i++)
            {
                ref MoverHitRec h = ref _mvHits[i];
                int hb = Core.Shapes.Owner[h.Slot];
                if (bodies.TypeOf(hb) != BodyType.Dynamic || IsCharacterBody(hb)) { continue; }
                var hn = new Vector3F(h.Nx, h.Ny, h.Nz);
                if (Vector3F.Dot(hn, c.Up) >= c.CosSlope) { continue; }
                float want = -Vector3F.Dot(displacement, hn) / dt;
                var bv = bodies.LinearVelocityOf(hb);
                float have = -(bv[0] * hn.X + bv[1] * hn.Y + bv[2] * hn.Z);
                float dv = (want - have) * def.PushStrength;
                if (!(dv > 0f)) { continue; }
                float impulse = Math.Min(dv * Mass[hb], def.MaxPushForce * dt);
                new Body(world, Core.HandleOfBody(hb)).ApplyImpulseAt(hn * -impulse, new Vector3D(h.Px, h.Py, h.Pz));
            }
        }

        // Standing on a dynamic body presses it down with the character's weight.
        if (ground == GroundState.OnGround && gslot >= 0)
        {
            int gb = Core.Shapes.Owner[gslot];
            if (bodies.TypeOf(gb) == BodyType.Dynamic && bodies.IsAwake(gb) && !IsCharacterBody(gb))
            {
                new Body(world, Core.HandleOfBody(gb)).ApplyImpulse(Opt.Gravity * (def.Mass * dt));
            }
        }

        int written = 0;
        for (int i = 0; i < n && written < hits.Length; i++)
        {
            ref MoverHitRec h = ref _mvHits[i];
            hits[written++] = new MoverHit3D(new Shape(world, Core.HandleOfShape(h.Slot)), new Vector3D(h.Px, h.Py, h.Pz), new Vector3F(h.Nx, h.Ny, h.Nz));
        }

        state.Position = p;
        state.Ground = ground;
        state.GroundNormal = ground != GroundState.InAir ? gn : default;
        state.GroundBody = ground == GroundState.OnGround ? new Body(world, Core.HandleOfBody(Core.Shapes.Owner[gslot])) : default;
        return new MoveResult3D(flags, MvDiff(p, pos), written);
    }

    /// <summary>Velocity of a point attached to a body: its linear velocity plus the rotation about the centre of mass.</summary>
    internal Vector3F BodyVelocityAt(int body, double px, double py, double pz)
    {
        var bodies = Core.Bodies;
        if (bodies.TypeOf(body) == BodyType.Static) { return default; }
        var v = bodies.LinearVelocityOf(body);
        var w = bodies.AngularVelocityOf(body);
        var p = bodies.PositionOf(body);
        var r = new Vector3F((float)(px - p[0]), (float)(py - p[1]), (float)(pz - p[2]));
        var wr = Vector3F.Cross(new Vector3F(w[0], w[1], w[2]), r);
        return new Vector3F(v[0] + wr.X, v[1] + wr.Y, v[2] + wr.Z);
    }
}
