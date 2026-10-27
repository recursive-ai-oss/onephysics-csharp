using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Broadphase;
using RecursiveAI.OnePhysics.Collision2D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private const float MoverMaxPushOut = 100f;
    private const double MoverBoundary = 2097152.0; // 2^21
    private const float MoverMinMove = 1e-6f;
    private const int MoverSlideIterations = 5;

    private struct PlaneMeta
    {
        public int Slot;
        public double Px, Py;
    }

    private struct MoverCtx
    {
        public Shape2D Capsule;
        public float Radius, HalfLength, CosSlope, Step, Skin, Range;
        public uint Mask;
        public int IgnoreBody;
    }

    private struct MoverHitRec
    {
        public int Slot;
        public double Px, Py;
        public float Nx, Ny;
    }

    private int[] _mvCand = new int[32];
    private int _mvCandCount;
    private CollisionPlane2D[] _mvPlanes = new CollisionPlane2D[32];
    private PlaneMeta[] _mvMeta = new PlaneMeta[32];
    private int _mvPlaneCount;
    private MoverHitRec[] _mvHits = new MoverHitRec[16];
    private int _mvHitCount;
    private bool _mvWall, _mvCeiling;

    // ------------------------------------------------------------------ candidates and planes

    private struct CandidateCollector : IOverlapCallback
    {
        public Engine2D E;
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
    private void MvGather(in Aabb2D box, in MoverCtx c)
    {
        _mvCandCount = 0;
        var col = new CandidateCollector { E = this, Mask = c.Mask, IgnoreBody = c.IgnoreBody };
        Bp.QueryAabb(in box, ref col, BodyTypeMask.Static);
        double pad = QueryPad();
        var grown = new Aabb2D(box.MinX - pad, box.MinY - pad, box.MaxX + pad, box.MaxY + pad);
        Bp.QueryAabb(in grown, ref col, BodyTypeMask.Kinematic | BodyTypeMask.Dynamic);
        Array.Sort(_mvCand, 0, _mvCandCount);
    }

    private static Aabb2D MvBox(double x, double y, float dx, float dy, in MoverCtx c, float grow)
    {
        double hx = c.Radius + grow, hy = c.HalfLength + c.Radius + grow;
        return new Aabb2D(x - hx + (double)Math.Min(0f, dx), y - hy + (double)Math.Min(0f, dy), x + hx + (double)Math.Max(0f, dx), y + hy + (double)Math.Max(0f, dy));
    }

    /// <summary>One plane per gathered shape that has a manifold point within the range: the contact normal and the smallest separation. Planes on ground within the slope limit keep no skin (the character rests on them); all others hold the skin distance.</summary>
    private void MvPlanes(double x, double y, in MoverCtx c)
    {
        _mvPlaneCount = 0;
        if (_mvPlanes.Length < _mvCandCount) { _mvPlanes = new CollisionPlane2D[Math.Max(_mvCandCount, _mvPlanes.Length * 2)]; _mvMeta = new PlaneMeta[_mvPlanes.Length]; }
        var bodies = Core.Bodies;
        for (int i = 0; i < _mvCandCount; i++)
        {
            int s = _mvCand[i];
            int b = Core.Shapes.Owner[s];
            FrameOf(b, out double ox, out double oy, out Rot2 bq);
            var xf = Transform2.InvMul(new Transform2(default, bq), new Transform2(new Vec2((float)(x - ox), (float)(y - oy)), Rot2.Identity));
            var m = Collide2D.Collide(in Geom[s], in c.Capsule, xf, c.Range);
            if (m.PointCount == 0) { continue; }
            float sep = m.Point0.Separation;
            Vec2 pt = m.Point0.Point;
            if (m.PointCount > 1 && m.Point1.Separation < sep) { sep = m.Point1.Separation; pt = m.Point1.Point; }
            Vec2 n = bq.Rotate(m.Normal);
            Vec2 wp = bq.Rotate(pt);
            bool walkable = n.Y >= c.CosSlope;
            int k = _mvPlaneCount++;
            _mvPlanes[k] = new CollisionPlane2D
            {
                Normal = new Vector2F(n.X, n.Y),
                Separation = walkable ? sep : sep - c.Skin,
                PushLimit = bodies.TypeOf(b) == BodyType.Dynamic ? 0f : MoverMaxPushOut,
            };
            _mvMeta[k] = new PlaneMeta { Slot = s, Px = ox + (double)wp.X, Py = oy + (double)wp.Y };
        }
    }

    // ------------------------------------------------------------------ casts

    /// <summary>
    /// Sweeps the capsule by (dx, dy) and returns the nearest shape it would touch. A shape that is already touching and not approached by at least 1% of the sweep does not count (the planes
    /// handle touching shapes); ties between equal fractions go to the lower shape slot.
    /// </summary>
    private bool MvCast(double x, double y, float dx, float dy, in MoverCtx c, out float fraction, out Vec2 normal, out int slot, out double px, out double py)
    {
        fraction = 1f; normal = default; slot = -1; px = 0; py = 0;
        float len = float.Sqrt(dx * dx + dy * dy);
        if (!(len > MoverMinMove)) { return false; }
        var box = MvBox(x, y, dx, dy, c, 1e-3f);
        MvGather(in box, in c);
        bool any = false;
        for (int i = 0; i < _mvCandCount; i++)
        {
            int s = _mvCand[i];
            int b = Core.Shapes.Owner[s];
            FrameOf(b, out double ox, out double oy, out Rot2 bq);
            var xf = Transform2.InvMul(new Transform2(default, bq), new Transform2(new Vec2((float)(x - ox), (float)(y - oy)), Rot2.Identity));
            Vec2 lt = bq.InverseRotate(new Vec2(dx, dy));
            var o = Distance2D.ShapeCast(in Geom[s], in c.Capsule, xf, lt, any ? fraction : 1f);
            if (!o.Hit) { continue; }
            Vec2 wn = bq.Rotate(o.Normal);
            if (wn.X == 0f && wn.Y == 0f) { continue; }
            if (-(dx * wn.X + dy * wn.Y) < 0.01f * len) { continue; }
            if (any && !(o.Fraction < fraction)) { continue; }
            any = true;
            fraction = o.Fraction;
            normal = wn;
            slot = s;
            Vec2 wp = bq.Rotate(o.Point);
            px = ox + (double)wp.X; py = oy + (double)wp.Y;
        }

        return any;
    }

    // ------------------------------------------------------------------ slide

    private void MvRecordHits(in MoverCtx c)
    {
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (!(_mvPlanes[i].Push > 1e-5f)) { continue; }
            float ny = _mvPlanes[i].Normal.Y;
            if (ny < c.CosSlope)
            {
                if (ny > -c.CosSlope) { _mvWall = true; } else { _mvCeiling = true; }
            }

            int slot = _mvMeta[i].Slot;
            bool seen = false;
            for (int h = 0; h < _mvHitCount; h++) { if (_mvHits[h].Slot == slot) { seen = true; break; } }
            if (seen) { continue; }
            if (_mvHitCount == _mvHits.Length) { Array.Resize(ref _mvHits, _mvHitCount * 2); }
            _mvHits[_mvHitCount++] = new MoverHitRec { Slot = slot, Px = _mvMeta[i].Px, Py = _mvMeta[i].Py, Nx = _mvPlanes[i].Normal.X, Ny = ny };
        }
    }

    /// <summary>
    /// Collide and slide: gather the planes around the capsule, clip the wanted movement against them, sweep along the clipped movement, and if something new stops the sweep early repeat from there
    /// with the rest of the movement (at most five rounds).
    /// </summary>
    private void MvSlide(ref double x, ref double y, float dx, float dy, in MoverCtx c)
    {
        float rx = dx, ry = dy;
        for (int it = 0; it < MoverSlideIterations; it++)
        {
            if (!(float.Sqrt(rx * rx + ry * ry) > MoverMinMove)) { break; }
            var box = MvBox(x, y, rx, ry, c, c.Range + 1e-3f);
            MvGather(in box, in c);
            MvPlanes(x, y, in c);
            var v = Mover2D.Solve(_mvPlanes.AsSpan(0, _mvPlaneCount), rx, ry, 50, 1e-5f);
            MvRecordHits(in c);
            float len = float.Sqrt(v.X * v.X + v.Y * v.Y);
            if (!(len > MoverMinMove)) { break; }
            if (!MvCast(x, y, v.X, v.Y, c, out float f, out _, out _, out _, out _))
            {
                x += (double)v.X; y += (double)v.Y;
                break;
            }

            x += (double)(f * v.X); y += (double)(f * v.Y);
            if (f * len <= MoverMinMove) { break; }
            rx = (1f - f) * v.X; ry = (1f - f) * v.Y;
        }
    }

    // ------------------------------------------------------------------ the move

    private bool MvStepAttempt(double x0, double y0, float sign, float forward, float progressNow, in MoverCtx c, out double nx, out double ny)
    {
        nx = x0; ny = y0;
        // Lift as far as the step offset allows (a ceiling stops it), move forward, then drop onto whatever is below: walkable ground no lower than where we started.
        float raise = c.Step;
        if (MvCast(x0, y0, 0f, c.Step, c, out float fu, out _, out _, out _, out _)) { raise = fu * c.Step; }
        if (raise < 1e-3f) { return false; }
        double px = x0, py = y0 + (double)raise;
        int hits = _mvHitCount;
        MvSlide(ref px, ref py, sign * forward, 0f, c);
        float progress = (float)(px - x0) * sign;
        if (!(progress > progressNow + 1e-3f)) { _mvHitCount = hits; return false; }
        // The slide may have ridden up a rounded edge; the drop must still end no higher than the step offset above where we started and no lower than where we started.
        float drop = (float)(py - y0) + 2e-3f;
        if (!MvCast(px, py, 0f, -drop, c, out float fd, out Vec2 n, out _, out _, out _) || n.Y < c.CosSlope) { _mvHitCount = hits; return false; }
        double landed = py - (double)(fd * drop);
        if (landed - y0 > (double)c.Step + 1e-3) { _mvHitCount = hits; return false; }
        nx = px; ny = landed;
        return true;
    }

    private void MvHorizontal(ref double x, ref double y, float h, bool tryStep, in MoverCtx c)
    {
        double x0 = x, y0 = y;
        bool wallBefore = _mvWall;
        _mvWall = false;
        int hitsBefore = _mvHitCount;
        MvSlide(ref x, ref y, h, 0f, c);
        bool wall = _mvWall;
        _mvWall = wallBefore | wall;
        float sign = h < 0f ? -1f : 1f;
        float progress = (float)(x - x0) * sign;
        float want = Math.Abs(h);
        if (!tryStep || !wall || c.Step <= 0f || !(progress < want - 1e-4f)) { return; }
        int keep = _mvHitCount;
        _mvHitCount = hitsBefore;
        // First try the movement as asked; a short move may leave the capsule's rounded bottom behind the step's edge, so a second try moves about a radius forward.
        if (MvStepAttempt(x0, y0, sign, want, progress, c, out double nx, out double ny) || (want < c.Radius && MvStepAttempt(x0, y0, sign, c.Radius + c.Skin, progress, c, out nx, out ny)))
        {
            x = nx; y = ny;
            _mvWall = wallBefore;
            return;
        }

        // No step: keep the plain slide's result (its hits were dropped when the attempt started; restore them).
        _mvHitCount = keep;
    }

    private void MvVertical(ref double x, ref double y, float v, bool wasGrounded, bool userUp, in MoverCtx c)
    {
        if (v > 0f)
        {
            MvSlide(ref x, ref y, 0f, v, c);
            return;
        }

        float down = -v;
        // Resolve overlaps first and see whether we already stand on walkable ground: then the downward movement is simply absorbed (no sliding down a gentle slope, no sinking).
        MvGather(MvBox(x, y, 0f, 0f, c, c.Range + 1e-3f), c);
        MvPlanes(x, y, c);
        var r = Mover2D.Solve(_mvPlanes.AsSpan(0, _mvPlaneCount), 0f, 0f, 50, 1e-5f);
        bool resting = false;
        for (int i = 0; i < _mvPlaneCount; i++)
        {
            if (_mvPlanes[i].Normal.Y >= c.CosSlope && _mvPlanes[i].Separation + r.X * _mvPlanes[i].Normal.X + r.Y * _mvPlanes[i].Normal.Y <= 1e-3f)
            {
                resting = true;
                // Pressing into the ground counts as a hit with it.
                if (down > MoverMinMove && _mvPlanes[i].Push <= 1e-5f) { _mvPlanes[i].Push = 1e-3f; }
            }
        }

        MvRecordHits(in c);

        x += (double)r.X; y += (double)r.Y;
        if (resting) { return; }

        float castLen = wasGrounded && !userUp && c.Step > down ? c.Step : down;
        if (!(castLen > MoverMinMove)) { return; }
        if (!MvCast(x, y, 0f, -castLen, c, out float f, out Vec2 n, out _, out _, out _))
        {
            MvSlide(ref x, ref y, 0f, -down, c);
            return;
        }

        float dist = f * castLen;
        if (n.Y >= c.CosSlope)
        {
            y -= (double)dist;
            return;
        }

        if (dist >= down)
        {
            y -= (double)down;
            return;
        }

        // A steep surface within reach of this move: go down to it and slide along it with what is left.
        y -= (double)dist;
        MvSlide(ref x, ref y, 0f, -(down - dist), c);
    }

    private static float Sq(float v) => v * v;

    public MoveResult2D MoveCapsule(PhysicsWorld2D world, in CharacterDef def, ref MoverState2D state, Vector2F displacement, float dt, Body ignore, Span<MoverHit2D> hits)
    {
        if (!float.IsFinite(displacement.X) || !float.IsFinite(displacement.Y) || Sq(displacement.X) + Sq(displacement.Y) > 1e10f)
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The displacement must be finite and at most 100000 long.");
        }

        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        var pos = state.Position;
        if (!double.IsFinite(pos.X) || !double.IsFinite(pos.Y) || Math.Abs(pos.X) > MoverBoundary || Math.Abs(pos.Y) > MoverBoundary)
        {
            throw new ArgumentOutOfRangeException(nameof(state), pos, "The mover position " + pos + " must be finite and within +-2^21 of the world origin.");
        }

        int ignoreBody = -1;
        if (ignore.World != null) { ignoreBody = Core.Resolve(ignore.Handle); }
        float halfLength = 0.5f * def.Height - def.Radius;
        double cosDeg = Math64.SinCos((double)def.SlopeLimit * (Math.PI / 180.0)).Item2;
        var c = new MoverCtx
        {
            Capsule = Shape2D.CreateCapsule(new Vec2(0f, -halfLength), new Vec2(0f, halfLength), def.Radius),
            Radius = def.Radius, HalfLength = halfLength, CosSlope = (float)cosDeg, Step = def.StepOffset, Skin = def.Skin, Range = 2f * def.Skin,
            Mask = def.LayerMask, IgnoreBody = ignoreBody,
        };

        bool wasGrounded = state.IsGrounded;
        // Ride the ground: the body underneath moves in this step, and so does the character with it.
        Vec2 carry = default;
        if (wasGrounded && state.GroundBody.World != null && state.GroundBody.IsValid)
        {
            carry = BodyVelocityAt(state.GroundBody.Handle.Index, pos.X, pos.Y) * dt;
        }

        float dx = displacement.X + carry.X, dy = displacement.Y + carry.Y;
        bool userUp = displacement.Y > 0f;
        double x = pos.X, y = pos.Y;
        _mvHitCount = 0; _mvWall = false; _mvCeiling = false;
        if (dx != 0f) { MvHorizontal(ref x, ref y, dx, wasGrounded, c); }
        MvVertical(ref x, ref y, dy, wasGrounded, userUp, c);

        if (Math.Abs(x) > MoverBoundary || Math.Abs(y) > MoverBoundary)
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The move would take the character beyond +-2^21 from the world origin.");
        }

        // Settle: undo any overlap with the skin zone the last movement ended in (and report those surfaces as hits), then find the ground: the best surface within the slope limit just under the capsule.
        bool grounded = false;
        float gnx = 0f, gny = 0f;
        int gslot = -1;
        c.Range = 3f * c.Skin;
        MvGather(MvBox(x, y, 0f, 0f, c, c.Range + 1e-3f), c);
        MvPlanes(x, y, c);
        var settle = Mover2D.Solve(_mvPlanes.AsSpan(0, _mvPlaneCount), 0f, 0f, 50, 1e-5f);
        MvRecordHits(in c);
        x += (double)settle.X; y += (double)settle.Y;
        if (!userUp)
        {
            float best = c.CosSlope;
            for (int i = 0; i < _mvPlaneCount; i++)
            {
                float ny = _mvPlanes[i].Normal.Y;
                float sep = _mvPlanes[i].Separation + settle.X * _mvPlanes[i].Normal.X + settle.Y * ny;
                if (ny < c.CosSlope || sep > 2f * c.Skin) { continue; }
                if (!grounded || ny > best) { best = ny; grounded = true; gnx = _mvPlanes[i].Normal.X; gny = ny; gslot = _mvMeta[i].Slot; }
            }
        }

        var flags = CharacterContact.None;
        if (_mvWall) { flags |= CharacterContact.Sides; }
        if (_mvCeiling) { flags |= CharacterContact.Above; }
        if (grounded) { flags |= CharacterContact.Below; }

        // Pushes: a dynamic body the character walked into is brought up to the walking speed along the push direction.
        int n = _mvHitCount;
        if (def.PushStrength > 0f)
        {
            for (int i = 0; i < n; i++)
            {
                ref MoverHitRec h = ref _mvHits[i];
                int hb = Core.Shapes.Owner[h.Slot];
                if (Core.Bodies.TypeOf(hb) != BodyType.Dynamic || h.Ny >= c.CosSlope) { continue; }
                float want = (displacement.X * -h.Nx + displacement.Y * -h.Ny) / dt;
                var bv = Core.Bodies.LinearVelocityOf(hb);
                float have = bv[0] * -h.Nx + bv[1] * -h.Ny;
                float dv = (want - have) * def.PushStrength;
                if (!(dv > 0f)) { continue; }
                float mass = Mass[hb];
                var body = new Body(world, Core.HandleOfBody(hb));
                body.ApplyImpulseAt(new Vector2F(-h.Nx * dv * mass, -h.Ny * dv * mass), new Vector2D(h.Px, h.Py));
            }
        }

        int written = 0;
        for (int i = 0; i < n && written < hits.Length; i++)
        {
            ref MoverHitRec h = ref _mvHits[i];
            hits[written++] = new MoverHit2D(new Shape(world, Core.HandleOfShape(h.Slot)), new Vector2D(h.Px, h.Py), new Vector2F(h.Nx, h.Ny));
        }

        state.Position = new Vector2D(x, y);
        state.IsGrounded = grounded;
        state.GroundNormal = grounded ? new Vector2F(gnx, gny) : default;
        state.GroundBody = grounded ? new Body(world, Core.HandleOfBody(Core.Shapes.Owner[gslot])) : default;
        return new MoveResult2D(flags, new Vector2F((float)(x - pos.X), (float)(y - pos.Y)), written);
    }

    /// <summary>Velocity of a point attached to a body: its linear velocity plus the rotation about the centre of mass.</summary>
    internal Vec2 BodyVelocityAt(int body, double px, double py)
    {
        var bodies = Core.Bodies;
        if (bodies.TypeOf(body) == BodyType.Static) { return default; }
        var v = bodies.LinearVelocityOf(body);
        float w = bodies.AngularVelocityOf(body)[0];
        var p = bodies.PositionOf(body);
        float rx = (float)(px - p[0]), ry = (float)(py - p[1]);
        return new Vec2(v[0] - w * ry, v[1] + w * rx);
    }
}
