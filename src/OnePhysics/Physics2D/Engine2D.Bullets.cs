using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private int[] _bullets = new int[16];
    private int _bulletCount;
    private double _bulletPad;

    /// <summary>True for a dynamic body flagged as a bullet: its continuous collision also covers moving bodies (see <see cref="RunBullets"/>).</summary>
    public bool IsBullet(int b) => (Core.Bodies.Flags[b] & (byte)BodyBits.Bullet) != 0;

    /// <summary>
    /// Continuous collision for bullets (Catto, GDC 2013, "Continuous Collision": a fast body swept against everything else, conservative advancement). Runs after every other body has reached its
    /// final pose of the step, so each bullet sees the swept motion of the bodies it may hit. Bullets never sweep against each other, which makes every bullet's result independent of the others
    /// and of the order and thread they are processed in. Each bullet writes only its own pose; the tie between equal times of impact goes to the lower shape slot.
    /// </summary>
    public void RunBullets()
    {
        if (!Opt.ContinuousEnabled) { return; }
        var bodies = Core.Bodies;
        var awake = Core.AwakeBodies;
        int n = 0;
        foreach (int b in awake)
        {
            if (bodies.TypeOf(b) == BodyType.Dynamic && IsBullet(b))
            {
                if (n == _bullets.Length) { Array.Resize(ref _bullets, n * 2); }
                _bullets[n++] = b;
            }
        }

        _bulletCount = n;
        if (n == 0) { return; }

        // How far any non-bullet body's shapes can have moved from where the broadphase boxes were taken at the start of the step: its centre's displacement plus the reach of its rotation.
        // Moving proxies are looked up with their boxes grown by this, so a body that crossed a bullet's path during the step is still a candidate.
        double pad = 0;
        var pos = bodies.Position;
        var rot = bodies.Rotation;
        foreach (int b in awake)
        {
            if (IsBullet(b)) { continue; }
            float mx = (float)(pos[2 * b] - StartPos[2 * b]), my = (float)(pos[2 * b + 1] - StartPos[2 * b + 1]);
            float spin = Math.Abs(StartRot[2 * b] * rot[2 * b + 1] - StartRot[2 * b + 1] * rot[2 * b]) * MaxExtent[b];
            double reach = (double)(float.Sqrt(mx * mx + my * my) + spin);
            if (reach > pad) { pad = reach; }
        }

        _bulletPad = pad * (1.0 + 1e-6) + 1e-4;
        Core.Executor.For((n + 63) / 64, new BulletTask(this));
    }

    private readonly struct BulletTask : IParallelBody
    {
        private readonly Engine2D _e;

        public BulletTask(Engine2D e) => _e = e;

        public void Run(int task)
        {
            int lo = task * 64, hi = Math.Min(lo + 64, _e._bulletCount);
            for (int i = lo; i < hi; i++) { _e.ContinuousBullet(_e._bullets[i]); }
        }
    }

    private struct BulletQuery : IOverlapCallback
    {
        public Engine2D E;
        public int Body;       // the bullet
        public int Shape;      // the bullet's shape being swept
        public Sweep2D Sweep;  // the bullet's motion relative to its start centre
        public double Cx, Cy;  // start position of the bullet's centre of mass
        public Vec2 End;       // end position of the bullet's centre of mass relative to its start
        public float MinExtent;
        public float Best;     // smallest fraction so far
        public int BestSlot;   // shape slot that gave it
        public bool Hit;

        public bool Report(int key, int userId)
        {
            var e = E;
            var core = e.Core;
            int other = userId;
            int ob = core.Shapes.Owner[other];
            if (ob == Body || e.IsSensor(Shape) || e.IsSensor(other) || !e.CanCollide(Shape, other)) { return true; }
            var bodies = core.Bodies;
            bool isStatic = bodies.TypeOf(ob) == BodyType.Static;
            if (!isStatic && e.IsBullet(ob)) { return true; }
            var p = bodies.PositionOf(ob);
            var r = bodies.RotationOf(ob);
            var q2 = new Rot2(r[0], r[1]);
            var c2 = new Vec2((float)(p[0] - Cx), (float)(p[1] - Cy));
            Vec2 c1 = c2;
            Rot2 q1 = q2;
            if (!isStatic)
            {
                c1 = new Vec2((float)(e.StartPos[2 * ob] - Cx), (float)(e.StartPos[2 * ob + 1] - Cy));
                q1 = new Rot2(e.StartRot[2 * ob], e.StartRot[2 * ob + 1]);
            }

            ref readonly var g = ref e.Geom[other];
            if (isStatic && (g.Kind == ShapeKind2D.Segment || g.Kind == ShapeKind2D.ChainSegment))
            {
                // Same rule as for static continuous collision: only a body whose centre really approaches the segment line counts.
                var v = g.Vertices;
                Vec2 edge = v[1] - v[0];
                float len = edge.Length;
                if (!(len > 0f)) { return true; }
                Vec2 l0 = q2.InverseRotate(new Vec2(-c2.X, -c2.Y)) - v[0];
                Vec2 l1 = q2.InverseRotate(new Vec2(End.X - c2.X, End.Y - c2.Y)) - v[0];
                float d0 = (edge.X * l0.Y - edge.Y * l0.X) / len;
                float d1 = (edge.X * l1.Y - edge.Y * l1.X) / len;
                if (g.Kind == ShapeKind2D.ChainSegment && !g.IsTwoSided)
                {
                    if (d0 > 0f) { return true; }
                    d0 = -d0; d1 = -d1;
                }
                else if (d0 < 0f) { d0 = -d0; d1 = -d1; }

                if (d0 - d1 < 0.25f * MinExtent) { return true; }
            }

            var lc = isStatic ? default : new Vec2(e.LocalCenter[2 * ob], e.LocalCenter[2 * ob + 1]);
            var sweepOther = new Sweep2D(lc, c1, c2, q1, q2);
            var toi = Distance2D.TimeOfImpact(in g, sweepOther, in e.Geom[Shape], Sweep, Best, Constants2D.LinearSlop);
            if ((toi.State == ToiState.Hit || toi.State == ToiState.Failed) && toi.Fraction > 0f && (toi.Fraction < Best || (Hit && toi.Fraction == Best && other < BestSlot)))
            {
                Best = toi.Fraction;
                BestSlot = other;
                Hit = true;
            }

            return true;
        }
    }

    private void ContinuousBullet(int b)
    {
        var bodies = Core.Bodies;
        var pos = bodies.PositionOf(b);
        var rot = bodies.RotationOf(b);
        double c1x = StartPos[2 * b], c1y = StartPos[2 * b + 1];
        float mx = (float)(pos[0] - c1x), my = (float)(pos[1] - c1y);
        float q1c = StartRot[2 * b], q1s = StartRot[2 * b + 1], q2c = rot[0], q2s = rot[1];
        float spin = Math.Abs(q1c * q2s - q1s * q2c) * MaxExtent[b];
        float moved = float.Sqrt(mx * mx + my * my) + spin;
        if (!(moved > 0.5f * MinExtent[b])) { return; }

        var lc = new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]);
        var sweep = new Sweep2D(lc, default, new Vec2(mx, my), new Rot2(q1c, q1s), new Rot2(q2c, q2s));
        const BodyTypeMask moving = BodyTypeMask.Kinematic | BodyTypeMask.Dynamic;
        float best = 1f;
        int bestSlot = int.MaxValue;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            var xf1 = new Transform2(-new Rot2(q1c, q1s).Rotate(lc), new Rot2(q1c, q1s));
            var xf2 = new Transform2(new Vec2(mx, my) - new Rot2(q2c, q2s).Rotate(lc), new Rot2(q2c, q2s));
            var a1 = Geom[s].ComputeAabb(xf1);
            var a2 = Geom[s].ComputeAabb(xf2);
            var box = new Aabb2D(
                c1x + (double)Math.Min(a1.Lower.X, a2.Lower.X), c1y + (double)Math.Min(a1.Lower.Y, a2.Lower.Y),
                c1x + (double)Math.Max(a1.Upper.X, a2.Upper.X), c1y + (double)Math.Max(a1.Upper.Y, a2.Upper.Y));
            var q = new BulletQuery { E = this, Body = b, Shape = s, Sweep = sweep, Cx = c1x, Cy = c1y, End = new Vec2(mx, my), MinExtent = MinExtent[b], Best = best, BestSlot = bestSlot };
            Bp.QueryAabb(in box, ref q, BodyTypeMask.Static);
            var grown = new Aabb2D(box.MinX - _bulletPad, box.MinY - _bulletPad, box.MaxX + _bulletPad, box.MaxY + _bulletPad);
            Bp.QueryAabb(in grown, ref q, moving);
            if (q.Hit && (q.Best < best || (q.Best == best && q.BestSlot < bestSlot))) { best = q.Best; bestSlot = q.BestSlot; }
        }

        if (best >= 1f) { return; }
        float t = best;
        pos[0] = c1x + (double)(t * mx);
        pos[1] = c1y + (double)(t * my);
        float nc = (1f - t) * q1c + t * q2c, ns = (1f - t) * q1s + t * q2s;
        float mag = float.Sqrt(nc * nc + ns * ns);
        if (mag > 0f) { rot[0] = nc / mag; rot[1] = ns / mag; }
        Dp[2 * b] = (float)(pos[0] - c1x); Dp[2 * b + 1] = (float)(pos[1] - c1y);
    }
}
