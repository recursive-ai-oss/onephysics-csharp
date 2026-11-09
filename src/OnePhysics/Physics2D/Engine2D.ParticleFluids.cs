using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D : IFluidHost
{
    private FluidStore? _particleFluids;

    /// <summary>Particle fluids; created on first use and registered with the world.</summary>
    internal FluidStore ParticleFluids => _particleFluids ??= new FluidStore(this);

    WorldCore IFluidHost.Core => Core;
    bool IFluidHost.Planar => true;
    int IFluidHost.EngineSectionId => Id;
    FluidStore IFluidHost.ParticleFluids => ParticleFluids;
    Vector3F IFluidHost.Gravity => new(Opt.Gravity.X, Opt.Gravity.Y, 0f);
    ReplayMutationScope IFluidHost.FluidMutation(string operation) => World!.RecordMutation(operation);
    int IFluidHost.BodyHighWater => Core.Bodies.HighWater;
    int IFluidHost.OwnerOf(int shape) => Core.Shapes.Owner[shape];

    private struct FluidShapeQuery : IOverlapCallback
    {
        public List<int> Shapes;
        public bool Report(int proxy, int userId) { Shapes.Add(userId); return true; }
    }

    void IFluidHost.GatherShapes(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, List<int> shapes)
    {
        var q = new FluidShapeQuery { Shapes = shapes };
        Bp.QueryAabb(new Aabb2D(minX, minY, maxX, maxY), ref q, BodyTypeMask.All);
        shapes.Sort();
        int w = 0;
        for (int i = 0; i < shapes.Count; i++)
        {
            if (i > 0 && shapes[i] == shapes[i - 1]) { continue; }
            shapes[w++] = shapes[i];
        }

        shapes.RemoveRange(w, shapes.Count - w);
    }

    bool IFluidHost.ShapeCollides(int s, int layer, int group, uint category, uint mask)
    {
        if ((ShapeFlags[s] & FlagSensor) != 0) { return false; }
        if (((LayerMatrix[layer] >> ShapeLayer[s]) & 1u) == 0) { return false; }
        if (Group[s] == group && group != 0) { return group > 0; }
        return (mask & Category[s]) != 0 && (category & Mask[s]) != 0;
    }

    void IFluidHost.LoadBody(int b, out FluidBody f)
    {
        var bodies = Core.Bodies;
        f = default;
        var type = bodies.TypeOf(b);
        var p = bodies.PositionOf(b);
        var r = bodies.RotationOf(b);
        var v = bodies.LinearVelocityOf(b);
        var w = bodies.AngularVelocityOf(b);
        f.X = p[0]; f.Y = p[1]; f.Z = 0.0;
        // The planar rotation (cos, sin) as the quaternion about z with half the angle: sqrt((1 +- c) / 2).
        float c = r[0], s = r[1];
        float qw = float.Sqrt(float.Max(0f, (1f + c) * 0.5f)), qz = float.Sqrt(float.Max(0f, (1f - c) * 0.5f));
        if (s < 0f) { qz = -qz; }
        f.Qz = qz; f.Qw = qw;
        f.Body = b;
        bool awake = bodies.IsAwake(b);
        f.Dynamic = type == BodyType.Dynamic && bodies.InverseMass[b] > 0f;
        f.Moves = type == BodyType.Kinematic || (f.Dynamic && awake);
        f.Asleep = f.Dynamic && !awake;
        f.Vx = v[0]; f.Vy = v[1]; f.Wz = w[0];
        if (f.Dynamic)
        {
            f.InvMass = bodies.InverseMass[b];
            f.Iw5 = bodies.InverseInertia[b];
            var g = Opt.Gravity;
            float gs = GravityScale[b];
            f.Gx = g.X * gs; f.Gy = g.Y * gs;
        }
    }

    private static Rot2 PlanarRot(float qz, float qw) => new(qw * qw - qz * qz, 2f * qw * qz);

    void IFluidHost.ShapeFrame(int s, in FluidBody pose, out FluidFrame frame)
    {
        int b = Core.Shapes.Owner[s];
        var q = PlanarRot(pose.Qz, pose.Qw);
        var arm = q.Rotate(new Vec2(LocalCenter[2 * b], LocalCenter[2 * b + 1]));
        // The shape's frame is the body's frame: the centre of mass minus the rotated local centre.
        frame = new FluidFrame { X = pose.X - arm.X, Y = pose.Y - arm.Y, Z = 0.0, Qz = pose.Qz, Qw = pose.Qw };
    }

    void IFluidHost.ShapeBox(int s, in FluidFrame fr, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)
    {
        var bb = Geom[s].ComputeAabb(new Transform2(default, PlanarRot(fr.Qz, fr.Qw)));
        minX = fr.X + bb.Lower.X; minY = fr.Y + bb.Lower.Y; minZ = -1.0; maxX = fr.X + bb.Upper.X; maxY = fr.Y + bb.Upper.Y; maxZ = 1.0;
    }

    bool IFluidHost.CollideSphere(int s, in FluidFrame fr, double cx, double cy, double cz, float radius, float spec, out FluidHit hit)
    {
        hit = default;
        if (!(Math.Abs(cx - fr.X) < 2e6 && Math.Abs(cy - fr.Y) < 2e6)) { return false; }
        var q = PlanarRot(fr.Qz, fr.Qw);
        var rel = q.InverseRotate(new Vec2((float)(cx - fr.X), (float)(cy - fr.Y)));
        if (!(Math.Abs(rel.X) < 4e6f && Math.Abs(rel.Y) < 4e6f)) { return false; }
        var circle = Shape2D.CreateCircle(default, radius);
        var m = Collide2D.Collide(in Geom[s], in circle, new Transform2(rel, Rot2.Identity), spec);
        if (m.PointCount == 0) { return false; }
        int best = 0;
        if (m.PointCount > 1 && m.Point1.Separation < m.Point0.Separation) { best = 1; }
        var p = best == 0 ? m.Point0 : m.Point1;
        var n = q.Rotate(m.Normal);
        var pt = q.Rotate(p.Point);
        hit.Nx = n.X; hit.Ny = n.Y; hit.Separation = p.Separation;
        hit.Px = fr.X + pt.X; hit.Py = fr.Y + pt.Y;
        hit.Friction = Friction[s];
        return true;
    }

    bool IFluidHost.SweepSphere(int s, in FluidFrame fr, double s0x, double s0y, double s0z, float mx, float my, float mz, float radius, out FluidHit hit)
    {
        hit = default;
        var q = PlanarRot(fr.Qz, fr.Qw);
        var circle = Shape2D.CreateCircle(default, radius);
        var motion = new Vec2(mx, my);
        var rel0 = q.InverseRotate(new Vec2((float)(s0x - fr.X), (float)(s0y - fr.Y)));
        var h = Distance2D.ShapeCast(in Geom[s], in circle, new Transform2(rel0, Rot2.Identity), q.InverseRotate(motion), 1f);
        // A zero normal means the particle already overlaps the shape at the start; the discrete test handles that.
        if (!(h.Hit && h.Fraction < 1f && (h.Normal.X != 0f || h.Normal.Y != 0f))) { return false; }
        var n = q.Rotate(h.Normal);
        hit.Nx = n.X; hit.Ny = n.Y; hit.Separation = 0f;
        hit.Px = s0x + motion.X * h.Fraction; hit.Py = s0y + motion.Y * h.Fraction;
        hit.Friction = Friction[s];
        return true;
    }

    void IFluidHost.ApplyBodyImpulse(int b, float dvx, float dvy, float dvz, float dwx, float dwy, float dwz, float wakeSpeed)
    {
        var bodies = Core.Bodies;
        if (!bodies.IsDynamic(b)) { return; }
        if (!bodies.IsAwake(b))
        {
            float dv2 = dvx * dvx + dvy * dvy, dw2 = dwz * dwz;
            if (!(dv2 > wakeSpeed * wakeSpeed || dw2 > wakeSpeed * wakeSpeed)) { return; }
            Core.WakeIsland(bodies.Island[b]);
            bodies.SleepTime[b] = 0f;
        }

        var v = bodies.LinearVelocityOf(b); var w = bodies.AngularVelocityOf(b);
        v[0] += dvx; v[1] += dvy;
        w[0] += dwz;
    }

    bool IFluidHost.BodyAlive(int body, int generation) => Core.Bodies.Handles.IsAlive(body, generation);

    /// <summary>A lattice of points inside the body's shapes (8 per axis for one shape, fewer for several; at most 64 points), in the body frame relative to the centre of mass, with their depths.</summary>
    int IFluidHost.SamplePoints(int b, Span<Vector3F> points, Span<float> depths)
    {
        int shapes = 0;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { if (!IsSensor(s)) { shapes++; } }
        if (shapes == 0) { return 0; }
        int per = shapes == 1 ? 8 : shapes <= 3 ? 5 : shapes <= 8 ? 3 : 1;
        int n = 0;
        var tiny = Shape2D.CreateCircle(default, 1e-3f);
        float cx = LocalCenter[2 * b], cy = LocalCenter[2 * b + 1];
        for (int s = ShapeHead[b]; s >= 0 && n < points.Length; s = NextShape[s])
        {
            if (IsSensor(s)) { continue; }
            var bb = Geom[s].ComputeAabb(Transform2.Identity);
            for (int j = 0; j < per && n < points.Length; j++)
            {
                for (int i = 0; i < per && n < points.Length; i++)
                {
                    var lp = new Vec2(bb.Lower.X + (bb.Upper.X - bb.Lower.X) * ((i + 0.5f) / per), bb.Lower.Y + (bb.Upper.Y - bb.Lower.Y) * ((j + 0.5f) / per));
                    var m = Collide2D.Collide(in Geom[s], in tiny, new Transform2(lp, Rot2.Identity), 0f);
                    if (m.PointCount == 0 || !(m.Point0.Separation < 0f)) { continue; }
                    depths[n] = -m.Point0.Separation;
                    points[n++] = new Vector3F(lp.X - cx, lp.Y - cy, 0f);
                }
            }
        }

        return n;
    }

    internal bool ParticleFluidsConsistent() => _particleFluids == null || _particleFluids.Consistent();


}
