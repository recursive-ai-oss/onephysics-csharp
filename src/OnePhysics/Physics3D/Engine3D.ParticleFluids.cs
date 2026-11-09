using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fluids;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D : IFluidHost
{
    private FluidStore? _particleFluids;

    /// <summary>Particle fluids; created on first use and registered with the world.</summary>
    internal FluidStore ParticleFluids => _particleFluids ??= new FluidStore(this);

    WorldCore IFluidHost.Core => Core;
    bool IFluidHost.Planar => false;
    int IFluidHost.EngineSectionId => Id;
    FluidStore IFluidHost.ParticleFluids => ParticleFluids;
    Vector3F IFluidHost.Gravity => Opt.Gravity;
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
        Bp.QueryAabb(new Aabb3D(minX, minY, minZ, maxX, maxY, maxZ), ref q, BodyTypeMask.All);
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
        var q = RotationOfBody(b);
        var v = bodies.LinearVelocityOf(b);
        var w = bodies.AngularVelocityOf(b);
        f.X = p[0]; f.Y = p[1]; f.Z = p[2];
        f.Qx = q.X; f.Qy = q.Y; f.Qz = q.Z; f.Qw = q.W;
        f.Body = b;
        bool awake = bodies.IsAwake(b);
        f.Dynamic = type == BodyType.Dynamic && bodies.InverseMass[b] > 0f;
        f.Moves = type == BodyType.Kinematic || (f.Dynamic && awake);
        f.Asleep = f.Dynamic && !awake;
        f.Vx = v[0]; f.Vy = v[1]; f.Vz = v[2]; f.Wx = w[0]; f.Wy = w[1]; f.Wz = w[2];
        if (f.Dynamic)
        {
            f.InvMass = bodies.InverseMass[b];
            Span<float> iw = stackalloc float[6];
            WorldInverseInertia(b, iw);
            f.Iw0 = iw[0]; f.Iw1 = iw[1]; f.Iw2 = iw[2]; f.Iw3 = iw[3]; f.Iw4 = iw[4]; f.Iw5 = iw[5];
            var g = Opt.Gravity;
            float gs = GravityScale[b];
            f.Gx = g.X * gs; f.Gy = g.Y * gs; f.Gz = g.Z * gs;
        }
    }

    void IFluidHost.ShapeFrame(int s, in FluidBody pose, out FluidFrame frame)
    {
        int b = Core.Shapes.Owner[s];
        var q = new QuaternionF(pose.Qx, pose.Qy, pose.Qz, pose.Qw);
        var l = Local[s];
        var arm = q.Rotate(l.Translation - LocalCenterOf(b));
        var rw = q * l.Rotation;
        frame = new FluidFrame { X = pose.X + arm.X, Y = pose.Y + arm.Y, Z = pose.Z + arm.Z, Qx = rw.X, Qy = rw.Y, Qz = rw.Z, Qw = rw.W };
    }

    void IFluidHost.ShapeBox(int s, in FluidFrame fr, out double minX, out double minY, out double minZ, out double maxX, out double maxY, out double maxZ)
    {
        var g = Geoms[GeomOf[s]]!;
        var rw = new QuaternionF(fr.Qx, fr.Qy, fr.Qz, fr.Qw);
        Vector3F mn, mx;
        if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, new RigidTransformF(rw, default), out mn, out mx); }
        else { RotatedBox(g.LocalBounds, rw, out mn, out mx); }
        minX = fr.X + mn.X; minY = fr.Y + mn.Y; minZ = fr.Z + mn.Z; maxX = fr.X + mx.X; maxY = fr.Y + mx.Y; maxZ = fr.Z + mx.Z;
    }

    /// <summary>The deepest contact of a sphere with the shape's collision geometry: the discrete test of a soft-body particle, the same code.</summary>
    private bool FluidCollide(int s, in FluidFrame fr, double cx, double cy, double cz, float radius, float spec, out FluidHit hit)
    {
        hit = default;
        var g = Geoms[GeomOf[s]]!;
        var rw = new QuaternionF(fr.Qx, fr.Qy, fr.Qz, fr.Qw);
        var inv = rw.Conjugated();
        if (!(Math.Abs(cx - fr.X) < 2e6 && Math.Abs(cy - fr.Y) < 2e6 && Math.Abs(cz - fr.Z) < 2e6)) { return false; }
        var rel = inv.Rotate(new Vector3F((float)(cx - fr.X), (float)(cy - fr.Y), (float)(cz - fr.Z)));
        if (!(float.Abs(rel.X) < 4e6f && float.Abs(rel.Y) < 4e6f && float.Abs(rel.Z) < 4e6f)) { return false; }
        var sphere = ConvexShape3D.Sphere(radius);
        var gen = NarrowScratch.Gen;
        var pose = new RigidTransformF(QuaternionF.Identity, rel);
        bool any = false;
        float best = float.MaxValue;
        Vector3F bn = default, bp = default;
        if (g.IsConvex)
        {
            var m = gen.Collide(g.Convex, sphere, pose, spec);
            Best(in m, ref any, ref best, ref bn, ref bp);
        }
        else if (g.Compound != null)
        {
            var children = NarrowScratch.Children;
            gen.CollideCompoundConvex(g.Compound, sphere, pose, children, spec);
            for (int k = 0; k < children.Count; k++) { Best(children[k].Manifold, ref any, ref best, ref bn, ref bp); }
        }
        else if (g.Soup != null)
        {
            var tris = NarrowScratch.Tris;
            tris.Clear();
            g.Soup.CollectCandidates(sphere, pose, spec, tris, NarrowScratch.Ids);
            var triBuf = NarrowScratch.TriBuf;
            gen.BeginPin(2);
            for (int t = 0; t < tris.Count; t++)
            {
                var tri = tris[t];
                var tshape = triBuf.Set(tri.V0, tri.V1, tri.V2);
                var m = gen.Collide(tshape, sphere, pose, spec);
                Best(in m, ref any, ref best, ref bn, ref bp);
            }

            gen.EndPin();
        }

        if (!any) { return false; }
        var n = rw.Rotate(bn);
        var pt = rw.Rotate(bp);
        hit.Nx = n.X; hit.Ny = n.Y; hit.Nz = n.Z; hit.Separation = best;
        hit.Px = fr.X + pt.X; hit.Py = fr.Y + pt.Y; hit.Pz = fr.Z + pt.Z;
        hit.Friction = Friction[s];
        return true;
    }

    private static void Best(in Manifold3D m, ref bool any, ref float best, ref Vector3F bn, ref Vector3F bp)
    {
        for (int k = 0; k < m.PointCount; k++)
        {
            var p = m[k];
            if (!any || p.Separation < best) { any = true; best = p.Separation; bn = m.Normal; bp = p.Point; }
        }
    }

    bool IFluidHost.CollideSphere(int s, in FluidFrame fr, double cx, double cy, double cz, float radius, float spec, out FluidHit hit) => FluidCollide(s, in fr, cx, cy, cz, radius, spec, out hit);

    bool IFluidHost.SweepSphere(int s, in FluidFrame fr, double s0x, double s0y, double s0z, float mx, float my, float mz, float radius, out FluidHit hit)
    {
        hit = default;
        var g = Geoms[GeomOf[s]]!;
        var rw = new QuaternionF(fr.Qx, fr.Qy, fr.Qz, fr.Qw);
        var inv = rw.Conjugated();
        var sphere = ConvexShape3D.Sphere(radius);
        var motion = new Vector3F(mx, my, mz);
        float bestF = 2f;
        Vector3F bestN = default;
        if (g.IsConvex || g.Compound != null)
        {
            var bpos = new Vector3F((float)(fr.X - s0x), (float)(fr.Y - s0y), (float)(fr.Z - s0z));
            if (g.IsConvex)
            {
                if (Gjk3D.ShapeCast(sphere, g.Convex, new RigidTransformF(rw, bpos), motion, out var h) && !h.StartsOverlapping && h.Fraction < bestF) { bestF = h.Fraction; bestN = -h.Normal; }
            }
            else
            {
                var comp = g.Compound!;
                for (int j = 0; j < comp.SlotCount; j++)
                {
                    if (!comp.IsLive(j)) { continue; }
                    var child = comp.GetChild(j);
                    var l = child.Local;
                    var place = new RigidTransformF(rw * l.Rotation, bpos + rw.Rotate(l.Translation));
                    if (Gjk3D.ShapeCast(sphere, child.Shape, place, motion, out var h) && !h.StartsOverlapping && h.Fraction < bestF) { bestF = h.Fraction; bestN = -h.Normal; }
                }
            }
        }
        else if (g.Soup != null)
        {
            var rel0 = inv.Rotate(new Vector3F((float)(s0x - fr.X), (float)(s0y - fr.Y), (float)(s0z - fr.Z)));
            if (g.Soup.ShapeCast(sphere, new RigidTransformF(QuaternionF.Identity, rel0), inv.Rotate(motion), out var h, NarrowScratch.Ids) && !h.StartsOverlapping && h.Fraction < bestF)
            {
                bestF = h.Fraction; bestN = rw.Rotate(h.Normal);
            }
        }

        if (!(bestF < 1f)) { return false; }
        hit.Nx = bestN.X; hit.Ny = bestN.Y; hit.Nz = bestN.Z; hit.Separation = 0f;
        hit.Px = s0x + motion.X * bestF; hit.Py = s0y + motion.Y * bestF; hit.Pz = s0z + motion.Z * bestF;
        hit.Friction = Friction[s];
        return true;
    }

    void IFluidHost.ApplyBodyImpulse(int b, float dvx, float dvy, float dvz, float dwx, float dwy, float dwz, float wakeSpeed)
    {
        var bodies = Core.Bodies;
        if (!bodies.IsDynamic(b)) { return; }
        if (!bodies.IsAwake(b))
        {
            float dv2 = dvx * dvx + dvy * dvy + dvz * dvz, dw2 = dwx * dwx + dwy * dwy + dwz * dwz;
            if (!(dv2 > wakeSpeed * wakeSpeed || dw2 > wakeSpeed * wakeSpeed)) { return; }
            Core.WakeIsland(bodies.Island[b]);
            bodies.SleepTime[b] = 0f;
        }

        var v = bodies.LinearVelocityOf(b); var w = bodies.AngularVelocityOf(b);
        v[0] += dvx; v[1] += dvy; v[2] += dvz;
        w[0] += dwx; w[1] += dwy; w[2] += dwz;
    }

    bool IFluidHost.BodyAlive(int body, int generation) => Core.Bodies.Handles.IsAlive(body, generation);

    /// <summary>A lattice of points inside the body's convex and compound shapes (4 per axis for one shape, 3 for up to three, 2 for up to eight, 1 beyond; at most 64 points), in the body frame relative to the centre of mass.</summary>
    int IFluidHost.SamplePoints(int b, Span<Vector3F> points, Span<float> depths)
    {
        int shapes = 0;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s]) { if (!IsSensor(s)) { shapes++; } }
        if (shapes == 0) { return 0; }
        int per = shapes == 1 ? 4 : shapes <= 3 ? 3 : shapes <= 8 ? 2 : 1;
        int n = 0;
        var sphere = ConvexShape3D.Sphere(1e-3f);
        var gen = NarrowScratch.Gen;
        var com = LocalCenterOf(b);
        for (int s = ShapeHead[b]; s >= 0 && n < points.Length; s = NextShape[s])
        {
            if (IsSensor(s)) { continue; }
            var g = Geoms[GeomOf[s]]!;
            if (!g.IsConvex && g.Compound == null) { continue; }
            Vector3F mn, mx;
            if (g.IsConvex) { ShapeBounds3D.Compute(g.Convex, RigidTransformF.Identity, out mn, out mx); }
            else { mn = g.LocalBounds.Min; mx = g.LocalBounds.Max; }
            var l = Local[s];
            for (int k = 0; k < per && n < points.Length; k++)
            {
                for (int j = 0; j < per && n < points.Length; j++)
                {
                    for (int i = 0; i < per && n < points.Length; i++)
                    {
                        var lp = new Vector3F(mn.X + (mx.X - mn.X) * ((i + 0.5f) / per), mn.Y + (mx.Y - mn.Y) * ((j + 0.5f) / per), mn.Z + (mx.Z - mn.Z) * ((k + 0.5f) / per));
                        float deepest = 0f;
                        var pose = new RigidTransformF(QuaternionF.Identity, lp);
                        if (g.IsConvex) { var m = gen.Collide(g.Convex, sphere, pose, 0f); if (m.PointCount > 0 && m[0].Separation < 0f) { deepest = -m[0].Separation; } }
                        else
                        {
                            var children = NarrowScratch.Children;
                            gen.CollideCompoundConvex(g.Compound!, sphere, pose, children, 0f);
                            for (int c = 0; c < children.Count; c++) { var m = children[c].Manifold; if (m.PointCount > 0 && -m[0].Separation > deepest) { deepest = -m[0].Separation; } }
                        }

                        if (deepest > 0f) { depths[n] = deepest; points[n++] = l.Rotation.Rotate(lp) + l.Translation - com; }
                    }
                }
            }
        }

        return n;
    }

    internal bool ParticleFluidsConsistent() => _particleFluids == null || _particleFluids.Consistent();
}
