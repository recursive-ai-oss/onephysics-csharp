using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Continuous collision for turning bodies. The linear cast in Continuous holds a shape at its start orientation, so the far end of a long shape that swings into a surface during the step is
    // never seen: the shape ends the step inside the surface, and the contact solver is left to push it out of a penetration the speculative contacts did not cover. Here every corner of a box or
    // hull is swept along the path the body's pose takes over the step (the same interpolation the clamp uses) as a short chain of segments, each cast against the static shapes as a ray; the
    // earliest corner that would end the step well behind a surface it started in front of decides the body's fraction of the step (Catto, GDC 2013, continuous collision: time of impact by
    // casting the points of one body against the other; Ericson, Real-Time Collision Detection, 5.3 for segments against a surface).

    private struct CornerRay : IRayCallback
    {
        public Engine3D E;
        public int MoverShape;
        public double Ox, Oy, Oz, Dx, Dy, Dz;
        public double Best;
        public Vector3F Normal;
        public bool Hit;

        public double Report(int proxy, int userId, double maxFraction)
        {
            var e = E;
            if (e.IsSensor(MoverShape) || e.IsSensor(userId) || !e.CanCollide(MoverShape, userId)) { return -1.0; }
            e.ShapeFrame(userId, out double ox, out double oy, out double oz, out QuaternionF rs);
            var inv = rs.Conjugated();
            var g = e.Geoms[e.GeomOf[userId]]!;
            var lo = inv.Rotate(new Vector3F((float)(Ox - ox), (float)(Oy - oy), (float)(Oz - oz)));
            var lt = inv.Rotate(new Vector3F((float)Dx, (float)Dy, (float)Dz));
            float f; Vector3F n;
            if (g.IsConvex)
            {
                if (g.Convex.Kind == ConvexShapeKind3D.Empty) { return -1.0; }
                if (!Gjk3D.RayCast(g.Convex, RigidTransformF.Identity, lo, lt, 1f, out var hit)) { return -1.0; }
                f = hit.Fraction; n = hit.Normal;
            }
            else if (g.Compound != null)
            {
                if (!ClipToReach(lo, lt, out var co, out var cd, out float t0, out float t1)) { return -1.0; }
                if (!g.Compound.RayCast(co, cd, 1f, out var hit)) { return -1.0; }
                f = t0 + (hit.Fraction * (t1 - t0)); n = hit.Normal;
            }
            else
            {
                if (!g.Soup!.RayCast(lo, lt, 1f, out var hit)) { return -1.0; }
                // A corner coming up through the back of a triangle is not crossing a surface it started in front of.
                if (hit.BackFace) { return -1.0; }
                f = hit.Fraction; n = hit.Normal;
            }

            if (!(f >= 0f) || f > Best) { return -1.0; }
            Best = f; Normal = rs.Rotate(n); Hit = true;
            return f;
        }
    }

    /// <summary>
    /// The smallest fraction of the step at which a corner of one of the body's boxes or hulls reaches a static surface it would end the step more than the speculative distance behind, or
    /// <paramref name="best"/> if none does. The path is the body's centre moving by <paramref name="m"/> and its rotation interpolated from <paramref name="qStart"/> to <paramref name="qEnd"/>.
    /// A corner that starts within half the speculative distance of the surface is left to the contact solver (it is resting, or sliding, there).
    /// </summary>
    private float CornerSweep(int b, double cx, double cy, double cz, Vector3F m, QuaternionF qStart, QuaternionF qEnd, float spin, float best)
    {
        float angle = spin / MaxExtent[b];
        int segments = 1 + (int)(angle / 0.25f);
        if (segments > 8) { segments = 8; }
        Span<QuaternionF> q = stackalloc QuaternionF[9];
        float dot = qStart.X * qEnd.X + qStart.Y * qEnd.Y + qStart.Z * qEnd.Z + qStart.W * qEnd.W;
        float sgn = dot < 0f ? -1f : 1f;
        for (int k = 0; k <= segments; k++)
        {
            float t = (float)k / segments;
            float nx = (1f - t) * qStart.X + t * sgn * qEnd.X, ny = (1f - t) * qStart.Y + t * sgn * qEnd.Y, nz = (1f - t) * qStart.Z + t * sgn * qEnd.Z, nw = (1f - t) * qStart.W + t * sgn * qEnd.W;
            float mag = float.Sqrt(nx * nx + ny * ny + nz * nz + nw * nw);
            q[k] = mag > 0f ? new QuaternionF(nx / mag, ny / mag, nz / mag, nw / mag) : qStart;
        }

        var lc = LocalCenterOf(b);
        float tolerance = Opt.SpeculativeDistance;
        for (int s = ShapeHead[b]; s >= 0; s = NextShape[s])
        {
            var g = Geoms[GeomOf[s]]!;
            if (g.Soup != null || IsSensor(s)) { continue; }
            var l = Local[s];
            if (g.IsConvex) { best = SweepShapeCorners(b, s, g.Convex, l, lc, cx, cy, cz, m, q, segments, tolerance, best); }
            else if (g.Compound != null)
            {
                var comp = g.Compound;
                for (int j = 0; j < comp.SlotCount; j++)
                {
                    if (!comp.IsLive(j)) { continue; }
                    var child = comp.GetChild(j);
                    var childPose = new RigidTransformF(l.Rotation * child.Local.Rotation, l.Rotation.Rotate(child.Local.Translation) + l.Translation);
                    best = SweepShapeCorners(b, s, child.Shape, childPose, lc, cx, cy, cz, m, q, segments, tolerance, best);
                }
            }
        }

        return best;
    }

    private float SweepShapeCorners(int b, int shape, in ConvexShape3D convex, in RigidTransformF pose, Vector3F lc, double cx, double cy, double cz, Vector3F m, ReadOnlySpan<QuaternionF> q, int segments,
        float tolerance, float best)
    {
        var callback = new CornerRay { E = this, MoverShape = shape };
        if (convex.Kind == ConvexShapeKind3D.Hull && convex.Hull != null)
        {
            var vertices = convex.Hull.Vertices;
            for (int v = 0; v < vertices.Length; v++) { best = SweepCorner(ref callback, vertices[v], pose, lc, cx, cy, cz, m, q, segments, tolerance, best); }
        }
        else if (convex.Kind == ConvexShapeKind3D.Box)
        {
            var h = convex.CoreSupport(Vector3F.One);
            for (int k = 0; k < 8; k++)
            {
                best = SweepCorner(ref callback, new Vector3F((k & 1) != 0 ? h.X : -h.X, (k & 2) != 0 ? h.Y : -h.Y, (k & 4) != 0 ? h.Z : -h.Z), pose, lc, cx, cy, cz, m, q, segments, tolerance, best);
            }
        }

        return best;
    }

    private float SweepCorner(ref CornerRay callback, Vector3F corner, in RigidTransformF pose, Vector3F lc, double cx, double cy, double cz, Vector3F m, ReadOnlySpan<QuaternionF> q, int segments,
        float tolerance, float best)
    {
        {
            // The corner's offset from the centre of mass in the body's frame.
            var r = pose.Rotation.Rotate(corner) + pose.Translation - lc;
            Vector3F a = q[0].Rotate(r);
            double px = cx + (double)a.X, py = cy + (double)a.Y, pz = cz + (double)a.Z;
            Vector3F e = q[segments].Rotate(r);
            double endX = cx + (double)m.X + (double)e.X, endY = cy + (double)m.Y + (double)e.Y, endZ = cz + (double)m.Z + (double)e.Z;
            double travelled = 0.0;
            for (int k = 0; k < segments; k++)
            {
                if ((float)k / segments >= best) { break; }
                float t1 = (float)(k + 1) / segments;
                Vector3F b1 = q[k + 1].Rotate(r);
                double nx = cx + (double)(t1 * m.X) + (double)b1.X, ny = cy + (double)(t1 * m.Y) + (double)b1.Y, nz = cz + (double)(t1 * m.Z) + (double)b1.Z;
                double dx = nx - px, dy = ny - py, dz = nz - pz;
                double length = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                if (length > 0.0)
                {
                    callback.Ox = px; callback.Oy = py; callback.Oz = pz; callback.Dx = dx; callback.Dy = dy; callback.Dz = dz;
                    callback.Best = 1.0; callback.Hit = false;
                    Bp.RayCast(new Ray3D(px, py, pz, dx, dy, dz), 1.0, ref callback, BodyTypeMask.Static);
                    if (callback.Hit)
                    {
                        double f = callback.Best;
                        double hx = px + f * dx, hy = py + f * dy, hz = pz + f * dz;
                        var n = callback.Normal;
                        // How far behind the surface the corner ends the step, and how far it went before reaching it.
                        double depth = ((double)n.X * (hx - endX)) + ((double)n.Y * (hy - endY)) + ((double)n.Z * (hz - endZ));
                        double travel = travelled + f * length;
                        if (depth >= tolerance && travel >= 0.5 * tolerance)
                        {
                            float t = (k + (float)f) / segments;
                            if (t < best) { best = t; }
                            break;
                        }
                    }
                }

                travelled += length;
                px = nx; py = ny; pz = nz;
            }
        }

        return best;
    }
}
