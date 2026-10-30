using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// The skinned sleeve of <c>soft-skinned-3d</c> (docs/spec/scenes.md): an open tube of radius r and length L along y on a two-joint arm (joint 0 at the tube's base, joint 1 at L/2). The ring at the base is the cuff: maximum distance 0.
/// Every other ring may stray 0.06 m from its skinned position and keeps out of a back-stop sphere of radius 0.04 centred 0.04 m behind it. Weights blend the two joints over the middle three rings.
/// The arm swings by a triangle wave of the step number: joint 0 about z by 0.5 * tri(step, 90), joint 1 about x by 0.9 * tri(step + 20, 70) further. Only IEEE-exact arithmetic builds the mesh.
/// </summary>
public static class SleeveRig
{
    public const double MaxDistance = 0.06, BackstopDistance = 0.04, BackstopRadius = 0.04;

    static float Tri(int step, int period)
    {
        int m = step % period;
        double x = (double)m / period;
        return (float)(x < 0.5 ? 4 * x - 1 : 3 - 4 * x);
    }

    /// <summary>The ring r, point a of the sleeve: P = radius, length, around (multiple of 4), rings. Circle points from the rational parametrisation (1 - t^2, 2t) / (1 + t^2) of each quadrant.</summary>
    public static SoftMeshData Mesh(double radius, double length, int around, int rings)
    {
        if (around % 4 != 0 || around < 8 || rings < 3) throw new InvalidDataException("sleeve needs around a multiple of 4, at least 8, and at least 3 rings");
        var m = new SoftMeshData { V = new float[3 * around * rings] };
        int q = around / 4;
        for (int r = 0; r < rings; r++)
            for (int a = 0; a < around; a++)
            {
                var (cx, cz) = Circle(a, q);
                int o = 3 * (r * around + a);
                m.V[o] = (float)(radius * cx); m.V[o + 1] = (float)(length * r / (rings - 1)); m.V[o + 2] = (float)(radius * cz);
            }
        var tris = new List<int>();
        for (int r = 0; r + 1 < rings; r++)
            for (int a = 0; a < around; a++)
            {
                int b = (a + 1) % around;
                int i00 = r * around + a, i01 = r * around + b, i10 = (r + 1) * around + a, i11 = (r + 1) * around + b;
                tris.AddRange([i00, i01, i10, i01, i11, i10]);
            }
        m.Tris = tris.ToArray();
        return m;
    }

    static (double X, double Z) Circle(int a, int perQuadrant)
    {
        int quad = a / perQuadrant;
        double t = (double)(a % perQuadrant) / perQuadrant, d = 1 + t * t;
        double x = (1 - t * t) / d, z = 2 * t / d;
        return quad switch { 0 => (x, z), 1 => (-z, x), 2 => (-x, -z), _ => (z, -x) };
    }

    public static float Weight1(int ring, int rings) => Math.Clamp((ring - (rings - 1) / 2f + 1f) / 2f, 0f, 1f);

    public static P3.SoftSkin MakeSkin(SoftDef d, P3.SoftBodyMesh mesh)
    {
        int around = (int)d.P[2], rings = (int)d.P[3], n = around * rings, q = around / 4;
        double length = d.P[1];
        var joints = new int[4 * n]; var weights = new float[4 * n]; var max = new float[n]; var bd = new float[n]; var br = new float[n]; var nrm = new Vector3F[n];
        for (int r = 0; r < rings; r++)
            for (int a = 0; a < around; a++)
            {
                int i = r * around + a;
                float t = Weight1(r, rings);
                joints[4 * i] = 0; joints[4 * i + 1] = 1; weights[4 * i] = 1f - t; weights[4 * i + 1] = t;
                var (cx, cz) = Circle(a, q);
                nrm[i] = new Vector3F((float)cx, 0f, (float)cz);
                max[i] = r == 0 ? 0f : (float)MaxDistance;
                if (r > 0) { bd[i] = (float)BackstopDistance; br[i] = (float)BackstopRadius; }
            }
        return P3.SoftSkin.Create(mesh, new[] { -1, 0 }, Bind(length), joints, weights, max, bd, br, nrm);
    }

    public static Vector3F[] Bind(double length) => [new Vector3F(0f, 0f, 0f), new Vector3F(0f, (float)(0.5 * length), 0f)];

    /// <summary>The joint poses of a step for a sleeve whose bind pose sits at <paramref name="at"/> (no body rotation).</summary>
    public static P3.SkinJointPose[] Pose(int step, Vector3D at, double length)
    {
        var qRoot = QuaternionF.CreateFromAxisAngle(new Vector3F(0f, 0f, 1f), 0.5f * Tri(step, 90));
        var qTip = qRoot * QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), 0.9f * Tri(step + 20, 70));
        var bind = Bind(length);
        var o = qRoot.Rotate(bind[1] - bind[0]);
        return [new P3.SkinJointPose(at, qRoot), new P3.SkinJointPose(new Vector3D(at.X + o.X, at.Y + o.Y, at.Z + o.Z), qTip)];
    }

    /// <summary>Independent double-precision skinning of one particle (used by the metrics): returns the target and the skinned unit normal.</summary>
    public static (double X, double Y, double Z, double Nx, double Ny, double Nz) Skin(SoftDef d, SoftMeshData mesh, int i, P3.SkinJointPose[] pose)
    {
        int around = (int)d.P[2], rings = (int)d.P[3], q = around / 4;
        int r = i / around, a = i % around;
        double t1 = Weight1(r, rings), t0 = 1 - t1;
        var bind = Bind(d.P[1]);
        double px = 0, py = 0, pz = 0, nx = 0, ny = 0, nz = 0;
        var (cx, cz) = Circle(a, q);
        for (int j = 0; j < 2; j++)
        {
            double w = j == 0 ? t0 : t1;
            if (w == 0) continue;
            var rot = pose[j].Rotation;
            double x = mesh.V[3 * i] - bind[j].X, y = mesh.V[3 * i + 1] - bind[j].Y, z = mesh.V[3 * i + 2] - bind[j].Z;
            var (rx, ry, rz) = Rotate(rot, x, y, z);
            px += w * (pose[j].Position.X + rx); py += w * (pose[j].Position.Y + ry); pz += w * (pose[j].Position.Z + rz);
            var (mx, my, mz) = Rotate(rot, cx, 0, cz);
            nx += w * mx; ny += w * my; nz += w * mz;
        }
        double l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
        return (px, py, pz, nx / l, ny / l, nz / l);
    }

    static (double, double, double) Rotate(QuaternionF q, double x, double y, double z)
    {
        double qx = q.X, qy = q.Y, qz = q.Z, qw = q.W;
        double tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z), tz = 2 * (qx * y - qy * x);
        return (x + qw * tx + (qy * tz - qz * ty), y + qw * ty + (qz * tx - qx * tz), z + qw * tz + (qx * ty - qy * tx));
    }
}
