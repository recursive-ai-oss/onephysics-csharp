using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// The state of one soft body: particles in structure-of-arrays form and the constraints derived from its mesh. Positions are doubles (so a body far from the origin keeps its precision, like rigid bodies),
/// everything else float32. The constraint lists are derived data: they are rebuilt from the mesh and the definition whenever a body is created or restored, never saved.
/// </summary>
internal sealed partial class SoftBodyData
{
    public SoftBodyDef Def = null!;
    public int N;
    public double[] Pos = Array.Empty<double>();      // 3 per particle, world coordinates relative to the world origin
    public double[] Prev = Array.Empty<double>();     // positions at the start of the current substep
    public float[] Vel = Array.Empty<float>();        // 3 per particle
    public float[] InvMass0 = Array.Empty<float>();   // inverse mass the particle has when not pinned
    public float[] InvMass = Array.Empty<float>();    // effective inverse mass: 0 for a pinned particle
    public byte[] Pinned = Array.Empty<byte>();
    public float[] Rest = Array.Empty<float>();       // rest positions in the scaled mesh frame, 3 per particle
    public float Radius;
    public float TotalMass;
    public ConvexShape3D SphereShape;                 // what one particle collides as
    public bool HadMovingContact;                     // touched an awake dynamic or a kinematic shape in the last step (keeps the body from sleeping)

    // Derived constraints.
    public int[] EdgeA = Array.Empty<int>(), EdgeB = Array.Empty<int>();
    public float[] EdgeRest = Array.Empty<float>();
    public long[] EdgeKey = Array.Empty<long>();      // sorted a << 20 | b with a < b, to ask whether two particles share an edge
    public int[] BendIdx = Array.Empty<int>();        // 4 per bend constraint: the shared edge's two particles, then the two far corners
    public float[] BendK = Array.Empty<float>();      // 4 per bend constraint: the cotangent weights K0..K3 of the rest shape
    public float[] BendS = Array.Empty<float>();      // 3 / (2 (A0 + A1)) of the rest shape
    public float[] BendM0 = Array.Empty<float>();     // |m|^2 of the rest shape (0 for a flat rest shape)
    public long[] HingeKey = Array.Empty<long>();     // sorted keys of the two far corners, to ask whether two particles are bend neighbours
    public int BendCount => BendS.Length;
    public int[] Tets = Array.Empty<int>();           // 4 per tetrahedron
    public float[] TetRest = Array.Empty<float>();    // six times the rest volume
    public int[] Tris = Array.Empty<int>();           // surface triangles, for pressure
    public float PressureRest;                        // rest volume of the closed surface; 0 when there is no pressure constraint
    public float[] Lambda = Array.Empty<float>();     // per constraint in solver order (edges, bends, tetrahedra, pressure), scratch of the current substep

    // The step works on float32 copies of the positions relative to an origin near the body (X, XP: current and substep-start positions), so the constraint kernels run on floats and
    // vector lanes without converting; Pos and Prev (double) are the state between steps and are written back at the end of each step.
    public float[] X = Array.Empty<float>(), XP = Array.Empty<float>();
    public double Ox, Oy, Oz;
    public float[] GradP = Array.Empty<float>();      // gradient scratch of the pressure constraint (closed surfaces)

    // Skinning (null Def.Skin: none). Joint poses are state; targets and normals are scratch recomputed from them.
    public float[] Bind = Array.Empty<float>();            // joint bind positions, scaled with the mesh, 3 per joint
    public double[] JointPos = Array.Empty<double>();      // current joint positions, 3 per joint
    public float[] JointRot = Array.Empty<float>();        // current joint rotations x y z w, 4 per joint
    public double[] JointPosPrev = Array.Empty<double>();  // the same at the end of the previous step
    public float[] JointRotPrev = Array.Empty<float>();
    public double[] TgtPrev = Array.Empty<double>();       // skinned positions for the previous pose, 3 per particle (scratch)
    public double[] TgtCur = Array.Empty<double>();        // skinned positions for the current pose (scratch)
    public float[] NrmCur = Array.Empty<float>();          // skinned normals for the current pose (scratch)
    public bool SkinMoved;                                 // the pose changed since the last step (scratch)
    public bool IsSkinned => Def.Skin != null;

    // Sleep.
    public bool Asleep;
    public float SleepTime;

    // Scratch of the current step.
    public int Base;                                  // index of the first particle in the global particle numbering
    public double MinX, MinY, MinZ, MaxX, MaxY, MaxZ;
    public float MaxSpeed;
    public int CandStart, CandCount;

    public int ConstraintCount => EdgeA.Length + BendS.Length + Tets.Length / 4 + (PressureRest > 0f ? 1 : 0);

    /// <summary>Builds a body from a validated definition: particles at the mesh vertices placed by the definition's pose, masses, and the constraint lists.</summary>
    public static SoftBodyData Create(SoftBodyDef def)
    {
        var d = new SoftBodyData { Def = def };
        d.Build();
        var mesh = def.Mesh!;
        var p = def.Position;
        var q = def.Rotation;
        float qn = float.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
        q = new QuaternionF(q.X / qn, q.Y / qn, q.Z / qn, q.W / qn);
        d.Def.Rotation = q;
        for (int i = 0; i < d.N; i++)
        {
            var r = q.Rotate(new Vector3F(d.Rest[3 * i], d.Rest[3 * i + 1], d.Rest[3 * i + 2]));
            d.Pos[3 * i] = p.X + r.X; d.Pos[3 * i + 1] = p.Y + r.Y; d.Pos[3 * i + 2] = p.Z + r.Z;
            d.Vel[3 * i] = def.LinearVelocity.X; d.Vel[3 * i + 1] = def.LinearVelocity.Y; d.Vel[3 * i + 2] = def.LinearVelocity.Z;
        }

        Array.Copy(d.Pos, d.Prev, d.Pos.Length);
        if (def.PinnedParticles != null)
        {
            foreach (int i in def.PinnedParticles) { d.Pinned[i] = 1; d.InvMass[i] = 0f; }
        }

        d.Def.PinnedParticles = null;
        _ = mesh;
        if (d.IsSkinned)
        {
            d.SetBindPose(p, q);
            for (int i = 0; i < d.N; i++) { d.RefreshInvMass(i); }
            // Start exactly on the skinned surface: the bind pose carries the rest shape to the definition's pose.
            d.ComputeTargets(d.JointPos, d.JointRot, d.TgtCur, null);
            Array.Copy(d.TgtCur, d.Pos, d.Pos.Length);
            Array.Copy(d.TgtCur, d.Prev, d.Prev.Length);
        }

        return d;
    }

    /// <summary>Derives masses, rest values and the constraint lists from <see cref="Def"/>; allocates the particle arrays.</summary>
    public void Build()
    {
        var mesh = Def.Mesh!;
        N = mesh.VertexCount;
        float s = Def.Scale;
        Pos = new double[3 * N]; Prev = new double[3 * N]; Vel = new float[3 * N];
        InvMass0 = new float[N]; InvMass = new float[N]; Pinned = new byte[N]; Rest = new float[3 * N];
        var verts = mesh.Vertices;
        for (int i = 0; i < N; i++) { Rest[3 * i] = verts[i].X * s; Rest[3 * i + 1] = verts[i].Y * s; Rest[3 * i + 2] = verts[i].Z * s; }

        var tris = mesh.Triangles;
        var tets = mesh.Tetrahedra;
        // Mass shares: a third of the area of each triangle for a surface, a quarter of the volume of each tetrahedron for a volume.
        var weight = new double[N];
        double total = 0.0;
        double sumArea = 0.0;
        for (int e = 0; e < tris.Length / 3; e++)
        {
            double a = Area(tris[3 * e], tris[3 * e + 1], tris[3 * e + 2]);
            sumArea += a;
            if (tets.Length == 0) { for (int k = 0; k < 3; k++) { weight[tris[3 * e + k]] += a / 3.0; } total += a; }
        }

        double volume = 0.0;
        for (int e = 0; e < tets.Length / 4; e++)
        {
            double v6 = Volume6(tets[4 * e], tets[4 * e + 1], tets[4 * e + 2], tets[4 * e + 3]);
            volume += v6 / 6.0;
            for (int k = 0; k < 4; k++) { weight[tets[4 * e + k]] += v6 / 24.0; }
            total += v6 / 6.0;
        }

        double mass = Def.Mass;
        if (mass == 0.0) { mass = tets.Length > 0 ? 1000.0 * volume : 0.3 * sumArea; }
        if (!(mass > 0.0) || !double.IsFinite(mass)) { mass = 1.0; }
        TotalMass = (float)mass;
        for (int i = 0; i < N; i++)
        {
            double m = mass * weight[i] / total;
            float mf = (float)m;
            if (!(mf > 1e-30f) || !float.IsFinite(mf)) { mf = 1e-30f; }
            InvMass0[i] = 1f / mf; InvMass[i] = InvMass0[i];
        }

        // Edges from every triangle and tetrahedron.
        var keys = new List<long>(tris.Length + tets.Length * 3 / 2);
        for (int e = 0; e < tris.Length / 3; e++)
        {
            for (int k = 0; k < 3; k++) { keys.Add(EdgeOf(tris[3 * e + k], tris[3 * e + (k + 1) % 3])); }
        }

        for (int e = 0; e < tets.Length / 4; e++)
        {
            for (int a = 0; a < 3; a++) { for (int b = a + 1; b < 4; b++) { keys.Add(EdgeOf(tets[4 * e + a], tets[4 * e + b])); } }
        }

        keys.Sort();
        var edgeKeys = new List<long>(keys.Count);
        for (int i = 0; i < keys.Count; i++) { if (i == 0 || keys[i] != keys[i - 1]) { edgeKeys.Add(keys[i]); } }
        EdgeKey = edgeKeys.ToArray();
        int ne = EdgeKey.Length;
        EdgeA = new int[ne]; EdgeB = new int[ne]; EdgeRest = new float[ne];
        for (int i = 0; i < ne; i++)
        {
            EdgeA[i] = (int)(EdgeKey[i] >> 20); EdgeB[i] = (int)(EdgeKey[i] & 0xFFFFF);
            EdgeRest[i] = Dist(EdgeA[i], EdgeB[i]);
        }

        // Bend constraints: the far corners of two triangles that share an edge. Only for a surface mesh.
        if (tets.Length == 0)
        {
            int nt = tris.Length / 3;
            var hk = new long[3 * nt];
            for (int e = 0; e < nt; e++)
            {
                for (int k = 0; k < 3; k++) { hk[3 * e + k] = (EdgeOf(tris[3 * e + k], tris[3 * e + (k + 1) % 3]) << 22) | (long)e; }
            }

            Array.Sort(hk);
            var bi = new List<int>(); var bk = new List<float>(); var bs = new List<float>(); var bm = new List<float>();
            var far = new List<long>();
            for (int i = 0; i + 1 < hk.Length; i++)
            {
                if ((hk[i] >> 22) != (hk[i + 1] >> 22)) { continue; }
                int t1 = (int)(hk[i] & 0x3FFFFF), t2 = (int)(hk[i + 1] & 0x3FFFFF);
                long edge = hk[i] >> 22;
                int p0 = (int)(edge >> 20), p1 = (int)(edge & 0xFFFFF);
                int o1 = Opposite(tris, t1, p0, p1), o2 = Opposite(tris, t2, p0, p1);
                if (o1 == o2 || o1 < 0 || o2 < 0) { continue; }
                if (!BendCoefficients(p0, p1, o1, o2, out float k0, out float k1, out float k2, out float k3, out float bendS, out float m0)) { continue; }
                bi.Add(p0); bi.Add(p1); bi.Add(o1); bi.Add(o2);
                bk.Add(k0); bk.Add(k1); bk.Add(k2); bk.Add(k3);
                bs.Add(bendS); bm.Add(m0);
                far.Add(EdgeOf(o1, o2));
            }

            BendIdx = bi.ToArray(); BendK = bk.ToArray(); BendS = bs.ToArray(); BendM0 = bm.ToArray();
            far.Sort();
            var uniq = new List<long>(far.Count);
            for (int i = 0; i < far.Count; i++) { if (i == 0 || far[i] != far[i - 1]) { uniq.Add(far[i]); } }
            HingeKey = uniq.ToArray();
        }

        Tets = tets.ToArray();
        TetRest = new float[Tets.Length / 4];
        for (int e = 0; e < TetRest.Length; e++) { TetRest[e] = (float)Volume6(Tets[4 * e], Tets[4 * e + 1], Tets[4 * e + 2], Tets[4 * e + 3]); }
        Tris = tris.ToArray();
        PressureRest = 0f;
        if (mesh.IsClosed)
        {
            double v6 = 0.0;
            for (int e = 0; e < Tris.Length / 3; e++) { v6 += Volume6Origin(Tris[3 * e], Tris[3 * e + 1], Tris[3 * e + 2]); }
            PressureRest = (float)(v6 / 6.0);
            if (!(PressureRest > 0f))
            {
                PressureRest = 0f;
                if (Def.InternalPressure > 0f) { throw new SoftBodyException(SoftBodyError.NotClosed, "the closed surface encloses no volume (its triangles face inward or cancel out).", "Mesh"); }
            }
        }

        double edgeSum = 0.0;
        for (int i = 0; i < ne; i++) { edgeSum += EdgeRest[i]; }
        Radius = Def.ParticleRadius != 0f ? Def.ParticleRadius : (float)Math.Max(1e-4, 0.25 * edgeSum / Math.Max(1, ne));
        Lambda = new float[ConstraintCount];
        if (Def.Skin != null)
        {
            int nj = Def.Skin.JointCount;
            Bind = new float[3 * nj];
            for (int j = 0; j < 3 * nj; j++) { Bind[j] = Def.Skin.BindArray[j] * s; }
            JointPos = new double[3 * nj]; JointRot = new float[4 * nj]; JointPosPrev = new double[3 * nj]; JointRotPrev = new float[4 * nj];
            TgtPrev = new double[3 * N]; TgtCur = new double[3 * N]; NrmCur = new float[3 * N];
        }
        X = new float[3 * N]; XP = new float[3 * N];
        GradP = PressureRest > 0f ? new float[3 * N] : Array.Empty<float>();   // the pressure may be switched on later through the handle
        ColourConstraints();
        SphereShape = ConvexShape3D.Sphere(Radius);
    }

    private static long EdgeOf(int a, int b) => a < b ? ((long)a << 20) | (long)b : ((long)b << 20) | (long)a;

    private static int Opposite(ReadOnlySpan<int> tris, int t, int e0, int e1)
    {
        for (int k = 0; k < 3; k++)
        {
            int v = tris[3 * t + k];
            if (v != e0 && v != e1) { return v; }
        }

        return -1;
    }

    // Isometric bending of a hinge (Bergou, Wardetzky, Harmon, Zorin, Grinspun 2006, a quadratic bending model for inextensible surfaces; as constraint: Bender et al. 2014):
    // the cotangent weights K of the four particles give m = sum K_i x_i, which is zero when the hinge is flat, and the bending energy is 3 |m|^2 / (2 (A0 + A1)).
    private bool BendCoefficients(int p0, int p1, int p2, int p3, out float k0, out float k1, out float k2, out float k3, out float s, out float m0sq)
    {
        k0 = k1 = k2 = k3 = s = m0sq = 0f;
        double e0x = (double)Rest[3 * p1] - Rest[3 * p0], e0y = (double)Rest[3 * p1 + 1] - Rest[3 * p0 + 1], e0z = (double)Rest[3 * p1 + 2] - Rest[3 * p0 + 2];
        double e1x = (double)Rest[3 * p2] - Rest[3 * p0], e1y = (double)Rest[3 * p2 + 1] - Rest[3 * p0 + 1], e1z = (double)Rest[3 * p2 + 2] - Rest[3 * p0 + 2];
        double e2x = (double)Rest[3 * p3] - Rest[3 * p0], e2y = (double)Rest[3 * p3 + 1] - Rest[3 * p0 + 1], e2z = (double)Rest[3 * p3 + 2] - Rest[3 * p0 + 2];
        double e3x = (double)Rest[3 * p2] - Rest[3 * p1], e3y = (double)Rest[3 * p2 + 1] - Rest[3 * p1 + 1], e3z = (double)Rest[3 * p2 + 2] - Rest[3 * p1 + 2];
        double e4x = (double)Rest[3 * p3] - Rest[3 * p1], e4y = (double)Rest[3 * p3 + 1] - Rest[3 * p1 + 1], e4z = (double)Rest[3 * p3 + 2] - Rest[3 * p1 + 2];
        double c01 = Cot(e0x, e0y, e0z, e1x, e1y, e1z), c02 = Cot(e0x, e0y, e0z, e2x, e2y, e2z);
        double c03 = Cot(-e0x, -e0y, -e0z, e3x, e3y, e3z), c04 = Cot(-e0x, -e0y, -e0z, e4x, e4y, e4z);
        if (double.IsNaN(c01 + c02 + c03 + c04)) { return false; }
        double a0 = 0.5 * Cross(e0x, e0y, e0z, e1x, e1y, e1z), a1 = 0.5 * Cross(e0x, e0y, e0z, e2x, e2y, e2z);
        if (!(a0 > 0.0) || !(a1 > 0.0)) { return false; }
        double K0 = c03 + c04, K1 = c01 + c02, K2 = -c01 - c03, K3 = -c02 - c04;
        double sd = 3.0 / (2.0 * (a0 + a1));
        // m0 = sum K_i x_i (the weights sum to zero, so measure from particle 0)
        double mx = K1 * e0x + K2 * e1x + K3 * e2x, my = K1 * e0y + K2 * e1y + K3 * e2y, mz = K1 * e0z + K2 * e1z + K3 * e2z;
        k0 = (float)K0; k1 = (float)K1; k2 = (float)K2; k3 = (float)K3; s = (float)sd; m0sq = (float)(mx * mx + my * my + mz * mz);
        return float.IsFinite(k0 + k1 + k2 + k3 + s + m0sq);
    }

    private static double Cross(double ax, double ay, double az, double bx, double by, double bz)
    {
        double x = ay * bz - az * by, y = az * bx - ax * bz, z = ax * by - ay * bx;
        return Math.Sqrt(x * x + y * y + z * z);
    }

    private static double Cot(double ax, double ay, double az, double bx, double by, double bz)
    {
        double c = Cross(ax, ay, az, bx, by, bz);
        return c > 0.0 ? (ax * bx + ay * by + az * bz) / c : double.NaN;
    }

    private float Dist(int a, int b)
    {
        double dx = (double)Rest[3 * a] - Rest[3 * b], dy = (double)Rest[3 * a + 1] - Rest[3 * b + 1], dz = (double)Rest[3 * a + 2] - Rest[3 * b + 2];
        return (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private double Area(int a, int b, int c)
    {
        double ux = (double)Rest[3 * b] - Rest[3 * a], uy = (double)Rest[3 * b + 1] - Rest[3 * a + 1], uz = (double)Rest[3 * b + 2] - Rest[3 * a + 2];
        double wx = (double)Rest[3 * c] - Rest[3 * a], wy = (double)Rest[3 * c + 1] - Rest[3 * a + 1], wz = (double)Rest[3 * c + 2] - Rest[3 * a + 2];
        double nx = uy * wz - uz * wy, ny = uz * wx - ux * wz, nz = ux * wy - uy * wx;
        return 0.5 * Math.Sqrt(nx * nx + ny * ny + nz * nz);
    }

    private double Volume6(int a, int b, int c, int d)
    {
        double ux = (double)Rest[3 * b] - Rest[3 * a], uy = (double)Rest[3 * b + 1] - Rest[3 * a + 1], uz = (double)Rest[3 * b + 2] - Rest[3 * a + 2];
        double wx = (double)Rest[3 * c] - Rest[3 * a], wy = (double)Rest[3 * c + 1] - Rest[3 * a + 1], wz = (double)Rest[3 * c + 2] - Rest[3 * a + 2];
        double sx = (double)Rest[3 * d] - Rest[3 * a], sy = (double)Rest[3 * d + 1] - Rest[3 * a + 1], sz = (double)Rest[3 * d + 2] - Rest[3 * a + 2];
        return sx * (uy * wz - uz * wy) + sy * (uz * wx - ux * wz) + sz * (ux * wy - uy * wx);
    }

    // Six times the signed volume of the tetrahedron from the first particle's rest position to a triangle: summed over a closed surface, the enclosed volume times six.
    private double Volume6Origin(int a, int b, int c)
    {
        double ox = Rest[0], oy = Rest[1], oz = Rest[2];
        double ax = Rest[3 * a] - ox, ay = Rest[3 * a + 1] - oy, az = Rest[3 * a + 2] - oz;
        double bx = Rest[3 * b] - ox, by = Rest[3 * b + 1] - oy, bz = Rest[3 * b + 2] - oz;
        double cx = Rest[3 * c] - ox, cy = Rest[3 * c + 1] - oy, cz = Rest[3 * c + 2] - oz;
        return ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx);
    }
}
