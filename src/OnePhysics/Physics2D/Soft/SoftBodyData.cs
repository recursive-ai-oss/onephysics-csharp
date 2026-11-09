using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The state of one 2D soft body: particles in structure-of-arrays form and the constraints derived from its mesh. Positions are doubles (so a body far from the origin keeps its precision, like rigid bodies),
/// everything else float32. The constraint lists are derived data: they are rebuilt from the mesh and the definition whenever a body is created or restored, never saved.
/// </summary>
internal sealed partial class SoftBodyData
{
    /// <summary>Most particles in a block; see <see cref="PartitionParticles"/>. A change moves results, so it is a constant.</summary>
    internal const int DefaultBlockSize = 80;

    /// <summary>Most blocks of a body: a bigger body gets bigger blocks, because the boundary between blocks grows with their number.</summary>
    internal const int MaxBlocks = 32;

    /// <summary>Most clusters of a shape-matching body.</summary>
    internal const int MaxClusters = 256;

    public SoftBodyDef Def = null!;
    public int N;
    public double[] Pos = Array.Empty<double>();      // 2 per particle, world coordinates relative to the world origin
    public double[] Prev = Array.Empty<double>();     // positions at the start of the current substep
    public float[] Vel = Array.Empty<float>();        // 2 per particle
    public float[] InvMass0 = Array.Empty<float>();   // inverse mass the particle has when not pinned
    public float[] InvMass = Array.Empty<float>();    // effective inverse mass: 0 for a pinned or kinematic particle
    public byte[] Pinned = Array.Empty<byte>();
    public float[] Rest = Array.Empty<float>();       // rest positions in the scaled mesh frame, 2 per particle
    public float Radius;
    public float TotalMass;
    public Shape2D ParticleShape;                     // what one particle collides as
    public bool HadMovingContact;                     // touched an awake dynamic or a kinematic shape in the last step (keeps the body from sleeping)
    internal int BlockSize = DefaultBlockSize;

    // Derived constraints, in the documented order (the public raw arrays).
    public int[] EdgeA = Array.Empty<int>(), EdgeB = Array.Empty<int>();
    public float[] EdgeRest = Array.Empty<float>();
    public long[] EdgeKey = Array.Empty<long>();      // sorted a << 20 | b with a < b
    public int[] BendA = Array.Empty<int>(), BendB = Array.Empty<int>();   // the two far particles of a bend (the neighbours of an outline vertex)
    public float[] BendRest = Array.Empty<float>();
    public long[] HingeKey = Array.Empty<long>();     // sorted keys of the far particles, to ask whether two particles are bend neighbours
    public int[] Tris = Array.Empty<int>();           // 3 per triangle
    public float[] TriRest2 = Array.Empty<float>();   // twice the rest area of each triangle
    public int[] Outline = Array.Empty<int>();
    public float PressureRest;                        // rest area enclosed by the outline; 0 when there is none
    public float[] EdgeRest0 = Array.Empty<float>(), TriRest20 = Array.Empty<float>();   // the rest values of the undeformed mesh, for the plastic limit
    public long[] Broken = Array.Empty<long>();       // sorted keys of the broken edges (tearing); their constraints do not exist
    public float[] Lambda = Array.Empty<float>();     // per constraint in solver order (edges, bends, triangles, pressure), scratch of the current substep

    // The step works on float32 copies of the positions relative to an origin near the body (X, XP: current and substep-start positions).
    public float[] X = Array.Empty<float>(), XP = Array.Empty<float>();
    public double Ox, Oy;
    public float[] GradP = Array.Empty<float>();      // gradient scratch of the pressure constraint

    // Kinematic targets (HasTargets false: none). Targets are state; the interpolated target of a substep is scratch.
    public bool HasTargets;
    public byte[] TargetOn = Array.Empty<byte>();
    public float[] TargetMax = Array.Empty<float>();      // maximum distance of a particle from its target; 0 makes it kinematic
    public double[] TargetPos = Array.Empty<double>();    // current targets, 2 per particle
    public double[] TargetPrev = Array.Empty<double>();   // the targets at the end of the previous step
    public float[] TargetNow = Array.Empty<float>();      // interpolated targets of the current substep, relative to the step's origin (scratch)
    public bool TargetsMoved;                             // a target differs from the previous step's (scratch)

    // Sleep.
    public bool Asleep;
    public float SleepTime;

    // Scratch of the current step.
    public int Base;                                  // index of the first particle in the global particle numbering
    public double MinX, MinY, MaxX, MaxY;
    public float MaxSpeed;
    public int CandStart, CandCount;

    public int ConstraintCount => EdgeA.Length + BendA.Length + Tris.Length / 3 + (PressureRest > 0f ? 1 : 0);

    /// <summary>Builds a body from a validated definition: particles at the mesh vertices placed by the definition's pose, masses, and the constraint lists.</summary>
    public static SoftBodyData Create(SoftBodyDef def, int blockSize = DefaultBlockSize)
    {
        var d = new SoftBodyData { Def = def, BlockSize = blockSize };
        d.Build();
        var p = def.Position;
        var (sn, cs) = Math64.SinCos(def.Angle);
        float c = (float)cs, s = (float)sn;
        float w = def.AngularVelocity;
        for (int i = 0; i < d.N; i++)
        {
            float rx = c * d.Rest[2 * i] - s * d.Rest[2 * i + 1], ry = s * d.Rest[2 * i] + c * d.Rest[2 * i + 1];
            d.Pos[2 * i] = p.X + rx; d.Pos[2 * i + 1] = p.Y + ry;
            d.Vel[2 * i] = def.LinearVelocity.X - w * ry; d.Vel[2 * i + 1] = def.LinearVelocity.Y + w * rx;
        }

        Array.Copy(d.Pos, d.Prev, d.Pos.Length);
        if (def.PinnedParticles != null)
        {
            foreach (int i in def.PinnedParticles) { d.Pinned[i] = 1; d.InvMass[i] = 0f; d.Vel[2 * i] = 0f; d.Vel[2 * i + 1] = 0f; }
        }

        d.Def.PinnedParticles = null;
        return d;
    }

    /// <summary>Derives masses, rest values and the constraint lists from <see cref="Def"/>; allocates the particle arrays.</summary>
    public void Build()
    {
        var mesh = Def.Mesh!;
        N = mesh.VertexCount;
        float s = Def.Scale;
        Pos = new double[2 * N]; Prev = new double[2 * N]; Vel = new float[2 * N];
        InvMass0 = new float[N]; InvMass = new float[N]; Pinned = new byte[N]; Rest = new float[2 * N];
        var verts = mesh.Vertices;
        for (int i = 0; i < N; i++) { Rest[2 * i] = verts[i].X * s; Rest[2 * i + 1] = verts[i].Y * s; }

        Tris = mesh.Triangles.ToArray();
        Outline = mesh.Outline.ToArray();
        int nt = Tris.Length / 3;

        // Mass shares: a third of the area of each triangle; for a skin, half the length of the two segments at each vertex.
        var weight = new double[N];
        double total = 0.0;
        double enclosed = 0.0;
        if (nt > 0)
        {
            for (int e = 0; e < nt; e++)
            {
                double a = 0.5 * Cross(Tris[3 * e], Tris[3 * e + 1], Tris[3 * e + 2]);
                for (int k = 0; k < 3; k++) { weight[Tris[3 * e + k]] += a / 3.0; }
                total += a;
            }
        }
        else
        {
            int n = Outline.Length;
            for (int i = 0; i < n; i++)
            {
                int a = Outline[i], b = Outline[(i + 1) % n];
                double len = Dist(a, b);
                weight[a] += 0.5 * len; weight[b] += 0.5 * len;
                total += len;
            }
        }

        if (Outline.Length >= 3)
        {
            int n = Outline.Length;
            double ox = Rest[2 * Outline[0]], oy = Rest[2 * Outline[0] + 1], twice = 0.0;
            for (int i = 0; i < n; i++)
            {
                int a = Outline[i], b = Outline[(i + 1) % n];
                double ax = Rest[2 * a] - ox, ay = Rest[2 * a + 1] - oy, bx = Rest[2 * b] - ox, by = Rest[2 * b + 1] - oy;
                twice += ax * by - ay * bx;
            }

            enclosed = 0.5 * twice;
        }

        double areaForMass = nt > 0 ? total : enclosed;
        double mass = Def.Mass;
        if (mass == 0.0) { mass = (nt > 0 ? 1000.0 : 10.0) * areaForMass; }
        if (!(mass > 0.0) || !double.IsFinite(mass)) { mass = 1.0; }
        TotalMass = (float)mass;
        for (int i = 0; i < N; i++)
        {
            double m = mass * weight[i] / total;
            float mf = (float)m;
            if (!(mf > 1e-30f) || !float.IsFinite(mf)) { mf = 1e-30f; }
            InvMass0[i] = 1f / mf; InvMass[i] = InvMass0[i];
        }

        // Edges: every distinct edge of the triangles, or every segment of the skin.
        var keys = new List<long>(Math.Max(Tris.Length, Outline.Length));
        if (nt > 0)
        {
            for (int e = 0; e < nt; e++) { for (int k = 0; k < 3; k++) { keys.Add(EdgeOf(Tris[3 * e + k], Tris[3 * e + (k + 1) % 3])); } }
        }
        else
        {
            for (int i = 0; i < Outline.Length; i++) { keys.Add(EdgeOf(Outline[i], Outline[(i + 1) % Outline.Length])); }
        }

        EdgeKey = Distinct(keys);
        if (Broken.Length != 0) { EdgeKey = Array.FindAll(EdgeKey, k => !SoftTopology.Has(Broken, k)); }
        int ne = EdgeKey.Length;
        EdgeA = new int[ne]; EdgeB = new int[ne]; EdgeRest = new float[ne];
        for (int i = 0; i < ne; i++)
        {
            EdgeA[i] = (int)(EdgeKey[i] >> 20); EdgeB[i] = (int)(EdgeKey[i] & 0xFFFFF);
            EdgeRest[i] = (float)Dist(EdgeA[i], EdgeB[i]);
        }

        // Bends: along the skin of a mesh without triangles, the two neighbours of every outline vertex keep their distance.
        if (nt == 0 && Outline.Length >= 4)
        {
            int n = Outline.Length;
            var far = new List<long>(n);
            for (int i = 0; i < n; i++)
            {
                // A hinge at an outline vertex needs both of its segments.
                if (Broken.Length != 0 && (SoftTopology.Has(Broken, EdgeOf(Outline[(i + n - 1) % n], Outline[i])) || SoftTopology.Has(Broken, EdgeOf(Outline[i], Outline[(i + 1) % n])))) { continue; }
                far.Add(EdgeOf(Outline[(i + n - 1) % n], Outline[(i + 1) % n]));
            }

            HingeKey = Distinct(far);
            BendA = new int[HingeKey.Length]; BendB = new int[HingeKey.Length]; BendRest = new float[HingeKey.Length];
            for (int i = 0; i < HingeKey.Length; i++)
            {
                BendA[i] = (int)(HingeKey[i] >> 20); BendB[i] = (int)(HingeKey[i] & 0xFFFFF);
                BendRest[i] = (float)Dist(BendA[i], BendB[i]);
            }
        }

        TriRest2 = new float[nt];
        for (int e = 0; e < nt; e++) { TriRest2[e] = (float)Cross(Tris[3 * e], Tris[3 * e + 1], Tris[3 * e + 2]); }
        EdgeRest0 = (float[])EdgeRest.Clone(); TriRest20 = (float[])TriRest2.Clone();

        PressureRest = 0f;
        if (Outline.Length >= 3)
        {
            PressureRest = (float)enclosed;
            if (!(PressureRest > 0f))
            {
                PressureRest = 0f;
                if (Def.InternalPressure > 0f) { throw new SoftBodyException(SoftBodyError.NotClosed, "the outline encloses no area (it winds clockwise or cancels out).", "Mesh"); }
            }
        }

        double edgeSum = 0.0;
        for (int i = 0; i < ne; i++) { edgeSum += EdgeRest[i]; }
        Radius = Def.ParticleRadius != 0f ? Def.ParticleRadius : (float)Math.Max(1e-4, 0.25 * edgeSum / Math.Max(1, ne));
        Lambda = new float[ConstraintCount];
        X = new float[2 * N]; XP = new float[2 * N];
        GradP = PressureRest > 0f ? new float[2 * N] : Array.Empty<float>();   // the pressure may be switched on later through the handle
        ParticleShape = Shape2D.CreateCircle(default, Radius);
        ColourConstraints();
        BuildClusters();
        BuildSurface();
    }

    /// <summary>Particle <paramref name="i"/> is held by a target with maximum distance 0: kinematic, never collides.</summary>
    public bool IsTargetKinematic(int i) => HasTargets && TargetOn[i] != 0 && TargetMax[i] == 0f;

    /// <summary>Recomputes the effective inverse mass of a particle from its pin and target.</summary>
    public void RefreshInvMass(int i) => InvMass[i] = Pinned[i] != 0 || IsTargetKinematic(i) ? 0f : InvMass0[i];

    /// <summary>Allocates the target arrays (all particles untargeted) when they do not exist yet.</summary>
    public void EnsureTargets()
    {
        if (HasTargets) { return; }
        HasTargets = true;
        TargetOn = new byte[N]; TargetMax = new float[N]; TargetPos = new double[2 * N]; TargetPrev = new double[2 * N]; TargetNow = new float[2 * N];
    }

    private static long EdgeOf(int a, int b) => a < b ? ((long)a << 20) | (long)b : ((long)b << 20) | (long)a;

    private static long[] Distinct(List<long> keys)
    {
        keys.Sort();
        var res = new List<long>(keys.Count);
        for (int i = 0; i < keys.Count; i++) { if (i == 0 || keys[i] != keys[i - 1]) { res.Add(keys[i]); } }
        return res.ToArray();
    }

    private double Dist(int a, int b)
    {
        double dx = (double)Rest[2 * a] - Rest[2 * b], dy = (double)Rest[2 * a + 1] - Rest[2 * b + 1];
        return Math.Sqrt(dx * dx + dy * dy);
    }

    // Twice the signed area of the rest triangle.
    private double Cross(int a, int b, int c)
    {
        double ux = (double)Rest[2 * b] - Rest[2 * a], uy = (double)Rest[2 * b + 1] - Rest[2 * a + 1];
        double wx = (double)Rest[2 * c] - Rest[2 * a], wy = (double)Rest[2 * c + 1] - Rest[2 * a + 1];
        return ux * wy - uy * wx;
    }

    /// <summary>A copy for a world fork: every array of state is copied; derived constraint data is shared (it is never written after the body is built, except the scratch arrays, which are copied).</summary>
    public SoftBodyData CloneForFork()
    {
        var c = (SoftBodyData)MemberwiseClone();
        c.Def = Def.Clone();
        c.Pos = (double[])Pos.Clone(); c.Prev = (double[])Prev.Clone(); c.Vel = (float[])Vel.Clone();
        c.InvMass = (float[])InvMass.Clone(); c.Pinned = (byte[])Pinned.Clone();
        c.Lambda = (float[])Lambda.Clone();
        c.ClonePlastic();
        c.X = (float[])X.Clone(); c.XP = (float[])XP.Clone();
        c.GradP = (float[])GradP.Clone();
        c.MatchMu = (float[])MatchMu.Clone();
        c.TargetOn = (byte[])TargetOn.Clone(); c.TargetMax = (float[])TargetMax.Clone();
        c.TargetPos = (double[])TargetPos.Clone(); c.TargetPrev = (double[])TargetPrev.Clone(); c.TargetNow = (float[])TargetNow.Clone();
        return c;
    }
}
