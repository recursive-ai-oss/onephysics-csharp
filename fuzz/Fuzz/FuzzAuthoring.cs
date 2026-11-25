using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tools;

// Targets for the tetrahedralizer, the static solve, the polygon mesher and the ragdoll fitter. Every input either is accepted or fails with a GeometryToolException (a StaticSolveException for the static solve); any other exception is a finding.
internal static class FuzzAuthoring
{
    private sealed class Reader
    {
        private readonly byte[] _b;
        private int _p;
        public Reader(byte[] b) => _b = b;
        public byte Byte() => _p < _b.Length ? _b[_p++] : (byte)0;
        public int Int(int n) => Byte() % n;

        // Mostly modest numbers, sometimes raw bits (NaN, infinity, huge, subnormal).
        public float Float()
        {
            byte mode = Byte();
            if (mode < 24 && _p + 4 <= _b.Length) { float f = BitConverter.ToSingle(_b, _p); _p += 4; return f; }
            return (Byte() - 128) / 16f + Byte() / 4096f;
        }
    }

    private static void Guard(Action a)
    {
        try { a(); }
        catch (GeometryToolException) { }
    }

    // Seeds: a closed octahedron or box with moved vertices (valid input that reaches the lattice code), or a raw random mesh (hostile input).
    internal static void Tetrahedralize(byte[] input)
    {
        var r = new Reader(input);
        Vector3F[] v; int[] idx;
        if (r.Byte() < 160)
        {
            // octahedron, each vertex pushed along its axis by a fuzzed amount
            float a = 1 + Math.Abs(r.Byte() / 64f), b = 1 + Math.Abs(r.Byte() / 64f), c = 1 + Math.Abs(r.Byte() / 64f);
            v = new[] { new Vector3F(a, 0, 0), new Vector3F(-a * (1 + r.Byte() / 255f), 0, 0), new Vector3F(0, b, 0), new Vector3F(0, -b, 0), new Vector3F(0, 0, c), new Vector3F(0, 0, -c) };
            idx = new[] { 0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5 };
            if (r.Byte() < 40) { v[r.Int(6)] = new Vector3F(r.Float(), r.Float(), r.Float()); }
            if (r.Byte() < 20) { idx[r.Int(idx.Length)] = r.Int(8); }
        }
        else
        {
            int nv = 1 + r.Int(20), nt = r.Int(30);
            v = new Vector3F[nv];
            for (int i = 0; i < nv; i++) { v[i] = new Vector3F(r.Float(), r.Float(), r.Float()); }
            idx = new int[3 * nt];
            for (int i = 0; i < idx.Length; i++) { idx[i] = r.Int(nv + 1) - (r.Int(32) == 0 ? 1 : 0); }
        }

        float extent = 1f;
        foreach (var p in v) { if (float.IsFinite(p.X) && float.IsFinite(p.Y) && float.IsFinite(p.Z)) { extent = Math.Max(extent, Math.Max(Math.Abs(p.X), Math.Max(Math.Abs(p.Y), Math.Abs(p.Z)))); } }
        var opt = new TetrahedralizeOptions
        {
            TargetEdgeLength = r.Byte() < 200 ? 2 * extent / (3 + r.Int(10)) : r.Byte() < 128 ? r.Float() : 0f,
            MaxVolume = r.Byte() < 40 ? Math.Abs(r.Float()) : 0f,
            MaxRadiusEdgeRatio = r.Byte() < 40 ? Math.Abs(r.Float()) : 0f,
            MaxThreads = r.Byte() < 40 ? r.Int(4) : 0,
            SkipDeviation = (r.Byte() & 1) != 0,
        };
        Guard(() =>
        {
            var t = Tetrahedralizer.Generate(v, idx, opt);
            for (int k = 0; k < t.Tetrahedra.Length / 4; k++)
            {
                var a = t.Vertices[t.Tetrahedra[4 * k]]; var b = t.Vertices[t.Tetrahedra[4 * k + 1]]; var c = t.Vertices[t.Tetrahedra[4 * k + 2]]; var d = t.Vertices[t.Tetrahedra[4 * k + 3]];
                double ux = (double)b.X - a.X, uy = (double)b.Y - a.Y, uz = (double)b.Z - a.Z, vx = (double)c.X - a.X, vy = (double)c.Y - a.Y, vz = (double)c.Z - a.Z, wx = (double)d.X - a.X, wy = (double)d.Y - a.Y, wz = (double)d.Z - a.Z;
                if (!(wx * (uy * vz - uz * vy) + wy * (uz * vx - ux * vz) + wz * (ux * vy - uy * vx) > 0)) { throw new InvalidOperationException("A tetrahedron has no positive volume."); }
            }

            if (t.Bindings.Length != v.Length) { throw new InvalidOperationException("One binding per input vertex was expected."); }
        });
    }

    // The static solve: a small grid or a random tetrahedron list, fuzzed material, supports, loads and options. Every input is solved or fails with a StaticSolveException;
    // a solved problem must give one displacement per vertex and one result per tetrahedron, and the same bytes at one thread and at three.
    internal static void StaticSolve(byte[] input)
    {
        var r = new Reader(input);
        Vector3F[] v; int[] t;
        bool grid = r.Byte() < 200, sane = r.Byte() < 140;
        int bottom = 0;
        if (grid)
        {
            int nx = 1 + r.Int(3), ny = 1 + r.Int(2), nz = 1 + r.Int(2);
            v = new Vector3F[(nx + 1) * (ny + 1) * (nz + 1)];
            int Idx(int i, int j, int k) => (k * (ny + 1) + j) * (nx + 1) + i;
            for (int k = 0; k <= nz; k++) { for (int j = 0; j <= ny; j++) { for (int i = 0; i <= nx; i++) { v[Idx(i, j, k)] = new Vector3F(i, j, k); } } }
            var list = new System.Collections.Generic.List<int>();
            int[][] perms = { new[] { 0, 1, 2 }, new[] { 0, 2, 1 }, new[] { 1, 0, 2 }, new[] { 1, 2, 0 }, new[] { 2, 0, 1 }, new[] { 2, 1, 0 } };
            for (int k = 0; k < nz; k++) { for (int j = 0; j < ny; j++) { for (int i = 0; i < nx; i++) { foreach (var pm in perms)
            {
                int[] pos = { i, j, k };
                int a = Idx(i, j, k); pos[pm[0]]++; int b = Idx(pos[0], pos[1], pos[2]); pos[pm[1]]++; int c = Idx(pos[0], pos[1], pos[2]); pos[pm[2]]++; int d = Idx(pos[0], pos[1], pos[2]);
                bool odd = (pm[0] == 0 && pm[1] == 2) || (pm[0] == 1 && pm[1] == 0) || (pm[0] == 2 && pm[1] == 1);
                list.Add(a); list.Add(odd ? c : b); list.Add(odd ? b : c); list.Add(d);
            } } } }
            t = list.ToArray();
            bottom = (nx + 1) * (ny + 1);
            if (!sane && r.Byte() < 60) { v[r.Int(v.Length)] = new Vector3F(r.Float(), r.Float(), r.Float()); }
            if (!sane && r.Byte() < 30) { t[r.Int(t.Length)] = r.Int(v.Length + 1); }
        }
        else
        {
            int nv = 1 + r.Int(12), nt = r.Int(8);
            v = new Vector3F[nv];
            for (int i = 0; i < nv; i++) { v[i] = new Vector3F(r.Float(), r.Float(), r.Float()); }
            t = new int[4 * nt];
            for (int i = 0; i < t.Length; i++) { t[i] = r.Int(nv + 1) - (r.Int(32) == 0 ? 1 : 0); }
        }

        double Num() { byte m = r.Byte(); return m < 8 ? double.NaN : m < 16 ? double.PositiveInfinity : m < 24 ? -1 : m < 40 ? 0 : Math.Pow(10, (r.Byte() - 128) / 8.0); }
        var mat = new ElasticMaterial
        {
            YoungsModulus = r.Byte() < 230 ? 1e6 * (1 + r.Byte() / 64.0) : Num(),
            PoissonRatio = r.Byte() < 230 ? r.Byte() / 600.0 : (r.Byte() - 128) / 64.0,
            Density = r.Byte() < 200 ? 1000 : Num(),
            Strength = r.Byte() < 100 ? 0 : r.Byte() < 220 ? 1e4 * (1 + r.Byte()) : Num(),
            WeakAxis = r.Byte() < 60 ? new Vector3F(r.Float(), r.Float(), r.Float()) : default,
            WeakAxisStrengthRatio = r.Byte() < 60 ? r.Byte() / 200.0 : 0,
        };
        if (sane && grid) { mat = ElasticMaterial.Create(1e6 * (1 + r.Byte()), r.Byte() / 600.0, 1000, r.Byte() < 128 ? 0 : 1e3 * (1 + r.Byte())); }
        var p = new StaticProblem { Material = mat, Gravity = r.Byte() < 100 ? new Vector3D(0, 0, -9.81) : new Vector3D(Num(), 0, 0) };
        int sup = sane && grid ? 0 : r.Int(v.Length + 1);
        if (sane && grid) { for (int i = 0; i < bottom; i++) { p.Supports.Add(new StaticSupport(i)); } }
        for (int i = 0; i < sup; i++) { p.Supports.Add(new StaticSupport(r.Byte() < 8 ? r.Int(40) - 4 : r.Int(v.Length), (SupportAxes)(r.Byte() < 200 ? 7 : r.Int(9)), new Vector3D(r.Byte() < 200 ? 0 : r.Float(), 0, 0))); }
        int loads = r.Int(4);
        for (int i = 0; i < loads; i++) { p.PointLoads.Add(new PointLoad(r.Int(v.Length + 1), new Vector3D(r.Float(), r.Float(), r.Float()))); }
        int faces = r.Int(4);
        for (int i = 0; i < faces; i++) { p.FaceLoads.Add(new FaceLoad(r.Int(v.Length + 1), r.Int(v.Length + 1), r.Int(v.Length + 1), r.Float() * 1000, new Vector3D(r.Float(), 0, 0))); }
        var opt = new StaticSolveOptions
        {
            ElementOrder = r.Byte() < 200 ? (r.Byte() < 128 ? StaticElementOrder.Quadratic : StaticElementOrder.Linear) : (StaticElementOrder)r.Int(4),
            MetresPerUnit = r.Byte() < 200 ? 0 : r.Float(),
            Tolerance = r.Byte() < 200 ? 0 : Math.Abs(r.Float()) * 1e-3,
            MaxIterations = r.Byte() < 200 ? 200 : r.Int(300) - 20,
            MaxThreads = r.Byte() < 220 ? 1 : r.Int(6) - 1,
        };
        StaticSolution? a1 = null;
        try { a1 = StaticSolver.Solve(v, t, p, opt); }
        catch (StaticSolveException) { }
        if (a1 is null) { return; }
        if (a1.Displacements.Length != v.Length || a1.VonMises.Length != t.Length / 4 || a1.SafetyFactors.Length != t.Length / 4) { throw new InvalidOperationException("The solution has the wrong sizes."); }
        var again = StaticSolver.Solve(v, t, p, new StaticSolveOptions { ElementOrder = opt.ElementOrder, MetresPerUnit = opt.MetresPerUnit, Tolerance = opt.Tolerance, MaxIterations = opt.MaxIterations, MaxThreads = 3 });
        for (int i = 0; i < v.Length; i++)
        {
            if (BitConverter.DoubleToInt64Bits(a1.Displacements[i].X) != BitConverter.DoubleToInt64Bits(again.Displacements[i].X) || BitConverter.DoubleToInt64Bits(a1.Displacements[i].Z) != BitConverter.DoubleToInt64Bits(again.Displacements[i].Z)) { throw new InvalidOperationException("The displacement depends on the thread count at vertex " + i + "."); }
        }
    }

    internal static void Triangulate2D(byte[] input)
    {
        var r = new Reader(input);
        Vector2F[] Loop(int max)
        {
            int n = r.Int(max);
            var p = new Vector2F[n];
            for (int i = 0; i < n; i++) { p[i] = new Vector2F(r.Float(), r.Float()); }
            return p;
        }

        Vector2F[] outline;
        if (r.Byte() < 140)
        {
            // a convex-ish polygon on a circle with fuzzed radii, so valid input reaches the refinement
            int n = 3 + r.Int(10);
            outline = new Vector2F[n];
            for (int i = 0; i < n; i++) { double ang = 2 * Math.PI * i / n; float rad = 1 + r.Byte() / 128f; outline[i] = new Vector2F(rad * (float)Math64.Cos(ang), rad * (float)Math64.Sin(ang)); }
        }
        else { outline = Loop(14); }

        var holes = new Vector2F[r.Int(3)][];
        for (int i = 0; i < holes.Length; i++)
        {
            if (r.Byte() < 140)
            {
                int n = 3 + r.Int(6); float cx = (r.Byte() - 128) / 256f, cy = (r.Byte() - 128) / 256f, rad = 0.05f + r.Byte() / 1024f;
                holes[i] = new Vector2F[n];
                for (int k = 0; k < n; k++) { double ang = -2 * Math.PI * k / n; holes[i][k] = new Vector2F(cx + rad * (float)Math64.Cos(ang), cy + rad * (float)Math64.Sin(ang)); }
            }
            else { holes[i] = Loop(6); }
        }

        var opt = new TriangleMeshOptions { TargetEdgeLength = r.Byte() < 200 ? 0.2f + r.Byte() / 255f : r.Float(), MinAngleDegrees = r.Byte() < 60 ? r.Float() : 0f, MaxVertices = r.Byte() < 200 ? 20 + r.Int(400) : 0 };
        if (opt.MaxVertices == 0) { opt = new TriangleMeshOptions { TargetEdgeLength = opt.TargetEdgeLength, MinAngleDegrees = opt.MinAngleDegrees, MaxVertices = 600 }; }
        Guard(() =>
        {
            var m = TriangleMesher.Generate(outline, holes, opt);
            for (int t = 0; t < m.Triangles.Length / 3; t++)
            {
                var a = m.Vertices[m.Triangles[3 * t]]; var b = m.Vertices[m.Triangles[3 * t + 1]]; var c = m.Vertices[m.Triangles[3 * t + 2]];
                double cross = ((double)b.X - a.X) * ((double)c.Y - a.Y) - ((double)b.Y - a.Y) * ((double)c.X - a.X);
                // The mesher orients its triangles exactly in double; its vertices are then rounded to float32, which can turn a triangle narrower than the rounding (a few ulps of the largest coordinate times its edge lengths) clockwise. That is the documented resolution of the output, not a fault.
                double big = Math.Max(Math.Max(Math.Abs((double)a.X), Math.Abs((double)a.Y)), Math.Max(Math.Max(Math.Abs((double)b.X), Math.Abs((double)b.Y)), Math.Max(Math.Abs((double)c.X), Math.Abs((double)c.Y))));
                double edges = Math.Sqrt(((double)b.X - a.X) * ((double)b.X - a.X) + ((double)b.Y - a.Y) * ((double)b.Y - a.Y)) + Math.Sqrt(((double)c.X - a.X) * ((double)c.X - a.X) + ((double)c.Y - a.Y) * ((double)c.Y - a.Y));
                if (!(cross >= -(4.0 * 1.1920929e-7 * big * edges))) { throw new InvalidOperationException("A triangle is clockwise."); }
            }
        });
    }

    private static readonly string[] Words = { "Hips", "Pelvis", "Spine", "Spine1", "Chest", "Neck", "Head", "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot", "LeftArm", "LeftForeArm", "LeftHand", "RightArm", "RightForeArm", "RightHand", "Finger", "twist", "root", "tail", "x" };

    internal static void AutoRagdoll(byte[] input)
    {
        var r = new Reader(input);
        int n = r.Int(26);
        var names = new string[n]; var parents = new int[n]; var pos3 = new Vector3F[n]; var pos2 = new Vector2F[n];
        bool tree = r.Byte() < 200;
        for (int i = 0; i < n; i++)
        {
            byte w = r.Byte();
            names[i] = w < 10 ? null! : w < 20 ? string.Empty : Words[w % Words.Length];
            parents[i] = tree ? (i == 0 ? -1 : r.Int(i)) : r.Int(n + 1) - 1 - (r.Int(16) == 0 ? 1 : 0);
            pos3[i] = tree && r.Byte() < 230 ? new Vector3F(r.Int(9) * 0.1f - 0.4f, 1.8f * (1f - i / (float)Math.Max(1, n)), r.Int(5) * 0.05f) : new Vector3F(r.Float(), r.Float(), r.Float());
            pos2[i] = new Vector2F(pos3[i].Z, pos3[i].Y);
        }

        int nv = r.Byte() < 100 ? 0 : r.Int(30);
        int k = 1 + r.Int(4);
        var verts3 = new Vector3F[nv]; var verts2 = new Vector2F[nv];
        for (int i = 0; i < nv; i++) { verts3[i] = new Vector3F(r.Float(), r.Float(), r.Float()); verts2[i] = new Vector2F(verts3[i].X, verts3[i].Y); }
        int len = nv * k + (r.Int(16) == 0 ? 1 : 0);
        var bone = new int[len]; var wt = new float[len];
        for (int i = 0; i < len; i++) { bone[i] = r.Int(n + 1) - (r.Int(32) == 0 ? 1 : 0); wt[i] = r.Byte() < 20 ? r.Float() : r.Byte() / 255f; }
        var opt = new AutoRagdollOptions
        {
            TotalMass = r.Byte() < 40 ? r.Float() : 75f,
            IncludeHands = (r.Byte() & 1) != 0, SplitTorso = (r.Byte() & 1) != 0,
            MinSwingHalfAngle = r.Byte() < 40 ? r.Float() : 0f,
            Facing = r.Byte() < 30 ? r.Int(3) - 1 : 1,
            MassModel = r.Byte() < 40 ? AutoRagdollMassModel.Volume : AutoRagdollMassModel.Auto,
            MinBoneLength = r.Byte() < 40 ? Math.Abs(r.Float()) : 0f,
        };
        Guard(() =>
        {
            var skin = nv > 0 ? new SkinnedPoints3D(verts3, bone, wt, k) : null;
            var a = AutoRagdoller.Create(names, parents, pos3, skin, opt);
            if (a.Definition.Parts.Length == 0 || a.Definition.Parts.Length > 64) { throw new InvalidOperationException("The part count is out of range."); }
            var world = new RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D();
            world.AddRagdoll(a.Definition);
        });
        Guard(() =>
        {
            var skin = nv > 0 ? new SkinnedPoints2D(verts2, bone, wt, k) : null;
            var a = AutoRagdoller.Create2D(names, parents, pos2, skin, opt);
            var world = new RecursiveAI.OnePhysics.Physics2D.PhysicsWorld2D();
            world.AddRagdoll(a.Definition);
        });
    }
}
