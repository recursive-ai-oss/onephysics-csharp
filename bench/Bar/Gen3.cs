namespace Bar;

/// <summary>Deterministic 3D scene generators; same rules as the 2D ones (seeded, IEEE-exact arithmetic, no trigonometry).</summary>
public static class Gen3
{
    static Scene Base3(string name, int steps, params string[] metrics)
    {
        var s = Gen.Base(name, steps, metrics);
        s.Dim = 3; s.Gy = -10; s.DumpEvery = 60;
        return s;
    }

    static BodyDef Add3(Scene s, string type, double x, double y, double z, double qx = 0, double qy = 0, double qz = 0, double qw = 1)
    {
        var b = new BodyDef { Type = type, X = x, Y = y, Z = z, Qx = qx, Qy = qy, Qz = qz, Qw = qw };
        s.Bodies.Add(b); return b;
    }

    static void Sh(BodyDef b, string kind, double density, double radius, params double[] p) => Gen.Shape(b, kind, density, radius, p);
    static void Box3(BodyDef b, double hx, double hy, double hz, double cx = 0, double cy = 0, double cz = 0, double density = 1) =>
        Sh(b, "box", density, 0, hx, hy, hz, cx, cy, cz, 0, 0, 0, 1);

    static readonly double[] Octahedron = [0.5, 0, 0, -0.5, 0, 0, 0, 0.5, 0, 0, -0.5, 0, 0, 0, 0.5, 0, 0, -0.5];
    static readonly double[] Wedge = [-0.4, -0.3, -0.3, 0.4, -0.3, -0.3, 0.4, -0.3, 0.3, -0.4, -0.3, 0.3, -0.4, 0.3, 0, 0.4, 0.3, 0];

    // One of five convex shapes inside a 0.58 radius of the body origin.
    static void Mixed(BodyDef b, int kind)
    {
        switch (kind % 5)
        {
            case 0: Sh(b, "sphere", 1, 0.4, 0, 0, 0); break;
            case 1: Sh(b, "capsule", 1, 0.25, 0, -0.2, 0, 0, 0.2, 0); break;
            case 2: Box3(b, 0.3, 0.3, 0.3); break;
            case 3: Sh(b, "hull", 1, 0, Octahedron); break;
            default: Sh(b, "hull", 1, 0, Wedge); break;
        }
    }

    // A small random tilt as a unit quaternion (sqrt is exact, so the file is identical everywhere).
    static BodyDef Tilted(Scene s, Gen.Rng r, double x, double y, double z)
    {
        double a = r.Range(-0.3, 0.3), b = r.Range(-0.3, 0.3), c = r.Range(-0.3, 0.3), n = Math.Sqrt(a * a + b * b + c * c + 1);
        return Add3(s, "dynamic", x, y, z, a / n, b / n, c / n, 1 / n);
    }

    static void Container(Scene s, double half, double height)
    {
        var c = Add3(s, "static", 0, 0, 0);
        Box3(c, half + 1, 0.5, half + 1, 0, -0.5, 0);
        Box3(c, 0.5, height / 2, half, -half - 0.5, height / 2, 0);
        Box3(c, 0.5, height / 2, half, half + 0.5, height / 2, 0);
        Box3(c, half, height / 2, 0.5, 0, height / 2, -half - 0.5);
        Box3(c, half, height / 2, 0.5, 0, height / 2, half + 0.5);
    }

    static Scene Pyramid(int n)
    {
        var s = Base3($"pyramid-3d-{n}", 600, "maxPenetration", "finalPenetration", "collapse", "maxSpeed", "nan", "restKE", "jitter", "escapes");
        Box3(Add3(s, "static", 0, -1, 0), n + 10, 1, n + 10);
        for (int i = 0; i < n; i++)
            for (int a = 0; a < n - i; a++)
                for (int c = 0; c < n - i; c++)
                    Box3(Add3(s, "dynamic", a - (n - i - 1) / 2.0, 0.5 + i, c - (n - i - 1) / 2.0), 0.5, 0.5, 0.5);
        s.Params["top"] = [(s.Bodies.Count - 1).ToString()];
        s.Params["container"] = [(-n - 5).ToString(), "-5", (n + 5).ToString(), (n + 10).ToString()];
        return s;
    }

    static void PileBodies(Scene s, Gen.Rng r, int count, int cols, double x0, double dx, double y0, double dy)
    {
        for (int i = 0; i < count; i++)
        {
            int layer = i / (cols * cols), k = i % (cols * cols);
            Mixed(Tilted(s, r, x0 + (k % cols) * dx, y0 + layer * dy, x0 + (k / cols) * dx), (int)(r.Next() % 5));
        }
    }

    static Scene LargePile(string name = "large-pile-3d")
    {
        var s = Base3(name, 600, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "restKE", "jitter");
        Container(s, 9, 80);
        PileBodies(s, new Gen.Rng(3), 3000, 14, -7.8, 1.2, 2, 1.2);
        s.Params["container"] = ["-9", "0", "9", "80"];
        return s;
    }

    static Scene Queries()
    {
        var s = LargePile("queries-3d");
        s.Steps = 300; s.Settle = 300; s.QueryReps = 10; s.DumpEvery = 100;
        s.Metrics.Clear(); s.Metrics.AddRange(["maxSpeed", "nan", "escapes"]);
        var r = new Gen.Rng(98);
        double[] Dir()
        {
            double x, y, z, l; do { x = r.Range(-1, 1); y = r.Range(-1, 1); z = r.Range(-1, 1); l = x * x + y * y + z * z; } while (l < 1e-4 || l > 1);
            l = Math.Sqrt(l); return [x / l, y / l, z / l];
        }
        for (int i = 0; i < 3000; i++) { var d = Dir(); s.Queries.Add(new QueryDef { Kind = "ray", P = [r.Range(-9, 9), r.Range(0, 50), r.Range(-9, 9), d[0], d[1], d[2], 40] }); }
        for (int i = 0; i < 3000; i++) s.Queries.Add(new QueryDef { Kind = "overlap", P = [r.Range(-8, 8), r.Range(0, 25), r.Range(-8, 8), r.Range(0.5, 2.5)] });
        for (int i = 0; i < 3000; i++) { var d = Dir(); s.Queries.Add(new QueryDef { Kind = "cast", P = [r.Range(-8, 8), r.Range(0, 40), r.Range(-8, 8), 0.3, 15 * d[0], 15 * d[1], 15 * d[2]] }); }
        return s;
    }

    static Scene JointChains()
    {
        const int chains = 16, links = 40;
        var s = Base3("joint-chains-3d", 600, "jointError", "maxSpeed", "nan");
        s.DumpEvery = 30;
        for (int c = 0; c < chains; c++)
        {
            double z = c * 3 - 22.5;
            int anchor = s.Bodies.Count;
            Add3(s, "static", 0, 50, z);
            for (int i = 0; i < links; i++) Box3(Add3(s, "dynamic", 0.5 + i, 50, z), 0.5, 0.1, 0.1);
            Sh(Add3(s, "dynamic", links + 0.8, 50, z), "sphere", 10, 0.8, 0, 0, 0);
            s.Joints.Add(new JointDef { Kind = "point", A = anchor, B = anchor + 1, Bx = -0.5 });
            for (int i = 1; i < links; i++) s.Joints.Add(new JointDef { Kind = "point", A = anchor + i, B = anchor + i + 1, Ax = 0.5, Bx = -0.5 });
            s.Joints.Add(new JointDef { Kind = "point", A = anchor + links, B = anchor + links + 1, Ax = 0.5, Bx = -0.8 });
        }
        return s;
    }

    static Scene JointGrid()
    {
        const int n = 40;
        var s = Base3("joint-grid-3d", 600, "jointError", "maxSpeed", "nan", "escapes");
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++) Box3(Add3(s, j == 0 ? "static" : "dynamic", i, n - j, 0), 0.4, 0.4, 0.2);
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                int id = j * n + i;
                if (i + 1 < n && j > 0) s.Joints.Add(new JointDef { Kind = "point", A = id, B = id + 1, Ax = 0.5, Bx = -0.5 });
                if (j + 1 < n) s.Joints.Add(new JointDef { Kind = "point", A = id, B = id + n, Ay = -0.5, By = 0.5 });
            }
        s.Params["container"] = ["-60", "-100", "100", "80"];
        return s;
    }

    // Eleven-part limp ragdoll: point joints only (limits and powered joints belong to the ragdoll stage).
    static Scene RagdollPile()
    {
        var s = Base3("ragdoll-pile-3d", 600, "jointError", "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "restKE");
        Container(s, 10, 60);
        for (int k = 0; k < 150; k++)
        {
            double ox = (k % 5) * 3 - 6, oz = ((k / 5) % 5) * 3 - 6, oy = 3 + (k / 25) * 3;
            int t = s.Bodies.Count;
            Box3(Add3(s, "dynamic", ox, oy + 1.2, oz), 0.25, 0.35, 0.12);                  // torso t
            Sh(Add3(s, "dynamic", ox, oy + 1.7, oz), "sphere", 1, 0.15, 0, 0, 0);          // head t+1
            foreach (double side in new[] { -1.0, 1.0 })
            {
                Sh(Add3(s, "dynamic", ox + side * 0.4, oy + 1.3, oz), "capsule", 1, 0.07, 0, -0.12, 0, 0, 0.12, 0);   // upper arm
                Sh(Add3(s, "dynamic", ox + side * 0.4, oy + 1.06, oz), "capsule", 1, 0.06, 0, -0.12, 0, 0, 0.12, 0);  // forearm
                Sh(Add3(s, "dynamic", ox + side * 0.12, oy + 0.67, oz), "capsule", 1, 0.09, 0, -0.18, 0, 0, 0.18, 0); // thigh
                Sh(Add3(s, "dynamic", ox + side * 0.12, oy + 0.39, oz), "capsule", 1, 0.08, 0, -0.1, 0, 0, 0.1, 0);   // shin
            }
            int L = t + 2, R = t + 6; // left (side -1) and right limb groups: arm, forearm, thigh, shin
            void J(int a, int b, double ax, double ay, double bx, double by) => s.Joints.Add(new JointDef { Kind = "point", A = a, B = b, Ax = ax, Ay = ay, Bx = bx, By = by });
            J(t, t + 1, 0, 0.35, 0, -0.15);
            J(t, L, -0.4, 0.22, 0, 0.12); J(t, R, 0.4, 0.22, 0, 0.12);
            J(L, L + 1, 0, -0.12, 0, 0.12); J(R, R + 1, 0, -0.12, 0, 0.12);
            J(t, L + 2, -0.12, -0.35, 0, 0.18); J(t, R + 2, 0.12, -0.35, 0, 0.18);
            J(L + 2, L + 3, 0, -0.18, 0, 0.1); J(R + 2, R + 3, 0, -0.18, 0, 0.1);
        }
        s.Params["container"] = ["-10", "0", "10", "60"];
        return s;
    }

    static (double[] h, int n, double cell) Terrain()
    {
        var r = new Gen.Rng(11);
        var knots = new double[9 * 9]; for (int i = 0; i < knots.Length; i++) knots[i] = r.Range(-3, 3);
        const int n = 63; const double cell = 2;
        var h = new double[(n + 1) * (n + 1)];
        for (int j = 0; j <= n; j++)
            for (int i = 0; i <= n; i++)
            {
                double u = i / 8.0, v = j / 8.0; int iu = Math.Min((int)u, 7), iv = Math.Min((int)v, 7); double fu = u - iu, fv = v - iv;
                double su = fu * fu * (3 - 2 * fu), sv = fv * fv * (3 - 2 * fv);
                double a = knots[iv * 9 + iu] * (1 - su) + knots[iv * 9 + iu + 1] * su, b = knots[(iv + 1) * 9 + iu] * (1 - su) + knots[(iv + 1) * 9 + iu + 1] * su;
                h[j * (n + 1) + i] = a * (1 - sv) + b * sv;
            }
        return (h, n, cell);
    }

    static Scene TerrainScene(bool heightfield)
    {
        var s = Base3(heightfield ? "heightfield-terrain-3d" : "mesh-terrain-3d", 600, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "tunnelled");
        var (h, n, cell) = Terrain();
        var t = Add3(s, "static", 0, 0, 0);
        double o = -n * cell / 2;
        if (heightfield) Sh(t, "heightfield", 0, 0, [n + 1, n + 1, cell, o, 0, o, .. h]);
        else
        {
            var p = new List<double> { (n + 1) * (n + 1), n * n * 6 };
            for (int j = 0; j <= n; j++) for (int i = 0; i <= n; i++) { p.Add(o + i * cell); p.Add(h[j * (n + 1) + i]); p.Add(o + j * cell); }
            for (int j = 0; j < n; j++)
                for (int i = 0; i < n; i++)
                {
                    int a = j * (n + 1) + i, b = a + 1, c = a + n + 1, d = c + 1;
                    p.AddRange([a, c, b, b, c, d]);
                }
            Sh(t, "mesh", 0, 0, p.ToArray());
        }
        PileBodies(s, new Gen.Rng(5), 1296, 36, -21, 1.2, 10, 1.2);
        s.Params["container"] = ["-70", "-20", "70", "60"];
        s.Params["floor"] = ["-15"];
        return s;
    }

    static Scene SleepWake()
    {
        var s = Base3("sleep-wake-3d", 360, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "asleepBeforeWake", "awakeFinal");
        s.DumpEvery = 30;
        Box3(Add3(s, "static", 0, -1, 0), 40, 1, 40);
        for (int a = 0; a < 6; a++)
            for (int c = 0; c < 6; c++)
                for (int r = 0; r < 40; r++) { var b = Add3(s, "dynamic", (a - 2.5) * 1.1, 0.5 + r, (c - 2.5) * 1.1); b.Asleep = true; Box3(b, 0.5, 0.5, 0.5); }
        Sh(Add3(s, "dynamic", 0, 81, 0), "sphere", 10, 2.0, 0, 0, 0);
        s.Events.Add(new EventDef { Step = 120, Kind = "wake_all" });
        s.Params["container"] = ["-40", "-5", "40", "200"];
        return s;
    }

    // ---- characters-3d ----

    // Smooth hills on a coarse grid of knots (ring of zeros, so zone edges meet the flat ground): bilinear with smoothstep weights.
    static double[] Hills(Gen.Rng r, int cols, int rows, double cell, double amplitude)
    {
        int ku = (int)(cols * cell / 10) + 1, kv = (int)(rows * cell / 10) + 1;
        var knots = new double[(ku + 1) * (kv + 1)];
        for (int j = 1; j < kv; j++) for (int i = 1; i < ku; i++) knots[j * (ku + 1) + i] = r.Range(-amplitude, amplitude);
        var h = new double[(cols + 1) * (rows + 1)];
        for (int j = 0; j <= rows; j++)
            for (int i = 0; i <= cols; i++)
            {
                double u = i * cell / 10, v = j * cell / 10; int iu = Math.Min((int)u, ku - 1), iv = Math.Min((int)v, kv - 1); double fu = u - iu, fv = v - iv;
                double su = fu * fu * (3 - 2 * fu), sv = fv * fv * (3 - 2 * fv);
                double a = knots[iv * (ku + 1) + iu] * (1 - su) + knots[iv * (ku + 1) + iu + 1] * su, b = knots[(iv + 1) * (ku + 1) + iu] * (1 - su) + knots[(iv + 1) * (ku + 1) + iu + 1] * su;
                h[j * (cols + 1) + i] = a * (1 - sv) + b * sv;
            }
        return h;
    }

    // Block (voxel chunk) heights: levels of 0.25 m whose neighbours differ by at most one level.
    static int[,] BlockLevels(Gen.Rng r, int cols, int rows)
    {
        var lv = new int[cols, rows];
        for (int i = 0; i < cols; i++)
            for (int j = 0; j < rows; j++)
            {
                int lo = 0, hi = 6;
                if (i == 0) hi = 1;
                if (i > 0) { lo = Math.Max(lo, lv[i - 1, j] - 1); hi = Math.Min(hi, lv[i - 1, j] + 1); }
                if (j > 0) { lo = Math.Max(lo, lv[i, j - 1] - 1); hi = Math.Min(hi, lv[i, j - 1] + 1); }
                if (lo > hi) lo = hi;
                lv[i, j] = lo + (int)(r.Next() % (ulong)(hi - lo + 1));
            }
        return lv;
    }

    static void Quad(List<double> v, List<int> idx, double[] a, double[] b, double[] c, double[] d)
    {
        int k = v.Count / 3;
        foreach (var p in new[] { a, b, c, d }) v.AddRange(p);
        idx.AddRange([k, k + 1, k + 2, k, k + 2, k + 3]);
    }

    static Scene Characters3()
    {
        var s = Base3("characters-3d", 600, "maxPenetration", "maxSpeed", "nan", "escapes", "tunnelled", "charJitter", "stairSuccess", "slopeViolations");
        s.DumpEvery = 30;
        s.Params["engines"] = ["onephysics", "jolt"]; // the engines that have a character controller driver
        var r = new Gen.Rng(21);

        // Flat ground under the stairs, crates, block field, ramps and platforms (x -20 .. 170).
        Box3(Add3(s, "static", 75, -0.5, 0), 95, 0.5, 12);

        // R1: a triangle-mesh hill field, x -100 .. -60 (1 m cells, alternating diagonals so seams run both ways).
        {
            const int cols = 40, rows = 20; var h = Hills(r, cols, rows, 1.0, 0.8);
            var p = new List<double> { (cols + 1) * (rows + 1), cols * rows * 6 };
            for (int j = 0; j <= rows; j++) for (int i = 0; i <= cols; i++) { p.Add(-100 + i); p.Add(h[j * (cols + 1) + i]); p.Add(-10 + j); }
            for (int j = 0; j < rows; j++)
                for (int i = 0; i < cols; i++)
                {
                    int a = j * (cols + 1) + i, b = a + 1, c = a + cols + 1, d = c + 1;
                    if (((i + j) & 1) == 0) p.AddRange([a, c, b, b, c, d]); else p.AddRange([a, c, d, a, d, b]);
                }
            Sh(Add3(s, "static", 0, 0, 0), "mesh", 0, 0, p.ToArray());
            var terrainMesh = h;
            AddWalkers(s, r, 50, i => (-98 + r.Range(0, 36), -9 + r.Range(0, 18)), (x, z) => Bilinear(terrainMesh, cols, rows, 1.0, x + 100, z + 10));
        }

        // R2: the same kind of hills as a height field, x -60 .. -20.
        {
            const int cols = 40, rows = 20; var h = Hills(r, cols, rows, 1.0, 0.8);
            var p = new List<double> { cols + 1, rows + 1, 1.0, -60, 0, -10 };
            p.AddRange(h);
            Sh(Add3(s, "static", 0, 0, 0), "heightfield", 0, 0, p.ToArray());
            var terrainField = h;
            AddWalkers(s, r, 50, i => (-58 + r.Range(0, 36), -9 + r.Range(0, 18)), (x, z) => Bilinear(terrainField, cols, rows, 1.0, x + 60, z + 10));
        }

        // R3: forty 0.6 m crates (about 22 kg) in 20 lanes and forty pushers, half of them pushing.
        for (int lane = 0; lane < 20; lane++)
            for (int k = 0; k < 2; k++)
                Box3(Add3(s, "dynamic", -12 + k * 3.0 + (lane % 3) * 0.5, 0.3 + 0.001, -9.5 + lane), 0.3, 0.3, 0.3, 0, 0, 0, 100);
        for (int i = 0; i < 40; i++)
        {
            var b = Add3(s, "dynamic", -19 + (i % 2) * 1.5, 0.95 + 0.001, -9.5 + i / 2);
            Sh(b, "capsule", 1, 0.3, 0, -0.6, 0, 0, 0.6, 0);
            b.Script = new CharacterScript { Speed = 1.0 + 0.25 * (i % 3), Period = 480, Jump = 0, Stop = 420, Push = i % 2 == 0 ? 0.8 : 0 };
        }

        // R4: three staircases of twelve 0.5 m treads with risers of 0.20, 0.25 and 0.29 m, each ending in a plateau; twelve climbers on each.
        double[] rise = [0.20, 0.25, 0.29], z0s = [-10, -4, 2];
        for (int g = 0; g < 3; g++)
        {
            for (int k = 1; k <= 12; k++) Box3(Add3(s, "static", 0.25 + 0.5 * (k - 1), rise[g] * k / 2, z0s[g] + 3), 0.25, rise[g] * k / 2, 3);
            Box3(Add3(s, "static", 14, rise[g] * 12 / 2, z0s[g] + 3), 8, rise[g] * 12 / 2, 3);
            int first = s.Bodies.Count;
            for (int i = 0; i < 12; i++)
            {
                var b = Add3(s, "dynamic", -5 + (i % 3) * 0.7, 0.95 + 0.001, z0s[g] + 0.5 + 0.5 * i);
                Sh(b, "capsule", 1, 0.3, 0, -0.6, 0, 0, 0.6, 0);
                b.Script = new CharacterScript { Speed = 2.0, Period = 900, Jump = 0, Stop = 420, Push = 0 };
            }
            s.Params["stairs" + g] = [first.ToString(), "12", (rise[g] * 12 + 0.9 - 0.15).ToString("R", System.Globalization.CultureInfo.InvariantCulture)];
        }

        // R5: a field of 1 m blocks (a voxel chunk as one mesh), x 34 .. 58, heights in steps of 0.25 m.
        {
            const int cols = 24, rows = 20; var lv = BlockLevels(r, cols, rows);
            var v = new List<double>(); var idx = new List<int>();
            double H(int i, int j) => (i < 0 || j < 0 || i >= cols || j >= rows) ? 0 : 0.25 * lv[i, j];
            for (int i = 0; i < cols; i++)
                for (int j = 0; j < rows; j++)
                {
                    double x0 = 34 + i, x1 = x0 + 1, z0 = -10 + j, z1 = z0 + 1, h = H(i, j);
                    Quad(v, idx, [x0, h, z0], [x0, h, z1], [x1, h, z1], [x1, h, z0]);
                    double lo;
                    if ((lo = H(i - 1, j)) < h) Quad(v, idx, [x0, lo, z0], [x0, lo, z1], [x0, h, z1], [x0, h, z0]);
                    if ((lo = H(i + 1, j)) < h) Quad(v, idx, [x1, lo, z1], [x1, lo, z0], [x1, h, z0], [x1, h, z1]);
                    if ((lo = H(i, j - 1)) < h) Quad(v, idx, [x1, lo, z0], [x0, lo, z0], [x0, h, z0], [x1, h, z0]);
                    if ((lo = H(i, j + 1)) < h) Quad(v, idx, [x0, lo, z1], [x1, lo, z1], [x1, h, z1], [x0, h, z1]);
                }
            var p = new List<double> { v.Count / 3, idx.Count };
            p.AddRange(v); foreach (int k in idx) p.Add(k);
            Sh(Add3(s, "static", 0, 0, 0), "mesh", 0, 0, p.ToArray());
            for (int i = 0; i < 40; i++)
            {
                var b = Add3(s, "dynamic", 32.6 + (i % 2) * 0.8, H(0, i / 2) + 0.95 + 0.05, -9.5 + i / 2);
                Sh(b, "capsule", 1, 0.3, 0, -0.6, 0, 0, 0.6, 0);
                b.Script = new CharacterScript { Speed = 1.5, Period = 300, Jump = i % 4 == 0 ? 120 : 0, Stop = 420, Push = 0 };
            }
        }

        // R6: four ramps of 30, 40, 50 and 60 degrees (the slope limit is 45) rising from x = 66 to 80; ten walkers at the foot of each.
        {
            double[] tan = [0.5773502691896257, 0.8390996311772800, 1.1917535925942100, 1.7320508075688772], z0r = [-10, -5, 0, 5];
            for (int g = 0; g < 4; g++)
            {
                double top = tan[g] * 14;
                Sh(Add3(s, "static", 0, 0, 0), "mesh", 0, 0, [4, 6, 66, 0, z0r[g], 66, 0, z0r[g] + 5, 80, top, z0r[g], 80, top, z0r[g] + 5, 0, 1, 2, 2, 1, 3]);
                int first = s.Bodies.Count;
                for (int i = 0; i < 10; i++)
                {
                    var b = Add3(s, "dynamic", 61 + (i % 2) * 1.2, 0.95 + 0.001, z0r[g] + 0.25 + 0.5 * i);
                    Sh(b, "capsule", 1, 0.3, 0, -0.6, 0, 0, 0.6, 0);
                    b.Script = new CharacterScript { Speed = 1.5, Period = 900, Jump = 0, Stop = 420, Push = 0 };
                }
                if (g >= 2) s.Params["steep" + (g - 2)] = [first.ToString(), "10", "1.35"];
            }
        }

        // R7: three platforms sliding at 1.5 m/s and three turning at 0.4 rad/s, top at y = 1, carrying 44 characters.
        for (int g = 0; g < 6; g++)
        {
            bool moving = g < 3; double px = moving ? 98 : 125, pz = -6 + 6.0 * (g % 3);
            var pl = Add3(s, "kinematic", px, 0.75, pz);
            if (moving) pl.Vx = 1.5; else pl.Wy = 0.4;
            Box3(pl, 2, 0.25, 2);
            int n = g < 2 ? 8 : 7;
            if (!moving) { if (g == 3) s.Params["rotatingRiders"] = []; s.Params["rotatingRiders"] = [.. s.Params["rotatingRiders"], .. new[] { (s.Bodies.Count).ToString(), n.ToString() }]; }
            for (int i = 0; i < n; i++)
            {
                var b = Add3(s, "dynamic", px - 1.2 + 0.8 * (i % 4), 1.0 + 0.95 + 0.05, pz - 0.9 + 1.2 * (i / 4));
                Sh(b, "capsule", 1, 0.3, 0, -0.6, 0, 0, 0.6, 0);
                b.Script = new CharacterScript { Speed = 0.5, Period = 60 + 30 * (i % 5), Jump = 0, Stop = 420, Push = 0 };
            }
        }

        s.Params["container"] = ["-105", "-10", "175", "40"];
        s.Params["floor"] = ["-5"];
        return s;
    }

    static double Bilinear(double[] h, int cols, int rows, double cell, double u, double v)
    {
        int i = Math.Min(Math.Max((int)(u / cell), 0), cols - 1), j = Math.Min(Math.Max((int)(v / cell), 0), rows - 1);
        double fu = u / cell - i, fv = v / cell - j;
        double a = h[j * (cols + 1) + i] * (1 - fu) + h[j * (cols + 1) + i + 1] * fu, b = h[(j + 1) * (cols + 1) + i] * (1 - fu) + h[(j + 1) * (cols + 1) + i + 1] * fu;
        return a * (1 - fv) + b * fv;
    }

    // Walkers dropped just above hilly terrain; the surface height at the start is the interpolated one plus a small margin (the mover settles them).
    static void AddWalkers(Scene s, Gen.Rng r, int count, Func<int, (double x, double z)> place, Func<double, double, double> height)
    {
        for (int i = 0; i < count; i++)
        {
            var (x, z) = place(i);
            var b = Add3(s, "dynamic", x, height(x, z) + 0.95 + 0.25, z);
            Sh(b, "capsule", 1, 0.3, 0, -0.6, 0, 0, 0.6, 0);
            b.Script = new CharacterScript { Speed = 1.0 + 0.5 * (i % 4), Period = 60 + 30 * (i % 5), Jump = i % 3 == 0 ? 150 : 0, Stop = 420, Push = 0 };
        }
    }

    public static List<Scene> All() => [Pyramid(10), Pyramid(30), LargePile(), JointChains(), JointGrid(), RagdollPile(), TerrainScene(false), TerrainScene(true), SleepWake(), Queries(), Characters3()];
}
