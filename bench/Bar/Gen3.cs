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

    public static List<Scene> All() => [Pyramid(10), Pyramid(30), LargePile(), JointChains(), JointGrid(), RagdollPile(), TerrainScene(false), TerrainScene(true), SleepWake(), Queries()];
}
