using System.Globalization;
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

    // A thirteen-part humanoid (1.8 m, 75 kg) per person: name, parent, shape (box: half extents; capsule: half length and radius; sphere: radius), centre, mass, joint position, twist limit, swing limits, drive gains.
    // The numbers are those of the library's humanoid preset; joint frames have their twist axis along +y.
    static readonly (int Parent, char Shape, double A, double B, double C, double X, double Y, double Z, double Mass, double Jx, double Jy, double Jz, double Twist, double SwingY, double SwingZ, double K, double Damp, double Max)[] Person =
    [
        (-1, 'b', 0.15, 0.08, 0.09, 0, 0.98, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        (0, 'b', 0.17, 0.18, 0.10, 0, 1.24, 0, 25, 0, 1.06, 0, 0.4, 0.5, 0.5, 6000, 400, 900),
        (1, 's', 0.11, 0, 0, 0, 1.55, 0, 5, 0, 1.44, 0, 0.6, 0.6, 0.6, 600, 40, 150),
        (1, 'c', 0.11, 0.045, 0, 0.22, 1.225, 0, 2.5, 0.22, 1.38, 0, 1.0, 1.6, 1.6, 800, 60, 200),
        (3, 'c', 0.095, 0.04, 0, 0.22, 0.935, 0, 1.5, 0.22, 1.07, 0, 0.3, 2.4, 0.1, 500, 40, 120),
        (1, 'c', 0.11, 0.045, 0, -0.22, 1.225, 0, 2.5, -0.22, 1.38, 0, 1.0, 1.6, 1.6, 800, 60, 200),
        (5, 'c', 0.095, 0.04, 0, -0.22, 0.935, 0, 1.5, -0.22, 1.07, 0, 0.3, 2.4, 0.1, 500, 40, 120),
        (0, 'c', 0.13, 0.08, 0, 0.10, 0.71, 0, 8, 0.10, 0.92, 0, 0.6, 1.2, 1.0, 12000, 800, 1500),
        (7, 'c', 0.15, 0.06, 0, 0.10, 0.29, 0, 4, 0.10, 0.50, 0, 0.1, 2.3, 0.08, 9000, 600, 1200),
        (8, 'b', 0.06, 0.04, 0.15, 0.10, 0.04, 0.05, 1.2, 0.10, 0.08, 0, 0.3, 0.6, 0.4, 5000, 300, 600),
        (0, 'c', 0.13, 0.08, 0, -0.10, 0.71, 0, 8, -0.10, 0.92, 0, 0.6, 1.2, 1.0, 12000, 800, 1500),
        (10, 'c', 0.15, 0.06, 0, -0.10, 0.29, 0, 4, -0.10, 0.50, 0, 0.1, 2.3, 0.08, 9000, 600, 1200),
        (11, 'b', 0.06, 0.04, 0.15, -0.10, 0.04, 0.05, 1.2, -0.10, 0.08, 0, 0.3, 0.6, 0.4, 5000, 300, 600),
    ];

    // Fifty powered humanoids on a floor, each driven to a looping pose (the left arm swings out, the elbow bends, the head and torso turn; triangle wave, period 120/140/160 steps), every fifth hit from behind by a thrown box.
    // Drive axes are in the joint frame (x = the twist axis = +y of the body): world z is joint z, world x is joint -y.
    static Scene RagdollPowered()
    {
        var s = Base3("ragdoll-powered-3d", 600, "poseErrorP95", "fallen", "jointError", "maxSpeed", "nan", "escapes");
        s.DumpEvery = 10; s.Substeps = 8;
        Box3(Add3(s, "static", 0, -1, 0), 40, 1, 40);
        var heads = new List<string>();
        for (int k = 0; k < 50; k++)
        {
            double ox = (k % 10 - 4.5) * 2.5, oz = (k / 10) * 2.5 - 5;
            int first = s.Bodies.Count;
            heads.Add((first + 2).ToString(System.Globalization.CultureInfo.InvariantCulture));
            foreach (var p in Person)
            {
                var b = Add3(s, "dynamic", ox + p.X, p.Y + 0.002, oz + p.Z);
                switch (p.Shape)
                {
                    case 'b': Box3(b, p.A, p.B, p.C, 0, 0, 0, p.Mass / (8 * p.A * p.B * p.C)); break;
                    case 's': Sh(b, "sphere", p.Mass / (4.0 / 3 * Math.PI * p.A * p.A * p.A), p.A, 0, 0, 0); break;
                    default: Sh(b, "capsule", p.Mass / (Math.PI * p.B * p.B * 2 * p.A + 4.0 / 3 * Math.PI * p.B * p.B * p.B), p.B, 0, -p.A, 0, 0, p.A, 0); break;
                }
            }

            int period = 120 + 20 * (k % 3);
            for (int i = 1; i < Person.Length; i++)
            {
                var c = Person[i]; var par = Person[c.Parent];
                var j = new JointDef
                {
                    Kind = "swingtwist", A = first + c.Parent, B = first + i,
                    Ax = c.Jx - par.X, Ay = c.Jy - par.Y, Az = c.Jz - par.Z, Bx = c.Jx - c.X, By = c.Jy - c.Y, Bz = c.Jz - c.Z, Ux = 0, Uy = 1, Uz = 0,
                    HasLimit = true, Lower = -c.Twist, Upper = c.Twist, HasSwing = true, SwingY = c.SwingY, SwingZ = c.SwingZ,
                    HasDrive = true, DriveK = c.K, DriveC = c.Damp, DriveMax = c.Max, DriveAx = 0, DriveAy = 0, DriveAz = 1, DriveAmp = 0, DrivePeriod = period,
                };
                if (i == 3) j.DriveAmp = 1.0;                                    // left upper arm: out sideways by up to a radian about world z
                if (i == 4) { j.DriveAy = 1; j.DriveAz = 0; j.DriveAmp = 0.8; }  // left elbow: bends about world x
                if (i == 2) { j.DriveAx = 1; j.DriveAz = 0; j.DriveAmp = 0.5; }  // head: turns about the vertical (the twist axis)
                if (i == 1) { j.DriveAx = 1; j.DriveAz = 0; j.DriveAmp = 0.2; }  // torso: twists about the vertical
                s.Joints.Add(j);
            }

            if (k % 5 == 0)
            {
                var box = Add3(s, "dynamic", ox, 1.5, oz - 2.5);
                box.Vz = 6;
                Box3(box, 0.125, 0.125, 0.125, 0, 0, 0, 100);
            }
        }

        s.Params["container"] = ["-40", "-5", "40", "30"];
        s.Params["heads"] = heads.ToArray();
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
        s.Params["escapeMargin"] = ["60"]; // the ball landing at about 40 m/s throws boxes up to 30 m past the slab; only bodies beyond that count as escaped
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

    // ---- vehicles-3d ----

    // Height of the vehicle terrain on a 61 x 241 grid of 1 m cells: rolling hills of 0.12 m (smoothstep knots), speed bumps (2 m wide triangles of 0.1 m) at z = 30, 60, 90, 150 and 200, and one ramp (0.4 m rise over 5 m,
    // 2 m flat top, 5 m descent) from z = 120.
    static double[] VehicleTerrain(ulong seed)
    {
        var h = Hills(new Gen.Rng(seed), 60, 240, 1.0, 0.12);
        for (int j = 0; j <= 240; j++)
        {
            double f = 0;
            foreach (int zc in new[] { 30, 60, 90, 150, 200 }) f = Math.Max(f, Math.Max(0, 0.1 - 0.05 * Math.Abs(j - zc)));
            if (j >= 120 && j <= 132) f = Math.Max(f, j <= 125 ? 0.08 * (j - 120) : j <= 127 ? 0.4 : 0.4 - 0.08 * (j - 127));
            for (int i = 0; i <= 60; i++) h[j * 61 + i] += f;
        }
        return h;
    }

    static double VehicleHeightAt(double[] h, double x, double z)
    {
        int i = Math.Clamp((int)x, 0, 59), j = Math.Clamp((int)z, 0, 239); double fu = x - i, fv = z - j;
        return (h[j * 61 + i] * (1 - fu) + h[j * 61 + i + 1] * fu) * (1 - fv) + (h[(j + 1) * 61 + i] * (1 - fu) + h[(j + 1) * 61 + i + 1] * fu) * fv;
    }

    static Scene Vehicles3()
    {
        var s = Base3("vehicles-3d", 600, "maxPenetration", "maxSpeed", "nan", "escapes", "tunnelled", "wheelThroughGround", "rollovers", "suspensionJitter");
        s.DumpEvery = 10;
        s.Params["engines"] = ["onephysics", "jolt"]; // the engines that have a wheeled-vehicle driver
        var hf = VehicleTerrain(31); var ms = VehicleTerrain(32);
        // Height field over x -60 .. 0, triangle mesh (1 m cells, alternating diagonals) over x 0 .. 60; both z 0 .. 240.
        Sh(Add3(s, "static", 0, 0, 0), "heightfield", 0, 0, [61, 241, 1.0, -60, 0, 0, .. hf]);
        {
            var p = new List<double> { 61 * 241, 60 * 240 * 6 };
            for (int j = 0; j <= 240; j++) for (int i = 0; i <= 60; i++) { p.Add(i); p.Add(ms[j * 61 + i]); p.Add(j); }
            for (int j = 0; j < 240; j++)
                for (int i = 0; i < 60; i++)
                {
                    int a = j * 61 + i, b = a + 1, c = a + 61, d = c + 1;
                    if (((i + j) & 1) == 0) p.AddRange([a, c, b, b, c, d]); else p.AddRange([a, c, d, a, d, b]);
                }
            Sh(Add3(s, "static", 0, 0, 0), "mesh", 0, 0, p.ToArray());
        }
        s.Params["terrainHf"] = ["-60", "0"];
        s.Params["terrainMesh"] = ["0", "0"];
        s.Params["wheelRadius"] = ["0.34"];
        int first = s.Bodies.Count;
        // Two fields of 50 cars: ten lanes 5.8 m apart, five rows 25 m apart. Cars of a lane share a throttle, so they keep their spacing; the steering is a slalom of the same phase for every car.
        for (int area = 0; area < 2; area++)
            for (int lane = 0; lane < 10; lane++)
                for (int row = 0; row < 5; row++)
                {
                    double lx = 3 + 5.8 * lane, z = 8 + 25 * row, x = (area == 0 ? -60 : 0) + lx;
                    double ground = VehicleHeightAt(area == 0 ? hf : ms, lx, z);
                    var b = Add3(s, "dynamic", x, ground + 0.34 + 0.5 - 0.11 + 0.05, z);
                    Box3(b, 0.9, 0.25, 2.1, 0, 0, 0, 1500.0 / 3.78);
                    b.Vehicle = new VehicleScript { Preset = "car", Throttle = 0.35 + 0.04 * lane, SteerAmp = 0.12, SteerPeriod = 150, SteerFrom = 90, BrakeFrom = 450 };
                }
        s.Params["cars"] = [first.ToString(), (s.Bodies.Count - first).ToString()];
        s.Params["container"] = ["-80", "-20", "280", "60"];
        s.Params["floor"] = ["-15"];
        return s;
    }

    // ---- buoyancy-3d ----

    // Height of the centre of a sphere of radius r and density ratio f (body density over fluid density) floating with its centre at height c above the surface: the cap below the surface holds f of the sphere.
    static double FloatingSphereCentre(double r, double f)
    {
        double lo = 0, hi = 2 * r;   // depth of the lowest point below the surface
        for (int i = 0; i < 80; i++)
        {
            double h = 0.5 * (lo + hi), v = Math.PI * h * h * (3 * r - h) / 3;
            if (v < f * 4.0 / 3.0 * Math.PI * r * r * r) lo = h; else hi = h;
        }
        return r - 0.5 * (lo + hi);
    }

    // 500 bodies dropped into a 52 x 42 m pool of water 4 m deep (surface at y = 0, a current of (0.2, 0, 0.1) m/s), on a 25 x 20 grid 2 m apart: spheres of density 500, flat boxes of 600 and
    // hexagonal prisms of 600 (they float, each at a depth known from Archimedes), capsules and octahedra of 2500 (they sink to the floor). Shapes of the same fluid density ratio cannot tip over:
    // the boxes are twice as wide as tall, the prisms wider than tall.
    static Scene Buoyancy()
    {
        var s = Base3("buoyancy-3d", 600, "equilibriumDepthError", "sinkersAfloat", "maxSpeed", "nan", "escapes", "restKE");
        s.DumpEvery = 60;
        const double W = 27, Z = 22, FloorTop = -4;
        Box3(Add3(s, "static", 0, FloorTop - 1, 0), W + 2, 1, Z + 2);
        Box3(Add3(s, "static", -W - 1, 2, 0), 1, 6, Z + 2); Box3(Add3(s, "static", W + 1, 2, 0), 1, 6, Z + 2);
        Box3(Add3(s, "static", 0, 2, -Z - 1), W + 2, 6, 1); Box3(Add3(s, "static", 0, 2, Z + 1), W + 2, 6, 1);
        var hex = new double[18];
        for (int k = 0; k < 6; k++)
        {
            double a = k * Math.PI / 3, x = 0.6 * Math.Cos(a), z = 0.6 * Math.Sin(a);
            hex[3 * k] = x; hex[3 * k + 1] = -0.2; hex[3 * k + 2] = z;
        }
        var prism = new List<double>();
        for (int k = 0; k < 6; k++) prism.AddRange([hex[3 * k], 0.2, hex[3 * k + 2]]);
        prism.InsertRange(0, hex);
        var rng = new Gen.Rng(61);
        var floatY = new List<string>(); var sinkers = new List<string>();
        const double surface = 0;
        for (int iz = 0; iz < 20; iz++)
            for (int ix = 0; ix < 25; ix++)
            {
                int idx = s.Bodies.Count;
                double x = (ix - 12) * 2.0 + rng.Range(-0.1, 0.1), z = (iz - 9.5) * 2.0 + rng.Range(-0.1, 0.1);
                int kind = (ix + iz) % 5;
                var b = Add3(s, "dynamic", x, 1.0 + rng.Range(0, 0.6), z);
                switch (kind)
                {
                    case 0:
                        Sh(b, "sphere", 500, 0.45, 0, 0, 0);
                        floatY.AddRange([idx.ToString(), (surface + FloatingSphereCentre(0.45, 0.5)).ToString("R", CultureInfo.InvariantCulture)]);
                        break;
                    case 1:
                        Box3(b, 0.5, 0.25, 0.5, 0, 0, 0, 600);
                        floatY.AddRange([idx.ToString(), (surface - 0.6 * 0.5 + 0.25).ToString("R", CultureInfo.InvariantCulture)]);
                        break;
                    case 2:
                        Sh(b, "hull", 600, 0, prism.ToArray());
                        floatY.AddRange([idx.ToString(), (surface - 0.6 * 0.4 + 0.2).ToString("R", CultureInfo.InvariantCulture)]);
                        break;
                    case 3:
                        Sh(b, "capsule", 2500, 0.25, 0, -0.3, 0, 0, 0.3, 0);
                        sinkers.Add(idx.ToString());
                        break;
                    default:
                        Sh(b, "hull", 2500, 0, Octahedron);
                        sinkers.Add(idx.ToString());
                        break;
                }
            }
        s.Params["fluid"] = [surface.ToString("R", CultureInfo.InvariantCulture), "1000", "0.2", "0", "0.1"];
        s.Params["floatY"] = floatY.ToArray();
        s.Params["sinkers"] = sinkers.ToArray();
        s.Params["poolFloor"] = [FloorTop.ToString("R", CultureInfo.InvariantCulture)];
        s.Params["engines"] = ["onephysics", "jolt"];
        s.Params["container"] = ["-" + (W + 3), (FloorTop - 2).ToString(CultureInfo.InvariantCulture), (W + 3).ToString(CultureInfo.InvariantCulture), "10"];
        s.Params["floor"] = [(FloorTop - 2).ToString(CultureInfo.InvariantCulture)];
        return s;
    }

    // ---- tracked-3d and motorcycle-3d ----

    // Tanks on the rough terrain of the vehicles scene (one height field over x 0 .. 60, z 0 .. 240): six lanes 9.5 m apart, four rows 55 m apart. Tanks of a lane share a throttle (0.25 + 0.04 lane), so they keep their spacing;
    // steering starts at step 90 as a triangle wave of 0.15 and period 240, and everything brakes from step 450.
    static Scene Tracked3()
    {
        var s = Base3("tracked-3d", 600, "maxPenetration", "maxSpeed", "nan", "escapes", "tunnelled", "wheelThroughGround", "rollovers", "suspensionJitter");
        s.DumpEvery = 10;
        s.Params["engines"] = ["onephysics"]; // the engines that have a tracked-vehicle driver
        var hf = VehicleTerrain(41);
        Sh(Add3(s, "static", 0, 0, 0), "heightfield", 0, 0, [61, 241, 1.0, 0, 0, 0, .. hf]);
        s.Params["terrainHf"] = ["0", "0"];
        s.Params["wheelRadius"] = ["0.45"];
        s.Params["wheelsPerCar"] = ["10"];
        int first = s.Bodies.Count;
        for (int lane = 0; lane < 6; lane++)
            for (int row = 0; row < 4; row++)
            {
                double lx = 5 + 9.5 * lane, z = 10 + 55 * row;
                double ground = VehicleHeightAt(hf, lx, z);
                var b = Add3(s, "dynamic", lx, ground + 0.45 + 0.4 - 0.05 + 0.2 + 0.2, z);
                Box3(b, 1.6, 0.5, 3.0, 0, 0, 0, 20000.0 / 19.2);
                b.Vehicle = new VehicleScript { Preset = "tank", Throttle = 0.25 + 0.04 * lane, SteerAmp = 0.15, SteerPeriod = 240, SteerFrom = 90, BrakeFrom = 450 };
            }
        s.Params["cars"] = [first.ToString(), (s.Bodies.Count - first).ToString()];
        s.Params["container"] = ["-20", "-5", "280", "20"];
        s.Params["floor"] = ["-15"];
        return s;
    }

    // Motorcycles on the same terrain: five lanes 12 m apart, four rows 55 m apart. Bikes of a lane share a throttle (0.12 + 0.015 lane), steering starts at step 60 as a triangle wave of 0.25 and period 300 (a slalom;
    // the lean controller follows it), and everything brakes from step 450.
    static Scene Motorcycle3()
    {
        var s = Base3("motorcycle-3d", 600, "maxPenetration", "maxSpeed", "nan", "escapes", "tunnelled", "wheelThroughGround", "fallen", "leanError", "suspensionJitter");
        s.DumpEvery = 10;
        s.Params["engines"] = ["onephysics"]; // the engines that have a motorcycle driver
        var hf = VehicleTerrain(42);
        Sh(Add3(s, "static", 0, 0, 0), "heightfield", 0, 0, [61, 241, 1.0, 0, 0, 0, .. hf]);
        s.Params["terrainHf"] = ["0", "0"];
        s.Params["wheelRadius"] = ["0.32"];
        s.Params["wheelsPerCar"] = ["2"];
        int first = s.Bodies.Count;
        for (int lane = 0; lane < 5; lane++)
            for (int row = 0; row < 4; row++)
            {
                double lx = 6 + 12 * lane, z = 10 + 55 * row;
                double ground = VehicleHeightAt(hf, lx, z);
                var b = Add3(s, "dynamic", lx, ground + 0.32 + 0.35 - 0.1 + 0.05, z);
                Box3(b, 0.15, 0.2, 0.8, 0, 0, 0, 250.0 / 0.192);
                b.Vehicle = new VehicleScript { Preset = "bike", Throttle = 0.12 + 0.015 * lane, SteerAmp = 0.25, SteerPeriod = 300, SteerFrom = 60, BrakeFrom = 450 };
            }
        s.Params["cars"] = [first.ToString(), (s.Bodies.Count - first).ToString()];
        s.Params["bikes"] = [first.ToString(), (s.Bodies.Count - first).ToString()];
        s.Params["container"] = ["-20", "-5", "280", "20"];
        s.Params["floor"] = ["-15"];
        return s;
    }


    // ---- soft bodies ----

    static void Soft(Scene s, string preset, string kind, double x, double y, double z, double mass, params double[] p) =>
        s.Softs.Add(new SoftDef { Preset = preset, Kind = kind, X = x, Y = y, Z = z, Mass = mass, P = p });

    static readonly string[] SoftMetricNames = ["softMaxSpeed", "softPenetration", "softFinalPenetration", "softRestKE", "softEscapes", "softTunnelled", "softNan"];

    static Scene SoftBase(string name, int steps, params string[] metrics)
    {
        var s = Base3(name, steps, [.. SoftMetricNames, .. metrics]);
        s.DumpEvery = 30;
        s.Params["engines"] = ["onephysics", "jitter2"]; // the engines that have a soft-body driver
        return s;
    }

    // A 4 x 4 m cloth of 24 x 24 cells, pinned at the two corners of one edge, held 2 m above a floor with a sphere (radius 0.8) and a box (1.2 m) under it: it falls over both and hangs from its pins.
    static Scene ClothDrape()
    {
        var s = SoftBase("cloth-drape-3d", 600, "edgeStrain");
        Box3(Add3(s, "static", 0, -1, 0), 40, 1, 40);
        Sh(Add3(s, "static", -0.8, 0.8, 0.3), "sphere", 0, 0.8, 0, 0, 0);
        Box3(Add3(s, "static", 1.1, 0.6, 0.5), 0.6, 0.6, 0.6);
        Soft(s, "cloth", "cloth", 0, 2.0, 0, 2, 4, 4, 24, 2);
        s.Params["container"] = ["-8", "-5", "8", "10"];
        s.Params["floor"] = ["-1"];
        return s;
    }

    // Twelve jelly cubes (0.6 m, 3 x 3 x 3 cells, 3 kg) and twelve rubber balls (radius 0.35, 3 kg) dropped in a walled 5 x 5 m yard.
    static Scene SoftPile()
    {
        var s = SoftBase("soft-pile-3d", 600, "volumeError");
        Container(s, 2.5, 6);
        var r = new Gen.Rng(87);
        for (int i = 0; i < 24; i++)
        {
            double x = -1.5 + (i % 4) * 1.0 + r.Range(-0.1, 0.1), z = -1.5 + ((i / 4) % 3) * 1.5 + r.Range(-0.1, 0.1), y = 0.8 + (i / 12) * 1.2 + (i % 2) * 0.5;
            if (i % 2 == 0) Soft(s, "jelly", "box", x, y, z, 3, 0.6, 0.6, 0.6, 3, 3, 3);
            else Soft(s, "rubber", "ball", x, y, z, 3, 0.35, 3);
        }
        s.Params["container"] = ["-3", "-2", "3", "12"];
        s.Params["floor"] = ["-1"];
        return s;
    }

    // Six jelly cushions (1.6 x 0.4 x 1.6 m, 8 kg) with a 0.5 m rigid box of 5 kg released 0.8 m above each: the cushions stop the boxes and push them back.
    static Scene SoftRigid()
    {
        var s = SoftBase("soft-rigid-3d", 600, "volumeError", "maxSpeed", "nan", "restKE");
        Box3(Add3(s, "static", 0, -1, 0), 40, 1, 40);
        for (int i = 0; i < 6; i++)
        {
            double x = (i % 3 - 1) * 3.0, z = (i / 3 - 0.5) * 3.0;
            Soft(s, "jelly", "box", x, 0.25, z, 8, 1.6, 0.4, 1.6, 6, 2, 6);
            Box3(Add3(s, "dynamic", x, 1.5 + 0.1 * i, z), 0.25, 0.25, 0.25, 0, 0, 0, 40);
        }
        s.Params["container"] = ["-10", "-5", "10", "10"];
        s.Params["floor"] = ["-1"];
        return s;
    }

    // The mesh terrain of mesh-terrain-3d with ten jelly cubes and ten rubber cubes dropped on its hills near the origin.
    static Scene SoftMesh()
    {
        var s = TerrainScene(false);
        s.Name = "soft-mesh-3d";
        s.Bodies.RemoveRange(1, s.Bodies.Count - 1);
        s.Metrics.Clear(); s.Metrics.AddRange([.. SoftMetricNames, "volumeError"]);
        s.Steps = 600; s.DumpEvery = 30;
        var (h, n, cell) = Terrain();
        double o = -n * cell / 2;
        var r = new Gen.Rng(88);
        for (int i = 0; i < 20; i++)
        {
            double x = -4.5 + (i % 5) * 2.2 + r.Range(-0.2, 0.2), z = -3.5 + (i / 5) * 2.2 + r.Range(-0.2, 0.2);
            double top = Bilinear(h, n, n, cell, x - o, z - o);
            if (i % 2 == 0) Soft(s, "jelly", "box", x, top + 1.0 + (i % 3) * 0.3, z, 2.5, 0.6, 0.6, 0.6, 3, 3, 3);
            else Soft(s, "rubber", "box", x, top + 1.0 + (i % 3) * 0.3, z, 2.0, 0.5, 0.5, 0.5, 3, 3, 3);
        }
        s.Params["engines"] = ["onephysics", "jitter2"];
        s.Params["container"] = ["-70", "-20", "70", "60"];
        s.Params["floor"] = ["-15"];
        return s;
    }

    public static List<Scene> All() => [Pyramid(10), Pyramid(30), LargePile(), JointChains(), JointGrid(), RagdollPile(), RagdollPowered(), TerrainScene(false), TerrainScene(true), SleepWake(), Queries(), Characters3(), Vehicles3(), Buoyancy(), Tracked3(), Motorcycle3(), ClothDrape(), SoftPile(), SoftRigid(), SoftMesh()];
}
