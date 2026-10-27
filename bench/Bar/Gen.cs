using System.Security.Cryptography;
using System.Text;

namespace Bar;

/// <summary>Deterministic 2D scene generators. Only IEEE-exact arithmetic is used, so the files are byte-identical on every platform.</summary>
public static class Gen
{
    // xorshift64* with a fixed seed; doubles come from the top 53 bits.
    sealed class Rng
    {
        ulong s;
        public Rng(ulong seed) { s = seed == 0 ? 0x9E3779B97F4A7C15UL : seed; }
        public ulong Next() { s ^= s >> 12; s ^= s << 25; s ^= s >> 27; return s * 0x2545F4914F6CDD1DUL; }
        public double Next01() => (Next() >> 11) * (1.0 / 9007199254740992.0);
        public double Range(double a, double b) => a + (b - a) * Next01();
    }

    static readonly double[] Pentagon = [0, 1, -0.9510565163, 0.3090169944, -0.5877852523, -0.8090169944, 0.5877852523, -0.8090169944, 0.9510565163, 0.3090169944];
    static readonly double[] Hexagon = [1, 0, 0.5, 0.8660254038, -0.5, 0.8660254038, -1, 0, -0.5, -0.8660254038, 0.5, -0.8660254038];

    static double[] Scale(double[] p, double k) => p.Select(v => v * k).ToArray();

    static Scene Base(string name, int steps, params string[] metrics)
    {
        var s = new Scene { Name = name, Steps = steps };
        s.Metrics.AddRange(metrics);
        s.Materials.Add((0.6, 0.0));
        return s;
    }

    static BodyDef Add(Scene s, string type, double x, double y, double angle = 0, double vx = 0, double vy = 0, double w = 0)
    {
        var b = new BodyDef { Type = type, X = x, Y = y, Angle = angle, Vx = vx, Vy = vy, W = w };
        s.Bodies.Add(b); return b;
    }

    static void Shape(BodyDef b, string kind, double density, double radius, params double[] p) =>
        b.Shapes.Add(new ShapeDef { Kind = kind, Material = 0, Density = density, Radius = radius, P = p });

    static void Box(BodyDef b, double hx, double hy, double cx = 0, double cy = 0, double angle = 0, double density = 1) =>
        Shape(b, "box", density, 0, hx, hy, cx, cy, angle);

    // One of five mixed shapes, chosen by index, all within a 0.55 extent of the body origin.
    static void MixedShape(BodyDef b, int kind)
    {
        switch (kind % 5)
        {
            case 0: Shape(b, "circle", 1, 0.45, 0, 0); break;
            case 1: Shape(b, "capsule", 1, 0.25, -0.25, 0, 0.25, 0); break;
            case 2: Box(b, 0.4, 0.4); break;
            case 3: Shape(b, "polygon", 1, 0, Scale(Pentagon, 0.5)); break;
            default: Shape(b, "polygon", 1, 0.1, Scale(Hexagon, 0.35)); break;
        }
    }

    static void Container(Scene s, double halfWidth, double height)
    {
        var c = Add(s, "static", 0, 0);
        Box(c, halfWidth + 1, 0.5, 0, -0.5);
        Box(c, 0.5, height / 2, -halfWidth - 0.5, height / 2);
        Box(c, 0.5, height / 2, halfWidth + 0.5, height / 2);
    }

    static Scene Pyramid(int n)
    {
        var s = Base($"pyramid-{n}", 600, "maxPenetration", "finalPenetration", "collapse", "maxSpeed", "nan", "restKE", "jitter", "escapes");
        s.DumpEvery = 60;
        var g = Add(s, "static", 0, -1); Box(g, n + 10, 1);
        int top = 0;
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n - i; j++)
            {
                var b = Add(s, "dynamic", (j - (n - i - 1) / 2.0) * 1.0, 0.5 + i);
                Box(b, 0.5, 0.5); top = s.Bodies.Count - 1;
            }
        s.Params["top"] = [top.ToString()];
        s.Params["container"] = [(-n - 5).ToString(), "-5", (n + 5).ToString(), (n + 10).ToString()];
        return s;
    }

    static void PileBodies(Scene s, Rng rng, int count, int cols, double x0, double dx, double y0, double dy)
    {
        for (int i = 0; i < count; i++)
        {
            int r = i / cols, c = i % cols;
            var b = Add(s, "dynamic", x0 + c * dx, y0 + r * dy, rng.Range(-0.5, 0.5));
            MixedShape(b, (int)(rng.Next() % 5));
        }
    }

    static Scene LargePile(string name = "large-pile")
    {
        var s = Base(name, 600, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "restKE", "jitter");
        s.DumpEvery = 60;
        Container(s, 31, 150);
        PileBodies(s, new Rng(1), 4000, 50, -29.4, 1.2, 2, 1.2);
        s.Params["container"] = ["-31", "0", "31", "150"];
        return s;
    }

    static Scene Tumbler()
    {
        var s = Base("tumbler", 600, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes");
        s.DumpEvery = 60;
        var c = Add(s, "kinematic", 0, 0, 0, 0, 0, 0.5);
        Box(c, 13, 0.5, 0, -12.5); Box(c, 13, 0.5, 0, 12.5); Box(c, 0.5, 12, -12.5, 0); Box(c, 0.5, 12, 12.5, 0);
        for (int i = 0; i < 2000; i++)
        {
            double x = -10.78 + (i % 45) * 0.49, y = -10.78 + (i / 45) * 0.49;
            var b = Add(s, "dynamic", x, y);
            if (i % 2 == 0) Box(b, 0.2, 0.2); else Shape(b, "circle", 1, 0.22, 0, 0);
        }
        s.Params["container"] = ["-18.5", "-18.5", "18.5", "18.5"];
        return s;
    }

    static Scene JointGrid()
    {
        const int n = 100;
        var s = Base("joint-grid", 600, "jointError", "maxSpeed", "nan", "escapes");
        s.DumpEvery = 60;
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                var b = Add(s, j == 0 ? "static" : "dynamic", i, n - j);
                Box(b, 0.4, 0.4);
            }
        for (int j = 0; j < n; j++)
            for (int i = 0; i < n; i++)
            {
                int id = j * n + i;
                if (i + 1 < n && j > 0) s.Joints.Add(new JointDef { A = id, B = id + 1, Ax = 0.5, Bx = -0.5 });
                if (j + 1 < n) s.Joints.Add(new JointDef { A = id, B = id + n, Ay = -0.5, By = 0.5 });
            }
        s.Params["container"] = ["-60", "-200", "160", "150"];
        return s;
    }

    static Scene Chain()
    {
        const int links = 100;
        var s = Base("chain", 600, "jointError", "maxSpeed", "nan");
        s.DumpEvery = 30;
        Add(s, "static", 0, 50);
        for (int i = 0; i < links; i++) Box(Add(s, "dynamic", 0.5 + i, 50), 0.5, 0.125);
        Shape(Add(s, "dynamic", links + 1, 50), "circle", 10, 1.0, 0, 0);
        s.Joints.Add(new JointDef { A = 0, B = 1, Bx = -0.5 });
        for (int i = 1; i < links; i++) s.Joints.Add(new JointDef { A = i, B = i + 1, Ax = 0.5, Bx = -0.5 });
        s.Joints.Add(new JointDef { A = links, B = links + 1, Ax = 0.5, Bx = -1.0 });
        return s;
    }

    static Scene SleepWake()
    {
        var s = Base("sleep-wake", 360, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "asleepBeforeWake", "awakeFinal");
        s.DumpEvery = 30;
        var g = Add(s, "static", 0, -1); Box(g, 80, 1);
        for (int c = 0; c < 60; c++)
            for (int r = 0; r < 50; r++)
            {
                var b = Add(s, "dynamic", (c - 29.5) * 1.1, 0.5 + r); b.Asleep = true; Box(b, 0.5, 0.5);
            }
        Shape(Add(s, "dynamic", 0, 81), "circle", 10, 2.0, 0, 0);
        s.Events.Add(new EventDef { Step = 120, Kind = "wake_all" });
        s.Params["container"] = ["-80", "-5", "80", "200"];
        return s;
    }

    static double SmoothNoise(double[] knots, double x)
    {
        int i = (int)Math.Floor(x); double f = x - i; double t = f * f * (3 - 2 * f);
        return knots[i] * (1 - t) + knots[i + 1] * t;
    }

    static Scene MixedTerrain()
    {
        var s = Base("mixed-terrain", 600, "maxPenetration", "finalPenetration", "maxSpeed", "nan", "escapes", "tunnelled");
        s.DumpEvery = 60;
        var rng = new Rng(7);
        var knots = new double[17]; for (int i = 0; i < knots.Length; i++) knots[i] = rng.Range(-6, 6);
        var pts = new List<double>();
        for (int i = 0; i <= 300; i++) { double x = -150 + i; pts.Add(x); pts.Add(SmoothNoise(knots, i / 20.0)); }
        var t = Add(s, "static", 0, 0);
        Shape(t, "chain", 0, 0, pts.ToArray());
        Shape(t, "segment", 0, 0, -150, pts[1], -150, 100);
        Shape(t, "segment", 0, 0, 150, pts[^1], 150, 100);
        PileBodies(s, rng, 1500, 230, -137.6, 1.2, 12, 1.2);
        s.Params["container"] = ["-155", "-20", "155", "100"];
        s.Params["floor"] = ["-12"];
        return s;
    }

    static Scene Bullets()
    {
        var s = Base("bullets", 180, "tunnelled", "maxSpeed", "nan");
        s.DumpEvery = 5;
        var g = Add(s, "static", 0, -7); Box(g, 100, 1);
        var w = Add(s, "static", 20, 24); Box(w, 0.1, 30);
        for (int i = 0; i < 200; i++)
        {
            var b = Add(s, "dynamic", 0, 0.25 * i, 0, 150, 0); b.Bullet = true;
            Shape(b, "circle", 1, 0.1, 0, 0);
        }
        s.Params["wall"] = ["20.1"];
        s.Params["container"] = ["-5", "-10", "60", "80"];
        return s;
    }

    static Scene Queries()
    {
        var s = LargePile("queries");
        s.Steps = 300; s.Settle = 300; s.QueryReps = 10; s.DumpEvery = 100;
        s.Metrics.Clear(); s.Metrics.AddRange(["maxSpeed", "nan", "escapes"]);
        var r = new Rng(99);
        for (int i = 0; i < 4000; i++)
        {
            double dx, dy, l; do { dx = r.Range(-1, 1); dy = r.Range(-1, 1); l = dx * dx + dy * dy; } while (l < 1e-4 || l > 1);
            l = Math.Sqrt(l);
            s.Queries.Add(new QueryDef { Kind = "ray", P = [r.Range(-35, 35), r.Range(0, 130), dx / l, dy / l, 40] });
        }
        for (int i = 0; i < 4000; i++)
            s.Queries.Add(new QueryDef { Kind = "overlap", P = [r.Range(-30, 30), r.Range(0, 60), r.Range(0.5, 3)] });
        for (int i = 0; i < 4000; i++)
        {
            double dx, dy, l; do { dx = r.Range(-1, 1); dy = r.Range(-1, 1); l = dx * dx + dy * dy; } while (l < 1e-4 || l > 1);
            l = Math.Sqrt(l);
            s.Queries.Add(new QueryDef { Kind = "cast", P = [r.Range(-30, 30), r.Range(0, 100), 0.3, 20 * dx / l, 20 * dy / l] });
        }
        return s;
    }

    public static List<Scene> All() =>
        [Pyramid(20), Pyramid(100), LargePile(), Tumbler(), JointGrid(), Chain(), SleepWake(), MixedTerrain(), Bullets(), Queries()];

    /// <summary>Writes every scene into <paramref name="dir"/> and a SHA256SUMS manifest beside them.</summary>
    public static void WriteAll(string dir)
    {
        Directory.CreateDirectory(dir);
        var sums = new StringBuilder();
        foreach (var s in All().OrderBy(x => x.Name, StringComparer.Ordinal))
        {
            var path = Path.Combine(dir, s.Name + ".scene");
            s.Save(path);
            sums.Append(Convert.ToHexString(SHA256.HashData(File.ReadAllBytes(path))).ToLowerInvariant()).Append("  ").Append(s.Name).Append(".scene\n");
        }
        File.WriteAllText(Path.Combine(dir, "SHA256SUMS"), sums.ToString());
    }
}
