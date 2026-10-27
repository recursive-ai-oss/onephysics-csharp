using System.Globalization;

namespace Bar;

/// <summary>Reads per-step state dumps and computes the stability metrics named by a scene. Engine-neutral: poses and velocities only.</summary>
public static class Metrics
{
    public sealed class Frame { public int Step; public double[] D = []; } // 7 doubles per body: x y angle vx vy w awake

    public static (int n, List<Frame> frames) ReadDump(string path)
    {
        using var f = File.OpenRead(path);
        using var r = new BinaryReader(f);
        var magic = r.ReadBytes(8);
        if (System.Text.Encoding.ASCII.GetString(magic) != "OPDUMP01") throw new InvalidDataException("bad dump magic");
        int n = (int)r.ReadUInt32(); r.ReadUInt32();
        var frames = new List<Frame>();
        while (f.Position < f.Length)
        {
            var fr = new Frame { Step = (int)r.ReadUInt32() };
            r.ReadUInt32();
            fr.D = new double[n * 7];
            for (int i = 0; i < fr.D.Length; i++) fr.D[i] = r.ReadDouble();
            frames.Add(fr);
        }
        return (n, frames);
    }

    // A convex core (points) inflated by a radius; segments have two points, circles one.
    sealed class Part { public int Body; public double[] Local = []; public double Radius; }

    static List<Part> Parts(Scene s)
    {
        var parts = new List<Part>();
        for (int bi = 0; bi < s.Bodies.Count; bi++)
            foreach (var sh in s.Bodies[bi].Shapes)
            {
                var p = sh.P;
                switch (sh.Kind)
                {
                    case "circle": parts.Add(new Part { Body = bi, Local = [p[0], p[1]], Radius = sh.Radius }); break;
                    case "capsule": parts.Add(new Part { Body = bi, Local = [p[0], p[1], p[2], p[3]], Radius = sh.Radius }); break;
                    case "segment": parts.Add(new Part { Body = bi, Local = [p[0], p[1], p[2], p[3]], Radius = 0 }); break;
                    case "polygon": parts.Add(new Part { Body = bi, Local = p, Radius = sh.Radius }); break;
                    case "box":
                    {
                        double hx = p[0], hy = p[1], cx = p[2], cy = p[3], a = p[4];
                        double c = Math.Cos(a), sn = Math.Sin(a);
                        var pts = new double[8]; int k = 0;
                        foreach (var (qx, qy) in new[] { (-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy) })
                        { pts[k++] = cx + c * qx - sn * qy; pts[k++] = cy + sn * qx + c * qy; }
                        parts.Add(new Part { Body = bi, Local = pts, Radius = sh.Radius }); break;
                    }
                    case "chain":
                        for (int i = 0; i + 3 < p.Length; i += 2)
                            parts.Add(new Part { Body = bi, Local = [p[i], p[i + 1], p[i + 2], p[i + 3]], Radius = 0 });
                        break;
                    default: throw new InvalidDataException("unknown shape " + sh.Kind);
                }
            }
        return parts;
    }

    static List<(double x, double y)> Hull(List<(double x, double y)> pts)
    {
        pts.Sort((a, b) => a.x != b.x ? a.x.CompareTo(b.x) : a.y.CompareTo(b.y));
        for (int i = pts.Count - 1; i > 0; i--) if (pts[i] == pts[i - 1]) pts.RemoveAt(i);
        var h = new List<(double x, double y)>();
        static double Cross((double x, double y) o, (double x, double y) a, (double x, double y) b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
        for (int pass = 0; pass < 2; pass++)
        {
            int start = h.Count;
            foreach (var p in pass == 0 ? pts : Enumerable.Reverse(pts))
            {
                while (h.Count >= start + 2 && Cross(h[^2], h[^1], p) <= 0) h.RemoveAt(h.Count - 1);
                h.Add(p);
            }
            h.RemoveAt(h.Count - 1);
        }
        if (h.Count == 0) h.Add(pts[0]);
        return h;
    }

    static double SegDist(double px, double py, double ax, double ay, double bx, double by)
    {
        double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
        double t = l2 > 0 ? Math.Clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0, 1) : 0;
        double qx = ax + t * dx - px, qy = ay + t * dy - py;
        return Math.Sqrt(qx * qx + qy * qy);
    }

    // Signed distance between two convex cores: the distance from the origin to the Minkowski difference A - B (negative inside).
    internal static double CoreDistance(List<(double x, double y)> a, List<(double x, double y)> b)
    {
        var d = new List<(double x, double y)>(a.Count * b.Count);
        foreach (var p in a) foreach (var q in b) d.Add((p.x - q.x, p.y - q.y));
        var h = Hull(d);
        if (h.Count == 1) return Math.Sqrt(h[0].x * h[0].x + h[0].y * h[0].y);
        if (h.Count == 2) return SegDist(0, 0, h[0].x, h[0].y, h[1].x, h[1].y);
        double inside = double.NegativeInfinity, outside = double.PositiveInfinity;
        for (int i = 0; i < h.Count; i++)
        {
            var p = h[i]; var q = h[(i + 1) % h.Count];
            double ex = q.x - p.x, ey = q.y - p.y, len = Math.Sqrt(ex * ex + ey * ey);
            if (len == 0) continue;
            double sd = (ey * -p.x - ex * -p.y) / len; // signed distance of the origin from the edge along its outward normal (ey,-ex), CCW hull
            inside = Math.Max(inside, sd);
            outside = Math.Min(outside, SegDist(0, 0, p.x, p.y, q.x, q.y));
        }
        return inside <= 0 ? inside : outside;
    }

    /// <summary>Checks the penetration measure against closed-form answers; returns the number of failures.</summary>
    public static int SelfTest()
    {
        int bad = 0;
        void Check(string what, double got, double want) { if (Math.Abs(got - want) > 1e-9) { Console.Error.WriteLine($"selftest {what}: got {got}, want {want}"); bad++; } }
        List<(double x, double y)> Box(double cx, double cy, double h) => [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)];
        Check("boxes overlapping by 0.1", CoreDistance(Box(0, 0, 0.5), Box(0.9, 0, 0.5)), -0.1);
        Check("boxes 0.25 apart", CoreDistance(Box(0, 0, 0.5), Box(1.25, 0, 0.5)), 0.25);
        Check("corner distance", CoreDistance(Box(0, 0, 0.5), Box(2, 2, 0.5)), Math.Sqrt(2));
        Check("point inside box", CoreDistance([(0.2, 0)], Box(0, 0, 0.5)), -0.3);
        Check("segment crossing box", CoreDistance([(-2, 0), (2, 0)], Box(0, 0, 0.5)), -0.5);
        Check("segment beside box", CoreDistance([(-2, 1), (2, 1)], Box(0, 0, 0.5)), 0.5);
        Check("two points", CoreDistance([(0, 0)], [(3, 4)]), 5);
        return bad;
    }

    sealed class MassProps { public double M, Cx, Cy, I; } // I about the centre of mass

    static MassProps Mass(BodyDef b)
    {
        double m = 0, mx = 0, my = 0, io = 0;
        foreach (var sh in b.Shapes)
        {
            double a, cx, cy, ic; var p = sh.P;
            switch (sh.Kind)
            {
                case "circle": a = Math.PI * sh.Radius * sh.Radius; cx = p[0]; cy = p[1]; ic = 0.5 * a * sh.Radius * sh.Radius; break;
                case "box": a = 4 * p[0] * p[1]; cx = p[2]; cy = p[3]; ic = a * (p[0] * p[0] + p[1] * p[1]) / 3; break;
                case "capsule":
                {
                    double l = Math.Sqrt((p[2] - p[0]) * (p[2] - p[0]) + (p[3] - p[1]) * (p[3] - p[1])), r = sh.Radius;
                    a = 2 * r * l + Math.PI * r * r; cx = 0.5 * (p[0] + p[2]); cy = 0.5 * (p[1] + p[3]);
                    ic = 2 * r * l * (l * l + 4 * r * r) / 12 + Math.PI * r * r * (0.5 * r * r + 0.25 * l * l); break;
                }
                case "polygon":
                {
                    double ar = 0, sx = 0, sy = 0, sI = 0; int n = p.Length / 2;
                    for (int i = 1; i + 1 < n; i++)
                    {
                        double x1 = p[2 * i] - p[0], y1 = p[2 * i + 1] - p[1], x2 = p[2 * i + 2] - p[0], y2 = p[2 * i + 3] - p[1];
                        double cr = x1 * y2 - y1 * x2, ta = 0.5 * cr;
                        ar += ta; sx += ta * (x1 + x2) / 3; sy += ta * (y1 + y2) / 3;
                        sI += cr / 12 * (x1 * x1 + x2 * x2 + x1 * x2 + y1 * y1 + y2 * y2 + y1 * y2);
                    }
                    a = ar; cx = p[0] + sx / ar; cy = p[1] + sy / ar;
                    ic = sI - ar * ((sx / ar) * (sx / ar) + (sy / ar) * (sy / ar)); // sI is about the first vertex
                    break;
                }
                default: continue; // segments and chains have no mass
            }
            double dm = a * sh.Density;
            m += dm; mx += dm * cx; my += dm * cy; io += dm / a * (ic + a * (cx * cx + cy * cy));
        }
        if (m <= 0) return new MassProps();
        double ccx = mx / m, ccy = my / m;
        return new MassProps { M = m, Cx = ccx, Cy = ccy, I = Math.Max(0, io - m * (ccx * ccx + ccy * ccy)) };
    }

    public static SortedDictionary<string, double> Compute(Scene s, string dumpPath)
    {
        if (s.Dim == 3) return Metrics3.Compute(s, dumpPath);
        var (n, frames) = ReadDump(dumpPath);
        if (n != s.Bodies.Count) throw new InvalidDataException($"dump has {n} bodies, scene has {s.Bodies.Count}");
        var r = new SortedDictionary<string, double>(StringComparer.Ordinal);
        var want = new HashSet<string>(s.Metrics);
        var dyn = Enumerable.Range(0, n).Where(i => s.Bodies[i].Type == "dynamic").ToArray();
        var mass = s.Bodies.Select(Mass).ToArray();
        var last = frames[^1];

        double maxSpeed = 0; long nan = 0;
        foreach (var fr in frames)
            for (int i = 0; i < n; i++)
            {
                for (int k = 0; k < 6; k++) if (!double.IsFinite(fr.D[i * 7 + k])) nan++;
                if (s.Bodies[i].Type == "dynamic") { double vx = fr.D[i * 7 + 3], vy = fr.D[i * 7 + 4]; var sp = Math.Sqrt(vx * vx + vy * vy); if (double.IsFinite(sp)) maxSpeed = Math.Max(maxSpeed, sp); }
            }
        if (want.Contains("maxSpeed")) r["maxSpeed"] = maxSpeed;
        if (want.Contains("nan")) r["nan"] = nan;

        if (want.Contains("restKE"))
        {
            double ke = 0, m = 0;
            foreach (var i in dyn)
            {
                var mp = mass[i]; double vx = last.D[i * 7 + 3], vy = last.D[i * 7 + 4], w = last.D[i * 7 + 5];
                ke += 0.5 * mp.M * (vx * vx + vy * vy) + 0.5 * mp.I * w * w; m += mp.M;
            }
            r["restKE"] = m > 0 ? ke / m : 0;
        }
        if (want.Contains("jitter") && frames.Count >= 2)
        {
            var prev = frames[^2]; double dt = (last.Step - prev.Step) * s.Dt, j = 0;
            foreach (var i in dyn)
            {
                double dx = last.D[i * 7] - prev.D[i * 7], dy = last.D[i * 7 + 1] - prev.D[i * 7 + 1];
                j = Math.Max(j, Math.Sqrt(dx * dx + dy * dy) / dt);
            }
            r["jitter"] = j;
        }
        if (want.Contains("charJitter") && frames.Count >= 3)
        {
            // Characters standing still (or riding constant-velocity ground) should not change velocity: the largest change between consecutive dumps over the last four intervals.
            double worst = 0;
            for (int k = frames.Count - 4; k + 1 < frames.Count; k++)
            {
                if (k < 0) continue;
                for (int i = 0; i < n; i++)
                {
                    if (s.Bodies[i].Char == null) continue;
                    double dvx = frames[k + 1].D[i * 7 + 3] - frames[k].D[i * 7 + 3], dvy = frames[k + 1].D[i * 7 + 4] - frames[k].D[i * 7 + 4];
                    worst = Math.Max(worst, Math.Sqrt(dvx * dvx + dvy * dvy));
                }
            }

            r["charJitter"] = worst;
        }
        if (want.Contains("asleepBeforeWake") && s.Events.Count > 0)
        {
            var f = frames.FirstOrDefault(x => x.Step == s.Events[0].Step);
            r["asleepBeforeWake"] = f == null ? -1 : dyn.Count(i => f.D[i * 7 + 6] < 0.5);
        }
        if (want.Contains("awakeFinal")) r["awakeFinal"] = dyn.Count(i => last.D[i * 7 + 6] > 0.5);
        if (want.Contains("escapes") && s.Params.TryGetValue("container", out var c))
        {
            double x0 = double.Parse(c[0], CultureInfo.InvariantCulture) - 0.5, y0 = double.Parse(c[1], CultureInfo.InvariantCulture) - 0.5;
            double x1 = double.Parse(c[2], CultureInfo.InvariantCulture) + 0.5, y1 = double.Parse(c[3], CultureInfo.InvariantCulture) + 0.5;
            int worst = 0;
            foreach (var fr in frames)
            {
                int cnt = 0;
                foreach (var i in dyn) { double x = fr.D[i * 7], y = fr.D[i * 7 + 1]; if (!(x >= x0 && x <= x1 && y >= y0 && y <= y1)) cnt++; }
                worst = Math.Max(worst, cnt);
            }
            r["escapes"] = worst;
        }
        if (want.Contains("tunnelled"))
        {
            int worst = 0;
            foreach (var fr in frames)
            {
                int cnt = 0;
                foreach (var i in dyn)
                {
                    double x = fr.D[i * 7], y = fr.D[i * 7 + 1];
                    if ((s.Params.TryGetValue("wall", out var w) && x > double.Parse(w[0], CultureInfo.InvariantCulture) + 0.1) ||
                        (s.Params.TryGetValue("floor", out var fl) && y < double.Parse(fl[0], CultureInfo.InvariantCulture))) cnt++;
                }
                worst = Math.Max(worst, cnt);
            }
            r["tunnelled"] = worst;
        }
        if (want.Contains("collapse") && s.Params.TryGetValue("top", out var top))
        {
            int t = int.Parse(top[0]);
            r["topDrop"] = s.Bodies[t].Y - last.D[t * 7 + 1];
            r["topLateral"] = Math.Abs(s.Bodies[t].X - last.D[t * 7]);
        }
        if (want.Contains("jointError"))
        {
            double worst = 0;
            foreach (var fr in frames)
                foreach (var j in s.Joints)
                {
                    var (ax, ay) = World(fr, j.A, j.Ax, j.Ay); var (bx, by) = World(fr, j.B, j.Bx, j.By);
                    var e = Math.Sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
                    if (double.IsFinite(e)) worst = Math.Max(worst, e); else worst = double.PositiveInfinity;
                }
            r["jointError"] = worst;
        }
        if (want.Contains("maxPenetration")) r["maxPenetration"] = Penetration(s, frames);
        if (want.Contains("finalPenetration")) r["finalPenetration"] = Penetration(s, [last]);
        return r;
    }

    static (double, double) World(Frame f, int b, double lx, double ly)
    {
        double a = f.D[b * 7 + 2], c = Math.Cos(a), sn = Math.Sin(a);
        return (f.D[b * 7] + c * lx - sn * ly, f.D[b * 7 + 1] + sn * lx + c * ly);
    }

    static double Penetration(Scene s, List<Frame> frames)
    {
        var parts = Parts(s);
        var jointed = new HashSet<long>();
        foreach (var j in s.Joints) jointed.Add(((long)Math.Min(j.A, j.B) << 32) | (uint)Math.Max(j.A, j.B));
        double worst = 0;
        var wx = new double[parts.Count][]; // world core points per part
        var minX = new double[parts.Count]; var maxX = new double[parts.Count]; var minY = new double[parts.Count]; var maxY = new double[parts.Count];
        foreach (var fr in frames)
        {
            for (int k = 0; k < parts.Count; k++)
            {
                var pt = parts[k]; var b = pt.Body;
                double a = fr.D[b * 7 + 2], c = Math.Cos(a), sn = Math.Sin(a), bx = fr.D[b * 7], by = fr.D[b * 7 + 1];
                var w = new double[pt.Local.Length];
                double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue;
                for (int i = 0; i < w.Length; i += 2)
                {
                    w[i] = bx + c * pt.Local[i] - sn * pt.Local[i + 1]; w[i + 1] = by + sn * pt.Local[i] + c * pt.Local[i + 1];
                    x0 = Math.Min(x0, w[i]); x1 = Math.Max(x1, w[i]); y0 = Math.Min(y0, w[i + 1]); y1 = Math.Max(y1, w[i + 1]);
                }
                wx[k] = w; minX[k] = x0 - pt.Radius; maxX[k] = x1 + pt.Radius; minY[k] = y0 - pt.Radius; maxY[k] = y1 + pt.Radius;
                if (!double.IsFinite(x0 + x1 + y0 + y1)) { minX[k] = maxX[k] = minY[k] = maxY[k] = double.NaN; }
            }
            var order = Enumerable.Range(0, parts.Count).Where(k => double.IsFinite(minX[k])).OrderBy(k => minX[k]).ToArray();
            for (int ii = 0; ii < order.Length; ii++)
            {
                int i = order[ii]; var pi = parts[i]; bool di = s.Bodies[pi.Body].Type == "dynamic";
                for (int jj = ii + 1; jj < order.Length; jj++)
                {
                    int j = order[jj];
                    if (minX[j] > maxX[i]) break;
                    if (minY[j] > maxY[i] || maxY[j] < minY[i]) continue;
                    var pj = parts[j];
                    if (pi.Body == pj.Body) continue;
                    bool dj = s.Bodies[pj.Body].Type == "dynamic";
                    if (!di && !dj) continue;
                    if (jointed.Contains(((long)Math.Min(pi.Body, pj.Body) << 32) | (uint)Math.Max(pi.Body, pj.Body))) continue;
                    var a = Pts(wx[i]); var b = Pts(wx[j]);
                    double sd = CoreDistance(a, b) - pi.Radius - pj.Radius;
                    if (-sd > worst) worst = -sd;
                }
            }
        }
        return worst;
    }

    static List<(double x, double y)> Pts(double[] w)
    {
        var l = new List<(double x, double y)>(w.Length / 2);
        for (int i = 0; i < w.Length; i += 2) l.Add((w[i], w[i + 1]));
        return l;
    }
}
