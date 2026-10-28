using System.Globalization;

namespace Bar;

/// <summary>3D stability metrics from pose dumps. Penetration uses a small GJK distance routine with an EPA fallback written for this tool.</summary>
public static class Metrics3
{
    const int St = 14; // x y z qx qy qz qw vx vy vz wx wy wz awake

    readonly record struct V(double X, double Y, double Z)
    {
        public static V operator +(V a, V b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static V operator -(V a, V b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static V operator -(V a) => new(-a.X, -a.Y, -a.Z);
        public static V operator *(double k, V a) => new(k * a.X, k * a.Y, k * a.Z);
        public double Dot(V b) => X * b.X + Y * b.Y + Z * b.Z;
        public V Cross(V b) => new(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);
        public double Len2 => Dot(this);
    }

    static V Rot(double qx, double qy, double qz, double qw, V v)
    {
        // v' = v + 2w(q x v) + 2 q x (q x v)
        var q = new V(qx, qy, qz); var t = 2 * q.Cross(v);
        return v + qw * t + q.Cross(t);
    }

    sealed class Part { public int Body; public V[] Local = []; public double Radius; }

    static List<Part> Parts(Scene s)
    {
        var parts = new List<Part>();
        for (int bi = 0; bi < s.Bodies.Count; bi++)
            foreach (var sh in s.Bodies[bi].Shapes)
            {
                var p = sh.P;
                switch (sh.Kind)
                {
                    case "sphere": parts.Add(new Part { Body = bi, Local = [new V(p[0], p[1], p[2])], Radius = sh.Radius }); break;
                    case "capsule": parts.Add(new Part { Body = bi, Local = [new V(p[0], p[1], p[2]), new V(p[3], p[4], p[5])], Radius = sh.Radius }); break;
                    case "hull":
                    {
                        var pts = new V[p.Length / 3];
                        for (int i = 0; i < pts.Length; i++) pts[i] = new V(p[3 * i], p[3 * i + 1], p[3 * i + 2]);
                        parts.Add(new Part { Body = bi, Local = pts, Radius = sh.Radius }); break;
                    }
                    case "box":
                    {
                        var pts = new V[8]; int k = 0; var c = new V(p[3], p[4], p[5]);
                        foreach (var sx in new[] { -1, 1 }) foreach (var sy in new[] { -1, 1 }) foreach (var sz in new[] { -1, 1 })
                            pts[k++] = c + Rot(p[6], p[7], p[8], p[9], new V(sx * p[0], sy * p[1], sz * p[2]));
                        parts.Add(new Part { Body = bi, Local = pts, Radius = sh.Radius }); break;
                    }
                    case "mesh":
                    {
                        int nv = (int)p[0], ni = (int)p[1];
                        for (int t = 0; t < ni; t += 3)
                        {
                            V Vx(int idx) { int o = 2 + 3 * (int)p[2 + 3 * nv + idx]; return new V(p[o], p[o + 1], p[o + 2]); }
                            parts.Add(new Part { Body = bi, Local = [Vx(t), Vx(t + 1), Vx(t + 2)] });
                        }
                        break;
                    }
                    case "heightfield":
                    {
                        int nx = (int)p[0], nz = (int)p[1]; double cell = p[2]; var o = new V(p[3], p[4], p[5]);
                        V At(int i, int j) => new V(o.X + i * cell, o.Y + p[6 + j * nx + i], o.Z + j * cell);
                        for (int j = 0; j + 1 < nz; j++)
                            for (int i = 0; i + 1 < nx; i++)
                            {
                                parts.Add(new Part { Body = bi, Local = [At(i, j), At(i, j + 1), At(i + 1, j)] });
                                parts.Add(new Part { Body = bi, Local = [At(i + 1, j), At(i, j + 1), At(i + 1, j + 1)] });
                            }
                        break;
                    }
                    default: throw new InvalidDataException("unknown shape " + sh.Kind);
                }
            }
        return parts;
    }

    // ---- GJK distance between two point-cloud cores, EPA when they overlap ----

    static V Support(V[] a, V d) { var best = a[0]; double bd = best.Dot(d); for (int i = 1; i < a.Length; i++) { double v = a[i].Dot(d); if (v > bd) { bd = v; best = a[i]; } } return best; }

    // Closest point to the origin on triangle abc; mask says which of a, b, c carry the answer (bit 0, 1, 2).
    static V ClosestTri(V a, V b, V c, out int mask)
    {
        var ab = b - a; var ac = c - a; var ap = -a;
        double d1 = ab.Dot(ap), d2 = ac.Dot(ap);
        if (d1 <= 0 && d2 <= 0) { mask = 1; return a; }
        var bp = -b; double d3 = ab.Dot(bp), d4 = ac.Dot(bp);
        if (d3 >= 0 && d4 <= d3) { mask = 2; return b; }
        double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) { mask = 3; return a + (d1 / (d1 - d3)) * ab; }
        var cp = -c; double d5 = ab.Dot(cp), d6 = ac.Dot(cp);
        if (d6 >= 0 && d5 <= d6) { mask = 4; return c; }
        double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) { mask = 5; return a + (d2 / (d2 - d6)) * ac; }
        double va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) { mask = 6; return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b); }
        double denom = 1 / (va + vb + vc); mask = 7;
        return a + (vb * denom) * ab + (vc * denom) * ac;
    }

    // Signed core distance: positive separation, or minus the penetration depth; NaN if EPA could not decide (flat overlap).
    static double CoreDistance(V[] A, V[] B)
    {
        V Sup(V d) => Support(A, d) - Support(B, -d);
        var simplex = new List<V> { Sup(new V(1, 0, 0)) };
        var v = simplex[0];
        for (int it = 0; it < 100; it++)
        {
            double v2 = v.Len2;
            if (v2 < 1e-24) return Epa(A, B, simplex);
            var w = Sup(-v);
            if (v2 - v.Dot(w) <= 1e-12 * v2) return Math.Sqrt(v2);
            if (simplex.Contains(w)) return Math.Sqrt(v2);
            simplex.Add(w);
            switch (simplex.Count)
            {
                case 2:
                {
                    var a = simplex[0]; var b = simplex[1]; var ab = b - a; double t = -a.Dot(ab) / ab.Len2;
                    if (t <= 0) { simplex = [a]; v = a; } else if (t >= 1) { simplex = [b]; v = b; } else v = a + t * ab;
                    break;
                }
                case 3:
                {
                    v = ClosestTri(simplex[0], simplex[1], simplex[2], out int m);
                    simplex = Pick(simplex, m); break;
                }
                default: // tetrahedron: the origin is inside unless it lies outside a face
                {
                    var p = simplex; bool inside = true; double best = double.MaxValue; V bv = default; List<V>? keep = null;
                    int[][] faces = [[0, 1, 2, 3], [0, 3, 1, 2], [0, 2, 3, 1], [1, 3, 2, 0]];
                    foreach (var f in faces)
                    {
                        var a = p[f[0]]; var b = p[f[1]]; var c = p[f[2]]; var opp = p[f[3]];
                        var n = (b - a).Cross(c - a);
                        double so = n.Dot(opp - a), sp = n.Dot(-a);
                        if (so * sp < 0 || (so == 0 && sp != 0)) // origin on the far side of this face from the fourth vertex
                        {
                            inside = false;
                            var cpnt = ClosestTri(a, b, c, out int m);
                            if (cpnt.Len2 < best) { best = cpnt.Len2; bv = cpnt; keep = Pick([a, b, c], m); }
                        }
                    }
                    if (inside) return Epa(A, B, simplex);
                    v = bv; simplex = keep!; break;
                }
            }
        }
        return Math.Sqrt(v.Len2);
    }

    static List<V> Pick(List<V> s, int mask)
    {
        var r = new List<V>();
        if ((mask & 1) != 0) r.Add(s[0]);
        if ((mask & 2) != 0) r.Add(s[1]);
        if ((mask & 4) != 0) r.Add(s[2]);
        return r;
    }

    sealed class Face { public int A, B, C; public V N; public double D; public bool Dead; }

    static double Epa(V[] A, V[] B, List<V> start)
    {
        V Sup(V d) => Support(A, d) - Support(B, -d);
        var pts = new List<V>(start);
        // Grow the starting simplex to a tetrahedron around the origin.
        V[] axes = [new V(1, 0, 0), new V(-1, 0, 0), new V(0, 1, 0), new V(0, -1, 0), new V(0, 0, 1), new V(0, 0, -1)];
        foreach (var ax in axes)
        {
            if (pts.Count >= 4) break;
            var w = Sup(ax);
            if (pts.Contains(w)) continue;
            if (pts.Count == 3 && Math.Abs((pts[1] - pts[0]).Cross(pts[2] - pts[0]).Dot(w - pts[0])) < 1e-12) continue;
            if (pts.Count == 2 && (pts[1] - pts[0]).Cross(w - pts[0]).Len2 < 1e-18) continue;
            pts.Add(w);
        }
        if (pts.Count < 4) return double.NaN;
        var cen = 0.25 * (pts[0] + pts[1] + pts[2] + pts[3]);
        {
            // The origin must lie inside the starting tetrahedron. When the cores only touch, the origin is on the boundary of the
            // Minkowski difference and the tetrahedron can miss it; the depth is then zero.
            int[][] tf = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];
            foreach (var t in tf)
            {
                var n = (pts[t[1]] - pts[t[0]]).Cross(pts[t[2]] - pts[t[0]]); double l = Math.Sqrt(n.Len2); if (l < 1e-18) continue;
                n = (1 / l) * n; if (n.Dot(cen - pts[t[0]]) > 0) n = -n;
                if (n.Dot(-pts[t[0]]) > 1e-9) return 0;
            }
        }
        var faces = new List<Face>();
        void AddFace(int a, int b, int c)
        {
            var n = (pts[b] - pts[a]).Cross(pts[c] - pts[a]); double l = Math.Sqrt(n.Len2);
            if (l < 1e-18) return;
            n = (1 / l) * n;
            // Outward is away from a point inside the polytope; orienting by the sign of the distance would flip faces through the origin when the cores touch.
            if (n.Dot(pts[a] - cen) < 0) { n = -n; (b, c) = (c, b); }
            double d = n.Dot(pts[a]);
            faces.Add(new Face { A = a, B = b, C = c, N = n, D = d });
        }
        AddFace(0, 1, 2); AddFace(0, 1, 3); AddFace(0, 2, 3); AddFace(1, 2, 3);
        for (int it = 0; it < 128; it++)
        {
            Face? f = null;
            foreach (var x in faces) if (!x.Dead && (f == null || x.D < f.D)) f = x;
            if (f == null) return double.NaN;
            var w = Sup(f.N);
            if (w.Dot(f.N) - f.D < 1e-9) return -f.D;
            var edges = new List<(int, int)>();
            foreach (var x in faces)
            {
                if (x.Dead || x.N.Dot(w - pts[x.A]) <= 1e-12) continue;
                x.Dead = true;
                foreach (var e in new[] { (x.A, x.B), (x.B, x.C), (x.C, x.A) })
                {
                    int k = edges.IndexOf((e.Item2, e.Item1));
                    if (k >= 0) edges.RemoveAt(k); else edges.Add(e);
                }
            }
            int wi = pts.Count; pts.Add(w);
            foreach (var (a, b) in edges) AddFace(a, b, wi);
            faces.RemoveAll(x => x.Dead);
        }
        double best = double.MaxValue; foreach (var x in faces) best = Math.Min(best, x.D);
        return -best;
    }

    public static int SelfTest()
    {
        int bad = 0;
        void Check(string what, double got, double want) { if (!(Math.Abs(got - want) < 1e-6)) { Console.Error.WriteLine($"selftest3 {what}: got {got}, want {want}"); bad++; } }
        V[] Box(double cx, double cy, double cz, double h) { var l = new List<V>(); foreach (var a in new[] { -1, 1 }) foreach (var b in new[] { -1, 1 }) foreach (var c in new[] { -1, 1 }) l.Add(new V(cx + a * h, cy + b * h, cz + c * h)); return l.ToArray(); }
        Check("boxes apart", CoreDistance(Box(0, 0, 0, 0.5), Box(1.25, 0, 0, 0.5)), 0.25);
        Check("boxes overlap", CoreDistance(Box(0, 0, 0, 0.5), Box(0.9, 0.1, 0, 0.5)), -0.1);
        Check("corner apart", CoreDistance(Box(0, 0, 0, 0.5), Box(2, 2, 2, 0.5)), Math.Sqrt(3));
        Check("point in box", CoreDistance([new V(0.2, 0, 0)], Box(0, 0, 0, 0.5)), -0.3);
        Check("segment beside box", CoreDistance([new V(-2, 1, 0), new V(2, 1, 0)], Box(0, 0, 0, 0.5)), 0.5);
        Check("boxes touching", CoreDistance(Box(0, 0, 0, 0.5), Box(1, 0, 0, 0.5)), 0);
        Check("box resting on a large box", CoreDistance(Box(0, 0.5, 0, 0.5), [new V(-20, -2, -20), new V(20, -2, -20), new V(-20, 0, -20), new V(20, 0, -20), new V(-20, -2, 20), new V(20, -2, 20), new V(-20, 0, 20), new V(20, 0, 20)]), 0);
        Check("points", CoreDistance([new V(0, 0, 0)], [new V(2, 3, 6)]), 7);
        Check("triangle vs box", CoreDistance([new V(-2, 0.3, -2), new V(2, 0.3, -2), new V(0, 0.3, 3)], Box(0, 0, 0, 0.5)), -0.2);
        return bad;
    }

    // Terrain grids of the vehicles scene: (origin x, origin z, cell, columns, rows, heights) of the height field and the mesh, read back from the shapes the generator wrote.
    static List<(double Ox, double Oz, double Cell, int Nx, int Nz, double[] H)> VehicleGrids(Scene s)
    {
        var grids = new List<(double, double, double, int, int, double[])>();
        foreach (var b in s.Bodies.Where(b => b.Type == "static"))
            foreach (var sh in b.Shapes)
            {
                var p = sh.P;
                if (sh.Kind == "heightfield") { int nx = (int)p[0], nz = (int)p[1]; grids.Add((p[3], p[5], p[2], nx, nz, p.Skip(6).Take(nx * nz).ToArray())); }
                else if (sh.Kind == "mesh")
                {
                    int nv = (int)p[0]; var h = new double[nv]; for (int i = 0; i < nv; i++) h[i] = p[3 + 3 * i];
                    grids.Add((p[2], p[4], 1.0, 61, 241, h));
                }
            }
        return grids;
    }

    static double GridHeight(List<(double Ox, double Oz, double Cell, int Nx, int Nz, double[] H)> grids, double x, double z)
    {
        foreach (var g in grids)
        {
            double u = (x - g.Ox) / g.Cell, v = (z - g.Oz) / g.Cell;
            if (u < 0 || v < 0 || u > g.Nx - 1 || v > g.Nz - 1) continue;
            int i = Math.Min((int)u, g.Nx - 2), j = Math.Min((int)v, g.Nz - 2); double fu = u - i, fv = v - j;
            return (g.H[j * g.Nx + i] * (1 - fu) + g.H[j * g.Nx + i + 1] * fu) * (1 - fv) + (g.H[(j + 1) * g.Nx + i] * (1 - fu) + g.H[(j + 1) * g.Nx + i + 1] * fu) * fv;
        }
        return double.NaN;
    }

    // Vehicle metrics from the wheel centres the drivers dump (three doubles per wheel, four wheels per car, cars in body order):
    //  wheelThroughGround: wheel samples whose centre is lower than half a wheel radius above the terrain (bilinear on the generator's grid) under it, summed over all dumps;
    //  rollovers: cars whose body up axis points below the horizon (y component under 0.2) in any dump;
    //  suspensionJitter: for cars at rest (speed under 0.2 m/s) in the last dump, the largest change of a wheel centre's height above the body centre between the last dumps (m).
    static void VehicleMetrics(Scene s, List<(int step, double[] d)> frames, List<double[]> wheelFrames, HashSet<string> want, SortedDictionary<string, double> r)
    {
        int first = int.Parse(s.Params["cars"][0], CultureInfo.InvariantCulture), count = int.Parse(s.Params["cars"][1], CultureInfo.InvariantCulture);
        double radius = double.Parse(s.Params["wheelRadius"][0], CultureInfo.InvariantCulture);
        if (wheelFrames.Count == 0 || wheelFrames[^1].Length != 12 * count) { return; } // engines without wheel data report nothing for these
        var grids = VehicleGrids(s);
        long through = 0; var rolled = new HashSet<int>();
        for (int f = 0; f < frames.Count; f++)
        {
            var d = frames[f].d; var w = wheelFrames[f];
            for (int c = 0; c < count; c++)
            {
                int b = first + c;
                double qx = d[b * St + 3], qy = d[b * St + 4], qz = d[b * St + 5], qw = d[b * St + 6];
                double upY = 1 - 2 * (qx * qx + qz * qz);
                if (upY < 0.2) rolled.Add(c);
                for (int k = 0; k < 4; k++)
                {
                    int o = 12 * c + 3 * k;
                    double ground = GridHeight(grids, w[o], w[o + 2]);
                    if (!double.IsNaN(ground) && w[o + 1] < ground + 0.5 * radius) through++;
                }
            }
        }
        if (want.Contains("wheelThroughGround")) r["wheelThroughGround"] = through;
        if (want.Contains("rollovers")) r["rollovers"] = rolled.Count;
        if (want.Contains("suspensionJitter") && frames.Count >= 3)
        {
            double worst = 0; var a = frames[^2].d; var z = frames[^1].d; var wa = wheelFrames[^2]; var wz = wheelFrames[^1];
            for (int c = 0; c < count; c++)
            {
                int b = first + c;
                if (Math.Sqrt(new V(z[b * St + 7], z[b * St + 8], z[b * St + 9]).Len2) > 0.2) continue;
                for (int k = 0; k < 4; k++)
                {
                    int o = 12 * c + 3 * k;
                    worst = Math.Max(worst, Math.Abs((wz[o + 1] - z[b * St + 1]) - (wa[o + 1] - a[b * St + 1])));
                }
            }
            r["suspensionJitter"] = worst;
        }
    }

    // ---- metrics ----

    public static SortedDictionary<string, double> Compute(Scene s, string dumpPath)
    {
        int n = s.Bodies.Count;
        var frames = new List<(int step, double[] d)>();
        var wheelFrames = new List<double[]>();
        using (var f = File.OpenRead(dumpPath))
        using (var rd = new BinaryReader(f))
        {
            if (System.Text.Encoding.ASCII.GetString(rd.ReadBytes(8)) != "OPDUMP01") throw new InvalidDataException("bad dump magic");
            if ((int)rd.ReadUInt32() != n) throw new InvalidDataException("dump body count differs from the scene");
            if (rd.ReadUInt32() != 3) throw new InvalidDataException("not a 3D dump");
            while (f.Position < f.Length)
            {
                int step = (int)rd.ReadUInt32(); int extra = (int)rd.ReadUInt32();
                var d = new double[n * St]; for (int i = 0; i < d.Length; i++) d[i] = rd.ReadDouble();
                var wh = new double[extra]; for (int i = 0; i < wh.Length; i++) wh[i] = rd.ReadDouble();
                frames.Add((step, d)); wheelFrames.Add(wh);
            }
        }
        var r = new SortedDictionary<string, double>(StringComparer.Ordinal);
        var want = new HashSet<string>(s.Metrics);
        var dyn = Enumerable.Range(0, n).Where(i => s.Bodies[i].Type == "dynamic").ToArray();
        var last = frames[^1];
        double P(string k, int i = 0) => double.Parse(s.Params[k][i], CultureInfo.InvariantCulture);

        double maxSpeed = 0; long nan = 0;
        foreach (var (_, d) in frames)
            for (int i = 0; i < n; i++)
            {
                for (int k = 0; k < 13; k++) if (!double.IsFinite(d[i * St + k])) nan++;
                if (s.Bodies[i].Type == "dynamic") { var sp = Math.Sqrt(new V(d[i * St + 7], d[i * St + 8], d[i * St + 9]).Len2); if (double.IsFinite(sp)) maxSpeed = Math.Max(maxSpeed, sp); }
            }
        if (want.Contains("maxSpeed")) r["maxSpeed"] = maxSpeed;
        if (want.Contains("nan")) r["nan"] = nan;

        // Rest energy per unit mass, approximated per body as 1/2 (v^2 + w^2 R^2) with R the body's bounding radius.
        double Radius(BodyDef b) { double m = 0; foreach (var sh in b.Shapes) { foreach (var v in Parts(new Scene { Bodies = [new BodyDef { Shapes = [sh] }] }).SelectMany(p => p.Local)) m = Math.Max(m, Math.Sqrt(v.Len2) + sh.Radius); if (sh.Kind == "sphere") m = Math.Max(m, sh.Radius); } return m; }
        if (want.Contains("restKE"))
        {
            double sum = 0;
            foreach (var i in dyn)
            {
                var d = last.d; double R = Radius(s.Bodies[i]);
                sum += 0.5 * (new V(d[i * St + 7], d[i * St + 8], d[i * St + 9]).Len2 + new V(d[i * St + 10], d[i * St + 11], d[i * St + 12]).Len2 * R * R);
            }
            r["restKERaw"] = dyn.Length > 0 ? sum / dyn.Length : 0;
            var radii = new Dictionary<int, double>();
            foreach (var i in dyn) radii[i] = Radius(s.Bodies[i]);
            r["restKE"] = Robust.RestKe(frames.Count, dyn, (f, i) =>
            {
                var d = frames[f].d; double R = radii[i];
                return 0.5 * (new V(d[i * St + 7], d[i * St + 8], d[i * St + 9]).Len2 + new V(d[i * St + 10], d[i * St + 11], d[i * St + 12]).Len2 * R * R);
            });
        }
        if (want.Contains("jitter") && frames.Count >= 2)
        {
            var prev = frames[^2]; double dt = (last.step - prev.step) * s.Dt, j = 0;
            foreach (var i in dyn) j = Math.Max(j, Math.Sqrt(new V(last.d[i * St] - prev.d[i * St], last.d[i * St + 1] - prev.d[i * St + 1], last.d[i * St + 2] - prev.d[i * St + 2]).Len2) / dt);
            r["jitterRaw"] = j;
            r["jitter"] = Robust.Jitter(frames.Count, k => frames[k].step, s.Dt, dyn, (a, b, i) =>
                Math.Sqrt(new V(frames[b].d[i * St] - frames[a].d[i * St], frames[b].d[i * St + 1] - frames[a].d[i * St + 1], frames[b].d[i * St + 2] - frames[a].d[i * St + 2]).Len2));
        }
        if (want.Contains("charJitter") && frames.Count >= 3)
        {
            // Characters standing still (or riding constant-velocity ground) should not change velocity: the largest change between consecutive dumps over the last four intervals.
            // Riders of rotating platforms (params rotatingRiders: first count pairs) change velocity by design and are skipped.
            var skip = new HashSet<int>();
            if (s.Params.TryGetValue("rotatingRiders", out var rr))
                for (int q = 0; q + 1 < rr.Length; q += 2) for (int i = int.Parse(rr[q], CultureInfo.InvariantCulture); i < int.Parse(rr[q], CultureInfo.InvariantCulture) + int.Parse(rr[q + 1], CultureInfo.InvariantCulture); i++) skip.Add(i);
            double worst = 0;
            for (int k = frames.Count - 4; k + 1 < frames.Count; k++)
            {
                if (k < 0) continue;
                for (int i = 0; i < n; i++)
                {
                    if (s.Bodies[i].Script == null || skip.Contains(i)) continue;
                    var a = frames[k].d; var b = frames[k + 1].d;
                    worst = Math.Max(worst, Math.Sqrt(new V(b[i * St + 7] - a[i * St + 7], b[i * St + 8] - a[i * St + 8], b[i * St + 9] - a[i * St + 9]).Len2));
                }
            }
            r["charJitter"] = worst;
        }
        if (want.Contains("stairSuccess"))
        {
            // Share of the scripted climbers (params stairsN: first count minY) that reach minY in any dumped frame.
            int total = 0, ok = 0;
            for (int g = 0; s.Params.ContainsKey("stairs" + g); g++)
            {
                int first = (int)P("stairs" + g, 0), count = (int)P("stairs" + g, 1); double minY = P("stairs" + g, 2);
                for (int i = first; i < first + count; i++) { total++; if (frames.Any(f => f.d[i * St + 1] >= minY)) ok++; }
            }
            r["stairSuccess"] = total == 0 ? 1 : (double)ok / total;
        }
        if (want.Contains("slopeViolations"))
        {
            // Walkers on the ramps steeper than the limit (params steepN: first count maxY) that get above maxY in any dumped frame.
            int bad = 0;
            for (int g = 0; s.Params.ContainsKey("steep" + g); g++)
            {
                int first = (int)P("steep" + g, 0), count = (int)P("steep" + g, 1); double maxY = P("steep" + g, 2);
                for (int i = first; i < first + count; i++) if (frames.Any(f => f.d[i * St + 1] > maxY)) bad++;
            }
            r["slopeViolations"] = bad;
        }
        if (want.Contains("asleepBeforeWake") && s.Events.Count > 0)
        {
            var f = frames.FirstOrDefault(x => x.step == s.Events[0].Step);
            r["asleepBeforeWake"] = f.d == null ? -1 : dyn.Count(i => f.d[i * St + 13] < 0.5);
        }
        if (want.Contains("awakeFinal")) r["awakeFinal"] = dyn.Count(i => last.d[i * St + 13] > 0.5);
        if (want.Contains("escapes") && s.Params.ContainsKey("container"))
        {
            // container x0 y0 x1 y1 bounds x and y; z uses the same horizontal extent
            int Escapes(double margin)
            {
                double x0 = P("container", 0) - margin, y0 = P("container", 1) - margin, x1 = P("container", 2) + margin, y1 = P("container", 3) + margin;
                int worst = 0;
                foreach (var (_, d) in frames)
                {
                    int cnt = 0;
                    foreach (var i in dyn) { double x = d[i * St], y = d[i * St + 1], z = d[i * St + 2]; if (!(x >= x0 && x <= x1 && y >= y0 && y <= y1 && z >= x0 && z <= x1)) cnt++; }
                    worst = Math.Max(worst, cnt);
                }
                return worst;
            }
            r["escapesRaw"] = Escapes(Robust.DefaultEscapeMargin);
            r["escapes"] = Escapes(Robust.EscapeMargin(s));
        }
        if (want.Contains("tunnelled"))
        {
            int worst = 0;
            foreach (var (_, d) in frames)
            {
                int cnt = 0;
                foreach (var i in dyn) if (s.Params.ContainsKey("floor") && d[i * St + 1] < P("floor")) cnt++;
                worst = Math.Max(worst, cnt);
            }
            r["tunnelled"] = worst;
        }
        if (s.Params.ContainsKey("cars")) VehicleMetrics(s, frames, wheelFrames, want, r);
        if (want.Contains("collapse") && s.Params.ContainsKey("top"))
        {
            int t = (int)P("top");
            r["topDrop"] = s.Bodies[t].Y - last.d[t * St + 1];
            r["topLateral"] = Math.Sqrt(Math.Pow(s.Bodies[t].X - last.d[t * St], 2) + Math.Pow(s.Bodies[t].Z - last.d[t * St + 2], 2));
        }
        if (want.Contains("jointError"))
        {
            double worst = 0;
            foreach (var (_, d) in frames)
                foreach (var j in s.Joints)
                {
                    var a = World(d, j.A, new V(j.Ax, j.Ay, j.Az)); var b = World(d, j.B, new V(j.Bx, j.By, j.Bz));
                    var e = Math.Sqrt((a - b).Len2);
                    worst = double.IsFinite(e) ? Math.Max(worst, e) : double.PositiveInfinity;
                }
            r["jointError"] = worst;
        }
        if (want.Contains("fallen") && s.Params.TryGetValue("heads", out var heads))
        {
            // Heads (params heads: body indices) that end below half of their starting height.
            r["fallen"] = heads.Select(h => int.Parse(h, CultureInfo.InvariantCulture)).Count(h => last.d[h * St + 1] < 0.5 * s.Bodies[h].Y);
        }
        if (want.Contains("poseErrorP95")) r["poseErrorP95"] = PoseErrorP95(s, frames);
        if (want.Contains("maxPenetration") || want.Contains("finalPenetration"))
        {
            var pen = Penetration(s, (want.Contains("maxPenetration") ? frames : frames.TakeLast(Robust.Window).ToList()).Select(x => x.d).ToList(), out _);
            if (want.Contains("maxPenetration")) r["maxPenetration"] = pen.Max(Robust.Deepest);
            if (want.Contains("finalPenetration"))
            {
                r["finalPenetrationRaw"] = Robust.Deepest(pen[^1]);
                r["finalPenetration"] = Robust.FinalPenetration(pen);
            }
        }
        return r;
    }

    /// <summary>Triangle wave from 0 up to 1 and back to 0 over <paramref name="period"/> steps (exact arithmetic).</summary>
    public static double Tri01(int step, int period)
    {
        int phase = step % period; double half = period / 2.0;
        return phase < half ? phase / half : (period - phase) / half;
    }

    static (double X, double Y, double Z, double W) QMul((double X, double Y, double Z, double W) a, (double X, double Y, double Z, double W) b) =>
        (a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y, a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X, a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W, a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

    static (double X, double Y, double Z, double W) QConj((double X, double Y, double Z, double W) a) => (-a.X, -a.Y, -a.Z, a.W);

    /// <summary>The pose error of the driven swing-twist joints: for every dumped frame from step 60 on, the angle between each joint's relative rotation (frame B in frame A, frames being the bodies' orientations times the joint frame) and its drive target, collected over all joints and frames; returns the 95th percentile (docs/scenes.md).</summary>
    static double PoseErrorP95(Scene s, List<(int step, double[] d)> frames)
    {
        var errs = new List<double>();
        var joints = s.Joints.Where(j => j.Kind == "swingtwist" && j.HasDrive).ToList();
        var fr = new List<(double X, double Y, double Z, double W)>();
        foreach (var j in joints)
        {
            double w = 1 + j.Ux;
            if (w < 1e-9) { fr.Add((0.0, 1.0, 0.0, 0.0)); continue; }
            double n = Math.Sqrt(j.Uz * j.Uz + j.Uy * j.Uy + w * w);
            fr.Add((0.0, -j.Uz / n, j.Uy / n, w / n));
        }

        foreach (var (step, d) in frames)
        {
            if (step < 60) continue;
            for (int k = 0; k < joints.Count; k++)
            {
                var j = joints[k]; var f = fr[k];
                var qa = (X: d[j.A * St + 3], Y: d[j.A * St + 4], Z: d[j.A * St + 5], W: d[j.A * St + 6]);
                var qb = (X: d[j.B * St + 3], Y: d[j.B * St + 4], Z: d[j.B * St + 5], W: d[j.B * St + 6]);
                var rel = QMul(QConj(QMul(qa, f)), QMul(qb, f));
                double h = 0.5 * j.DriveAmp * Tri01(step - 1, j.DrivePeriod), sn = Math.Sin(h);
                var t = (X: j.DriveAx * sn, Y: j.DriveAy * sn, Z: j.DriveAz * sn, W: Math.Cos(h));
                var e = QMul(rel, QConj(t));
                double ang = 2 * Math.Atan2(Math.Sqrt(e.X * e.X + e.Y * e.Y + e.Z * e.Z), Math.Abs(e.W));
                errs.Add(double.IsFinite(ang) ? ang : double.PositiveInfinity);
            }
        }

        if (errs.Count == 0) return 0;
        errs.Sort();
        return errs[(int)Math.Ceiling(0.95 * errs.Count) - 1];
    }

    static V World(double[] d, int b, V local) => new V(d[b * St], d[b * St + 1], d[b * St + 2]) + Rot(d[b * St + 3], d[b * St + 4], d[b * St + 5], d[b * St + 6], local);

    /// <summary>Overlap depth of every overlapping shape pair, per frame, deepest first.</summary>
    static List<double[]> Penetration(Scene s, List<double[]> frames, out int undecided)
    {
        var parts = Parts(s);
        undecided = 0;
        var jointed = new HashSet<long>();
        foreach (var j in s.Joints) jointed.Add(((long)Math.Min(j.A, j.B) << 32) | (uint)Math.Max(j.A, j.B));
        var result = new List<double[]>();
        var wp = new V[parts.Count][]; var lo = new V[parts.Count]; var hi = new V[parts.Count];
        foreach (var d in frames)
        {
            var depths = new List<double>();
            for (int k = 0; k < parts.Count; k++)
            {
                var pt = parts[k]; int b = pt.Body; var pos = new V(d[b * St], d[b * St + 1], d[b * St + 2]);
                double qx = d[b * St + 3], qy = d[b * St + 4], qz = d[b * St + 5], qw = d[b * St + 6];
                var w = new V[pt.Local.Length];
                var mn = new V(double.MaxValue, double.MaxValue, double.MaxValue); var mx = new V(double.MinValue, double.MinValue, double.MinValue);
                for (int i = 0; i < w.Length; i++)
                {
                    w[i] = pos + Rot(qx, qy, qz, qw, pt.Local[i]);
                    mn = new V(Math.Min(mn.X, w[i].X), Math.Min(mn.Y, w[i].Y), Math.Min(mn.Z, w[i].Z));
                    mx = new V(Math.Max(mx.X, w[i].X), Math.Max(mx.Y, w[i].Y), Math.Max(mx.Z, w[i].Z));
                }
                wp[k] = w; lo[k] = mn - new V(pt.Radius, pt.Radius, pt.Radius); hi[k] = mx + new V(pt.Radius, pt.Radius, pt.Radius);
            }
            var order = Enumerable.Range(0, parts.Count).Where(k => double.IsFinite(lo[k].X + hi[k].X + lo[k].Y + hi[k].Y + lo[k].Z + hi[k].Z)).OrderBy(k => lo[k].X).ToArray();
            for (int ii = 0; ii < order.Length; ii++)
            {
                int i = order[ii]; var pi = parts[i]; bool di = s.Bodies[pi.Body].Type == "dynamic";
                for (int jj = ii + 1; jj < order.Length; jj++)
                {
                    int j = order[jj];
                    if (lo[j].X > hi[i].X) break;
                    if (lo[j].Y > hi[i].Y || hi[j].Y < lo[i].Y || lo[j].Z > hi[i].Z || hi[j].Z < lo[i].Z) continue;
                    var pj = parts[j];
                    if (pi.Body == pj.Body) continue;
                    if (!di && s.Bodies[pj.Body].Type != "dynamic") continue;
                    if (s.Bodies[pi.Body].Script != null && s.Bodies[pj.Body].Script != null) continue; // scripted characters are on a layer that ignores itself
                    if (jointed.Contains(((long)Math.Min(pi.Body, pj.Body) << 32) | (uint)Math.Max(pi.Body, pj.Body))) continue;
                    double sd = CoreDistance(wp[i], wp[j]);
                    if (double.IsNaN(sd)) { undecided++; continue; }
                    sd -= pi.Radius + pj.Radius;
                    if (sd < 0) depths.Add(-sd);
                }
            }
            depths.Sort((a, b) => b.CompareTo(a));
            result.Add(depths.ToArray());
        }
        return result;
    }
}
