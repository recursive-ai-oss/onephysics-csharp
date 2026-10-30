using System.Globalization;

namespace Bar;

public static partial class Metrics3
{
    // Soft-body metrics from the particle data the drivers dump (six doubles per particle: x y z vx vy vz; the bodies of the scene in record order).
    //  softMaxSpeed         largest particle speed over all dumps (m/s);
    //  softPenetration      deepest particle sphere into any rigid shape over all dumps (m); the sphere's radius is a quarter of the body's mean rest edge length, for every engine;
    //  softFinalPenetration 95th percentile of the particle depths in the last dump, the median of the last three dumps (m);
    //  edgeStrain           95th percentile of |length / rest length - 1| over the edges of every soft body in the last dump;
    //  volumeError          largest |V / V0 - 1| over the soft volumes (tetrahedra, signed volumes) in the last dump;
    //  softRestKE           the median over the last three dumps of the 90th percentile of the per-particle kinetic energy per unit mass, 1/2 v^2;
    //  softEscapes          particles outside the scene's container grown by the escape margin, worst dump;
    //  softTunnelled        particles whose centre is inside a solid rigid shape in the last dump (triangle soups are not solid);
    //  softNan              non-finite particle values over all dumps.
    static void SoftMetrics(Scene s, List<(int step, double[] d)> frames, List<double[]> extra, HashSet<string> want, SortedDictionary<string, double> r)
    {
        var meshes = s.Softs.Select(SoftGen.Build).ToArray();
        int total = meshes.Sum(m => m.Count);
        if (extra.Count == 0 || extra[^1].Length != 6 * total) return; // engines without soft bodies report nothing for these
        var starts = new int[meshes.Length]; for (int i = 1; i < meshes.Length; i++) starts[i] = starts[i - 1] + meshes[i - 1].Count;
        var radius = new double[total]; // per particle
        var edges = meshes.Select(SoftGen.Edges).ToArray();
        for (int b = 0; b < meshes.Length; b++)
        {
            double sum = 0; var m = meshes[b];
            foreach (var (a, c) in edges[b]) sum += Math.Sqrt(Dist2(m.V, a, c));
            double rad = 0.25 * sum / edges[b].Count;
            for (int i = 0; i < m.Count; i++) radius[starts[b] + i] = rad;
        }

        double Pd(int f, int i, int k) => extra[f][6 * i + k];
        long nan = 0; double maxSpeed = 0;
        for (int f = 0; f < extra.Count; f++)
            for (int i = 0; i < total; i++)
            {
                for (int k = 0; k < 6; k++) if (!double.IsFinite(Pd(f, i, k))) nan++;
                double sp = Math.Sqrt(Pd(f, i, 3) * Pd(f, i, 3) + Pd(f, i, 4) * Pd(f, i, 4) + Pd(f, i, 5) * Pd(f, i, 5));
                if (double.IsFinite(sp)) maxSpeed = Math.Max(maxSpeed, sp);
            }
        if (want.Contains("softNan")) r["softNan"] = nan;
        if (want.Contains("softMaxSpeed")) r["softMaxSpeed"] = maxSpeed;

        double Pct(double[] a, double q) { var t = a.OrderBy(x => x).ToArray(); return t.Length == 0 ? 0 : t[Math.Min(t.Length - 1, (int)Math.Ceiling(q * t.Length) - 1)]; }

        if (want.Contains("softRestKE"))
        {
            var vals = new List<double>();
            for (int f = Math.Max(0, extra.Count - Robust.Window); f < extra.Count; f++)
                vals.Add(Pct(Enumerable.Range(0, total).Select(i => 0.5 * (Pd(f, i, 3) * Pd(f, i, 3) + Pd(f, i, 4) * Pd(f, i, 4) + Pd(f, i, 5) * Pd(f, i, 5))).ToArray(), 0.9));
            vals.Sort(); r["softRestKE"] = vals[vals.Count / 2];
        }

        if (want.Contains("edgeStrain") || want.Contains("volumeError"))
        {
            int f = extra.Count - 1;
            var strain = new List<double>(); double volErr = 0;
            for (int b = 0; b < meshes.Length; b++)
            {
                var m = meshes[b];
                foreach (var (a, c) in edges[b])
                {
                    double rest = Math.Sqrt(Dist2(m.V, a, c));
                    double dx = Pd(f, starts[b] + a, 0) - Pd(f, starts[b] + c, 0), dy = Pd(f, starts[b] + a, 1) - Pd(f, starts[b] + c, 1), dz = Pd(f, starts[b] + a, 2) - Pd(f, starts[b] + c, 2);
                    strain.Add(Math.Abs(Math.Sqrt(dx * dx + dy * dy + dz * dz) / rest - 1));
                }
                if (m.Tets.Length > 0)
                {
                    double v0 = 0, v1 = 0;
                    for (int t = 0; t < m.Tets.Length; t += 4)
                    {
                        v0 += TetVolume(i => (m.V[3 * m.Tets[t + i]], m.V[3 * m.Tets[t + i] + 1], m.V[3 * m.Tets[t + i] + 2]));
                        v1 += TetVolume(i => (Pd(f, starts[b] + m.Tets[t + i], 0), Pd(f, starts[b] + m.Tets[t + i], 1), Pd(f, starts[b] + m.Tets[t + i], 2)));
                    }
                    volErr = Math.Max(volErr, Math.Abs(v1 / v0 - 1));
                }
            }
            if (want.Contains("edgeStrain")) r["edgeStrain"] = Pct(strain.ToArray(), 0.95);
            if (want.Contains("volumeError")) r["volumeError"] = volErr;
        }

        if (want.Contains("skinMaxDistance") || want.Contains("skinBackstop") || want.Contains("skinCuffError"))
        {
            // Each skinned sleeve against an independent double-precision skinning of the pose that was set before the last step of the frame.
            double excess = 0, deficit = 0, cuff = 0;
            for (int b = 0; b < s.Softs.Count; b++)
            {
                var d = s.Softs[b];
                if (d.Kind != "sleeve") continue;
                int around = (int)d.P[2];
                for (int f = 1; f < extra.Count; f++)
                {
                    var pose = SleeveRig.Pose(frames[f].step - 1, new RecursiveAI.OneMath.Vector3D(d.X, d.Y, d.Z), d.P[1]);
                    for (int i = 0; i < meshes[b].Count; i++)
                    {
                        var t = SleeveRig.Skin(d, meshes[b], i, pose);
                        double dx = Pd(f, starts[b] + i, 0) - t.X, dy = Pd(f, starts[b] + i, 1) - t.Y, dz = Pd(f, starts[b] + i, 2) - t.Z;
                        double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                        if (i < around) { cuff = Math.Max(cuff, dist); continue; }
                        excess = Math.Max(excess, dist - SleeveRig.MaxDistance);
                        double ex = dx + t.Nx * SleeveRig.BackstopDistance, ey = dy + t.Ny * SleeveRig.BackstopDistance, ez = dz + t.Nz * SleeveRig.BackstopDistance;
                        deficit = Math.Max(deficit, SleeveRig.BackstopRadius - Math.Sqrt(ex * ex + ey * ey + ez * ez));
                    }
                }
            }
            r["skinMaxDistance"] = excess; r["skinBackstop"] = deficit; r["skinCuffError"] = cuff;
        }

        if (want.Contains("softEscapes") && s.Params.ContainsKey("container"))
        {
            double m0 = Robust.EscapeMargin(s);
            double x0 = double.Parse(s.Params["container"][0], CultureInfo.InvariantCulture) - m0, y0 = double.Parse(s.Params["container"][1], CultureInfo.InvariantCulture) - m0;
            double x1 = double.Parse(s.Params["container"][2], CultureInfo.InvariantCulture) + m0, y1 = double.Parse(s.Params["container"][3], CultureInfo.InvariantCulture) + m0;
            int worst = 0;
            for (int f = 0; f < extra.Count; f++)
            {
                int cnt = 0;
                for (int i = 0; i < total; i++) { double x = Pd(f, i, 0), y = Pd(f, i, 1), z = Pd(f, i, 2); if (!(x >= x0 && x <= x1 && y >= y0 && y <= y1 && z >= x0 && z <= x1)) cnt++; }
                worst = Math.Max(worst, cnt);
            }
            r["softEscapes"] = worst;
        }

        if (want.Contains("softPenetration") || want.Contains("softFinalPenetration") || want.Contains("softTunnelled"))
        {
            var parts = Parts(s);
            var depth = new List<double[]>(); var inside = new List<int>();
            for (int f = 0; f < extra.Count; f++)
            {
                var d = frames[f].d;
                var world = parts.Select(p => p.Local.Select(l => { var q = Rot(d[p.Body * St + 3], d[p.Body * St + 4], d[p.Body * St + 5], d[p.Body * St + 6], l); return new V(d[p.Body * St] + q.X, d[p.Body * St + 1] + q.Y, d[p.Body * St + 2] + q.Z); }).ToArray()).ToArray();
                var lo = world.Select(w => (X: w.Min(v => v.X), Y: w.Min(v => v.Y), Z: w.Min(v => v.Z))).ToArray();
                var hi = world.Select(w => (X: w.Max(v => v.X), Y: w.Max(v => v.Y), Z: w.Max(v => v.Z))).ToArray();
                var dep = new double[total]; int ins = 0;
                for (int i = 0; i < total; i++)
                {
                    var c = new V(Pd(f, i, 0), Pd(f, i, 1), Pd(f, i, 2));
                    if (!double.IsFinite(c.X + c.Y + c.Z)) continue;
                    double worstDepth = 0; bool within = false;
                    for (int p = 0; p < parts.Count; p++)
                    {
                        double reach = parts[p].Radius + radius[i];
                        if (c.X < lo[p].X - reach || c.X > hi[p].X + reach || c.Y < lo[p].Y - reach || c.Y > hi[p].Y + reach || c.Z < lo[p].Z - reach || c.Z > hi[p].Z + reach) continue;
                        double dist = CoreDistance([c], world[p]);
                        if (!double.IsFinite(dist)) continue;
                        worstDepth = Math.Max(worstDepth, reach - dist);
                        if (world[p].Length != 3 && dist - parts[p].Radius < 0) within = true;
                    }
                    dep[i] = worstDepth; if (within) ins++;
                }
                depth.Add(dep); inside.Add(ins);
            }
            if (want.Contains("softPenetration")) r["softPenetration"] = depth.Max(a => a.Length == 0 ? 0 : a.Max());
            if (want.Contains("softFinalPenetration"))
            {
                var v = depth.TakeLast(Robust.Window).Select(a => Pct(a, 0.95)).OrderBy(x => x).ToList();
                r["softFinalPenetration"] = v[v.Count / 2];
            }
            if (want.Contains("softTunnelled")) r["softTunnelled"] = inside[^1];
        }
    }

    static double Dist2(float[] v, int a, int b)
    {
        double dx = v[3 * a] - v[3 * b], dy = v[3 * a + 1] - v[3 * b + 1], dz = v[3 * a + 2] - v[3 * b + 2];
        return dx * dx + dy * dy + dz * dz;
    }

    // Signed volume of the tetrahedron with corners given by the accessor.
    static double TetVolume(Func<int, (double X, double Y, double Z)> c)
    {
        var a = c(0); var b = c(1); var cc = c(2); var d = c(3);
        double bx = b.X - a.X, by = b.Y - a.Y, bz = b.Z - a.Z, cx = cc.X - a.X, cy = cc.Y - a.Y, cz = cc.Z - a.Z, dx = d.X - a.X, dy = d.Y - a.Y, dz = d.Z - a.Z;
        return (bx * (cy * dz - cz * dy) - by * (cx * dz - cz * dx) + bz * (cx * dy - cy * dx)) / 6.0;
    }
}
