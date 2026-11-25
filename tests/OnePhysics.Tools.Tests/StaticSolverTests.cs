using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class StaticSolverTests
{
    private static readonly ElasticMaterial Steel = new() { YoungsModulus = 200e9, PoissonRatio = 0.3, Density = 7850, Strength = 250e6 };

    // Prints the accuracy tables when STATIC_REPORT names a file (used to write docs/spec/tools.md); the assertions hold either way.
    private static void Report(string line)
    {
        string? path = Environment.GetEnvironmentVariable("STATIC_REPORT");
        if (!string.IsNullOrEmpty(path)) { File.AppendAllText(path, line + "\n"); }
    }

    private static double TipDeflection(CantileverModel m, StaticSolution s)
    {
        double sum = 0;
        foreach (int v in m.Tip) { sum += s.Displacements[v].Z; }
        return -sum / m.Tip.Count;
    }

    // Euler-Bernoulli and Timoshenko tip deflections of a rectangular cantilever under an end load.
    private static (double Eb, double Timo) BeamTip(double L, double b, double h, double e, double nu, double load)
    {
        double inertia = b * h * h * h / 12, area = b * h, g = e / (2 * (1 + nu));
        double kappa = 10 * (1 + nu) / (12 + 11 * nu);
        double eb = load * L * L * L / (3 * e * inertia);
        return (eb, eb + load * L / (kappa * g * area));
    }

    [Theory]
    [InlineData(10, 4)]
    [InlineData(20, 4)]
    public void EndLoadedCantileverMatchesBeamTheory(int slenderness, int cellsAcross)
    {
        double h = 0.1, b = 0.1, L = slenderness * h, load = 1000;
        var m = CantileverModel.BuildGrid((float)L, (float)b, (float)h, cellsAcross * slenderness, cellsAcross, cellsAcross);
        var sol = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, load), new StaticSolveOptions { Tolerance = 1e-8 });
        Assert.True(sol.Converged, "residual " + sol.Residual + " after " + sol.Iterations);
        var (eb, timo) = BeamTip(L, b, h, Steel.YoungsModulus, Steel.PoissonRatio, load);
        double tip = TipDeflection(m, sol);
        Report($"cantilever L/h={slenderness} tets={m.Tetrahedra.Length / 4} dofs={sol.DegreesOfFreedom} tip={tip:E4} EB={eb:E4} Timo={timo:E4} errTimo={(tip / timo - 1) * 100:F2}% errEB={(tip / eb - 1) * 100:F2}% iters={sol.Iterations}");
        Assert.InRange(tip / timo, 0.95, 1.05);

        // Reactions balance the load.
        Assert.Equal(load, sol.TotalReaction.Z, 3);
        Assert.True(Math.Abs(sol.TotalReaction.X) < 1e-6 * load && Math.Abs(sol.TotalReaction.Y) < 1e-6 * load);

        // Bending stress: sigma_xx = M z / I with M = P (L - x), in a slab one height from the root, against the beam formula at each element's centroid.
        double inertia = b * h * h * h / 12;
        double worst = 0, peakRef = 6 * load * L / (b * h * h);
        var t = m.Tetrahedra; var v = m.Vertices;
        int counted = 0;
        for (int k = 0; k < t.Length / 4; k++)
        {
            double cx = 0, cz = 0;
            for (int j = 0; j < 4; j++) { cx += v[t[4 * k + j]].X; cz += v[t[4 * k + j]].Z; }
            cx /= 4; cz /= 4;
            if (cx < h || cx > 2 * h) { continue; }
            double expected = load * (L - cx) * cz / inertia;
            worst = Math.Max(worst, Math.Abs(sol.Stresses[k].XX - expected));
            counted++;
        }

        Assert.True(counted > 10);
        // Root stress: the top-fibre elements next to the root (a clamp restrains the section's Poisson contraction, so the corner itself carries a local concentration that the
        // beam formula does not have; the mean over the top layer of the first half height is the bending stress the root section carries).
        double sumFe = 0, sumBeam = 0, cornerPeak = 0;
        int nTop = 0;
        for (int k = 0; k < t.Length / 4; k++)
        {
            double cx = 0, cz = 0;
            for (int j = 0; j < 4; j++) { cx += v[t[4 * k + j]].X; cz += v[t[4 * k + j]].Z; }
            cx /= 4; cz /= 4;
            if (cx > 0.5 * h) { continue; }
            cornerPeak = Math.Max(cornerPeak, sol.PeakVonMises[k]);
            if (cz < 0.25 * h) { continue; }
            sumFe += sol.Stresses[k].XX; sumBeam += load * (L - cx) * cz / inertia; nTop++;
        }

        Assert.True(nTop > 0);
        Report($"  slab stress max abs error = {worst / peakRef * 100:F2}% of 6PL/bh^2; root top layer mean sigma_xx FE/beam = {sumFe / sumBeam:F4}; clamp-corner peak von Mises = {cornerPeak / peakRef:F3} of 6PL/bh^2; residual {sol.Residual:E1}");
        Assert.True(worst / peakRef < 0.05);
        Assert.InRange(sumFe / sumBeam, 0.95, 1.05);
    }

    [Fact]
    public void RefinementTableConverges()
    {
        // Tip deflection of the L/h = 10 beam against Timoshenko as the grid is refined across the section (cells across; 4 per height along x per cell across).
        double h = 0.1, b = 0.1, L = 1.0, load = 1000;
        var (_, timo) = BeamTip(L, b, h, Steel.YoungsModulus, Steel.PoissonRatio, load);
        double lastQ = 0;
        foreach (int c in new[] { 1, 2, 3, 4, 6 })
        {
            var m = CantileverModel.BuildGrid((float)L, (float)b, (float)h, 10 * c, c, c);
            var q = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, load), new StaticSolveOptions { Tolerance = 1e-8 });
            var l = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, load), new StaticSolveOptions { ElementOrder = StaticElementOrder.Linear, Tolerance = 1e-8 });
            Report($"refinement grid {c} cells across: tets={m.Tetrahedra.Length / 4} quadratic tip/Timo={TipDeflection(m, q) / timo:F4} (iters {q.Iterations}) linear tip/Timo={TipDeflection(m, l) / timo:F4} (iters {l.Iterations})");
            lastQ = TipDeflection(m, q) / timo;
        }

        Assert.InRange(lastQ, 0.97, 1.03);
        // The same beam cut from a box by the tetrahedralizer: the lattice boundary is ragged, so the section is slightly off and the error is the mesh's, not the elements'.
        foreach (float cell in new[] { 0.05f, 0.033f, 0.025f })
        {
            var m = CantileverModel.BuildLattice((float)L, (float)b, (float)h, cell);
            var q = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, load), new StaticSolveOptions { Tolerance = 1e-8 });
            Report($"refinement tetrahedralizer cell {cell}: tets={m.Tetrahedra.Length / 4} volume ratio {q.Volume / (L * b * h):F4} quadratic tip/Timo={TipDeflection(m, q) / timo:F4}");
        }
    }

    [Fact]
    public void LinearElementsLockInBending()
    {
        double h = 0.1, b = 0.1, L = 1.0, load = 1000;
        var m = CantileverModel.BuildGrid((float)L, (float)b, (float)h, 40, 4, 4);
        var lin = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, load), new StaticSolveOptions { ElementOrder = StaticElementOrder.Linear, Tolerance = 1e-8 });
        var quad = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, load), new StaticSolveOptions { Tolerance = 1e-8 });
        var (_, timo) = BeamTip(L, b, h, Steel.YoungsModulus, Steel.PoissonRatio, load);
        double rl = TipDeflection(m, lin) / timo, rq = TipDeflection(m, quad) / timo;
        Report($"linear vs quadratic, same mesh ({m.Tetrahedra.Length / 4} tets): tip/Timoshenko linear {rl:F3}, quadratic {rq:F3}");
        Assert.True(rl < 0.8);
        Assert.InRange(rq, 0.95, 1.05);
    }

    [Fact]
    public void SelfWeightCantileverMatchesBeamTheory()
    {
        double h = 0.1, b = 0.1, L = 1.0;
        var m = CantileverModel.BuildGrid((float)L, (float)b, (float)h, 40, 4, 4);
        var sol = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.SelfWeight(Steel), new StaticSolveOptions { Tolerance = 1e-8 });
        Assert.True(sol.Converged, "residual " + sol.Residual + " after " + sol.Iterations);
        double w = Steel.Density * 9.81 * b * h, inertia = b * h * h * h / 12;
        double eb = w * L * L * L * L / (8 * Steel.YoungsModulus * inertia);
        // Shear deformation under a distributed load adds 3 w L^2 / (8 kappa G A) at the tip... small for L/h = 10, included for the comparison.
        double kappa = 10 * (1 + Steel.PoissonRatio) / (12 + 11 * Steel.PoissonRatio), g = Steel.YoungsModulus / (2 * (1 + Steel.PoissonRatio));
        double timo = eb + w * L * L / (2 * kappa * g * b * h);
        double tip = TipDeflection(m, sol);
        Report($"self weight L/h=10 tip={tip:E4} EB={eb:E4} Timo={timo:E4} err={(tip / timo - 1) * 100:F2}%");
        Assert.InRange(tip / timo, 0.95, 1.05);
        Assert.Equal(Steel.Density * 9.81 * sol.Volume, sol.TotalReaction.Z, 3);
        Assert.Equal(Steel.Density * sol.Volume, sol.Mass, 6);
    }

    [Fact]
    public void UniformCompressionIsExact()
    {
        var (cv, ct) = StaticMeshes.Grid(0, 1, 0, 1, 0, 1, 3, 3, 3);
        var mesh = (Vertices: cv, Tetrahedra: ct);
        var mat = ElasticMaterial.Create(1e6, 0.25, 1000, 1e5);
        foreach (var order in new[] { StaticElementOrder.Quadratic, StaticElementOrder.Linear })
        {
            var p = new StaticProblem { Material = mat };
            for (int i = 0; i < mesh.Vertices.Length; i++)
            {
                var q = mesh.Vertices[i];
                var axes = SupportAxes.None;
                if (Math.Abs(q.X) < 1e-4) { axes |= SupportAxes.X; }
                if (Math.Abs(q.Y) < 1e-4) { axes |= SupportAxes.Y; }
                if (Math.Abs(q.Z) < 1e-4) { axes |= SupportAxes.Z; }
                if (axes != SupportAxes.None) { p.Supports.Add(new StaticSupport(i, axes)); }
            }

            double pressure = 2000;
            var t = mesh.Tetrahedra;
            var top = new HashSet<int>(Enumerable.Range(0, mesh.Vertices.Length).Where(i => Math.Abs(mesh.Vertices[i].Z - 1) < 1e-4));
            int[][] faces = { new[] { 0, 1, 2 }, new[] { 0, 1, 3 }, new[] { 0, 2, 3 }, new[] { 1, 2, 3 } };
            for (int k = 0; k < t.Length / 4; k++)
            {
                foreach (var f in faces)
                {
                    int a = t[4 * k + f[0]], b = t[4 * k + f[1]], c = t[4 * k + f[2]];
                    if (top.Contains(a) && top.Contains(b) && top.Contains(c)) { p.FaceLoads.Add(new FaceLoad(a, b, c, pressure)); }
                }
            }

            var sol = StaticSolver.Solve(mesh.Vertices, mesh.Tetrahedra, p, new StaticSolveOptions { ElementOrder = order, Tolerance = 1e-14 });
            Assert.True(sol.Converged);
            double worstU = 0, worstS = 0;
            for (int i = 0; i < mesh.Vertices.Length; i++)
            {
                var q = mesh.Vertices[i];
                double ez = -pressure / mat.YoungsModulus, ex = mat.PoissonRatio * pressure / mat.YoungsModulus;
                var d = sol.Displacements[i];
                worstU = Math.Max(worstU, Math.Max(Math.Abs(d.X - ex * q.X), Math.Max(Math.Abs(d.Y - ex * q.Y), Math.Abs(d.Z - ez * q.Z))) / Math.Abs(ez));
            }

            for (int k = 0; k < t.Length / 4; k++)
            {
                var s = sol.Stresses[k];
                worstS = Math.Max(worstS, Math.Max(Math.Abs(s.ZZ + pressure), Math.Max(Math.Max(Math.Abs(s.XX), Math.Abs(s.YY)), Math.Max(Math.Abs(s.XY), Math.Max(Math.Abs(s.YZ), Math.Abs(s.ZX))))) / pressure);
            }

            Report($"uniform compression {order}: worst displacement error {worstU:E2} (relative to the compression), worst stress error {worstS:E2} (relative to the pressure), {sol.Iterations} iterations");
            Assert.True(worstU < 1e-9, "displacement " + worstU);
            Assert.True(worstS < 1e-8, "stress " + worstS);
            Assert.Equal(pressure, sol.MaxVonMises, 4);
            Assert.Equal(mat.Strength / pressure, sol.MinSafetyFactor, 6);
        }
    }

    // A quarter-symmetric plate with a circular hole, in uniaxial tension along x, half its thickness: structured mesh (radial lines from the hole to the outer rectangle, six tetrahedra per cell).
    private sealed class HolePlate
    {
        public Vector3F[] V = Array.Empty<Vector3F>();
        public int[] T = Array.Empty<int>();
        public List<(int, int, int)> EndFaces = new();
        public double A, Sx, Sy;
    }

    private static HolePlate BuildHolePlate(double a, double sx, double sy, double halfThickness, int nr, int nAlong, int nAcross, int nz, double grade)
    {
        int nth = nAlong + nAcross;
        var vs = new List<Vector3F>();
        int Idx(int i, int j, int k) => (k * (nth + 1) + j) * (nr + 1) + i;
        for (int k = 0; k <= nz; k++)
        {
            for (int j = 0; j <= nth; j++)
            {
                double th = Math.PI / 2 * j / nth;
                double px, py;
                if (j <= nAlong) { px = sx; py = sy * j / nAlong; } else { px = sx * (1 - (double)(j - nAlong) / nAcross); py = sy; }
                double ix = a * Math.Cos(th), iy = a * Math.Sin(th);
                for (int i = 0; i <= nr; i++)
                {
                    double s = Math.Pow((double)i / nr, grade);
                    vs.Add(new Vector3F((float)(ix + s * (px - ix)), (float)(iy + s * (py - iy)), (float)(halfThickness * k / nz)));
                }
            }
        }

        var tets = new List<int>();
        int[][] perms = { new[] { 0, 1, 2 }, new[] { 0, 2, 1 }, new[] { 1, 0, 2 }, new[] { 1, 2, 0 }, new[] { 2, 0, 1 }, new[] { 2, 1, 0 } };
        for (int k = 0; k < nz; k++)
        {
            for (int j = 0; j < nth; j++)
            {
                for (int i = 0; i < nr; i++)
                {
                    foreach (var pm in perms)
                    {
                        var c = new int[3]; // step along axes pm[0], pm[1], pm[2] in turn
                        int[] pos = { i, j, k };
                        int v0 = Idx(i, j, k);
                        pos[pm[0]]++; int v1 = Idx(pos[0], pos[1], pos[2]);
                        pos[pm[1]]++; int v2 = Idx(pos[0], pos[1], pos[2]);
                        pos[pm[2]]++; int v3 = Idx(pos[0], pos[1], pos[2]);
                        // Even permutations of (i, j, k) are right-handed here, odd ones need two vertices swapped.
                        bool odd = (pm[0] == 0 && pm[1] == 2) || (pm[0] == 1 && pm[1] == 0) || (pm[0] == 2 && pm[1] == 1);
                        if (odd) { tets.Add(v0); tets.Add(v2); tets.Add(v1); tets.Add(v3); } else { tets.Add(v0); tets.Add(v1); tets.Add(v2); tets.Add(v3); }
                        _ = c;
                    }
                }
            }
        }

        var plate = new HolePlate { V = vs.ToArray(), T = tets.ToArray(), A = a, Sx = sx, Sy = sy };
        var endSet = new HashSet<int>(Enumerable.Range(0, plate.V.Length).Where(i => Math.Abs(plate.V[i].X - sx) < 1e-5 * sx));
        int[][] faces = { new[] { 0, 1, 2 }, new[] { 0, 1, 3 }, new[] { 0, 2, 3 }, new[] { 1, 2, 3 } };
        for (int k = 0; k < plate.T.Length / 4; k++)
        {
            foreach (var f in faces)
            {
                int p0 = plate.T[4 * k + f[0]], p1 = plate.T[4 * k + f[1]], p2 = plate.T[4 * k + f[2]];
                if (endSet.Contains(p0) && endSet.Contains(p1) && endSet.Contains(p2)) { plate.EndFaces.Add((p0, p1, p2)); }
            }
        }

        return plate;
    }

    private static (double Peak, int Tets) HoleCase(int nr, int nAlong, int nAcross, int nz, StaticElementOrder order, double sigma, double[]? thin = null)
    {
        double a = 1, sy = 5, sx = 15, half = 0.25;
        var plate = BuildHolePlate(a, sx, sy, half, nr, nAlong, nAcross, nz, 1.6);
        var p = new StaticProblem { Material = ElasticMaterial.Create(1e9, 0.3, 0, 1e7) };
        for (int i = 0; i < plate.V.Length; i++)
        {
            var q = plate.V[i];
            var axes = SupportAxes.None;
            if (Math.Abs(q.X) < 1e-5) { axes |= SupportAxes.X; }
            if (Math.Abs(q.Y) < 1e-5) { axes |= SupportAxes.Y; }
            if (Math.Abs(q.Z) < 1e-5) { axes |= SupportAxes.Z; }
            if (axes != SupportAxes.None) { p.Supports.Add(new StaticSupport(i, axes)); }
        }

        foreach (var (v0, v1, v2) in plate.EndFaces) { p.FaceLoads.Add(new FaceLoad(v0, v1, v2, -sigma)); }
        // Ends at x = sx are pulled outward (+x) by a negative pressure; the other outer faces are free.
        var sol = StaticSolver.Solve(plate.V, plate.T, p, new StaticSolveOptions { ElementOrder = order, Tolerance = 1e-11 });
        Assert.True(sol.Converged);
        double peak = 0;
        for (int i = 0; i < plate.V.Length; i++)
        {
            var q = plate.V[i];
            if (Math.Abs(q.X) < 1e-5 && Math.Abs(q.Y - a) < 1e-5) { peak = Math.Max(peak, sol.VertexVonMises[i]); }
        }

        return (peak / sigma, plate.T.Length / 4);
    }

    [Fact]
    public void PlateWithHoleHasKirschConcentration()
    {
        // Hole diameter 2 in a plate 10 wide: the finite-width stress concentration on the gross section is Kt_net (1 - d/W)^-1 with
        // Kt_net = 3.00 - 3.13 (d/W) + 3.66 (d/W)^2 - 1.53 (d/W)^3 (Peterson), 3.135 for d/W = 0.2.
        double dw = 0.2;
        double ktNet = 3.00 - 3.13 * dw + 3.66 * dw * dw - 1.53 * dw * dw * dw;
        double expected = ktNet / (1 - dw);
        double last = 0;
        foreach (var (nr, na, nc, nz) in new[] { (6, 6, 8, 1), (10, 8, 12, 1), (16, 10, 16, 2), (24, 14, 24, 2) })
        {
            var (peak, tets) = HoleCase(nr, na, nc, nz, StaticElementOrder.Quadratic, 1e6);
            Report($"hole plate quadratic tets={tets} peak/sigma={peak:F4} expected {expected:F4} err={(peak / expected - 1) * 100:F2}%");
            last = peak;
        }

        var (lin, lt) = HoleCase(16, 10, 16, 2, StaticElementOrder.Linear, 1e6);
        Report($"hole plate linear tets={lt} peak/sigma={lin:F4} err={(lin / expected - 1) * 100:F2}%");
        Assert.InRange(last / expected, 0.95, 1.05);
    }

    [Fact]
    public void ResultsAreBitIdenticalAtEveryThreadCount()
    {
        var m = CantileverModel.BuildGrid(1f, 0.1f, 0.1f, 20, 2, 2);
        StaticSolution? first = null;
        foreach (int threads in new[] { 1, 4, 8 })
        {
            var sol = StaticSolver.Solve(m.Vertices, m.Tetrahedra, m.EndLoad(Steel, 500), new StaticSolveOptions { MaxThreads = threads });
            if (first is null) { first = sol; continue; }
            Assert.Equal(first.Iterations, sol.Iterations);
            Assert.Equal(BitConverter.DoubleToInt64Bits(first.Residual), BitConverter.DoubleToInt64Bits(sol.Residual));
            for (int i = 0; i < sol.Displacements.Length; i++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(first.Displacements[i].X), BitConverter.DoubleToInt64Bits(sol.Displacements[i].X));
                Assert.Equal(BitConverter.DoubleToInt64Bits(first.Displacements[i].Y), BitConverter.DoubleToInt64Bits(sol.Displacements[i].Y));
                Assert.Equal(BitConverter.DoubleToInt64Bits(first.Displacements[i].Z), BitConverter.DoubleToInt64Bits(sol.Displacements[i].Z));
            }

            for (int k = 0; k < sol.VonMises.Length; k++)
            {
                Assert.Equal(BitConverter.DoubleToInt64Bits(first.PeakVonMises[k]), BitConverter.DoubleToInt64Bits(sol.PeakVonMises[k]));
                Assert.Equal(BitConverter.DoubleToInt64Bits(first.SafetyFactors[k]), BitConverter.DoubleToInt64Bits(sol.SafetyFactors[k]));
            }
        }
    }

    [Fact]
    public void MetresPerUnitScalesTheProblem()
    {
        // The same beam in millimetres gives the same displacement in metres.
        var a = CantileverModel.BuildGrid(1f, 0.1f, 0.1f, 20, 2, 2);
        var sa = StaticSolver.Solve(a.Vertices, a.Tetrahedra, a.EndLoad(Steel, 500));
        var verts = a.Vertices.Select(v => new Vector3F(v.X * 1000, v.Y * 1000, v.Z * 1000)).ToArray();
        var sb = StaticSolver.Solve(verts, a.Tetrahedra, a.EndLoad(Steel, 500), new StaticSolveOptions { MetresPerUnit = 0.001 });
        Assert.Equal(sa.MaxDisplacement, sb.MaxDisplacement, 9);
        Assert.Equal(sa.MaxVonMises, sb.MaxVonMises, sa.MaxVonMises * 1e-5);
    }

    [Fact]
    public void LayerKnockDownLowersTheSafetyFactorAcrossTheLayers()
    {
        // A bar pulled along z: with layers stacked along z the tension is across them; stacked along x it is along them.
        var (lv, lt) = StaticMeshes.Grid(-0.05f, 0.05f, -0.05f, 0.05f, 0, 0.5f, 2, 2, 10);
        var mesh = (Vertices: lv, Tetrahedra: lt);
        double[] sf = new double[3];
        for (int c = 0; c < 3; c++)
        {
            var mat = ElasticMaterial.Create(2e9, 0.35, 1200, 40e6);
            if (c == 1) { mat = new ElasticMaterial { YoungsModulus = 2e9, PoissonRatio = 0.35, Density = 1200, Strength = 40e6, WeakAxis = new Vector3F(0, 0, 1), WeakAxisStrengthRatio = 0.5 }; }
            if (c == 2) { mat = new ElasticMaterial { YoungsModulus = 2e9, PoissonRatio = 0.35, Density = 1200, Strength = 40e6, WeakAxis = new Vector3F(1, 0, 0), WeakAxisStrengthRatio = 0.5 }; }
            var p = new StaticProblem { Material = mat };
            var t = mesh.Tetrahedra;
            var low = new HashSet<int>(); var high = new HashSet<int>();
            for (int i = 0; i < mesh.Vertices.Length; i++)
            {
                if (Math.Abs(mesh.Vertices[i].Z) < 1e-4) { low.Add(i); }
                if (Math.Abs(mesh.Vertices[i].Z - 0.5f) < 1e-4) { high.Add(i); }
            }

            // All of the bottom face held in z, one vertex pinned fully and a second held in y so that no rigid motion is left.
            int pin = low.Min();
            int other = low.First(i => mesh.Vertices[i].X != mesh.Vertices[pin].X);
            foreach (int i in low) { p.Supports.Add(new StaticSupport(i, i == pin ? SupportAxes.All : i == other ? SupportAxes.Z | SupportAxes.Y : SupportAxes.Z)); }
            int[][] faces = { new[] { 0, 1, 2 }, new[] { 0, 1, 3 }, new[] { 0, 2, 3 }, new[] { 1, 2, 3 } };
            for (int k = 0; k < t.Length / 4; k++)
            {
                foreach (var f in faces)
                {
                    int a = t[4 * k + f[0]], b = t[4 * k + f[1]], d = t[4 * k + f[2]];
                    if (high.Contains(a) && high.Contains(b) && high.Contains(d)) { p.FaceLoads.Add(new FaceLoad(a, b, d, -10e6)); }
                }
            }

            var sol = StaticSolver.Solve(mesh.Vertices, mesh.Tetrahedra, p, new StaticSolveOptions { Tolerance = 1e-8 });
            Assert.True(sol.Converged);
            sf[c] = sol.MinSafetyFactor;
        }

        Report($"layer knock-down: tension across layers SF {sf[1]:F3}, along layers {sf[2]:F3}, isotropic {sf[0]:F3}");
        Assert.InRange(sf[0], 3.9, 4.1);
        Assert.InRange(sf[1], 1.9, 2.1);
        Assert.InRange(sf[2], 3.9, 4.1);
    }

    // One tetrahedron with three vertices fixed and a load on the fourth, then spoiled one way at a time.
    private static (Vector3F[] V, int[] T, StaticProblem P) Tiny()
    {
        var v = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1) };
        var t = new[] { 0, 1, 2, 3 };
        var p = new StaticProblem { Material = ElasticMaterial.Create(1e6, 0.3, 1000, 1e4) };
        p.Supports.Add(new StaticSupport(0)); p.Supports.Add(new StaticSupport(1)); p.Supports.Add(new StaticSupport(2));
        p.PointLoads.Add(new PointLoad(3, new Vector3D(0, 0, -10)));
        return (v, t, p);
    }

    private static StaticSolveException Fails(Vector3F[] v, int[] t, StaticProblem p, StaticSolveOptions o = default)
        => Assert.Throws<StaticSolveException>(() => StaticSolver.Solve(v, t, p, o));

    private static StaticSolveException FailsWith(Vector3F[] v, int[] t, StaticProblem q, FaceLoad f)
    {
        q.FaceLoads.Add(f);
        return Fails(v, t, q);
    }

    [Fact]
    public void ValidTinyModelSolves()
    {
        var (v, t, p) = Tiny();
        var sol = StaticSolver.Solve(v, t, p);
        Assert.True(sol.Converged);
        Assert.True(sol.Displacements[3].Z < 0);
        Assert.Equal(10, sol.TotalReaction.Z, 6);
    }

    [Fact]
    public void HostileInputsAreTypedErrorsThatNameTheInput()
    {
        var (v, t, p) = Tiny();

        var e = Fails(v, t, new StaticProblem { Material = p.Material, PointLoads = { new PointLoad(3, new Vector3D(0, 0, -1)) } });
        Assert.Equal(StaticSolveError.NoSupports, e.Error);

        var one = new StaticProblem { Material = p.Material }; one.Supports.Add(new StaticSupport(0));
        e = Fails(v, t, one);
        Assert.Equal(StaticSolveError.Mechanism, e.Error);

        var line = new StaticProblem { Material = p.Material }; line.Supports.Add(new StaticSupport(0)); line.Supports.Add(new StaticSupport(1));
        e = Fails(v, t, line);
        Assert.Equal(StaticSolveError.Mechanism, e.Error);   // a rotation about the line through the two fixed points

        var rollers = new StaticProblem { Material = p.Material }; rollers.Supports.Add(new StaticSupport(0, SupportAxes.Z)); rollers.Supports.Add(new StaticSupport(1, SupportAxes.Z)); rollers.Supports.Add(new StaticSupport(2, SupportAxes.Z));
        e = Fails(v, t, rollers);
        Assert.Equal(StaticSolveError.Mechanism, e.Error);   // sliding in the plane

        var bad = (Vector3F[])v.Clone(); bad[2] = new Vector3F(float.NaN, 0, 0);
        e = Fails(bad, t, p);
        Assert.Equal(StaticSolveError.NonFinite, e.Error); Assert.Contains("vertices[2]", e.Message);

        var inf = (Vector3F[])v.Clone(); inf[1] = new Vector3F(float.PositiveInfinity, 0, 0);
        Assert.Equal(StaticSolveError.NonFinite, Fails(inf, t, p).Error);

        e = Fails(v, new[] { 0, 2, 1, 3 }, p);
        Assert.Equal(StaticSolveError.InvertedTetrahedron, e.Error); Assert.Contains("tetrahedron 0", e.Message);

        e = Fails(new[] { v[0], v[1], v[2], new Vector3F(1, 1, 0) }, t, p);
        Assert.Equal(StaticSolveError.DegenerateTetrahedron, e.Error);

        e = Fails(v, new[] { 0, 1, 2, 2 }, p);
        Assert.Equal(StaticSolveError.DegenerateTetrahedron, e.Error);

        e = Fails(v, new[] { 0, 1, 2, 7 }, p);
        Assert.Equal(StaticSolveError.BadIndex, e.Error); Assert.Contains("tetrahedra[3]", e.Message);

        e = Fails(v, new[] { 0, 1, 2 }, p);
        Assert.Equal(StaticSolveError.BadIndex, e.Error);

        e = Fails(v, Array.Empty<int>(), p);
        Assert.Equal(StaticSolveError.Empty, e.Error);

        var extra = v.Append(new Vector3F(5, 5, 5)).ToArray();
        e = Fails(extra, t, p);
        Assert.Equal(StaticSolveError.UnusedVertex, e.Error); Assert.Contains("vertices[4]", e.Message);
    }

    [Fact]
    public void HostileMaterialsLoadsAndOptionsAreTypedErrors()
    {
        var (v, t, p) = Tiny();
        StaticProblem With(ElasticMaterial m) { var q = new StaticProblem { Material = m, Gravity = p.Gravity }; q.Supports.AddRange(p.Supports); q.PointLoads.AddRange(p.PointLoads); return q; }

        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(ElasticMaterial.Create(0, 0.3))).Error);
        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(ElasticMaterial.Create(-5, 0.3))).Error);
        Assert.Equal(StaticSolveError.NonFinite, Fails(v, t, With(ElasticMaterial.Create(double.NaN, 0.3))).Error);
        Assert.Equal(StaticSolveError.NonFinite, Fails(v, t, With(ElasticMaterial.Create(1e6, double.NaN))).Error);
        var nu = Fails(v, t, With(ElasticMaterial.Create(1e6, 0.5)));
        Assert.Equal(StaticSolveError.OutOfRange, nu.Error); Assert.Contains("PoissonRatio", nu.Message);
        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(ElasticMaterial.Create(1e6, 0.7))).Error);
        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(ElasticMaterial.Create(1e6, -1))).Error);
        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(ElasticMaterial.Create(1e6, 0.3, -1))).Error);
        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(new ElasticMaterial { YoungsModulus = 1e6, PoissonRatio = 0.3, Strength = 1e4, WeakAxisStrengthRatio = 0.5 })).Error);
        Assert.Equal(StaticSolveError.OutOfRange, Fails(v, t, With(new ElasticMaterial { YoungsModulus = 1e6, PoissonRatio = 0.3, Strength = 1e4, WeakAxis = new Vector3F(0, 0, 1), WeakAxisStrengthRatio = 1.5 })).Error);

        var load = With(p.Material); load.PointLoads.Add(new PointLoad(0, new Vector3D(double.NaN, 0, 0)));
        Assert.Equal(StaticSolveError.NonFinite, Fails(v, t, load).Error);
        var badVertex = With(p.Material); badVertex.PointLoads.Add(new PointLoad(9, new Vector3D(1, 0, 0)));
        Assert.Equal(StaticSolveError.BadIndex, Fails(v, t, badVertex).Error);
        var dup = With(p.Material); dup.Supports.Add(new StaticSupport(0, SupportAxes.X));
        Assert.Equal(StaticSolveError.BadSupport, Fails(v, t, dup).Error);
        var none = With(p.Material); none.Supports.Clear(); none.Supports.Add(new StaticSupport(0, SupportAxes.None));
        Assert.Equal(StaticSolveError.BadSupport, Fails(v, t, none).Error);
        var grav = With(p.Material); grav.Gravity = new Vector3D(0, double.NaN, 0);
        Assert.Equal(StaticSolveError.NonFinite, Fails(v, t, grav).Error);

        // A face load must name three vertices of one tetrahedron, all distinct.
        Assert.Equal(StaticSolveError.BadIndex, FailsWith(v, t, With(p.Material), new FaceLoad(0, 1, 7, 5)).Error);
        var two = new[] { v[0], v[1], v[2], v[3], new Vector3F(1, 1, 1) };
        var twoTets = new[] { 0, 1, 2, 3, 1, 2, 3, 4 };
        var interior = FailsWith(two, twoTets, With(p.Material), new FaceLoad(1, 2, 3, 5));
        Assert.Equal(StaticSolveError.BadFace, interior.Error); Assert.Contains("interior", interior.Message);
        Assert.Equal(StaticSolveError.BadFace, FailsWith(two, twoTets, With(p.Material), new FaceLoad(0, 1, 4, 5)).Error);
        Assert.Equal(StaticSolveError.BadFace, FailsWith(v, t, With(p.Material), new FaceLoad(1, 1, 2, 5)).Error);

        foreach (var o in new[]
        {
            new StaticSolveOptions { MaxThreads = -1 }, new StaticSolveOptions { MaxThreads = 1000 }, new StaticSolveOptions { MetresPerUnit = -1 }, new StaticSolveOptions { MetresPerUnit = double.NaN },
            new StaticSolveOptions { Tolerance = 1 }, new StaticSolveOptions { Tolerance = double.NaN }, new StaticSolveOptions { MaxIterations = -3 }, new StaticSolveOptions { ElementOrder = (StaticElementOrder)7 },
        })
        {
            Assert.Equal(StaticSolveError.InvalidOptions, Fails(v, t, p, o).Error);
        }
    }

    [Fact]
    public void SeparateUnsupportedPartIsAMechanism()
    {
        // Two tetrahedra that share no vertex; only the first is held.
        var v = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1), new Vector3F(5, 0, 0), new Vector3F(6, 0, 0), new Vector3F(5, 1, 0), new Vector3F(5, 0, 1) };
        var t = new[] { 0, 1, 2, 3, 4, 5, 6, 7 };
        var p = new StaticProblem { Material = ElasticMaterial.Create(1e6, 0.3) };
        p.Supports.Add(new StaticSupport(0)); p.Supports.Add(new StaticSupport(1)); p.Supports.Add(new StaticSupport(2));
        var e = Fails(v, t, p);
        Assert.Equal(StaticSolveError.Mechanism, e.Error); Assert.Contains("vertex 4", e.Message);
    }

    [Fact]
    public void FaceLoadsPushInwardAndCarryTheirResultant()
    {
        // A block under a pressure on its top face: the reaction is the pressure times the area, and a positive pressure compresses.
        var (v, t) = StaticMeshes.Grid(0, 2, 0, 3, 0, 1, 2, 3, 1);
        var p = new StaticProblem { Material = ElasticMaterial.Create(1e7, 0.2) };
        for (int i = 0; i < v.Length; i++) { if (v[i].Z == 0) { p.Supports.Add(new StaticSupport(i, i == 0 ? SupportAxes.All : i == 1 ? SupportAxes.Z | SupportAxes.Y : SupportAxes.Z)); } }
        int[][] faces = { new[] { 0, 1, 2 }, new[] { 0, 1, 3 }, new[] { 0, 2, 3 }, new[] { 1, 2, 3 } };
        var top = new HashSet<int>(Enumerable.Range(0, v.Length).Where(i => v[i].Z == 1));
        for (int k = 0; k < t.Length / 4; k++) { foreach (var f in faces) { int a = t[4 * k + f[0]], b = t[4 * k + f[1]], c = t[4 * k + f[2]]; if (top.Contains(a) && top.Contains(b) && top.Contains(c)) { p.FaceLoads.Add(new FaceLoad(c, a, b, 1000)); } } }
        foreach (var order in new[] { StaticElementOrder.Quadratic, StaticElementOrder.Linear })
        {
            var sol = StaticSolver.Solve(v, t, p, new StaticSolveOptions { ElementOrder = order });
            Assert.True(sol.Converged);
            Assert.Equal(6000, sol.TotalReaction.Z, 3);
            Assert.True(sol.Displacements.Max(d => d.Z) <= 1e-12);
        }
    }
}
