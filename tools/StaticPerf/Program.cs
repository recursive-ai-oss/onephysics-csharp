using System.Diagnostics;
using RecursiveAI.OnePhysics.Tools;
using RecursiveAI.OnePhysics.Tools.Tests;

// Indicative time and memory of the static solve on a beam of N cells across (10 N x N x N cells, six tetrahedra each): StaticPerf <cells> <threads> <reps> [linear]
int cells = int.Parse(args[0]), threads = int.Parse(args[1]), reps = int.Parse(args[2]);
var order = args.Length > 3 ? StaticElementOrder.Linear : StaticElementOrder.Quadratic;
var m = CantileverModel.BuildGrid(1f, 0.1f, 0.1f, 10 * cells, cells, cells);
var p = m.EndLoad(ElasticMaterial.Create(200e9, 0.3, 7850, 250e6), 1000);
StaticSolution? s = null;
var times = new List<double>();
for (int i = 0; i < reps; i++)
{
    var sw = Stopwatch.StartNew();
    s = StaticSolver.Solve(m.Vertices, m.Tetrahedra, p, new StaticSolveOptions { MaxThreads = threads, ElementOrder = order });
    times.Add(sw.Elapsed.TotalMilliseconds);
}

times.Sort();
Console.WriteLine($"{order} tets={m.Tetrahedra.Length / 4} dofs={s!.DegreesOfFreedom} threads={threads} iterations={s.Iterations} residual={s.Residual:E2} converged={s.Converged} solve_ms min={times[0]:F0} median={times[times.Count / 2]:F0} max={times[^1]:F0} peak_working_set_MB={Process.GetCurrentProcess().PeakWorkingSet64 / 1e6:F0}");
