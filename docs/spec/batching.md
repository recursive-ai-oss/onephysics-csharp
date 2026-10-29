# Width-independent batching

**Question.** If constraint colouring and batch assignment are computed in a fixed order with no reference to the SIMD
width (batches have a fixed logical lane count `L = 16`, chosen once), does a SIMD solver kernel produce bit-identical
results scalar, at 128, 256 and 512 bits and with `Vector<T>`, at any thread count and on every platform?

**Answer on this evidence: yes.** Five kernel widths, 1, 3 and 8 threads, x64 with and without AVX-512 and AVX2,
without any hardware intrinsics, with tiering off, NativeAOT, browser WebAssembly interpreted and compiled, and Apple
arm64 JIT and NativeAOT all produced the same per-step digests over 1000 steps of a 4096-sphere pile with about 16 700 contacts.

## What was built (`experiments/batching/`)

A prototype, not the engine's solver. It lives outside `src/` because it uses `Vector512` and `Vector<T>`.

- **Scene.** 4096 unit-mass spheres (radius 0.5, solid-sphere inertia) dropped as a jittered lattice into a box with a
  floor and four walls (`Pile`). The jitter is a fixed integer congruential sequence. Float32 throughout; positions are
  relative to the box corner, so the double-position rule is not exercised.
- **Narrowphase.** Sphere-sphere and sphere-plane, found with a uniform grid. Contacts are listed in key order (lower body,
  then higher body or plane), so the list order never depends on the grid traversal and warm starting a contact from the
  previous step is a merge of two sorted lists. Friction impulses are kept as world-space vectors and re-projected onto the
  new tangent basis, which is derived from the normal by a fixed rule.
- **Colouring.** Contacts in list order take the lowest colour, below a cap of 10, that neither dynamic body has used yet
  (a 64-bit mask per body; the static body is not tracked). A colour's contacts fill batches of `L = 16` slots in list order,
  the last batch padded with inert slots. Contacts finding no colour (about 40 of 16 700 here; the cap is deliberately below
  the 12 colours the pile needs) form the overflow set, solved serially one lane at a time in list order before the colours.
  Nothing in this depends on the SIMD width or the thread count.
- **Solver.** Soft step (Catto, "Solver2D" and the soft-step write-up; sequential impulses, GDC 2005/2006), 4 substeps of
  1/240 s: gravity, warm start, solve with soft contact bias (30 Hz, damping ratio 10, push-out at most 3 m/s), move,
  relax. Friction (two tangents, clamped by the accumulated normal impulse) before the normal. No restitution or rolling
  resistance; contact anchors do not rotate within a step (exact to first order for spheres).
- **Kernel.** Written once over `ILane<T>` (static abstract operators on a lane group), so every width runs the same
  sequence of `+ - * /`, compares and selects; RyuJIT specialises each struct instantiation. Implementations: `Lanes1`
  (scalar), `Lanes128`, `Lanes256`, `Lanes512`, `LanesT` (`Vector<float>`). No FMA, no estimates, no horizontal reductions:
  lanes of a batch are different constraints on different bodies, so nothing is ever summed across lanes. Max, min and
  negation are built from compare and select so signed zeros follow one rule at every width. Body state is gathered into
  lanes with scalar loads and scattered back (static bodies are never written), because a batch's bodies are disjoint.
- **Threads.** A colour's batches are split into contiguous chunks, one per worker of a small spinning pool; the thread count
  is an argument. Chunks write disjoint bodies, so the split cannot change a result.
- **State digest.** Each step the bodies (velocity, angular velocity, position) and contact impulses are folded into two
  64-bit words by a multiply-xor hash and written, with the contact and overflow counts, to the hash-matrix scenario.

## Sanity of the simulation

From `BatchingExperiment metrics` (4096 spheres, 16 colours cap 10, V256, 1 thread):

| Step | Contacts | Kinetic energy | Max penetration | Max speed |
|---|---|---|---|---|
| 1 | 6 376 | 85.9 | 0.098 (initial overlap) | 0.37 m/s |
| 100 | 16 670 | 31.3 | 0.032 | 1.7 m/s |
| 300 | 16 737 | 0.13 | 0.032 | 0.067 m/s |
| 1000 | 16 737 | 0.017 | 0.032 | 0.049 m/s |

The pile settles: after 300 steps (5 s) the kinetic energy of 4096 bodies is under 0.2,
the deepest overlap is 3.2 % of the diameter, 16 layers deep, and nothing escapes the box. Slow creep continues at
roughly 0.05 m/s as spheres find denser packings; there is no sleeping here.

## Results

See `docs/hash-matrix.md` for the harness. Every column produced the same manifest:
per-scenario hashes and manifest `a694a2895416dca5...`, 10 scenarios each (5 widths x {4096 spheres, 1000 steps; 512 spheres, 200 steps}).

## Pitfalls the kernel avoids

- Unary minus of a vector may be implemented as `0 - x` (a different sign of zero than scalar negation); the kernel builds
  negation, min and max from `0 - x`, compare and select at every width. On this runtime the two happen to agree
  (`BatchingExperiment check` prints both), but nothing promises it.
- Any reduction across lanes, any lane-dependent batch assignment and any width-dependent padding would break the premise;
  none is present.

## Running

```
dotnet run -c Release --project experiments/batching -- check [--small] [--steps N]      # all widths x threads 1, 3, 8, in-process
dotnet run -c Release --project experiments/batching -- metrics                          # settling table
experiments/batching/bench.sh [OUTDIR]                                                   # timing run (takes the benchmark lock)
tools/hash-matrix.sh                                                                     # the full column matrix
```
