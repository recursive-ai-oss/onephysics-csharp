# Diagnosis

Normative, language-neutral. `world.Diagnose()` (both worlds) and `ReplayDiagnostics.Diagnose(stream, firstStep, lastStep)` return a list of findings about a world: what is wrong with it or worth knowing, with the measured values and a suggested change. This page defines what is measured, how, and the threshold for each code with the evidence it comes from. A port must produce the same findings in the same order with the same values.

Nothing here changes a world's results, snapshot or hash: diagnosis reads the world, and everything that needs time runs in a structural copy (`Fork()`), or, for a recording, in a playback world the call owns.

## Types

- `DiagnosticCode`: the stable number of a check, `OPD` + number as text. Numbers are never reused or renumbered.
- `DiagnosticSeverity`: `Info` 0, `Warning` 1, `Error` 2.
- `Finding`: code, severity, up to 8 body handles and up to 8 joint handles (ascending slot order), a list of values `(name, value as double, unit, threshold or NaN)`, an explanation and a suggestion (English text, built from the values with `G4` invariant formatting).
- `DiagnosisReport`: findings, `StepsObserved`, `FirstStep` (the world's step count when the examined state began), `LastStep`, `Suppressed`.
- `DiagnoseOptions`: `ProbeSteps` (0..100000, default 90), `Dt` (finite positive float, default 1/60), `MaxFindingsPerCode` (1..1000, default 16).

Invalid options raise `ArgumentOutOfRangeException` naming the value before anything is examined.

## Method

A diagnosis is a **state pass** followed by an optional **observation** and a **judgement**.

1. **State pass** on the world as it is (live) or at `firstStep` (window): settings (gravity, sleep options), per dynamic body (size, speed), every solving joint (mass ratio, anchor distances, initial anchor gap), and the contacts the world holds (once).
2. **Observation** of N steps. Live: if `ProbeSteps > 0` and some dynamic body is awake, the world is forked, the fork's `DetailedInspection` is switched on (it changes no result), and the fork is stepped N = `ProbeSteps` times with `Dt`, with callbacks suppressed; a step that throws an `InvalidOperationException` or `ArgumentException` ends the observation early and adds an `OPD1016` Info finding. Window: the playback world is played from the start of the recording to `firstStep` (state pass), `DetailedInspection` is switched on, and each step to `lastStep` (N = `lastStep - firstStep`, each step's recorded dt) is observed; a window outside the recording raises `ArgumentOutOfRangeException`.
3. **Judgement** over what was seen, at the world after the last observed step.

Observed step *i* of N (0-based) belongs to **phase** `min(2, floor(3i / N))`: the first, middle and last third. A *mean over a third* is a plain mean over the steps of that third; "not falling" for a quantity means *last-third mean ≥ 0.9 × middle-third mean*. Trend findings need at least 6 observed steps.

Everything is arithmetic on float32 state in double with IEEE-exact operations (`+ − × ÷ sqrt`); no transcendental function is called.

**Motion** of a body is its linear speed plus its angular speed times its reach (the farthest shape point from the centre of mass), the same quantity the sleep test and continuous collision use. **Thickness** of a body is twice the smallest half-width of its shapes (for a polygon the nearest side from the centroid plus the rounding radius).

**Order.** Findings are ordered by ascending code, then descending severity, then descending metric, then ascending key (the first body slot, else the first joint slot), then creation order. At most `MaxFindingsPerCode` are kept per code; `Suppressed` counts the rest. Entity lists are ascending slots.

**Cost** (indicative; x64 desktop, 1 thread, defaults; not a gated row). The state pass alone (`ProbeSteps = 0`) takes about 2 ms on the standard 2D large pile (4000 bodies), 4-20 ms on large-pile-3d and 3.5 ms on ragdoll-pile-3d. With the default 90 probe steps (a fork plus 90 steps of the world, which dominates) it takes 0.2 s on the 2D large pile, 0.3-0.7 s on large-pile-3d and 0.14-0.27 s on ragdoll-pile-3d, as built and after 120 settling steps; the 4960-body pyramid-3d-30 takes 4 s. Diagnosis allocates (findings, a copy of the world); it is a tool for authors and tests, not for the step loop.

## Catalogue

Thresholds are metres, seconds and kilograms. "Reach" is the body's farthest shape point from its centre of mass.

### OPD1001 JointsFighting
A group of dynamic bodies connected by joints (connected components over dynamic bodies, a static or kinematic end adds its rows to the dynamic body's group) locks more degrees of freedom than the bodies have: sum of locked rows of its joints `R` > `D × N` (`D` = 6 in 3D, 3 in 2D; `N` the bodies; Kutzbach counting). Locked rows per joint kind — 3D: fixed 6, point 3, cone 3, swing-twist 3, hinge 5, slider 5, wheel 4, path 2, distance 1, gear 1, rack-and-pinion 1, pulley 1, six-DOF the locked axes, others 0; 2D: weld 3, revolute 2, prismatic 2, path 2, wheel 1, distance 1, gear 1, rack-and-pinion 1, pulley 1, others 0.
- **Warning** when some point-like joint (3D fixed, point, hinge, cone, swing-twist; 2D revolute, weld) of the group has a persistent anchor gap (below); **Error** when that gap exceeds 2 m. Otherwise **Info**: over-constrained on paper (a loop of hinges with exactly consistent axes is).
- Joints of a flagged group are not reported again as OPD1012.
- Evidence: a box fixed to two posts with anchors a metre inconsistent holds a 0.5 m gap on both joints indefinitely (test); the 100 × 100 revolute lattice of the standard joint-grid scene locks 4N of 3N and is Info (it is consistent).

### OPD1002 JointMassRatio
A solving joint between two dynamic bodies with mass ratio `max/min` above 50: **Info** (> 50), **Warning** (> 600), **Error** (> 1000).
- Evidence: 50 is the limit of the per-island rule that puts "mild" joints on the contact-only schedule (docs/spec/world3d.md, round 33). A chain of 8 point joints with an end mass of R times the others, 240 steps at 1/60 (3D, then 2D), maximum deviation of a link's centre distance from its rest length and peak speed: R = 50: 0.074 m, 8 m/s (2D 0.065 m); 500: 0.49 m, 52 m/s (2D 0.42 m, 13 m/s); 1000: 2.6 m, 134 m/s (2D 1.4 m, 73 m/s); 1500: 5.0 m, 218 m/s; 2000: 6.9 m, 375 m/s (2D 5.0 m, 151 m/s); 10 000: 14 m, 911 m/s. The standard joint-chains-3d scene (536:1) passes its stability threshold (joint error ≤ 1.0 m). So 600 sits just above the measured-to-hold chain and 1000 is where the stretch passes the scene threshold by 2.5×.

### OPD1003 ContactMassRatio
Two dynamic bodies in contact (checked once, at the first state that has contacts) with mass ratio above 100: **Info**, **Warning** above 300, **Error** above 500.
- Evidence: a 1 m box of density R on a 1 m box of density 1 on a floor, 300 steps. 3D, final overlap of the lower box: R = 100: 0.036 m; 200: 0.053; 300: 0.087; **500: the heavy box falls through (overlap 1 m)**, as do 700 and 1000. 2D: 100: 0.024 m; 300: 0.072; 500: 0.12; 1000: 0.23; 10 000: 0.93.

### OPD1004 ImplausibleSize
A dynamic body with shapes whose thickness is below the speculative contact distance (`SpeculativeDistance`, default 0.02 m): **Warning** (both faces are inside contact range at once). A dynamic body whose reach exceeds 1000 m: **Info** (float32 resolves shape points that far out only to ~1e-4 m; contacts are tuned for smaller things). Metres are assumed (a length unit other than metres is not yet part of the world).

### OPD1005 ImplausibleSpeed
Largest motion seen for a dynamic body (state and observation) at or above 50% of `MaxLinearSpeed` (default 400 m/s): **Warning**; at or above 95%: **Error** (the engine clamps the speed at the limit, which breaks momentum). Evidence: the standard bullets scene peaks at 160 m/s.

### OPD1006 ImplausibleGravity
Gravity magnitude above 50 m/s²: **Warning** (a world authored in centimetres, millimetres or pixels). Non-zero below 0.1 m/s²: **Info**. Zero is fine.

### OPD1007 TunnellingRisk
For a dynamic body (not flagged bullet when continuous collision is on), the largest travel per step seen (motion × dt) exceeds its own thickness plus the thickness of the thinnest *other* body that it could pass: with continuous collision off, any body (static included): **Warning**; on, only non-static bodies (continuous collision sweeps against static shapes, but against moving ones only a bullet is swept): **Info**.

### OPD1008 NeverSleeps
- World-level **Info** when `SleepEnabled` is false and dynamic bodies exist.
- **Info** per island with a body flagged never-sleep (checked on the state).
- Observed (needs `SleepEnabled`, and an observed time of at least 3 × `TimeToSleep`): dynamic bodies awake at every observed step, grouped by island at the end (a group with a never-sleep body is left to the Info above). Each group is classed by its largest per-body mean motion in the last third (`late`) and middle third (`mid`), against the sleep threshold `t`:
  - touches or is joined to a kinematic body whose motion exceeds `t`: **Info** per group (the engine keeps such an island awake);
  - `late > 20 t`: one aggregate **Info** "still moving";
  - `late < 0.9 × mid`: one aggregate **Info** "still settling";
  - otherwise one aggregate **Warning**: nearly at rest but not slowing — jitter or creep.
- Evidence: 20 t = 1 m/s at the default threshold; the standard pyramid-100 scene is still settling after 3.5 s (Info), and a frictionless ball in a closed box at 0.3 m/s never slows (Warning).

### OPD1009 ShapesOverlapping
At the first state that has contacts (a world must have stepped once, so the probe's first step supplies them for a fresh world), a pair of bodies in contact whose stored separation is a penetration deeper than 0.05 m and deeper than the thinner dynamic body's half-thickness, with both dynamic bodies moving no faster than 1 m/s (a deep contact between fast bodies is an impact the solver is about to resolve): **Warning**; deeper than twice the half-thickness: **Error**.

### OPD1010 AnchorFarFromBody
A joint anchor on a dynamic body farther from its centre of mass than 8 × the body's reach + 0.05 m: **Warning**. The engine refuses 40 × reach + 0.05 m at creation (float32 conditioning of the anchor block); this fires at a fifth of that.

### OPD1011 AnchorsApart
A point-like joint (list under OPD1001) whose two world anchors start more than max(0.05 m, 0.5 × the smaller dynamic body's reach) apart: **Warning**. The first steps must close the gap with whatever speed it takes.

### OPD1012 JointNotConverging
A point-like joint, not in an OPD1001 group, whose anchor gap (world distance between the two anchors, read from the poses at every observed step, asleep or awake) has a last-third mean above 0.3 m and is not falling: **Warning**; above 2 m: **Error**. Evidence: the stability thresholds of the standard scenes (joint-grid 0.1, ragdoll pile 0.3, 2D chain 0.4, 3D 500:1 chain 1.0 as a maximum) — the settled joint-chains-3d scene's last-third mean gap stays below 0.3 m.

### OPD1013 PenetrationNotFalling
Per observed step the deepest contact is taken among *resting* contacts (every dynamic body of the pair moving no faster than 1 m/s) whose depth exceeds 0.1 m and the thinner body's half-thickness; the last-third mean of that depth (zero for steps with none) exceeds 0.1 m and is not falling: **Warning**; the mean depth over the half-thickness above 2: **Error**. Evidence: a 0.5 m box between a floor and a kinematic block descending at 0.4 m/s. The standard large-pile scenes' settled resting contacts stay under 0.8 half-thicknesses.

### OPD1014 MotorSaturated
A motor row of a joint is active with a speed error above 0.1 (m/s or rad/s) for at least 0.5 s of consecutive solved steps, and the error at the end is not below 0.9 × the error 0.25 s into that streak: **Warning**. A joint that falls asleep keeps its streak. Not reported when OPD1015 is.

### OPD1015 LimitPinned
A joint with a limit and a motor holds the limit row active while the motor row is active for 0.5 s of consecutive solved steps: **Info**.

### OPD1016 ProbeStopped
Stepping the copy threw. **Info**; the message names the exception.

## Limits

- Overlap and contact mass ratio read contacts, which a world that never stepped does not have; with `ProbeSteps = 0` they are found only in a world that has contacts.
- Joint motor and limit findings read joint rows, collected by the observation.
- The copy has no filter delegates, callbacks or custom-field bindings.
- Soft bodies, rods, fluids, characters and vehicles are not examined (their bodies and joints are, when they are rigid).
