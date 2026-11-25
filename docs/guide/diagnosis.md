# Finding out what is wrong with a world

A scene can be wrong without anything throwing: a joint between a feather and an anvil, two boxes created inside each other, a body that never falls asleep, a gravity written in centimetres. `world.Diagnose()` looks the world over and says so, in words an author can act on without looking at a screen: each finding has a stable code, a severity, the bodies and joints involved, the measured values, an explanation and a suggested change.

<!-- example: Guide/Diagnosis.cs -->
```csharp
DiagnosisReport report = world.Diagnose();               // examines this world and a copy of it stepped for 1.5 s; this world is not touched
foreach (Finding f in report.Findings)
{
    Console.WriteLine(f);                                // OPD1002 Error: The revolute joint connects bodies of ... Suggestion: ...
}
```

`Diagnose` reads the state you have (settings, sizes, masses, speeds, anchors, the contacts the world holds) and, unless you ask for `ProbeSteps = 0` or everything is asleep, steps a **copy** of the world for `ProbeSteps` steps (default 90 of 1/60 s) to see what needs time: whether joints settle, whether a motor gets to its speed, whether the bodies fall asleep. The world itself is never stepped or changed; its snapshot bytes and hash are the same afterwards. The copy has no event handlers, pair filter or custom-field callbacks, as with any `Fork()`.

## Reading a finding

- `Code` / `Id`: `DiagnosticCode.JointMassRatio` is `OPD1002`. Numbers are never reused.
- `Severity`: `Info` (unusual, often intended), `Warning` (very likely a mistake or a source of instability), `Error` (beyond what the engine is measured to handle). `report.HasErrors`, `report.Worst`.
- `Bodies`, `Joints`: handles, at most eight each, in ascending slot order. `world.GetBody(handle)` gives the `Body`.
- `Values`: the measured numbers, each with its unit and the limit it was judged against.
- `Explanation`, `Suggestion`: plain words.

<!-- example: Guide/Diagnosis.cs -->
```csharp
if (report.With(DiagnosticCode.JointMassRatio).Any(f => f.Severity == DiagnosticSeverity.Error)) { Console.WriteLine("lighten the heavy body or add substeps"); }
```

Per code at most `MaxFindingsPerCode` (default 16) findings are kept, the worst first; `report.Suppressed` counts the rest. The list, its order and every value are the same on every platform and thread count.

## Over a recording

An agent or a QA run often has a recording, not a world. `ReplayDiagnostics.Diagnose(stream, firstStep, lastStep)` plays it, examines the state at `firstStep` and watches each step to `lastStep`, and returns what the live form would have found over the same steps (the recorded time steps are used; the recorded hashes are checked as it plays).

<!-- example: Guide/Diagnosis.cs -->
```csharp
recording.Position = 0;
DiagnosisReport window = ReplayDiagnostics.Diagnose(recording, firstStep: 0, lastStep: 60);
```

## What it finds

The catalogue is in [the spec](../spec/diagnosis.md): sizes, speeds and gravity that do not fit metres; heavy-on-light mass ratios across a joint or in contact; joints that fight each other or never reach their anchors; anchors far from their bodies or apart at creation; shapes created inside each other; contact penetration that does not go down; bodies at risk of tunnelling; islands that never sleep and why; motors that cannot reach their speed and limits held against a motor. Every threshold there comes from a measurement of the engine, with the scene it was measured on.
