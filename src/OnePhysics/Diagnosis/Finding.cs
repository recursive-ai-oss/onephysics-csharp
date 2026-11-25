using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace RecursiveAI.OnePhysics.Diagnosis;

/// <summary>How serious a finding is. Info: unusual, often intended. Warning: very likely a mistake or a source of instability. Error: the simulation is outside what the engine is measured to handle.</summary>
public enum DiagnosticSeverity
{
    /// <summary>Unusual, often intended; nothing is wrong yet.</summary>
    Info = 0,

    /// <summary>Very likely a mistake, or a source of visible instability.</summary>
    Warning = 1,

    /// <summary>Beyond what the engine is measured to handle: expect wrong or exploding results.</summary>
    Error = 2,
}

/// <summary>One measured value behind a finding: a name, the value, its unit and the threshold it was judged against (NaN when the finding has none).</summary>
/// <param name="Name">What was measured, for example <c>massRatio</c>.</param>
/// <param name="Value">The measured value.</param>
/// <param name="Unit">The unit: <c>m</c>, <c>m/s</c>, <c>rad</c>, <c>s</c>, <c>kg</c>, <c>ratio</c> or <c>count</c>.</param>
/// <param name="Threshold">The limit the value was compared against, or NaN.</param>
public readonly record struct FindingValue(string Name, double Value, string Unit, double Threshold = double.NaN)
{
    /// <summary>Text such as <c>massRatio = 120 ratio (limit 50)</c>.</summary>
    public override string ToString()
    {
        var text = Name + " = " + Value.ToString("G6", CultureInfo.InvariantCulture) + " " + Unit;
        return double.IsNaN(Threshold) ? text : text + " (limit " + Threshold.ToString("G6", CultureInfo.InvariantCulture) + ")";
    }
}

/// <summary>
/// One thing a diagnosis found wrong, or worth knowing, about a world: a stable code, a severity, the bodies and joints involved, the measured values, an explanation in plain words and a suggested change.
/// Findings are immutable. A world's findings list, its order and every value are the same on every platform.
/// </summary>
public sealed class Finding
{
    private readonly BodyHandle[] _bodies;
    private readonly JointHandle[] _joints;
    private readonly FindingValue[] _values;

    internal Finding(DiagnosticCode code, DiagnosticSeverity severity, BodyHandle[] bodies, JointHandle[] joints, FindingValue[] values, string explanation, string suggestion)
    {
        Code = code; Severity = severity; _bodies = bodies; _joints = joints; _values = values; Explanation = explanation; Suggestion = suggestion;
    }

    /// <summary>The stable code.</summary>
    public DiagnosticCode Code { get; }

    /// <summary>The code as text, <c>OPD</c> and the number, for logs and filters.</summary>
    public string Id => "OPD" + ((int)Code).ToString(CultureInfo.InvariantCulture);

    /// <summary>How serious it is.</summary>
    public DiagnosticSeverity Severity { get; }

    /// <summary>The bodies involved, at most eight, in ascending slot order. Handles belong to the world that was diagnosed (to the recorded world's id for a replay window).</summary>
    public IReadOnlyList<BodyHandle> Bodies => _bodies;

    /// <summary>The joints involved, at most eight, in ascending slot order.</summary>
    public IReadOnlyList<JointHandle> Joints => _joints;

    /// <summary>The measured values, in a fixed order per code.</summary>
    public IReadOnlyList<FindingValue> Values => _values;

    /// <summary>What was found and why it matters, in plain words.</summary>
    public string Explanation { get; }

    /// <summary>A concrete change to try.</summary>
    public string Suggestion { get; }

    /// <summary>One line: id, severity, explanation, values and suggestion.</summary>
    public override string ToString()
    {
        var sb = new StringBuilder();
        sb.Append(Id).Append(' ').Append(Severity).Append(": ").Append(Explanation);
        if (_values.Length > 0)
        {
            sb.Append(" [");
            for (int i = 0; i < _values.Length; i++) { if (i > 0) { sb.Append("; "); } sb.Append(_values[i].ToString()); }
            sb.Append(']');
        }

        sb.Append(" Suggestion: ").Append(Suggestion);
        return sb.ToString();
    }
}

/// <summary>
/// The result of a diagnosis: the findings in a fixed order (by code, then most serious first, then by the entities' slots), how many steps were observed and how many findings were left out by the per-code limit.
/// </summary>
public sealed class DiagnosisReport
{
    internal DiagnosisReport(Finding[] findings, int stepsObserved, long firstStep, long lastStep, int suppressed)
    {
        Findings = findings; StepsObserved = stepsObserved; FirstStep = firstStep; LastStep = lastStep; Suppressed = suppressed;
    }

    /// <summary>The findings, in a fixed order: ascending code, then descending severity, then descending measure, then ascending entity slot.</summary>
    public IReadOnlyList<Finding> Findings { get; }

    /// <summary>How many steps were simulated and observed (zero when only the current state was examined).</summary>
    public int StepsObserved { get; }

    /// <summary>The world's step count when the examined state began.</summary>
    public long FirstStep { get; }

    /// <summary>The world's step count after the last observed step; equals <see cref="FirstStep"/> when no step was observed.</summary>
    public long LastStep { get; }

    /// <summary>How many findings were dropped because a code had more than <see cref="DiagnoseOptions.MaxFindingsPerCode"/>; the worst ones are kept.</summary>
    public int Suppressed { get; }

    /// <summary>The most serious severity among the findings, or null when there are none.</summary>
    public DiagnosticSeverity? Worst
    {
        get
        {
            DiagnosticSeverity? worst = null;
            foreach (var f in Findings) { if (worst == null || f.Severity > worst) { worst = f.Severity; } }
            return worst;
        }
    }

    /// <summary>True when at least one finding has Error severity.</summary>
    public bool HasErrors => Worst == DiagnosticSeverity.Error;

    /// <summary>True when no finding was made.</summary>
    public bool IsClean => Findings.Count == 0;

    /// <summary>Finds the findings with a code.</summary>
    public IEnumerable<Finding> With(DiagnosticCode code)
    {
        foreach (var f in Findings) { if (f.Code == code) { yield return f; } }
    }

    /// <summary>True when any finding has the code.</summary>
    public bool Has(DiagnosticCode code)
    {
        foreach (var f in Findings) { if (f.Code == code) { return true; } }
        return false;
    }

    /// <summary>One finding per line, with a header line.</summary>
    public override string ToString()
    {
        var sb = new StringBuilder();
        sb.Append(Findings.Count.ToString(CultureInfo.InvariantCulture)).Append(" finding(s) over ").Append(StepsObserved.ToString(CultureInfo.InvariantCulture)).Append(" observed step(s)");
        if (Suppressed > 0) { sb.Append(", ").Append(Suppressed.ToString(CultureInfo.InvariantCulture)).Append(" more left out"); }
        foreach (var f in Findings) { sb.Append('\n').Append(f.ToString()); }
        return sb.ToString();
    }
}

/// <summary>Settings of a diagnosis. The defaults suit a world of metre-sized things stepped at 60 Hz.</summary>
public sealed class DiagnoseOptions
{
    /// <summary>
    /// How many steps of a copy of the world to simulate and watch, 0..100000. Default 90 (1.5 s at 60 Hz, three times the default time to sleep). 0 examines only the current state: it finds
    /// sizes, speeds, mass ratios, anchors and settings, but nothing that needs time (convergence, motors and limits, bodies that never sleep, overlap before the first step). The world itself is never stepped.
    /// </summary>
    public int ProbeSteps { get; set; } = 90;

    /// <summary>The time step of the probe, seconds, finite and positive. Default 1/60. Use the step your game uses.</summary>
    public float Dt { get; set; } = 1f / 60f;

    /// <summary>The most findings kept per code, 1..1000. The worst are kept; <see cref="DiagnosisReport.Suppressed"/> counts the rest. Default 16.</summary>
    public int MaxFindingsPerCode { get; set; } = 16;

    internal void Validate()
    {
        if (ProbeSteps < 0 || ProbeSteps > 100000) { throw new ArgumentOutOfRangeException(nameof(ProbeSteps), ProbeSteps, "ProbeSteps must be within 0..100000."); }
        if (!(Dt > 0f) || !float.IsFinite(Dt)) { throw new ArgumentOutOfRangeException(nameof(Dt), Dt, "Dt must be finite and positive."); }
        if (MaxFindingsPerCode < 1 || MaxFindingsPerCode > 1000) { throw new ArgumentOutOfRangeException(nameof(MaxFindingsPerCode), MaxFindingsPerCode, "MaxFindingsPerCode must be within 1..1000."); }
    }
}
