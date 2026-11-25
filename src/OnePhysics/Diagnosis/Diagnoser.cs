using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Diagnosis;

/// <summary>
/// The diagnosis of one world, over its current state and an observed run of steps. The same object serves a live world (state read from the world, steps taken in a copy) and a replay window
/// (state read from the playback world after each step). Everything it measures is double arithmetic over float32 state with exact operations, so the findings are the same on every platform.
/// Thresholds and the evidence behind them are in docs/spec/diagnosis.md.
/// </summary>
internal sealed partial class Diagnoser
{
    // Thresholds. Each is explained, with its evidence, in docs/spec/diagnosis.md.
    internal const double GapWarn = 0.3, GapError = 2.0;                        // metres of anchor separation, mean over the last third
    internal const double Falling = 0.9;                                          // a last-third mean at least this fraction of the middle-third mean has stopped falling
    internal const double AnchorFarReach = 8.0, AnchorFarSlack = 0.05;            // reaches (the engine refuses 40)
    internal const double AnchorsApartReach = 0.5, AnchorsApartFloor = 0.05;      // reach fraction, metres
    internal const double JointRatioInfo = 50.0, JointRatioWarn = 600.0, JointRatioError = 1000.0;
    internal const double ContactRatioInfo = 100.0, ContactRatioWarn = 300.0, ContactRatioError = 500.0;
    internal const double SpeedWarn = 0.5, SpeedError = 0.95;                      // fractions of the speed limit
    internal const double GravityHigh = 50.0, GravityLow = 0.1;                    // m/s^2
    internal const double HugeBody = 1000.0;                                       // metres of reach
    internal const double OverlapRel = 1.0, OverlapFloor = 0.05;                   // of the thinnest half-width, metres
    internal const double PenetrationRel = 1.0, PenetrationFloor = 0.1;
    internal const double SlowMotion = 1.0;                                        // m/s: a contact between bodies slower than this is resting, not an impact
    internal const double JitterBand = 20.0;                                       // times the sleep threshold
    internal const double MotorError = 0.1, PinnedFraction = 0.9;                  // m/s or rad/s
    internal const double PinnedSeconds = 0.5;                                     // a limit held against its motor this long is pinned
    internal const double SleepWindowFactor = 3.0;                                 // times TimeToSleep
    internal const int MinTrendSteps = 6;
    internal const int MaxEntities = 8;

    private sealed class Candidate
    {
        public DiagnosticCode Code;
        public DiagnosticSeverity Severity;
        public double Metric;
        public int Key;
        public int Order;
        public BodyHandle[] Bodies = Array.Empty<BodyHandle>();
        public JointHandle[] Joints = Array.Empty<JointHandle>();
        public FindingValue[] Values = Array.Empty<FindingValue>();
        public string Explanation = "";
        public string Suggestion = "";
    }

    private readonly DiagnoseOptions _o;
    private readonly List<Candidate> _cands = new();
    private WorldCore _handles = null!;            // the core whose id the handles carry
    private float _dt;
    private long _firstStep, _lastStep;
    private int _total, _observed;
    private bool _contactScanDone;
    private float _unitsPerMetre = 1f;

    /// <summary>Names the world's length unit and what a world unit is in metres, for a world that is not in metres: every value in a finding is in metres, whatever the unit.</summary>
    private string UnitNote() => _unitsPerMetre == 1f ? "" : " The world's length unit is " + N(1.0 / _unitsPerMetre) + " m per world unit (one world unit is " + N(1.0 / _unitsPerMetre) + " m, and " + N(_unitsPerMetre) + " world units make a metre); every length, speed and acceleration in this finding is given in metres.";

    public Diagnoser(DiagnoseOptions options) { _o = options; _dt = options.Dt; }

    /// <summary>The step used for travel estimates before a step has been observed.</summary>
    public float Dt { get => _dt; set => _dt = value; }

    private static string N(double v) => v.ToString("G4", CultureInfo.InvariantCulture);

    private static FindingValue V(string name, double value, string unit, double threshold = double.NaN) => new(name, value, unit, threshold);

    private void Add(DiagnosticCode code, DiagnosticSeverity severity, double metric, ReadOnlySpan<int> bodies, ReadOnlySpan<int> joints, FindingValue[] values, string explanation, string suggestion)
    {
        var c = new Candidate { Code = code, Severity = severity, Metric = metric, Order = _cands.Count, Values = values, Explanation = explanation, Suggestion = suggestion };
        int nb = Math.Min(bodies.Length, MaxEntities), nj = Math.Min(joints.Length, MaxEntities);
        c.Bodies = new BodyHandle[nb]; c.Joints = new JointHandle[nj];
        for (int i = 0; i < nb; i++) { c.Bodies[i] = _handles.HandleOfBody(bodies[i]); }
        for (int i = 0; i < nj; i++) { c.Joints[i] = _handles.HandleOfJoint(joints[i]); }
        c.Key = nb > 0 ? bodies[0] : nj > 0 ? joints[0] : -1;
        _cands.Add(c);
    }

    /// <summary>Orders the candidates, keeps the worst per code and builds the report.</summary>
    public DiagnosisReport Report()
    {
        _cands.Sort(static (x, y) =>
        {
            int c = ((int)x.Code).CompareTo((int)y.Code);
            if (c != 0) { return c; }
            c = y.Severity.CompareTo(x.Severity);
            if (c != 0) { return c; }
            c = y.Metric.CompareTo(x.Metric);
            if (c != 0) { return c; }
            c = x.Key.CompareTo(y.Key);
            return c != 0 ? c : x.Order.CompareTo(y.Order);
        });
        var list = new List<Finding>();
        int suppressed = 0, run = 0;
        DiagnosticCode last = 0;
        foreach (var c in _cands)
        {
            run = c.Code == last ? run + 1 : 1;
            last = c.Code;
            if (run > _o.MaxFindingsPerCode) { suppressed++; continue; }
            list.Add(new Finding(c.Code, c.Severity, c.Bodies, c.Joints, c.Values, c.Explanation, c.Suggestion));
        }

        return new DiagnosisReport(list.ToArray(), _observed, _firstStep, _lastStep, suppressed);
    }

    private static int Phase(int step, int total) => total <= 0 ? 0 : (int)System.Math.Min(2L, 3L * step / total);
}
