using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Diagnosis;

internal sealed partial class Diagnoser
{
    // Per body slot. A slot whose generation changed is a different body, so its numbers start again.
    private int[] _bGen = Array.Empty<int>(), _bSeen = Array.Empty<int>(), _bAwake = Array.Empty<int>(), _bLateN = Array.Empty<int>(), _bEarlyN = Array.Empty<int>();
    private double[] _bMaxMotion = Array.Empty<double>(), _bMaxTravel = Array.Empty<double>(), _bLateMotion = Array.Empty<double>(), _bEarlyMotion = Array.Empty<double>();

    // Per constraint slot (joints). Three-phase arrays hold the first, middle and last third of the observed steps.
    // Streaks of consecutive solved steps: the limit held against the motor, and the motor active with a speed error that is not closing. A joint that falls asleep keeps its streak.
    private int[] _jPin = Array.Empty<int>(), _jSat = Array.Empty<int>();
    private double[] _jSatMark = Array.Empty<double>(), _jSatLast = Array.Empty<double>();
    private int[] _jGen = Array.Empty<int>(), _jSolved = Array.Empty<int>();
    private int[] _jGapN = Array.Empty<int>();
    private double[] _jGapSum = Array.Empty<double>();

    // Deepest contact of each step.
    private readonly double[] _penDepth = new double[3], _penRel = new double[3];
    private readonly int[] _penN = new int[3];
    private int _penA = -1, _penB = -1;
    private double _penLastDepth, _penLastRel;
    private double _time;

    private void EnsureBodies(int n)
    {
        if (_bGen.Length >= n) { return; }
        int size = System.Math.Max(n, System.Math.Max(16, _bGen.Length * 2));
        Array.Resize(ref _bGen, size); Array.Resize(ref _bSeen, size); Array.Resize(ref _bAwake, size); Array.Resize(ref _bLateN, size); Array.Resize(ref _bEarlyN, size); Array.Resize(ref _bEarlyMotion, size);
        Array.Resize(ref _bMaxMotion, size); Array.Resize(ref _bMaxTravel, size); Array.Resize(ref _bLateMotion, size);
    }

    private void EnsureJoints(int n)
    {
        if (_jGen.Length >= n) { return; }
        int size = System.Math.Max(n, System.Math.Max(16, _jGen.Length * 2));
        Array.Resize(ref _jGen, size); Array.Resize(ref _jPin, size); Array.Resize(ref _jSat, size); Array.Resize(ref _jSatMark, size); Array.Resize(ref _jSatLast, size); Array.Resize(ref _jSolved, size);
        Array.Resize(ref _jGapN, size * 3); Array.Resize(ref _jGapSum, size * 3);
    }

    private void TrackBody(DiagnosisHost h, int s)
    {
        int gen = h.Core.Bodies.Handles.GenerationOf(s);
        if (_bGen[s] == gen) { return; }
        _bGen[s] = gen; _bSeen[s] = 0; _bAwake[s] = 0; _bLateN[s] = 0; _bEarlyN[s] = 0; _bEarlyMotion[s] = 0;
        _bMaxMotion[s] = 0; _bMaxTravel[s] = 0; _bLateMotion[s] = 0;
    }

    private void TrackJoint(DiagnosisHost h, int c)
    {
        int gen = h.Core.Constraints.Handles.GenerationOf(c);
        if (_jGen[c] == gen) { return; }
        _jGen[c] = gen; _jPin[c] = 0; _jSat[c] = 0; _jSatMark[c] = 0; _jSatLast[c] = 0; _jSolved[c] = 0;
        for (int p = 0; p < 3; p++) { _jGapN[3 * c + p] = 0; _jGapSum[3 * c + p] = 0; }
    }

    /// <summary>Sets up an observation of <paramref name="totalSteps"/> steps; the thirds of the window are cut from this number.</summary>
    public void BeginObservation(DiagnosisHost h, int totalSteps)
    {
        _total = totalSteps; _observed = 0; _time = 0;
        EnsureBodies(h.Core.Bodies.HighWater);
        EnsureJoints(h.Core.Constraints.HighWater);
    }

    /// <summary>Reads the state after one more step of <paramref name="dt"/> seconds.</summary>
    public void Observe(DiagnosisHost h, float dt)
    {
        var core = h.Core;
        var bodies = core.Bodies;
        var cs = core.Constraints;
        _dt = dt;
        int step = _observed++;
        _time += dt;
        _lastStep = h.StepCount;
        int phase = Phase(step, _total);
        EnsureBodies(bodies.HighWater);
        EnsureJoints(cs.HighWater);

        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (!bodies.IsLive(s) || bodies.TypeOf(s) != BodyType.Dynamic) { continue; }
            TrackBody(h, s);
            _bSeen[s]++;
            if (!bodies.IsAwake(s)) { continue; }
            _bAwake[s]++;
            double motion = h.Motion(s);
            if (motion > _bMaxMotion[s]) { _bMaxMotion[s] = motion; }
            double travel = motion * dt;
            if (travel > _bMaxTravel[s]) { _bMaxTravel[s] = travel; }
            if (phase == 2) { _bLateMotion[s] += motion; _bLateN[s]++; }
            else if (phase == 1) { _bEarlyMotion[s] += motion; _bEarlyN[s]++; }
        }

        bool rows = core.Options.DetailedInspection;
        double deepRel = 0, deepDepth = 0;
        int deepA = -1, deepB = -1;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c)) { continue; }
            var kind = cs.KindOf(c);
            if (kind == ConstraintKind.Joint) { ObserveJoint(h, c, phase, rows); continue; }
            if (kind != ConstraintKind.Contact || cs.PointCount[c] == 0) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            bool wa = bodies.TypeOf(a) == BodyType.Dynamic && bodies.IsAwake(a), wb = bodies.TypeOf(b) == BodyType.Dynamic && bodies.IsAwake(b);
            if (!wa && !wb) { continue; }
            double depth = Depth(core, c);
            if (!(depth > 0)) { continue; }
            if (depth <= PenetrationFloor) { continue; }
            double thin = HalfThin(h, a, b);
            if (!(thin > 0)) { continue; }
            double rel = depth / thin;
            if (rel <= PenetrationRel) { continue; }
            // Only resting contacts count: a body still moving at an impact is about to be pushed out.
            if ((bodies.TypeOf(a) == BodyType.Dynamic && h.Motion(a) > SlowMotion) || (bodies.TypeOf(b) == BodyType.Dynamic && h.Motion(b) > SlowMotion)) { continue; }
            if (rel > deepRel) { deepRel = rel; deepDepth = depth; deepA = System.Math.Min(a, b); deepB = System.Math.Max(a, b); }
        }

        if (deepA >= 0)
        {
            _penDepth[phase] += deepDepth; _penRel[phase] += deepRel; _penN[phase]++;
            _penA = deepA; _penB = deepB; _penLastDepth = deepDepth; _penLastRel = deepRel;
        }
        else { _penN[phase]++; }

        if (!_contactScanDone && cs.Count > 0) { ScanContactsOnce(h); }
    }

    private void ObserveJoint(DiagnosisHost h, int c, int phase, bool rows)
    {
        var core = h.Core;
        TrackJoint(h, c);
        var f = h.Joint(c);
        if (!f.Solves) { return; }
        // The anchor gap is read from the poses, so a joint that has gone to sleep at a stretched pose still counts.
        if (f.Coincident) { _jGapSum[3 * c + phase] += f.Gap; _jGapN[3 * c + phase]++; }
        bool solved = false, motor = false, limit = false;
        double motorError = 0;
        if (rows)
        {
            try
            {
                var rowsView = core.JointReadback.Inspect(core.HandleOfJoint(c)).Rows;
                foreach (var r in rowsView)
                {
                    if ((r.Flags & JointRowBits.Enabled) == 0) { continue; }
                    solved = true;
                    if ((r.Flags & (JointRowBits.Motor | JointRowBits.Active)) == (JointRowBits.Motor | JointRowBits.Active))
                    {
                        motor = true;
                        motorError = System.Math.Max(motorError, System.Math.Abs((double)r.Error));
                    }

                    if ((r.Flags & (JointRowBits.Limit | JointRowBits.Active)) == (JointRowBits.Limit | JointRowBits.Active)) { limit = true; }
                }
            }
            catch (DetailedInspectionDisabledException) { rows = false; }
            catch (PhysicsException) { solved = false; }
        }

        if (!solved) { return; }
        _jSolved[c]++;
        _jPin[c] = limit && motor ? _jPin[c] + 1 : 0;
        if (motor && motorError > MotorError)
        {
            _jSat[c]++;
            if (_jSat[c] == (int)System.Math.Ceiling(0.5 * PinnedSeconds / _dt)) { _jSatMark[c] = motorError; }
            _jSatLast[c] = motorError;
        }
        else { _jSat[c] = 0; }
    }
}
