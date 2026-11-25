using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Diagnosis;

internal sealed partial class Diagnoser
{
    private static double Mean(double sum, int n) => n > 0 ? sum / n : 0;

    /// <summary>Judges everything that needed time. <paramref name="h"/> is the world after the last observed step (or the starting world when nothing was observed).</summary>
    public void Finish(DiagnosisHost h)
    {
        _handles = _handles ?? h.Core;
        EnsureBodies(h.Core.Bodies.HighWater);
        EnsureJoints(h.Core.Constraints.HighWater);
        if (_observed > 0) { _lastStep = h.StepCount; }
        FinishSpeed(h);
        FinishJoints(h);
        FinishPenetration();
        FinishSleep(h);
        FinishTunnelling(h);
    }

    /// <summary>Records that stepping the probe copy failed.</summary>
    public void ProbeStopped(Exception e)
    {
        Add(DiagnosticCode.ProbeStopped, DiagnosticSeverity.Info, _observed, default, default,
            [V("stepsObserved", _observed, "count")],
            "The probe stopped after " + _observed + " step(s) because stepping a copy of the world threw " + e.GetType().Name + ": " + e.Message,
            "Fix what the exception names; a copy of the world has no event handlers, filters or custom-field callbacks, so a world that needs them to step may fail only in the copy.");
    }

    private void FinishSpeed(DiagnosisHost h)
    {
        var bodies = h.Core.Bodies;
        double cap = h.MaxLinearSpeed;
        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (!bodies.IsLive(s) || bodies.TypeOf(s) != BodyType.Dynamic || _bGen[s] != bodies.Handles.GenerationOf(s)) { continue; }
            double m = _bMaxMotion[s];
            if (!(m >= SpeedWarn * cap)) { continue; }
            var sev = m >= SpeedError * cap ? DiagnosticSeverity.Error : DiagnosticSeverity.Warning;
            Add(DiagnosticCode.ImplausibleSpeed, sev, m / cap, [s], default,
                [V("speed", m, "m/s", (sev == DiagnosticSeverity.Error ? SpeedError : SpeedWarn) * cap), V("speedLimit", cap, "m/s")],
                "Body reaches " + N(m) + " m/s (linear speed plus the speed of its farthest point from spinning), " + N(m / cap * 100) + "% of the engine's " + N(cap) + " m/s limit. At the limit the engine clamps the speed, which breaks momentum and the stability of anything it hits.",
                "Check the units: a speed beyond a few hundred metres per second in a metre-scaled world is usually a unit mistake or an impulse far too large. Lower the force or impulse that gives it that speed.");
        }
    }

    private void FinishJoints(DiagnosisHost h)
    {
        var core = h.Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        int D = h.Dimension == 3 ? 6 : 3;
        bool trend = _observed >= MinTrendSteps;

        // Joint graph over dynamic bodies: union-find with the lowest slot as the root, so the components come out in a fixed order.
        var parent = new int[bodies.HighWater];
        for (int i = 0; i < parent.Length; i++) { parent[i] = i; }
        int Find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
        var facts = new Dictionary<int, JointFacts>();
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint) { continue; }
            var f = h.Joint(c);
            if (!f.Solves) { continue; }
            facts[c] = f;
            int a = cs.BodyA[c], b = cs.BodyB[c];
            if (bodies.TypeOf(a) == BodyType.Dynamic && bodies.TypeOf(b) == BodyType.Dynamic)
            {
                int ra = Find(a), rb = Find(b);
                if (ra != rb) { if (ra < rb) { parent[rb] = ra; } else { parent[ra] = rb; } }
            }
        }

        var compBodies = new SortedDictionary<int, List<int>>();
        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (bodies.IsLive(s) && bodies.TypeOf(s) == BodyType.Dynamic) { int r = Find(s); if (!compBodies.TryGetValue(r, out var l)) { compBodies[r] = l = new List<int>(); } l.Add(s); }
        }

        var compRows = new Dictionary<int, int>();
        var compJoints = new Dictionary<int, List<int>>();
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!facts.TryGetValue(c, out var f)) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            int anchor = bodies.TypeOf(a) == BodyType.Dynamic ? a : bodies.TypeOf(b) == BodyType.Dynamic ? b : -1;
            if (anchor < 0) { continue; }
            int root = Find(anchor);
            compRows[root] = (compRows.TryGetValue(root, out int r0) ? r0 : 0) + f.LockedRows;
            if (!compJoints.TryGetValue(root, out var jl)) { compJoints[root] = jl = new List<int>(); }
            jl.Add(c);
        }

        var fighting = new HashSet<int>();
        foreach (var (root, list) in compBodies)
        {
            if (!compRows.TryGetValue(root, out int rows)) { continue; }
            int n = list.Count;
            if (rows <= D * n) { continue; }
            var joints = compJoints[root];
            double worst = 0;
            int worstJoint = -1;
            foreach (int c in joints)
            {
                if (_jGen[c] != cs.Handles.GenerationOf(c)) { continue; }
                double g0 = Mean(_jGapSum[3 * c + 1], _jGapN[3 * c + 1]), g2 = Mean(_jGapSum[3 * c + 2], _jGapN[3 * c + 2]);
                if (trend && _jGapN[3 * c + 2] > 0 && g2 > GapWarn && g2 >= Falling * g0 && g2 > worst) { worst = g2; worstJoint = c; }
            }

            var js = joints.ToArray();
            var bs = list.ToArray();
            int excess = rows - D * n;
            if (worstJoint >= 0)
            {
                foreach (int c in joints) { fighting.Add(c); }
                var sev = worst > GapError ? DiagnosticSeverity.Error : DiagnosticSeverity.Warning;
                Add(DiagnosticCode.JointsFighting, sev, worst, bs, js,
                    [V("lockedRows", rows, "count"), V("freedomOfBodies", D * n, "count"), V("excessRows", excess, "count"), V("anchorGap", worst, "m", GapWarn)],
                    "A group of " + n + " bodies joined by " + joints.Count + " joints locks " + rows + " degrees of freedom but the bodies have only " + D * n + ", so the joints pull against each other: the worst anchor gap stays at " + N(worst) + " m and does not shrink over " + N(_time) + " s.",
                    "Remove a joint from the loop, replace a locking joint with one that leaves a degree of freedom free (a hinge for a fixed joint, a point joint for a hinge), or make the loop's anchors consistent so one set of positions satisfies every joint.");
            }
            else
            {
                Add(DiagnosticCode.JointsFighting, DiagnosticSeverity.Info, excess, bs, js,
                    [V("lockedRows", rows, "count"), V("freedomOfBodies", D * n, "count"), V("excessRows", excess, "count")],
                    "A group of " + n + " bodies joined by " + joints.Count + " joints locks " + rows + " degrees of freedom but the bodies have only " + D * n + ". That is over-constrained in general; it works only when the anchors and axes are exactly consistent" + (trend ? " (they were, over the observed time)." : " (no time was observed, so this was not checked)."),
                    "If the loop is meant to move, check that its joint axes are parallel and anchors consistent, or loosen one joint.");
            }
        }

        // Joints that never reached their anchors, outside any over-constrained group.
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!facts.TryGetValue(c, out var f) || _jGen[c] != cs.Handles.GenerationOf(c) || fighting.Contains(c)) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            double g0 = Mean(_jGapSum[3 * c + 1], _jGapN[3 * c + 1]), g2 = Mean(_jGapSum[3 * c + 2], _jGapN[3 * c + 2]);
            if (f.Coincident && trend && _jGapN[3 * c + 2] > 0 && g2 > GapWarn && g2 >= Falling * g0)
            {
                var sev = g2 > GapError ? DiagnosticSeverity.Error : DiagnosticSeverity.Warning;
                double ratio = 0;
                if (bodies.TypeOf(a) == BodyType.Dynamic && bodies.TypeOf(b) == BodyType.Dynamic && h.Mass(a) > 0 && h.Mass(b) > 0) { ratio = System.Math.Max(h.Mass(a), h.Mass(b)) / System.Math.Min(h.Mass(a), h.Mass(b)); }
                Add(DiagnosticCode.JointNotConverging, sev, g2, [System.Math.Min(a, b), System.Math.Max(a, b)], [c],
                    [V("anchorGap", g2, "m", GapWarn), V("middleThirdGap", g0, "m"), V("massRatio", ratio, "ratio")],
                    "The " + f.Type + " joint's anchors stay " + N(g2) + " m apart over the last third of " + N(_time) + " s (" + N(g0) + " m in the middle third): the solve is not pulling them together." + (ratio > JointRatioInfo ? " The bodies' mass ratio is " + N(ratio) + ":1." : ""),
                    "Add substeps or velocity iterations for this world, reduce the mass ratio or the forces on the joint, or check that a motor or another joint is not holding the anchors apart.");
            }

            if (_jGen[c] != cs.Handles.GenerationOf(c)) { continue; }
            if (f.LimitOn && f.MotorOn && _jPin[c] * (double)_dt >= PinnedSeconds)
            {
                double held = _jPin[c] * (double)_dt;
                Add(DiagnosticCode.LimitPinned, DiagnosticSeverity.Info, held, [System.Math.Min(a, b), System.Math.Max(a, b)], [c],
                    [V("heldSeconds", held, "s", PinnedSeconds)],
                    "The " + f.Type + " joint's limit has been held in contact for " + N(held) + " s while its motor drives into it. The motor pushes against the stop with everything it has and the joint cannot move.",
                    "Stop the motor when the joint reaches the limit, or move the limit or the motor's target so the joint has room.");
                continue;
            }

            double sat = _jSat[c] * (double)_dt;
            if (f.MotorOn && sat >= PinnedSeconds && _jSatLast[c] >= Falling * _jSatMark[c])
            {
                Add(DiagnosticCode.MotorSaturated, DiagnosticSeverity.Warning, _jSatLast[c], [System.Math.Min(a, b), System.Math.Max(a, b)], [c],
                    [V("speedError", _jSatLast[c], "m/s or rad/s", MotorError), V("motorActiveSeconds", sat, "s", PinnedSeconds)],
                    "The " + f.Type + " joint's motor has been active for " + N(sat) + " s and stays " + N(_jSatLast[c]) + " (m/s or rad/s) short of its target speed, with no sign of closing: it works at its maximum force or torque and cannot get there.",
                    "Raise the motor's maximum force or torque, lower its target speed, or reduce the load (mass, friction, the other joints holding the body).");
            }
        }
    }

    private void FinishPenetration()
    {
        if (_observed < MinTrendSteps || _penN[2] == 0 || _penA < 0) { return; }
        double m0 = Mean(_penDepth[1], _penN[1]), m2 = Mean(_penDepth[2], _penN[2]), r2 = Mean(_penRel[2], _penN[2]);
        if (!(m2 > PenetrationFloor && r2 > PenetrationRel && m2 >= Falling * m0)) { return; }
        var sev = r2 > 2.0 ? DiagnosticSeverity.Error : DiagnosticSeverity.Warning;
        Add(DiagnosticCode.PenetrationNotFalling, sev, r2, [_penA, _penB], default,
            [V("depth", m2, "m", PenetrationFloor), V("middleThirdDepth", m0, "m"), V("depthOverHalfThickness", r2, "ratio", PenetrationRel)],
            "The deepest contact stays " + N(m2) + " m deep (" + N(r2) + " times the half-thickness of the thinner body) over the last third of " + N(_time) + " s, against " + N(m0) + " m in the middle third: the contact solve is not pushing the bodies apart.",
            "Look for something pressing the bodies together that the solver cannot move: a kinematic body or a motor crushing a body against a wall, a stack too heavy for its bottom body, or too little substepping.");
    }

    private void FinishSleep(DiagnosisHost h)
    {
        var core = h.Core;
        var bodies = core.Bodies;
        if (_observed == 0 || !h.SleepEnabled || _time < SleepWindowFactor * h.TimeToSleep) { return; }
        double threshold = h.SleepThreshold;
        var awake = new List<(int Island, int Slot)>();
        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (!bodies.IsLive(s) || bodies.TypeOf(s) != BodyType.Dynamic || _bGen[s] != bodies.Handles.GenerationOf(s)) { continue; }
            if (_bSeen[s] == _observed && _bAwake[s] == _observed) { awake.Add((bodies.Island[s], s)); }
        }

        awake.Sort(static (a, b) => a.Island != b.Island ? a.Island.CompareTo(b.Island) : a.Slot.CompareTo(b.Slot));
        var moving = new List<int>(); var settling = new List<int>(); var jitter = new List<int>();
        int movingGroups = 0, settlingGroups = 0, jitterGroups = 0;
        double movingLate = 0, settlingLate = 0, jitterLate = 0;
        int i = 0;
        while (i < awake.Count)
        {
            int j = i;
            while (j < awake.Count && awake[j].Island == awake[i].Island) { j++; }
            var slots = new int[j - i];
            bool flagged = false, driven = false;
            double late = 0, early = 0;
            for (int k = i; k < j; k++)
            {
                int s = awake[k].Slot;
                slots[k - i] = s;
                if ((bodies.Flags[s] & (byte)BodyBits.NoSleep) != 0) { flagged = true; }
                late = System.Math.Max(late, Mean(_bLateMotion[s], _bLateN[s]));
                early = System.Math.Max(early, Mean(_bEarlyMotion[s], _bEarlyN[s]));
                if (TouchesMovingKinematic(h, s, threshold)) { driven = true; }
            }

            i = j;
            if (flagged) { continue; }   // the settings finding already says so
            if (driven)
            {
                Add(DiagnosticCode.NeverSleeps, DiagnosticSeverity.Info, late / threshold, slots, default,
                    [V("bodies", slots.Length, "count"), V("observed", _time, "s"), V("sleepThreshold", threshold, "m/s")],
                    "A group of " + slots.Length + " bodies stayed awake for the whole observed " + N(_time) + " s because it touches or is joined to a kinematic body that moves faster than the sleep threshold; the engine keeps such an island awake.",
                    "Expected while the kinematic body moves. Stop or park the kinematic body to let the group sleep.");
            }
            else if (late > JitterBand * threshold)
            {
                AppendSlots(moving, slots); movingGroups++; movingLate = System.Math.Max(movingLate, late);
            }
            else if (early > 0 && late < Falling * early)
            {
                AppendSlots(settling, slots); settlingGroups++; settlingLate = System.Math.Max(settlingLate, late);
            }
            else
            {
                AppendSlots(jitter, slots); jitterGroups++; jitterLate = System.Math.Max(jitterLate, late);
            }
        }

        if (jitterGroups > 0)
        {
            jitter.Sort();
            Add(DiagnosticCode.NeverSleeps, DiagnosticSeverity.Warning, jitterLate / threshold, jitter.ToArray(), default,
                [V("groups", jitterGroups, "count"), V("bodies", jitter.Count, "count"), V("observed", _time, "s"), V("motion", jitterLate, "m/s", threshold), V("sleepThreshold", threshold, "m/s")],
                jitterGroups + " group(s) of " + jitter.Count + " bodies stayed awake for the whole observed " + N(_time) + " s while moving only " + N(jitterLate) + " m/s on average in the last third at most (sleep threshold " + N(threshold) + " m/s), and no slower than in the middle third: they are nearly at rest but jitter or creep, so their sleep timers keep restarting.",
                "Raise Options.SleepThreshold a little, add linear and angular damping or rolling resistance, add velocity iterations or substeps, or look for joints or contacts that fight (see the other findings).");
        }

        if (movingGroups > 0)
        {
            moving.Sort();
            Add(DiagnosticCode.NeverSleeps, DiagnosticSeverity.Info, movingLate / threshold, moving.ToArray(), default,
                [V("groups", movingGroups, "count"), V("bodies", moving.Count, "count"), V("observed", _time, "s"), V("motion", movingLate, "m/s", JitterBand * threshold)],
                movingGroups + " group(s) of " + moving.Count + " bodies stayed awake for the whole observed " + N(_time) + " s and were still moving fast (up to " + N(movingLate) + " m/s on average in the last third; sleep threshold " + N(threshold) + " m/s). They have simply not come to rest.",
                "Nothing to fix if they should keep moving. If they should settle, look for what drives them: a motor, a conveyor surface, a field, a rolling body on a slope.");
        }

        if (settlingGroups > 0)
        {
            settling.Sort();
            Add(DiagnosticCode.NeverSleeps, DiagnosticSeverity.Info, settlingLate / threshold, settling.ToArray(), default,
                [V("groups", settlingGroups, "count"), V("bodies", settling.Count, "count"), V("observed", _time, "s"), V("motion", settlingLate, "m/s", threshold)],
                settlingGroups + " group(s) of " + settling.Count + " bodies stayed awake for the whole observed " + N(_time) + " s but were moving more slowly at the end than in the middle: they are still settling.",
                "Nothing to fix unless they never settle; observe a longer time to see whether they sleep.");
        }
    }

    // Appends slots in ascending order within the group; the caller sorts the whole list, so the entity list is the lowest slots overall.
    private static void AppendSlots(List<int> list, int[] slots) { list.AddRange(slots); }

    private static bool TouchesMovingKinematic(DiagnosisHost h, int body, double threshold)
    {
        var core = h.Core;
        var bodies = core.Bodies;
        var cs = core.Constraints;
        for (int e = bodies.FirstEdge(body); e >= 0; e = cs.NextEdge(e))
        {
            int c = e >> 1;
            int other = (e & 1) == 0 ? cs.BodyB[c] : cs.BodyA[c];
            if (bodies.TypeOf(other) == BodyType.Kinematic && h.Motion(other) > threshold) { return true; }
        }

        return false;
    }

    private void FinishTunnelling(DiagnosisHost h)
    {
        var core = h.Core;
        var bodies = core.Bodies;
        bool ccd = h.ContinuousEnabled;
        // The two thinnest bodies among all bodies and among the movable ones, so that the thinnest *other* body is known for each candidate.
        double a1 = double.MaxValue, a2 = double.MaxValue, m1 = double.MaxValue, m2 = double.MaxValue;
        int a1s = -1, m1s = -1;
        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (!bodies.IsLive(s)) { continue; }
            float me = h.MinExtent(s);
            if (me >= float.MaxValue / 2) { continue; }
            double t = 2.0 * me;
            if (t < a1) { a2 = a1; a1 = t; a1s = s; } else if (t < a2) { a2 = t; }
            if (bodies.TypeOf(s) != BodyType.Static)
            {
                if (t < m1) { m2 = m1; m1 = t; m1s = s; } else if (t < m2) { m2 = t; }
            }
        }

        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (!bodies.IsLive(s) || bodies.TypeOf(s) != BodyType.Dynamic || _bGen[s] != bodies.Handles.GenerationOf(s)) { continue; }
            if (ccd && (bodies.Flags[s] & (byte)BodyBits.Bullet) != 0) { continue; }
            float me = h.MinExtent(s);
            if (me >= float.MaxValue / 2) { continue; }
            double self = 2.0 * me;
            double other = ccd ? (s == m1s ? m2 : m1) : (s == a1s ? a2 : a1);
            if (other >= double.MaxValue / 2) { continue; }
            double travel = _bMaxTravel[s];
            if (!(travel > self + other)) { continue; }
            Add(DiagnosticCode.TunnellingRisk, ccd ? DiagnosticSeverity.Info : DiagnosticSeverity.Warning, travel / (self + other), [s], default,
                [V("travelPerStep", travel, "m", self + other), V("thickness", self, "m"), V("thinnestOtherThickness", other, "m"), V("continuousCollision", ccd ? 1 : 0, "flag")],
                "Body moves up to " + N(travel) + " m in one step, more than its own " + N(self) + " m thickness plus the " + N(other) + " m of the thinnest " + (ccd ? "movable body it can meet" : "body in the world") + ", so it can pass through one in a single step" + (ccd ? ". Continuous collision protects against static shapes, but against moving bodies only a body with the bullet flag is swept." : ", and continuous collision is switched off."),
                ccd ? "Set the bullet flag on the fast body, or make the thin things thicker, or use a smaller time step." : "Enable Options.ContinuousEnabled, set the bullet flag on fast bodies, or make the thin things thicker.");
        }
    }
}
