using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Diagnosis;

internal sealed partial class Diagnoser
{
    /// <summary>
    /// Examines the world as it is now: its settings, the sizes, masses, speeds and anchors, and the contacts it already holds. Reads only; nothing is stepped.
    /// </summary>
    public void StaticPass(DiagnosisHost h)
    {
        _handles = h.Core;
        _firstStep = _lastStep = h.StepCount;
        var core = h.Core;
        var bodies = core.Bodies;
        var cs = core.Constraints;
        EnsureBodies(bodies.HighWater);

        int dynamicCount = 0;
        var noSleep = new List<(int Island, int Slot)>();
        for (int s = 0; s < bodies.HighWater; s++)
        {
            if (!bodies.IsLive(s) || bodies.TypeOf(s) != BodyType.Dynamic) { continue; }
            dynamicCount++;
            TrackBody(h, s);
            double motion = h.Motion(s);
            _bMaxMotion[s] = System.Math.Max(_bMaxMotion[s], motion);
            _bMaxTravel[s] = System.Math.Max(_bMaxTravel[s], motion * _dt);
            CheckSize(h, s);
            if ((bodies.Flags[s] & (byte)BodyBits.NoSleep) != 0) { noSleep.Add((bodies.Island[s], s)); }
        }

        CheckGravity(h.GravityMagnitude);
        CheckSleepSettings(h.SleepEnabled, dynamicCount, noSleep);
        CheckJoints(h);
        if (cs.Count > 0) { ScanContactsOnce(h); }
    }

    private void CheckGravity(double mag)
    {
        if (mag > GravityHigh)
        {
            Add(DiagnosticCode.ImplausibleGravity, DiagnosticSeverity.Warning, mag, default, default,
                [V("gravity", mag, "m/s^2", GravityHigh)],
                "Gravity is " + N(mag) + " m/s^2, which is " + N(mag / 9.81) + " times Earth's. The engine's tolerances (contact distance, rest thresholds, speed limit) are in metres, so a world authored in centimetres, millimetres or pixels behaves as if it were enormous.",
                "If one world unit is not a metre, scale the scene to metres, or set the gravity to about 9.81 in metres per second squared.");
        }
        else if (mag > 0 && mag < GravityLow)
        {
            Add(DiagnosticCode.ImplausibleGravity, DiagnosticSeverity.Info, GravityLow / mag, default, default,
                [V("gravity", mag, "m/s^2", GravityLow)],
                "Gravity is " + N(mag) + " m/s^2, far weaker than any planet's. It is legitimate for a low-gravity world; it is also what a world authored in a larger unit than metres looks like.",
                "If the world is meant to be Earth-like, set the gravity to about 9.81 in metres per second squared and size the scene in metres.");
        }
    }

    private void CheckSleepSettings(bool sleepEnabled, int dynamicCount, List<(int Island, int Slot)> noSleep)
    {
        if (!sleepEnabled && dynamicCount > 0)
        {
            Add(DiagnosticCode.NeverSleeps, DiagnosticSeverity.Info, 0, default, default,
                [V("dynamicBodies", dynamicCount, "count")],
                "Sleeping is switched off for the whole world, so no island ever sleeps and every body is simulated every step.",
                "Set Options.SleepEnabled to true unless the host needs every body awake.");
        }

        if (noSleep.Count == 0) { return; }
        noSleep.Sort(static (a, b) => a.Island != b.Island ? a.Island.CompareTo(b.Island) : a.Slot.CompareTo(b.Slot));
        int i = 0;
        while (i < noSleep.Count)
        {
            int j = i;
            while (j < noSleep.Count && noSleep[j].Island == noSleep[i].Island) { j++; }
            var slots = new int[j - i];
            for (int k = i; k < j; k++) { slots[k - i] = noSleep[k].Slot; }
            Add(DiagnosticCode.NeverSleeps, DiagnosticSeverity.Info, slots.Length, slots, default,
                [V("noSleepBodies", slots.Length, "count")],
                "Body flagged never to sleep holds its whole island awake (" + slots.Length + " such body(ies) in this island).",
                "Clear the body's no-sleep setting unless it must be simulated every step; an island sleeps only when every body in it has rested.");
            i = j;
        }
    }

    private void CheckSize(DiagnosisHost h, int s)
    {
        float min = h.MinExtent(s);
        if (min >= float.MaxValue / 2) { return; }   // no shape
        double thick = 2.0 * min, spec = h.SpeculativeDistance;
        if (thick < spec)
        {
            Add(DiagnosticCode.ImplausibleSize, DiagnosticSeverity.Warning, spec / System.Math.Max(thick, 1e-9), [s], default,
                [V("thickness", thick, "m", spec)],
                "Dynamic body is " + N(thick) + " m thick, thinner than the " + N(spec) + " m distance within which contacts are made ahead of touching. Both faces are then in contact range at once and the contact solve cannot tell which side the other shape is on.",
                "Make the shape at least a few times thicker than " + N(spec) + " m, or, if the scene is not in metres, rescale it to metres.");
        }

        double reach = h.MaxExtent(s);
        if (reach > HugeBody)
        {
            Add(DiagnosticCode.ImplausibleSize, DiagnosticSeverity.Info, reach / HugeBody, [s], default,
                [V("reach", reach, "m", HugeBody)],
                "Dynamic body reaches " + N(reach) + " m from its centre of mass. Float32 positions of shape points that far out resolve only to " + N(reach * 1.2e-7) + " m, and its contacts are tuned for much smaller things.",
                "Make large things static, split them into smaller shapes, or rescale the scene to metres.");
        }
    }

    private void CheckJoints(DiagnosisHost h)
    {
        var core = h.Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint) { continue; }
            var f = h.Joint(c);
            if (!f.Solves) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            bool da = bodies.TypeOf(a) == BodyType.Dynamic, db = bodies.TypeOf(b) == BodyType.Dynamic;
            if (da && db)
            {
                double ma = h.Mass(a), mb = h.Mass(b);
                if (ma > 0 && mb > 0)
                {
                    double ratio = System.Math.Max(ma, mb) / System.Math.Min(ma, mb);
                    if (ratio > JointRatioInfo)
                    {
                        var sev = ratio > JointRatioError ? DiagnosticSeverity.Error : ratio > JointRatioWarn ? DiagnosticSeverity.Warning : DiagnosticSeverity.Info;
                        string why = sev == DiagnosticSeverity.Info
                            ? "above 50 the island leaves the cheaper contact-only schedule and runs all substeps; a chain with a 500:1 end mass is the heaviest ratio measured to hold"
                            : sev == DiagnosticSeverity.Warning ? "beyond the 500:1 chain the engine was measured on, the heavy body drags the light one faster than the iterations can correct, and a 1000:1 chain stretches by several body lengths"
                            : "at 1000:1 and beyond the measured chain stretched by several body lengths and reached 130 m/s: expect the light body to be flung or the joint to stretch";
                        Add(DiagnosticCode.JointMassRatio, sev, ratio, [System.Math.Min(a, b), System.Math.Max(a, b)], [c],
                            [V("massRatio", ratio, "ratio", sev == DiagnosticSeverity.Info ? JointRatioInfo : sev == DiagnosticSeverity.Warning ? JointRatioWarn : JointRatioError), V("heavyMass", System.Math.Max(ma, mb), "kg"), V("lightMass", System.Math.Min(ma, mb), "kg")],
                            "The " + f.Type + " joint connects bodies of " + N(ma) + " kg and " + N(mb) + " kg (ratio " + N(ratio) + ":1); " + why + ".",
                            "Bring the masses closer (a light body hanging from a heavy one wants a similar mass), or make the heavy side kinematic or static, or add substeps.");
                    }
                }
            }

            if (da) { CheckAnchorFar(h, c, a, f, f.ArmA, "A"); }
            if (db) { CheckAnchorFar(h, c, b, f, f.ArmB, "B"); }
            if (f.Coincident && (da || db))
            {
                double reach = double.MaxValue;
                if (da) { reach = System.Math.Min(reach, h.MaxExtent(a)); }
                if (db) { reach = System.Math.Min(reach, h.MaxExtent(b)); }
                double limit = System.Math.Max(AnchorsApartFloor, AnchorsApartReach * reach);
                if (f.Gap > limit)
                {
                    Add(DiagnosticCode.AnchorsApart, DiagnosticSeverity.Warning, f.Gap / limit, [System.Math.Min(a, b), System.Math.Max(a, b)], [c],
                        [V("anchorGap", f.Gap, "m", limit)],
                        "The " + f.Type + " joint's anchors start " + N(f.Gap) + " m apart although the joint keeps them together. The first steps pull the bodies together with whatever speed it takes, which throws them about.",
                        "Place the bodies so the anchors coincide when the joint is created, or compute each local anchor from one world point.");
                }
            }
        }
    }

    private void CheckAnchorFar(DiagnosisHost h, int c, int body, JointFacts f, double arm, string side)
    {
        double reach = h.MaxExtent(body);
        double limit = AnchorFarReach * reach + AnchorFarSlack;
        if (!(reach > 0) || arm <= limit) { return; }
        Add(DiagnosticCode.AnchorFarFromBody, DiagnosticSeverity.Warning, arm / limit, [body], [c],
            [V("anchorDistance", arm, "m", limit), V("bodyReach", reach, "m")],
            "Anchor " + side + " of the " + f.Type + " joint is " + N(arm) + " m from its body's centre of mass, but the body reaches only " + N(reach) + " m. A long lever arm like this makes the joint's effective mass ill-conditioned; the engine refuses 40 reaches.",
            "Anchor the joint inside or near the body, or give the body a shape that reaches the anchor.");
    }

    private struct PairHit
    {
        public int A, B;
        public double Depth, Rel, Ratio;
    }

    private static double HalfThin(DiagnosisHost h, int a, int b)
    {
        var bodies = h.Core.Bodies;
        double m = double.MaxValue;
        if (bodies.TypeOf(a) == BodyType.Dynamic) { m = System.Math.Min(m, h.MinExtent(a)); }
        if (bodies.TypeOf(b) == BodyType.Dynamic) { m = System.Math.Min(m, h.MinExtent(b)); }
        return m >= float.MaxValue / 2 ? 0 : m;
    }

    private static double Depth(WorldCore core, int c)
    {
        var cs = core.Constraints;
        var p = cs.Payload(c);
        double depth = 0;
        int n = cs.PointCount[c];
        for (int i = 0; i < n; i++) { depth = System.Math.Max(depth, -(double)p[ManifoldLayout.Separation(i)]); }
        return depth;
    }

    /// <summary>
    /// Looks at every contact once, at the first state that has contacts: deep overlaps (shapes created inside each other) and heavy-on-light pairs. Both are properties of the starting arrangement,
    /// so a later scan would only see what the solver has already pushed apart.
    /// </summary>
    private void ScanContactsOnce(DiagnosisHost h)
    {
        var core = h.Core;
        var cs = core.Constraints;
        var bodies = core.Bodies;
        var hits = new List<PairHit>();
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Contact || cs.PointCount[c] == 0) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            bool da = bodies.TypeOf(a) == BodyType.Dynamic, db = bodies.TypeOf(b) == BodyType.Dynamic;
            if (!da && !db) { continue; }
            double depth = Depth(core, c);
            double thin = HalfThin(h, a, b);
            double ratio = 0;
            if (da && db)
            {
                double ma = h.Mass(a), mb = h.Mass(b);
                if (ma > 0 && mb > 0) { ratio = System.Math.Max(ma, mb) / System.Math.Min(ma, mb); }
            }

            double rel = thin > 0 ? depth / thin : 0;
            // A deep contact between bodies that are still moving is an impact, which the solver is about to resolve; only a deep overlap at rest says the shapes began inside each other.
            bool slow = (!da || h.Motion(a) <= SlowMotion) && (!db || h.Motion(b) <= SlowMotion);
            if ((slow && depth > OverlapFloor && rel > OverlapRel) || ratio > ContactRatioInfo)
            {
                hits.Add(new PairHit { A = System.Math.Min(a, b), B = System.Math.Max(a, b), Depth = slow ? depth : 0, Rel = slow ? rel : 0, Ratio = ratio });
            }
        }

        _contactScanDone = true;
        if (hits.Count == 0) { return; }
        hits.Sort(static (x, y) => x.A != y.A ? x.A.CompareTo(y.A) : x.B != y.B ? x.B.CompareTo(y.B) : y.Depth.CompareTo(x.Depth));
        for (int i = 0; i < hits.Count; i++)
        {
            var p = hits[i];
            if (i > 0 && hits[i - 1].A == p.A && hits[i - 1].B == p.B) { continue; }   // one finding per pair: the deepest contact is first
            double ratio = p.Ratio, rel = p.Rel;
            for (int k = i + 1; k < hits.Count && hits[k].A == p.A && hits[k].B == p.B; k++) { ratio = System.Math.Max(ratio, hits[k].Ratio); }
            if (p.Depth > OverlapFloor && rel > OverlapRel)
            {
                var sev = rel >= 2.0 ? DiagnosticSeverity.Error : DiagnosticSeverity.Warning;
                Add(DiagnosticCode.ShapesOverlapping, sev, rel, [p.A, p.B], default,
                    [V("depth", p.Depth, "m", OverlapFloor), V("depthOverHalfThickness", rel, "ratio", OverlapRel)],
                    "Two bodies overlap by " + N(p.Depth) + " m, " + N(rel) + " times the half-thickness of the thinner one, at the start of the examined time. If they were created like that, the solver separates them with a violent push that the rest of the scene feels; in a running world it is what a fast impact leaves.",
                    "Place the bodies apart when creating them (or let one start clear of the other and move it in).");
            }

            if (ratio > ContactRatioInfo)
            {
                var sev = ratio > ContactRatioError ? DiagnosticSeverity.Error : ratio > ContactRatioWarn ? DiagnosticSeverity.Warning : DiagnosticSeverity.Info;
                Add(DiagnosticCode.ContactMassRatio, sev, ratio, [p.A, p.B], default,
                    [V("massRatio", ratio, "ratio", sev == DiagnosticSeverity.Info ? ContactRatioInfo : sev == DiagnosticSeverity.Warning ? ContactRatioWarn : ContactRatioError)],
                    "Two dynamic bodies in contact differ in mass by " + N(ratio) + ":1. A heavy body resting on a light one sinks into it, because the solver corrects each contact with the same effort whatever the masses.",
                    "Give the light body more mass, make the heavy one kinematic if it is moved by hand, or raise the substeps.");
            }
        }

    }
}
