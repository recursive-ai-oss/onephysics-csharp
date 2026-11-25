using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>What the settle loop needs from a world of either dimension.</summary>
internal interface ISettleHost
{
    WorldCore Core { get; }
    /// <summary>Mass in kilograms of a dynamic body slot.</summary>
    float MassOf(int slot);
    /// <summary>One fixed step with sleeping disabled and callbacks suppressed.</summary>
    void Step(float dt);
    /// <summary>Damps the particles of awake soft bodies and rods (velocity divided by 1 + c dt) and returns their worst speed.</summary>
    float DampAndScanDeformables(float linearDamping, float dt);
}

internal readonly struct SettleParams
{
    public float Dt { get; init; }
    public int MaxSteps { get; init; }
    public float SpeedTolerance { get; init; }
    public float AngularSpeedTolerance { get; init; }
    public float AccelerationTolerance { get; init; }
    public int StableSteps { get; init; }
    public float ExtraLinearDamping { get; init; }
    public float ExtraAngularDamping { get; init; }
    public bool Detail { get; init; }
    public bool HasBounds { get; init; }
    public double[]? BoundsMin { get; init; }
    public double[]? BoundsMax { get; init; }
    public float SupportTolerance { get; init; }
}

/// <summary>
/// The settle loop shared by both worlds. Equilibrium is judged on speed, angular speed and acceleration together: speed alone would pass a ball at the top of a bounce,
/// acceleration alone would pass a block sliding at a constant speed. Acceleration is the change of velocity over the last step, so a body held by contacts or joints reads zero.
/// </summary>
internal static class SettleRunner
{
    public static SettleReport Run<THost>(THost host, in SettleParams p) where THost : ISettleHost
    {
        var core = host.Core;
        var bodies = core.Bodies;
        int lin = bodies.Layout.LinearLanes, ang = bodies.Layout.AngularLanes, pos = bodies.Layout.PositionLanes;
        int n = bodies.HighWater;
        var prev = new float[Math.Max(n * lin, 1)];
        var lv = bodies.LinearVelocity;
        lv.Slice(0, n * lin).CopyTo(prev);
        var report = new SettleReport();
        var forces = p.Detail ? new float[n] : null;
        int stable = 0, steps = 0;
        bool converged = false;
        // Damping is implicit (v / (1 + c dt)), so any c >= 0 is unconditionally stable and c = 0 skips the pass.
        float linDamp = p.ExtraLinearDamping > 0 ? 1f / (1f + p.ExtraLinearDamping * p.Dt) : 1f;
        float angDamp = p.ExtraAngularDamping > 0 ? 1f / (1f + p.ExtraAngularDamping * p.Dt) : 1f;
        float invDt = 1f / p.Dt;
        while (steps < p.MaxSteps)
        {
            host.Step(p.Dt); steps++;
            n = bodies.HighWater;
            if (prev.Length < n * lin) { Array.Resize(ref prev, n * lin); }
            lv = bodies.LinearVelocity; var av = bodies.AngularVelocity;
            if (forces != null) { Array.Clear(forces); }
            float worstSpeed = 0, worstAng = 0, worstForce = 0, worstAcc = 0; int worstSpeedBody = -1, worstForceBody = -1, judged = 0;
            for (int b = 0; b < n; b++)
            {
                if (!bodies.IsDynamic(b) || !bodies.IsAwake(b)) { for (int k = 0; k < lin; k++) { prev[b * lin + k] = lv[b * lin + k]; } continue; }
                if (linDamp != 1f) { for (int k = 0; k < lin; k++) { lv[b * lin + k] *= linDamp; } }
                if (angDamp != 1f) { for (int k = 0; k < ang; k++) { av[b * ang + k] *= angDamp; } }
                double sq = 0, dq = 0, aq = 0;
                for (int k = 0; k < lin; k++) { double v = lv[b * lin + k]; sq += v * v; double d = v - prev[b * lin + k]; dq += d * d; prev[b * lin + k] = lv[b * lin + k]; }
                for (int k = 0; k < ang; k++) { double w = av[b * ang + k]; aq += w * w; }
                if (p.HasBounds && Outside(bodies, b, pos, p)) { continue; }
                judged++;
                float speed = (float)Math.Sqrt(sq), angular = (float)Math.Sqrt(aq);
                float acc = (float)Math.Sqrt(dq) * invDt;
                float force = host.MassOf(b) * acc;
                if (speed > worstSpeed) { worstSpeed = speed; worstSpeedBody = b; }
                if (angular > worstAng) { worstAng = angular; }
                if (force > worstForce) { worstForce = force; worstForceBody = b; }
                if (acc > worstAcc) { worstAcc = acc; }
                if (forces != null) { forces[b] = force; }
            }
            float deformable = host.DampAndScanDeformables(p.ExtraLinearDamping, p.Dt);
            report.WorstSpeed = worstSpeed; report.WorstAngularSpeed = worstAng; report.WorstForce = worstForce; report.WorstDeformableSpeed = deformable;
            report.BodiesJudged = judged;
            report.WorstSpeedBody = worstSpeedBody < 0 ? default : core.HandleOfBody(worstSpeedBody);
            report.WorstForceBody = worstForceBody < 0 ? default : core.HandleOfBody(worstForceBody);
            bool at = worstSpeed <= p.SpeedTolerance && worstAng <= p.AngularSpeedTolerance && worstAcc <= p.AccelerationTolerance && deformable <= p.SpeedTolerance;
            stable = at ? stable + 1 : 0;
            if (stable >= p.StableSteps) { converged = true; break; }
        }
        report.Converged = converged; report.StepsUsed = steps;
        Classify(core, p, report, forces);
        return report;
    }

    private static bool Outside(BodyStore bodies, int b, int pos, in SettleParams p)
    {
        var x = bodies.PositionOf(b);
        for (int k = 0; k < pos; k++) { if (x[k] < p.BoundsMin![k] || x[k] > p.BoundsMax![k]) { return true; } }
        return false;
    }

    // Support is reachability from a static or kinematic body through joints and touching contacts, by union-find with the lowest slot as root.
    private static void Classify(WorldCore core, in SettleParams p, SettleReport report, float[]? forces)
    {
        var bodies = core.Bodies; var cons = core.Constraints;
        int n = bodies.HighWater, pos = bodies.Layout.PositionLanes;
        var parent = new int[n];
        for (int i = 0; i < n; i++) { parent[i] = i; }
        var anchored = new bool[n];
        for (int i = 0; i < n; i++) { anchored[i] = bodies.IsLive(i) && bodies.TypeOf(i) != BodyType.Dynamic; }
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c)) { continue; }
            var kind = cons.KindOf(c);
            int a = cons.BodyA[c], bb = cons.BodyB[c];
            if (a < 0 || bb < 0 || a >= n || bb >= n) { continue; }
            if (kind == ConstraintKind.Contact)
            {
                if (cons.PointCount[c] == 0 || !Touching(core, c, p.SupportTolerance)) { continue; }
            }
            else if (kind != ConstraintKind.Joint) { continue; }
            int ra = Find(parent, a), rb = Find(parent, bb);
            if (ra == rb) { continue; }
            int lo = Math.Min(ra, rb), hi = Math.Max(ra, rb);
            parent[hi] = lo; anchored[lo] |= anchored[hi];
        }
        int unsupported = 0, outside = 0;
        var unsupportedList = p.Detail ? new System.Collections.Generic.List<BodyHandle>() : null;
        var outsideList = p.Detail ? new System.Collections.Generic.List<BodyHandle>() : null;
        var results = p.Detail ? new System.Collections.Generic.List<SettleBodyResult>() : null;
        for (int b = 0; b < n; b++)
        {
            if (!bodies.IsDynamic(b)) { continue; }
            bool supported = anchored[Find(parent, b)];
            bool inBounds = !p.HasBounds || !Outside(bodies, b, pos, p);
            if (!supported) { unsupported++; unsupportedList?.Add(core.HandleOfBody(b)); }
            if (!inBounds) { outside++; outsideList?.Add(core.HandleOfBody(b)); }
            if (results != null && bodies.IsAwake(b)) { results.Add(new SettleBodyResult(core.HandleOfBody(b), 0, 0, 0, supported, inBounds)); }
        }
        report.UnsupportedCount = unsupported; report.OutOfBoundsCount = outside;
        if (p.Detail)
        {
            report.Unsupported = unsupportedList!.ToArray(); report.OutOfBounds = outsideList!.ToArray();
            report.Bodies = FillResiduals(core, results!, forces!);
        }
    }

    private static SettleBodyResult[] FillResiduals(WorldCore core, System.Collections.Generic.List<SettleBodyResult> list, float[] forces)
    {
        var bodies = core.Bodies; int lin = bodies.Layout.LinearLanes, ang = bodies.Layout.AngularLanes;
        var lv = bodies.LinearVelocity; var av = bodies.AngularVelocity;
        var result = new SettleBodyResult[list.Count];
        for (int i = 0; i < result.Length; i++)
        {
            int b = list[i].Body.Index; double sq = 0, aq = 0;
            for (int k = 0; k < lin; k++) { double v = lv[b * lin + k]; sq += v * v; }
            for (int k = 0; k < ang; k++) { double w = av[b * ang + k]; aq += w * w; }
            result[i] = new SettleBodyResult(list[i].Body, (float)Math.Sqrt(sq), (float)Math.Sqrt(aq), b < forces.Length ? forces[b] : 0f, list[i].IsSupported, list[i].IsInBounds);
        }
        return result;
    }

    private static bool Touching(WorldCore core, int c, float tolerance)
    {
        var cons = core.Constraints; var payload = cons.Payload(c); int points = cons.PointCount[c];
        for (int i = 0; i < points; i++)
        {
            if (payload[ManifoldLayout.Separation(i)] <= tolerance || payload[ManifoldLayout.NormalImpulse(i)] > 0f) { return true; }
        }
        return false;
    }

    private static int Find(int[] parent, int i)
    {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }
}
