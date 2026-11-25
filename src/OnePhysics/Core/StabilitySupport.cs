using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Where an assembly touches the rest of the world.</summary>
internal readonly record struct SupportPoint(double X, double Y, double Z, float Friction, float Impulse);

internal static class StabilitySupport
{
    /// <summary>
    /// The sorted distinct slots of the bodies in an assembly; every one must be a live dynamic body of this world.
    /// </summary>
    public static int[] Slots(WorldCore core, ReadOnlySpan<BodyHandle> handles)
    {
        if (handles.IsEmpty) throw new ArgumentException("An assembly needs at least one body.", nameof(handles));
        var slots = new List<int>(handles.Length);
        foreach (var h in handles)
        {
            int s = core.Resolve(h);
            if (core.Bodies.TypeOf(s) != BodyType.Dynamic) throw new ArgumentException("Every body of an assembly must be dynamic: a static or kinematic body is the ground, not part of the load.", nameof(handles));
            slots.Add(s);
        }
        slots.Sort();
        var result = new List<int>(slots.Count);
        foreach (int s in slots) { if (result.Count == 0 || result[^1] != s) result.Add(s); }
        return result.ToArray();
    }

    /// <summary>
    /// Contact points between a member of the assembly and any body outside it, in constraint-slot then point order, whose separation is at most the tolerance.
    /// </summary>
    public static List<SupportPoint> Gather(WorldCore core, int[] members, float tolerance)
    {
        var points = new List<SupportPoint>();
        var cons = core.Constraints;
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c) || cons.KindOf(c) != ConstraintKind.Contact || cons.PointCount[c] == 0) continue;
            bool a = Array.BinarySearch(members, cons.BodyA[c]) >= 0, b = Array.BinarySearch(members, cons.BodyB[c]) >= 0;
            if (a == b) continue;
            var view = new ContactView(core, c);
            for (int i = 0; i < view.PointCount; i++)
            {
                if (view.Separation(i) > tolerance) continue;
                var p = view.Point(i);
                points.Add(new SupportPoint(p.X, p.Y, p.Z, view.Friction, Math.Max(view.NormalImpulse(i), 0f)));
            }
        }
        return points;
    }

    /// <summary>The friction a push has to beat: the mean of the support contacts' friction weighted by their normal impulse (unweighted when no impulse is stored).</summary>
    public static float EffectiveFriction(List<SupportPoint> points)
    {
        double sum = 0, weight = 0;
        foreach (var p in points) { sum += (double)p.Friction * p.Impulse; weight += p.Impulse; }
        if (weight > 0) return (float)(sum / weight);
        if (points.Count == 0) return 0f;
        double plain = 0;
        foreach (var p in points) plain += p.Friction;
        return (float)(plain / points.Count);
    }
}
