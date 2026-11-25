using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Options for <see cref="PhysicsWorld3D.CheckInsertion"/>.</summary>
public sealed class InsertionOptions3D
{
    /// <summary>The clearance profile is sampled at <c>Stations + 1</c> evenly spaced points from the start to the end of the travelled path, 1 to 4096. Default 16.</summary>
    public int Stations { get; set; } = 16;
    /// <summary>Lateral directions probed at each station, evenly spaced around the path axis, 4 to 360. Direction 0 is fixed by the axis (see <see cref="InsertionReport3D.MinClearanceDirection"/>); with a multiple of 4 and an axis along a world axis the probes include the world axes. Default 8.</summary>
    public int ProbeDirections { get; set; } = 8;
    /// <summary>The farthest lateral probe in metres; a clearance at least this large is reported as this value. Shape casts are accurate to a fraction of the cast length, so a short probe keeps the clearance accurate. Finite and positive. Default 0.05.</summary>
    public float ProbeDistance { get; set; } = 0.05f;
    /// <summary>The part seats when nothing blocks it within this many metres of the full path. Finite, nonnegative. Default 0.001.</summary>
    public float SeatTolerance { get; set; } = 0.001f;
    /// <summary>The part counts as clear only when its smallest lateral clearance is at least this many metres. Finite, nonnegative. Default 0.</summary>
    public float RequiredClearance { get; set; }
    /// <summary>Ignore bodies joined directly to a part body, as <see cref="PhysicsWorld3D.SweepBody(Body, Vector3F, out QueryHit3D, float, bool, in QueryFilter3D)"/> does. Default false.</summary>
    public bool ExcludeJointed { get; set; }
    /// <summary>Which shapes of the mate count. Default: every non-sensor shape.</summary>
    public QueryFilter3D Filter { get; set; }

    internal void Validate()
    {
        if (Stations < 1 || Stations > 4096) throw new ArgumentOutOfRangeException(nameof(Stations), Stations, "Stations must be within 1..4096.");
        if (ProbeDirections < 4 || ProbeDirections > 360) throw new ArgumentOutOfRangeException(nameof(ProbeDirections), ProbeDirections, "ProbeDirections must be within 4..360.");
        if (!(ProbeDistance > 0f) || !float.IsFinite(ProbeDistance)) throw new ArgumentOutOfRangeException(nameof(ProbeDistance), ProbeDistance, "ProbeDistance must be finite and positive.");
        if (!(SeatTolerance >= 0f) || !float.IsFinite(SeatTolerance)) throw new ArgumentOutOfRangeException(nameof(SeatTolerance), SeatTolerance, "SeatTolerance must be finite and nonnegative.");
        if (!(RequiredClearance >= 0f) || !float.IsFinite(RequiredClearance)) throw new ArgumentOutOfRangeException(nameof(RequiredClearance), RequiredClearance, "RequiredClearance must be finite and nonnegative.");
    }
}

/// <summary>What happened when a part was pushed along an axis into its mate: where it first touches, whether it seats, and how much room it has at the sides along the way.</summary>
public sealed class InsertionReport3D
{
    internal InsertionReport3D() { }

    /// <summary>The path axis as a unit vector.</summary>
    public Vector3F Axis { get; internal set; }
    /// <summary>The requested path length, metres.</summary>
    public float PathLength { get; internal set; }
    /// <summary>True when nothing blocks the part within <see cref="InsertionOptions3D.SeatTolerance"/> of the full path and its smallest lateral clearance is at least <see cref="InsertionOptions3D.RequiredClearance"/>.</summary>
    public bool Seats { get; internal set; }
    /// <summary>True when something blocks the part before the end of the path.</summary>
    public bool IsBlocked { get; internal set; }
    /// <summary>True when the part already overlaps its mate at the start (first contact at distance zero).</summary>
    public bool StartsOverlapping { get; internal set; }
    /// <summary>Distance along the axis from the start to the first contact, metres; positive infinity when nothing is hit within the path. Accurate to about 2e-4 of <see cref="PathLength"/>.</summary>
    public float FirstContactDistance { get; internal set; } = float.PositiveInfinity;
    /// <summary>Where the first contact is, world coordinates (the origin when nothing is hit).</summary>
    public Vector3D FirstContactPoint { get; internal set; }
    /// <summary>Surface normal of the mate at the first contact, facing the part; zero when the part starts overlapping.</summary>
    public Vector3F FirstContactNormal { get; internal set; }
    /// <summary>The body the part touches first; the null handle when nothing is hit.</summary>
    public BodyHandle Blocker { get; internal set; }
    /// <summary>How far the part travels before it stops, metres: the first contact distance, or the path length when unobstructed. The profile spans 0 to this.</summary>
    public float TravelDistance { get; internal set; }
    /// <summary>The smallest lateral clearance at each of <c>Stations + 1</c> points from 0 to <see cref="TravelDistance"/> (one seat tolerance short of it when the part touches something, since resting on its mate along the axis would read as no clearance at the sides), metres: how far the part can move sideways, in the probed directions, before touching its mate (capped at the probe distance).</summary>
    public float[] ClearanceProfile { get; internal set; } = [];
    /// <summary>The smallest value of <see cref="ClearanceProfile"/>.</summary>
    public float MinClearance { get; internal set; }
    /// <summary>Distance along the path of the first station with the smallest clearance, metres.</summary>
    public float MinClearanceDistance { get; internal set; }
    /// <summary>The lateral direction in which the smallest clearance was found.</summary>
    public Vector3F MinClearanceDirection { get; internal set; }

    internal static InsertionReport3D Compute(PhysicsWorld3D world, ReadOnlySpan<Body> part, Vector3F axis, float distance, InsertionOptions3D o)
    {
        o.Validate();
        if (part.IsEmpty) throw new ArgumentException("A part needs at least one body.", nameof(part));
        double al = Math.Sqrt((double)axis.X * axis.X + (double)axis.Y * axis.Y + (double)axis.Z * axis.Z);
        if (!(al > 0) || !double.IsFinite(al)) throw new ArgumentOutOfRangeException(nameof(axis), axis, "The path axis must be finite and nonzero.");
        if (!(distance > 0f) || !float.IsFinite(distance) || distance > PhysicsWorld3D.MaxQueryDistance) throw new ArgumentOutOfRangeException(nameof(distance), distance, "The path length must be finite, positive and at most MaxQueryDistance metres.");
        double ax = axis.X / al, ay = axis.Y / al, az = axis.Z / al;
        var dir = new Vector3F((float)ax, (float)ay, (float)az);
        var engine = world.Engine;
        var handles = new BodyHandle[part.Length];
        for (int i = 0; i < handles.Length; i++)
        {
            if (!ReferenceEquals(part[i].World, world)) throw new InvalidHandleException("A body of the part is null or belongs to a different world.");
            handles[i] = part[i].RawHandle;
            if (part[i].Type == BodyType.Static) throw new ArgumentException("A static body cannot be inserted.", nameof(part));
        }
        var slots = new int[handles.Length];
        for (int i = 0; i < slots.Length; i++) slots[i] = world.Core.Resolve(handles[i]);
        Array.Sort(slots);
        var filter = o.Filter;
        var r = new InsertionReport3D { Axis = dir, PathLength = distance };
        engine.SweepExcluded = slots;
        try
        {
            // First contact: the nearest hit over every body of the part (ties keep the earlier body in slot order).
            bool hitAny = false; QueryHit3D best = default;
            foreach (int slot in slots)
            {
                var body = Find(part, world, slot);
                if (world.SweepBody(body, dir, out var hit, distance, o.ExcludeJointed, in filter) && (!hitAny || hit.Distance < best.Distance)) { best = hit; hitAny = true; }
            }
            if (hitAny)
            {
                r.FirstContactDistance = best.Distance; r.FirstContactPoint = best.Point; r.FirstContactNormal = best.Normal; r.Blocker = best.Body.RawHandle;
                r.StartsOverlapping = best.Distance <= 0f;
            }
            r.IsBlocked = hitAny && best.Distance < distance - o.SeatTolerance;
            r.TravelDistance = hitAny ? Math.Min(best.Distance, distance) : distance;

            // A horizontal reference from the axis: cross the axis with the world axis it is least aligned with.
            double bx = Math.Abs(ax), by = Math.Abs(ay), bz = Math.Abs(az);
            double hx = 0, hy = 0, hz = 0;
            if (bx <= by && bx <= bz) hx = 1; else if (by <= bz) hy = 1; else hz = 1;
            double e1x = hy * az - hz * ay, e1y = hz * ax - hx * az, e1z = hx * ay - hy * ax;
            double l1 = Math.Sqrt(e1x * e1x + e1y * e1y + e1z * e1z); e1x /= l1; e1y /= l1; e1z /= l1;
            double e2x = ay * e1z - az * e1y, e2y = az * e1x - ax * e1z, e2z = ax * e1y - ay * e1x;
            int m = o.ProbeDirections, n = o.Stations;
            var lateral = new Vector3F[m];
            for (int j = 0; j < m; j++)
            {
                double phi = 2.0 * Math.PI * j / m, c = Math64.Cos(phi), s = Math64.Sin(phi);
                lateral[j] = new Vector3F((float)(c * e1x + s * e2x), (float)(c * e1y + s * e2y), (float)(c * e1z + s * e2z));
            }
            // Stop the profile one seat tolerance short of the first contact: a part resting on its mate along the axis touches it sideways too, which would read as no clearance.
            float span = hitAny ? Math.Max(0f, r.TravelDistance - o.SeatTolerance) : r.TravelDistance;
            var profile = new float[n + 1];
            float min = float.PositiveInfinity; int minStation = 0, minDir = 0;
            for (int k = 0; k <= n; k++)
            {
                double travel = (double)span * k / n;
                engine.SweepShiftX = ax * travel; engine.SweepShiftY = ay * travel; engine.SweepShiftZ = az * travel;
                float station = o.ProbeDistance; int stationDir = 0;
                for (int j = 0; j < m; j++)
                {
                    foreach (int slot in slots)
                    {
                        var body = Find(part, world, slot);
                        float d = world.SweepBody(body, lateral[j], out var hit, o.ProbeDistance, o.ExcludeJointed, in filter) ? hit.Distance : o.ProbeDistance;
                        if (d < station) { station = d; stationDir = j; }
                    }
                }
                profile[k] = station;
                if (station < min) { min = station; minStation = k; minDir = stationDir; }
            }
            r.ClearanceProfile = profile; r.MinClearance = min; r.MinClearanceDistance = (float)((double)span * minStation / n); r.MinClearanceDirection = lateral[minDir];
            r.Seats = !r.IsBlocked && min >= o.RequiredClearance;
            return r;
        }
        finally { engine.SweepExcluded = null; engine.SweepShiftX = engine.SweepShiftY = engine.SweepShiftZ = 0; }
    }

    private static Body Find(ReadOnlySpan<Body> part, PhysicsWorld3D world, int slot)
    {
        foreach (var b in part) { if (world.Core.Resolve(b.RawHandle) == slot) return b; }
        throw new InvalidOperationException("A part body disappeared during the check.");
    }
}

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Pushes a part (one body, or several bodies moved together) along an axis into its mate without moving anything, and reports where it first touches, whether it seats, and the room it has at the sides along the way.
    /// Built on <see cref="SweepBody(Body, Vector3F, out QueryHit3D, float, bool, in QueryFilter3D)"/>: the first contact is the whole-body sweep along the axis; the clearance profile casts the part sideways, from stations along the path it can travel, in evenly spaced directions around the axis.
    /// The other bodies of the part are ignored by every sweep. The part is not moved, woken or changed, and the world's state is untouched. Allocates the report and its profile.
    /// </summary>
    /// <param name="part">The bodies that move together. At least one; none static.</param>
    /// <param name="axis">Direction of travel; normalised. Finite and nonzero.</param>
    /// <param name="distance">Path length in metres: finite, positive, at most <see cref="MaxQueryDistance"/>. To test seating, give the depth at which the part is fully home.</param>
    /// <param name="options">Defaults when null.</param>
    /// <exception cref="ArgumentOutOfRangeException">The axis, distance or an option is out of range.</exception>
    /// <exception cref="ArgumentException">The part is empty or holds a static body.</exception>
    /// <exception cref="InvalidHandleException">A body is null or belongs to another world.</exception>
    /// <exception cref="StaleHandleException">A body was destroyed.</exception>
    public InsertionReport3D CheckInsertion(ReadOnlySpan<Body> part, Vector3F axis, float distance, InsertionOptions3D? options = null)
    {
        _core.FlushPostStep();
        return InsertionReport3D.Compute(this, part, axis, distance, options ?? new InsertionOptions3D());
    }
}

public readonly partial struct Body
{
    /// <summary>The insertion check of this body alone; see <see cref="PhysicsWorld3D.CheckInsertion"/>.</summary>
    public InsertionReport3D CheckInsertion(Vector3F axis, float distance, InsertionOptions3D? options = null)
    {
        var (world, _) = Resolve();
        return world.CheckInsertion([this], axis, distance, options);
    }
}
