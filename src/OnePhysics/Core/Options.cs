using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>
/// Lane counts that decide how the dimension-independent storage is laid out: how many numbers a position, a rotation, a velocity and an inverse inertia take.
/// The same arrays and the same scheduler serve 2D and 3D; only these strides differ.
/// </summary>
public readonly struct DimensionLayout : IEquatable<DimensionLayout>
{
    /// <summary>Doubles per body position (2 or 3).</summary>
    public int PositionLanes { get; }

    /// <summary>Floats per rotation: 2 for (cos, sin) in 2D, 4 for a unit quaternion (x, y, z, w) in 3D.</summary>
    public int RotationLanes { get; }

    /// <summary>Floats per linear velocity (2 or 3).</summary>
    public int LinearLanes { get; }

    /// <summary>Floats per angular velocity: 1 in 2D, 3 in 3D.</summary>
    public int AngularLanes { get; }

    /// <summary>Floats per inverse inertia: 1 in 2D, 6 in 3D (upper triangle xx, xy, xz, yy, yz, zz of the symmetric 3x3 tensor in the body frame).</summary>
    public int InertiaLanes { get; }

    /// <summary>Contact points a manifold may carry: 2 in 2D, 4 in 3D.</summary>
    public int MaxManifoldPoints { get; }

    /// <summary>Floats of persistent per-constraint payload (manifold geometry and warm-start impulses, or joint rows).</summary>
    public int PayloadFloats { get; }

    private DimensionLayout(int pos, int rot, int lin, int ang, int inertia, int points, int payload)
    {
        PositionLanes = pos; RotationLanes = rot; LinearLanes = lin; AngularLanes = ang; InertiaLanes = inertia; MaxManifoldPoints = points; PayloadFloats = payload;
    }

    /// <summary>The 2D layout.</summary>
    public static DimensionLayout Planar => new(2, 2, 2, 1, 1, 2, ManifoldLayout.Header + 2 * ManifoldLayout.PointStride + 7);

    /// <summary>The 3D layout, large enough for four-point manifolds with friction and rolling rows, or a six-row joint.</summary>
    public static DimensionLayout Spatial => new(3, 4, 3, 3, 6, 4, ManifoldLayout.Header + 4 * ManifoldLayout.PointStride + 7 + ManifoldLayout.RecycleFloats);

    /// <summary>True when every lane count is one of the supported 2D or 3D values.</summary>
    public bool IsSupported => Equals(Planar) || Equals(Spatial);

    /// <summary>Fills a rotation with the identity: (1, 0) in 2D, (0, 0, 0, 1) in 3D.</summary>
    public void WriteIdentityRotation(Span<float> rotation)
    {
        rotation.Clear();
        rotation[RotationLanes == 4 ? 3 : 0] = 1f;
    }

    /// <inheritdoc/>
    public bool Equals(DimensionLayout o) => PositionLanes == o.PositionLanes && RotationLanes == o.RotationLanes && LinearLanes == o.LinearLanes
        && AngularLanes == o.AngularLanes && InertiaLanes == o.InertiaLanes && MaxManifoldPoints == o.MaxManifoldPoints && PayloadFloats == o.PayloadFloats;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is DimensionLayout l && Equals(l);

    /// <inheritdoc/>
    public override int GetHashCode() => PositionLanes * 100 + PayloadFloats;

    /// <summary>Equality of all lane counts.</summary>
    public static bool operator ==(DimensionLayout a, DimensionLayout b) => a.Equals(b);

    /// <summary>Inequality of any lane count.</summary>
    public static bool operator !=(DimensionLayout a, DimensionLayout b) => !a.Equals(b);

    internal void Write(StateWriter w)
    {
        w.WriteInt32(PositionLanes); w.WriteInt32(RotationLanes); w.WriteInt32(LinearLanes); w.WriteInt32(AngularLanes);
        w.WriteInt32(InertiaLanes); w.WriteInt32(MaxManifoldPoints); w.WriteInt32(PayloadFloats);
    }

    internal static DimensionLayout Read(StateReader r)
    {
        return new DimensionLayout(r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32(), r.ReadInt32());
    }
}

/// <summary>
/// Offsets (in floats) inside a contact constraint's payload. A manifold has a header, then up to four points of <see cref="PointStride"/> floats each;
/// 2D uses the first two. Unused points are zero. Engines may use the trailing floats after the points for their own per-manifold data.
/// </summary>
public static class ManifoldLayout
{
    /// <summary>Header: normal x, y, z (0..2), friction (3), restitution (4), rolling resistance (5), then rolling impulse x, y, z (6..8).</summary>
    public const int Header = 9;

    /// <summary>Offset of the normal.</summary>
    public const int Normal = 0;

    /// <summary>Offset of the seven spare floats after four points in a 3D payload; the 3D narrowphase keeps per-pair search hints there between steps. Nothing else reads them.</summary>
    public const int HintOffset = Header + 4 * PointStride;

    /// <summary>Offset of the floats after the hint that the 3D narrowphase uses to reuse a manifold while the two bodies barely move (see <see cref="RecycleFloats"/>). Nothing else reads them.</summary>
    public const int RecycleOffset = HintOffset + 7;

    /// <summary>Floats of the 3D payload kept for manifold reuse: an owner flag, the normal in the owner's frame, the relative pose of B in A when the manifold was made (3 + 4), and per point the arms in each body's own frame and the separation then (7 each).</summary>
    public const int RecycleFloats = 40;

    /// <summary>Offset of the mixed friction coefficient.</summary>
    public const int Friction = 3;

    /// <summary>Offset of the mixed restitution coefficient.</summary>
    public const int Restitution = 4;

    /// <summary>Offset of the mixed rolling resistance.</summary>
    public const int RollingResistance = 5;

    /// <summary>Offset of the accumulated rolling impulse (three floats).</summary>
    public const int RollingImpulse = 6;

    /// <summary>Floats per point: anchor on A (3), anchor on B (3), separation, normal impulse, tangent-1 impulse, tangent-2 impulse, largest normal impulse seen.</summary>
    public const int PointStride = 12;

    /// <summary>Offset of point <paramref name="i"/>'s anchor on body A.</summary>
    public static int AnchorA(int i) => Header + i * PointStride;

    /// <summary>Offset of point <paramref name="i"/>'s anchor on body B.</summary>
    public static int AnchorB(int i) => Header + i * PointStride + 3;

    /// <summary>Offset of point <paramref name="i"/>'s separation (negative when penetrating).</summary>
    public static int Separation(int i) => Header + i * PointStride + 6;

    /// <summary>Offset of point <paramref name="i"/>'s accumulated normal impulse.</summary>
    public static int NormalImpulse(int i) => Header + i * PointStride + 7;

    /// <summary>Offset of point <paramref name="i"/>'s accumulated first tangent impulse.</summary>
    public static int Tangent1Impulse(int i) => Header + i * PointStride + 8;

    /// <summary>Offset of point <paramref name="i"/>'s accumulated second tangent impulse (unused in 2D).</summary>
    public static int Tangent2Impulse(int i) => Header + i * PointStride + 9;

    /// <summary>Offset of point <paramref name="i"/>'s largest normal impulse.</summary>
    public static int MaxNormalImpulse(int i) => Header + i * PointStride + 10;
}

/// <summary>Which categories of events a world records. Disabled kinds cost nothing.</summary>
[Flags]
public enum EventMask : ushort
{
    /// <summary>No events.</summary>
    None = 0,

    /// <summary>A contact constraint was created.</summary>
    ContactBegin = 1 << 0,

    /// <summary>A contact constraint exists at the end of a step and asked for persist events.</summary>
    ContactPersist = 1 << 1,

    /// <summary>A contact constraint was removed.</summary>
    ContactEnd = 1 << 2,

    /// <summary>A sensor overlap began.</summary>
    SensorEnter = 1 << 3,

    /// <summary>A sensor overlap ended.</summary>
    SensorExit = 1 << 4,

    /// <summary>A joint broke.</summary>
    JointBreak = 1 << 5,

    /// <summary>A body fell asleep.</summary>
    Sleep = 1 << 6,

    /// <summary>A body woke.</summary>
    Wake = 1 << 7,

    /// <summary>A contact approached faster than the engine's hit-speed threshold. Not part of <see cref="Default"/>.</summary>
    ContactHit = 1 << 8,

    /// <summary>Everything except per-step persist events and hit events.</summary>
    Default = 0xFF & ~ContactPersist,

    /// <summary>Every kind.</summary>
    All = 0x1FF,
}

/// <summary>Options of a <see cref="WorldCore"/>. All values are inputs to the simulation except <see cref="ThreadCount"/> and <see cref="WorldId"/>, which never change a result and are not saved in snapshots.</summary>
public sealed class WorldOptions
{
    /// <summary>Lane counts of the storage. Default <see cref="DimensionLayout.Spatial"/>.</summary>
    public DimensionLayout Layout { get; set; } = DimensionLayout.Spatial;

    /// <summary>Worker threads for the parallel phases; 1 runs everything on the calling thread. Never decided by the machine; default 1. Results do not depend on it.</summary>
    public int ThreadCount { get; set; } = 1;

    /// <summary>Id stamped into handles to detect handles from another world. At least 1. Default 0 picks the next number from a process-wide counter; hosts that want ids independent of creation order set it.</summary>
    public int WorldId { get; set; }

    /// <summary>Gravity in world units per second squared (the z component is ignored by 2D engines). Default (0, -9.81, 0).</summary>
    public Vector3F Gravity { get; set; } = new(0f, -9.81f, 0f);

    /// <summary>Substeps per step (at least 1). Default 4.</summary>
    public int Substeps { get; set; } = 4;

    /// <summary>Solve passes with bias per substep (at least 1). Default 1.</summary>
    public int VelocityIterations { get; set; } = 1;

    /// <summary>Relax passes without bias per substep (at least 0). Default 1.</summary>
    public int RelaxIterations { get; set; } = 1;

    /// <summary>Whether islands may fall asleep. Default true.</summary>
    public bool SleepEnabled { get; set; } = true;

    /// <summary>Seconds a body must stay at rest before its island may sleep (positive). Default 0.5.</summary>
    public float TimeToSleep { get; set; } = 0.5f;

    /// <summary>Motion metric (engine-defined, typically a speed in units per second) at or below which a body counts as at rest. Default 0.05.</summary>
    public float SleepThreshold { get; set; } = 0.05f;

    /// <summary>Which events are recorded. Default <see cref="EventMask.Default"/>.</summary>
    public EventMask Events { get; set; } = EventMask.Default;

    /// <summary>Checks every value and throws <see cref="ArgumentOutOfRangeException"/> naming the property and the limit.</summary>
    public void Validate()
    {
        if (!Layout.IsSupported) { throw new ArgumentOutOfRangeException(nameof(Layout), "Layout must be DimensionLayout.Planar or DimensionLayout.Spatial."); }
        if (ThreadCount < 1 || ThreadCount > 1024) { throw new ArgumentOutOfRangeException(nameof(ThreadCount), ThreadCount, "ThreadCount must be in 1..1024."); }
        if (WorldId < 0) { throw new ArgumentOutOfRangeException(nameof(WorldId), WorldId, "WorldId must be 0 (automatic) or at least 1."); }
        if (!float.IsFinite(Gravity.X) || !float.IsFinite(Gravity.Y) || !float.IsFinite(Gravity.Z)) { throw new ArgumentOutOfRangeException(nameof(Gravity), "Gravity must be finite."); }
        if (Substeps < 1 || Substeps > 256) { throw new ArgumentOutOfRangeException(nameof(Substeps), Substeps, "Substeps must be in 1..256."); }
        if (VelocityIterations < 1 || VelocityIterations > 256) { throw new ArgumentOutOfRangeException(nameof(VelocityIterations), VelocityIterations, "VelocityIterations must be in 1..256."); }
        if (RelaxIterations < 0 || RelaxIterations > 256) { throw new ArgumentOutOfRangeException(nameof(RelaxIterations), RelaxIterations, "RelaxIterations must be in 0..256."); }
        if (!(TimeToSleep > 0f) || !float.IsFinite(TimeToSleep)) { throw new ArgumentOutOfRangeException(nameof(TimeToSleep), TimeToSleep, "TimeToSleep must be positive and finite."); }
        if (!(SleepThreshold >= 0f) || !float.IsFinite(SleepThreshold)) { throw new ArgumentOutOfRangeException(nameof(SleepThreshold), SleepThreshold, "SleepThreshold must be non-negative and finite."); }
    }

    internal void CopySimulationValuesFrom(WorldOptions o)
    {
        Layout = o.Layout; Gravity = o.Gravity; Substeps = o.Substeps; VelocityIterations = o.VelocityIterations; RelaxIterations = o.RelaxIterations;
        SleepEnabled = o.SleepEnabled; TimeToSleep = o.TimeToSleep; SleepThreshold = o.SleepThreshold; Events = o.Events;
    }

    internal WorldOptions Clone() => (WorldOptions)MemberwiseClone();

    internal void Write(StateWriter w)
    {
        Layout.Write(w);
        w.WriteSingle(Gravity.X); w.WriteSingle(Gravity.Y); w.WriteSingle(Gravity.Z);
        w.WriteInt32(Substeps); w.WriteInt32(VelocityIterations); w.WriteInt32(RelaxIterations);
        w.WriteBool(SleepEnabled); w.WriteSingle(TimeToSleep); w.WriteSingle(SleepThreshold);
        w.WriteUInt32((uint)Events);
    }

    /// <summary>Reads the saved options into a copy of <paramref name="current"/> (keeping its thread count and world id) and validates them.</summary>
    internal static WorldOptions Read(StateReader r, WorldOptions current)
    {
        var o = current.Clone();
        o.Layout = DimensionLayout.Read(r);
        o.Gravity = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        o.Substeps = r.ReadInt32(); o.VelocityIterations = r.ReadInt32(); o.RelaxIterations = r.ReadInt32();
        o.SleepEnabled = r.ReadBool(); o.TimeToSleep = r.ReadSingle(); o.SleepThreshold = r.ReadSingle();
        uint ev = r.ReadUInt32();
        if ((ev & ~0x1FFu) != 0) { throw r.Error("unknown event mask bits " + ev.ToString(CultureInfo.InvariantCulture)); }
        o.Events = (EventMask)ev;
        try
        {
            o.Validate();
        }
        catch (ArgumentException e)
        {
            throw r.Error("invalid saved options: " + e.Message);
        }

        return o;
    }
}
