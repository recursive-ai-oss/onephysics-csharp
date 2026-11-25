using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// Options for <see cref="PhysicsWorld3D.Settle"/>. Tolerances are in metres, seconds and radians. Every field has a default that settles a metre-scale scene; the options are copied when Settle starts.
/// </summary>
public sealed class SettleOptions3D
{
    /// <summary>This set of options with the tolerances that were never set (metre values: SpeedTolerance, AccelerationTolerance, SupportTolerance) at their defaults for a world of the given unit. The world applies this itself.</summary>
    /// <param name="unit">The world's length unit.</param>
    public SettleOptions3D WithDefaultsFor(LengthUnit unit)
    {
        var s = new UnitScaler(unit.Scale, false);
        var c = (SettleOptions3D)MemberwiseClone();
        if (s.IsIdentity) { return c; }
        float speedtolerance = SpeedTolerance; s.D(ref speedtolerance, 0.01f, 1); c.SpeedTolerance = speedtolerance;
        float accelerationtolerance = AccelerationTolerance; s.D(ref accelerationtolerance, 0.25f, 1); c.AccelerationTolerance = accelerationtolerance;
        float supporttolerance = SupportTolerance; s.D(ref supporttolerance, 0.005f, 1); c.SupportTolerance = supporttolerance;
        return c;
    }

    internal const int EncodedSize = 88;

    /// <summary>Fixed step in seconds, finite and positive. Default 1/60.</summary>
    public float Dt { get; set; } = 1f / 60f;
    /// <summary>Step budget, at least 1. The run stops when it converges or has taken this many steps. Default 1200 (twenty seconds at 1/60).</summary>
    public int MaxSteps { get; set; } = 1200;
    /// <summary>Largest linear speed, in m/s, a body or particle may keep and still count as at rest. Default 0.01.</summary>
    public float SpeedTolerance { get; set; } = 0.01f;
    /// <summary>Largest angular speed, in rad/s, a body may keep. Default 0.02.</summary>
    public float AngularSpeedTolerance { get; set; } = 0.02f;
    /// <summary>Largest change of linear velocity per second, in m/s², a body may show over a step. A body held by contacts shows about zero; a ball at the top of a bounce shows gravity. Default 0.25.</summary>
    public float AccelerationTolerance { get; set; } = 0.25f;
    /// <summary>Consecutive steps all tolerances must hold for the world to count as converged, at least 1. Default 20.</summary>
    public int StableSteps { get; set; } = 20;
    /// <summary>Extra linear damping in 1/s applied to awake dynamic bodies, soft-body and rod particles after every step (velocity divided by 1 + c dt). Default 0, finite and nonnegative.</summary>
    public float ExtraLinearDamping { get; set; }
    /// <summary>Extra angular damping in 1/s for rigid bodies. Default 0, finite and nonnegative.</summary>
    public float ExtraAngularDamping { get; set; }
    /// <summary>Bodies whose centre of mass lies outside this box are reported and are not judged for convergence. Null (the default) means no bounds.</summary>
    public (Vector3D Min, Vector3D Max)? Bounds { get; set; }
    /// <summary>A contact counts as touching for support when a point's separation is at most this many metres. Default 0.005.</summary>
    public float SupportTolerance { get; set; } = 0.005f;
    /// <summary>When true the report carries per-body arrays (residuals, unsupported bodies, bodies out of bounds). Default false: the report allocates nothing beyond itself.</summary>
    public bool ReportBodies { get; set; }

    /// <summary>Checks every field.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A field is out of range, or the bounds are not finite and ordered.</exception>
    public void Validate()
    {
        if (!(Dt > 0f) || !float.IsFinite(Dt)) throw new ArgumentOutOfRangeException(nameof(Dt), Dt, "Dt must be finite and positive.");
        if (MaxSteps < 1) throw new ArgumentOutOfRangeException(nameof(MaxSteps), MaxSteps, "MaxSteps must be at least 1.");
        if (StableSteps < 1) throw new ArgumentOutOfRangeException(nameof(StableSteps), StableSteps, "StableSteps must be at least 1.");
        Check(SpeedTolerance, nameof(SpeedTolerance)); Check(AngularSpeedTolerance, nameof(AngularSpeedTolerance)); Check(AccelerationTolerance, nameof(AccelerationTolerance));
        Check(ExtraLinearDamping, nameof(ExtraLinearDamping)); Check(ExtraAngularDamping, nameof(ExtraAngularDamping)); Check(SupportTolerance, nameof(SupportTolerance));
        if (Bounds is { } b)
        {
            var (lo, hi) = b;
            if (!(double.IsFinite(lo.X) && double.IsFinite(lo.Y) && double.IsFinite(lo.Z) && double.IsFinite(hi.X) && double.IsFinite(hi.Y) && double.IsFinite(hi.Z) && lo.X <= hi.X && lo.Y <= hi.Y && lo.Z <= hi.Z))
                throw new ArgumentOutOfRangeException(nameof(Bounds), "Bounds must be finite with Min no greater than Max on every axis.");
        }
    }

    private static void Check(float v, string name) { if (!(v >= 0f) || !float.IsFinite(v)) throw new ArgumentOutOfRangeException(name, v, name + " must be finite and nonnegative."); }

    internal SettleParams ToParams() => new()
    {
        Dt = Dt, MaxSteps = MaxSteps, SpeedTolerance = SpeedTolerance, AngularSpeedTolerance = AngularSpeedTolerance, AccelerationTolerance = AccelerationTolerance,
        StableSteps = StableSteps, ExtraLinearDamping = ExtraLinearDamping, ExtraAngularDamping = ExtraAngularDamping, Detail = ReportBodies, SupportTolerance = SupportTolerance,
        HasBounds = Bounds.HasValue,
        BoundsMin = Bounds is { } b ? new[] { b.Min.X, b.Min.Y, b.Min.Z } : null,
        BoundsMax = Bounds is { } c ? new[] { c.Max.X, c.Max.Y, c.Max.Z } : null,
    };

    // The replay record's options block, little-endian in declaration order.
    internal void Write(Span<byte> d)
    {
        d.Slice(0, EncodedSize).Clear();
        BinaryPrimitives.WriteSingleLittleEndian(d, Dt);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(4), MaxSteps);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(8), SpeedTolerance);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(12), AngularSpeedTolerance);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(16), AccelerationTolerance);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(20), StableSteps);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(24), ExtraLinearDamping);
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(28), ExtraAngularDamping);
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(32), (ReportBodies ? 1 : 0) | (Bounds.HasValue ? 2 : 0));
        BinaryPrimitives.WriteSingleLittleEndian(d.Slice(36), SupportTolerance);
        if (Bounds is { } b)
        {
            BinaryPrimitives.WriteDoubleLittleEndian(d.Slice(40), b.Min.X); BinaryPrimitives.WriteDoubleLittleEndian(d.Slice(48), b.Min.Y); BinaryPrimitives.WriteDoubleLittleEndian(d.Slice(56), b.Min.Z);
            BinaryPrimitives.WriteDoubleLittleEndian(d.Slice(64), b.Max.X); BinaryPrimitives.WriteDoubleLittleEndian(d.Slice(72), b.Max.Y); BinaryPrimitives.WriteDoubleLittleEndian(d.Slice(80), b.Max.Z);
        }
    }

    internal static SettleOptions3D Read(ReadOnlySpan<byte> d)
    {
        if (d.Length != EncodedSize) throw new ArgumentException("A 3D settle options block is " + EncodedSize + " bytes.");
        int flags = BinaryPrimitives.ReadInt32LittleEndian(d.Slice(32));
        if ((flags & ~3) != 0) throw new ArgumentException("Unknown settle option flags.");
        var o = new SettleOptions3D
        {
            Dt = BinaryPrimitives.ReadSingleLittleEndian(d), MaxSteps = BinaryPrimitives.ReadInt32LittleEndian(d.Slice(4)),
            SpeedTolerance = BinaryPrimitives.ReadSingleLittleEndian(d.Slice(8)), AngularSpeedTolerance = BinaryPrimitives.ReadSingleLittleEndian(d.Slice(12)),
            AccelerationTolerance = BinaryPrimitives.ReadSingleLittleEndian(d.Slice(16)), StableSteps = BinaryPrimitives.ReadInt32LittleEndian(d.Slice(20)),
            ExtraLinearDamping = BinaryPrimitives.ReadSingleLittleEndian(d.Slice(24)), ExtraAngularDamping = BinaryPrimitives.ReadSingleLittleEndian(d.Slice(28)),
            ReportBodies = (flags & 1) != 0, SupportTolerance = BinaryPrimitives.ReadSingleLittleEndian(d.Slice(36)),
        };
        if ((flags & 2) != 0)
        {
            o.Bounds = (new Vector3D(BinaryPrimitives.ReadDoubleLittleEndian(d.Slice(40)), BinaryPrimitives.ReadDoubleLittleEndian(d.Slice(48)), BinaryPrimitives.ReadDoubleLittleEndian(d.Slice(56))),
                        new Vector3D(BinaryPrimitives.ReadDoubleLittleEndian(d.Slice(64)), BinaryPrimitives.ReadDoubleLittleEndian(d.Slice(72)), BinaryPrimitives.ReadDoubleLittleEndian(d.Slice(80))));
        }
        o.Validate();
        return o;
    }
}
