using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Surface properties of a shape: friction, restitution and rolling resistance. Presets are starting points, not measurements.</summary>
public readonly struct Material3D : IEquatable<Material3D>
{
    /// <summary>Creates a material.</summary>
    /// <param name="friction">Coulomb friction coefficient; finite and at least 0.</param>
    /// <param name="restitution">Bounciness in 0..1.</param>
    /// <param name="rollingResistance">Resistance to rolling, as a fraction of the normal impulse times 1 m; finite and at least 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside its range.</exception>
    public Material3D(float friction, float restitution = 0f, float rollingResistance = 0f)
    {
        if (!(friction >= 0f) || !float.IsFinite(friction)) { throw new ArgumentOutOfRangeException(nameof(friction), friction, "Friction must be finite and at least 0."); }
        if (!(restitution >= 0f && restitution <= 1f)) { throw new ArgumentOutOfRangeException(nameof(restitution), restitution, "Restitution must be within 0..1."); }
        if (!(rollingResistance >= 0f) || !float.IsFinite(rollingResistance)) { throw new ArgumentOutOfRangeException(nameof(rollingResistance), rollingResistance, "Rolling resistance must be finite and at least 0."); }
        Friction = friction;
        Restitution = restitution;
        RollingResistance = rollingResistance;
    }

    /// <summary>Coulomb friction coefficient.</summary>
    public float Friction { get; }

    /// <summary>Bounciness in 0..1.</summary>
    public float Restitution { get; }

    /// <summary>Rolling resistance coefficient.</summary>
    public float RollingResistance { get; }

    /// <summary>Very slippery.</summary>
    public static Material3D Ice => new(0.05f);

    /// <summary>Grippy and bouncy.</summary>
    public static Material3D Rubber => new(1f, 0.8f);

    /// <summary>Moderate friction, a little bounce.</summary>
    public static Material3D Wood => new(0.5f, 0.1f);

    /// <summary>Moderate friction, almost no bounce.</summary>
    public static Material3D Metal => new(0.4f, 0.05f);

    /// <inheritdoc />
    public bool Equals(Material3D other) => Friction == other.Friction && Restitution == other.Restitution && RollingResistance == other.RollingResistance;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is Material3D m && Equals(m);

    /// <inheritdoc />
    public override int GetHashCode() => unchecked((BitConverter.SingleToInt32Bits(Friction) * 31 + BitConverter.SingleToInt32Bits(Restitution)) * 31 + BitConverter.SingleToInt32Bits(RollingResistance));

    /// <summary>Equality.</summary>
    public static bool operator ==(Material3D a, Material3D b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(Material3D a, Material3D b) => !a.Equals(b);
}

/// <summary>Combines the materials of two touching shapes (the lower shape slot first). Must be a pure function of its arguments.</summary>
public delegate Material3D MaterialMixer(Material3D a, Material3D b);

/// <summary>Decides whether two shapes whose bounds overlap should collide at all. Called while pairs are created, never for existing pairs.</summary>
public delegate bool PairPredicate(Shape a, Shape b);

/// <summary>World configuration. Defaults suit human-scale scenes in metres, seconds and kilograms.</summary>
public sealed class WorldOptions3D
{
    /// <summary>Gravity in m/s². Default (0, -9.81, 0).</summary>
    public Vector3F Gravity { get; set; } = new(0f, -9.81f, 0f);

    /// <summary>Substeps per step, 1..256. Default 4.</summary>
    public int Substeps { get; set; } = 4;

    /// <summary>Biased solver iterations per substep, 1..256. Default 1.</summary>
    public int VelocityIterations { get; set; } = 1;

    /// <summary>Relaxation iterations per substep, 0..256. Default 1.</summary>
    public int RelaxIterations { get; set; } = 1;

    /// <summary>Threads used by a step, 1..1024. The result never depends on it. Default 1.</summary>
    public int ThreadCount { get; set; } = 1;

    /// <summary>World id used in handles; 0 picks one. Default 0.</summary>
    public int WorldId { get; set; }

    /// <summary>Whether resting bodies fall asleep. Default true.</summary>
    public bool SleepEnabled { get; set; } = true;

    /// <summary>Seconds a body must stay below the threshold to sleep. Default 0.5.</summary>
    public float TimeToSleep { get; set; } = 0.5f;

    /// <summary>Speed (m/s, linear plus rotation at the farthest point) under which a body counts as resting. Default 0.05.</summary>
    public float SleepThreshold { get; set; } = 0.05f;

    /// <summary>Which events are recorded. Default: everything but contact persist.</summary>
    public EventMask Events { get; set; } = EventMask.Default;

    /// <summary>Contact stiffness in Hz (capped at a quarter of the substep rate). Default 30.</summary>
    public float ContactHertz { get; set; } = 30f;

    /// <summary>Contact damping ratio. Default 10.</summary>
    public float ContactDampingRatio { get; set; } = 10f;

    /// <summary>Largest speed at which penetration is pushed out, m/s. Default 3.</summary>
    public float PushOutVelocity { get; set; } = 3f;

    /// <summary>Penetration tolerated before the soft contact pushes, metres. Default 0.</summary>
    public float ContactSlop { get; set; }

    /// <summary>Approach speed under which restitution is ignored, m/s. Default 1.</summary>
    public float RestitutionThreshold { get; set; } = 1f;

    /// <summary>Speed limit, m/s. Default 400.</summary>
    public float MaxLinearSpeed { get; set; } = 400f;

    /// <summary>Rotation limit per substep, radians. Default π/4.</summary>
    public float MaxRotationPerSubstep { get; set; } = 0.7853982f;

    /// <summary>Whether fast bodies are stopped at static geometry. Default true.</summary>
    public bool ContinuousEnabled { get; set; } = true;

    /// <summary>Stiffness of the rigid rows of joints in Hz (capped at a quarter of the substep rate). Default 60.</summary>
    public float JointHertz { get; set; } = 60f;

    /// <summary>Damping ratio of the rigid rows of joints. Default 2.</summary>
    public float JointDampingRatio { get; set; } = 2f;

    /// <summary>Distance at which surfaces get a speculative contact, metres. Default 0.02.</summary>
    public float SpeculativeDistance { get; set; } = 0.02f;

    /// <summary>Margin of the broadphase boxes, metres. Fixed at construction. Default 0.1.</summary>
    public double BroadphaseMargin { get; set; } = 0.1;

    /// <summary>Replaces the default material combination (friction: geometric mean; restitution and rolling resistance: maximum). Never saved.</summary>
    public MaterialMixer? MaterialMixer { get; set; }

    /// <summary>Optional veto on new shape pairs. Never saved.</summary>
    public PairPredicate? PairFilter { get; set; }

    internal WorldOptions3D Clone() => (WorldOptions3D)MemberwiseClone();

    /// <summary>Checks every value.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside its range.</exception>
    public void Validate()
    {
        if (!float.IsFinite(Gravity.X) || !float.IsFinite(Gravity.Y) || !float.IsFinite(Gravity.Z)) { throw new ArgumentOutOfRangeException(nameof(Gravity), "Gravity must be finite."); }
        Positive(ContactHertz, nameof(ContactHertz), true);
        Positive(ContactDampingRatio, nameof(ContactDampingRatio), true);
        Positive(PushOutVelocity, nameof(PushOutVelocity), true);
        Positive(ContactSlop, nameof(ContactSlop), true);
        Positive(RestitutionThreshold, nameof(RestitutionThreshold), true);
        Positive(MaxLinearSpeed, nameof(MaxLinearSpeed), false);
        Positive(MaxRotationPerSubstep, nameof(MaxRotationPerSubstep), false);
        Positive(SpeculativeDistance, nameof(SpeculativeDistance), true);
        Positive(JointHertz, nameof(JointHertz), true);
        Positive(JointDampingRatio, nameof(JointDampingRatio), true);
        if (!double.IsFinite(BroadphaseMargin) || BroadphaseMargin < 0.0) { throw new ArgumentOutOfRangeException(nameof(BroadphaseMargin), BroadphaseMargin, "BroadphaseMargin must be finite and at least 0."); }
        ToCore().Validate();
    }

    private static void Positive(float v, string name, bool allowZero)
    {
        if (!float.IsFinite(v) || v < 0f || (!allowZero && v == 0f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and " + (allowZero ? "at least 0." : "positive.")); }
    }

    internal WorldOptions ToCore() => new()
    {
        Layout = DimensionLayout.Spatial,
        ThreadCount = ThreadCount,
        WorldId = WorldId,
        Gravity = Gravity,
        Substeps = Substeps,
        VelocityIterations = VelocityIterations,
        RelaxIterations = RelaxIterations,
        SleepEnabled = SleepEnabled,
        TimeToSleep = TimeToSleep,
        SleepThreshold = SleepThreshold,
        Events = Events,
    };
}

/// <summary>Everything needed to create a body; defaults make a dynamic body at the origin.</summary>
public struct BodyDef
{
    /// <summary>Creates a definition with the defaults.</summary>
    public BodyDef()
    {
        Type = BodyType.Dynamic;
        Rotation = QuaternionF.Identity;
        GravityScale = 1f;
        AllowSleep = true;
    }

    /// <summary>Static, kinematic or dynamic. Default dynamic.</summary>
    public BodyType Type { get; set; }

    /// <summary>Body origin in world coordinates, within ±2^21 of the world origin.</summary>
    public Vector3D Position { get; set; }

    /// <summary>Orientation; any non-zero finite quaternion (normalised on creation). Default identity.</summary>
    public QuaternionF Rotation { get; set; }

    /// <summary>Initial linear velocity, m/s.</summary>
    public Vector3F LinearVelocity { get; set; }

    /// <summary>Initial angular velocity, rad/s.</summary>
    public Vector3F AngularVelocity { get; set; }

    /// <summary>Linear damping, 1/s; finite, at least 0.</summary>
    public float LinearDamping { get; set; }

    /// <summary>Angular damping, 1/s; finite, at least 0.</summary>
    public float AngularDamping { get; set; }

    /// <summary>Multiplier on gravity. Default 1.</summary>
    public float GravityScale { get; set; }

    /// <summary>Locks rotation (infinite inertia).</summary>
    public bool FixedRotation { get; set; }

    /// <summary>Marks a fast body that should get continuous collision. Continuous collision runs for every fast dynamic body regardless.</summary>
    public bool IsBullet { get; set; }

    /// <summary>Whether the body may sleep. Default true.</summary>
    public bool AllowSleep { get; set; }

    /// <summary>Creates a dynamic body asleep.</summary>
    public bool StartAsleep { get; set; }
}

/// <summary>Everything needed to create a shape besides its geometry; defaults suit a wooden object.</summary>
public struct ShapeDef
{
    /// <summary>Creates a definition with the defaults.</summary>
    public ShapeDef()
    {
        Density = 1f;
        Friction = 0.6f;
        CategoryBits = 1u;
        MaskBits = uint.MaxValue;
        LocalRotation = QuaternionF.Identity;
    }

    /// <summary>Density in kg/m³; 0 adds no mass. Default 1.</summary>
    public float Density { get; set; }

    /// <summary>Friction coefficient. Default 0.6.</summary>
    public float Friction { get; set; }

    /// <summary>Restitution in 0..1. Default 0.</summary>
    public float Restitution { get; set; }

    /// <summary>Rolling resistance. Default 0.</summary>
    public float RollingResistance { get; set; }

    /// <summary>Collision category bits. Default 1.</summary>
    public uint CategoryBits { get; set; }

    /// <summary>Categories this shape collides with. Default all.</summary>
    public uint MaskBits { get; set; }

    /// <summary>Non-zero: shapes in one group always (positive) or never (negative) collide, ignoring masks.</summary>
    public int GroupIndex { get; set; }

    /// <summary>Position of the shape in the body's frame.</summary>
    public Vector3F LocalPosition { get; set; }

    /// <summary>Orientation of the shape in the body's frame; any non-zero finite quaternion. Default identity.</summary>
    public QuaternionF LocalRotation { get; set; }

    /// <summary>Copies a material into the definition.</summary>
    public ShapeDef WithMaterial(Material3D m)
    {
        var d = this;
        d.Friction = m.Friction;
        d.Restitution = m.Restitution;
        d.RollingResistance = m.RollingResistance;
        return d;
    }
}

/// <summary>Passed to collision callbacks: who touched whom and where.</summary>
public readonly struct Collision
{
    internal Collision(Body self, Body other, Shape selfShape, Shape otherShape, Vector3D point, Vector3F normal, bool hasPoint)
    {
        Body = self;
        Other = other;
        Shape = selfShape;
        OtherShape = otherShape;
        Point = point;
        Normal = normal;
        HasPoint = hasPoint;
    }

    /// <summary>The body whose callback runs.</summary>
    public Body Body { get; }

    /// <summary>The body it touched.</summary>
    public Body Other { get; }

    /// <summary>The callback body's shape.</summary>
    public Shape Shape { get; }

    /// <summary>The other body's shape.</summary>
    public Shape OtherShape { get; }

    /// <summary>A contact point in world coordinates (the first point of the contact); valid when <see cref="HasPoint"/>.</summary>
    public Vector3D Point { get; }

    /// <summary>Contact normal pointing from <see cref="Body"/> towards <see cref="Other"/>; valid when <see cref="HasPoint"/>.</summary>
    public Vector3F Normal { get; }

    /// <summary>False for end events and when the contact is gone.</summary>
    public bool HasPoint { get; }
}

/// <summary>A body has left the range the engine supports; <see cref="Body"/> names it.</summary>
public sealed class BodyOutOfBoundsException : PhysicsException
{
    /// <summary>The offending body.</summary>
    public BodyHandle Body { get; }

    /// <summary>Creates the exception.</summary>
    public BodyOutOfBoundsException(BodyHandle body, string message) : base(message) { Body = body; }
}
