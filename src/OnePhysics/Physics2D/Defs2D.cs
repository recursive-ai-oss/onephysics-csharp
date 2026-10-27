using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Friction, restitution and rolling resistance of a shape.</summary>
public readonly struct Material2D : IEquatable<Material2D>
{
    /// <summary>Creates a material. Friction and rolling resistance must be at least 0, restitution within 0..1.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or outside its range.</exception>
    public Material2D(float friction, float restitution = 0f, float rollingResistance = 0f)
    {
        if (!(friction >= 0f) || !float.IsFinite(friction)) { throw new ArgumentOutOfRangeException(nameof(friction), friction, "Friction must be finite and at least 0."); }
        if (!(restitution >= 0f && restitution <= 1f)) { throw new ArgumentOutOfRangeException(nameof(restitution), restitution, "Restitution must be within 0..1."); }
        if (!(rollingResistance >= 0f) || !float.IsFinite(rollingResistance)) { throw new ArgumentOutOfRangeException(nameof(rollingResistance), rollingResistance, "Rolling resistance must be finite and at least 0."); }
        Friction = friction;
        Restitution = restitution;
        RollingResistance = rollingResistance;
    }

    /// <summary>Coulomb friction coefficient. Default for <see cref="ShapeDef"/> is 0.6.</summary>
    public float Friction { get; }

    /// <summary>Bounciness: 0 absorbs all approach speed, 1 reflects it.</summary>
    public float Restitution { get; }

    /// <summary>Resistance to rolling, as a coefficient on the normal load (a torque limit per unit of normal impulse).</summary>
    public float RollingResistance { get; }

    /// <summary>Slippery ice-like surface: friction 0.05.</summary>
    public static Material2D Ice => new(0.05f);

    /// <summary>Rubber: friction 1, restitution 0.8.</summary>
    public static Material2D Rubber => new(1f, 0.8f);

    /// <summary>Wood: friction 0.5, restitution 0.1.</summary>
    public static Material2D Wood => new(0.5f, 0.1f);

    /// <summary>Metal: friction 0.4, restitution 0.05.</summary>
    public static Material2D Metal => new(0.4f, 0.05f);

    /// <summary>Equal in every field.</summary>
    public bool Equals(Material2D other) => Friction == other.Friction && Restitution == other.Restitution && RollingResistance == other.RollingResistance;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Material2D m && Equals(m);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked((BitConverter.SingleToInt32Bits(Friction) * 31 + BitConverter.SingleToInt32Bits(Restitution)) * 31 + BitConverter.SingleToInt32Bits(RollingResistance));

    /// <summary>Field-wise equality.</summary>
    public static bool operator ==(Material2D a, Material2D b) => a.Equals(b);

    /// <summary>Field-wise inequality.</summary>
    public static bool operator !=(Material2D a, Material2D b) => !a.Equals(b);
}

/// <summary>Combines the materials of two touching shapes into the values a contact uses. Must be a pure function of its arguments.</summary>
/// <param name="a">The material of the first shape (the one with the lower shape index).</param>
/// <param name="b">The material of the second shape.</param>
public delegate Material2D MaterialMixer(Material2D a, Material2D b);

/// <summary>Decides whether two shapes may collide, beyond the category/mask/group rule. Called once when the pair is first found; must be a pure function of the shapes' static properties.</summary>
public delegate bool PairPredicate(Shape a, Shape b);

/// <summary>Options of a <see cref="PhysicsWorld2D"/>. Every value that influences the simulation is saved in snapshots and covered by the world hash; only <see cref="ThreadCount"/> and <see cref="WorldId"/> are not.</summary>
public sealed class WorldOptions2D
{
    /// <summary>Gravity in metres per second squared. Default (0, -9.81).</summary>
    public Vector2F Gravity { get; set; } = new(0f, -9.81f);

    /// <summary>Substeps per step, at least 1. Default 4: soft-step substeps of dt/4.</summary>
    public int Substeps { get; set; } = 4;

    /// <summary>Biased solve passes per substep, at least 1. Default 1.</summary>
    public int VelocityIterations { get; set; } = 1;

    /// <summary>Bias-free relax passes per substep, at least 0. Default 1.</summary>
    public int RelaxIterations { get; set; } = 1;

    /// <summary>Worker threads for the parallel phases; never changes a result. Default 1.</summary>
    public int ThreadCount { get; set; } = 1;

    /// <summary>Id stamped into handles; 0 takes the next number of a process-wide counter. Not saved.</summary>
    public int WorldId { get; set; }

    /// <summary>Whether resting islands fall asleep. Default true.</summary>
    public bool SleepEnabled { get; set; } = true;

    /// <summary>Seconds at rest before an island sleeps. Default 0.5.</summary>
    public float TimeToSleep { get; set; } = 0.5f;

    /// <summary>Speed (including the rim speed of rotation) at or below which a body counts as resting, in metres per second. Default 0.05.</summary>
    public float SleepThreshold { get; set; } = 0.05f;

    /// <summary>Which events the world records. Default: everything but per-step contact persist events (needed for <c>OnCollisionStay</c>).</summary>
    public EventMask Events { get; set; } = EventMask.Default;

    /// <summary>Natural frequency of soft contacts in Hz, capped at a quarter of the substep rate. Default 36.</summary>
    public float ContactHertz { get; set; } = 36f;

    /// <summary>Damping ratio of soft contacts. Default 10 (heavily overdamped).</summary>
    public float ContactDampingRatio { get; set; } = 10f;

    /// <summary>Largest speed at which penetration is pushed out, metres per second. Default 3.</summary>
    public float PushOutVelocity { get; set; } = 3f;

    /// <summary>Penetration in metres that the soft contact tolerates before it starts pushing out. Default 0: any penetration is pushed out. A positive slop lets contacts rest quietly but every layer of a stack then sinks by about that much.</summary>
    public float ContactSlop { get; set; }

    /// <summary>Natural frequency in Hz of the soft rigid rows of joints (position error correction), capped at a quarter of the substep rate. Default 60.</summary>
    public float JointHertz { get; set; } = 60f;

    /// <summary>Damping ratio of the soft rigid rows of joints. Default 2.</summary>
    public float JointDampingRatio { get; set; } = 2f;

    /// <summary>Approach speed below which restitution is ignored, metres per second. Default 1.</summary>
    public float RestitutionThreshold { get; set; } = 1f;

    /// <summary>Speed limit applied to every dynamic body each substep, metres per second. Default 400.</summary>
    public float MaxLinearSpeed { get; set; } = 400f;

    /// <summary>Largest rotation per substep in radians; faster spin is clamped. Default 0.25 pi (an eighth of a turn).</summary>
    public float MaxRotationPerSubstep { get; set; } = 0.7853982f;

    /// <summary>Whether fast bodies get continuous collision against static shapes. Default true.</summary>
    public bool ContinuousEnabled { get; set; } = true;

    /// <summary>How far a moving body's broadphase box extends past its shapes, in metres. Fixed at world creation (not changed by restoring a snapshot). Default 0.1.</summary>
    public double BroadphaseMargin { get; set; } = 0.1;

    /// <summary>Replaces the default material combine rule (geometric mean of friction, maximum of restitution and of rolling resistance). Null keeps the default.</summary>
    public MaterialMixer? MaterialMixer { get; set; }

    /// <summary>Extra rule applied when a pair of shapes is first found; return false to keep them from colliding. Null accepts every pair the category/mask/group rule accepts.</summary>
    public PairPredicate? PairFilter { get; set; }

    internal WorldOptions2D Clone() => (WorldOptions2D)MemberwiseClone();

    /// <summary>Checks every value; throws <see cref="ArgumentOutOfRangeException"/> naming the first bad one.</summary>
    internal void Validate()
    {
        if (!float.IsFinite(Gravity.X) || !float.IsFinite(Gravity.Y)) { throw new ArgumentOutOfRangeException(nameof(Gravity), "Gravity must be finite."); }
        Positive(ContactHertz, nameof(ContactHertz), true);
        Positive(ContactDampingRatio, nameof(ContactDampingRatio), true);
        Positive(PushOutVelocity, nameof(PushOutVelocity), true);
        Positive(ContactSlop, nameof(ContactSlop), true);
        Positive(RestitutionThreshold, nameof(RestitutionThreshold), true);
        Positive(JointHertz, nameof(JointHertz), true);
        Positive(JointDampingRatio, nameof(JointDampingRatio), true);
        Positive(MaxLinearSpeed, nameof(MaxLinearSpeed), false);
        Positive(MaxRotationPerSubstep, nameof(MaxRotationPerSubstep), false);
        if (!(BroadphaseMargin >= 0.0 && BroadphaseMargin <= 100.0)) { throw new ArgumentOutOfRangeException(nameof(BroadphaseMargin), BroadphaseMargin, "BroadphaseMargin must be within 0..100."); }
    }

    private static void Positive(float v, string name, bool allowZero)
    {
        if (!float.IsFinite(v) || v < 0f || (!allowZero && v == 0f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and " + (allowZero ? "at least 0." : "positive.")); }
    }

    internal WorldOptions ToCore()
    {
        return new WorldOptions
        {
            Layout = DimensionLayout.Planar,
            ThreadCount = ThreadCount,
            WorldId = WorldId,
            Gravity = new Vector3F(Gravity.X, Gravity.Y, 0f),
            Substeps = Substeps,
            VelocityIterations = VelocityIterations,
            RelaxIterations = RelaxIterations,
            SleepEnabled = SleepEnabled,
            TimeToSleep = TimeToSleep,
            SleepThreshold = SleepThreshold,
            Events = Events,
        };
    }
}

/// <summary>Creation parameters of a body. All fields have defaults: a dynamic body at the origin, at rest, awake.</summary>
public struct BodyDef
{
    /// <summary>Creates a definition with the defaults.</summary>
    public BodyDef()
    {
        Type = BodyType.Dynamic;
        GravityScale = 1f;
        AllowSleep = true;
    }

    /// <summary>Static (never moves), kinematic (moves at its velocity, not pushed) or dynamic. Default dynamic.</summary>
    public BodyType Type { get; set; }

    /// <summary>Position of the body origin, metres, within +-2^21 of the world origin. Default (0, 0).</summary>
    public Vector2D Position { get; set; }

    /// <summary>Rotation in radians. Default 0.</summary>
    public float Angle { get; set; }

    /// <summary>Velocity of the centre of mass, metres per second. Default zero.</summary>
    public Vector2F LinearVelocity { get; set; }

    /// <summary>Angular velocity, radians per second. Default 0.</summary>
    public float AngularVelocity { get; set; }

    /// <summary>Linear damping, 1/s (velocity decays as 1/(1 + dt * damping) per substep). Default 0.</summary>
    public float LinearDamping { get; set; }

    /// <summary>Angular damping, 1/s. Default 0.</summary>
    public float AngularDamping { get; set; }

    /// <summary>Multiplier on world gravity for this body. Default 1.</summary>
    public float GravityScale { get; set; }

    /// <summary>When true the body never rotates (infinite rotational inertia). Default false.</summary>
    public bool FixedRotation { get; set; }

    /// <summary>Continuous collision against dynamic bodies as well as static ones (static-only is already on for every fast body). Default false.</summary>
    public bool IsBullet { get; set; }

    /// <summary>When false the body keeps its island awake forever. Default true.</summary>
    public bool AllowSleep { get; set; }

    /// <summary>Create the body asleep; it wakes when touched or woken. Ignored for static and kinematic bodies. Default false.</summary>
    public bool StartAsleep { get; set; }
}

/// <summary>Creation parameters of a shape. Defaults: density 1 kg/m², friction 0.6, restitution 0, collides with everything.</summary>
public struct ShapeDef
{
    /// <summary>Creates a definition with the defaults.</summary>
    public ShapeDef()
    {
        Density = 1f;
        Friction = 0.6f;
        CategoryBits = 1u;
        MaskBits = uint.MaxValue;
    }

    /// <summary>Mass per area, kg/m². At least 0; a body whose shapes all have density 0 gets unit mass and inertia. Default 1.</summary>
    public float Density { get; set; }

    /// <summary>Friction coefficient, at least 0. Default 0.6.</summary>
    public float Friction { get; set; }

    /// <summary>Restitution within 0..1. Default 0.</summary>
    public float Restitution { get; set; }

    /// <summary>Rolling resistance, at least 0. Default 0.</summary>
    public float RollingResistance { get; set; }

    /// <summary>Which categories this shape belongs to (bit set). Default 1.</summary>
    public uint CategoryBits { get; set; }

    /// <summary>Which categories this shape collides with. Two shapes collide when each one's mask includes the other's category. Default all.</summary>
    public uint MaskBits { get; set; }

    /// <summary>Shapes with the same non-zero group always collide when the group is positive and never when it is negative, overriding category and mask. Default 0.</summary>
    public int GroupIndex { get; set; }

    /// <summary>Sets friction, restitution and rolling resistance from a material.</summary>
    public ShapeDef WithMaterial(Material2D m)
    {
        var d = this;
        d.Friction = m.Friction;
        d.Restitution = m.Restitution;
        d.RollingResistance = m.RollingResistance;
        return d;
    }
}

/// <summary>What a collision callback is told: the other body and shapes, and where and how they touch.</summary>
public readonly struct Collision
{
    internal Collision(Body self, Body other, Shape selfShape, Shape otherShape, Vector2D point, Vector2F normal, bool hasPoint)
    {
        Body = self;
        Other = other;
        Shape = selfShape;
        OtherShape = otherShape;
        Point = point;
        Normal = normal;
        HasPoint = hasPoint;
    }

    /// <summary>The body the callback belongs to.</summary>
    public Body Body { get; }

    /// <summary>The body it touched.</summary>
    public Body Other { get; }

    /// <summary>The shape of <see cref="Body"/> involved.</summary>
    public Shape Shape { get; }

    /// <summary>The shape of <see cref="Other"/> involved.</summary>
    public Shape OtherShape { get; }

    /// <summary>A world position on the contact (first manifold point) at the end of the step; zero when <see cref="HasPoint"/> is false.</summary>
    public Vector2D Point { get; }

    /// <summary>Contact normal pointing from <see cref="Body"/> towards <see cref="Other"/>; zero when <see cref="HasPoint"/> is false.</summary>
    public Vector2F Normal { get; }

    /// <summary>False for exit callbacks and for contacts that began and ended inside one step.</summary>
    public bool HasPoint { get; }
}

/// <summary>
/// A moving body's bounds left the range the broadphase supports (±2^22 from the world origin), or its position became not finite. <see cref="PhysicsWorld2D.Step"/> raises it before it changes
/// anything, so the world is intact: the host moves the body back, destroys it or shifts the origin, then steps again. Until then every step raises it again.
/// </summary>
public sealed class BodyOutOfBoundsException : PhysicsException
{
    /// <summary>The offending body.</summary>
    public BodyHandle Body { get; }

    /// <summary>Creates the exception for <paramref name="body"/>; the message gives its position.</summary>
    public BodyOutOfBoundsException(BodyHandle body, string message) : base(message) { Body = body; }
}
