using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Friction, restitution and rolling resistance of a shape.</summary>
public readonly struct Material2D : IEquatable<Material2D>
{
    /// <summary>Creates a material. Friction and rolling resistance must be at least 0, restitution within 0..1.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or outside its range.</exception>
    public Material2D(float friction, float restitution = 0f, float rollingResistance = 0f)
    {
        if (!(friction >= 0f && friction <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(nameof(friction), friction, "Friction must be within 0..1e15."); }
        if (!(restitution >= 0f && restitution <= 1f)) { throw new ArgumentOutOfRangeException(nameof(restitution), restitution, "Restitution must be within 0..1."); }
        if (!(rollingResistance >= 0f && rollingResistance <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(nameof(rollingResistance), rollingResistance, "Rolling resistance must be within 0..1e15."); }
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

    /// <summary>Pairs of shapes with overlapping boxes the world should have room for (0..16777216). With <see cref="PhysicsWorld2D.Warmup"/> or at creation, the pair table and the per-step pair lists are sized to it so steps within it allocate nothing. 0 (the default) grows on demand; <c>Warmup()</c> then assumes eight per body. Never changes a result; not saved.</summary>
    public int PairCapacity { get; set; }

    /// <summary>Contact constraints (and joints) the world should have room for (0..16777216): constraint storage, colors, solver rows and event buffers. 0 (the default) grows on demand; <c>Warmup()</c> then assumes six per body. Never changes a result; not saved.</summary>
    public int ContactCapacity { get; set; }

    /// <summary>Islands the world should have room for (0..16777216). 0 (the default) grows on demand; <c>Warmup()</c> then assumes one per body. Never changes a result; not saved.</summary>
    public int IslandCapacity { get; set; }

    /// <summary>Hits the query scratch of the thread that calls <c>Warmup()</c> should hold (0..1048576), so the first queries of up to that many hits allocate nothing. 0 keeps the default of 64. Never changes a result; not saved.</summary>
    public int QueryScratchCapacity { get; set; }

    internal const int MaxCapacity = 1 << 24;

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

    /// <summary>Approach speed in metres per second above which a contact raises a hit event (<c>EventMask.ContactHit</c>, <c>OnCollisionHit</c>). Default 1. Saved in snapshots when it differs from the default.</summary>
    public float HitSpeedThreshold { get; set; } = 1f;

    /// <summary>Replaces the default material combine rule (geometric mean of friction, maximum of restitution and of rolling resistance). Null keeps the default.</summary>
    public MaterialMixer? MaterialMixer { get; set; }

    /// <summary>Extra rule applied when a pair of shapes is first found; return false to keep them from colliding. Null accepts every pair the category/mask/group rule accepts.</summary>
    public PairPredicate? PairFilter { get; set; }

    internal WorldOptions2D Clone() => (WorldOptions2D)MemberwiseClone();

    /// <summary>Checks every value; throws <see cref="ArgumentOutOfRangeException"/> naming the first bad one.</summary>
    internal void Validate()
    {
        if (PairCapacity < 0 || PairCapacity > MaxCapacity) { throw new ArgumentOutOfRangeException(nameof(PairCapacity), PairCapacity, "PairCapacity must be in 0..16777216."); }
        if (ContactCapacity < 0 || ContactCapacity > MaxCapacity) { throw new ArgumentOutOfRangeException(nameof(ContactCapacity), ContactCapacity, "ContactCapacity must be in 0..16777216."); }
        if (IslandCapacity < 0 || IslandCapacity > MaxCapacity) { throw new ArgumentOutOfRangeException(nameof(IslandCapacity), IslandCapacity, "IslandCapacity must be in 0..16777216."); }
        if (QueryScratchCapacity < 0 || QueryScratchCapacity > 1 << 20) { throw new ArgumentOutOfRangeException(nameof(QueryScratchCapacity), QueryScratchCapacity, "QueryScratchCapacity must be in 0..1048576."); }
        if (!float.IsFinite(Gravity.X) || !float.IsFinite(Gravity.Y)) { throw new ArgumentOutOfRangeException(nameof(Gravity), "Gravity must be finite."); }
        Positive(ContactHertz, nameof(ContactHertz), true);
        Positive(ContactDampingRatio, nameof(ContactDampingRatio), true);
        Positive(PushOutVelocity, nameof(PushOutVelocity), true);
        Positive(ContactSlop, nameof(ContactSlop), true);
        Positive(HitSpeedThreshold, nameof(HitSpeedThreshold), true);
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
    /// <summary>Static (never moves), kinematic (moves at its velocity, not pushed) or dynamic. Default dynamic.</summary>
    public BodyType Type { readonly get => (BodyType)(_type ^ (byte)(BodyType.Dynamic)); set => _type = (byte)((byte)value ^ (byte)(BodyType.Dynamic)); }

    private byte _type;

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
    public float GravityScale { readonly get => ZeroDefault.Get(_gravityScale, 1f); set => _gravityScale = ZeroDefault.Set(value, 1f); }

    private uint _gravityScale;

    /// <summary>When true the body never rotates (infinite rotational inertia). Default false.</summary>
    public bool FixedRotation { get; set; }

    /// <summary>Continuous collision against dynamic bodies as well as static ones (static-only is already on for every fast body). Default false.</summary>
    public bool IsBullet { get; set; }

    /// <summary>When false the body keeps its island awake forever. Default true.</summary>
    public bool AllowSleep { readonly get => !_allowSleepOff; set => _allowSleepOff = !value; }

    private bool _allowSleepOff;

    /// <summary>Create the body asleep; it wakes when touched or woken. Ignored for static and kinematic bodies. Default false.</summary>
    public bool StartAsleep { get; set; }
}

/// <summary>Creation parameters of a shape. Defaults: density 1 kg/m², friction 0.6, restitution 0, collides with everything.</summary>
public struct ShapeDef
{
    /// <summary>Mass per area, kg/m². At least 0; a body whose shapes all have density 0 gets unit mass and inertia. Default 1. A dynamic body's total mass, and its inertia when not zero, must lie within 1e-18..1e30 so every inverse stays finite; a shape that would take the body outside that range is refused with an <see cref="ArgumentOutOfRangeException"/> naming the mass and inertia, and the body is unchanged.</summary>
    public float Density { readonly get => ZeroDefault.Get(_density, 1f); set => _density = ZeroDefault.Set(value, 1f); }

    private uint _density;

    /// <summary>Friction coefficient, 0 to 1e15. Default 0.6.</summary>
    public float Friction { readonly get => ZeroDefault.Get(_friction, 0.6f); set => _friction = ZeroDefault.Set(value, 0.6f); }

    private uint _friction;

    /// <summary>Restitution within 0..1. Default 0.</summary>
    public float Restitution { get; set; }

    /// <summary>Rolling resistance, at least 0. Default 0.</summary>
    public float RollingResistance { get; set; }

    /// <summary>Which categories this shape belongs to (bit set). Default 1.</summary>
    public uint CategoryBits { readonly get => _categoryBits ^ (1u); set => _categoryBits = value ^ (1u); }

    private uint _categoryBits;

    /// <summary>Which categories this shape collides with. Two shapes collide when each one's mask includes the other's category. Default all.</summary>
    public uint MaskBits { readonly get => _maskBits ^ (uint.MaxValue); set => _maskBits = value ^ (uint.MaxValue); }

    private uint _maskBits;

    /// <summary>Shapes with the same non-zero group always collide when the group is positive and never when it is negative, overriding category and mask. Default 0.</summary>
    public int GroupIndex { get; set; }

    /// <summary>
    /// A sensor (trigger) shape detects overlaps and produces <c>OnTriggerEnter</c> and <c>OnTriggerExit</c> but no contact response, and adds no mass to its body. Sensors do not detect other sensors.
    /// Fixed at creation. Default false.
    /// </summary>
    public bool IsSensor { get; set; }

    /// <summary>Unity-style layer, 0..31, used by the world's layer collision matrix and by query layer masks. Default 0.</summary>
    public int Layer { get; set; }

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

/// <summary>One point of a contact manifold, in world coordinates relative to the world origin.</summary>
public readonly struct ContactPoint2D
{
    internal ContactPoint2D(Vector2D point, float separation, float normalImpulse, float tangentImpulse)
    {
        Point = point;
        Separation = separation;
        NormalImpulse = normalImpulse;
        TangentImpulse = tangentImpulse;
    }

    /// <summary>The point on the first shape's surface.</summary>
    public Vector2D Point { get; }

    /// <summary>Signed gap along the normal at the end of the step; negative when the shapes overlap, up to the speculative distance (0.02) when they only nearly touch.</summary>
    public float Separation { get; }

    /// <summary>Estimated impulse along the normal over the whole step: the solver's accumulated impulse of the last substep times the substep count (kg m/s).</summary>
    public float NormalImpulse { get; }

    /// <summary>Estimated friction impulse over the whole step, signed along the tangent (normal rotated clockwise); same scaling as <see cref="NormalImpulse"/>.</summary>
    public float TangentImpulse { get; }
}

/// <summary>What a collision callback is told: the other body and shapes, and where and how they touch.</summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "Handle fields, not properties, so that `hit.Body.LinearVelocity = v` compiles: a property result is not a variable (CS1612), a readonly field is.")]
public readonly struct Collision
{
    private readonly ContactPoint2D _p0, _p1;

    internal Collision(Body self, Body other, Shape selfShape, Shape otherShape, Vector2F normal, int pointCount, in ContactPoint2D p0, in ContactPoint2D p1, float approachSpeed)
    {
        Body = self;
        Other = other;
        Shape = selfShape;
        OtherShape = otherShape;
        Normal = normal;
        PointCount = pointCount;
        _p0 = p0;
        _p1 = p1;
        ApproachSpeed = approachSpeed;
    }

    /// <summary>The body the callback belongs to.</summary>
    public readonly Body Body;

    /// <summary>The body it touched. For an exit caused by destroying a body this handle is stale (<see cref="Body.IsValid"/> is false).</summary>
    public readonly Body Other;

    /// <summary>The shape of <see cref="Body"/> involved.</summary>
    public readonly Shape Shape;

    /// <summary>The shape of <see cref="Other"/> involved.</summary>
    public readonly Shape OtherShape;

    /// <summary>Contact normal pointing from <see cref="Body"/> towards <see cref="Other"/>; zero when <see cref="HasPoint"/> is false.</summary>
    public Vector2F Normal { get; }

    /// <summary>Number of manifold points (0, 1 or 2). 0 for exit callbacks and for contacts that began and ended inside one step.</summary>
    public int PointCount { get; }

    /// <summary>False when there are no manifold points.</summary>
    public bool HasPoint => PointCount > 0;

    /// <summary>A world position on the contact (the first manifold point) at the end of the step; zero when <see cref="HasPoint"/> is false.</summary>
    public Vector2D Point => _p0.Point;

    /// <summary>The manifold point with the given index.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is not within 0..<see cref="PointCount"/>-1.</exception>
    public ContactPoint2D GetPoint(int index)
    {
        if ((uint)index >= (uint)PointCount) { throw new ArgumentOutOfRangeException(nameof(index), index, "A contact point index must be within 0.." + (PointCount - 1).ToString(System.Globalization.CultureInfo.InvariantCulture) + "."); }
        return index == 0 ? _p0 : _p1;
    }

    /// <summary>Sum of the points' <see cref="ContactPoint2D.NormalImpulse"/>.</summary>
    public float NormalImpulse => PointCount == 0 ? 0f : PointCount == 1 ? _p0.NormalImpulse : _p0.NormalImpulse + _p1.NormalImpulse;

    /// <summary>Sum of the points' <see cref="ContactPoint2D.TangentImpulse"/>.</summary>
    public float TangentImpulse => PointCount == 0 ? 0f : PointCount == 1 ? _p0.TangentImpulse : _p0.TangentImpulse + _p1.TangentImpulse;

    /// <summary>For hit events (<c>OnCollisionHit</c>): the approach speed along the normal at the start of the step; 0 for other callbacks.</summary>
    public float ApproachSpeed { get; }

    /// <inheritdoc/>
    public override string ToString() =>
        "Collision(" + Body + " - " + Other + ", " + PointCount + (PointCount == 1 ? " point" : " points") + ")";
}

/// <summary>What a trigger callback is told: the other body and shapes of a sensor overlap.</summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "Handle fields, not properties, so that `hit.Body.LinearVelocity = v` compiles: a property result is not a variable (CS1612), a readonly field is.")]
public readonly struct Trigger
{
    internal Trigger(Body self, Body other, Shape selfShape, Shape otherShape, bool selfIsSensor)
    {
        Body = self;
        Other = other;
        Shape = selfShape;
        OtherShape = otherShape;
        SelfIsSensor = selfIsSensor;
    }

    /// <summary>The body the callback belongs to.</summary>
    public readonly Body Body;

    /// <summary>The other body. After an exit caused by destroying a body or shape this handle may be stale (<see cref="Body.IsValid"/> is false).</summary>
    public readonly Body Other;

    /// <summary>The shape of <see cref="Body"/> involved.</summary>
    public readonly Shape Shape;

    /// <summary>The shape of <see cref="Other"/> involved.</summary>
    public readonly Shape OtherShape;

    /// <summary>True when <see cref="Shape"/> is the sensor, false when <see cref="OtherShape"/> is.</summary>
    public bool SelfIsSensor { get; }

    /// <inheritdoc/>
    public override string ToString() =>
        "Trigger(" + Body + " - " + Other + ")";
}

/// <summary>
/// A moving body's bounds left the range the broadphase supports (±2^22 from the world origin), or its position became not finite. <see cref="PhysicsWorld2D.Step"/> raises it before it changes
/// anything, so the world is intact: the host moves the body back, destroys it or shifts the origin, then steps again. Until then every step raises it again.
/// </summary>
public sealed class BodyOutOfBoundsException : PhysicsException
{
    /// <summary>The offending body; destroy it or move it back, then step again.</summary>
    public Body Body { get; }

    /// <summary>Creates the exception for <paramref name="body"/>; the message gives its position.</summary>
    public BodyOutOfBoundsException(Body body, string message) : base(message) { Body = body; }
}
