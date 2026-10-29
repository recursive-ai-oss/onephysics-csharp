using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Surface properties of a shape: friction, restitution and rolling resistance. Presets are starting points, not measurements.</summary>
public readonly struct Material3D : IEquatable<Material3D>
{
    /// <summary>The rolling resistance of a material or shape that does not set one: 0.003. A sphere or capsule touches a surface at one point or along a line, where Coulomb friction alone has no
    /// torque to spin it down (about the contact normal, or about the capsule's axis); without any resistance a ragdoll's heads and limbs spin on indefinitely and a pile never comes to rest. This
    /// small cap on the rolling torque, a fraction of the normal impulse times 1 m, removes that spin and slows a free rolling ball only as a rubber tyre on a road would.</summary>
    public const float DefaultRollingResistance = 0.003f;

    /// <summary>Creates a material.</summary>
    /// <param name="friction">Coulomb friction coefficient; finite and at least 0.</param>
    /// <param name="restitution">Bounciness in 0..1.</param>
    /// <param name="rollingResistance">Resistance to rolling, as a fraction of the normal impulse times 1 m; finite and at least 0. Default <see cref="DefaultRollingResistance"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside its range.</exception>
    public Material3D(float friction, float restitution = 0f, float rollingResistance = DefaultRollingResistance)
    {
        if (!(friction >= 0f && friction <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(nameof(friction), friction, "Friction must be within 0..1e15."); }
        if (!(restitution >= 0f && restitution <= 1f)) { throw new ArgumentOutOfRangeException(nameof(restitution), restitution, "Restitution must be within 0..1."); }
        if (!(rollingResistance >= 0f && rollingResistance <= MaterialLimits.MaxCoefficient)) { throw new ArgumentOutOfRangeException(nameof(rollingResistance), rollingResistance, "Rolling resistance must be within 0..1e15."); }
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

    /// <summary>Pairs of shapes with overlapping boxes the world should have room for (0..16777216). With <see cref="PhysicsWorld3D.Warmup"/> or at creation, the pair table and the per-step pair lists are sized to it so steps within it allocate nothing. 0 (the default) grows on demand; <c>Warmup()</c> then assumes eight per body. Never changes a result; not saved.</summary>
    public int PairCapacity { get; set; }

    /// <summary>Contact constraints (and joints) the world should have room for (0..16777216): constraint storage, colors, solver rows and event buffers. 0 (the default) grows on demand; <c>Warmup()</c> then assumes six per body. Never changes a result; not saved.</summary>
    public int ContactCapacity { get; set; }

    /// <summary>Islands the world should have room for (0..16777216). 0 (the default) grows on demand; <c>Warmup()</c> then assumes one per body. Never changes a result; not saved.</summary>
    public int IslandCapacity { get; set; }

    /// <summary>Hits the query scratch of the thread that calls <c>Warmup()</c> should hold (0..1048576), so the first queries of up to that many hits allocate nothing. 0 keeps the default of 64. Never changes a result; not saved.</summary>
    public int QueryScratchCapacity { get; set; }

    internal const int MaxCapacity = 1 << 24;

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

    /// <summary>Approach speed (m/s, along the contact normal at the start of the step) above which a contact reports a hit event, when <see cref="EventMask.ContactHit"/> is on. At least 0. Default 1.</summary>
    public float HitSpeedThreshold { get; set; } = 1f;

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
        if (PairCapacity < 0 || PairCapacity > MaxCapacity) { throw new ArgumentOutOfRangeException(nameof(PairCapacity), PairCapacity, "PairCapacity must be in 0..16777216."); }
        if (ContactCapacity < 0 || ContactCapacity > MaxCapacity) { throw new ArgumentOutOfRangeException(nameof(ContactCapacity), ContactCapacity, "ContactCapacity must be in 0..16777216."); }
        if (IslandCapacity < 0 || IslandCapacity > MaxCapacity) { throw new ArgumentOutOfRangeException(nameof(IslandCapacity), IslandCapacity, "IslandCapacity must be in 0..16777216."); }
        if (QueryScratchCapacity < 0 || QueryScratchCapacity > 1 << 20) { throw new ArgumentOutOfRangeException(nameof(QueryScratchCapacity), QueryScratchCapacity, "QueryScratchCapacity must be in 0..1048576."); }
        if (!float.IsFinite(Gravity.X) || !float.IsFinite(Gravity.Y) || !float.IsFinite(Gravity.Z)) { throw new ArgumentOutOfRangeException(nameof(Gravity), "Gravity must be finite."); }
        Positive(ContactHertz, nameof(ContactHertz), true);
        Positive(ContactDampingRatio, nameof(ContactDampingRatio), true);
        Positive(PushOutVelocity, nameof(PushOutVelocity), true);
        Positive(ContactSlop, nameof(ContactSlop), true);
        Positive(RestitutionThreshold, nameof(RestitutionThreshold), true);
        Positive(MaxLinearSpeed, nameof(MaxLinearSpeed), false);
        Positive(MaxRotationPerSubstep, nameof(MaxRotationPerSubstep), false);
        Positive(SpeculativeDistance, nameof(SpeculativeDistance), true);
        Positive(HitSpeedThreshold, nameof(HitSpeedThreshold), true);
        Positive(JointHertz, nameof(JointHertz), true);
        Positive(JointDampingRatio, nameof(JointDampingRatio), true);
        if (!double.IsFinite(BroadphaseMargin) || BroadphaseMargin < 0.0) { throw new ArgumentOutOfRangeException(nameof(BroadphaseMargin), BroadphaseMargin, "BroadphaseMargin must be finite and at least 0."); }
        // One reusable options object for the check: Validate runs on every step, and building a new one each time would allocate.
        _coreCheck ??= new WorldOptions();
        FillCore(_coreCheck);
        _coreCheck.Validate();
    }

    private static void Positive(float v, string name, bool allowZero)
    {
        if (!float.IsFinite(v) || v < 0f || (!allowZero && v == 0f)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and " + (allowZero ? "at least 0." : "positive.")); }
    }

    private WorldOptions? _coreCheck;

    internal WorldOptions ToCore()
    {
        var o = new WorldOptions();
        FillCore(o);
        return o;
    }

    private void FillCore(WorldOptions o)
    {
        o.Layout = DimensionLayout.Spatial;
        o.ThreadCount = ThreadCount;
        o.WorldId = WorldId;
        o.Gravity = Gravity;
        o.Substeps = Substeps;
        o.VelocityIterations = VelocityIterations;
        o.RelaxIterations = RelaxIterations;
        o.SleepEnabled = SleepEnabled;
        o.TimeToSleep = TimeToSleep;
        o.SleepThreshold = SleepThreshold;
        o.Events = Events;
    }
}

/// <summary>Everything needed to create a body; defaults make a dynamic body at the origin.</summary>
public struct BodyDef
{
    /// <summary>Static, kinematic or dynamic. Default dynamic.</summary>
    public BodyType Type { readonly get => (BodyType)(_type ^ (byte)(BodyType.Dynamic)); set => _type = (byte)((byte)value ^ (byte)(BodyType.Dynamic)); }

    private byte _type;

    /// <summary>Body origin in world coordinates, within ±2^21 of the world origin.</summary>
    public Vector3D Position { get; set; }

    /// <summary>Orientation; any non-zero finite quaternion (normalised on creation). Default identity.</summary>
    public QuaternionF Rotation { readonly get => _rotation.Get(QuaternionF.Identity); set => _rotation.Set(value, QuaternionF.Identity); }

    private ZeroDefaultQuat _rotation;

    /// <summary>Initial linear velocity, m/s.</summary>
    public Vector3F LinearVelocity { get; set; }

    /// <summary>Initial angular velocity, rad/s.</summary>
    public Vector3F AngularVelocity { get; set; }

    /// <summary>Linear damping, 1/s; finite, at least 0.</summary>
    public float LinearDamping { get; set; }

    /// <summary>Angular damping, 1/s; finite, at least 0.</summary>
    public float AngularDamping { get; set; }

    /// <summary>Multiplier on gravity. Default 1.</summary>
    public float GravityScale { readonly get => ZeroDefault.Get(_gravityScale, 1f); set => _gravityScale = ZeroDefault.Set(value, 1f); }

    private uint _gravityScale;

    /// <summary>Locks rotation (infinite inertia).</summary>
    public bool FixedRotation { get; set; }

    /// <summary>Marks a fast body that should get continuous collision. Continuous collision runs for every fast dynamic body regardless.</summary>
    public bool IsBullet { get; set; }

    /// <summary>Whether the body may sleep. Default true.</summary>
    public bool AllowSleep { readonly get => !_allowSleepOff; set => _allowSleepOff = !value; }

    private bool _allowSleepOff;

    /// <summary>Creates a dynamic body asleep.</summary>
    public bool StartAsleep { get; set; }
}

/// <summary>Everything needed to create a shape besides its geometry; defaults suit a wooden object.</summary>
public struct ShapeDef
{
    /// <summary>Density in kg/m³; 0 adds no mass. Default 1. A dynamic body's total mass and each diagonal entry of its inertia tensor must lie within 1e-18..1e30, and the tensor must be invertible; a shape that would break that is refused with an <see cref="ArgumentOutOfRangeException"/> naming the mass and inertia, and the body is unchanged.</summary>
    public float Density { readonly get => ZeroDefault.Get(_density, 1f); set => _density = ZeroDefault.Set(value, 1f); }

    private uint _density;

    /// <summary>Friction coefficient. Default 0.6.</summary>
    public float Friction { readonly get => ZeroDefault.Get(_friction, 0.6f); set => _friction = ZeroDefault.Set(value, 0.6f); }

    private uint _friction;

    /// <summary>Restitution in 0..1. Default 0.</summary>
    public float Restitution { get; set; }

    /// <summary>Rolling resistance. Default <see cref="Material3D.DefaultRollingResistance"/>.</summary>
    public float RollingResistance { readonly get => ZeroDefault.Get(_rollingResistance, Material3D.DefaultRollingResistance); set => _rollingResistance = ZeroDefault.Set(value, Material3D.DefaultRollingResistance); }

    private uint _rollingResistance;

    /// <summary>Collision category bits. Default 1.</summary>
    public uint CategoryBits { readonly get => _categoryBits ^ (1u); set => _categoryBits = value ^ (1u); }

    private uint _categoryBits;

    /// <summary>Categories this shape collides with. Default all.</summary>
    public uint MaskBits { readonly get => _maskBits ^ (uint.MaxValue); set => _maskBits = value ^ (uint.MaxValue); }

    private uint _maskBits;

    /// <summary>Non-zero: shapes in one group always (positive) or never (negative) collide, ignoring masks.</summary>
    public int GroupIndex { get; set; }

    /// <summary>A sensor (trigger): produces no contact response, adds no mass and reports overlaps as trigger events. Fixed at creation.</summary>
    public bool IsSensor { get; set; }

    /// <summary>Layer 0..31 for the world's layer collision matrix and query layer masks. Default 0.</summary>
    public int Layer { get; set; }

    /// <summary>Position of the shape in the body's frame.</summary>
    public Vector3F LocalPosition { get; set; }

    /// <summary>Orientation of the shape in the body's frame; any non-zero finite quaternion. Default identity.</summary>
    public QuaternionF LocalRotation { readonly get => _localRotation.Get(QuaternionF.Identity); set => _localRotation.Set(value, QuaternionF.Identity); }

    private ZeroDefaultQuat _localRotation;

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

/// <summary>One manifold point of a contact as a callback sees it.</summary>
public readonly struct ContactPoint3D
{
    internal ContactPoint3D(Vector3D point, float separation, float normalImpulse, float tangentImpulse)
    {
        Point = point;
        Separation = separation;
        NormalImpulse = normalImpulse;
        TangentImpulse = tangentImpulse;
    }

    /// <summary>World position of the point.</summary>
    public Vector3D Point { get; }

    /// <summary>Signed gap along the normal; negative when the shapes overlap.</summary>
    public float Separation { get; }

    /// <summary>Estimate of the normal impulse over the step: the last substep's accumulated impulse times the substep count.</summary>
    public float NormalImpulse { get; }

    /// <summary>Estimate of the friction impulse over the step (length of the two tangent impulses, same scaling).</summary>
    public float TangentImpulse { get; }
}

/// <summary>
/// Passed to collision callbacks: who touched whom and where. One event describes a pair of shapes, however many triangles or compound children touch; the points are those of the pair's first
/// sub-shape contact (lowest sub-shape ids), as they are at the end of the step.
/// </summary>
[System.Diagnostics.CodeAnalysis.SuppressMessage("Design", "CA1051:Do not declare visible instance fields", Justification = "Handle fields, not properties, so that `hit.Body.LinearVelocity = v` compiles: a property result is not a variable (CS1612), a readonly field is.")]
public readonly struct Collision
{
    private readonly ContactPoint3D _p0, _p1, _p2, _p3;

    internal Collision(Body self, Body other, Shape selfShape, Shape otherShape, Vector3F normal, int pointCount, in ContactPoint3D p0, in ContactPoint3D p1, in ContactPoint3D p2, in ContactPoint3D p3, float approachSpeed)
    {
        Body = self;
        Other = other;
        Shape = selfShape;
        OtherShape = otherShape;
        Normal = normal;
        PointCount = pointCount;
        _p0 = p0; _p1 = p1; _p2 = p2; _p3 = p3;
        ApproachSpeed = approachSpeed;
    }

    /// <summary>The body whose callback runs.</summary>
    public readonly Body Body;

    /// <summary>The body it touched. For an exit caused by destroying a body this handle is stale.</summary>
    public readonly Body Other;

    /// <summary>The callback body's shape.</summary>
    public readonly Shape Shape;

    /// <summary>The other body's shape.</summary>
    public readonly Shape OtherShape;

    /// <summary>Contact normal pointing from <see cref="Body"/> towards <see cref="Other"/>; zero when there are no points.</summary>
    public Vector3F Normal { get; }

    /// <summary>Number of manifold points, 0 to 4. 0 for exit callbacks and for contacts that began and ended inside one step.</summary>
    public int PointCount { get; }

    /// <summary>False when there are no manifold points.</summary>
    public bool HasPoint => PointCount > 0;

    /// <summary>The first manifold point in world coordinates; zero when <see cref="HasPoint"/> is false.</summary>
    public Vector3D Point => _p0.Point;

    /// <summary>The manifold point with the given index.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is not within 0..<see cref="PointCount"/>-1.</exception>
    public ContactPoint3D GetPoint(int index)
    {
        if ((uint)index >= (uint)PointCount) { throw new ArgumentOutOfRangeException(nameof(index), index, "A contact point index must be within 0.." + (PointCount - 1).ToString(System.Globalization.CultureInfo.InvariantCulture) + "."); }
        return index == 0 ? _p0 : index == 1 ? _p1 : index == 2 ? _p2 : _p3;
    }

    /// <summary>Sum of the points' normal impulses.</summary>
    public float NormalImpulse => PointCount == 0 ? 0f : ((_p0.NormalImpulse + (PointCount > 1 ? _p1.NormalImpulse : 0f)) + (PointCount > 2 ? _p2.NormalImpulse : 0f)) + (PointCount > 3 ? _p3.NormalImpulse : 0f);

    /// <summary>For hit events: the approach speed along the normal at the start of the step; 0 for other callbacks.</summary>
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

    /// <summary>The other body. After an exit caused by destroying a body or shape this handle may be stale.</summary>
    public readonly Body Other;

    /// <summary>The callback body's shape.</summary>
    public readonly Shape Shape;

    /// <summary>The other body's shape.</summary>
    public readonly Shape OtherShape;

    /// <summary>True when <see cref="Shape"/> is the sensor, false when <see cref="OtherShape"/> is.</summary>
    public bool SelfIsSensor { get; }

    /// <inheritdoc/>
    public override string ToString() =>
        "Trigger(" + Body + " - " + Other + ")";
}

/// <summary>A body has left the range the engine supports; <see cref="Body"/> names it.</summary>
public sealed class BodyOutOfBoundsException : PhysicsException
{
    /// <summary>The offending body; destroy it or move it back, then step again.</summary>
    public Body Body { get; }

    /// <summary>Creates the exception.</summary>
    public BodyOutOfBoundsException(Body body, string message) : base(message) { Body = body; }
}
