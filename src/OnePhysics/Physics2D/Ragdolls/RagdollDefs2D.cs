using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>A ragdoll definition that cannot be accepted: a parent index that does not come before its part, no parts, too many parts. The message names the part and the field. Out-of-range numbers raise <see cref="ArgumentOutOfRangeException"/> instead. Nothing has changed when this is thrown.</summary>
public sealed class InvalidRagdollException : PhysicsException
{
    /// <summary>Creates the exception with a message naming what was wrong.</summary>
    public InvalidRagdollException(string message) : base(message) { }
}

/// <summary>The geometry of a ragdoll part's body.</summary>
public enum RagdollShapeKind : byte
{
    /// <summary>A capsule along the body's x axis: <see cref="RagdollPartDef.ShapeHalfSize"/> is (half length of the straight part, radius).</summary>
    Capsule = 0,

    /// <summary>A circle: <see cref="RagdollPartDef.ShapeHalfSize"/>.X is the radius.</summary>
    Circle = 1,

    /// <summary>A box: <see cref="RagdollPartDef.ShapeHalfSize"/> holds the half extents.</summary>
    Box = 2,
}

/// <summary>Which parts of one ragdoll collide with each other.</summary>
public enum RagdollSelfCollision : byte
{
    /// <summary>No two parts of the ragdoll collide (they share a negative collision group).</summary>
    None = 0,

    /// <summary>Parts joined directly to each other do not collide; every other pair does. The default.</summary>
    NonAdjacent = 1,

    /// <summary>Every pair of parts collides, joined or not.</summary>
    All = 2,
}

/// <summary>
/// One part of a ragdoll: a body with one shape, and the revolute joint that attaches it to its parent. Positions and angles are in the ragdoll's own space in the bind pose (the pose the ragdoll is built in);
/// <see cref="PhysicsWorld2D.AddRagdoll(RagdollDef, Vector2D, float, ReadOnlySpan{float})"/> places that space in the world.
/// </summary>
public sealed class RagdollPartDef
{
    /// <summary>A label for the host (bone name); not used by the engine. Default null.</summary>
    public string? Name { get; set; }

    /// <summary>Index of the parent part in <see cref="RagdollDef.Parts"/>, or -1 for the root. A parent must come before its children, so the skeleton cannot contain a cycle. Exactly one part is the root. Default -1.</summary>
    public int Parent { get; set; } = -1;

    /// <summary>The shape. Default capsule.</summary>
    public RagdollShapeKind Shape { get; set; } = RagdollShapeKind.Capsule;

    /// <summary>Half sizes of the shape, m: see <see cref="RagdollShapeKind"/> (box half extents; capsule half length and radius; circle radius). Components used must be positive and finite. Default (0.1, 0.05).</summary>
    public Vector2F ShapeHalfSize { get; set; } = new(0.1f, 0.05f);

    /// <summary>Position of the body (its origin, the shape's centre) in ragdoll space, m. Default the origin.</summary>
    public Vector2F Position { get; set; }

    /// <summary>Orientation of the body in ragdoll space, radians. Default 0.</summary>
    public float Angle { get; set; }

    /// <summary>Mass, kg; the density follows from the shape's area. Positive. Default 1.</summary>
    public float Mass { get; set; } = 1f;

    /// <summary>Friction of the part's shape. At least 0. Default 0.6.</summary>
    public float Friction { get; set; } = 0.6f;

    /// <summary>Where the joint to the parent sits, in ragdoll space, m. Ignored for the root. Default the origin.</summary>
    public Vector2F JointPosition { get; set; }

    /// <summary>Lower limit of the joint angle, radians within -π..π: the part's angle minus its parent's, minus the same in the bind pose (0 in the bind pose). Default -0.5.</summary>
    public float LowerAngle { get; set; } = -0.5f;

    /// <summary>Upper limit of the joint angle, radians within -π..π, at least the lower limit. Default 0.5.</summary>
    public float UpperAngle { get; set; } = 0.5f;

    /// <summary>Response rate of the pose drive of this part's joint, Hz: the drive asks for a joint speed of 2π · this · the angle error. At least 0. Default 6.</summary>
    public float DriveHertz { get; set; } = 6f;

    /// <summary>Largest joint speed the drive asks for, rad/s. At least 0. Default 12.</summary>
    public float MaxDriveSpeed { get; set; } = 12f;

    /// <summary>Largest torque the drive may apply at full strength, N·m. At least 0. Default 100.</summary>
    public float MaxTorque { get; set; } = 100f;

    /// <summary>Reaction force above which the joint breaks and the part comes off (dismemberment), N. Infinity (default) never breaks. Positive.</summary>
    public float BreakForce { get; set; } = float.PositiveInfinity;

    /// <summary>Reaction torque above which the joint breaks, N·m. Infinity (default) never breaks. Positive.</summary>
    public float BreakTorque { get; set; } = float.PositiveInfinity;

    internal RagdollPartDef Clone() => (RagdollPartDef)MemberwiseClone();

    internal float Area()
    {
        const float Pi = 3.1415927f;
        float a = ShapeHalfSize.X, b = ShapeHalfSize.Y;
        return Shape switch
        {
            RagdollShapeKind.Circle => Pi * a * a,
            RagdollShapeKind.Capsule => 4f * a * b + Pi * b * b,
            _ => 4f * a * b,
        };
    }
}

/// <summary>
/// A skeleton as a list of parts with parents before children, plus how the ragdoll behaves as a whole. <see cref="Humanoid"/> is a thirteen-part person in side view. Definitions are copied when a ragdoll is made, so one can be reused.
/// </summary>
public sealed class RagdollDef
{
    /// <summary>The most parts one ragdoll may have.</summary>
    public const int MaxParts = 64;

    /// <summary>The parts, parents first. At least one, at most <see cref="MaxParts"/>.</summary>
    public RagdollPartDef[] Parts { get; set; } = Array.Empty<RagdollPartDef>();

    /// <summary>Which parts of the ragdoll collide with each other. Default <see cref="RagdollSelfCollision.NonAdjacent"/>.</summary>
    public RagdollSelfCollision SelfCollision { get; set; } = RagdollSelfCollision.NonAdjacent;

    /// <summary>Angular damping of every part's body, 1/s: a little keeps limp limbs from swinging forever. At least 0. Default 0.05.</summary>
    public float AngularDamping { get; set; } = 0.05f;

    /// <summary>Collision category bits of the parts' shapes. Default 1.</summary>
    public uint CategoryBits { get; set; } = 1u;

    /// <summary>Collision mask bits of the parts' shapes. Default all.</summary>
    public uint MaskBits { get; set; } = uint.MaxValue;

    internal RagdollDef Clone()
    {
        if (Parts == null) { return new RagdollDef { SelfCollision = SelfCollision, AngularDamping = AngularDamping, CategoryBits = CategoryBits, MaskBits = MaskBits, Parts = null! }; }
        var d = new RagdollDef { SelfCollision = SelfCollision, AngularDamping = AngularDamping, CategoryBits = CategoryBits, MaskBits = MaskBits, Parts = new RagdollPartDef[Parts.Length] };
        for (int i = 0; i < Parts.Length; i++) { d.Parts[i] = Parts[i]?.Clone()!; }
        return d;
    }

    private static string Num(float v) => v.ToString("R", CultureInfo.InvariantCulture);

    private static string Where(int i, RagdollPartDef p) => "Ragdoll part " + i.ToString(CultureInfo.InvariantCulture) + (p.Name != null ? " (" + p.Name + ")" : "");

    private static void Finite(float v, int i, RagdollPartDef p, string field)
    {
        if (!float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Where(i, p) + ": " + field + " = " + Num(v) + " is not finite."); }
    }

    private static void Positive(float v, int i, RagdollPartDef p, string field)
    {
        if (!(v > 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Where(i, p) + ": " + field + " = " + Num(v) + " must be finite and above 0."); }
    }

    private static void AtLeastZero(float v, int i, RagdollPartDef p, string field)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Where(i, p) + ": " + field + " = " + Num(v) + " must be finite and at least 0."); }
    }

    private static void Position(Vector2F v, int i, RagdollPartDef p, string field)
    {
        Finite(v.X, i, p, field); Finite(v.Y, i, p, field);
        if (Math.Abs(v.X) > 1048576f || Math.Abs(v.Y) > 1048576f) { throw new ArgumentOutOfRangeException(field, v, Where(i, p) + ": " + field + " is outside the supported range of +-2^20."); }
    }

    /// <summary>Checks the whole definition. Throws <see cref="InvalidRagdollException"/> for a bad skeleton and <see cref="ArgumentOutOfRangeException"/> for a number out of range, each naming the part and the field.</summary>
    internal void Validate()
    {
        if (Parts == null || Parts.Length == 0) { throw new InvalidRagdollException("A ragdoll needs at least one part."); }
        if (Parts.Length > MaxParts) { throw new InvalidRagdollException("A ragdoll has at most " + MaxParts.ToString(CultureInfo.InvariantCulture) + " parts; this one has " + Parts.Length.ToString(CultureInfo.InvariantCulture) + "."); }
        if (!Enum.IsDefined(SelfCollision)) { throw new ArgumentOutOfRangeException(nameof(SelfCollision), SelfCollision, "Unknown self-collision mode."); }
        if (!(AngularDamping >= 0f) || !float.IsFinite(AngularDamping)) { throw new ArgumentOutOfRangeException(nameof(AngularDamping), AngularDamping, "Ragdoll AngularDamping must be finite and at least 0."); }
        int roots = 0;
        for (int i = 0; i < Parts.Length; i++)
        {
            var p = Parts[i];
            if (p == null) { throw new InvalidRagdollException("Ragdoll part " + i.ToString(CultureInfo.InvariantCulture) + " is null."); }
            if (p.Parent == -1) { roots++; }
            else if (p.Parent < -1 || p.Parent >= Parts.Length)
            {
                throw new InvalidRagdollException(Where(i, p) + ": Parent " + p.Parent.ToString(CultureInfo.InvariantCulture) + " is not a part index (0.." + (Parts.Length - 1).ToString(CultureInfo.InvariantCulture) + ") or -1.");
            }
            else if (p.Parent >= i)
            {
                throw new InvalidRagdollException(Where(i, p) + ": Parent " + p.Parent.ToString(CultureInfo.InvariantCulture) + " must come before the part (a parent after its child, or the part itself, makes a cycle).");
            }

            if (!Enum.IsDefined(p.Shape)) { throw new ArgumentOutOfRangeException("Shape", p.Shape, Where(i, p) + ": unknown shape kind."); }
            switch (p.Shape)
            {
                case RagdollShapeKind.Circle: Positive(p.ShapeHalfSize.X, i, p, "ShapeHalfSize.X (radius)"); break;
                case RagdollShapeKind.Capsule: Positive(p.ShapeHalfSize.X, i, p, "ShapeHalfSize.X (half length)"); Positive(p.ShapeHalfSize.Y, i, p, "ShapeHalfSize.Y (radius)"); break;
                default: Positive(p.ShapeHalfSize.X, i, p, "ShapeHalfSize.X"); Positive(p.ShapeHalfSize.Y, i, p, "ShapeHalfSize.Y"); break;
            }

            if (Math.Abs(p.ShapeHalfSize.X) > 65536f || Math.Abs(p.ShapeHalfSize.Y) > 65536f) { throw new ArgumentOutOfRangeException("ShapeHalfSize", p.ShapeHalfSize, Where(i, p) + ": ShapeHalfSize is beyond 65536 m."); }
            Position(p.Position, i, p, "Position");
            Finite(p.Angle, i, p, "Angle");
            Positive(p.Mass, i, p, "Mass");
            AtLeastZero(p.Friction, i, p, "Friction");
            AtLeastZero(p.DriveHertz, i, p, "DriveHertz");
            AtLeastZero(p.MaxDriveSpeed, i, p, "MaxDriveSpeed");
            AtLeastZero(p.MaxTorque, i, p, "MaxTorque");
            if (!(p.BreakForce > 0f) || !(p.BreakTorque > 0f)) { throw new ArgumentOutOfRangeException("BreakForce", p.BreakForce, Where(i, p) + ": BreakForce and BreakTorque must be above 0 (infinity never breaks)."); }
            if (p.Parent >= 0)
            {
                Position(p.JointPosition, i, p, "JointPosition");
                Finite(p.LowerAngle, i, p, "LowerAngle"); Finite(p.UpperAngle, i, p, "UpperAngle");
                if (p.LowerAngle > p.UpperAngle) { throw new InvalidRagdollException(Where(i, p) + ": LowerAngle " + Num(p.LowerAngle) + " is above UpperAngle " + Num(p.UpperAngle) + "."); }
                if (p.LowerAngle < -3.1415927f || p.UpperAngle > 3.1415927f) { throw new ArgumentOutOfRangeException("LowerAngle", "[" + Num(p.LowerAngle) + ", " + Num(p.UpperAngle) + "]", Where(i, p) + ": angle limits must lie within -pi..pi."); }
            }
        }

        if (roots != 1) { throw new InvalidRagdollException("A ragdoll has exactly one root part (Parent = -1); this one has " + roots.ToString(CultureInfo.InvariantCulture) + "."); }
    }

    /// <summary>
    /// A thirteen-part person of 1.8 m and 75 kg standing in the bind pose in side view, arms hanging, feet on y = 0, facing +x: pelvis (the root), torso, head, then near arm (upper arm, forearm), far arm,
    /// then near leg (thigh, shin, foot) and far leg. Parts in order: pelvis, torso, head, upper arm, forearm, upper arm, forearm, thigh, shin, foot, thigh, shin, foot. Pose angles are joint angles from the bind pose,
    /// positive counter-clockwise (a limb swings forward). The two arms and the two legs overlap in a side view, so self-collision is off.
    /// </summary>
    public static RagdollDef Humanoid => CreateHumanoid();

    /// <summary>A person of the given height and mass (the geometry scales with height, the gains with mass and height). See <see cref="Humanoid"/> for the layout.</summary>
    /// <param name="height">Standing height, m; finite and above 0. Default 1.8.</param>
    /// <param name="mass">Total mass, kg; finite and above 0. Default 75.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is not positive and finite.</exception>
    public static RagdollDef CreateHumanoid(float height = 1.8f, float mass = 75f)
    {
        if (!(height > 0f) || !float.IsFinite(height) || height > 1000f) { throw new ArgumentOutOfRangeException(nameof(height), height, "The height must be finite, above 0 and at most 1000 m."); }
        if (!(mass > 0f) || !float.IsFinite(mass) || mass > 1e7f) { throw new ArgumentOutOfRangeException(nameof(mass), mass, "The mass must be finite, above 0 and at most 1e7 kg."); }
        float s = height / 1.8f, m = mass / 75f, g = m * s;   // torque scales with weight times lever arm
        var parts = new RagdollPartDef[13];
        const float Up = 1.5707964f;                           // a limb's capsule axis (its local x) points up in the bind pose

        RagdollPartDef P(string name, int parent, RagdollShapeKind kind, float a, float b, float x, float y, float angle, float kg, float jx, float jy, float lower, float upper, float maxTorque, float friction = 0.6f) => new()
        {
            Name = name, Parent = parent, Shape = kind, ShapeHalfSize = new Vector2F(a * s, b * s), Position = new Vector2F(x * s, y * s), Angle = angle, Mass = kg * m,
            JointPosition = new Vector2F(jx * s, jy * s), LowerAngle = lower, UpperAngle = upper, MaxTorque = maxTorque * g, Friction = friction,
        };

        const RagdollShapeKind Cap = RagdollShapeKind.Capsule, Box = RagdollShapeKind.Box, Cir = RagdollShapeKind.Circle;
        parts[0] = P("pelvis", -1, Box, 0.12f, 0.08f, 0f, 0.98f, 0f, 10f, 0, 0, 0, 0, 0);
        parts[1] = P("torso", 0, Box, 0.13f, 0.18f, 0f, 1.24f, 0f, 25f, 0f, 1.06f, -0.5f, 0.5f, 900f);
        parts[2] = P("head", 1, Cir, 0.11f, 0f, 0f, 1.55f, 0f, 5f, 0f, 1.44f, -0.6f, 0.6f, 150f);
        for (int side = 0; side < 2; side++)
        {
            int arm = 3 + 2 * side, leg = 7 + 3 * side;
            parts[arm] = P("upper arm", 1, Cap, 0.11f, 0.045f, 0f, 1.225f, Up, 2.5f, 0f, 1.38f, -1.2f, 3.0f, 200f);
            parts[arm + 1] = P("forearm", arm, Cap, 0.095f, 0.04f, 0f, 0.935f, Up, 1.5f, 0f, 1.07f, -0.05f, 2.5f, 120f);
            parts[leg] = P("thigh", 0, Cap, 0.13f, 0.08f, 0f, 0.71f, Up, 8f, 0f, 0.92f, -0.7f, 1.5f, 1500f);
            parts[leg + 1] = P("shin", leg, Cap, 0.15f, 0.06f, 0f, 0.29f, Up, 4f, 0f, 0.50f, -2.4f, 0.05f, 1200f);
            parts[leg + 2] = P("foot", leg + 1, Box, 0.14f, 0.04f, 0.06f, 0.04f, 0f, 1.2f, 0f, 0.08f, -0.7f, 0.4f, 600f, 1.0f);
        }

        return new RagdollDef { Parts = parts, SelfCollision = RagdollSelfCollision.None };
    }
}
