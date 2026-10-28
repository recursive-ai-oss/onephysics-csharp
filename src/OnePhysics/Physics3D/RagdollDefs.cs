using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A ragdoll definition that cannot be accepted: a parent index that does not come before its part, no parts, too many parts. The message names the part and the field. Out-of-range numbers raise <see cref="ArgumentOutOfRangeException"/> instead. Nothing has changed when this is thrown.</summary>
public sealed class InvalidRagdollException : PhysicsException
{
    /// <summary>Creates the exception with a message naming what was wrong.</summary>
    public InvalidRagdollException(string message) : base(message) { }
}

/// <summary>The geometry of a ragdoll part's body.</summary>
public enum RagdollShapeKind : byte
{
    /// <summary>A capsule along the body's y axis: <see cref="RagdollPartDef.ShapeSize"/> is (half length of the straight part, radius).</summary>
    Capsule = 0,

    /// <summary>A sphere: <see cref="RagdollPartDef.ShapeSize"/>.X is the radius.</summary>
    Sphere = 1,

    /// <summary>A box: <see cref="RagdollPartDef.ShapeSize"/> holds the half extents.</summary>
    Box = 2,
}

/// <summary>Which parts of one ragdoll collide with each other.</summary>
public enum RagdollSelfCollision : byte
{
    /// <summary>No two parts of the ragdoll collide (they share a negative collision group).</summary>
    None = 0,

    /// <summary>Parts joined directly to each other do not collide; every other pair does. The default: it keeps the joints free of contact fights and still stops a limb passing through the body.</summary>
    NonAdjacent = 1,

    /// <summary>Every pair of parts collides, joined or not.</summary>
    All = 2,
}

/// <summary>
/// One part of a ragdoll: a body with one shape, and the swing-twist joint that attaches it to its parent. Positions and rotations are in the ragdoll's own space in the bind pose (the pose the ragdoll is built in);
/// <see cref="PhysicsWorld3D.AddRagdoll(RagdollDef, Vector3D, QuaternionF, ReadOnlySpan{QuaternionF})"/> places that space in the world.
/// </summary>
public sealed class RagdollPartDef
{
    /// <summary>A label for the host (bone name); not used by the engine. Default null.</summary>
    public string? Name { get; set; }

    /// <summary>Index of the parent part in <see cref="RagdollDef.Parts"/>, or -1 for the root. A parent must come before its children, so the skeleton cannot contain a cycle. Exactly one part is the root. Default -1.</summary>
    public int Parent { get; set; } = -1;

    /// <summary>The shape. Default capsule.</summary>
    public RagdollShapeKind Shape { get; set; } = RagdollShapeKind.Capsule;

    /// <summary>Shape size, m: see <see cref="RagdollShapeKind"/>. Components used must be positive and finite. Default (0.1, 0.05, 0.05).</summary>
    public Vector3F ShapeSize { get; set; } = new(0.1f, 0.05f, 0.05f);

    /// <summary>Position of the body (its centre) in ragdoll space, m. Default the origin.</summary>
    public Vector3F Position { get; set; }

    /// <summary>Orientation of the body in ragdoll space; any finite non-zero quaternion (normalized). Default the identity.</summary>
    public QuaternionF Rotation { get; set; } = QuaternionF.Identity;

    /// <summary>Mass, kg; the density follows from the shape's volume. Positive. Default 1.</summary>
    public float Mass { get; set; } = 1f;

    /// <summary>Friction of the part's shape. At least 0. Default 0.6.</summary>
    public float Friction { get; set; } = 0.6f;

    /// <summary>Where the joint to the parent sits, in ragdoll space, m. Ignored for the root. Default the origin.</summary>
    public Vector3F JointPosition { get; set; }

    /// <summary>Orientation of the joint frame in ragdoll space. The frame's x axis is the twist axis; y and z are the swing axes. Ignored for the root. Default the identity.</summary>
    public QuaternionF JointRotation { get; set; } = QuaternionF.Identity;

    /// <summary>Lower twist limit, radians within -π..π. Default -0.5.</summary>
    public float LowerTwist { get; set; } = -0.5f;

    /// <summary>Upper twist limit, radians within -π..π. Default 0.5.</summary>
    public float UpperTwist { get; set; } = 0.5f;

    /// <summary>Half angle of the swing about the joint frame's y axis, radians within (0, π]. Default 0.5.</summary>
    public float SwingYHalfAngle { get; set; } = 0.5f;

    /// <summary>Half angle of the swing about the joint frame's z axis, radians within (0, π]. Default 0.5.</summary>
    public float SwingZHalfAngle { get; set; } = 0.5f;

    /// <summary>Stiffness of the pose drive of this part's joint at full strength, N·m per radian of error. At least 0. Default 500.</summary>
    public float DriveStiffness { get; set; } = 500f;

    /// <summary>Damping of the pose drive, N·m·s per radian. At least 0. Default 40.</summary>
    public float DriveDamping { get; set; } = 40f;

    /// <summary>Largest torque the drive may apply at full strength, N·m. At least 0. Default 100.</summary>
    public float MaxTorque { get; set; } = 100f;

    internal RagdollPartDef Clone() => (RagdollPartDef)MemberwiseClone();

    internal float Volume()
    {
        const float Pi = 3.1415927f;
        float a = ShapeSize.X, b = ShapeSize.Y, c = ShapeSize.Z;
        return Shape switch
        {
            RagdollShapeKind.Sphere => 4f / 3f * Pi * a * a * a,
            RagdollShapeKind.Capsule => Pi * b * b * (2f * a) + 4f / 3f * Pi * b * b * b,
            _ => 8f * a * b * c,
        };
    }
}

/// <summary>
/// A skeleton as a list of parts with parents before children, plus how the ragdoll behaves as a whole. <see cref="Humanoid"/> is a thirteen-part person. Definitions are copied when a ragdoll is made, so one can be reused.
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

    private static void Position(Vector3F v, int i, RagdollPartDef p, string field)
    {
        Finite(v.X, i, p, field); Finite(v.Y, i, p, field); Finite(v.Z, i, p, field);
        if (Math.Abs(v.X) > 1048576f || Math.Abs(v.Y) > 1048576f || Math.Abs(v.Z) > 1048576f) { throw new ArgumentOutOfRangeException(field, v, Where(i, p) + ": " + field + " is outside the supported range of +-2^20."); }
    }

    private static void Rotation(QuaternionF q, int i, RagdollPartDef p, string field)
    {
        float l2 = q.LengthSquared();
        if (!float.IsFinite(l2) || !(l2 > 1e-30f)) { throw new ArgumentOutOfRangeException(field, q, Where(i, p) + ": " + field + " must be a finite, non-zero quaternion."); }
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
                case RagdollShapeKind.Sphere: Positive(p.ShapeSize.X, i, p, "ShapeSize.X (radius)"); break;
                case RagdollShapeKind.Capsule: Positive(p.ShapeSize.X, i, p, "ShapeSize.X (half length)"); Positive(p.ShapeSize.Y, i, p, "ShapeSize.Y (radius)"); break;
                default: Positive(p.ShapeSize.X, i, p, "ShapeSize.X"); Positive(p.ShapeSize.Y, i, p, "ShapeSize.Y"); Positive(p.ShapeSize.Z, i, p, "ShapeSize.Z"); break;
            }

            if (Math.Abs(p.ShapeSize.X) > 65536f || Math.Abs(p.ShapeSize.Y) > 65536f || Math.Abs(p.ShapeSize.Z) > 65536f) { throw new ArgumentOutOfRangeException("ShapeSize", p.ShapeSize, Where(i, p) + ": ShapeSize is beyond 65536 m."); }
            Position(p.Position, i, p, "Position");
            Rotation(p.Rotation, i, p, "Rotation");
            Positive(p.Mass, i, p, "Mass");
            AtLeastZero(p.Friction, i, p, "Friction");
            AtLeastZero(p.DriveStiffness, i, p, "DriveStiffness");
            AtLeastZero(p.DriveDamping, i, p, "DriveDamping");
            AtLeastZero(p.MaxTorque, i, p, "MaxTorque");
            if (p.Parent >= 0)
            {
                Position(p.JointPosition, i, p, "JointPosition");
                Rotation(p.JointRotation, i, p, "JointRotation");
                Finite(p.LowerTwist, i, p, "LowerTwist"); Finite(p.UpperTwist, i, p, "UpperTwist");
                if (p.LowerTwist > p.UpperTwist) { throw new InvalidRagdollException(Where(i, p) + ": LowerTwist " + Num(p.LowerTwist) + " is above UpperTwist " + Num(p.UpperTwist) + "."); }
                if (p.LowerTwist < -3.1415927f || p.UpperTwist > 3.1415927f) { throw new ArgumentOutOfRangeException("LowerTwist", "[" + Num(p.LowerTwist) + ", " + Num(p.UpperTwist) + "]", Where(i, p) + ": twist limits must lie within -pi..pi."); }
                if (!(p.SwingYHalfAngle > 0f) || !(p.SwingYHalfAngle <= 3.1415927f)) { throw new ArgumentOutOfRangeException("SwingYHalfAngle", p.SwingYHalfAngle, Where(i, p) + ": SwingYHalfAngle = " + Num(p.SwingYHalfAngle) + " must be above 0 and at most pi."); }
                if (!(p.SwingZHalfAngle > 0f) || !(p.SwingZHalfAngle <= 3.1415927f)) { throw new ArgumentOutOfRangeException("SwingZHalfAngle", p.SwingZHalfAngle, Where(i, p) + ": SwingZHalfAngle = " + Num(p.SwingZHalfAngle) + " must be above 0 and at most pi."); }
            }
        }

        if (roots != 1) { throw new InvalidRagdollException("A ragdoll has exactly one root part (Parent = -1); this one has " + roots.ToString(CultureInfo.InvariantCulture) + "."); }
    }

    /// <summary>
    /// A thirteen-part person of 1.8 m and 75 kg standing in the bind pose, arms hanging, feet on y = 0, facing +z: pelvis (the root), torso, head, two arms of upper arm and forearm, two legs of thigh, shin and foot.
    /// Parts in order: pelvis, torso, head, then left upper arm, left forearm, right upper arm, right forearm (left is +x), then left thigh, shin, foot and right thigh, shin, foot. The drive gains hold the figure on its feet.
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
        const float R = 0.70710678f;
        var up = new QuaternionF(0f, 0f, R, R);              // twist axis (frame x) along +y

        RagdollPartDef P(string name, int parent, RagdollShapeKind kind, float a, float b, float c, float x, float y, float z, float kg, float jx, float jy, float jz,
            float twist, float swingY, float swingZ, float k, float damp, float max, float friction = 0.6f) => new()
        {
            Name = name, Parent = parent, Shape = kind, ShapeSize = new Vector3F(a * s, b * s, c * s), Position = new Vector3F(x * s, y * s, z * s), Mass = kg * m,
            JointPosition = new Vector3F(jx * s, jy * s, jz * s), JointRotation = up, LowerTwist = -twist, UpperTwist = twist, SwingYHalfAngle = swingY, SwingZHalfAngle = swingZ,
            DriveStiffness = k * g, DriveDamping = damp * g, MaxTorque = max * g, Friction = friction,
        };

        const RagdollShapeKind Cap = RagdollShapeKind.Capsule, Box = RagdollShapeKind.Box, Sph = RagdollShapeKind.Sphere;
        parts[0] = P("pelvis", -1, Box, 0.15f, 0.08f, 0.09f, 0f, 0.98f, 0f, 10f, 0, 0, 0, 0, 0.5f, 0.5f, 0, 0, 0);
        parts[1] = P("torso", 0, Box, 0.17f, 0.18f, 0.10f, 0f, 1.24f, 0f, 25f, 0f, 1.06f, 0f, 0.4f, 0.5f, 0.5f, 6000f, 400f, 900f);
        parts[2] = P("head", 1, Sph, 0.11f, 0f, 0f, 0f, 1.55f, 0f, 5f, 0f, 1.44f, 0f, 0.6f, 0.6f, 0.6f, 600f, 40f, 150f);
        for (int side = 0; side < 2; side++)
        {
            float sx = side == 0 ? 1f : -1f;
            int arm = 3 + 2 * side, leg = 7 + 3 * side;
            parts[arm] = P("upper arm", 1, Cap, 0.11f, 0.045f, 0f, sx * 0.22f, 1.225f, 0f, 2.5f, sx * 0.22f, 1.38f, 0f, 1.0f, 1.6f, 1.6f, 800f, 60f, 200f);
            parts[arm + 1] = P("forearm", arm, Cap, 0.095f, 0.04f, 0f, sx * 0.22f, 0.935f, 0f, 1.5f, sx * 0.22f, 1.07f, 0f, 0.3f, 2.4f, 0.1f, 500f, 40f, 120f);
            parts[leg] = P("thigh", 0, Cap, 0.13f, 0.08f, 0f, sx * 0.10f, 0.71f, 0f, 8f, sx * 0.10f, 0.92f, 0f, 0.6f, 1.2f, 1.0f, 12000f, 800f, 1500f);
            parts[leg + 1] = P("shin", leg, Cap, 0.15f, 0.06f, 0f, sx * 0.10f, 0.29f, 0f, 4f, sx * 0.10f, 0.50f, 0f, 0.1f, 2.3f, 0.08f, 9000f, 600f, 1200f);
            parts[leg + 2] = P("foot", leg + 1, Box, 0.06f, 0.04f, 0.15f, sx * 0.10f, 0.04f, 0.05f, 1.2f, sx * 0.10f, 0.08f, 0f, 0.3f, 0.6f, 0.4f, 5000f, 300f, 600f, 1.0f);
        }

        return new RagdollDef { Parts = parts };
    }
}
