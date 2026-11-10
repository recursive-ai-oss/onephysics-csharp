using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>One shape of a link: the geometry and its surface and filtering settings.</summary>
public readonly struct ArticulationShape
{
    /// <summary>Creates a link shape.</summary>
    public ArticulationShape(ShapeSpec3D spec, ShapeDef def = default) { Spec = spec; Def = def; }

    /// <summary>The geometry, in the link's frame (use <see cref="ShapeDef.LocalPosition"/> and <see cref="ShapeDef.LocalRotation"/> to place it).</summary>
    public ShapeSpec3D Spec { get; }

    /// <summary>Density, friction, filtering and the other shape settings; the defaults are those of any shape.</summary>
    public ShapeDef Def { get; }
}

/// <summary>
/// One link of an articulation: where it hangs from its parent, how its joint moves, its mass and its shapes. A link's frame is its joint frame: the shapes, the centre of mass and the joint axis are given in it.
/// The layout maps one to one onto a URDF link and its parent joint (origin, axis, limits, effort and velocity limits, inertial block).
/// </summary>
public sealed class ArticulationLinkDef
{
    /// <summary>Index of the parent link in <see cref="ArticulationDef.Links"/>, which must be an earlier link; -1 for the root (the first link). Default -1.</summary>
    public int Parent { get; set; } = -1;

    /// <summary>How this link moves relative to its parent. Ignored for the root, whose motion is <see cref="ArticulationDef.FixedBase"/>. Default <see cref="ArticulationJointKind.Fixed"/>.</summary>
    public ArticulationJointKind Joint { get; set; }

    /// <summary>Joint axis in the link's frame, revolute and prismatic joints; scaled to unit length. Must be finite and non-zero. Default +X.</summary>
    public Vector3F Axis { get; set; } = new(1f, 0f, 0f);

    /// <summary>Position of the link's frame at joint position zero, in the parent link's frame, metres. Default zero.</summary>
    public Vector3F Origin { get; set; }

    /// <summary>Orientation of the link's frame at joint position zero, in the parent link's frame. Any non-zero finite quaternion (normalised). Default identity.</summary>
    public QuaternionF OriginRotation { get; set; } = QuaternionF.Identity;

    /// <summary>Mass in kg, finite and at least 0. 0 (default) derives the mass, centre of mass and inertia from the shapes' densities.</summary>
    public float Mass { get; set; }

    /// <summary>Centre of mass in the link's frame, metres; used when <see cref="Mass"/> is above 0. Default the link's origin.</summary>
    public Vector3F CenterOfMass { get; set; }

    /// <summary>Inertia tensor about the centre of mass in the link's axes, kg·m²; symmetric and positive semi-definite. Used when <see cref="Mass"/> is above 0; the default zero tensor is a point mass.</summary>
    public Matrix3x3F InertiaTensor { get; set; }

    /// <summary>Whether <see cref="LowerLimit"/> and <see cref="UpperLimit"/> apply (revolute: radians, prismatic: metres). Default false.</summary>
    public bool HasLimits { get; set; }

    /// <summary>Lower joint limit when <see cref="HasLimits"/>.</summary>
    public float LowerLimit { get; set; }

    /// <summary>Upper joint limit when <see cref="HasLimits"/>; at least <see cref="LowerLimit"/>.</summary>
    public float UpperLimit { get; set; }

    /// <summary>Largest rotation angle of a spherical joint from its zero orientation, radians, 0 to 2π; 0 (default) for none.</summary>
    public float SwingLimit { get; set; }

    /// <summary>Position and velocity drive of the joint. Default none.</summary>
    public ArticulationDrive Drive { get; set; }

    /// <summary>Dry friction of the joint: the largest force or torque it resists motion with, finite, at least 0. Default 0.</summary>
    public float Friction { get; set; }

    /// <summary>Viscous damping of the joint (URDF's dynamics damping), force per velocity, at least 0. Default 0.</summary>
    public float Damping { get; set; }

    /// <summary>Rotor inertia added to the joint (kg·m² or kg), at least 0. Default 0.</summary>
    public float Armature { get; set; }

    /// <summary>Largest joint speed (URDF's velocity limit), at least 0; 0 (default) for none.</summary>
    public float MaxVelocity { get; set; }

    /// <summary>Joint position at creation (radians or metres). Default 0.</summary>
    public float InitialPosition { get; set; }

    /// <summary>Joint orientation at creation for a spherical joint. Default identity.</summary>
    public QuaternionF InitialRotation { get; set; } = QuaternionF.Identity;

    /// <summary>Target orientation of a spherical joint's drive. Default identity.</summary>
    public QuaternionF TargetRotation { get; set; } = QuaternionF.Identity;

    /// <summary>The link's shapes, in its frame. Each becomes a shape of the link's body, which collides, is queried and raises events like any other.</summary>
    public List<ArticulationShape> Shapes { get; } = new();

    /// <summary>Adds a shape and returns the link.</summary>
    public ArticulationLinkDef AddShape(ShapeSpec3D spec, ShapeDef def = default) { Shapes.Add(new ArticulationShape(spec, def)); return this; }
}

/// <summary>
/// A tree of rigid links joined by revolute, prismatic, spherical and fixed joints, simulated in joint coordinates so it can never come apart, with a fixed or floating base. Links are listed parents first; the first is the root.
/// Create one with <see cref="PhysicsWorld3D.AddArticulation(ArticulationDef)"/>; the presets (<see cref="RobotArm"/>, <see cref="Quadruped"/>, <see cref="Chain"/>) fill in a complete definition.
/// </summary>
public sealed class ArticulationDef
{
    /// <summary>The links, parents first. At least 1, at most 2048.</summary>
    public List<ArticulationLinkDef> Links { get; } = new();

    /// <summary>True for a base fixed in the world (an arm on a table); false (default) for a floating base the other links move (a walker, a vehicle).</summary>
    public bool FixedBase { get; set; }

    /// <summary>World position of the root link's frame, metres, within ±2^21. Default zero.</summary>
    public Vector3D Position { get; set; }

    /// <summary>World orientation of the root link's frame. Default identity.</summary>
    public QuaternionF Rotation { get; set; } = QuaternionF.Identity;

    /// <summary>
    /// Substeps per world step for the articulation's own integration and contact solve, 1 to 64, or 0 (default) for automatic: at least 4, and more when the articulation is stiff enough for the explicit part of the step to need them
    /// (a taut chain under a heavy end: the substep count then grows with the square root of the mass ratio, up to 64).
    /// </summary>
    public int Substeps { get; set; }

    /// <summary>Iterations of the contact and joint-limit solve in each substep, 0 to 256. Default 8.</summary>
    public int Iterations { get; set; } = 8;

    /// <summary>Whether links that are not neighbours collide with each other. Neighbouring links never collide. Default false.</summary>
    public bool SelfCollision { get; set; }

    /// <summary>Adds the force that holds the present pose against gravity to every joint each substep, so drives only have to move the arm. Default false.</summary>
    public bool GravityCompensation { get; set; }

    /// <summary>Multiplier on the world's gravity for this articulation. Default 1.</summary>
    public float GravityScale { get; set; } = 1f;

    /// <summary>Adds a link and returns it.</summary>
    public ArticulationLinkDef AddLink(ArticulationLinkDef link) { Links.Add(link); return link; }

    private static Matrix3x3F Rod(float mass, float length, float radius) =>
        new(mass * (3f * radius * radius + length * length) / 12f, 0f, 0f, 0f, 0.5f * mass * radius * radius, 0f, 0f, 0f, mass * (3f * radius * radius + length * length) / 12f);

    /// <summary>
    /// A serial arm on a fixed base: <paramref name="joints"/> revolute joints alternating about the up axis and a horizontal axis, links as capsules along +Y, each joint with a position drive and gravity compensation on.
    /// The default is a seven-joint arm about 0.9 m long reaching straight up.
    /// </summary>
    /// <param name="reach">Length of the whole arm, metres; above 0.</param>
    /// <param name="joints">Number of joints, 1 to 64.</param>
    /// <param name="mass">Total mass of the arm, kg; above 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public static ArticulationDef RobotArm(float reach = 0.9f, int joints = 7, float mass = 8f)
    {
        if (!(reach > 0f) || !float.IsFinite(reach)) { throw new ArgumentOutOfRangeException(nameof(reach), reach, "reach must be finite and above 0."); }
        if (joints < 1 || joints > 64) { throw new ArgumentOutOfRangeException(nameof(joints), joints, "joints must be within 1..64."); }
        if (!(mass > 0f) || !float.IsFinite(mass)) { throw new ArgumentOutOfRangeException(nameof(mass), mass, "mass must be finite and above 0."); }
        var def = new ArticulationDef { FixedBase = true, GravityCompensation = true };
        float len = reach / joints, m = mass / joints, radius = Math.Min(0.04f, 0.2f * len);
        // The base is a short fat capsule standing on the base frame's plane (its lowest point is at height 0) and the first joint sits on top of it.
        var baseLink = new ArticulationLinkDef { Mass = 2f * m, InertiaTensor = Rod(2f * m, 0.1f, 0.06f), CenterOfMass = new Vector3F(0f, 0.085f, 0f) };
        baseLink.AddShape(ShapeSpec3D.Capsule(0.025f, 0.06f), new ShapeDef { LocalPosition = new Vector3F(0f, 0.085f, 0f) });
        def.AddLink(baseLink);
        for (int i = 0; i < joints; i++)
        {
            // The effective inertia of what hangs beyond this joint, a thin rod each, for a drive that settles in about a third of a second.
            float beyond = (joints - i) * m, arm = 0.5f * (joints - i) * len;
            float inertia = beyond * arm * arm / 1.5f + 0.01f;   // the drive's force limit is four times the torque that holds the arm out horizontally, so a push cannot crush what it meets
            var link = new ArticulationLinkDef
            {
                Parent = i, Joint = ArticulationJointKind.Revolute, Axis = i % 2 == 0 ? new Vector3F(0f, 1f, 0f) : new Vector3F(1f, 0f, 0f),
                Origin = new Vector3F(0f, i == 0 ? 0.11f : len, 0f), Mass = m, CenterOfMass = new Vector3F(0f, 0.5f * len, 0f), InertiaTensor = Rod(m, len, radius),
                Drive = ArticulationDrive.Position(inertia, 0.3f, maxForce: 5f + 4f * 9.81f * beyond * arm), Damping = 0.02f, HasLimits = i % 2 == 1, LowerLimit = -2.4f, UpperLimit = 2.4f,
            };
            link.AddShape(ShapeSpec3D.Capsule(0.5f * (len - 2f * radius), radius), new ShapeDef { LocalPosition = new Vector3F(0f, 0.5f * len, 0f), Density = m / (3.14159f * radius * radius * len) });
            def.AddLink(link);
        }

        return def;
    }

    /// <summary>
    /// A hanging chain on a fixed base: <paramref name="links"/> revolute links about z, each of length <paramref name="linkLength"/> and mass <paramref name="linkMass"/>, the last carrying <paramref name="endMass"/>. The chain lies along +X from its base.
    /// With <paramref name="endMass"/> = 1000 × <paramref name="linkMass"/> it is the classic mass-ratio stress test.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public static ArticulationDef Chain(int links = 16, float linkLength = 0.25f, float linkMass = 1f, float endMass = 1f)
    {
        if (links < 1 || links > 2047) { throw new ArgumentOutOfRangeException(nameof(links), links, "links must be within 1..2047."); }
        if (!(linkLength > 0f) || !float.IsFinite(linkLength)) { throw new ArgumentOutOfRangeException(nameof(linkLength), linkLength, "linkLength must be finite and above 0."); }
        if (!(linkMass > 0f) || !float.IsFinite(linkMass)) { throw new ArgumentOutOfRangeException(nameof(linkMass), linkMass, "linkMass must be finite and above 0."); }
        if (!(endMass > 0f) || !float.IsFinite(endMass)) { throw new ArgumentOutOfRangeException(nameof(endMass), endMass, "endMass must be finite and above 0."); }
        var def = new ArticulationDef { FixedBase = true };
        def.AddLink(new ArticulationLinkDef { Mass = linkMass, InertiaTensor = Rod(linkMass, 0.05f, 0.05f) });
        float radius = 0.4f * linkLength;
        for (int i = 0; i < links; i++)
        {
            float m = i == links - 1 ? endMass : linkMass;
            var link = new ArticulationLinkDef
            {
                Parent = i, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0f, 0f, 1f), Origin = new Vector3F(i == 0 ? 0f : linkLength, 0f, 0f),
                Mass = m, CenterOfMass = new Vector3F(0.5f * linkLength, 0f, 0f), InertiaTensor = new Matrix3x3F(0.5f * m * radius * radius * 0.5f, 0, 0, 0, m * linkLength * linkLength / 12f + 0.25f * m * radius * radius, 0, 0, 0, m * linkLength * linkLength / 12f + 0.25f * m * radius * radius),
            };
            // A capsule lying along the link (its axis is Y, so it is turned a quarter about z).
            link.AddShape(ShapeSpec3D.Capsule(0.5f * linkLength, Math.Min(radius, 0.05f)), new ShapeDef { LocalPosition = new Vector3F(0.5f * linkLength, 0f, 0f), LocalRotation = new QuaternionF(0f, 0f, -0.70710678f, 0.70710678f), Density = 1f });
            def.AddLink(link);
        }

        return def;
    }

    /// <summary>
    /// A four-legged walker on a floating base: a torso box (forward +X, up +Y) and four legs of three joints each (hip abduction about X, hip and knee pitch about Z) with position drives set to a standing pose. About 0.55 m long and 12 kg.
    /// </summary>
    /// <param name="scale">Uniform size multiplier (mass scales with its cube); above 0.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="scale"/> is not finite and above 0.</exception>
    public static ArticulationDef Quadruped(float scale = 1f)
    {
        if (!(scale > 0f) || !float.IsFinite(scale)) { throw new ArgumentOutOfRangeException(nameof(scale), scale, "scale must be finite and above 0."); }
        float s = scale, s3 = s * s * s;
        var def = new ArticulationDef();
        float torsoM = 8f * s3;
        var torso = new ArticulationLinkDef { Mass = torsoM, InertiaTensor = new Matrix3x3F(torsoM / 12f * (0.15f * 0.15f + 0.25f * 0.25f) * s * s, 0, 0, 0, torsoM / 12f * (0.5f * 0.5f + 0.25f * 0.25f) * s * s, 0, 0, 0, torsoM / 12f * (0.5f * 0.5f + 0.15f * 0.15f) * s * s) };
        torso.AddShape(ShapeSpec3D.Box(new Vector3F(0.5f * s, 0.15f * s, 0.25f * s)));
        def.AddLink(torso);
        float thigh = 0.25f * s, shin = 0.25f * s, r = 0.03f * s;
        float hipM = 0.35f * s3, thighM = 0.6f * s3, shinM = 0.4f * s3;
        for (int leg = 0; leg < 4; leg++)
        {
            float x = (leg < 2 ? 1f : -1f) * 0.2f * s, z = (leg % 2 == 0 ? 1f : -1f) * 0.125f * s;
            int hip = def.Links.Count;
            // Hip abduction: a small link at the torso's corner turning about the forward axis.
            def.AddLink(new ArticulationLinkDef
            {
                Parent = 0, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(1f, 0f, 0f), Origin = new Vector3F(x, -0.075f * s, z), Mass = hipM, InertiaTensor = new Matrix3x3F(1e-3f * s3 * s * s, 0, 0, 0, 1e-3f * s3 * s * s, 0, 0, 0, 1e-3f * s3 * s * s),
                HasLimits = true, LowerLimit = -0.6f, UpperLimit = 0.6f, Drive = new ArticulationDrive { Stiffness = 60f * s3 * s * s, Damping = 3f * s3 * s * s, MaxForce = 40f * s3 * s },
            });
            var up = new ArticulationLinkDef
            {
                Parent = hip, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0f, 0f, 1f), Mass = thighM, CenterOfMass = new Vector3F(0f, -0.5f * thigh, 0f),
                InertiaTensor = new Matrix3x3F(thighM * thigh * thigh / 12f, 0, 0, 0, 0.5f * thighM * r * r, 0, 0, 0, thighM * thigh * thigh / 12f),
                HasLimits = true, LowerLimit = -1.2f, UpperLimit = 1.2f, Drive = new ArticulationDrive { Stiffness = 70f * s3 * s * s, Damping = 3f * s3 * s * s, MaxForce = 40f * s3 * s, TargetPosition = 0.45f }, InitialPosition = 0.45f,
            };
            up.AddShape(ShapeSpec3D.Capsule(0.5f * thigh - r, r), new ShapeDef { LocalPosition = new Vector3F(0f, -0.5f * thigh, 0f), Friction = 0.9f, Density = 1f });
            int upIndex = def.AddLinkIndex(up);
            var lower = new ArticulationLinkDef
            {
                Parent = upIndex, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0f, 0f, 1f), Origin = new Vector3F(0f, -thigh, 0f), Mass = shinM, CenterOfMass = new Vector3F(0f, -0.5f * shin, 0f),
                InertiaTensor = new Matrix3x3F(shinM * shin * shin / 12f, 0, 0, 0, 0.5f * shinM * r * r, 0, 0, 0, shinM * shin * shin / 12f),
                HasLimits = true, LowerLimit = -2.2f, UpperLimit = -0.05f, Drive = new ArticulationDrive { Stiffness = 70f * s3 * s * s, Damping = 3f * s3 * s * s, MaxForce = 40f * s3 * s, TargetPosition = -0.9f }, InitialPosition = -0.9f,
            };
            lower.AddShape(ShapeSpec3D.Capsule(0.5f * shin - r, r), new ShapeDef { LocalPosition = new Vector3F(0f, -0.5f * shin, 0f), Friction = 0.9f, Density = 1f });
            lower.AddShape(ShapeSpec3D.Sphere(1.3f * r), new ShapeDef { LocalPosition = new Vector3F(0f, -shin, 0f), Friction = 1.2f, Density = 1f });
            def.AddLink(lower);
        }

        return def;
    }

    private int AddLinkIndex(ArticulationLinkDef link) { Links.Add(link); return Links.Count - 1; }

}
