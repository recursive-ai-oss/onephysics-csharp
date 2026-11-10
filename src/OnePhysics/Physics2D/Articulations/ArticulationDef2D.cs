using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>One shape of a link: the geometry and its surface and filtering settings.</summary>
public readonly struct ArticulationShape
{
    /// <summary>Creates a link shape.</summary>
    public ArticulationShape(Shape2D geometry, ShapeDef def = default) { Geometry = geometry; Def = def; }

    /// <summary>The geometry, in the link's frame.</summary>
    public Shape2D Geometry { get; }

    /// <summary>Density, friction, filtering and the other shape settings; the defaults are those of any shape.</summary>
    public ShapeDef Def { get; }
}

/// <summary>
/// One link of a planar articulation: where it hangs from its parent, how its joint moves, its mass and its shapes. A link's frame is its joint frame: the shapes, the centre of mass and the prismatic axis are given in it.
/// The layout maps one to one onto a URDF link and its parent joint (origin, axis, limits, effort and velocity limits, inertial block) of a model that lives in the plane. Masses are per unit depth.
/// </summary>
public sealed class ArticulationLinkDef
{
    /// <summary>Index of the parent link in <see cref="ArticulationDef.Links"/>, which must be an earlier link; -1 for the root (the first link). Default -1.</summary>
    public int Parent { get; set; } = -1;

    /// <summary>How this link moves relative to its parent: <see cref="ArticulationJointKind.Fixed"/> (default), <see cref="ArticulationJointKind.Revolute"/> (turns about the plane's normal) or <see cref="ArticulationJointKind.Prismatic"/> (slides along <see cref="Axis"/>). Spherical joints do not exist in 2D. Ignored for the root.</summary>
    public ArticulationJointKind Joint { get; set; }

    /// <summary>Sliding direction of a prismatic joint in the link's frame; scaled to unit length. Must be finite and non-zero. Default +X.</summary>
    public Vector2F Axis { get; set; } = new(1f, 0f);

    /// <summary>Position of the link's frame at joint position zero, in the parent link's frame, metres. Default zero.</summary>
    public Vector2F Origin { get; set; }

    /// <summary>Angle of the link's frame at joint position zero relative to the parent's, radians. Default 0.</summary>
    public float OriginAngle { get; set; }

    /// <summary>Mass in kg per metre of depth, finite and at least 0. 0 (default) derives the mass, centre of mass and inertia from the shapes' densities.</summary>
    public float Mass { get; set; }

    /// <summary>Centre of mass in the link's frame, metres; used when <see cref="Mass"/> is above 0. Default the link's origin.</summary>
    public Vector2F CenterOfMass { get; set; }

    /// <summary>Rotational inertia about the centre of mass, kg·m²; at least 0. Used when <see cref="Mass"/> is above 0; the default zero is a point mass.</summary>
    public float Inertia { get; set; }

    /// <summary>Whether <see cref="LowerLimit"/> and <see cref="UpperLimit"/> apply (revolute: radians, prismatic: metres). Default false.</summary>
    public bool HasLimits { get; set; }

    /// <summary>Lower joint limit when <see cref="HasLimits"/>.</summary>
    public float LowerLimit { get; set; }

    /// <summary>Upper joint limit when <see cref="HasLimits"/>; at least <see cref="LowerLimit"/>.</summary>
    public float UpperLimit { get; set; }

    /// <summary>Position and velocity drive of the joint. Default none.</summary>
    public ArticulationDrive Drive { get; set; }

    /// <summary>Dry friction of the joint: the largest force or torque it resists motion with, finite, at least 0. Default 0.</summary>
    public float Friction { get; set; }

    /// <summary>Viscous damping of the joint (URDF's dynamics damping), force per velocity, at least 0. Default 0.</summary>
    public float Damping { get; set; }

    /// <summary>Rotor inertia added to the joint, at least 0. Default 0.</summary>
    public float Armature { get; set; }

    /// <summary>Largest joint speed (URDF's velocity limit), at least 0; 0 (default) for none.</summary>
    public float MaxVelocity { get; set; }

    /// <summary>Joint position at creation (radians or metres). Default 0.</summary>
    public float InitialPosition { get; set; }

    /// <summary>The link's shapes, in its frame. Each becomes a shape of the link's body, which collides, is queried and raises events like any other.</summary>
    public List<ArticulationShape> Shapes { get; } = new();

    /// <summary>Adds a shape and returns the link.</summary>
    public ArticulationLinkDef AddShape(Shape2D geometry, ShapeDef def = default) { Shapes.Add(new ArticulationShape(geometry, def)); return this; }
}

/// <summary>
/// A tree of rigid links joined by revolute, prismatic and fixed joints in the plane, simulated in joint coordinates so it can never come apart, with a fixed or floating base. Links are listed parents first; the first is the root.
/// Create one with <see cref="PhysicsWorld2D.AddArticulation(ArticulationDef)"/>; the presets (<see cref="Arm"/>, <see cref="Chain"/>) fill in a complete definition.
/// </summary>
public sealed class ArticulationDef
{
    /// <summary>The links, parents first. At least 1, at most 2048.</summary>
    public List<ArticulationLinkDef> Links { get; } = new();

    /// <summary>True for a base fixed in the world; false (default) for a floating base (x, y and angle) the other links move.</summary>
    public bool FixedBase { get; set; }

    /// <summary>World position of the root link's frame, metres, within ±2^21. Default zero.</summary>
    public Vector2D Position { get; set; }

    /// <summary>World angle of the root link's frame, radians counter-clockwise. Default 0.</summary>
    public float Angle { get; set; }

    /// <summary>
    /// Substeps per world step for the articulation's own integration and contact solve, 1 to 64, or 0 (default) for automatic: at least 4, and more when the articulation is stiff enough for the explicit part of the step to need them.
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

    private static float Rod(float mass, float length, float radius) => mass * (3f * radius * radius + length * length) / 12f;

    /// <summary>
    /// A planar arm on a fixed base standing on the root frame's x axis: <paramref name="joints"/> revolute joints, links as capsules along +Y, each joint with a position drive and gravity compensation on. The default is a three-joint arm 0.9 m long reaching straight up.
    /// </summary>
    /// <param name="reach">Length of the whole arm, metres; above 0.</param>
    /// <param name="joints">Number of joints, 1 to 64.</param>
    /// <param name="mass">Total mass of the arm, kg per metre of depth; above 0.</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public static ArticulationDef Arm(float reach = 0.9f, int joints = 3, float mass = 4f)
    {
        if (!(reach > 0f) || !float.IsFinite(reach)) { throw new ArgumentOutOfRangeException(nameof(reach), reach, "reach must be finite and above 0."); }
        if (joints < 1 || joints > 64) { throw new ArgumentOutOfRangeException(nameof(joints), joints, "joints must be within 1..64."); }
        if (!(mass > 0f) || !float.IsFinite(mass)) { throw new ArgumentOutOfRangeException(nameof(mass), mass, "mass must be finite and above 0."); }
        var def = new ArticulationDef { FixedBase = true, GravityCompensation = true };
        float len = reach / joints, m = mass / joints, radius = Math.Min(0.04f, 0.2f * len);
        var baseLink = new ArticulationLinkDef { Mass = 2f * m, Inertia = Rod(2f * m, 0.1f, 0.06f), CenterOfMass = new Vector2F(0f, 0.085f) };
        baseLink.AddShape(Shape2D.CreateCapsule(new Vec2(0f, 0.06f), new Vec2(0f, 0.11f), 0.06f));
        def.AddLink(baseLink);
        for (int i = 0; i < joints; i++)
        {
            float beyond = (joints - i) * m, arm = 0.5f * (joints - i) * len;
            float inertia = beyond * arm * arm / 1.5f + 0.01f;   // the drive's force limit is four times the torque that holds the arm out horizontally, so a push cannot crush what it meets
            var link = new ArticulationLinkDef
            {
                Parent = i, Joint = ArticulationJointKind.Revolute, Origin = new Vector2F(0f, i == 0 ? 0.11f : len), Mass = m, CenterOfMass = new Vector2F(0f, 0.5f * len), Inertia = Rod(m, len, radius),
                Drive = ArticulationDrive.Position(inertia, 0.3f, maxForce: 5f + 4f * 9.81f * beyond * arm), Damping = 0.02f, HasLimits = true, LowerLimit = -2.6f, UpperLimit = 2.6f,
            };
            link.AddShape(Shape2D.CreateCapsule(new Vec2(0f, radius), new Vec2(0f, len - radius), radius));
            def.AddLink(link);
        }

        return def;
    }

    /// <summary>
    /// A hanging chain on a fixed base: <paramref name="links"/> revolute links, each of length <paramref name="linkLength"/> and mass <paramref name="linkMass"/>, the last carrying <paramref name="endMass"/>. The chain lies along +X from its base.
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
        def.AddLink(new ArticulationLinkDef { Mass = linkMass, Inertia = 0.01f });
        float radius = Math.Min(0.05f, 0.4f * linkLength);
        for (int i = 0; i < links; i++)
        {
            float m = i == links - 1 ? endMass : linkMass;
            var link = new ArticulationLinkDef
            {
                Parent = i, Joint = ArticulationJointKind.Revolute, Origin = new Vector2F(i == 0 ? 0f : linkLength, 0f), Mass = m, CenterOfMass = new Vector2F(0.5f * linkLength, 0f), Inertia = Rod(m, linkLength, radius),
            };
            link.AddShape(Shape2D.CreateCapsule(new Vec2(0f, 0f), new Vec2(linkLength, 0f), radius));
            def.AddLink(link);
        }

        return def;
    }
}
