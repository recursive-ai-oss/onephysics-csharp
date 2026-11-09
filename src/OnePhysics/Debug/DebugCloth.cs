using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>What a <see cref="DebugClothLink"/> joins.</summary>
public enum DebugClothLinkKind : byte
{
    /// <summary>A seam between two soft-body particles.</summary>
    Seam = 0,

    /// <summary>A particle held to a point of a rigid body.</summary>
    Attachment = 1,

    /// <summary>A long-range tether from a particle to its anchor.</summary>
    Tether = 2,
}

/// <summary>A line to draw for a seam, an attachment or a tether: from <see cref="From"/> (the cloth particle) to <see cref="To"/> (the other particle, the attached point of the rigid body, or the anchor). World-relative metres.</summary>
public readonly struct DebugClothLink
{
    /// <summary>Creates an entry.</summary>
    public DebugClothLink(DebugClothLinkKind kind, int softBody, int particle, Vector3D from, Vector3D to, float length)
    { Kind = kind; SoftBody = softBody; Particle = particle; From = from; To = to; Length = length; }

    /// <summary>What the line joins.</summary>
    public DebugClothLinkKind Kind { get; }

    /// <summary>Slot of the soft body that owns <see cref="Particle"/>.</summary>
    public int SoftBody { get; }

    /// <summary>The cloth particle.</summary>
    public int Particle { get; }

    /// <summary>The particle's position.</summary>
    public Vector3D From { get; }

    /// <summary>The other end.</summary>
    public Vector3D To { get; }

    /// <summary>A tether's longest allowed length; 0 for seams and attachments.</summary>
    public float Length { get; }
}

/// <summary>The aerodynamic force on one triangle of a cloth, for drawing as an arrow from the triangle's centre. Newtons, world-relative.</summary>
public readonly struct DebugClothWind
{
    /// <summary>Creates an entry.</summary>
    public DebugClothWind(int softBody, int triangle, Vector3D center, Vector3F force)
    { SoftBody = softBody; Triangle = triangle; Center = center; Force = force; }

    /// <summary>Slot of the soft body.</summary>
    public int SoftBody { get; }

    /// <summary>Triangle number in the body's mesh.</summary>
    public int Triangle { get; }

    /// <summary>Centre of the triangle.</summary>
    public Vector3D Center { get; }

    /// <summary>Force from the relative flow (wind of the area fields minus the triangle's velocity), N.</summary>
    public Vector3F Force { get; }
}
