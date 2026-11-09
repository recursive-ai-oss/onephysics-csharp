namespace RecursiveAI.OnePhysics;

/// <summary>What kind of deformable a query hit belongs to.</summary>
public enum DeformableKind : byte
{
    /// <summary>A soft body (a volume or a closed or open surface without fabric).</summary>
    SoftBody = 0,

    /// <summary>Cloth: a soft body with fabric (3D) or a strip (2D).</summary>
    Cloth = 1,

    /// <summary>A rod: a rope, cable, hair strand or any other rod.</summary>
    Rod = 2,
}

/// <summary>Which kinds of deformable a query looks at (flags).</summary>
[System.Flags]
public enum DeformableKinds : byte
{
    /// <summary>No kind.</summary>
    None = 0,

    /// <summary>Soft bodies that are not cloth.</summary>
    SoftBodies = 1,

    /// <summary>Cloth and strips.</summary>
    Cloth = 2,

    /// <summary>Rods, ropes, cables and hair.</summary>
    Rods = 4,

    /// <summary>Every kind (the default of a filter).</summary>
    All = 7,
}

/// <summary>The part of a deformable a query hit: a particle, a surface triangle (3D soft bodies and cloth) or a segment (rod segments, 2D soft-body edges).</summary>
public enum DeformableElement : byte
{
    /// <summary>One particle, as a sphere (a disc in 2D) of the body's particle radius. Used for bodies with no surface.</summary>
    Particle = 0,

    /// <summary>A surface triangle of a 3D soft body or cloth, treated as the bare two-sided triangle (the particle radius is only a contact skin and is not part of it).</summary>
    Triangle = 1,

    /// <summary>A segment between two neighbouring particles: a rod segment (a capsule of the rod's radius) or an edge of a 2D soft body's outline or strip (a capsule of the particle radius).</summary>
    Segment = 2,
}
