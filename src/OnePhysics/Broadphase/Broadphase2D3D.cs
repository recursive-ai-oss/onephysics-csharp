using System;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Broadphase;

/// <summary>The dynamic AABB tree for 2D: <see cref="Aabb2D"/> boxes, perimeter as the SAH measure.</summary>
public sealed class DynamicTree2D : DynamicTree<Aabb2D>
{
    /// <summary>Creates an empty tree.</summary>
    public DynamicTree2D(int initialCapacity = 64)
        : base(initialCapacity)
    {
    }
}

/// <summary>The dynamic AABB tree for 3D: <see cref="Aabb3D"/> boxes, surface area as the SAH measure.</summary>
public sealed class DynamicTree3D : DynamicTree<Aabb3D>
{
    /// <summary>Creates an empty tree.</summary>
    public DynamicTree3D(int initialCapacity = 64)
        : base(initialCapacity)
    {
    }
}

/// <summary>The 2D broadphase: three <see cref="Aabb2D"/> trees, the move buffer and pair finding. See <see cref="Broadphase{TBox}"/>.</summary>
public sealed class Broadphase2D : Broadphase<Aabb2D>
{
    /// <summary>Snapshot section id used when none is given.</summary>
    public const int DefaultSectionId = 0x42503244;

    /// <summary>Creates an empty broadphase.</summary>
    public Broadphase2D(BroadphaseOptions? options = null, int sectionId = DefaultSectionId)
        : base(options, sectionId)
    {
    }

    /// <summary>Reports new tight bounds and the displacement since the last report; see the span overload on the base class.</summary>
    public bool MoveProxy(int key, in Aabb2D tight, double displacementX, double displacementY)
    {
        Span<double> d = stackalloc double[2];
        d[0] = displacementX;
        d[1] = displacementY;
        return MoveProxy(key, tight, d);
    }
}

/// <summary>The 3D broadphase: three <see cref="Aabb3D"/> trees, the move buffer and pair finding. See <see cref="Broadphase{TBox}"/>.</summary>
public sealed class Broadphase3D : Broadphase<Aabb3D>
{
    /// <summary>Snapshot section id used when none is given.</summary>
    public const int DefaultSectionId = 0x42503344;

    /// <summary>Creates an empty broadphase.</summary>
    public Broadphase3D(BroadphaseOptions? options = null, int sectionId = DefaultSectionId)
        : base(options, sectionId)
    {
    }

    /// <summary>Reports new tight bounds and the displacement since the last report; see the span overload on the base class.</summary>
    public bool MoveProxy(int key, in Aabb3D tight, double displacementX, double displacementY, double displacementZ)
    {
        Span<double> d = stackalloc double[3];
        d[0] = displacementX;
        d[1] = displacementY;
        d[2] = displacementZ;
        return MoveProxy(key, tight, d);
    }
}
