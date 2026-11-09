using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Destruction;

/// <summary>Everything about a shape except its geometry that a fragment inherits from the shape it was cut from.</summary>
internal readonly struct ShapeTemplate
{
    public ShapeTemplate(float density, float friction, float restitution, float rolling, uint category, uint mask, int group, int layer)
    {
        Density = density; Friction = friction; Restitution = restitution; Rolling = rolling; Category = category; Mask = mask; Group = group; Layer = layer;
    }

    public float Density { get; }
    public float Friction { get; }
    public float Restitution { get; }
    public float Rolling { get; }
    public uint Category { get; }
    public uint Mask { get; }
    public int Group { get; }
    public int Layer { get; }
}

/// <summary>A convex shape of a body read back as a piece in the body's frame.</summary>
internal sealed class ShapeInfo
{
    public int Shape;
    public int Generation;
    public Piece Piece = null!;
    public ShapeTemplate Template;
}

/// <summary>What the destruction store needs from an engine; implemented by both engines (a 2D world embeds its plane at z = 0).</summary>
internal interface IDestructionHost
{
    WorldCore DestructionCore { get; }
    int EngineSectionId { get; }
    DestructionStore Destruction { get; }
    bool Planar { get; }
    /// <summary>Appends every shape of the body; false when one is not a convex polygon or polyhedron (then fracture is refused).</summary>
    bool ReadShapes(int body, List<ShapeInfo> into);
    /// <summary>True when the body takes part in no joint, vehicle, character, ragdoll or other machine, so replacing it cannot orphan anything.</summary>
    bool CanReplace(int body);
    Vector3F LocalCenter(int body);
    /// <summary>Rotates a body-frame vector into world axes.</summary>
    Vector3F ToWorld(int body, Vector3F bodyFrame);
    /// <summary>Rotates a world-axes vector into the body frame.</summary>
    Vector3F ToBody(int body, Vector3F world);
    /// <summary>World-frame inertia tensor times a vector (about the centre of mass).</summary>
    Vector3F InertiaTimes(int body, Vector3F v);
    /// <summary>Inverse world-frame inertia tensor times a vector.</summary>
    Vector3F InverseInertiaTimes(int body, Vector3F v);
    float MassOf(int body);
    Vector3F Gravity { get; }
    float GravityScaleOf(int body);
    long StepCount { get; }
    /// <summary>Creates a body with the source body's origin, rotation, damping, gravity scale and flags, no shapes and no velocity.</summary>
    BodyHandle CloneBody(int source, BodyType type);
    /// <summary>Adds a convex shape from float points in the body frame; throws a hull-building exception on a degenerate point set.</summary>
    ShapeHandle AddPiece(BodyHandle body, Vector3F[] points, in ShapeTemplate template);
    void DestroyBody(BodyHandle body);
}

/// <summary>Fracture settings shared by both dimensions (the public definitions convert to this).</summary>
internal sealed class FractureSettings
{
    public int CellCount = 16;
    public FracturePattern Pattern;
    public ulong Seed = 1;
    public Vector3F[] Points = [];
    public Vector3D? ImpactPoint;
    public float ImpactRadius;
    public float ClusterPower = 2f;
    public float ImpulseThreshold;
    public int Depth;
    public int ChildCellCount;
    public float ChildThresholdScale = 2f;
    public float MinFragmentVolume = 0.001f;
    public bool MergeSmall = true;

    public FractureSettings Clone()
    {
        var c = (FractureSettings)MemberwiseClone();
        c.Points = (Vector3F[])Points.Clone();
        return c;
    }
}

/// <summary>Bond-stress settings of a breakable compound.</summary>
internal sealed class BondSettings
{
    /// <summary>Tensile strength, Pa (N/m in 2D, per metre of thickness).</summary>
    public float Strength = 1e5f;
    /// <summary>Weight of the shear stress against the tensile stress in the failure measure.</summary>
    public float ShearFactor = 1f;
    /// <summary>Consecutive steps a bond must stay over strength before it breaks.</summary>
    public int OverloadSteps = 1;
}

internal sealed class Part
{
    public int Shape;
    public int ShapeGeneration;
    public bool Anchored;
    public float Measure;       // volume (3D) or area (2D) of the part
    public float Mass;          // kg
    public Vector3F Centroid;   // body frame
}

internal sealed class Bond
{
    public int A, B;
    public float Measure;       // shared face area (3D) or edge length (2D)
    public Vector3F Centroid;   // body frame
    public Vector3F Normal;     // from part A into part B
    public float Strength;      // Pa; 0 uses the record's strength
    public bool Alive = true;
    public int Overload;        // consecutive steps over strength
    public float Stress;        // last evaluated, Pa (transient: written to snapshots only for desync visibility)
}

/// <summary>What the store knows about one body: whether it fractures on impact, and its bond graph when it is a breakable compound.</summary>
internal sealed class Record
{
    public int Body, Generation;
    public FractureSettings? Fracture;
    public BondSettings? Bonds;
    public Part[] Parts = [];
    public Bond[] BondList = [];

    // Transient per step.
    public float Peak;
    public Vector3F PeakRel;
    public int LoadHead = -1;
}

/// <summary>A fragment tracked by the debris policy.</summary>
internal sealed class Fragment
{
    public int Body, Generation;
    public long Sequence;
    public long BirthStep;
    public float Measure;
    public int AsleepSteps;
    public bool Frozen;
}

/// <summary>One fracture, slice or bond break, for events.</summary>
internal readonly struct FractureRecord
{
    public FractureRecord(int kind, BodyHandle source, int firstFragment, int fragmentCount, float impulse, Vector3D point)
    {
        Kind = kind; Source = source; FirstFragment = firstFragment; FragmentCount = fragmentCount; Impulse = impulse; Point = point;
    }

    public int Kind { get; }
    public BodyHandle Source { get; }
    public int FirstFragment { get; }
    public int FragmentCount { get; }
    public float Impulse { get; }
    public Vector3D Point { get; }
}

/// <summary>Why a body broke.</summary>
public enum FractureKind
{
    /// <summary>A contact impulse above the body's threshold.</summary>
    Impact = 0,

    /// <summary>The caller asked for it.</summary>
    Manual = 1,

    /// <summary>Stress broke bonds and the compound fell apart into components.</summary>
    BondBreak = 2,

    /// <summary>The body was cut by a plane or line.</summary>
    Slice = 3,
}

/// <summary>Counters of the destruction features, cumulative over the world's life and saved with it.</summary>
public readonly struct DestructionCounters
{
    internal DestructionCounters(long fractures, long slices, long bondBreaks, long created, long alive, long culled, long frozen, long expired, long dropped, long merged)
    {
        Fractures = fractures; Slices = slices; BondBreaks = bondBreaks; FragmentsCreated = created; FragmentsAlive = alive; FragmentsCulled = culled; FragmentsFrozen = frozen; FragmentsExpired = expired; SliversDropped = dropped; SliversMerged = merged;
    }

    /// <summary>Bodies replaced by their cells (impact and manual).</summary>
    public long Fractures { get; }
    /// <summary>Bodies cut by a slice.</summary>
    public long Slices { get; }
    /// <summary>Bonds broken by stress.</summary>
    public long BondBreaks { get; }
    /// <summary>Bodies created by fracture, slicing and bond breaks.</summary>
    public long FragmentsCreated { get; }
    /// <summary>Fragments the debris policy currently tracks.</summary>
    public long FragmentsAlive { get; }
    /// <summary>Fragments removed to hold the budget.</summary>
    public long FragmentsCulled { get; }
    /// <summary>Fragments turned static after resting.</summary>
    public long FragmentsFrozen { get; }
    /// <summary>Fragments removed at the end of their lifetime.</summary>
    public long FragmentsExpired { get; }
    /// <summary>Cells too small or too thin to be a body that were deleted.</summary>
    public long SliversDropped { get; }
    /// <summary>Cells too small to be a body that were merged into a neighbour.</summary>
    public long SliversMerged { get; }
}
