using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>One bond of a breakable compound as plain data, in world coordinates relative to the world origin.</summary>
public readonly struct DebugBond
{
    /// <summary>Creates a bond overlay.</summary>
    public DebugBond(BodyHandle body, int partA, int partB, Vector3D point, Vector3F normal, float area, float stress, float strength, bool intact)
    {
        Body = body; PartA = partA; PartB = partB; Point = point; Normal = normal; Area = area; Stress = stress; Strength = strength; Intact = intact;
    }

    /// <summary>The compound's body.</summary>
    public BodyHandle Body { get; }
    /// <summary>First part index.</summary>
    public int PartA { get; }
    /// <summary>Second part index.</summary>
    public int PartB { get; }
    /// <summary>Centre of the shared face (z = 0 in 2D).</summary>
    public Vector3D Point { get; }
    /// <summary>Unit normal of the shared face, from part A into part B, in world axes.</summary>
    public Vector3F Normal { get; }
    /// <summary>Face area in square metres (length in metres in 2D).</summary>
    public float Area { get; }
    /// <summary>Stress of the last step in pascals; 0 when broken or unloaded.</summary>
    public float Stress { get; }
    /// <summary>The stress at which this bond breaks, in pascals.</summary>
    public float Strength { get; }
    /// <summary>False once the bond has broken.</summary>
    public bool Intact { get; }
}

public sealed partial class DebugGeometry
{
    private DebugBond[] _bonds = new DebugBond[8];
    private int _nBonds;

    /// <summary>Bonds of every breakable compound in body slot order, then bond order; valid until Clear or the next world collection. Filled by <see cref="DebugContent.Destruction"/>.</summary>
    public ReadOnlySpan<DebugBond> Bonds => _bonds.AsSpan(0, _nBonds);

    /// <summary>Appends a bond overlay; grows owned storage when needed.</summary>
    public void Add(in DebugBond bond) => Push(ref _bonds, ref _nBonds, in bond);

    internal void ClearBonds() => _nBonds = 0;
}
