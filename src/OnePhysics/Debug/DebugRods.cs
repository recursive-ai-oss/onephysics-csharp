using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>One rod segment, ordered by rod slot then segment index. World-relative metres; material X axis is the segment tangent in its rest state.</summary>
public readonly struct DebugRodSegment
{
    /// <summary>Creates a segment overlay with a material frame and circular radius in metres.</summary>
    public DebugRodSegment(int rod, int segment, Vector3D start, Vector3D end, QuaternionF frame, float radius)
    { Rod = rod; Segment = segment; Start = start; End = end; Frame = frame; Radius = radius; }
    /// <summary>Rod slot.</summary>
    public int Rod { get; }
    /// <summary>Segment index within the rod.</summary>
    public int Segment { get; }
    /// <summary>First particle, metres.</summary>
    public Vector3D Start { get; }
    /// <summary>Second particle, metres.</summary>
    public Vector3D End { get; }
    /// <summary>Material orientation at the segment midpoint.</summary>
    public QuaternionF Frame { get; }
    /// <summary>Circular section radius, metres.</summary>
    public float Radius { get; }
}

public sealed partial class DebugGeometry
{
    private DebugRodSegment[] _rodSegments = new DebugRodSegment[4];
    private int _nRodSegments;
    /// <summary>Rod segment polylines and frames in slot and segment order, valid until the next collection or Clear.</summary>
    public ReadOnlySpan<DebugRodSegment> RodSegments => _rodSegments.AsSpan(0,_nRodSegments);
    /// <summary>Appends a rod segment; owned storage grows when necessary.</summary>
    public void Add(in DebugRodSegment segment) => Push(ref _rodSegments,ref _nRodSegments,in segment);
}
