using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Fields;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>Plain-data field overlay, ordered by field slot. Positions in metres, direction/axis in world coordinates.</summary>
public readonly struct DebugField
{
    /// <summary>Constructs an overlay from a copied field definition.</summary>
    public DebugField(int index,FieldDefinition definition) { Index=index;Kind=definition.Kind;Minimum=definition.Minimum;Maximum=definition.Maximum;Center=definition.Center;Direction=definition.Direction;Axis=definition.Axis;Radius=definition.Radius;Strength=definition.Strength;SurfaceHeight=definition.SurfaceHeight; }
    /// <summary>Stable field slot.</summary>
    public int Index { get; }
    /// <summary>Effect kind.</summary>
    public FieldKind Kind { get; }
    /// <summary>Inclusive region minimum.</summary>
    public Vector3D Minimum { get; }
    /// <summary>Inclusive region maximum.</summary>
    public Vector3D Maximum { get; }
    /// <summary>Source/axis point.</summary>
    public Vector3D Center { get; }
    /// <summary>Constant vector or conveyor direction.</summary>
    public Vector3F Direction { get; }
    /// <summary>Fluid/platform normal or vortex/line axis.</summary>
    public Vector3F Axis { get; }
    /// <summary>Radial cutoff, metres.</summary>
    public float Radius { get; }
    /// <summary>Magnitude in the field's documented units.</summary>
    public float Strength { get; }
    /// <summary>Fluid surface dot(Axis,position), metres.</summary>
    public double SurfaceHeight { get; }
}

public sealed partial class DebugGeometry
{
    private DebugField[] _fields=new DebugField[4];
    private int _nFields;
    /// <summary>Fields in slot order, valid until Clear or the next world collection.</summary>
    public ReadOnlySpan<DebugField> Fields => _fields.AsSpan(0,_nFields);
    /// <summary>Appends a field overlay; grows owned storage when needed.</summary>
    public void Add(in DebugField field) => Push(ref _fields,ref _nFields,in field);
}
