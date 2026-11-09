using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>What produced a <see cref="DebugAeroForce"/>.</summary>
public enum DebugAeroKind : byte
{
    /// <summary>An aerodynamic surface on a shape.</summary>
    Surface = 0,

    /// <summary>A thruster.</summary>
    Thruster = 1,

    /// <summary>The hull drag of a boat, at its centre of mass.</summary>
    Hull = 2,

    /// <summary>A keel, rudder or hydrofoil of a boat.</summary>
    Foil = 3,

    /// <summary>A propeller of a boat.</summary>
    Propeller = 4,

    /// <summary>A lift fan of a hovercraft.</summary>
    Fan = 5,

    /// <summary>A propulsion thruster of a hovercraft.</summary>
    HoverThruster = 6,
}

/// <summary>A lifting or bluff surface in world space, in the order shape surfaces by shape slot, then boat foils by boat slot. World-relative metres; planar worlds leave z at 0.</summary>
public readonly struct DebugAeroSurface
{
    /// <summary>Creates an entry.</summary>
    public DebugAeroSurface(BodyHandle body, DebugAeroKind kind, int index, Vector3D center, Vector3F chord, Vector3F normal, float area, bool lifting)
    { Body = body; Kind = kind; Index = index; Center = center; Chord = chord; Normal = normal; Area = area; IsLifting = lifting; }

    /// <summary>The body the surface is on.</summary>
    public BodyHandle Body { get; }

    /// <summary>Surface or foil.</summary>
    public DebugAeroKind Kind { get; }

    /// <summary>Shape slot for a surface, foil index within its boat for a foil.</summary>
    public int Index { get; }

    /// <summary>Where the force acts.</summary>
    public Vector3D Center { get; }

    /// <summary>Unit chord (leading edge toward trailing edge); zero for a bluff body.</summary>
    public Vector3F Chord { get; }

    /// <summary>Unit lifting side; zero for a bluff body.</summary>
    public Vector3F Normal { get; }

    /// <summary>Reference area in m² (m per metre of depth in a planar world); 0 while it is still to be derived from the shape.</summary>
    public float Area { get; }

    /// <summary>True for a lifting surface, false for a bluff body.</summary>
    public bool IsLifting { get; }
}

/// <summary>A force of the last step as an arrow: where it acts, which way and how hard (newtons; per metre of depth in a planar world). Zero forces are not emitted.</summary>
public readonly struct DebugAeroForce
{
    /// <summary>Creates an entry.</summary>
    public DebugAeroForce(BodyHandle body, DebugAeroKind kind, int index, Vector3D point, Vector3F force)
    { Body = body; Kind = kind; Index = index; Point = point; Force = force; }

    /// <summary>The body pushed.</summary>
    public BodyHandle Body { get; }

    /// <summary>What made it.</summary>
    public DebugAeroKind Kind { get; }

    /// <summary>Shape slot, thruster slot, boat slot or hovercraft slot, by kind; for a foil, propeller, fan or hover thruster the boat or hovercraft slot (and see the order of entries).</summary>
    public int Index { get; }

    /// <summary>Where it acts, world coordinates.</summary>
    public Vector3D Point { get; }

    /// <summary>The force, world axes.</summary>
    public Vector3F Force { get; }
}

public sealed partial class DebugGeometry
{
    private DebugAeroSurface[] _aeroSurfaces = new DebugAeroSurface[4];
    private int _nAeroSurfaces;
    private DebugAeroForce[] _aeroForces = new DebugAeroForce[8];
    private int _nAeroForces;

    /// <summary>Aerodynamic surfaces and boat foils, valid until the next collection or Clear.</summary>
    public ReadOnlySpan<DebugAeroSurface> AeroSurfaces => _aeroSurfaces.AsSpan(0, _nAeroSurfaces);

    /// <summary>Forces of the last step from surfaces, thrusters, boats and hovercraft: surfaces by shape slot, thrusters by slot, then boats and hovercraft by slot (each: hull, foils, propellers; fans, thrusters).</summary>
    public ReadOnlySpan<DebugAeroForce> AeroForces => _aeroForces.AsSpan(0, _nAeroForces);

    /// <summary>Appends a surface; owned storage grows when necessary.</summary>
    public void Add(in DebugAeroSurface s) => Push(ref _aeroSurfaces, ref _nAeroSurfaces, in s);

    /// <summary>Appends a force arrow; owned storage grows when necessary.</summary>
    public void Add(in DebugAeroForce f) => Push(ref _aeroForces, ref _nAeroForces, in f);
}
