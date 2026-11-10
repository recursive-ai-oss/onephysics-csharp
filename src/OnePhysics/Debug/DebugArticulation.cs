using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>One link's joint of an articulation, in world space: where the joint is, which way it turns or slides, where it stands and whether it sits at a limit. World-relative metres; planar worlds leave z at 0.</summary>
public readonly struct DebugArticulationJoint
{
    /// <summary>Creates an entry.</summary>
    public DebugArticulationJoint(BodyHandle body, int articulation, int link, int parent, ArticulationJointKind kind, Vector3D anchor, Vector3F axis, Vector3D centerOfMass, float position, float velocity, bool atLimit)
    {
        Body = body; Articulation = articulation; Link = link; Parent = parent; Kind = kind; Anchor = anchor; Axis = axis; CenterOfMass = centerOfMass; Position = position; Velocity = velocity; AtLimit = atLimit;
    }

    /// <summary>The body of the link.</summary>
    public BodyHandle Body { get; }

    /// <summary>The articulation's slot.</summary>
    public int Articulation { get; }

    /// <summary>The link's index in its articulation.</summary>
    public int Link { get; }

    /// <summary>The parent link's index, or -1 for the root.</summary>
    public int Parent { get; }

    /// <summary>The kind of joint (the root reports <see cref="ArticulationJointKind.Fixed"/>, floating base or not).</summary>
    public ArticulationJointKind Kind { get; }

    /// <summary>The joint point: the origin of the link's frame.</summary>
    public Vector3D Anchor { get; }

    /// <summary>The joint axis in world axes, unit; zero for fixed and spherical joints.</summary>
    public Vector3F Axis { get; }

    /// <summary>The link's centre of mass.</summary>
    public Vector3D CenterOfMass { get; }

    /// <summary>The joint position (radians or metres); 0 for joints without a single coordinate.</summary>
    public float Position { get; }

    /// <summary>The joint velocity; 0 for joints without a single coordinate.</summary>
    public float Velocity { get; }

    /// <summary>True when the joint is within a millimetre (or a milliradian) of a limit.</summary>
    public bool AtLimit { get; }
}

/// <summary>A contact point of a link with another body in the last step: where, which way it pushes the other body, how deep, and the impulse the articulation's solve gave it.</summary>
public readonly struct DebugArticulationContact
{
    /// <summary>Creates an entry.</summary>
    public DebugArticulationContact(int articulation, int link, Vector3D point, Vector3F normal, float separation, float normalImpulse)
    {
        Articulation = articulation; Link = link; Point = point; Normal = normal; Separation = separation; NormalImpulse = normalImpulse;
    }

    /// <summary>The articulation's slot.</summary>
    public int Articulation { get; }

    /// <summary>The link's index.</summary>
    public int Link { get; }

    /// <summary>The contact point on the link at the start of the step.</summary>
    public Vector3D Point { get; }

    /// <summary>Unit normal pointing from the link towards the other body.</summary>
    public Vector3F Normal { get; }

    /// <summary>Gap at the start of the step (negative when overlapping).</summary>
    public float Separation { get; }

    /// <summary>The normal impulse of the last substep, newton-seconds.</summary>
    public float NormalImpulse { get; }
}

public sealed partial class DebugGeometry
{
    private DebugArticulationJoint[] _artJoints = new DebugArticulationJoint[8];
    private int _nArtJoints;
    private DebugArticulationContact[] _artContacts = new DebugArticulationContact[8];
    private int _nArtContacts;

    /// <summary>The joints of every articulation link, articulations by slot and links by index, valid until the next collection or Clear.</summary>
    public ReadOnlySpan<DebugArticulationJoint> ArticulationJoints => _artJoints.AsSpan(0, _nArtJoints);

    /// <summary>The contact points of articulation links in the last step, articulations by slot, in the order the solve took them.</summary>
    public ReadOnlySpan<DebugArticulationContact> ArticulationContacts => _artContacts.AsSpan(0, _nArtContacts);

    /// <summary>Appends a joint; owned storage grows when necessary.</summary>
    public void Add(in DebugArticulationJoint j) => Push(ref _artJoints, ref _nArtJoints, in j);

    /// <summary>Appends a contact point; owned storage grows when necessary.</summary>
    public void Add(in DebugArticulationContact c) => Push(ref _artContacts, ref _nArtContacts, in c);

    private void ClearArticulations() { _nArtJoints = 0; _nArtContacts = 0; }
}
