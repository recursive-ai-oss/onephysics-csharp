using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>A point-to-body rod attachment. Anchors are in the body's authoring frame (not its centre-of-mass frame), in metres.</summary>
public readonly struct RodAttachment
{
    /// <summary>Creates an attachment; validated against the destination world by AddRod. Break force is positive finite newtons, default float.MaxValue (effectively unbreakable).</summary>
    public RodAttachment(int particle, BodyHandle body, Vector2F localAnchor = default, float breakForce = float.MaxValue)
    { Particle=particle; Body=body; LocalAnchor=localAnchor; BreakForce=breakForce; }
    /// <summary>Particle index, 0..ParticleCount-1.</summary>
    public int Particle { get; }
    /// <summary>Body identity, including world and generation; must be alive when attached.</summary>
    public BodyHandle Body { get; }
    /// <summary>Body-local anchor, metres, finite components within ±2^21.</summary>
    public Vector2F LocalAnchor { get; }
    /// <summary>Force threshold in newtons. Exceeding it breaks before applying that correction; destroying the body also breaks the attachment.</summary>
    public float BreakForce { get; }
}
