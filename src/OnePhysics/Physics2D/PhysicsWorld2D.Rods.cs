using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Deterministic diagnostics of the last rod phase, initially zero. Units and sampling are documented by RodStepStatistics.</summary>
    [Experimental("OPHYS0001")]
    public RodStepStatistics RodStatistics => _engine.Rods.Statistics;
    /// <summary>Number of live rods, ropes, cables and hair strands; initially zero.</summary>
    [Experimental("OPHYS0001")]
    public int RodCount => _engine.Rods.Count;
    /// <summary>Gets the live rod in a stable slot. Throws ArgumentOutOfRangeException for a dead or out-of-range slot.</summary>
    [Experimental("OPHYS0001")]
    public Rod GetRod(int index)
    {
        var store = _engine.Rods;
        if ((uint)index >= (uint)store.Data.Length || store.Data[index] == null) throw new ArgumentOutOfRangeException(nameof(index),index,"No live rod in this slot.");
        return new Rod(store,index,store.Generations[index]);
    }
    /// <summary>Adds a copied rod definition. Throws ArgumentNullException or ArgumentOutOfRangeException before modifying the world.</summary>
    [Experimental("OPHYS0001")]
    public Rod AddRod(RodDef definition)
    {
        ArgumentNullException.ThrowIfNull(definition); definition.Validate();
        var points = new Vector3D[definition.Points.Length];
        for (int i = 0; i < points.Length; i++) { var p = definition.Points[i]; points[i] = new Vector3D(p.X,p.Y,0); }
        foreach (var a in definition.Attachments) _engine.Rods.ValidateAttachment(a.Body,new Vector3F(a.LocalAnchor.X,a.LocalAnchor.Y,0),a.BreakForce);
        var v = definition.Velocity;
        var data = new RodParticles(points,definition.Material,true,definition.PinnedParticles,definition.ClampedSegments,new Vector3F(v.X,v.Y,0),definition.Substeps,definition.Iterations,definition.Damping,definition.GravityScale,definition.ClampStart);
        data.Friction=definition.Friction; data.Restitution=definition.Restitution; data.CollisionEnabled=definition.CollisionEnabled; data.SelfCollision=definition.SelfCollision; data.CollideWithAttachedBodies=definition.CollideWithAttachedBodies;
        data.Category=definition.CategoryBits; data.Mask=definition.MaskBits; data.Group=definition.GroupIndex; data.Layer=definition.Layer;
        foreach (var a in definition.Attachments) _engine.Rods.Bind(data,a.Particle,a.Body,new Vector3F(a.LocalAnchor.X,a.LocalAnchor.Y,0),a.BreakForce);
        var h = _engine.Rods.Add(data); return new Rod(_engine.Rods,h.Slot,h.Generation);
    }
    /// <summary>Adds a straight flexible rope, default 32 segments, first end pinned, last end free. Throws ArgumentOutOfRangeException for invalid coordinates or segment count 1..4096.</summary>
    [Experimental("OPHYS0001")]
    public Rod AddRope(Vector2D from, Vector2D to, int segments = 32, bool pinStart = true, bool pinEnd = false)
        => AddRod(LineRod(from,to,segments,pinStart,pinEnd,RodMaterial.Rope,false));
    /// <summary>Adds a straight stiff cable, default 32 segments and a clamped first end. Last end free. Same limits as AddRope.</summary>
    [Experimental("OPHYS0001")]
    public Rod AddCable(Vector2D from, Vector2D to, int segments = 32, bool pinStart = true, bool pinEnd = false)
        => AddRod(LineRod(from,to,segments,pinStart,pinEnd,RodMaterial.Cable,true));
    /// <summary>Adds a thin hair strand, default 16 segments and a clamped first end. Same limits as AddRope.</summary>
    [Experimental("OPHYS0001")]
    public Rod AddHair(Vector2D from, Vector2D to, int segments = 16)
        => AddRod(LineRod(from,to,segments,true,false,RodMaterial.Hair,true));
    private static RodDef LineRod(Vector2D from, Vector2D to, int segments, bool pinStart, bool pinEnd, RodMaterial material, bool clamp)
    {
        if (segments < 1 || segments > 4096) throw new ArgumentOutOfRangeException(nameof(segments));
        var points = new Vector2D[segments+1];
        for (int i = 0; i <= segments; i++) points[i] = from+(to-from)*((double)i/segments);
        return new RodDef { Points = points, Material = material, PinnedParticles = pinStart ? (pinEnd ? [0,segments] : [0]) : (pinEnd ? [segments] : []), ClampStart = clamp && pinStart };
    }
}
