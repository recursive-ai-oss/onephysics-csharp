using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Rods;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Generation-checked rod handle. Destroyed or default handles throw InvalidOperationException on access. The world owns all storage.</summary>
[Experimental("OPHYS0001")]
public readonly struct Rod
{
    private readonly RodStore? _store;
    private readonly int _generation;
    internal Rod(RodStore store, int index, int generation) { _store = store; Index = index; _generation = generation; }
    /// <summary>Stable rod slot, reused lowest-first after destruction.</summary>
    public int Index { get; }
    /// <summary>Whether the rod still exists in the original world.</summary>
    public bool IsValid => _store != null && _store.Alive(Index,_generation);
    private RodStore Store => IsValid ? _store! : throw new InvalidOperationException("Rod has been destroyed or is uninitialized.");
    private RodParticles Data => Store.Data[Index]!;
    /// <summary>Number of centerline particles (segments plus one).</summary>
    public int ParticleCount => Data.PositionData.Length;
    /// <summary>Read a particle position in metres. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public Vector3D GetPosition(int particle)
    {
        var d = Data; Check(particle,d.PositionData.Length); var p = d.PositionData[particle]; return new(p.X,p.Y,p.Z);
    }
    /// <summary>Read velocity in m/s. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public Vector3F GetVelocity(int particle)
    {
        var d = Data; Check(particle,d.PositionData.Length); var p = d.VelocityData[particle]; return new(p.X,p.Y,p.Z);
    }
    /// <summary>Pin or release a particle at its current position. Releasing restores its material-derived mass. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public void SetPinned(int particle, bool pinned)
    {
        var store = Store; var data = Data; Check(particle,data.PositionData.Length);
        using var mutation = store.Host.RodMutation("Rod.SetPinned"); data.SetPinned(particle,pinned);
    }
    /// <summary>Drive a pinned particle towards a target over the next step's substeps; later steps hold it there. Coordinates finite within ±2^21 metres, rounded to the world 2^-32 grid. Throws InvalidOperationException if not pinned.</summary>
    public void SetTarget(int particle, Vector3D target)
    {
        var store = Store; var data = Data; Check(particle,data.PositionData.Length);
        if (!data.Pinned[particle]) throw new InvalidOperationException("Only a pinned particle has a kinematic target.");
        if (!(double.IsFinite(target.X) && Math.Abs(target.X) <= 2097152 && double.IsFinite(target.Y) && Math.Abs(target.Y) <= 2097152 && double.IsFinite(target.Z) && Math.Abs(target.Z) <= 2097152))
            throw new ArgumentOutOfRangeException(nameof(target),target,"Target must be finite within ±2097152 metres.");
        using var mutation = store.Host.RodMutation("Rod.SetTarget");
        data.PinTarget[particle] = new Vector3D(WorldCore.Grid(target.X),WorldCore.Grid(target.Y),WorldCore.Grid(target.Z));
    }
    /// <summary>Set a particle velocity, m/s, finite components within ±10000. A pinned particle's velocity is determined by its target instead.</summary>
    public void SetVelocity(int particle, Vector3F velocity)
    {
        var store = Store; var data = Data; Check(particle,data.PositionData.Length);
        RodMaterial.Check(velocity.X,-10000,10000,nameof(velocity)); RodMaterial.Check(velocity.Y,-10000,10000,nameof(velocity)); RodMaterial.Check(velocity.Z,-10000,10000,nameof(velocity)); 
        using var mutation = store.Host.RodMutation("Rod.SetVelocity");
        data.VelocityData[particle] = new Vector3F(velocity.X,velocity.Y,velocity.Z);
    }
    /// <summary>Attaches a particle to a live body, replacing any pin/attachment. Local anchor is in metres in the body's authoring frame. Invalid identity or parameters throw before mutation.</summary>
    public void Attach(int particle, Body body, Vector3F localAnchor = default, float breakForce = float.MaxValue)
    {
        var store=Store; var d=Data; Check(particle,d.PositionData.Length);
        var anchor=new Vector3F(localAnchor.X,localAnchor.Y,localAnchor.Z);
        store.ValidateAttachment(body.RawHandle,anchor,breakForce);
        using var mutation=store.Host.RodMutation("Rod.Attach"); store.Bind(d,particle,body.RawHandle,anchor,breakForce);
    }
    /// <summary>Releases a body attachment; no effect on a world pin. Clears the broken flag and force readback.</summary>
    public void Detach(int particle)
    {
        var store=Store; var d=Data; Check(particle,d.PositionData.Length);
        using var mutation=store.Host.RodMutation("Rod.Detach"); d.AttachmentBody[particle]=-1; d.AttachmentBroken[particle]=false; d.AttachmentForce[particle]=default;
    }
    /// <summary>Average force on the particle from its body attachment during the last step, newtons; zero if never attached.</summary>
    public Vector3F GetAttachmentForce(int particle)
    { var d=Data; Check(particle,d.PositionData.Length); var f=d.AttachmentForce[particle]; return new(f.X,f.Y,f.Z); }
    /// <summary>True if the attachment broke by force or loss of the bound body's generation. Rebinding or Detach clears it.</summary>
    public bool IsAttachmentBroken(int particle) { var d=Data; Check(particle,d.PositionData.Length); return d.AttachmentBroken[particle]; }
    /// <summary>Prescribes a clamped segment's material orientation at the step boundary. Any finite nonzero quaternion is normalized. Throws InvalidOperationException unless this segment was clamped in the definition.</summary>
    public void SetFrameTarget(int segment,QuaternionF rotation)
    {
        var store=Store;var d=Data;Check(segment,d.FrameData.Length);
        if(!d.Clamped[segment])throw new InvalidOperationException("A frame target requires a clamped segment.");
        double norm=(double)rotation.X*rotation.X+(double)rotation.Y*rotation.Y+(double)rotation.Z*rotation.Z+(double)rotation.W*rotation.W;
        if(!(norm>1e-24&&norm<1e24))throw new ArgumentOutOfRangeException(nameof(rotation),rotation,"Expected a finite nonzero quaternion with squared length 1e-24..1e24.");
        var q=RodParticles.Unit(rotation);
        using var mutation=store.Host.RodMutation("Rod.SetFrameTarget");d.FrameData[segment]=q;d.AngularVelocity[segment]=default;
    }
    /// <summary>Maximum absolute relative length error over all segments, dimensionless.</summary>
    public double MaximumStretch => Data.MaximumStretch;
    /// <summary>Raw particle and segment arrays; experimental, single-writer access bypasses validation and recording.</summary>
    public RodParticles Particles => Data;
    /// <summary>Destroy the rod. Later accesses throw InvalidOperationException.</summary>
    public void Destroy() => Store.Remove(Index);
    private static void Check(int index, int count)
    {
        if ((uint)index >= (uint)count) throw new ArgumentOutOfRangeException(nameof(index),index,"Index outside rod storage.");
    }
}
