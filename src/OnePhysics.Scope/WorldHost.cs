using System;
using System.Globalization;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// What the module needs from a world, written once for the planar and the spatial engine. The two worlds have the same
/// member names with different math types, so each gets a small adapter and everything else is shared.
/// </summary>
internal abstract class WorldHost
{
    /// <summary>2 or 3.</summary>
    public abstract int Dimension { get; }

    /// <summary>The world's id, which its handles carry.</summary>
    public abstract int Id { get; }

    /// <summary>The world's completed step count (absolute, as the replay format counts).</summary>
    public abstract long StepCount { get; }

    /// <summary>The origin that debug geometry is relative to.</summary>
    public abstract (double X, double Y, double Z) Origin { get; }

    /// <summary>OnePhysics' own world hash.</summary>
    public abstract ulong Hash();

    public abstract void Step(float dt);

    public abstract byte[] Snapshot();

    public abstract void Restore(ReadOnlySpan<byte> snapshot);

    public abstract ReadOnlySpan<PhysicsEvent> Events { get; }

    public abstract StepCounters Counters { get; }

    public abstract void CollectDebug(DebugContent content, DebugGeometry into);

    /// <summary>Starts the world's own recorder; used once to read the initial records a replay begins with.</summary>
    public abstract ReplayRecorder StartRecording(Stream stream);

    public abstract bool IsBodyAlive(BodyHandle body);

    public abstract BodyType? TypeOf(BodyHandle body);

    /// <summary>Applies a poke to a body. The caller has checked the body is alive.</summary>
    public abstract void ApplyPoke(BodyHandle body, in StepOp op);

    /// <summary>Fills an entity's inspector rows. False when the body no longer exists.</summary>
    public abstract bool InspectBody(BodyHandle body, PropertyList into);

    /// <summary>Fills a joint's inspector rows. False when the joint no longer exists.</summary>
    public abstract bool InspectJoint(JointHandle joint, PropertyList into);

    protected const string NoDetail = "forces and impulses appear after the first step";

    /// <summary>Row helper: a vector value from doubles.</summary>
    protected static ScopeValue Vec(double x, double y, double z) =>
        ScopeValue.OfVector(new System.Numerics.Vector3((float)x, (float)y, (float)z));

    protected static string Num(float v) => v.ToString("R", CultureInfo.InvariantCulture);
}
