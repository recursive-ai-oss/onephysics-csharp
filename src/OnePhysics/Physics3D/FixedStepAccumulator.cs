using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>A presentation pose at a body's origin, in metres and unit quaternion orientation. Does not change simulation state.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct BodyPose
{
    /// <summary>Creates a presentation value. Does not validate or normalize inputs; never feeds simulation.</summary>
    public BodyPose(Vector3D position, QuaternionF rotation) { Position = position; Rotation = rotation; }
    /// <summary>Position of the body origin, in metres.</summary>
    public Vector3D Position { get; }
    /// <summary>Body orientation.</summary>
    public QuaternionF Rotation { get; }
}

/// <summary>Caller-driven fixed-step accumulator with render interpolation. Supplies a fixed dt to the world; never reads a clock. Retains previous/current poses by slot and generation. Allocate once; subsequent frames allocate only when the body-slot high-water mark grows. Presentation state is not snapshotted or hashed. Use one helper per world and call Resynchronize after external stepping, teleports, restore or origin shifting.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class FixedStepAccumulator
{
    private readonly FixedStepClock _clock;
    private BodyPose[] _previous = Array.Empty<BodyPose>(), _current = Array.Empty<BodyPose>();
    private int[] _generation = Array.Empty<int>();

    /// <summary>Creates a helper. Defaults: 1/60 s fixed dt, at most 8 steps per frame. Captures initial poses. Backlog beyond the step limit is carried without dropping time.</summary>
    /// <exception cref="ArgumentNullException">World is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Fixed dt is not finite and positive, or the step limit is not positive.</exception>
    public FixedStepAccumulator(PhysicsWorld3D world, float fixedDt = 1f / 60f, int maximumStepsPerFrame = 8)
    {
        ArgumentNullException.ThrowIfNull(world); World = world;
        _clock = new(fixedDt, maximumStepsPerFrame); Resynchronize();
    }
    /// <summary>The world advanced by this helper. Its own Step API remains clock-free.</summary>
    public PhysicsWorld3D World { get; }
    /// <summary>Fixed step duration, in seconds.</summary>
    public float FixedDt => _clock.FixedDt;
    /// <summary>Maximum completed steps per Advance call.</summary>
    public int MaximumStepsPerFrame => _clock.MaximumStepsPerFrame;
    /// <summary>Unconsumed caller time, in seconds, including any capped backlog.</summary>
    public double Remainder => _clock.Remainder;
    /// <summary>Remainder/fixed dt, clamped to 0..1. A capped backlog renders the latest pose (alpha 1). Normally renders one fixed step behind the simulation.</summary>
    public double InterpolationAlpha => _clock.Alpha;

    /// <summary>Adds caller-supplied frame time in seconds and completes at most MaximumStepsPerFrame fixed steps. Zero frame time may drain a carried backlog. Time is consumed only after a successful Step. Returns the number of completed steps. World exceptions propagate.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Frame time is negative/non-finite, or adding it overflows the remainder.</exception>
    public int Advance(double frameDt)
    {
        _clock.Accumulate(frameDt);
        int completed = 0;
        while (completed < MaximumStepsPerFrame && _clock.CanStep)
        {
            CaptureBeforeStep(); World.Step(FixedDt); _clock.CommitStep(); completed++;
            CaptureAfterStep();
        }
        return completed;
    }

    /// <summary>Captures live poses as both previous and current without changing world or carried time. Call after teleports, restore, origin shifts or external stepping to avoid interpolating across discontinuities.</summary>
    public void Resynchronize() { CaptureBeforeStep(); CaptureAfterStep(); Array.Copy(_current, _previous, _current.Length); }

    /// <summary>Last pre-step pose. Newly created bodies use their current pose until captured. Throws for a stale/default body or a body belonging to a different world.</summary>
    /// <exception cref="InvalidHandleException">Body is invalid or belongs to another world.</exception>
    public BodyPose GetPreviousPose(Body body) { int b = Resolve(body); return IsCaptured(body, b) ? _previous[b] : Read(body); }
    /// <summary>Last post-step pose. Newly created bodies use their current pose until captured. Throws for a stale/default body or a body belonging to a different world.</summary>
    /// <exception cref="InvalidHandleException">Body is invalid or belongs to another world.</exception>
    public BodyPose GetCurrentPose(Body body) { int b = Resolve(body); return IsCaptured(body, b) ? _current[b] : Read(body); }

    /// <summary>Interpolates position linearly in double and orientation along the shortest arc using InterpolationAlpha. Endpoints return exact stored poses. Does not feed the world.</summary>
    /// <exception cref="InvalidHandleException">Body is invalid or belongs to another world.</exception>
    public BodyPose GetInterpolatedPose(Body body)
    {
        var a = GetPreviousPose(body); var b = GetCurrentPose(body); double alpha = InterpolationAlpha;
        if (alpha == 0) return a;
        if (alpha == 1) return b;
        var rotation = QuaternionF.Slerp(a.Rotation, b.Rotation, (float)alpha);
        return new(new(a.Position.X + (b.Position.X - a.Position.X) * alpha, a.Position.Y + (b.Position.Y - a.Position.Y) * alpha, a.Position.Z + (b.Position.Z - a.Position.Z) * alpha), rotation);
    }

    private int Resolve(Body body)
    {
        if (!ReferenceEquals(body.World, World)) throw new InvalidHandleException("Body belongs to a different world or is the default value.");
        return World.ResolveBody(body.RawHandle);
    }
    private bool IsCaptured(Body body, int b) => b < _generation.Length && _generation[b] == body.RawHandle.Generation;
    private static BodyPose Read(Body body) => new(body.Position, body.Rotation);
    private void EnsureCapacity()
    {
        int n = World.Core.Bodies.HighWater;
        if (n <= _current.Length) return;
        int capacity = Math.Max(n, Math.Max(16, _current.Length * 2));
        Array.Resize(ref _current, capacity); Array.Resize(ref _previous, capacity); Array.Resize(ref _generation, capacity);
    }
    private void CaptureBeforeStep()
    {
        EnsureCapacity();
        foreach (var body in World.Bodies)
        {
            int b = body.RawHandle.Index; var pose = Read(body);
            _previous[b] = pose; _current[b] = pose; _generation[b] = body.RawHandle.Generation;
        }
    }
    private void CaptureAfterStep()
    {
        EnsureCapacity();
        foreach (var body in World.Bodies)
        {
            int b = body.RawHandle.Index; var pose = Read(body);
            if (!IsCaptured(body, b)) _previous[b] = pose;
            _current[b] = pose; _generation[b] = body.RawHandle.Generation;
        }
    }
}
