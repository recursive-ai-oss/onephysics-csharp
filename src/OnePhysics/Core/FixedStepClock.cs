using System;

namespace RecursiveAI.OnePhysics;

// Caller-supplied frame durations never enter world state. A capped frame carries its full backlog.
internal sealed class FixedStepClock
{
    public FixedStepClock(float fixedDt, int maximumStepsPerFrame)
    {
        if (!float.IsFinite(fixedDt) || fixedDt <= 0) throw new ArgumentOutOfRangeException(nameof(fixedDt), fixedDt, "Fixed dt must be finite and positive.");
        if (maximumStepsPerFrame <= 0) throw new ArgumentOutOfRangeException(nameof(maximumStepsPerFrame), maximumStepsPerFrame, "Maximum steps must be positive.");
        FixedDt = fixedDt; MaximumStepsPerFrame = maximumStepsPerFrame;
    }
    public float FixedDt { get; }
    public int MaximumStepsPerFrame { get; }
    public double Remainder { get; private set; }
    public double Alpha => Math.Min(1, Remainder / FixedDt);
    public bool CanStep => Remainder >= FixedDt;
    public void Accumulate(double frameDt)
    {
        if (!double.IsFinite(frameDt) || frameDt < 0 || !double.IsFinite(Remainder + frameDt)) throw new ArgumentOutOfRangeException(nameof(frameDt), frameDt, "Frame duration must be finite, nonnegative and must not overflow the carried remainder.");
        Remainder += frameDt;
    }
    public void CommitStep() => Remainder -= FixedDt;
}
