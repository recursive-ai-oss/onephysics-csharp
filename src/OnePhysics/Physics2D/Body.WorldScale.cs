using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>
    /// This body's own time scale, 1 by default, in [1/1024, 4]. A body's island advances <c>dt * scale</c> each step, at the smallest scale among its bodies and the time-scale regions that hold them
    /// (slow motion below 1, fast forward above 1). Dynamic bodies only; invalid or stale handles throw <see cref="InvalidHandleException"/>.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The scale is outside [1/1024, 4].</exception>
    /// <exception cref="BodyTypeException">The body is not dynamic (setter).</exception>
    public float TimeScale
    {
        get { var (w, _) = Resolve(); return w.Core.GetBodyTimeScale(_handle); }
        set { using var mutation = _world?.RecordMutation("Body.TimeScale"); var (w, _) = Resolve(); w.Core.SetBodyTimeScale(_handle, value); }
    }
}
