using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Body
{
    /// <summary>Per-body sleep thresholds and rest duration. Default inherits world settings. Setting wakes the island and resets this body's rest timer. Only dynamic bodies accept overrides.</summary>
    /// <exception cref="InvalidHandleException">The body is invalid or stale.</exception>
    /// <exception cref="BodyTypeException">The body is not dynamic.</exception>
    public BodySleepSettings SleepSettings
    {
        get { var (w, _) = Resolve(); return w.Core.GetBodySleepSettings(_handle); }
        set { using var mutation = _world?.RecordMutation("Body.SleepSettings"); var (w, _) = Resolve(); w.Core.SetBodySleepSettings(_handle, value); }
    }

    /// <summary>When true this body prevents automatic sleep for its entire island. Default false (unless BodyDef.AllowSleep was false). Setting wakes the island and resets this body's rest timer. Does not prevent an explicit Sleep call. Dynamic bodies only.</summary>
    /// <exception cref="InvalidHandleException">The body is invalid or stale.</exception>
    /// <exception cref="BodyTypeException">The body is not dynamic.</exception>
    public bool NeverSleep
    {
        get { var (w, b) = Resolve(); return (w.Core.Bodies.Flags[b] & (byte)BodyBits.NoSleep) != 0; }
        set
        {
            using var mutation = _world?.RecordMutation("Body.NeverSleep");
            var (w, b) = Resolve();
            if (!w.Core.Bodies.IsDynamic(b)) throw new BodyTypeException(nameof(NeverSleep), _handle, w.Core.Bodies.TypeOf(b), true);
            if (value) w.Core.Bodies.Flags[b] |= (byte)BodyBits.NoSleep;
            else w.Core.Bodies.Flags[b] &= unchecked((byte)~(byte)BodyBits.NoSleep);
            w.Core.Wake(_handle); w.Core.Bodies.SleepTime[b] = 0;
        }
    }
}
