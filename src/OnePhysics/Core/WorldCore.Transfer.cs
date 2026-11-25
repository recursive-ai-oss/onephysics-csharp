namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    /// <summary>The body's own sleep settings (the default value when it has none); for body-set transfer.</summary>
    internal BodySleepSettings BodySleepOf(int body) => _bodySleep.Get(body);

    /// <summary>Stores sleep settings for a live dynamic body without waking it or touching its sleep timer; for body-set transfer.</summary>
    internal void SetBodySleepRaw(int body, BodySleepSettings settings) => _bodySleep.Set(body, settings);

    /// <summary>The body's local time scale (1 when none was set); for body-set transfer.</summary>
    internal float BodyTimeScaleOf(int body) => ScaleStore.GetBodyScale(body);

    /// <summary>
    /// The colour the next <see cref="AddConstraint"/> should take when it is free, or -1. Body-set transfer sets it so that constraints keep the solve order they had in the source world; a colour that a body of the
    /// constraint already uses is ignored and the first free one is taken as usual. Cleared by the call it applies to.
    /// </summary>
    internal int ColorHint = -1;

    private byte AssignColor(int a, bool aDynamic, int b, bool bDynamic)
    {
        int hint = ColorHint;
        ColorHint = -1;
        return hint >= 0 ? _coloring.AssignPreferred(a, aDynamic, b, bDynamic, hint) : _coloring.Assign(a, aDynamic, b, bDynamic);
    }
}
