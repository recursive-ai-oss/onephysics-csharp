using System;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    private readonly BodySleepStore _bodySleep;

    /// <summary>Gets a body's sleep policy. Default inherits world thresholds and duration. Validates world, slot and generation.</summary>
    /// <exception cref="InvalidHandleException">The handle is invalid or stale.</exception>
    public BodySleepSettings GetBodySleepSettings(BodyHandle body) => _bodySleep.Get(Resolve(body));

    /// <summary>Sets a dynamic body's sleep policy and wakes its island, resetting its rest timer. Default restores world policy. Static and kinematic bodies are rejected. Call only between steps.</summary>
    /// <exception cref="InvalidHandleException">The handle is invalid or stale.</exception>
    /// <exception cref="BodyTypeException">The body is not dynamic.</exception>
    public void SetBodySleepSettings(BodyHandle body, BodySleepSettings settings)
    {
        int b = Resolve(body);
        if (!_bodies.IsDynamic(b)) throw new BodyTypeException(nameof(SetBodySleepSettings), body, _bodies.TypeOf(b), true);
        _bodySleep.Set(b, settings); Wake(body); _bodies.SleepTime[b] = 0;
    }

    private float BodySleepLimit(int b, float inherited) { var settings = _bodySleep.Get(b); return settings.IsOverride ? settings.TimeToSleep : inherited; }
    private float AdjustedSleepTime(int b)
    {
        float value = _bodies.SleepTime[b]; var settings = _bodySleep.Get(b);
        return settings.IsOverride ? value - settings.TimeToSleep + _options.TimeToSleep : value;
    }
    private bool BodyIsAtRest(int b, float motion, float inherited)
    {
        var s = _bodySleep.Get(b);
        if (!s.IsOverride) return motion <= inherited;
        // The engine reports MaxValue for a body carried by a moving kinematic contact.
        if (motion == float.MaxValue) return false;
        var v = _bodies.LinearVelocityOf(b); var w = _bodies.AngularVelocityOf(b);
        double linear2 = 0, angular2 = 0;
        for (int i = 0; i < v.Length; i++) linear2 += (double)v[i] * v[i];
        for (int i = 0; i < w.Length; i++) angular2 += (double)w[i] * w[i];
        return linear2 <= (double)s.LinearThreshold * s.LinearThreshold && angular2 <= (double)s.AngularThreshold * s.AngularThreshold;
    }
}
