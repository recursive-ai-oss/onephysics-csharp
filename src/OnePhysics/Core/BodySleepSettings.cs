using System;

namespace RecursiveAI.OnePhysics;

/// <summary>Per-body sleep policy in both dimensions. The default value inherits the world's sleep threshold and time. A policy set explicitly tests linear and angular speed separately; the whole island must be at rest before sleeping.</summary>
public readonly struct BodySleepSettings
{
    /// <summary>Overrides sleep thresholds, in m/s and rad/s, and the required continuous rest duration in seconds. Thresholds must be finite and nonnegative; duration must be finite and positive. Does not override a disabled world sleep option or a body's NeverSleep flag.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A threshold or duration is outside its range.</exception>
    public BodySleepSettings(float linearThreshold, float angularThreshold, float timeToSleep)
    {
        if (!float.IsFinite(linearThreshold) || linearThreshold < 0) throw new ArgumentOutOfRangeException(nameof(linearThreshold), linearThreshold, "Sleep threshold must be finite and nonnegative.");
        if (!float.IsFinite(angularThreshold) || angularThreshold < 0) throw new ArgumentOutOfRangeException(nameof(angularThreshold), angularThreshold, "Sleep threshold must be finite and nonnegative.");
        if (!float.IsFinite(timeToSleep) || timeToSleep <= 0) throw new ArgumentOutOfRangeException(nameof(timeToSleep), timeToSleep, "Time to sleep must be finite and positive.");
        LinearThreshold = linearThreshold; AngularThreshold = angularThreshold; TimeToSleep = timeToSleep; IsOverride = true;
    }

    /// <summary>False for the default value, which inherits world settings.</summary>
    public bool IsOverride { get; }
    /// <summary>Maximum linear speed in m/s while at rest; ignored when IsOverride is false.</summary>
    public float LinearThreshold { get; }
    /// <summary>Maximum angular speed in rad/s while at rest; ignored when IsOverride is false.</summary>
    public float AngularThreshold { get; }
    /// <summary>Continuous rest required in seconds; ignored when IsOverride is false.</summary>
    public float TimeToSleep { get; }
}
