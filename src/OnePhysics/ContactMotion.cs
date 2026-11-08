namespace RecursiveAI.OnePhysics;

/// <summary>Optional contact speeds for audio and haptics. Metres/second, nonnegative; zero when collection is disabled. Impact is captured before solving at manifold birth; sliding and rolling describe the end of the step.</summary>
public readonly struct ContactMotion
{
    /// <summary>Approach speed at first manifold creation, in metres/second.</summary>
    public float ImpactSpeed { get; }
    /// <summary>Largest residual tangential relative point speed, in metres/second.</summary>
    public float SlidingSpeed { get; }
    /// <summary>Largest relative rolling angular speed times the smaller contact arm length, in metres/second.</summary>
    public float RollingSpeed { get; }
    internal ContactMotion(float impact, float sliding, float rolling) { ImpactSpeed = impact; SlidingSpeed = sliding; RollingSpeed = rolling; }
    internal static ContactMotion Max(ContactMotion a, ContactMotion b) => new(System.Math.Max(a.ImpactSpeed, b.ImpactSpeed), System.Math.Max(a.SlidingSpeed, b.SlidingSpeed), System.Math.Max(a.RollingSpeed, b.RollingSpeed));
}
