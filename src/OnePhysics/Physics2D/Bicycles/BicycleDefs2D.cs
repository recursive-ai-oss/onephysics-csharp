using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A side-view bicycle: one rigid frame (with the rider and the wheels' mass) on two wheels that sweep for the ground like those of a <see cref="Vehicle"/>. A 2D world has no lean or steering, so what is left is
/// the pitch dynamics: the weight transfer under braking and driving, wheelies and stoppies, and the balance of front and rear brake. The rider shifts weight with the <c>lean</c> input of <see cref="Bicycle.Ride"/>.
/// Axes: the bicycle faces +X, up is +Y; positions are measured from the ground under the rear wheel's contact point.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class BicycleDef
{
    /// <summary>Distance between the two contact points, m. Default 1.02.</summary>
    public float WheelBase { get; set; } = 1.02f;

    /// <summary>Rear wheel radius, m. Default 0.3.</summary>
    public float RearRadius { get; set; } = 0.3f;

    /// <summary>Front wheel radius, m. Default 0.35.</summary>
    public float FrontRadius { get; set; } = 0.35f;

    /// <summary>Mass of the bicycle with the rider, kg. Default 94.</summary>
    public float Mass { get; set; } = 94f;

    /// <summary>Centre of mass ahead of the rear contact point, m. Default 0.342.</summary>
    public float CenterX { get; set; } = 0.342f;

    /// <summary>Height of the centre of mass above the ground, m. Default 0.861.</summary>
    public float CenterHeight { get; set; } = 0.861f;

    /// <summary>Pitch inertia of the whole bicycle about its centre of mass, with the wheels held fixed to the frame (their spin inertia included), kg m². Default 16.1.</summary>
    public float PitchInertia { get; set; } = 16.1f;

    /// <summary>Spin inertia of the rear wheel, kg m². Default 0.12.</summary>
    public float RearSpinInertia { get; set; } = 0.12f;

    /// <summary>Spin inertia of the front wheel, kg m². Default 0.28.</summary>
    public float FrontSpinInertia { get; set; } = 0.28f;

    /// <summary>Tyre friction coefficient. Default 1.</summary>
    public float TyreFriction { get; set; } = 1f;

    /// <summary>Natural frequency of a tyre's vertical spring, Hz. Default 15. Within 1..25.</summary>
    public float TyreHertz { get; set; } = 15f;

    /// <summary>Torque of the pedals on the rear wheel at full input, N·m. Default 15.</summary>
    public float MaxPedalTorque { get; set; } = 15f;

    /// <summary>Front brake torque at full input, N·m. Default 400.</summary>
    public float MaxFrontBrakeTorque { get; set; } = 400f;

    /// <summary>Rear brake torque at full input, N·m. Default 300.</summary>
    public float MaxRearBrakeTorque { get; set; } = 300f;

    /// <summary>Torque of the rider's weight shift on the frame at full input, N·m. Default 150.</summary>
    public float MaxLeanTorque { get; set; } = 150f;

    /// <summary>Pitch beyond which the bicycle counts as flipped, radians. Default 1.2. Within 0.1..3.</summary>
    public float FlipAngle { get; set; } = 1.2f;

    /// <summary>A copy.</summary>
    public BicycleDef Clone() => (BicycleDef)MemberwiseClone();

    /// <summary>Checks every value; throws an <see cref="ArgumentOutOfRangeException"/> naming the field, the value and the bound.</summary>
    public void Validate()
    {
        VehicleCheck2D.Positive(WheelBase, nameof(WheelBase)); VehicleCheck2D.Within(WheelBase, 0.1f, 20f, nameof(WheelBase));
        VehicleCheck2D.Positive(RearRadius, nameof(RearRadius)); VehicleCheck2D.Within(RearRadius, 0.01f, 10f, nameof(RearRadius));
        VehicleCheck2D.Positive(FrontRadius, nameof(FrontRadius)); VehicleCheck2D.Within(FrontRadius, 0.01f, 10f, nameof(FrontRadius));
        VehicleCheck2D.Positive(Mass, nameof(Mass)); VehicleCheck2D.Within(Mass, 1e-2f, 1e5f, nameof(Mass));
        VehicleCheck2D.Within(CenterX, -100f, 100f, nameof(CenterX)); VehicleCheck2D.Within(CenterHeight, 0f, 100f, nameof(CenterHeight));
        VehicleCheck2D.Positive(PitchInertia, nameof(PitchInertia)); VehicleCheck2D.Within(PitchInertia, 1e-4f, 1e6f, nameof(PitchInertia));
        VehicleCheck2D.Positive(RearSpinInertia, nameof(RearSpinInertia)); VehicleCheck2D.Within(RearSpinInertia, 1e-4f, 1e4f, nameof(RearSpinInertia));
        VehicleCheck2D.Positive(FrontSpinInertia, nameof(FrontSpinInertia)); VehicleCheck2D.Within(FrontSpinInertia, 1e-4f, 1e4f, nameof(FrontSpinInertia));
        VehicleCheck2D.Within(TyreFriction, 0f, 100f, nameof(TyreFriction)); VehicleCheck2D.Within(TyreHertz, 1f, 25f, nameof(TyreHertz));
        VehicleCheck2D.Within(MaxPedalTorque, 0f, 1e5f, nameof(MaxPedalTorque)); VehicleCheck2D.Within(MaxFrontBrakeTorque, 0f, 1e5f, nameof(MaxFrontBrakeTorque));
        VehicleCheck2D.Within(MaxRearBrakeTorque, 0f, 1e5f, nameof(MaxRearBrakeTorque)); VehicleCheck2D.Within(MaxLeanTorque, 0f, 1e5f, nameof(MaxLeanTorque));
        VehicleCheck2D.Within(FlipAngle, 0.1f, 3f, nameof(FlipAngle));
    }

    /// <summary>The side view of the benchmark bicycle of Meijaard et al. 2007: 94 kg in all, centre of mass 0.342 m ahead of the rear contact point and 0.861 m up (frame, rider and both wheels combined), 0.3 m and 0.35 m wheels.</summary>
    public static BicycleDef Benchmark => new() { WheelBase = 1.02f, RearRadius = 0.3f, FrontRadius = 0.35f, Mass = 94f, CenterX = 0.3421f, CenterHeight = 0.8612f, PitchInertia = 16.11f, RearSpinInertia = 0.12f, FrontSpinInertia = 0.28f };

    /// <summary>A road bicycle with a rider: 700c wheels, 1.0 m wheel base, 80 kg all told, centre of mass 0.34 m ahead of the rear contact point and 1.0 m up.</summary>
    public static BicycleDef Road => new()
    {
        WheelBase = 1.0f, RearRadius = 0.34f, FrontRadius = 0.34f, Mass = 80f, CenterX = 0.34f, CenterHeight = 1.0f, PitchInertia = 12f,
        RearSpinInertia = 0.15f, FrontSpinInertia = 0.15f, TyreFriction = 0.9f, MaxPedalTorque = 20f,
    };
}
