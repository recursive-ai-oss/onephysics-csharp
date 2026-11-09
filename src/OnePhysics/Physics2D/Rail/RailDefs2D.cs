using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A side-view rail vehicle: a car on steel wheels that run along a <see cref="TrackDef"/>'s vertical profile (grades, crests and sags), with traction and brakes limited by wheel-rail adhesion, and couplers
/// (buffers and a draw gear with slack) to its neighbours in a train. A 2D world has no lateral dynamics, so the track guides the car by its profile only: the wheels sweep for the rail like the wheels of a
/// <see cref="Vehicle"/>, with steel tyres whose friction is the track's adhesion coefficient. Axes: the car faces +X, up is +Y.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public sealed class RailDef
{
    /// <summary>Mass of the car, kg. Default 30000.</summary>
    public float CarMass { get; set; } = 30000f;

    /// <summary>Length and height of the car's box, m. Default 12 × 3. The centre of mass is the box centre.</summary>
    public Vector2F CarSize { get; set; } = new(12f, 3f);

    /// <summary>Height of the car's floor above the rail heads, m: the box stands on this clearance. Default 1.</summary>
    public float CarFloorHeight { get; set; } = 1f;

    /// <summary>Number of wheelsets (side view: one wheel each), 2..8. Default 4.</summary>
    public int Axles { get; set; } = 4;

    /// <summary>Distance between the first and the last wheelset, m; the others are spread evenly. Default 8.</summary>
    public float AxleSpan { get; set; } = 8f;

    /// <summary>Wheel radius, m. Default 0.46.</summary>
    public float WheelRadius { get; set; } = 0.46f;

    /// <summary>Moment of inertia of a wheelset about its axle, kg m². Default 100.</summary>
    public float WheelsetSpinInertia { get; set; } = 100f;

    /// <summary>Natural frequency of the wheel-rail normal spring, Hz. Default 20. Within 1..25.</summary>
    public float ContactHertz { get; set; } = 20f;

    /// <summary>Largest traction force at the rails over all driven wheelsets, N. Default 0 (a trailing car).</summary>
    public float MaxTractionForce { get; set; }

    /// <summary>Largest power, W; the force falls as power over speed above it. Default 0 means no power limit.</summary>
    public float MaxPower { get; set; }

    /// <summary>Bit mask of the driven wheelsets in order from the rear. Default 0.</summary>
    public uint DrivenWheelsets { get; set; }

    /// <summary>Largest braking force at the rails over all wheelsets, N. Keep it below the adhesion limit (friction times the weight). Default 100000.</summary>
    public float MaxBrakeForce { get; set; } = 100000f;

    /// <summary>Distance from the car's centre to each coupler along its length, m. Default 6.25.</summary>
    public float CouplerReach { get; set; } = 6.25f;

    /// <summary>Height of the couplers above the rail head, m. Default 1.</summary>
    public float CouplerHeight { get; set; } = 1f;

    /// <summary>Distance between two coupled couplers at rest, m. Default 0.6.</summary>
    public float CouplerFreeLength { get; set; } = 0.6f;

    /// <summary>Slack of the draw gear beyond the free length, m. Default 0.05.</summary>
    public float CouplerSlack { get; set; } = 0.05f;

    /// <summary>Stiffness of the buffers in compression, N/m. Default 4e6.</summary>
    public float BufferStiffness { get; set; } = 4e6f;

    /// <summary>Stiffness of the draw gear in tension, N/m. Default 8e6.</summary>
    public float DrawStiffness { get; set; } = 8e6f;

    /// <summary>Damping of both, N·s/m. Default 2e5.</summary>
    public float CouplerDamping { get; set; } = 2e5f;

    /// <summary>Tension at which the draw gear breaks, N. Default 1.5e6; infinity never breaks.</summary>
    public float CouplerBreakForce { get; set; } = 1.5e6f;

    /// <summary>A copy.</summary>
    public RailDef Clone() => (RailDef)MemberwiseClone();

    /// <summary>Checks every value; throws an <see cref="ArgumentOutOfRangeException"/> naming the field, the value and the bound.</summary>
    public void Validate()
    {
        VehicleCheck2D.Positive(CarMass, nameof(CarMass)); VehicleCheck2D.Within(CarMass, 1f, 1e7f, nameof(CarMass));
        VehicleCheck2D.Vec(CarSize, nameof(CarSize));
        if (!(CarSize.X > 0f && CarSize.Y > 0f)) { throw new ArgumentOutOfRangeException(nameof(CarSize), CarSize, "CarSize must be positive."); }
        VehicleCheck2D.Within(CarFloorHeight, 0f, 10f, nameof(CarFloorHeight));
        if (Axles < 2 || Axles > 8) { throw new ArgumentOutOfRangeException(nameof(Axles), Axles, "Axles must be 2..8."); }
        VehicleCheck2D.Positive(AxleSpan, nameof(AxleSpan)); VehicleCheck2D.Within(AxleSpan, 0.1f, 100f, nameof(AxleSpan));
        VehicleCheck2D.Positive(WheelRadius, nameof(WheelRadius)); VehicleCheck2D.Within(WheelRadius, 0.05f, 3f, nameof(WheelRadius));
        VehicleCheck2D.Positive(WheelsetSpinInertia, nameof(WheelsetSpinInertia)); VehicleCheck2D.Within(WheelsetSpinInertia, 1e-2f, 1e7f, nameof(WheelsetSpinInertia));
        VehicleCheck2D.Within(ContactHertz, 1f, 25f, nameof(ContactHertz));
        VehicleCheck2D.Within(MaxTractionForce, 0f, 1e8f, nameof(MaxTractionForce)); VehicleCheck2D.Within(MaxPower, 0f, 1e10f, nameof(MaxPower)); VehicleCheck2D.Within(MaxBrakeForce, 0f, 1e8f, nameof(MaxBrakeForce));
        if (DrivenWheelsets >> Axles != 0u) { throw new ArgumentOutOfRangeException(nameof(DrivenWheelsets), DrivenWheelsets, "DrivenWheelsets names a wheelset the vehicle does not have."); }
        VehicleCheck2D.Within(CouplerReach, 0f, 100f, nameof(CouplerReach)); VehicleCheck2D.Within(CouplerHeight, -10f, 10f, nameof(CouplerHeight));
        VehicleCheck2D.Within(CouplerFreeLength, 0.01f, 10f, nameof(CouplerFreeLength)); VehicleCheck2D.Within(CouplerSlack, 0f, 5f, nameof(CouplerSlack));
        VehicleCheck2D.Within(BufferStiffness, 1f, 1e10f, nameof(BufferStiffness)); VehicleCheck2D.Within(DrawStiffness, 1f, 1e10f, nameof(DrawStiffness)); VehicleCheck2D.Within(CouplerDamping, 0f, 1e9f, nameof(CouplerDamping));
        if (!(CouplerBreakForce > 0f)) { throw new ArgumentOutOfRangeException(nameof(CouplerBreakForce), CouplerBreakForce, "CouplerBreakForce must be positive (infinity never breaks)."); }
    }

    /// <summary>A tram: a 20 t, 12 m car on four wheelsets, driven on all of them (60 kN, 400 kW).</summary>
    public static RailDef Tram => new() { CarMass = 20000f, CarSize = new Vector2F(12f, 2.8f), CarFloorHeight = 0.6f, WheelRadius = 0.34f, WheelsetSpinInertia = 45f, AxleSpan = 7.5f, MaxTractionForce = 60000f, MaxPower = 400000f, DrivenWheelsets = 0xF, MaxBrakeForce = 60000f, CouplerHeight = 0.9f };

    /// <summary>A freight wagon: a 40 t, 10 m car on two wheelsets 5.5 m apart, no traction.</summary>
    public static RailDef Freight => new() { CarMass = 36000f, CarSize = new Vector2F(9.5f, 2.6f), Axles = 2, AxleSpan = 5.5f, MaxBrakeForce = 60000f, CouplerReach = 5.25f };

    /// <summary>A locomotive: an 80 t, 18 m body on four wheelsets, driven on all of them (300 kN, 3 MW).</summary>
    public static RailDef Locomotive => new() { CarMass = 68000f, CarSize = new Vector2F(18f, 3.8f), AxleSpan = 13f, WheelsetSpinInertia = 150f, MaxTractionForce = 300000f, MaxPower = 3000000f, DrivenWheelsets = 0xF, MaxBrakeForce = 250000f, CouplerReach = 9.25f };
}
