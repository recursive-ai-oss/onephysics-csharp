using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A rail vehicle: one car on one or two bogies (or directly on its wheelsets), with the wheel-rail contact model that makes it run on a <see cref="TrackDef"/> without any constraint pinning it to the line.
/// Every wheelset is a rigid body with conical treads and flanges; the wheel-rail normal force is a stiff spring-damper where the wheel meets the rail head, the tangential force is Kalker's linear creep law with
/// the Shen-Hedrick-Elkins saturation at the friction limit (Kalker, <i>Three-Dimensional Elastic Bodies in Rolling Contact</i>, 1990; Shen, Hedrick and Elkins, Vehicle System Dynamics 12, 1983), and when the
/// lateral play is used up the flange meets the rail's gauge face with a steep contact angle, so hunting, curving, flange climb and derailment all come from the contact forces.
/// Suspensions are six-degree-of-freedom spring joints between wheelsets, bogies and the car. Axes of the vehicle: +X right, +Y up, +Z forward.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class RailDef
{
    // ---- car
    /// <summary>Mass of the car body, kg (zero for a bare wheelset: <see cref="Wheelset"/>). Default 30000.</summary>
    public float CarMass { get; set; } = 30000f;

    /// <summary>Size of the car body's box: width, height, length, m. Used for collision and for the inertia. Default 2.8 × 3 × 15.</summary>
    public Vector3F CarSize { get; set; } = new(2.8f, 3f, 15f);

    /// <summary>Height of the car's centre of mass above the rail head, m. Default 1.8.</summary>
    public float CarCenterHeight { get; set; } = 1.8f;

    /// <summary>Height of the car body's floor above the rail heads, m: the box of <see cref="CarSize"/> stands on this clearance and its centre of mass is <see cref="CarCenterHeight"/> above the rails. Default 1.0.</summary>
    public float CarFloorHeight { get; set; } = 1f;

    // ---- running gear
    /// <summary>Number of bogies, 0 or 2. With 0 the wheelsets are attached to the car directly. Default 2.</summary>
    public int Bogies { get; set; } = 2;

    /// <summary>Wheelsets per bogie (or on the car when <see cref="Bogies"/> is 0), 1..3. Default 2.</summary>
    public int AxlesPerBogie { get; set; } = 2;

    /// <summary>Distance between the centres of the two bogies, m. Default 10.</summary>
    public float BogieSpacing { get; set; } = 10f;

    /// <summary>Distance between neighbouring wheelsets of one bogie (or of the car), m. Default 2.5.</summary>
    public float AxleSpacing { get; set; } = 2.5f;

    /// <summary>Mass of a bogie frame, kg. Default 3000.</summary>
    public float BogieMass { get; set; } = 3000f;

    /// <summary>Inertia of a bogie frame about its x, y and z axes, kg m². Default (2000, 3000, 3000).</summary>
    public Vector3F BogieInertia { get; set; } = new(2000f, 3000f, 3000f);

    // ---- wheelset
    /// <summary>Radius of a wheel at its nominal tread point, m. Default 0.46.</summary>
    public float WheelRadius { get; set; } = 0.46f;

    /// <summary>Distance between the nominal tread contact points of the two wheels, m. Default 1.5 (standard gauge rails of 70 mm head).</summary>
    public float TreadSpan { get; set; } = 1.5f;

    /// <summary>Conicity of the tread: the radius shrinks by this much per metre towards the outer edge. Default 0.05 (1:20). Within 0..0.3.</summary>
    public float Conicity { get; set; } = 0.05f;

    /// <summary>Lateral play of a centred wheelset before a flange touches a rail, one side, m. Default 0.007.</summary>
    public float FlangeClearance { get; set; } = 0.007f;

    /// <summary>Angle of the flange face from the horizontal, radians. Default 70° (1.2217). Within 0.5..1.5.</summary>
    public float FlangeAngle { get; set; } = 1.2217305f;

    /// <summary>Mass of a wheelset, kg. Default 1500.</summary>
    public float WheelsetMass { get; set; } = 1500f;

    /// <summary>Moment of inertia of a wheelset about its axle, kg m². Default 100.</summary>
    public float WheelsetSpinInertia { get; set; } = 100f;

    /// <summary>Moment of inertia of a wheelset about its other two axes, kg m². Default 700.</summary>
    public float WheelsetTransverseInertia { get; set; } = 700f;

    // ---- contact
    /// <summary>Natural frequency of the wheel-rail normal spring against the load it carries, Hz. Default 20. Within 1..100.</summary>
    public float ContactHertz { get; set; } = 20f;

    /// <summary>Multiplies the creep coefficients of Kalker's linear theory (1 for steel on steel). Default 1. Within 0..100.</summary>
    public float CreepScale { get; set; } = 1f;

    // ---- suspension (frequencies of the suspended mass; angular ones against the effective inertia of the pair)
    /// <summary>Primary suspension (wheelset to bogie, or to the car): natural frequencies (lateral, vertical, longitudinal) of the mass it carries, Hz. Default (8, 6, 10).</summary>
    public Vector3F PrimaryHertz { get; set; } = new(8f, 6f, 10f);

    /// <summary>Primary suspension angular frequencies (about the axle, the vertical axis (yaw) and the longitudinal axis (roll)), Hz. The axle axis is free. Default (0, 6, 8).</summary>
    public Vector3F PrimaryAngularHertz { get; set; } = new(0f, 6f, 8f);

    /// <summary>Damping ratio of the primary suspension. Default 0.3.</summary>
    public float PrimaryDamping { get; set; } = 0.3f;

    /// <summary>Secondary suspension (bogie to car): natural frequencies (lateral, vertical, longitudinal) of the car's mass, Hz. Default (1, 1.2, 4).</summary>
    public Vector3F SecondaryHertz { get; set; } = new(1f, 1.2f, 4f);

    /// <summary>Secondary suspension angular frequencies (pitch, yaw, roll), Hz. Default (1, 2, 1.5).</summary>
    public Vector3F SecondaryAngularHertz { get; set; } = new(1f, 2f, 1.5f);

    /// <summary>Damping ratio of the secondary suspension. Default 0.3.</summary>
    public float SecondaryDamping { get; set; } = 0.3f;

    // ---- traction and braking
    /// <summary>Largest traction force at the rails over all driven wheelsets, N. Default 0 (a trailing car).</summary>
    public float MaxTractionForce { get; set; }

    /// <summary>Largest power, W; the force falls as power over speed above it. Default 0 means no power limit.</summary>
    public float MaxPower { get; set; }

    /// <summary>Bit mask of the driven wheelsets in order (bit 0 the first wheelset of the first bogie ...). Default 0 (none).</summary>
    public uint DrivenWheelsets { get; set; }

    /// <summary>Largest braking force at the rails over all wheelsets, N. Keep it below the adhesion limit (friction times the weight): a brake that locks the wheels takes the lateral guidance away. Default 100000.</summary>
    public float MaxBrakeForce { get; set; } = 100000f;

    // ---- couplers
    /// <summary>Distance from the car's centre to each coupler along its length, m. Default 8.25 (a 15 m car with buffers).</summary>
    public float CouplerReach { get; set; } = 8.25f;

    /// <summary>Height of the couplers above the rail head, m. Default 1.0.</summary>
    public float CouplerHeight { get; set; } = 1f;

    /// <summary>Distance between two coupled couplers at rest, m. Default 0.6.</summary>
    public float CouplerFreeLength { get; set; } = 0.6f;

    /// <summary>Slack of the draw gear: how far the couplers can be pulled apart beyond the free length before it carries tension, m. Default 0.05.</summary>
    public float CouplerSlack { get; set; } = 0.05f;

    /// <summary>Stiffness of the buffers in compression, N/m. Default 4e6.</summary>
    public float BufferStiffness { get; set; } = 4e6f;

    /// <summary>Stiffness of the draw gear in tension, N/m. Default 8e6.</summary>
    public float DrawStiffness { get; set; } = 8e6f;

    /// <summary>Damping of both, N·s/m. Default 2e5.</summary>
    public float CouplerDamping { get; set; } = 2e5f;

    /// <summary>Tension at which the draw gear breaks, N. Default 1.5e6 (a screw coupling's rating); infinity never breaks.</summary>
    public float CouplerBreakForce { get; set; } = 1.5e6f;

    /// <summary>A copy.</summary>
    public RailDef Clone() => (RailDef)MemberwiseClone();

    /// <summary>Number of wheelsets of the vehicle.</summary>
    public int WheelsetCount => (Bogies == 0 ? 1 : Bogies) * AxlesPerBogie;

    /// <summary>Checks every value; throws an <see cref="ArgumentOutOfRangeException"/> naming the field, the value and the bound.</summary>
    public void Validate()
    {
        VehicleCheck.Within(CarMass, 0f, 1e7f, nameof(CarMass));
        VehicleCheck.Vec(CarSize, nameof(CarSize));
        if (CarMass > 0f && !(CarSize.X > 0f && CarSize.Y > 0f && CarSize.Z > 0f)) { throw new ArgumentOutOfRangeException(nameof(CarSize), CarSize, "CarSize must be positive in every direction."); }
        VehicleCheck.Within(CarCenterHeight, -10f, 20f, nameof(CarCenterHeight)); VehicleCheck.Within(CarFloorHeight, 0f, 10f, nameof(CarFloorHeight));
        if (Bogies != 0 && Bogies != 2) { throw new ArgumentOutOfRangeException(nameof(Bogies), Bogies, "Bogies must be 0 or 2."); }
        if (AxlesPerBogie < 1 || AxlesPerBogie > 3) { throw new ArgumentOutOfRangeException(nameof(AxlesPerBogie), AxlesPerBogie, "AxlesPerBogie must be 1..3."); }
        VehicleCheck.Within(BogieSpacing, 0f, 100f, nameof(BogieSpacing)); VehicleCheck.Within(AxleSpacing, 0f, 20f, nameof(AxleSpacing));
        if (Bogies == 2 && !(BogieSpacing > (AxlesPerBogie - 1) * AxleSpacing)) { throw new ArgumentOutOfRangeException(nameof(BogieSpacing), BogieSpacing, "The bogies must be further apart than the wheelsets of one bogie span."); }
        if (AxlesPerBogie > 1 && !(AxleSpacing > 0f)) { throw new ArgumentOutOfRangeException(nameof(AxleSpacing), AxleSpacing, "AxleSpacing must be positive with more than one wheelset."); }
        VehicleCheck.Positive(BogieMass, nameof(BogieMass)); VehicleCheck.Within(BogieMass, 1f, 1e6f, nameof(BogieMass));
        VehicleCheck.Vec(BogieInertia, nameof(BogieInertia));
        if (!(BogieInertia.X > 0f && BogieInertia.Y > 0f && BogieInertia.Z > 0f)) { throw new ArgumentOutOfRangeException(nameof(BogieInertia), BogieInertia, "BogieInertia must be positive."); }
        VehicleCheck.Positive(WheelRadius, nameof(WheelRadius)); VehicleCheck.Within(WheelRadius, 0.05f, 3f, nameof(WheelRadius));
        VehicleCheck.Positive(TreadSpan, nameof(TreadSpan)); VehicleCheck.Within(TreadSpan, 0.2f, 3.5f, nameof(TreadSpan));
        VehicleCheck.Within(Conicity, 0f, 0.3f, nameof(Conicity)); VehicleCheck.Within(FlangeClearance, 0f, 0.1f, nameof(FlangeClearance)); VehicleCheck.Within(FlangeAngle, 0.5f, 1.5f, nameof(FlangeAngle));
        VehicleCheck.Positive(WheelsetMass, nameof(WheelsetMass)); VehicleCheck.Within(WheelsetMass, 1f, 1e6f, nameof(WheelsetMass));
        VehicleCheck.Positive(WheelsetSpinInertia, nameof(WheelsetSpinInertia)); VehicleCheck.Within(WheelsetSpinInertia, 1e-2f, 1e7f, nameof(WheelsetSpinInertia));
        VehicleCheck.Positive(WheelsetTransverseInertia, nameof(WheelsetTransverseInertia)); VehicleCheck.Within(WheelsetTransverseInertia, 1e-2f, 1e7f, nameof(WheelsetTransverseInertia));
        VehicleCheck.Within(ContactHertz, 1f, 100f, nameof(ContactHertz)); VehicleCheck.Within(CreepScale, 0f, 100f, nameof(CreepScale));
        VehicleCheck.Vec(PrimaryHertz, nameof(PrimaryHertz)); VehicleCheck.Vec(PrimaryAngularHertz, nameof(PrimaryAngularHertz)); VehicleCheck.Vec(SecondaryHertz, nameof(SecondaryHertz)); VehicleCheck.Vec(SecondaryAngularHertz, nameof(SecondaryAngularHertz));
        CheckHertz(PrimaryHertz, nameof(PrimaryHertz), true); CheckHertz(PrimaryAngularHertz, nameof(PrimaryAngularHertz), false); CheckHertz(SecondaryHertz, nameof(SecondaryHertz), true); CheckHertz(SecondaryAngularHertz, nameof(SecondaryAngularHertz), false);
        VehicleCheck.Within(PrimaryDamping, 0f, 10f, nameof(PrimaryDamping)); VehicleCheck.Within(SecondaryDamping, 0f, 10f, nameof(SecondaryDamping));
        VehicleCheck.Within(MaxTractionForce, 0f, 1e8f, nameof(MaxTractionForce)); VehicleCheck.Within(MaxPower, 0f, 1e10f, nameof(MaxPower)); VehicleCheck.Within(MaxBrakeForce, 0f, 1e8f, nameof(MaxBrakeForce));
        if (DrivenWheelsets >> WheelsetCount != 0u) { throw new ArgumentOutOfRangeException(nameof(DrivenWheelsets), DrivenWheelsets, "DrivenWheelsets names a wheelset the vehicle does not have."); }
        VehicleCheck.Within(CouplerReach, 0f, 100f, nameof(CouplerReach)); VehicleCheck.Within(CouplerHeight, -10f, 10f, nameof(CouplerHeight));
        VehicleCheck.Within(CouplerFreeLength, 0.01f, 10f, nameof(CouplerFreeLength)); VehicleCheck.Within(CouplerSlack, 0f, 5f, nameof(CouplerSlack));
        VehicleCheck.Within(BufferStiffness, 1f, 1e10f, nameof(BufferStiffness)); VehicleCheck.Within(DrawStiffness, 1f, 1e10f, nameof(DrawStiffness)); VehicleCheck.Within(CouplerDamping, 0f, 1e9f, nameof(CouplerDamping));
        if (!(CouplerBreakForce > 0f)) { throw new ArgumentOutOfRangeException(nameof(CouplerBreakForce), CouplerBreakForce, "CouplerBreakForce must be positive (infinity never breaks)."); }
    }

    private static void CheckHertz(Vector3F v, string name, bool positive)
    {
        foreach (float c in new[] { v.X, v.Y, v.Z })
        {
            if (!(c >= 0f && c <= 100f) || (positive && !(c > 0f))) { throw new ArgumentOutOfRangeException(name, v, name + " must be within " + (positive ? ">0" : "0") + "..100 Hz."); }
        }
    }

    /// <summary>A bare wheelset on the track, no car: for studying the wheel-rail contact itself (hunting, curving). One wheelset, free to run.</summary>
    public static RailDef Wheelset => new()
    {
        CarMass = 0f, Bogies = 0, AxlesPerBogie = 1, CarSize = new Vector3F(1f, 1f, 1f), CouplerReach = 0.5f, MaxBrakeForce = 0f,
    };

    /// <summary>A tram: a 20 t, 12 m car on two two-axle bogies, 0.34 m wheels, softer secondary suspension, driven on all wheelsets (200 kN, 400 kW).</summary>
    public static RailDef Tram => new()
    {
        CarMass = 20000f, CarSize = new Vector3F(2.4f, 3f, 12f), CarCenterHeight = 1.7f, CarFloorHeight = 0.7f, Bogies = 2, AxlesPerBogie = 2, BogieSpacing = 7.5f, AxleSpacing = 1.8f, BogieMass = 2500f,
        BogieInertia = new Vector3F(1500f, 2200f, 2200f), WheelRadius = 0.34f, WheelsetMass = 900f, WheelsetSpinInertia = 45f, WheelsetTransverseInertia = 300f,
        PrimaryHertz = new Vector3F(9f, 7f, 12f), PrimaryAngularHertz = new Vector3F(0f, 7f, 9f), SecondaryHertz = new Vector3F(1.1f, 1.3f, 4f),
        MaxTractionForce = 60000f, MaxPower = 400000f, DrivenWheelsets = 0xF, MaxBrakeForce = 60000f, CouplerReach = 6.25f, CouplerHeight = 0.9f,
    };

    /// <summary>A four-wheel freight wagon: a 40 t car of 9 m directly on two wheelsets 5.5 m apart, firm primary suspension, no traction.</summary>
    public static RailDef Freight => new()
    {
        CarMass = 36000f, CarSize = new Vector3F(2.9f, 2.6f, 9.5f), CarCenterHeight = 1.9f, Bogies = 0, AxlesPerBogie = 2, AxleSpacing = 5.5f, WheelsetMass = 1300f,
        PrimaryHertz = new Vector3F(5f, 4f, 7f), PrimaryAngularHertz = new Vector3F(0f, 5f, 6f), PrimaryDamping = 0.4f, MaxBrakeForce = 60000f, CouplerReach = 5.25f,
    };

    /// <summary>A bogie locomotive: an 80 t, 18 m body on two two-axle bogies, driven on all four wheelsets (300 kN, 3 MW).</summary>
    public static RailDef Locomotive => new()
    {
        CarMass = 68000f, CarSize = new Vector3F(3f, 3.8f, 18f), CarCenterHeight = 1.9f, Bogies = 2, AxlesPerBogie = 2, BogieSpacing = 10.5f, AxleSpacing = 2.8f, BogieMass = 5000f,
        BogieInertia = new Vector3F(3500f, 5000f, 5000f), WheelsetMass = 2000f, WheelsetSpinInertia = 150f, WheelsetTransverseInertia = 1000f, PrimaryHertz = new Vector3F(9f, 7f, 12f),
        PrimaryAngularHertz = new Vector3F(0f, 7f, 9f), MaxTractionForce = 300000f, MaxPower = 3000000f, DrivenWheelsets = 0xF, MaxBrakeForce = 250000f, CouplerReach = 9.25f,
    };
}
