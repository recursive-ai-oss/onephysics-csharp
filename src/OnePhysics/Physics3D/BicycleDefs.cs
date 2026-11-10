using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// One rigid part of a bicycle in the bicycle's own axes: <b>x forward, y right, z down</b>, origin on the ground under the rear wheel's contact point, the bicycle upright with the steering straight
/// (the axes of Meijaard, Papadopoulos, Ruina and Schwab, "Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review", Proc. R. Soc. A 463, 2007).
/// The part's mass includes the wheel that is mounted on it; its inertia is about its own centre of mass.
/// </summary>
public sealed class BicyclePart
{
    /// <summary>Mass of the part with its wheel, kg. Positive.</summary>
    public float Mass { get; set; } = 1f;

    /// <summary>Centre of mass, forward of the rear contact point, m.</summary>
    public float CenterX { get; set; }

    /// <summary>Centre of mass below the ground plane, m (negative above the ground).</summary>
    public float CenterZ { get; set; }

    /// <summary>Inertia tensor entry Ixx about the centre of mass, kg m².</summary>
    public float Ixx { get; set; } = 1f;

    /// <summary>Inertia tensor entry Ixz (the tensor entry, not the product of inertia; the sign convention of the paper), kg m².</summary>
    public float Ixz { get; set; }

    /// <summary>Inertia tensor entry Iyy, kg m².</summary>
    public float Iyy { get; set; } = 1f;

    /// <summary>Inertia tensor entry Izz, kg m².</summary>
    public float Izz { get; set; } = 1f;

    /// <summary>A copy.</summary>
    public BicyclePart Clone() => (BicyclePart)MemberwiseClone();

    internal void Validate(string where)
    {
        VehicleCheck.Positive(Mass, where + ".Mass"); VehicleCheck.Within(Mass, 1e-3f, 1e5f, where + ".Mass");
        VehicleCheck.Within(CenterX, -100f, 100f, where + ".CenterX"); VehicleCheck.Within(CenterZ, -100f, 100f, where + ".CenterZ");
        VehicleCheck.Positive(Ixx, where + ".Ixx"); VehicleCheck.Positive(Iyy, where + ".Iyy"); VehicleCheck.Positive(Izz, where + ".Izz");
        VehicleCheck.Within(Ixx, 1e-6f, 1e6f, where + ".Ixx"); VehicleCheck.Within(Iyy, 1e-6f, 1e6f, where + ".Iyy"); VehicleCheck.Within(Izz, 1e-6f, 1e6f, where + ".Izz");
        VehicleCheck.Within(Ixz, -1e6f, 1e6f, where + ".Ixz");
        // A physical tensor: the triangle inequalities on the principal moments hold when the diagonal ones do, and the 2x2 block must be positive definite.
        if (!(Ixx * Izz > Ixz * Ixz)) { throw new ArgumentOutOfRangeException(where + ".Ixz", Ixz, where + " has an inertia tensor that is not positive definite (Ixz² must be below Ixx·Izz)."); }
    }
}

/// <summary>A bicycle wheel: a thin disc that spins about its axle.</summary>
public sealed class BicycleWheel
{
    /// <summary>Radius, m. Positive.</summary>
    public float Radius { get; set; } = 0.3f;

    /// <summary>Moment of inertia about the axle, kg m². Positive. This is the wheel's spin; its inertia about the other axes belongs to the <see cref="BicyclePart"/> it is mounted on.</summary>
    public float SpinInertia { get; set; } = 0.1f;

    /// <summary>A copy.</summary>
    public BicycleWheel Clone() => (BicycleWheel)MemberwiseClone();

    internal void Validate(string where)
    {
        VehicleCheck.Positive(Radius, where + ".Radius"); VehicleCheck.Within(Radius, 0.01f, 10f, where + ".Radius");
        VehicleCheck.Positive(SpinInertia, where + ".SpinInertia"); VehicleCheck.Within(SpinInertia, 1e-6f, 1e4f, where + ".SpinInertia");
    }
}

/// <summary>
/// A bicycle in the Whipple model: a rear frame (with the rider and the rear wheel), a front assembly (fork, handlebar and front wheel) that turns about the steering axis, and two wheels on tyres.
/// The bicycle is built from two rigid bodies joined by a hinge on the steering axis; each wheel is a one-wheel vehicle on its body, so the tyre forces are the ones every vehicle has and the self-stability,
/// weave and capsize come from the physics rather than from a controller. The wheels' spin angular momentum is carried by the gyroscopic coupling of each wheel with its body (the wheel's own inertia about the
/// other axes is part of the body's).
/// <para>
/// Geometry follows the benchmark of Meijaard et al. 2007: <see cref="WheelBase"/> and <see cref="Trail"/> between the contact points and the steering-axis foot, the steering axis tilted back by
/// <see cref="HeadAngle"/> from the vertical. Heading is +Z, the bicycle's right is +X, up is +Y.
/// </para>
/// </summary>
public sealed class BicycleDef
{
    /// <summary>Distance between the two contact points, m. Default 1.02 (the benchmark).</summary>
    public float WheelBase { get; set; } = 1.02f;

    /// <summary>Trail: how far the front contact point lies behind the foot of the steering axis, m. Default 0.08.</summary>
    public float Trail { get; set; } = 0.08f;

    /// <summary>Angle of the steering axis from the vertical, radians, leaning back. Default π/10 (18°). Within 0..π/4.</summary>
    public float HeadAngle { get; set; } = 0.31415927f;

    /// <summary>The rear wheel.</summary>
    public BicycleWheel RearWheel { get; set; } = new() { Radius = 0.3f, SpinInertia = 0.12f };

    /// <summary>The front wheel.</summary>
    public BicycleWheel FrontWheel { get; set; } = new() { Radius = 0.35f, SpinInertia = 0.28f };

    /// <summary>The rear frame including the rider and the rear wheel's mass.</summary>
    public BicyclePart RearFrame { get; set; } = new();

    /// <summary>The front assembly including the front wheel's mass.</summary>
    public BicyclePart Front { get; set; } = new();

    /// <summary>Tyre friction coefficient. Default 1. Within 0..100.</summary>
    public float TyreFriction { get; set; } = 1f;

    /// <summary>Cornering stiffness of the tyres: the Pacejka lateral B factor (force per unit slip angle, normalised). Higher values approach the benchmark's no-slip wheels. Default 80. Within 1..1000.</summary>
    public float TyreStiffness { get; set; } = 80f;

    /// <summary>Natural frequency of a tyre's vertical spring, Hz. Default 15. Within 1..25.</summary>
    public float TyreHertz { get; set; } = 15f;

    /// <summary>Steering torque at full input, N·m. Default 20. At least 0.</summary>
    public float MaxSteerTorque { get; set; } = 20f;

    /// <summary>Torque of the pedals on the rear wheel at full input, N·m. Default 15. At least 0.</summary>
    public float MaxPedalTorque { get; set; } = 15f;

    /// <summary>Front brake torque at full input, N·m. Default 150. At least 0.</summary>
    public float MaxFrontBrakeTorque { get; set; } = 150f;

    /// <summary>Rear brake torque at full input, N·m. Default 100. At least 0.</summary>
    public float MaxRearBrakeTorque { get; set; } = 100f;

    /// <summary>Torque the rider's lean exerts on the rear frame about its forward axis at full input, N·m (the rider shifting weight against the ground contact). Default 60. At least 0.</summary>
    public float MaxLeanTorque { get; set; } = 60f;

    /// <summary>Roll angle beyond which the bicycle counts as fallen, radians. Default 1.0. Within 0.1..1.5.</summary>
    public float FallAngle { get; set; } = 1f;

    /// <summary>A copy.</summary>
    public BicycleDef Clone()
    {
        var c = (BicycleDef)MemberwiseClone();
        c.RearWheel = RearWheel.Clone(); c.FrontWheel = FrontWheel.Clone(); c.RearFrame = RearFrame.Clone(); c.Front = Front.Clone();
        return c;
    }

    /// <summary>Checks every value; throws an <see cref="ArgumentOutOfRangeException"/> naming the field, the value and the bound.</summary>
    public void Validate()
    {
        if (RearWheel == null || FrontWheel == null || RearFrame == null || Front == null) { throw new ArgumentNullException(nameof(RearWheel), "A bicycle needs both wheels and both parts."); }
        VehicleCheck.Positive(WheelBase, nameof(WheelBase)); VehicleCheck.Within(WheelBase, 0.1f, 20f, nameof(WheelBase));
        VehicleCheck.Within(Trail, -5f, 5f, nameof(Trail));
        VehicleCheck.Within(HeadAngle, 0f, 0.7853982f, nameof(HeadAngle));
        RearWheel.Validate(nameof(RearWheel)); FrontWheel.Validate(nameof(FrontWheel)); RearFrame.Validate(nameof(RearFrame)); Front.Validate(nameof(Front));
        VehicleCheck.Within(TyreFriction, 0f, 100f, nameof(TyreFriction)); VehicleCheck.Within(TyreStiffness, 1f, 1000f, nameof(TyreStiffness)); VehicleCheck.Within(TyreHertz, 1f, 25f, nameof(TyreHertz));
        VehicleCheck.Within(MaxSteerTorque, 0f, 1e5f, nameof(MaxSteerTorque)); VehicleCheck.Within(MaxPedalTorque, 0f, 1e5f, nameof(MaxPedalTorque));
        VehicleCheck.Within(MaxFrontBrakeTorque, 0f, 1e5f, nameof(MaxFrontBrakeTorque)); VehicleCheck.Within(MaxRearBrakeTorque, 0f, 1e5f, nameof(MaxRearBrakeTorque));
        VehicleCheck.Within(MaxLeanTorque, 0f, 1e5f, nameof(MaxLeanTorque)); VehicleCheck.Within(FallAngle, 0.1f, 1.5f, nameof(FallAngle));
    }

    /// <summary>
    /// The benchmark bicycle of Meijaard et al. 2007 (their Table 1): wheel base 1.02 m, trail 0.08 m, head angle 18°, a 85 kg rear frame with rider, a 4 kg front assembly, 0.3 m and 0.35 m wheels.
    /// Uncontrolled it is self-stable between about 4.3 and 6.0 m/s.
    /// </summary>
    public static BicycleDef Benchmark
    {
        get
        {
            // The paper lists the wheels apart from the frames; each wheel's mass and its inertia about the transverse axes are folded into the part it is mounted on here
            // (a wheel is axisymmetric, so this is exact), leaving only the spin inertia separate.
            var rearWheel = Fold(2f, 0.3f, 0f, -0.3f, 0.0603f, 0.12f);
            var frontWheel = Fold(3f, 0.35f, 1.02f, -0.35f, 0.1405f, 0.28f);
            var rear = Combine(new Part(85f, 0.3f, -0.9f, 9.2f, 2.4f, 11f, 2.8f), rearWheel);
            var front = Combine(new Part(4f, 0.9f, -0.7f, 0.05892f, -0.00756f, 0.06f, 0.00708f), frontWheel);
            return new BicycleDef
            {
                WheelBase = 1.02f, Trail = 0.08f, HeadAngle = 0.31415927f,
                RearWheel = new BicycleWheel { Radius = 0.3f, SpinInertia = 0.12f }, FrontWheel = new BicycleWheel { Radius = 0.35f, SpinInertia = 0.28f },
                RearFrame = rear, Front = front,
            };
        }
    }

    /// <summary>A road bicycle with a rider: 700c wheels (0.34 m), 1.0 m wheel base, 72° head angle, 10 kg bicycle and 70 kg rider, tuned so it is self-stable at city speeds.</summary>
    public static BicycleDef Road
    {
        get
        {
            // Wheels: 1.3 kg each at 0.34 m, rim-weighted: I_spin = 0.9 m r², transverse half of that.
            var rw = Fold(1.3f, 0.34f, 0f, -0.34f, 0.9f * 1.3f * 0.34f * 0.34f * 0.5f, 0.9f * 1.3f * 0.34f * 0.34f);
            var fw = Fold(1.3f, 0.34f, 1.0f, -0.34f, 0.9f * 1.3f * 0.34f * 0.34f * 0.5f, 0.9f * 1.3f * 0.34f * 0.34f);
            var rear = Combine(new Part(75f, 0.3f, -0.95f, 8.5f, 2.0f, 10f, 2.6f), rw);
            var front = Combine(new Part(3.5f, 0.88f, -0.75f, 0.06f, -0.008f, 0.06f, 0.008f), fw);
            float spin = 0.9f * 1.3f * 0.34f * 0.34f;
            return new BicycleDef
            {
                WheelBase = 1.0f, Trail = 0.065f, HeadAngle = 0.3490659f,
                RearWheel = new BicycleWheel { Radius = 0.34f, SpinInertia = spin }, FrontWheel = new BicycleWheel { Radius = 0.34f, SpinInertia = spin },
                RearFrame = rear, Front = front,
            };
        }
    }

    private readonly record struct Part(float Mass, float X, float Z, float Ixx, float Ixz, float Iyy, float Izz);

    // A wheel (axle along y) with its centre at (x, z): a part with the wheel's transverse inertia (xx = zz) and its spin inertia as yy.
    private static Part Fold(float mass, float radius, float x, float z, float transverse, float spin) => new(mass, x, z, transverse, 0f, spin, transverse);

    // Combines two parts into one about the joint centre of mass: m = m1 + m2, parallel-axis shifts in the x-z plane.
    private static BicyclePart Combine(Part a, Part b)
    {
        float m = a.Mass + b.Mass;
        float cx = (a.Mass * a.X + b.Mass * b.X) / m, cz = (a.Mass * a.Z + b.Mass * b.Z) / m;
        float ixx = 0f, ixz = 0f, iyy = 0f, izz = 0f;
        foreach (var p in new[] { a, b })
        {
            float dx = p.X - cx, dz = p.Z - cz;
            ixx += p.Ixx + p.Mass * dz * dz;
            izz += p.Izz + p.Mass * dx * dx;
            iyy += p.Iyy + p.Mass * (dx * dx + dz * dz);
            ixz += p.Ixz - p.Mass * dx * dz;
        }

        return new BicyclePart { Mass = m, CenterX = cx, CenterZ = cz, Ixx = ixx, Ixz = ixz, Iyy = iyy, Izz = izz };
    }
}
