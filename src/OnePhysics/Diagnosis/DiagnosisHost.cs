using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Diagnosis;

/// <summary>What the diagnosis needs to know about one joint that the shared core does not store: its kind, its anchors and which optional parts it has. Filled by the dimension's host.</summary>
internal struct JointFacts
{
    /// <summary>Readable kind: "hinge", "revolute", "fixed".</summary>
    public string Type;

    /// <summary>False for joints that apply no impulse (a collision filter), which no check looks at.</summary>
    public bool Solves;

    /// <summary>True when the kind keeps its two anchors at one point, so their separation is the position error.</summary>
    public bool Coincident;

    /// <summary>World-space distance between the two anchors, metres; meaningful for any kind with two anchors.</summary>
    public double Gap;

    /// <summary>Distance of each anchor from its body's centre of mass, metres.</summary>
    public float ArmA, ArmB;

    /// <summary>True when a limit or a motor is switched on.</summary>
    public bool LimitOn, MotorOn;

    /// <summary>How many of the bodies' relative degrees of freedom the joint locks hard (a hinge in 3D locks 5, a revolute joint in 2D locks 2); limits, motors and springs are not counted.</summary>
    public int LockedRows;
}

/// <summary>
/// The per-dimension half of a diagnosis: reads the engine's own arrays (mass, extents, joint records) and can fork and step a copy. Everything else reads the shared <see cref="WorldCore"/>.
/// A host only reads its world, except <see cref="Fork"/>'s result, which the diagnosis owns and steps.
/// </summary>
internal abstract class DiagnosisHost
{
    /// <summary>2 or 3.</summary>
    public abstract int Dimension { get; }

    /// <summary>World units per metre of the diagnosed world's length unit. Every length, speed and acceleration this host reports is already converted to metres, so the thresholds of the diagnosis are metre values whatever the unit.</summary>
    public abstract float UnitsPerMetre { get; }

    /// <summary>The shared core of the diagnosed world.</summary>
    public WorldCore Core { get; protected init; } = null!;

    /// <summary>Mass of a dynamic body, kg (0 when it has none).</summary>
    public abstract float Mass(int body);

    /// <summary>Distance from the centre of mass to the farthest shape point, metres.</summary>
    public abstract float MaxExtent(int body);

    /// <summary>Smallest half-width of any of the body's shapes, metres; float.MaxValue for a body with no shape.</summary>
    public abstract float MinExtent(int body);

    /// <summary>The engine's linear speed limit, m/s.</summary>
    public abstract float MaxLinearSpeed { get; }

    /// <summary>The distance within which contacts are made before shapes touch, metres.</summary>
    public abstract float SpeculativeDistance { get; }

    /// <summary>The size of the gravity vector, m/s^2, as the world's options say now (the core copies options only when it steps).</summary>
    public abstract double GravityMagnitude { get; }

    /// <summary>Whether bodies may fall asleep.</summary>
    public abstract bool SleepEnabled { get; }

    /// <summary>Seconds of rest before an island sleeps.</summary>
    public abstract float TimeToSleep { get; }

    /// <summary>The motion below which a body counts as at rest, m/s.</summary>
    public abstract float SleepThreshold { get; }

    /// <summary>Whether continuous collision is on.</summary>
    public abstract bool ContinuousEnabled { get; }

    /// <summary>Joint facts for a live joint slot.</summary>
    public abstract JointFacts Joint(int slot);

    /// <summary>Collects the joint rows of each step; takes effect at the next step. Does not change the simulation.</summary>
    public abstract bool DetailedInspection { get; set; }

    /// <summary>Steps this world with callbacks suppressed.</summary>
    public abstract void Step(float dt);

    /// <summary>An independent copy of the whole simulation state (the host of the copy is a new object).</summary>
    public abstract DiagnosisHost Fork();

    /// <summary>The world's step count.</summary>
    public long StepCount => Core.StepCount;

    /// <summary>Linear speed of a body, m/s.</summary>
    public double Speed(int body)
    {
        var v = Core.Bodies.LinearVelocityOf(body);
        double s = 0;
        for (int i = 0; i < v.Length; i++) { s += (double)v[i] * v[i]; }
        return System.Math.Sqrt(s) / UnitsPerMetre;
    }

    /// <summary>Angular speed of a body, rad/s.</summary>
    public double Spin(int body)
    {
        var w = Core.Bodies.AngularVelocityOf(body);
        double s = 0;
        for (int i = 0; i < w.Length; i++) { s += (double)w[i] * w[i]; }
        return System.Math.Sqrt(s);
    }

    /// <summary>The motion the sleep test and the continuous-collision test use: linear speed plus the speed of the farthest shape point from spinning, m/s.</summary>
    public double Motion(int body) => Speed(body) + Spin(body) * MaxExtent(body);
}
