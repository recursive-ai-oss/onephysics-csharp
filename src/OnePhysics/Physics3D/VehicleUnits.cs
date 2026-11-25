using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class TyreDef
{
    internal TyreDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        float v = LowSpeed; s.D(ref v, 1f, 1); c.LowSpeed = v;
        return c;
    }
}

public sealed partial class WheelDef
{
    internal WheelDef Scaled(UnitScaler s)
    {
        var c = Clone();
        c.Tyre = Tyre.Scaled(s);
        if (s.IsIdentity) { return c; }
        var p = Position; s.E(ref p, 1); c.Position = p;
        float r = Radius; s.D(ref r, 0.35f, 1); c.Radius = r;
        float w = Width; s.D(ref w, 0.25f, 1); c.Width = w;
        float lo = SuspensionMinLength; s.D(ref lo, 0.1f, 1); c.SuspensionMinLength = lo;
        float hi = SuspensionMaxLength; s.D(ref hi, 0.5f, 1); c.SuspensionMaxLength = hi;
        float pre = PreloadLength; s.E(ref pre, 1); c.PreloadLength = pre;
        float i = Inertia; s.D(ref i, 1f, 2); c.Inertia = i;
        float b = MaxBrakeTorque; s.D(ref b, 3000f, 2); c.MaxBrakeTorque = b;
        float h = MaxHandbrakeTorque; s.E(ref h, 2); c.MaxHandbrakeTorque = h;
        return c;
    }

    internal static WheelDef[] Scaled(WheelDef[] wheels, UnitScaler s)
    {
        var a = new WheelDef[wheels.Length];
        for (int i = 0; i < a.Length; i++) { a[i] = wheels[i]?.Scaled(s)!; }
        return a;
    }
}

public sealed partial class EngineDef
{
    internal EngineDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        float t = MaxTorque; s.D(ref t, 400f, 2); c.MaxTorque = t;
        float i = Inertia; s.D(ref i, 0.5f, 2); c.Inertia = i;
        return c;
    }
}

public sealed partial class VehicleDef
{
    // World units per metre the members are expressed in; 1 for a definition written in metres. Validation checks the metre equivalent, so the ranges are those of metres whatever the unit.
    internal float Units = 1f;

    /// <summary>
    /// This definition with the members that were never set at their defaults for a world of the given unit (wheel radius and width, suspension lengths, wheel and engine inertia, brake and engine torque, tyre low speed): a wheel
    /// that was never given a radius is 0.35 m, which in a millimetre world is 350 units. The world applies this itself when the vehicle is added; use <see cref="InUnit"/> on the presets.
    /// </summary>
    /// <param name="unit">The world's length unit.</param>
    public VehicleDef WithDefaultsFor(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, false));

    /// <summary>
    /// This definition, written in metres, newtons and kilograms (the presets <see cref="Car"/>, <see cref="Truck"/> and <see cref="Kart"/>, or a vehicle you sized in metres), expressed in <paramref name="unit"/>: every length (wheel positions,
    /// radii, widths, suspension lengths, preload), torque and moment of inertia (wheel, engine, brakes, air control; kilogram world units squared per second squared and kilogram world units squared) is converted.
    /// Masses, ratios, frequencies, damping ratios and angles do not depend on the length unit.
    /// </summary>
    /// <param name="unit">The target length unit.</param>
    public VehicleDef InUnit(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, true));

    internal VehicleDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        c.Units = s.K;
        c.Wheels = WheelDef.Scaled(c.Wheels, s);
        c.Engine = Engine.Scaled(s);
        return c;
    }
}

public sealed partial class TrackedVehicleDef
{
    // World units per metre the members are expressed in; 1 for a definition written in metres. Validation checks the metre equivalent, so the ranges are those of metres whatever the unit.
    internal float Units = 1f;

    /// <summary>This definition with the members that were never set (sprocket radius, track speed, torques, hull size, wheels) at their defaults for a world of the given unit. The world applies this itself when the vehicle is added.</summary>
    /// <param name="unit">The world's length unit.</param>
    public TrackedVehicleDef WithDefaultsFor(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, false));

    /// <summary>This definition, written in metres (for example <see cref="Tank"/>), expressed in <paramref name="unit"/>: the wheels, sprocket radius, belt speed, drive and brake torque, hull size and air control. The hull mass is in kilograms and stays.</summary>
    /// <param name="unit">The target length unit.</param>
    public TrackedVehicleDef InUnit(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, true));

    internal TrackedVehicleDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        c.Units = s.K;
        c.LeftWheels = WheelDef.Scaled(c.LeftWheels, s); c.RightWheels = WheelDef.Scaled(c.RightWheels, s);
        c.Grip = Grip.Scaled(s);
        float r = SprocketRadius; s.D(ref r, 0.3f, 1); c.SprocketRadius = r;
        float v = MaxTrackSpeed; s.D(ref v, 12f, 1); c.MaxTrackSpeed = v;
        float d = MaxDriveTorque; s.D(ref d, 20000f, 2); c.MaxDriveTorque = d;
        float b = MaxBrakeTorque; s.D(ref b, 120000f, 2); c.MaxBrakeTorque = b;
        var size = ChassisSize; s.D(ref size, new Vector3F(3.2f, 1f, 6f), 1); c.ChassisSize = size;
        return c;
    }
}

public sealed partial class MotorcycleDef
{
    // World units per metre the members are expressed in; 1 for a definition written in metres. Validation checks the metre equivalent, so the ranges are those of metres whatever the unit.
    internal float Units = 1f;

    /// <summary>This definition with the members that were never set at their defaults for a world of the given unit. The world applies this itself when the motorcycle is added.</summary>
    /// <param name="unit">The world's length unit.</param>
    public MotorcycleDef WithDefaultsFor(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, false));

    /// <summary>This definition, written in metres (for example <see cref="Road"/>), expressed in <paramref name="unit"/>: wheels, engine, speeds, lean torque and chassis size. The chassis mass is in kilograms and stays.</summary>
    /// <param name="unit">The target length unit.</param>
    public MotorcycleDef InUnit(LengthUnit unit) => Scaled(new UnitScaler(unit.Scale, true));

    internal MotorcycleDef Scaled(UnitScaler s)
    {
        var c = Clone();
        if (s.IsIdentity) { return c; }
        c.Units = s.K;
        c.Front = Front.Scaled(s); c.Rear = Rear.Scaled(s); c.Engine = Engine.Scaled(s);
        float lean = MaxLeanTorque; s.D(ref lean, 1500f, 2); c.MaxLeanTorque = lean;
        float a = SteerFadeStartSpeed; s.D(ref a, 5f, 1); c.SteerFadeStartSpeed = a;
        float e = SteerFadeEndSpeed; s.D(ref e, 30f, 1); c.SteerFadeEndSpeed = e;
        var size = ChassisSize; s.D(ref size, new Vector3F(0.3f, 0.4f, 1.6f), 1); c.ChassisSize = size;
        return c;
    }
}
