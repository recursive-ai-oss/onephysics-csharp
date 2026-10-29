using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// The parameters of one joint, as the engine keeps and snapshots them (indexed by constraint slot). One record serves every joint type; the meaning of the shared fields is listed per type in docs/spec/world2d.md.
/// The last five fields are runtime bookkeeping, not part of a snapshot: the bodies' slots and generations and the joint's own generation, used to notice that the core removed the joint.
/// </summary>
internal struct JointData2D
{
    public const byte FlagLimit = 1, FlagMotor = 2, FlagSpring = 4, FlagCollide = 8;
    public const float MaxLocal = 2097152f;

    public byte Type;
    public byte Flags;
    public float LAx, LAy, LBx, LBy;     // anchors in the bodies' origin frames
    public float AxX, AxY;               // unit axis in A's frame (prismatic, wheel)
    public float RefAngle, RefC, RefS;   // reference angle and its rotation
    public float Lower, Upper;           // limits: radians, metres, or lengths
    public float MotorSpeed, MaxMotor, MaxMotor2;
    public float Hertz, Damping, Hertz2, Damping2;
    public float Length;                 // distance joint rest length
    public float OffX, OffY, OffAngle;   // motor joint offsets
    public float Correction;
    public double TargetX, TargetY;      // mouse target, world coordinates relative to the world origin
    public float BreakForce, BreakTorque;

    // Runtime only.
    public int BodyA, BodyB, GenA, GenB, Gen;

    public readonly bool Collide => (Flags & FlagCollide) != 0;
    public readonly bool Limit => (Flags & FlagLimit) != 0;
    public readonly bool Motor => (Flags & FlagMotor) != 0;
    public readonly bool Spring => (Flags & FlagSpring) != 0;

    public static string Name(JointType2D t) => t.ToString().ToLowerInvariant();

    private static string Num(float v) => v.ToString("R", CultureInfo.InvariantCulture);

    private static void Finite(float v, string field, JointType2D t)
    {
        if (!float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Name(t) + " joint: " + field + " = " + Num(v) + " is not finite."); }
    }

    private static void AtLeastZero(float v, string field, JointType2D t)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Name(t) + " joint: " + field + " = " + Num(v) + " must be finite and at least 0."); }
    }

    private static void Local(float x, float y, string field, JointType2D t)
    {
        Finite(x, field, t); Finite(y, field, t);
        if (Math.Abs(x) > MaxLocal || Math.Abs(y) > MaxLocal) { throw new ArgumentOutOfRangeException(field, "(" + Num(x) + ", " + Num(y) + ")", Name(t) + " joint: " + field + " is outside the supported range of +-2^21."); }
    }

    /// <summary>Checks every value the type uses. Throws <see cref="ArgumentOutOfRangeException"/> for non-finite or out-of-range numbers and <see cref="InvalidJointException"/> for limits the wrong way round, a zero axis and similar.</summary>
    public readonly void Validate()
    {
        var t = (JointType2D)Type;
        if (t < JointType2D.Revolute || t > JointType2D.Mouse) { throw new InvalidJointException("Unknown joint type " + Type.ToString(CultureInfo.InvariantCulture) + "."); }
        if (float.IsNaN(BreakForce) || BreakForce < 0f) { throw new ArgumentOutOfRangeException(nameof(BreakForce), BreakForce, Name(t) + " joint: BreakForce must be at least 0 (infinity means never)."); }
        if (float.IsNaN(BreakTorque) || BreakTorque < 0f) { throw new ArgumentOutOfRangeException(nameof(BreakTorque), BreakTorque, Name(t) + " joint: BreakTorque must be at least 0 (infinity means never)."); }
        if (t != JointType2D.Motor) { Local(LBx, LBy, "LocalAnchorB", t); }
        if (t != JointType2D.Motor && t != JointType2D.Mouse) { Local(LAx, LAy, "LocalAnchorA", t); }
        switch (t)
        {
            case JointType2D.Revolute:
                Finite(RefAngle, "ReferenceAngle", t);
                if (Limit)
                {
                    Finite(Lower, "LowerAngle", t); Finite(Upper, "UpperAngle", t);
                    if (Lower > Upper) { throw new InvalidJointException("revolute joint: LowerAngle " + Num(Lower) + " is above UpperAngle " + Num(Upper) + "."); }
                    if (Lower < -3.1415927f || Upper > 3.1415927f) { throw new ArgumentOutOfRangeException("LowerAngle", "[" + Num(Lower) + ", " + Num(Upper) + "]", "revolute joint: angle limits must lie within -pi..pi."); }
                }

                MotorSpring(t);
                break;
            case JointType2D.Prismatic:
            case JointType2D.Wheel:
                Finite(AxX, "LocalAxisA", t); Finite(AxY, "LocalAxisA", t);
                if (t == JointType2D.Prismatic) { Finite(RefAngle, "ReferenceAngle", t); }
                if (Limit)
                {
                    Finite(Lower, "LowerTranslation", t); Finite(Upper, "UpperTranslation", t);
                    if (Lower > Upper) { throw new InvalidJointException(Name(t) + " joint: LowerTranslation " + Num(Lower) + " is above UpperTranslation " + Num(Upper) + "."); }
                }

                MotorSpring(t);
                break;
            case JointType2D.Weld:
                Finite(RefAngle, "ReferenceAngle", t);
                AtLeastZero(Hertz, "LinearHertz", t); AtLeastZero(Damping, "LinearDampingRatio", t);
                AtLeastZero(Hertz2, "AngularHertz", t); AtLeastZero(Damping2, "AngularDampingRatio", t);
                break;
            case JointType2D.Distance:
                if (!(Length >= 0.005f) || !float.IsFinite(Length)) { throw new ArgumentOutOfRangeException("Length", Length, "distance joint: Length = " + Num(Length) + " must be finite and at least 0.005."); }
                AtLeastZero(Lower, "MinLength", t);
                if (float.IsNaN(Upper) || Upper < Lower) { throw new InvalidJointException("distance joint: MaxLength " + Num(Upper) + " is below MinLength " + Num(Lower) + "."); }
                MotorSpring(t);
                break;
            case JointType2D.Motor:
                Local(OffX, OffY, "LinearOffset", t); Finite(OffAngle, "AngularOffset", t);
                AtLeastZero(MaxMotor, "MaxForce", t); AtLeastZero(MaxMotor2, "MaxTorque", t);
                if (!(Correction >= 0f && Correction <= 1f)) { throw new ArgumentOutOfRangeException("CorrectionFactor", Correction, "motor joint: CorrectionFactor = " + Num(Correction) + " must be within 0..1."); }
                break;
            case JointType2D.Mouse:
                if (!double.IsFinite(TargetX) || !double.IsFinite(TargetY) || Math.Abs(TargetX) > 2097152.0 || Math.Abs(TargetY) > 2097152.0)
                {
                    throw new ArgumentOutOfRangeException("Target", "(" + TargetX.ToString("R", CultureInfo.InvariantCulture) + ", " + TargetY.ToString("R", CultureInfo.InvariantCulture) + ")", "mouse joint: Target must be finite and within +-2^21.");
                }

                AtLeastZero(Hertz, "Hertz", t); AtLeastZero(Damping, "DampingRatio", t); AtLeastZero(MaxMotor, "MaxForce", t);
                break;
        }
    }

    private readonly void MotorSpring(JointType2D t)
    {
        Finite(MotorSpeed, "MotorSpeed", t);
        AtLeastZero(MaxMotor, t == JointType2D.Prismatic || t == JointType2D.Distance ? "MaxMotorForce" : "MaxMotorTorque", t);
        AtLeastZero(Hertz, "SpringHertz", t);
        AtLeastZero(Damping, "SpringDampingRatio", t);
    }

    /// <summary>Normalizes the axis of a prismatic or wheel joint and computes the reference rotation; call after <see cref="Validate"/>-independent checks and before storing.</summary>
    public void Prepare()
    {
        var t = (JointType2D)Type;
        if (t == JointType2D.Prismatic || t == JointType2D.Wheel)
        {
            float len = float.Sqrt(AxX * AxX + AxY * AxY);
            if (!(len > 1e-12f) || !float.IsFinite(len)) { throw new InvalidJointException(Name(t) + " joint: LocalAxisA (" + Num(AxX) + ", " + Num(AxY) + ") has no usable direction."); }
            AxX /= len; AxY /= len;
        }

        SetReference(t == JointType2D.Motor ? OffAngle : RefAngle);
    }

    /// <summary>Sets the reference rotation (cosine and sine, correctly rounded, from onemath): the reference angle, or a motor joint's angular offset.</summary>
    public void SetReference(float angle)
    {
        var (s, c) = Math64.SinCos(angle);
        RefC = (float)c; RefS = (float)s;
    }

    // ---- snapshot ----

    public readonly void Write(StateWriter w)
    {
        w.WriteByte(Type); w.WriteByte(Flags);
        w.WriteSingle(LAx); w.WriteSingle(LAy); w.WriteSingle(LBx); w.WriteSingle(LBy);
        w.WriteSingle(AxX); w.WriteSingle(AxY); w.WriteSingle(RefAngle);
        w.WriteSingle(Lower); w.WriteSingle(Upper);
        w.WriteSingle(MotorSpeed); w.WriteSingle(MaxMotor); w.WriteSingle(MaxMotor2);
        w.WriteSingle(Hertz); w.WriteSingle(Damping); w.WriteSingle(Hertz2); w.WriteSingle(Damping2);
        w.WriteSingle(Length); w.WriteSingle(OffX); w.WriteSingle(OffY); w.WriteSingle(OffAngle); w.WriteSingle(Correction);
        w.WriteDouble(TargetX); w.WriteDouble(TargetY);
        w.WriteSingle(BreakForce); w.WriteSingle(BreakTorque);
    }

    /// <summary>Bytes one joint record takes in a snapshot.</summary>
    public const int RecordBytes = 2 + 4 * 23 + 16;

    public static JointData2D Read(StateReader r)
    {
        var d = default(JointData2D);
        d.Type = r.ReadByte(); d.Flags = r.ReadByte();
        if ((d.Flags & ~(FlagLimit | FlagMotor | FlagSpring | FlagCollide)) != 0) { throw r.Error("invalid joint flags"); }
        d.LAx = r.ReadSingle(); d.LAy = r.ReadSingle(); d.LBx = r.ReadSingle(); d.LBy = r.ReadSingle();
        d.AxX = r.ReadSingle(); d.AxY = r.ReadSingle(); d.RefAngle = r.ReadSingle();
        d.Lower = r.ReadSingle(); d.Upper = r.ReadSingle();
        d.MotorSpeed = r.ReadSingle(); d.MaxMotor = r.ReadSingle(); d.MaxMotor2 = r.ReadSingle();
        d.Hertz = r.ReadSingle(); d.Damping = r.ReadSingle(); d.Hertz2 = r.ReadSingle(); d.Damping2 = r.ReadSingle();
        d.Length = r.ReadSingle(); d.OffX = r.ReadSingle(); d.OffY = r.ReadSingle(); d.OffAngle = r.ReadSingle(); d.Correction = r.ReadSingle();
        d.TargetX = r.ReadDouble(); d.TargetY = r.ReadDouble();
        d.BreakForce = r.ReadSingle(); d.BreakTorque = r.ReadSingle();
        try
        {
            d.Validate();
            var t = (JointType2D)d.Type;
            if (t == JointType2D.Prismatic || t == JointType2D.Wheel)
            {
                float n2 = d.AxX * d.AxX + d.AxY * d.AxY;
                if (!(Math.Abs(n2 - 1f) <= 1e-3f)) { throw new InvalidJointException("axis is not a unit vector"); }
            }

            d.SetReference(t == JointType2D.Motor ? d.OffAngle : d.RefAngle);
        }
        catch (Exception e) when (e is ArgumentException or PhysicsException)
        {
            throw r.Error("invalid joint record: " + e.Message);
        }

        return d;
    }
}
