using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

[InlineArray(6)]
internal struct JF6
{
    private float _e;
}

[InlineArray(6)]
internal struct JB6
{
    private byte _e;
}

/// <summary>Per-axis settings of a six-degrees-of-freedom joint: axes 0..2 are translation along the x, y, z axes of frame A, 3..5 rotation about them (3 is the twist, 4 and 5 the swing).</summary>
internal struct SixDofData
{
    public JB6 Mode;        // 0 locked, 1 limited, 2 free
    public JF6 Lo, Hi;      // limits of limited axes: metres, or radians
    public JF6 MotorSpeed, MaxMotor;
    public JF6 Hertz, Damping;
    public byte MotorMask, SpringMask;   // bit i: axis i has a motor / a spring
}

/// <summary>
/// The parameters of one 3D joint, as the engine keeps and snapshots them (indexed by constraint slot). One record serves every joint type; docs/world3d.md lists the meaning of the shared fields per type.
/// A joint has a frame on each body (a point and a rotation in the body's origin frame): the frame's x axis is the joint's main axis (hinge axis, slider axis, twist axis, gear axis).
/// The last five fields are runtime bookkeeping, not part of a snapshot.
/// </summary>
internal struct JointData3D
{
    public const byte FlagLimit = 1, FlagMotor = 2, FlagSpring = 4, FlagCollide = 8, FlagLoop = 16, FlagDrive = 32;
    public const float MaxLocal = 2097152f;
    public const int MaxPathPoints = 256;

    public byte Type;
    public byte Flags;
    public byte Aux;                      // path: rotation mode
    public float LAx, LAy, LAz, LBx, LBy, LBz;
    public QuaternionF QA, QB;            // frames in the bodies' origin frames
    public float Lower, Upper;
    public float Swing1, Swing2;          // cone half angle; swing-twist half angles about frame y and z
    public float MotorSpeed, MaxMotor;
    public float Hertz, Damping;
    public float Length, Ratio;
    public double FxA, FyA, FzA, FxB, FyB, FzB;   // pulley fixed points, world coordinates relative to the world origin
    public float UpX, UpY, UpZ;           // path: up vector in A's frame
    public Vector3F[]? Path;              // path points in A's origin frame; never mutated once set
    public QuaternionF DriveTarget;       // swing-twist pose drive (FlagDrive): the wanted rotation of frame B relative to frame A; Length and Ratio are the drive's torque stiffness (N m/rad) and damping (N m s/rad), MaxMotor its torque limit
    public SixDofData Six;
    public float BreakForce, BreakTorque;

    // Runtime only.
    public int BodyA, BodyB, GenA, GenB, Gen;

    public readonly bool Collide => (Flags & FlagCollide) != 0;
    public readonly bool Limit => (Flags & FlagLimit) != 0;
    public readonly bool Motor => (Flags & FlagMotor) != 0;
    public readonly bool Spring => (Flags & FlagSpring) != 0;
    public readonly bool Drive => (Flags & FlagDrive) != 0;

    public static string Name(JointType3D t) => t switch
    {
        JointType3D.RackAndPinion => "rack-and-pinion",
        JointType3D.SwingTwist => "swing-twist",
        JointType3D.SixDof => "six-dof",
        _ => t.ToString().ToLowerInvariant(),
    };

    private static string Num(float v) => v.ToString("R", CultureInfo.InvariantCulture);

    private static void Finite(float v, string field, JointType3D t)
    {
        if (!float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Name(t) + " joint: " + field + " = " + Num(v) + " is not finite."); }
    }

    private static void AtLeastZero(float v, string field, JointType3D t)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(field, v, Name(t) + " joint: " + field + " = " + Num(v) + " must be finite and at least 0."); }
    }

    private static void Local(float x, float y, float z, string field, JointType3D t)
    {
        Finite(x, field, t); Finite(y, field, t); Finite(z, field, t);
        if (Math.Abs(x) > MaxLocal || Math.Abs(y) > MaxLocal || Math.Abs(z) > MaxLocal) { throw new ArgumentOutOfRangeException(field, "(" + Num(x) + ", " + Num(y) + ", " + Num(z) + ")", Name(t) + " joint: " + field + " is outside the supported range of +-2^21."); }
    }

    private static void AngleRange(float lo, float hi, string what, JointType3D t)
    {
        Finite(lo, "Lower" + what, t); Finite(hi, "Upper" + what, t);
        if (lo > hi) { throw new InvalidJointException(Name(t) + " joint: Lower" + what + " " + Num(lo) + " is above Upper" + what + " " + Num(hi) + "."); }
        if (lo < -3.1415927f || hi > 3.1415927f) { throw new ArgumentOutOfRangeException("Lower" + what, "[" + Num(lo) + ", " + Num(hi) + "]", Name(t) + " joint: angle limits must lie within -pi..pi."); }
    }

    private static void HalfAngle(float v, string field, JointType3D t)
    {
        if (!(v > 0f) || !(v <= 3.1415927f)) { throw new ArgumentOutOfRangeException(field, v, Name(t) + " joint: " + field + " = " + Num(v) + " must be above 0 and at most pi."); }
    }

    private static bool UsableQuaternion(QuaternionF q)
    {
        float l2 = q.LengthSquared();
        return float.IsFinite(l2) && l2 > 1e-30f;
    }

    /// <summary>Checks every value the type uses. Throws <see cref="ArgumentOutOfRangeException"/> for non-finite or out-of-range numbers and <see cref="InvalidJointException"/> for limits the wrong way round and similar.</summary>
    public readonly void Validate()
    {
        var t = (JointType3D)Type;
        if (t < JointType3D.Fixed || t > JointType3D.Pulley) { throw new InvalidJointException("Unknown joint type " + Type.ToString(CultureInfo.InvariantCulture) + "."); }
        if (float.IsNaN(BreakForce) || BreakForce < 0f) { throw new ArgumentOutOfRangeException(nameof(BreakForce), BreakForce, Name(t) + " joint: BreakForce must be at least 0 (infinity means never)."); }
        if (float.IsNaN(BreakTorque) || BreakTorque < 0f) { throw new ArgumentOutOfRangeException(nameof(BreakTorque), BreakTorque, Name(t) + " joint: BreakTorque must be at least 0 (infinity means never)."); }
        Local(LAx, LAy, LAz, "LocalFrameA.Position", t);
        Local(LBx, LBy, LBz, "LocalFrameB.Position", t);
        if (!UsableQuaternion(QA)) { throw new ArgumentOutOfRangeException("LocalFrameA.Rotation", QA, Name(t) + " joint: LocalFrameA.Rotation must be a finite, non-zero quaternion."); }
        if (!UsableQuaternion(QB)) { throw new ArgumentOutOfRangeException("LocalFrameB.Rotation", QB, Name(t) + " joint: LocalFrameB.Rotation must be a finite, non-zero quaternion."); }
        if ((Flags & FlagDrive) != 0)
        {
            if (t != JointType3D.SwingTwist) { throw new InvalidJointException(Name(t) + " joint: only a swing-twist joint has a pose drive."); }
            AtLeastZero(Length, "DriveStiffness", t); AtLeastZero(Ratio, "DriveDamping", t); AtLeastZero(MaxMotor, "DriveMaxTorque", t);
            if (!UsableQuaternion(DriveTarget)) { throw new ArgumentOutOfRangeException("DriveTarget", DriveTarget, "swing-twist joint: DriveTarget must be a finite, non-zero quaternion."); }
        }

        switch (t)
        {
            case JointType3D.Fixed:
            case JointType3D.Point:
                break;
            case JointType3D.Distance:
                if (!(Length >= 0.005f) || !float.IsFinite(Length)) { throw new ArgumentOutOfRangeException("Length", Length, "distance joint: Length = " + Num(Length) + " must be finite and at least 0.005."); }
                AtLeastZero(Lower, "MinLength", t);
                if (float.IsNaN(Upper) || Upper < Lower) { throw new InvalidJointException("distance joint: MaxLength " + Num(Upper) + " is below MinLength " + Num(Lower) + "."); }
                AtLeastZero(Hertz, "SpringHertz", t); AtLeastZero(Damping, "SpringDampingRatio", t);
                break;
            case JointType3D.Hinge:
                if (Limit) { AngleRange(Lower, Upper, "Angle", t); }
                MotorSpring(t, "MaxMotorTorque");
                break;
            case JointType3D.Slider:
                if (Limit)
                {
                    Finite(Lower, "LowerTranslation", t); Finite(Upper, "UpperTranslation", t);
                    if (Lower > Upper) { throw new InvalidJointException("slider joint: LowerTranslation " + Num(Lower) + " is above UpperTranslation " + Num(Upper) + "."); }
                }

                MotorSpring(t, "MaxMotorForce");
                break;
            case JointType3D.Cone:
                HalfAngle(Swing1, "HalfConeAngle", t);
                break;
            case JointType3D.SwingTwist:
                AngleRange(Lower, Upper, "TwistAngle", t);
                HalfAngle(Swing1, "SwingYHalfAngle", t); HalfAngle(Swing2, "SwingZHalfAngle", t);
                break;
            case JointType3D.SixDof:
                for (int i = 0; i < 6; i++)
                {
                    string axis = i < 3 ? "Linear" + "XYZ"[i] : "Angular" + "XYZ"[i - 3];
                    if (Six.Mode[i] > 2) { throw new InvalidJointException("six-dof joint: " + axis + " mode " + Six.Mode[i].ToString(CultureInfo.InvariantCulture) + " is not locked, limited or free."); }
                    if (Six.Mode[i] == 1)
                    {
                        Finite(Six.Lo[i], axis + " lower limit", t); Finite(Six.Hi[i], axis + " upper limit", t);
                        if (Six.Lo[i] > Six.Hi[i]) { throw new InvalidJointException("six-dof joint: " + axis + " lower limit " + Num(Six.Lo[i]) + " is above the upper limit " + Num(Six.Hi[i]) + "."); }
                        if (i >= 3 && (Six.Lo[i] < -3.1415927f || Six.Hi[i] > 3.1415927f)) { throw new ArgumentOutOfRangeException(axis + " limit", "[" + Num(Six.Lo[i]) + ", " + Num(Six.Hi[i]) + "]", "six-dof joint: angle limits must lie within -pi..pi."); }
                    }

                    Finite(Six.MotorSpeed[i], axis + " motor speed", t); AtLeastZero(Six.MaxMotor[i], axis + " maximum motor force", t);
                    AtLeastZero(Six.Hertz[i], axis + " spring hertz", t); AtLeastZero(Six.Damping[i], axis + " spring damping ratio", t);
                }

                if ((Six.MotorMask | Six.SpringMask) > 63) { throw new InvalidJointException("six-dof joint: motor or spring mask has bits beyond the six axes."); }
                break;
            case JointType3D.Path:
                if (Path == null || Path.Length < 2 || Path.Length > MaxPathPoints) { throw new InvalidJointException("path joint: a path needs 2 to " + MaxPathPoints.ToString(CultureInfo.InvariantCulture) + " points."); }
                for (int i = 0; i < Path.Length; i++) { Local(Path[i].X, Path[i].Y, Path[i].Z, "Path[" + i.ToString(CultureInfo.InvariantCulture) + "]", t); }
                Finite(UpX, "Up", t); Finite(UpY, "Up", t); Finite(UpZ, "Up", t);
                if (Aux > 2) { throw new InvalidJointException("path joint: rotation mode " + Aux.ToString(CultureInfo.InvariantCulture) + " is not free, tangent or full."); }
                Finite(MotorSpeed, "MotorSpeed", t); AtLeastZero(MaxMotor, "MaxMotorForce", t);
                break;
            case JointType3D.Gear:
            case JointType3D.RackAndPinion:
                if (!float.IsFinite(Ratio) || Math.Abs(Ratio) < 1e-6f || Math.Abs(Ratio) > 1e6f) { throw new ArgumentOutOfRangeException("Ratio", Ratio, Name(t) + " joint: Ratio = " + Num(Ratio) + " must be finite with an absolute value within 1e-6..1e6."); }
                break;
            case JointType3D.Pulley:
                if (!float.IsFinite(Ratio) || !(Ratio >= 1e-6f) || Ratio > 1e6f) { throw new ArgumentOutOfRangeException("Ratio", Ratio, "pulley joint: Ratio = " + Num(Ratio) + " must be within 1e-6..1e6."); }
                Fixed(FxA, FyA, FzA, "FixedPointA", t); Fixed(FxB, FyB, FzB, "FixedPointB", t);
                AtLeastZero(Lower, "MinLength", t);
                if (float.IsNaN(Upper) || Upper < Lower) { throw new InvalidJointException("pulley joint: MaxLength " + Num(Upper) + " is below MinLength " + Num(Lower) + "."); }
                break;
        }
    }

    private static void Fixed(double x, double y, double z, string field, JointType3D t)
    {
        if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z) || Math.Abs(x) > 2097152.0 || Math.Abs(y) > 2097152.0 || Math.Abs(z) > 2097152.0)
        {
            throw new ArgumentOutOfRangeException(field, "(" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + ", " + z.ToString("R", CultureInfo.InvariantCulture) + ")", Name(t) + " joint: " + field + " must be finite and within +-2^21.");
        }
    }

    private readonly void MotorSpring(JointType3D t, string maxName)
    {
        Finite(MotorSpeed, "MotorSpeed", t);
        AtLeastZero(MaxMotor, maxName, t);
        AtLeastZero(Hertz, "SpringHertz", t);
        AtLeastZero(Damping, "SpringDampingRatio", t);
    }

    /// <summary>Normalizes the frame rotations (a unit quaternion is what the solver assumes) and rounds Length for the types that have a derived one. Call after <see cref="Validate"/>.</summary>
    public void Prepare()
    {
        QA = Engine3D.Normalize(QA, "LocalFrameA.Rotation");
        QB = Engine3D.Normalize(QB, "LocalFrameB.Rotation");
        if ((JointType3D)Type == JointType3D.Path)
        {
            float l2 = UpX * UpX + UpY * UpY + UpZ * UpZ;
            if (!(l2 > 1e-30f)) { UpX = 0f; UpY = 1f; UpZ = 0f; }
            else { float inv = 1f / float.Sqrt(l2); UpX *= inv; UpY *= inv; UpZ *= inv; }
        }

        if ((Flags & FlagDrive) != 0) { DriveTarget = Engine3D.Normalize(DriveTarget, "DriveTarget"); }
    }

    // ---- snapshot ----

    /// <summary>Bytes of a record without path points.</summary>
    public const int MinRecordBytes = 3 + 4 * 27 + 8 * 6 + 6 + 4 * 6 * 6 + 2 + 4 * 2 + 4;

    public readonly void Write(StateWriter w)
    {
        w.WriteByte(Type); w.WriteByte(Flags); w.WriteByte(Aux);
        w.WriteSingle(LAx); w.WriteSingle(LAy); w.WriteSingle(LAz); w.WriteSingle(LBx); w.WriteSingle(LBy); w.WriteSingle(LBz);
        w.WriteSingle(QA.X); w.WriteSingle(QA.Y); w.WriteSingle(QA.Z); w.WriteSingle(QA.W);
        w.WriteSingle(QB.X); w.WriteSingle(QB.Y); w.WriteSingle(QB.Z); w.WriteSingle(QB.W);
        w.WriteSingle(Lower); w.WriteSingle(Upper); w.WriteSingle(Swing1); w.WriteSingle(Swing2);
        w.WriteSingle(MotorSpeed); w.WriteSingle(MaxMotor); w.WriteSingle(Hertz); w.WriteSingle(Damping);
        w.WriteSingle(Length); w.WriteSingle(Ratio);
        w.WriteSingle(UpX); w.WriteSingle(UpY); w.WriteSingle(UpZ);
        w.WriteDouble(FxA); w.WriteDouble(FyA); w.WriteDouble(FzA); w.WriteDouble(FxB); w.WriteDouble(FyB); w.WriteDouble(FzB);
        for (int i = 0; i < 6; i++) { w.WriteByte(Six.Mode[i]); }
        for (int i = 0; i < 6; i++) { w.WriteSingle(Six.Lo[i]); }
        for (int i = 0; i < 6; i++) { w.WriteSingle(Six.Hi[i]); }
        for (int i = 0; i < 6; i++) { w.WriteSingle(Six.MotorSpeed[i]); }
        for (int i = 0; i < 6; i++) { w.WriteSingle(Six.MaxMotor[i]); }
        for (int i = 0; i < 6; i++) { w.WriteSingle(Six.Hertz[i]); }
        for (int i = 0; i < 6; i++) { w.WriteSingle(Six.Damping[i]); }
        w.WriteByte(Six.MotorMask); w.WriteByte(Six.SpringMask);
        w.WriteSingle(BreakForce); w.WriteSingle(BreakTorque);
        int n = Path?.Length ?? 0;
        w.WriteInt32(n);
        for (int i = 0; i < n; i++) { w.WriteSingle(Path![i].X); w.WriteSingle(Path[i].Y); w.WriteSingle(Path[i].Z); }
        // The pose drive's target follows only when the flag is set, so a joint without one keeps its bytes.
        if ((Flags & FlagDrive) != 0) { w.WriteSingle(DriveTarget.X); w.WriteSingle(DriveTarget.Y); w.WriteSingle(DriveTarget.Z); w.WriteSingle(DriveTarget.W); }
    }

    public static JointData3D Read(StateReader r)
    {
        var d = default(JointData3D);
        d.Type = r.ReadByte(); d.Flags = r.ReadByte(); d.Aux = r.ReadByte();
        if ((d.Flags & ~(FlagLimit | FlagMotor | FlagSpring | FlagCollide | FlagLoop | FlagDrive)) != 0) { throw r.Error("invalid joint flags"); }
        d.LAx = r.ReadSingle(); d.LAy = r.ReadSingle(); d.LAz = r.ReadSingle(); d.LBx = r.ReadSingle(); d.LBy = r.ReadSingle(); d.LBz = r.ReadSingle();
        d.QA = new QuaternionF(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        d.QB = new QuaternionF(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        d.Lower = r.ReadSingle(); d.Upper = r.ReadSingle(); d.Swing1 = r.ReadSingle(); d.Swing2 = r.ReadSingle();
        d.MotorSpeed = r.ReadSingle(); d.MaxMotor = r.ReadSingle(); d.Hertz = r.ReadSingle(); d.Damping = r.ReadSingle();
        d.Length = r.ReadSingle(); d.Ratio = r.ReadSingle();
        d.UpX = r.ReadSingle(); d.UpY = r.ReadSingle(); d.UpZ = r.ReadSingle();
        d.FxA = r.ReadDouble(); d.FyA = r.ReadDouble(); d.FzA = r.ReadDouble(); d.FxB = r.ReadDouble(); d.FyB = r.ReadDouble(); d.FzB = r.ReadDouble();
        for (int i = 0; i < 6; i++) { d.Six.Mode[i] = r.ReadByte(); }
        for (int i = 0; i < 6; i++) { d.Six.Lo[i] = r.ReadSingle(); }
        for (int i = 0; i < 6; i++) { d.Six.Hi[i] = r.ReadSingle(); }
        for (int i = 0; i < 6; i++) { d.Six.MotorSpeed[i] = r.ReadSingle(); }
        for (int i = 0; i < 6; i++) { d.Six.MaxMotor[i] = r.ReadSingle(); }
        for (int i = 0; i < 6; i++) { d.Six.Hertz[i] = r.ReadSingle(); }
        for (int i = 0; i < 6; i++) { d.Six.Damping[i] = r.ReadSingle(); }
        d.Six.MotorMask = r.ReadByte(); d.Six.SpringMask = r.ReadByte();
        d.BreakForce = r.ReadSingle(); d.BreakTorque = r.ReadSingle();
        int n = r.ReadCount(12, "joint path points");
        if (n > MaxPathPoints) { throw r.Error("too many joint path points"); }
        if (n > 0)
        {
            d.Path = new Vector3F[n];
            for (int i = 0; i < n; i++) { d.Path[i] = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()); }
        }

        if ((d.Flags & FlagDrive) != 0) { d.DriveTarget = new QuaternionF(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle()); }

        try
        {
            d.Validate();
            float la = d.QA.LengthSquared(), lb = d.QB.LengthSquared();
            if (!(Math.Abs(la - 1f) <= 1e-3f) || !(Math.Abs(lb - 1f) <= 1e-3f)) { throw new InvalidJointException("frame rotation is not a unit quaternion"); }
            if (d.Drive && !(Math.Abs(d.DriveTarget.LengthSquared() - 1f) <= 1e-3f)) { throw new InvalidJointException("drive target is not a unit quaternion"); }
            if ((JointType3D)d.Type == JointType3D.Path && Math.Abs(d.UpX * d.UpX + d.UpY * d.UpY + d.UpZ * d.UpZ - 1f) > 1e-3f) { throw new InvalidJointException("path up vector is not a unit vector"); }
        }
        catch (Exception e) when (e is ArgumentException or PhysicsException)
        {
            throw r.Error("invalid joint record: " + e.Message);
        }

        return d;
    }
}
