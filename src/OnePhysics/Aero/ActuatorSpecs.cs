using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Aero;

/// <summary>Small vector helpers for the force models: plain IEEE operations in a fixed order.</summary>
internal static class V3
{
    public static float Dot(Vector3F a, Vector3F b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    public static Vector3F Cross(Vector3F a, Vector3F b) => new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
    public static Vector3F Add(Vector3F a, Vector3F b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static Vector3F Sub(Vector3F a, Vector3F b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static Vector3F Mul(Vector3F a, float s) => new(a.X * s, a.Y * s, a.Z * s);
    public static float Len2(Vector3F a) => a.X * a.X + a.Y * a.Y + a.Z * a.Z;
    public static float Len(Vector3F a) => float.Sqrt(Len2(a));
    public static bool Finite(Vector3F a) => float.IsFinite(a.X) && float.IsFinite(a.Y) && float.IsFinite(a.Z);

    /// <summary>The vector scaled to unit length; the caller has checked it is finite and not tiny.</summary>
    public static Vector3F Unit(Vector3F a) { float l = Len(a); return new Vector3F(a.X / l, a.Y / l, a.Z / l); }

    /// <summary>Rodrigues' rotation of v about the unit axis by the angle whose sine and cosine are given.</summary>
    public static Vector3F Rotate(Vector3F v, Vector3F axis, float sin, float cos)
    {
        var k = Cross(axis, v);
        float d = Dot(axis, v) * (1f - cos);
        return new Vector3F(v.X * cos + k.X * sin + axis.X * d, v.Y * cos + k.Y * sin + axis.Y * d, v.Z * cos + k.Z * sin + axis.Z * d);
    }

    public static ArgumentOutOfRangeException Range(string what, object? v, string message) => new(what, v, message);
    public static ArgumentOutOfRangeException Range(string what, string message) => new(what, message);
    public static ArgumentNullException Null(string what, string message) => new(what, message);
    public static ArgumentException Arg(string message, string what) => new(message, what);

    public static float Clamp01(float x) => x < 0f ? 0f : x > 1f ? 1f : x;

    public static void WriteV(StateWriter w, Vector3F v) { w.NamedSingle(v.X); w.NamedSingle(v.Y); w.NamedSingle(v.Z); }
    public static Vector3F ReadV(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    /// <summary>Checks a vector's components are finite and within the bound; names the field in the error.</summary>
    public static void Check(Vector3F v, string what, float bound, bool planar)
    {
        if (!Finite(v) || !(Math.Abs(v.X) <= bound && Math.Abs(v.Y) <= bound && Math.Abs(v.Z) <= bound))
        {
            throw new ArgumentOutOfRangeException(what, v, what + " is " + v + " but every component must be finite and within ±" + bound.ToString(CultureInfo.InvariantCulture) + ".");
        }

        if (planar && v.Z != 0f) { throw new ArgumentOutOfRangeException(what, v, what + " is " + v + " but a 2D world has no z axis."); }
    }

    public static void CheckRange(float v, float lo, float hi, string what)
    {
        if (!(v >= lo && v <= hi)) { throw new ArgumentOutOfRangeException(what, v, what + " is " + v.ToString("R", CultureInfo.InvariantCulture) + " but must be within " + lo.ToString(CultureInfo.InvariantCulture) + " to " + hi.ToString(CultureInfo.InvariantCulture) + "."); }
    }

    /// <summary>Normalises a direction that must have length at least 1e-6; a planar one must lie in the plane.</summary>
    public static Vector3F Direction(Vector3F v, string what, bool planar)
    {
        Check(v, what, 1e6f, planar);
        float l = Len(v);
        if (!(l >= 1e-6f)) { throw new ArgumentOutOfRangeException(what, v, what + " is " + v + " but must not be (nearly) zero."); }
        return new Vector3F(v.X / l, v.Y / l, v.Z / l);
    }
}

/// <summary>A lifting or bluff surface in the frame of its owner (a shape's frame for a shape's surface, the body's for a foil), copied and checked when it is attached.</summary>
internal sealed class SurfaceSpec
{
    public AeroSurfaceKind Kind;
    public AeroCoefficients Coef;
    public float Area;
    public Vector3F Chord = new(1f, 0f, 0f), Normal = new(0f, 1f, 0f), Center;
    public AeroMedium Medium;

    public SurfaceSpec Clone() => (SurfaceSpec)MemberwiseClone();

    /// <summary>Validates, normalises the axes and makes the normal exactly perpendicular to the chord.</summary>
    public void Prepare(bool planar, bool areaRequired)
    {
        if (!Enum.IsDefined(Kind)) { throw V3.Range(nameof(Kind), Kind, "Unknown surface kind."); }
        if (!Enum.IsDefined(Medium)) { throw V3.Range(nameof(Medium), Medium, "Unknown medium."); }
        Coef.Validate();
        if (!(Area >= 0f && Area <= 1e6f) || (areaRequired && !(Area > 0f))) { throw V3.Range(nameof(Area), Area, "Area is " + Area.ToString("R", CultureInfo.InvariantCulture) + " but must be within " + (areaRequired ? "above 0" : "0 (automatic)") + " to 10^6 m²."); }
        Chord = V3.Direction(Chord, nameof(Chord), planar);
        var n = V3.Direction(Normal, nameof(Normal), planar);
        float d = V3.Dot(n, Chord);
        if (Math.Abs(d) > 0.99f) { throw V3.Range(nameof(Normal), Normal, "Normal is parallel to Chord."); }
        Normal = V3.Unit(V3.Sub(n, V3.Mul(Chord, d)));
        V3.Check(Center, nameof(Center), 1e6f, planar);
    }

    public void Write(StateWriter w)
    {
        w.NamedByte((byte)Kind); w.NamedByte((byte)Medium);
        w.NamedSingle(Area); w.NamedSingle(Coef.LiftSlope); w.NamedSingle(Coef.ZeroLiftAngle); w.NamedSingle(Coef.StallAngle); w.NamedSingle(Coef.StallBlend); w.NamedSingle(Coef.MinDrag); w.NamedSingle(Coef.InducedDrag); w.NamedSingle(Coef.PlateDrag);
        V3.WriteV(w, Chord); V3.WriteV(w, Normal); V3.WriteV(w, Center);
    }

    public const int Bytes = 2 + 8 * 4 + 36;

    public static SurfaceSpec Read(StateReader r, bool planar, bool areaRequired)
    {
        var s = new SurfaceSpec { Kind = (AeroSurfaceKind)r.ReadByte(), Medium = (AeroMedium)r.ReadByte() };
        s.Area = r.ReadSingle();
        s.Coef = new AeroCoefficients { LiftSlope = r.ReadSingle(), ZeroLiftAngle = r.ReadSingle(), StallAngle = r.ReadSingle(), StallBlend = r.ReadSingle(), MinDrag = r.ReadSingle(), InducedDrag = r.ReadSingle(), PlateDrag = r.ReadSingle() };
        s.Chord = V3.ReadV(r); s.Normal = V3.ReadV(r); s.Center = V3.ReadV(r);
        try
        {
            // The stored axes were normalised when written; a different value means the bytes did not come from a world. The values read are kept as they are, so a restore writes the same bytes again.
            var c = s.Clone();
            c.Prepare(planar, areaRequired);
            if (V3.Len2(V3.Sub(c.Chord, s.Chord)) > 1e-8f || V3.Len2(V3.Sub(c.Normal, s.Normal)) > 1e-8f) { throw V3.Range("Chord", "stored axes are not unit and perpendicular"); }
        }
        catch (ArgumentException e) { throw r.Error("invalid aero surface: " + e.Message); }
        return s;
    }
}

/// <summary>A thruster definition as the engine keeps it.</summary>
internal sealed class ThrusterSpec
{
    public Vector3F Position, Direction = new(0f, 1f, 0f), GimbalAxis;
    public float MaxThrust, ResponseTime, MaxGimbal, ReverseFraction, SpinTorque, PitchSpeed, ImmersionDepth = 0.3f;
    public AeroMedium Medium;

    public ThrusterSpec Clone() => (ThrusterSpec)MemberwiseClone();

    public void Prepare(bool planar)
    {
        if (!Enum.IsDefined(Medium)) { throw V3.Range(nameof(Medium), Medium, "Unknown medium."); }
        V3.Check(Position, nameof(Position), 1e6f, planar);
        Direction = V3.Direction(Direction, nameof(Direction), planar);
        V3.CheckRange(MaxThrust, 0f, 1e9f, nameof(MaxThrust));
        V3.CheckRange(ResponseTime, 0f, 60f, nameof(ResponseTime));
        V3.CheckRange(MaxGimbal, 0f, 1.2f, nameof(MaxGimbal));
        V3.CheckRange(ReverseFraction, 0f, 1f, nameof(ReverseFraction));
        V3.CheckRange(SpinTorque, -100f, 100f, nameof(SpinTorque));
        if (planar && SpinTorque != 0f) { throw V3.Range(nameof(SpinTorque), SpinTorque, "SpinTorque is " + SpinTorque.ToString("R", CultureInfo.InvariantCulture) + " but a 2D world has no torque about the thrust axis."); }
        V3.CheckRange(PitchSpeed, 0f, 1e5f, nameof(PitchSpeed));
        V3.CheckRange(ImmersionDepth, 0.001f, 1000f, nameof(ImmersionDepth));
        if (GimbalAxis.X != 0f || GimbalAxis.Y != 0f || GimbalAxis.Z != 0f) { GimbalAxis = V3.Direction(GimbalAxis, nameof(GimbalAxis), planar); }
        else { V3.Check(GimbalAxis, nameof(GimbalAxis), 1f, planar); }
    }

    public void Write(StateWriter w)
    {
        V3.WriteV(w, Position); V3.WriteV(w, Direction); V3.WriteV(w, GimbalAxis);
        w.NamedSingle(MaxThrust); w.NamedSingle(ResponseTime); w.NamedSingle(MaxGimbal); w.NamedSingle(ReverseFraction); w.NamedSingle(SpinTorque); w.NamedSingle(PitchSpeed); w.NamedSingle(ImmersionDepth);
        w.NamedByte((byte)Medium);
    }

    public const int Bytes = 36 + 28 + 1;

    public static ThrusterSpec Read(StateReader r, bool planar)
    {
        var t = new ThrusterSpec { Position = V3.ReadV(r), Direction = V3.ReadV(r), GimbalAxis = V3.ReadV(r) };
        t.MaxThrust = r.ReadSingle(); t.ResponseTime = r.ReadSingle(); t.MaxGimbal = r.ReadSingle(); t.ReverseFraction = r.ReadSingle(); t.SpinTorque = r.ReadSingle(); t.PitchSpeed = r.ReadSingle(); t.ImmersionDepth = r.ReadSingle();
        t.Medium = (AeroMedium)r.ReadByte();
        try
        {
            var c = t.Clone();
            c.Prepare(planar);
            if (V3.Len2(V3.Sub(c.Direction, t.Direction)) > 1e-8f || V3.Len2(V3.Sub(c.GimbalAxis, t.GimbalAxis)) > 1e-8f) { throw V3.Range("Direction", "stored axes are not unit"); }
        }
        catch (ArgumentException e) { throw r.Error("invalid thruster: " + e.Message); }
        return t;
    }

    public static void WriteMany(StateWriter w, ThrusterSpec[] a) { w.NamedInt32(a.Length); foreach (var t in a) { t.Write(w); } }

    public static ThrusterSpec[] ReadMany(StateReader r, bool planar)
    {
        int n = r.ReadCount(Bytes, "thrusters");
        if (n > ActuatorLimits.MaxParts) { throw r.Error("too many thrusters in one component"); }
        var a = new ThrusterSpec[n];
        for (int i = 0; i < n; i++) { a[i] = Read(r, planar); }
        return a;
    }
}

internal static class ActuatorLimits
{
    /// <summary>Most thrusters, fans or foils in one boat or hovercraft.</summary>
    public const int MaxParts = 64;
}

/// <summary>A boat definition as the engine keeps it: hull drag, foils (keel, rudder) and propellers.</summary>
internal sealed class BoatSpec
{
    public Vector3F Forward = new(0f, 0f, 1f), Up = new(0f, 1f, 0f);
    public float DragLongitudinal = 0.05f, DragLateral = 0.5f, DragVertical = 1f, HeaveDamping = 3f, MaxRudderAngle = 0.6f, RudderRate = 1.5f;
    public SurfaceSpec[] Foils = Array.Empty<SurfaceSpec>();
    public float[] SteerGain = Array.Empty<float>();
    public ThrusterSpec[] Propellers = Array.Empty<ThrusterSpec>();

    public BoatSpec Clone()
    {
        var b = (BoatSpec)MemberwiseClone();
        b.Foils = new SurfaceSpec[Foils.Length]; for (int i = 0; i < Foils.Length; i++) { b.Foils[i] = Foils[i].Clone(); }
        b.SteerGain = (float[])SteerGain.Clone();
        b.Propellers = new ThrusterSpec[Propellers.Length]; for (int i = 0; i < Propellers.Length; i++) { b.Propellers[i] = Propellers[i].Clone(); }
        return b;
    }

    public void Prepare(bool planar)
    {
        Forward = V3.Direction(Forward, nameof(Forward), planar);
        Up = V3.Direction(Up, nameof(Up), planar);
        if (Math.Abs(V3.Dot(Forward, Up)) > 0.99f) { throw V3.Range(nameof(Up), Up, "Up is parallel to Forward."); }
        V3.CheckRange(DragLongitudinal, 0f, 1000f, nameof(DragLongitudinal));
        V3.CheckRange(DragLateral, 0f, 1000f, nameof(DragLateral));
        V3.CheckRange(DragVertical, 0f, 1000f, nameof(DragVertical));
        V3.CheckRange(HeaveDamping, 0f, 1000f, nameof(HeaveDamping));
        V3.CheckRange(MaxRudderAngle, 0f, 1.2f, nameof(MaxRudderAngle));
        V3.CheckRange(RudderRate, 0.01f, 100f, nameof(RudderRate));
        if (Foils.Length > ActuatorLimits.MaxParts || Propellers.Length > ActuatorLimits.MaxParts) { throw V3.Range(nameof(Foils), "A boat takes at most " + ActuatorLimits.MaxParts.ToString(CultureInfo.InvariantCulture) + " foils and propellers each."); }
        if (SteerGain.Length != Foils.Length) { throw V3.Arg("SteerGain needs one entry per foil.", nameof(SteerGain)); }
        for (int i = 0; i < Foils.Length; i++)
        {
            if (Foils[i] == null) { throw V3.Null(nameof(Foils), "Foil " + i.ToString(CultureInfo.InvariantCulture) + " is null."); }
            Foils[i].Prepare(planar, areaRequired: true);
            V3.CheckRange(SteerGain[i], -10f, 10f, nameof(SteerGain));
        }

        for (int i = 0; i < Propellers.Length; i++)
        {
            if (Propellers[i] == null) { throw V3.Null(nameof(Propellers), "Propeller " + i.ToString(CultureInfo.InvariantCulture) + " is null."); }
            Propellers[i].Prepare(planar);
        }
    }

    public void Write(StateWriter w)
    {
        V3.WriteV(w, Forward); V3.WriteV(w, Up);
        w.NamedSingle(DragLongitudinal); w.NamedSingle(DragLateral); w.NamedSingle(DragVertical); w.NamedSingle(HeaveDamping); w.NamedSingle(MaxRudderAngle); w.NamedSingle(RudderRate);
        w.NamedInt32(Foils.Length);
        for (int i = 0; i < Foils.Length; i++) { Foils[i].Write(w); w.NamedSingle(SteerGain[i]); }
        ThrusterSpec.WriteMany(w, Propellers);
    }

    public static BoatSpec Read(StateReader r, bool planar)
    {
        var b = new BoatSpec { Forward = V3.ReadV(r), Up = V3.ReadV(r) };
        b.DragLongitudinal = r.ReadSingle(); b.DragLateral = r.ReadSingle(); b.DragVertical = r.ReadSingle(); b.HeaveDamping = r.ReadSingle(); b.MaxRudderAngle = r.ReadSingle(); b.RudderRate = r.ReadSingle();
        int n = r.ReadCount(SurfaceSpec.Bytes + 4, "boat foils");
        if (n > ActuatorLimits.MaxParts) { throw r.Error("too many foils"); }
        b.Foils = new SurfaceSpec[n]; b.SteerGain = new float[n];
        for (int i = 0; i < n; i++) { b.Foils[i] = SurfaceSpec.Read(r, planar, true); b.SteerGain[i] = r.ReadSingle(); }
        b.Propellers = ThrusterSpec.ReadMany(r, planar);
        try
        {
            var c = b.Clone();
            c.Prepare(planar);
            if (V3.Len2(V3.Sub(c.Forward, b.Forward)) > 1e-8f || V3.Len2(V3.Sub(c.Up, b.Up)) > 1e-8f) { throw V3.Range("Forward", "stored axes are not unit"); }
        }
        catch (ArgumentException e) { throw r.Error("invalid boat: " + e.Message); }
        return b;
    }
}

/// <summary>One lift fan of a hovercraft.</summary>
internal sealed class HoverFanSpec
{
    public Vector3F Position, Direction = new(0f, -1f, 0f);
    public float MaxLift, Range = 1f, Damping;

    public HoverFanSpec Clone() => (HoverFanSpec)MemberwiseClone();

    public void Prepare(bool planar)
    {
        V3.Check(Position, nameof(Position), 1e6f, planar);
        Direction = V3.Direction(Direction, nameof(Direction), planar);
        V3.CheckRange(MaxLift, 0f, 1e9f, nameof(MaxLift));
        V3.CheckRange(Range, 0.001f, 1e4f, nameof(Range));
        V3.CheckRange(Damping, 0f, 1e9f, nameof(Damping));
    }

    public void Write(StateWriter w) { V3.WriteV(w, Position); V3.WriteV(w, Direction); w.NamedSingle(MaxLift); w.NamedSingle(Range); w.NamedSingle(Damping); }

    public const int Bytes = 24 + 12;

    public static HoverFanSpec Read(StateReader r, bool planar)
    {
        var f = new HoverFanSpec { Position = V3.ReadV(r), Direction = V3.ReadV(r) };
        f.MaxLift = r.ReadSingle(); f.Range = r.ReadSingle(); f.Damping = r.ReadSingle();
        try
        {
            var c = f.Clone();
            c.Prepare(planar);
            if (V3.Len2(V3.Sub(c.Direction, f.Direction)) > 1e-8f) { throw V3.Range("Direction", "stored axis is not unit"); }
        }
        catch (ArgumentException e) { throw r.Error("invalid hover fan: " + e.Message); }
        return f;
    }
}

/// <summary>A hovercraft definition as the engine keeps it.</summary>
internal sealed class HoverSpec
{
    public Vector3F Forward = new(0f, 0f, 1f), Up = new(0f, 1f, 0f);
    public float SkirtDrag, AngularDamping;
    public HoverFanSpec[] Fans = Array.Empty<HoverFanSpec>();
    public ThrusterSpec[] Thrusters = Array.Empty<ThrusterSpec>();

    public HoverSpec Clone()
    {
        var h = (HoverSpec)MemberwiseClone();
        h.Fans = new HoverFanSpec[Fans.Length]; for (int i = 0; i < Fans.Length; i++) { h.Fans[i] = Fans[i].Clone(); }
        h.Thrusters = new ThrusterSpec[Thrusters.Length]; for (int i = 0; i < Thrusters.Length; i++) { h.Thrusters[i] = Thrusters[i].Clone(); }
        return h;
    }

    public void Prepare(bool planar)
    {
        Forward = V3.Direction(Forward, nameof(Forward), planar);
        Up = V3.Direction(Up, nameof(Up), planar);
        if (Math.Abs(V3.Dot(Forward, Up)) > 0.99f) { throw V3.Range(nameof(Up), Up, "Up is parallel to Forward."); }
        V3.CheckRange(SkirtDrag, 0f, 1e9f, nameof(SkirtDrag));
        V3.CheckRange(AngularDamping, 0f, 1e9f, nameof(AngularDamping));
        if (Fans.Length == 0) { throw V3.Arg("A hovercraft needs at least one lift fan.", nameof(Fans)); }
        if (Fans.Length > ActuatorLimits.MaxParts || Thrusters.Length > ActuatorLimits.MaxParts) { throw V3.Range(nameof(Fans), "A hovercraft takes at most " + ActuatorLimits.MaxParts.ToString(CultureInfo.InvariantCulture) + " fans and thrusters each."); }
        for (int i = 0; i < Fans.Length; i++)
        {
            if (Fans[i] == null) { throw V3.Null(nameof(Fans), "Fan " + i.ToString(CultureInfo.InvariantCulture) + " is null."); }
            Fans[i].Prepare(planar);
        }

        for (int i = 0; i < Thrusters.Length; i++)
        {
            if (Thrusters[i] == null) { throw V3.Null(nameof(Thrusters), "Thruster " + i.ToString(CultureInfo.InvariantCulture) + " is null."); }
            Thrusters[i].Prepare(planar);
        }
    }

    public void Write(StateWriter w)
    {
        V3.WriteV(w, Forward); V3.WriteV(w, Up); w.NamedSingle(SkirtDrag); w.NamedSingle(AngularDamping);
        w.NamedInt32(Fans.Length);
        foreach (var f in Fans) { f.Write(w); }
        ThrusterSpec.WriteMany(w, Thrusters);
    }

    public static HoverSpec Read(StateReader r, bool planar)
    {
        var h = new HoverSpec { Forward = V3.ReadV(r), Up = V3.ReadV(r), SkirtDrag = r.ReadSingle(), AngularDamping = r.ReadSingle() };
        int n = r.ReadCount(HoverFanSpec.Bytes, "hover fans");
        if (n > ActuatorLimits.MaxParts) { throw r.Error("too many hover fans"); }
        h.Fans = new HoverFanSpec[n];
        for (int i = 0; i < n; i++) { h.Fans[i] = HoverFanSpec.Read(r, planar); }
        h.Thrusters = ThrusterSpec.ReadMany(r, planar);
        try
        {
            var c = h.Clone();
            c.Prepare(planar);
            if (V3.Len2(V3.Sub(c.Forward, h.Forward)) > 1e-8f || V3.Len2(V3.Sub(c.Up, h.Up)) > 1e-8f) { throw V3.Range("Forward", "stored axes are not unit"); }
        }
        catch (ArgumentException e) { throw r.Error("invalid hovercraft: " + e.Message); }
        return h;
    }
}
