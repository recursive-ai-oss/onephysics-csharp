using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Scene;

/// <summary>A soft body of a scene: a generator kind with its parameters (docs/spec/scenes.md, "soft" records). Every engine builds the same mesh from the deterministic soft mesh generators.</summary>
public sealed class SoftDef
{
    internal int SourceLine { get; set; }
    /// <summary>Scene Preset field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Preset { get; set; } = "jelly";   // cloth, rubber, jelly: each engine's own starting values for that kind of material
    /// <summary>Scene Kind field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Kind { get; set; } = "box";       // cloth, box, ball
    /// <summary>Scene X field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double X { get; set; }
    /// <summary>Scene Y field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Y { get; set; }
    /// <summary>Scene Z field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Z { get; set; }
    /// <summary>Scene Qx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qx { get; set; }
    /// <summary>Scene Qy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qy { get; set; }
    /// <summary>Scene Qz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qz { get; set; }
    /// <summary>Scene Qw field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qw { get; set; } = 1;
    /// <summary>Scene Mass field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Mass { get; set; } = 1;
    /// <summary>Scene P field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double[] P { get; set; } = [];           // cloth: width depth n pinMode (0 none, 1 four corners); box: sx sy sz nx ny nz; ball: radius n
}

/// <summary>One shape attached to the most recent body. Parameters are in the body's local frame.</summary>
public sealed class ShapeDef
{
    /// <summary>Body-frame surface settings. Default stationary, two-sided and isotropic; validated when loading.</summary>
    public SurfaceProperties Surface { get; set; }
    /// <summary>Optional global material index per triangle/segment. Empty uses the shape's scalar material.</summary>
    public int[] MaterialIndices { get; set; } = [];

    internal int SourceLine { get; set; }
    /// <summary>Scene Kind field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Kind { get; set; } = "";        // circle, box, capsule, polygon, segment, chain
    /// <summary>Scene Material field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Material { get; set; }
    /// <summary>Scene Density field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Density { get; set; }
    /// <summary>Scene Radius field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Radius { get; set; }           // skin radius: circle/capsule/polygon
    /// <summary>Scene P field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double[] P { get; set; } = [];         // kind-specific numbers, see docs/spec/scenes.md
}

/// <summary>The scripted input of a character: walk at <c>Speed</c> m/s, reversing every <c>Period</c> steps; jump every <c>Jump</c> steps (0 = never); stop walking and jumping at step <c>Stop</c>.</summary>
public sealed class CharacterScript
{
    /// <summary>Scene Speed field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Speed { get; set; }
    /// <summary>Scene Push field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Push { get; set; }
    /// <summary>Scene Period field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Period { get; set; }
    /// <summary>Scene Jump field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Jump { get; set; }
    /// <summary>Scene Stop field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Stop { get; set; }
}

/// <summary>The scripted drive of a vehicle chassis: throttle held until step <c>BrakeFrom</c>, then full brake; steering follows a triangle wave of amplitude <c>SteerAmp</c> (fraction of full lock) and period <c>SteerPeriod</c> steps that starts at its peak (a cosine-like slalom: the car's sideways offset stays on one side, bounded) between <c>SteerFrom</c> and <c>BrakeFrom</c>.</summary>
public sealed class VehicleScript
{
    /// <summary>Scene Preset field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Preset { get; set; } = "car";
    /// <summary>Scene Throttle field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Throttle { get; set; }
    /// <summary>Scene SteerAmp field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double SteerAmp { get; set; }
    /// <summary>Scene SteerPeriod field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int SteerPeriod { get; set; }
    /// <summary>Scene SteerFrom field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int SteerFrom { get; set; }
    /// <summary>Scene BrakeFrom field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int BrakeFrom { get; set; }

    /// <summary>Triangle wave with period 1 and amplitude 1, starting at 0 and rising (a quarter period later it peaks).</summary>
    static double Tri(double p) { double f = p - Math.Floor(p); return f < 0.25 ? 4 * f : f < 0.75 ? 2 - 4 * f : 4 * f - 4; }

    /// <summary>Inputs for a step: throttle, steer, brake.</summary>
    public (double Throttle, double Steer, double Brake) At(int step) =>
        step >= BrakeFrom ? (0, 0, 1) : (Throttle, step >= SteerFrom ? SteerAmp * Tri((step - SteerFrom) / (double)SteerPeriod + 0.25) : 0, 0);
}

/// <summary>Mutable BodyDef authoring record, using the dimensional scene units and field order.</summary>
public sealed class BodyDef
{
    /// <summary>Optional rigid-body locks and caps. Defaults inherit world limits.</summary>
    public BodyControlSettings Controls { get; set; }
    /// <summary>Optional separate sleep thresholds and duration; default inherits the world policy.</summary>
    public BodySleepSettings SleepSettings { get; set; }
    /// <summary>Prevents automatic island sleep; default false.</summary>
    public bool NeverSleep { get; set; }
    /// <summary>Optional dynamic mass override in kg.</summary>
    public float? MassOverride { get; set; }
    /// <summary>Optional local center override in metres. Planar records use X/Y with Z zero.</summary>
    public Vector3F? CenterOverride { get; set; }
    /// <summary>Optional principal inertia override in kg m². Planar records use X only.</summary>
    public Vector3F? InertiaOverride { get; set; }
    /// <summary>Principal inertia frame in 3D. Default identity; not used in 2D.</summary>
    public QuaternionF InertiaRotation { get; set; } = QuaternionF.Identity;
    /// <summary>Optional target body-origin position for a kinematic body's next step; planar Z is zero.</summary>
    public Vector3D? TargetPosition { get; set; }
    /// <summary>Target orientation in 3D, normalized when building the world.</summary>
    public QuaternionF TargetRotation { get; set; } = QuaternionF.Identity;
    /// <summary>Target planar angle, radians; used with TargetPosition in 2D.</summary>
    public float TargetAngle { get; set; }

    internal int SourceLine { get; set; }
    /// <summary>Scene Vehicle field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public VehicleScript? Vehicle { get; set; }    // set on a dynamic box body that a vehicle-aware driver turns into a car; other drivers see a plain box
    /// <summary>Scene Script field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public CharacterScript? Script { get; set; }   // set on a body (a capsule) that a character-aware driver moves as a character; other drivers see a plain dynamic capsule
    /// <summary>Scene Type field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Type { get; set; } = "dynamic"; // static, kinematic, dynamic
    /// <summary>Scene X field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double X { get; set; }
    /// <summary>Scene Y field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Y { get; set; }
    /// <summary>Scene Angle field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Angle { get; set; }
    /// <summary>Scene Vx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Vx { get; set; }
    /// <summary>Scene Vy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Vy { get; set; }
    /// <summary>Scene W field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double W { get; set; }   // 2D: W is the angular velocity
    /// <summary>Scene Z field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Z { get; set; }
    /// <summary>Scene Qx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qx { get; set; }
    /// <summary>Scene Qy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qy { get; set; }
    /// <summary>Scene Qz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qz { get; set; }
    /// <summary>Scene Qw field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Qw { get; set; } = 1;
    /// <summary>Scene Vz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Vz { get; set; }
    /// <summary>Scene Wx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Wx { get; set; }
    /// <summary>Scene Wy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Wy { get; set; } // 3D: orientation quaternion; W is the z component of the angular velocity
    /// <summary>Scene Bullet field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public bool Bullet { get; set; }
    /// <summary>Scene Asleep field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public bool Asleep { get; set; }
    /// <summary>Scene Shapes field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<ShapeDef> Shapes { get; set; } = [];
}

/// <summary>Mutable JointDef authoring record, using the dimensional scene units and field order.</summary>
public sealed class JointDef
{
    internal int SourceLine { get; set; }
    /// <summary>Type-specific machine joint parameters; empty by default. Ordered fields are specified in the joint scene format.</summary>
    public double[] Data { get; set; } = [];
    /// <summary>Whole steps until joint expiry; zero means unlimited. Must be nonnegative.</summary>
    public int LifetimeSteps { get; set; }
    /// <summary>Whole-step force threshold in N; infinity means unlimited.</summary>
    public float BreakForce { get; set; } = float.PositiveInfinity;
    /// <summary>Whole-step torque threshold in N m; infinity means unlimited.</summary>
    public float BreakTorque { get; set; } = float.PositiveInfinity;
    /// <summary>Scene Kind field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Kind { get; set; } = "revolute";
    /// <summary>Scene A field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int A { get; set; }
    /// <summary>Scene B field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int B { get; set; }                // body indices; -1 is not allowed (use a static body)
    /// <summary>Scene Ax field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Ax { get; set; }
    /// <summary>Scene Ay field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Ay { get; set; }
    /// <summary>Scene Bx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Bx { get; set; }
    /// <summary>Scene By field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double By { get; set; }   // anchors in the local frames of A and B
    /// <summary>Scene Az field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Az { get; set; }
    /// <summary>Scene Bz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Bz { get; set; }
    /// <summary>Scene Ux field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Ux { get; set; }
    /// <summary>Scene Uy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Uy { get; set; }
    /// <summary>Scene Uz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Uz { get; set; } // 3D: third anchor components and the hinge axis (local to both bodies)
    /// <summary>Scene HasLimit field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public bool HasLimit { get; set; } /// <summary>Scene Lower field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Lower { get; set; }
    /// <summary>Scene Upper field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Upper { get; set; }
    /// <summary>Scene HasMotor field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public bool HasMotor { get; set; } /// <summary>Scene MotorSpeed field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double MotorSpeed { get; set; }
    /// <summary>Scene MaxTorque field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double MaxTorque { get; set; }
    /// <summary>Scene HasSwing field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public bool HasSwing { get; set; } /// <summary>Scene SwingY field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double SwingY { get; set; }
    /// <summary>Scene SwingZ field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double SwingZ { get; set; }                                    // swingtwist: half angles about the frame y and z axes
    /// <summary>Scene HasDrive field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public bool HasDrive { get; set; } /// <summary>Scene DriveK field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveK { get; set; }
    /// <summary>Scene DriveC field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveC { get; set; }
    /// <summary>Scene DriveMax field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveMax { get; set; }
    /// <summary>Scene DriveAx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveAx { get; set; }
    /// <summary>Scene DriveAy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveAy { get; set; }
    /// <summary>Scene DriveAz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveAz { get; set; }
    /// <summary>Scene DriveAmp field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double DriveAmp { get; set; } /// <summary>Scene DrivePeriod field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int DrivePeriod { get; set; }   // swingtwist pose drive, see docs/spec/scenes.md
}

/// <summary>Mutable QueryDef authoring record, using the dimensional scene units and field order.</summary>
public sealed class QueryDef
{
    /// <summary>Scene Kind field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Kind { get; set; } = "";        // ray, overlap, cast
    /// <summary>Scene P field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double[] P { get; set; } = [];
}

/// <summary>Mutable EventDef authoring record, using the dimensional scene units and field order.</summary>
public sealed class EventDef
{
    /// <summary>Scene Step field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Step { get; set; }
    /// <summary>Scene Kind field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Kind { get; set; } = "";        // wake_all
}

/// <summary>A versioned, engine-neutral scene definition. Mutable authoring data; SI units and record fields are specified in docs/spec/scenes.md. Version 2 also carries exact world state.</summary>
public sealed partial class Scene
{
    /// <summary>Scene Name field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public string Name { get; set; } = "";
    /// <summary>Scene Dim field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Dim { get; set; } = 2;
    /// <summary>Scene Dt field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Dt { get; set; } = 1.0 / 60.0;
    /// <summary>Scene Steps field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Steps { get; set; } = 600;
    /// <summary>Scene Gx field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Gx { get; set; }
    /// <summary>Scene Gy field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Gy { get; set; } = -10;
    /// <summary>Scene Gz field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public double Gz { get; set; }
    /// <summary>Scene Substeps field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Substeps { get; set; }            // 0 = engine default
    /// <summary>Scene DumpEvery field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int DumpEvery { get; set; } = 30;
    /// <summary>Scene Warmup field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Warmup { get; set; } = 60;
    /// <summary>Scene Settle field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int Settle { get; set; }              // steps run before queries are timed
    /// <summary>Scene QueryReps field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public int QueryReps { get; set; } = 1;
    /// <summary>Scene Metrics field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<string> Metrics { get; set; } = [];
    /// <summary>Scene Params field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public Dictionary<string, string[]> Params { get; set; } = new();
    /// <summary>Scene Materials field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<(double Friction, double Restitution)> Materials { get; set; } = [];
    /// <summary>Optional world linear/angular/push-out caps. Constraints must be None. Null uses engine defaults.</summary>
    public BodyControlSettings? WorldLimits { get; set; }
    /// <summary>Authoring bodies in stable insertion order. Null entries are rejected.</summary>
    public List<BodyDef> Bodies { get; set; } = [];
    /// <summary>Scene Joints field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<JointDef> Joints { get; set; } = [];
    /// <summary>Scene Queries field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<QueryDef> Queries { get; set; } = [];
    /// <summary>Scene Events field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<EventDef> Events { get; set; } = [];
    /// <summary>Scene Softs field; defaults and SI units follow the scene record specification. Invalid values are rejected when loading a world.</summary>
    public List<SoftDef> Softs { get; set; } = [];

    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
    static float BreakThreshold(string s)
    {
        if (s == "Infinity") return float.PositiveInfinity;
        double v = D(s);
        if (v > float.MaxValue || v < 0) throw new FormatException("expected nonnegative float32 break threshold or Infinity");
        return (float)v;
    }
    static double D(string s) { double v = double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture); if (!double.IsFinite(v)) throw new FormatException("expected finite number, found " + s); return v; }
    static int I(string s) => int.Parse(s, CultureInfo.InvariantCulture);

    /// <summary>Reads a finite numeric parameter in invariant culture. Throws for a missing key, index or invalid number.</summary>
    public double ParamD(string key, int i = 0) => D(Params[key][i]);

    /// <summary>Reads a UTF-8 scene file. Throws SceneFormatException naming the file and line for invalid records.</summary>
    public static Scene Load(string path)
    {
        using var input = new StreamReader(path);
        return Read(input, path);
    }

    /// <summary>Reads scene text incrementally. Lines are limited to 1 MiB and the input to 128 MiB. Throws SceneFormatException for malformed input.</summary>
    public static Scene Read(TextReader input, string inputName = "scene")
    {
        ArgumentNullException.ThrowIfNull(input);
        ArgumentNullException.ThrowIfNull(inputName);
        var s = new Scene();
        s.InputName = inputName;
        BodyDef? cur = null;
        int number = 0;
        bool header = false;
        foreach (var raw in ReadLines(input, inputName))
        {
            number++; s.ActiveLine = number;
            var line = raw; int h = line.IndexOf('#'); if (h >= 0) line = line[..h];
            var t = line.Split([' ', '\t'], StringSplitOptions.RemoveEmptyEntries);
            if (t.Length == 0) continue;
            try
            {
            if (!header) { if (t.Length != 2 || t[0] != "scene" || (t[1] != "1" && t[1] != "2")) throw new FormatException("expected scene 1 or scene 2 header"); header = true; s.Version = I(t[1]); continue; }
            if (s.Version == 2) { s.ReadStateRecord(t); continue; }
            s.ValidateArity(t);
            switch (t[0])
            {
                case "scene": throw new FormatException("duplicate scene header");
                case "name": s.Name = t[1]; break;
                case "dim": s.Dim = I(t[1]); if (s.Dim != 2 && s.Dim != 3) throw new InvalidDataException("dim must be 2 or 3"); break;
                case "dt": s.Dt = D(t[1]); break;
                case "steps": s.Steps = I(t[1]); break;
                case "gravity": s.Gx = D(t[1]); s.Gy = D(t[2]); if (s.Dim == 3) s.Gz = D(t[3]); break;
                case "substeps": s.Substeps = I(t[1]); break;
                case "dump_every": s.DumpEvery = I(t[1]); break;
                case "warmup": s.Warmup = I(t[1]); break;
                case "settle": s.Settle = I(t[1]); break;
                case "query_reps": s.QueryReps = I(t[1]); break;
                case "metrics": s.Metrics.AddRange(t.Skip(1)); break;
                case "param": s.Params[t[1]] = t.Skip(2).ToArray(); break;
                case "material": if (I(t[1]) != s.Materials.Count) throw new FormatException("expected contiguous material index"); s.Materials.Add((D(t[2]), D(t[3]))); break;
                case "body" when s.Dim == 3:
                    cur = new BodyDef { SourceLine = number, Type = t[1], X = D(t[2]), Y = D(t[3]), Z = D(t[4]), Qx = D(t[5]), Qy = D(t[6]), Qz = D(t[7]), Qw = D(t[8]), Vx = D(t[9]), Vy = D(t[10]), Vz = D(t[11]), Wx = D(t[12]), Wy = D(t[13]), W = D(t[14]) };
                    foreach (var f in t[15].Split(',')) { if (f == "bullet") cur.Bullet = true; else if (f == "asleep") cur.Asleep = true; else if (f != "-") throw new FormatException("expected bullet, asleep or - flag"); }
                    s.Bodies.Add(cur); break;
                case "body":
                    cur = new BodyDef { SourceLine = number, Type = t[1], X = D(t[2]), Y = D(t[3]), Angle = D(t[4]), Vx = D(t[5]), Vy = D(t[6]), W = D(t[7]) };
                    foreach (var f in t[8].Split(',')) { if (f == "bullet") cur.Bullet = true; else if (f == "asleep") cur.Asleep = true; else if (f != "-") throw new FormatException("expected bullet, asleep or - flag"); }
                    s.Bodies.Add(cur); break;
                case "world_limits": s.WorldLimits = new(RigidbodyConstraints.None, (float)D(t[1]), (float)D(t[2]), (float)D(t[3])); break;
                case "controls":
                    if (cur == null) throw new FormatException("controls after body");
                    cur.Controls = new((RigidbodyConstraints)I(t[1]), (float)D(t[2]), (float)D(t[3]), (float)D(t[4])); break;
                case "sleep":
                    if (cur == null || (t[1] != "0" && t[1] != "1")) throw new FormatException("sleep after body with 0/1 never-sleep flag");
                    cur.NeverSleep = t[1] == "1";
                    if (D(t[4]) == 0) { if (D(t[2]) != 0 || D(t[3]) != 0) throw new FormatException("zero thresholds when inheriting sleep time"); cur.SleepSettings = default; }
                    else cur.SleepSettings = new((float)D(t[2]), (float)D(t[3]), (float)D(t[4])); break;
                case "mass":
                    if (cur == null) throw new FormatException("mass after body"); cur.MassOverride = (float)D(t[1]); break;
                case "center":
                    if (cur == null) throw new FormatException("center after body"); cur.CenterOverride = new((float)D(t[1]), (float)D(t[2]), s.Dim == 3 ? (float)D(t[3]) : 0); break;
                case "inertia":
                    if (cur == null) throw new FormatException("inertia after body");
                    cur.InertiaOverride = s.Dim == 2 ? new Vector3F((float)D(t[1]),0,0) : new Vector3F((float)D(t[1]),(float)D(t[2]),(float)D(t[3]));
                    if (s.Dim == 3) cur.InertiaRotation = new((float)D(t[4]),(float)D(t[5]),(float)D(t[6]),(float)D(t[7])); break;
                case "target":
                    if (cur == null) throw new FormatException("target after body");
                    cur.TargetPosition = new(D(t[1]),D(t[2]),s.Dim == 3 ? D(t[3]) : 0);
                    if (s.Dim == 2) cur.TargetAngle = (float)D(t[3]); else cur.TargetRotation = new((float)D(t[4]),(float)D(t[5]),(float)D(t[6]),(float)D(t[7])); break;
                case "shape":
                    if (cur == null) throw new InvalidDataException("shape before body");
                    cur.Shapes.Add(new ShapeDef { SourceLine = number, Kind = t[1], Material = I(t[2]), Density = D(t[3]), Radius = D(t[4]), P = t.Skip(5).Select(D).ToArray() });
                    break;
                case "surface":
                    if (cur == null || cur.Shapes.Count == 0) throw new FormatException("surface requires a preceding shape");
                    cur.Shapes[^1].Surface = ReadSurface(t); cur.Shapes[^1].Surface.Validate(s.Dim); break;
                case "material_indices":
                    if (cur == null || cur.Shapes.Count == 0) throw new FormatException("material_indices requires a preceding shape");
                    cur.Shapes[^1].MaterialIndices = t.Skip(1).Select(I).ToArray(); break;
                case "character":
                    if (cur == null) throw new InvalidDataException("character before body");
                    cur.Script = new CharacterScript { Speed = D(t[1]), Period = I(t[2]), Jump = I(t[3]), Stop = I(t[4]), Push = D(t[5]) };
                    break;
                case "vehicle":
                    if (cur == null) throw new InvalidDataException("vehicle before body");
                    cur.Vehicle = new VehicleScript { Preset = t[1], Throttle = D(t[2]), SteerAmp = D(t[3]), SteerPeriod = I(t[4]), SteerFrom = I(t[5]), BrakeFrom = I(t[6]) };
                    break;
                case "joint":
                    var j = new JointDef { SourceLine = number, Kind = t[1], A = I(t[2]), B = I(t[3]), Ax = D(t[4]), Ay = D(t[5]), Bx = D(t[6]), By = D(t[7]) };
                    int k0 = 8;
                    if (s.Dim == 3)
                    {
                        j.Az = D(t[6]); j.Bx = D(t[7]); j.By = D(t[8]); j.Bz = D(t[9]); k0 = 10;
                        if (j.Kind == "hinge" || j.Kind == "swingtwist") { j.Ux = D(t[10]); j.Uy = D(t[11]); j.Uz = D(t[12]); k0 = 13; }
                    }
                    for (int k = k0; k < t.Length;)
                    {
                        if (t[k] == "limit") { j.HasLimit = true; j.Lower = D(t[k + 1]); j.Upper = D(t[k + 2]); k += 3; }
                        else if (t[k] == "motor") { j.HasMotor = true; j.MotorSpeed = D(t[k + 1]); j.MaxTorque = D(t[k + 2]); k += 3; }
                        else if (t[k] == "swing") { j.HasSwing = true; j.SwingY = D(t[k + 1]); j.SwingZ = D(t[k + 2]); k += 3; }
                        else if (t[k] == "drive")
                        {
                            j.HasDrive = true; j.DriveK = D(t[k + 1]); j.DriveC = D(t[k + 2]); j.DriveMax = D(t[k + 3]);
                            j.DriveAx = D(t[k + 4]); j.DriveAy = D(t[k + 5]); j.DriveAz = D(t[k + 6]); j.DriveAmp = D(t[k + 7]); j.DrivePeriod = I(t[k + 8]); k += 9;
                        }
                        else if (t[k] == "data") { int n = I(t[k + 1]); j.Data = new double[n]; for (int z = 0; z < n; z++) j.Data[z] = D(t[k + 2 + z]); k += 2 + n; }
                        else if (t[k] == "lifetime") { j.LifetimeSteps = I(t[k + 1]); k += 2; }
                        else if (t[k] == "break") { j.BreakForce = BreakThreshold(t[k + 1]); j.BreakTorque = BreakThreshold(t[k + 2]); k += 3; }
                        else throw new InvalidDataException("unknown joint option " + t[k]);
                    }
                    s.Joints.Add(j); break;
                case "query": s.Queries.Add(new QueryDef { Kind = t[1], P = t.Skip(2).Select(D).ToArray() }); break;
                case "soft":
                    s.Softs.Add(new SoftDef { SourceLine = number, Preset = t[1], Kind = t[2], X = D(t[3]), Y = D(t[4]), Z = D(t[5]), Qx = D(t[6]), Qy = D(t[7]), Qz = D(t[8]), Qw = D(t[9]), Mass = D(t[10]), P = t.Skip(11).Select(D).ToArray() });
                    break;
                case "event": s.Events.Add(new EventDef { Step = I(t[1]), Kind = t[2] }); break;
                default: throw new InvalidDataException("unknown record " + t[0]);
            }
            }
            catch (Exception e) when (e is FormatException or OverflowException or ArgumentException or InvalidDataException)
            { throw new SceneFormatException(inputName, number, "valid " + t[0] + " record: " + e.Message); }
        }
        if (!header) throw new SceneFormatException(inputName, number + 1, "scene header");
        s.Validate();
        return s;
    }

    /// <summary>Writes canonical UTF-8 scene text without a byte-order mark. Round-trip numeric formatting preserves every double bit.</summary>
    public void Save(string path)
    {
        using var output = new StreamWriter(path, false, new UTF8Encoding(false));
        Write(output);
    }

    /// <summary>Writes canonical text to a caller-owned writer. Full-state scenes preserve every snapshot byte; authoring scenes retain the version 1 vocabulary.</summary>
    public void Write(TextWriter output)
    {
        ArgumentNullException.ThrowIfNull(output);
        Validate();
        if (Version == 2) { WriteState(output); return; }
        var sb = new StringBuilder();
        sb.Append("scene 1\n");
        if (Name.Length > 0) sb.Append("name ").Append(Name).Append('\n');
        sb.Append("dim ").Append(Dim).Append('\n');
        sb.Append("dt ").Append(F(Dt)).Append("\nsteps ").Append(Steps).Append('\n');
        sb.Append("gravity ").Append(F(Gx)).Append(' ').Append(F(Gy)); if (Dim == 3) sb.Append(' ').Append(F(Gz)); sb.Append('\n');
        if (Substeps > 0) sb.Append("substeps ").Append(Substeps).Append('\n');
        if (WorldLimits is BodyControlSettings limits) sb.Append("world_limits ").Append(F(limits.MaxLinearSpeed)).Append(' ').Append(F(limits.MaxAngularSpeed)).Append(' ').Append(F(limits.MaxDepenetrationSpeed)).Append('\n');
        sb.Append("dump_every ").Append(DumpEvery).Append("\nwarmup ").Append(Warmup).Append('\n');
        if (Settle > 0) sb.Append("settle ").Append(Settle).Append("\nquery_reps ").Append(QueryReps).Append('\n');
        sb.Append("metrics ").Append(string.Join(' ', Metrics)).Append('\n');
        foreach (var kv in Params.OrderBy(k => k.Key, StringComparer.Ordinal)) sb.Append("param ").Append(kv.Key).Append(' ').Append(string.Join(' ', kv.Value)).Append('\n');
        for (int i = 0; i < Materials.Count; i++) sb.Append("material ").Append(i).Append(' ').Append(F(Materials[i].Friction)).Append(' ').Append(F(Materials[i].Restitution)).Append('\n');
        foreach (var b in Bodies)
        {
            var fl = new List<string>(); if (b.Bullet) fl.Add("bullet"); if (b.Asleep) fl.Add("asleep");
            if (Dim == 3)
                sb.Append("body ").Append(b.Type).Append(' ').Append(F(b.X)).Append(' ').Append(F(b.Y)).Append(' ').Append(F(b.Z)).Append(' ')
                  .Append(F(b.Qx)).Append(' ').Append(F(b.Qy)).Append(' ').Append(F(b.Qz)).Append(' ').Append(F(b.Qw)).Append(' ')
                  .Append(F(b.Vx)).Append(' ').Append(F(b.Vy)).Append(' ').Append(F(b.Vz)).Append(' ').Append(F(b.Wx)).Append(' ').Append(F(b.Wy)).Append(' ').Append(F(b.W)).Append(' ')
                  .Append(fl.Count == 0 ? "-" : string.Join(',', fl)).Append('\n');
            else
            sb.Append("body ").Append(b.Type).Append(' ').Append(F(b.X)).Append(' ').Append(F(b.Y)).Append(' ').Append(F(b.Angle)).Append(' ')
              .Append(F(b.Vx)).Append(' ').Append(F(b.Vy)).Append(' ').Append(F(b.W)).Append(' ').Append(fl.Count == 0 ? "-" : string.Join(',', fl)).Append('\n');
            if (!b.Controls.IsDefault) sb.Append("controls ").Append((int)b.Controls.Constraints).Append(' ').Append(F(b.Controls.MaxLinearSpeed)).Append(' ').Append(F(b.Controls.MaxAngularSpeed)).Append(' ').Append(F(b.Controls.MaxDepenetrationSpeed)).Append('\n');
            if (b.SleepSettings.IsOverride || b.NeverSleep) sb.Append("sleep ").Append(b.NeverSleep ? 1 : 0).Append(' ').Append(F(b.SleepSettings.LinearThreshold)).Append(' ').Append(F(b.SleepSettings.AngularThreshold)).Append(' ').Append(F(b.SleepSettings.TimeToSleep)).Append('\n');
            if (b.MassOverride.HasValue) sb.Append("mass ").Append(F(b.MassOverride.Value)).Append('\n');
            if (b.CenterOverride is Vector3F center) { sb.Append("center ").Append(F(center.X)).Append(' ').Append(F(center.Y)); if (Dim == 3) sb.Append(' ').Append(F(center.Z)); sb.Append('\n'); }
            if (b.InertiaOverride is Vector3F inertia) { sb.Append("inertia ").Append(F(inertia.X)); if (Dim == 3) sb.Append(' ').Append(F(inertia.Y)).Append(' ').Append(F(inertia.Z)).Append(' ').Append(F(b.InertiaRotation.X)).Append(' ').Append(F(b.InertiaRotation.Y)).Append(' ').Append(F(b.InertiaRotation.Z)).Append(' ').Append(F(b.InertiaRotation.W)); sb.Append('\n'); }
            if (b.TargetPosition is Vector3D target) { sb.Append("target ").Append(F(target.X)).Append(' ').Append(F(target.Y)); if (Dim == 3) sb.Append(' ').Append(F(target.Z)).Append(' ').Append(F(b.TargetRotation.X)).Append(' ').Append(F(b.TargetRotation.Y)).Append(' ').Append(F(b.TargetRotation.Z)).Append(' ').Append(F(b.TargetRotation.W)); else sb.Append(' ').Append(F(b.TargetAngle)); sb.Append('\n'); }
            foreach (var sh in b.Shapes)
            {
                sb.Append("shape ").Append(sh.Kind).Append(' ').Append(sh.Material).Append(' ').Append(F(sh.Density)).Append(' ').Append(F(sh.Radius));
                foreach (var v in sh.P) sb.Append(' ').Append(F(v));
                sb.Append('\n');
                WriteSurface(sb, sh.Surface);
                if (sh.MaterialIndices.Length != 0) { sb.Append("material_indices"); foreach (int index in sh.MaterialIndices) sb.Append(' ').Append(index); sb.Append('\n'); }
            }
            if (b.Vehicle != null) sb.Append("vehicle ").Append(b.Vehicle.Preset).Append(' ').Append(F(b.Vehicle.Throttle)).Append(' ').Append(F(b.Vehicle.SteerAmp)).Append(' ').Append(b.Vehicle.SteerPeriod).Append(' ').Append(b.Vehicle.SteerFrom).Append(' ').Append(b.Vehicle.BrakeFrom).Append('\n');
            if (b.Script != null) sb.Append("character ").Append(F(b.Script.Speed)).Append(' ').Append(b.Script.Period).Append(' ').Append(b.Script.Jump).Append(' ').Append(b.Script.Stop).Append(' ').Append(F(b.Script.Push)).Append('\n');
        }
        foreach (var j in Joints)
        {
            sb.Append("joint ").Append(j.Kind).Append(' ').Append(j.A).Append(' ').Append(j.B).Append(' ');
            if (Dim == 3)
            {
                sb.Append(F(j.Ax)).Append(' ').Append(F(j.Ay)).Append(' ').Append(F(j.Az)).Append(' ').Append(F(j.Bx)).Append(' ').Append(F(j.By)).Append(' ').Append(F(j.Bz));
                if (j.Kind == "hinge" || j.Kind == "swingtwist") sb.Append(' ').Append(F(j.Ux)).Append(' ').Append(F(j.Uy)).Append(' ').Append(F(j.Uz));
            }
            else sb.Append(F(j.Ax)).Append(' ').Append(F(j.Ay)).Append(' ').Append(F(j.Bx)).Append(' ').Append(F(j.By));
            if (j.HasLimit) sb.Append(" limit ").Append(F(j.Lower)).Append(' ').Append(F(j.Upper));
            if (j.HasMotor) sb.Append(" motor ").Append(F(j.MotorSpeed)).Append(' ').Append(F(j.MaxTorque));
            if (j.HasSwing) sb.Append(" swing ").Append(F(j.SwingY)).Append(' ').Append(F(j.SwingZ));
            if (j.HasDrive) sb.Append(" drive ").Append(F(j.DriveK)).Append(' ').Append(F(j.DriveC)).Append(' ').Append(F(j.DriveMax)).Append(' ').Append(F(j.DriveAx)).Append(' ').Append(F(j.DriveAy)).Append(' ').Append(F(j.DriveAz)).Append(' ').Append(F(j.DriveAmp)).Append(' ').Append(j.DrivePeriod);
            if (j.Data.Length != 0) { sb.Append(" data ").Append(j.Data.Length); foreach (double v in j.Data) sb.Append(' ').Append(F(v)); }
            if (j.LifetimeSteps != 0) sb.Append(" lifetime ").Append(j.LifetimeSteps);
            if (float.IsFinite(j.BreakForce) || float.IsFinite(j.BreakTorque)) sb.Append(" break ").Append(F(j.BreakForce)).Append(' ').Append(F(j.BreakTorque));
            sb.Append('\n');
        }
        foreach (var q in Queries) { sb.Append("query ").Append(q.Kind); foreach (var v in q.P) sb.Append(' ').Append(F(v)); sb.Append('\n'); }
        foreach (var f in Softs)
        {
            sb.Append("soft ").Append(f.Preset).Append(' ').Append(f.Kind).Append(' ').Append(F(f.X)).Append(' ').Append(F(f.Y)).Append(' ').Append(F(f.Z)).Append(' ')
              .Append(F(f.Qx)).Append(' ').Append(F(f.Qy)).Append(' ').Append(F(f.Qz)).Append(' ').Append(F(f.Qw)).Append(' ').Append(F(f.Mass));
            foreach (var v in f.P) sb.Append(' ').Append(F(v));
            sb.Append('\n');
        }
        foreach (var e in Events) sb.Append("event ").Append(e.Step).Append(' ').Append(e.Kind).Append('\n');
        output.Write(sb.ToString());
    }
}
