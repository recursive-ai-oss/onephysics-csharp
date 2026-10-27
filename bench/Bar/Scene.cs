using System.Globalization;
using System.Text;

namespace Bar;

/// <summary>One shape attached to the most recent body. Parameters are in the body's local frame.</summary>
public sealed class ShapeDef
{
    public string Kind = "";        // circle, box, capsule, polygon, segment, chain
    public int Material;
    public double Density;
    public double Radius;           // skin radius: circle/capsule/polygon
    public double[] P = [];         // kind-specific numbers, see docs/scenes.md
}

public sealed class BodyDef
{
    public string Type = "dynamic"; // static, kinematic, dynamic
    public double X, Y, Angle, Vx, Vy, W;   // 2D: W is the angular velocity
    public double Z, Qx, Qy, Qz, Qw = 1, Vz, Wx, Wy; // 3D: orientation quaternion; W is the z component of the angular velocity
    public bool Bullet, Asleep;
    public List<ShapeDef> Shapes = [];
}

public sealed class JointDef
{
    public string Kind = "revolute";
    public int A, B;                // body indices; -1 is not allowed (use a static body)
    public double Ax, Ay, Bx, By;   // anchors in the local frames of A and B
    public double Az, Bz, Ux, Uy, Uz; // 3D: third anchor components and the hinge axis (local to both bodies)
    public bool HasLimit; public double Lower, Upper;
    public bool HasMotor; public double MotorSpeed, MaxTorque;
}

public sealed class QueryDef
{
    public string Kind = "";        // ray, overlap, cast
    public double[] P = [];
}

public sealed class EventDef
{
    public int Step;
    public string Kind = "";        // wake_all
}

public sealed class Scene
{
    public string Name = "";
    public int Dim = 2;
    public double Dt = 1.0 / 60.0;
    public int Steps = 600;
    public double Gx, Gy = -10, Gz;
    public int Substeps;            // 0 = engine default
    public int DumpEvery = 30;
    public int Warmup = 60;
    public int Settle;              // steps run before queries are timed
    public int QueryReps = 1;
    public List<string> Metrics = [];
    public Dictionary<string, string[]> Params = new();
    public List<(double Friction, double Restitution)> Materials = [];
    public List<BodyDef> Bodies = [];
    public List<JointDef> Joints = [];
    public List<QueryDef> Queries = [];
    public List<EventDef> Events = [];

    static string F(double v) => v.ToString("R", CultureInfo.InvariantCulture);
    static double D(string s) => double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
    static int I(string s) => int.Parse(s, CultureInfo.InvariantCulture);

    public double ParamD(string key, int i = 0) => D(Params[key][i]);

    public static Scene Load(string path)
    {
        var s = new Scene();
        BodyDef? cur = null;
        foreach (var raw in File.ReadLines(path))
        {
            var line = raw; int h = line.IndexOf('#'); if (h >= 0) line = line[..h];
            var t = line.Split([' ', '\t'], StringSplitOptions.RemoveEmptyEntries);
            if (t.Length == 0) continue;
            switch (t[0])
            {
                case "scene": if (t[1] != "1") throw new InvalidDataException("unsupported scene version " + t[1]); break;
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
                case "material": s.Materials.Add((D(t[2]), D(t[3]))); break;
                case "body" when s.Dim == 3:
                    cur = new BodyDef { Type = t[1], X = D(t[2]), Y = D(t[3]), Z = D(t[4]), Qx = D(t[5]), Qy = D(t[6]), Qz = D(t[7]), Qw = D(t[8]), Vx = D(t[9]), Vy = D(t[10]), Vz = D(t[11]), Wx = D(t[12]), Wy = D(t[13]), W = D(t[14]) };
                    foreach (var f in t[15].Split(',')) { if (f == "bullet") cur.Bullet = true; else if (f == "asleep") cur.Asleep = true; }
                    s.Bodies.Add(cur); break;
                case "body":
                    cur = new BodyDef { Type = t[1], X = D(t[2]), Y = D(t[3]), Angle = D(t[4]), Vx = D(t[5]), Vy = D(t[6]), W = D(t[7]) };
                    foreach (var f in t[8].Split(',')) { if (f == "bullet") cur.Bullet = true; else if (f == "asleep") cur.Asleep = true; }
                    s.Bodies.Add(cur); break;
                case "shape":
                    if (cur == null) throw new InvalidDataException("shape before body");
                    cur.Shapes.Add(new ShapeDef { Kind = t[1], Material = I(t[2]), Density = D(t[3]), Radius = D(t[4]), P = t.Skip(5).Select(D).ToArray() });
                    break;
                case "joint":
                    var j = new JointDef { Kind = t[1], A = I(t[2]), B = I(t[3]), Ax = D(t[4]), Ay = D(t[5]), Bx = D(t[6]), By = D(t[7]) };
                    int k0 = 8;
                    if (s.Dim == 3)
                    {
                        j.Az = D(t[6]); j.Bx = D(t[7]); j.By = D(t[8]); j.Bz = D(t[9]); k0 = 10;
                        if (j.Kind == "hinge") { j.Ux = D(t[10]); j.Uy = D(t[11]); j.Uz = D(t[12]); k0 = 13; }
                    }
                    for (int k = k0; k < t.Length;)
                    {
                        if (t[k] == "limit") { j.HasLimit = true; j.Lower = D(t[k + 1]); j.Upper = D(t[k + 2]); k += 3; }
                        else if (t[k] == "motor") { j.HasMotor = true; j.MotorSpeed = D(t[k + 1]); j.MaxTorque = D(t[k + 2]); k += 3; }
                        else throw new InvalidDataException("unknown joint option " + t[k]);
                    }
                    s.Joints.Add(j); break;
                case "query": s.Queries.Add(new QueryDef { Kind = t[1], P = t.Skip(2).Select(D).ToArray() }); break;
                case "event": s.Events.Add(new EventDef { Step = I(t[1]), Kind = t[2] }); break;
                default: throw new InvalidDataException("unknown record " + t[0]);
            }
        }
        return s;
    }

    public void Save(string path)
    {
        var sb = new StringBuilder();
        sb.Append("scene 1\nname ").Append(Name).Append("\ndim ").Append(Dim).Append('\n');
        sb.Append("dt ").Append(F(Dt)).Append("\nsteps ").Append(Steps).Append('\n');
        sb.Append("gravity ").Append(F(Gx)).Append(' ').Append(F(Gy)); if (Dim == 3) sb.Append(' ').Append(F(Gz)); sb.Append('\n');
        if (Substeps > 0) sb.Append("substeps ").Append(Substeps).Append('\n');
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
            foreach (var sh in b.Shapes)
            {
                sb.Append("shape ").Append(sh.Kind).Append(' ').Append(sh.Material).Append(' ').Append(F(sh.Density)).Append(' ').Append(F(sh.Radius));
                foreach (var v in sh.P) sb.Append(' ').Append(F(v));
                sb.Append('\n');
            }
        }
        foreach (var j in Joints)
        {
            sb.Append("joint ").Append(j.Kind).Append(' ').Append(j.A).Append(' ').Append(j.B).Append(' ');
            if (Dim == 3)
            {
                sb.Append(F(j.Ax)).Append(' ').Append(F(j.Ay)).Append(' ').Append(F(j.Az)).Append(' ').Append(F(j.Bx)).Append(' ').Append(F(j.By)).Append(' ').Append(F(j.Bz));
                if (j.Kind == "hinge") sb.Append(' ').Append(F(j.Ux)).Append(' ').Append(F(j.Uy)).Append(' ').Append(F(j.Uz));
            }
            else sb.Append(F(j.Ax)).Append(' ').Append(F(j.Ay)).Append(' ').Append(F(j.Bx)).Append(' ').Append(F(j.By));
            if (j.HasLimit) sb.Append(" limit ").Append(F(j.Lower)).Append(' ').Append(F(j.Upper));
            if (j.HasMotor) sb.Append(" motor ").Append(F(j.MotorSpeed)).Append(' ').Append(F(j.MaxTorque));
            sb.Append('\n');
        }
        foreach (var q in Queries) { sb.Append("query ").Append(q.Kind); foreach (var v in q.P) sb.Append(' ').Append(F(v)); sb.Append('\n'); }
        foreach (var e in Events) sb.Append("event ").Append(e.Step).Append(' ').Append(e.Kind).Append('\n');
        File.WriteAllText(path, sb.ToString());
    }
}
