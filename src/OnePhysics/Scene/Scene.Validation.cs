using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Scene;

public sealed partial class Scene
{
    private void ValidateArity(string[] t)
    {
        int exact = t[0] switch
        {
            "name" or "dim" or "dt" or "steps" or "substeps" or "dump_every" or "warmup" or "settle" or "query_reps" => 2,
            "gravity" => Dim + 1, "material" => 4, "body" => Dim == 2 ? 9 : 16,
            "character" => 6, "vehicle" => 7, "event" => 3, _ => -1
        };
        if (exact >= 0 && t.Length != exact) throw new FormatException($"{exact} tokens for {t[0]}");
        int min = t[0] switch { "param" => 3, "shape" => 5, "query" => 2, "soft" => 12, "joint" => Dim == 2 ? 8 : 10, _ => 1 };
        if (t.Length < min) throw new FormatException($"at least {min} tokens for {t[0]}");
        if (t[0] == "joint")
        {
            int k = Dim == 2 ? 8 : t[1] is "hinge" or "swingtwist" ? 13 : 10;
            if (t.Length < k) throw new FormatException("complete joint axis");
            while (k < t.Length)
            {
                int n = t[k] switch { "limit" or "motor" or "swing" => 3, "drive" => 9, _ => throw new FormatException("known joint option") };
                if (t.Length - k < n) throw new FormatException("complete " + t[k] + " joint option");
                k += n;
            }
        }
    }

    /// <summary>Validates authoring topology, ranges and geometry, or full-state framing. Throws SceneFormatException identifying the source record. Full-state snapshot contents are validated by CreateWorld2D/3D.</summary>
    public void Validate()
    {
        if (Dim != 2 && Dim != 3) throw Error("dimension 2 or 3");
        if (Version == 2) { if (_state == null || !_stateEnd || _stateWritten != _state.Length || !_dimSeen || !_idSeen) throw Error("complete state and end record"); return; }
        if (!(Dt > 0 && Dt <= float.MaxValue) || !double.IsFinite(Dt) || Steps < 0 || Warmup < 0 || Settle < 0 || DumpEvery < 1 || QueryReps < 1 || Substeps < 0 || Substeps > 64) throw Error("positive finite dt and valid step counts");
        if (Bodies == null || Joints == null || Softs == null || Materials == null || Queries == null || Events == null || Params == null) throw Error("non-null scene collections");
        if (Bodies.Count > 100000 || Joints.Count > 100000 || Softs.Count > 10000) throw Error("at most 100000 bodies/joints and 10000 soft bodies");
        Finite([Gx, Gy, Gz]);
        foreach (var m in Materials) if (!(m.Friction >= 0 && m.Friction <= 1e15 && m.Restitution >= 0 && m.Restitution <= 1)) throw Error("friction in [0,1e15] and restitution in [0,1]");
        foreach (var b in Bodies)
        {
            if (b == null) throw Error("non-null body");
            ActiveLine = b.SourceLine;
            Position(b.X, b.Y, b.Z);
            Finite([b.Angle, b.Vx, b.Vy, b.Vz, b.W, b.Wx, b.Wy, b.Qx, b.Qy, b.Qz, b.Qw]);
            if (b.Type is not ("static" or "kinematic" or "dynamic") || b.Shapes == null) throw Error("static, kinematic or dynamic body with shape list");
            if (Dim == 3 && b.Qx * b.Qx + b.Qy * b.Qy + b.Qz * b.Qz + b.Qw * b.Qw == 0) throw Error("nonzero rotation quaternion");
            foreach (var sh in b.Shapes)
            {
                if (sh == null) throw Error("non-null shape");
                ActiveLine = sh.SourceLine;
                if (sh.Material < 0 || sh.Material >= Materials.Count || !(sh.Density >= 0 && sh.Density <= 1e15) || !(sh.Radius >= 0 && sh.Radius <= 2097152)) throw Error("valid material reference, nonnegative density and radius");
                ValidateShape(sh);
            }
            if (b.Script != null)
            {
                if (b.Shapes.Count != 1 || b.Shapes[0].Kind != "capsule" || b.Script.Period < 1 || b.Script.Jump < 0 || b.Script.Stop < 0 || b.Script.Push < 0) throw Error("one capsule and valid character period/jump/stop/push");
                Finite([b.Script.Speed, b.Script.Push]);
            }
            if (b.Vehicle != null)
            {
                if (Dim != 3 || b.Vehicle.Preset is not ("car" or "tank" or "bike") || b.Vehicle.SteerPeriod < 1) throw Error("spatial vehicle preset and positive steering period");
                Finite([b.Vehicle.Throttle, b.Vehicle.SteerAmp]);
            }
        }
        foreach (var j in Joints)
        {
            if (j == null) throw Error("non-null joint");
            ActiveLine = j.SourceLine;
            if (j.A < 0 || j.A >= Bodies.Count || j.B < 0 || j.B >= Bodies.Count || j.A == j.B) throw Error("two distinct existing joint bodies");
            if (Dim == 2 ? j.Kind != "revolute" : j.Kind is not ("point" or "hinge" or "swingtwist")) throw Error("supported authoring joint kind");
            Finite([j.Ax,j.Ay,j.Az,j.Bx,j.By,j.Bz,j.Ux,j.Uy,j.Uz,j.Lower,j.Upper,j.MotorSpeed,j.MaxTorque,j.SwingY,j.SwingZ,j.DriveK,j.DriveC,j.DriveMax,j.DriveAx,j.DriveAy,j.DriveAz,j.DriveAmp]);
            if (j.HasDrive && j.DrivePeriod < 1) throw Error("positive drive period");
        }
        foreach (var d in Softs)
        {
            if (d == null) throw Error("non-null soft body");
            ActiveLine = d.SourceLine;
            Position(d.X,d.Y,d.Z); Finite([d.Mass,d.Qx,d.Qy,d.Qz,d.Qw]);
            if (Dim != 3 || !(d.Mass > 0) || d.Preset is not ("cloth" or "rubber" or "jelly") || d.P == null) throw Error("positive mass and spatial cloth/rubber/jelly preset");
            Finite(d.P);
            int count = d.Kind switch { "cloth" or "sleeve" => 4, "box" => 6, "ball" => 2, _ => throw Error("cloth, box, ball or sleeve generator") };
            if (d.P.Length != count) throw Error("complete soft generator parameters");
            int start = d.Kind == "box" ? 3 : d.Kind is "cloth" or "sleeve" ? 2 : 1;
            for (int i = 0; i < start; i++) if (!(d.P[i] > 0 && d.P[i] <= 2097152)) throw Error("positive bounded soft extent");
            for (int i = start; i < count; i++) if (d.P[i] != Math.Truncate(d.P[i]) || d.P[i] < (d.Kind == "cloth" && i == 3 ? 0 : 1) || d.P[i] > 128) throw Error("bounded integer soft resolution (1..128)");
            if (d.Kind == "ball" && (d.P[1]+1)*(d.P[1]+1)*(d.P[1]+1) > 100000) throw Error("at most 100000 soft particles");
            if (d.Kind == "box" && (d.P[3]+1)*(d.P[4]+1)*(d.P[5]+1) > 100000) throw Error("at most 100000 soft particles");
        }
        foreach (var q in Queries) { if (q == null || q.P == null || q.Kind is not ("ray" or "cast" or "overlap")) throw Error("known query kind and payload"); Finite(q.P); int n = q.Kind == "overlap" ? Dim + 1 : 2 * Dim + 1; if (q.P.Length != n) throw Error("complete query parameters"); }
        foreach (var e in Events) if (e == null || e.Step < 0 || e.Kind != "wake_all") throw Error("nonnegative wake_all event step");
        if (Params.TryGetValue("fluid", out var f)) { if (f.Length != 5) throw Error("five fluid parameters"); foreach (var v in f) { try { D(v); } catch (FormatException e) { throw Error(e.Message); } } }
    }

    private void Position(double x, double y, double z)
    {
        if (!(Math.Abs(x) <= 2097152 && Math.Abs(y) <= 2097152 && Math.Abs(z) <= 2097152)) throw Error($"finite coordinates within ±2097152, found ({x}, {y}, {z})");
    }
    private void Finite(double[] values) { foreach (double v in values) if (!double.IsFinite(v) || Math.Abs(v) > float.MaxValue) throw Error("finite float-representable numeric fields"); }

    private void ValidateShape(ShapeDef sh)
    {
        var p = sh.P;
        if (p == null || p.Length > 1000000) throw Error("non-null bounded shape parameters");
        Finite(p);
        int expected = (Dim, sh.Kind) switch
        {
            (2,"circle") => 2, (2,"capsule" or "segment") => 4, (2,"box") => 5,
            (3,"sphere") => 3, (3,"capsule") => 6, (3,"box") => 10,
            (2,"polygon" or "chain") or (3,"hull" or "mesh" or "heightfield") => -1,
            _ => throw Error("known shape kind for the dimension")
        };
        if (expected >= 0 && p.Length != expected) throw Error($"{expected} parameters for {sh.Kind}");
        if (sh.Kind is "polygon" or "chain" && (p.Length < (sh.Kind == "polygon" ? 6 : 4) || p.Length % 2 != 0)) throw Error("complete planar vertices");
        if (sh.Kind == "hull" && (p.Length < 12 || p.Length % 3 != 0)) throw Error("at least four complete hull points");
        try
        {
            if (Dim == 2)
            {
                Vec2 P(int i) => new((float)p[i], (float)p[i+1]);
                switch (sh.Kind)
                {
                    case "circle": Shape2D.CreateCircle(P(0), (float)sh.Radius); break;
                    case "capsule": Shape2D.CreateCapsule(P(0), P(2), (float)sh.Radius); break;
                    case "segment": Shape2D.CreateSegment(P(0), P(2)); break;
                    case "box": Shape2D.CreateBox((float)p[0], (float)p[1], (float)sh.Radius); break;
                    case "polygon": Shape2D.CreatePolygon(Enumerable.Range(0,p.Length/2).Select(i=>P(i*2)).ToArray(), (float)sh.Radius); break;
                    case "chain": for(int i=0;i+3<p.Length;i+=2) Shape2D.CreateSegment(P(i),P(i+2)); break;
                }
            }
            else
            {
                switch (sh.Kind)
                {
                    case "sphere": ConvexShape3D.Sphere((float)sh.Radius); break;
                    case "capsule": ConvexShape3D.Capsule(new Vector3F((float)(p[3]-p[0]),(float)(p[4]-p[1]),(float)(p[5]-p[2])).Length()*0.5f,(float)sh.Radius); break;
                    case "box": ConvexShape3D.Box(new((float)p[0],(float)p[1],(float)p[2])); break;
                    case "hull": ConvexHull3D.Build(Enumerable.Range(0,p.Length/3).Select(i=>new Vector3F((float)p[3*i],(float)p[3*i+1],(float)p[3*i+2])).ToArray()); break;
                    case "mesh":
                        if (p.Length < 2 || p[0] < 3 || p[1] < 3 || p[0] != Math.Truncate(p[0]) || p[1] != Math.Truncate(p[1]) || 2+3*p[0]+p[1] != p.Length || p[1]%3 != 0) throw Error("mesh vertex/index counts matching payload");
                        int nv=(int)p[0], ni=(int)p[1]; var v=new Vector3F[nv]; var idx=new int[ni];
                        for(int i=0;i<nv;i++) v[i]=new((float)p[2+3*i],(float)p[3+3*i],(float)p[4+3*i]);
                        for(int i=0;i<ni;i++) { double n=p[2+3*nv+i]; if(n<0 || n>=nv || n!=Math.Truncate(n)) throw Error("mesh indices naming vertices"); idx[i]=(int)n; }
                        TriangleMesh3D.Create(v,idx); break;
                    case "heightfield":
                        if(p.Length<6 || p[0]<2 || p[1]<2 || p[0]!=Math.Truncate(p[0]) || p[1]!=Math.Truncate(p[1]) || 6+p[0]*p[1]!=p.Length) throw Error("height field dimensions matching payload");
                        HeightField3D.Create(p.Skip(6).Select(x=>(float)x).ToArray(),(int)p[0],(int)p[1],new((float)p[2],1,(float)p[2])); break;
                }
            }
        }
        catch (Exception e) when(e is ArgumentException or PhysicsException) { if(e is SceneFormatException) throw; throw Error("valid "+sh.Kind+" geometry: "+e.Message); }
    }
}
