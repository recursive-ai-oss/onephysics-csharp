using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>A side-view track: its definition (copied), the evaluator and the static chain body that is the rail surface.</summary>
internal sealed class TrackState2D
{
    public TrackDef Def = new();
    public TrackGeometry Geo = null!;
    public int Body = -1, BodyGen;

    public TrackState2D CloneForFork()
    {
        var c = (TrackState2D)MemberwiseClone();
        c.Def = Def.Clone();
        return c;
    }
}

/// <summary>A side-view rail vehicle: its car, its wheeled vehicle (wheel 0 the rear wheelset), the driver's inputs and the couplers (index 0 the front end, 1 the rear).</summary>
internal sealed class RailState2D
{
    public RailDef Def = new();
    public int Car = -1, CarGen, Veh = -1, VehGen, Track = -1, TrackGen;
    public float Throttle, Brake;
    public int[] CoupleVeh = { -1, -1 }, CoupleGen = new int[2], CoupleEnd = new int[2];
    public byte[] Broken = new byte[2];
    public float[] CouplerForce = new float[2];

    public RailState2D CloneForFork()
    {
        var c = (RailState2D)MemberwiseClone();
        c.Def = Def.Clone();
        c.CoupleVeh = (int[])CoupleVeh.Clone(); c.CoupleGen = (int[])CoupleGen.Clone(); c.CoupleEnd = (int[])CoupleEnd.Clone(); c.Broken = (byte[])Broken.Clone(); c.CouplerForce = (float[])CouplerForce.Clone();
        return c;
    }
}

internal sealed partial class Engine2D
{
    /// <summary>Slots of the tracks and of the rail vehicles.</summary>
    public HandleTable TrackHandles = new(), RailHandles = new();

    public TrackState2D?[] Tracks = new TrackState2D?[2];
    public RailState2D?[] Rails = new RailState2D?[2];

    public int CreateTrack(TrackState2D t)
    {
        int i = TrackHandles.Allocate();
        if (i >= Tracks.Length) { Array.Resize(ref Tracks, Math.Max(i + 1, Tracks.Length * 2)); }
        Tracks[i] = t;
        return i;
    }

    public int CreateRailVehicle(RailState2D r)
    {
        int i = RailHandles.Allocate();
        if (i >= Rails.Length) { Array.Resize(ref Rails, Math.Max(i + 1, Rails.Length * 2)); }
        Rails[i] = r;
        return i;
    }

    public void DestroyTrack(int i)
    {
        Tracks[i] = null;
        TrackHandles.Free(i);
    }

    public void DestroyRailVehicle(int i)
    {
        var r = Rails[i]!;
        for (int e = 0; e < 2; e++)
        {
            int o = r.CoupleVeh[e];
            if (o >= 0 && RailHandles.IsAlive(o, r.CoupleGen[e])) { var os = Rails[o]!; os.CoupleVeh[r.CoupleEnd[e]] = -1; os.CouplerForce[r.CoupleEnd[e]] = 0f; }
        }

        Rails[i] = null;
        RailHandles.Free(i);
    }

    /// <summary>Called when a body is destroyed: a rail vehicle whose car it is goes (its wheeled vehicle goes with the body); a track whose chain it is loses its body.</summary>
    public void DropRailOfBody(int body)
    {
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (RailHandles.IsLive(i) && Rails[i]!.Car == body) { DestroyRailVehicle(i); }
        }

        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (TrackHandles.IsLive(i) && Tracks[i]!.Body == body) { Tracks[i]!.Body = -1; }
        }
    }

    private bool RailLive(RailState2D r) => Core.Bodies.Handles.IsAlive(r.Car, r.CarGen) && VehHandles.IsAlive(r.Veh, r.VehGen);

    /// <summary>Before the vehicles run, once per step, in slot order: hands the drive and brake torques to the wheelsets, then runs the couplers between vehicles.</summary>
    public void StepRail(float h)
    {
        if (RailHandles.Count == 0) { return; }
        var bodies = Core.Bodies;
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            if (!RailLive(r)) { continue; }
            bool active = r.Throttle != 0f || r.Brake > 0f;
            if (!bodies.IsAwake(r.Car))
            {
                if (!active) { continue; }
                WakeBodyIsland(r.Car);
            }
            else if (active) { bodies.SleepTime[r.Car] = 0f; }

            var v = Vehicles[r.Veh]!;
            var def = r.Def;
            int n = def.Axles, driven = 0;
            for (int k = 0; k < n; k++) { if ((def.DrivenWheelsets >> k & 1u) != 0u) { driven++; } }
            v.Forward = 0f; v.Brake = 0f; v.Handbrake = 0f; v.Pitch = 0f;
            for (int k = 0; k < n; k++)
            {
                v.WheelBrake[k] = r.Brake;
                v.ExtraTorque[k] = 0f;
                if ((def.DrivenWheelsets >> k & 1u) != 0u && r.Throttle != 0f && driven > 0)
                {
                    float f = def.MaxTractionForce / driven;
                    if (def.MaxPower > 0f) { f = Math.Min(f, def.MaxPower / driven / Math.Max(Math.Abs(v.Omega[k]) * def.WheelRadius, 1f)); }
                    v.ExtraTorque[k] = r.Throttle * f * def.WheelRadius;
                }
            }
        }

        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (RailHandles.IsLive(i)) { StepCouplers(i, Rails[i]!, h); }
        }
    }

    private void StepCouplers(int slot, RailState2D r, float h)
    {
        var bodies = Core.Bodies;
        for (int e = 0; e < 2; e++)
        {
            int o = r.CoupleVeh[e];
            if (o < 0 || o <= slot || r.Broken[e] != 0 || !RailHandles.IsAlive(o, r.CoupleGen[e])) { continue; }
            var os = Rails[o]!;
            int oe = r.CoupleEnd[e];
            if (!RailLive(r) || !RailLive(os)) { continue; }
            int a = r.Car, b = os.Car;
            if (!bodies.IsAwake(a) && !bodies.IsAwake(b)) { continue; }
            if (!bodies.IsAwake(a)) { WakeBodyIsland(a); }
            if (!bodies.IsAwake(b)) { WakeBodyIsland(b); }
            var da = r.Def; var db = os.Def;
            var ra = bodies.RotationOf(a); var rb = bodies.RotationOf(b);
            float ax = e == 0 ? da.CouplerReach : -da.CouplerReach, ay = da.CouplerHeight - (da.CarFloorHeight + 0.5f * da.CarSize.Y);
            float bx = oe == 0 ? db.CouplerReach : -db.CouplerReach, by = db.CouplerHeight - (db.CarFloorHeight + 0.5f * db.CarSize.Y);
            float aax = ra[0] * ax - ra[1] * ay, aay = ra[1] * ax + ra[0] * ay;
            float bbx = rb[0] * bx - rb[1] * by, bby = rb[1] * bx + rb[0] * by;
            var pa = bodies.PositionOf(a); var pb = bodies.PositionOf(b);
            float dx = (float)(pb[0] - pa[0]) + bbx - aax, dy = (float)(pb[1] - pa[1]) + bby - aay;
            float dist = float.Sqrt(dx * dx + dy * dy);
            if (!(dist > 1e-4f)) { continue; }
            float nx = dx / dist, ny = dy / dist;
            float x = dist - da.CouplerFreeLength;
            PointVelocity(a, aax, aay, out float vax, out float vay); PointVelocity(b, bbx, bby, out float vbx, out float vby);
            float vrel = (vbx - vax) * nx + (vby - vay) * ny;
            float force = 0f;
            float invPair = bodies.InverseMass[a] + bodies.InverseMass[b];
            if (x < 0f)
            {
                float f = da.BufferStiffness * -x + da.CouplerDamping * Math.Max(-vrel, 0f);
                if (invPair > 0f) { f = Math.Min(f, 0.5f * -x / (h * h * invPair)); }
                force = -f;
            }
            else if (x > da.CouplerSlack)
            {
                float ext = x - da.CouplerSlack;
                float f = da.DrawStiffness * ext + da.CouplerDamping * Math.Max(vrel, 0f);
                if (invPair > 0f) { f = Math.Min(f, 0.5f * ext / (h * h * invPair)); }
                if (f > da.CouplerBreakForce) { r.Broken[e] = 1; os.Broken[oe] = 1; r.CouplerForce[e] = 0f; os.CouplerForce[oe] = 0f; continue; }
                force = f;
            }

            r.CouplerForce[e] = force; os.CouplerForce[oe] = force;
            if (force == 0f) { continue; }
            float jx = nx * h * force, jy = ny * h * force;
            ApplyImpulseAtArm(a, aax, aay, jx, jy);
            ApplyImpulseAtArm(b, bbx, bby, -jx, -jy);
        }
    }

    // ------------------------------------------------------------------ origin shift and debug

    /// <summary>After the core shifted the origin: tracks are world geometry and move with it exactly (their chains are static bodies the core already shifted; the evaluator follows the definition's start point).</summary>
    public void ShiftRail(double dx, double dy)
    {
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            t.Def.Start = new Vector3D(t.Def.Start.X - dx, t.Def.Start.Y - dy, t.Def.Start.Z);
            t.Geo = new TrackGeometry(t.Def);
        }
    }

    /// <summary>Emits every track's profile as a polyline (z = 0).</summary>
    public void AddDebugRail(Debug.DebugGeometry g)
    {
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            double len = t.Geo.Length;
            int n = (int)Math.Min(2048.0, Math.Ceiling(len / 2.0)) + 1;
            var pts = new Vector3D[n];
            for (int k = 0; k < n; k++) { var f = t.Geo.FrameAt(len * k / (n - 1)); pts[k] = new Vector3D(f.Position.X, f.Position.Y, 0.0); }
            g.Add(new Debug.DebugTrack(i, pts, t.Def.Gauge));
        }
    }

    // ------------------------------------------------------------------ snapshot

    /// <summary>Id of the rail snapshot section ("RAIL"), shared with the 3D engine (a world is one or the other).</summary>
    public const int RailSectionId = 0x4C494152;

    private static void WriteRailDef(StateWriter w, RailDef d)
    {
        w.NamedSingle(d.CarMass); w.NamedSingle(d.CarSize.X); w.NamedSingle(d.CarSize.Y); w.NamedSingle(d.CarFloorHeight); w.NamedInt32(d.Axles); w.NamedSingle(d.AxleSpan);
        w.NamedSingle(d.WheelRadius); w.NamedSingle(d.WheelsetSpinInertia); w.NamedSingle(d.ContactHertz);
        w.NamedSingle(d.MaxTractionForce); w.NamedSingle(d.MaxPower); w.NamedUInt32(d.DrivenWheelsets); w.NamedSingle(d.MaxBrakeForce);
        w.NamedSingle(d.CouplerReach); w.NamedSingle(d.CouplerHeight); w.NamedSingle(d.CouplerFreeLength); w.NamedSingle(d.CouplerSlack);
        w.NamedSingle(d.BufferStiffness); w.NamedSingle(d.DrawStiffness); w.NamedSingle(d.CouplerDamping); w.NamedSingle(d.CouplerBreakForce);
    }

    private static RailDef ReadRailDef(StateReader r)
    {
        var d = new RailDef
        {
            CarMass = r.ReadSingle(), CarSize = new Vector2F(r.ReadSingle(), r.ReadSingle()), CarFloorHeight = r.ReadSingle(), Axles = r.ReadInt32(), AxleSpan = r.ReadSingle(),
            WheelRadius = r.ReadSingle(), WheelsetSpinInertia = r.ReadSingle(), ContactHertz = r.ReadSingle(),
            MaxTractionForce = r.ReadSingle(), MaxPower = r.ReadSingle(), DrivenWheelsets = r.ReadUInt32(), MaxBrakeForce = r.ReadSingle(),
            CouplerReach = r.ReadSingle(), CouplerHeight = r.ReadSingle(), CouplerFreeLength = r.ReadSingle(), CouplerSlack = r.ReadSingle(),
            BufferStiffness = r.ReadSingle(), DrawStiffness = r.ReadSingle(), CouplerDamping = r.ReadSingle(), CouplerBreakForce = r.ReadSingle(),
        };
        try { d.Validate(); }
        catch (ArgumentException e) { throw r.Error("invalid rail vehicle definition: " + e.Message); }
        return d;
    }

    internal void WriteRail(StateWriter w)
    {
        w.NamedInt32(1);   // section version
        w.DescribeTable("track");
        TrackHandles.Write(w);
        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            w.DescribeEntity("track", i, TrackHandles.GenerationOf(i));
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            TrackDefIo.Write(w, t.Def);
            w.NamedInt32(t.Body); w.NamedInt32(t.BodyGen);
        }

        w.DescribeTable("railvehicle");
        RailHandles.Write(w);
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            w.DescribeEntity("railvehicle", i, RailHandles.GenerationOf(i));
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            WriteRailDef(w, r.Def);
            w.NamedInt32(r.Car); w.NamedInt32(r.CarGen); w.NamedInt32(r.Veh); w.NamedInt32(r.VehGen); w.NamedInt32(r.Track); w.NamedInt32(r.TrackGen);
            w.NamedSingle(r.Throttle); w.NamedSingle(r.Brake);
            for (int e = 0; e < 2; e++) { w.NamedInt32(r.CoupleVeh[e]); w.NamedInt32(r.CoupleGen[e]); w.NamedInt32(r.CoupleEnd[e]); w.NamedByte(r.Broken[e]); w.NamedSingle(r.CouplerForce[e]); }
        }
    }

    private sealed class RailBlock
    {
        public HandleTable Tracks = new(), Vehicles = new();
        public TrackState2D?[] TrackStates = Array.Empty<TrackState2D?>();
        public RailState2D?[] VehicleStates = Array.Empty<RailState2D?>();
    }

    private static RailBlock ReadRail(StateReader r, int bodyCount)
    {
        int version = r.ReadInt32();
        if (version != 1) { throw r.Error("unsupported 2D rail section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var b = new RailBlock { Tracks = HandleTable.Read(r, "track") };
        b.TrackStates = new TrackState2D?[Math.Max(b.Tracks.HighWater, 1)];
        for (int i = 0; i < b.Tracks.HighWater; i++)
        {
            if (!b.Tracks.IsLive(i)) { continue; }
            var t = new TrackState2D { Def = TrackDefIo.Read(r) };
            try { t.Def.Validate(true); }
            catch (ArgumentException e) { throw r.Error("invalid 2D track: " + e.Message); }
            t.Body = r.ReadInt32(); t.BodyGen = r.ReadInt32();
            if (t.Body < -1 || t.Body >= bodyCount) { throw r.Error("invalid track body"); }
            t.Geo = new TrackGeometry(t.Def);
            b.TrackStates[i] = t;
        }

        b.Vehicles = HandleTable.Read(r, "railvehicle");
        b.VehicleStates = new RailState2D?[Math.Max(b.Vehicles.HighWater, 1)];
        for (int i = 0; i < b.Vehicles.HighWater; i++)
        {
            if (!b.Vehicles.IsLive(i)) { continue; }
            var s = new RailState2D { Def = ReadRailDef(r) };
            s.Car = r.ReadInt32(); s.CarGen = r.ReadInt32(); s.Veh = r.ReadInt32(); s.VehGen = r.ReadInt32(); s.Track = r.ReadInt32(); s.TrackGen = r.ReadInt32();
            if (s.Car < 0 || s.Car >= bodyCount || s.CarGen < 1 || s.Veh < 0 || s.VehGen < 1 || s.Track < -1 || s.Track >= b.Tracks.HighWater) { throw r.Error("invalid rail vehicle car, wheels or track"); }
            s.Throttle = r.ReadSingle(); s.Brake = r.ReadSingle();
            if (!(Math.Abs(s.Throttle) <= 1f) || !(s.Brake >= 0f && s.Brake <= 1f)) { throw r.Error("invalid rail vehicle inputs"); }
            for (int e = 0; e < 2; e++)
            {
                s.CoupleVeh[e] = r.ReadInt32(); s.CoupleGen[e] = r.ReadInt32(); s.CoupleEnd[e] = r.ReadInt32(); s.Broken[e] = r.ReadByte(); s.CouplerForce[e] = r.ReadSingle();
                if (s.CoupleVeh[e] < -1 || s.CoupleVeh[e] >= b.Vehicles.HighWater || s.CoupleEnd[e] < 0 || s.CoupleEnd[e] > 1 || s.Broken[e] > 1 || !float.IsFinite(s.CouplerForce[e])) { throw r.Error("invalid coupler"); }
            }

            b.VehicleStates[i] = s;
        }

        return b;
    }

    private void CommitRail(RailBlock? b)
    {
        if (b == null)
        {
            TrackHandles.Reset(); RailHandles.Reset();
            Array.Clear(Tracks); Array.Clear(Rails);
            return;
        }

        TrackHandles = b.Tracks; RailHandles = b.Vehicles;
        Tracks = b.TrackStates; Rails = b.VehicleStates;
    }

    /// <summary>After a restore: every rail vehicle's car and wheeled vehicle exist as the generations it names (the vehicle on the car, with one wheel per wheelset) and its coupler partners are live.</summary>
    internal bool RailConsistent()
    {
        var bodies = Core.Bodies;
        for (int i = 0; i < RailHandles.HighWater; i++)
        {
            if (!RailHandles.IsLive(i)) { continue; }
            var r = Rails[i]!;
            if (!RailLive(r) || Vehicles[r.Veh]!.Body != r.Car || Vehicles[r.Veh]!.Wheels != r.Def.Axles || bodies.TypeOf(r.Car) != BodyType.Dynamic) { return false; }
            if (r.Track >= 0 && !TrackHandles.IsAlive(r.Track, r.TrackGen)) { return false; }
            for (int e = 0; e < 2; e++)
            {
                int o = r.CoupleVeh[e];
                if (o >= 0 && (!RailHandles.IsAlive(o, r.CoupleGen[e]) || Rails[o]!.CoupleVeh[r.CoupleEnd[e]] != i)) { return false; }
            }
        }

        for (int i = 0; i < TrackHandles.HighWater; i++)
        {
            if (!TrackHandles.IsLive(i)) { continue; }
            var t = Tracks[i]!;
            if (t.Body >= 0 && !bodies.Handles.IsAlive(t.Body, t.BodyGen)) { return false; }
        }

        return true;
    }

    private RailSection2D? _railSection;

    internal ISnapshotSection RailSnapshotSection => _railSection ??= new RailSection2D(this);

    /// <summary>The optional snapshot section that carries tracks and rail vehicles. Written only while one exists, so worlds without rail keep their bytes and hashes.</summary>
    private sealed class RailSection2D(Engine2D e) : ISnapshotSection
    {
        public int SectionId => RailSectionId;
        public bool IsOptional => true;
        public bool IsPresent => e.TrackHandles.Count > 0 || e.RailHandles.Count > 0;

        public void Reset()
        {
            if (e.TrackHandles.HighWater == 0 && e.RailHandles.HighWater == 0) { return; }
            e.CommitRail(null);
        }

        public void Write(StateWriter w) => e.WriteRail(w);

        public void Read(StateReader r) => e.CommitRail(ReadRail(r, e.RestoredBodyCount));

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine2D)destination.GetSnapshotSection(Engine2D.Id);
            dst.TrackHandles = new HandleTable(); dst.TrackHandles.CopyFrom(e.TrackHandles);
            dst.RailHandles = new HandleTable(); dst.RailHandles.CopyFrom(e.RailHandles);
            dst.Tracks = new TrackState2D?[e.Tracks.Length];
            for (int i = 0; i < dst.Tracks.Length; i++) { dst.Tracks[i] = e.Tracks[i]?.CloneForFork(); }
            dst.Rails = new RailState2D?[e.Rails.Length];
            for (int i = 0; i < dst.Rails.Length; i++) { dst.Rails[i] = e.Rails[i]?.CloneForFork(); }
            return dst.RailSnapshotSection;
        }
    }
}
