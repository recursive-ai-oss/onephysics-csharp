using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The saved state of one bicycle: its two bodies, its two one-wheel vehicles, the definition and the rider's inputs.</summary>
internal sealed class BikeState
{
    public int Frame, FrameGen, Fork, ForkGen;
    public int RearVeh, RearVehGen, FrontVeh, FrontVehGen;
    public BicycleDef Def = new();
    public float Steer, Pedal, FrontBrake, RearBrake, Lean;

    public BikeState CloneForFork()
    {
        var c = (BikeState)MemberwiseClone();
        c.Def = Def.Clone();
        return c;
    }

    /// <summary>The steering axis in the rear frame's axes: up and back by the head angle.</summary>
    public Vector3F SteerAxisLocal => new(0f, Math32.Cos(Def.HeadAngle), -Math32.Sin(Def.HeadAngle));
}

internal sealed partial class Engine3D
{
    /// <summary>Slots of the bicycles (handles with generations, lowest free slot first).</summary>
    public HandleTable BikeHandles = new();

    public BikeState?[] Bikes = new BikeState?[2];

    public bool HasBicycles => BikeHandles.Count > 0;

    public int CreateBicycle(BikeState s)
    {
        int b = BikeHandles.Allocate();
        if (b >= Bikes.Length) { Array.Resize(ref Bikes, Math.Max(b + 1, Bikes.Length * 2)); }
        Bikes[b] = s;
        return b;
    }

    /// <summary>Category bit of a bicycle's collision shapes; a bicycle's wheel casts ignore shapes that carry it.</summary>
    internal const uint BikeCategory = 1u << 29;

    // True when the body is a bicycle's frame or fork: its first shape carries the bicycle category.
    private bool IsBikeBody(int body) => ShapeHead[body] >= 0 && (Category[ShapeHead[body]] & BikeCategory) != 0;

    // The other body of a bicycle whose first body is being destroyed; it goes once the first body's removal has finished (it cannot be removed while the first is half gone).
    private BodyHandle _bikeOrphan;

    private void DestroyBikeOrphan()
    {
        var h = _bikeOrphan;
        if (h.IsNull) { return; }
        _bikeOrphan = default;
        if (Core.Bodies.Handles.IsAlive(h.Index, h.Generation)) { DestroyBody(h); }
    }

    public void DestroyBicycle(int i)
    {
        Bikes[i] = null;
        BikeHandles.Free(i);
    }

    /// <summary>Called when a body is destroyed: a bicycle that has it as its frame or fork goes, and so does the other body (its wheels' vehicles go with their bodies).</summary>
    public void DropBicycleOfBody(int body)
    {
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            if (!BikeHandles.IsLive(i)) { continue; }
            var s = Bikes[i]!;
            if (s.Frame != body && s.Fork != body) { continue; }
            DestroyBicycle(i);
            int other = s.Frame == body ? s.Fork : s.Frame, otherGen = s.Frame == body ? s.ForkGen : s.FrameGen;
            if (Core.Bodies.Handles.IsAlive(other, otherGen)) { _bikeOrphan = new BodyHandle(Core.Id, other, otherGen); }
        }
    }

    private bool BikeLive(BikeState s) =>
        Core.Bodies.Handles.IsAlive(s.Frame, s.FrameGen) && Core.Bodies.Handles.IsAlive(s.Fork, s.ForkGen)
        && VehHandles.IsAlive(s.RearVeh, s.RearVehGen) && VehHandles.IsAlive(s.FrontVeh, s.FrontVehGen);

    /// <summary>
    /// Before the vehicles run, once per step, in slot order: hands the rider's pedal torque and brake levers to the two wheels, applies the steering and lean torques, and adds the gyroscopic
    /// coupling of each spinning wheel to its body. The wheels' own forces follow in the vehicle stage.
    /// </summary>
    public void StepBicycles(float h)
    {
        if (BikeHandles.Count == 0) { return; }
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            if (BikeHandles.IsLive(i)) { StepBike(Bikes[i]!, h); }
        }
    }

    private void StepBike(BikeState s, float h)
    {
        if (!BikeLive(s)) { return; }
        var bodies = Core.Bodies;
        var def = s.Def;
        int fr = s.Frame, fk = s.Fork;
        bool active = s.Steer != 0f || s.Pedal != 0f || s.FrontBrake > 0f || s.RearBrake > 0f || s.Lean != 0f;
        if (!bodies.IsAwake(fr) || !bodies.IsAwake(fk))
        {
            if (!active) { return; }
            WakeBodyIsland(fr); WakeBodyIsland(fk);
        }
        else if (active) { bodies.SleepTime[fr] = 0f; bodies.SleepTime[fk] = 0f; }

        Core.KeepJointSchedule(fr);
        var rear = Vehicles[s.RearVeh]!; var front = Vehicles[s.FrontVeh]!;
        rear.Forward = 0f; rear.Right = 0f; rear.Handbrake = 0f; rear.Brake = s.RearBrake;
        front.Forward = 0f; front.Right = 0f; front.Handbrake = 0f; front.Brake = s.FrontBrake;
        rear.ExtraTorque[0] = s.Pedal * def.MaxPedalTorque;

        RefreshWorldInertia(fr); RefreshWorldInertia(fk);
        var qFrame = RotationOfBody(fr);
        // Steering: equal and opposite torques about the steering axis.
        if (s.Steer != 0f)
        {
            var axis = qFrame.Rotate(s.SteerAxisLocal);
            var j = axis * (h * s.Steer * def.MaxSteerTorque);
            AddAngularImpulse(fk, j);
            AddAngularImpulse(fr, j * -1f);
        }

        // The rider's lean: a roll torque on the frame, positive to the right (a positive rotation about the heading raises the left side).
        if (s.Lean != 0f)
        {
            var heading = qFrame.Rotate(new Vector3F(0f, 0f, 1f));
            AddAngularImpulse(fr, heading * (-h * s.Lean * def.MaxLeanTorque));
        }

        // Gyroscopic coupling: a rotor of angular momentum H on a body turning at w exerts -w x H on it (the rotor's own spin-up is the tyre's business).
        Gyro(fr, rear.Omega[0], def.RearWheel.SpinInertia, h);
        Gyro(fk, front.Omega[0], def.FrontWheel.SpinInertia, h);
    }

    private void Gyro(int body, float spin, float spinInertia, float h)
    {
        var q = RotationOfBody(body);
        var axle = q.Rotate(new Vector3F(1f, 0f, 0f));
        var hv = axle * (spinInertia * spin);
        var w = Core.Bodies.AngularVelocityOf(body);
        var wv = new Vector3F(w[0], w[1], w[2]);
        AddAngularImpulse(body, Vector3F.Cross(wv, hv) * -h);
    }

    private void AddAngularImpulse(int body, Vector3F j)
    {
        var bodies = Core.Bodies;
        if (!(bodies.InverseMass[body] > 0f)) { return; }
        var dw = ApplyWorldInertia(body, j);
        var w = bodies.AngularVelocityOf(body);
        w[0] += dw.X; w[1] += dw.Y; w[2] += dw.Z;
    }

    /// <summary>Roll of the frame from the vertical, radians, positive when the bicycle leans to its right.</summary>
    public float BikeLean(BikeState s)
    {
        var q = RotationOfBody(s.Frame);
        var up = q.Rotate(new Vector3F(0f, 1f, 0f));
        var f = q.Rotate(new Vector3F(0f, 0f, 1f));
        float l = float.Sqrt(f.X * f.X + f.Z * f.Z);
        if (!(l > 1e-6f)) { return 0f; }
        float fx = f.X / l, fz = f.Z / l;
        return Math32.Atan2(up.X * fz - up.Z * fx, up.Y);
    }

    /// <summary>Steering angle about the steering axis, radians, positive to the right.</summary>
    public float BikeSteerAngle(BikeState s)
    {
        var qf = RotationOfBody(s.Frame); var qk = RotationOfBody(s.Fork);
        var rel = qf.Conjugated() * qk;
        var a = s.SteerAxisLocal;
        float sinHalf = rel.X * a.X + rel.Y * a.Y + rel.Z * a.Z;
        float angle = 2f * Math32.Atan2(sinHalf, rel.W);
        return angle > 3.1415927f ? angle - 6.2831855f : angle < -3.1415927f ? angle + 6.2831855f : angle;
    }

    // ------------------------------------------------------------------ snapshot

    /// <summary>Id of the bicycle snapshot section ("BIKE").</summary>
    public const int BicycleSectionId = 0x454B4942;

    private static void WritePart(StateWriter w, BicyclePart p)
    {
        w.NamedSingle(p.Mass); w.NamedSingle(p.CenterX); w.NamedSingle(p.CenterZ); w.NamedSingle(p.Ixx); w.NamedSingle(p.Ixz); w.NamedSingle(p.Iyy); w.NamedSingle(p.Izz);
    }

    private static BicyclePart ReadPart(StateReader r) =>
        new() { Mass = r.ReadSingle(), CenterX = r.ReadSingle(), CenterZ = r.ReadSingle(), Ixx = r.ReadSingle(), Ixz = r.ReadSingle(), Iyy = r.ReadSingle(), Izz = r.ReadSingle() };

    internal void WriteBicycles(StateWriter w)
    {
        w.NamedInt32(1);   // section version
        w.DescribeTable("bicycle");
        BikeHandles.Write(w);
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            w.DescribeEntity("bicycle", i, BikeHandles.GenerationOf(i));
            if (!BikeHandles.IsLive(i)) { continue; }
            var s = Bikes[i]!;
            w.NamedInt32(s.Frame); w.NamedInt32(s.FrameGen); w.NamedInt32(s.Fork); w.NamedInt32(s.ForkGen);
            w.NamedInt32(s.RearVeh); w.NamedInt32(s.RearVehGen); w.NamedInt32(s.FrontVeh); w.NamedInt32(s.FrontVehGen);
            var d = s.Def;
            w.NamedSingle(d.WheelBase); w.NamedSingle(d.Trail); w.NamedSingle(d.HeadAngle);
            w.NamedSingle(d.RearWheel.Radius); w.NamedSingle(d.RearWheel.SpinInertia); w.NamedSingle(d.FrontWheel.Radius); w.NamedSingle(d.FrontWheel.SpinInertia);
            WritePart(w, d.RearFrame); WritePart(w, d.Front);
            w.NamedSingle(d.TyreFriction); w.NamedSingle(d.TyreStiffness); w.NamedSingle(d.TyreHertz);
            w.NamedSingle(d.MaxSteerTorque); w.NamedSingle(d.MaxPedalTorque); w.NamedSingle(d.MaxFrontBrakeTorque); w.NamedSingle(d.MaxRearBrakeTorque); w.NamedSingle(d.MaxLeanTorque); w.NamedSingle(d.FallAngle);
            w.NamedSingle(s.Steer); w.NamedSingle(s.Pedal); w.NamedSingle(s.FrontBrake); w.NamedSingle(s.RearBrake); w.NamedSingle(s.Lean);
        }
    }

    private sealed class BikeBlock
    {
        public HandleTable Handles = new();
        public BikeState?[] States = Array.Empty<BikeState?>();
    }

    private static BikeBlock ReadBicycles(StateReader r, int bodyCount)
    {
        int version = r.ReadInt32();
        if (version != 1) { throw r.Error("unsupported bicycle section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var b = new BikeBlock { Handles = HandleTable.Read(r, "bicycle") };
        if (b.Handles.Count == 0) { throw r.Error("a bicycle section holding no bicycles"); }
        b.States = new BikeState?[Math.Max(b.Handles.HighWater, 1)];
        for (int i = 0; i < b.Handles.HighWater; i++)
        {
            if (!b.Handles.IsLive(i)) { continue; }
            var s = new BikeState { Frame = r.ReadInt32(), FrameGen = r.ReadInt32(), Fork = r.ReadInt32(), ForkGen = r.ReadInt32(), RearVeh = r.ReadInt32(), RearVehGen = r.ReadInt32(), FrontVeh = r.ReadInt32(), FrontVehGen = r.ReadInt32() };
            if (s.Frame < 0 || s.Frame >= bodyCount || s.Fork < 0 || s.Fork >= bodyCount || s.FrameGen < 1 || s.ForkGen < 1 || s.Frame == s.Fork) { throw r.Error("invalid bicycle bodies"); }
            if (s.RearVeh < 0 || s.FrontVeh < 0 || s.RearVehGen < 1 || s.FrontVehGen < 1 || s.RearVeh == s.FrontVeh) { throw r.Error("invalid bicycle wheels"); }
            var d = new BicycleDef { WheelBase = r.ReadSingle(), Trail = r.ReadSingle(), HeadAngle = r.ReadSingle() };
            d.RearWheel = new BicycleWheel { Radius = r.ReadSingle(), SpinInertia = r.ReadSingle() };
            d.FrontWheel = new BicycleWheel { Radius = r.ReadSingle(), SpinInertia = r.ReadSingle() };
            d.RearFrame = ReadPart(r); d.Front = ReadPart(r);
            d.TyreFriction = r.ReadSingle(); d.TyreStiffness = r.ReadSingle(); d.TyreHertz = r.ReadSingle();
            d.MaxSteerTorque = r.ReadSingle(); d.MaxPedalTorque = r.ReadSingle(); d.MaxFrontBrakeTorque = r.ReadSingle(); d.MaxRearBrakeTorque = r.ReadSingle(); d.MaxLeanTorque = r.ReadSingle(); d.FallAngle = r.ReadSingle();
            try { d.Validate(); }
            catch (ArgumentException e) { throw r.Error("invalid bicycle definition: " + e.Message); }
            s.Def = d;
            s.Steer = r.ReadSingle(); s.Pedal = r.ReadSingle(); s.FrontBrake = r.ReadSingle(); s.RearBrake = r.ReadSingle(); s.Lean = r.ReadSingle();
            if (!(Math.Abs(s.Steer) <= 1f) || !(Math.Abs(s.Pedal) <= 1f) || !(Math.Abs(s.Lean) <= 1f) || !(s.FrontBrake >= 0f && s.FrontBrake <= 1f) || !(s.RearBrake >= 0f && s.RearBrake <= 1f)) { throw r.Error("invalid bicycle inputs"); }
            b.States[i] = s;
        }

        return b;
    }

    private void CommitBicycles(BikeBlock? b)
    {
        if (b == null)
        {
            BikeHandles.Reset();
            Array.Clear(Bikes);
            return;
        }

        BikeHandles = b.Handles;
        Bikes = b.States;
    }

    /// <summary>After a restore: every bicycle's bodies and wheels must exist as the generations it names, the wheels sitting on those bodies.</summary>
    internal bool BicyclesConsistent()
    {
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            if (!BikeHandles.IsLive(i)) { continue; }
            var s = Bikes[i]!;
            if (!BikeLive(s)) { return false; }
            if (Vehicles[s.RearVeh]!.Body != s.Frame || Vehicles[s.FrontVeh]!.Body != s.Fork) { return false; }
            if (Vehicles[s.RearVeh]!.Wheels != 1 || Vehicles[s.FrontVeh]!.Wheels != 1) { return false; }
            if (Core.Bodies.TypeOf(s.Frame) != BodyType.Dynamic || Core.Bodies.TypeOf(s.Fork) != BodyType.Dynamic) { return false; }
        }

        return true;
    }

    private BikeSection? _bikeSection;

    internal ISnapshotSection BicycleSnapshotSection => _bikeSection ??= new BikeSection(this);

    /// <summary>The optional snapshot section that carries bicycles. Written only while one exists, so worlds without bicycles keep their bytes and hashes.</summary>
    private sealed class BikeSection(Engine3D e) : ISnapshotSection
    {
        public int SectionId => BicycleSectionId;
        public bool IsOptional => true;
        public bool IsPresent => e.BikeHandles.Count > 0;

        public void Reset()
        {
            if (e.BikeHandles.HighWater == 0) { return; }
            e.CommitBicycles(null);
        }

        public void Write(StateWriter w) => e.WriteBicycles(w);

        public void Read(StateReader r) => e.CommitBicycles(ReadBicycles(r, e.RestoredBodyCount));

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine3D)destination.GetSnapshotSection(Engine3D.Id);
            dst.BikeHandles = new HandleTable();
            dst.BikeHandles.CopyFrom(e.BikeHandles);
            dst.Bikes = new BikeState?[e.Bikes.Length];
            for (int i = 0; i < dst.Bikes.Length; i++) { dst.Bikes[i] = e.Bikes[i]?.CloneForFork(); }
            return dst.BicycleSnapshotSection;
        }
    }
}
