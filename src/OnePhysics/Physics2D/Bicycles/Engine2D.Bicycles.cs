using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>The saved state of one side-view bicycle: its frame body, its vehicle (front wheel 0, rear wheel 1), the definition and the rider's inputs.</summary>
internal sealed class BikeState2D
{
    public int Frame, FrameGen, Veh, VehGen;
    public BicycleDef Def = new();
    public float Pedal, FrontBrake, RearBrake, Lean;

    public BikeState2D CloneForFork()
    {
        var c = (BikeState2D)MemberwiseClone();
        c.Def = Def.Clone();
        return c;
    }
}

internal sealed partial class Engine2D
{
    /// <summary>Slots of the bicycles (handles with generations, lowest free slot first).</summary>
    public HandleTable BikeHandles = new();

    public BikeState2D?[] Bikes = new BikeState2D?[2];

    public int CreateBicycle(BikeState2D s)
    {
        int b = BikeHandles.Allocate();
        if (b >= Bikes.Length) { Array.Resize(ref Bikes, Math.Max(b + 1, Bikes.Length * 2)); }
        Bikes[b] = s;
        return b;
    }

    public void DestroyBicycle(int i)
    {
        Bikes[i] = null;
        BikeHandles.Free(i);
    }

    /// <summary>Called when a body is destroyed: a bicycle on it goes (its vehicle goes with the body).</summary>
    public void DropBicycleOfBody(int body)
    {
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            if (BikeHandles.IsLive(i) && Bikes[i]!.Frame == body) { DestroyBicycle(i); }
        }
    }

    private bool BikeLive(BikeState2D s) => Core.Bodies.Handles.IsAlive(s.Frame, s.FrameGen) && VehHandles.IsAlive(s.Veh, s.VehGen);

    /// <summary>Before the vehicles run, once per step, in slot order: hands the rider's pedal torque, brake levers and weight shift to the wheels and the frame.</summary>
    public void StepBicycles()
    {
        if (BikeHandles.Count == 0) { return; }
        var bodies = Core.Bodies;
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            if (!BikeHandles.IsLive(i)) { continue; }
            var s = Bikes[i]!;
            if (!BikeLive(s)) { continue; }
            bool active = s.Pedal != 0f || s.FrontBrake > 0f || s.RearBrake > 0f || s.Lean != 0f;
            if (!bodies.IsAwake(s.Frame))
            {
                if (!active) { continue; }
                WakeBodyIsland(s.Frame);
            }
            else if (active) { bodies.SleepTime[s.Frame] = 0f; }

            var v = Vehicles[s.Veh]!;
            v.Forward = 0f; v.Brake = 0f; v.Handbrake = 0f; v.Pitch = s.Lean;
            v.WheelBrake[0] = s.FrontBrake; v.WheelBrake[1] = s.RearBrake;
            v.ExtraTorque[1] = s.Pedal * s.Def.MaxPedalTorque;
        }
    }

    /// <summary>Pitch of the frame, radians, positive nose up.</summary>
    public float BikePitch(BikeState2D s)
    {
        var r = Core.Bodies.RotationOf(s.Frame);
        return Math32.Atan2(r[1], r[0]);
    }

    // ------------------------------------------------------------------ snapshot

    /// <summary>Id of the bicycle snapshot section ("BIKE"), shared with the 3D engine (a world is one or the other).</summary>
    public const int BicycleSectionId = 0x454B4942;

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
            w.NamedInt32(s.Frame); w.NamedInt32(s.FrameGen); w.NamedInt32(s.Veh); w.NamedInt32(s.VehGen);
            var d = s.Def;
            w.NamedSingle(d.WheelBase); w.NamedSingle(d.RearRadius); w.NamedSingle(d.FrontRadius); w.NamedSingle(d.Mass); w.NamedSingle(d.CenterX); w.NamedSingle(d.CenterHeight);
            w.NamedSingle(d.PitchInertia); w.NamedSingle(d.RearSpinInertia); w.NamedSingle(d.FrontSpinInertia); w.NamedSingle(d.TyreFriction); w.NamedSingle(d.TyreHertz);
            w.NamedSingle(d.MaxPedalTorque); w.NamedSingle(d.MaxFrontBrakeTorque); w.NamedSingle(d.MaxRearBrakeTorque); w.NamedSingle(d.MaxLeanTorque); w.NamedSingle(d.FlipAngle);
            w.NamedSingle(s.Pedal); w.NamedSingle(s.FrontBrake); w.NamedSingle(s.RearBrake); w.NamedSingle(s.Lean);
        }
    }

    private sealed class BikeBlock
    {
        public HandleTable Handles = new();
        public BikeState2D?[] States = Array.Empty<BikeState2D?>();
    }

    private static BikeBlock ReadBicycles(StateReader r, int bodyCount)
    {
        int version = r.ReadInt32();
        if (version != 1) { throw r.Error("unsupported 2D bicycle section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var b = new BikeBlock { Handles = HandleTable.Read(r, "bicycle") };
        if (b.Handles.Count == 0) { throw r.Error("a bicycle section holding no bicycles"); }
        b.States = new BikeState2D?[Math.Max(b.Handles.HighWater, 1)];
        for (int i = 0; i < b.Handles.HighWater; i++)
        {
            if (!b.Handles.IsLive(i)) { continue; }
            var s = new BikeState2D { Frame = r.ReadInt32(), FrameGen = r.ReadInt32(), Veh = r.ReadInt32(), VehGen = r.ReadInt32() };
            if (s.Frame < 0 || s.Frame >= bodyCount || s.FrameGen < 1 || s.Veh < 0 || s.VehGen < 1) { throw r.Error("invalid bicycle frame or wheels"); }
            var d = new BicycleDef
            {
                WheelBase = r.ReadSingle(), RearRadius = r.ReadSingle(), FrontRadius = r.ReadSingle(), Mass = r.ReadSingle(), CenterX = r.ReadSingle(), CenterHeight = r.ReadSingle(),
                PitchInertia = r.ReadSingle(), RearSpinInertia = r.ReadSingle(), FrontSpinInertia = r.ReadSingle(), TyreFriction = r.ReadSingle(), TyreHertz = r.ReadSingle(),
                MaxPedalTorque = r.ReadSingle(), MaxFrontBrakeTorque = r.ReadSingle(), MaxRearBrakeTorque = r.ReadSingle(), MaxLeanTorque = r.ReadSingle(), FlipAngle = r.ReadSingle(),
            };
            try { d.Validate(); }
            catch (ArgumentException e) { throw r.Error("invalid bicycle definition: " + e.Message); }
            s.Def = d;
            s.Pedal = r.ReadSingle(); s.FrontBrake = r.ReadSingle(); s.RearBrake = r.ReadSingle(); s.Lean = r.ReadSingle();
            if (!(Math.Abs(s.Pedal) <= 1f) || !(Math.Abs(s.Lean) <= 1f) || !(s.FrontBrake >= 0f && s.FrontBrake <= 1f) || !(s.RearBrake >= 0f && s.RearBrake <= 1f)) { throw r.Error("invalid bicycle inputs"); }
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

    /// <summary>After a restore: every bicycle's frame and vehicle must exist as the generations it names, the vehicle sitting on the frame with two wheels.</summary>
    internal bool BicyclesConsistent()
    {
        for (int i = 0; i < BikeHandles.HighWater; i++)
        {
            if (!BikeHandles.IsLive(i)) { continue; }
            var s = Bikes[i]!;
            if (!BikeLive(s) || Vehicles[s.Veh]!.Body != s.Frame || Vehicles[s.Veh]!.Wheels != 2 || Core.Bodies.TypeOf(s.Frame) != BodyType.Dynamic) { return false; }
        }

        return true;
    }

    private BikeSection? _bikeSection;

    internal ISnapshotSection BicycleSnapshotSection => _bikeSection ??= new BikeSection(this);

    /// <summary>The optional snapshot section that carries bicycles. Written only while one exists, so worlds without bicycles keep their bytes and hashes.</summary>
    private sealed class BikeSection(Engine2D e) : ISnapshotSection
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
            var dst = (Engine2D)destination.GetSnapshotSection(Engine2D.Id);
            dst.BikeHandles = new HandleTable();
            dst.BikeHandles.CopyFrom(e.BikeHandles);
            dst.Bikes = new BikeState2D?[e.Bikes.Length];
            for (int i = 0; i < dst.Bikes.Length; i++) { dst.Bikes[i] = e.Bikes[i]?.CloneForFork(); }
            return dst.BicycleSnapshotSection;
        }
    }
}
