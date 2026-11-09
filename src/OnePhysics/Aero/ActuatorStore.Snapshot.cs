using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Aero;

internal sealed partial class ActuatorStore
{
    int ISnapshotSection.SectionId => Id;

    bool ISnapshotSection.IsOptional => true;

    bool ISnapshotSection.IsPresent => IsPresent;

    /// <summary>Whether the section is written: anything exists, or the air density was changed.</summary>
    public bool IsPresent => HasWork || AirDensity != DefaultAirDensity;

    void ISnapshotSection.Reset() { Clear(); }

    private void Clear()
    {
        AirDensity = DefaultAirDensity;
        ThrusterHandles.Reset(); BoatHandles.Reset(); HoverHandles.Reset();
        Array.Clear(Thrusters); Array.Clear(Boats); Array.Clear(Hovers); Array.Clear(Surfaces);
        _surfaceCount = 0;
        Array.Fill(_boatOfBody, -1); Array.Fill(_hoverOfBody, -1);
        Array.Clear(_submerged);
    }

    private const int SectionVersion = 1;

    void ISnapshotSection.Write(StateWriter w)
    {
        w.NamedInt32(SectionVersion);
        w.NamedSingle(AirDensity);

        w.DescribeTable("thruster");
        ThrusterHandles.Write(w);
        for (int i = 0; i < ThrusterHandles.HighWater; i++)
        {
            w.DescribeEntity("thruster", i, ThrusterHandles.GenerationOf(i));
            if (!ThrusterHandles.IsLive(i)) { continue; }
            var t = Thrusters[i]!;
            w.NamedInt32(t.Body); w.NamedInt32(t.BodyGen);
            w.NamedSingle(t.Throttle); w.NamedSingle(t.Gimbal1); w.NamedSingle(t.Gimbal2); w.NamedSingle(t.Output);
            t.Spec.Write(w);
        }

        w.DescribeTable("boat");
        BoatHandles.Write(w);
        for (int i = 0; i < BoatHandles.HighWater; i++)
        {
            w.DescribeEntity("boat", i, BoatHandles.GenerationOf(i));
            if (!BoatHandles.IsLive(i)) { continue; }
            var b = Boats[i]!;
            w.NamedInt32(b.Body); w.NamedInt32(b.BodyGen);
            w.NamedSingle(b.Throttle); w.NamedSingle(b.Steer); w.NamedSingle(b.Rudder);
            b.Spec.Write(w);
            for (int k = 0; k < b.PropOutput.Length; k++) { w.NamedSingle(b.PropOutput[k]); }
        }

        w.DescribeTable("hovercraft");
        HoverHandles.Write(w);
        for (int i = 0; i < HoverHandles.HighWater; i++)
        {
            w.DescribeEntity("hovercraft", i, HoverHandles.GenerationOf(i));
            if (!HoverHandles.IsLive(i)) { continue; }
            var h = Hovers[i]!;
            w.NamedInt32(h.Body); w.NamedInt32(h.BodyGen);
            w.NamedSingle(h.Lift); w.NamedSingle(h.Throttle); w.NamedSingle(h.Steer);
            h.Spec.Write(w);
            for (int k = 0; k < h.ThrusterOutput.Length; k++) { w.NamedSingle(h.ThrusterOutput[k]); }
        }

        // Surfaces, by ascending shape slot.
        w.DescribeEntity("world", -1);
        w.NamedInt32(_surfaceCount);
        for (int s = 0; s < Surfaces.Length; s++)
        {
            var st = Surfaces[s];
            if (st == null) { continue; }
            w.DescribeEntity("aero", s, st.ShapeGen);
            w.NamedInt32(s); w.NamedInt32(st.ShapeGen);
            st.Spec.Write(w);
        }
    }

    void ISnapshotSection.Read(StateReader r)
    {
        bool planar = _host.Planar;
        int version = r.ReadInt32();
        if (version != SectionVersion) { throw r.Error("unsupported actuator section version " + version.ToString(CultureInfo.InvariantCulture)); }
        float air = r.ReadSingle();
        if (!(air >= 0f && air <= 1e4f)) { throw r.Error("invalid air density " + air.ToString("R", CultureInfo.InvariantCulture)); }

        var th = HandleTable.Read(r, "thruster");
        var thrusters = new ThrusterState?[Math.Max(th.HighWater, 1)];
        for (int i = 0; i < th.HighWater; i++)
        {
            if (!th.IsLive(i)) { continue; }
            var t = new ThrusterState { Body = r.ReadInt32(), BodyGen = r.ReadInt32() };
            CheckBody(r, t.Body, t.BodyGen);
            t.Throttle = r.ReadSingle(); t.Gimbal1 = r.ReadSingle(); t.Gimbal2 = r.ReadSingle(); t.Output = r.ReadSingle();
            t.Spec = ThrusterSpec.Read(r, planar);
            CheckThrusterState(r, t.Throttle, t.Gimbal1, t.Gimbal2, t.Output, t.Spec);
            thrusters[i] = t;
        }

        var bh = HandleTable.Read(r, "boat");
        var boats = new BoatState?[Math.Max(bh.HighWater, 1)];
        for (int i = 0; i < bh.HighWater; i++)
        {
            if (!bh.IsLive(i)) { continue; }
            var b = new BoatState { Body = r.ReadInt32(), BodyGen = r.ReadInt32() };
            CheckBody(r, b.Body, b.BodyGen);
            b.Throttle = r.ReadSingle(); b.Steer = r.ReadSingle(); b.Rudder = r.ReadSingle();
            b.Spec = BoatSpec.Read(r, planar);
            b.PropOutput = ReadOutputs(r, b.Spec.Propellers.Length);
            b.FoilForce = new Vector3F[b.Spec.Foils.Length]; b.FoilChord = new Vector3F[b.Spec.Foils.Length]; b.FoilNormal = new Vector3F[b.Spec.Foils.Length]; b.PropForce = new Vector3F[b.Spec.Propellers.Length];
            if (!(Math.Abs(b.Throttle) <= 1f) || !(Math.Abs(b.Steer) <= 1f) || !(Math.Abs(b.Rudder) <= 1.2f)) { throw r.Error("invalid boat inputs"); }
            boats[i] = b;
        }

        var hh = HandleTable.Read(r, "hovercraft");
        var hovers = new HoverState?[Math.Max(hh.HighWater, 1)];
        for (int i = 0; i < hh.HighWater; i++)
        {
            if (!hh.IsLive(i)) { continue; }
            var h = new HoverState { Body = r.ReadInt32(), BodyGen = r.ReadInt32() };
            CheckBody(r, h.Body, h.BodyGen);
            h.Lift = r.ReadSingle(); h.Throttle = r.ReadSingle(); h.Steer = r.ReadSingle();
            h.Spec = HoverSpec.Read(r, planar);
            h.ThrusterOutput = ReadOutputs(r, h.Spec.Thrusters.Length);
            h.ThrusterForce = new Vector3F[h.Spec.Thrusters.Length];
            if (!(h.Lift >= 0f && h.Lift <= 1f) || !(Math.Abs(h.Throttle) <= 1f) || !(Math.Abs(h.Steer) <= 1f)) { throw r.Error("invalid hovercraft inputs"); }
            h.FanHeight = new float[h.Spec.Fans.Length]; h.FanForce = new float[h.Spec.Fans.Length];
            for (int k = 0; k < h.FanHeight.Length; k++) { h.FanHeight[k] = h.Spec.Fans[k].Range; }
            hovers[i] = h;
        }

        int n = r.ReadCount(8 + SurfaceSpec.Bytes, "aero surfaces");
        var surfaces = new SurfaceState?[8];
        int last = -1;
        for (int k = 0; k < n; k++)
        {
            int slot = r.ReadInt32(), gen = r.ReadInt32();
            if (slot <= last || slot > 1 << 26 || gen < 1) { throw r.Error("aero surfaces out of order or for an invalid shape"); }
            last = slot;
            if (slot >= surfaces.Length) { Array.Resize(ref surfaces, Math.Max(slot + 1, surfaces.Length * 2)); }
            surfaces[slot] = new SurfaceState { ShapeGen = gen, Spec = SurfaceSpec.Read(r, planar, false) };
        }

        // Commit.
        AirDensity = air;
        ThrusterHandles = th; Thrusters = thrusters;
        BoatHandles = bh; Boats = boats;
        HoverHandles = hh; Hovers = hovers;
        Surfaces = surfaces; _surfaceCount = n;
        Array.Fill(_boatOfBody, -1); Array.Fill(_hoverOfBody, -1);
        for (int i = 0; i < bh.HighWater; i++) { if (bh.IsLive(i)) { Index(ref _boatOfBody, boats[i]!.Body, i); } }
        for (int i = 0; i < hh.HighWater; i++) { if (hh.IsLive(i)) { Index(ref _hoverOfBody, hovers[i]!.Body, i); } }
    }

    private static void CheckBody(StateReader r, int body, int gen)
    {
        if (body < 0 || body > 1 << 26 || gen < 1) { throw r.Error("invalid body reference in an actuator"); }
    }

    private static void CheckThrusterState(StateReader r, float throttle, float g1, float g2, float output, ThrusterSpec spec)
    {
        if (!(Math.Abs(throttle) <= 1e6f) || !(Math.Abs(g1) <= 1e6f) || !(Math.Abs(g2) <= 1e6f) || !(output >= -1f && output <= 1f)) { throw r.Error("invalid thruster state"); }
    }

    private static float[] ReadOutputs(StateReader r, int n)
    {
        var a = new float[n];
        for (int i = 0; i < n; i++)
        {
            a[i] = r.ReadSingle();
            if (!(a[i] >= -1f && a[i] <= 1f)) { throw r.Error("invalid thruster output"); }
        }

        return a;
    }

    // ------------------------------------------------------------------ fork

    ISnapshotSection ISnapshotSection.Fork(WorldCore destination)
    {
        var dst = ((IActuatorHost)destination.GetSnapshotSection(_host.EngineSectionId)).Actuators;
        dst.CopyFrom(this);
        return dst;
    }

    private void CopyFrom(ActuatorStore src)
    {
        AirDensity = src.AirDensity;
        ThrusterHandles = new HandleTable(); ThrusterHandles.CopyFrom(src.ThrusterHandles);
        BoatHandles = new HandleTable(); BoatHandles.CopyFrom(src.BoatHandles);
        HoverHandles = new HandleTable(); HoverHandles.CopyFrom(src.HoverHandles);
        Thrusters = new ThrusterState?[src.Thrusters.Length];
        for (int i = 0; i < Thrusters.Length; i++)
        {
            var t = src.Thrusters[i];
            if (t != null) { Thrusters[i] = new ThrusterState { Body = t.Body, BodyGen = t.BodyGen, Spec = t.Spec.Clone(), Throttle = t.Throttle, Gimbal1 = t.Gimbal1, Gimbal2 = t.Gimbal2, Output = t.Output, LastForce = t.LastForce }; }
        }

        Boats = new BoatState?[src.Boats.Length];
        for (int i = 0; i < Boats.Length; i++)
        {
            var b = src.Boats[i];
            if (b != null) { Boats[i] = new BoatState { Body = b.Body, BodyGen = b.BodyGen, Spec = b.Spec.Clone(), Throttle = b.Throttle, Steer = b.Steer, Rudder = b.Rudder, PropOutput = (float[])b.PropOutput.Clone(), LastHullForce = b.LastHullForce, FoilForce = (Vector3F[])b.FoilForce.Clone(), FoilChord = (Vector3F[])b.FoilChord.Clone(), FoilNormal = (Vector3F[])b.FoilNormal.Clone(), PropForce = (Vector3F[])b.PropForce.Clone() }; }
        }

        Hovers = new HoverState?[src.Hovers.Length];
        for (int i = 0; i < Hovers.Length; i++)
        {
            var h = src.Hovers[i];
            if (h != null)
            {
                Hovers[i] = new HoverState
                {
                    Body = h.Body, BodyGen = h.BodyGen, Spec = h.Spec.Clone(), Lift = h.Lift, Throttle = h.Throttle, Steer = h.Steer,
                    ThrusterOutput = (float[])h.ThrusterOutput.Clone(), ThrusterForce = (Vector3F[])h.ThrusterForce.Clone(), FanHeight = (float[])h.FanHeight.Clone(), FanForce = (float[])h.FanForce.Clone(),
                };
            }
        }

        Surfaces = new SurfaceState?[src.Surfaces.Length];
        for (int i = 0; i < Surfaces.Length; i++)
        {
            var st = src.Surfaces[i];
            if (st != null) { Surfaces[i] = new SurfaceState { Spec = st.Spec.Clone(), ShapeGen = st.ShapeGen, AutoArea = st.AutoArea, LastForce = st.LastForce }; }
        }

        _surfaceCount = src._surfaceCount;
        _boatOfBody = (int[])src._boatOfBody.Clone();
        _hoverOfBody = (int[])src._hoverOfBody.Clone();
        _submerged = (float[])src._submerged.Clone();
    }
}
