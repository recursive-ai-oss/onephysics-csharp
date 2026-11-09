using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Aero;

internal sealed partial class ActuatorStore
{
    /// <summary>
    /// Emits the surfaces and the last step's forces as plain data, in a fixed order: shape surfaces by shape slot, thrusters by slot, boats by slot (hull, foils, propellers) and hovercraft by slot (fans, thrusters).
    /// Points are taken at the bodies' present poses.
    /// </summary>
    public void AddDebug(DebugGeometry g)
    {
        if (!HasWork) { return; }
        var core = _host.Core;
        var bodies = core.Bodies;
        int hw = Math.Min(Surfaces.Length, core.Shapes.HighWater);
        for (int s = 0; s < hw; s++)
        {
            var st = Surfaces[s];
            if (st == null || !core.Shapes.Handles.IsAlive(s, st.ShapeGen)) { continue; }
            int b = core.Shapes.Owner[s];
            if (!bodies.IsLive(b) || !bodies.IsDynamic(b)) { continue; }
            var spec = st.Spec;
            var c = Ctx(b);
            var centre = V3.Add(_host.ShapeOffset(s), _host.ShapeToBody(s, spec.Center));
            var at = At(c, Arm(c, centre));
            bool lifting = spec.Kind == AeroSurfaceKind.Lifting;
            var chord = lifting ? _host.Rotate(b, _host.ShapeToBody(s, spec.Chord)) : default;
            var normal = lifting ? _host.Rotate(b, _host.ShapeToBody(s, spec.Normal)) : default;
            float area = spec.Area > 0f ? spec.Area : float.IsNaN(st.AutoArea) ? 0f : st.AutoArea;
            g.Add(new DebugAeroSurface(core.HandleOfBody(b), DebugAeroKind.Surface, s, at, chord, normal, area, lifting));
            Emit(g, core.HandleOfBody(b), DebugAeroKind.Surface, s, at, st.LastForce);
        }

        for (int i = 0; i < ThrusterHandles.HighWater; i++)
        {
            if (!ThrusterHandles.IsLive(i)) { continue; }
            var t = Thrusters[i]!;
            if (!Live(t.Body, t.BodyGen)) { continue; }
            var c = Ctx(t.Body);
            Emit(g, core.HandleOfBody(t.Body), DebugAeroKind.Thruster, i, At(c, Arm(c, t.Spec.Position)), t.LastForce);
        }

        for (int i = 0; i < BoatHandles.HighWater; i++)
        {
            if (!BoatHandles.IsLive(i)) { continue; }
            var s = Boats[i]!;
            if (!Live(s.Body, s.BodyGen)) { continue; }
            var h = core.HandleOfBody(s.Body);
            var c = Ctx(s.Body);
            Emit(g, h, DebugAeroKind.Hull, i, c.Com, s.LastHullForce);
            for (int k = 0; k < s.Spec.Foils.Length; k++)
            {
                var f = s.Spec.Foils[k];
                var at = At(c, Arm(c, f.Center));
                var chordB = s.FoilChord[k].X == 0f && s.FoilChord[k].Y == 0f && s.FoilChord[k].Z == 0f ? f.Chord : s.FoilChord[k];
                var normalB = s.FoilNormal[k].X == 0f && s.FoilNormal[k].Y == 0f && s.FoilNormal[k].Z == 0f ? f.Normal : s.FoilNormal[k];
                g.Add(new DebugAeroSurface(h, DebugAeroKind.Foil, k, at, _host.Rotate(s.Body, chordB), _host.Rotate(s.Body, normalB), f.Area, true));
                Emit(g, h, DebugAeroKind.Foil, i, at, s.FoilForce[k]);
            }

            for (int k = 0; k < s.Spec.Propellers.Length; k++) { Emit(g, h, DebugAeroKind.Propeller, i, At(c, Arm(c, s.Spec.Propellers[k].Position)), s.PropForce[k]); }
        }

        for (int i = 0; i < HoverHandles.HighWater; i++)
        {
            if (!HoverHandles.IsLive(i)) { continue; }
            var s = Hovers[i]!;
            if (!Live(s.Body, s.BodyGen)) { continue; }
            var h = core.HandleOfBody(s.Body);
            var c = Ctx(s.Body);
            for (int k = 0; k < s.Spec.Fans.Length; k++)
            {
                var fan = s.Spec.Fans[k];
                var dir = _host.Rotate(s.Body, fan.Direction);
                Emit(g, h, DebugAeroKind.Fan, i, At(c, Arm(c, fan.Position)), V3.Mul(dir, -s.FanForce[k]));
            }

            for (int k = 0; k < s.Spec.Thrusters.Length; k++) { Emit(g, h, DebugAeroKind.HoverThruster, i, At(c, Arm(c, s.Spec.Thrusters[k].Position)), s.ThrusterForce[k]); }
        }
    }

    private static void Emit(DebugGeometry g, BodyHandle body, DebugAeroKind kind, int index, Vector3D at, Vector3F force)
    {
        if (force.X == 0f && force.Y == 0f && force.Z == 0f) { return; }
        g.Add(new DebugAeroForce(body, kind, index, at, force));
    }

    // ------------------------------------------------------------------ region selection

    /// <summary>A region takes the actuators of the bodies it holds, and the surfaces of the shapes it holds. Nothing here pulls another body in: the hovercraft's reaction on what lies under it is applied to that body's accumulators at the start of the step and is not state.</summary>
    internal void SelectRegion(RegionSelection s)
    {
        for (int i = 0; i < ThrusterHandles.HighWater; i++) { if (ThrusterHandles.IsLive(i) && Selected(s, Thrusters[i]!.Body)) { s.Add("thruster", i); } }
        for (int i = 0; i < BoatHandles.HighWater; i++) { if (BoatHandles.IsLive(i) && Selected(s, Boats[i]!.Body)) { s.Add("boat", i); } }
        for (int i = 0; i < HoverHandles.HighWater; i++) { if (HoverHandles.IsLive(i) && Selected(s, Hovers[i]!.Body)) { s.Add("hovercraft", i); } }
        for (int sh = 0; sh < Surfaces.Length; sh++) { if (Surfaces[sh] != null && s.Contains("shape", sh)) { s.Add("aero", sh); } }
    }

    private static bool Selected(RegionSelection s, int body) => (uint)body < (uint)s.Bodies.Length && s.Bodies[body];
}
