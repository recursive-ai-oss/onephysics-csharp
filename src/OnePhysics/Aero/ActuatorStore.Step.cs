using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Aero;

internal sealed partial class ActuatorStore
{
    /// <summary>What a force model needs to know about the body it pushes, read once per body per step.</summary>
    private struct BodyCtx
    {
        public int B;
        public Vector3D Com;
        public Vector3F Lc, Vel, Omega;
        public float Mass;
    }

    private BodyCtx Ctx(int b) => new()
    {
        B = b, Com = _host.PositionOf(b), Lc = _host.LocalCenterOf(b), Vel = _host.LinearVelocityOf(b), Omega = _host.AngularVelocityOf(b), Mass = _host.MassOf(b),
    };

    private Vector3F Arm(in BodyCtx c, Vector3F bodyPoint) => _host.Rotate(c.B, V3.Sub(bodyPoint, c.Lc));

    private static Vector3D At(in BodyCtx c, Vector3F arm) => new(c.Com.X + arm.X, c.Com.Y + arm.Y, c.Com.Z + arm.Z);

    private static Vector3F PointVelocity(in BodyCtx c, Vector3F arm) => V3.Add(c.Vel, V3.Cross(c.Omega, arm));

    private void Apply(in BodyCtx c, Vector3F force, Vector3F arm, Vector3F extraTorque) =>
        _host.AddForce(c.B, force, V3.Add(V3.Cross(arm, force), extraTorque));

    /// <summary>Keeps a body that is being commanded awake.</summary>
    private void Wake(int b)
    {
        var bodies = _host.Core.Bodies;
        if (!bodies.IsAwake(b)) { _host.Core.WakeIsland(bodies.Island[b]); }
        bodies.SleepTime[b] = 0f;
    }

    private bool Live(int b, int gen)
    {
        var bodies = _host.Core.Bodies;
        return bodies.Handles.IsAlive(b, gen) && bodies.IsDynamic(b);
    }

    /// <summary>Runs the actuators once, before the world steps.</summary>
    public void Step(float dt)
    {
        if (!HasWork) { return; }
        StepSurfaces(dt);
        for (int i = 0; i < ThrusterHandles.HighWater; i++) { if (ThrusterHandles.IsLive(i)) { StepThruster(Thrusters[i]!, dt); } }
        for (int i = 0; i < BoatHandles.HighWater; i++) { if (BoatHandles.IsLive(i)) { StepBoat(Boats[i]!, dt); } }
        for (int i = 0; i < HoverHandles.HighWater; i++) { if (HoverHandles.IsLive(i)) { StepHover(Hovers[i]!, dt); } }
    }

    // ------------------------------------------------------------------ the medium

    /// <summary>Wind at a point: the sum of the flow of every area field whose box holds it, in slot order. The fields' own force is separate.</summary>
    public Vector3F WindAt(Vector3D p)
    {
        var fields = _host.FieldsOf;
        float x = 0f, y = 0f, z = 0f;
        var defs = fields.Definitions;
        for (int i = 0; i < defs.Length; i++)
        {
            var d = defs[i];
            if (d == null || d.Kind != Fields.FieldKind.Area) { continue; }
            if (p.X < d.Minimum.X || p.X > d.Maximum.X || p.Y < d.Minimum.Y || p.Y > d.Maximum.Y || p.Z < d.Minimum.Z || p.Z > d.Maximum.Z) { continue; }
            x += d.Flow.X; y += d.Flow.Y; z += d.Flow.Z;
        }

        return new Vector3F(x, y, z);
    }

    private bool WaterAt(int index, out WaterSample w)
    {
        int nf = _host.FluidCount;
        if (index < nf) { return _host.GetFluid(index, out w); }
        var defs = _host.FieldsOf.Definitions;
        int k = index - nf;
        w = default;
        if (k >= defs.Length) { return false; }
        var d = defs[k];
        if (d == null || d.Kind != Fields.FieldKind.Buoyancy) { return false; }
        w = new WaterSample { Normal = d.Axis, Offset = d.SurfaceHeight, Density = d.Density, Flow = d.Flow, Limited = true, Min = d.Minimum, Max = d.Maximum };
        return true;
    }

    private int WaterCount => _host.FluidCount + _host.FieldsOf.Definitions.Length;

    /// <summary>The first water (fluids in slot order, then buoyancy fields) the point is under or within <paramref name="reach"/> above.</summary>
    public bool FindWater(Vector3D p, float reach, out WaterSample water)
    {
        int n = WaterCount;
        for (int i = 0; i < n; i++)
        {
            if (!WaterAt(i, out water)) { continue; }
            if (water.HeightOf(p) < reach && water.Covers(p, reach)) { return true; }
        }

        water = default;
        return false;
    }

    /// <summary>How far under the surface a thing of the given size is: 0 clear of the water, 1 wholly under, linear in between around the surface.</summary>
    private static float Immersion(float height, float size) => V3.Clamp01(0.5f - height / size);

    // ------------------------------------------------------------------ surfaces

    private void StepSurfaces(float dt)
    {
        if (_surfaceCount == 0) { return; }
        var core = _host.Core;
        var bodies = core.Bodies;
        int hw = Math.Min(Surfaces.Length, core.Shapes.HighWater);
        BodyCtx c = default;
        int ctxBody = -1;
        for (int s = 0; s < hw; s++)
        {
            var st = Surfaces[s];
            if (st == null || !core.Shapes.Handles.IsAlive(s, st.ShapeGen)) { continue; }
            int b = core.Shapes.Owner[s];
            if (!bodies.IsLive(b) || !bodies.IsDynamic(b) || !bodies.IsAwake(b) || _host.ShapeIsSensor(s)) { continue; }
            if (ctxBody != b) { c = Ctx(b); ctxBody = b; }
            var spec = st.Spec;
            var centre = V3.Add(_host.ShapeOffset(s), _host.ShapeToBody(s, spec.Center));
            var arm = Arm(c, centre);
            st.LastForce = default;
            if (spec.Kind == AeroSurfaceKind.Bluff) { st.LastForce = StepBluff(c, s, spec, arm, dt); continue; }
            float area = spec.Area;
            if (!(area > 0f))
            {
                if (float.IsNaN(st.AutoArea)) { st.AutoArea = _host.SilhouetteArea(s, _host.ShapeToBody(s, spec.Normal)); }
                area = st.AutoArea;
            }

            if (!(area > 0f)) { continue; }
            var chord = _host.Rotate(b, _host.ShapeToBody(s, spec.Chord));
            var normal = _host.Rotate(b, _host.ShapeToBody(s, spec.Normal));
            st.LastForce = ApplyLifting(c, spec, area, chord, normal, arm, dt);
        }
    }

    /// <summary>Density, flow and the immersion of a point for a surface: false when the surface is out of its medium.</summary>
    private bool Medium(AeroMedium medium, Vector3D at, float size, out float density, out Vector3F flow)
    {
        if (medium == AeroMedium.Air) { density = AirDensity; flow = WindAt(at); return density > 0f; }
        density = 0f; flow = default;
        if (!FindWater(at, size, out var w)) { return false; }
        float im = Immersion(w.HeightOf(at), size);
        if (!(im > 0f)) { return false; }
        density = w.Density * im; flow = w.Flow;
        return true;
    }

    private Vector3F ApplyLifting(in BodyCtx c, SurfaceSpec spec, float area, Vector3F chordW, Vector3F normalW, Vector3F arm, float dt)
    {
        var at = At(c, arm);
        if (!Medium(spec.Medium, at, float.Sqrt(area), out float rho, out var flow)) { return default; }
        var u = V3.Sub(flow, PointVelocity(c, arm));
        float uc = V3.Dot(u, chordW), un = V3.Dot(u, normalW);
        spec.Coef.Force(uc, un, area, rho, out float fc, out float fn);
        var f = V3.Add(V3.Mul(chordW, fc), V3.Mul(normalW, fn));
        float speed = float.Sqrt(uc * uc + un * un);
        f = CapAt(c, f, arm, speed, dt);
        Apply(c, f, arm, default);
        return f;
    }

    private Vector3F StepBluff(in BodyCtx c, int shape, SurfaceSpec spec, Vector3F arm, float dt)
    {
        var at = At(c, arm);
        float size = spec.Area > 0f ? float.Sqrt(spec.Area) : 1f;
        if (!Medium(spec.Medium, at, size, out float rho, out var flow)) { return default; }
        var u = V3.Sub(flow, PointVelocity(c, arm));
        float speed = V3.Len(u);
        if (!(speed > 1e-6f)) { return default; }
        float area = spec.Area;
        if (!(area > 0f))
        {
            // The silhouette seen along the flow, in the body's frame.
            var dir = _host.RotateInverse(c.B, V3.Mul(u, 1f / speed));
            area = _host.SilhouetteArea(shape, dir);
            if (!(area > 0f)) { return default; }
        }

        float k = 0.5f * rho * spec.Coef.MinDrag * area * speed;
        var f = CapAt(c, V3.Mul(u, k), arm, speed, dt);
        Apply(c, f, arm, default);
        return f;
    }

    /// <summary>
    /// Scales a force down so that, applied for one step at its point, it could at most take the point's speed relative to the flow down to zero: a damping force on a light body, or at a long arm, would otherwise overshoot and the explicit step
    /// would blow up. The limit uses the effective inverse mass of the body at that point along the force.
    /// </summary>
    private Vector3F CapAt(in BodyCtx c, Vector3F f, Vector3F arm, float speed, float dt)
    {
        float m = V3.Len(f);
        if (!(m > 0f)) { return f; }
        float w = _host.InverseMassAt(c.B, arm, V3.Mul(f, 1f / m));
        if (!(w > 0f)) { return f; }
        float limit = speed / (dt * w);
        return m > limit ? V3.Mul(f, limit / m) : f;
    }

    /// <summary>Scales a force down so that it could at most stop the body's motion relative to the flow within the step.</summary>
    private static Vector3F Cap(Vector3F f, float limit)
    {
        float m = V3.Len(f);
        if (m > limit && m > 0f) { return V3.Mul(f, limit / m); }
        return f;
    }

    // ------------------------------------------------------------------ thrusters

    private void StepThruster(ThrusterState t, float dt)
    {
        t.LastForce = default;
        if (!Live(t.Body, t.BodyGen)) { return; }
        var bodies = _host.Core.Bodies;
        bool commanded = t.Throttle != 0f || t.Output != 0f;
        if (!bodies.IsAwake(t.Body))
        {
            if (!commanded) { return; }
            Wake(t.Body);
        }
        else if (commanded) { bodies.SleepTime[t.Body] = 0f; }

        var c = Ctx(t.Body);
        t.LastForce = default;
        float output = t.Output;
        t.LastForce = Drive(c, t.Spec, ref output, t.Throttle, t.Gimbal1, t.Gimbal2, dt);
        t.Output = output;
    }

    /// <summary>
    /// One thruster for one step: the response lags the throttle (implicit first order, so any step length is stable), the thrust acts along the possibly gimballed axis at its position, falls with the speed along the axis
    /// relative to the medium when the propeller has a pitch speed, fades in as a propeller goes under water, and adds its spin reaction. Returns the force.
    /// </summary>
    private Vector3F Drive(in BodyCtx c, ThrusterSpec t, ref float output, float throttle, float gimbal1, float gimbal2, float dt)
    {
        float lo = -t.ReverseFraction;
        float target = throttle < lo ? lo : throttle > 1f ? 1f : throttle;
        output += (target - output) * (dt / (t.ResponseTime + dt));
        // A closed throttle lets the thrust die away to exactly zero (so the body can sleep), not to a denormal.
        if (target == 0f && Math.Abs(output) < 1e-4f) { output = 0f; }
        if (output == 0f || t.MaxThrust == 0f) { return default; }
        var dirB = t.Direction;
        if (t.MaxGimbal > 0f && (gimbal1 != 0f || gimbal2 != 0f)) { dirB = Gimbal(t, gimbal1, gimbal2); }
        var dirW = _host.Rotate(c.B, dirB);
        var arm = Arm(c, t.Position);
        var at = At(c, arm);
        Vector3F flow;
        float im = 1f;
        if (t.Medium == AeroMedium.Water)
        {
            if (!FindWater(at, t.ImmersionDepth, out var w)) { return default; }
            im = Immersion(w.HeightOf(at), t.ImmersionDepth);
            if (!(im > 0f)) { return default; }
            flow = w.Flow;
        }
        else { flow = WindAt(at); }

        float thrust = t.MaxThrust * output * im;
        if (t.PitchSpeed > 0f)
        {
            // The propeller can only push the water back as fast as its pitch: thrust runs out as the craft reaches that speed along the axis.
            float va = V3.Dot(V3.Sub(PointVelocity(c, arm), flow), dirW) * (output > 0f ? 1f : -1f);
            thrust *= V3.Clamp01(1f - va / t.PitchSpeed);
        }

        var f = V3.Mul(dirW, thrust);
        Apply(c, f, arm, t.SpinTorque != 0f ? V3.Mul(dirW, t.SpinTorque * thrust) : default);
        return f;
    }

    /// <summary>
    /// The thrust axis turned by the gimbal commands (fractions of the maximum angle, clamped to ±1): the first about the gimbal axis (or, with none given, about z in a planar world and about an axis perpendicular to the thrust in a
    /// spatial one), the second, spatial only, about the axis perpendicular to both.
    /// </summary>
    private Vector3F Gimbal(ThrusterSpec t, float g1, float g2)
    {
        var d = t.Direction;
        Vector3F a1 = t.GimbalAxis;
        bool planar = _host.Planar;
        if (a1.X == 0f && a1.Y == 0f && a1.Z == 0f)
        {
            if (planar) { a1 = new Vector3F(0f, 0f, 1f); }
            else
            {
                var hint = Math.Abs(d.Y) < 0.9f ? new Vector3F(0f, 1f, 0f) : new Vector3F(1f, 0f, 0f);
                a1 = V3.Unit(V3.Cross(d, hint));
            }
        }

        float ang1 = (g1 < -1f ? -1f : g1 > 1f ? 1f : g1) * t.MaxGimbal;
        var (s1, c1) = Math32.SinCos(ang1);
        var d1 = V3.Rotate(d, a1, s1, c1);
        if (planar || g2 == 0f) { return d1; }
        var a2v = V3.Cross(d, a1);
        if (!(V3.Len2(a2v) > 1e-6f)) { return d1; }
        var a2 = V3.Unit(a2v);
        float ang2 = (g2 < -1f ? -1f : g2 > 1f ? 1f : g2) * t.MaxGimbal;
        var (s2, c2) = Math32.SinCos(ang2);
        return V3.Rotate(d1, a2, s2, c2);
    }

    // ------------------------------------------------------------------ boats

    private void StepBoat(BoatState s, float dt)
    {
        s.LastHullForce = default;
        Array.Clear(s.FoilForce); Array.Clear(s.PropForce);
        if (!Live(s.Body, s.BodyGen)) { return; }
        var bodies = _host.Core.Bodies;
        bool commanded = s.Throttle != 0f || s.Steer != 0f;
        if (!bodies.IsAwake(s.Body))
        {
            if (!commanded) { return; }
            Wake(s.Body);
        }
        else if (commanded) { bodies.SleepTime[s.Body] = 0f; }

        var spec = s.Spec;
        var c = Ctx(s.Body);
        bool planar = _host.Planar;

        // The rudder angle follows the command at a limited rate.
        float target = (s.Steer < -1f ? -1f : s.Steer > 1f ? 1f : s.Steer) * spec.MaxRudderAngle;
        float step = spec.RudderRate * dt;
        float diff = target - s.Rudder;
        s.Rudder += diff > step ? step : diff < -step ? -step : diff;

        var fwd = _host.Rotate(s.Body, spec.Forward);
        var up = _host.Rotate(s.Body, spec.Up);
        // In a right-handed frame, facing forward with up above, the right-hand side is forward × up.
        var right = planar ? new Vector3F(0f, 0f, 1f) : V3.Cross(fwd, up);

        // Hull drag: a quadratic law per axis of the hull, in volumetric coefficients (force = ½ρ·C·V^(2/3)·|u|·u, with the submerged volume V), against the water's flow at the centre of mass.
        s.LastHullForce = default;
        float measure = s.Body < _submerged.Length ? _submerged[s.Body] : 0f;
        if (measure > 0f && FindWater(c.Com, _host.ReachOf(s.Body), out var water))
        {
            var u = V3.Sub(water.Flow, c.Vel);
            float refArea;
            if (planar) { refArea = float.Sqrt(measure); }
            else { float cb = Math32.Cbrt(measure); refArea = cb * cb; }
            float k = 0.5f * water.Density * refArea;
            float uf = V3.Dot(u, fwd), ul = V3.Dot(u, right), uv = V3.Dot(u, up);
            float ff = HullAxis(k * spec.DragLongitudinal, uf, c.Mass, dt);
            float fl = planar ? 0f : HullAxis(k * spec.DragLateral, ul, c.Mass, dt);
            float fv = k * spec.DragVertical * Math.Abs(uv) * uv + spec.HeaveDamping * water.Density * measure * uv;
            float capV = c.Mass * Math.Abs(uv) / dt;
            fv = fv > capV ? capV : fv < -capV ? -capV : fv;
            var f = V3.Add(V3.Add(V3.Mul(fwd, ff), V3.Mul(right, fl)), V3.Mul(up, fv));
            s.LastHullForce = f;
            _host.AddForce(s.Body, f, default);
        }

        // Foils: keel and rudder, lifting surfaces in water. A steered foil turns about the craft's up axis (z in a planar world).
        var steerAxis = planar ? new Vector3F(0f, 0f, 1f) : spec.Up;
        for (int i = 0; i < spec.Foils.Length; i++)
        {
            var foil = spec.Foils[i];
            var chordB = foil.Chord; var normalB = foil.Normal;
            float gain = spec.SteerGain[i];
            if (gain != 0f && s.Rudder != 0f)
            {
                // A positive rudder command turns the boat to the right (clockwise seen from above): the trailing edge swings to the right, which is a turn of the chord (it points aft) about the up axis by the positive angle.
                var (sa, ca) = Math32.SinCos(s.Rudder * gain);
                chordB = V3.Rotate(chordB, steerAxis, sa, ca);
                normalB = V3.Rotate(normalB, steerAxis, sa, ca);
            }

            s.FoilForce[i] = ApplyLifting(c, foil, foil.Area, _host.Rotate(s.Body, chordB), _host.Rotate(s.Body, normalB), Arm(c, foil.Center), dt);
            s.FoilChord[i] = chordB; s.FoilNormal[i] = normalB;
        }

        for (int i = 0; i < spec.Propellers.Length; i++)
        {
            float o = s.PropOutput[i];
            s.PropForce[i] = Drive(c, spec.Propellers[i], ref o, s.Throttle, s.Steer, 0f, dt);
            s.PropOutput[i] = o;
        }
    }

    /// <summary>The drag along one hull axis, pointing along the relative flow, never more than stops the hull's motion along that axis within the step.</summary>
    private static float HullAxis(float k, float u, float mass, float dt)
    {
        float m = k * Math.Abs(u) * u;
        float cap = mass * Math.Abs(u) / dt;
        return m > cap ? cap : m < -cap ? -cap : m;
    }

    // ------------------------------------------------------------------ hovercraft

    private void StepHover(HoverState s, float dt)
    {
        Array.Clear(s.FanForce); Array.Clear(s.ThrusterForce);
        if (!Live(s.Body, s.BodyGen)) { return; }
        var bodies = _host.Core.Bodies;
        bool commanded = s.Lift > 0f || s.Throttle != 0f;
        if (!bodies.IsAwake(s.Body))
        {
            if (!commanded) { return; }
            Wake(s.Body);
        }
        else if (commanded) { bodies.SleepTime[s.Body] = 0f; }

        var spec = s.Spec;
        var c = Ctx(s.Body);
        float lift = s.Lift < 0f ? 0f : s.Lift > 1f ? 1f : s.Lift;
        float cushion = 0f;
        var core = _host.Core;
        for (int i = 0; i < spec.Fans.Length; i++)
        {
            var fan = spec.Fans[i];
            var arm = Arm(c, fan.Position);
            var origin = At(c, arm);
            var dir = _host.Rotate(s.Body, fan.Direction);

            // The nearest surface along the fan's axis: any shape but the craft's own, or the surface of a fluid (which carries the flow).
            float dist = fan.Range;
            bool hit = false;
            Vector3F surfaceVel = default;
            int hitBody = -1;
            if (_host.Cast(origin, dir, fan.Range, s.Body, out float d0, out _, out int hb)) { hit = true; dist = d0; hitBody = hb; }
            int nw = WaterCount;
            for (int w = 0; w < nw; w++)
            {
                if (!WaterAt(w, out var water) || !water.Covers(origin, fan.Range)) { continue; }
                float h = water.HeightOf(origin);
                float dn = V3.Dot(water.Normal, dir);
                if (!(h > 0f) || !(dn < 0f)) { continue; }
                float t = h / -dn;
                if (t <= fan.Range && (!hit || t < dist)) { hit = true; dist = t; surfaceVel = water.Flow; hitBody = -1; }
            }

            s.FanHeight[i] = hit ? dist : fan.Range;
            s.FanForce[i] = 0f;
            if (!hit) { continue; }
            bool dynamicHit = hitBody >= 0 && core.Bodies.IsDynamic(hitBody);
            Vector3F hitArm = default;
            if (dynamicHit)
            {
                var hc = _host.PositionOf(hitBody);
                hitArm = new Vector3F((float)(origin.X + dir.X * dist - hc.X), (float)(origin.Y + dir.Y * dist - hc.Y), (float)(origin.Z + dir.Z * dist - hc.Z));
                surfaceVel = PointVelocityOf(hitBody, hitArm);
            }

            // Ground effect: the cushion pressure falls off with the square of the remaining gap (zero at the range), and the cushion damps the closing speed.
            float cush = 1f - dist / fan.Range;
            cush = cush < 0f ? 0f : cush;
            float closing = V3.Dot(V3.Sub(PointVelocity(c, arm), surfaceVel), dir);
            float f = fan.MaxLift * lift * cush * cush + fan.Damping * cush * closing;
            if (f < 0f) { f = 0f; }
            cushion += cush * cush;
            s.FanForce[i] = f;
            if (f == 0f) { continue; }
            var force = V3.Mul(dir, -f);
            Apply(c, force, arm, default);
            if (dynamicHit)
            {
                // The air pushes back on what is underneath, at the spot the fan's axis meets it.
                var back = V3.Mul(dir, f);
                if (!bodies.IsAwake(hitBody)) { Wake(hitBody); }
                _host.AddForce(hitBody, back, V3.Cross(hitArm, back));
            }
        }

        // Skirt: drag on the motion along the surface and damping of rotation, both in proportion to how much cushion there is.
        float k = spec.Fans.Length > 0 ? cushion / spec.Fans.Length : 0f;
        if (k > 0f)
        {
            if (spec.SkirtDrag > 0f)
            {
                var up = _host.Rotate(s.Body, spec.Up);
                var along = V3.Sub(c.Vel, V3.Mul(up, V3.Dot(c.Vel, up)));
                float sp = V3.Len(along);
                var drag = V3.Mul(along, -spec.SkirtDrag * k);
                _host.AddForce(s.Body, Cap(drag, c.Mass * sp / dt), default);
            }

            if (spec.AngularDamping > 0f)
            {
                float w = V3.Len(c.Omega);
                if (w > 0f)
                {
                    float torque = spec.AngularDamping * k * w;
                    float invI = _host.InverseInertiaAbout(s.Body, V3.Mul(c.Omega, 1f / w));
                    if (invI > 0f && torque > w / (dt * invI)) { torque = w / (dt * invI); }
                    _host.AddForce(s.Body, default, V3.Mul(c.Omega, -torque / w));
                }
            }
        }

        for (int i = 0; i < spec.Thrusters.Length; i++)
        {
            float o = s.ThrusterOutput[i];
            s.ThrusterForce[i] = Drive(c, spec.Thrusters[i], ref o, s.Throttle, s.Steer, 0f, dt);
            s.ThrusterOutput[i] = o;
        }
    }

    private Vector3F PointVelocityOf(int body, Vector3F arm) => V3.Add(_host.LinearVelocityOf(body), V3.Cross(_host.AngularVelocityOf(body), arm));
}
