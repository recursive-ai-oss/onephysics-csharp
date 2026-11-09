#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Aero;
using RecursiveAI.OnePhysics.Physics2D;

// Side-view scenes for boats, hovercraft, gliders and thrusters: the hash-matrix and smoke scene (a boat, a hovercraft, a glider in a gust and a lander, snapshotted and restored half-way) and the two stability scenes (boats-2d,
// thrusters-2d). Scripts use only arithmetic on the step number and the state of the craft, so the scenes are byte-identical on every platform.
internal static class Actuators2DScene
{
    internal const float Dt = 1f / 60f;

    internal sealed class Stability
    {
        internal required string Name;
        internal required PhysicsWorld2D World { get; set; }
        internal Boat[] Boats = Array.Empty<Boat>();
        internal Body[] Craft = Array.Empty<Body>();
        internal Thruster[] Thrusters = Array.Empty<Thruster>();
        internal Action<int> Script = _ => { };
        internal int Steps = 600;
        internal double TargetHeight;        // thrusters-2d: the height the lander is sent to hold
        internal double GroundTop;           // thrusters-2d: the top of the landing pad

        internal void Rebind(PhysicsWorld2D w)
        {
            World = w;
            for (int i = 0; i < Craft.Length; i++)
            {
                var h = Craft[i].RawHandle;
                Craft[i] = w.GetBody(new BodyHandle(w.Id, h.Index, h.Generation));
                if (i < Boats.Length && w.TryGetBoat(Craft[i], out var b)) { Boats[i] = b; }
            }

            if (Thrusters.Length != 0) { Thrusters = w.GetThrusters(); }
        }
    }

    private static PhysicsWorld2D NewWorld(int threads, int id) => new(new WorldOptions2D { ThreadCount = threads, WorldId = id });

    /// <summary>A boat's hull: a box with its shape lifted so that the body origin sits on the waterline when it floats, plus the boat.</summary>
    internal static (Body Hull, Boat Boat) AddHull(PhysicsWorld2D w, Vector2D at, BoatDef def, float length, float height, float mass)
    {
        float draft = mass / (1000f * length);
        var hull = w.CreateBody(BodyType.Dynamic, at);
        w.CreateBox(hull, new Vector2F(length, height), new Vector2F(0f, 0.5f * height - draft), 0f, 0f, new ShapeDef { Density = mass / (length * height), Friction = 0.2f });
        return (hull, w.AddBoat(hull, def));
    }

    // ------------------------------------------------------------------ boats-2d

    private static Stability BoatsScene(int threads)
    {
        var w = NewWorld(threads, 65);
        w.AddBuoyancyEffector(0.0, 1000f);
        // A channel 80 m long: a floor, two quay walls and a pier standing in the middle.
        w.AddBox(new Vector2F(90f, 1f), new Vector2D(0, -7), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 8f), new Vector2D(-40.5, -1), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 8f), new Vector2D(40.5, -1), type: BodyType.Static);
        w.AddBox(new Vector2F(1f, 3f), new Vector2D(0, -3.5), type: BodyType.Static);
        var craft = new List<Body>(); var boats = new List<Boat>();
        void Add(BoatDef def, float l, float h, float m, double x)
        {
            var (hull, boat) = AddHull(w, new Vector2D(x, 0), def, l, h, m);
            craft.Add(hull); boats.Add(boat);
        }

        Add(BoatDef.Dinghy, 3f, 0.5f, 150f, -30);
        Add(BoatDef.Dinghy, 3f, 0.5f, 150f, -22);
        Add(BoatDef.Speedboat, 6f, 0.8f, 800f, 30);
        Add(BoatDef.Barge, 20f, 1.5f, 20000f, 15);
        for (int i = 0; i < 8; i++) { w.AddBox(new Vector2F(0.7f, 0.7f), new Vector2D(-12 + 3 * i, 0.2), density: 300f); }
        var s = new Stability { Name = "boats-2d", World = w, Boats = boats.ToArray(), Craft = craft.ToArray() };
        float[] throttle = { 0.8f, -0.5f, -0.35f, -0.6f };
        s.Script = step =>
        {
            for (int i = 0; i < s.Boats.Length; i++)
            {
                s.Boats[i].Throttle = throttle[i] * ((step + 90 * i) / 150 % 2 == 0 ? 1f : -1f);
                s.Boats[i].Steer = (step / 45 + i) % 2 == 0 ? 0.5f : -0.5f;
            }
        };
        return s;
    }

    // ------------------------------------------------------------------ thrusters-2d

    private static Stability LanderScene(int threads)
    {
        var w = NewWorld(threads, 66);
        w.AddBox(new Vector2F(60f, 1f), new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.9f, 0f, 0f));
        // The lander: a 2 x 2 body of 400 kg (per metre of depth), a main engine at the centre pushing up and two side thrusters at the top corners.
        var lander = w.AddBox(new Vector2F(2f, 2f), new Vector2D(0, 30), density: 100f);
        lander.LinearVelocity = new Vector2F(1.5f, -3f);
        var rig = ThrusterRig.Lander(2.2f * 400f * 9.81f, 600f, 1f, 1f);
        var main = lander.AddThruster(rig[0]); var left = lander.AddThruster(rig[1]); var right = lander.AddThruster(rig[2]);
        var s = new Stability { Name = "thrusters-2d", World = w, Craft = new[] { lander }, Thrusters = new[] { main, left, right }, TargetHeight = 12.0, GroundTop = 0.0 };
        s.Script = step =>
        {
            var l = s.Craft[0];
            // Hold 12 m until step 300, then come down at a speed that shrinks with the height (a soft touchdown).
            double target = 12.0;
            var v = l.LinearVelocity;
            double targetV = step < 300 ? 0.0 : -Math.Clamp(0.6 * (l.Position.Y - 1.0), 0.3, 3.0);
            float angle = l.Angle, omega = l.AngularVelocity;
            // Height: a PD on the vertical error with gravity feed-forward, tilted thrust corrected by 1/cos.
            float mass = l.Mass;
            float a = step < 300 ? 3f * (float)(target - l.Position.Y) + 2.5f * (float)(targetV - v.Y) : 2.5f * (float)(targetV - v.Y);
            float cos = MathF.Max(0.5f, Math32.Cos(angle));
            float thrust = mass * (9.81f + a) / cos;
            s.Thrusters[0].Throttle = Math.Clamp(thrust / (2.2f * 400f * 9.81f), 0f, 1f);
            // Attitude and sideways drift: command the tilt that pushes against the drift, then the torque that makes it.
            float wantAngle = Math.Clamp(0.12f * v.X, -0.25f, 0.25f);   // a counter-clockwise tilt points the thrust toward -X
            float torque = -40f * (angle - wantAngle) * mass * 0.3f - 18f * omega * mass * 0.3f;
            // The left thruster (top left, pushing right) turns the lander clockwise with a one-metre lever arm, the right one counter-clockwise.
            s.Thrusters[1].Throttle = Math.Clamp(-torque / 600f, 0f, 1f);
            s.Thrusters[2].Throttle = Math.Clamp(torque / 600f, 0f, 1f);
        };
        return s;
    }

    internal static Stability Build(string name, int threads) => name switch
    {
        "boats-2d" => BoatsScene(threads),
        "thrusters-2d" => LanderScene(threads),
        _ => throw new ArgumentException("unknown actuator scene " + name),
    };

    internal static readonly string[] Names = { "boats-2d", "thrusters-2d" };

    // ------------------------------------------------------------------ hash-matrix and smoke scene

    /// <summary>Runs 240 steps of a composite scene; <paramref name="perStep"/> receives the running hash after each. At step 110 the world is snapshotted and restored into a fresh one when <paramref name="restoreHalfway"/> is set, which must not change a bit.</summary>
    internal static ulong Run(int threads, bool restoreHalfway, Action<ulong>? perStep = null)
    {
        var w = NewWorld(threads, 67);
        w.AddBuoyancyEffector(0.0, 1000f);
        w.AddWind(new Vector2F(1.5f, -0.5f));
        w.AddWind(new Vector2F(0f, 3f), new Vector2D(-40, 0), new Vector2D(-10, 500));
        w.AddBox(new Vector2F(600f, 1f), new Vector2D(0, -7), type: BodyType.Static);
        w.AddBox(new Vector2F(100f, 1f), new Vector2D(200, -0.5), type: BodyType.Static, material: new Material2D(0.7f, 0f, 0f));
        var (hull, boat) = AddHull(w, new Vector2D(0, 0), BoatDef.Dinghy, 3f, 0.5f, 150f);
        var craft = w.CreateBody(BodyType.Dynamic, new Vector2D(180, 1));
        w.CreateBox(craft, new Vector2F(2.2f, 0.4f), default, 0f, 0f, new ShapeDef { Density = 300f / 0.88f });
        var hover = w.AddHovercraft(craft, HovercraftDef.ForMass(300f));
        var glider = w.CreateBody(BodyType.Dynamic, new Vector2D(-30, 200));
        w.CreateBox(glider, new Vector2F(1.5f, 0.2f), default, 0f, 0f, new ShapeDef { Density = 20f, Aero = AeroSurface.Body(0.1f, 0.03f) });
        var wing = AeroSurface.Wing(1.2f, 8f); wing.CenterOfPressure = new Vector2F(-0.08f, 0f);
        var tail = AeroSurface.Wing(0.25f, 4f); tail.CenterOfPressure = new Vector2F(-1.2f, 0f); tail.Forward = new Vector2F(1f, -0.13f);
        w.CreateBox(glider, new Vector2F(0.02f, 0.02f), new Vector2F(0f, 0.2f), 0f, 0f, new ShapeDef { Density = 1f, Aero = wing });
        w.CreateBox(glider, new Vector2F(0.02f, 0.02f), new Vector2F(0f, 0.2f), 0f, 0f, new ShapeDef { Density = 1f, Aero = tail });
        glider.LinearVelocity = new Vector2F(13f, -1f);
        var lander = w.AddBox(new Vector2F(2f, 2f), new Vector2D(210, 20), density: 100f);
        var rig = ThrusterRig.Lander(2.2f * 400f * 9.81f, 600f, 1f, 1f);
        var thrusters = new Thruster[rig.Length];
        for (int i = 0; i < rig.Length; i++) { thrusters[i] = lander.AddThruster(rig[i]); }

        ulong hash = 0xCBF29CE484222325UL;
        void Mix(ulong x) { hash = (hash ^ x) * 0x100000001B3UL; hash ^= hash >> 29; }
        void MixF(float f) => Mix(BitConverter.SingleToUInt32Bits(f));
        for (int step = 0; step < 240; step++)
        {
            if (step == 110 && restoreHalfway)
            {
                var snap = w.Snapshot();
                w = NewWorld(threads, 67);
                w.Restore(snap);
                hull = w.GetBody(new BodyHandle(w.Id, hull.RawHandle.Index, hull.RawHandle.Generation));
                craft = w.GetBody(new BodyHandle(w.Id, craft.RawHandle.Index, craft.RawHandle.Generation));
                w.TryGetBoat(hull, out boat); w.TryGetHovercraft(craft, out hover);
                thrusters = w.GetThrusters();
            }

            boat.Throttle = 0.9f; boat.Steer = (step / 40) % 2 == 0 ? 0.5f : -0.5f;
            hover.Lift = Math.Min(1f, step / 60f); hover.Throttle = 0.6f; hover.Steer = (step / 70) % 2 == 0 ? 0.4f : -0.4f;
            thrusters[0].Throttle = 0.6f + (step % 40 < 20 ? 0.1f : 0f); thrusters[1].Throttle = step % 60 < 30 ? 1f : 0f; thrusters[2].Throttle = step % 90 < 30 ? 1f : 0f;
            thrusters[0].Gimbal = (step / 50) % 2 == 0 ? 0.5f : -0.5f;
            w.Step(Dt);
            Mix(w.ComputeHash());
            MixF(boat.ForwardSpeed); MixF(boat.RudderAngle); MixF(boat.HullForce.X);
            for (int k = 0; k < hover.FanCount; k++) { MixF(hover.GetFanGap(k)); MixF(hover.GetFanForce(k)); }
            foreach (var t in thrusters) { MixF(t.Output); MixF(t.Force.Y); }
            perStep?.Invoke(hash);
        }

        return hash;
    }
}
