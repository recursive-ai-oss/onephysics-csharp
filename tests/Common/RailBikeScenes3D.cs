#pragma warning disable OPHYS0002
#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// Rail and bicycle scenes in 3D: the hash-matrix and smoke scenes and the two stability scenes (rail-3d, bicycles-3d), all driven by scripts that read only world state, so a restored or forked copy continues bit for bit.
internal static class RailBikeScenes3D
{
    internal const float Dt = 1f / 60f;
    internal static readonly string[] Names = { "rail-3d", "bicycles-3d" };

    internal sealed class Scene
    {
        internal string Name = "";
        internal PhysicsWorld3D World = null!;
        internal int Steps = 600;
        internal RailVehicle[] Rail = Array.Empty<RailVehicle>();
        internal Bicycle[] Bikes = Array.Empty<Bicycle>();
        internal Action<int> Script = _ => { };
        // Bicycles listed here start below the self-stable band and are expected to fall; the rest must stay up.
        internal int ExpectedFalls;
        // Falls so far: a bicycle that has fallen is removed by the script (its bodies have no collision shape of their own, so a fallen frame would drop through the ground).
        internal int Falls;

        /// <summary>Points the scene at a restored or forked copy of its world: handles are looked up again in it.</summary>
        internal void Rebind(PhysicsWorld3D w) { World = w; Rail = w.GetRailVehicles(); Bikes = w.GetBicycles(); }

        /// <summary>Metrics over the fixed run, accumulated step by step with <see cref="Measure"/>.</summary>
        internal readonly SortedDictionary<string, double> M = new();
        private readonly double[] _start = new double[16];
        private bool _started;

        internal void Measure(int step)
        {
            double maxSpeed = M.GetValueOrDefault("maxSpeed"), nan = M.GetValueOrDefault("nan"), der = M.GetValueOrDefault("derailments"), climb = M.GetValueOrDefault("maxClimbRatio"), esc = M.GetValueOrDefault("escapes");
            var w = World;
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) { continue; }
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type != BodyType.Dynamic) { continue; }
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || !float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y + (double)v.Z * v.Z));
                if (Math.Abs(p.X) > 1000 || Math.Abs(p.Z) > 1000 || p.Y < -5 || p.Y > 100) { esc++; }
            }

            if (!_started)
            {
                _started = true;
                for (int i = 0; i < Rail.Length && i < _start.Length; i++) { _start[i] = Rail[i].Car.Position.Z; }
                for (int i = 0; i < Bikes.Length && i < _start.Length; i++) { _start[i] = Bikes[i].Frame.Position.Z; }
            }

            for (int i = 0; i < Rail.Length; i++)
            {
                var r = Rail[i];
                bool derailed = false;
                for (int k = 0; k < r.WheelsetCount; k++)
                {
                    var ws = r.GetWheelset(k);
                    if (ws.IsDerailed) { derailed = true; }
                    if (Math.Max(ws.VerticalLoadLeft, ws.VerticalLoadRight) > 20000f) { climb = Math.Max(climb, ws.ClimbRatio); }
                }

                if (derailed) { der++; }
            }

                        M["maxSpeed"] = maxSpeed; M["nan"] = nan; M["escapes"] = esc; M["maxClimbRatio"] = climb;
            M["derailments"] = der;   // wheelset-steps without tread contact
                        if (step == Steps - 1)
            {
                M["fallen"] = Falls;
                double minProgress = double.MaxValue;
                for (int i = 0; i < Rail.Length; i++) { minProgress = Math.Min(minProgress, Rail[i].Car.Position.Z - _start[i]); }
                for (int i = 0; i < Bikes.Length; i++) { if (Bikes[i].IsValid) { minProgress = Math.Min(minProgress, Bikes[i].Frame.Position.Z - _start[i]); } }
                M["minProgress"] = minProgress;
                int broken = 0;
                for (int i = 0; i < Rail.Length; i++) { if (Rail[i].IsCouplerBroken(true)) { broken++; } if (Rail[i].IsCouplerBroken(false)) { broken++; } }
                M["brokenCouplers"] = broken;
            }
        }
    }

    private static PhysicsWorld3D NewWorld(int threads, int id) => new(new WorldOptions3D { ThreadCount = threads, WorldId = id });

    internal static Scene Build(string name, int threads) => name switch
    {
        "rail-3d" => RailScene(threads),
        "bicycles-3d" => BicyclesScene(threads),
        _ => throw new ArgumentOutOfRangeException(nameof(name), name, "unknown scene"),
    };

    // A loco with two tram cars up a 4 % grade and into a canted curve, below balancing speed; a second train of two trams through a diverging switch. Throttle follows a speed target, so the script reads only world state.
    private static Scene RailScene(int threads)
    {
        var w = NewWorld(threads, 171);
        var def = TrackDef.Curve(400, 200, 60, 0.1f, 80);   // 80 m straight, transition, 200 m arc, transition, straight
        def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(20, 0), new Vector2F(75, 2), new Vector2F(480, 2) };
        var main = w.AddTrack(def);
        var side = TrackDef.Straight(300); side.Start = new Vector3D(30, 0, 0);
        var siding = w.AddTrack(side);
        siding.AddSwitch(60, new TrackDef { Start = new Vector3D(30, 0, 60), Elements = new[] { TrackElement.Transition(40, 0f, 1f / 300), TrackElement.Arc(100, 300), TrackElement.Straight(200) } }, true);

        var rail = new List<RailVehicle>();
        var loco = w.AddRailVehicle(main, 40, RailDef.Locomotive, 14f);
        rail.Add(loco);
        double s = 40 - 9.25 - 0.6 - 0.02 - 6.25;
        for (int i = 0; i < 2; i++)
        {
            var car = w.AddRailVehicle(main, s, RailDef.Tram, 14f);
            rail[^1].Couple(car, thisFront: false, otherFront: true);
            rail.Add(car);
            s -= 6.25 + 0.6 + 0.02 + 6.25;
        }

        var t1 = w.AddRailVehicle(siding, 30, RailDef.Tram, 9f);
        rail.Add(t1);
        var t2 = w.AddRailVehicle(siding, 30 - 6.25 - 0.6 - 0.02 - 6.25, RailDef.Tram, 9f);
        t1.Couple(t2, thisFront: false, otherFront: true);
        rail.Add(t2);

        var sc = new Scene { Name = "rail-3d", World = w, Rail = rail.ToArray() };
        sc.Script = step =>
        {
            // Hold a target speed with the traction of the lead vehicles; the cars of the second train trail on the first tram's coupler.
            float Throttle(RailVehicle v, float target) => Math.Clamp(0.25f + 0.4f * (target - v.Speed), 0f, 1f);
            sc.Rail[0].Drive(throttle: Throttle(sc.Rail[0], 16f));
            sc.Rail[3].Drive(throttle: Throttle(sc.Rail[3], 10f));
            sc.Rail[1].Drive(throttle: Throttle(sc.Rail[1], 16f));
            sc.Rail[2].Drive(throttle: Throttle(sc.Rail[2], 16f));
            sc.Rail[4].Drive(throttle: Throttle(sc.Rail[4], 10f));
        };
        return sc;
    }

    // Riders on a flat course, uncontrolled: six inside the self-stable band (they must stay up) and two below it (they fall), with a small lean-rate perturbation each.
    private static Scene BicyclesScene(int threads)
    {
        var w = NewWorld(threads, 172);
        w.AddBox(new Vector3F(600, 1, 600), at: new Vector3D(0, -0.5, 0), type: BodyType.Static, material: new Material3D(1f, 0f, 0f));
        var bikes = new List<Bicycle>();
        float[] speeds = { 5f, 5.5f, 6f, 6.5f, 5f, 5.5f, 2.5f, 3f };
        bool[] benchmark = { false, false, false, false, true, true, true, false };
        for (int i = 0; i < speeds.Length; i++)
        {
            var def = benchmark[i] ? BicycleDef.Benchmark : BicycleDef.Road;
            var bike = w.AddBicycle(new Vector3D(-30 + 8 * i, 0, -100), def, speeds[i]);
            float kick = (i % 2 == 0 ? -1f : 1f) * 0.05f;
            var fr = bike.Frame; var fk = bike.Fork;
            fr.AngularVelocity = new Vector3F(0, 0, kick); fk.AngularVelocity = new Vector3F(0, 0, kick);
            bikes.Add(bike);
        }

        var sc = new Scene { Name = "bicycles-3d", World = w, Bikes = bikes.ToArray(), ExpectedFalls = 2 };
        sc.Script = _ =>
        {
            foreach (var b in sc.Bikes)
            {
                if (b.IsValid && b.HasFallen) { sc.Falls++; b.Destroy(); }
            }
        };
        return sc;
    }
}
