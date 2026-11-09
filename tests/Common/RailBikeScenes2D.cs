#pragma warning disable OPHYS0003
using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics2D;

// Rail and bicycle scenes in 2D (side view): a train over a grade with couplers, and riders braking, pedalling and leaning. Scripts read only world state, so a restored or forked copy continues bit for bit.
internal static class RailBikeScenes2D
{
    internal const float Dt = 1f / 60f;
    internal static readonly string[] Names = { "rail-2d", "bicycles-2d" };

    internal sealed class Scene
    {
        internal string Name = "";
        internal PhysicsWorld2D World = null!;
        internal int Steps = 600;
        internal RailVehicle[] Rail = Array.Empty<RailVehicle>();
        internal Bicycle[] Bikes = Array.Empty<Bicycle>();
        internal Track Track;
        private readonly double[] _offset = new double[16];
        internal Action<int> Script = _ => { };
        internal readonly SortedDictionary<string, double> M = new();
        private readonly double[] _start = new double[16];
        private bool _started;

        internal void Rebind(PhysicsWorld2D w) { World = w; Rail = w.GetRailVehicles(); Bikes = w.GetBicycles(); }

        internal void Measure(int step)
        {
            double maxSpeed = M.GetValueOrDefault("maxSpeed"), nan = M.GetValueOrDefault("nan"), esc = M.GetValueOrDefault("escapes"), pen = M.GetValueOrDefault("maxPenetration"), off = M.GetValueOrDefault("offTrackSteps"), flips = M.GetValueOrDefault("flips");
            var w = World;
            int hw = w.Core.Bodies.HighWater;
            for (int b = 0; b < hw; b++)
            {
                if (!w.Core.Bodies.IsLive(b)) { continue; }
                var body = w.GetBody(w.Core.HandleOfBody(b));
                if (body.Type != BodyType.Dynamic) { continue; }
                var v = body.LinearVelocity; var p = body.Position;
                if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !float.IsFinite(v.X) || !float.IsFinite(v.Y)) { nan++; continue; }
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt((double)v.X * v.X + (double)v.Y * v.Y));
                if (Math.Abs(p.X) > 2000 || p.Y < -5 || p.Y > 100) { esc++; }
                foreach (var c in w.GetContacts(body)) { for (int i = 0; i < c.PointCount; i++) { pen = Math.Max(pen, -(double)c.Separation(i)); } }
            }

            if (!_started)
            {
                _started = true;
                for (int i = 0; i < Rail.Length && i < _start.Length; i++) { _start[i] = Rail[i].Car.Position.X; _offset[i] = Rail[i].Car.Position.Y - Track.SurfaceAt(Rail[i].Car.Position.X, out _).Y; }
                for (int i = 0; i < Bikes.Length && i < _start.Length; i++) { _start[i] = Bikes[i].Frame.Position.X; }
            }

            // Derailed: the car's height above the rail has changed by more than 0.4 m since the start, or it has tilted 0.5 rad off the track's slope (a wheel briefly unloading over a crest is not a derailment).
            for (int i = 0; i < Rail.Length; i++)
            {
                var car = Rail[i].Car; var p = car.Position;
                double rise = p.Y - Track.SurfaceAt(p.X, out float grade).Y - _offset[i];
                if (Math.Abs(rise) > 0.4 || Math.Abs(car.Angle - Math.Atan(grade)) > 0.5) { off++; }
            }

            int flipped = 0;
            for (int i = 0; i < Bikes.Length; i++) { if (Bikes[i].HasFlipped) { flipped++; } }
            flips = Math.Max(flips, flipped);
            M["maxSpeed"] = maxSpeed; M["nan"] = nan; M["escapes"] = esc; M["maxPenetration"] = pen; M["derailments"] = off; M["flips"] = flips;
            if (step == Steps - 1)
            {
                double minProgress = double.MaxValue;
                for (int i = 0; i < Rail.Length; i++) { minProgress = Math.Min(minProgress, Rail[i].Car.Position.X - _start[i]); }
                for (int i = 0; i < Bikes.Length; i++) { minProgress = Math.Min(minProgress, Bikes[i].Frame.Position.X - _start[i]); }
                M["minProgress"] = minProgress;
                int broken = 0;
                for (int i = 0; i < Rail.Length; i++) { if (Rail[i].IsCouplerBroken(true)) { broken++; } if (Rail[i].IsCouplerBroken(false)) { broken++; } }
                M["brokenCouplers"] = broken;
            }
        }
    }

    internal static Scene Build(string name, int threads) => name switch
    {
        "rail-2d" => RailScene(threads),
        "bicycles-2d" => BicyclesScene(threads),
        _ => throw new ArgumentOutOfRangeException(nameof(name), name, "unknown scene"),
    };

    // A tram-and-wagon train up a 4 % grade, run in with slack; a second, heavier train with a locomotive brakes on the crest.
    private static Scene RailScene(int threads)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 181 });
        var def = TrackDef.Straight(2000);
        def.Elevation = new[] { new Vector2F(0, 0), new Vector2F(60, 0), new Vector2F(160, 4), new Vector2F(2000, 4) };
        var track = w.AddTrack(def);
        var rail = new List<RailVehicle>();
        var loco = w.AddRailVehicle(track, 40, RailDef.Locomotive, 8f);
        rail.Add(loco);
        double s = 40 - 9.25 - 0.6 - 0.02 - 5.25;
        for (int i = 0; i < 3; i++)
        {
            var car = w.AddRailVehicle(track, s, RailDef.Freight, 8f);
            rail[^1].Couple(car, thisFront: false, otherFront: true);
            rail.Add(car);
            s -= 5.25 + 0.6 + 0.02 + 5.25;
        }

        var sc = new Scene { Name = "rail-2d", World = w, Rail = rail.ToArray(), Track = track };
        sc.Script = step =>
        {
            var l = sc.Rail[0];
            l.Drive(throttle: Math.Clamp(0.3f + 0.3f * (12f - l.Speed), 0f, 1f), brake: step >= 450 ? 0.3f : 0f);
        };
        return sc;
    }

    // Riders on a flat road: steady pedalling at several speeds, a rear-brake stop, a front-brake stop hard enough to lift the rear wheel, and a rider leaning back under power.
    private static Scene BicyclesScene(int threads)
    {
        var w = new PhysicsWorld2D(new WorldOptions2D { ThreadCount = threads, WorldId = 182 });
        w.AddBox(new Vector2F(3000, 1), at: new Vector2D(0, -0.5), type: BodyType.Static, material: new Material2D(0.9f, 0f, 0f));
        var bikes = new List<Bicycle>();
        for (int i = 0; i < 6; i++)
        {
            var def = i % 2 == 0 ? BicycleDef.Road : BicycleDef.Benchmark;
            bikes.Add(w.AddBicycle(new Vector2D(20 + 25 * i, 0), def, 6f + i));
        }

        var sc = new Scene { Name = "bicycles-2d", World = w, Bikes = bikes.ToArray() };
        sc.Script = step =>
        {
            for (int i = 0; i < sc.Bikes.Length; i++)
            {
                var b = sc.Bikes[i];
                switch (i)
                {
                    case 0: b.Ride(pedal: 0.4f); break;
                    case 1: b.Ride(rearBrake: step >= 120 ? 1f : 0f); break;
                    case 2: b.Ride(frontBrake: step >= 120 ? 0.5f : 0f); break;
                    case 3: b.Ride(pedal: 0.6f, lean: step >= 200 && step < 260 ? 0.5f : 0f); break;
                    case 4: b.Ride(frontBrake: step >= 240 ? 0.3f : 0f, rearBrake: step >= 240 ? 0.3f : 0f); break;
                    default: b.Ride(pedal: 0.2f, lean: step >= 300 ? -0.3f : 0f); break;
                }
            }
        };
        return sc;
    }
}
