using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Destruction;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace Bar;

/// <summary>
/// Stability scenes of the destruction features (own thresholds; no reference engine has the feature): 600 steps each, reporting the fastest body, how far a body sank into the floor, the fragment count,
/// bodies that left the scene, and the largest gain of mechanical energy over the start (a fragment must never leave faster than physics allows).
/// </summary>
internal static class DestructionStability
{
    private sealed class Rng(ulong s)
    {
        private ulong _s = s;
        public double Next() { _s = unchecked(_s * 6364136223846793005UL + 1442695040888963407UL); return (_s >> 11) * (1.0 / 9007199254740992.0); }
    }

    private readonly record struct Result(string Name, double MaxSpeed, double Penetration, int Bodies, int Fragments, int Escapes, double EnergyGain, long Fractures, long BondBreaks, bool Pass);

    internal static int Run(Opts opts)
    {
        var results = new List<Result>
        {
            Wall3D(), Runtime3D(), Bridge3D(), Fracture2D(), Slice2D(),
        };
        int bad = 0;
        foreach (var r in results)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{r.Name} steps=600 maxSpeed={r.MaxSpeed:R} maxPenetration={r.Penetration:R} bodies={r.Bodies} fragments={r.Fragments} escapes={r.Escapes} energyGain={r.EnergyGain:R} fractures={r.Fractures} bondBreaks={r.BondBreaks} {(r.Pass ? "PASS" : "FAIL")}"));
            if (!r.Pass) bad++;
        }
        return bad;
    }

    private static P3.Body[] Dynamics3(P3.PhysicsWorld3D w)
    {
        var l = new List<P3.Body>();
        foreach (var b in w.Bodies) if (!b.IsStatic) l.Add(b);
        return l.ToArray();
    }

    private static double Energy3(P3.PhysicsWorld3D w, double g)
    {
        double e = 0;
        foreach (var b in w.Bodies)
        {
            if (b.IsStatic) continue;
            var v = b.LinearVelocity; var om = b.AngularVelocity;
            var rot = b.Rotation; var iw = b.Inertia * rot.Conjugated().Rotate(om);
            e += 0.5 * b.Mass * (v.X * v.X + v.Y * v.Y + v.Z * v.Z) + 0.5 * (om.X * iw.X + om.Y * iw.Y + om.Z * iw.Z) + b.Mass * g * b.WorldCenterOfMass.Y;
        }
        return e;
    }

    private static readonly DebugGeometry s_geometry = new();

    // The deepest overlap in any contact manifold of the step (negative separation); fragments overlapping each other count as well as the floor.
    private static double Sink3(P3.PhysicsWorld3D w, double floorTop)
    {
        _ = floorTop;
        w.GetDebugGeometry(DebugContent.Contacts, s_geometry);
        double sink = 0;
        foreach (var c in s_geometry.Contacts) sink = Math.Max(sink, -c.Separation);
        return sink;
    }

    private static double Sink2(P2.PhysicsWorld2D w, double floorTop)
    {
        _ = floorTop;
        w.GetDebugGeometry(DebugContent.Contacts, s_geometry);
        double sink = 0;
        foreach (var c in s_geometry.Contacts) sink = Math.Max(sink, -c.Separation);
        return sink;
    }

    private static Result Measure3(string name, P3.PhysicsWorld3D w, Action<int> perStep, double speedLimit, double penLimit, int maxBodies, bool freeEnergy = false, double floorTop = 0, double energyLimit = 0.02)
    {
        double maxSpeed = 0, minY = 0, gain = 0;
        for (int i = 0; i < 600; i++)
        {
            perStep(i);
            // Energy is compared across the step itself: bodies the scene adds or cuts between steps change it by construction.
            double pre = Energy3(w, 9.81);
            w.Step(1f / 60);
            foreach (var b in Dynamics3(w))
            {
                var v = b.LinearVelocity;
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
                minY = Math.Min(minY, b.WorldCenterOfMass.Y);
            }
            minY = Math.Min(minY, -Sink3(w, floorTop));
            if (!freeEnergy) gain = Math.Max(gain, (Energy3(w, 9.81) - pre) / Math.Max(1.0, Math.Abs(pre)));
        }
        int escapes = 0;
        foreach (var b in Dynamics3(w)) { var p = b.WorldCenterOfMass; if (Math.Abs(p.X) > 80 || Math.Abs(p.Z) > 80 || p.Y < -1 || p.Y > 80) escapes++; }
        var c = w.DestructionCounters;
        bool pass = double.IsFinite(maxSpeed) && maxSpeed <= speedLimit && -minY <= penLimit && escapes == 0 && w.BodyCount <= maxBodies && gain <= energyLimit;
        return new Result(name, maxSpeed, Math.Max(0, -minY), w.BodyCount, (int)c.FragmentsAlive, escapes, gain, c.Fractures + c.Slices, c.BondBreaks, pass);
    }

    private static P3.FractureAsset WallAsset(int cells, ulong seed)
        => P3.FractureAsset.FromHull([new(-3, 0, -0.3f), new(3, 0, -0.3f), new(-3, 4, -0.3f), new(3, 4, -0.3f), new(-3, 0, 0.3f), new(3, 0, 0.3f), new(-3, 4, 0.3f), new(3, 4, 0.3f)], new P3.FractureDef { CellCount = cells, Seed = seed });

    // Projectiles into a pre-fractured wall.
    private static Result Wall3D()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(80, 1, 80), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.AddBox(new(80, 20, 1), new Vector3D(0, 10, 25), type: BodyType.Static); // stops the projectiles and flying cells
        // The wall is a dynamic compound standing on the floor under its own weight; projectiles break its bonds.
        w.AddBreakable(WallAsset(48, 5), new Vector3D(0, 0, 0), def: new BreakableDef { Strength = 4e5f }, density: 1800f);
        var rng = new Rng(7);
        var balls = new List<P3.Body>();
        return Measure3("fracture-wall-3d", w, step =>
        {
            if (step % 120 == 20)
            {
                var b = w.AddSphere(0.3f, new Vector3D(-1.5 + 3 * rng.Next(), 1 + 2 * rng.Next(), -12), density: 3000f);
                b.LinearVelocity = new(0, 2, 35);
                balls.Add(b);
            }
        }, 55, 0.55, 400);
    }

    // Dropped crates shattering on impact.
    private static Result Runtime3D()
    {
        var w = new P3.PhysicsWorld3D();
        w.AddBox(new(80, 1, 80), new Vector3D(0, -0.5, 0), type: BodyType.Static);
        w.Debris = new() { MaxFragments = 300, SleepToStaticSteps = 120 };
        var rng = new Rng(3);
        for (int i = 0; i < 24; i++)
        {
            var c = w.AddBox(new(0.8f, 0.8f, 0.8f), new Vector3D(-6 + (i % 6) * 2.4, 2 + (i / 6) * 3 + rng.Next(), -4 + (i / 6) * 2.5 + rng.Next()));
            c.SetFracturable(new P3.FractureDef { CellCount = 12, ImpulseThreshold = 4f, Seed = (ulong)(100 + i), Depth = 1, Pattern = FracturePattern.Clustered });
        }
        return Measure3("fracture-runtime-3d", w, _ => { }, 25, 0.95, 500);
    }

    // A static beam bridge anchored at both ends under a growing load until it collapses.
    private static Result Bridge3D()
    {
        var w = new P3.PhysicsWorld3D(new() { SleepEnabled = false });
        w.AddBox(new(80, 1, 80), new Vector3D(0, -3.5, 0), type: BodyType.Static);
        const int n = 10; const float L = 1f, h = 0.5f;
        var cells = new List<Vector3F[]>();
        for (int i = 0; i < n; i++)
        {
            var p = new Vector3F[8];
            for (int k = 0; k < 8; k++) p[k] = new Vector3F(i * L + ((k & 1) != 0 ? L : 0), (k & 2) != 0 ? h / 2 : -h / 2, (k & 4) != 0 ? 1f : -1f);
            cells.Add(p);
        }
        var asset = P3.FractureAsset.FromCells(cells);
        var bridge = w.AddBreakable(asset, new Vector3D(-5, 0, 0), def: new BreakableDef { Strength = 3e6f, AnchorParts = [0, n - 1] }, type: BodyType.Static, density: 1500f);
        var load = w.AddBox(new(0.8f, 0.8f, 0.8f), new Vector3D(0, h / 2 + 0.4 + 0.001, 0), density: 400f);
        return Measure3("breakable-bridge-3d", w, step => { if (step % 10 == 0 && load.IsValid) load.Mass = load.Mass * 1.08f; }, 5, 0.7, 100, freeEnergy: true, floorTop: -3);
    }

    private static P2.Body[] Dynamics2(P2.PhysicsWorld2D w)
    {
        var l = new List<P2.Body>();
        foreach (var b in w.Bodies) if (!b.IsStatic) l.Add(b);
        return l.ToArray();
    }

    private static double Energy2(P2.PhysicsWorld2D w, double g)
    {
        double e = 0;
        foreach (var b in w.Bodies)
        {
            if (b.IsStatic) continue;
            var v = b.LinearVelocity;
            e += 0.5 * b.Mass * (v.X * v.X + v.Y * v.Y) + 0.5 * b.Inertia * b.AngularVelocity * b.AngularVelocity + b.Mass * g * b.WorldCenterOfMass.Y;
        }
        return e;
    }

    private static Result Measure2(string name, P2.PhysicsWorld2D w, Action<int> perStep, double speedLimit, double penLimit, int maxBodies, double energyLimit = 0.02)
    {
        double maxSpeed = 0, minY = 0, gain = 0;
        for (int i = 0; i < 600; i++)
        {
            perStep(i);
            double pre = Energy2(w, 9.81);
            w.Step(1f / 60);
            foreach (var b in Dynamics2(w))
            {
                var v = b.LinearVelocity;
                maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y));
                minY = Math.Min(minY, b.WorldCenterOfMass.Y);
            }
            minY = Math.Min(minY, -Sink2(w, 0));
            gain = Math.Max(gain, (Energy2(w, 9.81) - pre) / Math.Max(1.0, Math.Abs(pre)));
        }
        int escapes = 0;
        foreach (var b in Dynamics2(w)) { var p = b.WorldCenterOfMass; if (Math.Abs(p.X) > 80 || p.Y < -1 || p.Y > 80) escapes++; }
        var c = w.DestructionCounters;
        bool pass = double.IsFinite(maxSpeed) && maxSpeed <= speedLimit && -minY <= penLimit && escapes == 0 && w.BodyCount <= maxBodies && gain <= energyLimit;
        return new Result(name, maxSpeed, Math.Max(0, -minY), w.BodyCount, (int)c.FragmentsAlive, escapes, gain, c.Fractures + c.Slices, c.BondBreaks, pass);
    }

    private static Result Fracture2D()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new(80, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        w.Debris = new() { MaxFragments = 200, SleepToStaticSteps = 120 };
        var rng = new Rng(11);
        for (int i = 0; i < 20; i++)
        {
            var c = w.AddBox(new(0.8f, 0.8f), new Vector2D(-8 + (i % 10) * 1.8, 2 + (i / 10) * 3 + rng.Next()), (float)rng.Next());
            c.SetFracturable(new P2.FractureDef { CellCount = 10, ImpulseThreshold = 3f, Seed = (ulong)(50 + i), Depth = 1 });
        }
        return Measure2("fracture-2d", w, _ => { }, 15, 0.2, 400);
    }

    private static Result Slice2D()
    {
        var w = new P2.PhysicsWorld2D();
        w.AddBox(new(80, 1), new Vector2D(0, -0.5), type: BodyType.Static);
        var rng = new Rng(5);
        return Measure2("slice-2d", w, step =>
        {
            if (step % 30 == 0 && step < 360) w.AddBox(new(1.2f, 0.8f), new Vector2D(-4 + 8 * rng.Next(), 6), (float)(rng.Next() * 3));
            if (step % 15 == 7 && step < 420)
            {
                // Cut the first two bodies that are in the air and big enough, along a tilted line through their centres.
                int cut = 0;
                foreach (var b in Dynamics2(w))
                {
                    if (cut == 2) break;
                    if (b.Mass < 0.5f) continue;
                    var p = b.WorldCenterOfMass;
                    try { w.Slice(b, p, new Vector2F((float)(rng.Next() - 0.5), 1f)); cut++; } catch (FractureException) { }
                }
            }
        }, 15, 0.2, 200, energyLimit: 0.06);
    }
}
