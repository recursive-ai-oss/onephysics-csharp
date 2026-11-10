using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Physics3D;

// The 3D walkers scene: sixteen active humanoids, each on its own lane of ground (they collide with the ground and not with each other), walking 50 m on flat ground, up or down a 3 degree slope, or along a 1.5 m wide path
// with a 90 degree turn. The host side is a few lines of arithmetic on the step number and the figures' poses (a command speed per walker, a point to aim at on the path), so the scene is byte-identical on every platform
// and a restored or forked world continues it bit for bit.
internal static class Walkers3DScene
{
    internal const float Dt = 1f / 60f;
    internal const int Count = 16;
    internal const float Goal = 50f;            // path length, m
    internal const float PathWidth = 1.5f;
    internal const double LookAhead = 1.2;      // pure pursuit: the figure aims at the point of the path 1.2 m ahead of its own projection onto it
    internal const float LaneSpacing = 4f;
    internal const float SlopeDegrees = 3f;
    internal const int StartStep = 60;          // the figures stand for a second, then walk (staggered by three steps each)

    internal enum Terrain { Flat, Up, Down, Path }

    internal sealed class Scene
    {
        internal required PhysicsWorld3D World;
        internal required Terrain Ground;
        internal required Ragdoll[] Rags;
        internal required float[] Speed;       // commanded speeds, m/s
        internal required int Seed;

        /// <summary>After the world was replaced by a fork or a restore: the same figures in the new world.</summary>
        internal void Rebind(PhysicsWorld3D w)
        {
            World = w;
            var rs = w.GetRagdolls();
            for (int k = 0; k < Count; k++) { Rags[k] = rs[k]; }
        }
    }

    // The lane's origin in x; walker k starts at (LaneX(k), 0, 0) and walks towards +z (the path turns towards +x after the first half).
    internal static double LaneX(int k) => -LaneSpacing * k;

    internal static float SpeedOf(int seed, int k) => 0.6f + 0.05f * ((k * 5 + seed * 3) % 9);   // 0.6 .. 1.0 m/s

    internal static float GroundHeight(Terrain t, double z)
    {
        double s = Math64.Sin(SlopeDegrees * Math.PI / 180.0) / Math64.Cos(SlopeDegrees * Math.PI / 180.0);
        return t == Terrain.Up ? (float)(s * z) : t == Terrain.Down ? (float)(-s * z) : 0f;
    }

    internal static Scene Create(Terrain terrain, int seed, int threads, int worldId, RagdollDef? def = null)
    {
        var o = new WorldOptions3D { ThreadCount = threads, WorldId = worldId };
        Ragdoll.ApplyRecommendedOptions(o);
        var w = new PhysicsWorld3D(o);
        if (terrain == Terrain.Up || terrain == Terrain.Down)
        {
            double a = SlopeDegrees * Math.PI / 180.0 * (terrain == Terrain.Up ? -1.0 : 1.0);
            double sa = Math64.Sin(a), ca = Math64.Cos(a);
            // A box whose top surface passes through the origin, rotated about x so that the surface rises (or falls) along +z.
            var rot = QuaternionF.CreateFromAxisAngle(new Vector3F(1f, 0f, 0f), (float)a);
            var ground = w.AddBox(new Vector3F(LaneSpacing * Count + 40f, 1f, 400f), at: new Vector3D(-LaneSpacing * (Count - 1) * 0.5, -0.5 * ca, 0.5 * sa), type: BodyType.Static);
            ground.SetTransform(new Vector3D(-LaneSpacing * (Count - 1) * 0.5, -0.5 * ca, 0.5 * sa), rot);
        }
        else { w.AddBox(new Vector3F(LaneSpacing * Count + 80f, 1f, 400f), at: new Vector3D(-LaneSpacing * (Count - 1) * 0.5, -0.5, 0), type: BodyType.Static); }

        var d = def ?? RagdollDef.Humanoid;
        d.CategoryBits = 2u; d.MaskBits = 1u;   // figures collide with the ground, not each other
        var rags = new Ragdoll[Count];
        var speed = new float[Count];
        for (int k = 0; k < Count; k++)
        {
            rags[k] = w.AddRagdoll(d, new Vector3D(LaneX(k), 0, 0));
            rags[k].EnableActive();
            rags[k].Stand();
            speed[k] = SpeedOf(seed, k);
        }

        return new Scene { World = w, Ground = terrain, Rags = rags, Speed = speed, Seed = seed };
    }

    /// <summary>Where the path of lane <paramref name="k"/> is at arc length <paramref name="s"/>: first half along +z, then along +x.</summary>
    internal static (double X, double Z) PathPoint(int k, double s)
    {
        double half = 0.5 * Goal;
        return s <= half ? (LaneX(k), s) : (LaneX(k) + (s - half), half);
    }

    /// <summary>Arc length of the closest point of the path to (x, z), and the distance from the centre line.</summary>
    internal static (double S, double Off) Project(int k, double x, double z)
    {
        double half = 0.5 * Goal;
        double lx = x - LaneX(k);
        // Leg one: x = 0, z in [0, half]; leg two: z = half, x in [0, half]. Pick the nearer of the two projections.
        double z1 = Math.Clamp(z, 0.0, half); double d1 = Math.Sqrt(lx * lx + (z - z1) * (z - z1));
        double x2 = Math.Clamp(lx, 0.0, half); double d2 = Math.Sqrt((lx - x2) * (lx - x2) + (z - half) * (z - half));
        return d1 <= d2 ? (z1, d1) : (half + x2, d2);
    }

    /// <summary>The host's command for the coming step (step number <paramref name="step"/>, 0-based): aim at a point 1.2 m ahead on the path, or stand at the end of it.</summary>
    internal static void Command(Scene sc, int step)
    {
        for (int k = 0; k < Count; k++)
        {
            if (step < StartStep + 3 * k) { continue; }
            var r = sc.Rags[k];
            var p = r.GetBody(0).Position;
            double x = p.X, z = p.Z;
            if (sc.Ground == Terrain.Path)
            {
                var (s, _) = Project(k, x, z);
                if (s >= Goal - 0.05) { r.Walk(new Vector2F(0f, 0f), (float)(0.5 * Math.PI)); continue; }
                var (tx, tz) = PathPoint(k, Math.Min(s + LookAhead, Goal));
                double dx = tx - x, dz = tz - z;
                double len = Math.Sqrt(dx * dx + dz * dz);
                if (len < 0.05) { len = 0.05; }
                double v = sc.Speed[k];
                r.Walk(new Vector2F((float)(v * dx / len), (float)(v * dz / len)), (float)Math64.Atan2(dx, dz));
            }
            else
            {
                if (z >= Goal) { r.Walk(new Vector2F(0f, 0f), 0f); continue; }
                r.Walk(new Vector2F(0f, sc.Speed[k]), 0f);
            }
        }
    }
}
