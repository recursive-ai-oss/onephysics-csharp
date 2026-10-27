using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Core.Reference;

/// <summary>
/// A deliberately small 3D engine on top of <see cref="WorldCore"/>: round particles that fall under gravity onto a floor, push each other apart
/// through contact constraints found with a sorted grid, and can be tied together with distance joints. It exists to exercise the scheduler, islands, sleeping, events,
/// snapshots and hashes end to end without a real collision engine, and as a worked example of the stage interface. It is not a physics solver to build games on.
/// </summary>
public sealed class ParticleScene
{
    /// <summary>The world (layout <see cref="DimensionLayout.Spatial"/>).</summary>
    public WorldCore World { get; }

    /// <summary>Particle radius in metres. Default 0.25.</summary>
    public float Radius { get; set; } = 0.25f;

    /// <summary>Extra distance within which a contact is created before particles touch (speculative margin). Default 0.025.</summary>
    public float Margin { get; set; } = 0.025f;

    /// <summary>Whether contacts ask for persist events. Default false.</summary>
    public bool ReportPersist { get; set; }

    internal int[] ShapeOfBody = new int[64];
    internal readonly List<long> NewPairs = new();
    internal readonly List<int> Doomed = new();

    /// <summary>Creates a scene over a new world with the given options (layout forced to spatial).</summary>
    public ParticleScene(WorldOptions? options = null)
    {
        var o = options ?? new WorldOptions();
        o.Layout = DimensionLayout.Spatial;
        World = new WorldCore(o);
    }

    /// <summary>The stage set that steps this scene.</summary>
    public ParticleStages Stages => new(this);

    /// <summary>Steps the world by <paramref name="dt"/>.</summary>
    public void Step(float dt) => World.Step(Stages, dt);

    /// <summary>Adds a dynamic particle with its shape slot.</summary>
    public BodyHandle AddParticle(double x, double y, double z, float vx = 0f, float vy = 0f, float vz = 0f)
    {
        var body = World.CreateBody(BodyType.Dynamic, stackPosition(x, y, z), linearVelocity: new[] { vx, vy, vz });
        World.CreateShape(body);
        return body;

        static double[] stackPosition(double a, double b, double c) => new[] { a, b, c };
    }

    /// <summary>Adds a static body with a shape (a fixed particle).</summary>
    public BodyHandle AddStaticParticle(double x, double y, double z)
    {
        var body = World.CreateBody(BodyType.Static, new[] { x, y, z });
        World.CreateShape(body);
        return body;
    }

    /// <summary>Adds a kinematic particle that moves at a constant velocity.</summary>
    public BodyHandle AddKinematicParticle(double x, double y, double z, float vx, float vy, float vz)
    {
        var body = World.CreateBody(BodyType.Kinematic, new[] { x, y, z }, linearVelocity: new[] { vx, vy, vz });
        World.CreateShape(body);
        return body;
    }

    /// <summary>Adds a distance joint holding the two particles at their current distance. A positive <paramref name="breakImpulse"/> breaks it when the accumulated impulse of a substep exceeds that value.</summary>
    public JointHandle AddDistanceJoint(BodyHandle a, BodyHandle b, float breakImpulse = 0f)
    {
        int ia = World.Resolve(a), ib = World.Resolve(b);
        var pa = World.Bodies.PositionOf(ia);
        var pb = World.Bodies.PositionOf(ib);
        double dx = pb[0] - pa[0], dy = pb[1] - pa[1], dz = pb[2] - pa[2];
        float rest = (float)Math.Sqrt(dx * dx + dy * dy + dz * dz);
        var desc = new ConstraintDesc { Kind = ConstraintKind.Joint, BodyA = a, BodyB = b };
        int c = World.AddConstraint(desc);
        var payload = World.Constraints.Payload(c);
        payload[ParticleStages.JointRest] = rest;
        payload[ParticleStages.JointBreakImpulse] = breakImpulse;
        return World.HandleOfJoint(c);
    }

    /// <summary>
    /// Builds a scene of <paramref name="count"/> particles on a jittered lattice above the floor, tying neighbours in the same row with distance joints, from a seed.
    /// The same seed always gives the same scene.
    /// </summary>
    public static ParticleScene CreatePile(int count, ulong seed, int threadCount = 1, int worldId = 0)
    {
        var scene = new ParticleScene(new WorldOptions { ThreadCount = threadCount, WorldId = worldId });
        var rng = new SplitMix64(seed);
        int side = 1;
        while (side * side * side < count) { side++; }
        float spacing = scene.Radius * 2.2f;
        var handles = new BodyHandle[count];
        for (int i = 0; i < count; i++)
        {
            int x = i % side, z = (i / side) % side, y = i / (side * side);
            double jx = (rng.NextDouble() - 0.5) * 0.05, jy = (rng.NextDouble() - 0.5) * 0.05, jz = (rng.NextDouble() - 0.5) * 0.05;
            handles[i] = scene.AddParticle((x - side / 2) * spacing + jx, scene.Radius + 0.5 + y * spacing + jy, (z - side / 2) * spacing + jz);
        }

        for (int i = 0; i + 1 < count; i++)
        {
            if ((i + 1) % side != 0 && (rng.Next() & 3) == 0) { scene.AddDistanceJoint(handles[i], handles[i + 1]); }
        }

        return scene;
    }
}

/// <summary>A small seeded generator (SplitMix64) so that scenes and tests never depend on a platform random source.</summary>
public struct SplitMix64
{
    private ulong _state;

    /// <summary>Creates a generator from a seed.</summary>
    public SplitMix64(ulong seed) { _state = seed; }

    /// <summary>The next 64 random bits.</summary>
    public ulong Next()
    {
        ulong z = unchecked(_state += 0x9E3779B97F4A7C15UL);
        z = unchecked((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL);
        z = unchecked((z ^ (z >> 27)) * 0x94D049BB133111EBUL);
        return z ^ (z >> 31);
    }

    /// <summary>A double in [0, 1) with 53 random bits.</summary>
    public double NextDouble() => (Next() >> 11) * (1.0 / 9007199254740992.0);
}

/// <summary>The <see cref="IWorldStages"/> of <see cref="ParticleScene"/>.</summary>
public readonly struct ParticleStages : IWorldStages
{
    /// <summary>Payload offset of a joint's accumulated impulse.</summary>
    public const int JointImpulse = 0;

    /// <summary>Payload offset of a joint's rest length.</summary>
    public const int JointRest = 1;

    /// <summary>Payload offset of a joint's prepared direction (three floats).</summary>
    public const int JointNormal = 2;

    /// <summary>Payload offset of a joint's break impulse (0 for unbreakable).</summary>
    public const int JointBreakImpulse = 5;

    private const float Damping = 2f;
    private const float Baumgarte = 0.2f;
    private const float MaxPush = 1f;

    private readonly ParticleScene _scene;

    internal ParticleStages(ParticleScene scene) { _scene = scene; }

    // ---------------------------------------------------------------- contact discovery (serial; every add and remove in ascending pair order)

    private struct Cell
    {
        public long Key;
        public int Body;
    }

    private static long CellKey(double x, double y, double z, double inverseCell)
    {
        long cx = (long)Math.Floor(x * inverseCell), cy = (long)Math.Floor(y * inverseCell), cz = (long)Math.Floor(z * inverseCell);
        return CellOf(cx, cy, cz);
    }

    private static long CellOf(long cx, long cy, long cz) => ((cx & 0x1FFFFF) << 42) | ((cy & 0x1FFFFF) << 21) | (cz & 0x1FFFFF);

    /// <inheritdoc/>
    public void UpdateContacts(WorldCore world)
    {
        var bodies = world.Bodies;
        var cons = world.Constraints;
        var shapes = world.Shapes;
        int nb = bodies.HighWater;
        if (_scene.ShapeOfBody.Length < nb) { _scene.ShapeOfBody = new int[Math.Max(nb, _scene.ShapeOfBody.Length * 2)]; }
        var shapeOf = _scene.ShapeOfBody;
        Array.Fill(shapeOf, -1, 0, nb);
        for (int s = shapes.HighWater - 1; s >= 0; s--)
        {
            if (shapes.Handles.IsLive(s)) { shapeOf[shapes.Owner[s]] = s; }
        }

        double r = _scene.Radius, m = _scene.Margin;
        double addDist = 2 * r + m, removeDist = 2 * r + 2 * m;

        // Existing contacts, ordered by pair key.
        var existing = new List<(long Key, int Id)>();
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (cons.IsLive(c) && cons.Kind[c] == (byte)ConstraintKind.Contact) { existing.Add((((long)cons.BodyA[c] << 32) | (uint)cons.BodyB[c], c)); }
        }

        existing.Sort((x, y) => x.Key.CompareTo(y.Key));
        var doomed = _scene.Doomed;
        doomed.Clear();
        foreach (var (key, id) in existing)
        {
            int a = (int)(key >> 32), b = (int)(key & 0xFFFFFFFF);
            bool aw = Moves(bodies, a), bw = Moves(bodies, b);
            if (!aw && !bw) { continue; }
            if (Dist2(bodies, a, b) > removeDist * removeDist) { doomed.Add(id); }
        }

        foreach (int id in doomed) { world.RemoveConstraint(id); }

        // Sorted grid over every live body; seeds are the bodies that move this step.
        double inv = 1.0 / (4 * r);
        var cells = new List<Cell>(nb);
        for (int b = 0; b < nb; b++)
        {
            if (!bodies.IsLive(b) || shapeOf[b] < 0) { continue; }
            var p = bodies.PositionOf(b);
            cells.Add(new Cell { Key = CellKey(p[0], p[1], p[2], inv), Body = b });
        }

        cells.Sort((x, y) => x.Key != y.Key ? x.Key.CompareTo(y.Key) : x.Body.CompareTo(y.Body));
        var keys = new long[cells.Count];
        for (int i = 0; i < keys.Length; i++) { keys[i] = cells[i].Key; }

        var found = _scene.NewPairs;
        found.Clear();
        foreach (int b in world.AwakeBodies)
        {
            if (shapeOf[b] < 0) { continue; }
            var p = bodies.PositionOf(b);
            long cx = (long)Math.Floor(p[0] * inv), cy = (long)Math.Floor(p[1] * inv), cz = (long)Math.Floor(p[2] * inv);
            for (long dx = -1; dx <= 1; dx++)
            {
                for (long dy = -1; dy <= 1; dy++)
                {
                    for (long dz = -1; dz <= 1; dz++)
                    {
                        long k = CellOf(cx + dx, cy + dy, cz + dz);
                        int at = Array.BinarySearch(keys, k);
                        if (at < 0) { continue; }
                        while (at > 0 && keys[at - 1] == k) { at--; }
                        for (; at < keys.Length && keys[at] == k; at++)
                        {
                            int o = cells[at].Body;
                            if (o == b) { continue; }
                            bool oMoves = Moves(bodies, o);
                            if (oMoves && o < b) { continue; }
                            if (!bodies.IsDynamic(b) && !bodies.IsDynamic(o)) { continue; }
                            if (Dist2(bodies, b, o) >= addDist * addDist) { continue; }
                            int lo = Math.Min(b, o), hi = Math.Max(b, o);
                            found.Add(((long)lo << 32) | (uint)hi);
                        }
                    }
                }
            }
        }

        found.Sort();
        var live = new List<long>();
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (cons.IsLive(c) && cons.Kind[c] == (byte)ConstraintKind.Contact) { live.Add(((long)cons.BodyA[c] << 32) | (uint)cons.BodyB[c]); }
        }

        live.Sort();
        long previous = -1;
        foreach (long key in found)
        {
            if (key == previous) { continue; }
            previous = key;
            if (live.BinarySearch(key) >= 0) { continue; }
            int a = (int)(key >> 32), b = (int)(key & 0xFFFFFFFF);
            var desc = new ConstraintDesc
            {
                Kind = ConstraintKind.Contact,
                BodyA = world.HandleOfBody(a),
                BodyB = world.HandleOfBody(b),
                ShapeA = world.HandleOfShape(shapeOf[a]),
                ShapeB = world.HandleOfShape(shapeOf[b]),
                Flags = _scene.ReportPersist ? ConstraintBits.ReportPersist : ConstraintBits.None,
            };
            world.AddConstraint(desc);
        }
    }

    private static bool Moves(BodyStore bodies, int b) => bodies.TypeOf(b) == BodyType.Kinematic || (bodies.TypeOf(b) == BodyType.Dynamic && bodies.IsAwake(b));

    private static double Dist2(BodyStore bodies, int a, int b)
    {
        var pa = bodies.PositionOf(a); var pb = bodies.PositionOf(b);
        double dx = pb[0] - pa[0], dy = pb[1] - pa[1], dz = pb[2] - pa[2];
        return dx * dx + dy * dy + dz * dz;
    }

    // ---------------------------------------------------------------- per-step stages (parallel-safe)

    /// <inheritdoc/>
    public void Prepare(WorldCore world, ReadOnlySpan<int> constraints, in StepContext context)
    {
        var bodies = world.Bodies;
        var cons = world.Constraints;
        double r2 = 2.0 * _scene.Radius;
        foreach (int c in constraints)
        {
            Direction(bodies, cons.BodyA[c], cons.BodyB[c], out float nx, out float ny, out float nz, out double len);
            var p = cons.Payload(c);
            int o = cons.Kind[c] == (byte)ConstraintKind.Contact ? ManifoldLayout.Normal : JointNormal;
            p[o] = nx; p[o + 1] = ny; p[o + 2] = nz;
            if (cons.Kind[c] == (byte)ConstraintKind.Contact) { p[ManifoldLayout.Separation(0)] = (float)(len - r2); }
        }
    }

    private static void Direction(BodyStore bodies, int a, int b, out float nx, out float ny, out float nz, out double len)
    {
        var pa = bodies.PositionOf(a); var pb = bodies.PositionOf(b);
        double dx = pb[0] - pa[0], dy = pb[1] - pa[1], dz = pb[2] - pa[2];
        len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        if (len > 1e-9) { nx = (float)(dx / len); ny = (float)(dy / len); nz = (float)(dz / len); }
        else { nx = 0f; ny = 1f; nz = 0f; }
    }

    /// <inheritdoc/>
    public void IntegrateVelocities(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context)
    {
        var store = world.Bodies;
        var g = world.Options.Gravity;
        float h = context.SubstepDt;
        float damp = 1f / (1f + h * Damping);
        foreach (int b in bodies)
        {
            if (store.TypeOf(b) != BodyType.Dynamic) { continue; }
            var v = store.LinearVelocityOf(b);
            v[0] = (v[0] + g.X * h) * damp;
            v[1] = (v[1] + g.Y * h) * damp;
            v[2] = (v[2] + g.Z * h) * damp;
        }
    }

    /// <inheritdoc/>
    public void IntegratePositions(WorldCore world, ReadOnlySpan<int> bodies, in StepContext context)
    {
        var store = world.Bodies;
        double h = context.SubstepDt;
        double r = _scene.Radius;
        foreach (int b in bodies)
        {
            var p = store.PositionOf(b);
            var v = store.LinearVelocityOf(b);
            p[0] += v[0] * h; p[1] += v[1] * h; p[2] += v[2] * h;
            if (store.TypeOf(b) == BodyType.Dynamic && p[1] < r)
            {
                // The floor: push out, remove the downward speed and rub off the sideways speed.
                p[1] = r;
                if (v[1] < 0f) { v[1] = 0f; }
                v[0] *= 0.5f; v[2] *= 0.5f;
            }
        }
    }

    /// <inheritdoc/>
    public void SolveBatch(WorldCore world, ReadOnlySpan<int> constraints, SolvePhase phase, in StepContext context)
    {
        if (phase == SolvePhase.Restitution) { return; }
        var bodies = world.Bodies;
        var cons = world.Constraints;
        var inverseMass = bodies.InverseMass;
        float invH = context.InverseSubstepDt;
        double r2 = 2.0 * _scene.Radius;
        foreach (int c in constraints)
        {
            int a = cons.BodyA[c], b = cons.BodyB[c];
            float ia = inverseMass[a], ib = inverseMass[b];
            float k = ia + ib;
            if (!(k > 0f)) { continue; }
            bool contact = cons.Kind[c] == (byte)ConstraintKind.Contact;
            var p = cons.Payload(c);
            int no = contact ? ManifoldLayout.Normal : JointNormal;
            int io = contact ? ManifoldLayout.NormalImpulse(0) : JointImpulse;
            float nx = p[no], ny = p[no + 1], nz = p[no + 2];
            var va = bodies.LinearVelocityOf(a); var vb = bodies.LinearVelocityOf(b);

            float lambda;
            if (phase == SolvePhase.WarmStart)
            {
                lambda = p[io];
            }
            else
            {
                float vn = (vb[0] - va[0]) * nx + (vb[1] - va[1]) * ny + (vb[2] - va[2]) * nz;
                Direction(bodies, a, b, out _, out _, out _, out double len);
                float bias;
                if (contact)
                {
                    float s = (float)(len - r2);
                    bias = s > 0f ? s * invH : (phase == SolvePhase.Solve ? Math.Max(Baumgarte * s * invH, -MaxPush) : 0f);
                }
                else
                {
                    float cdist = (float)(len - p[JointRest]);
                    bias = phase == SolvePhase.Solve ? Baumgarte * cdist * invH : 0f;
                }

                lambda = -(vn + bias) / k;
                float acc = p[io];
                float next = contact ? Math.Max(acc + lambda, 0f) : acc + lambda;
                lambda = next - acc;
                p[io] = next;
            }

            if (ia > 0f) { va[0] -= ia * lambda * nx; va[1] -= ia * lambda * ny; va[2] -= ia * lambda * nz; }
            if (ib > 0f) { vb[0] += ib * lambda * nx; vb[1] += ib * lambda * ny; vb[2] += ib * lambda * nz; }
        }
    }

    /// <inheritdoc/>
    public void StoreResults(WorldCore world, ReadOnlySpan<int> constraints, int eventSlot, in StepContext context)
    {
        var cons = world.Constraints;
        foreach (int c in constraints)
        {
            if (cons.Kind[c] != (byte)ConstraintKind.Joint) { continue; }
            var p = cons.Payload(c);
            float limit = p[JointBreakImpulse];
            float used = Math.Abs(p[JointImpulse]);
            if (limit > 0f && used > limit)
            {
                cons.Flags[c] |= (byte)ConstraintBits.BreakRequested;
                cons.BreakValue[c] = used;
            }
        }
    }

    /// <inheritdoc/>
    public float BodyMotion(WorldCore world, int body)
    {
        var v = world.Bodies.LinearVelocityOf(body);
        return (float)Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
}
