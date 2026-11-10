using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>
/// A soft body in a 2D world: a small struct handle (world, slot, generation) made by <see cref="PhysicsWorld2D.AddSoftBody(SoftBodyMesh, Vector2D, SoftMaterial)"/> or <see cref="PhysicsWorld2D.AddSoftBody(SoftBodyDef)"/>.
/// It allocates nothing and throws <see cref="StaleHandleException"/> once the body is destroyed. Positions are read without allocation, one particle at a time or into a span you own.
/// </summary>
/// <remarks>Particles are numbered like the mesh vertices. There is no callback anywhere on this type.</remarks>
public readonly struct SoftBody : IEquatable<SoftBody>
{
    private readonly PhysicsWorld2D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal SoftBody(PhysicsWorld2D world, int index, int generation)
    {
        _world = world;
        _index = index;
        _generation = generation;
    }

    private SoftBodyData Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default SoftBody is not attached to a world."); }
        var e = _world.Engine;
        if (!e.SoftHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("SoftBody(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the soft body was destroyed.");
        }

        return e.SoftData[_index]!;
    }

    private SoftBodyData ResolveParticle(int particle, string name)
    {
        var d = Resolve();
        if ((uint)particle >= (uint)d.N) { throw new ArgumentOutOfRangeException(name, particle, "particle index " + particle.ToString(CultureInfo.InvariantCulture) + " is outside 0 to " + (d.N - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        return d;
    }

    /// <summary>The body's slot in its world (see <see cref="PhysicsWorld2D.GetSoftBody"/>).</summary>
    public int Slot => _index;

    /// <summary>True while the body exists.</summary>
    public bool IsValid => _world != null && _world.Engine.SoftHandles.IsAlive(_index, _generation);

    /// <summary>Removes the body from the world; the handle becomes stale.</summary>
    /// <exception cref="StaleHandleException">The body was already destroyed.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.Destroy");
        Resolve();
        _world!.Engine.RemoveSoftBody(_index);
    }

    /// <summary>A copy of the definition the body was made from, with the current pins, compliances and other settings (not the particles' current positions).</summary>
    public SoftBodyDef Definition
    {
        get
        {
            var d = Resolve();
            var c = d.Def.Clone();
            var pins = new List<int>();
            for (int i = 0; i < d.N; i++) { if (d.Pinned[i] != 0) { pins.Add(i); } }
            c.PinnedParticles = pins;
            return c;
        }
    }

    /// <summary>The rest mesh.</summary>
    public SoftBodyMesh Mesh => Resolve().Def.Mesh!;

    /// <summary>Number of particles (the mesh's vertex count).</summary>
    public int ParticleCount => Resolve().N;

    /// <summary>Total mass of the body, kilograms.</summary>
    public float Mass => Resolve().TotalMass;

    /// <summary>Radius of the discs the particles collide as, metres.</summary>
    public float ParticleRadius => Resolve().Radius;

    /// <summary>Whether the body is asleep. A sleeping body costs almost nothing; it wakes when an awake rigid body comes near, or on <see cref="WakeUp"/>.</summary>
    public bool IsAsleep => Resolve().Asleep;

    /// <summary>Wakes the body and restarts its sleep timer.</summary>
    public void WakeUp()
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.WakeUp");
        var d = Resolve();
        d.Asleep = false;
        d.SleepTime = 0f;
    }

    /// <summary>Position of a particle in world coordinates.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public Vector2D GetPosition(int particle)
    {
        var d = ResolveParticle(particle, nameof(particle));
        return new Vector2D(d.Pos[2 * particle], d.Pos[2 * particle + 1]);
    }

    /// <summary>Velocity of a particle, m/s.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public Vector2F GetVelocity(int particle)
    {
        var d = ResolveParticle(particle, nameof(particle));
        return new Vector2F(d.Vel[2 * particle], d.Vel[2 * particle + 1]);
    }

    /// <summary>Copies every particle position (world coordinates) into <paramref name="destination"/> without allocating.</summary>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is shorter than <see cref="ParticleCount"/>.</exception>
    public void CopyPositions(Span<Vector2D> destination)
    {
        var d = Resolve();
        if (destination.Length < d.N) { throw new ArgumentException("destination holds " + destination.Length.ToString(CultureInfo.InvariantCulture) + " positions; " + d.N.ToString(CultureInfo.InvariantCulture) + " are needed.", nameof(destination)); }
        for (int i = 0; i < d.N; i++) { destination[i] = new Vector2D(d.Pos[2 * i], d.Pos[2 * i + 1]); }
    }

    /// <summary>Copies every particle position relative to <paramref name="origin"/> into <paramref name="destination"/> as float32 (an outline to draw, say) without allocating. The subtraction is done in double.</summary>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is shorter than <see cref="ParticleCount"/>.</exception>
    public void CopyPositions(Span<Vector2F> destination, Vector2D origin)
    {
        var d = Resolve();
        if (destination.Length < d.N) { throw new ArgumentException("destination holds " + destination.Length.ToString(CultureInfo.InvariantCulture) + " positions; " + d.N.ToString(CultureInfo.InvariantCulture) + " are needed.", nameof(destination)); }
        for (int i = 0; i < d.N; i++) { destination[i] = new Vector2F((float)(d.Pos[2 * i] - origin.X), (float)(d.Pos[2 * i + 1] - origin.Y)); }
    }

    /// <summary>Axis-aligned bounds of the particle centres (not including the radius).</summary>
    public (Vector2D Min, Vector2D Max) Bounds
    {
        get
        {
            var d = Resolve();
            double x0 = double.MaxValue, y0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue;
            for (int i = 0; i < d.N; i++)
            {
                double x = d.Pos[2 * i], y = d.Pos[2 * i + 1];
                if (x < x0) { x0 = x; } if (x > x1) { x1 = x; }
                if (y < y0) { y0 = y; } if (y > y1) { y1 = y; }
            }

            return (new Vector2D(x0, y0), new Vector2D(x1, y1));
        }
    }

    /// <summary>Centre of mass, world coordinates.</summary>
    public Vector2D CenterOfMass
    {
        get
        {
            var d = Resolve();
            double sx = 0, sy = 0, sm = 0;
            for (int i = 0; i < d.N; i++)
            {
                double m = 1.0 / d.InvMass0[i];
                sx += m * d.Pos[2 * i]; sy += m * d.Pos[2 * i + 1]; sm += m;
            }

            return new Vector2D(sx / sm, sy / sm);
        }
    }

    /// <summary>Total linear momentum, kg·m/s: the sum of mass times velocity over the free particles.</summary>
    public Vector2D Momentum
    {
        get
        {
            var d = Resolve();
            double sx = 0, sy = 0;
            for (int i = 0; i < d.N; i++)
            {
                if (d.InvMass[i] == 0f) { continue; }
                double m = 1.0 / d.InvMass[i];
                sx += m * d.Vel[2 * i]; sy += m * d.Vel[2 * i + 1];
            }

            return new Vector2D(sx, sy);
        }
    }

    /// <summary>The area enclosed by the current outline, square metres (the shoelace sum). Meaningful for a mesh with a closed outline.</summary>
    public double Area
    {
        get
        {
            var d = Resolve();
            var o = d.Outline;
            if (o.Length < 3) { return 0.0; }
            double ox = d.Pos[2 * o[0]], oy = d.Pos[2 * o[0] + 1], twice = 0.0;
            for (int i = 0; i < o.Length; i++)
            {
                int a = o[i], b = o[(i + 1) % o.Length];
                double ax = d.Pos[2 * a] - ox, ay = d.Pos[2 * a + 1] - oy, bx = d.Pos[2 * b] - ox, by = d.Pos[2 * b + 1] - oy;
                twice += ax * by - ay * bx;
            }

            return 0.5 * twice;
        }
    }

    /// <summary>The same area at rest (scaled mesh), square metres.</summary>
    public double RestArea
    {
        get
        {
            var d = Resolve();
            var o = d.Outline;
            if (o.Length < 3) { return 0.0; }
            double ox = d.Rest[2 * o[0]], oy = d.Rest[2 * o[0] + 1], twice = 0.0;
            for (int i = 0; i < o.Length; i++)
            {
                int a = o[i], b = o[(i + 1) % o.Length];
                double ax = d.Rest[2 * a] - ox, ay = d.Rest[2 * a + 1] - oy, bx = d.Rest[2 * b] - ox, by = d.Rest[2 * b + 1] - oy;
                twice += ax * by - ay * bx;
            }

            return 0.5 * twice;
        }
    }

    // ------------------------------------------------------------------ kinematic targets (the 2D counterpart of skinning)

    private static void CheckPosition(Vector2D p, string name)
    {
        const double Bound = 2097152.0;
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || Math.Abs(p.X) > Bound || Math.Abs(p.Y) > Bound)
        {
            throw new ArgumentOutOfRangeException(name, p, "Position " + p + " is outside the supported range of ±2^21 (2097152).");
        }
    }

    /// <summary>
    /// Gives a particle a target position: the particle is held within <paramref name="maxDistance"/> of it (a hard limit that wins over contacts, like the maximum-distance attachments of animated cloth), 0 making the particle
    /// kinematic (it sits on the target and collides with nothing). Targets move linearly from where they were at the last step to the new ones across the step's substeps. Wakes the body.
    /// </summary>
    /// <param name="particle">Index of the particle.</param>
    /// <param name="position">The target, world coordinates, finite and within ±2^21.</param>
    /// <param name="maxDistance">Finite, 0 to 10^6 metres.</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public void SetTarget(int particle, Vector2D position, float maxDistance = 0f)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.SetTarget");
        var d = ResolveParticle(particle, nameof(particle));
        CheckPosition(position, nameof(position));
        Check(maxDistance, nameof(maxDistance), 0f, 1e6f);
        PlaceTarget(d, particle, position, maxDistance, false);
        WakeUp();
    }

    private static void PlaceTarget(SoftBodyData d, int particle, Vector2D position, float maxDistance, bool teleport)
    {
        d.EnsureTargets();
        int k = 2 * particle;
        if (d.TargetOn[particle] == 0 || teleport) { d.TargetPrev[k] = teleport ? position.X : d.Pos[k]; d.TargetPrev[k + 1] = teleport ? position.Y : d.Pos[k + 1]; }
        d.TargetOn[particle] = 1;
        d.TargetMax[particle] = maxDistance;
        d.TargetPos[k] = position.X; d.TargetPos[k + 1] = position.Y;
        d.RefreshInvMass(particle);
        if (teleport)
        {
            d.Pos[k] = position.X; d.Pos[k + 1] = position.Y; d.Prev[k] = position.X; d.Prev[k + 1] = position.Y;
            d.Vel[k] = 0f; d.Vel[k + 1] = 0f;
        }
    }

    /// <summary>Removes a particle's target; the particle is free again (or pinned, if it is pinned).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public void ClearTarget(int particle)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.ClearTarget");
        var d = ResolveParticle(particle, nameof(particle));
        if (!d.HasTargets) { return; }
        d.TargetOn[particle] = 0;
        d.RefreshInvMass(particle);
        WakeUp();
    }

    /// <summary>Whether a particle has a target.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public bool HasTarget(int particle) { var d = ResolveParticle(particle, nameof(particle)); return d.HasTargets && d.TargetOn[particle] != 0; }

    /// <summary>
    /// Targets every particle at the rest shape placed at <paramref name="at"/> and rotated by <paramref name="angleRadians"/> (the mesh's own space placed in the world, as <see cref="SoftBodyDef.Position"/> and <see cref="SoftBodyDef.Angle"/>
    /// place it at creation): the body follows a driven pose, as a skinned character follows its skeleton. Call it before <see cref="PhysicsWorld2D.Step"/>.
    /// </summary>
    /// <param name="at">Where the mesh origin goes, world coordinates, finite and within ±2^21.</param>
    /// <param name="angleRadians">Finite.</param>
    /// <param name="maxDistance">Distance limit of every particle, 0 (kinematic) to 10^6.</param>
    /// <param name="teleport">True to put the particles on their targets at once with zero velocity (to place or reset a body).</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range.</exception>
    public void SetTargetPose(Vector2D at, float angleRadians, float maxDistance = 0f, bool teleport = false)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.SetTargetPose");
        var d = Resolve();
        CheckPosition(at, nameof(at));
        if (!float.IsFinite(angleRadians)) { throw new ArgumentOutOfRangeException(nameof(angleRadians), angleRadians, "angleRadians must be finite."); }
        Check(maxDistance, nameof(maxDistance), 0f, 1e6f);
        var (sn, cs) = Math64.SinCos(angleRadians);
        float c = (float)cs, s = (float)sn;
        for (int i = 0; i < d.N; i++)
        {
            float rx = c * d.Rest[2 * i] - s * d.Rest[2 * i + 1], ry = s * d.Rest[2 * i] + c * d.Rest[2 * i + 1];
            PlaceTarget(d, i, new Vector2D(at.X + rx, at.Y + ry), maxDistance, teleport);
        }

        WakeUp();
    }

    /// <summary>Holds a particle where it is (infinite mass) or releases it. A pinned particle can still be moved with <see cref="SetPosition"/>. Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public void SetPinned(int particle, bool pinned)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.SetPinned");
        var d = ResolveParticle(particle, nameof(particle));
        d.Pinned[particle] = pinned ? (byte)1 : (byte)0;
        d.RefreshInvMass(particle);
        d.Vel[2 * particle] = 0f; d.Vel[2 * particle + 1] = 0f;
        WakeUp();
    }

    /// <summary>Whether a particle is pinned.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public bool IsPinned(int particle) => ResolveParticle(particle, nameof(particle)).Pinned[particle] != 0;

    /// <summary>Moves a particle to a position (a kinematic handle for a pinned particle, or a teleport). Its velocity is set to zero. Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is out of range or the position is not finite or beyond ±2^21.</exception>
    public void SetPosition(int particle, Vector2D position)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.SetPosition");
        var d = ResolveParticle(particle, nameof(particle));
        CheckPosition(position, nameof(position));
        int k = 2 * particle;
        d.Pos[k] = position.X; d.Pos[k + 1] = position.Y;
        d.Prev[k] = position.X; d.Prev[k + 1] = position.Y;
        d.Vel[k] = 0f; d.Vel[k + 1] = 0f;
        WakeUp();
    }

    /// <summary>Gives a particle an impulse (a velocity change of impulse times its inverse mass). Wakes the body. Pinned particles ignore it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is out of range or the impulse is not finite.</exception>
    public void ApplyImpulse(int particle, Vector2F impulse)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.ApplyImpulse");
        var d = ResolveParticle(particle, nameof(particle));
        if (!float.IsFinite(impulse.X) || !float.IsFinite(impulse.Y)) { throw new ArgumentOutOfRangeException(nameof(impulse), impulse, "impulse must be finite."); }
        float w = d.InvMass[particle];
        int k = 2 * particle;
        d.Vel[k] += impulse.X * w; d.Vel[k + 1] += impulse.Y * w;
        WakeUp();
    }

    /// <summary>Adds a velocity to every free particle (a push or a throw). Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The velocity is not finite.</exception>
    public void AddVelocity(Vector2F velocity)
    {
        using var replayMutation = _world?.RecordMutation("SoftBody.AddVelocity");
        var d = Resolve();
        if (!float.IsFinite(velocity.X) || !float.IsFinite(velocity.Y)) { throw new ArgumentOutOfRangeException(nameof(velocity), velocity, "velocity must be finite."); }
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            d.Vel[2 * i] += velocity.X; d.Vel[2 * i + 1] += velocity.Y;
        }

        WakeUp();
    }

    /// <summary>Compliance of the edge (stretch) constraints, m/N; 0 to 10^6. Takes effect on the next step and wakes the body.</summary>
    public float EdgeCompliance { get => Resolve().Def.EdgeCompliance; set { using var replayMutation = _world?.RecordMutation("SoftBody.EdgeCompliance"); var d = Resolve(); Check(value, nameof(EdgeCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.EdgeCompliance = value; WakeUp(); } }

    /// <summary>Compliance of the bend constraints, m/N; 0 to 10^6.</summary>
    public float BendCompliance { get => Resolve().Def.BendCompliance; set { using var replayMutation = _world?.RecordMutation("SoftBody.BendCompliance"); var d = Resolve(); Check(value, nameof(BendCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.BendCompliance = value; WakeUp(); } }

    /// <summary>Compliance of the area and pressure constraints; 0 to 10^6.</summary>
    public float AreaCompliance { get => Resolve().Def.AreaCompliance; set { using var replayMutation = _world?.RecordMutation("SoftBody.AreaCompliance"); var d = Resolve(); Check(value, nameof(AreaCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.AreaCompliance = value; WakeUp(); } }

    /// <summary>Compliance of shape matching, seconds squared; 0 to 10^6.</summary>
    public float MatchCompliance { get => Resolve().Def.MatchCompliance; set { using var replayMutation = _world?.RecordMutation("SoftBody.MatchCompliance"); var d = Resolve(); Check(value, nameof(MatchCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.MatchCompliance = value; WakeUp(); } }

    /// <summary>The enclosed area a closed outline tries to hold as a multiple of its rest area; 0 turns pressure off.</summary>
    /// <exception cref="SoftBodyException">Setting a value above 0 on a mesh that is not a closed outline.</exception>
    public float InternalPressure
    {
        get => Resolve().Def.InternalPressure;
        set
        {
            using var replayMutation = _world?.RecordMutation("SoftBody.InternalPressure");
            var d = Resolve();
            Check(value, nameof(InternalPressure), 0f, 100f);
            if (value > 0f && d.PressureRest <= 0f) { throw new SoftBodyException(SoftBodyError.NotClosed, "InternalPressure " + value.ToString("R", CultureInfo.InvariantCulture) + " needs a closed outline that winds counter-clockwise.", nameof(value)); }
            d.Def.InternalPressure = value;
            WakeUp();
        }
    }

    /// <summary>Coulomb friction coefficient against shapes; 0 to 100.</summary>
    public float Friction { get => Resolve().Def.Friction; set { using var replayMutation = _world?.RecordMutation("SoftBody.Friction"); var d = Resolve(); Check(value, nameof(Friction), 0f, 100f); d.Def.Friction = value; } }

    /// <summary>Multiplier on gravity.</summary>
    public float GravityScale { get => Resolve().Def.GravityScale; set { using var replayMutation = _world?.RecordMutation("SoftBody.GravityScale"); var d = Resolve(); Check(value, nameof(GravityScale), -1e6f, 1e6f); d.Def.GravityScale = value; WakeUp(); } }

    private static void Check(float v, string name, float lo, float hi)
    {
        if (!float.IsFinite(v) || v < lo || v > hi) { throw new ArgumentOutOfRangeException(name, v, name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected a finite value from " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + "."); }
    }

    /// <summary>Whether the body's boundary stops other bodies' particles (and its own, with self-collision); see <see cref="SoftBodyDef.SurfaceCollision"/>. Takes effect on the next step.</summary>
    public bool SurfaceCollision { get => Resolve().Def.SurfaceCollision; set { using var replayMutation = _world?.RecordMutation("SoftBody.SurfaceCollision"); var d = Resolve(); d.Def.SurfaceCollision = value; WakeUp(); } }

    /// <summary>Strain at which an edge tears (0 turns it off); see <see cref="SoftBodyDef.TearStrain"/>.</summary>
    public float TearStrain { get => Resolve().Def.TearStrain; set { using var replayMutation = _world?.RecordMutation("SoftBody.TearStrain"); var d = Resolve(); Check(value, nameof(TearStrain), 0f, 100f); d.Def.TearStrain = value; WakeUp(); } }

    /// <summary>Constraint force at which an edge tears, newtons (0 turns it off); see <see cref="SoftBodyDef.TearForce"/>.</summary>
    public float TearForce { get => Resolve().Def.TearForce; set { using var replayMutation = _world?.RecordMutation("SoftBody.TearForce"); var d = Resolve(); Check(value, nameof(TearForce), 0f, 1e12f); d.Def.TearForce = value; WakeUp(); } }

    /// <summary>Plastic yield strain (0 turns plasticity off); see <see cref="SoftBodyDef.PlasticYield"/>.</summary>
    public float PlasticYield { get => Resolve().Def.PlasticYield; set { using var replayMutation = _world?.RecordMutation("SoftBody.PlasticYield"); var d = Resolve(); Check(value, nameof(PlasticYield), 0f, 10f); d.Def.PlasticYield = value; WakeUp(); } }

    /// <summary>Fraction of the strain beyond the yield strain the rest shape takes up each step, 0 to 1.</summary>
    public float PlasticCreep { get => Resolve().Def.PlasticCreep; set { using var replayMutation = _world?.RecordMutation("SoftBody.PlasticCreep"); var d = Resolve(); Check(value, nameof(PlasticCreep), 0f, 1f); d.Def.PlasticCreep = value; } }

    /// <summary>Largest plastic strain, 0 to 10.</summary>
    public float PlasticMax { get => Resolve().Def.PlasticMax; set { using var replayMutation = _world?.RecordMutation("SoftBody.PlasticMax"); var d = Resolve(); Check(value, nameof(PlasticMax), 0f, 10f); d.Def.PlasticMax = value; } }

    /// <summary>Number of edges that have torn and still hold two sides together without a constraint (edges whose two sides came apart are free edges again and are not counted).</summary>
    public int BrokenEdgeCount => Resolve().Broken.Length;

    /// <summary>The rest length of every edge as plasticity has left it (the same order as <see cref="RawEdgeA"/>); equal to <see cref="RawEdgeRestLengths"/>.</summary>
    public ReadOnlySpan<float> RawCurrentEdgeRestLengths => Resolve().EdgeRest;

    // ---- raw tier: the storage itself, valid until the next step, restore or destroy

    /// <summary>The raw particle positions: x, y doubles per particle, world coordinates. Read-only; use <see cref="SetPosition"/> to move a particle.</summary>
    public ReadOnlySpan<double> RawPositions => Resolve().Pos;

    /// <summary>The raw particle velocities: x, y floats per particle.</summary>
    public ReadOnlySpan<float> RawVelocities => Resolve().Vel;

    /// <summary>The raw inverse masses (0 for a pinned or kinematic particle).</summary>
    public ReadOnlySpan<float> RawInverseMasses => Resolve().InvMass;

    /// <summary>The first particle of each edge constraint; edges are ordered by (smaller index, larger index) and every edge appears once.</summary>
    public ReadOnlySpan<int> RawEdgeA => Resolve().EdgeA;

    /// <summary>The second particle of each edge constraint (larger than the first).</summary>
    public ReadOnlySpan<int> RawEdgeB => Resolve().EdgeB;

    /// <summary>The rest length of each edge constraint, metres.</summary>
    public ReadOnlySpan<float> RawEdgeRestLengths => Resolve().EdgeRest;

    /// <summary>The triangles (three particle indices each; empty for an outline-only mesh).</summary>
    public ReadOnlySpan<int> RawTriangles => Resolve().Tris;

    /// <summary>Twice the rest area of each triangle, square metres.</summary>
    public ReadOnlySpan<float> RawTriangleRestAreas2 => Resolve().TriRest2;

    /// <summary>The outline particles in counter-clockwise order (for drawing the skin).</summary>
    public ReadOnlySpan<int> Outline => Resolve().Outline;

    /// <inheritdoc/>
    public bool Equals(SoftBody other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is SoftBody s && Equals(s);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Equality.</summary>
    public static bool operator ==(SoftBody a, SoftBody b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(SoftBody a, SoftBody b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => "SoftBody(#" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}
