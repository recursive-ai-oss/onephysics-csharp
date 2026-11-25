using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Destruction;
using RecursiveAI.OnePhysics.Presentation;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PresentationWorld3D
{
    // One followed authoritative body. Kept in a list in the order the bodies were followed, which is the order poses are written.
    private sealed class Mirror
    {
        public PhysicsWorld3D Authority = null!;
        public BodyHandle Source;
        public Body Body;
        public bool IsStatic;
        public bool HasTarget;
        public Vector3D TargetPosition;
        public QuaternionF TargetRotation;
    }

    private readonly List<Mirror> _mirrors = new();
    private readonly List<(int Slot, int Generation)> _pinned = new();
    private long[] _seq = new long[16];
    private int[] _seqGeneration = new int[16];
    private long _nextSeq = 1;

    /// <summary>Number of followed bodies.</summary>
    public int MirrorCount => _mirrors.Count;

    /// <summary>Number of bodies of type Dynamic the <see cref="PresentationQuality.MaxBodies"/> budget counts: every body of that type in the follower's world except the pinned ones.</summary>
    public int CosmeticBodyCount
    {
        get
        {
            int n = 0;
            var bodies = _world.Core.Bodies;
            for (int b = 0; b < bodies.HighWater; b++) { if (bodies.IsDynamic(b) && !IsPinned(b)) n++; }
            return n;
        }
    }

    private int FindMirror(PhysicsWorld3D authority, BodyHandle source)
    {
        for (int i = 0; i < _mirrors.Count; i++) { var m = _mirrors[i]; if (ReferenceEquals(m.Authority, authority) && m.Source == source) return i; }
        return -1;
    }

    /// <summary>
    /// Follows an authoritative body: a body with copies of its shapes appears in the follower (static for a static body, kinematic otherwise) at the same pose, and from then on <see cref="Observe"/> moves it to the authoritative pose.
    /// Sensors are not copied; geometry that is edited in place later (voxel edits) is not tracked. Following a body that is already followed returns its mirror.
    /// </summary>
    /// <returns>The mirror, a body of <see cref="World"/>: attach cosmetics to it (rods, joints) or let debris collide with it.</returns>
    /// <exception cref="InvalidHandleException">The body is stale, or the default <see cref="Body"/>.</exception>
    /// <exception cref="ArgumentException">The body belongs to the follower's own world.</exception>
    public Body Follow(Body authority)
    {
        var aw = authority.World ?? throw new InvalidHandleException("The default Body is not attached to a world.");
        if (ReferenceEquals(aw, _world)) throw new ArgumentException("A follower cannot follow its own bodies.", nameof(authority));
        int ab = aw.ResolveBody(authority.RawHandle);
        int existing = FindMirror(aw, authority.RawHandle);
        if (existing >= 0) return _mirrors[existing].Body;
        var type = aw.Core.Bodies.TypeOf(ab) == BodyType.Static ? BodyType.Static : BodyType.Kinematic;
        var position = authority.Position;
        CheckPose(position);
        var body = _world.CreateBody(type, position, authority.Rotation);
        try
        {
            var e = aw.Engine;
            for (int s = e.ShapeHead[ab]; s >= 0; s = e.NextShape[s])
            {
                if ((e.ShapeFlags[s] & Engine3D.FlagSensor) != 0) continue;
                var local = e.Local[s];
                var def = new ShapeDef
                {
                    Density = e.Density[s], Friction = e.Friction[s], Restitution = e.Restitution[s], RollingResistance = e.Rolling[s],
                    CategoryBits = e.Category[s], MaskBits = e.Mask[s], GroupIndex = e.Group[s], Layer = e.ShapeLayer[s],
                    LocalPosition = local.Translation, LocalRotation = local.Rotation,
                };
                _world.Engine.AddShape(body.RawHandle, (byte[])e.Geoms[e.GeomOf[s]]!.Recipe.Clone(), in def);
            }
        }
        catch { body.Destroy(); throw; }

        _mirrors.Add(new Mirror { Authority = aw, Source = authority.RawHandle, Body = body, IsStatic = type == BodyType.Static });
        return body;
    }

    /// <summary>Stops following a body and destroys its mirror. Returns false when it was not followed.</summary>
    public bool Unfollow(Body authority)
    {
        var aw = authority.World;
        if (aw == null) return false;
        int i = FindMirror(aw, authority.RawHandle);
        if (i < 0) return false;
        RemoveMirror(i);
        return true;
    }

    /// <summary>The mirror of an authoritative body, if it is followed.</summary>
    public bool TryGetMirror(Body authority, out Body mirror)
    {
        mirror = default;
        var aw = authority.World;
        if (aw == null) return false;
        int i = FindMirror(aw, authority.RawHandle);
        if (i < 0) return false;
        mirror = _mirrors[i].Body;
        return true;
    }

    private void RemoveMirror(int i)
    {
        var m = _mirrors[i];
        _mirrors.RemoveAt(i);
        if (m.Body.IsValid) m.Body.Destroy();
    }

    /// <summary>Keeps a body of type Dynamic out of the <see cref="PresentationQuality.MaxBodies"/> budget: it is neither counted nor culled (a cosmetic ragdoll, a prop the host manages itself).</summary>
    /// <exception cref="InvalidHandleException">The body is stale or not in this follower's world.</exception>
    public void Pin(Body body)
    {
        if (body.World != _world) throw new ArgumentException("The body is not in the follower's world.", nameof(body));
        var h = body.RawHandle; _world.ResolveBody(h);
        if (!_pinned.Contains((h.Index, h.Generation))) _pinned.Add((h.Index, h.Generation));
    }

    /// <summary>Makes a pinned body count against the budget again. Does nothing when it was not pinned.</summary>
    public void Unpin(Body body) { var h = body.RawHandle; _pinned.Remove((h.Index, h.Generation)); }

    private bool IsPinned(int slot)
    {
        if (_pinned.Count == 0) return false;
        int gen = _world.Core.Bodies.Handles.GenerationOf(slot);
        return _pinned.Contains((slot, gen));
    }

    private static void CheckPose(Vector3D p)
    {
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || Math.Abs(p.X) > PoseBound || Math.Abs(p.Y) > PoseBound || Math.Abs(p.Z) > PoseBound)
            throw new ArgumentOutOfRangeException(nameof(p), p, "A followed body's position " + p + " is outside the supported range of +-2^21 (2097152).");
    }

    // ------------------------------------------------------------------ observing the authority

    /// <summary>
    /// Reads the authoritative world after its step: notes the pose of every followed body of it (applied by the next <see cref="Step(float)"/>), follows or removes bodies and spawns cosmetics for its fracture events and mapped events.
    /// The authority is only read. Changes the follower between steps; with a recorder attached they are written as a snapshot before the next step.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="authority"/> is null.</exception>
    /// <exception cref="ArgumentException">The authority is the follower's own world.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A followed body is outside the supported position range.</exception>
    public void Observe(PhysicsWorld3D authority)
    {
        ArgumentNullException.ThrowIfNull(authority);
        if (ReferenceEquals(authority, _world)) throw new ArgumentException("A follower cannot observe its own world.", nameof(authority));
        // Poses first, validated before any is kept, so a rejected body leaves the notes as they were.
        for (int i = 0; i < _mirrors.Count; i++)
        {
            var m = _mirrors[i];
            if (!ReferenceEquals(m.Authority, authority) || m.IsStatic || !authority.IsBodyAlive(m.Source)) continue;
            CheckPose(authority.GetBody(m.Source).Position);
        }

        // Fractures first: the destroyed source's mirror is still there to be replaced by its fragments.
        ObserveFractures(authority);
        for (int i = _mirrors.Count - 1; i >= 0; i--)
        {
            var m = _mirrors[i];
            if (!ReferenceEquals(m.Authority, authority)) continue;
            if (!authority.IsBodyAlive(m.Source) || !m.Body.IsValid) RemoveMirror(i);   // the authoritative body is gone (destroyed, fractured) or the host destroyed the mirror
        }

        for (int i = 0; i < _mirrors.Count; i++)
        {
            var m = _mirrors[i];
            if (!ReferenceEquals(m.Authority, authority) || m.IsStatic) continue;
            var b = authority.GetBody(m.Source);
            m.TargetPosition = b.Position; m.TargetRotation = b.Rotation; m.HasTarget = true;
        }

        ObserveEvents(authority);
    }

    private void ObserveFractures(PhysicsWorld3D authority)
    {
        var events = authority.FractureEvents;
        for (int i = 0; i < events.Length; i++)
        {
            var ev = events[i];
            if (FractureMapper != null) { FractureMapper(this, authority, in ev); continue; }
            DefaultFracture(authority, in ev, i);
        }
    }

    private void ObserveEvents(PhysicsWorld3D authority)
    {
        var handler = EventMapper;
        var mask = MappedEvents;
        if (handler == null || mask == EventMask.None) return;
        var events = authority.Events;
        for (int i = 0; i < events.Length; i++)
        {
            var e = events[i];
            if (((int)mask & (1 << (int)e.Kind)) == 0) continue;
            handler(this, authority, in e);
        }
    }

    /// <summary>The default response to a fracture: the followed source and its mirror are replaced by mirrors of the authoritative fragments (when <see cref="PresentationDebris.FollowFragments"/>), and a burst of shards flies out at the impact point.</summary>
    private void DefaultFracture(PhysicsWorld3D authority, in FractureEvent ev, int index)
    {
        int src = FindMirror(authority, ev.Source);
        var fragments = authority.FractureFragments;
        var mean = default(Vector3F);
        int alive = 0;
        for (int k = 0; k < ev.FragmentCount && ev.FirstFragment + k < fragments.Length; k++)
        {
            var f = fragments[ev.FirstFragment + k];
            if (!f.IsValid) continue;
            var v = f.LinearVelocity; mean = new Vector3F(mean.X + v.X, mean.Y + v.Y, mean.Z + v.Z); alive++;
        }

        if (alive > 0) mean = new Vector3F(mean.X / alive, mean.Y / alive, mean.Z / alive);
        if (src >= 0)
        {
            RemoveMirror(src);
            if (_debris.FollowFragments)
            {
                for (int k = 0; k < ev.FragmentCount && ev.FirstFragment + k < fragments.Length; k++)
                {
                    var f = fragments[ev.FirstFragment + k];
                    if (f.IsValid && f.Type != BodyType.Static) Follow(f);
                }
            }
        }

        SpawnShards(ev.Point, mean, (ulong)index);
    }

    /// <summary>
    /// Spawns the default burst of cosmetic shards around <paramref name="at"/>: <see cref="PresentationDebris.Count"/> small boxes with deterministic offsets and velocities (the outward speed plus <paramref name="baseVelocity"/>).
    /// The stream is keyed by <see cref="PresentationDebris.Seed"/>, the follower's step count and <paramref name="key"/>, so equal calls at equal steps give equal shards on every platform.
    /// </summary>
    /// <returns>The shards, oldest first.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="at"/> or <paramref name="baseVelocity"/> is not finite, or <paramref name="at"/> is beyond +-2^21.</exception>
    public Body[] SpawnShards(Vector3D at, Vector3F baseVelocity, ulong key = 0)
    {
        CheckPose(at);
        if (!float.IsFinite(baseVelocity.X) || !float.IsFinite(baseVelocity.Y) || !float.IsFinite(baseVelocity.Z)) throw new ArgumentOutOfRangeException(nameof(baseVelocity), baseVelocity, "The velocity must be finite.");
        var d = _debris;
        var result = new Body[d.Count];
        var rng = new SeedRng(d.Seed ^ ((ulong)_world.StepCount * 0x9E3779B97F4A7C15UL) ^ (key * 0xD1B54A32D192ED03UL));
        for (int k = 0; k < result.Length; k++)
        {
            // A point in the ball of radius Radius by rejection from the cube (at most 16 tries; the origin if none lands), then the launch direction from the same offset.
            double ox = 0, oy = 0, oz = 0;
            for (int t = 0; t < 16; t++)
            {
                double x = 2 * rng.NextDouble() - 1, y = 2 * rng.NextDouble() - 1, z = 2 * rng.NextDouble() - 1;
                if (x * x + y * y + z * z <= 1.0) { ox = x; oy = y; oz = z; break; }
            }

            double len = double.Sqrt(ox * ox + oy * oy + oz * oz);
            double dx = 0, dy = 1, dz = 0;
            if (len > 1e-6) { dx = ox / len; dy = oy / len; dz = oz / len; }
            double speed = d.Speed * (0.5 + rng.NextDouble());
            var pos = new Vector3D(at.X + ox * d.Radius, at.Y + oy * d.Radius, at.Z + oz * d.Radius);
            CheckPose(pos);
            var shard = _world.AddBox(new Vector3F(d.Size, d.Size, d.Size), pos, density: d.Density);
            shard.SetLinearVelocity(new Vector3F(baseVelocity.X + (float)(dx * speed), baseVelocity.Y + (float)(dy * speed), baseVelocity.Z + (float)(dz * speed)));
            Stamp(shard.RawHandle.Index);
            result[k] = shard;
        }

        return result;
    }

    // ------------------------------------------------------------------ budgets

    private void Stamp(int slot)
    {
        if (slot >= _seq.Length) { Array.Resize(ref _seq, Math.Max(slot + 1, _seq.Length * 2)); Array.Resize(ref _seqGeneration, _seq.Length); }
        _seq[slot] = _nextSeq++;
        _seqGeneration[slot] = _world.Core.Bodies.Handles.GenerationOf(slot);
    }

    private struct Candidate { public int Slot; public float Mass; public long Seq; }

    private void Maintain()
    {
        // Mirrors whose authoritative body or follower body is gone are cleaned up by Observe; a host that never observes still gets its own destroyed mirrors dropped here.
        for (int i = _mirrors.Count - 1; i >= 0; i--) if (!_mirrors[i].Body.IsValid) _mirrors.RemoveAt(i);
        ApplyQuality();
        CullBodies();
    }

    private void CullBodies()
    {
        var bodies = _world.Core.Bodies;
        int limit = _quality.MaxBodies;
        List<Candidate>? list = null;
        int count = 0;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsDynamic(b) || IsPinned(b)) continue;
            // A body nobody stamped (added by the host) is numbered when first seen, in slot order.
            if (b >= _seq.Length || _seqGeneration[b] != bodies.Handles.GenerationOf(b)) Stamp(b);
            count++;
        }

        if (count <= limit) return;
        list = new List<Candidate>(count);
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsDynamic(b) || IsPinned(b)) continue;
            list.Add(new Candidate { Slot = b, Mass = _world.Engine.Mass[b], Seq = _seq[b] });
        }

        // Lightest first, then oldest, then lowest slot: a total order, so the same bodies go on every platform.
        list.Sort(static (x, y) =>
        {
            int c = x.Mass.CompareTo(y.Mass);
            if (c != 0) return c;
            c = x.Seq.CompareTo(y.Seq);
            return c != 0 ? c : x.Slot.CompareTo(y.Slot);
        });
        for (int i = 0; i < count - limit; i++) _world.GetBody(_world.Core.HandleOfBody(list[i].Slot)).Destroy();
    }

    private int[] _rodGeneration = Array.Empty<int>(), _rodSubsteps = Array.Empty<int>(), _rodIterations = Array.Empty<int>();

    private void ApplyQuality()
    {
        var q = _quality;
        var options = _world.Options;
        if (q.SoftSubsteps != 0 && options.SoftSubsteps != q.SoftSubsteps) options.SoftSubsteps = q.SoftSubsteps;
        if (q.SoftIterations != 0 && options.SoftIterations != q.SoftIterations) options.SoftIterations = q.SoftIterations;
        var engine = _world.Engine;
        if (engine.HasParticleFluids)
        {
            var settings = _world.FluidSettings;
            if (settings.MaxParticles != q.FluidBudget) { settings.MaxParticles = q.FluidBudget; _world.FluidSettings = settings; }
            var store = engine.ParticleFluids;
            if (store.Count > q.FluidBudget) store.CullOldest(q.FluidBudget);
        }

        var rods = engine.Rods;
        if (rods.Data.Length != 0 || _rodGeneration.Length != 0)
        {
            if (_rodGeneration.Length < rods.Data.Length) { Array.Resize(ref _rodGeneration, rods.Data.Length); Array.Resize(ref _rodSubsteps, rods.Data.Length); Array.Resize(ref _rodIterations, rods.Data.Length); }
            for (int i = 0; i < rods.Data.Length; i++)
            {
                var d = rods.Data[i];
                if (d == null) { _rodGeneration[i] = 0; continue; }
                int gen = rods.Generations[i] + 1;   // 0 marks "not recorded"
                if (_rodGeneration[i] != gen) { _rodGeneration[i] = gen; _rodSubsteps[i] = d.Substeps; _rodIterations[i] = d.Iterations; }
                d.Substeps = q.RodSubsteps == 0 ? _rodSubsteps[i] : Math.Min(_rodSubsteps[i], q.RodSubsteps);
                d.Iterations = q.RodIterations == 0 ? _rodIterations[i] : Math.Min(_rodIterations[i], q.RodIterations);
            }
        }
    }
}
