using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The presets of the happy path: starting values for <see cref="SoftBodyDef"/> that behave like the named material at metre scale.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public enum SoftMaterial
{
    /// <summary>Thin, inextensible, easy to fold; self-collision on. Use with a surface mesh such as <see cref="SoftBodyMesh.Cloth"/>.</summary>
    Cloth = 0,

    /// <summary>A stiff elastic volume or a pressurised shell that springs back fast.</summary>
    Rubber = 1,

    /// <summary>A wobbly volume.</summary>
    Jelly = 2,
}

/// <summary>
/// A soft body in a 3D world: a small struct handle (world, slot, generation) made by <see cref="PhysicsWorld3D.AddSoftBody(SoftBodyMesh, Vector3D, SoftMaterial)"/> or <see cref="PhysicsWorld3D.AddSoftBody(SoftBodyDef)"/>.
/// It allocates nothing and throws <see cref="StaleHandleException"/> once the body is destroyed. Positions are read without allocation, one particle at a time or into a span you own.
/// </summary>
/// <remarks>Particles are numbered like the mesh vertices. There is no callback anywhere on this type.</remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct SoftBody : IEquatable<SoftBody>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal SoftBody(PhysicsWorld3D world, int index, int generation)
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

    /// <summary>The body's slot in its world (see <see cref="PhysicsWorld3D.GetSoftBody"/>).</summary>
    public int Slot => _index;

    /// <summary>True while the body exists.</summary>
    public bool IsValid => _world != null && _world.Engine.SoftHandles.IsAlive(_index, _generation);

    /// <summary>Removes the body from the world; the handle becomes stale.</summary>
    /// <exception cref="StaleHandleException">The body was already destroyed.</exception>
    public void Destroy()
    {
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

    /// <summary>Radius of the spheres the particles collide as, metres.</summary>
    public float ParticleRadius => Resolve().Radius;

    /// <summary>Whether the body is asleep. A sleeping body costs almost nothing; it wakes when an awake rigid body comes near, or on <see cref="WakeUp"/>.</summary>
    public bool IsAsleep => Resolve().Asleep;

    /// <summary>Wakes the body and restarts its sleep timer.</summary>
    public void WakeUp()
    {
        var d = Resolve();
        d.Asleep = false;
        d.SleepTime = 0f;
    }

    /// <summary>Position of a particle in world coordinates.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public Vector3D GetPosition(int particle)
    {
        var d = ResolveParticle(particle, nameof(particle));
        int k = 3 * particle;
        return new Vector3D(d.Pos[k], d.Pos[k + 1], d.Pos[k + 2]);
    }

    /// <summary>Velocity of a particle, m/s.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public Vector3F GetVelocity(int particle)
    {
        var d = ResolveParticle(particle, nameof(particle));
        int k = 3 * particle;
        return new Vector3F(d.Vel[k], d.Vel[k + 1], d.Vel[k + 2]);
    }

    /// <summary>Copies every particle position (world coordinates) into <paramref name="destination"/> without allocating.</summary>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is shorter than <see cref="ParticleCount"/>.</exception>
    public void CopyPositions(Span<Vector3D> destination)
    {
        var d = Resolve();
        if (destination.Length < d.N) { throw new ArgumentException("destination holds " + destination.Length.ToString(CultureInfo.InvariantCulture) + " positions; " + d.N.ToString(CultureInfo.InvariantCulture) + " are needed.", nameof(destination)); }
        for (int i = 0; i < d.N; i++) { destination[i] = new Vector3D(d.Pos[3 * i], d.Pos[3 * i + 1], d.Pos[3 * i + 2]); }
    }

    /// <summary>Copies every particle position relative to <paramref name="origin"/> into <paramref name="destination"/> as float32 (a mesh to draw, say) without allocating. The subtraction is done in double.</summary>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is shorter than <see cref="ParticleCount"/>.</exception>
    public void CopyPositions(Span<Vector3F> destination, Vector3D origin)
    {
        var d = Resolve();
        if (destination.Length < d.N) { throw new ArgumentException("destination holds " + destination.Length.ToString(CultureInfo.InvariantCulture) + " positions; " + d.N.ToString(CultureInfo.InvariantCulture) + " are needed.", nameof(destination)); }
        for (int i = 0; i < d.N; i++) { destination[i] = new Vector3F((float)(d.Pos[3 * i] - origin.X), (float)(d.Pos[3 * i + 1] - origin.Y), (float)(d.Pos[3 * i + 2] - origin.Z)); }
    }

    /// <summary>Axis-aligned bounds of the particle centres (not including the radius).</summary>
    public (Vector3D Min, Vector3D Max) Bounds
    {
        get
        {
            var d = Resolve();
            double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
            for (int i = 0; i < d.N; i++)
            {
                double x = d.Pos[3 * i], y = d.Pos[3 * i + 1], z = d.Pos[3 * i + 2];
                if (x < x0) { x0 = x; } if (x > x1) { x1 = x; }
                if (y < y0) { y0 = y; } if (y > y1) { y1 = y; }
                if (z < z0) { z0 = z; } if (z > z1) { z1 = z; }
            }

            return (new Vector3D(x0, y0, z0), new Vector3D(x1, y1, z1));
        }
    }

    /// <summary>Centre of mass, world coordinates.</summary>
    public Vector3D CenterOfMass
    {
        get
        {
            var d = Resolve();
            double sx = 0, sy = 0, sz = 0, sm = 0;
            for (int i = 0; i < d.N; i++)
            {
                double m = 1.0 / d.InvMass0[i];
                sx += m * d.Pos[3 * i]; sy += m * d.Pos[3 * i + 1]; sz += m * d.Pos[3 * i + 2]; sm += m;
            }

            return new Vector3D(sx / sm, sy / sm, sz / sm);
        }
    }

    /// <summary>Total linear momentum, kg·m/s: the sum of mass times velocity over the free particles.</summary>
    public Vector3D Momentum
    {
        get
        {
            var d = Resolve();
            double sx = 0, sy = 0, sz = 0;
            for (int i = 0; i < d.N; i++)
            {
                if (d.InvMass[i] == 0f) { continue; }
                double m = 1.0 / d.InvMass[i];
                sx += m * d.Vel[3 * i]; sy += m * d.Vel[3 * i + 1]; sz += m * d.Vel[3 * i + 2];
            }

            return new Vector3D(sx, sy, sz);
        }
    }

    /// <summary>The volume enclosed by the current surface, cubic metres: the sum of the signed tetrahedra from the first particle to each surface triangle. Meaningful for a closed surface and for a tetrahedral mesh's boundary.</summary>
    public double Volume
    {
        get
        {
            var d = Resolve();
            double ox = d.Pos[0], oy = d.Pos[1], oz = d.Pos[2], v6 = 0.0;
            var t = d.Tris;
            for (int e = 0; e < t.Length / 3; e++)
            {
                int a = 3 * t[3 * e], b = 3 * t[3 * e + 1], c = 3 * t[3 * e + 2];
                double ax = d.Pos[a] - ox, ay = d.Pos[a + 1] - oy, az = d.Pos[a + 2] - oz;
                double bx = d.Pos[b] - ox, by = d.Pos[b + 1] - oy, bz = d.Pos[b + 2] - oz;
                double cx = d.Pos[c] - ox, cy = d.Pos[c + 1] - oy, cz = d.Pos[c + 2] - oz;
                v6 += ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx);
            }

            return v6 / 6.0;
        }
    }

    /// <summary>The same volume at rest, cubic metres (meaningful where <see cref="Volume"/> is).</summary>
    public double RestVolume
    {
        get
        {
            var d = Resolve();
            double sum = 0.0;
            if (d.Tets.Length > 0) { for (int e = 0; e < d.TetRest.Length; e++) { sum += d.TetRest[e] / 6.0; } }
            else { sum = d.PressureRest; }
            return sum;
        }
    }

    // ------------------------------------------------------------------ skinning

    private SoftBodyData ResolveSkinned()
    {
        var d = Resolve();
        if (d.Def.Skin == null) { throw new InvalidOperationException("This soft body has no skin: set SoftBodyDef.Skin when you create it."); }
        return d;
    }

    /// <summary>True when the body was made with a <see cref="SoftBodyDef.Skin"/>.</summary>
    public bool HasSkin => Resolve().Def.Skin != null;

    /// <summary>Number of joints of the skin's skeleton.</summary>
    /// <exception cref="InvalidOperationException">The body has no skin.</exception>
    public int SkeletonJointCount => ResolveSkinned().Def.Skin!.JointCount;

    /// <summary>
    /// Poses the skeleton the way ragdoll poses are given (<see cref="Ragdoll.SetPose"/>): one rotation per joint, each relative to its parent in the bind pose's axes (the identity is the bind pose), entry 0 turning the whole figure
    /// about the root joint, and <paramref name="at"/> and <paramref name="rotation"/> placing the mesh's own space in the world (the bind pose of a body made at <see cref="SoftBodyDef.Position"/> and <see cref="SoftBodyDef.Rotation"/> is
    /// <c>at</c> = that position and <c>rotation</c> = that rotation). Call it before <see cref="PhysicsWorld3D.Step"/>; the skinned targets move linearly to this pose over the step. Wakes the body if the pose changed.
    /// </summary>
    /// <param name="pose">One finite, non-zero rotation per joint (normalised on use).</param>
    /// <param name="at">Where the mesh space's origin goes, world coordinates, finite and within ±2^21.</param>
    /// <param name="rotation">Orientation of the mesh space; the default (zero) value means no rotation.</param>
    /// <param name="teleport">True to put the particles on their skinned positions at once, with zero velocity, instead of letting the step move the targets (use it to place or reset a body).</param>
    /// <exception cref="InvalidOperationException">The body has no skin.</exception>
    /// <exception cref="ArgumentException">The pose has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry or <paramref name="rotation"/> is not a usable rotation, or <paramref name="at"/> is not finite or beyond ±2^21.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public void SetSkeletonPose(ReadOnlySpan<QuaternionF> pose, Vector3D at, QuaternionF rotation = default, bool teleport = false)
    {
        var d = ResolveSkinned();
        int nj = d.Def.Skin!.JointCount;
        if (pose.Length != nj) { throw new ArgumentException("The pose has " + pose.Length.ToString(CultureInfo.InvariantCulture) + " rotations; the skeleton has " + nj.ToString(CultureInfo.InvariantCulture) + " joints.", nameof(pose)); }
        for (int i = 0; i < nj; i++)
        {
            float l2 = pose[i].LengthSquared();
            if (!float.IsFinite(l2) || !(l2 > 1e-30f)) { throw new ArgumentOutOfRangeException(nameof(pose), pose[i], "Pose entry " + i.ToString(CultureInfo.InvariantCulture) + " must be a finite, non-zero quaternion."); }
        }

        CheckPosition(at, nameof(at));
        QuaternionF qRoot = default;
        if (rotation.X != 0f || rotation.Y != 0f || rotation.Z != 0f || rotation.W != 0f) { qRoot = Unit(rotation, nameof(rotation)); }
        else { qRoot = QuaternionF.Identity; }
        bool changed = false;
        var parents = d.Def.Skin.ParentArray;
        for (int j = 0; j < nj; j++)
        {
            var dj = Unit(pose[j], nameof(pose));
            QuaternionF rj, rp;
            Vector3D pj;
            if (parents[j] < 0)
            {
                rp = qRoot;
                var o = qRoot.Rotate(new Vector3F(d.Bind[0], d.Bind[1], d.Bind[2]));
                pj = new Vector3D(at.X + o.X, at.Y + o.Y, at.Z + o.Z);
            }
            else
            {
                int p = parents[j];
                rp = new QuaternionF(d.JointRot[4 * p], d.JointRot[4 * p + 1], d.JointRot[4 * p + 2], d.JointRot[4 * p + 3]);
                // The joint's bind position carried by the parent: parent position plus the parent's rotation of the bind offset.
                var o = rp.Rotate(new Vector3F(d.Bind[3 * j] - d.Bind[3 * p], d.Bind[3 * j + 1] - d.Bind[3 * p + 1], d.Bind[3 * j + 2] - d.Bind[3 * p + 2]));
                pj = new Vector3D(d.JointPos[3 * p] + o.X, d.JointPos[3 * p + 1] + o.Y, d.JointPos[3 * p + 2] + o.Z);
            }

            rj = Unit(rp * dj, nameof(pose));
            changed |= StoreJoint(d, j, pj, rj);
        }

        Finish(d, changed, teleport);
    }

    /// <summary>Sets the world pose of every joint directly (for an animation system that already has joint transforms): joint j carries a point bound to it from <c>x</c> to <c>Position + Rotation * (x - bindPosition_j)</c>. Otherwise as <see cref="SetSkeletonPose"/>.</summary>
    /// <param name="joints">One pose per joint; positions finite and within ±2^21, rotations finite and non-zero (normalised on use).</param>
    /// <param name="teleport">True to put the particles on their skinned positions at once, with zero velocity.</param>
    /// <exception cref="InvalidOperationException">The body has no skin.</exception>
    /// <exception cref="ArgumentException">The array has the wrong number of entries.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An entry is not usable.</exception>
    /// <exception cref="StaleHandleException">The body was destroyed.</exception>
    public void SetSkeletonTransforms(ReadOnlySpan<SkinJointPose> joints, bool teleport = false)
    {
        var d = ResolveSkinned();
        int nj = d.Def.Skin!.JointCount;
        if (joints.Length != nj) { throw new ArgumentException("The pose has " + joints.Length.ToString(CultureInfo.InvariantCulture) + " joints; the skeleton has " + nj.ToString(CultureInfo.InvariantCulture) + ".", nameof(joints)); }
        for (int j = 0; j < nj; j++)
        {
            CheckPosition(joints[j].Position, nameof(joints));
            float l2 = joints[j].Rotation.LengthSquared();
            if (!float.IsFinite(l2) || !(l2 > 1e-30f)) { throw new ArgumentOutOfRangeException(nameof(joints), joints[j].Rotation, "Joint " + j.ToString(CultureInfo.InvariantCulture) + " needs a finite, non-zero rotation."); }
        }

        bool changed = false;
        for (int j = 0; j < nj; j++) { changed |= StoreJoint(d, j, joints[j].Position, Unit(joints[j].Rotation, nameof(joints))); }
        Finish(d, changed, teleport);
    }

    /// <summary>The skinned position of a particle for the pose last set (not where the particle is). Zero allocations.</summary>
    /// <exception cref="InvalidOperationException">The body has no skin.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public Vector3D GetSkinnedPosition(int particle)
    {
        var d = ResolveSkinned();
        if ((uint)particle >= (uint)d.N) { throw new ArgumentOutOfRangeException(nameof(particle), particle, "particle index " + particle.ToString(CultureInfo.InvariantCulture) + " is outside 0 to " + (d.N - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        d.ComputeTargets(d.JointPos, d.JointRot, d.TgtCur, null, particle, particle + 1);
        return new Vector3D(d.TgtCur[3 * particle], d.TgtCur[3 * particle + 1], d.TgtCur[3 * particle + 2]);
    }

    /// <summary>Copies every particle's skinned position for the pose last set into <paramref name="destination"/> without allocating.</summary>
    /// <exception cref="InvalidOperationException">The body has no skin.</exception>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is shorter than <see cref="ParticleCount"/>.</exception>
    public void CopySkinnedPositions(Span<Vector3D> destination)
    {
        var d = ResolveSkinned();
        if (destination.Length < d.N) { throw new ArgumentException("destination holds " + destination.Length.ToString(CultureInfo.InvariantCulture) + " positions; " + d.N.ToString(CultureInfo.InvariantCulture) + " are needed.", nameof(destination)); }
        d.ComputeTargets(d.JointPos, d.JointRot, d.TgtCur, null);
        for (int i = 0; i < d.N; i++) { destination[i] = new Vector3D(d.TgtCur[3 * i], d.TgtCur[3 * i + 1], d.TgtCur[3 * i + 2]); }
    }

    private static void CheckPosition(Vector3D p, string name)
    {
        const double Bound = 2097152.0;
        if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || Math.Abs(p.X) > Bound || Math.Abs(p.Y) > Bound || Math.Abs(p.Z) > Bound)
        {
            throw new ArgumentOutOfRangeException(name, p, "Position " + p + " is outside the supported range of ±2^21 (2097152).");
        }
    }

    private static QuaternionF Unit(QuaternionF q, string name)
    {
        float l = float.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
        if (!(l > 1e-15f) || !float.IsFinite(l)) { throw new ArgumentOutOfRangeException(name, q, "The rotation " + q + " is not a usable (finite, non-zero) quaternion."); }
        return new QuaternionF(q.X / l, q.Y / l, q.Z / l, q.W / l);
    }

    private static bool StoreJoint(SoftBodyData d, int j, Vector3D p, QuaternionF q)
    {
        bool changed = d.JointPos[3 * j] != p.X || d.JointPos[3 * j + 1] != p.Y || d.JointPos[3 * j + 2] != p.Z
            || d.JointRot[4 * j] != q.X || d.JointRot[4 * j + 1] != q.Y || d.JointRot[4 * j + 2] != q.Z || d.JointRot[4 * j + 3] != q.W;
        d.JointPos[3 * j] = p.X; d.JointPos[3 * j + 1] = p.Y; d.JointPos[3 * j + 2] = p.Z;
        d.JointRot[4 * j] = q.X; d.JointRot[4 * j + 1] = q.Y; d.JointRot[4 * j + 2] = q.Z; d.JointRot[4 * j + 3] = q.W;
        return changed;
    }

    private void Finish(SoftBodyData d, bool changed, bool teleport)
    {
        if (teleport)
        {
            d.JointPos.CopyTo(d.JointPosPrev, 0);
            d.JointRot.CopyTo(d.JointRotPrev, 0);
            d.ComputeTargets(d.JointPos, d.JointRot, d.TgtCur, null);
            Array.Copy(d.TgtCur, d.Pos, d.Pos.Length);
            Array.Copy(d.TgtCur, d.Prev, d.Prev.Length);
            Array.Clear(d.Vel);
            WakeUp();
        }
        else if (changed) { WakeUp(); }
    }

    /// <summary>Holds a particle where it is (infinite mass) or releases it. A pinned particle can still be moved with <see cref="SetPosition"/>. Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public void SetPinned(int particle, bool pinned)
    {
        var d = ResolveParticle(particle, nameof(particle));
        d.Pinned[particle] = pinned ? (byte)1 : (byte)0;
        d.RefreshInvMass(particle);
        d.Vel[3 * particle] = 0f; d.Vel[3 * particle + 1] = 0f; d.Vel[3 * particle + 2] = 0f;
        WakeUp();
    }

    /// <summary>Whether a particle is pinned.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public bool IsPinned(int particle) => ResolveParticle(particle, nameof(particle)).Pinned[particle] != 0;

    /// <summary>Moves a particle to a position (a kinematic handle for a pinned particle, or a teleport). Its velocity is set to zero. Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is out of range or the position is not finite or beyond ±2^21.</exception>
    public void SetPosition(int particle, Vector3D position)
    {
        var d = ResolveParticle(particle, nameof(particle));
        const double Bound = 2097152.0;
        if (!double.IsFinite(position.X) || !double.IsFinite(position.Y) || !double.IsFinite(position.Z) || Math.Abs(position.X) > Bound || Math.Abs(position.Y) > Bound || Math.Abs(position.Z) > Bound)
        {
            throw new ArgumentOutOfRangeException(nameof(position), position, "Position " + position + " is outside the supported range of ±2^21 (2097152).");
        }

        int k = 3 * particle;
        d.Pos[k] = position.X; d.Pos[k + 1] = position.Y; d.Pos[k + 2] = position.Z;
        d.Prev[k] = position.X; d.Prev[k + 1] = position.Y; d.Prev[k + 2] = position.Z;
        d.Vel[k] = 0f; d.Vel[k + 1] = 0f; d.Vel[k + 2] = 0f;
        WakeUp();
    }

    /// <summary>Gives a particle an impulse (a velocity change of impulse times its inverse mass). Wakes the body. Pinned particles ignore it.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is out of range or the impulse is not finite.</exception>
    public void ApplyImpulse(int particle, Vector3F impulse)
    {
        var d = ResolveParticle(particle, nameof(particle));
        if (!float.IsFinite(impulse.X) || !float.IsFinite(impulse.Y) || !float.IsFinite(impulse.Z)) { throw new ArgumentOutOfRangeException(nameof(impulse), impulse, "impulse must be finite."); }
        float w = d.InvMass[particle];
        int k = 3 * particle;
        d.Vel[k] += impulse.X * w; d.Vel[k + 1] += impulse.Y * w; d.Vel[k + 2] += impulse.Z * w;
        WakeUp();
    }

    /// <summary>Adds a velocity to every free particle (a push or a throw). Wakes the body.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The velocity is not finite.</exception>
    public void AddVelocity(Vector3F velocity)
    {
        var d = Resolve();
        if (!float.IsFinite(velocity.X) || !float.IsFinite(velocity.Y) || !float.IsFinite(velocity.Z)) { throw new ArgumentOutOfRangeException(nameof(velocity), velocity, "velocity must be finite."); }
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            d.Vel[3 * i] += velocity.X; d.Vel[3 * i + 1] += velocity.Y; d.Vel[3 * i + 2] += velocity.Z;
        }

        WakeUp();
    }

    /// <summary>Compliance of the edge (stretch) constraints, m/N; 0 to 10^6. Takes effect on the next step and wakes the body.</summary>
    public float EdgeCompliance { get => Resolve().Def.EdgeCompliance; set { var d = Resolve(); Check(value, nameof(EdgeCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.EdgeCompliance = value; WakeUp(); } }

    /// <summary>Compliance of the bend constraints, m/N; 0 to 10^6.</summary>
    public float BendCompliance { get => Resolve().Def.BendCompliance; set { var d = Resolve(); Check(value, nameof(BendCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.BendCompliance = value; WakeUp(); } }

    /// <summary>Compliance of the volume and pressure constraints; 0 to 10^6.</summary>
    public float VolumeCompliance { get => Resolve().Def.VolumeCompliance; set { var d = Resolve(); Check(value, nameof(VolumeCompliance), 0f, SoftBodyDef.MaxCompliance); d.Def.VolumeCompliance = value; WakeUp(); } }

    /// <summary>The enclosed volume a closed surface tries to hold as a multiple of its rest volume; 0 turns pressure off.</summary>
    /// <exception cref="SoftBodyException">Setting a value above 0 on a mesh that is not a closed surface.</exception>
    public float InternalPressure
    {
        get => Resolve().Def.InternalPressure;
        set
        {
            var d = Resolve();
            Check(value, nameof(InternalPressure), 0f, 100f);
            if (value > 0f && d.PressureRest <= 0f) { throw new SoftBodyException(SoftBodyError.NotClosed, "InternalPressure " + value.ToString("R", CultureInfo.InvariantCulture) + " needs a closed, consistently oriented triangle surface.", nameof(value)); }
            d.Def.InternalPressure = value;
            WakeUp();
        }
    }

    /// <summary>Coulomb friction coefficient against shapes; 0 to 100.</summary>
    public float Friction { get => Resolve().Def.Friction; set { var d = Resolve(); Check(value, nameof(Friction), 0f, 100f); d.Def.Friction = value; } }

    /// <summary>Multiplier on gravity.</summary>
    public float GravityScale { get => Resolve().Def.GravityScale; set { var d = Resolve(); Check(value, nameof(GravityScale), -1e6f, 1e6f); d.Def.GravityScale = value; WakeUp(); } }

    private static void Check(float v, string name, float lo, float hi)
    {
        if (!float.IsFinite(v) || v < lo || v > hi) { throw new ArgumentOutOfRangeException(name, v, name + " is " + v.ToString("R", CultureInfo.InvariantCulture) + "; expected a finite value from " + lo.ToString("R", CultureInfo.InvariantCulture) + " to " + hi.ToString("R", CultureInfo.InvariantCulture) + "."); }
    }

    // ---- raw tier: the storage itself, valid until the next step, restore or destroy

    /// <summary>The raw particle positions: x, y, z doubles per particle, world coordinates. Read-only; use <see cref="SetPosition"/> to move a particle.</summary>
    public ReadOnlySpan<double> RawPositions => Resolve().Pos;

    /// <summary>The raw particle velocities: x, y, z floats per particle.</summary>
    public ReadOnlySpan<float> RawVelocities => Resolve().Vel;

    /// <summary>The raw inverse masses (0 for a pinned particle).</summary>
    public ReadOnlySpan<float> RawInverseMasses => Resolve().InvMass;

    /// <summary>The first particle of each edge constraint; edges are ordered by (smaller index, larger index) and every edge appears once.</summary>
    public ReadOnlySpan<int> RawEdgeA => Resolve().EdgeA;

    /// <summary>The second particle of each edge constraint (larger than the first).</summary>
    public ReadOnlySpan<int> RawEdgeB => Resolve().EdgeB;

    /// <summary>The rest length of each edge constraint, metres.</summary>
    public ReadOnlySpan<float> RawEdgeRestLengths => Resolve().EdgeRest;

    /// <summary>The bend constraints: four particle indices each, the two particles of the shared edge and then the two far corners.</summary>
    public ReadOnlySpan<int> RawBendParticles => Resolve().BendIdx;

    /// <summary>The volume constraints: four particle indices per tetrahedron.</summary>
    public ReadOnlySpan<int> RawTetrahedra => Resolve().Tets;

    /// <summary>Six times the rest volume of each tetrahedron, cubic metres.</summary>
    public ReadOnlySpan<float> RawTetrahedronRestVolumes6 => Resolve().TetRest;

    /// <summary>The surface triangles (three particle indices each) for drawing: the mesh's own triangles, or the boundary of a tetrahedral mesh.</summary>
    public ReadOnlySpan<int> SurfaceTriangles => Resolve().Tris;

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
