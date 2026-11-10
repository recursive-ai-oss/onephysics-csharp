using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Articulations;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A tree of links in joint coordinates (see <see cref="ArticulationDef"/>): read and write joint positions and velocities, drive joints to targets, aim a point of a link at a place in the world, and ask the dynamics questions
/// (mass matrix, inverse dynamics, Jacobians). A small struct handle like <see cref="Body"/>: it allocates nothing and fails with a <see cref="StaleHandleException"/> once the articulation or any of its links is destroyed.
/// Joint coordinates: a fixed base has none of its own; a floating base has a position and orientation (7 position values, 6 velocity values: angular then linear, in the base frame); each revolute or prismatic joint has one; a spherical joint
/// has a quaternion (4 position values, 3 velocity values: angular velocity in the link frame). Links come in definition order, so the coordinates of link i come after those of link i − 1.
/// </summary>
/// <example><c>var arm = world.AddArticulation(ArticulationDef.RobotArm(), at: new Vector3D(0, 0.5, 0)); arm.SetTaskTarget(arm.LinkCount - 1, default, new Vector3D(0.3, 0.9, 0.2)); world.Step(1f / 60f);</c></example>
public readonly struct Articulation : IEquatable<Articulation>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index, _generation;

    internal Articulation(PhysicsWorld3D world, int index, int generation) { _world = world; _index = index; _generation = generation; }

    private ArticulationState Resolve()
    {
        if (_world == null) { throw new InvalidHandleException("The default Articulation is not attached to a world."); }
        var s = _world.Engine.Articulations;
        if (!s.Handles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Articulation(world " + _world.Id.ToString(CultureInfo.InvariantCulture) + ", #" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ") is stale: the articulation or one of its links was destroyed.");
        }

        return s.States[_index]!;
    }

    private static void Link(ArticulationState s, int link, string name)
    {
        if ((uint)link >= (uint)s.Model.N) { throw new ArgumentOutOfRangeException(name, link, name + " must be within 0.." + (s.Model.N - 1).ToString(CultureInfo.InvariantCulture) + "."); }
    }

    private static void Finite(double v, string name)
    {
        if (!double.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite."); }
    }

    private static void NonNegative(float v, string name)
    {
        if (!(v >= 0f) || !float.IsFinite(v)) { throw new ArgumentOutOfRangeException(name, v, name + " must be finite and at least 0."); }
    }

    /// <summary>True while the articulation exists.</summary>
    public bool IsValid => _world != null && _world.Engine.Articulations.Handles.IsAlive(_index, _generation);

    /// <summary>Number of links, the root included.</summary>
    public int LinkCount => Resolve().Model.N;

    /// <summary>Number of velocity coordinates (degrees of freedom), the floating base's six included.</summary>
    public int VelocityCount => Resolve().Model.TotalDof;

    /// <summary>Number of position coordinates (a quaternion counts four).</summary>
    public int PositionCount => Resolve().Model.TotalPos;

    /// <summary>The body of a link; shapes added to it collide like any other. The root is link 0.</summary>
    /// <param name="link">Link index.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    public Body GetLink(int link)
    {
        var s = Resolve();
        Link(s, link, nameof(link));
        return new Body(_world!, new BodyHandle(_world!.Id, s.Body[link], s.BodyGen[link]));
    }

    /// <summary>True when the base is fixed in the world.</summary>
    public bool FixedBase => Resolve().FixedBase;

    /// <summary>Whether the force that holds the pose against gravity is added to every joint each substep.</summary>
    public bool GravityCompensation
    {
        get => Resolve().GravityCompensation;
        set { using var replayMutation = _world?.RecordMutation("Articulation.GravityCompensation"); Resolve().GravityCompensation = value; }
    }

    /// <summary>Substeps per world step, 1 to 64, or 0 for automatic (see <see cref="ArticulationDef.Substeps"/>).</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside 0..64.</exception>
    public int Substeps
    {
        get => Resolve().Substeps;
        set { using var replayMutation = _world?.RecordMutation("Articulation.Substeps"); var s = Resolve(); if (value < 0 || value > 64) { throw new ArgumentOutOfRangeException(nameof(value), value, "Substeps must be within 0..64."); } s.Substeps = value; }
    }

    /// <summary>Iterations of the contact and joint-limit solve per substep, 0 to 256.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside 0..256.</exception>
    public int Iterations
    {
        get => Resolve().Iterations;
        set { using var replayMutation = _world?.RecordMutation("Articulation.Iterations"); var s = Resolve(); if (value < 0 || value > 256) { throw new ArgumentOutOfRangeException(nameof(value), value, "Iterations must be within 0..256."); } s.Iterations = value; }
    }

    // ------------------------------------------------------------------ joint coordinates

    /// <summary>Copies all position coordinates into <paramref name="destination"/> (<see cref="PositionCount"/> values).</summary>
    /// <exception cref="ArgumentException">The span is shorter than <see cref="PositionCount"/>.</exception>
    public void GetPositions(Span<double> destination)
    {
        var m = Resolve().Model;
        if (destination.Length < m.TotalPos) { throw new ArgumentException("destination needs " + m.TotalPos.ToString(CultureInfo.InvariantCulture) + " values.", nameof(destination)); }
        m.Q.AsSpan().CopyTo(destination);
    }

    /// <summary>Sets all position coordinates and moves the link bodies to match; quaternions are normalised.</summary>
    /// <exception cref="ArgumentException">The span is not <see cref="PositionCount"/> long.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, or a quaternion is zero.</exception>
    public void SetPositions(ReadOnlySpan<double> positions)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetPositions");
        var s = Resolve(); var m = s.Model;
        if (positions.Length != m.TotalPos) { throw new ArgumentException("positions needs " + m.TotalPos.ToString(CultureInfo.InvariantCulture) + " values.", nameof(positions)); }
        for (int i = 0; i < positions.Length; i++) { if (!(Math.Abs(positions[i]) <= 2097152.0)) { throw new ArgumentOutOfRangeException(nameof(positions), positions[i], "position value " + i.ToString(CultureInfo.InvariantCulture) + " is not finite or exceeds ±2^21."); } }
        for (int k = 0; k < m.N; k++)
        {
            int o = m.Kind[k] == JointKind.Spherical ? m.PosOff[k] : m.Kind[k] == JointKind.Free ? m.PosOff[k] + 3 : -1;
            if (o >= 0 && !(positions[o] * positions[o] + positions[o + 1] * positions[o + 1] + positions[o + 2] * positions[o + 2] + positions[o + 3] * positions[o + 3] > 1e-18)) { throw new ArgumentOutOfRangeException(nameof(positions), positions[o], "a joint orientation must be a non-zero quaternion."); }
        }

        positions.CopyTo(m.Q);
        for (int k = 0; k < m.N; k++)
        {
            int o = m.Kind[k] == JointKind.Spherical ? m.PosOff[k] : m.Kind[k] == JointKind.Free ? m.PosOff[k] + 3 : -1;
            if (o < 0) { continue; }
            var q = new Q4(m.Q[o], m.Q[o + 1], m.Q[o + 2], m.Q[o + 3]).Normalized();
            m.Q[o] = q.X; m.Q[o + 1] = q.Y; m.Q[o + 2] = q.Z; m.Q[o + 3] = q.W;
        }

        _world!.Engine.Articulations.Place(_index);
    }

    /// <summary>Copies all velocity coordinates into <paramref name="destination"/> (<see cref="VelocityCount"/> values).</summary>
    /// <exception cref="ArgumentException">The span is shorter than <see cref="VelocityCount"/>.</exception>
    public void GetVelocities(Span<double> destination)
    {
        var m = Resolve().Model;
        if (destination.Length < m.TotalDof) { throw new ArgumentException("destination needs " + m.TotalDof.ToString(CultureInfo.InvariantCulture) + " values.", nameof(destination)); }
        m.Qd.AsSpan().CopyTo(destination);
    }

    /// <summary>Sets all velocity coordinates.</summary>
    /// <exception cref="ArgumentException">The span is not <see cref="VelocityCount"/> long.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or exceeds 1e6.</exception>
    public void SetVelocities(ReadOnlySpan<double> velocities)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetVelocities");
        var s = Resolve(); var m = s.Model;
        if (velocities.Length != m.TotalDof) { throw new ArgumentException("velocities needs " + m.TotalDof.ToString(CultureInfo.InvariantCulture) + " values.", nameof(velocities)); }
        for (int i = 0; i < velocities.Length; i++) { if (!(Math.Abs(velocities[i]) <= 1e6)) { throw new ArgumentOutOfRangeException(nameof(velocities), velocities[i], "velocity value " + i.ToString(CultureInfo.InvariantCulture) + " is not finite or exceeds 1e6."); } }
        velocities.CopyTo(m.Qd);
        _world!.Engine.Articulations.Place(_index);
    }

    /// <summary>The position of a revolute or prismatic joint: radians or metres from its zero.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    /// <exception cref="InvalidOperationException">The joint is not revolute or prismatic.</exception>
    public float GetJointPosition(int link) { var s = Resolve(); OneDof(s, link); return (float)s.Model.Q[s.Model.PosOff[link]]; }

    /// <summary>The velocity of a revolute or prismatic joint.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    /// <exception cref="InvalidOperationException">The joint is not revolute or prismatic.</exception>
    public float GetJointVelocity(int link) { var s = Resolve(); OneDof(s, link); return (float)s.Model.Qd[s.Model.VelOff[link]]; }

    /// <summary>Sets the position of a revolute or prismatic joint and moves the link bodies to match.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or <paramref name="position"/> is not finite.</exception>
    /// <exception cref="InvalidOperationException">The joint is not revolute or prismatic.</exception>
    public void SetJointPosition(int link, float position)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetJointPosition");
        var s = Resolve(); OneDof(s, link); Finite(position, nameof(position));
        s.Model.Q[s.Model.PosOff[link]] = position;
        _world!.Engine.Articulations.Place(_index);
    }

    /// <summary>Sets the velocity of a revolute or prismatic joint.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or <paramref name="velocity"/> is not finite.</exception>
    /// <exception cref="InvalidOperationException">The joint is not revolute or prismatic.</exception>
    public void SetJointVelocity(int link, float velocity)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetJointVelocity");
        var s = Resolve(); OneDof(s, link); Finite(velocity, nameof(velocity));
        s.Model.Qd[s.Model.VelOff[link]] = velocity;
        _world!.Engine.Articulations.Place(_index);
    }

    private static void OneDof(ArticulationState s, int link)
    {
        Link(s, link, nameof(link));
        var k = s.Model.Kind[link];
        if (k != JointKind.Revolute && k != JointKind.Prismatic) { throw new InvalidOperationException("Link " + link.ToString(CultureInfo.InvariantCulture) + " has a " + k + " joint; this call needs a revolute or prismatic one."); }
    }

    // ------------------------------------------------------------------ drives, limits and forces

    /// <summary>Sets the drive of a joint; see <see cref="ArticulationDrive"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or a value is not finite or is negative.</exception>
    public void SetDrive(int link, ArticulationDrive drive)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetDrive");
        var s = Resolve(); Link(s, link, nameof(link));
        NonNegative(drive.Stiffness, "Stiffness"); NonNegative(drive.Damping, "Damping"); NonNegative(drive.MaxForce, "MaxForce"); Finite(drive.TargetPosition, "TargetPosition"); Finite(drive.TargetVelocity, "TargetVelocity");
        var m = s.Model;
        m.Stiff[link] = drive.Stiffness; m.Damp[link] = drive.Damping; m.MaxForce[link] = drive.MaxForce; m.Target[link] = drive.TargetPosition; m.TargetVel[link] = drive.TargetVelocity;
    }

    /// <summary>The drive of a joint.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    public ArticulationDrive GetDrive(int link)
    {
        var s = Resolve(); Link(s, link, nameof(link));
        var m = s.Model;
        return new ArticulationDrive { Stiffness = (float)m.Stiff[link], Damping = (float)m.Damp[link], MaxForce = (float)m.MaxForce[link], TargetPosition = (float)m.Target[link], TargetVelocity = (float)m.TargetVel[link] };
    }

    /// <summary>Sets the position (and velocity) a joint's drive pulls towards, leaving its stiffness and damping.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or a value is not finite.</exception>
    public void SetJointTarget(int link, float position, float velocity = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetJointTarget");
        var s = Resolve(); Link(s, link, nameof(link)); Finite(position, nameof(position)); Finite(velocity, nameof(velocity));
        s.Model.Target[link] = position; s.Model.TargetVel[link] = velocity;
    }

    /// <summary>Sets the orientation a spherical joint's drive pulls towards.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or the quaternion is zero or not finite.</exception>
    /// <exception cref="InvalidOperationException">The joint is not spherical.</exception>
    public void SetJointRotationTarget(int link, QuaternionF target)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetJointRotationTarget");
        var s = Resolve(); Link(s, link, nameof(link));
        if (s.Model.Kind[link] != JointKind.Spherical) { throw new InvalidOperationException("Link " + link.ToString(CultureInfo.InvariantCulture) + " has no spherical joint."); }
        if (!(target.LengthSquared() > 1e-18f) || !float.IsFinite(target.LengthSquared())) { throw new ArgumentOutOfRangeException(nameof(target), target, "a target orientation must be a finite, non-zero quaternion."); }
        s.Model.TargetRot[link] = PhysicsWorld3D.Q(target).Normalized();
    }

    /// <summary>Sets the joint force (newtons or newton-metres) added every substep on top of the drive, until changed. Use it for feed-forward.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or the force is not finite.</exception>
    public void SetJointForce(int link, float force)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetJointForce");
        var s = Resolve(); Link(s, link, nameof(link)); Finite(force, nameof(force));
        s.Model.FeedFwd[link] = force;
    }

    /// <summary>Limits a revolute or prismatic joint to a range (radians or metres); enforced as a constraint, not a spring.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range, a value is not finite, or lower exceeds upper.</exception>
    /// <exception cref="InvalidOperationException">The joint is not revolute or prismatic.</exception>
    public void SetLimits(int link, float lower, float upper)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetLimits");
        var s = Resolve(); OneDof(s, link); Finite(lower, nameof(lower)); Finite(upper, nameof(upper));
        if (lower > upper) { throw new ArgumentOutOfRangeException(nameof(lower), lower, "lower must not exceed upper."); }
        s.Model.Lower[link] = lower; s.Model.Upper[link] = upper; s.Model.HasLimit[link] = true;
    }

    /// <summary>Removes the limits of a joint.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    public void ClearLimits(int link)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.ClearLimits");
        var s = Resolve(); Link(s, link, nameof(link));
        s.Model.HasLimit[link] = false;
    }

    /// <summary>Applies a force at a world point of a link for the next step (the force is consumed by it).</summary>
    /// <param name="link">Link index.</param>
    /// <param name="force">World-axes force, newtons.</param>
    /// <param name="worldPoint">World point of application.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range or a value is not finite.</exception>
    public void ApplyForce(int link, Vector3F force, Vector3D worldPoint)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.ApplyForce");
        var s = Resolve(); Link(s, link, nameof(link));
        Finite(force.X, nameof(force)); Finite(force.Y, nameof(force)); Finite(force.Z, nameof(force)); Finite(worldPoint.X, nameof(worldPoint)); Finite(worldPoint.Y, nameof(worldPoint)); Finite(worldPoint.Z, nameof(worldPoint));
        var m = s.Model; m.ForwardKinematics();
        var rt = m.Rw[link].Transposed();
        var f = new D3(force.X, force.Y, force.Z);
        var arm = new D3(worldPoint.X, worldPoint.Y, worldPoint.Z) - m.Pw[link];
        var fl = rt * f; var nl = rt * D3.Cross(arm, f);
        var e = s.External;
        e[6 * link] += nl.X; e[6 * link + 1] += nl.Y; e[6 * link + 2] += nl.Z; e[6 * link + 3] += fl.X; e[6 * link + 4] += fl.Y; e[6 * link + 5] += fl.Z;
        s.AnyExternal = true;
    }

    // ------------------------------------------------------------------ task space

    /// <summary>
    /// Aims a point of a link at a place in the world: each step a damped-least-squares inverse kinematics solve turns the error into targets for the joints that have a position drive (stiffness above 0), which pull the arm there.
    /// </summary>
    /// <param name="link">The link carrying the point.</param>
    /// <param name="localPoint">The point in the link's frame (default: its origin).</param>
    /// <param name="target">World position to reach.</param>
    /// <param name="gain">Fraction of each solved step taken, above 0 (default 1).</param>
    /// <param name="damping">Damping of the least-squares solve; larger is steadier near singular poses (default 0.05).</param>
    /// <param name="maxStep">Largest joint change per step, radians or metres (default 0.1).</param>
    /// <exception cref="ArgumentOutOfRangeException">An argument is out of range or not finite.</exception>
    public void SetTaskTarget(int link, Vector3F localPoint, Vector3D target, float gain = 1f, float damping = 0.05f, float maxStep = 0.1f)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetTaskTarget");
        var s = Resolve(); Link(s, link, nameof(link));
        Finite(localPoint.X, nameof(localPoint)); Finite(localPoint.Y, nameof(localPoint)); Finite(localPoint.Z, nameof(localPoint)); Finite(target.X, nameof(target)); Finite(target.Y, nameof(target)); Finite(target.Z, nameof(target));
        if (!(gain > 0f) || !float.IsFinite(gain)) { throw new ArgumentOutOfRangeException(nameof(gain), gain, "gain must be finite and above 0."); }
        NonNegative(damping, nameof(damping));
        if (!(maxStep > 0f) || !float.IsFinite(maxStep)) { throw new ArgumentOutOfRangeException(nameof(maxStep), maxStep, "maxStep must be finite and above 0."); }
        s.HasTarget = true; s.TargetLink = link; s.TargetLocal = new D3(localPoint.X, localPoint.Y, localPoint.Z); s.TargetPosition = new D3(target.X, target.Y, target.Z);
        s.IkGain = gain; s.IkDamping = damping; s.IkMaxStep = maxStep;
    }

    /// <summary>Also asks the task target's link to take an orientation (call after <see cref="SetTaskTarget"/>).</summary>
    /// <exception cref="InvalidOperationException">There is no task target.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The quaternion is zero or not finite.</exception>
    public void SetTaskOrientation(QuaternionF orientation)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetTaskOrientation");
        var s = Resolve();
        if (!s.HasTarget) { throw new InvalidOperationException("Set a task target first."); }
        if (!(orientation.LengthSquared() > 1e-18f) || !float.IsFinite(orientation.LengthSquared())) { throw new ArgumentOutOfRangeException(nameof(orientation), orientation, "an orientation must be a finite, non-zero quaternion."); }
        s.TargetOrientation = true; s.TargetRotation = PhysicsWorld3D.Q(orientation).Normalized();
    }

    /// <summary>Stops steering towards the task target; the joint targets stay where they are.</summary>
    public void ClearTaskTarget() { using var replayMutation = _world?.RecordMutation("Articulation.ClearTaskTarget"); var s = Resolve(); s.HasTarget = false; s.TargetOrientation = false; }

    /// <summary>The world position of a point fixed in a link.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    public Vector3D GetPointPosition(int link, Vector3F localPoint)
    {
        var s = Resolve(); Link(s, link, nameof(link));
        var m = s.Model; m.ForwardKinematics();
        var p = m.WorldPoint(link, new D3(localPoint.X, localPoint.Y, localPoint.Z));
        return new Vector3D(p.X, p.Y, p.Z);
    }

    // ------------------------------------------------------------------ dynamics

    private static readonly D3 NoGravity = D3.Zero;

    private D3 WorldGravity() { var g = _world!.Options.Gravity; return new D3(g.X, g.Y, g.Z); }

    /// <summary>The joint-space mass matrix M(q) at the present positions: <see cref="VelocityCount"/> squared values, row-major, symmetric.</summary>
    /// <exception cref="ArgumentException">The span is shorter than that.</exception>
    public void ComputeMassMatrix(Span<double> destination)
    {
        var m = Resolve().Model; int n = m.TotalDof;
        if (destination.Length < n * n) { throw new ArgumentException("destination needs " + (n * n).ToString(CultureInfo.InvariantCulture) + " values.", nameof(destination)); }
        m.ForwardKinematics(); m.MassMatrix(destination);
    }

    /// <summary>
    /// Inverse dynamics (recursive Newton–Euler): the joint forces that produce <paramref name="accelerations"/> at the present positions and velocities, under the world's gravity: τ = M(q) q̈ + C(q, q̇) + g(q).
    /// Pass zeros for the forces that hold the present motion, and zero velocities first for gravity compensation alone (<see cref="ComputeGravityCompensation"/>).
    /// </summary>
    /// <param name="accelerations">Joint accelerations, <see cref="VelocityCount"/> values.</param>
    /// <param name="forces">Receives the joint forces, <see cref="VelocityCount"/> values.</param>
    /// <exception cref="ArgumentException">A span has the wrong length.</exception>
    public void ComputeInverseDynamics(ReadOnlySpan<double> accelerations, Span<double> forces)
    {
        var s = Resolve(); var m = s.Model; int n = m.TotalDof;
        if (accelerations.Length != n) { throw new ArgumentException("accelerations needs " + n.ToString(CultureInfo.InvariantCulture) + " values.", nameof(accelerations)); }
        if (forces.Length < n) { throw new ArgumentException("forces needs " + n.ToString(CultureInfo.InvariantCulture) + " values.", nameof(forces)); }
        m.ForwardKinematics();
        m.InverseDynamics(accelerations, WorldGravity() * s.GravityScale, ReadOnlySpan<double>.Empty, forces);
    }

    /// <summary>The joint forces that hold the present pose still against gravity (inverse dynamics at zero velocity and acceleration).</summary>
    /// <exception cref="ArgumentException">The span is shorter than <see cref="VelocityCount"/>.</exception>
    public void ComputeGravityCompensation(Span<double> forces)
    {
        var s = Resolve(); var m = s.Model; int n = m.TotalDof;
        if (forces.Length < n) { throw new ArgumentException("forces needs " + n.ToString(CultureInfo.InvariantCulture) + " values.", nameof(forces)); }
        m.ForwardKinematics();
        var saved = (double[])m.Qd.Clone();
        Array.Clear(m.Qd);
        m.InverseDynamics(new double[n], WorldGravity() * s.GravityScale, ReadOnlySpan<double>.Empty, forces);
        saved.CopyTo(m.Qd);
    }

    /// <summary>The Jacobian of a point of a link: the world velocity of the point per unit of each joint velocity, 3 rows by <see cref="VelocityCount"/> columns, row-major.</summary>
    /// <param name="link">The link.</param>
    /// <param name="localPoint">The point in the link's frame.</param>
    /// <param name="linear">Receives 3 × <see cref="VelocityCount"/> values.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="link"/> is out of range.</exception>
    /// <exception cref="ArgumentException">The span is too short.</exception>
    public void ComputePointJacobian(int link, Vector3F localPoint, Span<double> linear)
    {
        var s = Resolve(); Link(s, link, nameof(link));
        var m = s.Model; int n = m.TotalDof;
        if (linear.Length < 3 * n) { throw new ArgumentException("linear needs " + (3 * n).ToString(CultureInfo.InvariantCulture) + " values.", nameof(linear)); }
        m.ForwardKinematics();
        m.PointJacobian(link, new D3(localPoint.X, localPoint.Y, localPoint.Z), linear, Span<double>.Empty);
    }

    /// <summary>Kinetic energy of all links, joules.</summary>
    public double KineticEnergy { get { var m = Resolve().Model; m.ForwardKinematics(); return m.KineticEnergy(); } }

    /// <summary>Potential energy of the links in the world's gravity, joules, relative to the origin.</summary>
    public double PotentialEnergy { get { var s = Resolve(); s.Model.ForwardKinematics(); return s.Model.PotentialEnergy(WorldGravity() * s.GravityScale); } }

    /// <summary>The total mass of the links, kg.</summary>
    public double Mass { get { var m = Resolve().Model; double t = 0; for (int i = 0; i < m.N; i++) { t += m.Mass[i]; } return t; } }

    /// <summary>Contact points of the links with other bodies in the last step.</summary>
    public int ContactPointCount => Resolve().ContactPoints;

    /// <summary>The deepest overlap, metres, of any link contact point with another body at the end of the last step (0 when nothing overlaps or nothing touched).</summary>
    public float DeepestPenetration => (float)Resolve().DeepestPenetration;

    // ------------------------------------------------------------------ pose and lifetime

    /// <summary>Moves the base: the placement of a fixed base, or the position and orientation of a floating base (its velocity is kept). The links follow.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or the position exceeds ±2^21.</exception>
    public void SetBasePose(Vector3D position, QuaternionF rotation)
    {
        using var replayMutation = _world?.RecordMutation("Articulation.SetBasePose");
        var s = Resolve(); var m = s.Model;
        if (!(Math.Abs(position.X) <= 2097152.0 && Math.Abs(position.Y) <= 2097152.0 && Math.Abs(position.Z) <= 2097152.0)) { throw new ArgumentOutOfRangeException(nameof(position), position, "The base position must be finite and within ±2^21."); }
        if (!(rotation.LengthSquared() > 1e-18f) || !float.IsFinite(rotation.LengthSquared())) { throw new ArgumentOutOfRangeException(nameof(rotation), rotation, "A rotation must be a finite, non-zero quaternion."); }
        var q = PhysicsWorld3D.Q(rotation).Normalized();
        if (s.FixedBase) { m.SetBasePose(new D3(position.X, position.Y, position.Z), q); }
        else { int po = m.PosOff[0]; m.Q[po] = position.X; m.Q[po + 1] = position.Y; m.Q[po + 2] = position.Z; m.Q[po + 3] = q.X; m.Q[po + 4] = q.Y; m.Q[po + 5] = q.Z; m.Q[po + 6] = q.W; }
        _world!.Engine.Articulations.Place(_index);
    }

    /// <summary>Destroys the articulation and the bodies of its links.</summary>
    /// <exception cref="StaleHandleException">It was already destroyed.</exception>
    public void Destroy()
    {
        using var replayMutation = _world?.RecordMutation("Articulation.Destroy");
        var s = Resolve();
        var bodies = new BodyHandle[s.Body.Length];
        for (int k = 0; k < bodies.Length; k++) { bodies[k] = new BodyHandle(_world!.Id, s.Body[k], s.BodyGen[k]); }
        _world!.Engine.DestroyBody(bodies[0]);
    }

    /// <inheritdoc/>
    public bool Equals(Articulation other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Articulation a && Equals(a);

    /// <inheritdoc/>
    public override int GetHashCode() => (_index * 397) ^ _generation;

    /// <summary>Equality of handles.</summary>
    public static bool operator ==(Articulation a, Articulation b) => a.Equals(b);

    /// <summary>Inequality of handles.</summary>
    public static bool operator !=(Articulation a, Articulation b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => _world == null ? "Articulation(default)" : "Articulation(#" + _index.ToString(CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(CultureInfo.InvariantCulture) + ")";
}
