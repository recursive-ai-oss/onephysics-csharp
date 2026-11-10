using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Articulations;

/// <summary>One contact point of a link with something else, as the engine's narrowphase made it: where it is on the link, which way it pushes the other body, and what the other body is.</summary>
internal struct HostContact
{
    public int Constraint;
    /// <summary>The body of the articulation that makes the contact.</summary>
    public int LinkBody;
    public int Other;
    /// <summary>The contact point on the link, world coordinates.</summary>
    public D3 Point;
    /// <summary>The position of the link body (its centre of mass) and the contact point's offset from it. The articulation works from these two, not from <see cref="Point"/>, so a world shifted by a whole number of grid steps gives the same bits.</summary>
    public D3 LinkPosition, PointArm;
    /// <summary>Unit normal pointing from the link towards the other body.</summary>
    public D3 Normal;
    /// <summary>From the other body's centre of mass to the contact point on it.</summary>
    public D3 OtherArm;
    public double Separation, Friction;
}

/// <summary>What the articulation store needs from an engine. Planar engines present their bodies as 3D vectors with z = 0 and take back only the components in the plane.</summary>
internal interface IArticulationHost
{
    WorldCore Core { get; }
    bool Planar { get; }
    int EngineSectionId { get; }
    ArticulationStore Articulations { get; }

    /// <summary>World gravity (acceleration, m/s²).</summary>
    D3 Gravity { get; }

    /// <summary>Sets a link body's pose (the link frame's origin and orientation) and its velocity; the body's frame is the link frame.</summary>
    void SetLinkState(int body, D3 origin, Q4 rotation, D3 omega, D3 vOrigin);

    /// <summary>Sets only a link body's velocity.</summary>
    void SetLinkVelocity(int body, D3 omega, D3 vOrigin);

    /// <summary>
    /// Appends every contact point that involves a link body (<see cref="ArticulationStore.ArticulationOfBody"/> at least 0), in ascending constraint order and then point order; a contact between two links of different articulations
    /// appears once for each. Sensors and pairs without points are left out. Kinematic bodies have no constraint edges, so this scans the constraint table once.
    /// </summary>
    void GatherLinkContacts(List<HostContact> into);

    /// <summary>
    /// Appends the contact points of a dynamic body with static bodies (the ground it stands on, the wall it leans against): <see cref="HostContact.Normal"/> points from the static body towards <paramref name="body"/> (the way the
    /// contact pushes it), <see cref="HostContact.OtherArm"/> is from the body's centre of mass to the point, <see cref="HostContact.Other"/> the static body. Ascending constraint order, then point order.
    /// </summary>
    void GatherSupportContacts(int body, List<HostContact> into);

    /// <summary>Centre-of-mass position, linear and angular velocity of a body.</summary>
    void BodyMotion(int body, out D3 position, out D3 velocity, out D3 omega);

    /// <summary>Inverse mass and world-axes inverse inertia of a body; zeros unless it is dynamic.</summary>
    void BodyInverse(int body, out double invMass, out M3 invInertia);

    /// <summary>Adds to a dynamic body's velocities and wakes it when it sleeps.</summary>
    void AddBodyVelocity(int body, D3 dv, D3 dw);

    /// <summary>A step's contact speculation: contacts within this distance take part; also the push-out cap.</summary>
    double ContactSlop { get; }
    double PushOutVelocity { get; }
}

/// <summary>An articulation as the world keeps it: the model, the bodies that carry its links' shapes, its settings, and the task-space target.</summary>
internal sealed class ArticulationState
{
    public ArticulationModel Model = null!;
    /// <summary>Scratch for the step (not state; never copied by a fork or saved).</summary>
    public ArticulationWork? Work;
    public int[] Body = Array.Empty<int>(), BodyGen = Array.Empty<int>();
    public bool Planar;
    public bool FixedBase;
    public int Substeps, Iterations = 8;
    public bool SelfCollision, GravityCompensation;
    public double GravityScale = 1.0;
    // Task-space target (damped least squares on joint targets).
    public bool HasTarget;
    public int TargetLink;
    public D3 TargetLocal, TargetPosition;
    public bool TargetOrientation;
    public Q4 TargetRotation = Q4.Identity;
    public double IkGain = 1.0, IkDamping = 0.05, IkMaxStep = 0.1;
    /// <summary>Spatial forces applied by the host between steps (torque and force at each link origin, link axes), consumed by the next step.</summary>
    public double[] External = Array.Empty<double>();
    public bool AnyExternal;
    // Readback of the last step (not saved).
    public int ContactPoints;
    public double DeepestPenetration;
    public Debug.DebugArticulationContact[] LastContacts = Array.Empty<Debug.DebugArticulationContact>();
    public int LastContactCount;
    public double[] JointForce = Array.Empty<double>();
}

/// <summary>
/// Reduced-coordinate articulated bodies in both dimensions, one store shared by both engines (a planar world presents its bodies as 3D vectors with z = 0). An articulation is a tree of links joined by revolute, prismatic, spherical and fixed
/// joints, simulated in joint coordinates so it cannot come apart. Each link is a kinematic body in the world, so shapes, filters, queries, events and the narrowphase see it like any other; the store moves it from the joint coordinates and turns the
/// contacts the narrowphase makes with it into constraints on the joint velocities, solved together with joint limits and joint friction, with the other bodies' velocities updated in the same loop. It runs as a serial stage after the contact stage,
/// articulations in slot order; state lives in an optional snapshot section.
/// </summary>
internal sealed partial class ArticulationStore : ISnapshotSection
{
    /// <summary>Section id: "ARTI" read as a little-endian integer.</summary>
    public const int Id = 0x49545241;

    /// <summary>Most links in one articulation.</summary>
    public const int MaxLinks = 2048;

    private readonly IArticulationHost _host;

    public HandleTable Handles = new();
    public ArticulationState?[] States = new ArticulationState?[4];
    private int[] _artOfBody = Array.Empty<int>(), _linkOfBody = Array.Empty<int>();

    public ArticulationStore(IArticulationHost host) { _host = host; }

    public int Count => Handles.Count;

    public bool HasWork => Handles.Count > 0;

    // ------------------------------------------------------------------ body index

    private static void Index(ref int[] map, int body, int value)
    {
        if (body >= map.Length)
        {
            int old = map.Length;
            Array.Resize(ref map, Math.Max(body + 1, Math.Max(8, old * 2)));
            for (int i = old; i < map.Length; i++) { map[i] = -1; }
        }

        map[body] = value;
    }

    /// <summary>The articulation a body is a link of, or -1.</summary>
    public int ArticulationOfBody(int body) => (uint)body < (uint)_artOfBody.Length ? _artOfBody[body] : -1;

    /// <summary>The link index of a body inside its articulation, or -1.</summary>
    public int LinkOfBody(int body) => (uint)body < (uint)_linkOfBody.Length ? _linkOfBody[body] : -1;

    /// <summary>Whether two bodies are links of one articulation that does not collide with itself, or are adjacent links (neighbours never collide).</summary>
    public bool SameArticulationBlocks(int bodyA, int bodyB)
    {
        int a = ArticulationOfBody(bodyA);
        if (a < 0 || a != ArticulationOfBody(bodyB)) { return false; }
        var s = States[a]!;
        if (!s.SelfCollision) { return true; }
        int la = _linkOfBody[bodyA], lb = _linkOfBody[bodyB];
        return s.Model.Parent[la] == lb || s.Model.Parent[lb] == la;
    }

    // ------------------------------------------------------------------ creation

    /// <summary>Registers an articulation whose link bodies already exist (kinematic bodies in link order) and returns its slot.</summary>
    public int Create(ArticulationModel model, ReadOnlySpan<BodyHandleRef> bodies, ArticulationState settings)
    {
        int i = Handles.Allocate();
        if (i >= States.Length) { Array.Resize(ref States, Math.Max(i + 1, States.Length * 2)); }
        settings.Model = model;
        settings.Body = new int[model.N]; settings.BodyGen = new int[model.N];
        for (int k = 0; k < model.N; k++)
        {
            settings.Body[k] = bodies[k].Index; settings.BodyGen[k] = bodies[k].Generation;
            Index(ref _artOfBody, bodies[k].Index, i); Index(ref _linkOfBody, bodies[k].Index, k);
        }

        settings.External = new double[6 * model.N];
        settings.JointForce = new double[model.TotalDof];
        States[i] = settings;
        return i;
    }

    /// <summary>Registers an articulation made from a validated blueprint and puts its link bodies at the poses its joint coordinates give.</summary>
    public int CreateFrom(ArticulationBlueprint bp, ArticulationModel model, ReadOnlySpan<BodyHandleRef> bodies)
    {
        var st = new ArticulationState
        {
            Planar = _host.Planar, FixedBase = bp.FixedBase, Substeps = bp.Substeps, Iterations = bp.Iterations, SelfCollision = bp.SelfCollision, GravityCompensation = bp.GravityCompensation, GravityScale = bp.GravityScale,
        };
        int i = Create(model, bodies, st);
        Place(i);
        return i;
    }

    /// <summary>Puts every link body of an articulation at the pose its joint coordinates give, with the velocities they imply.</summary>
    public void Place(int i)
    {
        var s = States[i]!;
        var m = s.Model;
        // Poses are found relative to the root and put on the grid before the root's position is added back: both parts are exact, so the result does not depend on how far from the world origin the articulation is.
        m.BeginRelative();
        m.ForwardKinematics();
        for (int k = 0; k < m.N; k++)
        {
            m.LinkTwist(k, out var w, out var v);
            _host.SetLinkState(s.Body[k], Grid(m.Pw[k]) + m.Anchor, m.Qw[k], w, v);
        }

        m.EndRelative();
        m.ForwardKinematics();
    }

    public readonly struct BodyHandleRef
    {
        public readonly int Index, Generation;
        public BodyHandleRef(int index, int generation) { Index = index; Generation = generation; }
    }

    /// <summary>Removes the articulation; the bodies are the caller's to destroy.</summary>
    public void Remove(int i)
    {
        var s = States[i]!;
        for (int k = 0; k < s.Body.Length; k++)
        {
            if (s.Body[k] < _artOfBody.Length) { _artOfBody[s.Body[k]] = -1; _linkOfBody[s.Body[k]] = -1; }
        }

        States[i] = null;
        Handles.Free(i);
    }

    /// <summary>After a restore: every link names a live kinematic body of the saved generation, once, and the body indices agree.</summary>
    public bool Consistent()
    {
        var bodies = _host.Core.Bodies;
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var s = States[i]!;
            for (int k = 0; k < s.Body.Length; k++)
            {
                int b = s.Body[k];
                if (!bodies.Handles.IsAlive(b, s.BodyGen[k]) || bodies.TypeOf(b) != BodyType.Kinematic) { return false; }
                // A body listed twice would leave the index pointing at the later entry, so this also rejects duplicates.
                if (ArticulationOfBody(b) != i || LinkOfBody(b) != k) { return false; }
            }
        }

        return true;
    }

    private void RebuildIndex()
    {
        Array.Fill(_artOfBody, -1); Array.Fill(_linkOfBody, -1);
        for (int i = 0; i < Handles.HighWater; i++)
        {
            if (!Handles.IsLive(i)) { continue; }
            var s = States[i]!;
            for (int k = 0; k < s.Body.Length; k++) { Index(ref _artOfBody, s.Body[k], i); Index(ref _linkOfBody, s.Body[k], k); }
        }
    }
}
