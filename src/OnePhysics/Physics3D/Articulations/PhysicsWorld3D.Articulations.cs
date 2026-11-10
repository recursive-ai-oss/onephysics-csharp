using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Articulations;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>
    /// Creates an articulation: kinematic bodies for its links (with the links' shapes), registered so the world moves them from the joint coordinates every step. The links' bodies are reachable through
    /// <see cref="Articulation.GetLink"/>; destroying any of them destroys the whole articulation.
    /// </summary>
    /// <param name="def">The links, joints and settings; copied. Presets: <see cref="ArticulationDef.RobotArm"/>, <see cref="ArticulationDef.Quadruped"/>, <see cref="ArticulationDef.Chain"/>.</param>
    /// <returns>A handle to the articulation.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="def"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value in the definition is out of range; the message names the link and the member. Nothing is created.</exception>
    /// <exception cref="ArgumentException">The definition has no links, links without usable mass, or masses that leave the joint-space mass matrix singular. Nothing is created.</exception>
    public Articulation AddArticulation(ArticulationDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld3D.AddArticulation");
        ArgumentNullException.ThrowIfNull(def);
        var bp = Blueprint(def);
        bp.Validate(false, ArticulationStore.MaxLinks);
        var created = new List<BodyHandle>();
        try
        {
            // Bodies and shapes first: a link without a mass takes it from its shapes.
            for (int i = 0; i < bp.Links.Length; i++)
            {
                var b = _engine.CreateBody(new BodyDef { Type = BodyType.Kinematic });
                created.Add(b);
                var ld = def.Links[i];
                foreach (var sh in ld.Shapes) { _engine.AddShape(b, sh.Spec.Recipe.ToArray(), sh.Def); }
                int slot = _core.Resolve(b);
                _engine.SetLinkExtents(slot);
                if (bp.Links[i].Mass == 0.0)
                {
                    if (!_engine.ShapeMassProperties(slot, out float mass, out var center, out var inertia)) { throw new ArgumentException("Link " + i + " has no mass and no shape with a density to take one from.", nameof(def)); }
                    var l = bp.Links[i];
                    l.Mass = mass; l.Com = new D3(center.X, center.Y, center.Z);
                    l.Inertia = new M3(inertia.M11, inertia.M12, inertia.M13, inertia.M12, inertia.M22, inertia.M23, inertia.M13, inertia.M23, inertia.M33);
                }
            }

            bp.Validate(false, ArticulationStore.MaxLinks);
            var model = bp.BuildModel(false);
            var refs = new ArticulationStore.BodyHandleRef[created.Count];
            for (int i = 0; i < refs.Length; i++) { refs[i] = new ArticulationStore.BodyHandleRef(created[i].Index, created[i].Generation); }
            int index = _engine.Articulations.CreateFrom(bp, model, refs);
            return new Articulation(this, index, _engine.Articulations.Handles.GenerationOf(index));
        }
        catch
        {
            foreach (var h in created) { if (_core.Bodies.Handles.IsAlive(h.Index, h.Generation)) { _engine.DestroyBody(h); } }
            throw;
        }
    }

    /// <summary>Creates an articulation with its root link at a position and orientation, overriding the definition's.</summary>
    /// <param name="def">The definition; not changed.</param>
    /// <param name="at">World position of the root link's frame.</param>
    /// <param name="rotation">World orientation of the root link's frame; the default (all zero) is the identity.</param>
    /// <exception cref="ArgumentNullException"><paramref name="def"/> is null.</exception>
    public Articulation AddArticulation(ArticulationDef def, Vector3D at, QuaternionF rotation = default)
    {
        ArgumentNullException.ThrowIfNull(def);
        var copy = new ArticulationDef { FixedBase = def.FixedBase, Position = at, Rotation = rotation.LengthSquared() == 0f ? QuaternionF.Identity : rotation, Substeps = def.Substeps, Iterations = def.Iterations, SelfCollision = def.SelfCollision, GravityCompensation = def.GravityCompensation, GravityScale = def.GravityScale };
        copy.Links.AddRange(def.Links);
        return AddArticulation(copy);
    }

    /// <summary>The articulation a body is a link of, if there is one.</summary>
    /// <param name="body">Any body of this world.</param>
    /// <param name="articulation">The articulation.</param>
    /// <returns>True when the body is a link.</returns>
    public bool TryGetArticulation(Body body, out Articulation articulation)
    {
        articulation = default;
        if (!body.IsValid || body.World != this) { return false; }
        int i = _engine.Articulations.ArticulationOfBody(body.Handle.Index);
        if (i < 0) { return false; }
        articulation = new Articulation(this, i, _engine.Articulations.Handles.GenerationOf(i));
        return true;
    }

    /// <summary>The number of articulations in the world.</summary>
    public int ArticulationCount => _engine.Articulations.Count;

    private static ArticulationBlueprint Blueprint(ArticulationDef def)
    {
        var bp = new ArticulationBlueprint
        {
            FixedBase = def.FixedBase, BasePosition = new D3(def.Position.X, def.Position.Y, def.Position.Z), BaseRotation = Q(def.Rotation), Substeps = def.Substeps, Iterations = def.Iterations,
            SelfCollision = def.SelfCollision, GravityCompensation = def.GravityCompensation, GravityScale = def.GravityScale,
            Links = new LinkBlueprint[def.Links.Count],
        };
        for (int i = 0; i < bp.Links.Length; i++)
        {
            var l = def.Links[i];
            if (l == null) { throw new ArgumentException("Link " + i + " is null.", nameof(def)); }
            var t = l.InertiaTensor;
            bp.Links[i] = new LinkBlueprint
            {
                Parent = l.Parent, Joint = l.Joint, Axis = new D3(l.Axis.X, l.Axis.Y, l.Axis.Z), Origin = new D3(l.Origin.X, l.Origin.Y, l.Origin.Z), OriginRotation = Q(l.OriginRotation), Mass = l.Mass,
                Com = new D3(l.CenterOfMass.X, l.CenterOfMass.Y, l.CenterOfMass.Z), Inertia = new M3(t.M11, t.M12, t.M13, t.M21, t.M22, t.M23, t.M31, t.M32, t.M33),
                HasLimits = l.HasLimits, Lower = l.LowerLimit, Upper = l.UpperLimit, ConeLimit = l.SwingLimit, Friction = l.Friction, Armature = l.Armature, Damping = l.Damping, MaxVelocity = l.MaxVelocity,
                Stiffness = l.Drive.Stiffness, DriveDamping = l.Drive.Damping, MaxForce = l.Drive.MaxForce, TargetPosition = l.Drive.TargetPosition, TargetVelocity = l.Drive.TargetVelocity,
                InitialPosition = l.InitialPosition, InitialRotation = Q(l.InitialRotation), TargetRotation = Q(l.TargetRotation),
            };
        }

        return bp;
    }

    internal static Q4 Q(QuaternionF q) => new(q.X, q.Y, q.Z, q.W);
}
