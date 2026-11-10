using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Articulations;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>
    /// Creates an articulation: kinematic bodies for its links (with the links' shapes), registered so the world moves them from the joint coordinates every step. The links' bodies are reachable through
    /// <see cref="Articulation.GetLink"/>; destroying any of them destroys the whole articulation.
    /// </summary>
    /// <param name="def">The links, joints and settings; copied. Presets: <see cref="ArticulationDef.Arm"/>, <see cref="ArticulationDef.Chain"/>.</param>
    /// <returns>A handle to the articulation.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="def"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value in the definition is out of range; the message names the link and the member. Nothing is created.</exception>
    /// <exception cref="ArgumentException">The definition has no links, links without usable mass, or masses that leave the joint-space mass matrix singular. Nothing is created.</exception>
    public Articulation AddArticulation(ArticulationDef def)
    {
        using var replayMutation = RecordMutation("PhysicsWorld2D.AddArticulation");
        ArgumentNullException.ThrowIfNull(def);
        var bp = Blueprint(def);
        bp.Validate(true, ArticulationStore.MaxLinks);
        var created = new List<BodyHandle>();
        try
        {
            for (int i = 0; i < bp.Links.Length; i++)
            {
                var b = _engine.CreateBody(new BodyDef { Type = BodyType.Kinematic });
                created.Add(b);
                foreach (var sh in def.Links[i].Shapes) { _engine.AddShape(b, sh.Geometry, sh.Def); }
                int slot = _core.Resolve(b);
                _engine.SetLinkExtents(slot);
                if (bp.Links[i].Mass == 0.0)
                {
                    if (!_engine.ShapeMassProperties(slot, out float mass, out var center, out float inertia)) { throw new ArgumentException("Link " + i + " has no mass and no shape with a density to take one from.", nameof(def)); }
                    var l = bp.Links[i];
                    l.Mass = mass; l.Com = new D3(center.X, center.Y, 0.0); l.Inertia = Planar(inertia);
                }
            }

            bp.Validate(true, ArticulationStore.MaxLinks);
            var model = bp.BuildModel(true);
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

    /// <summary>Creates an articulation with its root link at a position and angle, overriding the definition's.</summary>
    /// <param name="def">The definition; not changed.</param>
    /// <param name="at">World position of the root link's frame.</param>
    /// <param name="angle">World angle of the root link's frame, radians counter-clockwise.</param>
    /// <exception cref="ArgumentNullException"><paramref name="def"/> is null.</exception>
    public Articulation AddArticulation(ArticulationDef def, Vector2D at, float angle = 0f)
    {
        ArgumentNullException.ThrowIfNull(def);
        var copy = new ArticulationDef { FixedBase = def.FixedBase, Position = at, Angle = angle, Substeps = def.Substeps, Iterations = def.Iterations, SelfCollision = def.SelfCollision, GravityCompensation = def.GravityCompensation, GravityScale = def.GravityScale };
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

    /// <summary>Every articulation in the world, by slot.</summary>
    /// <returns>A new array.</returns>
    public Articulation[] GetArticulations()
    {
        var s = _engine.Articulations;
        var list = new List<Articulation>();
        for (int i = 0; i < s.Handles.HighWater; i++) { if (s.Handles.IsLive(i)) { list.Add(new Articulation(this, i, s.Handles.GenerationOf(i))); } }
        return list.ToArray();
    }

    /// <summary>The number of articulations in the world.</summary>
    public int ArticulationCount => _engine.Articulations.Count;

    // A planar link turns about z only, so only the z moment matters; the other two moments are set to it so the tensor is a valid isotropic one.
    private static M3 Planar(double izz) => new(izz, 0, 0, 0, izz, 0, 0, 0, izz);

    private static ArticulationBlueprint Blueprint(ArticulationDef def)
    {
        var bp = new ArticulationBlueprint
        {
            FixedBase = def.FixedBase, BasePosition = new D3(def.Position.X, def.Position.Y, 0.0), BaseRotation = Q(def.Angle), Substeps = def.Substeps, Iterations = def.Iterations,
            SelfCollision = def.SelfCollision, GravityCompensation = def.GravityCompensation, GravityScale = def.GravityScale,
            Links = new LinkBlueprint[def.Links.Count],
        };
        for (int i = 0; i < bp.Links.Length; i++)
        {
            var l = def.Links[i];
            if (l == null) { throw new ArgumentException("Link " + i + " is null.", nameof(def)); }
            bp.Links[i] = new LinkBlueprint
            {
                Parent = l.Parent, Joint = l.Joint, Axis = l.Joint == ArticulationJointKind.Revolute ? new D3(0, 0, 1) : new D3(l.Axis.X, l.Axis.Y, 0), Origin = new D3(l.Origin.X, l.Origin.Y, 0), OriginRotation = Q(l.OriginAngle), Mass = l.Mass,
                Com = new D3(l.CenterOfMass.X, l.CenterOfMass.Y, 0), Inertia = Planar(l.Inertia),
                HasLimits = l.HasLimits, Lower = l.LowerLimit, Upper = l.UpperLimit, Friction = l.Friction, Armature = l.Armature, Damping = l.Damping, MaxVelocity = l.MaxVelocity,
                Stiffness = l.Drive.Stiffness, DriveDamping = l.Drive.Damping, MaxForce = l.Drive.MaxForce, TargetPosition = l.Drive.TargetPosition, TargetVelocity = l.Drive.TargetVelocity,
                InitialPosition = l.InitialPosition,
            };
        }

        return bp;
    }

    /// <summary>The quaternion of a rotation about z by <paramref name="angle"/> (a non-finite angle gives NaN, which the validation of the definition rejects).</summary>
    internal static Q4 Q(float angle)
    {
        var (s, c) = Math64.SinCos(0.5 * angle);
        return new Q4(0, 0, s, c);
    }
}
