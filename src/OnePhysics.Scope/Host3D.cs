using System;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.Physics3D;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>The spatial engine behind <see cref="WorldHost"/>.</summary>
internal sealed class Host3D : WorldHost
{
    private readonly PhysicsWorld3D _world;

    public Host3D(PhysicsWorld3D world)
    {
        _world = world;
        // Forces and torques are only collected for the latest step when this is on; it does not change what the world computes.
        world.Options.DetailedInspection = true;
    }

    public override int Dimension => 3;

    public override int Id => _world.Id;

    public override long StepCount => _world.StepCount;

    public override (double X, double Y, double Z) Origin => (_world.Origin.X, _world.Origin.Y, _world.Origin.Z);

    public override ulong Hash() => _world.ComputeHash();

    public override void Step(float dt) => _world.Step(dt);

    public override byte[] Snapshot() => _world.Snapshot();

    public override void Restore(ReadOnlySpan<byte> snapshot) => _world.Restore(snapshot);

    public override ReadOnlySpan<PhysicsEvent> Events => _world.Events;

    public override StepCounters Counters => _world.Counters;

    public override void CollectDebug(DebugContent content, DebugGeometry into) => _world.GetDebugGeometry(content, into);

    public override ReplayRecorder StartRecording(Stream stream) => _world.StartRecording(stream);

    public override bool IsBodyAlive(BodyHandle body) => _world.IsBodyAlive(body);

    public override BodyType? TypeOf(BodyHandle body) => _world.IsBodyAlive(body) ? _world.GetBody(body).Type : null;

    public override void ApplyPoke(BodyHandle handle, in StepOp op)
    {
        Body body = _world.GetBody(handle);
        switch (op.Kind)
        {
            case OpKind.Impulse:
                body.ApplyImpulse(new Vector3F(op.X, op.Y, op.Z));
                break;
            case OpKind.SetVelocity:
                body.SetLinearVelocity(new Vector3F(op.X, op.Y, op.Z));
                break;
            case OpKind.SetPosition:
                body.SetPosition(new Vector3D(op.X, op.Y, op.Z));
                break;
            default:
                body.Wake();
                break;
        }
    }

    public override bool InspectBody(BodyHandle handle, PropertyList into)
    {
        if (!_world.IsBodyAlive(handle))
        {
            return false;
        }

        Body body = _world.GetBody(handle);
        Vector3D p = body.Position;
        QuaternionF q = body.Rotation;
        Vector3F v = body.LinearVelocity;
        Vector3F w = body.AngularVelocity;
        into.Add("Transform", "Position", Vec(p.X, p.Y, p.Z));
        into.Add("Transform", "Rotation", ScopeValue.OfText($"{Num(q.X)} {Num(q.Y)} {Num(q.Z)} {Num(q.W)}"));
        into.Add("Motion", "Linear velocity", Vec(v.X, v.Y, v.Z));
        into.Add("Motion", "Angular velocity", Vec(w.X, w.Y, w.Z));
        into.Add("Body", "Type", ScopeValue.OfText(body.Type.ToString()));
        into.Add("Body", "Mass", ScopeValue.OfNumber(body.Mass));
        into.Add("Body", "Awake", ScopeValue.OfBool(body.IsAwake));
        try
        {
            var i = body.Inspect();
            into.Add("Solver", "Island", ScopeValue.OfInteger(i.Island));
            into.Add("Solver", "Sleep time", ScopeValue.OfNumber(i.SleepTime));
            into.Add("Solver", "Constraints", ScopeValue.OfInteger(i.ConstraintCount));
            into.Add("Solver", "Constraint colours", ScopeValue.OfInteger(i.ConstraintColorMask));
            Vector3D e = i.ExternalImpulse;
            into.Add("Solver", "External impulse", Vec(e.X, e.Y, e.Z));
        }
        catch (DetailedInspectionDisabledException)
        {
            into.Add("Solver", "Detail", ScopeValue.OfText(NoDetail));
        }

        return true;
    }

    public override bool InspectJoint(JointHandle handle, PropertyList into)
    {
        if (!_world.IsJointAlive(handle))
        {
            return false;
        }

        try
        {
            JointInspection j = _world.GetJoint(handle).Inspect();
            into.Add("Joint", "Broken", ScopeValue.OfBool(j.IsBroken));
            into.Add("Joint", "Limit active", ScopeValue.OfBool(j.IsLimitActive));
            into.Add("Joint", "Motor active", ScopeValue.OfBool(j.IsMotorActive));
            into.Add("Joint", "Island", ScopeValue.OfInteger(j.Island));
            into.Add("Joint", "Colour", ScopeValue.OfInteger(j.Color));
            into.Add("Forces", "Force on A", Vec(j.ForceA.X, j.ForceA.Y, j.ForceA.Z));
            into.Add("Forces", "Force on B", Vec(j.ForceB.X, j.ForceB.Y, j.ForceB.Z));
            into.Add("Forces", "Torque on A", Vec(j.TorqueA.X, j.TorqueA.Y, j.TorqueA.Z));
            into.Add("Forces", "Torque on B", Vec(j.TorqueB.X, j.TorqueB.Y, j.TorqueB.Z));
        }
        catch (DetailedInspectionDisabledException)
        {
            into.Add("Joint", "Detail", ScopeValue.OfText(NoDetail));
        }

        return true;
    }
}
