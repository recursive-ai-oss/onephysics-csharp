using System;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Diagnosis;

internal sealed class Host2D : DiagnosisHost
{
    private readonly PhysicsWorld2D _world;
    private readonly Engine2D _e;

    public Host2D(PhysicsWorld2D world)
    {
        _world = world; _e = world.Engine; Core = world.Core;
        _e.BindJoints();
    }

    public override float UnitsPerMetre => _e.Opt.UnitsPerMetre;
    public override int Dimension => 2;
    public override float Mass(int body) => _e.Mass[body];
    public override float MaxExtent(int body) => _e.MaxExtent[body] / _e.Opt.UnitsPerMetre;
    public override float MinExtent(int body) { float m = _e.MinExtent[body]; if (!(m < float.MaxValue / 2)) { m = _e.ComputeMinExtent(body); } return m < float.MaxValue / 2 ? m / _e.Opt.UnitsPerMetre : m; }
    public override float MaxLinearSpeed => _e.Opt.MaxLinearSpeed / _e.Opt.UnitsPerMetre;
    public override float SpeculativeDistance => _e.Opt.SpeculativeDistance / _e.Opt.UnitsPerMetre;
    public override bool ContinuousEnabled => _e.Opt.ContinuousEnabled;
    public override double GravityMagnitude { get { var g = _e.Opt.Gravity; return System.Math.Sqrt((double)g.X * g.X + (double)g.Y * g.Y) / _e.Opt.UnitsPerMetre; } }
    public override bool SleepEnabled => _e.Opt.SleepEnabled;
    public override float TimeToSleep => _e.Opt.TimeToSleep;
    public override float SleepThreshold => _e.Opt.SleepThreshold / _e.Opt.UnitsPerMetre;

    public override bool DetailedInspection { get => _e.Opt.DetailedInspection; set => _e.Opt.DetailedInspection = value; }

    public override void Step(float dt) { using var quiet = _world.SuppressCallbacks(); _world.Step(dt); }

    public override DiagnosisHost Fork() => new Host2D(_world.Fork());

    // Degrees of freedom each kind removes between its bodies (Kutzbach counting); the couplings take one, the soft drives none.
    private static int Locked(JointType2D t) => t switch
    {
        JointType2D.Weld => 3,
        JointType2D.Revolute or JointType2D.Prismatic => 2,
        JointType2D.Wheel or JointType2D.Distance or JointType2D.Gear or JointType2D.RackAndPinion or JointType2D.Pulley => 1,
        JointType2D.Path => 2,
        _ => 0,
    };

    public override JointFacts Joint(int c)
    {
        _e.BindJoints();
        ref readonly var j = ref _e.Joints[c];
        var t = (JointType2D)j.Type;
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        int a = cs.BodyA[c], b = cs.BodyB[c];
        var f = new JointFacts
        {
            Type = JointData2D.Name(t),
            Solves = t != JointType2D.Filter,
            Coincident = t is JointType2D.Revolute or JointType2D.Weld,
            LimitOn = j.Limit, MotorOn = j.Motor,
            LockedRows = Locked(t),
        };
        float ax = j.LAx - _e.LocalCenter[2 * a], ay = j.LAy - _e.LocalCenter[2 * a + 1];
        float bx = j.LBx - _e.LocalCenter[2 * b], by = j.LBy - _e.LocalCenter[2 * b + 1];
        f.ArmA = float.Sqrt(ax * ax + ay * ay) / _e.Opt.UnitsPerMetre; f.ArmB = float.Sqrt(bx * bx + by * by) / _e.Opt.UnitsPerMetre;
        var pa = bodies.PositionOf(a); var ra = bodies.RotationOf(a);
        var pb = bodies.PositionOf(b); var rb = bodies.RotationOf(b);
        double wax = pa[0] + (ra[0] * j.LAx - ra[1] * j.LAy), way = pa[1] + (ra[1] * j.LAx + ra[0] * j.LAy);
        double wbx = pb[0] + (rb[0] * j.LBx - rb[1] * j.LBy), wby = pb[1] + (rb[1] * j.LBx + rb[0] * j.LBy);
        double dx = wax - wbx, dy = way - wby;
        f.Gap = System.Math.Sqrt(dx * dx + dy * dy) / _e.Opt.UnitsPerMetre;
        return f;
    }
}
