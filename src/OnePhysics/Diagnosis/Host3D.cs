using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Diagnosis;

internal sealed class Host3D : DiagnosisHost
{
    private readonly PhysicsWorld3D _world;
    private readonly Engine3D _e;

    public Host3D(PhysicsWorld3D world)
    {
        _world = world; _e = world.Engine; Core = world.Core;
        _e.BindJoints();
    }

    public override int Dimension => 3;
    public override float Mass(int body) => _e.Mass[body];
    public override float MaxExtent(int body) => _e.MaxExtent[body];
    public override float MinExtent(int body) { float m = _e.MinExtent[body]; return m < float.MaxValue / 2 ? m : _e.ComputeMinExtent(body); }
    public override float MaxLinearSpeed => _e.Opt.MaxLinearSpeed;
    public override float SpeculativeDistance => _e.Opt.SpeculativeDistance;
    public override bool ContinuousEnabled => _e.Opt.ContinuousEnabled;
    public override double GravityMagnitude { get { var g = _e.Opt.Gravity; return System.Math.Sqrt((double)g.X * g.X + (double)g.Y * g.Y + (double)g.Z * g.Z); } }
    public override bool SleepEnabled => _e.Opt.SleepEnabled;
    public override float TimeToSleep => _e.Opt.TimeToSleep;
    public override float SleepThreshold => _e.Opt.SleepThreshold;

    public override bool DetailedInspection { get => _e.Opt.DetailedInspection; set => _e.Opt.DetailedInspection = value; }

    public override void Step(float dt) { using var quiet = _world.SuppressCallbacks(); _world.Step(dt); }

    public override DiagnosisHost Fork() => new Host3D(_world.Fork());

    public override JointFacts Joint(int c)
    {
        _e.BindJoints();
        ref readonly var j = ref _e.Joints[c];
        var t = (JointType3D)j.Type;
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        int a = cs.BodyA[c], b = cs.BodyB[c];
        var f = new JointFacts
        {
            Type = JointData3D.Name(t),
            Solves = t != JointType3D.Filter,
            Coincident = t is JointType3D.Fixed or JointType3D.Point or JointType3D.Hinge or JointType3D.Cone or JointType3D.SwingTwist,
            LimitOn = j.Limit, MotorOn = j.Motor,
            LockedRows = Locked(in j, t),
        };
        var la = new Vector3F(j.LAx, j.LAy, j.LAz);
        var lb = new Vector3F(j.LBx, j.LBy, j.LBz);
        var ca = new Vector3F(_e.LocalCenter[3 * a], _e.LocalCenter[3 * a + 1], _e.LocalCenter[3 * a + 2]);
        var cb = new Vector3F(_e.LocalCenter[3 * b], _e.LocalCenter[3 * b + 1], _e.LocalCenter[3 * b + 2]);
        f.ArmA = Length(la - ca); f.ArmB = Length(lb - cb);
        var wa = World(bodies, a, la); var wb = World(bodies, b, lb);
        double dx = wa.X - wb.X, dy = wa.Y - wb.Y, dz = wa.Z - wb.Z;
        f.Gap = System.Math.Sqrt(dx * dx + dy * dy + dz * dz);
        return f;
    }

    // Degrees of freedom each kind removes between its bodies (Kutzbach counting); the couplings (gear, rack, pulley) take one, the soft drives none.
    private static int Locked(in JointData3D j, JointType3D t)
    {
        switch (t)
        {
            case JointType3D.Fixed: return 6;
            case JointType3D.Point: case JointType3D.Cone: case JointType3D.SwingTwist: return 3;
            case JointType3D.Distance: case JointType3D.Gear: case JointType3D.RackAndPinion: case JointType3D.Pulley: return 1;
            case JointType3D.Hinge: case JointType3D.Slider: return 5;
            case JointType3D.Wheel: return 4;
            case JointType3D.Path: return 2;
            case JointType3D.SixDof:
                int n = 0;
                for (int i = 0; i < 6; i++) { if (j.Six.Mode[i] == (byte)JointAxisMode.Locked) { n++; } }
                return n;
            default: return 0;
        }
    }

    private static float Length(Vector3F v) => float.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);

    private static Vector3D World(BodyStore bodies, int b, Vector3F local)
    {
        var p = bodies.PositionOf(b);
        var r = bodies.RotationOf(b);
        var rotated = new QuaternionF(r[0], r[1], r[2], r[3]).Rotate(local);
        return new Vector3D(p[0] + rotated.X, p[1] + rotated.Y, p[2] + rotated.Z);
    }
}
