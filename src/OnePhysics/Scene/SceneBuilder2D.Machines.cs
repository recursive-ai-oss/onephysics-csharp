using System.IO;
using RecursiveAI.OneMath;
using P = RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Scene;

internal static partial class SceneBuilder2D
{
    private static bool BuildMachine(P.PhysicsWorld2D w, P.Body[] bodies, JointDef j)
    {
        if (j.Kind == "revolute") return false;
        var a = bodies[j.A]; var b = bodies[j.B]; var data = j.Data;
        double D(int i) => i < data.Length ? data[i] : throw new InvalidDataException("missing joint data value " + i);
        bool B(int i) => D(i) switch { 0 => false, 1 => true, _ => throw new InvalidDataException("joint boolean data must be 0 or 1") };
        Vector2F la = new((float)j.Ax, (float)j.Ay), lb = new((float)j.Bx, (float)j.By);
        P.Joint joint;
        switch (j.Kind)
        {
            case "filter": joint = w.AddFilterJoint(a, b); break;
            case "gear": joint = w.AddGear(a, b, (float)D(0)); break;
            case "rack": joint = w.CreateJoint(new P.RackAndPinionJointDef { BodyA = a, BodyB = b, Ratio = (float)D(0), LocalAxisB = new((float)D(1), (float)D(2)) }); break;
            case "pulley": joint = w.CreateJoint(new P.PulleyJointDef { BodyA = a, BodyB = b, LocalAnchorA = la, LocalAnchorB = lb, Ratio = (float)D(0), Length = (float)D(1), FixedPointA = new(D(2), D(3)), FixedPointB = new(D(4), D(5)) }); break;
            case "wheel": joint = w.CreateJoint(new P.WheelJointDef { BodyA = a, BodyB = b, LocalAnchorA = la, LocalAnchorB = lb, LocalAxisA = new((float)D(0), (float)D(1)), EnableSpring = true, SpringHertz = (float)D(2), SpringDampingRatio = (float)D(3), EnableLimit = j.HasLimit, LowerTranslation = (float)j.Lower, UpperTranslation = (float)j.Upper, EnableMotor = j.HasMotor, MotorSpeed = (float)j.MotorSpeed, MaxMotorTorque = (float)j.MaxTorque }); break;
            case "path":
            {
                int count = (data.Length - 2) / 2;
                if (data.Length < 6 || (data.Length & 1) != 0) throw new InvalidDataException("path data: loop align x0 y0 ...");
                var points = new Vector2F[count]; for (int i = 0; i < count; i++) points[i] = new((float)D(2 + 2 * i), (float)D(3 + 2 * i));
                joint = w.CreateJoint(new P.PathJointDef { BodyA = a, BodyB = b, LocalAnchorB = lb, Points = points, Loop = B(0), AlignTangent = B(1), EnableMotor = j.HasMotor, MotorSpeed = (float)j.MotorSpeed, MaxMotorForce = (float)j.MaxTorque }); break;
            }
            case "configurable":
            {
                P.JointAxisSettings Axis(int i) => new() { Mode = D(i) switch { 0 => P.JointAxisMode.Locked, 1 => P.JointAxisMode.Limited, 2 => P.JointAxisMode.Free, _ => throw new InvalidDataException("axis mode 0..2") }, Lower = (float)D(i + 1), Upper = (float)D(i + 2), EnableMotor = B(i + 3), MotorSpeed = (float)D(i + 4), MaxMotor = (float)D(i + 5), EnableSpring = B(i + 6), SpringHertz = (float)D(i + 7), SpringDampingRatio = (float)D(i + 8) };
                joint = w.CreateJoint(new P.ConfigurableJointDef { BodyA = a, BodyB = b, LocalAnchorA = la, LocalAnchorB = lb, LocalFrameAngleA = (float)D(0), ReferenceAngle = (float)D(1), X = Axis(2), Y = Axis(11), Angle = Axis(20) }); break;
            }
            default: throw new InvalidDataException("unsupported planar machine joint " + j.Kind);
        }
        joint.BreakForce = j.BreakForce; joint.BreakTorque = j.BreakTorque; joint.LifetimeStepsRemaining = j.LifetimeSteps;
        return true;
    }
}
