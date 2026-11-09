using System.IO;
using RecursiveAI.OneMath;
using P = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Scene;

internal static partial class SceneBuilder3D
{
    private static bool BuildMachine(P.PhysicsWorld3D w, P.Body[] bodies, JointDef j)
    {
        if (j.Kind is "point" or "swingtwist") return false;
        var a = bodies[j.A]; var b = bodies[j.B]; var data = j.Data;
        double D(int i) => i < data.Length ? data[i] : throw new InvalidDataException("missing joint data value " + i);
        var fa = new P.JointFrame3D(new((float)j.Ax, (float)j.Ay, (float)j.Az), QuaternionF.Identity);
        var fb = new P.JointFrame3D(new((float)j.Bx, (float)j.By, (float)j.Bz), QuaternionF.Identity);
        P.Joint joint;
        switch (j.Kind)
        {
            case "hinge": joint = w.CreateJoint(new P.HingeJointDef { BodyA = a, BodyB = b, LocalFrameA = fa, LocalFrameB = fb, EnableLimit = j.HasLimit, LowerAngle = (float)j.Lower, UpperAngle = (float)j.Upper, EnableMotor = j.HasMotor, MotorSpeed = (float)j.MotorSpeed, MaxMotorTorque = (float)j.MaxTorque }); break;
            case "filter": joint = w.AddFilterJoint(a, b); break;
            case "gear": joint = w.CreateJoint(new P.GearJointDef { BodyA = a, BodyB = b, LocalFrameA = fa, LocalFrameB = fb, Ratio = (float)D(0) }); break;
            case "pulley": joint = w.CreateJoint(new P.PulleyJointDef { BodyA = a, BodyB = b, LocalFrameA = fa, LocalFrameB = fb, Ratio = (float)D(0), FixedPointA = new(D(1), D(2), D(3)), FixedPointB = new(D(4), D(5), D(6)) }); break;
            case "wheel": joint = w.CreateJoint(new P.WheelJointDef { BodyA = a, BodyB = b, LocalFrameA = fa, LocalFrameB = fb, EnableSpring = true, SpringHertz = (float)D(0), SpringDampingRatio = (float)D(1), EnableLimit = j.HasLimit, LowerTranslation = (float)j.Lower, UpperTranslation = (float)j.Upper, EnableMotor = j.HasMotor, MotorSpeed = (float)j.MotorSpeed, MaxMotorTorque = (float)j.MaxTorque }); break;
            case "motor": joint = w.CreateJoint(new P.MotorJointDef { BodyA = a, BodyB = b, LinearOffset = fa.Position, AngularOffset = new((float)D(0), (float)D(1), (float)D(2), (float)D(3)), MaxForce = (float)D(4), MaxTorque = (float)D(5), CorrectionFactor = (float)D(6) }); break;
            case "mouse": joint = w.CreateJoint(new P.MouseJointDef { BodyA = a, BodyB = b, LocalAnchorB = fb.Position, Target = new(D(0), D(1), D(2)), Hertz = (float)D(3), DampingRatio = (float)D(4), MaxForce = (float)D(5) }); break;
            default: throw new InvalidDataException("unsupported spatial machine joint " + j.Kind);
        }
        joint.BreakForce = j.BreakForce; joint.BreakTorque = j.BreakTorque; joint.LifetimeStepsRemaining = j.LifetimeSteps;
        return true;
    }
}
