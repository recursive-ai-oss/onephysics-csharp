using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    private void InspectJointRows(int c, in JS j, in JointData2D d, ReadOnlySpan<float> s, ReadOnlySpan<float> p)
    {
        var read = Core.JointReadback;
        read.ClearRows(c);
        float cx = j.Dx + j.RBx - j.RAx, cy = j.Dy + j.RBy - j.RAy;
        float angle = SmallAngle(in j, s);
        float coordinate = 0f, velocity = 0f;
        bool angular = false;
        JointRowBits rigid = JointRowBits.Active;
        switch ((JointType2D)d.Type)
        {
            case JointType2D.Revolute:
                read.Row(c, 0, cx, rigid); read.Row(c, 1, cy, rigid);
                coordinate = ExactAngle(in j, s); velocity = j.WB - j.WA; angular = true;
                break;
            case JointType2D.Weld:
            case JointType2D.Motor:
                bool motorJoint = d.Type == (byte)JointType2D.Motor;
                var role = motorJoint ? JointRowBits.Motor : JointRowBits.None;
                var pointRole = role | (!motorJoint || d.MaxMotor > 0f ? JointRowBits.Active : 0) | (!motorJoint && d.Hertz > 0f ? JointRowBits.Spring : 0);
                var angleRole = role | (!motorJoint || d.MaxMotor2 > 0f ? JointRowBits.Active : 0) | (!motorJoint && d.Hertz2 > 0f ? JointRowBits.Spring : 0);
                read.Row(c, 0, d.Type == (byte)JointType2D.Motor ? j.Dx + j.RBx - j.RAx - (j.DAc * s[JRow.TgtX] - j.DAs * s[JRow.TgtY]) : cx, pointRole);
                read.Row(c, 1, d.Type == (byte)JointType2D.Motor ? j.Dy + j.RBy - j.RAy - (j.DAs * s[JRow.TgtX] + j.DAc * s[JRow.TgtY]) : cy, pointRole);
                read.Row(c, 2, angle, angleRole | JointRowBits.Angular);
                return;
            case JointType2D.Prismatic:
            case JointType2D.Wheel:
                var ax = ComputeAxes(in j, s);
                coordinate = ax.Translation; velocity = j.AxialVelocity(ax.Ax, ax.Ay, ax.A1, ax.A2);
                read.Row(c, 0, ax.Nx * cx + ax.Ny * cy, rigid);
                if (d.Type == (byte)JointType2D.Prismatic) { read.Row(c, 1, angle, rigid | JointRowBits.Angular); }
                break;
            case JointType2D.Distance:
                float len = float.Sqrt(cx * cx + cy * cy);
                coordinate = len;
                if (len > 0.005f) { float x = cx / len, y = cy / len; velocity = j.AxialVelocity(x, y, j.RAx * y - j.RAy * x, j.RBx * y - j.RBy * x); }
                if (!d.Spring || d.Lower >= d.Upper) { read.Row(c, 0, len - d.Length, rigid); }
                break;
            case JointType2D.Mouse:
                int b = Core.Constraints.BodyB[c];
                read.Row(c, 0, BodyState[(b + 1) * BS2D.Stride + BS2D.Dpx] + j.RBx - s[JRow.TgtX], rigid | JointRowBits.Spring);
                read.Row(c, 1, BodyState[(b + 1) * BS2D.Stride + BS2D.Dpy] + j.RBy - s[JRow.TgtY], rigid | JointRowBits.Spring);
                return;
        }
        JointRowBits unit = angular ? JointRowBits.Angular : 0;
        if (d.Spring && d.Hertz > 0f && (d.Type != (byte)JointType2D.Distance || d.Lower < d.Upper)) { read.Row(c, JPay.Spring, coordinate - (d.Type == (byte)JointType2D.Distance ? d.Length : 0f), rigid | JointRowBits.Spring | unit); }
        if (d.Motor && (d.Type != (byte)JointType2D.Distance || (d.Spring && d.Lower < d.Upper)))
        {
            bool wheel = d.Type == (byte)JointType2D.Wheel;
            read.Row(c, JPay.Motor, (wheel ? j.WB - j.WA : velocity) - d.MotorSpeed,
                JointRowBits.Motor | JointRowBits.Velocity | (wheel ? JointRowBits.Angular : unit) | (d.MaxMotor > 0f ? JointRowBits.Active : 0));
        }
        if (d.Limit)
        {
            float lo = coordinate - d.Lower, hi = d.Upper - coordinate;
            read.Row(c, JPay.Lower, lo, JointRowBits.Limit | unit | (lo <= 0f || p[JPay.Lower] != 0f ? JointRowBits.Active : 0));
            read.Row(c, JPay.Upper, hi, JointRowBits.Limit | unit | (hi <= 0f || p[JPay.Upper] != 0f ? JointRowBits.Active : 0));
        }
    }
}
