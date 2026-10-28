using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using RecursiveAI.OnePhysics.Debug;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>
    /// Adds one entry per live joint in constraint-slot order. Anchors are body origin + (double)(rotation * local anchor), the same way shapes are placed.
    /// A mouse joint reports its target as anchor A (it has no first anchor of its own); prismatic and wheel joints report their axis rotated into the world.
    /// </summary>
    public void AddDebugJoints(DebugGeometry geometry)
    {
        var cs = Core.Constraints;
        var bodies = Core.Bodies;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) != ConstraintKind.Joint || Joints[c].Type == 0) { continue; }
            ref readonly var j = ref Joints[c];
            int a = cs.BodyA[c], b = cs.BodyB[c];
            var qA = bodies.RotationOf(a);
            var pb = AnchorWorld(b, j.LBx, j.LBy);
            Vector3D anchorA = j.Type == (byte)JointType2D.Mouse ? new Vector3D(j.TargetX, j.TargetY, 0.0) : AnchorWorld(a, j.LAx, j.LAy);
            Vector3D axis = default;
            if (j.Type == (byte)JointType2D.Prismatic || j.Type == (byte)JointType2D.Wheel)
            {
                axis = new Vector3D((double)(qA[0] * j.AxX - qA[1] * j.AxY), (double)(qA[1] * j.AxX + qA[0] * j.AxY), 0.0);
            }

            geometry.Add(new DebugJoint(Core.HandleOfJoint(c), Core.HandleOfBody(a), Core.HandleOfBody(b), anchorA, pb, axis));
        }
    }

    private Vector3D AnchorWorld(int body, float lx, float ly)
    {
        var bodies = Core.Bodies;
        var p = bodies.PositionOf(body);
        var r = bodies.RotationOf(body);
        float cx = LocalCenter[2 * body], cy = LocalCenter[2 * body + 1];
        // Body origin = centre of mass - R * localCenter; anchor = origin + R * local.
        double ox = p[0] - (double)(r[0] * cx - r[1] * cy), oy = p[1] - (double)(r[1] * cx + r[0] * cy);
        return new Vector3D(ox + (double)(r[0] * lx - r[1] * ly), oy + (double)(r[1] * lx + r[0] * ly), 0.0);
    }
}
