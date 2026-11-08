using System;
using RecursiveAI.OnePhysics;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Debug;
using RecursiveAI.OnePhysics.LowLevel;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

internal static class InspectionSmoke
{
    internal static bool Run(out ulong digest)
    {
        var a = new P2.PhysicsWorld2D(new P2.WorldOptions2D { SleepEnabled = false });
        var b = new P3.PhysicsWorld3D(new P3.WorldOptions3D { SleepEnabled = false });
        var pivot = a.AddCircle(0.1f, new Vector2D(0, 2), BodyType.Static);
        var mass = a.AddBox(new Vector2F(1, 1), default);
        var hinge = a.AddHinge(pivot, mass, pivot.Position);
        var top = b.AddSphere(0.1f, new Vector3D(0, 2, 0), BodyType.Static);
        var body = b.AddBox(new Vector3F(1, 1, 1), default);
        var point = b.AddPoint(top, body, top.Position);
        long stamp = 0;
        a.Profile.TimestampSource = () => ++stamp; b.Profile.TimestampSource = () => ++stamp;
        for (int i = 0; i < 120; i++) { a.Step(1f / 60); b.Step(1f / 60); }
        var j2 = hinge.Inspect(); var j3 = point.Inspect();
        bool good = Math.Abs(j2.ForceB.Y / (mass.Mass * 9.81) - 1) < 0.02 && Math.Abs(j3.ForceB.Y / (body.Mass * 9.81) - 1) < 0.02;
        good &= j2.Rows[0].Flags != JointRowBits.None && j3.Rows[2].Flags != JointRowBits.None;
        good &= mass.Inspect().GravityImpulse.Y < 0 && body.Inspect().GravityImpulse.Y < 0;
        good &= a.Counters.Joints == 1 && b.Counters.Joints == 1;
        good &= a.Profile.Durations[(int)RecursiveAI.OnePhysics.StepStage.Solve] > 0 && b.Profile.Durations[(int)RecursiveAI.OnePhysics.StepStage.Solve] > 0;
        var geometry = new DebugGeometry();
        a.GetDebugGeometry(DebugContent.Inspection, geometry);
        good &= geometry.JointErrors.Length == 2 && geometry.InspectionVectors.Length == 2;
        b.GetDebugGeometry(DebugContent.Inspection, geometry);
        good &= geometry.JointErrors.Length == 3 && geometry.InspectionVectors.Length == 2;
        foreach (var c in body.Inspect().Constraints) { good &= !c.IsContact; }
        digest = 14695981039346656037UL;
        foreach (var world in new[] { a.Core, b.Core })
        {
            foreach (double value in world.JointReadback.Impulses.Slice(0, world.Constraints.HighWater * 12)) { Mix(ref digest, BitConverter.DoubleToUInt64Bits(value)); }
            foreach (double value in world.BodyReadback.Impulses.Slice(0, world.Bodies.HighWater * 15)) { Mix(ref digest, BitConverter.DoubleToUInt64Bits(value)); }
            foreach (var row in world.JointReadback.Rows.Slice(0, world.Constraints.HighWater * 32)) { Mix(ref digest, BitConverter.SingleToUInt32Bits(row.Error)); Mix(ref digest, (byte)row.Flags); }
        }
        return good;
    }
    private static void Mix(ref ulong hash, ulong value)
    {
        for (int i = 0; i < 8; i++) { hash ^= (byte)(value >> (i * 8)); hash = unchecked(hash * 1099511628211UL); }
    }
}
