using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType, SettleReport
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, Body, SettleOptions3D

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/settle.md.</summary>
public static class Settle
{
    public static (bool Clean, int Steps, double TipDegrees, float TippingPush, double Sag, bool Seats, float Clearance, ulong Hash) Run()
    {
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(40, 1, 40), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);
        var cabinet = world.AddBox(new Vector3F(1, 2, 1), at: new Vector3D(0, 1.2, 0));        // placed 0.2 m too high: it will drop
        var rope = world.AddRope(new Vector3D(-4, 4, 0), new Vector3D(-2, 4, 0), 16, pinStart: true, pinEnd: true);

        SettleReport report = world.Settle(new SettleOptions3D
        {
            ExtraLinearDamping = 1f,                                 // removes the bounce, shortens the run
            Bounds = (new Vector3D(-50, -5, -50), new Vector3D(50, 50, 50)),
            ReportBodies = true,                                     // per-body residuals and the lists below
        });
        if (!report.IsClean) throw new InvalidOperationException(report.ToString());   // not at rest, loose bodies, or a body that left the level

        StabilityMargin3D margin = cabinet.GetStabilityMargin();
        double tipDegrees = margin.MinTipAngle * 180 / Math.PI;     // how far the floor can tilt before the cabinet falls over
        float push = margin.MinTippingForce;                         // a horizontal push at its centre of mass, newtons

        RestShape3D hanging = world.GetRestShape(rope);              // positions, edges, frames and the sag, ready to bake into a model

        var peg = world.AddBox(new Vector3F(0.2f, 0.4f, 0.2f), at: new Vector3D(6, 1.5, 0));
        InsertionReport3D insertion = peg.CheckInsertion(new Vector3F(0, -1, 0), 1.3f);   // does it go 1.3 m down, and how much room does it have?
        return (report.IsClean, report.StepsUsed, tipDegrees, push, hanging.Sag, insertion.Seats, insertion.MinClearance, world.ComputeHash());
    }
}
