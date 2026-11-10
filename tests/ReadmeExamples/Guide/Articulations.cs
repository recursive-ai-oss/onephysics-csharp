using System;
using RecursiveAI.OneMath;                       // Vector3D, Vector3F
using RecursiveAI.OnePhysics;                    // BodyType, ArticulationDrive, ArticulationJointKind
using RecursiveAI.OnePhysics.Physics3D;          // PhysicsWorld3D, ArticulationDef, Articulation

namespace GuideExamples;

/// <summary>The code blocks of docs/guide/articulations.md.</summary>
public static class Articulations
{
    public static (double ToolError, double TorsoY, double SwingX, ulong Hash) Run()
    {
        double toolError = 0, torsoY = 0, swingX = 0;
        var world = new PhysicsWorld3D();
        world.AddBox(new Vector3F(20f, 1f, 20f), at: new Vector3D(0, -0.5, 0), type: BodyType.Static);

        // A seven-joint arm on the floor, steered by a task-space target on its tool point.
        var arm = world.AddArticulation(ArticulationDef.RobotArm(0.9f, 7, 8f), new Vector3D(0, 0, 0));
        var tool = new Vector3F(0, 0.1286f, 0);                                   // a point on the last link
        var goal = new Vector3D(0.3, 0.5, 0.3);
        arm.SetTaskTarget(arm.LinkCount - 1, tool, goal);

        // A quadruped on a floating base stands under its leg drives.
        var dog = world.AddArticulation(ArticulationDef.Quadruped(), new Vector3D(3, 0.62, 0));

        // A pendulum written by hand: two links, one revolute joint, no drive.
        var def = new ArticulationDef { FixedBase = true, Position = new Vector3D(-3, 4, 0) };
        def.AddLink(new ArticulationLinkDef { Mass = 1f, InertiaTensor = Matrix3x3F.Identity });
        def.AddLink(new ArticulationLinkDef { Parent = 0, Joint = ArticulationJointKind.Revolute, Axis = new Vector3F(0, 0, 1), Mass = 2f, CenterOfMass = new Vector3F(0, -0.5f, 0), InertiaTensor = Matrix3x3F.Identity * 0.1f, InitialPosition = 1f });
        var swing = world.AddArticulation(def);

        for (int i = 0; i < 300; i++) { world.Step(1f / 60f); }

        var tip = arm.GetPointPosition(arm.LinkCount - 1, tool);
        toolError = (tip - goal).Length();
        torsoY = dog.GetLink(0).Position.Y;
        swingX = swing.GetLink(1).Position.X;
        return (toolError, torsoY, swingX, world.ComputeHash());
    }

    public static (double Torque0, double Torque1) Dynamics()
    {
        var world = new PhysicsWorld3D();
        var arm = world.AddArticulation(ArticulationDef.RobotArm(0.9f, 3, 4f), new Vector3D(0, 0, 0));
        var holding = new double[arm.VelocityCount];
        arm.ComputeGravityCompensation(holding);                  // joint torques that hold the present pose
        var mass = new double[arm.VelocityCount * arm.VelocityCount];
        arm.ComputeMassMatrix(mass);                              // M(q), row-major
        return (holding[0], holding[1]);
    }
}
