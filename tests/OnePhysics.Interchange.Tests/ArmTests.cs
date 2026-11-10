using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;


namespace RecursiveAI.OnePhysics.Interchange.Tests;

public sealed class ArmTests
{
    private static void Near(float a, float b, string what) => Assert.True(Math.Abs(a - b) <= 1e-6f * Math.Max(1f, Math.Abs(a)), what + ": " + a + " vs " + b);

    internal static ulong Run(ArticulationDef def, int steps = 600)
    {
        var w = new PhysicsWorld3D(new WorldOptions3D { Gravity = new Vector3F(0, 0, -9.81f) });
        var art = w.AddArticulation(def);
        for (int i = 1; i < def.Links.Count; i++)
        {
            art.SetDrive(i, new ArticulationDrive { Stiffness = 60f, Damping = 6f, MaxForce = 50f, TargetPosition = 0.25f * i / 6f });
        }

        for (int i = 0; i < steps; i++) { w.Step(1f / 60f); }
        return w.ComputeHash();
    }

    [Fact]
    public void UrdfAndMjcfArmsAreTheSameDefinition()
    {
        var u = UrdfImporter.Import(Samples.Read("arm6.urdf")).Definition;
        var m = MjcfImporter.Import(Samples.Read("arm6.xml")).Definition;
        Assert.Equal(7, u.Links.Count);
        Assert.Equal(u.Links.Count, m.Links.Count);
        Assert.True(u.FixedBase && m.FixedBase);
        for (int i = 0; i < u.Links.Count; i++)
        {
            var a = u.Links[i]; var b = m.Links[i];
            string at = "link " + i + " ";
            Assert.Equal(a.Parent, b.Parent);
            Assert.Equal(a.Joint, b.Joint);
            Near(a.Mass, b.Mass, at + "mass");
            Near(a.Origin.X, b.Origin.X, at + "origin.x"); Near(a.Origin.Y, b.Origin.Y, at + "origin.y"); Near(a.Origin.Z, b.Origin.Z, at + "origin.z");
            Near(a.OriginRotation.W, b.OriginRotation.W, at + "q.w"); Near(a.OriginRotation.X, b.OriginRotation.X, at + "q.x");
            Near(a.CenterOfMass.Z, b.CenterOfMass.Z, at + "com");
            Near(a.InertiaTensor.M11, b.InertiaTensor.M11, at + "Ixx"); Near(a.InertiaTensor.M33, b.InertiaTensor.M33, at + "Izz");
            if (i > 0)
            {
                Near(a.Axis.X, b.Axis.X, at + "axis"); Near(a.Axis.Z, b.Axis.Z, at + "axis");
                Assert.Equal(a.HasLimits, b.HasLimits);
                Near(a.LowerLimit, b.LowerLimit, at + "lower"); Near(a.UpperLimit, b.UpperLimit, at + "upper");
                Near(a.Damping, b.Damping, at + "damping"); Near(a.Friction, b.Friction, at + "friction");
                Near(a.Drive.MaxForce, b.Drive.MaxForce, at + "effort");
            }

            Assert.Equal(a.Shapes.Count, b.Shapes.Count);
        }

        Assert.Equal(Run(u), Run(m));
    }

    [Fact]
    public void ArmMovesWhenDriven()
    {
        var def = UrdfImporter.Import(Samples.Read("arm6.urdf")).Definition;
        var w = new PhysicsWorld3D(new WorldOptions3D { Gravity = new Vector3F(0, 0, -9.81f) });
        var art = w.AddArticulation(def);
        art.SetDrive(1, new ArticulationDrive { Stiffness = 60f, Damping = 6f, MaxForce = 50f, TargetPosition = 0.8f });
        for (int i = 0; i < 300; i++) { w.Step(1f / 60f); }
        Assert.InRange(art.GetJointPosition(1), 0.5f, 1.0f);
    }

    [Fact]
    public void UrdfWarningsAndNames()
    {
        var m = UrdfImporter.Import(Samples.Read("arm6.urdf"));
        Assert.Equal("arm6", m.RobotName);
        Assert.Equal(6, m.FindJoint("joint6") );
        Assert.Equal(0, m.FindLink("base_link"));
        Assert.Empty(m.Warnings);
    }
}
