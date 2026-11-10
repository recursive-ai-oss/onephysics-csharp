using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;

#pragma warning disable OPHYS0003, OPHYS0002

namespace RecursiveAI.OnePhysics.Interchange.Tests;

public sealed class RobotTests
{
    private static void Near(double expected, double actual, double tol, string what) => Assert.True(Math.Abs(expected - actual) <= tol, what + ": expected " + expected + ", got " + actual);

    private const string Inertial = "<inertial><mass value=\"1\"/><inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/></inertial>";

    [Fact]
    public void UrdfWorldLinkAndMasslessToolLinkSimulate()
    {
        string xml = "<robot name=\"t\"><link name=\"world\"/><link name=\"base\">" + Inertial + "</link><link name=\"arm\">" + Inertial + "<collision><geometry><sphere radius=\"0.1\"/></geometry></collision></link><link name=\"tool\"/>" +
            "<joint name=\"w\" type=\"fixed\"><parent link=\"world\"/><child link=\"base\"/></joint>" +
            "<joint name=\"j\" type=\"revolute\"><parent link=\"base\"/><child link=\"arm\"/><origin xyz=\"0 0 0.5\"/><axis xyz=\"0 1 0\"/><limit lower=\"-3\" upper=\"3\" effort=\"1\"/></joint>" +
            "<joint name=\"t\" type=\"fixed\"><parent link=\"arm\"/><child link=\"tool\"/><origin xyz=\"0.3 0 0\"/></joint></robot>";
        var m = UrdfImporter.Import(xml);
        Assert.Equal(4, m.Definition.Links.Count);
        var w = new PhysicsWorld3D(new WorldOptions3D { Gravity = new Vector3F(0, 0, -9.81f) });
        var art = w.AddArticulation(m.Definition);
        for (int i = 0; i < 120; i++) { w.Step(1f / 60f); }
        // The arm hangs from its hinge and swings under gravity: it moved.
        Assert.True(Math.Abs(art.GetJointPosition(m.FindLink("arm"))) > 1e-3);
    }

    [Fact]
    public void UrdfReportsWhatItCannotExpress()
    {
        string xml = "<robot name=\"t\"><link name=\"a\">" + Inertial + "</link><link name=\"b\">" + Inertial + "<collision><geometry><mesh filename=\"m.stl\"/></geometry></collision></link><link name=\"c\">" + Inertial + "</link>" +
            "<joint name=\"j1\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/><axis xyz=\"0 0 1\"/><limit lower=\"0\" upper=\"1\"/></joint>" +
            "<joint name=\"j2\" type=\"planar\"><parent link=\"a\"/><child link=\"c\"/><mimic joint=\"j1\"/></joint></robot>";
        var m = UrdfImporter.Import(xml);
        var codes = m.Warnings.Select(x => x.Code).ToList();
        Assert.Contains("mesh-unresolved", codes);
        Assert.Contains("planar-joint", codes);
        Assert.Contains("mimic-joint", codes);
        Assert.Equal(ArticulationJointKind.Fixed, m.Definition.Links[m.FindLink("c")].Joint);
    }

    [Fact]
    public void UrdfMeshThroughResolverBecomesAHull()
    {
        var tet = new InterchangeMesh(new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, 0, 1) }, new[] { 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3 });
        string xml = "<robot><link name=\"a\">" + Inertial + "<collision><geometry><mesh filename=\"pkg://x/m.stl\" scale=\"2 2 2\"/></geometry></collision></link></robot>";
        var m = UrdfImporter.Import(xml, new UrdfOptions { Meshes = n => n == "pkg://x/m.stl" ? tet : null });
        Assert.Empty(m.Warnings);
        Assert.Single(m.Definition.Links[0].Shapes);
        new PhysicsWorld3D().AddArticulation(m.Definition);
    }

    [Fact]
    public void UrdfInertialFrameRotationRotatesTheTensor()
    {
        // A tensor diag(1, 2, 3) in a frame turned 90 degrees about z has x and y swapped in the link frame.
        string xml = "<robot><link name=\"a\"><inertial><origin xyz=\"0.1 0.2 0.3\" rpy=\"0 0 1.5707963267948966\"/><mass value=\"4\"/><inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"2\" iyz=\"0\" izz=\"3\"/></inertial></link></robot>";
        var l = UrdfImporter.Import(xml).Definition.Links[0];
        Near(2, l.InertiaTensor.M11, 1e-6, "Ixx"); Near(1, l.InertiaTensor.M22, 1e-6, "Iyy"); Near(3, l.InertiaTensor.M33, 1e-6, "Izz");
        Near(0.3, l.CenterOfMass.Z, 1e-7, "com z");
    }

    [Fact]
    public void UrdfRpyIsRollPitchYawAboutFixedAxes()
    {
        string xml = "<robot><link name=\"a\">" + Inertial + "</link><link name=\"b\">" + Inertial + "</link><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/><origin rpy=\"0.3 -0.4 0.5\"/></joint></robot>";
        var q = UrdfImporter.Import(xml).Definition.Links[1].OriginRotation;
        // R = Rz(0.5) Ry(-0.4) Rx(0.3): rotate the x axis and compare with the matrix product.
        var v = q.Rotate(new Vector3F(1, 0, 0));
        double cy = Math.Cos(-0.4), sy = Math.Sin(-0.4), cz = Math.Cos(0.5), sz = Math.Sin(0.5);
        Near(cz * cy, v.X, 1e-6, "x"); Near(sz * cy, v.Y, 1e-6, "y"); Near(-sy, v.Z, 1e-6, "z");
    }

    [Theory]
    [InlineData("<link name=\"a\"/><link name=\"b\"/>", "root")]
    [InlineData("<link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"b\"/></joint><joint name=\"k\" type=\"fixed\"><parent link=\"b\"/><child link=\"a\"/></joint>", "loop")]
    [InlineData("<link name=\"a\"/><link name=\"b\"/><link name=\"c\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"c\"/></joint><joint name=\"k\" type=\"fixed\"><parent link=\"b\"/><child link=\"c\"/></joint>", "parent")]
    [InlineData("<link name=\"a\"><inertial><mass value=\"-1\"/></inertial></link>", "negative")]
    [InlineData("<link name=\"a\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"a\"/><limit lower=\"0\" upper=\"1\"/></joint>", "itself")]
    public void UrdfStructureErrors(string body, string word)
    {
        var e = Assert.Throws<InterchangeException>(() => UrdfImporter.Import("<robot>" + body + "</robot>"));
        Assert.Contains(word, e.Message, StringComparison.OrdinalIgnoreCase);
    }

    private static MjcfModel Mj(string body, string header = "") => MjcfImporter.Import("<mujoco>" + header + "<worldbody>" + body + "</worldbody></mujoco>");

    [Fact]
    public void MjcfTwoJointsOnOneBodyBecomeTwoLinks()
    {
        var m = Mj("<body name=\"b\" pos=\"0 0 1\"><geom type=\"sphere\" size=\"0.1\" pos=\"0 0 0.5\"/><body name=\"c\" pos=\"0 0 1\"><joint name=\"yaw\" axis=\"0 0 1\" pos=\"0 0 0.2\"/><joint name=\"pitch\" axis=\"0 1 0\" pos=\"0 0 0.5\"/><geom type=\"sphere\" size=\"0.1\" pos=\"0 0 0.5\"/></body></body>");
        var d = m.Definition;
        Assert.Equal(3, d.Links.Count);
        Assert.Equal(0, d.Links[1].Parent); Assert.Equal(1, d.Links[2].Parent);
        Assert.Equal(new[] { "b", "yaw", "c" }, m.LinkNames.ToArray());
        Assert.Equal(new string?[] { null, "yaw", "pitch" }, m.JointNames.ToArray());
        // The yaw link sits at its anchor (body c at z = 1 plus 0.2); the pitch link is 0.3 further up; the geometry hangs on the last link.
        Near(1.2, d.Links[1].Origin.Z, 1e-6, "yaw origin"); Near(0.3, d.Links[2].Origin.Z, 1e-6, "pitch origin");
        Near(0.0, d.Links[2].Shapes[0].Def.LocalPosition.Z - 0.0, 0.0, "shape relative to the pitch anchor") ;
    }

    [Fact]
    public void MjcfJointAnchorShiftsTheLinkFrame()
    {
        // A body at (0,0,1) with a hinge anchored 0.2 up its own z: the link frame sits at (0,0,1.2), and the sphere at body-frame z 0.5 is at link-frame z 0.3.
        var m = Mj("<body name=\"root\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"c\" pos=\"0 0 1\"><joint axis=\"0 1 0\" pos=\"0 0 0.2\"/><geom type=\"sphere\" size=\"0.1\" pos=\"0 0 0.5\" mass=\"2\"/></body></body>");
        var l = m.Definition.Links[1];
        Near(1.2, l.Origin.Z, 1e-6, "origin"); Near(0.3, l.CenterOfMass.Z, 1e-6, "com"); Near(0.3, l.Shapes[0].Def.LocalPosition.Z, 1e-6, "shape");
        Near(2, l.Mass, 1e-6, "mass");
        // The solid sphere's inertia about its own centre: 2/5 m r^2.
        Near(0.4 * 2 * 0.01, l.InertiaTensor.M11, 1e-6, "inertia");
    }

    [Fact]
    public void MjcfDefaultsClassesAndUnits()
    {
        var m = MjcfImporter.Import("<mujoco><compiler angle=\"degree\"/><default><joint damping=\"0.5\" range=\"-90 90\"/><default class=\"soft\"><joint damping=\"2\"/></default></default><worldbody><body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/>" +
            "<body name=\"a\" childclass=\"soft\"><joint axis=\"0 1 0\"/><geom type=\"sphere\" size=\"0.1\"/></body><body name=\"b\"><joint axis=\"0 1 0\"/><geom type=\"sphere\" size=\"0.1\"/></body></body></worldbody></mujoco>");
        var a = m.Definition.Links[m.FindLink("a")]; var b = m.Definition.Links[m.FindLink("b")];
        Near(2, a.Damping, 1e-9, "class damping"); Near(0.5, b.Damping, 1e-9, "default damping");
        Assert.True(a.HasLimits);
        Near(-Math.PI / 2, a.LowerLimit, 1e-6, "range in radians"); Near(Math.PI / 2, a.UpperLimit, 1e-6, "range in radians");
    }

    [Fact]
    public void MjcfEulerSequenceAndOrientationForms()
    {
        // eulerseq "zyx" (lower case: turning axes), 90 degrees about z then 90 about the new y: R = Rz Ry, x -> -z.
        var m = Mj("<body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"c\" euler=\"90 90 0\"><geom type=\"sphere\" size=\"0.1\"/></body></body>", "<compiler eulerseq=\"zyx\"/>");
        var v = m.Definition.Links[1].OriginRotation.Rotate(new Vector3F(1, 0, 0));
        Near(0, v.X, 1e-6, "x"); Near(0, v.Y, 1e-6, "y"); Near(-1, v.Z, 1e-6, "z");
        // Upper case turns about the fixed axes: Z then Y gives R = Ry Rz, x -> y.
        var f = Mj("<body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"c\" euler=\"90 90 0\"><geom type=\"sphere\" size=\"0.1\"/></body></body>", "<compiler eulerseq=\"ZYX\"/>").Definition.Links[1].OriginRotation.Rotate(new Vector3F(1, 0, 0));
        Near(1, f.Y, 1e-6, "fixed-axis y");
        var q = Mj("<body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"c\" axisangle=\"0 0 1 90\"><geom type=\"sphere\" size=\"0.1\"/></body></body>").Definition.Links[1].OriginRotation.Rotate(new Vector3F(1, 0, 0));
        Near(1, q.Y, 1e-6, "axisangle");
        var z = Mj("<body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"c\" zaxis=\"1 0 0\"><geom type=\"sphere\" size=\"0.1\"/></body></body>").Definition.Links[1].OriginRotation.Rotate(new Vector3F(0, 0, 1));
        Near(1, z.X, 1e-6, "zaxis");
        var xy = Mj("<body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"c\" xyaxes=\"0 1 0 -1 0 0\"><geom type=\"sphere\" size=\"0.1\"/></body></body>").Definition.Links[1].OriginRotation.Rotate(new Vector3F(1, 0, 0));
        Near(1, xy.Y, 1e-6, "xyaxes");
    }

    [Fact]
    public void MjcfActuatorsBecomeDrives()
    {
        var model = MjcfImporter.Import("<mujoco><worldbody><body name=\"r\"><geom type=\"sphere\" size=\"0.1\"/><body name=\"a\"><joint name=\"ja\" axis=\"0 1 0\"/><geom type=\"sphere\" size=\"0.1\"/></body><body name=\"b\"><joint name=\"jb\" axis=\"0 1 0\"/><geom type=\"sphere\" size=\"0.1\"/></body><body name=\"c\"><joint name=\"jc\" axis=\"0 1 0\"/><geom type=\"sphere\" size=\"0.1\"/></body></body></worldbody>" +
            "<actuator><position joint=\"ja\" kp=\"40\" kv=\"4\" forcerange=\"-10 10\"/><velocity joint=\"jb\" kv=\"3\"/><motor joint=\"jc\" ctrlrange=\"-2 2\" gear=\"1\"/><general joint=\"ja\"/></actuator></mujoco>");
        var d = model.Definition;
        var a = d.Links[model.FindJoint("ja")].Drive; var b = d.Links[model.FindJoint("jb")].Drive; var c = d.Links[model.FindJoint("jc")].Drive;
        Assert.Equal((40f, 4f, 10f), (a.Stiffness, a.Damping, a.MaxForce));
        Assert.Equal((0f, 3f), (b.Stiffness, b.Damping));
        Assert.Equal(2f, c.MaxForce);
        Assert.Contains(model.Warnings, x => x.Code == "unsupported-actuator");
        Assert.Contains(model.Warnings, x => x.Code == "motor-actuator");
    }

    [Fact]
    public void MjcfPlaneAndWorldGeomsAreStaticShapes()
    {
        var m = Mj("<geom name=\"floor\" type=\"plane\" size=\"5 5 0.1\" pos=\"0 0 0.5\"/><body name=\"b\"><freejoint/><geom type=\"box\" size=\"0.1 0.1 0.1\"/></body>");
        var f = Assert.Single(m.WorldShapes);
        Assert.Equal("floor", f.Name);
        Assert.False(m.Definition.FixedBase);
        var w = new PhysicsWorld3D(new WorldOptions3D { Gravity = new Vector3F(0, 0, -10) });
        var floor = w.CreateBody(new BodyDef { Type = BodyType.Static });
        w.CreateShape(floor, f.Shape, new ShapeDef { Friction = f.Friction });
        var art = w.AddArticulation(m.Definition, new Vector3D(0, 0, 1.5));
        for (int i = 0; i < 300; i++) { w.Step(1f / 60f); }
        // The box dropped onto the plane at z = 0.5 and rests with its centre 0.1 above it.
        Near(0.6, art.GetLink(0).Position.Z, 0.02, "resting height");
    }

    [Fact]
    public void MjcfGeomMassesFromDensityAndMesh()
    {
        // A unit cube by density 1000 weighs 1000 kg; the same cube as a closed mesh gives the same mass and inertia.
        var cube = new InterchangeMesh(new[] { new Vector3F(-.5f, -.5f, -.5f), new Vector3F(.5f, -.5f, -.5f), new Vector3F(.5f, .5f, -.5f), new Vector3F(-.5f, .5f, -.5f), new Vector3F(-.5f, -.5f, .5f), new Vector3F(.5f, -.5f, .5f), new Vector3F(.5f, .5f, .5f), new Vector3F(-.5f, .5f, .5f) },
            new[] { 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5 });
        var box = Mj("<body><geom type=\"box\" size=\"0.5 0.5 0.5\"/></body>").Definition.Links[0];
        var mesh = MjcfImporter.Import("<mujoco><asset><mesh name=\"c\" file=\"c.obj\"/></asset><worldbody><body><geom type=\"mesh\" mesh=\"c\"/></body></worldbody></mujoco>", new MjcfOptions { Meshes = _ => cube }).Definition.Links[0];
        Near(1000, box.Mass, 1e-3, "box mass"); Near(1000, mesh.Mass, 1e-2, "mesh mass");
        Near(box.InertiaTensor.M11, mesh.InertiaTensor.M11, 1e-2, "inertia"); Near(1000.0 / 6, box.InertiaTensor.M22, 1e-2, "analytic cube inertia");
        // Capsule and cylinder against their closed forms.
        var cap = Mj("<body><geom type=\"capsule\" size=\"0.1 0.3\" mass=\"2\"/></body>").Definition.Links[0];
        double r = 0.1, h = 0.3, vs = 4.0 / 3 * Math.PI * r * r * r, vc = Math.PI * r * r * 2 * h, ms = 2 * vs / (vs + vc), mc = 2 - ms;
        double along = 0.5 * mc * r * r + 0.4 * ms * r * r;
        // Capsules lie along y in the engine: the long axis inertia is Iyy.
        Near(along, cap.InertiaTensor.M22, 1e-6, "capsule axis inertia");
    }

    [Fact]
    public void MjcfStructureErrors()
    {
        Assert.Throws<InterchangeException>(() => Mj("<body name=\"a\"/><body name=\"b\"/>"));
        Assert.Throws<InterchangeException>(() => Mj("<body name=\"a\"><joint type=\"hinge\"/></body>"));
        Assert.Throws<InterchangeException>(() => Mj("<body name=\"a\"><body name=\"b\"><freejoint/></body></body>"));
        Assert.Throws<InterchangeException>(() => Mj("<body name=\"a\"><geom type=\"box\" size=\"1 1\"/></body>"));
        Assert.Throws<InterchangeException>(() => Mj("<body name=\"a\"><geom type=\"sphere\" size=\"1\" quat=\"0 0 0 0\"/></body>"));
        Assert.Throws<InterchangeException>(() => MjcfImporter.Import("<mujoco><worldbody><body name=\"a\"><geom size=\"1\" class=\"nope\"/></body></worldbody></mujoco>"));
        Assert.Throws<InterchangeException>(() => MjcfImporter.Import("<mujoco><worldbody><body name=\"a\"><geom size=\"1\"/></body></worldbody><actuator><motor joint=\"zz\"/></actuator></mujoco>"));
    }
}
