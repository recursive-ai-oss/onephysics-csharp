using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>Every definition struct means its documented defaults at its zero value (<c>default</c>, a class field, an array element).</summary>
public class ZeroDefaultTests
{
    private const float Dt = 1f / 60f;

    /// <summary>The recorded property values of <c>new XDef()</c>, taken before definitions were made valid at zero; the zero value must read back exactly the same.</summary>
    [Fact]
    public void ZeroValueOfEveryDefReadsAsTheDocumentedDefaults()
    {
        var asm = typeof(P2.PhysicsWorld2D).Assembly;
        var sb = new StringBuilder();
        foreach (var t in asm.GetExportedTypes().Where(t => t.IsValueType && !t.IsEnum && t.Name.EndsWith("Def", StringComparison.Ordinal)).OrderBy(t => t.FullName, StringComparer.Ordinal))
        {
            object zero = Activator.CreateInstance(t)!;
            Dump(sb, t.FullName!.Replace("RecursiveAI.OnePhysics.", "", StringComparison.Ordinal), zero, 0);
        }

        string expected = File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "api", "def-defaults.txt")).Replace("\r\n", "\n", StringComparison.Ordinal);
        Assert.Equal(expected, sb.ToString().Replace("\r\n", "\n", StringComparison.Ordinal));
    }

    private static void Dump(StringBuilder sb, string prefix, object? o, int depth)
    {
        if (o is null) { sb.Append(prefix).Append("=null\n"); return; }
        var t = o.GetType();
        if (o is float f) { sb.Append(prefix).Append("=f").Append(BitConverter.SingleToUInt32Bits(f).ToString("X8", System.Globalization.CultureInfo.InvariantCulture)).Append('\n'); return; }
        if (o is double d) { sb.Append(prefix).Append("=d").Append(BitConverter.DoubleToUInt64Bits(d).ToString("X16", System.Globalization.CultureInfo.InvariantCulture)).Append('\n'); return; }
        if (t.IsPrimitive || t.IsEnum || o is string) { sb.Append(prefix).Append('=').Append(Convert.ToString(o, System.Globalization.CultureInfo.InvariantCulture)).Append('\n'); return; }
        if (depth > 3) { return; }
        if (t.IsArray) { sb.Append(prefix).Append("=array\n"); return; }
        foreach (var fi in t.GetFields(BindingFlags.Public | BindingFlags.Instance).OrderBy(x => x.Name, StringComparer.Ordinal)) { Dump(sb, prefix + "." + fi.Name, fi.GetValue(o), depth + 1); }
        foreach (var p in t.GetProperties(BindingFlags.Public | BindingFlags.Instance).OrderBy(x => x.Name, StringComparer.Ordinal))
        {
            if (p.GetIndexParameters().Length > 0 || !p.CanRead) { continue; }
            object? v;
            try { v = p.GetValue(o); }
            catch (TargetInvocationException e) { sb.Append(prefix).Append('.').Append(p.Name).Append("=throws ").Append(e.InnerException?.GetType().Name).Append('\n'); continue; }
            if (p.PropertyType.IsValueType) { Dump(sb, prefix + "." + p.Name, v, depth + 1); }
            else { sb.Append(prefix).Append('.').Append(p.Name).Append('=').Append(v is null ? "null" : "obj").Append('\n'); }
        }
    }

    [Fact]
    public void ZeroValueBodyAndShapeDefsBehaveLikeTheDocumentedDefaults2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 3 });
        w.AddBox(new Vector2F(40, 2), at: new Vector2D(0, -1), type: BodyType.Static);
        P2.BodyDef bd = default;
        bd.Position = new Vector2D(0, 3);
        var body = w.CreateBody(bd);
        P2.ShapeDef sd = default;
        w.CreateCircle(body, 0.5f, def: sd);
        for (int i = 0; i < 180; i++) { w.Step(Dt); }

        Assert.Equal(BodyType.Dynamic, body.Type);
        Assert.InRange(body.Position.Y, 0.45, 0.55);
        Assert.True(body.Mass > 0f);
    }

    [Fact]
    public void ZeroValueBodyAndShapeDefsBehaveLikeTheDocumentedDefaults3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 3 });
        w.AddBox(new Vector3F(40, 2, 40), at: new Vector3D(0, -1, 0), type: BodyType.Static);
        P3.BodyDef bd = default;
        bd.Position = new Vector3D(0, 3, 0);
        var body = w.CreateBody(bd);
        P3.ShapeDef sd = default;
        w.CreateSphere(body, 0.5f, sd);
        for (int i = 0; i < 180; i++) { w.Step(Dt); }

        Assert.Equal(BodyType.Dynamic, body.Type);
        Assert.InRange(body.Position.Y, 0.45, 0.55);
        Assert.True(body.Mass > 0f);
    }

    [Fact]
    public void ZeroValueJointDefsAreUnbreakableAndHold2D()
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 4 });
        var a = w.AddCircle(0.2f, new Vector2D(0, 5), BodyType.Static);
        var b = w.AddCircle(0.3f, new Vector2D(1, 5));
        var joints = new P2.Joint[7];
        var rev = default(P2.RevoluteJointDef); rev.BodyA = a; rev.BodyB = b; rev.LocalAnchorA = new Vector2F(1, 0);
        joints[0] = w.CreateJoint(rev);
        var pri = default(P2.PrismaticJointDef); pri.BodyA = a; pri.BodyB = b; pri.LocalAnchorA = new Vector2F(1, 0);
        joints[1] = w.CreateJoint(pri);
        var weld = default(P2.WeldJointDef); weld.BodyA = a; weld.BodyB = b; weld.LocalAnchorA = new Vector2F(1, 0);
        joints[2] = w.CreateJoint(weld);
        var wheel = default(P2.WheelJointDef); wheel.BodyA = a; wheel.BodyB = b; wheel.LocalAnchorA = new Vector2F(1, 0);
        joints[3] = w.CreateJoint(wheel);
        var dist = default(P2.DistanceJointDef); dist.BodyA = a; dist.BodyB = b;
        joints[4] = w.CreateJoint(dist);
        var motor = default(P2.MotorJointDef); motor.BodyA = a; motor.BodyB = b;
        joints[5] = w.CreateJoint(motor);
        var mouse = default(P2.MouseJointDef); mouse.BodyA = a; mouse.BodyB = b;
        joints[6] = w.CreateJoint(mouse);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        foreach (var j in joints) { Assert.True(j.IsValid); }
    }

    [Fact]
    public void ZeroValueJointDefsAreUnbreakableAndHold3D()
    {
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 4 });
        var a = w.AddSphere(0.2f, new Vector3D(0, 5, 0), BodyType.Static);
        var b = w.AddSphere(0.3f, new Vector3D(1, 5, 0));
        var joints = new P3.Joint[11];
        var fixedJ = default(P3.FixedJointDef); fixedJ.BodyA = a; fixedJ.BodyB = b; joints[0] = w.CreateJoint(fixedJ);
        var point = default(P3.PointJointDef); point.BodyA = a; point.BodyB = b; joints[1] = w.CreateJoint(point);
        var dist = default(P3.DistanceJointDef); dist.BodyA = a; dist.BodyB = b; joints[2] = w.CreateJoint(dist);
        var hinge = default(P3.HingeJointDef); hinge.BodyA = a; hinge.BodyB = b; joints[3] = w.CreateJoint(hinge);
        var slider = default(P3.SliderJointDef); slider.BodyA = a; slider.BodyB = b; joints[4] = w.CreateJoint(slider);
        var cone = default(P3.ConeJointDef); cone.BodyA = a; cone.BodyB = b; joints[5] = w.CreateJoint(cone);
        var st = default(P3.SwingTwistJointDef); st.BodyA = a; st.BodyB = b; joints[6] = w.CreateJoint(st);
        var six = default(P3.SixDofJointDef); six.BodyA = a; six.BodyB = b; joints[7] = w.CreateJoint(six);
        var path = default(P3.PathJointDef); path.BodyA = a; path.BodyB = b; path.Points = new[] { new Vector3F(0, 5, 0), new Vector3F(2, 5, 0), new Vector3F(4, 5, 0) }; joints[8] = w.CreateJoint(path);
        var pulley = default(P3.PulleyJointDef); pulley.BodyA = a; pulley.BodyB = b; joints[9] = w.CreateJoint(pulley);
        var c = w.AddSphere(0.3f, new Vector3D(-1, 5, 0));
        var gear = default(P3.GearJointDef); gear.BodyA = b; gear.BodyB = c; joints[10] = w.CreateJoint(gear);
        for (int i = 0; i < 30; i++) { w.Step(Dt); }
        foreach (var j in joints) { Assert.True(j.IsValid); }
    }

    [Fact]
    public void ZeroValueCharacterDefsAreTheHumanoidPreset()
    {
        var w2 = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 5 });
        P2.CharacterDef d2 = default;
        Assert.Equal(P2.CharacterDef.Humanoid.Radius, d2.Radius);
        Assert.Equal(P2.CharacterDef.Humanoid.Height, d2.Height);
        var c2 = w2.AddCharacter(new Vector2D(0, 2), d2);
        Assert.True(c2.IsValid);

        var w3 = new P3.PhysicsWorld3D(new P3.WorldOptions3D { WorldId = 5 });
        P3.CharacterDef d3 = default;
        Assert.Equal(P3.CharacterDef.Humanoid.Radius, d3.Radius);
        Assert.Equal(new Vector3F(0, 1, 0), d3.Up);
        var c3 = w3.AddCharacter(new Vector3D(0, 2, 0), d3);
        Assert.True(c3.IsValid);
    }

    [Fact]
    public void FieldsRoundTripAnyValueIncludingNonDefaultsAndSpecials()
    {
        P2.ShapeDef sd = default;
        sd.Density = 0f; sd.Friction = float.NaN; sd.MaskBits = 0u; sd.CategoryBits = uint.MaxValue;
        Assert.Equal(0f, sd.Density);
        Assert.True(float.IsNaN(sd.Friction));
        Assert.Equal(0u, sd.MaskBits);
        Assert.Equal(uint.MaxValue, sd.CategoryBits);
        P2.BodyDef bd = default;
        bd.Type = BodyType.Static; bd.AllowSleep = false; bd.GravityScale = -0f;
        Assert.Equal(BodyType.Static, bd.Type);
        Assert.False(bd.AllowSleep);
        Assert.True(float.IsNegative(bd.GravityScale));
        var rev = default(P2.RevoluteJointDef);
        rev.BreakForce = 12f;
        Assert.Equal(12f, rev.BreakForce);
        Assert.Equal(float.PositiveInfinity, rev.BreakTorque);
    }
}
