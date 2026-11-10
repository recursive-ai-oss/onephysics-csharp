using System;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;
using Xunit;


namespace RecursiveAI.OnePhysics.Interchange.Tests;

/// <summary>Mutated sample files: every input either imports or is refused with an <see cref="InterchangeException"/>; nothing else may escape.</summary>
public sealed class HostileInputTests
{
    private static string Mutate(string s, Random rng)
    {
        var sb = new StringBuilder(s);
        int edits = 1 + rng.Next(4);
        for (int e = 0; e < edits && sb.Length > 0; e++)
        {
            int p = rng.Next(sb.Length);
            switch (rng.Next(6))
            {
                case 0: sb.Remove(p, Math.Min(sb.Length - p, 1 + rng.Next(8))); break;
                case 1: sb[p] = (char)rng.Next(32, 127); break;
                case 2: sb.Insert(p, new[] { "-1", "1e300", "NaN", "0", "99999999999", "\"", "<", "[", "{", "inf" }[rng.Next(10)]); break;
                case 3: { int q = rng.Next(sb.Length); sb.Insert(p, sb.ToString(q, Math.Min(sb.Length - q, 12))); break; }
                case 4: sb[p] = char.IsDigit(sb[p]) ? (char)('0' + rng.Next(10)) : sb[p]; break;
                default: sb.Length = p; break;
            }
        }

        return sb.ToString();
    }

    private static void Run(string sample, Action<string> import, int seed, int count = 1500)
    {
        var rng = new Random(seed);
        int accepted = 0;
        for (int i = 0; i < count; i++)
        {
            string text = Mutate(sample, rng);
            try { import(text); accepted++; }
            catch (InterchangeException) { }
        }

        // The mutations are small, so a fair share still imports: the test would pass vacuously if every one were refused.
        Assert.True(accepted > count / 20, "only " + accepted + " of " + count + " mutations imported");
    }

    private static void Create(ArticulationDef d)
    {
        var w = new PhysicsWorld3D();
        try { w.AddArticulation(d); } catch (ArgumentException) { }
    }

    [Fact]
    public void Urdf() => Run(Samples.Read("arm6.urdf"), t => Create(UrdfImporter.Import(t).Definition), 1);

    [Fact]
    public void Mjcf() => Run(Samples.Read("arm6.xml"), t => Create(MjcfImporter.Import(t).Definition), 2);

    [Fact]
    public void Tmj() => Run(Samples.Read("level.tmj"), t => TiledImporter.Import(t), 3);

    [Fact]
    public void Tmx() => Run(Samples.Read("level.tmx"), t => TiledImporter.Import(t), 4);

    [Fact]
    public void Ldtk() => Run(Samples.Read("level.ldtk"), t => LdtkImporter.Import(t), 5);

    [Fact]
    public void Gltf()
    {
        var rng = new Random(6);
        string sample = InterchangeSamples.Twenty();
        int accepted = 0;
        for (int i = 0; i < 800; i++)
        {
            string text = Mutate(sample, rng);
            try
            {
                var m = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(text));
                accepted++;
                try { m.Scene.CreateWorld3D(1); } catch (RecursiveAI.OnePhysics.Scene.SceneFormatException) { }
            }
            catch (InterchangeException) { }
        }

        Assert.True(accepted > 40);
    }

    [Fact]
    public void DeepNestingAndHugeInputsAreRefused()
    {
        Assert.Throws<InterchangeException>(() => UrdfImporter.Import(new string('<', 10) + "x"));
        string deep = string.Concat(System.Linq.Enumerable.Repeat("<b>", 100000));
        Assert.Throws<InterchangeException>(() => MjcfImporter.Import("<mujoco><worldbody>" + deep + "</worldbody></mujoco>"));
        Assert.Throws<InterchangeException>(() => TiledImporter.Import(new string('[', 100000)));
        Assert.Throws<InterchangeException>(() => LdtkImporter.Import("{\"levels\":" + new string('[', 100000)));
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(new byte[] { 0x67, 0x6C, 0x54, 0x46, 2, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0x7F, 0, 0, 0, 0, 0, 0, 0, 0 }));
        // A DOCTYPE could pull in entities; it is refused outright.
        Assert.Throws<InterchangeException>(() => UrdfImporter.Import("<!DOCTYPE robot [<!ENTITY a \"b\">]><robot><link name=\"a\"/></robot>"));
    }

    [Fact]
    public void InvalidTextEncodingsAreRefused()
    {
        // A lone UTF-16 surrogate in a JSON string, and invalid UTF-8 in a glTF string (found by the fuzz target).
        Assert.Throws<InterchangeException>(() => TiledImporter.Import("{\"orientation\":\"orthogonal\",\"width\":1,\"height\":1,\"tilewidth\":8,\"tileheight\":8,\"x\":\"\ud800\"}"));
        Assert.Throws<InterchangeException>(() => LdtkImporter.Import("{\"levels\":[{\"identifier\":\"\ud800\",\"pxWid\":8,\"pxHei\":8,\"layerInstances\":[]}]}", "\ud800"));
        Assert.Throws<InterchangeException>(() => UrdfImporter.Import("<robot><link name=\"\ud800\"/></robot>"));
        Assert.Throws<InterchangeException>(() => MjcfImporter.Import("<mujoco><worldbody><body name=\"\udc00\"/></worldbody></mujoco>"));
        var bytes = Encoding.UTF8.GetBytes(InterchangeSamples.Twenty());
        int at = Array.IndexOf(bytes, (byte)'f', 100);
        bytes[at] = 0xF3;
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(bytes));
    }

    [Fact]
    public void ErrorsNameTheElement()
    {
        var e = Assert.Throws<InterchangeException>(() => UrdfImporter.Import("<robot><link name=\"a\"/><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"zz\"/></joint></robot>"));
        Assert.Equal("URDF", e.Format);
        Assert.Contains("zz", e.Message);
        Assert.Contains("joint[j]", e.Location);
        var m = Assert.Throws<InterchangeException>(() => MjcfImporter.Import("<mujoco><worldbody><body name=\"b\"><geom type=\"sphere\" size=\"-1\"/></body></worldbody></mujoco>"));
        Assert.Contains("body[b]", m.Location);
        var t = Assert.Throws<InterchangeException>(() => TiledImporter.Import("{\"orientation\":\"isometric\",\"width\":1,\"height\":1,\"tilewidth\":8,\"tileheight\":8}"));
        Assert.Contains("orthogonal", t.Message);
    }
}
