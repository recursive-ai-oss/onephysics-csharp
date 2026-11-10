using System;
using System.Buffers.Binary;
using System.Linq;
using System.Text;
using RecursiveAI.OneMath;
using Xunit;

namespace RecursiveAI.OnePhysics.Interchange.Tests;

public sealed class GltfMoreTests
{
    private static RecursiveAI.OnePhysics.Physics3D.Body[] All(RecursiveAI.OnePhysics.Physics3D.PhysicsWorld3D w)
    {
        var l = new System.Collections.Generic.List<RecursiveAI.OnePhysics.Physics3D.Body>();
        foreach (var b in w.Bodies) { l.Add(b); }
        return l.ToArray();
    }

    private static byte[] Glb(string json, byte[] bin)
    {
        var j = Encoding.UTF8.GetBytes(json);
        int jp = (4 - j.Length % 4) % 4, bp = (4 - bin.Length % 4) % 4;
        int total = 12 + 8 + j.Length + jp + (bin.Length > 0 ? 8 + bin.Length + bp : 0);
        var o = new byte[total];
        BinaryPrimitives.WriteUInt32LittleEndian(o, 0x46546C67); BinaryPrimitives.WriteUInt32LittleEndian(o.AsSpan(4), 2); BinaryPrimitives.WriteUInt32LittleEndian(o.AsSpan(8), (uint)total);
        int p = 12;
        BinaryPrimitives.WriteUInt32LittleEndian(o.AsSpan(p), (uint)(j.Length + jp)); BinaryPrimitives.WriteUInt32LittleEndian(o.AsSpan(p + 4), 0x4E4F534A); p += 8;
        j.CopyTo(o, p); for (int i = 0; i < jp; i++) { o[p + j.Length + i] = 0x20; } p += j.Length + jp;
        if (bin.Length > 0) { BinaryPrimitives.WriteUInt32LittleEndian(o.AsSpan(p), (uint)(bin.Length + bp)); BinaryPrimitives.WriteUInt32LittleEndian(o.AsSpan(p + 4), 0x004E4942); p += 8; bin.CopyTo(o, p); }
        return o;
    }

    private static byte[] QuadBin()
    {
        var b = new byte[48 + 12];
        float[] v = { -10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10 };
        for (int i = 0; i < 12; i++) { BinaryPrimitives.WriteInt32LittleEndian(b.AsSpan(4 * i), BitConverter.SingleToInt32Bits(v[i])); }
        ushort[] idx = { 0, 2, 1, 0, 3, 2 };   // counter-clockwise seen from above (+y)
        for (int i = 0; i < 6; i++) { BinaryPrimitives.WriteUInt16LittleEndian(b.AsSpan(48 + 2 * i), idx[i]); }
        return b;
    }

    private const string MeshDoc = "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":60}],\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":48},{\"buffer\":0,\"byteOffset\":48,\"byteLength\":12}]," +
        "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5123,\"count\":6,\"type\":\"SCALAR\"}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0},\"indices\":1}]}]," +
        "\"extensions\":{\"KHR_implicit_shapes\":{\"shapes\":[{\"type\":\"sphere\",\"sphere\":{\"radius\":0.5}}]}},\"scene\":0,\"scenes\":[{\"nodes\":[0,1]}],\"nodes\":[" +
        "{\"extensions\":{\"KHR_physics_rigid_bodies\":{\"collider\":{\"geometry\":{\"mesh\":0}}}}}," +
        "{\"translation\":[0,2,0],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"motion\":{\"mass\":1},\"collider\":{\"geometry\":{\"shape\":0}}}}}]}";

    [Fact]
    public void GlbMeshColliderCatchesASphere()
    {
        var model = GltfPhysicsImporter.Import(Glb(MeshDoc, QuadBin()));
        Assert.Empty(model.Warnings);
        Assert.Equal("mesh", model.Scene.Bodies[0].Shapes[0].Kind);
        var w = model.Scene.CreateWorld3D(1);
        for (int i = 0; i < 240; i++) { w.Step(1f / 60f); }
        var ball = All(w)[1];
        Assert.InRange(ball.Position.Y, 0.45, 0.55);
    }

    [Fact]
    public void BufferErrorsAreTyped()
    {
        var bin = QuadBin();
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Glb(MeshDoc.Replace("\"count\":4", "\"count\":400"), bin)));
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Glb(MeshDoc.Replace("\"POSITION\":0", "\"POSITION\":9"), bin)));
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Glb(MeshDoc.Replace("5123", "5126"), bin)));
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Glb(MeshDoc, bin.Take(30).ToArray())));
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(MeshDoc)));   // no binary chunk, no uri
        var e = Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes("{\"asset\":{\"version\":\"2.0\"},\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"children\":[0]}]}")));
        Assert.Contains("already has a parent", e.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void ExternalAndDataBuffers()
    {
        var bin = QuadBin();
        string ext = MeshDoc.Replace("{\"byteLength\":60}", "{\"byteLength\":60,\"uri\":\"quad.bin\"}");
        Assert.Throws<InterchangeException>(() => GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(ext)));
        var m = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(ext), new GltfOptions { Buffers = n => n == "quad.bin" ? bin : null });
        Assert.Equal(2, m.Scene.Bodies.Count);
        string data = MeshDoc.Replace("{\"byteLength\":60}", "{\"byteLength\":60,\"uri\":\"data:application/octet-stream;base64," + Convert.ToBase64String(bin) + "\"}");
        Assert.Equal(2, GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(data)).Scene.Bodies.Count);
    }

    // A box hanging from a hinge about z at the origin: A is static; the hinge nodes are children placed at the pivot (world origin) in both bodies.
    private const string HingeDoc = "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0,2]}],\"extensions\":{\"KHR_implicit_shapes\":{\"shapes\":[{\"type\":\"box\",\"box\":{\"size\":[0.2,0.2,0.2]}},{\"type\":\"box\",\"box\":{\"size\":[0.6,0.2,0.2]}}]}," +
        "\"KHR_physics_rigid_bodies\":{\"physicsJoints\":[{\"limits\":[{\"linearAxes\":[0,1,2],\"min\":0,\"max\":0},{\"angularAxes\":[0,1],\"min\":0,\"max\":0}]}]}}," +
        "\"nodes\":[{\"children\":[1],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"collider\":{\"geometry\":{\"shape\":0}}}}},{\"extensions\":{\"KHR_physics_rigid_bodies\":{\"joint\":{\"connectedNode\":3,\"joint\":0}}}}," +
        "{\"translation\":[0.8,0,0],\"children\":[3],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"motion\":{\"mass\":2},\"collider\":{\"geometry\":{\"shape\":1}}}}},{\"translation\":[-0.8,0,0]}]}";

    [Fact]
    public void HingeSwingsInItsPlane()
    {
        var model = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(HingeDoc));
        Assert.Empty(model.Warnings);
        var j = Assert.Single(model.Scene.Joints);
        Assert.Equal("swingtwist", j.Kind);
        Assert.Equal(1.0, Math.Abs(j.Uz));
        var w = model.Scene.CreateWorld3D(1);
        double lowest = 0, maxZ = 0, minR = 9, maxR = 0;
        for (int i = 0; i < 240; i++)
        {
            w.Step(1f / 60f);
            var p = All(w)[1].Position;
            lowest = Math.Min(lowest, p.Y); maxZ = Math.Max(maxZ, Math.Abs(p.Z));
            double r = Math.Sqrt(p.X * p.X + p.Y * p.Y); minR = Math.Min(minR, r); maxR = Math.Max(maxR, r);
        }

        Assert.True(lowest < -0.5, "the box swung down, lowest y = " + lowest);
        Assert.True(maxZ < 0.02, "it stays in the plane, z = " + maxZ);
        Assert.InRange(minR, 0.78, 0.82); Assert.InRange(maxR, 0.78, 0.82);
    }

    [Fact]
    public void OmiExtensionsAndUnsupportedPartsAreReported()
    {
        string doc = "{\"asset\":{\"version\":\"2.0\"},\"extensions\":{\"OMI_physics_shape\":{\"shapes\":[{\"type\":\"box\",\"box\":{\"size\":[1,1,1]}},{\"type\":\"sphere\"},{\"type\":\"weird\"}]}},\"scene\":0,\"scenes\":[{\"nodes\":[0,1,2]}],\"nodes\":[" +
            "{\"extensions\":{\"OMI_physics_body\":{\"motion\":{\"type\":\"dynamic\",\"mass\":3},\"collider\":{\"shape\":0}}}}," +
            "{\"translation\":[3,0,0],\"extensions\":{\"OMI_physics_body\":{\"collider\":{\"shape\":1},\"trigger\":{\"shape\":1}}}}," +
            "{\"translation\":[6,0,0],\"extensions\":{\"OMI_physics_body\":{\"collider\":{\"shape\":2}}}}]}";
        var m = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(doc));
        Assert.Equal(2, m.Scene.Bodies.Count);
        Assert.Equal("dynamic", m.Scene.Bodies[0].Type); Assert.Equal(3f, m.Scene.Bodies[0].MassOverride);
        Assert.Equal("static", m.Scene.Bodies[1].Type); Assert.Equal(0.5, m.Scene.Bodies[1].Shapes[0].Radius);
        var codes = m.Warnings.Select(x => x.Code).ToList();
        Assert.Contains("trigger", codes); Assert.Contains("unknown-shape", codes);
    }

    [Fact]
    public void NodeScaleAndRotationReachTheShape()
    {
        string doc = "{\"asset\":{\"version\":\"2.0\"},\"extensions\":{\"KHR_implicit_shapes\":{\"shapes\":[{\"type\":\"box\",\"box\":{\"size\":[1,1,1]}},{\"type\":\"capsule\",\"capsule\":{\"height\":1,\"radiusTop\":0.25,\"radiusBottom\":0.25}}]}},\"scene\":0,\"scenes\":[{\"nodes\":[0,1]}],\"nodes\":[" +
            "{\"scale\":[1,-2,3],\"translation\":[1,2,3],\"rotation\":[0,0,0.7071067811865476,0.7071067811865476],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"collider\":{\"geometry\":{\"shape\":0}}}}}," +
            "{\"translation\":[0,5,0],\"rotation\":[0,0,0.7071067811865476,0.7071067811865476],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"motion\":{\"mass\":1},\"collider\":{\"geometry\":{\"shape\":1}}}}}]}";
        var m = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(doc));
        var box = m.Scene.Bodies[0];
        Assert.Equal(0.5, box.Shapes[0].P[0], 12); Assert.Equal(1.0, box.Shapes[0].P[1], 12); Assert.Equal(1.5, box.Shapes[0].P[2], 12);   // absolute value of the scale applied to the size
        Assert.Equal(1.0, box.X); Assert.Equal(2.0, box.Y);
        Assert.Equal(0.7071067811865476, Math.Abs(box.Qz), 12);   // a mirrored node has two equivalent orientations; the box is symmetric
        // A capsule on its body's own frame is exact even when the body is rotated.
        Assert.Equal("capsule", m.Scene.Bodies[1].Shapes[0].Kind);
        Assert.DoesNotContain(m.Warnings, x => x.Code == "capsule-hull");
    }
}
