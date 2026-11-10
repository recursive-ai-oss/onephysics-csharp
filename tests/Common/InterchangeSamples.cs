using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Interchange;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Interchange.Tests;

/// <summary>The sample files of testdata/interchange, embedded in every project that uses them (so the NativeAOT and WebAssembly apps need no file system), and the runs that hash what they import.</summary>
internal static class InterchangeSamples
{
    private static string D(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    public static string Read(string name)
    {
        using var s = typeof(InterchangeSamples).Assembly.GetManifestResourceStream("interchange." + name) ?? throw new FileNotFoundException("embedded sample " + name);
        using var r = new StreamReader(s, Encoding.UTF8);
        return r.ReadToEnd();
    }

    /// <summary>A floor, nine boxes, five spheres, four capsules (every one a body with its collider on the same node) and a point joint between the last two boxes: 20 bodies in all.</summary>
    public static string Twenty()
    {
        var sb = new StringBuilder();
        sb.Append("{\"asset\":{\"version\":\"2.0\"},\"extensionsUsed\":[\"KHR_physics_rigid_bodies\",\"KHR_implicit_shapes\"],\"scene\":0,\"scenes\":[{\"nodes\":[");
        sb.Append(string.Join(",", Enumerable.Range(0, 20)));
        sb.Append("]}],\"extensions\":{\"KHR_implicit_shapes\":{\"shapes\":[{\"type\":\"box\",\"box\":{\"size\":[40,1,40]}},{\"type\":\"box\",\"box\":{\"size\":[0.5,0.5,0.5]}},{\"type\":\"sphere\",\"sphere\":{\"radius\":0.3}},{\"type\":\"capsule\",\"capsule\":{\"height\":0.6,\"radiusTop\":0.2,\"radiusBottom\":0.2}}]},");
        sb.Append("\"KHR_physics_rigid_bodies\":{\"physicsMaterials\":[{\"dynamicFriction\":0.5,\"staticFriction\":0.5,\"restitution\":0.1}],\"physicsJoints\":[{\"limits\":[{\"linearAxes\":[0,1,2],\"min\":0,\"max\":0}]}]}},\"nodes\":[");
        sb.Append("{\"name\":\"floor\",\"translation\":[0,-0.5,0],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"collider\":{\"geometry\":{\"shape\":0},\"physicsMaterial\":0}}}}");
        for (int i = 1; i < 20; i++)
        {
            int shape = i <= 9 ? 1 : i <= 14 ? 2 : 3;
            double x = (i % 5) * 0.9 - 1.8, y = 0.4 + (i / 5) * 0.8, z = (i % 3) * 0.7 - 0.7;
            sb.Append(",{\"name\":\"b" + i + "\",\"translation\":[" + D(x) + "," + D(y) + "," + D(z) + "],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"motion\":{\"mass\":" + D(1 + 0.25 * i) + "},\"collider\":{\"geometry\":{\"shape\":" + shape + "},\"physicsMaterial\":0}");
            if (i == 8) { sb.Append(",\"joint\":{\"connectedNode\":9,\"joint\":0}"); }
            sb.Append("}}}");
        }

        sb.Append("]}");
        return sb.ToString();
    }

    /// <summary>The arm drives used by every robot run: each joint to a small target, with a position drive.</summary>
    public static ulong RunArm(P3.ArticulationDef def, int steps, Action<int, ulong>? perStep = null)
    {
        // Explicit world ids: the automatic ones come from a process-wide counter, and a scenario that took some would change the hash of every id-dependent scenario that runs after it.
        var w = new P3.PhysicsWorld3D(new P3.WorldOptions3D { Gravity = new Vector3F(0, 0, -9.81f), WorldId = 90101 });
        var art = w.AddArticulation(def);
        for (int i = 1; i < def.Links.Count; i++) { art.SetDrive(i, new ArticulationDrive { Stiffness = 60f, Damping = 6f, MaxForce = 50f, TargetPosition = 0.25f * i / 6f }); }
        for (int i = 0; i < steps; i++) { w.Step(1f / 60f); perStep?.Invoke(i, w.ComputeHash()); }
        return w.ComputeHash();
    }

    public static ulong RunScene3D(RecursiveAI.OnePhysics.Scene.Scene scene, int steps, Action<int, ulong>? perStep = null)
    {
        scene.Params["world-id"] = new[] { "90102" };
        var w = scene.CreateWorld3D(1);
        for (int i = 0; i < steps; i++) { w.Step(1f / 60f); perStep?.Invoke(i, w.ComputeHash()); }
        return w.ComputeHash();
    }

    /// <summary>A map applied to a world with a ball and a box thrown over its slopes and one-way platform.</summary>
    public static ulong RunMap(Map2D map, int steps, Action<int, ulong>? perStep = null)
    {
        var w = new P2.PhysicsWorld2D(new P2.WorldOptions2D { WorldId = 90103 });
        map.Apply(w);
        var ball = w.AddCircle(0.3f, new Vector2D(6.0, 1.3), material: new P2.Material2D(0.2f));
        ball.LinearVelocity = new Vector2F(8f, 0f);
        var box = w.AddBox(new Vector2F(0.4f, 0.4f), new Vector2D(15.5, 1.5));
        box.LinearVelocity = new Vector2F(0f, 9f);
        for (int i = 0; i < steps; i++) { w.Step(1f / 60f); perStep?.Invoke(i, w.ComputeHash()); }
        return w.ComputeHash();
    }
}

/// <summary>The importers under NativeAOT and WebAssembly: each sample is imported and run, and the digest of the world hashes is printed so the apps' output can be compared with the JIT's.</summary>
internal static class InterchangeSmoke
{
    internal static bool Run(out ulong digest)
    {
        ulong h = 14695981039346656037UL;
        void Mix(ulong v) { h = (h ^ v) * 1099511628211UL; }
        var urdf = UrdfImporter.Import(InterchangeSamples.Read("arm6.urdf"));
        var mjcf = MjcfImporter.Import(InterchangeSamples.Read("arm6.xml"));
        ulong a = InterchangeSamples.RunArm(urdf.Definition, 120), b = InterchangeSamples.RunArm(mjcf.Definition, 120);
        Mix(a); Mix(b);
        var gltf = GltfPhysicsImporter.Import(Encoding.UTF8.GetBytes(InterchangeSamples.Twenty()));
        Mix(InterchangeSamples.RunScene3D(gltf.Scene, 120));
        var tmj = TiledImporter.Import(InterchangeSamples.Read("level.tmj")); var tmx = TiledImporter.Import(InterchangeSamples.Read("level.tmx")); var ldtk = LdtkImporter.Import(InterchangeSamples.Read("level.ldtk"));
        ulong t1 = InterchangeSamples.RunMap(tmj, 120), t2 = InterchangeSamples.RunMap(tmx, 120), t3 = InterchangeSamples.RunMap(ldtk, 120);
        Mix(t1); Mix(t2); Mix(t3);
        digest = h;
        return a == b && gltf.Scene.Bodies.Count == 20 && tmj.TileLayers.Count == 2 && ldtk.TileLayers.Count == 2 && t1 == t2 && urdf.Warnings.Count == 0;
    }
}
