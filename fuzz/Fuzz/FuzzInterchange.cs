using System;
using System.Text;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.Interchange;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

// Targets for the importers. The input bytes are the file itself (a mutated sample is the best way into a text format). Every input either imports or fails
// with an InterchangeException; what an importer accepts must then be accepted by the engine's own validation (a typed ArgumentException or SceneFormatException at worst).
internal static class FuzzInterchange
{
    internal const string Urdf = "<robot name=\"r\"><link name=\"a\"><inertial><origin xyz=\"0 0 0.1\" rpy=\"0 0 0\"/><mass value=\"2\"/><inertia ixx=\"0.1\" ixy=\"0\" ixz=\"0\" iyy=\"0.1\" iyz=\"0\" izz=\"0.1\"/></inertial>" +
        "<collision><origin xyz=\"0 0 0.1\"/><geometry><box size=\"0.1 0.1 0.2\"/></geometry></collision></link><link name=\"b\"><inertial><mass value=\"1\"/><inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/></inertial>" +
        "<collision><geometry><cylinder radius=\"0.05\" length=\"0.3\"/></geometry></collision></link><joint name=\"j\" type=\"revolute\"><parent link=\"a\"/><child link=\"b\"/><origin xyz=\"0 0 0.2\" rpy=\"1.57 0 0\"/><axis xyz=\"0 1 0\"/><limit lower=\"-1\" upper=\"1\" effort=\"5\" velocity=\"2\"/></joint></robot>";

    internal const string Mjcf = "<mujoco model=\"m\"><compiler angle=\"degree\"/><default><joint damping=\"0.1\"/><default class=\"big\"><geom size=\"0.2\"/></default></default><worldbody><geom type=\"plane\" size=\"5 5 0.1\"/>" +
        "<body name=\"base\" pos=\"0 0 1\"><freejoint/><geom type=\"box\" size=\"0.1 0.1 0.1\"/><body name=\"arm\" pos=\"0 0 0.2\" euler=\"0 90 0\"><joint type=\"hinge\" axis=\"0 1 0\" range=\"-30 30\"/><joint name=\"s\" type=\"slide\"/>" +
        "<geom type=\"capsule\" size=\"0.05 0.2\" mass=\"1\"/></body></body></worldbody><actuator><position joint=\"s\" kp=\"10\"/></actuator></mujoco>";

    internal const string Gltf = "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0,1,2]}],\"extensions\":{\"KHR_implicit_shapes\":{\"shapes\":[{\"type\":\"box\",\"box\":{\"size\":[1,1,1]}},{\"type\":\"sphere\",\"sphere\":{\"radius\":0.5}}]}," +
        "\"KHR_physics_rigid_bodies\":{\"physicsMaterials\":[{\"dynamicFriction\":0.5,\"staticFriction\":0.5,\"restitution\":0}],\"physicsJoints\":[{\"limits\":[{\"linearAxes\":[0,1,2],\"min\":0,\"max\":0}]}]}}," +
        "\"nodes\":[{\"translation\":[0,-1,0],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"collider\":{\"geometry\":{\"shape\":0},\"physicsMaterial\":0}}}}," +
        "{\"translation\":[0,1,0],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"motion\":{\"mass\":1},\"collider\":{\"geometry\":{\"shape\":1}},\"joint\":{\"connectedNode\":2,\"joint\":0}}}}," +
        "{\"translation\":[1,1,0],\"extensions\":{\"KHR_physics_rigid_bodies\":{\"motion\":{\"mass\":2},\"collider\":{\"geometry\":{\"shape\":0}}}}}]}";

    internal const string Tmj = "{\"orientation\":\"orthogonal\",\"width\":3,\"height\":2,\"tilewidth\":16,\"tileheight\":16,\"tilesets\":[{\"firstgid\":1,\"tilecount\":2,\"tiles\":[{\"id\":1,\"properties\":[{\"name\":\"oneway\",\"type\":\"bool\",\"value\":true}]}]}]," +
        "\"layers\":[{\"type\":\"tilelayer\",\"name\":\"Collision\",\"width\":3,\"height\":2,\"data\":[0,1,0,1,1,2]},{\"type\":\"objectgroup\",\"name\":\"o\",\"objects\":[{\"x\":4,\"y\":4,\"width\":8,\"height\":8,\"properties\":[{\"name\":\"body\",\"value\":\"dynamic\"}]},{\"x\":0,\"y\":0,\"polygon\":[{\"x\":0,\"y\":0},{\"x\":8,\"y\":0},{\"x\":0,\"y\":8}]}]}]}";

    internal const string Tmx = "<map orientation=\"orthogonal\" width=\"2\" height=\"2\" tilewidth=\"8\" tileheight=\"8\"><tileset firstgid=\"1\" tilecount=\"1\"><tile id=\"0\"><objectgroup><object x=\"0\" y=\"0\" width=\"8\" height=\"8\"/></objectgroup></tile></tileset>" +
        "<layer name=\"collision\" width=\"2\" height=\"2\"><data encoding=\"csv\">1,0,1,1</data></layer><objectgroup name=\"p\"><object x=\"1\" y=\"1\"><point/></object><object x=\"2\" y=\"2\" width=\"4\" height=\"4\"><ellipse/><properties><property name=\"body\" value=\"kinematic\"/></properties></object></objectgroup></map>";

    internal const string Ldtk = "{\"defs\":{\"layers\":[{\"uid\":1,\"intGridValues\":[{\"value\":1,\"identifier\":\"solid\"},{\"value\":2,\"identifier\":\"slope-right\"}]}]},\"levels\":[{\"identifier\":\"L\",\"pxWid\":48,\"pxHei\":32,\"layerInstances\":[" +
        "{\"__identifier\":\"Collision\",\"__type\":\"IntGrid\",\"__cWid\":3,\"__cHei\":2,\"__gridSize\":16,\"layerDefUid\":1,\"intGridCsv\":[0,2,0,1,1,1]},{\"__identifier\":\"E\",\"__type\":\"Entities\",\"entityInstances\":[{\"__identifier\":\"Crate\",\"px\":[4,4],\"width\":8,\"height\":8,\"__pivot\":[0,0],\"fieldInstances\":[{\"__identifier\":\"body\",\"__value\":\"dynamic\"}]}]}]}]}";

    private static string Text(byte[] b) => Encoding.UTF8.GetString(b);

    internal static void UrdfTarget(byte[] input)
    {
        try
        {
            var m = UrdfImporter.Import(Text(input), new UrdfOptions { Meshes = Mesh });
            Create(m.Definition);
        }
        catch (InterchangeException) { }
    }

    internal static void MjcfTarget(byte[] input)
    {
        try
        {
            var m = MjcfImporter.Import(Text(input), new MjcfOptions { Meshes = Mesh });
            Create(m.Definition);
        }
        catch (InterchangeException) { }
    }

    internal static void GltfTarget(byte[] input)
    {
        try
        {
            var m = GltfPhysicsImporter.Import(input);
            try { m.Scene.CreateWorld3D(1); }
            catch (RecursiveAI.OnePhysics.Scene.SceneFormatException) { }
        }
        catch (InterchangeException) { }
    }

    internal static void TiledTarget(byte[] input)
    {
        try { Apply(TiledImporter.Import(Text(input))); }
        catch (InterchangeException) { }
    }

    internal static void LdtkTarget(byte[] input)
    {
        try { Apply(LdtkImporter.Import(Text(input))); }
        catch (InterchangeException) { }
    }

    private static readonly Vector3F[] TetraVertices = { new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(0, 0, 1) };
    private static readonly int[] TetraIndices = { 0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3 };

    private static InterchangeMesh? Mesh(string name) => name.Length % 2 == 0 ? null : new InterchangeMesh(TetraVertices, TetraIndices);

    private static void Create(P3.ArticulationDef def)
    {
        var w = new P3.PhysicsWorld3D();
        try { w.AddArticulation(def); }
        catch (ArgumentException) { }
    }

    private static void Apply(Map2D map)
    {
        var w = new P2.PhysicsWorld2D();
        try { map.Apply(w); }
        catch (ArgumentException) { }
    }
}
