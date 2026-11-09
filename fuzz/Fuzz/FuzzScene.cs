using System.IO;
using System.Text;
using RecursiveAI.OnePhysics.Scene;
using RecursiveAI.OnePhysics.Physics2D;
using Document = RecursiveAI.OnePhysics.Scene.Scene;

internal static class FuzzScene
{
    public static void Run(byte[] bytes)
    {
        try
        {
            var scene = Document.Read(new StringReader(Encoding.UTF8.GetString(bytes)), "fuzz");
            if (scene.Dim == 2) scene.CreateWorld2D(); else scene.CreateWorld3D();
        }
        catch (SceneFormatException) { }
    }
    public static byte[] Authoring() => Encoding.UTF8.GetBytes("scene 1\ndim 2\nmaterial 0 0.6 0\nbody dynamic 0 1 0 0 0 0 -\ncontrols 32 4 3 0.2\nsleep 1 0.1 0.2 0.3\nmass 2\ncenter 0.1 0\ninertia 3\nshape box 0 1 0 0.5 0.5 0 0 0\n");
    public static byte[] State()
    {
        var world = new PhysicsWorld2D(new WorldOptions2D { WorldId = 24 });
        world.AddBox(new(1,1), at: new(0,2));
        return Encoding.UTF8.GetBytes(world.SaveScene());
    }
}
