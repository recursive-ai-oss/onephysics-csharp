using System.IO;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Physics3D;

public sealed partial class PhysicsWorld3D
{
    /// <summary>Saves all world state as canonical version 2 scene text. Includes pending live simulation options separately; delegates and host code are not serialized.</summary>
    public void SaveScene(TextWriter output) { var document = Scene.Scene.FromSnapshot(3, Id, Snapshot()); byte[] options = new byte[Replay.ReplayOptions3D.Size]; ((Replay.IRecordedWorld)this).WriteReplayOptions(options); document.SetLiveOptions(options); document.Write(output); }

    /// <summary>Returns canonical full-state scene text, including every shape and service and all solver caches. Allocates the resulting string and snapshot.</summary>
    public string SaveScene() { using var output = new StringWriter(CultureInfo.InvariantCulture); SaveScene(output); return output.ToString(); }

    /// <summary>Creates a world from authoring or full-state scene text. Thread count defaults to 1. Invalid input throws Scene.SceneFormatException naming its line.</summary>
    public static PhysicsWorld3D LoadScene(TextReader input, string inputName = "scene", int threadCount = 1) => Scene.Scene.Read(input, inputName).CreateWorld3D(threadCount);
}
