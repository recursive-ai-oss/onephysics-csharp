using System;
using System.IO;
using System.Globalization;

namespace RecursiveAI.OnePhysics.Physics2D;

public sealed partial class PhysicsWorld2D
{
    /// <summary>Saves all world state as canonical version 2 scene text. Includes pending live simulation options separately; delegates and host code are not serialized.</summary>
    public void SaveScene(TextWriter output) { var document = Scene.Scene.FromSnapshot(2, Id, Snapshot()); byte[] options = new byte[Replay.ReplayOptions2D.SizeWithSoft]; int optionLength = ((Replay.IRecordedWorld)this).WriteReplayOptions(options); options = options.AsSpan(0, optionLength).ToArray(); document.SetLiveOptions(options); document.Write(output); }

    /// <summary>Returns canonical full-state scene text, including every shape and service and all solver caches. Allocates the resulting string and snapshot.</summary>
    public string SaveScene() { using var output = new StringWriter(CultureInfo.InvariantCulture); SaveScene(output); return output.ToString(); }

    /// <summary>Creates a world from authoring or full-state scene text. Thread count defaults to 1. Invalid input throws Scene.SceneFormatException naming its line.</summary>
    public static PhysicsWorld2D LoadScene(TextReader input, string inputName = "scene", int threadCount = 1) => Scene.Scene.Read(input, inputName).CreateWorld2D(threadCount);
}
