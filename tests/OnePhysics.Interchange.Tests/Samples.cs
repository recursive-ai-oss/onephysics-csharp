using System;
using System.IO;

namespace RecursiveAI.OnePhysics.Interchange.Tests;

internal static class Samples
{
    public static string Read(string name) => File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "interchange", name));
}
