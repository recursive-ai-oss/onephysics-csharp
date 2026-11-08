using System;
using System.IO;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;

internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length != 1) { Console.Error.WriteLine("Usage: TrimCheck <trimmed physics assembly>"); return 2; }
        using var input = File.OpenRead(args[0]);
        using var pe = new PEReader(input);
        var reader = pe.GetMetadataReader();
        bool planar = false;
        foreach (var handle in reader.TypeDefinitions)
        {
            var type = reader.GetTypeDefinition(handle);
            string ns = reader.GetString(type.Namespace);
            if (ns.StartsWith("RecursiveAI.OnePhysics.Physics3D", StringComparison.Ordinal) || ns.StartsWith("RecursiveAI.OnePhysics.Collision3D", StringComparison.Ordinal))
            { Console.Error.WriteLine("Unexpected spatial type: " + ns + "." + reader.GetString(type.Name)); return 1; }
            planar |= ns == "RecursiveAI.OnePhysics.Physics2D" && reader.GetString(type.Name) == "PhysicsWorld2D";
        }
        Console.WriteLine(planar ? "OK: 2D retained, 3D engine and collision types absent" : "FAIL: 2D world missing");
        return planar ? 0 : 1;
    }
}
