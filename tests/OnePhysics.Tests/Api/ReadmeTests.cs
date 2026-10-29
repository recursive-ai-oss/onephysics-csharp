using System;
using System.Collections.Generic;
using System.IO;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>The code blocks of README.md are compiled (tests/ReadmeExamples, built without access to internals or the experimental raw tier) and run here; this test fails when the README and that code drift apart.</summary>
public class ReadmeTests
{
    // The README and the example sources are copied next to the test assembly by the project file.
    private static string RepoFile(params string[] parts) => Path.Combine(AppContext.BaseDirectory, "readme", parts[^1]);

    private static List<List<string>> ReadmeBlocks()
    {
        var blocks = new List<List<string>>();
        List<string>? cur = null;
        foreach (string line in File.ReadAllLines(RepoFile("README.md")))
        {
            if (cur == null) { if (line.StartsWith("```csharp", StringComparison.Ordinal)) { cur = new List<string>(); } }
            else if (line.StartsWith("```", StringComparison.Ordinal)) { blocks.Add(cur); cur = null; }
            else if (line.Trim().Length > 0) { cur.Add(line.Trim()); }
        }

        return blocks;
    }

    private static void AssertContainsInOrder(string file, List<string> block)
    {
        var lines = File.ReadAllLines(RepoFile("tests", "ReadmeExamples", file));
        int at = 0;
        foreach (string want in block)
        {
            while (at < lines.Length && lines[at].Trim() != want) { at++; }
            Assert.True(at < lines.Length, file + " does not contain, in order, the README line: " + want);
            at++;
        }
    }

    [Fact]
    public void ReadmeBlocksAreTheCompiledExamples()
    {
        var blocks = ReadmeBlocks();
        Assert.Equal(3, blocks.Count);
        AssertContainsInOrder("Quickstart2D.cs", blocks[0]);
        AssertContainsInOrder("Quickstart3D.cs", blocks[1]);
        AssertContainsInOrder("QuickstartRollback2D.cs", blocks[2]);
    }

    [Fact]
    public void ReadmeExamplesRunAndAreDeterministic()
    {
        var a2 = ReadmeExamples.Quickstart2D.Run();
        var b2 = ReadmeExamples.Quickstart2D.Run();
        Assert.Equal(a2, b2);
        Assert.True(a2.Bytes > 0);
        var a3 = ReadmeExamples.Quickstart3D.Run();
        Assert.Equal(a3, ReadmeExamples.Quickstart3D.Run());
        Assert.True(a3.Bytes > 0);
        var r = ReadmeExamples.QuickstartRollback2D.Run();
        Assert.Equal(r, ReadmeExamples.QuickstartRollback2D.Run());
    }
}
