using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Fuzz;

// Every file under testdata/fuzz/<target>/ is a minimised input that once made that fuzz target fail (a crash, a hang or a world mismatch).
// Replaying them keeps each fixed finding fixed; a target must accept the input or raise only the typed error it documents.
public class FuzzRegressionTests
{
    private static readonly string Root = Path.Combine(AppContext.BaseDirectory, "fuzz-regress");

    public static IEnumerable<object[]> Inputs() =>
        Directory.Exists(Root)
            ? Directory.GetFiles(Root, "*", SearchOption.AllDirectories).OrderBy(f => f, StringComparer.Ordinal)
                .Select(f => new object[] { Path.GetRelativePath(Root, f).Replace('\\', '/') })
            : new[] { new object[] { "none" } };

    [Theory]
    [MemberData(nameof(Inputs))]
    public void ReplaysWithoutFailure(string relative)
    {
        if (relative == "none") { return; }
        string target = relative.Split('/')[0];
        Assert.True(Program.Targets.TryGetValue(target, out var run), "no fuzz target named " + target);
        run!(File.ReadAllBytes(Path.Combine(Root, relative)));
    }

    [Fact]
    public void EveryTargetIsListed() => Assert.True(Program.Targets.Count >= 30);
}
