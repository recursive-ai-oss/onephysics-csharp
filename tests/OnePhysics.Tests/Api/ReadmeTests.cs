using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>
/// The C# code blocks of README.md and docs/guide/*.md are compiled (tests/ReadmeExamples, built without access to internals or the experimental raw tier) and run here.
/// Each block is preceded by <c>&lt;!-- example: File.cs --&gt;</c> naming the example source that contains its lines, verbatim and in order; this test fails when a page and its example drift apart.
/// </summary>
public class ReadmeTests
{
    // The pages and the example sources are copied next to the test assembly by the project file.
    private static string Root => Path.Combine(AppContext.BaseDirectory, "readme");

    private sealed record Block(string Page, string Example, List<string> Lines);

    private static IEnumerable<string> Pages()
    {
        yield return Path.Combine(Root, "README.md");
        foreach (string p in Directory.GetFiles(Path.Combine(Root, "guide"), "*.md").OrderBy(p => p, StringComparer.Ordinal)) { yield return p; }
    }

    private static List<Block> Blocks()
    {
        var blocks = new List<Block>();
        foreach (string page in Pages())
        {
            string example = "";
            List<string>? cur = null;
            foreach (string raw in File.ReadAllLines(page))
            {
                string line = raw.Trim();
                if (cur == null)
                {
                    if (line.StartsWith("<!-- example:", StringComparison.Ordinal)) { example = line["<!-- example:".Length..].Replace("-->", "").Trim(); }
                    else if (line.StartsWith("```csharp", StringComparison.Ordinal)) { cur = new List<string>(); }
                }
                else if (line.StartsWith("```", StringComparison.Ordinal))
                {
                    blocks.Add(new Block(Path.GetFileName(page), example, cur));
                    cur = null;
                    example = "";
                }
                else if (line.Length > 0 && !IsUsingDirective(line)) { cur.Add(line); }
            }
        }

        return blocks;
    }

    // Using directives sit at the top of the example file; a block may show them without repeating them where they cannot go.
    private static bool IsUsingDirective(string line) => line.StartsWith("using ", StringComparison.Ordinal) && line.EndsWith(';') && !line.StartsWith("using (", StringComparison.Ordinal) && !line.StartsWith("using var", StringComparison.Ordinal);

    [Fact]
    public void EveryCodeBlockIsAnExampleThatCompiles()
    {
        var blocks = Blocks();
        Assert.True(blocks.Count >= 15, "found only " + blocks.Count + " C# blocks");
        foreach (var b in blocks)
        {
            Assert.False(b.Example.Length == 0, b.Page + " has a C# block without an '<!-- example: File.cs -->' line before it");
            var lines = File.ReadAllLines(Path.Combine(Root, b.Example));
            int at = 0;
            foreach (string want in b.Lines)
            {
                while (at < lines.Length && lines[at].Trim() != want) { at++; }
                Assert.True(at < lines.Length, b.Example + " does not contain, in order, the line from " + b.Page + ": " + want);
                at++;
            }
        }
    }

    [Fact]
    public void ReadmeExamplesRunAndAreDeterministic()
    {
        var a2 = ReadmeExamples.Quickstart2D.Run();
        Assert.Equal(a2, ReadmeExamples.Quickstart2D.Run());
        Assert.True(a2.Bytes > 0);
        var a3 = ReadmeExamples.Quickstart3D.Run();
        Assert.Equal(a3, ReadmeExamples.Quickstart3D.Run());
        Assert.True(a3.Bytes > 0);
    }

    [Fact]
    public void GuideExamplesRunAndMeanWhatTheyShow()
    {
        var q = GuideExamples.QuickStart.Run();
        Assert.Equal(q, GuideExamples.QuickStart.Run());
        Assert.True(q.Grounded);
        Assert.True(q.Crate.Y > 0 && q.Crate.Y < 1.5, "the crate rests on the ground: " + q.Crate);

        var q3 = GuideExamples.QuickStart3D.Run();
        Assert.Equal(q3, GuideExamples.QuickStart3D.Run());

        var t2 = GuideExamples.Terrain2D.Run(); Assert.Equal(t2, GuideExamples.Terrain2D.Run());
        var tv = GuideExamples.Terrain3D.Voxels(); Assert.Equal(tv, GuideExamples.Terrain3D.Voxels());
        var ts = GuideExamples.Terrain3D.Sdf(); Assert.Equal(ts, GuideExamples.Terrain3D.Sdf()); Assert.InRange(ts.BallY, 0.9, 3.5);
        var soft = GuideExamples.SoftBodies.Run();
        Assert.Equal(soft, GuideExamples.SoftBodies.Run());
        var fluids = GuideExamples.Fluids.Run();
        Assert.Equal(fluids, GuideExamples.Fluids.Run());
        Assert.InRange(soft.ClothY, 0.0, 1.8);
        Assert.InRange(soft.CubeVolumeRatio, 0.9, 1.1);

        var wt = GuideExamples.Water.Run();
        Assert.Equal(wt, GuideExamples.Water.Run());
        var sw = GuideExamples.Swimming.Run();
        Assert.Equal(sw, GuideExamples.Swimming.Run());
        Assert.True(sw.Swimming && sw.Events > 0);
        var dq = GuideExamples.DeformableQueries.Run();
        Assert.Equal(dq, GuideExamples.DeformableQueries.Run());
        Assert.Equal(DeformableKind.Cloth, dq.Kind);
        Assert.True(dq.Contacts > 0);
        Assert.True(wt.Events >= 1);
        Assert.InRange(wt.CrateY, 1.8, 2.3);
        Assert.True(wt.Depth > 1.9f);

        var art = GuideExamples.Articulations.Run();
        Assert.Equal(art, GuideExamples.Articulations.Run());
        Assert.True(art.TorsoY > 0.4 && art.TorsoY < 0.7, "the quadruped stands: " + art.TorsoY);
        var dyn = GuideExamples.Articulations.Dynamics();
        Assert.True(double.IsFinite(dyn.Torque0) && double.IsFinite(dyn.Torque1));
        var ws = GuideExamples.WorldScale.Run();
        Assert.Equal(ws, GuideExamples.WorldScale.Run());
        Assert.Equal(1, ws.Tiles);
        Assert.Equal(0.25f, ws.Scale);

        var t = GuideExamples.Tiers.Run();
        Assert.Equal(t, GuideExamples.Tiers.Run());
        Assert.True(t.Contacts > 0);
        Assert.True(t.Hits > 0);

        var e = GuideExamples.Events.Run();
        Assert.Equal(e, GuideExamples.Events.Run());
        Assert.True(e.Enters > 0 && e.Polled > 0 && e.Triggers > 0);

        var r = GuideExamples.Rollback.Run();
        Assert.True(r.Equal, "resimulating from a restored snapshot reproduces the original hashes");
        Assert.Equal(100, r.Frames);

        var d = GuideExamples.Determinism.Run();
        Assert.Equal(2_000_000.0, d.Origin);
        Assert.Equal(2_000_000.0, d.HostX, 3);

        var qu = GuideExamples.Queries.Run();
        Assert.True(qu.Collide);
        Assert.True(qu.Overlaps >= 1);
        Assert.True(qu.Cast);
    }
}
