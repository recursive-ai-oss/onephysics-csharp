using System;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests;

/// <summary>The shared ability scenes run their full length and keep their promises; the hash-matrix and the stability runner use the same definitions.</summary>
public sealed class CharacterAbilitySceneTests(ITestOutputHelper output)
{
    private void Note(string line)
    {
        output.WriteLine(line);
        string? path = Environment.GetEnvironmentVariable("ONEPHYSICS_TEST_NOTES");
        if (path != null) { System.IO.File.AppendAllText(path, line + "\n"); }
    }

    [Theory]
    [InlineData("characters-soft-3d")]
    [InlineData("characters-ropes-3d")]
    [InlineData("climbing-3d")]
    [InlineData("riders-3d")]
    public void Scenes3DRunAndReportTheirMetrics(string name)
    {
        var s = CharacterAbilityScenes3D.Build(name, 1);
        for (int i = 0; i < s.Steps; i++) { s.Script(i); s.World.Step(CharacterAbilityScenes3D.Dt); s.Observe(i); }
        Note(name + " " + string.Join(' ', System.Linq.Enumerable.Select(s.Metrics, kv => kv.Key + "=" + kv.Value.ToString("R", System.Globalization.CultureInfo.InvariantCulture))));
    }

    [Theory]
    [InlineData("characters-soft-2d")]
    [InlineData("climbing-2d")]
    public void Scenes2DRunAndReportTheirMetrics(string name)
    {
        var s = CharacterAbilityScenes2D.Build(name, 1);
        for (int i = 0; i < s.Steps; i++) { s.Script(i); s.World.Step(CharacterAbilityScenes2D.Dt); s.Observe(i); }
        Note(name + " " + string.Join(' ', System.Linq.Enumerable.Select(s.Metrics, kv => kv.Key + "=" + kv.Value.ToString("R", System.Globalization.CultureInfo.InvariantCulture))));
    }
}
