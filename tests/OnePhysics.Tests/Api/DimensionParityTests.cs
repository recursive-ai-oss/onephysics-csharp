using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>
/// The two worlds expose the same members with 2D or 3D math types. Types are paired by name once the dimension suffix is dropped,
/// members are compared by kind and name (a short vocabulary table folds the dimension's own words, such as circle and sphere, together), and every difference must be listed in
/// <c>testdata/api/parity-allow.txt</c> with the reason physics does not apply in that dimension.
/// </summary>
public class DimensionParityTests
{
    private static readonly Regex Suffix = new("(2D|3D)$", RegexOptions.Compiled);

    /// <summary>The dimension's own vocabulary for the same thing, folded to one word on both sides: a circle is a sphere, a revolute joint a hinge, a weld a fixed joint, an angle a rotation, a prismatic joint a slider.</summary>
    private static readonly (string From, string To)[] Vocabulary =
    [
        ("Sphere", "Circle"), ("Hinge", "Revolute"), ("Fixed", "Weld"), ("Slider", "Prismatic"),
        ("Rotation", "Angle"), ("ConvexHull", "Polygon"), ("SoftBall", "SoftCircle"), ("LocalFrame", "LocalAnchor"),
    ];

    private static string Canon(string name)
    {
        name = Suffix.Replace(name, "");
        foreach (var (from, to) in Vocabulary) { name = name.Replace(from, to, StringComparison.Ordinal); }
        return name;
    }

    private static string Stem(Type t) => Canon(t.Name);

    private static Dictionary<string, Type> Collect(string ns)
    {
        var d = new Dictionary<string, Type>(StringComparer.Ordinal);
        foreach (var t in typeof(P2.PhysicsWorld2D).Assembly.GetExportedTypes())
        {
            if (t.Namespace != ns || t.IsNested) { continue; }
            d[Stem(t)] = t;
        }
        return d;
    }

    private static IEnumerable<string> Members(Type t)
    {
        const BindingFlags F = BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        foreach (var c in t.GetConstructors(F)) { yield return "constructor"; }
        foreach (var m in t.GetMethods(F))
        {
            if (m.IsSpecialName) { continue; }
            yield return "method " + Canon(m.Name);
        }
        foreach (var p in t.GetProperties(F)) { yield return "property " + Canon(p.Name); }
        foreach (var f in t.GetFields(F)) { yield return "field " + Canon(f.Name); }
        foreach (var e in t.GetEvents(F)) { yield return "event " + e.Name; }
    }

    private static string AllowPath() => Path.Combine(AppContext.BaseDirectory, "api", "parity-allow.txt");

    /// <summary>Every difference between the two engines' public surfaces, one line each, in a stable order.</summary>
    public static List<string> Differences()
    {
        var two = Collect("RecursiveAI.OnePhysics.Physics2D");
        var three = Collect("RecursiveAI.OnePhysics.Physics3D");
        var lines = new List<string>();
        foreach (string stem in two.Keys.Union(three.Keys).OrderBy(x => x, StringComparer.Ordinal))
        {
            bool in2 = two.TryGetValue(stem, out var a);
            bool in3 = three.TryGetValue(stem, out var b);
            if (!in2) { lines.Add($"{stem}: type only in 3D"); continue; }
            if (!in3) { lines.Add($"{stem}: type only in 2D"); continue; }
            var ma = Members(a!).Distinct().ToDictionary(x => x, _ => 1, StringComparer.Ordinal);
            var mb = Members(b!).Distinct().ToDictionary(x => x, _ => 1, StringComparer.Ordinal);
            foreach (string k in ma.Keys.Union(mb.Keys).OrderBy(x => x, StringComparer.Ordinal))
            {
                ma.TryGetValue(k, out int ca);
                mb.TryGetValue(k, out int cb);
                if (ca == cb) { continue; }
                if (cb == 0) { lines.Add($"{stem}: {k} only in 2D"); }
                else if (ca == 0) { lines.Add($"{stem}: {k} only in 3D"); }
                else { lines.Add($"{stem}: {k} overload count differs"); }
            }
        }
        return lines;
    }

    [Fact]
    public void PublicSurfacesMatchExceptForListedDifferences()
    {
        var patterns = File.ReadAllLines(AllowPath())
            .Select(l => l.Trim())
            .Where(l => l.Length > 0 && !l.StartsWith('#'))
            .Select(l => l.Split(" -- ", 2)[0].Trim())
            .Select(p => (Text: p, Match: new Regex("^" + Regex.Escape(p).Replace("\\*", ".*", StringComparison.Ordinal) + "( only in [23]D)?$", RegexOptions.CultureInvariant)))
            .ToList();
        var actual = Differences();
        var unlisted = actual.Where(a => !patterns.Any(p => p.Match.IsMatch(a))).ToList();
        var stale = patterns.Where(p => !actual.Any(a => p.Match.IsMatch(a))).Select(p => p.Text).ToList();
        Assert.False(unlisted.Count > 0, "unlisted differences:\n" + string.Join("\n", unlisted));
        Assert.False(stale.Count > 0, "allow-list entries that match nothing:\n" + string.Join("\n", stale));
    }

    [Fact]
    public void EveryAllowListEntryGivesAReason()
    {
        foreach (string l in File.ReadAllLines(AllowPath()).Select(x => x.Trim()).Where(x => x.Length > 0 && !x.StartsWith('#')))
        {
            Assert.Contains(" -- ", l, StringComparison.Ordinal);
        }
    }

    /// <summary>Writes the current differences next to the build output, for editing the allow-list.</summary>
    [Fact]
    public void DumpDifferences()
    {
        string? path = Environment.GetEnvironmentVariable("ONEPHYSICS_PARITY_DUMP");
        if (path is null) { return; }
        var sb = new StringBuilder();
        foreach (string l in Differences()) { sb.Append(l).Append('\n'); }
        File.WriteAllText(path, sb.ToString());
    }
}
