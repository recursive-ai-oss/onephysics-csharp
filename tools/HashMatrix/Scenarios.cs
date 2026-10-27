using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Registry of named scenarios. A scenario is a deterministic function that writes its per-step output bytes to a
/// <see cref="ScenarioContext"/>. To add one, create a file under <c>Scenarios/&lt;area&gt;/</c> with a module
/// initializer that calls <see cref="Register"/>; no shared file changes:
/// <code>
/// internal static class MyScenarios
/// {
///     [ModuleInitializer]
///     internal static void Init() =&gt; Scenarios.Register("area/thing", ctx =&gt;
///     {
///         for (int step = 0; step &lt; 100; step++) { /* advance */ ctx.Write(state); ctx.EndStep(); }
///     });
/// }
/// </code>
/// Names are lower-case, slash-separated, unique; the manifest orders them by ordinal comparison.
/// Module initializers need no reflection, so this works unchanged under NativeAOT, trimming and WebAssembly.
/// </summary>
public static class Scenarios
{
    private static readonly List<(string Name, Action<ScenarioContext> Run)> s_all = new();

    /// <summary>Registers a scenario. Throws if the name is already taken.</summary>
    public static void Register(string name, Action<ScenarioContext> run)
    {
        ArgumentException.ThrowIfNullOrEmpty(name);
        ArgumentNullException.ThrowIfNull(run);
        lock (s_all)
        {
            foreach (var (n, _) in s_all)
            {
                if (string.Equals(n, name, StringComparison.Ordinal))
                {
                    throw new InvalidOperationException($"Hash-matrix scenario '{name}' is registered twice.");
                }
            }
            s_all.Add((name, run));
        }
    }

    /// <summary>All scenarios sorted by ordinal name.</summary>
    public static (string Name, Action<ScenarioContext> Run)[] All()
    {
        lock (s_all)
        {
            var a = s_all.ToArray();
            Array.Sort(a, static (x, y) => string.CompareOrdinal(x.Name, y.Name));
            return a;
        }
    }
}
