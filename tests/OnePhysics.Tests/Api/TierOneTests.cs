using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

/// <summary>The happy path never asks for a struct callback: every public member that takes a struct-constrained type parameter has a plain overload (delegate or none) beside it, or is a middle-tier member listed here.</summary>
public class TierOneTests
{
    private static IEnumerable<MethodInfo> StructGenericMethods(Assembly asm)
    {
        const BindingFlags F = BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        foreach (var t in asm.GetExportedTypes())
        {
            foreach (var m in t.GetMethods(F))
            {
                if (!m.IsGenericMethodDefinition) { continue; }
                if (m.GetGenericArguments().Any(g => (g.GenericParameterAttributes & GenericParameterAttributes.NotNullableValueTypeConstraint) != 0)) { yield return m; }
            }
        }
    }

    private static string Key(MethodInfo m) => m.DeclaringType!.FullName!.Replace("RecursiveAI.OnePhysics.", "", StringComparison.Ordinal) + "." + m.Name;

    /// <summary>The middle-tier members that take a struct callback, one per type and name.</summary>
    public static IReadOnlyList<string> StructCallbackMembers() => StructGenericMethods(typeof(P2.PhysicsWorld2D).Assembly).Select(Key).Distinct(StringComparer.Ordinal).OrderBy(x => x, StringComparer.Ordinal).ToList();

    [Fact]
    public void EveryStructCallbackMemberHasAPlainCounterpart()
    {
        var asm = typeof(P2.PhysicsWorld2D).Assembly;
        var lines = new List<string>();
        foreach (var m in StructGenericMethods(asm))
        {
            if (!m.DeclaringType!.IsPublic) { continue; }
            // A plain counterpart: same type, same name (or the name without a trailing "Callback"/"Filter" spelling), no struct-constrained parameter.
            bool plain = m.DeclaringType.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly)
                .Any(o => o.Name == m.Name && !o.IsGenericMethodDefinition);
            if (!plain) { lines.Add(Key(m)); }
        }
        string path = Environment.GetEnvironmentVariable("ONEPHYSICS_TIER_DUMP") ?? "";
        if (path.Length > 0) { File.WriteAllLines(path, lines.Distinct(StringComparer.Ordinal).OrderBy(x => x, StringComparer.Ordinal)); }
        var allowed = File.ReadAllLines(Path.Combine(AppContext.BaseDirectory, "api", "tier1-struct-callbacks.txt"))
            .Select(l => l.Trim()).Where(l => l.Length > 0 && !l.StartsWith('#')).Select(l => l.Split(" -- ", 2)[0].Trim()).ToList();
        var unlisted = lines.Distinct(StringComparer.Ordinal).Except(allowed).ToList();
        Assert.False(unlisted.Count > 0, "struct-callback members without a plain counterpart:\n" + string.Join("\n", unlisted));
        var stale = allowed.Except(lines).ToList();
        Assert.False(stale.Count > 0, "stale entries:\n" + string.Join("\n", stale));
    }

    /// <summary>The happy-path world types expose no member that needs a struct callback or an interface implementation to be called.</summary>
    [Fact]
    public void HappyPathMembersTakeNoCallbackInterfaces()
    {
        const BindingFlags F = BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        foreach (var t in new[] { typeof(P2.PhysicsWorld2D), typeof(P3.PhysicsWorld3D), typeof(P2.Body), typeof(P3.Body), typeof(P2.Shape), typeof(P3.Shape), typeof(P2.Joint), typeof(P3.Joint), typeof(P2.Character), typeof(P3.Character) })
        {
            foreach (var m in t.GetMethods(F).Where(m => !m.IsSpecialName && !m.IsGenericMethodDefinition))
            {
                foreach (var p in m.GetParameters())
                {
                    bool callbackInterface = p.ParameterType.IsInterface && p.ParameterType.Namespace?.StartsWith("RecursiveAI.OnePhysics", StringComparison.Ordinal) == true;
                    Assert.False(callbackInterface, $"{t.Name}.{m.Name} takes the interface {p.ParameterType.Name}");
                }
            }
        }
    }
}
