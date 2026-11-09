using System;
using System.Linq;
using System.Reflection;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tests.Api;

public class ExtendedQueryParityTests
{
    [Fact]
    public void NewSurfaceHasMatchingOverloadCountsAndNoRequiredStructCallbacks()
    {
        Check(typeof(P2.PhysicsWorld2D), typeof(P3.PhysicsWorld3D), ["QueryBatch", "QueryAnyBatch", "QueryAllBatch", "QueryAabb", "Penetration", "PenetrationAny", "PenetrationAll", "ComputePenetration", "SweepBody", "Depenetrate", "Depenetrate"]);
        Check(typeof(P2.Body), typeof(P3.Body), ["Raycast", "ShapeCast", "OverlapShape", "ClosestPoint", "Sweep", "Depenetrate", "Depenetrate"]);
        Check(typeof(P2.Shape), typeof(P3.Shape), ["Raycast", "ShapeCast", "OverlapShape", "ClosestPoint", "Depenetrate"]);
        Check(typeof(P2.QueryInput2D), typeof(P3.QueryInput3D), ["Ray", "Cast", "Overlap", "Closest", "Penetration"]);
    }

    private static void Check(Type planar, Type spatial, string[] names)
    {
        foreach (string name in names)
        {
            var a = planar.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static).Where(m => m.Name == name).ToArray();
            var b = spatial.GetMethods(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static).Where(m => m.Name == name).ToArray();
            Assert.NotEmpty(a);
            Assert.Equal(a.Length, b.Length);
            Assert.Equal(a.Select(m => m.GetParameters().Length), b.Select(m => m.GetParameters().Length));
            Assert.All(a.Concat(b), method => Assert.False(method.IsGenericMethodDefinition));
        }
    }
}
