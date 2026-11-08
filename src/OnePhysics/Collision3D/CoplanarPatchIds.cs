using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

internal static class CoplanarPatchIds
{
    // Exact equal planes receive the lowest triangle id. Lookup order never affects the chosen id: triangles are visited in numeric order.
    internal static int[] Build(TriangleSoup3D soup)
    {
        var ids = new int[soup.TriangleIdCount];
        var planes = new SortedDictionary<(float X, float Y, float Z, float Offset), int>();
        for (int i = 0; i < ids.Length; i++)
        {
            ids[i] = i;
            if (!soup.TryGetTriangle(i, out var triangle)) continue;
            var n = triangle.Normal; var key = (n.X, n.Y, n.Z, Vector3F.Dot(n, triangle.V0));
            if (!planes.TryGetValue(key, out int first)) { first = i; planes.Add(key, first); }
            ids[i] = first;
        }
        return ids;
    }
}
