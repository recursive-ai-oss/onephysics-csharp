using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

internal static class CoplanarPatchIds
{
    internal static int[] Create(int count)
    {
        var parents = new int[count]; for (int i = 0; i < count; i++) parents[i] = i; return parents;
    }
    private static int Root(int[] parents, int index)
    {
        int root = index;
        while (parents[root] != root) root = parents[root];
        while (parents[index] != index) { int next = parents[index]; parents[index] = root; index = next; }
        return root;
    }
    internal static void Join(int[] parents, int a, int b)
    {
        int ra = Root(parents, a), rb = Root(parents, b); if (ra < rb) parents[rb] = ra; else parents[ra] = rb;
    }
    internal static void Finish(int[] parents) { for (int i = 0; i < parents.Length; i++) parents[i] = Root(parents, i); }
    internal static bool[] Shared(int[] ids)
    {
        var shared = new bool[ids.Length];
        for (int i = 0; i < ids.Length; i++) if (ids[i] != i) shared[ids[i]] = true;
        for (int i = 0; i < ids.Length; i++) shared[i] = shared[ids[i]];
        return shared;
    }
    internal static bool SamePlane(Vector3F a0, Vector3F a1, Vector3F a2, Vector3F b0, Vector3F b1, Vector3F b2)
    {
        var a = new SoupTriangle3D(0, a0, a1, a2, 7); var b = new SoupTriangle3D(0, b0, b1, b2, 7);
        var na = a.Normal; var nb = b.Normal;
        return na == nb && Vector3F.Dot(na, a0) == Vector3F.Dot(nb, b0);
    }
}
