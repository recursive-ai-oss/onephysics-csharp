using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

// Shared by the NativeAOT and browser-WASM smoke apps: calls the package and checks the answer.
internal static class SmokeProgram
{
    private static int Main()
    {
        float strength = Gravity.Strength(new Vector3F(3f, 4f, 0f));
        // Convex collision results must be bit-identical on every platform: hash a fixed set of queries.
        ulong hash = Collision3DHash.Compute();
        bool same = hash == Collision3DHash.Expected;
        Console.WriteLine(strength == 5f ? "OK 5" : "FAIL " + strength);
        Console.WriteLine((same ? "OK collision3d " : "FAIL collision3d ") + hash.ToString("x16", System.Globalization.CultureInfo.InvariantCulture));
        return strength == 5f && same ? 0 : 1;
    }
}
