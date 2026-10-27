using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;

// Shared by the NativeAOT and browser-WASM smoke apps: calls the package and checks the answer.
internal static class SmokeProgram
{
    private static int Main()
    {
        float strength = Gravity.Strength(new Vector3F(3f, 4f, 0f));
        Console.WriteLine(strength == 5f ? "OK 5" : "FAIL " + strength);
        return strength == 5f ? 0 : 1;
    }
}
