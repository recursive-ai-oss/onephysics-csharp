using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Tools.Tests;

namespace Bar;

// decomposed-pile-3d: 24 concave bodies (L, U, ring, mug), each a compound of the hulls the convex decomposer made from its mesh, dropped onto a floor for 600 steps.
// No reference engine runs this scene (the decomposition is ours), so the thresholds are our own.
internal static class DecomposedPileStability
{
    internal static int Run()
    {
        var world = DecomposedPileScene.Build(out var bodies);
        double maxSpeed = 0;
        int escapes = 0, nan = 0;
        for (int i = 0; i < 600; i++) { world.Step(1f / 60); foreach (var b in bodies) { var v = b.LinearVelocity; maxSpeed = Math.Max(maxSpeed, Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z)); } }
        var finalSpeeds = new List<double>();
        double lowest = double.MaxValue;
        foreach (var b in bodies)
        {
            var p = b.Position; var v = b.LinearVelocity;
            if (!double.IsFinite(p.X + p.Y + p.Z)) { nan++; continue; }
            if (Math.Abs(p.X) > 16 || Math.Abs(p.Z) > 16 || p.Y < -0.6) { escapes++; }
            lowest = Math.Min(lowest, p.Y);
            finalSpeeds.Add(Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
        }

        finalSpeeds.Sort();
        double jitter = finalSpeeds.Count == 0 ? 0 : finalSpeeds[(int)(0.99 * (finalSpeeds.Count - 1))];
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"decomposed-pile-3d steps=600 bodies={bodies.Length} maxSpeed={maxSpeed:R} finalJitter={jitter:R} lowestOrigin={lowest:R} escapes={escapes} nan={nan}"));
        return nan == 0 && escapes == 0 && maxSpeed < 12 && jitter < 0.1 ? 0 : 1;
    }
}
