#if STAGE_TIMERS
using System;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Stage boundaries of a step, for benchmark builds only. The file and every call site are compiled in only when the <c>STAGE_TIMERS</c> symbol is defined, so the shipped package has no trace of them and a step is identical either way.
/// The benchmark sets <see cref="OnMark"/> to a function that reads its own clock; the library never reads one.
/// </summary>
internal static class StageProbe
{
    /// <summary>Stage numbers; <see cref="Mark"/> says the named stage starts now (and the previous one ended).</summary>
    public const int Boxes = 0, Pairs = 1, Narrowphase = 2, Apply = 3, Coloring = 4, Prepare = 5, Solve = 6, Finalize = 7, Store = 8, Sleep = 9, Events = 10, End = 11, Count = 12;

    /// <summary>Called on the stepping thread at each stage boundary. Null by default.</summary>
    public static Action<int>? OnMark { get; set; }

    /// <summary>Reports a stage boundary.</summary>
    public static void Mark(int stage) => OnMark?.Invoke(stage);
}
#endif
