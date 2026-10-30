#if STAGE_TIMERS
using System;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>
/// Stage boundaries of a step, for benchmark builds only. The file and every call site are compiled in only when the <c>STAGE_TIMERS</c> symbol is defined, so the shipped package has no trace of them and a step is identical either way.
/// The benchmark sets <see cref="OnMark"/> to a function that reads its own clock; the library never reads one.
/// </summary>
internal static class StageProbe
{
    /// <summary>Soft step counts of the last substep, for the benchmark: particles, constraints, contacts, candidate shapes, particle pairs, colours.</summary>
    public static readonly int[] SoftCounts = new int[8];

    /// <summary>Stage numbers; <see cref="Mark"/> says the named stage starts now (and the previous one ended).</summary>
    public const int Boxes = 0, Pairs = 1, Narrowphase = 2, Apply = 3, Coloring = 4, Prepare = 5, Solve = 6, Finalize = 7, Store = 8, Sleep = 9, Events = 10, End = 11, Move = 12, Init = 13, Search = 14, Sort = 15, Alloc = 16, SoftSetup = 17, SoftPredict = 18, SoftDetect = 19, SoftPairs = 20, SoftConstraints = 21, SoftContacts = 22, SoftPairSolve = 23, SoftFinish = 24, SoftEnd = 25, Count = 26;

    /// <summary>Called on the stepping thread at each stage boundary. Null by default.</summary>
    public static Action<int>? OnMark { get; set; }

    /// <summary>Reports a stage boundary.</summary>
    public static void Mark(int stage) => OnMark?.Invoke(stage);

    /// <summary>
    /// Called on the stepping thread around each run of tasks of the solve program (an integration stage, or one colour or the overflow set of one pass) with the stage kind (0 integrate velocities, 1 integrate positions,
    /// 2 overflow, 3 colour), the phase (<see cref="SolvePhase"/>, 0 for integration), the number of tasks and whether it begins. Only the single-thread executor path reports; null by default.
    /// </summary>
    public static Action<int, int, int, bool>? OnSolveRun { get; set; }
}
#endif
