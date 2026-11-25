using System;

namespace RecursiveAI.OnePhysics;

/// <summary>
/// A diagnostic hook for tests and tools in this repository: the hash-matrix tool installs an observer to check, in the middle of every scenario, that a mark and a rollback leave a world bit for bit as it was.
/// Nothing in the library sets it; with no observer a step pays one read of a static field. The observer is called on the stepping thread after a world's <c>Step</c> returns, never for the steps a settle runs.
/// </summary>
internal static class StepObserver
{
    internal static Action<object, float>? Observer;

    internal static void Notify(object world, float dt, bool settling)
    {
        var o = Observer;
        if (o != null && !settling) { o(world, dt); }
    }
}
