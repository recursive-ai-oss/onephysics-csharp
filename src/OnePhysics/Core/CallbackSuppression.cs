using System;

namespace RecursiveAI.OnePhysics;

/// <summary>What a world calls when a <see cref="CallbackSuppression"/> scope ends.</summary>
internal interface ICallbackGate
{
    void EndSuppression();
}

/// <summary>
/// A scope during which a world runs no per-body, per-joint or character callbacks, returned by <c>SuppressCallbacks()</c> on the 2D and 3D worlds. Polled events are still produced and still in the same order,
/// so a rollback host can restore a snapshot and resimulate frames without replaying sounds and particles. A struct: it allocates nothing. Dispose it once (a <c>using</c> does); scopes nest.
/// </summary>
public readonly struct CallbackSuppression : IDisposable
{
    private readonly ICallbackGate? _gate;

    internal CallbackSuppression(ICallbackGate gate) { _gate = gate; }

    /// <summary>Ends the scope; callbacks run again after the outermost scope has ended.</summary>
    public void Dispose() => _gate?.EndSuppression();
}
