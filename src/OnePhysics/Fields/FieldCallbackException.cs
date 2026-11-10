namespace RecursiveAI.OnePhysics;

/// <summary>
/// A custom gravity field names a callback id that has no delegate bound in this world. Delegates are host context, not world state: a fork, and a snapshot restored into a fresh world, hold the field but not the delegate,
/// so the host binds it again (<c>BindGravityField</c>) before stepping. Raised at the start of <c>Step</c>, before anything changes, so binding the delegate and stepping again is safe.
/// </summary>
public sealed class FieldCallbackException : PhysicsException
{
    internal FieldCallbackException(int callbackId)
        : base("A CustomGravity field has no host callback bound to its CallbackId (" + callbackId.ToString(System.Globalization.CultureInfo.InvariantCulture) + "); bind one with BindGravityField before stepping.")
    { CallbackId = callbackId; }

    /// <summary>The callback id the field names.</summary>
    public int CallbackId { get; }
}
