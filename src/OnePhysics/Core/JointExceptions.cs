namespace RecursiveAI.OnePhysics.Core;

/// <summary>A joint definition or joint edit that cannot be accepted: the same body twice, no dynamic body, limits the wrong way round, a zero axis. The message names the joint kind and the field. Non-finite or out-of-range numbers raise <see cref="System.ArgumentOutOfRangeException"/> instead; a dead or foreign body raises the handle exceptions. Nothing has changed when this is thrown.</summary>
public sealed class InvalidJointException : PhysicsException
{
    /// <summary>Creates the exception with a message naming what was wrong.</summary>
    public InvalidJointException(string message) : base(message) { }
}
