using System;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>A joint definition or joint edit that cannot be accepted: the same body twice, no dynamic body, limits the wrong way round, a zero axis. The message names the joint kind and the field. Non-finite or out-of-range numbers raise <see cref="System.ArgumentOutOfRangeException"/> instead; a dead or foreign body raises the handle exceptions. Nothing has changed when this is thrown.</summary>
public sealed class InvalidJointException : ArgumentException
{
    /// <summary>Creates the exception with a message naming what was wrong.</summary>
    /// <param name="message">What was wrong.</param>
    /// <param name="paramName">The caller's parameter that held the bad value (usually the definition); null when none applies.</param>
    public InvalidJointException(string message, string? paramName = null) : base(message, paramName) { }
}
