using System;
using System.Runtime.Intrinsics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Runtime.CompilerServices;

using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Raised by shape constructors for input that cannot make a valid shape. The message names what was wrong and where.</summary>
public sealed class InvalidShapeException : ArgumentException
{
    /// <summary>The category of the problem.</summary>
    public ShapeError Error { get; }

    /// <summary>Creates the exception.</summary>
    /// <param name="error">The kind of failure.</param>
    /// <param name="message">What was wrong and what was expected.</param>
    /// <param name="paramName">The caller's parameter that held the bad value, or null when it is not a single parameter.</param>
    public InvalidShapeException(ShapeError error, string message, string? paramName = null) : base(message, paramName) { Error = error; }

    /// <summary>The same failure reported against the caller's parameter and in the caller's terms.</summary>
    internal InvalidShapeException ForParameter(string paramName, string? message = null) => new(Error, message ?? Message, paramName);
}
