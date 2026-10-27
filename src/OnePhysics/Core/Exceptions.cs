using System;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>Base class of the errors the physics core raises for bad handles and bad snapshot bytes. Argument errors use the standard argument exceptions.</summary>
public class PhysicsException : InvalidOperationException
{
    /// <summary>Creates the exception with a message naming what was wrong.</summary>
    public PhysicsException(string message) : base(message) { }
}

/// <summary>A handle that cannot be used: the default (null) handle, a handle of a kind the operation does not accept, or the base of the two specific cases below.</summary>
public class InvalidHandleException : PhysicsException
{
    /// <summary>Creates the exception with a message naming the handle and what was wrong with it.</summary>
    public InvalidHandleException(string message) : base(message) { }
}

/// <summary>The handle refers to an object that was destroyed (its slot was freed, and possibly reused by a newer object with a higher generation).</summary>
public sealed class StaleHandleException : InvalidHandleException
{
    /// <summary>Creates the exception with a message naming the handle.</summary>
    public StaleHandleException(string message) : base(message) { }
}

/// <summary>The handle was created by a different world.</summary>
public sealed class ForeignHandleException : InvalidHandleException
{
    /// <summary>Creates the exception with a message naming the handle and both world ids.</summary>
    public ForeignHandleException(string message) : base(message) { }
}

/// <summary>Snapshot bytes that are truncated, of an unknown version, internally inconsistent or otherwise not a snapshot this world can restore. The world is unchanged when this is thrown.</summary>
public sealed class SnapshotFormatException : PhysicsException
{
    /// <summary>Byte offset in the snapshot where the problem was found.</summary>
    public int Offset { get; }

    /// <summary>Creates the exception for a problem at <paramref name="offset"/>.</summary>
    public SnapshotFormatException(string message, int offset) : base(message + " (at byte " + offset.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")")
    {
        Offset = offset;
    }
}
