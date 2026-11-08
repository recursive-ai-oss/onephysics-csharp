using System;

namespace RecursiveAI.OnePhysics;

/// <summary>A delta or compressed snapshot has an invalid header, payload, checksum or base. The world is unchanged.</summary>
public sealed class SnapshotEncodingException : Exception
{
    /// <summary>Creates an encoding error naming the invalid part.</summary>
    /// <param name="message">Description of the invalid input.</param>
    public SnapshotEncodingException(string message) : base(message) { }
}
