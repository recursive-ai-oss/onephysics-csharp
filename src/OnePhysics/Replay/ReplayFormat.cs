using System;

namespace RecursiveAI.OnePhysics.Replay;

/// <summary>A malformed replay envelope. Snapshot payloads are validated separately by the destination world.</summary>
public sealed class ReplayFormatException : PhysicsException
{
    /// <summary>Creates an error naming the input, zero-based byte offset, and expected content.</summary>
    public ReplayFormatException(string inputName, long offset, string expected)
        : base($"Replay '{inputName}' at byte {offset}: expected {expected}.")
    {
        InputName = inputName;
        Offset = offset;
    }

    /// <summary>The input name supplied to the reader, for example a file name.</summary>
    public string InputName { get; }

    /// <summary>Zero-based byte offset relative to the start of the replay, including on non-seekable streams.</summary>
    public long Offset { get; }
}

/// <summary>Record kinds in a version 1 checkpoint replay. Unknown kinds are rejected rather than skipped.</summary>
public enum ReplayRecordKind : byte
{
    /// <summary>A mandatory, zero-length terminator. It leaves any following stream content unread.</summary>
    End = 0,
    /// <summary>A full, unmodified world snapshot, at the current completed step.</summary>
    Snapshot = 1,
    /// <summary>A positive finite float32 time step in seconds, optionally followed by its resulting world hash.</summary>
    Step = 2,
    /// <summary>Version 2: begin a simulated step, before recorded callback mutations.</summary>
    BeginStep = 3,
    /// <summary>Version 2: a named mutation effect, containing resulting world step count and full snapshot.</summary>
    Mutation = 4,
    /// <summary>Version 2: finish a step after callback mutations, with the final step count and optional hash.</summary>
    EndStep = 5,
    /// <summary>Version 2: live simulation options, separate from the last synchronized core options in snapshots.</summary>
    LiveOptions = 6
}
