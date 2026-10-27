using System;
using System.Buffers.Binary;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.HashMatrix;

/// <summary>
/// Collects the bytes a scenario produces, step by step. A scenario writes the state it wants compared and calls
/// <see cref="EndStep"/> after each simulation step; the runner keeps one SHA-256 per step, so a mismatch between two
/// platforms can be traced to the first step and scenario that differ. The scenario's hash is the SHA-256 of its step
/// digests in order. A scenario that never calls <see cref="EndStep"/> is a single step.
/// All writers are little-endian and write floating-point values by their bit patterns, so signed zeros count. Every
/// NaN is written as the same canonical quiet NaN, because the sign and payload of a NaN produced by arithmetic is
/// chosen by the CPU, not by this library.
/// </summary>
public sealed class ScenarioContext
{
    private Sha256 _step = new();
    private readonly Sha256 _all = new();
    private readonly List<byte[]> _digests = new();
    private readonly byte[] _scratch = new byte[8];
    private long _pending;

    /// <summary>Creates a context for a scenario that may use <paramref name="threads"/> workers.</summary>
    public ScenarioContext(int threads) => Threads = threads;

    /// <summary>
    /// Worker count a parallel scenario should use (at least 1). It comes from the runner's <c>--threads</c> option and is
    /// never derived from the machine. Results must not depend on it.
    /// </summary>
    public int Threads { get; }

    /// <summary>Number of steps closed so far.</summary>
    public int Steps => _digests.Count;

    /// <summary>Appends raw bytes to the current step.</summary>
    public void Write(ReadOnlySpan<byte> bytes)
    {
        _step.Append(bytes);
        _pending += bytes.Length;
    }

    /// <summary>Appends a 32-bit integer.</summary>
    public void Write(int value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(_scratch, value);
        Write(_scratch.AsSpan(0, 4));
    }

    /// <summary>Appends a 64-bit integer.</summary>
    public void Write(long value)
    {
        BinaryPrimitives.WriteInt64LittleEndian(_scratch, value);
        Write(_scratch.AsSpan(0, 8));
    }

    /// <summary>Appends the bit pattern of a float32.</summary>
    public void Write(float value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(_scratch, float.IsNaN(value) ? 0x7fc00000 : BitConverter.SingleToInt32Bits(value));
        Write(_scratch.AsSpan(0, 4));
    }

    /// <summary>Appends the bit pattern of a double.</summary>
    public void Write(double value)
    {
        BinaryPrimitives.WriteInt64LittleEndian(_scratch, double.IsNaN(value) ? 0x7ff8000000000000L : BitConverter.DoubleToInt64Bits(value));
        Write(_scratch.AsSpan(0, 8));
    }

    /// <summary>Closes the current step: its bytes are hashed and kept as that step's digest.</summary>
    public void EndStep()
    {
        byte[] digest = _step.Finish();
        _digests.Add(digest);
        _all.Append(digest);
        _step = new Sha256();
        _pending = 0;
    }

    /// <summary>The first 8 bytes of a step's digest, as 16 lower-case hex characters.</summary>
    public string StepDigest(int step) => Convert.ToHexStringLower(_digests[step].AsSpan(0, 8));

    /// <summary>Closes any bytes written after the last <see cref="EndStep"/> and returns the 32-byte scenario digest.</summary>
    public byte[] Finish()
    {
        if (_pending > 0 || _digests.Count == 0)
        {
            EndStep();
        }
        return _all.Finish();
    }
}
