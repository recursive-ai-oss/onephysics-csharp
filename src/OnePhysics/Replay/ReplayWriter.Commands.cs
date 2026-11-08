using System;
using System.Buffers.Binary;
using System.Text;

namespace RecursiveAI.OnePhysics.Replay;

public sealed partial class ReplayWriter
{
    private readonly bool _commandTrack;
    private readonly byte _dimension;
    private readonly byte[] _nameBuffer = new byte[512];
    private bool _frameOpen;

    /// <summary>Writes a named mutation effect as a full snapshot. Requires commandTrack=true at construction. The printable ASCII operation name must occupy 1..512 bytes. Resulting world step count may differ after an explicit restore.</summary>
    public void WriteMutation(string operation, long worldStep, ReadOnlySpan<byte> snapshot)
    {
        EnsureCommands();
        ArgumentNullException.ThrowIfNull(operation);
        ArgumentOutOfRangeException.ThrowIfNegative(worldStep);
        int nameLength = Encoding.UTF8.GetByteCount(operation);
        if (nameLength < 1 || nameLength > _nameBuffer.Length || snapshot.IsEmpty || snapshot.Length > int.MaxValue - nameLength - 10) throw new ArgumentException("Expected bounded nonempty operation name and snapshot.", nameof(operation));
        foreach (char c in operation) if (c < 32 || c > 126) throw new ArgumentException("Expected printable ASCII operation name.", nameof(operation));
        Encoding.UTF8.GetBytes(operation, _nameBuffer);
        Header(ReplayRecordKind.Mutation, 10 + nameLength + snapshot.Length);
        BinaryPrimitives.WriteInt64LittleEndian(_scratch, worldStep);
        BinaryPrimitives.WriteUInt16LittleEndian(_scratch.AsSpan(8), (ushort)nameLength);
        _stream.Write(_scratch.AsSpan(0, 10));
        _stream.Write(_nameBuffer.AsSpan(0, nameLength));
        _stream.Write(snapshot);
        _completedSteps = worldStep;
    }

    /// <summary>Begins a simulated step in a command track. Callback-issued mutations are written after this record and before EndStep.</summary>
    public void BeginStep(float dt)
    {
        EnsureCommands();
        if (_frameOpen) throw new InvalidOperationException("A command frame is already open.");
        if (!(dt > 0) || !float.IsFinite(dt)) throw new ArgumentOutOfRangeException(nameof(dt));
        if (_completedSteps == long.MaxValue) throw new InvalidOperationException("Replay step count overflow.");
        Header(ReplayRecordKind.BeginStep, 4);
        BinaryPrimitives.WriteSingleLittleEndian(_scratch, dt);
        _stream.Write(_scratch.AsSpan(0,4));
        _completedSteps++;
        _frameOpen = true;
    }

    /// <summary>Finishes a command frame with the final world counter and optional final hash, after callback-issued mutations.</summary>
    public void EndStep(long worldStep, ulong hash)
    {
        EnsureCommands();
        if (!_frameOpen) throw new InvalidOperationException("No command frame is open.");
        ArgumentOutOfRangeException.ThrowIfNegative(worldStep);
        int length = _hashTrack ? 16 : 8;
        Header(ReplayRecordKind.EndStep, length);
        BinaryPrimitives.WriteInt64LittleEndian(_scratch, worldStep);
        if (_hashTrack) BinaryPrimitives.WriteUInt64LittleEndian(_scratch.AsSpan(8), hash);
        _stream.Write(_scratch.AsSpan(0,length));
        _completedSteps = worldStep;
        _frameOpen = false;
    }

    internal void WriteLiveOptions(ReadOnlySpan<byte> bytes)
    {
        EnsureCommands();
        Header(ReplayRecordKind.LiveOptions,bytes.Length);
        _stream.Write(bytes);
    }

    internal void Restart(long worldStep, ReadOnlySpan<byte> snapshot)
    {
        if (!_ended || _frameOpen) throw new InvalidOperationException("Complete the previous segment first.");
        _ended = false;
        _completedSteps = worldStep;
        BinaryPrimitives.WriteUInt32LittleEndian(_scratch, Magic);
        BinaryPrimitives.WriteUInt16LittleEndian(_scratch.AsSpan(4), _commandTrack ? (ushort)2 : Version);
        _scratch[6] = _dimension;
        _scratch[7] = _hashTrack ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteInt64LittleEndian(_scratch.AsSpan(8), worldStep);
        _stream.Write(_scratch);
        WriteSnapshot(snapshot);
    }

    private void EnsureCommands()
    {
        EnsureOpen();
        if (!_commandTrack) throw new InvalidOperationException("Construct a command-track writer first.");
    }
}
