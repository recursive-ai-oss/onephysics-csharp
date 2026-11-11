using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Text;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// Turns replay records into step inputs, one at a time. Mutations and checkpoint snapshots before a step become state
/// changes at the start of its input; mutations recorded inside a step frame (made by callbacks) become changes after it.
/// A recorded poke (our own naming) becomes a poke. Used by the file reader and by the live stream, so both give the same inputs.
/// </summary>
internal sealed class StepAssembler
{
    private const string Input = "replay";
    private readonly ArrayBufferWriter<byte> _pre = new();
    private readonly ArrayBufferWriter<byte> _post = new();
    private bool _framed;
    private float _frameDt;
    private float _current;

    public StepAssembler(float currentDt) => _current = currentDt;

    /// <summary>The time step the world runs with after the last input made so far.</summary>
    public float CurrentDt => _current;

    /// <summary>Adds one record's payload (as the replay reader hands it). Returns a finished step's input, or null.</summary>
    /// <exception cref="ScopeFormatException">The record is malformed or a state in it is larger than a step input may be.</exception>
    public byte[]? Accept(ReplayRecordKind kind, ReadOnlySpan<byte> payload, out ulong? hash)
    {
        hash = null;
        switch (kind)
        {
            case ReplayRecordKind.Snapshot:
                AddState(_pre, OpKind.State, "Snapshot", payload);
                return null;
            case ReplayRecordKind.Mutation:
                if (payload.Length < 11)
                {
                    throw new ScopeFormatException(Input, "expected a mutation record with a name and a snapshot.");
                }

                int nameLength = BinaryPrimitives.ReadUInt16LittleEndian(payload[8..]);
                if (nameLength < 1 || nameLength > 512 || payload.Length <= 10 + nameLength)
                {
                    throw new ScopeFormatException(Input, "expected a mutation record whose name is 1 to 512 bytes and is followed by a snapshot.");
                }

                string name = Encoding.ASCII.GetString(payload.Slice(10, nameLength));
                ReadOnlySpan<byte> snapshot = payload[(10 + nameLength)..];
                if (!_framed && StepOps.TryParsePokeName(name, out StepOp poke))
                {
                    StepOps.WritePoke(_pre, poke);
                }
                else
                {
                    AddState(_framed ? _post : _pre, _framed ? OpKind.PostState : OpKind.State, name, snapshot);
                }

                return null;
            case ReplayRecordKind.BeginStep:
                if (payload.Length != 4)
                {
                    throw new ScopeFormatException(Input, "expected a 4-byte step start record.");
                }

                _framed = true;
                _frameDt = ReadDt(payload);
                return null;
            case ReplayRecordKind.Step:
                if (payload.Length is not (4 or 12))
                {
                    throw new ScopeFormatException(Input, "expected a 4 or 12-byte step record.");
                }

                if (payload.Length == 12)
                {
                    hash = BinaryPrimitives.ReadUInt64LittleEndian(payload[4..]);
                }

                return Finish(ReadDt(payload));
            case ReplayRecordKind.EndStep:
                if (!_framed || payload.Length is not (8 or 16))
                {
                    throw new ScopeFormatException(Input, "expected a step end record after a step start.");
                }

                if (payload.Length == 16)
                {
                    hash = BinaryPrimitives.ReadUInt64LittleEndian(payload[8..]);
                }

                _framed = false;
                return Finish(_frameDt);
            default:
                return null;
        }
    }

    private static float ReadDt(ReadOnlySpan<byte> payload)
    {
        float dt = BinaryPrimitives.ReadSingleLittleEndian(payload);
        if (!float.IsFinite(dt) || dt <= 0f)
        {
            throw new ScopeFormatException(Input, "expected a positive finite time step.");
        }

        return dt;
    }

    private static void AddState(ArrayBufferWriter<byte> into, OpKind kind, string name, ReadOnlySpan<byte> snapshot)
    {
        if (into.WrittenCount + snapshot.Length + name.Length + 16L > Recording.MaxInputBytes)
        {
            throw new ScopeFormatException(Input, $"expected the state changes around one step to fit in {Recording.MaxInputBytes} bytes; found a snapshot of {snapshot.Length}.");
        }

        StepOps.WriteState(into, kind, name, snapshot);
    }

    private byte[] Finish(float dt)
    {
        var input = new ArrayBufferWriter<byte>(_pre.WrittenCount + _post.WrittenCount + 5);
        if (BitConverter.SingleToUInt32Bits(dt) != BitConverter.SingleToUInt32Bits(_current))
        {
            StepOps.WriteDt(input, dt);
            _current = dt;
        }

        input.Write(_pre.WrittenSpan);
        input.Write(_post.WrittenSpan);
        if (input.WrittenCount > Recording.MaxInputBytes)
        {
            throw new ScopeFormatException(Input, $"expected one step's input to fit in {Recording.MaxInputBytes} bytes; found {input.WrittenCount}.");
        }

        _pre.ResetWrittenCount();
        _post.ResetWrittenCount();
        return input.WrittenSpan.ToArray();
    }
}
