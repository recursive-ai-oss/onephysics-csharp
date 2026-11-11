using System;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>A live stream: the records <see cref="LiveFeedStream"/> forwards, appended to a growing recording.</summary>
internal sealed class LiveSource(PhysicsSource source, float defaultDt) : ILiveSource
{
    private readonly StepAssembler _assembler = new(defaultDt);

    public IScopeSource Source => source;

    public void AppendRecord(ReadOnlySpan<byte> record)
    {
        int pos = 0;
        while (pos < record.Length)
        {
            if (record.Length - pos < 5)
            {
                throw new ScopeFormatException(source.Info.Name, $"expected a record header at offset {pos}.");
            }

            var kind = (ReplayRecordKind)record[pos];
            int length = System.Buffers.Binary.BinaryPrimitives.ReadInt32LittleEndian(record[(pos + 1)..]);
            if (length < 0 || length > record.Length - pos - 5)
            {
                throw new ScopeFormatException(source.Info.Name, $"expected {length} bytes of record at offset {pos + 5}, only {record.Length - pos - 5} remain.");
            }

            byte[]? input = _assembler.Accept(kind, record.Slice(pos + 5, length), out _);
            if (input is not null)
            {
                source.Recording.Append(input);
            }

            pos += 5 + length;
        }
    }
}
