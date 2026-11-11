using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Globalization;
using OneScope.Sdk;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>What one entry of a step's input does.</summary>
internal enum OpKind : byte
{
    /// <summary>Sets the time step (float32 seconds) for this and later steps.</summary>
    Dt = 1,
    /// <summary>Replaces the whole world state with a snapshot held in the input, before the step.</summary>
    State = 2,
    /// <summary>Replaces the whole world state with a snapshot held in the input, after the step (changes made by callbacks in a recorded frame).</summary>
    PostState = 3,
    /// <summary>Adds a linear impulse to a movable body before the step.</summary>
    Impulse = 0x10,
    /// <summary>Sets a body's linear velocity before the step.</summary>
    SetVelocity = 0x11,
    /// <summary>Sets a body's position before the step.</summary>
    SetPosition = 0x12,
    /// <summary>Wakes a body before the step.</summary>
    Wake = 0x13,
}

/// <summary>One decoded entry of a step's input.</summary>
internal readonly struct StepOp
{
    public StepOp(OpKind kind, ulong target, float x, float y, float z, uint index = 0, string? name = null, int offset = 0, int length = 0)
    {
        Name = name;
        Offset = offset;
        Length = length;
        Kind = kind;
        Target = target;
        X = x;
        Y = y;
        Z = z;
        Index = index;
    }

    public OpKind Kind { get; }

    /// <summary>The body entity id, for pokes.</summary>
    public ulong Target { get; }

    public float X { get; }

    public float Y { get; }

    public float Z { get; }

    /// <summary>The float bits of dt.</summary>
    public uint Index { get; }

    /// <summary>The operation name of a stored state (the replay's mutation name).</summary>
    public string? Name { get; }

    /// <summary>Where a stored state's snapshot bytes lie in the step input.</summary>
    public int Offset { get; }

    public int Length { get; }

    public bool IsPoke => Kind >= OpKind.Impulse;

    public float Dt => BitConverter.UInt32BitsToSingle(Index);
}

/// <summary>
/// The byte encoding of a step's input, little endian. A step's input is any number of these entries in order:
/// <c>01 f32 dt</c>, <c>02/03 u16 name length, name, u32 snapshot length, snapshot</c>, <c>10/11/12 u64 body, f32 x, f32 y, f32 z</c>, <c>13 u64 body</c>.
/// Entries are self-delimiting, so the concatenation of encoded pokes is a valid input. An empty input is a plain step at the current dt.
/// </summary>
internal static class StepOps
{
    private const string Input = "step input";

    public static void WriteDt(IBufferWriter<byte> into, float dt)
    {
        Span<byte> b = into.GetSpan(5);
        b[0] = (byte)OpKind.Dt;
        BinaryPrimitives.WriteSingleLittleEndian(b[1..], dt);
        into.Advance(5);
    }

    public static void WriteState(IBufferWriter<byte> into, OpKind kind, string name, ReadOnlySpan<byte> snapshot)
    {
        byte[] n = System.Text.Encoding.ASCII.GetBytes(name);
        Span<byte> b = into.GetSpan(7);
        b[0] = (byte)kind;
        BinaryPrimitives.WriteUInt16LittleEndian(b[1..], (ushort)n.Length);
        into.Advance(3);
        into.Write(n);
        b = into.GetSpan(4);
        BinaryPrimitives.WriteUInt32LittleEndian(b, (uint)snapshot.Length);
        into.Advance(4);
        into.Write(snapshot);
    }

    public static void WritePoke(IBufferWriter<byte> into, in StepOp op)
    {
        int size = op.Kind == OpKind.Wake ? 9 : 21;
        Span<byte> b = into.GetSpan(size);
        b[0] = (byte)op.Kind;
        BinaryPrimitives.WriteUInt64LittleEndian(b[1..], op.Target);
        if (size == 21)
        {
            BinaryPrimitives.WriteSingleLittleEndian(b[9..], op.X);
            BinaryPrimitives.WriteSingleLittleEndian(b[13..], op.Y);
            BinaryPrimitives.WriteSingleLittleEndian(b[17..], op.Z);
        }

        into.Advance(size);
    }

    /// <summary>Decodes a whole input, checking every length and value before anything is applied.</summary>
    /// <exception cref="ScopeFormatException">The input is malformed or names a stored snapshot that does not exist.</exception>
    public static void Parse(ReadOnlySpan<byte> input, List<StepOp> into)
    {
        into.Clear();
        int pos = 0;
        while (pos < input.Length)
        {
            var kind = (OpKind)input[pos];
            if (kind is OpKind.State or OpKind.PostState)
            {
                pos = ParseState(input, pos, kind, into);
                continue;
            }

            int size = kind switch
            {
                OpKind.Dt => 5,
                OpKind.Impulse or OpKind.SetVelocity or OpKind.SetPosition => 21,
                OpKind.Wake => 9,
                _ => 0,
            };
            if (size == 0)
            {
                throw new ScopeFormatException(Input, $"expected an operation code at offset {pos}, found {input[pos]}.");
            }

            if (input.Length - pos < size)
            {
                throw new ScopeFormatException(Input, $"expected {size} bytes for operation {input[pos]} at offset {pos}, only {input.Length - pos} remain.");
            }

            ReadOnlySpan<byte> b = input.Slice(pos + 1, size - 1);
            switch (kind)
            {
                case OpKind.Dt:
                    float dt = BinaryPrimitives.ReadSingleLittleEndian(b);
                    if (!float.IsFinite(dt) || dt <= 0f)
                    {
                        throw new ScopeFormatException(Input, $"expected a positive finite time step at offset {pos + 1}, found {dt.ToString(CultureInfo.InvariantCulture)}.");
                    }

                    into.Add(new StepOp(kind, 0, 0, 0, 0, BitConverter.SingleToUInt32Bits(dt)));
                    break;
                case OpKind.Wake:
                    into.Add(new StepOp(kind, BinaryPrimitives.ReadUInt64LittleEndian(b), 0, 0, 0));
                    break;
                default:
                    float x = BinaryPrimitives.ReadSingleLittleEndian(b[8..]);
                    float y = BinaryPrimitives.ReadSingleLittleEndian(b[12..]);
                    float z = BinaryPrimitives.ReadSingleLittleEndian(b[16..]);
                    if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z))
                    {
                        throw new ScopeFormatException(Input, $"expected finite numbers for operation {input[pos]} at offset {pos + 9}.");
                    }

                    into.Add(new StepOp(kind, BinaryPrimitives.ReadUInt64LittleEndian(b), x, y, z));
                    break;
            }

            pos += size;
        }
    }

    private static int ParseState(ReadOnlySpan<byte> input, int pos, OpKind kind, List<StepOp> into)
    {
        int p = pos + 1;
        if (input.Length - p < 2)
        {
            throw new ScopeFormatException(Input, $"expected a name length at offset {p}.");
        }

        int nameLength = BinaryPrimitives.ReadUInt16LittleEndian(input[p..]);
        p += 2;
        if (nameLength is < 1 or > 512 || input.Length - p < nameLength + 4)
        {
            throw new ScopeFormatException(Input, $"expected a state name of 1 to 512 bytes and a length after it at offset {p}.");
        }

        ReadOnlySpan<byte> name = input.Slice(p, nameLength);
        foreach (byte c in name)
        {
            if (c < 32 || c > 126)
            {
                throw new ScopeFormatException(Input, $"expected printable ASCII in the state name at offset {p}.");
            }
        }

        p += nameLength;
        uint length = BinaryPrimitives.ReadUInt32LittleEndian(input[p..]);
        p += 4;
        if (length == 0 || length > (uint)(input.Length - p))
        {
            throw new ScopeFormatException(Input, $"expected {length} bytes of snapshot at offset {p}, only {input.Length - p} remain.");
        }

        into.Add(new StepOp(kind, 0, 0, 0, 0, 0, System.Text.Encoding.ASCII.GetString(name), p, (int)length));
        return p + (int)length;
    }

    /// <summary>The name a poke gets in a written replay: decodable, so reading the replay back gives the poke again.</summary>
    public static string PokeName(in StepOp op)
    {
        var buffer = new ArrayBufferWriter<byte>(24);
        WritePoke(buffer, op);
        return "scope.poke." + Convert.ToHexString(buffer.WrittenSpan);
    }

    /// <summary>Reads a name made by <see cref="PokeName"/>; false when the name is not one.</summary>
    public static bool TryParsePokeName(string name, out StepOp op)
    {
        op = default;
        const string prefix = "scope.poke.";
        if (!name.StartsWith(prefix, StringComparison.Ordinal) || name.Length > prefix.Length + 42)
        {
            return false;
        }

        byte[] bytes;
        try
        {
            bytes = Convert.FromHexString(name.AsSpan(prefix.Length));
        }
        catch (FormatException)
        {
            return false;
        }

        var ops = new List<StepOp>(1);
        try
        {
            Parse(bytes, ops);
        }
        catch (ScopeFormatException)
        {
            return false;
        }

        if (ops.Count != 1 || !ops[0].IsPoke)
        {
            return false;
        }

        op = ops[0];
        return true;
    }
}
