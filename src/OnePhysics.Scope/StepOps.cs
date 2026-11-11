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
    /// <summary>Replaces the whole world state with a stored snapshot before the step.</summary>
    State = 2,
    /// <summary>Replaces the whole world state with a stored snapshot after the step (changes made by callbacks in a recorded frame).</summary>
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
    public StepOp(OpKind kind, ulong target, float x, float y, float z, uint index = 0)
    {
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

    /// <summary>The stored snapshot number for state operations, or the float bits of dt.</summary>
    public uint Index { get; }

    public bool IsPoke => Kind >= OpKind.Impulse;

    public float Dt => BitConverter.UInt32BitsToSingle(Index);
}

/// <summary>
/// The byte encoding of a step's input, little endian. A step's input is any number of these entries in order:
/// <c>01 f32 dt</c>, <c>02 u32 snapshot</c>, <c>03 u32 snapshot</c>, <c>10/11/12 u64 body, f32 x, f32 y, f32 z</c>, <c>13 u64 body</c>.
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

    public static void WriteState(IBufferWriter<byte> into, OpKind kind, uint snapshot)
    {
        Span<byte> b = into.GetSpan(5);
        b[0] = (byte)kind;
        BinaryPrimitives.WriteUInt32LittleEndian(b[1..], snapshot);
        into.Advance(5);
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
    public static void Parse(ReadOnlySpan<byte> input, int snapshotCount, List<StepOp> into)
    {
        into.Clear();
        int pos = 0;
        while (pos < input.Length)
        {
            var kind = (OpKind)input[pos];
            int size = kind switch
            {
                OpKind.Dt or OpKind.State or OpKind.PostState => 5,
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
                case OpKind.State:
                case OpKind.PostState:
                    uint index = BinaryPrimitives.ReadUInt32LittleEndian(b);
                    if (index >= (uint)snapshotCount)
                    {
                        throw new ScopeFormatException(Input, $"expected a stored snapshot number below {snapshotCount} at offset {pos + 1}, found {index}.");
                    }

                    into.Add(new StepOp(kind, 0, 0, 0, 0, index));
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
            Parse(bytes, 0, ops);
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
