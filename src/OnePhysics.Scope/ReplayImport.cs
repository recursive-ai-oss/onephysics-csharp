using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using System.Text;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// Reads a OnePhysics replay into a world setup and one input per step. The initial snapshot (and live options) build the world;
/// every later snapshot or named mutation becomes a stored state a step input refers to; our own pokes are read back as pokes.
/// The recorded hashes are not kept: the viewer's history computes them again, which is what detects a replay that does not reproduce.
/// </summary>
internal static class ReplayImport
{
    public static IScopeSource Open(byte[] data, string name, int threads)
    {
        try
        {
            return Read(data, name, threads);
        }
        catch (ReplayFormatException e)
        {
            throw new ScopeFormatException(name, e.Message);
        }
    }

    private static PhysicsSource Read(byte[] data, string name, int threads)
    {
        var reader = new ReplayReader(new MemoryStream(data, false), name);
        if (!reader.ReadNext() || reader.Kind != ReplayRecordKind.Snapshot)
        {
            throw new ScopeFormatException(name, "expected the initial snapshot record first.");
        }

        byte[] initial = reader.Payload.ToArray();
        byte[]? options = null;
        bool pending = false;
        if (reader.FormatVersion == 2 && reader.ReadNext())
        {
            if (reader.Kind == ReplayRecordKind.LiveOptions)
            {
                options = reader.Payload.ToArray();
            }
            else
            {
                pending = true;
            }
        }

        var states = new List<StoredState>();
        var inputs = new List<ReadOnlyMemory<byte>>();
        var pre = new ArrayBufferWriter<byte>();
        var post = new ArrayBufferWriter<byte>();
        bool framed = false;
        float frameDt = 0f;
        float current = 0f;
        float first = 0f;

        while (pending || reader.ReadNext())
        {
            pending = false;
            switch (reader.Kind)
            {
                case ReplayRecordKind.Snapshot:
                    StepOps.WriteState(pre, OpKind.State, Store(states, "Snapshot", reader.Payload));
                    break;
                case ReplayRecordKind.Mutation:
                    string op = Encoding.ASCII.GetString(reader.OperationName);
                    if (!framed && StepOps.TryParsePokeName(op, out StepOp poke))
                    {
                        StepOps.WritePoke(pre, poke);
                    }
                    else
                    {
                        StepOps.WriteState(framed ? post : pre, framed ? OpKind.PostState : OpKind.State, Store(states, op, reader.SnapshotPayload));
                    }

                    break;
                case ReplayRecordKind.BeginStep:
                    framed = true;
                    frameDt = reader.StepTime;
                    break;
                case ReplayRecordKind.Step:
                case ReplayRecordKind.EndStep:
                    float dt = reader.Kind == ReplayRecordKind.Step ? reader.StepTime : frameDt;
                    framed = false;
                    if (inputs.Count == 0)
                    {
                        first = current = dt;
                    }

                    var input = new ArrayBufferWriter<byte>(pre.WrittenCount + post.WrittenCount + 5);
                    if (BitConverter.SingleToUInt32Bits(dt) != BitConverter.SingleToUInt32Bits(current))
                    {
                        StepOps.WriteDt(input, dt);
                        current = dt;
                    }

                    input.Write(pre.WrittenSpan);
                    input.Write(post.WrittenSpan);
                    inputs.Add(input.WrittenSpan.ToArray());
                    pre.ResetWrittenCount();
                    post.ResetWrittenCount();
                    break;
            }
        }

        int dimension = reader.Dimension;
        var setup = new WorldSetup
        {
            Name = name,
            Dimension = dimension,
            Kind = options is null ? SetupKind.Snapshot : SetupKind.SnapshotWithOptions,
            Data = initial,
            Options = options,
            InitialStep = reader.InitialStep,
            DefaultDt = first > 0f ? first : 1f / 60f,
            States = states,
        };
        setup.CreateWorld(threads);
        return new OnePhysicsModule(threads).Make(setup, Recording.Create(setup.Header(), inputs));
    }

    private static uint Store(List<StoredState> states, string name, ReadOnlySpan<byte> snapshot)
    {
        states.Add(new StoredState(name, snapshot.ToArray()));
        return (uint)(states.Count - 1);
    }
}
