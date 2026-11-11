using System;
using System.Collections.Generic;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePhysics.Replay;

namespace RecursiveAI.OnePhysics.Scope;

/// <summary>
/// Reads a OnePhysics replay into a world setup, one input per step and the recorded per-step hashes. The initial snapshot (and live
/// options) build the world; every later snapshot or named mutation is held inside the step input it belongs to.
/// </summary>
internal static class ReplayImport
{
    /// <summary>Most steps one replay may hold. A step record is at least 5 bytes, so this also bounds the work a small file can ask for.</summary>
    public const int MaxSteps = 1 << 22;

    /// <summary>Most bytes all step inputs of one replay may hold together.</summary>
    public const long MaxTotalInputBytes = 1L << 30;

    public static PhysicsSource Open(byte[] data, string name, int threads)
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

        // The first step's time step is the world's starting one, so a replay at a constant dt carries no time-step operations.
        var inputs = new List<ReadOnlyMemory<byte>>();
        var hashes = new List<ulong>();
        StepAssembler? assembler = null;
        long total = 0;
        float firstDt = 0f;
        bool allHashed = reader.HasHashTrack;
        var pendingStates = new List<(ReplayRecordKind Kind, byte[] Payload)>();

        while (pending || reader.ReadNext())
        {
            pending = false;
            ReplayRecordKind kind = reader.Kind;
            if (kind is ReplayRecordKind.BeginStep or ReplayRecordKind.Step && firstDt == 0f)
            {
                firstDt = reader.StepTime;
                assembler = new StepAssembler(firstDt);
                foreach ((ReplayRecordKind k, byte[] p) in pendingStates)
                {
                    assembler.Accept(k, p, out _);
                }

                pendingStates.Clear();
            }

            if (assembler is null)
            {
                if (kind is ReplayRecordKind.Snapshot or ReplayRecordKind.Mutation)
                {
                    pendingStates.Add((kind, reader.Payload.ToArray()));
                }

                continue;
            }

            byte[]? input = assembler.Accept(kind, reader.Payload, out ulong? hash);
            if (input is null)
            {
                continue;
            }

            if (inputs.Count >= MaxSteps || (total += input.Length) > MaxTotalInputBytes)
            {
                throw new ScopeFormatException(name, $"expected at most {MaxSteps} steps and {MaxTotalInputBytes} bytes of step input.");
            }

            inputs.Add(input);
            allHashed &= hash.HasValue;
            hashes.Add(hash.GetValueOrDefault());
        }

        var setup = new WorldSetup
        {
            Name = name,
            Dimension = reader.Dimension,
            Kind = options is null ? SetupKind.Snapshot : SetupKind.SnapshotWithOptions,
            Data = initial,
            Options = options,
            InitialStep = reader.InitialStep,
            DefaultDt = firstDt > 0f ? firstDt : 1f / 60f,
        };
        ulong startHash = setup.CreateWorld(threads).Hash();
        ulong[] expected = [];
        if (allHashed && inputs.Count > 0)
        {
            expected = new ulong[inputs.Count + 1];
            expected[0] = startHash;
            hashes.CopyTo(expected, 1);
        }

        return new OnePhysicsModule(threads).Make(setup, Recording.Create(setup.Header(), inputs, setup.InitialStep, expected));
    }
}
