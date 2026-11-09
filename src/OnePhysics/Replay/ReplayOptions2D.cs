using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics2D;

namespace RecursiveAI.OnePhysics.Replay;

internal static class ReplayOptions2D
{
    internal const int Size = 128;

    /// <summary>The payload when the soft-body options differ from their defaults: the base payload followed by SoftSubsteps and SoftIterations. A world with default soft options writes the base payload, so existing recordings keep their bytes.</summary>
    internal const int SizeWithSoft = 136;

    internal static bool HasSoftOptions(WorldOptions2D o) => o.SoftSubsteps != 4 || o.SoftIterations != 3;

    internal static int WrittenSize(WorldOptions2D o) => HasSoftOptions(o) ? SizeWithSoft : Size;
    internal static void Write(WorldOptions2D o, double creationMargin, int worldId, Span<byte> bytes)
    {
        BinaryPrimitives.WriteInt32LittleEndian(bytes, worldId);
        BinaryPrimitives.WriteDoubleLittleEndian(bytes.Slice(4), creationMargin);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(12), o.Gravity.X);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(16), o.Gravity.Y);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(20), o.Substeps);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(24), o.ContactSubsteps);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(28), o.ContactVelocityIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(32), o.ContactRelaxIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(36), o.VelocityIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(40), o.RelaxIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(44), o.PairCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(48), o.ContactCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(52), o.IslandCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(56), o.QueryScratchCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(60), o.SleepEnabled ? 1 : 0);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(64), o.TimeToSleep);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(68), o.SleepThreshold);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(72), (int)o.Events);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(76), o.ContactHertz);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(80), o.ContactDampingRatio);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(84), o.PushOutVelocity);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(88), o.ContactSlop);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(92), o.JointHertz);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(96), o.JointDampingRatio);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(100), o.RestitutionThreshold);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(104), o.MaxLinearSpeed);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(108), o.MaxRotationPerSubstep);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(112), o.ContinuousEnabled ? 1 : 0);
        BinaryPrimitives.WriteDoubleLittleEndian(bytes.Slice(116), o.BroadphaseMargin);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(124), o.HitSpeedThreshold);
        if (HasSoftOptions(o))
        {
            BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(128), o.SoftSubsteps);
            BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(132), o.SoftIterations);
        }
    }
    internal static double Read(ReadOnlySpan<byte> bytes, WorldOptions2D o, out int worldId)
    {
        if (bytes.Length != Size && bytes.Length != SizeWithSoft) throw new ArgumentException("Invalid 2D live option payload length.", nameof(bytes));
        worldId = BinaryPrimitives.ReadInt32LittleEndian(bytes);
        if (worldId < 1) throw new ArgumentException("Invalid world identity.", nameof(bytes));
        double margin = BinaryPrimitives.ReadDoubleLittleEndian(bytes.Slice(4));
        if (!double.IsFinite(margin) || margin < 0 || margin > 1048576) throw new ArgumentException("Invalid creation margin.", nameof(bytes));
        o.Gravity = new(BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(12)), BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(16)));
        o.Substeps = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(20));
        o.ContactSubsteps = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(24));
        o.ContactVelocityIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(28));
        o.ContactRelaxIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(32));
        o.VelocityIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(36));
        o.RelaxIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(40));
        o.PairCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(44));
        o.ContactCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(48));
        o.IslandCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(52));
        o.QueryScratchCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(56));
        if (BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(60)) != 0 && BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(60)) != 1) throw new ArgumentException("Invalid boolean SleepEnabled.", nameof(bytes));
        o.SleepEnabled = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(60)) != 0;
        o.TimeToSleep = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(64));
        o.SleepThreshold = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(68));
        o.Events = (EventMask)BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(72));
        o.ContactHertz = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(76));
        o.ContactDampingRatio = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(80));
        o.PushOutVelocity = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(84));
        o.ContactSlop = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(88));
        o.JointHertz = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(92));
        o.JointDampingRatio = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(96));
        o.RestitutionThreshold = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(100));
        o.MaxLinearSpeed = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(104));
        o.MaxRotationPerSubstep = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(108));
        if (BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(112)) != 0 && BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(112)) != 1) throw new ArgumentException("Invalid boolean ContinuousEnabled.", nameof(bytes));
        o.ContinuousEnabled = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(112)) != 0;
        o.BroadphaseMargin = BinaryPrimitives.ReadDoubleLittleEndian(bytes.Slice(116));
        o.HitSpeedThreshold = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(124));
        if (bytes.Length == SizeWithSoft)
        {
            o.SoftSubsteps = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(128));
            o.SoftIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(132));
        }
        o.Validate();
        return margin;
    }
    // Capacity hints describe host preallocation, not simulation state. File loading must not
    // allocate a potentially enormous pool solely because a tiny input requests one.
    internal static void PrepareCreation(WorldOptions2D options)
    {
        options.PairCapacity = 0;
        options.ContactCapacity = 0;
        options.IslandCapacity = 0;
        options.QueryScratchCapacity = 0;
    }

    internal static void Copy(WorldOptions2D from, WorldOptions2D to)
    {
        to.Gravity = from.Gravity;
        to.Substeps = from.Substeps;
        to.ContactSubsteps = from.ContactSubsteps;
        to.ContactVelocityIterations = from.ContactVelocityIterations;
        to.ContactRelaxIterations = from.ContactRelaxIterations;
        to.VelocityIterations = from.VelocityIterations;
        to.RelaxIterations = from.RelaxIterations;
        to.PairCapacity = from.PairCapacity;
        to.ContactCapacity = from.ContactCapacity;
        to.IslandCapacity = from.IslandCapacity;
        to.QueryScratchCapacity = from.QueryScratchCapacity;
        to.SleepEnabled = from.SleepEnabled;
        to.TimeToSleep = from.TimeToSleep;
        to.SleepThreshold = from.SleepThreshold;
        to.Events = from.Events;
        to.ContactHertz = from.ContactHertz;
        to.ContactDampingRatio = from.ContactDampingRatio;
        to.PushOutVelocity = from.PushOutVelocity;
        to.ContactSlop = from.ContactSlop;
        to.JointHertz = from.JointHertz;
        to.JointDampingRatio = from.JointDampingRatio;
        to.RestitutionThreshold = from.RestitutionThreshold;
        to.MaxLinearSpeed = from.MaxLinearSpeed;
        to.MaxRotationPerSubstep = from.MaxRotationPerSubstep;
        to.ContinuousEnabled = from.ContinuousEnabled;
        to.BroadphaseMargin = from.BroadphaseMargin;
        to.HitSpeedThreshold = from.HitSpeedThreshold;
        to.SoftSubsteps = from.SoftSubsteps;
        to.SoftIterations = from.SoftIterations;
    }
}
