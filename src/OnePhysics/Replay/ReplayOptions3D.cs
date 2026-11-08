using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Replay;

internal static class ReplayOptions3D
{
    internal const int Size = 144;
    internal static void Write(WorldOptions3D o, double creationMargin, int worldId, Span<byte> bytes)
    {
        BinaryPrimitives.WriteInt32LittleEndian(bytes, worldId);
        BinaryPrimitives.WriteDoubleLittleEndian(bytes.Slice(4), creationMargin);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(12), o.SoftSubsteps);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(16), o.SoftIterations);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(20), o.Gravity.X);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(24), o.Gravity.Y);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(28), o.Gravity.Z);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(32), o.Substeps);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(36), o.VelocityIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(40), o.RelaxIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(44), o.ContactSubsteps);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(48), o.ContactVelocityIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(52), o.ContactRelaxIterations);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(56), o.PairCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(60), o.ContactCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(64), o.IslandCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(68), o.QueryScratchCapacity);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(72), o.SleepEnabled ? 1 : 0);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(76), o.TimeToSleep);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(80), o.SleepThreshold);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(84), (int)o.Events);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(88), o.ContactHertz);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(92), o.ContactDampingRatio);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(96), o.PushOutVelocity);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(100), o.ContactSlop);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(104), o.RestitutionThreshold);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(108), o.MaxLinearSpeed);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(112), o.MaxRotationPerSubstep);
        BinaryPrimitives.WriteInt32LittleEndian(bytes.Slice(116), o.ContinuousEnabled ? 1 : 0);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(120), o.JointHertz);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(124), o.JointDampingRatio);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(128), o.HitSpeedThreshold);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.Slice(132), o.SpeculativeDistance);
        BinaryPrimitives.WriteDoubleLittleEndian(bytes.Slice(136), o.BroadphaseMargin);
    }
    internal static double Read(ReadOnlySpan<byte> bytes, WorldOptions3D o, out int worldId)
    {
        if (bytes.Length != Size) throw new ArgumentException("Invalid 3D live option payload length.", nameof(bytes));
        worldId = BinaryPrimitives.ReadInt32LittleEndian(bytes);
        if (worldId < 1) throw new ArgumentException("Invalid world identity.", nameof(bytes));
        double margin = BinaryPrimitives.ReadDoubleLittleEndian(bytes.Slice(4));
        if (!double.IsFinite(margin) || margin < 0 || margin > 1048576) throw new ArgumentException("Invalid creation margin.", nameof(bytes));
        o.SoftSubsteps = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(12));
        o.SoftIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(16));
        o.Gravity = new(BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(20)), BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(24)), BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(28)));
        o.Substeps = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(32));
        o.VelocityIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(36));
        o.RelaxIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(40));
        o.ContactSubsteps = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(44));
        o.ContactVelocityIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(48));
        o.ContactRelaxIterations = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(52));
        o.PairCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(56));
        o.ContactCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(60));
        o.IslandCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(64));
        o.QueryScratchCapacity = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(68));
        if (BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(72)) != 0 && BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(72)) != 1) throw new ArgumentException("Invalid boolean SleepEnabled.", nameof(bytes));
        o.SleepEnabled = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(72)) != 0;
        o.TimeToSleep = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(76));
        o.SleepThreshold = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(80));
        o.Events = (EventMask)BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(84));
        o.ContactHertz = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(88));
        o.ContactDampingRatio = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(92));
        o.PushOutVelocity = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(96));
        o.ContactSlop = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(100));
        o.RestitutionThreshold = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(104));
        o.MaxLinearSpeed = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(108));
        o.MaxRotationPerSubstep = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(112));
        if (BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(116)) != 0 && BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(116)) != 1) throw new ArgumentException("Invalid boolean ContinuousEnabled.", nameof(bytes));
        o.ContinuousEnabled = BinaryPrimitives.ReadInt32LittleEndian(bytes.Slice(116)) != 0;
        o.JointHertz = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(120));
        o.JointDampingRatio = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(124));
        o.HitSpeedThreshold = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(128));
        o.SpeculativeDistance = BinaryPrimitives.ReadSingleLittleEndian(bytes.Slice(132));
        o.BroadphaseMargin = BinaryPrimitives.ReadDoubleLittleEndian(bytes.Slice(136));
        o.Validate();
        return margin;
    }
    internal static void Copy(WorldOptions3D from, WorldOptions3D to)
    {
        to.SoftSubsteps = from.SoftSubsteps;
        to.SoftIterations = from.SoftIterations;
        to.Gravity = from.Gravity;
        to.Substeps = from.Substeps;
        to.VelocityIterations = from.VelocityIterations;
        to.RelaxIterations = from.RelaxIterations;
        to.ContactSubsteps = from.ContactSubsteps;
        to.ContactVelocityIterations = from.ContactVelocityIterations;
        to.ContactRelaxIterations = from.ContactRelaxIterations;
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
        to.RestitutionThreshold = from.RestitutionThreshold;
        to.MaxLinearSpeed = from.MaxLinearSpeed;
        to.MaxRotationPerSubstep = from.MaxRotationPerSubstep;
        to.ContinuousEnabled = from.ContinuousEnabled;
        to.JointHertz = from.JointHertz;
        to.JointDampingRatio = from.JointDampingRatio;
        to.HitSpeedThreshold = from.HitSpeedThreshold;
        to.SpeculativeDistance = from.SpeculativeDistance;
        to.BroadphaseMargin = from.BroadphaseMargin;
    }
}
