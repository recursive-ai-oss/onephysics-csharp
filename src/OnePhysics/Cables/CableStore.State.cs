using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Cables;

internal sealed partial class CableStore
{
    private static void WriteVec(StateWriter w, Vector3F v) { w.NamedSingle(v.X); w.NamedSingle(v.Y); w.NamedSingle(v.Z); }
    private static Vector3F ReadVec(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    public void Write(StateWriter w)
    {
        w.NamedInt32(1, "version");
        w.NamedInt32(Data.Length, "slots");
        for (int i = 0; i < Data.Length; i++)
        {
            w.DescribeEntity("cable", i, Generations[i]); w.NamedInt32(Generations[i], "generation"); w.NamedBool(Data[i] != null, "live");
            if (Data[i] is not { } m) continue;
            w.NamedBool(m.Closed); w.NamedBool(m.Chain); w.NamedInt32(m.Iterations); w.NamedSingle(m.BreakForce); w.NamedSingle(m.Preload); w.NamedSingle(m.Pitch);
            w.NamedBool(m.Broken); w.NamedSingle(m.LastDt); w.NamedSingle(m.AdvanceDt);
            w.NamedInt32(m.Nodes.Length, "nodes");
            foreach (var n in m.Nodes)
            {
                w.NamedInt32((int)n.Kind); w.NamedInt32(n.Body); w.NamedInt32(n.BodyGeneration);
                w.NamedDouble(n.Local.X); w.NamedDouble(n.Local.Y); w.NamedDouble(n.Local.Z);
                WriteVec(w, n.Axis); WriteVec(w, n.Ref1); WriteVec(w, n.Ref2);
                w.NamedSingle(n.Radius); w.NamedInt32(n.Side); w.NamedSingle(n.Friction); w.NamedBool(n.NoSlip); w.NamedInt32((int)n.Mode);
                w.NamedSingle(n.Inertia); w.NamedSingle(n.Bearing); w.NamedBool(n.FixedAxle);
                w.NamedSingle(n.MotorSpeed); w.NamedSingle(n.MotorTorque); w.NamedSingle(n.BrakeTorque);
                w.NamedSingle(n.Spin); w.NamedDouble(n.RotorAngle); w.NamedDouble(n.AlphaIn); w.NamedDouble(n.AlphaOut); w.NamedDouble(n.Reference);
                w.NamedInt32(n.SlipState); w.NamedSingle(n.MotorImpulse); w.NamedSingle(n.BrakeImpulse);
            }
            w.NamedDoubles(m.Material); w.NamedDoubles(m.Impulse); w.NamedDoubles(m.Stretch); w.NamedDoubles(m.Geometric); w.NamedDoubles(m.Free); w.NamedDoubles(m.Wrap);
        }
    }

    public void Read(StateReader r)
    {
        int version = r.ReadInt32(); if (version != 1) throw r.Error("Unsupported pulley system section version.");
        int slots = r.ReadCount(5, "pulley system slots"); if (slots > Limit) throw r.Error("Pulley system slot limit 65536 exceeded.");
        var data = new CableData?[slots]; var generations = new int[slots]; int count = 0;
        var bodies = Host.CableCore.RestoringBodies ?? Host.CableCore.Bodies;
        for (int i = 0; i < slots; i++)
        {
            int gen = r.ReadInt32(); if (gen < 1) throw r.Error("Pulley system generation must be positive."); generations[i] = gen;
            if (!r.ReadBool()) continue;
            var m = new CableData
            {
                Closed = r.ReadBool(), Chain = r.ReadBool(), Iterations = r.ReadInt32(), BreakForce = r.ReadSingle(), Preload = r.ReadSingle(), Pitch = r.ReadSingle(),
                Broken = r.ReadBool(), LastDt = r.ReadSingle(), AdvanceDt = r.ReadSingle(),
            };
            if (m.Iterations < 1 || m.Iterations > 256 || !(m.BreakForce > 0) || float.IsNaN(m.BreakForce) || !(m.Preload >= 0 && float.IsFinite(m.Preload)) || !(m.Pitch >= 0 && float.IsFinite(m.Pitch)) || !(m.LastDt >= 0 && float.IsFinite(m.LastDt)) || !(m.AdvanceDt >= 0 && float.IsFinite(m.AdvanceDt)))
                throw r.Error("Invalid pulley system options.");
            int n = r.ReadCount(100, "pulley system nodes");
            if (n < 2 || n > NodeLimit) throw r.Error("A pulley system has 2 to 256 nodes.");
            m.Nodes = new CableNode[n];
            for (int k = 0; k < n; k++)
            {
                var node = new CableNode { Kind = (CableNodeKind)r.ReadInt32(), Body = r.ReadInt32(), BodyGeneration = r.ReadInt32() };
                node.Local = new(r.ReadDouble(), r.ReadDouble(), r.ReadDouble());
                node.Axis = ReadVec(r); node.Ref1 = ReadVec(r); node.Ref2 = ReadVec(r);
                node.Radius = r.ReadSingle(); node.Side = r.ReadInt32(); node.Friction = r.ReadSingle(); node.NoSlip = r.ReadBool(); node.Mode = (WheelSpin)r.ReadInt32();
                node.Inertia = r.ReadSingle(); node.Bearing = r.ReadSingle(); node.FixedAxle = r.ReadBool();
                node.MotorSpeed = r.ReadSingle(); node.MotorTorque = r.ReadSingle(); node.BrakeTorque = r.ReadSingle();
                node.Spin = r.ReadSingle(); node.RotorAngle = r.ReadDouble(); node.AlphaIn = r.ReadDouble(); node.AlphaOut = r.ReadDouble(); node.Reference = r.ReadDouble();
                node.SlipState = (byte)r.ReadInt32(); node.MotorImpulse = r.ReadSingle(); node.BrakeImpulse = r.ReadSingle();
                if (!ValidNode(node, Host.Planar, bodies)) throw r.Error($"Invalid pulley system node {k}.");
                m.Nodes[k] = node;
            }
            int spans = m.Spans;
            m.Material = new double[spans]; m.Impulse = new double[spans]; m.Stretch = new double[spans]; m.Geometric = new double[spans]; m.Free = new double[spans]; m.Wrap = new double[n];
            r.ReadDoubles(m.Material, true, "pulley system material"); r.ReadDoubles(m.Impulse, true, "pulley system impulses"); r.ReadDoubles(m.Stretch, true, "pulley system stretch");
            r.ReadDoubles(m.Geometric, true, "pulley system span lengths"); r.ReadDoubles(m.Free, true, "pulley system free lengths"); r.ReadDoubles(m.Wrap, true, "pulley system wrap angles");
            foreach (double v in m.Impulse) if (!(v >= 0)) throw r.Error("Negative rope tension impulse.");
            data[i] = m; count++;
        }
        Data = data; Generations = generations; Count = count;
    }

    private static bool ValidNode(CableNode n, bool planar, BodyStore bodies)
    {
        if (n.Kind < CableNodeKind.Anchor || n.Kind > CableNodeKind.Drum || (n.Mode != WheelSpin.Rotor && n.Mode != WheelSpin.Body) || (n.Side != 1 && n.Side != -1) || n.SlipState > 2) return false;
        if (n.Body < -1 || (n.Body >= 0 && (n.Body >= bodies.HighWater || n.BodyGeneration <= 0))) return false;
        foreach (double v in new[] { n.Local.X, n.Local.Y, n.Local.Z, n.RotorAngle, n.AlphaIn, n.AlphaOut, n.Reference }) if (!double.IsFinite(v)) return false;
        foreach (float v in new[] { n.Axis.X, n.Axis.Y, n.Axis.Z, n.Ref1.X, n.Ref1.Y, n.Ref1.Z, n.Ref2.X, n.Ref2.Y, n.Ref2.Z, n.Spin, n.MotorImpulse, n.BrakeImpulse, n.MotorSpeed }) if (!float.IsFinite(v)) return false;
        if (n.IsWheel)
        {
            if (!(n.Radius > 0 && float.IsFinite(n.Radius)) || !(n.Friction >= 0 && float.IsFinite(n.Friction)) || !(n.Inertia > 0) || float.IsNaN(n.Inertia) || !(n.Bearing >= 0 && float.IsFinite(n.Bearing))) return false;
            if (!(n.MotorTorque >= 0 && float.IsFinite(n.MotorTorque)) || !(n.BrakeTorque >= 0 && float.IsFinite(n.BrakeTorque))) return false;
            double l = (double)n.Axis.X * n.Axis.X + (double)n.Axis.Y * n.Axis.Y + (double)n.Axis.Z * n.Axis.Z;
            if (!(l > 0.999 && l < 1.001)) return false;
            if (planar && (n.Axis.X != 0 || n.Axis.Y != 0)) return false;
        }
        if (planar && n.Local.Z != 0) return false;
        return true;
    }
}
