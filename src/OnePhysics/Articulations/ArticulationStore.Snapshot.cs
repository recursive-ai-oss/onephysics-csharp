using System;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Articulations;

internal sealed partial class ArticulationStore
{
    int ISnapshotSection.SectionId => Id;

    bool ISnapshotSection.IsOptional => true;

    bool ISnapshotSection.IsPresent => Handles.Count > 0;

    void ISnapshotSection.Reset() { Clear(); }

    private void Clear()
    {
        Handles.Reset();
        Array.Clear(States);
        Array.Fill(_artOfBody, -1); Array.Fill(_linkOfBody, -1);
    }

    private const int SectionVersion = 1;

    private const byte FlagFixedBase = 1, FlagPlanar = 2, FlagSelfCollision = 4, FlagGravityComp = 8, FlagTarget = 16, FlagTargetOrientation = 32, FlagExternal = 64;

    private static void WriteD3(StateWriter w, D3 v) { w.NamedDouble(v.X); w.NamedDouble(v.Y); w.NamedDouble(v.Z); }
    private static void WriteQ4(StateWriter w, Q4 q) { w.NamedDouble(q.X); w.NamedDouble(q.Y); w.NamedDouble(q.Z); w.NamedDouble(q.W); }

    void ISnapshotSection.Write(StateWriter w)
    {
        w.NamedInt32(SectionVersion);
        w.DescribeTable("articulation");
        Handles.Write(w);
        for (int i = 0; i < Handles.HighWater; i++)
        {
            w.DescribeEntity("articulation", i, Handles.GenerationOf(i));
            if (!Handles.IsLive(i)) { continue; }
            var s = States[i]!;
            var m = s.Model;
            w.NamedInt32(m.N);
            byte flags = (byte)((s.FixedBase ? FlagFixedBase : 0) | (s.Planar ? FlagPlanar : 0) | (s.SelfCollision ? FlagSelfCollision : 0) | (s.GravityCompensation ? FlagGravityComp : 0)
                | (s.HasTarget ? FlagTarget : 0) | (s.TargetOrientation ? FlagTargetOrientation : 0) | (s.AnyExternal ? FlagExternal : 0));
            w.NamedByte(flags);
            w.NamedInt32(s.Substeps); w.NamedInt32(s.Iterations); w.NamedDouble(s.GravityScale);
            for (int k = 0; k < m.N; k++)
            {
                w.NamedInt32(m.Parent[k]); w.NamedByte((byte)m.Kind[k]);
                WriteD3(w, m.Axis[k]); WriteQ4(w, m.PlaceRot[k]); WriteD3(w, m.PlacePos[k]);
                w.NamedDouble(m.Mass[k]); WriteD3(w, m.Com[k]);
                var t = m.Icom[k];
                w.NamedDouble(t.A); w.NamedDouble(t.B); w.NamedDouble(t.C); w.NamedDouble(t.E); w.NamedDouble(t.F); w.NamedDouble(t.I);
                w.NamedInt32(s.Body[k]); w.NamedInt32(s.BodyGen[k]);
                w.NamedDouble(m.Lower[k]); w.NamedDouble(m.Upper[k]); w.NamedBool(m.HasLimit[k]);
                w.NamedDouble(m.Stiff[k]); w.NamedDouble(m.Damp[k]); w.NamedDouble(m.MaxForce[k]); w.NamedDouble(m.Target[k]); w.NamedDouble(m.TargetVel[k]);
                w.NamedDouble(m.FeedFwd[k]); w.NamedDouble(m.Friction[k]); w.NamedDouble(m.Armature[k]); w.NamedDouble(m.JointDamping[k]); w.NamedDouble(m.MaxVel[k]);
                w.NamedDouble(m.ConeLimit[k]); WriteQ4(w, m.TargetRot[k]);
            }

            w.NamedDoubles(m.Q); w.NamedDoubles(m.Qd);
            w.NamedInt32(s.TargetLink); WriteD3(w, s.TargetLocal); WriteD3(w, s.TargetPosition); WriteQ4(w, s.TargetRotation);
            w.NamedDouble(s.IkGain); w.NamedDouble(s.IkDamping); w.NamedDouble(s.IkMaxStep);
            if (s.AnyExternal) { w.NamedDoubles(s.External); }
        }
    }

    // Bytes per link in the section: the fixed part of the record above.
    private const int LinkBytes = 4 + 1 + 8 * (3 + 4 + 3 + 1 + 3 + 6) + 8 + 8 * 18 + 4 * 4;

    private static double Finite(StateReader r, double limit, string what)
    {
        double v = r.ReadDouble();
        if (!(Math.Abs(v) <= limit)) { throw r.Error("invalid articulation " + what + " " + v.ToString("R", CultureInfo.InvariantCulture)); }
        return v;
    }

    private static D3 ReadD3(StateReader r, double limit, string what) => new(Finite(r, limit, what), Finite(r, limit, what), Finite(r, limit, what));

    private static Q4 ReadQ4(StateReader r, string what)
    {
        var q = new Q4(Finite(r, 1.5, what), Finite(r, 1.5, what), Finite(r, 1.5, what), Finite(r, 1.5, what));
        double l2 = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
        if (!(l2 > 0.999 && l2 < 1.001)) { throw r.Error("articulation " + what + " is not a unit quaternion"); }
        return q;
    }

    void ISnapshotSection.Read(StateReader r)
    {
        bool planar = _host.Planar;
        int version = r.ReadInt32();
        if (version != SectionVersion) { throw r.Error("unsupported articulation section version " + version.ToString(CultureInfo.InvariantCulture)); }
        var handles = HandleTable.Read(r, "articulation");
        var states = new ArticulationState?[Math.Max(handles.HighWater, 1)];
        var usedBodies = new System.Collections.Generic.HashSet<int>();
        for (int i = 0; i < handles.HighWater; i++)
        {
            if (!handles.IsLive(i)) { continue; }
            int n = r.ReadCount(LinkBytes, "articulation links");
            if (n < 1 || n > MaxLinks) { throw r.Error("articulation with " + n.ToString(CultureInfo.InvariantCulture) + " links"); }
            byte flags = r.ReadByte();
            if ((flags & ~(FlagFixedBase | FlagPlanar | FlagSelfCollision | FlagGravityComp | FlagTarget | FlagTargetOrientation | FlagExternal)) != 0 || ((flags & FlagPlanar) != 0) != planar) { throw r.Error("invalid articulation flags"); }
            var st = new ArticulationState { Planar = planar, FixedBase = (flags & FlagFixedBase) != 0, SelfCollision = (flags & FlagSelfCollision) != 0, GravityCompensation = (flags & FlagGravityComp) != 0, HasTarget = (flags & FlagTarget) != 0, TargetOrientation = (flags & FlagTargetOrientation) != 0, AnyExternal = (flags & FlagExternal) != 0 };
            st.Substeps = r.ReadInt32(); st.Iterations = r.ReadInt32(); st.GravityScale = Finite(r, 1e6, "gravity scale");
            if (st.Substeps < 0 || st.Substeps > 64 || st.Iterations < 0 || st.Iterations > 256) { throw r.Error("invalid articulation substeps or iterations"); }
            var specs = new LinkSpec[n];
            var prm = new double[n * 14];
            var tgt = new Q4[n];
            var cone = new double[n];
            var lim = new bool[n];
            st.Body = new int[n]; st.BodyGen = new int[n];
            for (int k = 0; k < n; k++)
            {
                int parent = r.ReadInt32();
                byte kind = r.ReadByte();
                bool root = k == 0;
                if (root ? parent != -1 : (parent < 0 || parent >= k)) { throw r.Error("articulation link " + k.ToString(CultureInfo.InvariantCulture) + " has an invalid parent"); }
                bool okKind = root ? (kind == (byte)JointKind.Fixed || (planar ? kind == (byte)JointKind.FreePlanar : kind == (byte)JointKind.Free))
                    : (kind == (byte)JointKind.Fixed || kind == (byte)JointKind.Revolute || kind == (byte)JointKind.Prismatic || (!planar && kind == (byte)JointKind.Spherical));
                if (!okKind) { throw r.Error("articulation link " + k.ToString(CultureInfo.InvariantCulture) + " has an invalid joint kind"); }
                var sp = new LinkSpec { Parent = parent, Kind = (JointKind)kind };
                sp.Axis = ReadD3(r, 1.0000001, "axis");
                sp.PlacementRot = ReadQ4(r, "placement");
                sp.PlacementPos = ReadD3(r, 1e7, "placement");
                sp.Mass = Finite(r, 1e12, "mass");
                sp.Com = ReadD3(r, 1e7, "centre of mass");
                double a = Finite(r, 1e12, "inertia"), b = Finite(r, 1e12, "inertia"), c = Finite(r, 1e12, "inertia"), e = Finite(r, 1e12, "inertia"), f = Finite(r, 1e12, "inertia"), ii = Finite(r, 1e12, "inertia");
                sp.Inertia = new M3(a, b, c, b, e, f, c, f, ii);
                if (!(sp.Mass > 0.0) || a < 0 || e < 0 || ii < 0) { throw r.Error("articulation link " + k.ToString(CultureInfo.InvariantCulture) + " has no positive mass"); }
                if (kind == (byte)JointKind.Revolute || kind == (byte)JointKind.Prismatic)
                {
                    double l2 = sp.Axis.X * sp.Axis.X + sp.Axis.Y * sp.Axis.Y + sp.Axis.Z * sp.Axis.Z;
                    if (!(l2 > 0.999 && l2 < 1.001)) { throw r.Error("articulation joint axis is not a unit vector"); }
                }

                specs[k] = sp;
                st.Body[k] = r.ReadInt32(); st.BodyGen[k] = r.ReadInt32();
                if (st.Body[k] < 0 || st.Body[k] > 1 << 26 || st.BodyGen[k] < 1 || !usedBodies.Add(st.Body[k])) { throw r.Error("invalid or repeated body in an articulation"); }
                int po = k * 14;
                prm[po] = Finite(r, 1e9, "limit"); prm[po + 1] = Finite(r, 1e9, "limit"); lim[k] = r.ReadBool();
                for (int q = 2; q < 12; q++) { prm[po + q] = Finite(r, 1e15, "joint parameter"); }
                cone[k] = Finite(r, 7.0, "cone limit");
                tgt[k] = ReadQ4(r, "target rotation");
                if (lim[k] && !(prm[po] <= prm[po + 1])) { throw r.Error("articulation joint limits out of order"); }
                if (prm[po + 2] < 0 || prm[po + 3] < 0 || prm[po + 4] < 0 || prm[po + 9] < 0 || prm[po + 10] < 0 || prm[po + 11] < 0) { throw r.Error("negative joint parameter in an articulation"); }
            }

            var model = new ArticulationModel(specs);
            for (int k = 0; k < n; k++)
            {
                int po = k * 14;
                model.Lower[k] = prm[po]; model.Upper[k] = prm[po + 1]; model.HasLimit[k] = lim[k];
                model.Stiff[k] = prm[po + 2]; model.Damp[k] = prm[po + 3]; model.MaxForce[k] = prm[po + 4]; model.Target[k] = prm[po + 5]; model.TargetVel[k] = prm[po + 6];
                model.FeedFwd[k] = prm[po + 7]; model.Friction[k] = prm[po + 8]; model.Armature[k] = prm[po + 9]; model.JointDamping[k] = prm[po + 10]; model.MaxVel[k] = prm[po + 11];
                model.ConeLimit[k] = cone[k]; model.TargetRot[k] = tgt[k];
            }

            r.ReadDoubles(model.Q, true, "articulation positions");
            r.ReadDoubles(model.Qd, true, "articulation velocities");
            for (int k = 0; k < n; k++)
            {
                int po = model.PosOff[k];
                if (model.Kind[k] == JointKind.Spherical || model.Kind[k] == JointKind.Free || model.Kind[k] == JointKind.FreePlanar)
                {
                    int o = model.Kind[k] == JointKind.Spherical ? po : po + 3;
                    double l2 = model.Q[o] * model.Q[o] + model.Q[o + 1] * model.Q[o + 1] + model.Q[o + 2] * model.Q[o + 2] + model.Q[o + 3] * model.Q[o + 3];
                    if (!(l2 > 0.999 && l2 < 1.001)) { throw r.Error("articulation joint rotation is not a unit quaternion"); }
                }

                if (Math.Abs(model.Q[po]) > 1e7) { throw r.Error("articulation position out of range"); }
            }

            st.TargetLink = r.ReadInt32();
            st.TargetLocal = ReadD3(r, 1e7, "target point"); st.TargetPosition = ReadD3(r, 1e7, "target"); st.TargetRotation = ReadQ4(r, "target orientation");
            st.IkGain = Finite(r, 1e6, "gain"); st.IkDamping = Finite(r, 1e6, "damping"); st.IkMaxStep = Finite(r, 1e6, "step limit");
            if (st.TargetLink < 0 || st.TargetLink >= n) { throw r.Error("articulation target link out of range"); }
            st.External = new double[6 * n];
            if (st.AnyExternal) { r.ReadDoubles(st.External, true, "articulation external forces"); }
            st.JointForce = new double[model.TotalDof];
            st.Model = model;
            states[i] = st;
        }

        Handles = handles; States = states;
        RebuildIndex();
    }

    // ------------------------------------------------------------------ fork

    ISnapshotSection ISnapshotSection.Fork(WorldCore destination)
    {
        var dst = ((IArticulationHost)destination.GetSnapshotSection(_host.EngineSectionId)).Articulations;
        dst.CopyFrom(this);
        return dst;
    }

    internal static ArticulationModel CloneModel(ArticulationModel src)
    {
        var specs = (LinkSpec[])src.Specs.Clone();
        for (int k = 0; k < specs.Length; k++) { specs[k].PlacementRot = src.PlaceRot[k]; specs[k].PlacementPos = src.PlacePos[k]; }
        var m = new ArticulationModel(specs);
        Array.Copy(src.Q, m.Q, src.Q.Length); Array.Copy(src.Qd, m.Qd, src.Qd.Length);
        Array.Copy(src.Lower, m.Lower, src.N); Array.Copy(src.Upper, m.Upper, src.N); Array.Copy(src.HasLimit, m.HasLimit, src.N);
        Array.Copy(src.Stiff, m.Stiff, src.N); Array.Copy(src.Damp, m.Damp, src.N); Array.Copy(src.MaxForce, m.MaxForce, src.N); Array.Copy(src.Target, m.Target, src.N); Array.Copy(src.TargetVel, m.TargetVel, src.N);
        Array.Copy(src.FeedFwd, m.FeedFwd, src.N); Array.Copy(src.Friction, m.Friction, src.N); Array.Copy(src.Armature, m.Armature, src.N); Array.Copy(src.JointDamping, m.JointDamping, src.N); Array.Copy(src.MaxVel, m.MaxVel, src.N);
        Array.Copy(src.ConeLimit, m.ConeLimit, src.N); Array.Copy(src.TargetRot, m.TargetRot, src.N);
        m.ForwardKinematics();
        return m;
    }

    private void CopyFrom(ArticulationStore src)
    {
        Handles = new HandleTable(); Handles.CopyFrom(src.Handles);
        States = new ArticulationState?[src.States.Length];
        for (int i = 0; i < States.Length; i++)
        {
            var s = src.States[i];
            if (s == null) { continue; }
            States[i] = new ArticulationState
            {
                Model = CloneModel(s.Model), Body = (int[])s.Body.Clone(), BodyGen = (int[])s.BodyGen.Clone(), Planar = s.Planar, FixedBase = s.FixedBase, Substeps = s.Substeps, Iterations = s.Iterations,
                SelfCollision = s.SelfCollision, GravityCompensation = s.GravityCompensation, GravityScale = s.GravityScale, HasTarget = s.HasTarget, TargetLink = s.TargetLink, TargetLocal = s.TargetLocal,
                TargetPosition = s.TargetPosition, TargetOrientation = s.TargetOrientation, TargetRotation = s.TargetRotation, IkGain = s.IkGain, IkDamping = s.IkDamping, IkMaxStep = s.IkMaxStep,
                External = (double[])s.External.Clone(), AnyExternal = s.AnyExternal, ContactPoints = s.ContactPoints, DeepestPenetration = s.DeepestPenetration, JointForce = (double[])s.JointForce.Clone(),
            };
        }

        _artOfBody = (int[])src._artOfBody.Clone(); _linkOfBody = (int[])src._linkOfBody.Clone();
    }
}
