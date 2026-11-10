using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>What a cloth link joins.</summary>
public enum ClothLinkKind
{
    /// <summary>A particle of one soft body held to a particle of another (or of the same body): a seam, a stitch.</summary>
    Seam = 0,

    /// <summary>A particle held to a point of a rigid body, which feels the reaction.</summary>
    Attachment = 1,
}

internal struct ClothLink
{
    public int Kind;
    public int SoftA, PartA;      // soft body slot and particle
    public int SoftB, PartB;      // Seam: the other soft body slot and particle
    public int Rigid, RigidGen;   // Attachment: the rigid body
    public float Lx, Ly, Lz;      // Attachment: the point in the rigid body's frame, relative to its centre of mass
    public float Compliance;      // Seam: m/N
}

internal sealed partial class Engine3D
{
    // Seams and attachments of cloth. A seam is three scalar equality constraints (one per world axis) between two particles, solved with the particle pairs of every pass (XPBD multipliers per axis, compliance as for the
    // other constraints). An attachment holds a particle to a point of a rigid body the way a bilateral contact would: the particle moves by its share of inverse mass, the body by the rest (position-level shares and one
    // impulse at the end of the step, as for contacts), so momentum is conserved. Links are state: they are saved, hashed and forked.

    /// <summary>Id of the cloth-link snapshot section ("CLNK").</summary>
    public const int ClothLinkSectionId = 0x4B4E4C43;

    public HandleTable LinkHandles = new();
    public ClothLink[] Links = new ClothLink[4];
    private ClothLinkSection? _linkSection;

    public ISnapshotSection ClothLinkSnapshotSection => _linkSection ??= new ClothLinkSection(this);

    // Scratch of the step: the links that take part in this substep, with the rigid anchor at the end of the substep.
    private int[] _lkId = new int[4];
    private int _lkCount;
    private float[] _lkAnchor = new float[12], _lkR = new float[12];   // per active link: anchor relative to the particle body's origin, and r from the rigid body's centre of mass
    private float[] _lkAcc = new float[12];                            // per active link: the multiplier (seam) or applied correction (attachment) per axis
    private bool[] _lkCoupled = new bool[4];
    private float _lkH, _lkInvH;
    private int[] _slotIdx = new int[4];

    public int AddClothLink(in ClothLink l)
    {
        int i = LinkHandles.Allocate();
        if (i >= Links.Length) { Array.Resize(ref Links, Math.Max(i + 1, Links.Length * 2)); }
        Links[i] = l;
        return i;
    }

    public void RemoveClothLink(int i)
    {
        LinkHandles.Free(i);
        Links[i] = default;
    }

    /// <summary>Removes every link that touches a soft body that is going away.</summary>
    private void DropLinksOf(int softSlot)
    {
        for (int i = 0; i < LinkHandles.HighWater; i++)
        {
            if (!LinkHandles.IsLive(i)) { continue; }
            ref var l = ref Links[i];
            if (l.SoftA == softSlot || (l.Kind == (int)ClothLinkKind.Seam && l.SoftB == softSlot)) { RemoveClothLink(i); }
        }
    }

    // A seam wakes the sleeping side when the other is awake.
    private void LinkWake()
    {
        for (int i = 0; i < LinkHandles.HighWater; i++)
        {
            if (!LinkHandles.IsLive(i)) { continue; }
            ref var l = ref Links[i];
            if (l.Kind != (int)ClothLinkKind.Seam) { continue; }
            if (_wakeFlag[l.SoftA] == 2 && _wakeFlag[l.SoftB] == 1) { _wakeFlag[l.SoftB] = 4; }
            else if (_wakeFlag[l.SoftB] == 2 && _wakeFlag[l.SoftA] == 1) { _wakeFlag[l.SoftA] = 4; }
        }
    }

    // Which links are active this substep, and where their rigid anchors are at its end.
    private void PrepareLinks(float t1, float h, float invH)
    {
        _lkCount = 0;
        if (LinkHandles.HighWater == 0) { return; }
        GrowTo(ref _slotIdx, SoftHandles.HighWater);
        for (int i = 0; i < SoftHandles.HighWater; i++) { _slotIdx[i] = -1; }
        for (int a = 0; a < _awakeSoftCount; a++) { _slotIdx[Array.IndexOf(SoftData, _awakeSoft[a])] = a; }
        _lkH = h; _lkInvH = invH;
        var bodies = Core.Bodies;
        for (int i = 0; i < LinkHandles.HighWater; i++)
        {
            if (!LinkHandles.IsLive(i)) { continue; }
            ref var l = ref Links[i];
            if ((uint)l.SoftA >= (uint)_slotIdx.Length || _slotIdx[l.SoftA] < 0) { continue; }
            GrowTo(ref _lkId, _lkCount + 1); GrowTo(ref _lkAnchor, 3 * (_lkCount + 1)); GrowTo(ref _lkR, 3 * (_lkCount + 1)); GrowTo(ref _lkAcc, 3 * (_lkCount + 1)); GrowTo(ref _lkCoupled, _lkCount + 1);
            int k = _lkCount;
            if (l.Kind == (int)ClothLinkKind.Seam)
            {
                if (_slotIdx[l.SoftB] < 0) { continue; }
            }
            else
            {
                int b = l.Rigid;
                // The rigid body is gone: so is the attachment.
                if (!bodies.Handles.IsAlive(b, l.RigidGen)) { RemoveClothLink(i); continue; }
                var d = SoftData[l.SoftA]!;
                BodyPoseAt(b, t1, out double bx, out double by, out double bz, out var q);
                var r = q.Rotate(new Vector3F(l.Lx, l.Ly, l.Lz));
                _lkAnchor[3 * k] = (float)(bx + r.X - d.Ox); _lkAnchor[3 * k + 1] = (float)(by + r.Y - d.Oy); _lkAnchor[3 * k + 2] = (float)(bz + r.Z - d.Oz);
                _lkR[3 * k] = r.X; _lkR[3 * k + 1] = r.Y; _lkR[3 * k + 2] = r.Z;
                bool coupled = bodies.IsDynamic(b) && bodies.IsAwake(b) && bodies.InverseMass[b] > 0f;
                _lkCoupled[k] = coupled;
                if (coupled) { TouchSoftBody(b); }
            }

            _lkAcc[3 * k] = 0f; _lkAcc[3 * k + 1] = 0f; _lkAcc[3 * k + 2] = 0f;
            _lkId[k] = i;
            _lkCount++;
        }
    }

    private void SolveLinks()
    {
        for (int k = 0; k < _lkCount; k++)
        {
            ref var l = ref Links[_lkId[k]];
            if (l.Kind == (int)ClothLinkKind.Seam) { SolveSeam(k, ref l); } else { SolveAttachment(k, ref l); }
        }
    }

    private void SolveSeam(int k, ref ClothLink l)
    {
        var da = SoftData[l.SoftA]!; var db = SoftData[l.SoftB]!;
        int ka = 3 * l.PartA, kb = 3 * l.PartB;
        float wa = da.InvMass[l.PartA], wb = db.InvMass[l.PartB];
        float at = l.Compliance / (_lkH * _lkH);
        float wsum = wa + wb + at;
        if (!(wsum > 0f)) { return; }
        for (int c = 0; c < 3; c++)
        {
            float dx = SoftDelta(da, db, ka, kb, c);
            float lam = (-dx - at * _lkAcc[3 * k + c]) / wsum;
            _lkAcc[3 * k + c] += lam;
            da.X[ka + c] += wa * lam;
            db.X[kb + c] -= wb * lam;
        }
    }

    private void SolveAttachment(int k, ref ClothLink l)
    {
        var bodies = Core.Bodies;
        var d = SoftData[l.SoftA]!;
        int kp = 3 * l.PartA;
        float wp = d.InvMass[l.PartA];
        bool coupled = _lkCoupled[k];
        int b = l.Rigid;
        float rx = _lkR[3 * k], ry = _lkR[3 * k + 1], rz = _lkR[3 * k + 2];
        float invM = coupled ? bodies.InverseMass[b] : 0f;
        for (int c = 0; c < 3; c++)
        {
            float nx = c == 0 ? 1f : 0f, ny = c == 1 ? 1f : 0f, nz = c == 2 ? 1f : 0f;
            float rnx = 0f, rny = 0f, rnz = 0f, iwx = 0f, iwy = 0f, iwz = 0f, wb = 0f;
            if (coupled)
            {
                rnx = ry * nz - rz * ny; rny = rz * nx - rx * nz; rnz = rx * ny - ry * nx;
                ApplyIw(b, rnx, rny, rnz, out iwx, out iwy, out iwz);
                wb = invM + rnx * iwx + rny * iwy + rnz * iwz;
            }

            float cv = d.X[kp + c] - _lkAnchor[3 * k + c] + wb * _lkAcc[3 * k + c];
            float wsum = wp + wb;
            if (!(wsum > 0f)) { continue; }
            float lam = -cv / wsum;
            _lkAcc[3 * k + c] += lam;
            d.X[kp + c] += wp * lam;
            if (coupled)
            {
                int kb = 3 * b;
                float li = lam * _lkInvH;
                _rbJ[kb] -= nx * li; _rbJ[kb + 1] -= ny * li; _rbJ[kb + 2] -= nz * li;
                _rbL[kb] -= rnx * li; _rbL[kb + 1] -= rny * li; _rbL[kb + 2] -= rnz * li;
                float sb = lam * invM;
                _rbDp[kb] -= nx * sb; _rbDp[kb + 1] -= ny * sb; _rbDp[kb + 2] -= nz * sb;
                _rbDr[kb] -= iwx * lam; _rbDr[kb + 1] -= iwy * lam; _rbDr[kb + 2] -= iwz * lam;
            }
        }
    }

    // ------------------------------------------------------------------ snapshot section

    // Layout (little-endian, in this order): i32 version (1), the handle table, then per live link in slot order: i32 kind; i32 soft slot A, i32 particle A; seam: i32 soft slot B, i32 particle B, f32 compliance;
    // attachment: i32 rigid body, i32 rigid generation, 3 x f32 anchor relative to the body's centre of mass.
    private sealed class ClothLinkSection : ISnapshotSection
    {
        private readonly Engine3D _e;
        public ClothLinkSection(Engine3D e) => _e = e;

        public int SectionId => ClothLinkSectionId;
        public bool IsOptional => true;
        public bool IsPresent => _e.LinkHandles.HighWater > 0;

        public void Reset()
        {
            if (_e.LinkHandles.HighWater == 0) { return; }
            _e.LinkHandles = new HandleTable();
            _e.Links = new ClothLink[4];
        }

        public ISnapshotSection Fork(WorldCore destination)
        {
            var dst = (Engine3D)destination.GetSnapshotSection(Id);
            dst.LinkHandles.CopyFrom(_e.LinkHandles);
            dst.Links = (ClothLink[])_e.Links.Clone();
            return dst.ClothLinkSnapshotSection;
        }

        public void Write(StateWriter w)
        {
            w.NamedInt32(1);
            w.DescribeTable("clothlink");
            _e.LinkHandles.Write(w);
            for (int i = 0; i < _e.LinkHandles.HighWater; i++)
            {
                w.DescribeEntity("clothlink", i, _e.LinkHandles.GenerationOf(i));
                if (!_e.LinkHandles.IsLive(i)) { continue; }
                var l = _e.Links[i];
                w.NamedInt32(l.Kind); w.NamedInt32(l.SoftA); w.NamedInt32(l.PartA);
                if (l.Kind == (int)ClothLinkKind.Seam) { w.NamedInt32(l.SoftB); w.NamedInt32(l.PartB); w.NamedSingle(l.Compliance); }
                else { w.NamedInt32(l.Rigid); w.NamedInt32(l.RigidGen); w.NamedSingle(l.Lx); w.NamedSingle(l.Ly); w.NamedSingle(l.Lz); }
            }
        }

        public void Read(StateReader r)
        {
            int version = r.ReadInt32();
            if (version != 1) { throw r.Error("unsupported cloth-link section version " + version.ToString(CultureInfo.InvariantCulture)); }
            var handles = HandleTable.Read(r, "cloth links");
            var links = new ClothLink[Math.Max(4, handles.HighWater)];
            // Bodies are checked against the snapshot being restored, which the live store does not yet hold.
            var bodies = _e.Core.RestoringBodies ?? _e.Core.Bodies;
            for (int i = 0; i < handles.HighWater; i++)
            {
                if (!handles.IsLive(i)) { continue; }
                var l = new ClothLink { Kind = r.ReadInt32(), SoftA = r.ReadInt32(), PartA = r.ReadInt32() };
                if (l.Kind != 0 && l.Kind != 1) { throw r.Error("invalid cloth-link kind"); }
                if (!ValidParticle(_e, l.SoftA, l.PartA)) { throw r.Error("cloth link names a soft-body particle that does not exist"); }
                if (l.Kind == (int)ClothLinkKind.Seam)
                {
                    l.SoftB = r.ReadInt32(); l.PartB = r.ReadInt32(); l.Compliance = r.ReadSingle();
                    if (!ValidParticle(_e, l.SoftB, l.PartB)) { throw r.Error("cloth seam names a soft-body particle that does not exist"); }
                    if (!(l.Compliance >= 0f && l.Compliance <= SoftBodyDef.MaxCompliance)) { throw r.Error("invalid cloth-seam compliance"); }
                    if (l.SoftA == l.SoftB && l.PartA == l.PartB) { throw r.Error("cloth seam joins a particle to itself"); }
                }
                else
                {
                    l.Rigid = r.ReadInt32(); l.RigidGen = r.ReadInt32();
                    l.Lx = r.ReadSingle(); l.Ly = r.ReadSingle(); l.Lz = r.ReadSingle();
                    if (!float.IsFinite(l.Lx) || !float.IsFinite(l.Ly) || !float.IsFinite(l.Lz)) { throw r.Error("cloth attachment anchor is not finite"); }
                    if (!bodies.Handles.IsAlive(l.Rigid, l.RigidGen)) { throw r.Error("cloth attachment names a rigid body that does not exist"); }
                }

                links[i] = l;
            }

            _e.LinkHandles = handles;
            _e.Links = links;
        }

        private static bool ValidParticle(Engine3D e, int slot, int part)
        {
            if ((uint)slot >= (uint)e.SoftHandles.HighWater || !e.SoftHandles.IsLive(slot)) { return false; }
            return (uint)part < (uint)e.SoftData[slot]!.N;
        }
    }
}
