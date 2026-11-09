using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Raised at the start of a step when a soft-body particle is not finite or is outside the supported range; the world is unchanged.</summary>
[Experimental("OPHYS0003")]
public sealed class SoftBodyOutOfBoundsException : PhysicsException
{
    /// <summary>The offending soft body.</summary>
    public SoftBody SoftBody { get; }

    /// <summary>Index of the offending particle.</summary>
    public int Particle { get; }

    /// <summary>Creates the exception.</summary>
    public SoftBodyOutOfBoundsException(SoftBody body, int particle, string message) : base(message)
    {
        SoftBody = body;
        Particle = particle;
    }
}

internal sealed partial class Engine2D
{
    // Soft bodies are particle clouds solved by extended position-based dynamics after the rigid step; see docs/spec/softbody.md.
    // Their storage is their own (not the rigid body store): a soft body is not in the rigid islands, the contact stage or the rigid solver, so none of those change when bodies are soft.

    /// <summary>Largest number of particles in all soft bodies of a world (the particle numbering of the pair search uses 24 bits).</summary>
    public const int MaxSoftParticles = 1 << 24;

    public HandleTable SoftHandles = new();
    public SoftBodyData?[] SoftData = new SoftBodyData?[4];

    /// <summary>Block size for the soft bodies this world creates or restores. Always the default outside tests, which set it on their own world (it decides constraint order, so it must never be process-wide).</summary>
    internal int SoftBlockSizeForTests = SoftBodyData.DefaultBlockSize;
    private int _softParticles;

    public bool HasSoftBodies => SoftHandles.HighWater > 0;

    public SoftBody AddSoftBody(SoftBodyDef def)
    {
        def.Validate();
        var copy = def.Clone();
        long total = (long)_softParticles + copy.Mesh!.VertexCount;
        if (total > MaxSoftParticles)
        {
            throw new SoftBodyException(SoftBodyError.BadCount, "the world would hold " + total.ToString(CultureInfo.InvariantCulture) + " soft-body particles; the limit is " + MaxSoftParticles.ToString(CultureInfo.InvariantCulture) + ".", "def");
        }

        var data = SoftBodyData.Create(copy, SoftBlockSizeForTests);
        int i = SoftHandles.Allocate();
        if (i >= SoftData.Length) { Array.Resize(ref SoftData, Math.Max(i + 1, SoftData.Length * 2)); }
        SoftData[i] = data;
        _softParticles += data.N;
        return new SoftBody(World!, i, SoftHandles.GenerationOf(i));
    }

    public void RemoveSoftBody(int i)
    {
        _softParticles -= SoftData[i]!.N;
        SoftData[i] = null;
        SoftHandles.Free(i);
    }

    /// <summary>Moves every soft body with the world origin.</summary>
    public void ShiftSoft(double dx, double dy)
    {
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            for (int k = 0; k < d.N; k++)
            {
                d.Pos[2 * k] -= dx; d.Pos[2 * k + 1] -= dy;
                d.Prev[2 * k] -= dx; d.Prev[2 * k + 1] -= dy;
            }

            if (!d.HasTargets) { continue; }
            for (int k = 0; k < d.N; k++)
            {
                d.TargetPos[2 * k] -= dx; d.TargetPos[2 * k + 1] -= dy;
                d.TargetPrev[2 * k] -= dx; d.TargetPrev[2 * k + 1] -= dy;
            }
        }
    }

    /// <summary>Called before anything of a step changes: refuses a soft body with a particle that is not finite or has left the supported range.</summary>
    public void CheckSoftBounds()
    {
        const double Limit = 4194304.0;
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            if (d.Asleep) { continue; }
            for (int k = 0; k < d.N; k++)
            {
                double x = d.Pos[2 * k], y = d.Pos[2 * k + 1];
                float vx = d.Vel[2 * k], vy = d.Vel[2 * k + 1];
                if (!(Math.Abs(x) <= Limit && Math.Abs(y) <= Limit) || !float.IsFinite(vx + vy))
                {
                    var h = new SoftBody(World!, i, SoftHandles.GenerationOf(i));
                    throw new SoftBodyOutOfBoundsException(h, k, "Particle " + k.ToString(CultureInfo.InvariantCulture) + " of soft body #" + i.ToString(CultureInfo.InvariantCulture) + " is at (" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + ") with velocity (" + vx.ToString("R", CultureInfo.InvariantCulture) + ", " + vy.ToString("R", CultureInfo.InvariantCulture) + "); positions must be finite and within ±" + Limit.ToString("R", CultureInfo.InvariantCulture) + ". Destroy the body or move it back, then step again.");
                }
            }
        }
    }

    // ------------------------------------------------------------------ snapshot section

    /// <summary>Id of the 2D soft-body snapshot section ("SF2D").</summary>
    public const int SoftSectionId = 0x53463244;

    private const int SoftSectionVersion = 1;
    private SoftSection? _softSection;

    public ISnapshotSection SoftSnapshotSection => _softSection ??= new SoftSection(this);

    /// <summary>The optional snapshot section that carries soft bodies. It is written only once the world has made a soft body, so worlds without them keep their bytes and hashes.</summary>
    private sealed class SoftSection : ISnapshotSection
    {
        private readonly Engine2D _e;
        public SoftSection(Engine2D e) => _e = e;

        public int SectionId => SoftSectionId;
        public bool IsOptional => true;
        public bool IsPresent => _e.SoftHandles.HighWater > 0;

        public void Reset()
        {
            // A world that never made a soft body restores without allocating.
            if (_e.SoftHandles.HighWater == 0 && _e._softParticles == 0) { return; }
            _e.SoftHandles = new HandleTable();
            _e.SoftData = new SoftBodyData?[4];
            _e._softParticles = 0;
        }

        public ISnapshotSection Fork(WorldCore destination) => ((Engine2D)destination.GetSnapshotSection(Engine2D.Id)).SoftSnapshotSection;
        public void Write(StateWriter w) => _e.WriteSoft(w);

        public void Read(StateReader r) => _e.ReadSoft(r);
    }

    private void WriteSoft(StateWriter w)
    {
        w.NamedInt32(SoftSectionVersion);
        w.NamedInt32(Opt.SoftSubsteps); w.NamedInt32(Opt.SoftIterations);
        w.DescribeTable("softbody");
        SoftHandles.Write(w);
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            w.DescribeEntity("softbody", i, SoftHandles.GenerationOf(i));
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            var def = d.Def;
            var mesh = def.Mesh!;
            w.NamedInt32(mesh.VertexCount);
            for (int k = 0; k < mesh.VertexCount; k++) { var v = mesh.Vertices[k]; w.NamedSingle(v.X); w.NamedSingle(v.Y); }
            w.NamedInt32(mesh.TriangleCount); w.NamedInt32s(mesh.Triangles);
            w.NamedDouble(def.Position.X); w.NamedDouble(def.Position.Y);
            w.NamedSingle(def.Angle); w.NamedSingle(def.Scale);
            w.NamedSingle(def.LinearVelocity.X); w.NamedSingle(def.LinearVelocity.Y); w.NamedSingle(def.AngularVelocity);
            w.NamedSingle(def.Mass); w.NamedSingle(def.ParticleRadius);
            w.NamedByte((byte)def.Model);
            w.NamedSingle(def.EdgeCompliance); w.NamedSingle(def.EdgeDamping); w.NamedSingle(def.BendCompliance); w.NamedSingle(def.BendDamping);
            w.NamedSingle(def.AreaCompliance); w.NamedSingle(def.AreaDamping); w.NamedSingle(def.InternalPressure);
            w.NamedSingle(def.MatchCompliance); w.NamedBool(def.MatchQuadratic); w.NamedSingle(def.MatchBlend); w.NamedInt32(def.ClusterSize);
            w.NamedSingle(def.Friction); w.NamedSingle(def.LinearDamping); w.NamedSingle(def.GravityScale);
            w.NamedBool(def.SelfCollision); w.NamedBool(def.AllowSleep);
            w.NamedUInt32(def.CategoryBits); w.NamedUInt32(def.MaskBits); w.NamedInt32(def.GroupIndex); w.NamedInt32(def.Layer);
            w.NamedBool(d.Asleep); w.NamedSingle(d.SleepTime);
            w.NamedDoubles(d.Pos); w.NamedDoubles(d.Prev); w.NamedSingles(d.Vel); w.NamedBytes(d.Pinned);
            w.NamedBool(d.HasTargets);
            if (d.HasTargets) { w.NamedBytes(d.TargetOn); w.NamedSingles(d.TargetMax); w.NamedDoubles(d.TargetPos); w.NamedDoubles(d.TargetPrev); }
        }
    }

    private void ReadSoft(StateReader r)
    {
        int version = r.ReadInt32();
        if (version != SoftSectionVersion) { throw r.Error("unsupported 2D soft-body section version " + version.ToString(CultureInfo.InvariantCulture)); }
        int substeps = r.ReadInt32(), iterations = r.ReadInt32();
        if (substeps < 1 || substeps > 64 || iterations < 1 || iterations > 32) { throw r.Error("invalid soft-body options"); }
        var handles = HandleTable.Read(r, "soft bodies");
        var data = new SoftBodyData?[Math.Max(4, handles.HighWater)];
        long particles = 0;
        for (int i = 0; i < handles.HighWater; i++)
        {
            if (!handles.IsLive(i)) { continue; }
            int nv = r.ReadCount(8, "soft-body vertices");
            if (nv > SoftBodyMesh.MaxVertices) { throw r.Error("too many soft-body vertices"); }
            var verts = new Vector2F[nv];
            for (int k = 0; k < nv; k++) { verts[k] = new Vector2F(r.ReadSingle(), r.ReadSingle()); }
            int ne = r.ReadCount(12, "soft-body triangles");
            var idx = new int[ne * 3];
            r.ReadInt32s(idx);
            var def = new SoftBodyDef();
            try
            {
                def.Mesh = ne == 0 ? SoftBodyMesh.FromOutline(verts) : SoftBodyMesh.FromTriangles(verts, idx);
                // The mesh repairs a clockwise triangle by swapping two indices. A written snapshot never holds one, so a changed index list means the bytes are not ours and would not write back the same.
                if (ne > 0 && !def.Mesh.Triangles.SequenceEqual(idx)) { throw r.Error("soft-body triangle wound clockwise"); }
                def.Position = new Vector2D(r.ReadDouble(), r.ReadDouble());
                def.Angle = r.ReadSingle(); def.Scale = r.ReadSingle();
                def.LinearVelocity = new Vector2F(r.ReadSingle(), r.ReadSingle()); def.AngularVelocity = r.ReadSingle();
                def.Mass = r.ReadSingle(); def.ParticleRadius = r.ReadSingle();
                byte model = r.ReadByte();
                if (model > (byte)SoftBodyModel.Both) { throw r.Error("invalid soft-body model"); }
                def.Model = (SoftBodyModel)model;
                def.EdgeCompliance = r.ReadSingle(); def.EdgeDamping = r.ReadSingle(); def.BendCompliance = r.ReadSingle(); def.BendDamping = r.ReadSingle();
                def.AreaCompliance = r.ReadSingle(); def.AreaDamping = r.ReadSingle(); def.InternalPressure = r.ReadSingle();
                def.MatchCompliance = r.ReadSingle(); def.MatchQuadratic = r.ReadBool(); def.MatchBlend = r.ReadSingle(); def.ClusterSize = r.ReadInt32();
                def.Friction = r.ReadSingle(); def.LinearDamping = r.ReadSingle(); def.GravityScale = r.ReadSingle();
                def.SelfCollision = r.ReadBool(); def.AllowSleep = r.ReadBool();
                def.CategoryBits = r.ReadUInt32(); def.MaskBits = r.ReadUInt32(); def.GroupIndex = r.ReadInt32(); def.Layer = r.ReadInt32();
                def.Validate();
            }
            catch (SoftBodyException ex) { throw r.Error("invalid soft body: " + ex.Message); }

            var d = new SoftBodyData { Def = def, BlockSize = SoftBlockSizeForTests };
            try { d.Build(); }
            catch (SoftBodyException ex) { throw r.Error("invalid soft body: " + ex.Message); }
            particles += d.N;
            if (particles > MaxSoftParticles) { throw r.Error("too many soft-body particles"); }
            d.Asleep = r.ReadBool(); d.SleepTime = r.ReadSingle();
            if (!(d.SleepTime >= 0f) || !float.IsFinite(d.SleepTime)) { throw r.Error("invalid soft-body sleep time"); }
            r.ReadDoubles(d.Pos, true, "soft-body positions"); r.ReadDoubles(d.Prev, true, "soft-body previous positions"); r.ReadSingles(d.Vel, true, "soft-body velocities");
            r.ReadBytes(d.Pinned);
            if (r.ReadBool())
            {
                d.EnsureTargets();
                r.ReadBytes(d.TargetOn); r.ReadSingles(d.TargetMax, true, "soft-body target distances");
                r.ReadDoubles(d.TargetPos, true, "soft-body targets"); r.ReadDoubles(d.TargetPrev, true, "soft-body previous targets");
                for (int k = 0; k < d.N; k++)
                {
                    if (d.TargetOn[k] > 1) { throw r.Error("invalid soft-body target flag"); }
                    if (!(d.TargetMax[k] >= 0f && d.TargetMax[k] <= 1e6f)) { throw r.Error("soft-body target distance out of range"); }
                    if (!(Math.Abs(d.TargetPos[2 * k]) <= 4194304.0 && Math.Abs(d.TargetPos[2 * k + 1]) <= 4194304.0 && Math.Abs(d.TargetPrev[2 * k]) <= 4194304.0 && Math.Abs(d.TargetPrev[2 * k + 1]) <= 4194304.0)) { throw r.Error("soft-body target out of range"); }
                }
            }

            const double Limit = 4194304.0;
            for (int k = 0; k < 2 * d.N; k++)
            {
                if (!(Math.Abs(d.Pos[k]) <= Limit && Math.Abs(d.Prev[k]) <= Limit)) { throw r.Error("soft-body position out of range"); }
            }

            for (int k = 0; k < d.N; k++)
            {
                if (d.Pinned[k] > 1) { throw r.Error("invalid soft-body pin flag"); }
                d.RefreshInvMass(k);
            }

            d.Def.PinnedParticles = null;
            data[i] = d;
        }

        // Everything validated: replace the state.
        Opt.SoftSubsteps = substeps; Opt.SoftIterations = iterations;
        SoftHandles = handles;
        SoftData = data;
        _softParticles = (int)particles;
    }

    /// <summary>Emits one entry per soft body, in slot order: the particles as points, the edges as index pairs, and the targets (when there are any) as further points.</summary>
    public void AddDebugSoftBodies(Debug.DebugGeometry g)
    {
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            int first = -1;
            for (int p = 0; p < d.N; p++)
            {
                int k = g.AddPoint(new Vector3D(d.Pos[2 * p], d.Pos[2 * p + 1], 0.0));
                if (p == 0) { first = k; }
            }

            int firstIndex = -1;
            for (int e = 0; e < d.EdgeA.Length; e++)
            {
                int k = g.AddIndex(d.EdgeA[e]);
                g.AddIndex(d.EdgeB[e]);
                if (e == 0) { firstIndex = k; }
            }

            int firstTarget = -1;
            if (d.HasTargets)
            {
                for (int p = 0; p < d.N; p++)
                {
                    bool on = d.TargetOn[p] != 0;
                    int k = g.AddPoint(new Vector3D(on ? d.TargetPos[2 * p] : d.Pos[2 * p], on ? d.TargetPos[2 * p + 1] : d.Pos[2 * p + 1], 0.0));
                    if (p == 0) { firstTarget = k; }
                }
            }

            g.Add(new Debug.DebugSoftBody(i, SoftHandles.GenerationOf(i), d.Asleep, d.Radius, first, d.N, firstIndex, d.EdgeA.Length, firstTarget));
        }
    }
}
