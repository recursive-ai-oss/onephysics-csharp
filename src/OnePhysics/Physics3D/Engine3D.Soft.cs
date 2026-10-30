using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>Raised at the start of a step when a soft-body particle is not finite or is outside the supported range; the world is unchanged.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
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

internal sealed partial class Engine3D
{
    // Soft bodies are particle clouds solved by extended position-based dynamics after the rigid step; see docs/spec/softbody.md.
    // Their storage is their own (not the rigid body store): a soft body is not in the rigid islands, the contact stage or the rigid solver, so none of those change when bodies are soft.

    /// <summary>Largest number of particles in all soft bodies of a world (the particle numbering of the pair search uses 24 bits).</summary>
    public const int MaxSoftParticles = 1 << 24;

    public HandleTable SoftHandles = new();
    public SoftBodyData?[] SoftData = new SoftBodyData?[4];
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

        var data = SoftBodyData.Create(copy);
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
    public void ShiftSoft(double dx, double dy, double dz)
    {
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            for (int k = 0; k < d.N; k++)
            {
                d.Pos[3 * k] -= dx; d.Pos[3 * k + 1] -= dy; d.Pos[3 * k + 2] -= dz;
                d.Prev[3 * k] -= dx; d.Prev[3 * k + 1] -= dy; d.Prev[3 * k + 2] -= dz;
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
                double x = d.Pos[3 * k], y = d.Pos[3 * k + 1], z = d.Pos[3 * k + 2];
                float vx = d.Vel[3 * k], vy = d.Vel[3 * k + 1], vz = d.Vel[3 * k + 2];
                if (!(Math.Abs(x) <= Limit && Math.Abs(y) <= Limit && Math.Abs(z) <= Limit) || !float.IsFinite(vx + vy + vz))
                {
                    var h = new SoftBody(World!, i, SoftHandles.GenerationOf(i));
                    throw new SoftBodyOutOfBoundsException(h, k, "Particle " + k.ToString(CultureInfo.InvariantCulture) + " of soft body #" + i.ToString(CultureInfo.InvariantCulture) + " is at (" + x.ToString("R", CultureInfo.InvariantCulture) + ", " + y.ToString("R", CultureInfo.InvariantCulture) + ", " + z.ToString("R", CultureInfo.InvariantCulture) + ") with velocity (" + vx.ToString("R", CultureInfo.InvariantCulture) + ", " + vy.ToString("R", CultureInfo.InvariantCulture) + ", " + vz.ToString("R", CultureInfo.InvariantCulture) + "); positions must be finite and within ±" + Limit.ToString("R", CultureInfo.InvariantCulture) + ". Destroy the body or move it back, then step again.");
                }
            }
        }
    }

    // ------------------------------------------------------------------ snapshot section

    /// <summary>Id of the soft-body snapshot section ("SOFT").</summary>
    public const int SoftSectionId = 0x534F4654;

    private const int SoftSectionVersion = 1;
    private SoftSection? _softSection;

    public ISnapshotSection SoftSnapshotSection => _softSection ??= new SoftSection(this);

    /// <summary>The optional snapshot section that carries soft bodies. It is written only once the world has made a soft body, so worlds without them keep their bytes and hashes.</summary>
    private sealed class SoftSection : ISnapshotSection
    {
        private readonly Engine3D _e;
        public SoftSection(Engine3D e) => _e = e;

        public int SectionId => SoftSectionId;
        public bool IsOptional => true;
        public bool IsPresent => _e.SoftHandles.HighWater > 0;

        public void Reset()
        {
            _e.SoftHandles = new HandleTable();
            _e.SoftData = new SoftBodyData?[4];
            _e._softParticles = 0;
        }

        public void Write(StateWriter w) => _e.WriteSoft(w);

        public void Read(StateReader r) => _e.ReadSoft(r);
    }

    private void WriteSoft(StateWriter w)
    {
        w.WriteInt32(SoftSectionVersion);
        w.WriteInt32(Opt.SoftSubsteps); w.WriteInt32(Opt.SoftIterations);
        SoftHandles.Write(w);
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            var def = d.Def;
            var mesh = def.Mesh!;
            bool tets = mesh.TetrahedronCount > 0;
            w.WriteBool(tets);
            w.WriteInt32(mesh.VertexCount);
            for (int k = 0; k < mesh.VertexCount; k++) { var v = mesh.Vertices[k]; w.WriteSingle(v.X); w.WriteSingle(v.Y); w.WriteSingle(v.Z); }
            if (tets) { w.WriteInt32(mesh.TetrahedronCount); w.WriteInt32s(mesh.Tetrahedra); }
            else { w.WriteInt32(mesh.TriangleCount); w.WriteInt32s(mesh.Triangles); }
            w.WriteDouble(def.Position.X); w.WriteDouble(def.Position.Y); w.WriteDouble(def.Position.Z);
            w.WriteSingle(def.Rotation.X); w.WriteSingle(def.Rotation.Y); w.WriteSingle(def.Rotation.Z); w.WriteSingle(def.Rotation.W);
            w.WriteSingle(def.Scale);
            w.WriteSingle(def.LinearVelocity.X); w.WriteSingle(def.LinearVelocity.Y); w.WriteSingle(def.LinearVelocity.Z);
            w.WriteSingle(def.Mass); w.WriteSingle(def.ParticleRadius);
            w.WriteSingle(def.EdgeCompliance); w.WriteSingle(def.EdgeDamping); w.WriteSingle(def.BendCompliance); w.WriteSingle(def.BendDamping);
            w.WriteSingle(def.VolumeCompliance); w.WriteSingle(def.VolumeDamping); w.WriteSingle(def.InternalPressure);
            w.WriteSingle(def.Friction); w.WriteSingle(def.LinearDamping); w.WriteSingle(def.GravityScale);
            w.WriteBool(def.SelfCollision); w.WriteBool(def.AllowSleep);
            w.WriteUInt32(def.CategoryBits); w.WriteUInt32(def.MaskBits); w.WriteInt32(def.GroupIndex); w.WriteInt32(def.Layer);
            w.WriteBool(d.Asleep); w.WriteSingle(d.SleepTime);
            w.WriteDoubles(d.Pos); w.WriteDoubles(d.Prev); w.WriteSingles(d.Vel); w.WriteBytes(d.Pinned);
        }
    }

    private void ReadSoft(StateReader r)
    {
        int version = r.ReadInt32();
        if (version != SoftSectionVersion) { throw r.Error("unsupported soft-body section version " + version.ToString(CultureInfo.InvariantCulture)); }
        int substeps = r.ReadInt32(), iterations = r.ReadInt32();
        if (substeps < 1 || substeps > 64 || iterations < 1 || iterations > 32) { throw r.Error("invalid soft-body options"); }
        var handles = HandleTable.Read(r, "soft bodies");
        var data = new SoftBodyData?[Math.Max(4, handles.HighWater)];
        long particles = 0;
        for (int i = 0; i < handles.HighWater; i++)
        {
            if (!handles.IsLive(i)) { continue; }
            bool tets = r.ReadBool();
            int nv = r.ReadCount(12, "soft-body vertices");
            if (nv > SoftBodyMesh.MaxVertices) { throw r.Error("too many soft-body vertices"); }
            var verts = new Vector3F[nv];
            for (int k = 0; k < nv; k++) { verts[k] = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()); }
            int ne = r.ReadCount(tets ? 16 : 12, "soft-body elements");
            var idx = new int[ne * (tets ? 4 : 3)];
            r.ReadInt32s(idx);
            var def = new SoftBodyDef();
            try
            {
                def.Mesh = tets ? SoftBodyMesh.FromTetrahedra(verts, idx) : SoftBodyMesh.FromTriangles(verts, idx);
                def.Position = new Vector3D(r.ReadDouble(), r.ReadDouble(), r.ReadDouble());
                def.Rotation = new QuaternionF(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                def.Scale = r.ReadSingle();
                def.LinearVelocity = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                def.Mass = r.ReadSingle(); def.ParticleRadius = r.ReadSingle();
                def.EdgeCompliance = r.ReadSingle(); def.EdgeDamping = r.ReadSingle(); def.BendCompliance = r.ReadSingle(); def.BendDamping = r.ReadSingle();
                def.VolumeCompliance = r.ReadSingle(); def.VolumeDamping = r.ReadSingle(); def.InternalPressure = r.ReadSingle();
                def.Friction = r.ReadSingle(); def.LinearDamping = r.ReadSingle(); def.GravityScale = r.ReadSingle();
                def.SelfCollision = r.ReadBool(); def.AllowSleep = r.ReadBool();
                def.CategoryBits = r.ReadUInt32(); def.MaskBits = r.ReadUInt32(); def.GroupIndex = r.ReadInt32(); def.Layer = r.ReadInt32();
                def.Validate();
            }
            catch (SoftBodyException ex) { throw r.Error("invalid soft body: " + ex.Message); }

            var d = new SoftBodyData { Def = def };
            try { d.Build(); }
            catch (SoftBodyException ex) { throw r.Error("invalid soft body: " + ex.Message); }
            particles += d.N;
            if (particles > MaxSoftParticles) { throw r.Error("too many soft-body particles"); }
            d.Asleep = r.ReadBool(); d.SleepTime = r.ReadSingle();
            if (!(d.SleepTime >= 0f) || !float.IsFinite(d.SleepTime)) { throw r.Error("invalid soft-body sleep time"); }
            r.ReadDoubles(d.Pos, true, "soft-body positions"); r.ReadDoubles(d.Prev, true, "soft-body previous positions"); r.ReadSingles(d.Vel, true, "soft-body velocities");
            r.ReadBytes(d.Pinned);
            const double Limit = 4194304.0;
            for (int k = 0; k < 3 * d.N; k++)
            {
                if (!(Math.Abs(d.Pos[k]) <= Limit && Math.Abs(d.Prev[k]) <= Limit)) { throw r.Error("soft-body position out of range"); }
            }

            for (int k = 0; k < d.N; k++)
            {
                if (d.Pinned[k] > 1) { throw r.Error("invalid soft-body pin flag"); }
                d.InvMass[k] = d.Pinned[k] != 0 ? 0f : d.InvMass0[k];
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
}
