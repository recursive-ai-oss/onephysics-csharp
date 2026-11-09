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
        DropLinksOf(i);
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

            for (int j = 0; j < d.JointPos.Length / 3; j++)
            {
                d.JointPos[3 * j] -= dx; d.JointPos[3 * j + 1] -= dy; d.JointPos[3 * j + 2] -= dz;
                d.JointPosPrev[3 * j] -= dx; d.JointPosPrev[3 * j + 1] -= dy; d.JointPosPrev[3 * j + 2] -= dz;
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

    // Version 1 holds no skinned body. Version 2 adds, per body, a skin flag after the definition fields and the joint poses after the pins. Version 3 (always written) adds per body, after the skin, the surface switch,
    // the tearing and plasticity settings and the broken edges, after the sleep time the smoothed net speed that decides rest, and after the pose a flag for the rest values plasticity changed, followed by them.
    private const int SoftSectionVersion = 4;

    // Version 4 adds, per body after the broken edges, a flag for fabric settings and the settings themselves. It is written only while a body has them, so worlds without cloth keep their bytes and hashes (version 3).
    private bool AnyFabric()
    {
        for (int i = 0; i < SoftHandles.HighWater; i++) { if (SoftHandles.IsLive(i) && SoftData[i]!.Def.Fabric != null) { return true; } }
        return false;
    }
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
            // A world that never made a soft body restores without allocating.
            if (_e.SoftHandles.HighWater == 0 && _e._softParticles == 0) { return; }
            _e.SoftHandles = new HandleTable();
            _e.SoftData = new SoftBodyData?[4];
            _e._softParticles = 0;
        }

        public ISnapshotSection Fork(WorldCore destination) => ((Engine3D)destination.GetSnapshotSection(Id)).SoftSnapshotSection;
        public void Write(StateWriter w) => _e.WriteSoft(w);

        public void Read(StateReader r) => _e.ReadSoft(r);
    }

    private void WriteSoft(StateWriter w)
    {
        const bool anySkin = true;
        int wver = AnyFabric() ? 4 : 3;
        w.NamedInt32(wver);
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
            bool tets = mesh.TetrahedronCount > 0;
            w.NamedBool(tets);
            w.NamedInt32(mesh.VertexCount);
            for (int k = 0; k < mesh.VertexCount; k++) { var v = mesh.Vertices[k]; w.NamedSingle(v.X); w.NamedSingle(v.Y); w.NamedSingle(v.Z); }
            if (tets) { w.NamedInt32(mesh.TetrahedronCount); w.NamedInt32s(mesh.Tetrahedra); }
            else { w.NamedInt32(mesh.TriangleCount); w.NamedInt32s(mesh.Triangles); }
            w.NamedDouble(def.Position.X); w.NamedDouble(def.Position.Y); w.NamedDouble(def.Position.Z);
            w.NamedSingle(def.Rotation.X); w.NamedSingle(def.Rotation.Y); w.NamedSingle(def.Rotation.Z); w.NamedSingle(def.Rotation.W);
            w.NamedSingle(def.Scale);
            w.NamedSingle(def.LinearVelocity.X); w.NamedSingle(def.LinearVelocity.Y); w.NamedSingle(def.LinearVelocity.Z);
            w.NamedSingle(def.Mass); w.NamedSingle(def.ParticleRadius);
            w.NamedSingle(def.EdgeCompliance); w.NamedSingle(def.EdgeDamping); w.NamedSingle(def.BendCompliance); w.NamedSingle(def.BendDamping);
            w.NamedSingle(def.VolumeCompliance); w.NamedSingle(def.VolumeDamping); w.NamedSingle(def.InternalPressure);
            w.NamedSingle(def.Friction); w.NamedSingle(def.LinearDamping); w.NamedSingle(def.GravityScale);
            w.NamedBool(def.SelfCollision); w.NamedBool(def.AllowSleep);
            w.NamedUInt32(def.CategoryBits); w.NamedUInt32(def.MaskBits); w.NamedInt32(def.GroupIndex); w.NamedInt32(def.Layer);
            if (anySkin)
            {
                w.NamedBool(d.IsSkinned);
                if (d.IsSkinned)
                {
                    var sk = def.Skin!;
                    w.NamedInt32(sk.JointCount);
                    w.NamedInt32s(sk.ParentArray); w.NamedSingles(sk.BindArray); w.NamedInt32s(sk.JointArray); w.NamedSingles(sk.WeightArray);
                    w.NamedSingles(sk.MaxArray); w.NamedSingles(sk.BackDistArray); w.NamedSingles(sk.BackRadiusArray); w.NamedSingles(sk.NormalArray);
                }
            }

            w.NamedBool(def.SurfaceCollision); w.NamedSingle(def.TearStrain); w.NamedSingle(def.TearForce);
            w.NamedSingle(def.PlasticYield); w.NamedSingle(def.PlasticCreep); w.NamedSingle(def.PlasticMax);
            w.NamedInt32(d.Broken.Length);
            foreach (long k in d.Broken) { w.NamedInt64(k, "def.BrokenEdge"); }
            if (wver >= 4)
            {
                var fab = def.Fabric;
                w.NamedBool(fab != null);
                if (fab != null)
                {
                    w.NamedSingle(fab.WarpAxis.X); w.NamedSingle(fab.WarpAxis.Y); w.NamedSingle(fab.WarpAxis.Z);
                    w.NamedSingle(fab.WeftAxis.X); w.NamedSingle(fab.WeftAxis.Y); w.NamedSingle(fab.WeftAxis.Z);
                    w.NamedSingle(fab.WarpCompliance); w.NamedSingle(fab.WeftCompliance); w.NamedSingle(fab.ShearCompliance);
                    w.NamedSingle(fab.StrainLimit); w.NamedBool(fab.DihedralBend); w.NamedSingle(fab.Drag); w.NamedSingle(fab.LiftSlope); w.NamedSingle(fab.TetherStrain);
                    int na = fab.TetherAnchors?.Count ?? 0;
                    w.NamedInt32(na);
                    for (int k = 0; k < na; k++) { w.NamedInt32(fab.TetherAnchors![k]); }
                }
            }

            w.NamedBool(d.Asleep); w.NamedSingle(d.SleepTime); w.NamedSingle(d.StepSpeed2);
            w.NamedDoubles(d.Pos); w.NamedDoubles(d.Prev); w.NamedSingles(d.Vel); w.NamedBytes(d.Pinned);
            if (d.IsSkinned) { w.NamedDoubles(d.JointPos); w.NamedSingles(d.JointRot); w.NamedDoubles(d.JointPosPrev); w.NamedSingles(d.JointRotPrev); }
            w.NamedBool(d.PlasticTouched);
            if (d.PlasticTouched) { w.NamedSingles(d.EdgeRest); w.NamedSingles(d.TetRest); }
        }
    }

    private void ReadSoft(StateReader r)
    {
        int version = r.ReadInt32();
        if (version < 1 || version > SoftSectionVersion) { throw r.Error("unsupported soft-body section version " + version.ToString(CultureInfo.InvariantCulture)); }
        int substeps = r.ReadInt32(), iterations = r.ReadInt32();
        if (substeps < 1 || substeps > 64 || iterations < 1 || iterations > 32) { throw r.Error("invalid soft-body options"); }
        var handles = HandleTable.Read(r, "soft bodies");
        var data = new SoftBodyData?[Math.Max(4, handles.HighWater)];
        long particles = 0;
        long[] broken = Array.Empty<long>();
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
            broken = Array.Empty<long>();
            try
            {
                def.Mesh = tets ? SoftBodyMesh.FromTetrahedra(verts, idx) : SoftBodyMesh.FromTriangles(verts, idx);
                // The mesh repairs a tetrahedron of negative volume by swapping two indices. A written snapshot never holds one, so a changed index list means the bytes are not ours and would not write back the same.
                if (tets && !def.Mesh.Tetrahedra.SequenceEqual(idx)) { throw r.Error("soft-body tetrahedron with negative volume"); }
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
                if (version >= 2 && r.ReadBool()) { def.Skin = ReadSkin(r, nv); }
                if (version >= 3)
                {
                    def.SurfaceCollision = r.ReadBool(); def.TearStrain = r.ReadSingle(); def.TearForce = r.ReadSingle();
                    def.PlasticYield = r.ReadSingle(); def.PlasticCreep = r.ReadSingle(); def.PlasticMax = r.ReadSingle();
                    int nb = r.ReadCount(8, "soft-body broken edges");
                    broken = new long[nb];
                    for (int k = 0; k < nb; k++)
                    {
                        long key = r.ReadInt64();
                        int ea = (int)(key >> 20), eb = (int)(key & 0xFFFFF);
                        if (key < 0 || ea >= eb || eb >= nv || (k > 0 && key <= broken[k - 1])) { throw r.Error("invalid soft-body broken edge"); }
                        broken[k] = key;
                    }
                }

                if (version >= 4 && r.ReadBool())
                {
                    var fab = new ClothParams
                    {
                        WarpAxis = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()), WeftAxis = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()),
                        WarpCompliance = r.ReadSingle(), WeftCompliance = r.ReadSingle(), ShearCompliance = r.ReadSingle(),
                        StrainLimit = r.ReadSingle(), DihedralBend = r.ReadBool(), Drag = r.ReadSingle(), LiftSlope = r.ReadSingle(), TetherStrain = r.ReadSingle(),
                    };
                    int na = r.ReadCount(4, "fabric tether anchors");
                    var anchors = new List<int>(na);
                    for (int k = 0; k < na; k++) { anchors.Add(r.ReadInt32()); }
                    fab.TetherAnchors = anchors;
                    def.Fabric = fab;
                }

                def.Validate();
            }
            catch (SoftBodyException ex) { throw r.Error("invalid soft body: " + ex.Message); }

            var d = new SoftBodyData { Def = def, BlockSize = SoftBlockSizeForTests, Broken = broken };
            try { d.Build(); }
            catch (SoftBodyException ex) { throw r.Error("invalid soft body: " + ex.Message); }
            particles += d.N;
            if (particles > MaxSoftParticles) { throw r.Error("too many soft-body particles"); }
            d.Asleep = r.ReadBool(); d.SleepTime = r.ReadSingle();
            if (!(d.SleepTime >= 0f) || !float.IsFinite(d.SleepTime)) { throw r.Error("invalid soft-body sleep time"); }
            if (version >= 3)
            {
                d.StepSpeed2 = r.ReadSingle();
                if (!(d.StepSpeed2 >= 0f) || !float.IsFinite(d.StepSpeed2)) { throw r.Error("invalid soft-body net speed"); }
            }
            r.ReadDoubles(d.Pos, true, "soft-body positions"); r.ReadDoubles(d.Prev, true, "soft-body previous positions"); r.ReadSingles(d.Vel, true, "soft-body velocities");
            r.ReadBytes(d.Pinned);
            if (d.IsSkinned) { ReadPose(r, d); }
            if (version >= 3 && r.ReadBool())
            {
                r.ReadSingles(d.EdgeRest, true, "soft-body rest lengths"); r.ReadSingles(d.TetRest, true, "soft-body rest volumes");
                for (int k = 0; k < d.EdgeRest.Length; k++) { if (!(d.EdgeRest[k] > 0f)) { throw r.Error("soft-body rest length is not positive"); } }
                for (int k = 0; k < d.TetRest.Length; k++) { if (!(d.TetRest[k] > 0f)) { throw r.Error("soft-body rest volume is not positive"); } }
                d.PlasticTouched = true;
                d.SyncSolverRests();
            }

            const double Limit = 4194304.0;
            for (int k = 0; k < 3 * d.N; k++)
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

    private static SoftSkin ReadSkin(StateReader r, int particles)
    {
        int nj = r.ReadCount(4 + 12, "skin joints");
        if (nj < 1 || nj > SoftSkin.MaxJoints) { throw r.Error("invalid skin joint count"); }
        var parents = new int[nj]; r.ReadInt32s(parents);
        var bind = new float[3 * nj]; r.ReadSingles(bind, true, "skin bind positions");
        var joints = new int[4 * particles]; r.ReadInt32s(joints);
        var weights = new float[4 * particles]; r.ReadSingles(weights, true, "skin weights");
        var max = new float[particles]; r.ReadSingles(max, true, "skin maximum distances");
        var backDist = new float[particles]; r.ReadSingles(backDist, true, "skin back-stop distances");
        var backRadius = new float[particles]; r.ReadSingles(backRadius, true, "skin back-stop radii");
        var normals = new float[3 * particles]; r.ReadSingles(normals, true, "skin normals");
        var skin = SoftSkin.FromStored(particles, parents, bind, joints, weights, max, backDist, backRadius, normals, out string? problem);
        if (problem != null) { throw r.Error(problem); }
        return skin;
    }

    private static void ReadPose(StateReader r, SoftBodyData d)
    {
        r.ReadDoubles(d.JointPos, true, "skin joint positions"); r.ReadSingles(d.JointRot, true, "skin joint rotations");
        r.ReadDoubles(d.JointPosPrev, true, "skin previous joint positions"); r.ReadSingles(d.JointRotPrev, true, "skin previous joint rotations");
        int nj = d.JointRot.Length / 4;
        for (int j = 0; j < nj; j++)
        {
            for (int pass = 0; pass < 2; pass++)
            {
                var rot = pass == 0 ? d.JointRot : d.JointRotPrev;
                var pos = pass == 0 ? d.JointPos : d.JointPosPrev;
                float l2 = rot[4 * j] * rot[4 * j] + rot[4 * j + 1] * rot[4 * j + 1] + rot[4 * j + 2] * rot[4 * j + 2] + rot[4 * j + 3] * rot[4 * j + 3];
                if (!(Math.Abs(l2 - 1f) <= 1e-3f)) { throw r.Error("skin joint rotation is not a unit quaternion"); }
                if (!(Math.Abs(pos[3 * j]) <= 4194304.0 && Math.Abs(pos[3 * j + 1]) <= 4194304.0 && Math.Abs(pos[3 * j + 2]) <= 4194304.0)) { throw r.Error("skin joint position out of range"); }
            }
        }
    }

    /// <summary>Emits one entry per soft body, in slot order: the particles as points and the edges as index pairs.</summary>
    public void AddDebugSoftBodies(Debug.DebugGeometry g)
    {
        for (int i = 0; i < SoftHandles.HighWater; i++)
        {
            if (!SoftHandles.IsLive(i)) { continue; }
            var d = SoftData[i]!;
            int first = -1;
            for (int p = 0; p < d.N; p++)
            {
                int k = g.AddPoint(new Vector3D(d.Pos[3 * p], d.Pos[3 * p + 1], d.Pos[3 * p + 2]));
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
            if (d.IsSkinned)
            {
                d.ComputeTargets(d.JointPos, d.JointRot, d.TgtCur, null);
                for (int p = 0; p < d.N; p++)
                {
                    int k = g.AddPoint(new Vector3D(d.TgtCur[3 * p], d.TgtCur[3 * p + 1], d.TgtCur[3 * p + 2]));
                    if (p == 0) { firstTarget = k; }
                }
            }

            int firstBroken = -1;
            for (int e = 0; e < d.Broken.Length; e++)
            {
                int k = g.AddIndex((int)(d.Broken[e] >> 20));
                g.AddIndex((int)(d.Broken[e] & 0xFFFFF));
                if (e == 0) { firstBroken = k; }
            }

            g.Add(new Debug.DebugSoftBody(i, SoftHandles.GenerationOf(i), d.Asleep, d.Radius, first, d.N, firstIndex, d.EdgeA.Length, firstTarget, firstBroken, d.Broken.Length));
        }
    }
}
