using System;
using System.Globalization;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>The pose of one skeleton joint for skinning: where the joint's bind position has gone and how the joint is turned, both in world coordinates.</summary>
/// <remarks>A point that sits at <c>x</c> in the body's rest frame (the mesh scaled by <see cref="SoftBodyDef.Scale"/>) and is bound to joint <c>j</c> alone is carried to <c>Position + Rotation * (x - bindPosition_j)</c>.</remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct SkinJointPose
{
    /// <summary>Creates a joint pose.</summary>
    /// <param name="position">Where the joint's bind position is now, world coordinates, finite and within ±2^21.</param>
    /// <param name="rotation">The joint's rotation relative to the bind pose; finite and non-zero (normalised on use).</param>
    public SkinJointPose(Vector3D position, QuaternionF rotation)
    {
        Position = position;
        Rotation = rotation;
    }

    /// <summary>Where the joint's bind position is now, world coordinates.</summary>
    public Vector3D Position { get; }

    /// <summary>The joint's rotation relative to the bind pose.</summary>
    public QuaternionF Rotation { get; }
}

/// <summary>
/// Skinning data for a soft body (middle tier): a skeleton, a linear-blend skin and per-particle limits that keep the body near the skinned surface. Immutable; build one with
/// <see cref="Create"/> and set it as <see cref="SoftBodyDef.Skin"/>.
/// </summary>
/// <remarks>
/// <para>The <b>skinned position</b> of a particle is the blend, by weight, of the positions its joints carry the particle's rest position to (linear blend skinning); the <b>skinned normal</b> is the blend of the
/// rest normal rotated by the same joints, normalised. The host sets the joints' poses before each step (<see cref="SoftBody.SetSkeletonPose"/> or <see cref="SoftBody.SetSkeletonTransforms"/>); during the step
/// the targets move linearly from where they were at the end of the last step to where the new pose puts them.</para>
/// <para>Each particle has a <b>maximum distance</b> D: it may be anywhere in the sphere of radius D around its skinned position and never outside it. A particle with D = 0 is kinematic: it is moved to its skinned
/// position and has infinite mass for the solver. The optional <b>back-stop</b> keeps a particle out of a sphere of radius R whose centre lies b behind the skinned position along the skinned normal (b = back-stop distance):
/// a cloth cannot be pushed into the body it covers. The back-stop is applied before the maximum distance, and a particle that the maximum distance would put back inside the back-stop is placed on the circle where the two spheres meet,
/// so both limits hold after every step to within float32 rounding. R may not exceed b, so the skinned position itself is always allowed.</para>
/// <para>The ideas follow the published descriptions of attachment and long-range constraints in position-based dynamics (Müller et al. 2007; Kim, Chentanez, Müller 2012, "Long Range Attachments") and skinned soft bodies in
/// game physics engines; the solver and snapshot details are in <c>docs/spec/softbody.md</c>.</para>
/// </remarks>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public sealed class SoftSkin
{
    /// <summary>Most joints a skeleton may have.</summary>
    public const int MaxJoints = 256;

    /// <summary>Joint influences per particle (linear blend skinning with four weights). A particle with fewer joints gives the rest a weight of 0.</summary>
    public const int InfluencesPerParticle = 4;

    /// <summary>Largest maximum distance, back-stop distance and back-stop radius accepted, metres.</summary>
    public const float MaxLimit = 1e6f;

    internal readonly int[] ParentArray;
    internal readonly float[] BindArray;        // 3 per joint, in the mesh frame (before the definition's scale)
    internal readonly int[] JointArray;         // 4 per particle
    internal readonly float[] WeightArray;      // 4 per particle, summing to 1
    internal readonly float[] MaxArray;         // per particle
    internal readonly float[] BackDistArray;    // per particle
    internal readonly float[] BackRadiusArray;  // per particle
    internal readonly float[] NormalArray;      // 3 per particle, unit or zero, in the mesh frame

    private SoftSkin(int[] parents, float[] bind, int[] joints, float[] weights, float[] max, float[] backDist, float[] backRadius, float[] normals)
    {
        ParentArray = parents; BindArray = bind; JointArray = joints; WeightArray = weights; MaxArray = max; BackDistArray = backDist; BackRadiusArray = backRadius; NormalArray = normals;
    }

    /// <summary>Number of joints.</summary>
    public int JointCount => ParentArray.Length;

    /// <summary>Number of particles the skin was made for (the mesh's vertex count).</summary>
    public int ParticleCount => MaxArray.Length;

    /// <summary>The parent of a joint, or -1 for the root (joint 0).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="joint"/> is out of range.</exception>
    public int GetParent(int joint)
    {
        if ((uint)joint >= (uint)ParentArray.Length) { throw new ArgumentOutOfRangeException(nameof(joint), joint, "joint index " + joint.ToString(CultureInfo.InvariantCulture) + " is outside 0 to " + (ParentArray.Length - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        return ParentArray[joint];
    }

    /// <summary>The maximum distance of a particle, metres (0 means kinematic).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="particle"/> is out of range.</exception>
    public float GetMaxDistance(int particle)
    {
        if ((uint)particle >= (uint)MaxArray.Length) { throw new ArgumentOutOfRangeException(nameof(particle), particle, "particle index " + particle.ToString(CultureInfo.InvariantCulture) + " is outside 0 to " + (MaxArray.Length - 1).ToString(CultureInfo.InvariantCulture) + "."); }
        return MaxArray[particle];
    }

    /// <summary>
    /// Builds and validates a skin for <paramref name="mesh"/>.
    /// </summary>
    /// <param name="mesh">The body's mesh; the skin has one entry per vertex.</param>
    /// <param name="parents">The skeleton: one parent index per joint, parents before children; joint 0 is the only root and has parent -1 (the order the ragdoll definition uses). 1 to <see cref="MaxJoints"/> joints.</param>
    /// <param name="bindPositions">Where each joint is in the bind pose, in mesh coordinates (they are scaled by <see cref="SoftBodyDef.Scale"/> with the vertices). Finite, within ±2^21.</param>
    /// <param name="jointIndices"><see cref="InfluencesPerParticle"/> joint numbers per particle (vertex order). Where a weight is 0 the number is ignored but must still be in range.</param>
    /// <param name="weights"><see cref="InfluencesPerParticle"/> weights per particle: finite, not negative, a positive sum. They are normalised to sum to 1.</param>
    /// <param name="maxDistance">Per particle, metres, 0 to <see cref="MaxLimit"/>; 0 makes the particle kinematic.</param>
    /// <param name="backstopDistance">Per particle, metres, 0 to <see cref="MaxLimit"/>: how far behind the skinned position the back-stop sphere is centred. Empty means 0 everywhere.</param>
    /// <param name="backstopRadius">Per particle, metres, 0 to <see cref="MaxLimit"/>: the radius of the back-stop sphere, at most the back-stop distance (so the skinned position is outside it); 0 turns the back-stop off for the particle. Empty means no back-stops.</param>
    /// <param name="normals">Per particle rest normals in mesh coordinates (non-zero ones are normalised). Empty takes the area-weighted vertex normals of the mesh's triangles (the triangle winding decides which side is the front); a tetrahedral mesh
    /// then has no normals, and a back-stop needs them.</param>
    /// <exception cref="SoftBodyException">Any array has the wrong length or a value is not finite, out of range or inconsistent; the message names the array and the entry.</exception>
    public static SoftSkin Create(SoftBodyMesh mesh, ReadOnlySpan<int> parents, ReadOnlySpan<Vector3F> bindPositions, ReadOnlySpan<int> jointIndices, ReadOnlySpan<float> weights,
        ReadOnlySpan<float> maxDistance, ReadOnlySpan<float> backstopDistance = default, ReadOnlySpan<float> backstopRadius = default, ReadOnlySpan<Vector3F> normals = default)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        int n = mesh.VertexCount, nj = parents.Length;
        if (nj < 1 || nj > MaxJoints) { throw Err(SoftBodyError.BadCount, "parents has " + I(nj) + " joints; expected 1 to " + I(MaxJoints) + ".", nameof(parents)); }
        if (bindPositions.Length != nj) { throw Err(SoftBodyError.BadCount, "bindPositions has " + I(bindPositions.Length) + " entries; the skeleton has " + I(nj) + " joints.", nameof(bindPositions)); }
        if (jointIndices.Length != InfluencesPerParticle * n) { throw Err(SoftBodyError.BadCount, "jointIndices has " + I(jointIndices.Length) + " entries; expected " + I(InfluencesPerParticle * n) + " (4 per particle, " + I(n) + " particles).", nameof(jointIndices)); }
        if (weights.Length != InfluencesPerParticle * n) { throw Err(SoftBodyError.BadCount, "weights has " + I(weights.Length) + " entries; expected " + I(InfluencesPerParticle * n) + " (4 per particle, " + I(n) + " particles).", nameof(weights)); }
        if (maxDistance.Length != n) { throw Err(SoftBodyError.BadCount, "maxDistance has " + I(maxDistance.Length) + " entries; expected one per particle (" + I(n) + ").", nameof(maxDistance)); }
        if (backstopDistance.Length != 0 && backstopDistance.Length != n) { throw Err(SoftBodyError.BadCount, "backstopDistance has " + I(backstopDistance.Length) + " entries; expected none or one per particle (" + I(n) + ").", nameof(backstopDistance)); }
        if (backstopRadius.Length != 0 && backstopRadius.Length != n) { throw Err(SoftBodyError.BadCount, "backstopRadius has " + I(backstopRadius.Length) + " entries; expected none or one per particle (" + I(n) + ").", nameof(backstopRadius)); }
        if (normals.Length != 0 && normals.Length != n) { throw Err(SoftBodyError.BadCount, "normals has " + I(normals.Length) + " entries; expected none or one per particle (" + I(n) + ").", nameof(normals)); }

        var par = parents.ToArray();
        var bind = new float[3 * nj];
        for (int j = 0; j < nj; j++)
        {
            if (j == 0 ? par[j] != -1 : (par[j] < 0 || par[j] >= j))
            {
                throw Err(SoftBodyError.BadIndex, "parents[" + I(j) + "] is " + I(par[j]) + "; joint 0 is the root (-1) and every other joint needs a parent with a lower index.", nameof(parents));
            }

            var b = bindPositions[j];
            Finite3(b.X, b.Y, b.Z, nameof(bindPositions), j);
            if (Math.Abs(b.X) > SoftBodyDef.MaxCoordinate || Math.Abs(b.Y) > SoftBodyDef.MaxCoordinate || Math.Abs(b.Z) > SoftBodyDef.MaxCoordinate)
            {
                throw Err(SoftBodyError.OutOfRange, "bindPositions[" + I(j) + "] is " + b + "; coordinates must be within ±2^21.", nameof(bindPositions));
            }

            bind[3 * j] = b.X; bind[3 * j + 1] = b.Y; bind[3 * j + 2] = b.Z;
        }

        var idx = jointIndices.ToArray();
        var w = new float[weights.Length];
        for (int i = 0; i < n; i++)
        {
            float sum = 0f;
            for (int k = 0; k < InfluencesPerParticle; k++)
            {
                int e = InfluencesPerParticle * i + k;
                if ((uint)idx[e] >= (uint)nj) { throw Err(SoftBodyError.BadIndex, "jointIndices[" + I(e) + "] (particle " + I(i) + ", influence " + I(k) + ") is " + I(idx[e]) + "; joints are numbered 0 to " + I(nj - 1) + ".", nameof(jointIndices)); }
                float wk = weights[e];
                if (!float.IsFinite(wk)) { throw Err(SoftBodyError.NonFinite, "weights[" + I(e) + "] (particle " + I(i) + ") is " + F(wk) + "; weights must be finite.", nameof(weights)); }
                if (wk < 0f) { throw Err(SoftBodyError.OutOfRange, "weights[" + I(e) + "] (particle " + I(i) + ") is " + F(wk) + "; weights must not be negative.", nameof(weights)); }
                w[e] = wk;
                sum += wk;
            }

            if (!(sum > 1e-12f) || !float.IsFinite(sum)) { throw Err(SoftBodyError.OutOfRange, "the weights of particle " + I(i) + " sum to " + F(sum) + "; they need a positive, finite sum.", nameof(weights)); }
            for (int k = 0; k < InfluencesPerParticle; k++) { w[InfluencesPerParticle * i + k] /= sum; }
        }

        var max = maxDistance.ToArray();
        var bd = new float[n];
        var br = new float[n];
        for (int i = 0; i < n; i++)
        {
            Limit(max[i], nameof(maxDistance), i);
            if (backstopDistance.Length != 0) { bd[i] = backstopDistance[i]; Limit(bd[i], nameof(backstopDistance), i); }
            if (backstopRadius.Length != 0) { br[i] = backstopRadius[i]; Limit(br[i], nameof(backstopRadius), i); }
        }

        var nrm = new float[3 * n];
        if (normals.Length != 0)
        {
            for (int i = 0; i < n; i++)
            {
                var v = normals[i];
                Finite3(v.X, v.Y, v.Z, nameof(normals), i);
                float l = float.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
                if (!float.IsFinite(l)) { throw Err(SoftBodyError.OutOfRange, "normals[" + I(i) + "] is " + v + "; its length overflows.", nameof(normals)); }
                if (l > 1e-12f) { nrm[3 * i] = v.X / l; nrm[3 * i + 1] = v.Y / l; nrm[3 * i + 2] = v.Z / l; }
            }
        }
        else { VertexNormals(mesh, nrm); }

        for (int i = 0; i < n; i++)
        {
            if (br[i] > 0f)
            {
                if (nrm[3 * i] == 0f && nrm[3 * i + 1] == 0f && nrm[3 * i + 2] == 0f) { throw Err(SoftBodyError.InvalidDefinition, "backstopRadius[" + I(i) + "] is " + F(br[i]) + " but particle " + I(i) + " has no normal (give normals, or use a triangle mesh).", nameof(backstopRadius)); }
                if (br[i] > bd[i])
                {
                    throw Err(SoftBodyError.OutOfRange, "backstopRadius[" + I(i) + "] is " + F(br[i]) + ", more than backstopDistance " + F(bd[i]) + "; the skinned position itself would lie inside the back-stop.", nameof(backstopRadius));
                }
            }
        }

        return new SoftSkin(par, bind, idx, w, max, bd, br, nrm);
    }

    /// <summary>Rebuilds a skin from stored arrays without normalising anything (so a restored skin is bit-equal to the one that was saved); validates what a corrupt snapshot could break.</summary>
    internal static SoftSkin FromStored(int particles, int[] parents, float[] bind, int[] joints, float[] weights, float[] max, float[] backDist, float[] backRadius, float[] normals, out string? problem)
    {
        problem = null;
        int nj = parents.Length;
        for (int j = 0; j < nj; j++)
        {
            if (j == 0 ? parents[j] != -1 : (parents[j] < 0 || parents[j] >= j)) { problem = "invalid skin parent"; return null!; }
        }

        for (int k = 0; k < bind.Length; k++) { if (!(Math.Abs(bind[k]) <= SoftBodyDef.MaxCoordinate)) { problem = "invalid skin bind position"; return null!; } }
        for (int i = 0; i < particles; i++)
        {
            float sum = 0f;
            for (int k = 0; k < InfluencesPerParticle; k++)
            {
                int e = InfluencesPerParticle * i + k;
                if ((uint)joints[e] >= (uint)nj) { problem = "invalid skin joint index"; return null!; }
                if (!(weights[e] >= 0f) || !float.IsFinite(weights[e])) { problem = "invalid skin weight"; return null!; }
                sum += weights[e];
            }

            if (!(Math.Abs(sum - 1f) <= 1e-3f)) { problem = "skin weights do not sum to one"; return null!; }
            if (!(max[i] >= 0f && max[i] <= MaxLimit) || !(backDist[i] >= 0f && backDist[i] <= MaxLimit) || !(backRadius[i] >= 0f && backRadius[i] <= MaxLimit)) { problem = "invalid skin limit"; return null!; }
            float nx = normals[3 * i], ny = normals[3 * i + 1], nz = normals[3 * i + 2];
            float l2 = nx * nx + ny * ny + nz * nz;
            if (!float.IsFinite(l2) || !(l2 == 0f || Math.Abs(l2 - 1f) <= 1e-3f)) { problem = "invalid skin normal"; return null!; }
            if (backRadius[i] > 0f && (l2 == 0f || backRadius[i] > backDist[i])) { problem = "invalid skin back-stop"; return null!; }
        }

        return new SoftSkin(parents, bind, joints, weights, max, backDist, backRadius, normals);
    }

    // Area-weighted vertex normals of the mesh's triangles, normalised; vertices of a tetrahedral mesh get none.
    private static void VertexNormals(SoftBodyMesh mesh, float[] nrm)
    {
        var v = mesh.Vertices;
        var t = mesh.Triangles;
        if (mesh.TetrahedronCount > 0) { return; }
        for (int e = 0; e < t.Length / 3; e++)
        {
            int a = t[3 * e], b = t[3 * e + 1], c = t[3 * e + 2];
            var ab = v[b] - v[a];
            var ac = v[c] - v[a];
            float nx = ab.Y * ac.Z - ab.Z * ac.Y, ny = ab.Z * ac.X - ab.X * ac.Z, nz = ab.X * ac.Y - ab.Y * ac.X;
            for (int k = 0; k < 3; k++)
            {
                int p = t[3 * e + k];
                nrm[3 * p] += nx; nrm[3 * p + 1] += ny; nrm[3 * p + 2] += nz;
            }
        }

        for (int i = 0; i < nrm.Length / 3; i++)
        {
            float l = float.Sqrt(nrm[3 * i] * nrm[3 * i] + nrm[3 * i + 1] * nrm[3 * i + 1] + nrm[3 * i + 2] * nrm[3 * i + 2]);
            if (l > 1e-12f && float.IsFinite(l)) { nrm[3 * i] /= l; nrm[3 * i + 1] /= l; nrm[3 * i + 2] /= l; }
            else { nrm[3 * i] = 0f; nrm[3 * i + 1] = 0f; nrm[3 * i + 2] = 0f; }
        }
    }

    private static void Limit(float v, string name, int i)
    {
        if (!float.IsFinite(v)) { throw Err(SoftBodyError.NonFinite, name + "[" + I(i) + "] is " + F(v) + "; it must be finite.", name); }
        if (v < 0f || v > MaxLimit) { throw Err(SoftBodyError.OutOfRange, name + "[" + I(i) + "] is " + F(v) + "; expected 0 to " + F(MaxLimit) + ".", name); }
    }

    private static void Finite3(float x, float y, float z, string name, int i)
    {
        if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z)) { throw Err(SoftBodyError.NonFinite, name + "[" + I(i) + "] is (" + F(x) + ", " + F(y) + ", " + F(z) + "); coordinates must be finite.", name); }
    }

    private static SoftBodyException Err(SoftBodyError e, string message, string subject) => new(e, "SoftSkin: " + message, subject);

    private static string I(int v) => v.ToString(CultureInfo.InvariantCulture);

    private static string F(float v) => v.ToString("R", CultureInfo.InvariantCulture);
}
