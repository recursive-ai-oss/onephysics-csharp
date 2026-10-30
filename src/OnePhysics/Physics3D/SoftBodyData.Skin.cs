using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

// Skinning arithmetic of a soft body (docs/spec/softbody.md, "Skinning"). Everything here is float32/double IEEE arithmetic in a fixed order, so every platform and vector width gives the same bits.
internal sealed partial class SoftBodyData
{
    /// <summary>Sets the effective inverse mass of a particle: zero when it is pinned or, in a skinned body, kinematic (maximum distance 0).</summary>
    public void RefreshInvMass(int i)
    {
        bool kinematic = Pinned[i] != 0 || (Def.Skin != null && Def.Skin.MaxArray[i] == 0f);
        InvMass[i] = kinematic ? 0f : InvMass0[i];
    }

    /// <summary>True when the particle follows its skinned position exactly: maximum distance 0 and not pinned.</summary>
    public bool IsSkinKinematic(int i) => Def.Skin != null && Def.Skin.MaxArray[i] == 0f && Pinned[i] == 0;

    /// <summary>The bind pose placed at <paramref name="at"/> with orientation <paramref name="q"/>: joint j at <c>at + q * bind_j</c>, turned by <c>q</c>. Sets both the current and the previous pose.</summary>
    public void SetBindPose(Vector3D at, QuaternionF q)
    {
        int nj = JointRot.Length / 4;
        for (int j = 0; j < nj; j++)
        {
            var r = q.Rotate(new Vector3F(Bind[3 * j], Bind[3 * j + 1], Bind[3 * j + 2]));
            JointPos[3 * j] = at.X + r.X; JointPos[3 * j + 1] = at.Y + r.Y; JointPos[3 * j + 2] = at.Z + r.Z;
            JointRot[4 * j] = q.X; JointRot[4 * j + 1] = q.Y; JointRot[4 * j + 2] = q.Z; JointRot[4 * j + 3] = q.W;
        }

        JointPos.CopyTo(JointPosPrev, 0);
        JointRot.CopyTo(JointRotPrev, 0);
    }

    /// <summary>
    /// Linear blend skinning. For particle i with influences (j_k, w_k), k = 0..3 in order, skipping w_k = 0:
    /// <c>target = sum_k (double)w_k * (jointPos[j_k] + (double)(rot[j_k] * (rest_i - bind[j_k])))</c> accumulated in double in that order, the rotation and the difference in float32;
    /// <c>normal = normalize(sum_k w_k * (rot[j_k] * restNormal_i))</c> in float32, zero when the sum is too short or the rest normal is zero.
    /// </summary>
    public void ComputeTargets(double[] jp, float[] jr, double[] target, float[]? normals) => ComputeTargets(jp, jr, target, normals, 0, N);

    /// <summary>The same for particles <paramref name="from"/> up to (not including) <paramref name="to"/>; entries are written at their particle numbers.</summary>
    public void ComputeTargets(double[] jp, float[] jr, double[] target, float[]? normals, int from, int to)
    {
        var skin = Def.Skin!;
        var rest = Rest;
        for (int i = from; i < to; i++)
        {
            double ax = 0.0, ay = 0.0, az = 0.0;
            float sx = 0f, sy = 0f, sz = 0f;
            float rnx = skin.NormalArray[3 * i], rny = skin.NormalArray[3 * i + 1], rnz = skin.NormalArray[3 * i + 2];
            for (int k = 0; k < SoftSkin.InfluencesPerParticle; k++)
            {
                int e = SoftSkin.InfluencesPerParticle * i + k;
                float w = skin.WeightArray[e];
                if (w == 0f) { continue; }
                int j = skin.JointArray[e];
                var q = new QuaternionF(jr[4 * j], jr[4 * j + 1], jr[4 * j + 2], jr[4 * j + 3]);
                var r = q.Rotate(new Vector3F(rest[3 * i] - Bind[3 * j], rest[3 * i + 1] - Bind[3 * j + 1], rest[3 * i + 2] - Bind[3 * j + 2]));
                double dw = w;
                ax += dw * (jp[3 * j] + (double)r.X);
                ay += dw * (jp[3 * j + 1] + (double)r.Y);
                az += dw * (jp[3 * j + 2] + (double)r.Z);
                if (normals != null)
                {
                    var m = q.Rotate(new Vector3F(rnx, rny, rnz));
                    sx += w * m.X; sy += w * m.Y; sz += w * m.Z;
                }
            }

            target[3 * i] = ax; target[3 * i + 1] = ay; target[3 * i + 2] = az;
            if (normals != null)
            {
                float l = float.Sqrt(sx * sx + sy * sy + sz * sz);
                if (l > 1e-6f && float.IsFinite(l)) { normals[3 * i] = sx / l; normals[3 * i + 1] = sy / l; normals[3 * i + 2] = sz / l; }
                else { normals[3 * i] = 0f; normals[3 * i + 1] = 0f; normals[3 * i + 2] = 0f; }
            }
        }
    }

    /// <summary>Whether the current joint pose is bit-equal to the previous one.</summary>
    public bool PoseUnchanged()
    {
        var a = JointPos; var b = JointPosPrev;
        for (int i = 0; i < a.Length; i++) { if (a[i] != b[i]) { return false; } }
        var c = JointRot; var d = JointRotPrev;
        for (int i = 0; i < c.Length; i++) { if (c[i] != d[i]) { return false; } }
        return true;
    }

    /// <summary>The skinned position of particle i a fraction t of the way from the previous pose to the current one (t = 1 is exactly the current).</summary>
    public void TargetAt(int i, float t, out double x, out double y, out double z)
    {
        int k = 3 * i;
        if (t >= 1f) { x = TgtCur[k]; y = TgtCur[k + 1]; z = TgtCur[k + 2]; return; }
        double dt = t;
        x = TgtPrev[k] + (TgtCur[k] - TgtPrev[k]) * dt;
        y = TgtPrev[k + 1] + (TgtCur[k + 1] - TgtPrev[k + 1]) * dt;
        z = TgtPrev[k + 2] + (TgtCur[k + 2] - TgtPrev[k + 2]) * dt;
    }
}
