using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    // Skinned soft bodies: see docs/spec/softbody.md, "Skinning". The pose is set by the host between steps; each step moves the skinned targets linearly from the previous pose to the new one over the substeps.

    /// <summary>At the start of a step: skins the previous and the current pose, and widens the body's bounds to the targets so the collision candidates cover where the body is going.</summary>
    private static void PrepareSkin(SoftBodyData d)
    {
        d.SkinMoved = !d.PoseUnchanged();
        d.ComputeTargets(d.JointPos, d.JointRot, d.TgtCur, d.NrmCur);
        if (d.SkinMoved) { d.ComputeTargets(d.JointPosPrev, d.JointRotPrev, d.TgtPrev, null); }
        else { Array.Copy(d.TgtCur, d.TgtPrev, d.TgtCur.Length); }
        var t = d.TgtCur;
        for (int i = 0; i < d.N; i++)
        {
            double x = t[3 * i], y = t[3 * i + 1], z = t[3 * i + 2];
            if (x < d.MinX) { d.MinX = x; } if (x > d.MaxX) { d.MaxX = x; }
            if (y < d.MinY) { d.MinY = y; } if (y > d.MaxY) { d.MaxY = y; }
            if (z < d.MinZ) { d.MinZ = z; } if (z > d.MaxZ) { d.MaxZ = z; }
        }
    }

    /// <summary>After the prediction of a substep: kinematic skinned particles go to their skinned position at the end of the substep.</summary>
    private static void PredictSkin(SoftBodyData d, float t)
    {
        for (int i = 0; i < d.N; i++)
        {
            if (!d.IsSkinKinematic(i)) { continue; }
            d.TargetAt(i, t, out double x, out double y, out double z);
            int k = 3 * i;
            d.Pos[k] = x; d.Pos[k + 1] = y; d.Pos[k + 2] = z;
        }
    }

    /// <summary>
    /// The skin limits, applied last in a pass so that nothing after them can undo them. For each free particle, with <c>d</c> = position minus target (float32): the back-stop first (outside the sphere of radius R centred b behind the
    /// target along the skinned normal; pushed out along the line from that centre), then the maximum distance (|d| at most D; scaled back), and if the scaling put it inside the back-stop again, onto the circle where the two spheres meet,
    /// the nearest point to where it was.
    /// </summary>
    private static void SolveSkin(SoftBodyData d, float t)
    {
        var skin = d.Def.Skin!;
        var pos = d.Pos;
        for (int i = 0; i < d.N; i++)
        {
            if (d.InvMass[i] == 0f) { continue; }
            float maxD = skin.MaxArray[i];
            float radius = skin.BackRadiusArray[i];
            d.TargetAt(i, t, out double tx, out double ty, out double tz);
            int k = 3 * i;
            float dx = (float)(pos[k] - tx), dy = (float)(pos[k + 1] - ty), dz = (float)(pos[k + 2] - tz);
            bool changed = false;
            float nx = d.NrmCur[k], ny = d.NrmCur[k + 1], nz = d.NrmCur[k + 2];
            bool back = radius > 0f && (nx != 0f || ny != 0f || nz != 0f);
            float b = skin.BackDistArray[i];
            float cx = -nx * b, cy = -ny * b, cz = -nz * b;
            if (back)
            {
                float ex = dx - cx, ey = dy - cy, ez = dz - cz;
                float len2 = ex * ex + ey * ey + ez * ez;
                if (len2 < radius * radius)
                {
                    float len = float.Sqrt(len2);
                    if (len > 1e-9f) { float s = radius / len; dx = cx + ex * s; dy = cy + ey * s; dz = cz + ez * s; }
                    else { dx = cx + nx * radius; dy = cy + ny * radius; dz = cz + nz * radius; }
                    changed = true;
                }
            }

            float l2 = dx * dx + dy * dy + dz * dz;
            if (l2 > maxD * maxD)
            {
                float s = maxD / float.Sqrt(l2);
                dx *= s; dy *= s; dz *= s;
                changed = true;
                if (back)
                {
                    float ex = dx - cx, ey = dy - cy, ez = dz - cz;
                    if (ex * ex + ey * ey + ez * ez < radius * radius && b > 0f)
                    {
                        // Both spheres cut the point: the allowed region's boundary there is their intersection circle, at height a along the normal with radius rho.
                        float a = (radius * radius - maxD * maxD - b * b) / (2f * b);
                        float rho2 = maxD * maxD - a * a;
                        float rho = rho2 > 0f ? float.Sqrt(rho2) : 0f;
                        float h = dx * nx + dy * ny + dz * nz;
                        float ux = dx - h * nx, uy = dy - h * ny, uz = dz - h * nz;
                        float ul = float.Sqrt(ux * ux + uy * uy + uz * uz);
                        if (ul > 1e-9f) { ux /= ul; uy /= ul; uz /= ul; }
                        else
                        {
                            // On the axis: any direction perpendicular to the normal, from the axis least aligned with it.
                            float ax = Math.Abs(nx), ay = Math.Abs(ny), az = Math.Abs(nz);
                            float px = ax <= ay && ax <= az ? 1f : 0f, py = px == 0f && ay <= az ? 1f : 0f, pz = px == 0f && py == 0f ? 1f : 0f;
                            ux = ny * pz - nz * py; uy = nz * px - nx * pz; uz = nx * py - ny * px;
                            float ql = float.Sqrt(ux * ux + uy * uy + uz * uz);
                            ux /= ql; uy /= ql; uz /= ql;
                        }

                        dx = a * nx + rho * ux; dy = a * ny + rho * uy; dz = a * nz + rho * uz;
                    }
                }
            }

            if (changed) { pos[k] = tx + dx; pos[k + 1] = ty + dy; pos[k + 2] = tz + dz; }
        }
    }

    /// <summary>At the end of a step the current pose becomes the previous one.</summary>
    private static void EndSkinStep(SoftBodyData d)
    {
        if (!d.SkinMoved) { return; }
        d.JointPos.CopyTo(d.JointPosPrev, 0);
        d.JointRot.CopyTo(d.JointRotPrev, 0);
    }
}
