using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.Tools.Tests;

/// <summary>A generated person: a bone hierarchy with Mixamo-style names in a standing pose (arms hanging), and a skinned mesh of capsule-like limbs, a box torso and pelvis, a sphere head, box hands and feet.</summary>
public sealed class TestSkeleton
{
    public string[] Names = Array.Empty<string>();
    public int[] Parents = Array.Empty<int>();
    public Vector3F[] Positions = Array.Empty<Vector3F>();
    public Vector3F[] Points = Array.Empty<Vector3F>();
    public int[] PointBone = Array.Empty<int>();     // the dominant bone of each point
    public int[] BoneIndices = Array.Empty<int>();   // two influences per point
    public float[] BoneWeights = Array.Empty<float>();

    public SkinnedPoints3D Skin => new(Points, BoneIndices, BoneWeights, 2);

    public Vector2F[] Positions2D()
    {
        var p = new Vector2F[Positions.Length];
        for (int i = 0; i < p.Length; i++) { p[i] = new Vector2F(Positions[i].Z, Positions[i].Y); }
        return p;
    }

    public SkinnedPoints2D Skin2D()
    {
        var p = new Vector2F[Points.Length];
        for (int i = 0; i < p.Length; i++) { p[i] = new Vector2F(Points[i].Z, Points[i].Y); }
        return new SkinnedPoints2D(p, BoneIndices, BoneWeights, 2);
    }

    /// <summary>A standing person of the given height (1.8 m is the reference), left side at +x. <paramref name="rightFirst"/> lists the right limbs before the left ones.</summary>
    public static TestSkeleton Humanoid(float height = 1.8f, bool rightFirst = false, bool withMesh = true, string prefix = "mixamorig:")
    {
        float s = height / 1.8f;
        var names = new List<string>();
        var parents = new List<int>();
        var pos = new List<Vector3F>();
        int Bone(string name, int parent, float x, float y, float z) { names.Add(prefix + name); parents.Add(parent); pos.Add(new Vector3F(x * s, y * s, z * s)); return names.Count - 1; }

        int hips = Bone("Hips", -1, 0, 0.98f, 0);
        int sp0 = Bone("Spine", hips, 0, 1.06f, 0), sp1 = Bone("Spine1", sp0, 0, 1.17f, 0), sp2 = Bone("Spine2", sp1, 0, 1.285f, 0);
        int neck = Bone("Neck", sp2, 0, 1.40f, 0), head = Bone("Head", neck, 0, 1.44f, 0), headTop = Bone("HeadTop_End", head, 0, 1.66f, 0);
        var side = rightFirst ? new[] { -1f, 1f } : new[] { 1f, -1f };
        var sideName = rightFirst ? new[] { "Right", "Left" } : new[] { "Left", "Right" };
        var bones = new Dictionary<string, int>();
        for (int k = 0; k < 2; k++)
        {
            float sx = side[k]; string n = sideName[k];
            int sh = Bone(n + "Shoulder", sp2, sx * 0.06f, 1.40f, 0);
            int arm = Bone(n + "Arm", sh, sx * 0.25f, 1.38f, 0), fore = Bone(n + "ForeArm", arm, sx * 0.25f, 1.07f, 0), hand = Bone(n + "Hand", fore, sx * 0.25f, 0.80f, 0);
            int f1 = Bone(n + "HandIndex1", hand, sx * 0.25f, 0.72f, 0.02f); Bone(n + "HandIndex4", f1, sx * 0.25f, 0.66f, 0.02f);
            bones[n + "Arm"] = arm; bones[n + "ForeArm"] = fore; bones[n + "Hand"] = hand;
            int up = Bone(n + "UpLeg", hips, sx * 0.10f, 0.92f, 0), leg = Bone(n + "Leg", up, sx * 0.10f, 0.50f, 0), foot = Bone(n + "Foot", leg, sx * 0.10f, 0.08f, 0);
            int toe = Bone(n + "ToeBase", foot, sx * 0.10f, 0.03f, 0.12f); Bone(n + "Toe_End", toe, sx * 0.10f, 0.02f, 0.20f);
            bones[n + "UpLeg"] = up; bones[n + "Leg"] = leg; bones[n + "Foot"] = foot; bones[n + "ToeBase"] = toe;
        }

        var t = new TestSkeleton { Names = names.ToArray(), Parents = parents.ToArray(), Positions = pos.ToArray() };
        if (!withMesh) { return t; }

        var pts = new List<Vector3F>(); var bone = new List<int>(); var ind = new List<int>(); var wts = new List<float>();
        void Pt(float x, float y, float z, int b, int other = -1, float w = 0f)
        {
            pts.Add(new Vector3F(x * s, y * s, z * s)); bone.Add(b);
            ind.Add(b); ind.Add(other < 0 ? b : other); wts.Add(1f - w); wts.Add(w);
        }

        void Box(float x0, float x1, float y0, float y1, float z0, float z1, int b)
        {
            for (int i = 0; i < 8; i++) { Pt((i & 1) == 0 ? x0 : x1, (i & 2) == 0 ? y0 : y1, (i & 4) == 0 ? z0 : z1, b); }
        }

        void Tube(float x, float yTop, float yBottom, float r, int b, int nextBone)
        {
            for (int ring = 0; ring < 5; ring++)
            {
                float y = yTop + (yBottom - yTop) * ring / 4f;
                for (int a = 0; a < 8; a++)
                {
                    double ang = Math.PI * 2 * a / 8;
                    float w = ring == 4 ? 0.3f : 0f;
                    Pt(x + r * (float)Math64.Cos(ang), y, r * (float)Math64.Sin(ang), b, nextBone, w);
                }
            }
        }

        Box(-0.15f, 0.15f, 0.90f, 1.06f, -0.09f, 0.09f, hips);
        Box(-0.17f, 0.17f, 1.06f, 1.17f, -0.10f, 0.10f, sp0);
        Box(-0.17f, 0.17f, 1.17f, 1.285f, -0.10f, 0.10f, sp1);
        Box(-0.17f, 0.17f, 1.285f, 1.40f, -0.10f, 0.10f, sp2);
        // neck and head
        for (int a = 0; a < 8; a++) { double ang = Math.PI * 2 * a / 8; Pt(0.05f * (float)Math64.Cos(ang), 1.40f, 0.05f * (float)Math64.Sin(ang), neck); Pt(0.05f * (float)Math64.Cos(ang), 1.44f, 0.05f * (float)Math64.Sin(ang), neck); }
        float hc = 1.55f, hr = 0.11f;
        for (int i = -1; i <= 1; i++) { for (int j = -1; j <= 1; j++) { for (int k = -1; k <= 1; k++) { if (i == 0 && j == 0 && k == 0) { continue; } float l = (float)Math.Sqrt(i * i + j * j + k * k); Pt(hr * i / l, hc + hr * j / l, hr * k / l, head); } } }
        for (int k = 0; k < 2; k++)
        {
            float sx = side[k]; string n = sideName[k];
            Tube(sx * 0.25f, 1.38f, 1.07f, 0.045f, bones[n + "Arm"], bones[n + "ForeArm"]);
            Tube(sx * 0.25f, 1.07f, 0.80f, 0.04f, bones[n + "ForeArm"], bones[n + "Hand"]);
            Box(sx * 0.25f - 0.03f, sx * 0.25f + 0.03f, 0.70f, 0.80f, -0.015f, 0.015f, bones[n + "Hand"]);
            Tube(sx * 0.10f, 0.92f, 0.50f, 0.08f, bones[n + "UpLeg"], bones[n + "Leg"]);
            Tube(sx * 0.10f, 0.50f, 0.08f, 0.06f, bones[n + "Leg"], bones[n + "Foot"]);
            Box(sx * 0.10f - 0.055f, sx * 0.10f + 0.055f, 0f, 0.08f, -0.09f, 0.12f, bones[n + "Foot"]);
            Box(sx * 0.10f - 0.05f, sx * 0.10f + 0.05f, 0f, 0.04f, 0.12f, 0.20f, bones[n + "ToeBase"]);
        }

        t.Points = pts.ToArray(); t.PointBone = bone.ToArray(); t.BoneIndices = ind.ToArray(); t.BoneWeights = wts.ToArray();
        return t;
    }
}
