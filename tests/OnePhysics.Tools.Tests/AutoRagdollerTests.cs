using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath;
using Xunit;
using P2 = RecursiveAI.OnePhysics.Physics2D;
using P3 = RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Tools.Tests;

public class AutoRagdollerTests
{
    private static AutoRagdoll3D Make(TestSkeleton sk, AutoRagdollOptions? o = null, bool mesh = true) => AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions, mesh ? sk.Skin : null, o);

    [Fact]
    public void AHumanoidGetsThirteenPartsWithTableMasses()
    {
        var sk = TestSkeleton.Humanoid();
        var r = Make(sk);
        Assert.True(r.IsHumanoid);
        var parts = r.Definition.Parts;
        Assert.Equal(13, parts.Length);
        Assert.Equal(-1, parts[0].Parent);
        for (int i = 1; i < parts.Length; i++) { Assert.InRange(parts[i].Parent, 0, i - 1); }
        Assert.Equal(75.0, parts.Sum(p => (double)p.Mass), 3);
        // Winter's table, with the hand folded into the forearm: thigh 10 %, shank 4.65 %, foot 1.45 %, forearm and hand 2.2 %, upper arm 2.8 %, trunk 35.5 %, pelvis 14.2 %, head 8.1 %
        Assert.Equal(7.5, parts[7].Mass, 0.005); Assert.Equal(75 * 0.0465, parts[8].Mass, 0.005); Assert.Equal(75 * 0.0145, parts[9].Mass, 0.005);
        Assert.Equal(75 * 0.022, parts[4].Mass, 0.005); Assert.Equal(75 * 0.028, parts[3].Mass, 0.005);
        Assert.Equal(75 * 0.355, parts[1].Mass, 0.005); Assert.Equal(75 * 0.142, parts[0].Mass, 0.005); Assert.Equal(75 * 0.081, parts[2].Mass, 0.005);
        // The left leg (+x) comes first, as in the engine's own humanoid layout.
        Assert.True(parts[7].Position.X > 0 && parts[10].Position.X < 0);
        Assert.True(parts[3].Position.X > 0 && parts[5].Position.X < 0);
        var b = r.Biped!;
        Assert.Equal((0, 1, 2, 7, 8, 9, 10, 11, 12), (b.Pelvis, b.Torso, b.Head, b.LeftThigh, b.LeftShin, b.LeftFoot, b.RightThigh, b.RightShin, b.RightFoot));
        Assert.Equal(sk.Names.Length, r.PartOfBone.Length);
        Assert.Equal(0, r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:Hips")]);
        Assert.Equal(r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:LeftForeArm")], r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:LeftHandIndex4")]);
        Assert.Equal(r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:LeftFoot")], r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:LeftToe_End")]);
        Assert.Equal(r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:Spine")], r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:Spine2")]);
        Assert.Equal(r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:Spine2")], r.PartOfBone[Array.IndexOf(sk.Names, "mixamorig:LeftShoulder")]);
        Assert.Equal(BoneRole.Merge, r.Roles[Array.IndexOf(sk.Names, "mixamorig:LeftHandIndex1")]);
        Assert.Equal(BoneRole.Thigh, r.Roles[Array.IndexOf(sk.Names, "mixamorig:RightUpLeg")]);
        Assert.Equal(BoneRole.Shin, r.Roles[Array.IndexOf(sk.Names, "mixamorig:RightLeg")]);

        // the engine accepts it
        var w = new P3.PhysicsWorld3D();
        var rag = w.AddRagdoll(r.Definition);
        Assert.Equal(13, rag.PartCount);
    }

    private static bool Inside(ConvexHull3D hull, Vector3F p, float tol)
    {
        foreach (var f in hull.Faces) { if (f.Normal.X * p.X + f.Normal.Y * p.Y + f.Normal.Z * p.Z - f.Offset > tol) { return false; } }
        return true;
    }

    [Fact]
    public void ShapesLieInsideTheirPartOfTheMeshHull()
    {
        var sk = TestSkeleton.Humanoid();
        var r = Make(sk);
        var parts = r.Definition.Parts;
        for (int part = 0; part < parts.Length; part++)
        {
            var pts = new List<Vector3F>();
            for (int i = 0; i < sk.Points.Length; i++) { if (r.PartOfBone[sk.PointBone[i]] == part) { pts.Add(sk.Points[i]); } }
            Assert.True(pts.Count >= 8, "part " + part + " has " + pts.Count + " points");
            var hull = ConvexHull3D.Build(pts.ToArray());
            // convention check: the points themselves are on or inside, the hull's centre is well inside
            foreach (var v in pts) { Assert.True(Inside(hull, v, 1e-4f)); }
            var d = parts[part];
            var q = d.Rotation;
            Vector3F At(float x, float y, float z) => d.Position + q.Rotate(new Vector3F(x, y, z));
            var samples = new List<Vector3F>();
            switch (d.Shape)
            {
                case P3.RagdollShapeKind.Sphere:
                {
                    float rr = d.ShapeHalfSize.X;
                    foreach (var dir in new[] { new Vector3F(1, 0, 0), new Vector3F(-1, 0, 0), new Vector3F(0, 1, 0), new Vector3F(0, -1, 0), new Vector3F(0, 0, 1), new Vector3F(0, 0, -1) }) { samples.Add(At(dir.X * rr, dir.Y * rr, dir.Z * rr)); }
                    break;
                }

                case P3.RagdollShapeKind.Box:
                    for (int c = 0; c < 8; c++) { samples.Add(At((c & 1) == 0 ? -d.ShapeHalfSize.X : d.ShapeHalfSize.X, (c & 2) == 0 ? -d.ShapeHalfSize.Y : d.ShapeHalfSize.Y, (c & 4) == 0 ? -d.ShapeHalfSize.Z : d.ShapeHalfSize.Z)); }
                    break;
                default:
                {
                    float h = d.ShapeHalfSize.X, rr = d.ShapeHalfSize.Y;
                    samples.Add(At(0, h + rr, 0)); samples.Add(At(0, -h - rr, 0));
                    foreach (float yy in new[] { h, -h }) { samples.Add(At(rr, yy, 0)); samples.Add(At(-rr, yy, 0)); samples.Add(At(0, yy, rr)); samples.Add(At(0, yy, -rr)); }
                    break;
                }
            }

            // A forearm carries its hand, narrower than the arm; a foot's box has a corner above the lower toes: those two may stick out by a few centimetres, every other part by a centimetre.
            foreach (var s in samples) { Assert.True(Inside(hull, s, part == 4 || part == 6 ? 0.03f : part == 9 || part == 12 ? 0.045f : 0.012f), "part " + part + " (" + d.Name + ") sticks out of its mesh at " + s); }

            // and it is not smaller than the points it covers: the points' bounding box is at most 15 % larger than the shape's along the shape's own long axis
            float lenShape = d.Shape == P3.RagdollShapeKind.Capsule ? 2 * (d.ShapeHalfSize.X + d.ShapeHalfSize.Y) : d.Shape == P3.RagdollShapeKind.Sphere ? 2 * d.ShapeHalfSize.X : 2 * d.ShapeHalfSize.Y;
            float lo = float.MaxValue, hi = float.MinValue;
            foreach (var v in pts) { float y = v.Y; lo = Math.Min(lo, y); hi = Math.Max(hi, y); }
            if (d.Shape != P3.RagdollShapeKind.Sphere) { Assert.InRange(lenShape / (hi - lo), 0.85, 1.15); }
        }
    }

    [Fact]
    public void LimitsAndGainsFollowTheRoleTable()
    {
        var parts = Make(TestSkeleton.Humanoid()).Definition.Parts;
        Assert.Equal(0.08f, parts[8].SwingZHalfAngle); Assert.Equal(2.3f, parts[8].SwingYHalfAngle); Assert.Equal(0.1f, parts[8].UpperTwist);   // knee
        Assert.Equal(1.2f, parts[7].SwingYHalfAngle); Assert.Equal(1.0f, parts[7].SwingZHalfAngle);                                                   // hip
        Assert.Equal(2.4f, parts[4].SwingYHalfAngle); Assert.Equal(0.1f, parts[4].SwingZHalfAngle);                                                   // elbow
        Assert.Equal(1.6f, parts[3].SwingYHalfAngle);                                                                                                 // shoulder
        Assert.Equal(0.6f, parts[2].SwingYHalfAngle); Assert.Equal(-0.4f, parts[1].LowerTwist);
        Assert.All(parts, p => Assert.Equal(-p.UpperTwist, p.LowerTwist));
        // gains scale with mass over 75 kg and height over the engine humanoid's 1.66 m
        var heavy = Make(TestSkeleton.Humanoid(), new AutoRagdollOptions { TotalMass = 150f }).Definition.Parts;
        Assert.Equal(2 * parts[7].DriveStiffness, heavy[7].DriveStiffness, 1);
        Assert.Equal(2 * parts[7].MaxTorque, heavy[7].MaxTorque, 1);
        var floor = Make(TestSkeleton.Humanoid(), new AutoRagdollOptions { MinSwingHalfAngle = 0.6f }).Definition.Parts;
        Assert.All(floor, p => Assert.True(p.SwingYHalfAngle >= 0.6f && p.SwingZHalfAngle >= 0.6f));
        Assert.Equal(2.3f, floor[8].SwingYHalfAngle);
    }

    [Fact]
    public void HandsAndTorsoSplitChangeThePartCount()
    {
        var sk = TestSkeleton.Humanoid();
        var hands = Make(sk, new AutoRagdollOptions { IncludeHands = true });
        Assert.Equal(15, hands.Definition.Parts.Length);
        Assert.Equal(75.0, hands.Definition.Parts.Sum(p => (double)p.Mass), 3);
        Assert.Equal(75 * 0.006, hands.Definition.Parts.First(p => p.Name!.EndsWith("LeftHand", StringComparison.Ordinal)).Mass, 0.005);
        Assert.NotNull(hands.Biped);
        var split = Make(sk, new AutoRagdollOptions { SplitTorso = true });
        Assert.Equal(14, split.Definition.Parts.Length);
        Assert.Equal(75 * 0.139, split.Definition.Parts[1].Mass, 0.005);
        Assert.Equal(75 * 0.216, split.Definition.Parts[2].Mass, 0.005);
        Assert.Null(split.Biped);   // the head hangs from the chest, not from a child of the pelvis
        var both = Make(sk, new AutoRagdollOptions { SplitTorso = true, IncludeHands = true });
        Assert.Equal(16, both.Definition.Parts.Length);
        foreach (var d in new[] { hands, split, both }) { new P3.PhysicsWorld3D().AddRagdoll(d.Definition); }
    }

    [Fact]
    public void WithoutSkinProportionsAreUsed()
    {
        var sk = TestSkeleton.Humanoid();
        var r = Make(sk, null, mesh: false);
        Assert.True(r.IsHumanoid);
        Assert.Equal(13, r.Definition.Parts.Length);
        Assert.Equal(75.0, r.Definition.Parts.Sum(p => (double)p.Mass), 3);
        // the thigh capsule spans the hip-to-knee segment (0.42 m): half length + radius = 0.21
        var thigh = r.Definition.Parts[7];
        Assert.Equal(0.21f, thigh.ShapeHalfSize.X + thigh.ShapeHalfSize.Y, 3);
        new P3.PhysicsWorld3D().AddRagdoll(r.Definition);
        var tall = Make(TestSkeleton.Humanoid(2.4f), null, mesh: false);
        Assert.InRange(tall.Definition.Parts[7].ShapeHalfSize.Y / thigh.ShapeHalfSize.Y, 1.3, 1.4);
    }

    [Fact]
    public void BoneNamesFromOtherToolsAreRecognised()
    {
        string[] Rename(string[] names, Func<string, string> f) => names.Select(f).ToArray();
        var sk = TestSkeleton.Humanoid(withMesh: false);
        // Blender "name.L", Unity "Left UpperLeg", Biped "Bip01 L Thigh"
        string Blender(string n)
        {
            n = n.Replace("mixamorig:", "");
            n = n.Replace("LeftUpLeg", "thigh.L").Replace("RightUpLeg", "thigh.R").Replace("LeftLeg", "shin.L").Replace("RightLeg", "shin.R").Replace("LeftFoot", "foot.L").Replace("RightFoot", "foot.R").Replace("LeftToeBase", "toe.L").Replace("RightToeBase", "toe.R").Replace("LeftToe_End", "toe_end.L").Replace("RightToe_End", "toe_end.R");
            n = n.Replace("LeftForeArm", "forearm.L").Replace("RightForeArm", "forearm.R").Replace("LeftArm", "upper_arm.L").Replace("RightArm", "upper_arm.R").Replace("LeftHandIndex1", "f_index.01.L").Replace("RightHandIndex1", "f_index.01.R").Replace("LeftHandIndex4", "f_index.04.L").Replace("RightHandIndex4", "f_index.04.R").Replace("LeftHand", "hand.L").Replace("RightHand", "hand.R").Replace("LeftShoulder", "shoulder.L").Replace("RightShoulder", "shoulder.R");
            n = n.Replace("Hips", "pelvis").Replace("Spine2", "chest").Replace("Spine1", "spine.001").Replace("Spine", "spine").Replace("HeadTop_End", "head_end").Replace("Head", "head").Replace("Neck", "neck");
            return n;
        }

        string Unity(string n) => n.Replace("mixamorig:", "").Replace("LeftUpLeg", "Left UpperLeg").Replace("RightUpLeg", "Right UpperLeg").Replace("LeftLeg", "Left LowerLeg").Replace("RightLeg", "Right LowerLeg").Replace("LeftForeArm", "Left LowerArm").Replace("RightForeArm", "Right LowerArm").Replace("LeftArm", "Left UpperArm").Replace("RightArm", "Right UpperArm").Replace("LeftToeBase", "Left Toes").Replace("RightToeBase", "Right Toes").Replace("Toe_End", "Toes End").Replace("HandIndex", "Index Proximal ").Replace("Spine2", "UpperChest").Replace("Spine1", "Chest");
        string Biped(string n) => "Bip01 " + n.Replace("mixamorig:", "").Replace("Left", "L ").Replace("Right", "R ").Replace("UpLeg", "Thigh").Replace("ForeArm", "Forearm").Replace("ToeBase", "Toe0").Replace("Hips", "Pelvis").Replace("Toe_End", "Toe0Nub").Replace("HeadTop_End", "HeadNub").Replace("HandIndex1", "Finger1").Replace("HandIndex4", "Finger1Nub");
        foreach (var (label, f) in new (string, Func<string, string>)[] { ("blender", Blender), ("unity", Unity), ("biped", Biped) })
        {
            var names = Rename(sk.Names, f);
            var r = AutoRagdoller.Create(names, sk.Parents, sk.Positions);
            Assert.True(r.IsHumanoid, label + ": " + string.Join(",", names));
            Assert.Equal(13, r.Definition.Parts.Length);
        }
    }

    [Fact]
    public void RoleOverridesReplaceTheNameHeuristics()
    {
        var sk = TestSkeleton.Humanoid(withMesh: false);
        var names = sk.Names.Select((_, i) => "b" + i).ToArray();
        Assert.False(AutoRagdoller.Create(names, sk.Parents, sk.Positions).IsHumanoid);
        var map = new Dictionary<string, BoneRole>();
        for (int i = 0; i < sk.Names.Length; i++)
        {
            string n = sk.Names[i].Replace("mixamorig:", "");
            BoneRole role = n switch
            {
                "Hips" => BoneRole.Pelvis, "Spine" or "Spine1" => BoneRole.Spine, "Spine2" => BoneRole.Chest, "Neck" => BoneRole.Neck, "Head" => BoneRole.Head,
                "LeftArm" or "RightArm" => BoneRole.UpperArm, "LeftForeArm" or "RightForeArm" => BoneRole.Forearm, "LeftHand" or "RightHand" => BoneRole.Hand,
                "LeftUpLeg" or "RightUpLeg" => BoneRole.Thigh, "LeftLeg" or "RightLeg" => BoneRole.Shin, "LeftFoot" or "RightFoot" => BoneRole.Foot, "LeftToeBase" or "RightToeBase" => BoneRole.Toe,
                _ => BoneRole.Merge,
            };
            map[names[i]] = role;
        }

        var r = AutoRagdoller.Create(names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { RoleOverrides = map });
        Assert.True(r.IsHumanoid);
        Assert.Equal(13, r.Definition.Parts.Length);
    }

    [Fact]
    public void AGenericSkeletonGetsACapsulePerBone()
    {
        // a tail of five bones hanging from a body, none named like a person
        var names = new List<string> { "hub", "body" };
        var parents = new List<int> { -1, 0 };
        var pos = new List<Vector3F> { new(0, 0, 0), new(0, 0.5f, 0) };
        for (int i = 0; i < 5; i++) { names.Add("tail" + i); parents.Add(i == 0 ? 1 : 1 + i); pos.Add(new Vector3F(0, 0.5f, -0.2f * (i + 1))); }
        names.Add("tailTip"); parents.Add(6); pos.Add(new Vector3F(0, 0.5f, -1.2f));
        var r = AutoRagdoller.Create(names.ToArray(), parents.ToArray(), pos.ToArray(), null, new AutoRagdollOptions { TotalMass = 10f });
        Assert.False(r.IsHumanoid);
        Assert.Null(r.Biped);
        var parts = r.Definition.Parts;
        Assert.True(parts.Length >= 6);
        Assert.Equal(10.0, parts.Sum(p => (double)p.Mass), 3);
        Assert.All(parts, p => Assert.Equal(P3.RagdollShapeKind.Capsule, p.Shape));
        for (int i = 1; i < parts.Length; i++) { Assert.InRange(parts[i].Parent, 0, i - 1); }
        new P3.PhysicsWorld3D().AddRagdoll(r.Definition);
        // a flat generic part (a wing) becomes a box on its principal axes
        var wn = new[] { "hub", "wing" }; var wp = new[] { -1, 0 }; var wpos = new[] { new Vector3F(0, 0, 0), new Vector3F(1, 0, 0) };
        var pts = new List<Vector3F>(); var bi = new List<int>(); var bw = new List<float>();
        for (int i = 0; i < 40; i++) { float x = 1 + i * 0.1f; foreach (float z in new[] { -0.4f, 0.4f }) { pts.Add(new Vector3F(x, 0.01f * (i % 2), z)); bi.Add(1); bw.Add(1f); } }
        for (int i = 0; i < 6; i++) { pts.Add(new Vector3F(0.1f * i - 0.3f, 0.1f * (i % 2), 0.1f * (i % 3))); bi.Add(0); bw.Add(1f); }
        var flat = AutoRagdoller.Create(wn, wp, wpos, new SkinnedPoints3D(pts.ToArray(), bi.ToArray(), bw.ToArray(), 1));
        var wing = flat.Definition.Parts[flat.PartOfBone[1]];
        Assert.Equal(P3.RagdollShapeKind.Box, wing.Shape);
        float[] h = { wing.ShapeHalfSize.X, wing.ShapeHalfSize.Y, wing.ShapeHalfSize.Z };
        Assert.InRange(h[0], 0.38f, 0.42f);       // across the wing
        Assert.InRange(h[1], 1.9f, 2.1f);         // along it
        Assert.True(h[2] < 0.05f);                // thickness
    }

    [Fact]
    public void HostileInputsFailWithTypedErrors()
    {
        var sk = TestSkeleton.Humanoid();
        GeometryToolError Err(string[] n, int[] p, Vector3F[] x, SkinnedPoints3D? skin = null, AutoRagdollOptions? o = null)
        {
            var ex = Assert.Throws<GeometryToolException>(() => AutoRagdoller.Create(n, p, x, skin, o));
            Assert.False(string.IsNullOrEmpty(ex.Message));
            return ex.Error;
        }

        Assert.Equal(GeometryToolError.Empty, Err(Array.Empty<string>(), Array.Empty<int>(), Array.Empty<Vector3F>()));
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, sk.Parents.Take(3).ToArray(), sk.Positions));
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, sk.Parents, sk.Positions.Take(3).ToArray()));
        var p1 = (int[])sk.Parents.Clone(); p1[3] = 99;
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, p1, sk.Positions));
        var p2 = (int[])sk.Parents.Clone(); p2[3] = 3;
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, p2, sk.Positions));
        var p3 = (int[])sk.Parents.Clone(); p3[0] = 5;   // the root hangs below its own descendant: a cycle, and no root
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, p3, sk.Positions));
        var nan = (Vector3F[])sk.Positions.Clone(); nan[4] = new Vector3F(float.NaN, 0, 0);
        Assert.Equal(GeometryToolError.NonFinite, Err(sk.Names, sk.Parents, nan));
        var far = (Vector3F[])sk.Positions.Clone(); far[4] = new Vector3F(1e9f, 0, 0);
        Assert.Equal(GeometryToolError.OutOfRange, Err(sk.Names, sk.Parents, far));
        var same = sk.Positions.Select(_ => new Vector3F(1, 1, 1)).ToArray();
        Assert.Equal(GeometryToolError.Degenerate, Err(sk.Names, sk.Parents, same));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { TotalMass = 0f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { TotalMass = float.NaN }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { Density = -1f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { Friction = -1f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { MinSwingHalfAngle = 4f }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { Facing = 0 }));
        Assert.Equal(GeometryToolError.InvalidOptions, Err(sk.Names, sk.Parents, sk.Positions, null, new AutoRagdollOptions { MassModel = (AutoRagdollMassModel)9 }));

        var good = sk.Skin;
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, sk.Parents, sk.Positions, new SkinnedPoints3D(good.Vertices, good.BoneIndices.Take(5).ToArray(), good.BoneWeights, 2)));
        Assert.Equal(GeometryToolError.OutOfRange, Err(sk.Names, sk.Parents, sk.Positions, new SkinnedPoints3D(good.Vertices, good.BoneIndices, good.BoneWeights, 0)));
        var badBone = (int[])good.BoneIndices.Clone(); badBone[0] = 500;
        Assert.Equal(GeometryToolError.BadIndex, Err(sk.Names, sk.Parents, sk.Positions, new SkinnedPoints3D(good.Vertices, badBone, good.BoneWeights, 2)));
        var negW = (float[])good.BoneWeights.Clone(); negW[0] = -1f;
        Assert.Equal(GeometryToolError.NonFinite, Err(sk.Names, sk.Parents, sk.Positions, new SkinnedPoints3D(good.Vertices, good.BoneIndices, negW, 2)));
        var badV = (Vector3F[])good.Vertices.Clone(); badV[2] = new Vector3F(float.PositiveInfinity, 0, 0);
        Assert.Equal(GeometryToolError.NonFinite, Err(sk.Names, sk.Parents, sk.Positions, new SkinnedPoints3D(badV, good.BoneIndices, good.BoneWeights, 2)));
        // zero weights everywhere: no points to fit, proportions are used
        var zero = new SkinnedPoints3D(good.Vertices, good.BoneIndices, new float[good.BoneWeights.Length], 2);
        Assert.Equal(13, AutoRagdoller.Create(sk.Names, sk.Parents, sk.Positions, zero).Definition.Parts.Length);

        // too many parts: a hub with 70 bones around it
        string[] StarNames(int n) => new[] { "hub" }.Concat(Enumerable.Range(0, n).Select(i => "s" + i)).ToArray();
        int[] StarParents(int n) => new[] { -1 }.Concat(Enumerable.Repeat(0, n)).ToArray();
        Vector3F[] StarPositions(int n) => new[] { new Vector3F(0, 0, 0) }.Concat(Enumerable.Range(0, n).Select(i => new Vector3F((float)Math64.Cos(0.0897 * i), 0.5f + 0.01f * i, (float)Math64.Sin(0.0897 * i)))).ToArray();
        Assert.Equal(GeometryToolError.OutOfRange, Err(StarNames(70), StarParents(70), StarPositions(70)));
        Assert.Equal(64, AutoRagdoller.Create(StarNames(63), StarParents(63), StarPositions(63)).Definition.Parts.Length);
        // a bone name that is null or empty is allowed
        var nn = (string[])sk.Names.Clone(); nn[5] = null!; nn[6] = "";
        Assert.NotNull(AutoRagdoller.Create(nn, sk.Parents, sk.Positions));
    }

    [Fact]
    public void ThreeDimensionalPileFallsAndSettles()
    {
        var m = AutoRagdollScenes.RunPile3D();
        Assert.True(m.Passes(false), m.ToString());
        Assert.True(m.JointError <= 0.3, m.ToString());
    }

    [Fact]
    public void TheFigureStandsUnderTheBalanceControllerWithTheGeneratedBones()
    {
        var m = AutoRagdollScenes.RunStand3D(600, 12f);
        Assert.True(m.Passes(true), m.ToString());
    }

    [Fact]
    public void PileHashesDoNotDependOnTheThreadCount()
    {
        Assert.Equal(AutoRagdollScenes.HashPile3D(150, 1), AutoRagdollScenes.HashPile3D(150, 8));
    }

    [Fact]
    public void TwoDimensionalRagdollHasThirteenPartsAndSettles()
    {
        var sk = TestSkeleton.Humanoid();
        var r = AutoRagdoller.Create2D(sk.Names, sk.Parents, sk.Positions2D(), sk.Skin2D());
        Assert.True(r.IsHumanoid);
        var parts = r.Definition.Parts;
        Assert.Equal(13, parts.Length);
        Assert.Equal(75.0, parts.Sum(p => (double)p.Mass), 3);
        Assert.Equal(P2.RagdollSelfCollision.None, r.Definition.SelfCollision);
        Assert.Equal(-0.7f, parts[7].LowerAngle); Assert.Equal(1.5f, parts[7].UpperAngle);        // hip: forward flexion positive
        Assert.Equal(-2.4f, parts[8].LowerAngle); Assert.Equal(0.05f, parts[8].UpperAngle);       // knee bends backwards
        Assert.Equal(P2.RagdollShapeKind.Capsule, parts[7].Shape);
        Assert.Equal(MathF.PI / 2, parts[7].Angle, 3);
        Assert.InRange(parts[7].ShapeHalfSize.Y, 0.075f, 0.085f);   // radius from the silhouette
        // a figure facing -x has its limits mirrored
        var left = AutoRagdoller.Create2D(sk.Names, sk.Parents, sk.Positions2D(), sk.Skin2D(), new AutoRagdollOptions { Facing = -1 });
        Assert.Equal(-1.5f, left.Definition.Parts[7].LowerAngle); Assert.Equal(0.7f, left.Definition.Parts[7].UpperAngle);
        var b = r.Biped!;
        Assert.Equal((0, 1, 2, 7, 8, 9, 10, 11, 12), (b.Pelvis, b.Torso, b.Head, b.LeftThigh, b.LeftShin, b.LeftFoot, b.RightThigh, b.RightShin, b.RightFoot));
        new P2.PhysicsWorld2D().AddRagdoll(r.Definition);
        var m = AutoRagdollScenes.RunPile2D();
        Assert.True(m.Passes(false), m.ToString());
        Assert.True(m.JointError <= 0.2, m.ToString());
    }

    [Fact]
    public void TwoDimensionalHostileInputsFailWithTypedErrors()
    {
        var sk = TestSkeleton.Humanoid();
        var pos = sk.Positions2D();
        Assert.Throws<GeometryToolException>(() => AutoRagdoller.Create2D(sk.Names, sk.Parents, pos.Take(2).ToArray()));
        var nan = (Vector2F[])pos.Clone(); nan[1] = new Vector2F(float.NaN, 0);
        Assert.Equal(GeometryToolError.NonFinite, Assert.Throws<GeometryToolException>(() => AutoRagdoller.Create2D(sk.Names, sk.Parents, nan)).Error);
        var skin = sk.Skin2D();
        var bad = new SkinnedPoints2D(skin.Vertices, skin.BoneIndices.Select(_ => 999).ToArray(), skin.BoneWeights, 2);
        Assert.Equal(GeometryToolError.BadIndex, Assert.Throws<GeometryToolException>(() => AutoRagdoller.Create2D(sk.Names, sk.Parents, pos, bad)).Error);
    }

    [Fact]
    public void SameBitsOnEveryRun()
    {
        var sk = TestSkeleton.Humanoid();
        var a = Make(sk).Definition.Parts; var b = Make(sk).Definition.Parts;
        for (int i = 0; i < a.Length; i++)
        {
            Assert.Equal(a[i].Position, b[i].Position); Assert.Equal(a[i].Mass, b[i].Mass); Assert.Equal(a[i].ShapeHalfSize, b[i].ShapeHalfSize); Assert.Equal(a[i].JointRotation, b[i].JointRotation); Assert.Equal(a[i].Rotation, b[i].Rotation);
        }
    }
}
