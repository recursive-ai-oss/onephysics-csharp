using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;

namespace RecursiveAI.OnePhysics.Tools;

internal enum FitShape : byte { Capsule, Sphere, Box }

/// <summary>One ragdoll part as the fitter sees it, in double precision and dimension-neutral (2D leaves z at 0).</summary>
internal sealed class PartFit
{
    public BoneRole Kind;                    // Pelvis, Spine (abdomen), Chest (torso or chest), Head, UpperArm, Forearm, Hand, Thigh, Shin, Foot, or Generic
    public bool CombinedTorso;               // Chest standing for the whole torso
    public int Parent = -1;
    public int FirstBone;
    public readonly List<int> Bones = new();      // every bone the part carries, in bone index order
    public readonly List<int> Vertices = new();   // skin points dominated by those bones
    public double Sx, Sy, Sz, Ex, Ey, Ez;
    public double Dx, Dy = 1, Dz;                 // unit axis from start to end
    public FitShape Shape;
    public double Cx, Cy, Cz, A, B, C;
    public bool Oriented;                          // box with its own axes
    public double Ax0, Ay0, Az0, Ax1, Ay1, Az1, Ax2, Ay2, Az2;   // box axes (columns) when Oriented
    public double Fraction, Mass;
    public bool Terminal;
}

internal sealed class SkeletonModel
{
    public readonly List<PartFit> Parts = new();
    public int[] PartOfBone = Array.Empty<int>();
    public BoneRole[] Roles = Array.Empty<BoneRole>();
    public bool Humanoid;
    public int Pelvis = -1, Torso = -1, Head = -1, LeftThigh = -1, LeftShin = -1, LeftFoot = -1, RightThigh = -1, RightShin = -1, RightFoot = -1;
    public double Height;
    public bool Planar;
}

/// <summary>Skin data as doubles for the fitter.</summary>
internal sealed class MeshData
{
    public double[] X = Array.Empty<double>(), Y = Array.Empty<double>(), Z = Array.Empty<double>();
    public int[] Bones = Array.Empty<int>();
    public float[] Weights = Array.Empty<float>();
    public int K;
}

internal static class SkeletonFitter
{
    public const int MaxBones = 1024;
    public const int MaxParts = 64;
    public const int MaxSkinPoints = 1 << 24;

    // ------------------------------------------------------------------ names

    private static string Strip(string name)
    {
        int cut = Math.Max(name.LastIndexOf(':'), name.LastIndexOf('|'));
        return cut >= 0 ? name.Substring(cut + 1) : name;
    }

    // Lower-case letter tokens, split at separators, digits and lower-to-upper changes: "mixamorig:LeftUpLeg" -> left, up, leg.
    private static List<string> Tokens(string name)
    {
        string s = Strip(name);
        var list = new List<string>();
        var cur = new System.Text.StringBuilder();
        void Flush() { if (cur.Length > 0) { list.Add(cur.ToString()); cur.Clear(); } }
        for (int i = 0; i < s.Length; i++)
        {
            char c = s[i];
            if (c >= 'A' && c <= 'Z')
            {
                bool prevLower = i > 0 && s[i - 1] >= 'a' && s[i - 1] <= 'z';
                bool nextLower = i + 1 < s.Length && s[i + 1] >= 'a' && s[i + 1] <= 'z' && i > 0 && s[i - 1] >= 'A' && s[i - 1] <= 'Z';
                if (prevLower || nextLower) { Flush(); }
                cur.Append((char)(c + 32));
            }
            else if (c >= 'a' && c <= 'z') { cur.Append(c); }
            else { Flush(); }
        }

        Flush();
        return list;
    }

    private static bool Has(string joined, params string[] words)
    {
        foreach (string w in words) { if (joined.Contains(w, StringComparison.Ordinal)) { return true; } }
        return false;
    }

    private static BoneRole Heuristic(string name, out bool leg)
    {
        leg = false;
        var tok = Tokens(name);
        string j = string.Concat(tok);
        bool T(string w) => tok.Contains(w);
        if (Has(j, "twist", "roll", "helper", "corrective", "pole", "nub", "ctrl", "jiggle", "target", "end", "top") || T("ik")) { return BoneRole.Merge; }
        if (Has(j, "thumb", "index", "middle", "ring", "pinky", "pinkie", "finger")) { return BoneRole.Merge; }
        if (T("toe") || Has(j, "toebase", "toes") || T("ball")) { return BoneRole.Toe; }
        if (Has(j, "foot", "ankle")) { return BoneRole.Foot; }
        if (Has(j, "hand", "wrist", "palm")) { return BoneRole.Hand; }
        if (Has(j, "forearm", "lowerarm", "lowarm", "elbow", "radius", "ulna")) { return BoneRole.Forearm; }
        if (Has(j, "clavicle", "collar", "shoulder", "scapula")) { return BoneRole.Merge; }
        if (Has(j, "upperarm", "uparm", "humerus") || (T("arm") && !T("fore") && !T("lower"))) { return BoneRole.UpperArm; }
        if (Has(j, "thigh", "upleg", "upperleg", "femur")) { return BoneRole.Thigh; }
        if (Has(j, "calf", "shin", "lowerleg", "lowleg", "knee", "tibia")) { return BoneRole.Shin; }
        if (T("leg")) { leg = true; return BoneRole.Shin; }
        if (T("head")) { return BoneRole.Head; }
        if (T("neck")) { return BoneRole.Neck; }
        if (Has(j, "chest", "thorax", "ribcage", "torso")) { return BoneRole.Chest; }
        if (Has(j, "spine", "abdomen", "waist", "stomach", "lumbar", "belly")) { return BoneRole.Spine; }
        if (Has(j, "pelvis") || T("hips") || (T("hip") && !T("l") && !T("r") && !T("left") && !T("right"))) { return BoneRole.Pelvis; }
        if (T("hip")) { return BoneRole.Thigh; }
        if (Has(j, "root", "armature", "skeleton", "origin", "world", "reference", "bip", "master")) { return BoneRole.Merge; }
        return BoneRole.Generic;
    }

    // ------------------------------------------------------------------ run

    public static SkeletonModel Run(string?[] names, int[] parent, double[] x, double[] y, double[] z, MeshData? mesh, AutoRagdollOptions opt, bool planar)
    {
        int n = names.Length;
        Validate(names, parent, x, y, z, mesh, opt);
        var model = new SkeletonModel { Planar = planar };

        var roles = new BoneRole[n];
        var legWord = new bool[n];
        for (int i = 0; i < n; i++)
        {
            BoneRole r = Heuristic(names[i] ?? string.Empty, out legWord[i]);
            if (opt.RoleOverrides is not null && names[i] is not null && opt.RoleOverrides.TryGetValue(names[i]!, out var ov) && ov != BoneRole.Auto) { r = ov; legWord[i] = false; }
            roles[i] = r;
        }

        // A bone called just "leg" is a thigh when it hangs from the hips and a shin below a thigh.
        for (int i = 0; i < n; i++)
        {
            if (!legWord[i]) { continue; }
            int p = parent[i];
            while (p >= 0 && roles[p] == BoneRole.Merge) { p = parent[p]; }
            roles[i] = p >= 0 && roles[p] == BoneRole.Thigh ? BoneRole.Shin : BoneRole.Thigh;
        }

        model.Roles = roles;
        var children = new List<int>[n];
        for (int i = 0; i < n; i++) { children[i] = new List<int>(); }
        for (int i = 0; i < n; i++) { if (parent[i] >= 0) { children[parent[i]].Add(i); } }
        var depth = new int[n];
        for (int i = 0; i < n; i++) { int d = 0; for (int k = parent[i]; k >= 0; k = parent[k]) { d++; } depth[i] = d; }
        var subtree = new int[n];
        for (int i = 0; i < n; i++) { for (int k = i; k >= 0; k = parent[k]) { subtree[k]++; } }

        var explicitPart = new int[n];
        Array.Fill(explicitPart, -1);
        bool human = TryHumanoid(model, roles, parent, children, depth, x, y, z, opt, explicitPart);
        if (!human) { model.Parts.Clear(); Array.Fill(explicitPart, -1); BuildGeneric(model, roles, parent, children, subtree, x, y, z, opt, explicitPart); }
        model.Humanoid = human;

        // Bones without a part of their own go with the nearest ancestor that has one.
        var partOfBone = new int[n];
        for (int i = 0; i < n; i++)
        {
            int k = i;
            while (k >= 0 && explicitPart[k] < 0) { k = parent[k]; }
            partOfBone[i] = k >= 0 ? explicitPart[k] : -1;
        }

        model.PartOfBone = partOfBone;
        for (int i = 0; i < n; i++) { if (partOfBone[i] >= 0) { model.Parts[partOfBone[i]].Bones.Add(i); } }

        if (!human)
        {
            for (int p = 0; p < model.Parts.Count; p++)
            {
                var part = model.Parts[p];
                int pb = parent[part.FirstBone];
                int pp = pb >= 0 ? partOfBone[pb] : -1;
                part.Parent = p == 0 ? -1 : (pp >= 0 && pp < p ? pp : 0);
            }
        }

        double ymin = double.MaxValue, ymax = double.MinValue;
        if (mesh is not null && mesh.Y.Length > 0) { for (int i = 0; i < mesh.Y.Length; i++) { ymin = Math.Min(ymin, mesh.Y[i]); ymax = Math.Max(ymax, mesh.Y[i]); } }
        else { for (int i = 0; i < n; i++) { ymin = Math.Min(ymin, y[i]); ymax = Math.Max(ymax, y[i]); } ymax = ymin + (ymax - ymin) * 1.04; }
        double h = ymax - ymin;
        if (!(h > 1e-6))
        {
            // A skeleton lying along another axis or flat: use the largest extent of the bones.
            double e = 0;
            for (int i = 0; i < n; i++) { for (int k = i + 1; k < n; k++) { e = Math.Max(e, Math.Max(Math.Abs(x[i] - x[k]), Math.Max(Math.Abs(y[i] - y[k]), Math.Abs(z[i] - z[k])))); } }
            h = e;
        }

        if (!(h > 1e-6)) { throw Err(GeometryToolError.Degenerate, "the skeleton has no extent: every bone is at the same place", "positions"); }
        model.Height = h;

        if (mesh is not null) { AssignVertices(model, mesh); }
        foreach (var part in model.Parts) { ComputeEnds(model, part, x, y, z, human, planar, opt); }
        foreach (var part in model.Parts) { FitPart(model, part, mesh, planar, opt); }
        AssignMass(model, opt, planar);
        return model;
    }

    private static void Validate(string?[] names, int[] parent, double[] x, double[] y, double[] z, MeshData? mesh, AutoRagdollOptions opt)
    {
        int n = names.Length;
        if (n == 0) { throw Err(GeometryToolError.Empty, "the skeleton has no bones", "boneNames"); }
        if (n > MaxBones) { throw Err(GeometryToolError.OutOfRange, "the skeleton has " + MeshInput.Inv(n) + " bones, the limit is " + MeshInput.Inv(MaxBones), "boneNames"); }
        if (parent.Length != n) { throw Err(GeometryToolError.BadIndex, "parents has " + MeshInput.Inv(parent.Length) + " entries for " + MeshInput.Inv(n) + " bones", "parents"); }
        if (x.Length != n) { throw Err(GeometryToolError.BadIndex, "positions has " + MeshInput.Inv(x.Length) + " entries for " + MeshInput.Inv(n) + " bones", "positions"); }
        const double Bound = 32768.0;
        for (int i = 0; i < n; i++)
        {
            if (!double.IsFinite(x[i]) || !double.IsFinite(y[i]) || !double.IsFinite(z[i]))
            {
                throw Err(GeometryToolError.NonFinite, "positions[" + MeshInput.Inv(i) + "] is not finite", "positions");
            }

            if (Math.Abs(x[i]) > Bound || Math.Abs(y[i]) > Bound || Math.Abs(z[i]) > Bound)
            {
                throw Err(GeometryToolError.OutOfRange, "positions[" + MeshInput.Inv(i) + "] is beyond ±2^15", "positions");
            }

            if (parent[i] < -1 || parent[i] >= n || parent[i] == i)
            {
                throw Err(GeometryToolError.BadIndex, "parents[" + MeshInput.Inv(i) + "] = " + MeshInput.Inv(parent[i]) + " is not a bone index or -1 (and a bone cannot be its own parent)", "parents");
            }
        }

        for (int i = 0; i < n; i++)
        {
            int steps = 0;
            for (int k = i; k >= 0; k = parent[k])
            {
                if (++steps > n) { throw Err(GeometryToolError.BadIndex, "the parents of bone " + MeshInput.Inv(i) + " form a cycle", "parents"); }
            }
        }

        bool roots = false;
        for (int i = 0; i < n; i++) { if (parent[i] == -1) { roots = true; } }
        if (!roots) { throw Err(GeometryToolError.BadIndex, "no bone has parent -1", "parents"); }

        if (!(opt.TotalMass > 0f) || !float.IsFinite(opt.TotalMass) || opt.TotalMass > 1e7f) { throw Opt("TotalMass", opt.TotalMass, "finite, above 0 and at most 1e7"); }
        if (!(opt.Density > 0f) || !float.IsFinite(opt.Density) || opt.Density > 1e6f) { throw Opt("Density", opt.Density, "finite, above 0 and at most 1e6"); }
        if (!(opt.Friction >= 0f) || !float.IsFinite(opt.Friction) || opt.Friction > 100f) { throw Opt("Friction", opt.Friction, "finite, from 0 to 100"); }
        if (!(opt.MinBoneLength >= 0f) || !float.IsFinite(opt.MinBoneLength)) { throw Opt("MinBoneLength", opt.MinBoneLength, "finite and at least 0"); }
        if (!(opt.MinSwingHalfAngle >= 0f && opt.MinSwingHalfAngle <= 3.1415927f)) { throw Opt("MinSwingHalfAngle", opt.MinSwingHalfAngle, "from 0 to pi"); }
        if (opt.Facing != 1 && opt.Facing != -1) { throw Opt("Facing", opt.Facing, "+1 or -1"); }
        if (!Enum.IsDefined(opt.SelfCollision)) { throw Opt("SelfCollision", (int)opt.SelfCollision, "a defined mode"); }
        if (!Enum.IsDefined(opt.MassModel)) { throw Opt("MassModel", (int)opt.MassModel, "a defined model"); }

        if (mesh is not null)
        {
            if (mesh.K < 1 || mesh.K > 16) { throw Err(GeometryToolError.OutOfRange, "InfluencesPerVertex = " + MeshInput.Inv(mesh.K) + " must be from 1 to 16", "skin"); }
            int nv = mesh.X.Length;
            if (nv > MaxSkinPoints) { throw Err(GeometryToolError.OutOfRange, "the skin has " + MeshInput.Inv(nv) + " points, the limit is " + MeshInput.Inv(MaxSkinPoints), "skin"); }
            if (mesh.Bones.Length != nv * mesh.K || mesh.Weights.Length != nv * mesh.K) { throw Err(GeometryToolError.BadIndex, "BoneIndices and BoneWeights need " + MeshInput.Inv(nv) + " x " + MeshInput.Inv(mesh.K) + " entries; they have " + MeshInput.Inv(mesh.Bones.Length) + " and " + MeshInput.Inv(mesh.Weights.Length), "skin"); }
            for (int i = 0; i < nv; i++)
            {
                if (!double.IsFinite(mesh.X[i]) || !double.IsFinite(mesh.Y[i]) || !double.IsFinite(mesh.Z[i]) || Math.Abs(mesh.X[i]) > Bound || Math.Abs(mesh.Y[i]) > Bound || Math.Abs(mesh.Z[i]) > Bound)
                {
                    throw Err(GeometryToolError.NonFinite, "skin vertex " + MeshInput.Inv(i) + " is not finite or is beyond ±2^15", "skin");
                }
            }

            for (int i = 0; i < mesh.Bones.Length; i++)
            {
                if (!float.IsFinite(mesh.Weights[i]) || mesh.Weights[i] < 0f) { throw Err(GeometryToolError.NonFinite, "skin weight " + MeshInput.Inv(i) + " is negative or not finite", "skin"); }
                if (mesh.Weights[i] > 0f && (mesh.Bones[i] < 0 || mesh.Bones[i] >= n)) { throw Err(GeometryToolError.BadIndex, "skin influence " + MeshInput.Inv(i) + " names bone " + MeshInput.Inv(mesh.Bones[i]) + ", outside the " + MeshInput.Inv(n) + " bones", "skin"); }
            }
        }
    }

    private static GeometryToolException Err(GeometryToolError e, string message, string? param) => new(e, message, param);

    private static GeometryToolException Opt(string name, float v, string expected) => Err(GeometryToolError.InvalidOptions, name + " = " + v.ToString("R", CultureInfo.InvariantCulture) + " must be " + expected, "options");

    // ------------------------------------------------------------------ humanoid

    private static int FindDesc(int root, BoneRole role, List<int>[] children, BoneRole[] roles)
    {
        var queue = new List<int> { root };
        for (int h = 0; h < queue.Count; h++)
        {
            foreach (int c in children[queue[h]])
            {
                if (roles[c] == role) { return c; }
                queue.Add(c);
            }
        }

        return -1;
    }

    private static bool IsUnder(int b, int anc, int[] parent)
    {
        for (int k = parent[b]; k >= 0; k = parent[k]) { if (k == anc) { return true; } }
        return false;
    }

    private static bool TryHumanoid(SkeletonModel m, BoneRole[] roles, int[] parent, List<int>[] children, int[] depth, double[] x, double[] y, double[] z, AutoRagdollOptions opt, int[] explicitPart)
    {
        int n = roles.Length;
        int pelvis = -1;
        for (int i = 0; i < n && pelvis < 0; i++) { if (roles[i] == BoneRole.Pelvis) { pelvis = i; } }
        if (pelvis < 0) { return false; }
        var thighs = new List<int>();
        for (int i = 0; i < n; i++) { if (roles[i] == BoneRole.Thigh && IsUnder(i, pelvis, parent)) { thighs.Add(i); } }
        if (thighs.Count != 2) { return false; }
        int[] shins = new int[2], feet = new int[2];
        for (int s = 0; s < 2; s++)
        {
            shins[s] = FindDesc(thighs[s], BoneRole.Shin, children, roles);
            if (shins[s] < 0) { return false; }
            feet[s] = FindDesc(shins[s], BoneRole.Foot, children, roles);
            if (feet[s] < 0) { return false; }
        }

        bool InLeg(int b) => b == thighs[0] || b == thighs[1] || IsUnder(b, thighs[0], parent) || IsUnder(b, thighs[1], parent);
        var spine = new List<int>();
        for (int i = 0; i < n; i++) { if ((roles[i] == BoneRole.Spine || roles[i] == BoneRole.Chest) && IsUnder(i, pelvis, parent) && !InLeg(i)) { spine.Add(i); } }
        if (spine.Count == 0) { return false; }
        spine.Sort((a, b) => depth[a] != depth[b] ? depth[a].CompareTo(depth[b]) : a.CompareTo(b));
        int firstSpine = spine[0];
        spine.RemoveAll(b => b != firstSpine && !IsUnder(b, firstSpine, parent));
        int head = -1;
        for (int i = 0; i < n; i++)
        {
            if (roles[i] == BoneRole.Head && IsUnder(i, pelvis, parent) && !InLeg(i) && (head < 0 || depth[i] < depth[head])) { head = i; }
        }

        var neck = new List<int>();
        if (head >= 0)
        {
            for (int i = 0; i < n; i++) { if (roles[i] == BoneRole.Neck && IsUnder(head, i, parent)) { neck.Add(i); } }
            neck.Sort((a, b) => depth[a].CompareTo(depth[b]));
        }

        var uppers = new List<int>();
        for (int i = 0; i < n; i++) { if (roles[i] == BoneRole.UpperArm && IsUnder(i, pelvis, parent) && !InLeg(i)) { uppers.Add(i); } }
        bool arms = false;
        int[] fore = new int[2], hand = new int[2];
        if (uppers.Count == 2)
        {
            arms = true;
            for (int s = 0; s < 2; s++)
            {
                fore[s] = FindDesc(uppers[s], BoneRole.Forearm, children, roles);
                if (fore[s] < 0) { arms = false; break; }
                hand[s] = FindDesc(fore[s], BoneRole.Hand, children, roles);
            }
        }

        // Left is the side with the larger x (the figure faces +z, up is +y; in a side view the lower bone index).
        int[] legOrder = x[thighs[0]] >= x[thighs[1]] ? new[] { 0, 1 } : new[] { 1, 0 };
        int[] armOrder = arms && x[uppers[0]] < x[uppers[1]] ? new[] { 1, 0 } : new[] { 0, 1 };

        PartFit Add(BoneRole kind, int firstBone, List<int>? bones = null, bool combined = false)
        {
            var p = new PartFit { Kind = kind, FirstBone = firstBone, CombinedTorso = combined };
            m.Parts.Add(p);
            int idx = m.Parts.Count - 1;
            if (bones is null) { explicitPart[firstBone] = idx; }
            else { foreach (int b in bones) { explicitPart[b] = idx; } }
            p.Sx = x[firstBone]; p.Sy = y[firstBone]; p.Sz = z[firstBone];
            return p;
        }

        Add(BoneRole.Pelvis, pelvis); m.Pelvis = 0;
        bool split = opt.SplitTorso && spine.Count >= 2;
        int lastTorso;
        if (split)
        {
            int k = spine.Count / 2;
            var lower = spine.GetRange(0, k); var upper = spine.GetRange(k, spine.Count - k);
            var a = Add(BoneRole.Spine, lower[0], lower); a.Parent = 0;
            var b = Add(BoneRole.Chest, upper[0], upper); b.Parent = m.Parts.Count - 2;
            lastTorso = m.Parts.Count - 1; m.Torso = 1;
        }
        else
        {
            var a = Add(BoneRole.Chest, spine[0], spine, combined: true); a.Parent = 0;
            lastTorso = m.Parts.Count - 1; m.Torso = 1;
        }

        if (head >= 0)
        {
            var hb = new List<int>(neck) { head };
            var h = Add(BoneRole.Head, hb[0], hb); h.Parent = lastTorso; m.Head = m.Parts.Count - 1;
        }

        if (arms)
        {
            foreach (int s in armOrder)
            {
                var ua = Add(BoneRole.UpperArm, uppers[s]); ua.Parent = lastTorso;
                int uaIdx = m.Parts.Count - 1;
                var fa = Add(BoneRole.Forearm, fore[s]); fa.Parent = uaIdx;
                if (hand[s] >= 0 && opt.IncludeHands) { var hd = Add(BoneRole.Hand, hand[s]); hd.Parent = m.Parts.Count - 2; }
            }
        }

        for (int side = 0; side < 2; side++)
        {
            int s = legOrder[side];
            var th = Add(BoneRole.Thigh, thighs[s]); th.Parent = 0;
            int thIdx = m.Parts.Count - 1;
            var sh = Add(BoneRole.Shin, shins[s]); sh.Parent = thIdx;
            var ft = Add(BoneRole.Foot, feet[s]); ft.Parent = thIdx + 1;
            if (side == 0) { m.LeftThigh = thIdx; m.LeftShin = thIdx + 1; m.LeftFoot = thIdx + 2; }
            else { m.RightThigh = thIdx; m.RightShin = thIdx + 1; m.RightFoot = thIdx + 2; }
        }

        // Toes go with the foot (assigned through the ancestors); nothing else to record.
        _ = children;
        return true;
    }

    // ------------------------------------------------------------------ generic skeleton

    private static void BuildGeneric(SkeletonModel m, BoneRole[] roles, int[] parent, List<int>[] children, int[] subtree, double[] x, double[] y, double[] z, AutoRagdollOptions opt, int[] explicitPart)
    {
        int n = roles.Length;
        double ymin = double.MaxValue, ymax = double.MinValue;
        for (int i = 0; i < n; i++) { ymin = Math.Min(ymin, y[i]); ymax = Math.Max(ymax, y[i]); }
        double ext = 0;
        for (int i = 0; i < n; i++) { for (int k = i + 1; k < n; k++) { ext = Math.Max(ext, Dist(x, y, z, i, k)); } }
        double minLen = opt.MinBoneLength > 0 ? opt.MinBoneLength : 0.02 * Math.Max(ymax - ymin, ext * 0.5);
        // Depth-first pre-order, children in index order.
        var order = new List<int>();
        var stack = new Stack<int>();
        for (int i = n - 1; i >= 0; i--) { if (parent[i] < 0) { stack.Push(i); } }
        while (stack.Count > 0)
        {
            int b = stack.Pop();
            order.Add(b);
            var ch = children[b];
            for (int k = ch.Count - 1; k >= 0; k--) { stack.Push(ch[k]); }
        }

        foreach (int b in order)
        {
            if (roles[b] == BoneRole.Merge) { continue; }
            int primary = PrimaryChild(b, children, roles, subtree);
            double len = primary >= 0 ? Dist(x, y, z, b, primary) : (parent[b] >= 0 ? Dist(x, y, z, b, parent[b]) * 0.5 : 0);
            bool hasKids = primary >= 0;
            // A bone shorter than the minimum is folded into its parent when it has a part to fold into.
            if (m.Parts.Count > 0 && len < minLen) { continue; }
            var p = new PartFit { Kind = BoneRole.Generic, FirstBone = b };
            p.Sx = x[b]; p.Sy = y[b]; p.Sz = z[b];
            if (hasKids) { p.Ex = x[primary]; p.Ey = y[primary]; p.Ez = z[primary]; }
            else { p.Terminal = true; }
            m.Parts.Add(p);
            explicitPart[b] = m.Parts.Count - 1;
        }

        if (m.Parts.Count == 0) { throw Err(GeometryToolError.Degenerate, "no bone is long enough to make a part; lower MinBoneLength", "options"); }
        if (m.Parts.Count > MaxParts) { throw Err(GeometryToolError.OutOfRange, "the skeleton makes " + MeshInput.Inv(m.Parts.Count) + " parts, the ragdoll limit is " + MeshInput.Inv(MaxParts) + "; give role Merge to bones that need no part, or raise MinBoneLength", "options"); }
    }

    private static double Dist(double[] x, double[] y, double[] z, int a, int b) => Math.Sqrt((x[a] - x[b]) * (x[a] - x[b]) + (y[a] - y[b]) * (y[a] - y[b]) + (z[a] - z[b]) * (z[a] - z[b]));

    private static int PrimaryChild(int b, List<int>[] children, BoneRole[] roles, int[] subtree)
    {
        int best = -1;
        foreach (int c in children[b])
        {
            if (roles[c] == BoneRole.Merge) { continue; }
            if (best < 0 || subtree[c] > subtree[best]) { best = c; }
        }

        return best;
    }

    // ------------------------------------------------------------------ geometry

    private static void AssignVertices(SkeletonModel m, MeshData mesh)
    {
        int np = m.Parts.Count;
        var sum = new double[np];
        int nv = mesh.X.Length;
        for (int v = 0; v < nv; v++)
        {
            Array.Clear(sum);
            bool any = false;
            for (int k = 0; k < mesh.K; k++)
            {
                float w = mesh.Weights[v * mesh.K + k];
                if (!(w > 0f)) { continue; }
                int part = m.PartOfBone[mesh.Bones[v * mesh.K + k]];
                if (part < 0) { part = 0; }
                sum[part] += w; any = true;
            }

            if (!any) { continue; }
            int best = 0;
            for (int p = 1; p < np; p++) { if (sum[p] > sum[best]) { best = p; } }
            m.Parts[best].Vertices.Add(v);
        }
    }

    private static void ComputeEnds(SkeletonModel m, PartFit p, double[] x, double[] y, double[] z, bool humanoid, bool planar, AutoRagdollOptions opt)
    {
        int self = m.Parts.IndexOf(p);
        double H = m.Height;
        bool haveEnd = false;
        double ex = p.Sx, ey = p.Sy, ez = p.Sz;
        if (humanoid)
        {
            // The end of a limb or torso part is where its child part starts; terminal parts end at their farthest bone.
            PartFit? child = null;
            foreach (var q in m.Parts)
            {
                if (q.Parent != self || q == p) { continue; }
                bool chain = p.Kind switch
                {
                    BoneRole.Chest or BoneRole.Spine => q.Kind == BoneRole.Head || (q.Kind == BoneRole.Chest && p.Kind == BoneRole.Spine),
                    BoneRole.UpperArm => q.Kind == BoneRole.Forearm,
                    BoneRole.Forearm => q.Kind == BoneRole.Hand,
                    BoneRole.Thigh => q.Kind == BoneRole.Shin,
                    BoneRole.Shin => q.Kind == BoneRole.Foot,
                    _ => false,
                };
                if (chain && (child is null || q.Kind == BoneRole.Chest)) { child = q; }
            }

            if (child is not null) { ex = child.Sx; ey = child.Sy; ez = child.Sz; haveEnd = true; }
        }
        else if (!p.Terminal) { ex = p.Ex; ey = p.Ey; ez = p.Ez; haveEnd = true; }

        if (!haveEnd && humanoid && p.Kind == BoneRole.Forearm)
        {
            // A forearm that carries its hand ends at the wrist (the hand bone), not at the fingertips, so the capsule stays on the forearm's own axis.
            foreach (int b in p.Bones)
            {
                if (m.Roles[b] == BoneRole.Hand) { ex = x[b]; ey = y[b]; ez = z[b]; haveEnd = true; break; }
            }
        }

        if (!haveEnd && p.Kind != BoneRole.Pelvis)
        {
            double best = 0;
            foreach (int b in p.Bones)
            {
                double d = (x[b] - p.Sx) * (x[b] - p.Sx) + (y[b] - p.Sy) * (y[b] - p.Sy) + (z[b] - p.Sz) * (z[b] - p.Sz);
                if (d > best) { best = d; ex = x[b]; ey = y[b]; ez = z[b]; }
            }

            haveEnd = best > 1e-12 * H * H;
        }

        if (!haveEnd)
        {
            // Nothing beyond the joint: a default stub in the direction the part usually points.
            double dx = 0, dy = 1, dz = 0, len = 0.1 * H;
            switch (p.Kind)
            {
                case BoneRole.Head: dy = 1; len = 0.13 * H; break;
                case BoneRole.Foot: dx = planar ? opt.Facing : 0; dy = 0; dz = planar ? 0 : 1; len = 0.1 * H; break;
                case BoneRole.Hand: dy = -1; len = 0.09 * H; break;
                case BoneRole.Pelvis: dy = 0; len = 0; break;
                default:
                    if (p.Parent >= 0)
                    {
                        var q = m.Parts[p.Parent];
                        double qx = p.Sx - q.Sx, qy = p.Sy - q.Sy, qz = p.Sz - q.Sz, ql = Math.Sqrt(qx * qx + qy * qy + qz * qz);
                        if (ql > 1e-12) { dx = qx / ql; dy = qy / ql; dz = qz / ql; len = 0.5 * ql; }
                    }

                    break;
            }

            ex = p.Sx + dx * len; ey = p.Sy + dy * len; ez = p.Sz + dz * len;
        }

        if (planar) { ez = p.Sz; }
        p.Ex = ex; p.Ey = ey; p.Ez = ez;
        double ux = ex - p.Sx, uy = ey - p.Sy, uz = ez - p.Sz, ul = Math.Sqrt(ux * ux + uy * uy + uz * uz);
        if (ul > 1e-12) { p.Dx = ux / ul; p.Dy = uy / ul; p.Dz = uz / ul; } else { p.Dx = 0; p.Dy = 1; p.Dz = 0; }
    }

    // The typical distance: the median for 3D skin points, which sit all round a limb; the 90th percentile for 2D, where a side view of the points of a round limb puts most of them inside the silhouette
    // and only the two outline points mark its width.
    private static double Typical(List<double> v, bool planar)
    {
        if (v.Count == 0) { return 0; }
        v.Sort();
        if (planar) { return v[(int)(0.9 * (v.Count - 1) + 0.5)]; }
        return (v[(v.Count - 1) / 2] + v[v.Count / 2]) * 0.5;
    }

    // Radius over segment length for parts fitted without skin points (the proportions of an adult).
    private static double RadiusRatio(BoneRole kind) => kind switch
    {
        BoneRole.Thigh => 0.19,
        BoneRole.Shin => 0.143,
        BoneRole.UpperArm => 0.145,
        BoneRole.Forearm => 0.148,
        _ => 0.15,
    };

    private static void FitPart(SkeletonModel m, PartFit p, MeshData? mesh, bool planar, AutoRagdollOptions opt)
    {
        double H = m.Height;
        bool skin = mesh is not null && p.Vertices.Count >= 4;
        double minSize = 0.002 * H;
        bool humanBox = m.Humanoid && (p.Kind == BoneRole.Pelvis || p.Kind == BoneRole.Chest || p.Kind == BoneRole.Spine || p.Kind == BoneRole.Hand || p.Kind == BoneRole.Foot);
        FitShape shape = m.Humanoid && p.Kind == BoneRole.Head ? FitShape.Sphere : humanBox ? FitShape.Box : FitShape.Capsule;

        double len = Math.Sqrt((p.Ex - p.Sx) * (p.Ex - p.Sx) + (p.Ey - p.Sy) * (p.Ey - p.Sy) + (p.Ez - p.Sz) * (p.Ez - p.Sz));
        double mnx = double.MaxValue, mny = double.MaxValue, mnz = double.MaxValue, mxx = double.MinValue, mxy = double.MinValue, mxz = double.MinValue;
        if (skin)
        {
            foreach (int v in p.Vertices)
            {
                mnx = Math.Min(mnx, mesh!.X[v]); mxx = Math.Max(mxx, mesh.X[v]); mny = Math.Min(mny, mesh.Y[v]); mxy = Math.Max(mxy, mesh.Y[v]);
                if (!planar) { mnz = Math.Min(mnz, mesh.Z[v]); mxz = Math.Max(mxz, mesh.Z[v]); }
            }

            if (planar) { mnz = mxz = 0; }
        }

        if (shape == FitShape.Sphere)
        {
            p.Shape = FitShape.Sphere;
            if (skin)
            {
                p.Cx = 0.5 * (mnx + mxx); p.Cy = 0.5 * (mny + mxy); p.Cz = 0.5 * (mnz + mxz);
                var d = new List<double>(p.Vertices.Count);
                foreach (int v in p.Vertices) { double dx = mesh!.X[v] - p.Cx, dy = mesh.Y[v] - p.Cy, dz = planar ? 0 : mesh.Z[v] - p.Cz; d.Add(Math.Sqrt(dx * dx + dy * dy + dz * dz)); }
                p.A = Math.Max(Typical(d, planar), minSize);
            }
            else
            {
                double r = Math.Max(Math.Min(0.0611 * H, 0.5 * Math.Max(len, 0.1222 * H)), minSize);
                p.A = r;
                p.Cx = p.Ex - p.Dx * r; p.Cy = p.Ey - p.Dy * r; p.Cz = p.Ez - p.Dz * r;
            }

            return;
        }

        if (shape == FitShape.Box)
        {
            p.Shape = FitShape.Box;
            if (skin)
            {
                p.Cx = 0.5 * (mnx + mxx); p.Cy = 0.5 * (mny + mxy); p.Cz = 0.5 * (mnz + mxz);
                p.A = Math.Max(0.5 * (mxx - mnx), minSize); p.B = Math.Max(0.5 * (mxy - mny), minSize); p.C = planar ? 0 : Math.Max(0.5 * (mxz - mnz), minSize);
                return;
            }

            switch (p.Kind)
            {
                case BoneRole.Pelvis:
                    p.Cx = p.Sx; p.Cy = p.Sy; p.Cz = p.Sz; p.A = (planar ? 0.0667 : 0.0833) * H; p.B = 0.0444 * H; p.C = 0.05 * H; break;
                case BoneRole.Hand:
                    p.Cx = 0.5 * (p.Sx + p.Ex); p.Cy = 0.5 * (p.Sy + p.Ey); p.Cz = 0.5 * (p.Sz + p.Ez); p.A = 0.02 * H; p.B = Math.Max(0.5 * len, minSize); p.C = 0.03 * H; break;
                case BoneRole.Foot:
                {
                    double f = planar ? opt.Facing : 1;
                    p.Cx = p.Sx + (planar ? 0.0333 * H * f : 0); p.Cy = p.Sy - 0.0222 * H; p.Cz = p.Sz + (planar ? 0 : 0.0278 * H);
                    p.A = planar ? 0.0778 * H : 0.0333 * H; p.B = 0.0222 * H; p.C = 0.0833 * H;
                    break;
                }

                default:
                    p.Cx = 0.5 * (p.Sx + p.Ex); p.Cy = 0.5 * (p.Sy + p.Ey); p.Cz = 0.5 * (p.Sz + p.Ez);
                    p.A = (planar ? 0.0722 : 0.0944) * H; p.B = Math.Max(0.5 * len, minSize); p.C = 0.0556 * H; break;
            }

            p.A = Math.Max(p.A, minSize); p.B = Math.Max(p.B, minSize); p.C = planar ? 0 : Math.Max(p.C, minSize);
            return;
        }

        // Capsule along the bone axis (or an oriented box when a generic part's cross-section is flat).
        double tmin = 0, tmax = len, radius = RadiusRatio(p.Kind) * len;
        if (skin)
        {
            var perp = new List<double>(p.Vertices.Count);
            tmin = double.MaxValue; tmax = double.MinValue;
            foreach (int v in p.Vertices)
            {
                double rx = mesh!.X[v] - p.Sx, ry = mesh.Y[v] - p.Sy, rz = planar ? 0 : mesh.Z[v] - p.Sz;
                double t = rx * p.Dx + ry * p.Dy + rz * p.Dz;
                double qx = rx - t * p.Dx, qy = ry - t * p.Dy, qz = rz - t * p.Dz;
                perp.Add(Math.Sqrt(qx * qx + qy * qy + qz * qz));
                tmin = Math.Min(tmin, t); tmax = Math.Max(tmax, t);
            }

            radius = Typical(perp, planar);
            if (!planar && p.Kind == BoneRole.Generic && TryOrientedBox(p, mesh!, minSize)) { return; }
        }

        double L = Math.Max(tmax - tmin, 4 * minSize);
        radius = Math.Max(Math.Min(radius, 0.5 * L), minSize);
        p.Shape = FitShape.Capsule;
        p.A = Math.Max(0.5 * L - radius, minSize);
        p.B = radius;
        double mid = 0.5 * (tmin + tmax);
        p.Cx = p.Sx + p.Dx * mid; p.Cy = p.Sy + p.Dy * mid; p.Cz = p.Sz + p.Dz * mid;
    }

    // A box on the principal axes of the part's skin points (symmetric eigen decomposition of their covariance) when the cross-section across the bone is flat:
    // its two other principal moments differ by more than a factor of 4.
    private static bool TryOrientedBox(PartFit p, MeshData mesh, double minSize)
    {
        int cnt = p.Vertices.Count;
        double mx = 0, my = 0, mz = 0;
        foreach (int v in p.Vertices) { mx += mesh.X[v]; my += mesh.Y[v]; mz += mesh.Z[v]; }
        mx /= cnt; my /= cnt; mz /= cnt;
        double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
        foreach (int v in p.Vertices)
        {
            double a = mesh.X[v] - mx, b = mesh.Y[v] - my, c = mesh.Z[v] - mz;
            xx += a * a; xy += a * b; xz += a * c; yy += b * b; yz += b * c; zz += c * c;
        }

        var eig = SymmetricEigenD.Decompose(new Matrix3x3D(xx, xy, xz, xy, yy, yz, xz, yz, zz));
        double[][] axes = { new[] { eig.Vectors.M11, eig.Vectors.M21, eig.Vectors.M31 }, new[] { eig.Vectors.M12, eig.Vectors.M22, eig.Vectors.M32 }, new[] { eig.Vectors.M13, eig.Vectors.M23, eig.Vectors.M33 } };
        double[] vals = { eig.Values.X, eig.Values.Y, eig.Values.Z };
        int lng = 0; double bestDot = -1;
        for (int i = 0; i < 3; i++)
        {
            double d = Math.Abs(axes[i][0] * p.Dx + axes[i][1] * p.Dy + axes[i][2] * p.Dz);
            if (d > bestDot) { bestDot = d; lng = i; }
        }

        int w1 = (lng + 1) % 3, w2 = (lng + 2) % 3;
        double lo = Math.Min(vals[w1], vals[w2]), hi = Math.Max(vals[w1], vals[w2]);
        if (!(lo > 0) || hi < 4 * lo) { return false; }
        // Axes: y along the bone, x the wider cross-section axis, z completing a right-handed frame.
        int wide = vals[w1] >= vals[w2] ? w1 : w2;
        double[] ex = axes[wide], ey = axes[lng];
        if (ey[0] * p.Dx + ey[1] * p.Dy + ey[2] * p.Dz < 0) { ey = new[] { -ey[0], -ey[1], -ey[2] }; }
        double[] ez = { ex[1] * ey[2] - ex[2] * ey[1], ex[2] * ey[0] - ex[0] * ey[2], ex[0] * ey[1] - ex[1] * ey[0] };
        ex = new[] { ey[1] * ez[2] - ey[2] * ez[1], ey[2] * ez[0] - ey[0] * ez[2], ey[0] * ez[1] - ey[1] * ez[0] };
        double[] lo3 = { double.MaxValue, double.MaxValue, double.MaxValue }, hi3 = { double.MinValue, double.MinValue, double.MinValue };
        double[][] f = { ex, ey, ez };
        foreach (int v in p.Vertices)
        {
            for (int k = 0; k < 3; k++)
            {
                double t = mesh.X[v] * f[k][0] + mesh.Y[v] * f[k][1] + mesh.Z[v] * f[k][2];
                lo3[k] = Math.Min(lo3[k], t); hi3[k] = Math.Max(hi3[k], t);
            }
        }

        double c0 = 0.5 * (lo3[0] + hi3[0]), c1 = 0.5 * (lo3[1] + hi3[1]), c2 = 0.5 * (lo3[2] + hi3[2]);
        p.Cx = c0 * ex[0] + c1 * ey[0] + c2 * ez[0]; p.Cy = c0 * ex[1] + c1 * ey[1] + c2 * ez[1]; p.Cz = c0 * ex[2] + c1 * ey[2] + c2 * ez[2];
        p.A = Math.Max(0.5 * (hi3[0] - lo3[0]), minSize); p.B = Math.Max(0.5 * (hi3[1] - lo3[1]), minSize); p.C = Math.Max(0.5 * (hi3[2] - lo3[2]), minSize);
        p.Shape = FitShape.Box; p.Oriented = true;
        p.Ax0 = ex[0]; p.Ay0 = ex[1]; p.Az0 = ex[2]; p.Ax1 = ey[0]; p.Ay1 = ey[1]; p.Az1 = ey[2]; p.Ax2 = ez[0]; p.Ay2 = ez[1]; p.Az2 = ez[2];
        return true;
    }

    // ------------------------------------------------------------------ mass

    // Segment mass fractions of the body: Winter, "Biomechanics and Motor Control of Human Movement" (2009), table 4.1, from Dempster (1955).
    private static double TableFraction(PartFit p, bool handMerged) => p.Kind switch
    {
        BoneRole.Pelvis => 0.142,
        BoneRole.Chest => p.CombinedTorso ? 0.355 : 0.216,
        BoneRole.Spine => 0.139,
        BoneRole.Head => 0.081,
        BoneRole.UpperArm => 0.028,
        BoneRole.Forearm => handMerged ? 0.022 : 0.016,
        BoneRole.Hand => 0.006,
        BoneRole.Thigh => 0.100,
        BoneRole.Shin => 0.0465,
        BoneRole.Foot => 0.0145,
        _ => 0.0,
    };

    private static double ShapeVolume(PartFit p, bool planar)
    {
        const double Pi = Math.PI;
        if (planar)
        {
            return p.Shape switch { FitShape.Sphere => Pi * p.A * p.A, FitShape.Capsule => 4 * p.A * p.B + Pi * p.B * p.B, _ => 4 * p.A * p.B };
        }

        return p.Shape switch { FitShape.Sphere => 4.0 / 3 * Pi * p.A * p.A * p.A, FitShape.Capsule => Pi * p.B * p.B * (2 * p.A) + 4.0 / 3 * Pi * p.B * p.B * p.B, _ => 8 * p.A * p.B * p.C };
    }

    private static void AssignMass(SkeletonModel m, AutoRagdollOptions opt, bool planar)
    {
        bool table = m.Humanoid && opt.MassModel == AutoRagdollMassModel.Auto;
        bool handMerged = !opt.IncludeHands;
        double sum = 0;
        foreach (var p in m.Parts)
        {
            p.Fraction = table ? TableFraction(p, handMerged) : ShapeVolume(p, planar) * opt.Density;
            sum += p.Fraction;
        }

        if (!(sum > 0)) { throw Err(GeometryToolError.Degenerate, "the parts have no volume, so no mass can be shared among them", "options"); }
        foreach (var p in m.Parts) { p.Mass = opt.TotalMass * p.Fraction / sum; }
    }
}
