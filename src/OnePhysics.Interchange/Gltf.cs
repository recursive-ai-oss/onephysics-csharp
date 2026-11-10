using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OneMath;
using SceneFormat = RecursiveAI.OnePhysics.Scene;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Settings for <see cref="GltfPhysicsImporter"/>.</summary>
public sealed class GltfOptions
{
    /// <summary>Returns the bytes of an external buffer file by its uri, or null if the host does not have it. Data URIs and the binary chunk of a GLB need no resolver.</summary>
    public Func<string, byte[]?>? Buffers { get; set; }

    /// <summary>Density in kg/m³ given to every shape; the format has none, and a body's mass comes from <c>motion.mass</c> when it is given. Default 1000.</summary>
    public double Density { get; set; } = 1000.0;

    /// <summary>Largest number of nodes accepted. Default 200000.</summary>
    public int MaxNodes { get; set; } = 200_000;
}

/// <summary>The result of importing a glTF file with physics extensions.</summary>
public sealed class GltfPhysicsModel
{
    internal GltfPhysicsModel(SceneFormat.Scene scene, List<int> bodyNodes, List<string> bodyNames, List<InterchangeWarning> warnings)
    {
        Scene = scene; BodyNodes = bodyNodes; BodyNames = bodyNames; Warnings = warnings;
    }

    /// <summary>The 3D scene: bodies in the order of <see cref="BodyNodes"/>, gravity (0, -9.81, 0) (glTF is y up), materials 0 (friction 0.6, restitution 0) then one per physics material. Build a world with <c>Scene.CreateWorld3D</c>.</summary>
    public SceneFormat.Scene Scene { get; }

    /// <summary>The glTF node each scene body came from; -1 for the world-anchor body added for a joint to a fixed reference frame.</summary>
    public IReadOnlyList<int> BodyNodes { get; }

    /// <summary>The node names of the scene bodies, by index (empty when the node has none).</summary>
    public IReadOnlyList<string> BodyNames { get; }

    /// <summary>Everything the importer left out or approximated.</summary>
    public IReadOnlyList<InterchangeWarning> Warnings { get; }
}

/// <summary>
/// Reads glTF 2.0 files (JSON or GLB) with physics extensions into a 3D <see cref="SceneFormat.Scene"/>: <c>KHR_physics_rigid_bodies</c> with <c>KHR_implicit_shapes</c> (motion, colliders, physics materials, joints), and the older <c>OMI_physics_body</c> with <c>OMI_physics_shape</c>.
/// A node with <c>motion</c> is a body and its descendants' colliders belong to it; a collider with no moving ancestor is a static body. Sphere and box colliders and capsules and cylinders that sit on their body's origin are exact; other capsules and cylinders become convex hulls,
/// and mesh colliders keep their triangles on static bodies and become the convex hull of their points on moving ones. Joints map to point and twist joints when the limits describe a ball or a hinge; collision filters, triggers, drives and other joint shapes are reported as warnings.
/// </summary>
public static class GltfPhysicsImporter
{
    private const string F = "glTF";
    private const long MaxBytes = 256L * 1024 * 1024;

    /// <summary>Imports a glTF (UTF-8 JSON) or GLB file from memory.</summary>
    /// <exception cref="InterchangeException">The file is malformed or refers to something that is not there (a bad index, a buffer that runs past its end, an unsupported accessor type); the message names the JSON path.</exception>
    public static GltfPhysicsModel Import(ReadOnlySpan<byte> data, GltfOptions? options = null)
    {
        options ??= new GltfOptions();
        if (data.Length > MaxBytes) { throw new InterchangeException(F, "document", "the input is longer than " + MaxBytes + " bytes."); }
        byte[]? glbBin = null;
        ReadOnlyMemory<byte> json;
        var copy = data.ToArray();
        if (copy.Length >= 4 && BinaryPrimitives.ReadUInt32LittleEndian(copy) == 0x46546C67)
        {
            if (copy.Length < 20) { throw new InterchangeException(F, "GLB header", "the file is shorter than a GLB header and first chunk."); }
            uint version = BinaryPrimitives.ReadUInt32LittleEndian(copy.AsSpan(4));
            uint length = BinaryPrimitives.ReadUInt32LittleEndian(copy.AsSpan(8));
            if (version != 2) { throw new InterchangeException(F, "GLB header", "only GLB version 2 is supported, found " + version + "."); }
            if (length > copy.Length) { throw new InterchangeException(F, "GLB header", "the header says " + length + " bytes but the file has " + copy.Length + "."); }
            long pos = 12; ReadOnlyMemory<byte> found = default; bool haveJson = false;
            while (pos + 8 <= length)
            {
                uint clen = BinaryPrimitives.ReadUInt32LittleEndian(copy.AsSpan((int)pos));
                uint ctype = BinaryPrimitives.ReadUInt32LittleEndian(copy.AsSpan((int)pos + 4));
                pos += 8;
                if (pos + clen > length) { throw new InterchangeException(F, "GLB chunk at " + (pos - 8), "the chunk runs past the end of the file."); }
                if (ctype == 0x4E4F534A && !haveJson) { found = new ReadOnlyMemory<byte>(copy, (int)pos, (int)clen); haveJson = true; }
                else if (ctype == 0x004E4942 && glbBin == null) { glbBin = copy.AsSpan((int)pos, (int)clen).ToArray(); }
                pos += (clen + 3L) & ~3L;
            }

            if (!haveJson) { throw new InterchangeException(F, "GLB", "no JSON chunk."); }
            json = found;
        }
        else { json = copy; }

        var root = Js.Parse(json, F);
        return new Reader(root, options, glbBin).Run();
    }

    /// <summary>Imports a glTF or GLB file from a stream.</summary>
    public static GltfPhysicsModel Import(System.IO.Stream stream, GltfOptions? options = null) => Import(StreamText.ReadBytes(stream, MaxBytes, F), options);

    private struct Affine
    {
        public Matrix3x3D L;
        public Vector3D T;

        public static Affine Identity => new() { L = new Matrix3x3D(1, 0, 0, 0, 1, 0, 0, 0, 1), T = default };

        public static Affine operator *(Affine a, Affine b) => new() { L = a.L * b.L, T = a.L * b.T + a.T };
    }

    private sealed class ShapeDesc
    {
        public string Kind = "";   // sphere box cylinder capsule plane convex trimesh
        public double Radius, Height, RadiusTop, RadiusBottom;
        public double[] Size = new[] { 1.0, 1.0, 1.0 };
        public int Mesh = -1;
    }

    private sealed class BodyAcc
    {
        public int Node = -1;
        public string Name = "";
        public string Type = "static";
        public Vector3D Pos;
        public QuaternionD Rot = new(0, 0, 0, 1);
        public Matrix3x3D RotM = new(1, 0, 0, 0, 1, 0, 0, 0, 1);
        public Vector3D LinVel, AngVel;
        public double? Mass;
        public Vector3D? Com, InertiaDiag;
        public QuaternionD InertiaRot = new(0, 0, 0, 1);
        public readonly List<SceneFormat.ShapeDef> Shapes = new();
    }

    private sealed class Reader
    {
        private readonly Js _root;
        private readonly GltfOptions _opt;
        private readonly byte[]? _glbBin;
        private readonly List<InterchangeWarning> _warn = new();
        private readonly List<ShapeDesc?> _khrShapes = new();
        private readonly List<ShapeDesc?> _omiShapes = new();
        private readonly Dictionary<int, byte[]> _buffers = new();
        private Js? _nodes, _meshes, _accessors, _views, _bufferArr;
        private readonly List<(double Friction, double Restitution)> _materials = new();
        private Js? _joints;

        public Reader(Js root, GltfOptions opt, byte[]? glbBin) { _root = root; _opt = opt; _glbBin = glbBin; }

        private void Warn(string code, string where, string msg) => _warn.Add(new InterchangeWarning(code, where, msg));

        public GltfPhysicsModel Run()
        {
            if (!_root.IsObject) { throw _root.Fail("the document must be a JSON object."); }
            var asset = _root.Get("asset");
            if (asset != null && asset.Value.Str("version") is { } ver && !ver.StartsWith("2.", StringComparison.Ordinal)) { throw asset.Value.Fail("only glTF 2.x is supported, found '" + Io.Clip(ver) + "'."); }
            _nodes = _root.ArrOpt("nodes"); _meshes = _root.ArrOpt("meshes"); _accessors = _root.ArrOpt("accessors"); _views = _root.ArrOpt("bufferViews"); _bufferArr = _root.ArrOpt("buffers");
            int nodeCount = _nodes?.Length ?? 0;
            if (nodeCount > _opt.MaxNodes) { throw _nodes!.Value.Fail("more than " + _opt.MaxNodes + " nodes."); }
            var ext = _root.ObjOpt("extensions");
            ReadShapeLists(ext);
            ReadMaterialsAndJoints(ext);

            // World transforms by an explicit-stack walk of the chosen scene.
            var world = new Affine[nodeCount];
            var parent = new int[nodeCount];
            var seen = new bool[nodeCount];
            var order = new List<int>();
            Array.Fill(parent, -1);
            var roots = SceneRoots(nodeCount);
            var stack = new Stack<int>();
            for (int r = roots.Count - 1; r >= 0; r--) { stack.Push(roots[r]); world[roots[r]] = Local(roots[r]); }
            while (stack.Count > 0)
            {
                int n = stack.Pop();
                if (seen[n]) { throw _nodes!.Value.At(n).Fail("the node is reached twice; a node may have only one parent and the node tree must not loop."); }
                seen[n] = true; order.Add(n);
                var kids = _nodes!.Value.At(n).ArrOpt("children");
                if (kids == null) { continue; }
                for (int k = kids.Value.Length - 1; k >= 0; k--)
                {
                    int c = NodeIndex(kids.Value.At(k), nodeCount);
                    if (parent[c] >= 0 || seen[c]) { throw kids.Value.At(k).Fail("the node already has a parent."); }
                    parent[c] = n;
                    world[c] = world[n] * Local(c);
                    stack.Push(c);
                }
            }

            // Bodies: the nearest ancestor (or self) with motion owns each collider; a collider with no such ancestor makes a static body of its own.
            var bodies = new List<BodyAcc>();
            var owner = new int[nodeCount];     // body index of a node's own body, or -1
            Array.Fill(owner, -1);
            var bodyOf = new int[nodeCount];    // body a node belongs to, -1 when none yet
            Array.Fill(bodyOf, -1);
            foreach (int n in order)
            {
                var node = _nodes!.Value.At(n);
                var kh = node.ObjOpt("extensions") is { } ne ? ne.ObjOpt("KHR_physics_rigid_bodies") : null;
                var om = node.ObjOpt("extensions") is { } ne2 ? ne2.ObjOpt("OMI_physics_body") : null;
                var spec = kh ?? om;
                int inherited = parent[n] >= 0 ? bodyOf[parent[n]] : -1;
                bodyOf[n] = inherited;
                bool hasMotion = spec?.ObjOpt("motion") != null;
                bool hasCollider = spec?.ObjOpt("collider") != null;
                bool hasTrigger = spec?.ObjOpt("trigger") != null;
                if (spec == null) { continue; }
                if (hasMotion)
                {
                    var b = NewBody(n, world[n], node);
                    ReadMotion(b, spec.Value.Obj("motion"), kh != null);
                    owner[n] = bodies.Count; bodyOf[n] = bodies.Count;
                    bodies.Add(b);
                }
                else if (hasCollider && bodyOf[n] < 0)
                {
                    var b = NewBody(n, world[n], node);
                    owner[n] = bodies.Count; bodyOf[n] = bodies.Count;
                    bodies.Add(b);
                }

                if (hasTrigger) { Warn("trigger", node.Path, "triggers (sensor volumes) are not part of the scene format and are left out."); }
                if (hasCollider)
                {
                    var b = bodies[bodyOf[n]];
                    AddCollider(b, spec.Value.Obj("collider"), world[n], kh != null);
                }
            }

            // Joints.
            var joints = new List<(int A, int B, double[] AnchorA, double[] AnchorB, string Kind, double[] Axis, bool HasLimit, double Lo, double Hi)>();
            int worldAnchor = -1;
            int BodyForNode(int n)
            {
                if (bodyOf[n] >= 0) { return bodyOf[n]; }
                if (worldAnchor < 0)
                {
                    worldAnchor = bodies.Count;
                    bodies.Add(new BodyAcc { Node = -1, Name = "" });
                }

                return worldAnchor;
            }

            foreach (int n in order)
            {
                var node = _nodes!.Value.At(n);
                var kh = node.ObjOpt("extensions") is { } ne ? ne.ObjOpt("KHR_physics_rigid_bodies") : null;
                var jn = kh?.ObjOpt("joint");
                if (jn == null) { continue; }
                int other = NodeIndex(jn.Value.Need("connectedNode"), nodeCount);
                if (!seen[other]) { throw jn.Value.Fail("the connected node is not in the scene."); }
                int ji = jn.Value.Need("joint").Int();
                if (_joints == null || ji < 0 || ji >= _joints.Value.Length) { throw jn.Value.Need("joint").Fail("no physics joint with this index."); }
                if (jn.Value.Bool("enableCollision", false)) { Warn("joint-collision", jn.Value.Path, "enableCollision is not mapped: bodies joined by a joint never collide in a scene."); }
                int ba = BodyForNode(n), bb = BodyForNode(other);
                if (ba == bb) { Warn("joint-same-body", jn.Value.Path, "both nodes belong to the same body; the joint is left out."); continue; }
                ClassifyJoint(_joints.Value.At(ji), n, other, ba, bb, bodies, world, joints);
            }

            return Build(bodies, joints);
        }

        private List<int> SceneRoots(int nodeCount)
        {
            var roots = new List<int>();
            var scenes = _root.ArrOpt("scenes");
            if (scenes != null && scenes.Value.Length > 0)
            {
                int si = _root.Int("scene", 0);
                if (si < 0 || si >= scenes.Value.Length) { throw _root.Need("scene").Fail("no scene with this index."); }
                var ns = scenes.Value.At(si).ArrOpt("nodes");
                if (ns != null) { for (int i = 0; i < ns.Value.Length; i++) { roots.Add(NodeIndex(ns.Value.At(i), nodeCount)); } }
                return roots;
            }

            // No scenes: every node that is nobody's child.
            var isChild = new bool[nodeCount];
            for (int n = 0; n < nodeCount; n++)
            {
                var kids = _nodes!.Value.At(n).ArrOpt("children");
                if (kids == null) { continue; }
                for (int k = 0; k < kids.Value.Length; k++) { isChild[NodeIndex(kids.Value.At(k), nodeCount)] = true; }
            }

            for (int n = 0; n < nodeCount; n++) { if (!isChild[n]) { roots.Add(n); } }
            return roots;
        }

        private static int NodeIndex(Js v, int count)
        {
            int i = v.Int();
            if ((uint)i >= (uint)count) { throw v.Fail("no node with index " + i + "."); }
            return i;
        }

        private Affine Local(int n)
        {
            var node = _nodes!.Value.At(n);
            if (node.Get("matrix") is { } mj)
            {
                if (!mj.IsArray || mj.Length != 16) { throw mj.Fail("expected 16 numbers."); }
                var m = new double[16];
                for (int i = 0; i < 16; i++) { m[i] = mj.At(i).Num(); }
                // Column-major: columns are m[0..2], m[4..6], m[8..10]; translation m[12..14]. The last row must be 0 0 0 1 (no projection).
                if (m[3] != 0 || m[7] != 0 || m[11] != 0 || m[15] != 1) { throw mj.Fail("the matrix must be affine (last row 0 0 0 1)."); }
                return new Affine { L = new Matrix3x3D(m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]), T = new Vector3D(m[12], m[13], m[14]) };
            }

            var t = node.Nums("translation", 3) ?? new[] { 0.0, 0.0, 0.0 };
            var s = node.Nums("scale", 3) ?? new[] { 1.0, 1.0, 1.0 };
            var q = node.Nums("rotation", 4) ?? new[] { 0.0, 0.0, 0.0, 1.0 };
            double ql = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (!(ql > 1e-12)) { throw node.Fail("rotation must not be zero."); }
            var rot = new QuaternionD(q[0] / ql, q[1] / ql, q[2] / ql, q[3] / ql).ToMatrix();
            return new Affine { L = rot * Matrix3x3D.CreateScale(new Vector3D(s[0], s[1], s[2])), T = new Vector3D(t[0], t[1], t[2]) };
        }

        /// <summary>Splits a linear map into a proper rotation and per-axis scales (column lengths); a reflection is folded into the x scale's sign, which the sizes ignore.</summary>
        private static (Matrix3x3D R, Vector3D S) Decompose(Matrix3x3D l, Js where)
        {
            var c0 = l.Column(0); var c1 = l.Column(1); var c2 = l.Column(2);
            double s0 = c0.Length(), s1 = c1.Length(), s2 = c2.Length();
            if (!(s0 > 1e-12 && s1 > 1e-12 && s2 > 1e-12)) { throw where.Fail("the node's scale is zero on an axis; a collider needs a non-degenerate transform."); }
            var r = Matrix3x3D.FromColumns(c0 * (1 / s0), c1 * (1 / s1), c2 * (1 / s2));
            if (r.Determinant() < 0) { r = Matrix3x3D.FromColumns(c0 * (-1 / s0), c1 * (1 / s1), c2 * (1 / s2)); }
            return (r, new Vector3D(s0, s1, s2));
        }

        private static BodyAcc NewBody(int n, Affine w, Js node)
        {
            var (r, _) = Decompose(w.L, node);
            var b = new BodyAcc { Node = n, Name = node.Str("name") ?? "", Pos = w.T, RotM = r, Rot = Io.Normalize(QuaternionD.CreateFromRotationMatrix(r)) };
            return b;
        }

        private void ReadShapeLists(Js? ext)
        {
            if (ext == null) { return; }
            if (ext.Value.ObjOpt("KHR_implicit_shapes") is { } ks && ks.ArrOpt("shapes") is { } karr)
            {
                for (int i = 0; i < karr.Length; i++) { _khrShapes.Add(ReadShape(karr.At(i), true)); }
            }

            if (ext.Value.ObjOpt("OMI_physics_shape") is { } os && os.ArrOpt("shapes") is { } oarr)
            {
                for (int i = 0; i < oarr.Length; i++) { _omiShapes.Add(ReadShape(oarr.At(i), false)); }
            }
        }

        private ShapeDesc? ReadShape(Js s, bool khr)
        {
            if (!s.IsObject) { throw s.Fail("expected an object."); }
            string type = s.Str("type") ?? throw s.Fail("'type' is required.");
            var d = new ShapeDesc { Kind = type };
            var p = s.ObjOpt(type);
            switch (type)
            {
                case "sphere":
                    d.Radius = p?.Num("radius", 0.5) ?? 0.5;
                    if (!(d.Radius > 0)) { throw s.Fail("a sphere needs a radius above 0."); }
                    break;
                case "box":
                    d.Size = p?.Nums("size", 3) ?? new[] { 1.0, 1.0, 1.0 };
                    if (!(d.Size[0] > 0 && d.Size[1] > 0 && d.Size[2] > 0)) { throw s.Fail("a box needs a size above 0 on every axis."); }
                    break;
                case "capsule":
                case "cylinder":
                {
                    double r = khr ? 0.25 : 0.5;
                    d.Height = p?.Num("height", type == "capsule" ? (khr ? 0.5 : 1.0) : (khr ? 0.5 : 2.0)) ?? (khr ? 0.5 : (type == "capsule" ? 1.0 : 2.0));
                    d.RadiusTop = p?.Num("radiusTop", r) ?? r; d.RadiusBottom = p?.Num("radiusBottom", r) ?? r;
                    if (!(d.Height > 0) || d.RadiusTop < 0 || d.RadiusBottom < 0 || !(d.RadiusTop > 0 || d.RadiusBottom > 0)) { throw s.Fail("a " + type + " needs a height above 0, radii of at least 0 and one radius above 0."); }
                    break;
                }
                case "plane":
                    if (p != null && (p.Value.Get("sizeX") != null || p.Value.Get("sizeZ") != null)) { Warn("finite-plane", s.Path, "a plane with finite extents is imported as an infinite plane."); }
                    break;
                case "convex":
                case "trimesh":
                    d.Mesh = p?.Int("mesh", -1) ?? -1;
                    break;
                default:
                    Warn("unknown-shape", s.Path, "shape type '" + Io.Clip(type) + "' is not supported; colliders using it are left out.");
                    return null;
            }

            return d;
        }

        private void ReadMaterialsAndJoints(Js? ext)
        {
            _materials.Clear();
            var kh = ext?.ObjOpt("KHR_physics_rigid_bodies") ?? ext?.ObjOpt("OMI_physics_body");
            if (kh == null) { return; }
            if (kh.Value.ArrOpt("physicsMaterials") is { } mats)
            {
                for (int i = 0; i < mats.Length; i++)
                {
                    var m = mats.At(i);
                    double dyn = m.Num("dynamicFriction", m.Num("staticFriction", 0.6)), st = m.Num("staticFriction", dyn), rest = m.Num("restitution", 0);
                    if (dyn < 0 || st < 0 || rest < 0) { throw m.Fail("friction and restitution must not be negative."); }
                    if (Math.Abs(dyn - st) > 1e-9) { Warn("static-friction", m.Path, "static and dynamic friction differ; the dynamic value is used for both."); }
                    if (m.Get("frictionCombine") != null || m.Get("restitutionCombine") != null) { Warn("combine-rule", m.Path, "per-material combine rules are not mapped; the world's combine rule applies."); }
                    _materials.Add((dyn, rest));
                }
            }

            if (kh.Value.ArrOpt("collisionFilters") is { Length: > 0 } cf) { Warn("collision-filters", cf.Path, "collision filters are not imported; assign layers and masks to shapes after loading."); }
            _joints = kh.Value.ArrOpt("physicsJoints");
        }

        private static void ReadMotion(BodyAcc b, Js m, bool khr)
        {
            string type = khr ? (m.Bool("isKinematic", false) ? "kinematic" : "dynamic") : (m.Str("type") ?? "dynamic");
            if (type is not ("static" or "kinematic" or "dynamic")) { throw m.Fail("motion type must be static, kinematic or dynamic."); }
            b.Type = type;
            double? mass = m.Get("mass") is { } mv ? mv.Num() : null;
            if (mass < 0) { throw m.Need("mass").Fail("mass must not be negative."); }
            // A mass of zero stands for infinity: the body is moved by nothing, which is what a kinematic body is.
            if (mass == 0 && type == "dynamic") { b.Type = "kinematic"; }
            else if (mass > 0) { b.Mass = mass; }
            var lv = m.Nums("linearVelocity", 3); var av = m.Nums("angularVelocity", 3);
            if (lv != null) { b.LinVel = new Vector3D(lv[0], lv[1], lv[2]); }
            if (av != null) { b.AngVel = new Vector3D(av[0], av[1], av[2]); }
            if (m.Nums("centerOfMass", 3) is { } c) { b.Com = new Vector3D(c[0], c[1], c[2]); }
            if (m.Nums("inertiaDiagonal", 3) is { } d)
            {
                if (d[0] < 0 || d[1] < 0 || d[2] < 0) { throw m.Need("inertiaDiagonal").Fail("inertia must not be negative."); }
                if (d[0] > 0 && d[1] > 0 && d[2] > 0) { b.InertiaDiag = new Vector3D(d[0], d[1], d[2]); }
            }

            if (m.Nums("inertiaOrientation", 4) is { } q)
            {
                double l = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                if (!(l > 1e-12)) { throw m.Need("inertiaOrientation").Fail("the quaternion must not be zero."); }
                b.InertiaRot = new QuaternionD(q[0] / l, q[1] / l, q[2] / l, q[3] / l);
            }
        }

        private void AddCollider(BodyAcc body, Js collider, Affine nodeWorld, bool khr)
        {
            ShapeDesc? desc;
            bool convexHull = false;
            int meshIndex = -1;
            Js? geometry = collider.ObjOpt("geometry");
            Js where = geometry ?? collider;
            if (khr)
            {
                if (geometry == null) { throw collider.Fail("'geometry' is required."); }
                convexHull = geometry.Value.Bool("convexHull", false);
                if (geometry.Value.Get("shape") is { } sh)
                {
                    int i = sh.Int();
                    if ((uint)i >= (uint)_khrShapes.Count) { throw sh.Fail("no shape with index " + i + "."); }
                    desc = _khrShapes[i];
                    if (desc == null) { return; }
                    if (geometry.Value.Get("mesh") != null) { throw geometry.Value.Fail("give either shape or mesh, not both."); }
                }
                else if (geometry.Value.Get("mesh") is { } mj)
                {
                    meshIndex = mj.Int();
                    desc = new ShapeDesc { Kind = convexHull ? "convex" : "trimesh", Mesh = meshIndex };
                }
                else { throw geometry.Value.Fail("either shape or mesh is required."); }
            }
            else
            {
                var sh = collider.Need("shape");
                int i = sh.Int();
                if ((uint)i >= (uint)_omiShapes.Count) { throw sh.Fail("no shape with index " + i + "."); }
                desc = _omiShapes[i];
                if (desc == null) { return; }
                meshIndex = desc.Mesh;
                where = sh;
            }

            var (rotN, scale) = Decompose(nodeWorld.L, where);
            // The shape's frame in the body's frame: the body's rotation undone, scale kept (sizes take it below).
            var relT = body.RotM.Transposed() * (nodeWorld.T - body.Pos);
            var relR = body.RotM.Transposed() * rotN;
            var relQ = Io.Normalize(QuaternionD.CreateFromRotationMatrix(relR));
            int mat = 0;
            if (collider.Get("physicsMaterial") is { } pm)
            {
                int mi = pm.Int();
                if ((uint)mi >= (uint)_materials.Count) { throw pm.Fail("no physics material with index " + mi + "."); }
                mat = 1 + mi;
            }

            if (collider.Get("collisionFilter") != null) { Warn("collision-filter", collider.Path, "the collider's collision filter is not imported."); }
            bool identity = Math.Abs(relQ.X) < 1e-7 && Math.Abs(relQ.Y) < 1e-7 && Math.Abs(relQ.Z) < 1e-7;
            double sMax = Math.Max(scale.X, Math.Max(scale.Y, scale.Z));
            var sd = new SceneFormat.ShapeDef { Material = mat, Density = _opt.Density };
            switch (desc.Kind)
            {
                case "sphere":
                    if (Math.Abs(scale.X - scale.Y) > 1e-9 * sMax || Math.Abs(scale.X - scale.Z) > 1e-9 * sMax) { Warn("sphere-scale", where.Path, "a sphere under non-uniform scale takes the largest scale."); }
                    sd.Kind = "sphere"; sd.Radius = desc.Radius * sMax; sd.P = new[] { relT.X, relT.Y, relT.Z };
                    break;
                case "box":
                    sd.Kind = "box"; sd.P = new[] { 0.5 * desc.Size[0] * scale.X, 0.5 * desc.Size[1] * scale.Y, 0.5 * desc.Size[2] * scale.Z, relT.X, relT.Y, relT.Z, relQ.X, relQ.Y, relQ.Z, relQ.W };
                    break;
                case "capsule":
                {
                    double h = desc.Height * scale.Y, rt = desc.RadiusTop * Math.Max(scale.X, scale.Z), rb = desc.RadiusBottom * Math.Max(scale.X, scale.Z);
                    if (identity && Math.Abs(rt - rb) <= 1e-12 * Math.Max(rt, 1e-300))
                    {
                        sd.Kind = "capsule"; sd.Radius = rt; sd.P = new[] { relT.X, relT.Y - 0.5 * h, relT.Z, relT.X, relT.Y + 0.5 * h, relT.Z };
                    }
                    else
                    {
                        Warn("capsule-hull", where.Path, "a rotated or tapered capsule is imported as a convex hull of its end spheres.");
                        sd.Kind = "hull"; sd.P = CapsuleHull(h, rt, rb, relR, relT);
                    }

                    break;
                }
                case "cylinder":
                {
                    double h = desc.Height * scale.Y, rt = desc.RadiusTop * Math.Max(scale.X, scale.Z), rb = desc.RadiusBottom * Math.Max(scale.X, scale.Z);
                    bool atOrigin = identity && relT.X == 0 && relT.Y == 0 && relT.Z == 0;
                    if (atOrigin) { sd.Kind = "frustum"; sd.P = new[] { 0.5 * h, rt, rb }; }
                    else
                    {
                        Warn("cylinder-hull", where.Path, "a cylinder that is offset or rotated from its body is imported as a 24-sided prism (convex hull).");
                        sd.Kind = "hull"; sd.P = CylinderHull(h, rt, rb, relR, relT);
                    }

                    break;
                }
                case "plane":
                {
                    if (body.Type != "static") { Warn("plane-dynamic", where.Path, "a plane on a moving body is left out."); return; }
                    var n = relR * new Vector3D(0, 1, 0);
                    sd.Kind = "plane"; sd.P = new[] { n.X, n.Y, n.Z, n.X * relT.X + n.Y * relT.Y + n.Z * relT.Z };
                    break;
                }
                case "convex":
                case "trimesh":
                {
                    var (verts, tris) = ReadMesh(meshIndex, desc.Kind == "convex" || convexHull || body.Type != "static", where);
                    var pts = new double[verts.Length * 3];
                    for (int i = 0; i < verts.Length; i++)
                    {
                        var v = relR * new Vector3D(verts[i].X * scale.X, verts[i].Y * scale.Y, verts[i].Z * scale.Z) + relT;
                        pts[3 * i] = v.X; pts[3 * i + 1] = v.Y; pts[3 * i + 2] = v.Z;
                    }

                    if (desc.Kind == "trimesh" && !convexHull && body.Type == "static")
                    {
                        var p = new double[2 + pts.Length + tris.Length];
                        p[0] = verts.Length; p[1] = tris.Length;
                        Array.Copy(pts, 0, p, 2, pts.Length);
                        for (int i = 0; i < tris.Length; i++) { p[2 + pts.Length + i] = tris[i]; }
                        sd.Kind = "mesh"; sd.P = p;
                    }
                    else
                    {
                        if (desc.Kind == "trimesh" && !convexHull) { Warn("mesh-hull", where.Path, "a mesh collider on a moving body is imported as the convex hull of its vertices."); }
                        if (verts.Length < 4) { throw where.Fail("a convex hull needs at least four points."); }
                        sd.Kind = "hull"; sd.P = pts;
                    }

                    break;
                }
                default:
                    throw where.Fail("unsupported shape '" + desc.Kind + "'.");
            }

            body.Shapes.Add(sd);
        }

        private static double[] Ring(double[] acc, int start, double y, double r, int n, Matrix3x3D rot, Vector3D t, out int next)
        {
            next = start;
            for (int i = 0; i < n; i++)
            {
                var (s, c) = Math64.SinCos(2 * Math.PI * i / n);
                var v = rot * new Vector3D(r * c, y, r * s) + t;
                acc[next++] = v.X; acc[next++] = v.Y; acc[next++] = v.Z;
            }

            return acc;
        }

        private static double[] CylinderHull(double h, double rt, double rb, Matrix3x3D rot, Vector3D t)
        {
            const int n = 24;
            var acc = new double[3 * 2 * n];
            int k = 0;
            // A cone's apex ring has radius 0: all its points coincide, which the hull builder merges.
            Ring(acc, k, -0.5 * h, rb, n, rot, t, out k);
            Ring(acc, k, 0.5 * h, rt, n, rot, t, out _);
            return acc;
        }

        private static double[] CapsuleHull(double h, double rt, double rb, Matrix3x3D rot, Vector3D t)
        {
            const int n = 16;
            // Rings at the equators of both end spheres and two latitudes above each, plus the poles: the hull is inscribed in the capsule.
            var acc = new List<double>();
            void Cap(double centre, double r, int sign)
            {
                double[] lat = { 0.0, Math.PI / 6, Math.PI / 3 };
                foreach (double a in lat)
                {
                    var (s, c) = Math64.SinCos(a);
                    var ring = new double[3 * n];
                    Ring(ring, 0, centre + sign * r * s, r * c, n, rot, t, out _);
                    acc.AddRange(ring);
                }

                var pole = rot * new Vector3D(0, centre + sign * r, 0) + t;
                acc.Add(pole.X); acc.Add(pole.Y); acc.Add(pole.Z);
            }

            Cap(0.5 * h, rt, 1);
            Cap(-0.5 * h, rb, -1);
            return acc.ToArray();
        }

        private (Vector3F[] Vertices, int[] Triangles) ReadMesh(int mesh, bool hullOnly, Js where)
        {
            if (_meshes == null || (uint)mesh >= (uint)_meshes.Value.Length) { throw where.Fail("no mesh with index " + mesh + "."); }
            var m = _meshes.Value.At(mesh);
            var prims = m.ArrOpt("primitives") ?? throw m.Fail("'primitives' is required.");
            var verts = new List<Vector3F>();
            var tris = new List<int>();
            for (int p = 0; p < prims.Length; p++)
            {
                var prim = prims.At(p);
                if (prim.ObjOpt("extensions") is { } pe && (pe.Get("KHR_draco_mesh_compression") != null || pe.Get("EXT_meshopt_compression") != null)) { throw prim.Fail("compressed meshes are not supported as colliders."); }
                var attr = prim.Obj("attributes");
                int mode = prim.Int("mode", 4);
                var pos = ReadPositions(attr.Need("POSITION"));
                int baseIndex = verts.Count;
                if (verts.Count + pos.Length > 4_000_000) { throw prim.Fail("the mesh has more than 4 million vertices."); }
                verts.AddRange(pos);
                if (hullOnly) { continue; }
                if (mode is not (4 or 5 or 6)) { continue; }
                int[] idx = prim.Get("indices") is { } ia ? ReadIndices(ia) : Sequence(pos.Length);
                foreach (int i in idx) { if ((uint)i >= (uint)pos.Length) { throw prim.Fail("index " + i + " is outside the " + pos.Length + " vertices."); } }
                if (mode == 4)
                {
                    for (int i = 0; i + 2 < idx.Length; i += 3) { tris.Add(baseIndex + idx[i]); tris.Add(baseIndex + idx[i + 1]); tris.Add(baseIndex + idx[i + 2]); }
                }
                else if (mode == 5)
                {
                    for (int i = 0; i + 2 < idx.Length; i++)
                    {
                        // Alternate triangles are wound the other way so all face the same side.
                        if ((i & 1) == 0) { tris.Add(baseIndex + idx[i]); tris.Add(baseIndex + idx[i + 1]); tris.Add(baseIndex + idx[i + 2]); }
                        else { tris.Add(baseIndex + idx[i + 1]); tris.Add(baseIndex + idx[i]); tris.Add(baseIndex + idx[i + 2]); }
                    }
                }
                else
                {
                    for (int i = 1; i + 1 < idx.Length; i++) { tris.Add(baseIndex + idx[0]); tris.Add(baseIndex + idx[i]); tris.Add(baseIndex + idx[i + 1]); }
                }
            }

            if (verts.Count == 0) { throw m.Fail("the mesh has no vertices."); }
            if (!hullOnly && tris.Count == 0) { throw m.Fail("the mesh has no triangles."); }
            return (verts.ToArray(), tris.ToArray());
        }

        private static int[] Sequence(int n)
        {
            var r = new int[n];
            for (int i = 0; i < n; i++) { r[i] = i; }
            return r;
        }

        private byte[] Buffer(int index)
        {
            if (_buffers.TryGetValue(index, out var cached)) { return cached; }
            if (_bufferArr == null || (uint)index >= (uint)_bufferArr.Value.Length) { throw (_bufferArr ?? _root).Fail("no buffer with index " + index + "."); }
            var b = _bufferArr.Value.At(index);
            long len = b.Need("byteLength").Int();
            if (len < 0 || len > 256L * 1024 * 1024) { throw b.Fail("byteLength out of range."); }
            byte[] data;
            string? uri = b.Str("uri");
            if (uri == null)
            {
                if (index != 0 || _glbBin == null) { throw b.Fail("the buffer has no uri and there is no GLB binary chunk for it."); }
                data = _glbBin;
            }
            else if (uri.StartsWith("data:", StringComparison.Ordinal))
            {
                int comma = uri.IndexOf(',', StringComparison.Ordinal);
                if (comma < 0 || !uri.AsSpan(0, comma).EndsWith(";base64", StringComparison.Ordinal)) { throw b.Fail("only base64 data URIs are supported."); }
                try { data = Convert.FromBase64String(uri.Substring(comma + 1)); }
                catch (FormatException) { throw b.Fail("the data URI is not valid base64."); }
            }
            else
            {
                data = _opt.Buffers?.Invoke(uri) ?? throw b.Fail("the buffer file '" + Io.Clip(uri) + "' was not provided (GltfOptions.Buffers).");
            }

            if (data.Length < len) { throw b.Fail("the buffer holds " + data.Length + " bytes but byteLength is " + len + "."); }
            _buffers[index] = data;
            return data;
        }

        /// <summary>The byte range an accessor reads: the buffer, the offset of its first element, the stride, the element count and size, all checked against the buffer.</summary>
        private (byte[] Data, long Start, long Stride, int Count) Accessor(Js acc, string type, int componentType, int componentSize, int components)
        {
            var a = Resolve(acc, _accessors, "accessor");
            if (a.Get("sparse") != null) { throw a.Fail("sparse accessors are not supported."); }
            if (a.Str("type") != type) { throw a.Fail("expected type " + type + "."); }
            if (a.Int("componentType", 0) != componentType) { throw a.Fail("expected componentType " + componentType + "."); }
            int count = a.Need("count").Int();
            if (count < 0 || count > 16_000_000) { throw a.Need("count").Fail("count out of range."); }
            var viewJs = a.Get("bufferView") ?? throw a.Fail("an accessor without a bufferView is not supported.");
            var view = Resolve(viewJs, _views, "bufferView");
            var data = Buffer(view.Need("buffer").Int());
            long viewOffset = view.Int("byteOffset", 0), viewLength = view.Need("byteLength").Int();
            int stride = view.Int("byteStride", 0);
            long elem = (long)componentSize * components;
            if (stride == 0) { stride = (int)elem; }
            if (stride < elem || stride > 252) { throw view.Fail("byteStride out of range."); }
            long accOffset = a.Int("byteOffset", 0);
            if (viewOffset < 0 || viewLength < 0 || accOffset < 0 || viewOffset + viewLength > data.Length) { throw view.Fail("the buffer view lies outside its buffer."); }
            long need = count == 0 ? 0 : accOffset + (long)(count - 1) * stride + elem;
            if (need > viewLength) { throw a.Fail("the accessor reads " + need + " bytes but its buffer view has " + viewLength + "."); }
            return (data, viewOffset + accOffset, stride, count);
        }

        private static Js Resolve(Js index, Js? array, string what)
        {
            int i = index.Int();
            if (array == null || (uint)i >= (uint)array.Value.Length) { throw index.Fail("no " + what + " with index " + i + "."); }
            return array.Value.At(i);
        }

        private Vector3F[] ReadPositions(Js acc)
        {
            var (data, start, stride, count) = Accessor(acc, "VEC3", 5126, 4, 3);
            var r = new Vector3F[count];
            for (int i = 0; i < count; i++)
            {
                var s = data.AsSpan((int)(start + (long)i * stride), 12);
                float x = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s)), y = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s[4..])), z = BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(s[8..]));
                if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z)) { throw acc.Fail("vertex " + i + " is not finite."); }
                r[i] = new Vector3F(x, y, z);
            }

            return r;
        }

        private int[] ReadIndices(Js acc)
        {
            var a = Resolve(acc, _accessors, "accessor");
            int ct = a.Int("componentType", 0);
            int size = ct switch { 5121 => 1, 5123 => 2, 5125 => 4, _ => throw a.Fail("index componentType must be 5121, 5123 or 5125.") };
            var (data, start, stride, count) = Accessor(acc, "SCALAR", ct, size, 1);
            var r = new int[count];
            for (int i = 0; i < count; i++)
            {
                var s = data.AsSpan((int)(start + (long)i * stride), size);
                uint v = size == 1 ? s[0] : size == 2 ? BinaryPrimitives.ReadUInt16LittleEndian(s) : BinaryPrimitives.ReadUInt32LittleEndian(s);
                if (v > int.MaxValue) { throw acc.Fail("index " + i + " is out of range."); }
                r[i] = (int)v;
            }

            return r;
        }

        private sealed class AxisLimit { public double Lo = double.NegativeInfinity, Hi = double.PositiveInfinity; public bool Any; }

        private void ClassifyJoint(Js pj, int nodeA, int nodeB, int ba, int bb, List<BodyAcc> bodies, Affine[] world, List<(int, int, double[], double[], string, double[], bool, double, double)> output)
        {
            var lin = new AxisLimit[3]; var ang = new AxisLimit[3];
            for (int i = 0; i < 3; i++) { lin[i] = new AxisLimit(); ang[i] = new AxisLimit(); }
            if (pj.ArrOpt("limits") is { } limits)
            {
                for (int i = 0; i < limits.Length; i++)
                {
                    var l = limits.At(i);
                    var la = l.ArrOpt("linearAxes"); var aa = l.ArrOpt("angularAxes");
                    if ((la == null) == (aa == null)) { throw l.Fail("give exactly one of linearAxes and angularAxes."); }
                    var set = la ?? aa!.Value;
                    double lo = l.Num("min", double.NegativeInfinity), hi = l.Num("max", double.PositiveInfinity);
                    if (lo > hi) { throw l.Fail("min is above max."); }
                    if (l.Get("stiffness") != null) { Warn("soft-limit", l.Path, "soft joint limits are imported as hard limits."); }
                    var target = la != null ? lin : ang;
                    for (int k = 0; k < set.Length; k++)
                    {
                        int axis = set.At(k).Int();
                        if ((uint)axis > 2) { throw set.At(k).Fail("axes are 0, 1 or 2."); }
                        target[axis].Lo = Math.Max(target[axis].Lo, lo); target[axis].Hi = Math.Min(target[axis].Hi, hi); target[axis].Any = true;
                    }
                }
            }

            if (pj.ArrOpt("drives") is { Length: > 0 } dr) { Warn("joint-drive", dr.Path, "joint drives are not imported; drive the joint through the world API."); }
            bool linLocked = lin[0].Any && lin[1].Any && lin[2].Any && lin[0].Lo == 0 && lin[0].Hi == 0 && lin[1].Lo == 0 && lin[1].Hi == 0 && lin[2].Lo == 0 && lin[2].Hi == 0;
            if (!linLocked) { Warn("joint-linear", pj.Path, "only joints that lock all three linear axes (point, hinge) are mapped; the joint is left out."); return; }
            var A = bodies[ba]; var B = bodies[bb];
            Vector3D Anchor(BodyAcc b, int node) => b.RotM.Transposed() * (world[node].T - b.Pos);
            var anchorA = Anchor(A, nodeA); var anchorB = Anchor(B, nodeB);
            // Anchors are the joint node positions; bodies keep their scene poses.
            var (rotA, _) = Decompose(world[nodeA].L, pj);
            int freeAxes = 0, free = -1;
            for (int i = 0; i < 3; i++)
            {
                bool locked = ang[i].Any && ang[i].Lo == 0 && ang[i].Hi == 0;
                if (!locked) { freeAxes++; free = i; }
            }

            double[] aA = { anchorA.X, anchorA.Y, anchorA.Z }, aB = { anchorB.X, anchorB.Y, anchorB.Z };
            if (freeAxes == 3 && !ang[0].Any && !ang[1].Any && !ang[2].Any) { output.Add((ba, bb, aA, aB, "point", new double[3], false, 0, 0)); return; }
            if (freeAxes == 1)
            {
                var axisWorld = rotA.Column(free);
                bool limited = ang[free].Any && (double.IsFinite(ang[free].Lo) || double.IsFinite(ang[free].Hi));
                double lo = double.IsFinite(ang[free].Lo) ? ang[free].Lo : -Math.PI, hi = double.IsFinite(ang[free].Hi) ? ang[free].Hi : Math.PI;
                output.Add((ba, bb, aA, aB, "hinge", new[] { axisWorld.X, axisWorld.Y, axisWorld.Z }, limited, lo, hi));
                return;
            }

            Warn("joint-angular", pj.Path, "the angular limits describe neither a ball (no limit) nor a hinge (one free axis); the joint is left out.");
        }

        private GltfPhysicsModel Build(List<BodyAcc> bodies, List<(int A, int B, double[] AnchorA, double[] AnchorB, string Kind, double[] Axis, bool HasLimit, double Lo, double Hi)> joints)
        {
            var scene = new SceneFormat.Scene { Dim = 3, Gx = 0, Gy = -9.81, Gz = 0 };
            scene.Materials.Add((0.6, 0.0));
            foreach (var m in _materials) { scene.Materials.Add(m); }
            var nodes = new List<int>(); var names = new List<string>();
            foreach (var b in bodies)
            {
                var bd = new SceneFormat.BodyDef { Type = b.Type, X = b.Pos.X, Y = b.Pos.Y, Z = b.Pos.Z, Qx = b.Rot.X, Qy = b.Rot.Y, Qz = b.Rot.Z, Qw = b.Rot.W };
                // Velocities are given in node space; the scene wants world axes.
                var lv = b.RotM * b.LinVel; var av = b.RotM * b.AngVel;
                bd.Vx = lv.X; bd.Vy = lv.Y; bd.Vz = lv.Z; bd.Wx = av.X; bd.Wy = av.Y; bd.W = av.Z;
                if (b.Mass is double ms) { bd.MassOverride = (float)ms; }
                if (b.Com is Vector3D c) { bd.CenterOverride = Io.ToF(c); }
                if (b.InertiaDiag is Vector3D d) { bd.InertiaOverride = Io.ToF(d); bd.InertiaRotation = Io.ToF(b.InertiaRot); }
                bd.Shapes.AddRange(b.Shapes);
                if (b.Type != "static" && b.Shapes.Count == 0) { Warn("body-no-shape", b.Name, "a moving body has no collider and cannot be simulated; it is left out of the scene."); continue; }
                scene.Bodies.Add(bd); nodes.Add(b.Node); names.Add(b.Name);
            }

            // Bodies dropped above shift later indices; joints are mapped through the kept order.
            var map = new Dictionary<int, int>();
            int kept = 0;
            for (int i = 0; i < bodies.Count; i++)
            {
                var b = bodies[i];
                if (b.Type != "static" && b.Shapes.Count == 0) { continue; }
                map[i] = kept++;
            }

            foreach (var j in joints)
            {
                if (!map.TryGetValue(j.A, out int a) || !map.TryGetValue(j.B, out int b)) { Warn("joint-body", "joint", "a joint refers to a body that was left out."); continue; }
                var jd = new SceneFormat.JointDef { Kind = j.Kind == "point" ? "point" : "swingtwist", A = a, B = b, Ax = j.AnchorA[0], Ay = j.AnchorA[1], Az = j.AnchorA[2], Bx = j.AnchorB[0], By = j.AnchorB[1], Bz = j.AnchorB[2] };
                if (j.Kind == "hinge")
                {
                    jd.Ux = j.Axis[0]; jd.Uy = j.Axis[1]; jd.Uz = j.Axis[2];
                    jd.HasLimit = j.HasLimit; jd.Lower = j.Lo; jd.Upper = j.Hi;
                    // The twist joint's two swing angles are locked to a hairline for a hinge.
                    jd.HasSwing = true; jd.SwingY = 1e-4; jd.SwingZ = 1e-4;
                }

                scene.Joints.Add(jd);
            }

            return new GltfPhysicsModel(scene, nodes, names, _warn);
        }
    }
}
