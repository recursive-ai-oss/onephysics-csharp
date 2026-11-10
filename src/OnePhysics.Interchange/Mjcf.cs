using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Settings for <see cref="MjcfImporter"/>.</summary>
public sealed class MjcfOptions
{
    /// <summary>Finds the meshes of the asset section by their file name. Without one, mesh geoms are left out with a warning.</summary>
    public MeshResolver? Meshes { get; set; }

    /// <summary>Mass of the links that only place a joint (a body with two joints has one such link per extra joint), kg, above 0. Default 1e-3.</summary>
    public float MasslessLinkMass { get; set; } = 1e-3f;
}

/// <summary>A static shape the model's world body carries (a floor plane, fixed obstacles), for the host to attach to a static body.</summary>
public sealed class MjcfWorldShape
{
    internal MjcfWorldShape(string name, ShapeSpec3D shape, Vector3D position, QuaternionF rotation, float friction) { Name = name; Shape = shape; Position = position; Rotation = rotation; Friction = friction; }

    /// <summary>The geom's name, or an empty string.</summary>
    public string Name { get; }

    /// <summary>The geometry. A plane already has its pose folded into the plane equation.</summary>
    public ShapeSpec3D Shape { get; }

    /// <summary>World position of the geom's frame (zero for a plane).</summary>
    public Vector3D Position { get; }

    /// <summary>World orientation of the geom's frame (identity for a plane).</summary>
    public QuaternionF Rotation { get; }

    /// <summary>Sliding friction coefficient.</summary>
    public float Friction { get; }
}

/// <summary>The result of importing an MJCF file.</summary>
public sealed class MjcfModel
{
    internal MjcfModel(string name, ArticulationDef def, List<string> links, List<string?> joints, List<MjcfWorldShape> world, Vector3D gravity, double timestep, List<InterchangeWarning> warnings)
    {
        ModelName = name; Definition = def; LinkNames = links; JointNames = joints; WorldShapes = world; Gravity = gravity; TimeStep = timestep; Warnings = warnings;
    }

    /// <summary>The model's name.</summary>
    public string ModelName { get; }

    /// <summary>The articulation of the one body tree under the world body: a free joint on its top body makes a floating base, anything else a base fixed in the world.</summary>
    public ArticulationDef Definition { get; }

    /// <summary>The body name of each link of <see cref="Definition"/>, by index (empty for an unnamed body; links that only place an extra joint carry their joint's name).</summary>
    public IReadOnlyList<string> LinkNames { get; }

    /// <summary>The name of the joint that moves each link, by link index; null for the root and for welded bodies; empty for an unnamed joint.</summary>
    public IReadOnlyList<string?> JointNames { get; }

    /// <summary>Static shapes of the world body.</summary>
    public IReadOnlyList<MjcfWorldShape> WorldShapes { get; }

    /// <summary>The option element's gravity, default (0, 0, -9.81): MJCF models stand on z, so a host that keeps y up rotates the model or sets this gravity.</summary>
    public Vector3D Gravity { get; }

    /// <summary>The option element's timestep, seconds, default 0.002; not used by the importer.</summary>
    public double TimeStep { get; }

    /// <summary>Everything the importer left out or approximated.</summary>
    public IReadOnlyList<InterchangeWarning> Warnings { get; }

    /// <summary>The index of the first link with a body name, or -1.</summary>
    public int FindLink(string name) => ((List<string>)LinkNames).IndexOf(name);

    /// <summary>The index of the link moved by a named joint, or -1.</summary>
    public int FindJoint(string name) => ((List<string?>)JointNames).IndexOf(name);
}

/// <summary>
/// Reads MJCF models (the MuJoCo XML format, as its reference documentation describes it) into <see cref="ArticulationDef"/>s: nested bodies, hinge, slide, ball and free joints (several per body become a chain of links), geoms
/// (plane, sphere, capsule, cylinder, box, ellipsoid, mesh), default classes, the compiler's angle unit and Euler sequence, inertials or inertia computed from geoms, and motor, position and velocity actuators as joint drives.
/// Ellipsoids collide as a 42-vertex convex hull inscribed in the ellipsoid; masses use the exact ellipsoid. Contact filtering attributes, tendons, equality constraints, sensors, sites and visual settings are not imported.
/// </summary>
public static class MjcfImporter
{
    private const string F = "MJCF";

    /// <summary>Imports an MJCF document.</summary>
    /// <exception cref="InterchangeException">The document is malformed or invalid (an unknown joint in an actuator, a body that is not a tree, a bad number or enumeration); the message names the element.</exception>
    public static MjcfModel Import(string xml, MjcfOptions? options = null)
    {
        options ??= new MjcfOptions();
        if (!(options.MasslessLinkMass > 0f) || !float.IsFinite(options.MasslessLinkMass)) { throw new ArgumentOutOfRangeException(nameof(options), "MasslessLinkMass must be finite and above 0."); }
        var doc = Io.LoadXml(xml, F);
        var root = doc.Root;
        if (root == null || root.Name.LocalName != "mujoco") { throw new InterchangeException(F, "document", "the root element must be <mujoco>."); }
        var imp = new Reader(root, options);
        return imp.Run();
    }

    /// <summary>Imports an MJCF document from a stream of UTF-8 text.</summary>
    public static MjcfModel Import(Stream stream, MjcfOptions? options = null) => Import(Io.ReadAll(stream, F, Io.MaxChars), options);

    private sealed class Defaults
    {
        public string? Parent;
        public readonly Dictionary<string, XElement> Kinds = new(StringComparer.Ordinal);
    }

    private sealed class Reader
    {
        private static readonly string[] OrientationAttributes = { "quat", "axisangle", "euler", "xyaxes", "zaxis" };
        private static readonly string[] Structural = { "frame", "composite", "flexcomp", "replicate", "attach" };
        private static readonly string[] BodyOnly = { "joint", "freejoint", "inertial" };
        private static readonly string[] Unsupported = { "contact", "equality", "tendon", "sensor", "keyframe", "include", "custom", "extension", "deformable" };
        private static readonly string[] CompilerChanges = { "boundmass", "boundinertia", "settotalmass", "fusestatic", "balanceinertia" };
        private readonly XElement _root;
        private readonly MjcfOptions _opt;
        private readonly List<InterchangeWarning> _warn = new();
        private readonly Dictionary<string, Defaults> _classes = new(StringComparer.Ordinal);
        private readonly Dictionary<string, XElement> _meshAssets = new(StringComparer.Ordinal);
        private bool _degrees = true, _autolimits = true;
        private string _eulerSeq = "xyz";
        private string _inertiaFromGeom = "auto";
        private readonly ArticulationDef _def = new();
        private readonly List<string> _linkNames = new();
        private readonly List<string?> _jointNames = new();
        private readonly Dictionary<string, int> _jointLink = new(StringComparer.Ordinal);
        private readonly List<MjcfWorldShape> _world = new();
        private bool _hasRoot;

        public Reader(XElement root, MjcfOptions opt) { _root = root; _opt = opt; }

        public MjcfModel Run()
        {
            ReadCompiler();
            foreach (var d in _root.Elements("default")) { ReadDefaults(d, null, 0); }
            if (!_classes.ContainsKey("main")) { _classes["main"] = new Defaults(); }
            foreach (var a in _root.Elements("asset")) { foreach (var m in a.Elements("mesh")) { if (m.Attribute("name") is { } n) { _meshAssets[n.Value] = m; } } }
            foreach (var n in Unsupported)
            {
                if (_root.Element(n) != null) { Warn("unsupported-" + n, n, "<" + n + "> is not imported."); }
            }

            var gravity = new Vector3D(0, 0, -9.81);
            double timestep = 0.002;
            if (_root.Element("option") is { } o)
            {
                if (o.Attribute("gravity") is { } g) { var v = Io.Numbers(g.Value, 3, F, Io.Where(o), "gravity"); gravity = new Vector3D(v[0], v[1], v[2]); }
                if (o.Attribute("timestep") is { } t) { timestep = Io.Number(t.Value, F, Io.Where(o), "timestep"); }
            }

            var world = _root.Element("worldbody");
            if (world == null) { throw new InterchangeException(F, "mujoco", "no <worldbody> element."); }
            ReadWorld(world);
            if (!_hasRoot) { throw new InterchangeException(F, Io.Where(world), "the world body has no body; there is nothing to articulate."); }
            foreach (var act in _root.Elements("actuator")) { ReadActuators(act); }
            return new MjcfModel((string?)_root.Attribute("model") ?? "", _def, _linkNames, _jointNames, _world, gravity, timestep, _warn);
        }

        private void Warn(string code, string where, string message) => _warn.Add(new InterchangeWarning(code, where, message));

        private void ReadCompiler()
        {
            var c = _root.Element("compiler");
            if (c == null) { return; }
            string w = Io.Where(c);
            if (c.Attribute("angle") is { } a)
            {
                if (a.Value is not ("degree" or "radian")) { throw new InterchangeException(F, w, "angle must be 'degree' or 'radian', found '" + Io.Clip(a.Value) + "'."); }
                _degrees = a.Value == "degree";
            }

            if (c.Attribute("eulerseq") is { } e)
            {
                if (e.Value.Length != 3 || e.Value.Any(ch => "xyzXYZ".IndexOf(ch) < 0)) { throw new InterchangeException(F, w, "eulerseq needs three letters from x, y, z (lower case: rotating axes, upper case: fixed axes), found '" + Io.Clip(e.Value) + "'."); }
                _eulerSeq = e.Value;
            }

            if (c.Attribute("inertiafromgeom") is { } i)
            {
                if (i.Value is not ("auto" or "true" or "false")) { throw new InterchangeException(F, w, "inertiafromgeom must be auto, true or false."); }
                _inertiaFromGeom = i.Value;
            }

            if (c.Attribute("autolimits") is { } al)
            {
                if (al.Value is not ("true" or "false")) { throw new InterchangeException(F, w, "autolimits must be true or false."); }
                _autolimits = al.Value == "true";
            }

            foreach (var n in CompilerChanges)
            {
                if (c.Attribute(n) != null) { Warn("compiler-" + n, w, "compiler attribute '" + n + "' changes masses or the body tree and is not applied."); }
            }
        }

        private void ReadDefaults(XElement d, string? parent, int depth)
        {
            if (depth > 64) { throw new InterchangeException(F, Io.Where(d), "default classes are nested more than 64 deep."); }
            string name = (string?)d.Attribute("class") ?? (parent == null ? "main" : throw new InterchangeException(F, Io.Where(d), "a nested <default> needs a class attribute."));
            if (_classes.ContainsKey(name)) { throw new InterchangeException(F, Io.Where(d), "default class '" + Io.Clip(name) + "' is defined twice."); }
            var cls = new Defaults { Parent = parent };
            _classes[name] = cls;
            foreach (var e in d.Elements())
            {
                if (e.Name.LocalName == "default") { continue; }
                cls.Kinds[e.Name.LocalName] = e;
            }

            foreach (var child in d.Elements("default")) { ReadDefaults(child, name, depth + 1); }
        }

        private string? Attr(XElement e, string attribute, string cls)
        {
            if (e.Attribute(attribute) is { } a) { return a.Value; }
            string? c = (string?)e.Attribute("class") ?? cls;
            string kind = e.Name.LocalName;
            for (int guard = 0; c != null && guard < 80; guard++)
            {
                if (!_classes.TryGetValue(c, out var d)) { throw new InterchangeException(F, Io.Where(e), "unknown default class '" + Io.Clip(c) + "'."); }
                if (d.Kinds.TryGetValue(kind, out var el) && el.Attribute(attribute) is { } da) { return da.Value; }
                c = d.Parent;
            }

            return null;
        }

        private double Angle(double v) => _degrees ? v * (Math.PI / 180.0) : v;

        private double Num(XElement e, string attribute, string cls, double fallback)
        {
            string? s = Attr(e, attribute, cls);
            return s == null ? fallback : Io.Number(s, F, Io.Where(e), attribute);
        }

        private double[]? Nums(XElement e, string attribute, string cls, int count)
        {
            string? s = Attr(e, attribute, cls);
            return s == null ? null : Io.Numbers(s, count, F, Io.Where(e), attribute);
        }

        private Vector3D Pos(XElement e, string cls)
        {
            var v = Nums(e, "pos", cls, 3);
            return v == null ? new Vector3D(0, 0, 0) : new Vector3D(v[0], v[1], v[2]);
        }

        /// <summary>The frame orientation attributes, in MuJoCo's order of precedence: quat (w x y z), axisangle, euler, xyaxes, zaxis.</summary>
        private QuaternionD Orientation(XElement e, string cls)
        {
            string w = Io.Where(e);
            int given = OrientationAttributes.Count(n => Attr(e, n, cls) != null);
            if (given > 1) { throw new InterchangeException(F, w, "give only one of quat, axisangle, euler, xyaxes and zaxis."); }
            if (Nums(e, "quat", cls, 4) is { } q)
            {
                double l = Math.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                if (!(l > 1e-12)) { throw new InterchangeException(F, w, "quat must not be zero."); }
                return new QuaternionD(q[1] / l, q[2] / l, q[3] / l, q[0] / l);
            }

            if (Nums(e, "axisangle", cls, 4) is { } aa)
            {
                if (!(aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2] > 1e-24)) { throw new InterchangeException(F, w, "the axis of axisangle must not be zero."); }
                return Io.AxisAngle(aa[0], aa[1], aa[2], Angle(aa[3]));
            }

            if (Nums(e, "euler", cls, 3) is { } eu)
            {
                var r = new QuaternionD(0, 0, 0, 1);
                for (int i = 0; i < 3; i++)
                {
                    char ch = _eulerSeq[i];
                    var step = Io.AxisAngle(char.ToLowerInvariant(ch) == 'x' ? 1 : 0, char.ToLowerInvariant(ch) == 'y' ? 1 : 0, char.ToLowerInvariant(ch) == 'z' ? 1 : 0, Angle(eu[i]));
                    // Lower case: the next rotation is about the axes already turned (multiply on the right); upper case: about the fixed axes (on the left).
                    r = char.IsLower(ch) ? r * step : step * r;
                }

                return Io.Normalize(r);
            }

            if (Nums(e, "xyaxes", cls, 6) is { } xy)
            {
                double lx = Math.Sqrt(xy[0] * xy[0] + xy[1] * xy[1] + xy[2] * xy[2]);
                if (!(lx > 1e-12)) { throw new InterchangeException(F, w, "the x axis of xyaxes must not be zero."); }
                double ax = xy[0] / lx, ay = xy[1] / lx, az = xy[2] / lx;
                double d = ax * xy[3] + ay * xy[4] + az * xy[5];
                double bx = xy[3] - d * ax, by = xy[4] - d * ay, bz = xy[5] - d * az;
                double lb = Math.Sqrt(bx * bx + by * by + bz * bz);
                if (!(lb > 1e-12)) { throw new InterchangeException(F, w, "the two axes of xyaxes must not be parallel."); }
                bx /= lb; by /= lb; bz /= lb;
                double cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
                return Io.Normalize(QuaternionD.CreateFromRotationMatrix(new Matrix3x3D(ax, bx, cx, ay, by, cy, az, bz, cz)));
            }

            if (Nums(e, "zaxis", cls, 3) is { } z)
            {
                double l = Math.Sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
                if (!(l > 1e-12)) { throw new InterchangeException(F, w, "zaxis must not be zero."); }
                return ShortestArc(z[0] / l, z[1] / l, z[2] / l);
            }

            return new QuaternionD(0, 0, 0, 1);
        }

        /// <summary>The shortest rotation taking +z to a unit vector.</summary>
        private static QuaternionD ShortestArc(double x, double y, double z)
        {
            if (z < -0.999999999999) { return new QuaternionD(1, 0, 0, 0); }
            // q = (axis * sin(a/2), cos(a/2)) with axis = z-hat x v: (-y, x, 0); the half-angle form avoids any trigonometry.
            var q = new QuaternionD(-y, x, 0, 1 + z);
            return Io.Normalize(q);
        }

        private void ReadWorld(XElement world)
        {
            string w = Io.Where(world);
            foreach (var g in world.Elements("geom")) { ReadWorldGeom(g); }
            foreach (var n in BodyOnly) { if (world.Element(n) != null) { throw new InterchangeException(F, w, "the world body cannot have a <" + n + ">."); } }
            var tops = world.Elements("body").ToList();
            if (tops.Count > 1) { throw new InterchangeException(F, Io.Where(tops[1]), "the world body holds " + tops.Count + " bodies; one articulation per file is imported (split the file, or weld the bodies under one body)."); }
            foreach (var n in Structural) { if (world.Element(n) != null) { Warn("unsupported-" + n, w, "<" + n + "> is not imported."); } }
            if (tops.Count == 1) { BuildBodies(tops[0], "main"); }
        }

        private (ShapeSpec3D Spec, Vector3D Pos, QuaternionD Rot, double Volume, Func<double, (Vector3D, Matrix3x3D)> Inertia)? Geom(XElement g, string cls, bool allowPlane, out bool isPlane)
        {
            isPlane = false;
            string w = Io.Where(g);
            string type = Attr(g, "type", cls) ?? "sphere";
            var pos = Pos(g, cls);
            var rot = Orientation(g, cls);
            double[] size = Attr(g, "size", cls) is { } sz ? SplitNumbers(sz, w, "size") : Array.Empty<double>();
            if (Attr(g, "fromto", cls) is { } ft)
            {
                if (type is not ("capsule" or "cylinder" or "box" or "ellipsoid")) { throw new InterchangeException(F, w, "fromto applies to capsule, cylinder, box and ellipsoid geoms, not '" + Io.Clip(type) + "'."); }
                var f = Io.Numbers(ft, 6, F, w, "fromto");
                double dx = f[3] - f[0], dy = f[4] - f[1], dz = f[5] - f[2], len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                if (!(len > 1e-12)) { throw new InterchangeException(F, w, "fromto needs two different points."); }
                pos = new Vector3D(0.5 * (f[0] + f[3]), 0.5 * (f[1] + f[4]), 0.5 * (f[2] + f[5]));
                rot = ShortestArc(dx / len, dy / len, dz / len);
                double radius = Need(size, 0, w, "size");
                size = type is "capsule" or "cylinder" ? new[] { radius, 0.5 * len } : new[] { radius, size.Length > 1 ? size[1] : radius, 0.5 * len };
            }

            double density = Num(g, "density", cls, 1000);
            double? massAttr = Attr(g, "mass", cls) is { } ms ? Io.Number(ms, F, w, "mass") : null;
            if (density <= 0 && massAttr == null) { throw new InterchangeException(F, w, "density must be above 0."); }
            if (massAttr is < 0) { throw new InterchangeException(F, w, "mass must not be negative."); }
            ShapeSpec3D spec;
            double volume;
            Func<double, (Vector3D, Matrix3x3D)> inertia;
            var origin = new Vector3D(0, 0, 0);
            switch (type)
            {
                case "plane":
                    if (!allowPlane) { Warn("plane-in-body", w, "planes are static; a plane geom inside a body is left out."); return null; }
                    isPlane = true;
                    return (default, pos, rot, 0, null!);
                case "sphere":
                {
                    double r = Positive(Need(size, 0, w, "size radius"), w, "sphere radius");
                    spec = ShapeSpec3D.Sphere((float)r); volume = MassMath.SphereVolume(r);
                    inertia = m => (origin, MassMath.Sphere(m, r));
                    break;
                }
                case "capsule":
                {
                    double r = Positive(Need(size, 0, w, "size radius"), w, "capsule radius"), h = Positive(Need(size, 1, w, "size half-length"), w, "capsule half-length");
                    // Our capsules lie along y, MJCF's along z.
                    spec = ShapeSpec3D.Capsule((float)h, (float)r); volume = MassMath.CapsuleVolume(r, h);
                    inertia = m => (origin, MassMath.CapsuleZ(m, r, h));
                    rot *= Io.AxisAngle(1, 0, 0, Math.PI / 2);
                    return Finish(spec, pos, rot, volume, inertia, true);
                }
                case "cylinder":
                {
                    double r = Positive(Need(size, 0, w, "size radius"), w, "cylinder radius"), h = Positive(Need(size, 1, w, "size half-length"), w, "cylinder half-length");
                    spec = ShapeSpec3D.Cylinder((float)h, (float)r); volume = MassMath.CylinderVolume(r, h);
                    inertia = m => (origin, MassMath.CylinderZ(m, r, h));
                    rot *= Io.AxisAngle(1, 0, 0, Math.PI / 2);
                    return Finish(spec, pos, rot, volume, inertia, true);
                }
                case "box":
                {
                    double hx = Positive(Need(size, 0, w, "size"), w, "box half-size"), hy = Positive(Need(size, 1, w, "size"), w, "box half-size"), hz = Positive(Need(size, 2, w, "size"), w, "box half-size");
                    spec = ShapeSpec3D.Box(new Vector3F((float)(2 * hx), (float)(2 * hy), (float)(2 * hz))); volume = 8 * hx * hy * hz;
                    inertia = m => (origin, MassMath.Box(m, hx, hy, hz));
                    break;
                }
                case "ellipsoid":
                {
                    double a = Positive(Need(size, 0, w, "size"), w, "ellipsoid radius"), b = Positive(Need(size, 1, w, "size"), w, "ellipsoid radius"), c = Positive(Need(size, 2, w, "size"), w, "ellipsoid radius");
                    spec = ShapeSpec3D.ConvexHull(EllipsoidPoints(a, b, c)); volume = MassMath.EllipsoidVolume(a, b, c);
                    inertia = m => (origin, MassMath.Ellipsoid(m, a, b, c));
                    break;
                }
                case "mesh":
                {
                    string? mn = Attr(g, "mesh", cls);
                    if (mn == null) { throw new InterchangeException(F, w, "a mesh geom needs a mesh attribute."); }
                    if (!_meshAssets.TryGetValue(mn, out var asset)) { throw new InterchangeException(F, w, "the mesh asset '" + Io.Clip(mn) + "' is not defined."); }
                    string? file = (string?)asset.Attribute("file");
                    var m = file == null ? null : _opt.Meshes?.Invoke(file);
                    if (m == null) { Warn("mesh-unresolved", w, "mesh '" + Io.Clip(file ?? mn) + "' was not provided by the mesh resolver; the geom is left out."); return null; }
                    var sc = asset.Attribute("scale") is { } sa ? Io.Numbers(sa.Value, 3, F, Io.Where(asset), "scale") : new[] { 1.0, 1.0, 1.0 };
                    spec = MeshHull.Create(m, sc[0], sc[1], sc[2], F, w);
                    if (!MassMath.Mesh(m.Vertices, m.Indices, sc[0], sc[1], sc[2], out volume, out var cen, out var iOrigin)) { throw new InterchangeException(F, w, "the mesh '" + Io.Clip(mn) + "' encloses no volume, so it has no mass; give the geom no mesh or the body an <inertial>."); }
                    inertia = mass =>
                    {
                        // Unit density: scale by mass / volume, then move the tensor from the origin to the centroid.
                        double k = mass / volume;
                        var about = new Matrix3x3D(iOrigin.M11 * k, iOrigin.M12 * k, iOrigin.M13 * k, iOrigin.M21 * k, iOrigin.M22 * k, iOrigin.M23 * k, iOrigin.M31 * k, iOrigin.M32 * k, iOrigin.M33 * k);
                        return (cen, InertiaD.ShiftToCenter(about, mass, cen));
                    };
                    break;
                }
                default:
                    throw new InterchangeException(F, w, "geom type must be plane, sphere, capsule, cylinder, box, ellipsoid or mesh, found '" + Io.Clip(type) + "'.");
            }

            return Finish(spec, pos, rot, volume, inertia, true);
            (ShapeSpec3D, Vector3D, QuaternionD, double, Func<double, (Vector3D, Matrix3x3D)>)? Finish(ShapeSpec3D s, Vector3D p, QuaternionD r, double v, Func<double, (Vector3D, Matrix3x3D)> i, bool _) => (s, p, r, v, i);
        }

        /// <summary>The sliding coefficient, the first of the one to three numbers MJCF allows; default 1.</summary>
        private double Friction(XElement g, string cls)
        {
            string? f = Attr(g, "friction", cls);
            if (f == null) { return 1.0; }
            var parts = f.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length is < 1 or > 3) { throw new InterchangeException(F, Io.Where(g), "friction takes one to three numbers."); }
            double v = Io.Number(parts[0], F, Io.Where(g), "friction");
            if (v < 0) { throw new InterchangeException(F, Io.Where(g), "friction must not be negative."); }
            return v;
        }

        private static double[] SplitNumbers(string s, string where, string what)
        {
            var parts = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length > 6) { throw new InterchangeException(F, where, what + " has more numbers than any geom type takes."); }
            var r = new double[parts.Length];
            for (int i = 0; i < r.Length; i++) { r[i] = Io.Number(parts[i], F, where, what); }
            return r;
        }

        private static double Need(double[] size, int i, string where, string what)
        {
            if (i >= size.Length) { throw new InterchangeException(F, where, what + " needs at least " + (i + 1) + " numbers."); }
            return size[i];
        }

        private static double Positive(double v, string where, string what)
        {
            if (!(v > 0)) { throw new InterchangeException(F, where, what + " must be above 0."); }
            return v;
        }

        /// <summary>A unit icosahedron with each edge split once and the new points pushed out to the sphere (42 vertices), scaled to the ellipsoid.</summary>
        private static Vector3F[] EllipsoidPoints(double a, double b, double c)
        {
            double t = (1.0 + Math.Sqrt(5.0)) / 2.0;
            var v = new List<Vector3D>
            {
                new(-1, t, 0), new(1, t, 0), new(-1, -t, 0), new(1, -t, 0), new(0, -1, t), new(0, 1, t), new(0, -1, -t), new(0, 1, -t), new(t, 0, -1), new(t, 0, 1), new(-t, 0, -1), new(-t, 0, 1),
            };
            for (int i = 0; i < v.Count; i++) { v[i] = Unit(v[i]); }
            // Edges are the pairs at the icosahedron's edge length 2 / sqrt(1 + t^2) on the unit sphere.
            double edge2 = 4.0 / (1.0 + t * t);
            int n = v.Count;
            for (int i = 0; i < n; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    var d = v[i] - v[j];
                    if (Math.Abs(d.X * d.X + d.Y * d.Y + d.Z * d.Z - edge2) < 1e-9) { v.Add(Unit(new Vector3D(v[i].X + v[j].X, v[i].Y + v[j].Y, v[i].Z + v[j].Z))); }
                }
            }

            var r = new Vector3F[v.Count];
            for (int i = 0; i < r.Length; i++) { r[i] = new Vector3F((float)(v[i].X * a), (float)(v[i].Y * b), (float)(v[i].Z * c)); }
            return r;
        }

        private static Vector3D Unit(Vector3D p) { double l = Math.Sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z); return new Vector3D(p.X / l, p.Y / l, p.Z / l); }

        private void ReadWorldGeom(XElement g)
        {
            var r = Geom(g, "main", true, out bool plane);
            if (r == null) { return; }
            var (spec, pos, rot, _, _) = r.Value;
            double friction = Friction(g, "main");
            if (plane)
            {
                var n = rot.Rotate(new Vector3D(0, 0, 1));
                spec = ShapeSpec3D.Plane(new Vector3F((float)n.X, (float)n.Y, (float)n.Z), (float)(n.X * pos.X + n.Y * pos.Y + n.Z * pos.Z));
                pos = new Vector3D(0, 0, 0); rot = new QuaternionD(0, 0, 0, 1);
            }

            _world.Add(new MjcfWorldShape((string?)g.Attribute("name") ?? "", spec, pos, Io.ToF(rot), (float)friction));
        }

        private sealed class JointInfo
        {
            public XElement El = null!;
            public string Type = "hinge";
            public Vector3D Pos;
            public string Cls = "main";
        }

        private void BuildBodies(XElement top, string topClass)
        {
            var stack = new Stack<(XElement Body, int ParentLink, Vector3D ParentAnchor, string Cls, int Depth)>();
            stack.Push((top, -1, new Vector3D(0, 0, 0), topClass, 0));
            while (stack.Count > 0)
            {
                var (body, parentLink, parentAnchor, inherited, depth) = stack.Pop();
                if (depth > Io.MaxDepth) { throw new InterchangeException(F, Io.Where(body), "bodies are nested more than " + Io.MaxDepth + " deep."); }
                string w = Io.Where(body);
                string cls = (string?)body.Attribute("childclass") ?? inherited;
                var bodyPos = Pos(body, cls);
                var bodyRot = Orientation(body, cls);
                var joints = new List<JointInfo>();
                foreach (var j in body.Elements())
                {
                    string n = j.Name.LocalName;
                    if (n == "joint") { joints.Add(new JointInfo { El = j, Type = Attr(j, "type", cls) ?? "hinge", Pos = Pos(j, cls), Cls = cls }); }
                    else if (n == "freejoint") { joints.Add(new JointInfo { El = j, Type = "free", Pos = new Vector3D(0, 0, 0), Cls = cls }); }
                }

                foreach (var n in Structural) { if (body.Element(n) != null) { Warn("unsupported-" + n, w, "<" + n + "> is not imported."); } }
                if (body.Attribute("mocap") is { Value: "true" }) { Warn("unsupported-mocap", w, "mocap bodies are not supported; the body is imported as an ordinary body."); }

                bool isRoot = parentLink < 0;
                _hasRoot |= isRoot;
                var anchor = new Vector3D(0, 0, 0);
                if (isRoot)
                {
                    bool free = joints.Count > 0 && joints[0].Type == "free";
                    if (joints.Count > 1 || (joints.Count == 1 && !free)) { throw new InterchangeException(F, w, "the top body takes either one free joint (a floating base) or none (a base fixed in the world); a hinge or slide at the base is not supported."); }
                    _def.FixedBase = !free;
                    _def.Position = bodyPos;
                    _def.Rotation = Io.ToF(bodyRot);
                    var link = new ArticulationLinkDef();
                    AddLink(link, (string?)body.Attribute("name") ?? "", null, w);
                    ApplyContents(link, body, cls, anchor);
                }
                else
                {
                    if (joints.Any(j => j.Type == "free")) { throw new InterchangeException(F, w, "a free joint is only allowed on the top body."); }
                    // The first link sits at the first joint's anchor in the body's orientation; each further joint adds a link at its offset from the previous anchor.
                    if (joints.Count == 0)
                    {
                        var link = new ArticulationLinkDef { Parent = parentLink, Joint = ArticulationJointKind.Fixed, Origin = Io.ToF(bodyPos - parentAnchor), OriginRotation = Io.ToF(bodyRot) };
                        AddLink(link, (string?)body.Attribute("name") ?? "", null, w);
                        ApplyContents(link, body, cls, anchor);
                    }
                    else
                    {
                        int parent = parentLink;
                        Vector3D previous = default;
                        for (int k = 0; k < joints.Count; k++)
                        {
                            var ji = joints[k];
                            var link = new ArticulationLinkDef { Parent = parent };
                            link.Origin = k == 0 ? Io.ToF(bodyPos + bodyRot.Rotate(ji.Pos) - parentAnchor) : Io.ToF(ji.Pos - previous);
                            link.OriginRotation = k == 0 ? Io.ToF(bodyRot) : QuaternionF.Identity;
                            ApplyJoint(link, ji);
                            parent = AddLink(link, k == joints.Count - 1 ? ((string?)body.Attribute("name") ?? "") : ((string?)ji.El.Attribute("name") ?? ""), (string?)ji.El.Attribute("name") ?? "", w);
                            previous = ji.Pos;
                            if (k == joints.Count - 1)
                            {
                                anchor = ji.Pos;
                                ApplyContents(link, body, cls, anchor);
                            }
                            else { MakeMassless(link); }
                        }
                    }
                }

                int lastLink = _def.Links.Count - 1;
                if (_def.Links.Count > 2048) { throw new InterchangeException(F, w, "more than 2048 links."); }
                var kids = body.Elements("body").ToList();
                for (int i = kids.Count - 1; i >= 0; i--) { stack.Push((kids[i], lastLink, anchor, cls, depth + 1)); }
            }
        }

        private int AddLink(ArticulationLinkDef link, string linkName, string? jointName, string where)
        {
            _def.AddLink(link);
            _linkNames.Add(linkName);
            _jointNames.Add(_def.Links.Count == 1 ? null : jointName);
            if (!string.IsNullOrEmpty(jointName))
            {
                if (_jointLink.ContainsKey(jointName)) { throw new InterchangeException(F, where, "joint name '" + Io.Clip(jointName) + "' is used twice."); }
                _jointLink[jointName] = _def.Links.Count - 1;
            }

            return _def.Links.Count - 1;
        }

        private void MakeMassless(ArticulationLinkDef link)
        {
            link.Mass = _opt.MasslessLinkMass;
            link.InertiaTensor = Io.ToF(MassMath.Diagonal(1, 1, 1) * (_opt.MasslessLinkMass * 1e-4));
        }

        private void ApplyJoint(ArticulationLinkDef link, JointInfo ji)
        {
            var e = ji.El; string w = Io.Where(e), cls = ji.Cls;
            string type = ji.Type;
            double[] axis = Nums(e, "axis", cls, 3) ?? new[] { 0.0, 0.0, 1.0 };
            if (!(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2] > 1e-24)) { throw new InterchangeException(F, w, "the joint axis must not be zero."); }
            switch (type)
            {
                case "hinge": link.Joint = ArticulationJointKind.Revolute; break;
                case "slide": link.Joint = ArticulationJointKind.Prismatic; break;
                case "ball": link.Joint = ArticulationJointKind.Spherical; break;
                default: throw new InterchangeException(F, w, "joint type must be free, ball, slide or hinge, found '" + Io.Clip(type) + "'.");
            }

            link.Axis = Io.ToF(new Vector3D(axis[0], axis[1], axis[2]));
            double scale = type == "slide" ? 1.0 : (_degrees ? Math.PI / 180.0 : 1.0);
            string? limited = Attr(e, "limited", cls);
            if (limited != null && limited is not ("true" or "false" or "auto")) { throw new InterchangeException(F, w, "limited must be true, false or auto."); }
            var range = Nums(e, "range", cls, 2);
            bool isLimited = limited == "true" || (limited is null or "auto" && _autolimits && range != null);
            if (isLimited)
            {
                if (range == null) { throw new InterchangeException(F, w, "a limited joint needs a range."); }
                if (range[0] > range[1]) { throw new InterchangeException(F, w, "range lower " + range[0] + " is above upper " + range[1] + "."); }
                if (type == "ball") { link.SwingLimit = (float)Math.Min(2 * Math.PI, range[1] * scale); }
                else { link.HasLimits = true; link.LowerLimit = (float)(range[0] * scale); link.UpperLimit = (float)(range[1] * scale); }
            }

            double damping = Num(e, "damping", cls, 0), friction = Num(e, "frictionloss", cls, 0), armature = Num(e, "armature", cls, 0), stiffness = Num(e, "stiffness", cls, 0);
            if (damping < 0 || friction < 0 || armature < 0 || stiffness < 0) { throw new InterchangeException(F, w, "damping, frictionloss, armature and stiffness must not be negative."); }
            link.Damping = (float)damping; link.Friction = (float)friction; link.Armature = (float)armature;
            if (type != "ball")
            {
                link.InitialPosition = (float)(Num(e, "ref", cls, 0) * scale);
                if (stiffness > 0) { link.Drive = new ArticulationDrive { Stiffness = (float)stiffness, TargetPosition = (float)(Num(e, "springref", cls, 0) * scale) }; }
            }
            else if (stiffness > 0) { Warn("ball-spring", w, "a spring on a ball joint is not imported."); }

        }

        /// <summary>Mass, centre of mass, inertia and shapes of a body go on the link of its last joint, whose frame is shifted by the joint anchor.</summary>
        private void ApplyContents(ArticulationLinkDef link, XElement body, string cls, Vector3D anchor)
        {
            var parts = new List<MassPropertiesD>();
            var geoms = new List<(XElement El, ShapeSpec3D Spec, Vector3D Pos, QuaternionD Rot, double Volume, Func<double, (Vector3D, Matrix3x3D)> Inertia)>();
            foreach (var g in body.Elements("geom"))
            {
                var r = Geom(g, cls, false, out _);
                if (r == null) { continue; }
                var v = r.Value;
                geoms.Add((g, v.Spec, v.Pos, v.Rot, v.Volume, v.Inertia));
                double friction = Friction(g, cls);
                link.Shapes.Add(new ArticulationShape(v.Spec, new ShapeDef { LocalPosition = Io.ToF(v.Pos - anchor), LocalRotation = Io.ToF(v.Rot), Friction = (float)friction }));
                var contype = Attr(g, "contype", cls); var conaff = Attr(g, "conaffinity", cls);
                if ((contype != null && contype != "1") || (conaff != null && conaff != "1")) { Warn("contact-filter", Io.Where(g), "contype and conaffinity are not mapped; use shape layers and masks."); }
            }

            var inertial = body.Element("inertial");
            if (inertial != null && _inertiaFromGeom != "true")
            {
                string wi = Io.Where(inertial);
                double mass = Io.Number((string?)inertial.Attribute("mass") ?? throw new InterchangeException(F, wi, "mass is required."), F, wi, "mass");
                if (mass < 0) { throw new InterchangeException(F, wi, "mass must not be negative."); }
                var pos = inertial.Attribute("pos") is { } pa ? Io.Numbers(pa.Value, 3, F, wi, "pos") : throw new InterchangeException(F, wi, "pos is required.");
                Matrix3x3D tensor;
                if (inertial.Attribute("fullinertia") is { } fi)
                {
                    var f = Io.Numbers(fi.Value, 6, F, wi, "fullinertia");
                    tensor = new Matrix3x3D(f[0], f[3], f[4], f[3], f[1], f[5], f[4], f[5], f[2]);
                }
                else if (inertial.Attribute("diaginertia") is { } di) { var d = Io.Numbers(di.Value, 3, F, wi, "diaginertia"); tensor = MassMath.Diagonal(d[0], d[1], d[2]); }
                else { throw new InterchangeException(F, wi, "diaginertia or fullinertia is required."); }
                var rot = Orientation(inertial, cls);
                if (rot.X != 0 || rot.Y != 0 || rot.Z != 0) { tensor = InertiaD.Rotate(tensor, rot); }
                if (mass > 0) { SetMass(link, mass, new Vector3D(pos[0], pos[1], pos[2]) - anchor, tensor); }
                else { MakeMassless(link); }
                return;
            }

            if (_inertiaFromGeom == "false" && inertial == null) { MakeMassless(link); return; }
            foreach (var (el, _, pos, rot, volume, inertia) in geoms)
            {
                string we = Io.Where(el);
                double? massAttr = Attr(el, "mass", cls) is { } ms ? Io.Number(ms, F, we, "mass") : null;
                double m = massAttr ?? Num(el, "density", cls, 1000) * volume;
                if (m <= 0) { continue; }
                var (cen, tensor) = inertia(m);
                // From the geom's frame into the body's frame; the link's frame sits at the joint anchor.
                if (rot.X != 0 || rot.Y != 0 || rot.Z != 0) { tensor = InertiaD.Rotate(tensor, rot); }
                parts.Add(new MassPropertiesD(m, rot.Rotate(cen) + pos - anchor, tensor));
            }

            if (parts.Count == 0) { MakeMassless(link); return; }
            var total = InertiaD.Combine(parts.ToArray());
            SetMass(link, total.Mass, total.Center, total.Inertia);
        }

        private static void SetMass(ArticulationLinkDef link, double mass, Vector3D com, Matrix3x3D inertia)
        {
            link.Mass = (float)mass; link.CenterOfMass = Io.ToF(com); link.InertiaTensor = Io.ToF(inertia);
        }

        private void ReadActuators(XElement act)
        {
            var seen = new HashSet<int>();
            foreach (var a in act.Elements())
            {
                string kind = a.Name.LocalName, w = Io.Where(a);
                if (kind is not ("motor" or "position" or "velocity"))
                {
                    Warn("unsupported-actuator", w, "<" + kind + "> actuators are not imported; only motor, position and velocity map onto a joint drive.");
                    continue;
                }

                string? jn = (string?)a.Attribute("joint");
                if (jn == null) { Warn("actuator-target", w, "an actuator that does not name a joint is not imported."); continue; }
                if (!_jointLink.TryGetValue(jn, out int li)) { throw new InterchangeException(F, w, "the actuator's joint '" + Io.Clip(jn) + "' does not exist (or is a free joint)."); }
                string? gs = Attr(a, "gear", "main");
                double gearValue = 1;
                if (gs != null)
                {
                    var parts = gs.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                    gearValue = Io.Number(parts.Length > 0 ? parts[0] : "1", F, w, "gear");
                }

                if (gearValue != 1) { Warn("actuator-gear", w, "a gear ratio other than 1 is not mapped; the actuator is not imported."); continue; }
                var link = _def.Links[li];
                double maxForce = 0;
                if (Attr(a, "forcerange", "main") is { } fr)
                {
                    var r = Io.Numbers(fr, 2, F, w, "forcerange");
                    maxForce = Math.Max(Math.Abs(r[0]), Math.Abs(r[1]));
                }
                else if (kind == "motor" && Attr(a, "ctrlrange", "main") is { } cr)
                {
                    var r = Io.Numbers(cr, 2, F, w, "ctrlrange");
                    maxForce = Math.Max(Math.Abs(r[0]), Math.Abs(r[1]));
                }

                if (!seen.Add(li)) { Warn("actuator-twice", w, "the joint already has an actuator; the later one replaces its drive."); }
                var drive = link.Drive;
                drive.MaxForce = (float)maxForce;
                if (kind == "position")
                {
                    drive.Stiffness = (float)Num(a, "kp", "main", 1);
                    drive.Damping = (float)Num(a, "kv", "main", 0);
                }
                else if (kind == "velocity")
                {
                    drive.Stiffness = 0;
                    drive.Damping = (float)Num(a, "kv", "main", 1);
                }
                else { Warn("motor-actuator", w, "a motor applies a commanded force; only its force limit is imported (drive MaxForce). Command it with Articulation.SetJointForce."); }

                if (drive.Stiffness < 0 || drive.Damping < 0) { throw new InterchangeException(F, w, "kp and kv must not be negative."); }
                link.Drive = drive;
            }
        }
    }
}
