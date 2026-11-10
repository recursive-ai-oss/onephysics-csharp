using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics.Physics3D;

namespace RecursiveAI.OnePhysics.Interchange;

/// <summary>Settings for <see cref="UrdfImporter"/>.</summary>
public sealed class UrdfOptions
{
    /// <summary>True (default) fixes the root link to the world at the pose in the definition (a robot bolted to a table); false lets it float, as a mobile robot's base does.</summary>
    public bool FixedBase { get; set; } = true;

    /// <summary>Finds the meshes that collision geometry refers to. Without one, mesh collisions are left out with a warning.</summary>
    public MeshResolver? Meshes { get; set; }

    /// <summary>Mass given to a link that has no inertial block and no way to get one, kg, above 0. A link that only places other links (a world or base frame) would otherwise make the joint-space mass matrix singular. Default 1e-3.</summary>
    public float MasslessLinkMass { get; set; } = 1e-3f;
}

/// <summary>The result of importing a URDF file.</summary>
public sealed class UrdfModel
{
    internal UrdfModel(string robotName, ArticulationDef def, List<string> links, List<string?> joints, List<InterchangeWarning> warnings)
    {
        RobotName = robotName; Definition = def; LinkNames = links; JointNames = joints; Warnings = warnings;
    }

    /// <summary>The name given in the file's robot element.</summary>
    public string RobotName { get; }

    /// <summary>The articulation: one link per URDF link, parents first (depth first in file order), the root fixed or floating as the options said.</summary>
    public ArticulationDef Definition { get; }

    /// <summary>The URDF name of each link of <see cref="Definition"/>, by index.</summary>
    public IReadOnlyList<string> LinkNames { get; }

    /// <summary>The name of the joint that connects each link to its parent, by link index; null for the root.</summary>
    public IReadOnlyList<string?> JointNames { get; }

    /// <summary>Everything the importer left out or approximated.</summary>
    public IReadOnlyList<InterchangeWarning> Warnings { get; }

    /// <summary>The index of the link with a URDF name, or -1.</summary>
    public int FindLink(string name) => ((List<string>)LinkNames).IndexOf(name);

    /// <summary>The index of the link moved by the joint with a URDF name, or -1.</summary>
    public int FindJoint(string name) => ((List<string?>)JointNames).IndexOf(name);
}

/// <summary>
/// Reads URDF robot descriptions into <see cref="ArticulationDef"/>s. Units are those of the format (metres, kilograms, radians). Visual elements, materials, transmissions and simulator extensions are ignored.
/// Revolute, continuous, prismatic and fixed joints map directly; floating (below the root) and planar joints become fixed with a warning; mimic joints are kept as ordinary joints with a warning.
/// Collision geometry: box, sphere, cylinder and (through <see cref="UrdfOptions.Meshes"/>) mesh, the last as the convex hull of its vertices.
/// </summary>
public static class UrdfImporter
{
    private const string F = "URDF";

    /// <summary>Imports a URDF document.</summary>
    /// <param name="xml">The document text.</param>
    /// <param name="options">Settings; null for the defaults.</param>
    /// <exception cref="InterchangeException">The document is malformed or describes something that cannot be a robot (no root, a loop, an unknown link, a bad number); the message names the element.</exception>
    public static UrdfModel Import(string xml, UrdfOptions? options = null)
    {
        options ??= new UrdfOptions();
        if (!(options.MasslessLinkMass > 0f) || !float.IsFinite(options.MasslessLinkMass)) { throw new ArgumentOutOfRangeException(nameof(options), "MasslessLinkMass must be finite and above 0."); }
        var doc = Io.LoadXml(xml, F);
        var robot = doc.Root;
        if (robot == null || robot.Name.LocalName != "robot") { throw new InterchangeException(F, "document", "the root element must be <robot>."); }
        var warnings = new List<InterchangeWarning>();
        var links = new List<LinkData>();
        var byName = new Dictionary<string, int>(StringComparer.Ordinal);
        foreach (var e in robot.Elements("link"))
        {
            string name = Name(e);
            if (byName.ContainsKey(name)) { throw new InterchangeException(F, Io.Where(e), "link names must be unique."); }
            byName[name] = links.Count;
            links.Add(ReadLink(e, name, options, warnings));
            if (links.Count > 2048) { throw new InterchangeException(F, Io.Where(e), "more than 2048 links."); }
        }

        if (links.Count == 0) { throw new InterchangeException(F, "robot", "no <link> elements."); }
        var joints = new List<JointData>();
        var jointNames = new HashSet<string>(StringComparer.Ordinal);
        foreach (var e in robot.Elements("joint"))
        {
            var j = ReadJoint(e, byName, warnings);
            if (!jointNames.Add(j.Name)) { throw new InterchangeException(F, Io.Where(e), "joint names must be unique."); }
            if (links[j.Child].ParentJoint >= 0) { throw new InterchangeException(F, Io.Where(e), "link '" + links[j.Child].Name + "' already has a parent joint; a robot must be a tree (closed loops are not supported)."); }
            if (j.Parent == j.Child) { throw new InterchangeException(F, Io.Where(e), "a joint cannot connect a link to itself."); }
            links[j.Child].ParentJoint = joints.Count;
            joints.Add(j);
        }

        var roots = new List<int>();
        for (int i = 0; i < links.Count; i++) { if (links[i].ParentJoint < 0) { roots.Add(i); } }
        if (roots.Count == 0) { throw new InterchangeException(F, "robot", "every link has a parent: the joints form a loop."); }
        if (roots.Count > 1) { throw new InterchangeException(F, "robot", roots.Count + " links have no parent joint ('" + links[roots[0]].Name + "', '" + links[roots[1]].Name + "'...); a robot needs exactly one root."); }

        // Children in joint order, then a depth-first walk with an explicit stack so a long chain cannot overflow the call stack.
        var children = new List<int>[links.Count];
        for (int i = 0; i < children.Length; i++) { children[i] = new List<int>(); }
        for (int i = 0; i < joints.Count; i++) { children[joints[i].Parent].Add(i); }
        var order = new List<int>(links.Count);
        var stack = new Stack<int>();
        stack.Push(roots[0]);
        while (stack.Count > 0)
        {
            int l = stack.Pop();
            order.Add(l);
            for (int k = children[l].Count - 1; k >= 0; k--) { stack.Push(joints[children[l][k]].Child); }
        }

        if (order.Count != links.Count) { throw new InterchangeException(F, "robot", "links not reachable from the root '" + links[roots[0]].Name + "' (the joints contain a loop)."); }
        var position = new int[links.Count];
        for (int i = 0; i < order.Count; i++) { position[order[i]] = i; }
        var def = new ArticulationDef { FixedBase = options.FixedBase };
        var names = new List<string>(); var jnames = new List<string?>();
        foreach (int l in order)
        {
            var ld = links[l];
            var link = new ArticulationLinkDef();
            if (ld.ParentJoint >= 0)
            {
                var j = joints[ld.ParentJoint];
                link.Parent = position[j.Parent];
                ApplyJoint(link, j);
            }

            ApplyLink(link, ld, options);
            def.AddLink(link);
            names.Add(ld.Name); jnames.Add(ld.ParentJoint >= 0 ? joints[ld.ParentJoint].Name : null);
        }

        return new UrdfModel((string?)robot.Attribute("name") ?? "", def, names, jnames, warnings);
    }

    /// <summary>Imports a URDF document from a stream of UTF-8 text.</summary>
    public static UrdfModel Import(Stream stream, UrdfOptions? options = null) => Import(Io.ReadAll(stream, F, Io.MaxChars), options);

    private sealed class LinkData
    {
        public string Name = "";
        public int ParentJoint = -1;
        public bool HasInertial;
        public double Mass;
        public Vector3D Com;
        public Matrix3x3D Inertia;
        public List<ArticulationShape> Shapes = new();
    }

    private sealed class JointData
    {
        public string Name = "";
        public string Type = "";
        public int Parent, Child;
        public Vector3D Origin;
        public QuaternionD Rotation = new(0, 0, 0, 1);
        public Vector3D Axis = new(1, 0, 0);
        public bool HasLimit;
        public double Lower, Upper, Effort, Velocity, Damping, Friction;
    }

    private static string Name(XElement e)
    {
        string? n = (string?)e.Attribute("name");
        if (string.IsNullOrEmpty(n)) { throw new InterchangeException(F, Io.Where(e), "the 'name' attribute is required."); }
        return n;
    }

    private static (Vector3D, QuaternionD) Origin(XElement? parent)
    {
        var o = parent?.Element("origin");
        if (o == null) { return (new Vector3D(0, 0, 0), new QuaternionD(0, 0, 0, 1)); }
        string w = Io.Where(o);
        var p = new Vector3D(0, 0, 0);
        var q = new QuaternionD(0, 0, 0, 1);
        if (o.Attribute("xyz") is { } xyz) { var v = Io.Numbers(xyz.Value, 3, F, w, "xyz"); p = new Vector3D(v[0], v[1], v[2]); }
        if (o.Attribute("rpy") is { } rpy) { var v = Io.Numbers(rpy.Value, 3, F, w, "rpy"); q = Rpy(v[0], v[1], v[2]); }
        return (p, q);
    }

    /// <summary>The URDF convention: roll about x, then pitch about y, then yaw about z, all about the fixed parent axes, so R = Rz(yaw) Ry(pitch) Rx(roll).</summary>
    internal static QuaternionD Rpy(double roll, double pitch, double yaw) =>
        Io.AxisAngle(0, 0, 1, yaw) * Io.AxisAngle(0, 1, 0, pitch) * Io.AxisAngle(1, 0, 0, roll);

    private static LinkData ReadLink(XElement e, string name, UrdfOptions options, List<InterchangeWarning> warnings)
    {
        var ld = new LinkData { Name = name };
        var inertial = e.Element("inertial");
        if (inertial != null)
        {
            string w = Io.Where(inertial);
            var (p, q) = Origin(inertial);
            var m = inertial.Element("mass");
            double mass = m == null ? 0 : Io.Number((string?)m.Attribute("value") ?? "", F, Io.Where(m), "mass value");
            if (mass < 0) { throw new InterchangeException(F, w, "mass must not be negative."); }
            var i = inertial.Element("inertia");
            Matrix3x3D tensor = default;
            if (i != null)
            {
                string wi = Io.Where(i);
                double Get(string a) => Io.Number((string?)i.Attribute(a) ?? "0", F, wi, a);
                double ixx = Get("ixx"), ixy = Get("ixy"), ixz = Get("ixz"), iyy = Get("iyy"), iyz = Get("iyz"), izz = Get("izz");
                tensor = new Matrix3x3D(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
                // The inertial frame may be rotated against the link frame; the engine wants the tensor in the link's axes.
                if (q.X != 0 || q.Y != 0 || q.Z != 0) { tensor = InertiaD.Rotate(tensor, Io.Normalize(q)); }
            }

            ld.HasInertial = mass > 0; ld.Mass = mass; ld.Com = p; ld.Inertia = tensor;
        }

        foreach (var c in e.Elements("collision"))
        {
            string w = Io.Where(c);
            var (p, q) = Origin(c);
            var g = c.Element("geometry");
            var shape = g == null ? null : g.Elements().FirstOrDefault();
            if (shape == null) { throw new InterchangeException(F, w, "<collision> needs a <geometry> with one shape."); }
            string gw = Io.Where(shape);
            ShapeSpec3D spec;
            switch (shape.Name.LocalName)
            {
                case "box":
                {
                    var s = Io.Numbers((string?)shape.Attribute("size") ?? "", 3, F, gw, "size");
                    if (!(s[0] > 0 && s[1] > 0 && s[2] > 0)) { throw new InterchangeException(F, gw, "box size must be above 0 on every axis."); }
                    spec = ShapeSpec3D.Box(new Vector3F((float)s[0], (float)s[1], (float)s[2]));
                    break;
                }
                case "sphere":
                {
                    double r = Io.Number((string?)shape.Attribute("radius") ?? "", F, gw, "radius");
                    if (!(r > 0)) { throw new InterchangeException(F, gw, "sphere radius must be above 0."); }
                    spec = ShapeSpec3D.Sphere((float)r);
                    break;
                }
                case "cylinder":
                {
                    double r = Io.Number((string?)shape.Attribute("radius") ?? "", F, gw, "radius"), len = Io.Number((string?)shape.Attribute("length") ?? "", F, gw, "length");
                    if (!(r > 0 && len > 0)) { throw new InterchangeException(F, gw, "cylinder radius and length must be above 0."); }
                    // URDF cylinders stand along z, ours along y: turn a quarter about x.
                    spec = ShapeSpec3D.Cylinder((float)(0.5 * len), (float)r);
                    q *= Io.AxisAngle(1, 0, 0, Math.PI / 2);
                    break;
                }
                case "mesh":
                {
                    string file = (string?)shape.Attribute("filename") ?? "";
                    if (file.Length == 0) { throw new InterchangeException(F, gw, "mesh needs a filename."); }
                    var mesh = options.Meshes?.Invoke(file);
                    if (mesh == null) { warnings.Add(new InterchangeWarning("mesh-unresolved", gw, "mesh '" + Io.Clip(file) + "' was not provided by the mesh resolver; the collision shape is left out.")); continue; }
                    double[] scale = shape.Attribute("scale") is { } sa ? Io.Numbers(sa.Value, 3, F, gw, "scale") : new[] { 1.0, 1.0, 1.0 };
                    spec = MeshHull.Create(mesh, scale[0], scale[1], scale[2], F, gw);
                    break;
                }
                default:
                    warnings.Add(new InterchangeWarning("unknown-geometry", gw, "geometry '" + shape.Name.LocalName + "' is not supported; the collision shape is left out."));
                    continue;
            }

            ld.Shapes.Add(new ArticulationShape(spec, new ShapeDef { LocalPosition = Io.ToF(p), LocalRotation = Io.ToF(q) }));
        }

        return ld;
    }

    private static JointData ReadJoint(XElement e, Dictionary<string, int> byName, List<InterchangeWarning> warnings)
    {
        string w = Io.Where(e);
        var j = new JointData { Name = Name(e), Type = (string?)e.Attribute("type") ?? "" };
        if (j.Type is not ("revolute" or "continuous" or "prismatic" or "fixed" or "floating" or "planar")) { throw new InterchangeException(F, w, "joint type must be revolute, continuous, prismatic, fixed, floating or planar, found '" + Io.Clip(j.Type) + "'."); }
        int Link(string el)
        {
            string? n = (string?)e.Element(el)?.Attribute("link");
            if (n == null) { throw new InterchangeException(F, w, "<" + el + " link=...> is required."); }
            if (!byName.TryGetValue(n, out int idx)) { throw new InterchangeException(F, w, "<" + el + "> names the unknown link '" + Io.Clip(n) + "'."); }
            return idx;
        }

        j.Parent = Link("parent"); j.Child = Link("child");
        (j.Origin, j.Rotation) = Origin(e);
        if (e.Element("axis") is { } axis)
        {
            var a = Io.Numbers((string?)axis.Attribute("xyz") ?? "1 0 0", 3, F, Io.Where(axis), "xyz");
            if (!(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] > 1e-24)) { throw new InterchangeException(F, Io.Where(axis), "the joint axis must not be zero."); }
            j.Axis = new Vector3D(a[0], a[1], a[2]);
        }

        if (e.Element("limit") is { } lim)
        {
            string lw = Io.Where(lim);
            double N(string a, double d) => lim.Attribute(a) is { } at ? Io.Number(at.Value, F, lw, a) : d;
            j.Effort = N("effort", 0); j.Velocity = N("velocity", 0);
            if (j.Effort < 0 || j.Velocity < 0) { throw new InterchangeException(F, lw, "effort and velocity limits must not be negative."); }
            if (j.Type is "revolute" or "prismatic")
            {
                j.Lower = N("lower", 0); j.Upper = N("upper", 0);
                if (j.Lower > j.Upper) { throw new InterchangeException(F, lw, "lower limit " + j.Lower + " is above the upper limit " + j.Upper + "."); }
                j.HasLimit = true;
            }
        }
        else if (j.Type is "revolute" or "prismatic") { throw new InterchangeException(F, w, j.Type + " joints need a <limit> element."); }

        if (e.Element("dynamics") is { } dyn)
        {
            string dw = Io.Where(dyn);
            j.Damping = dyn.Attribute("damping") is { } d ? Io.Number(d.Value, F, dw, "damping") : 0;
            j.Friction = dyn.Attribute("friction") is { } f ? Io.Number(f.Value, F, dw, "friction") : 0;
            if (j.Damping < 0 || j.Friction < 0) { throw new InterchangeException(F, dw, "damping and friction must not be negative."); }
        }

        if (e.Element("mimic") != null) { warnings.Add(new InterchangeWarning("mimic-joint", w, "mimic joints are not supported; the joint moves freely and a host can drive it to follow its master.")); }
        if (j.Type == "floating") { warnings.Add(new InterchangeWarning("floating-joint", w, "a floating joint below the root is imported as a fixed joint; use UrdfOptions.FixedBase = false for a floating root.")); }
        if (j.Type == "planar") { warnings.Add(new InterchangeWarning("planar-joint", w, "planar joints are not supported; the joint is imported as fixed.")); }
        return j;
    }

    private static void ApplyJoint(ArticulationLinkDef link, JointData j)
    {
        link.Origin = Io.ToF(j.Origin);
        link.OriginRotation = Io.ToF(j.Rotation);
        link.Joint = j.Type switch
        {
            "revolute" or "continuous" => ArticulationJointKind.Revolute,
            "prismatic" => ArticulationJointKind.Prismatic,
            _ => ArticulationJointKind.Fixed,
        };
        if (link.Joint == ArticulationJointKind.Fixed) { return; }
        link.Axis = Io.ToF(j.Axis);
        if (j.HasLimit) { link.HasLimits = true; link.LowerLimit = (float)j.Lower; link.UpperLimit = (float)j.Upper; }
        link.Drive = new ArticulationDrive { MaxForce = (float)j.Effort };
        link.MaxVelocity = (float)j.Velocity;
        link.Damping = (float)j.Damping;
        link.Friction = (float)j.Friction;
    }

    private static void ApplyLink(ArticulationLinkDef link, LinkData ld, UrdfOptions options)
    {
        if (ld.HasInertial)
        {
            link.Mass = (float)ld.Mass; link.CenterOfMass = Io.ToF(ld.Com); link.InertiaTensor = Io.ToF(ld.Inertia);
        }
        else
        {
            // No inertial block: URDF means "no mass". The engine needs some for the joint-space mass matrix; a small one keeps the link out of the dynamics.
            link.Mass = options.MasslessLinkMass;
            link.InertiaTensor = Io.ToF(new Matrix3x3D(1, 0, 0, 0, 1, 0, 0, 0, 1) * (options.MasslessLinkMass * 1e-4));
        }

        link.Shapes.AddRange(ld.Shapes);
    }
}

internal static class MeshHull
{
    public const int MaxHullVertices = 4096;

    public static ShapeSpec3D Create(InterchangeMesh mesh, double sx, double sy, double sz, string format, string where)
    {
        if (mesh.Vertices.Length > 1_000_000) { throw new InterchangeException(format, where, "the mesh has more than a million vertices."); }
        if (!(Math.Abs(sx) > 0 && Math.Abs(sy) > 0 && Math.Abs(sz) > 0) || Math.Abs(sx) > 1e6 || Math.Abs(sy) > 1e6 || Math.Abs(sz) > 1e6) { throw new InterchangeException(format, where, "mesh scale must be non-zero and within ±1e6."); }
        var pts = new Vector3F[mesh.Vertices.Length];
        for (int i = 0; i < pts.Length; i++)
        {
            var v = mesh.Vertices[i];
            pts[i] = new Vector3F((float)(v.X * sx), (float)(v.Y * sy), (float)(v.Z * sz));
            if (!float.IsFinite(pts[i].X) || !float.IsFinite(pts[i].Y) || !float.IsFinite(pts[i].Z)) { throw new InterchangeException(format, where, "mesh vertex " + i + " is not finite after scaling."); }
        }

        try { return ShapeSpec3D.ConvexHull(pts); }
        catch (ArgumentException e) { throw new InterchangeException(format, where, "the mesh does not make a convex hull (" + e.Message + ")."); }
    }
}
