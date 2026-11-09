using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>
/// A description of a convex shape that a world can store in snapshots: the shape kind and the numbers that build it. Make one with the factory methods; they only record the numbers, and the
/// world validates them (with the shape factories' typed errors) when it builds the shape.
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct ShapeSpec3D
{
    internal const byte KSphere = 1, KBox = 2, KCapsule = 3, KTapered = 4, KCylinder = 5, KHull = 6, KTaperedCylinder = 10, KTriangle = 11, KEmpty = 12, KMassOffset = 13, KPlane = 14;

    private readonly byte[]? _recipe;

    private ShapeSpec3D(byte[] recipe) => _recipe = recipe;

    internal ReadOnlySpan<byte> Recipe => _recipe ?? throw new InvalidOperationException("This ShapeSpec3D is a default value; make one with a factory method such as ShapeSpec3D.Sphere.");

    private static ShapeSpec3D Make(byte kind, params float[] v)
    {
        var w = StateWriter.ToBuffer();
        w.NamedByte(kind);
        foreach (float f in v) { w.NamedSingle(f); }
        return new ShapeSpec3D(w.ToArray());
    }

    /// <summary>An infinite static half-space dot(normal, point) ≤ offset. Normal is normalized on attachment; offset is metres. Throws ArgumentOutOfRangeException on attachment for invalid normal/offset, or StaticOnlyShapeException on a non-static body.</summary>
    public static ShapeSpec3D Plane(Vector3F normal, float offset = 0f) => Make(KPlane, normal.X, normal.Y, normal.Z, offset);

    /// <summary>Offsets mass centre without moving geometry, in metres within ±2^21. Inertia uses the parallel-axis theorem. Works on convex, compound, mesh and height-field recipes. Nested offsets add, at most 16 decorators; validation occurs on attachment.</summary>
    public ShapeSpec3D WithCenterOfMassOffset(Vector3F offset)
    {
        ConvexShape3D.CheckOffset(offset);
        var w = StateWriter.ToBuffer();
        w.NamedByte(KMassOffset); w.NamedSingle(offset.X); w.NamedSingle(offset.Y); w.NamedSingle(offset.Z);
        w.NamedBytes(Recipe);
        return new ShapeSpec3D(w.ToArray());
    }

    /// <summary>A compound of convex parts; plane children are rejected. Parts and transforms are validated on attachment.</summary>
    public static ShapeSpec3D Compound(ReadOnlySpan<CompoundPart> parts) => new(Geom3D.CompoundRecipe(parts));

    /// <summary>A static triangle mesh. Vertices, indices and nonnegative flat tolerance are validated on attachment.</summary>
    public static ShapeSpec3D Mesh(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, double flatTolerance = 0.01) => new(Geom3D.MeshRecipe(vertices, indices, flatTolerance));

    /// <summary>A static sampled height field. Heights, scale, sample counts and optional cell holes are validated on attachment.</summary>
    public static ShapeSpec3D HeightField(ReadOnlySpan<float> heights, int nx, int nz, Vector3F scale, ReadOnlySpan<bool> holes = default, double flatTolerance = 0.01) => new(Geom3D.HeightFieldRecipe(heights, nx, nz, scale, holes, flatTolerance));

    /// <summary>No collision geometry or mass contribution, even when used as a sensor. Use body mass/inertia overrides for a physical placeholder.</summary>
    public static ShapeSpec3D Empty => Make(KEmpty);

    /// <summary>A sphere centred on the shape origin.</summary>
    public static ShapeSpec3D Sphere(float radius) => Make(KSphere, radius);

    /// <summary>A box.</summary>
    /// <param name="size">Full sizes along x, y and z (the same as <see cref="PhysicsWorld3D.AddBox"/>).</param>
    /// <param name="convexRadius">Rounding radius; the outer size stays the same. Default 0.</param>
    public static ShapeSpec3D Box(Vector3F size, float convexRadius = 0f) => Make(KBox, 0.5f * size.X, 0.5f * size.Y, 0.5f * size.Z, convexRadius);

    /// <summary>A capsule along the y axis.</summary>
    /// <param name="halfLength">Half the length of the segment between the cap centres.</param>
    /// <param name="radius">Cap radius.</param>
    public static ShapeSpec3D Capsule(float halfLength, float radius) => Make(KCapsule, halfLength, radius);

    /// <summary>A capsule whose caps have different radii, along the y axis.</summary>
    public static ShapeSpec3D TaperedCapsule(float halfLength, float topRadius, float bottomRadius) => Make(KTapered, halfLength, topRadius, bottomRadius);

    /// <summary>A cylinder along the y axis.</summary>
    public static ShapeSpec3D Cylinder(float halfHeight, float radius, float convexRadius = 0f) => Make(KCylinder, halfHeight, radius, convexRadius);

    /// <summary>A circular frustum along Y. Half height is positive, radii are non-negative and not both zero; all dimensions are finite and at most 2^21. Validated when attached to a body.</summary>
    public static ShapeSpec3D TaperedCylinder(float halfHeight, float topRadius, float bottomRadius) => Make(KTaperedCylinder, halfHeight, topRadius, bottomRadius);

    /// <summary>A double-sided triangular lamina with optional spherical inflation. Vertices are finite within ±2^21 metres and must form nonzero area. Inflation is non-negative, default zero. Validated on attachment.</summary>
    public static ShapeSpec3D Triangle(Vector3F a, Vector3F b, Vector3F c, float convexRadius = 0f) => Make(KTriangle, a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z, convexRadius);

    /// <summary>The convex hull of a point set (4 to 2^22 points).</summary>
    /// <param name="points">Points in the shape's frame; duplicates and interior points are fine.</param>
    /// <param name="convexRadius">Outward inflation of the hull. Default 0.</param>
    public static ShapeSpec3D ConvexHull(ReadOnlySpan<Vector3F> points, float convexRadius = 0f)
    {
        var w = StateWriter.ToBuffer();
        w.NamedByte(KHull);
        w.NamedSingle(convexRadius);
        w.NamedInt32(points.Length);
        foreach (var p in points) { w.NamedSingle(p.X); w.NamedSingle(p.Y); w.NamedSingle(p.Z); }
        return new ShapeSpec3D(w.ToArray());
    }
}

/// <summary>One convex piece of a compound shape.</summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0002")]
public readonly struct CompoundPart
{
    /// <summary>Creates a part.</summary>
    /// <param name="shape">The convex shape.</param>
    /// <param name="position">Where the part's frame sits in the compound's frame.</param>
    /// <param name="rotation">Orientation of the part in the compound's frame; any non-zero finite quaternion. Defaults to identity.</param>
    public CompoundPart(ShapeSpec3D shape, Vector3F position, QuaternionF rotation = default)
    {
        Shape = shape;
        Position = position;
        Rotation = rotation.LengthSquared() == 0f ? QuaternionF.Identity : rotation;
    }

    /// <summary>The convex shape.</summary>
    public ShapeSpec3D Shape { get; }

    /// <summary>Position in the compound's frame.</summary>
    public Vector3F Position { get; }

    /// <summary>Orientation in the compound's frame.</summary>
    public QuaternionF Rotation { get; }
}

internal enum GeomKind : byte { Convex = 1, Compound = 2, Mesh = 3, HeightField = 4 }

/// <summary>
/// Shared geometry of one or many shapes: the built collision object plus the recipe (bytes) it was built from. Recipes are what snapshots store; equal recipes share one object, so a thousand identical
/// hulls cost one hull. Recipe layout: a kind byte (1..6 convex, 7 compound, 8 mesh, 9 height field) then the numbers, little-endian; see docs/spec/world3d.md.
/// </summary>
internal sealed partial class Geom3D
{
    public const byte KCompound = 7, KMesh = 8, KHeightField = 9;

    public byte[] Recipe = Array.Empty<byte>();
    public GeomKind Kind;
    public ConvexShape3D Convex;
    public CompoundShape3D? Compound;
    public TriangleSoup3D? Soup;
    public MassPropertiesF UnitMass;      // unit density, in the geometry's own frame (convex and compound only)
    public Bounds3F LocalBounds;          // the geometry's bounds in its own frame (convex shapes use ShapeBounds3D instead)
    public Vector3F ConvexMin, ConvexMax; // convex only: padded bounds of the shape in its own frame, for rejecting query candidates before the narrow phase (derived, never saved)
    public byte FastKind;                 // convex only: 1 sphere, 2 box, 3 capsule (the shapes whose world box is a closed form of the rotation), else 0; derived, never saved
    public float[]? FastHull;             // FastKind 4 (a sharp hull): its vertices as x, y, z triples
    public float FastX, FastY, FastZ, FastR; // box half extents of the core and the convex radius, or the capsule's half height in FastY
    public int Refs;
    public BuoyancyCache? Buoyancy;       // polyhedra for fluid cutting, built on first use (derived, never saved)

    public bool IsConvex => Kind == GeomKind.Convex;

    /// <summary>Builds the geometry from a recipe. Malformed numbers surface as the typed errors of the shape factories, or as <see cref="SnapshotFormatException"/> from the reader.</summary>
    public static Geom3D Build(byte[] recipe)
    {
        var r = new StateReader(recipe);
        var g = new Geom3D { Recipe = recipe };
        byte kind = r.ReadByte();
        var offset = ReadOffsets(r, ref kind);
        if (kind >= ShapeSpec3D.KSphere && kind <= ShapeSpec3D.KHull || kind == ShapeSpec3D.KTaperedCylinder || kind == ShapeSpec3D.KTriangle || kind == ShapeSpec3D.KEmpty || kind == ShapeSpec3D.KPlane)
        {
            g.Kind = GeomKind.Convex;
            g.Convex = ReadConvex(r, kind).WithCenterOfMassOffset(offset);
            g.UnitMass = g.Convex.GetMassProperties();
            switch (g.Convex.Kind)
            {
                case ConvexShapeKind3D.Sphere: g.FastKind = 1; g.FastR = g.Convex.ConvexRadius; break;
                case ConvexShapeKind3D.Box: g.FastKind = 2; g.FastX = g.Convex.CoreSize.X; g.FastY = g.Convex.CoreSize.Y; g.FastZ = g.Convex.CoreSize.Z; g.FastR = g.Convex.ConvexRadius; break;
                case ConvexShapeKind3D.Hull when g.Convex.ConvexRadius == 0f && g.Convex.Hull is { } hull:
                    {
                        var vs = hull.Vertices;
                        var flat = new float[3 * vs.Length];
                        for (int i = 0; i < vs.Length; i++) { flat[3 * i] = vs[i].X; flat[3 * i + 1] = vs[i].Y; flat[3 * i + 2] = vs[i].Z; }
                        g.FastKind = 4; g.FastHull = flat;
                        break;
                    }

                case ConvexShapeKind3D.Capsule: g.FastKind = 3; g.FastY = g.Convex.CoreSize.Y; g.FastR = g.Convex.ConvexRadius; break;
            }

            ShapeBounds3D.Compute(g.Convex, RigidTransformF.Identity, out g.ConvexMin, out g.ConvexMax);
        }
        else if (kind == KCompound)
        {
            g.Kind = GeomKind.Compound;
            int n = r.ReadCount(7 * 4 + 1, "compound children");
            var children = new CompoundChild3D[n];
            for (int i = 0; i < n; i++)
            {
                var rot = new QuaternionF(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                var pos = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                byte ck = r.ReadByte();
                var shape = ReadConvex(r, ck);
                children[i] = new CompoundChild3D(shape, new RigidTransformF(rot.Normalized(), pos));
            }

            var c = Compound3D.Create(children);
            g.Compound = c;
            g.UnitMass = c.GetMassProperties();
            c.TryGetBounds(out var lo, out var hi);
            g.LocalBounds = new Bounds3F(lo, hi);
        }
        else if (kind == KMesh)
        {
            g.Kind = GeomKind.Mesh;
            double tol = r.ReadDouble();
            int nv = r.ReadCount(12, "mesh vertices");
            var v = new Vector3F[nv];
            for (int i = 0; i < nv; i++) { v[i] = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()); }
            int ni = r.ReadCount(4, "mesh indices");
            var idx = new int[ni];
            r.ReadInt32s(idx);
            var m = TriangleMesh3D.Create(v, idx, new MeshBuildOptions { FlatEdgeTolerance = tol });
            g.Soup = m;
            g.LocalBounds = m.Bounds;
        }
        else if (kind == KHeightField)
        {
            g.Kind = GeomKind.HeightField;
            double tol = r.ReadDouble();
            int nx = r.ReadInt32(), nz = r.ReadInt32();
            var scale = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            bool hasHoles = r.ReadBool();
            if (nx < 2 || nz < 2 || nx > HeightField3D.MaxSamplesPerSide || nz > HeightField3D.MaxSamplesPerSide) { throw r.Error("invalid height field size"); }
            long samples = (long)nx * nz;
            if (samples > r.Remaining / 4) { throw r.Error("height field larger than the data"); }
            var h = new float[samples];
            r.ReadSingles(h, false, "heights");
            bool[] holes = Array.Empty<bool>();
            if (hasHoles)
            {
                long cells = (long)(nx - 1) * (nz - 1);
                if (cells > r.Remaining) { throw r.Error("hole mask larger than the data"); }
                holes = new bool[cells];
                for (int i = 0; i < holes.Length; i++) { holes[i] = r.ReadBool(); }
            }

            var hf = HeightField3D.Create(h, nx, nz, scale, holes, new MeshBuildOptions { FlatEdgeTolerance = tol });
            g.Soup = hf;
            g.LocalBounds = hf.Bounds;
        }
        else
        {
            throw r.Error("unknown shape kind " + kind.ToString(System.Globalization.CultureInfo.InvariantCulture));
        }

        if (!g.IsConvex && (offset.X != 0 || offset.Y != 0 || offset.Z != 0))
        {
            var m = g.UnitMass;
            g.UnitMass = new MassPropertiesF(m.Mass, m.Center + offset, InertiaF.ShiftToPoint(m.Inertia, m.Mass, offset));
        }
        if (r.Remaining != 0) { throw r.Error("trailing bytes in a shape recipe"); }
        return g;
    }

    /// <summary>Reads the numbers of one convex shape (after its kind byte).</summary>
    private static Vector3F ReadOffsets(StateReader r, ref byte kind)
    {
        var offset = default(Vector3F);
        int depth = 0;
        while (kind == ShapeSpec3D.KMassOffset)
        {
            if (++depth > 16) throw r.Error("more than 16 mass-centre decorators");
            var next = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            ConvexShape3D.CheckOffset(next);
            offset += next;
            ConvexShape3D.CheckOffset(offset);
            kind = r.ReadByte();
        }
        return offset;
    }

    public static ConvexShape3D ReadConvex(StateReader r, byte kind)
    {
        var offset = ReadOffsets(r, ref kind);
        return ReadConvexGeometry(r, kind).WithCenterOfMassOffset(offset);
    }

    private static ConvexShape3D ReadConvexGeometry(StateReader r, byte kind)
    {
        switch (kind)
        {
            case ShapeSpec3D.KPlane:
            {
                var normal = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                return ConvexShape3D.Plane(normal, r.ReadSingle());
            }
            case ShapeSpec3D.KEmpty: return ConvexShape3D.Empty;
            case ShapeSpec3D.KSphere: return ConvexShape3D.Sphere(r.ReadSingle());
            case ShapeSpec3D.KBox:
            {
                var h = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                return ConvexShape3D.Box(h, r.ReadSingle());
            }
            case ShapeSpec3D.KCapsule: { float hh = r.ReadSingle(); return ConvexShape3D.Capsule(hh, r.ReadSingle()); }
            case ShapeSpec3D.KTapered: { float hh = r.ReadSingle(), t = r.ReadSingle(); return ConvexShape3D.TaperedCapsule(hh, t, r.ReadSingle()); }
            case ShapeSpec3D.KCylinder: { float hh = r.ReadSingle(), rad = r.ReadSingle(); return ConvexShape3D.Cylinder(hh, rad, r.ReadSingle()); }
            case ShapeSpec3D.KTaperedCylinder: { float hh = r.ReadSingle(), top = r.ReadSingle(); return ConvexShape3D.TaperedCylinder(hh, top, r.ReadSingle()); }
            case ShapeSpec3D.KTriangle:
            {
                var a = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                var b = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                var c = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                return ConvexShape3D.Triangle(a, b, c, r.ReadSingle());
            }
            case ShapeSpec3D.KHull:
            {
                float cr = r.ReadSingle();
                int n = r.ReadCount(12, "hull points");
                var p = new Vector3F[n];
                for (int i = 0; i < n; i++) { p[i] = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle()); }
                return ConvexShape3D.FromHull(ConvexHull3D.Build(p), cr);
            }
            default: throw r.Error("unknown convex shape kind " + kind.ToString(System.Globalization.CultureInfo.InvariantCulture));
        }
    }

    // ------------------------------------------------------------------ recipe writers for the non-convex kinds

    public static byte[] CompoundRecipe(ReadOnlySpan<CompoundPart> parts)
    {
        var w = StateWriter.ToBuffer();
        w.NamedByte(KCompound);
        w.NamedInt32(parts.Length);
        foreach (var p in parts)
        {
            w.NamedSingle(p.Rotation.X); w.NamedSingle(p.Rotation.Y); w.NamedSingle(p.Rotation.Z); w.NamedSingle(p.Rotation.W);
            w.NamedSingle(p.Position.X); w.NamedSingle(p.Position.Y); w.NamedSingle(p.Position.Z);
            w.NamedBytes(p.Shape.Recipe);
        }

        return w.ToArray();
    }

    public static byte[] MeshRecipe(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, double flatTolerance)
    {
        var w = StateWriter.ToBuffer();
        w.NamedByte(KMesh);
        w.NamedDouble(flatTolerance);
        w.NamedInt32(vertices.Length);
        foreach (var v in vertices) { w.NamedSingle(v.X); w.NamedSingle(v.Y); w.NamedSingle(v.Z); }
        w.NamedInt32(indices.Length);
        w.NamedInt32s(indices);
        return w.ToArray();
    }

    public static byte[] HeightFieldRecipe(ReadOnlySpan<float> heights, int nx, int nz, Vector3F scale, ReadOnlySpan<bool> holes, double flatTolerance)
    {
        var w = StateWriter.ToBuffer();
        w.NamedByte(KHeightField);
        w.NamedDouble(flatTolerance);
        w.NamedInt32(nx); w.NamedInt32(nz);
        w.NamedSingle(scale.X); w.NamedSingle(scale.Y); w.NamedSingle(scale.Z);
        w.NamedBool(holes.Length > 0);
        w.NamedSingles(heights);
        foreach (bool b in holes) { w.NamedBool(b); }
        return w.ToArray();
    }
}
