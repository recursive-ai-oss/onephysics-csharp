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
    internal const byte KSphere = 1, KBox = 2, KCapsule = 3, KTapered = 4, KCylinder = 5, KHull = 6;

    private readonly byte[]? _recipe;

    private ShapeSpec3D(byte[] recipe) => _recipe = recipe;

    internal ReadOnlySpan<byte> Recipe => _recipe ?? throw new InvalidOperationException("This ShapeSpec3D is a default value; make one with a factory method such as ShapeSpec3D.Sphere.");

    private static ShapeSpec3D Make(byte kind, params float[] v)
    {
        var w = StateWriter.ToBuffer();
        w.WriteByte(kind);
        foreach (float f in v) { w.WriteSingle(f); }
        return new ShapeSpec3D(w.ToArray());
    }

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

    /// <summary>The convex hull of a point set (4 to 2^22 points).</summary>
    /// <param name="points">Points in the shape's frame; duplicates and interior points are fine.</param>
    /// <param name="convexRadius">Outward inflation of the hull. Default 0.</param>
    public static ShapeSpec3D ConvexHull(ReadOnlySpan<Vector3F> points, float convexRadius = 0f)
    {
        var w = StateWriter.ToBuffer();
        w.WriteByte(KHull);
        w.WriteSingle(convexRadius);
        w.WriteInt32(points.Length);
        foreach (var p in points) { w.WriteSingle(p.X); w.WriteSingle(p.Y); w.WriteSingle(p.Z); }
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
internal sealed class Geom3D
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
        if (kind >= ShapeSpec3D.KSphere && kind <= ShapeSpec3D.KHull)
        {
            g.Kind = GeomKind.Convex;
            g.Convex = ReadConvex(r, kind);
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

        if (r.Remaining != 0) { throw r.Error("trailing bytes in a shape recipe"); }
        return g;
    }

    /// <summary>Reads the numbers of one convex shape (after its kind byte).</summary>
    public static ConvexShape3D ReadConvex(StateReader r, byte kind)
    {
        switch (kind)
        {
            case ShapeSpec3D.KSphere: return ConvexShape3D.Sphere(r.ReadSingle());
            case ShapeSpec3D.KBox:
            {
                var h = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
                return ConvexShape3D.Box(h, r.ReadSingle());
            }
            case ShapeSpec3D.KCapsule: { float hh = r.ReadSingle(); return ConvexShape3D.Capsule(hh, r.ReadSingle()); }
            case ShapeSpec3D.KTapered: { float hh = r.ReadSingle(), t = r.ReadSingle(); return ConvexShape3D.TaperedCapsule(hh, t, r.ReadSingle()); }
            case ShapeSpec3D.KCylinder: { float hh = r.ReadSingle(), rad = r.ReadSingle(); return ConvexShape3D.Cylinder(hh, rad, r.ReadSingle()); }
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
        w.WriteByte(KCompound);
        w.WriteInt32(parts.Length);
        foreach (var p in parts)
        {
            w.WriteSingle(p.Rotation.X); w.WriteSingle(p.Rotation.Y); w.WriteSingle(p.Rotation.Z); w.WriteSingle(p.Rotation.W);
            w.WriteSingle(p.Position.X); w.WriteSingle(p.Position.Y); w.WriteSingle(p.Position.Z);
            w.WriteBytes(p.Shape.Recipe);
        }

        return w.ToArray();
    }

    public static byte[] MeshRecipe(ReadOnlySpan<Vector3F> vertices, ReadOnlySpan<int> indices, double flatTolerance)
    {
        var w = StateWriter.ToBuffer();
        w.WriteByte(KMesh);
        w.WriteDouble(flatTolerance);
        w.WriteInt32(vertices.Length);
        foreach (var v in vertices) { w.WriteSingle(v.X); w.WriteSingle(v.Y); w.WriteSingle(v.Z); }
        w.WriteInt32(indices.Length);
        w.WriteInt32s(indices);
        return w.ToArray();
    }

    public static byte[] HeightFieldRecipe(ReadOnlySpan<float> heights, int nx, int nz, Vector3F scale, ReadOnlySpan<bool> holes, double flatTolerance)
    {
        var w = StateWriter.ToBuffer();
        w.WriteByte(KHeightField);
        w.WriteDouble(flatTolerance);
        w.WriteInt32(nx); w.WriteInt32(nz);
        w.WriteSingle(scale.X); w.WriteSingle(scale.Y); w.WriteSingle(scale.Z);
        w.WriteBool(holes.Length > 0);
        w.WriteSingles(heights);
        foreach (bool b in holes) { w.WriteBool(b); }
        return w.ToArray();
    }
}
