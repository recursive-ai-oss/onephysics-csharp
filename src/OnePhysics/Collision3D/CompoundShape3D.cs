using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Physics;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;
using System.Diagnostics.CodeAnalysis;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>A compound's child: a placed convex shape and the id of the material its surface uses.</summary>
[Experimental("OPHYS0001")]
public readonly struct CompoundChild3D
{
    /// <summary>Creates a child.</summary>
    /// <param name="placed">The shape and its transform in the compound's frame.</param>
    /// <param name="materialId">An id the world maps to a material; any value, reported back by queries and contacts.</param>
    public CompoundChild3D(in PlacedShape3D placed, int materialId = 0)
    {
        Placed = placed;
        MaterialId = materialId;
    }

    /// <summary>A child with a shape at the compound's origin.</summary>
    public CompoundChild3D(in ConvexShape3D shape, int materialId = 0)
        : this(PlacedShape3D.Create(shape), materialId)
    {
    }

    /// <summary>A child with a shape at a local transform.</summary>
    public CompoundChild3D(in ConvexShape3D shape, in RigidTransformF local, int materialId = 0)
        : this(PlacedShape3D.Create(shape, local), materialId)
    {
    }

    /// <summary>The shape and its transform in the compound's frame.</summary>
    public PlacedShape3D Placed { get; }

    /// <summary>The material id.</summary>
    public int MaterialId { get; }
}

/// <summary>A stable reference to a child of a <see cref="MutableCompound3D"/>: the child's id (its slot) and the slot's generation, which changes when the child is removed.</summary>
[Experimental("OPHYS0001")]
public readonly struct CompoundChildHandle3D : IEquatable<CompoundChildHandle3D>
{
    internal CompoundChildHandle3D(int index, int generation)
    {
        Index = index;
        Generation = generation;
    }

    /// <summary>The child id, the sub-shape id used in contacts and query results.</summary>
    public int Index { get; }

    /// <summary>Starts at 1 and increases each time the slot is freed; 0 only for a default handle, which is never valid.</summary>
    public int Generation { get; }

    /// <inheritdoc/>
    public bool Equals(CompoundChildHandle3D other) => Index == other.Index && Generation == other.Generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is CompoundChildHandle3D h && Equals(h);

    /// <inheritdoc/>
    public override int GetHashCode() => (Index * 397) ^ Generation;

    /// <summary>Equality of both fields.</summary>
    public static bool operator ==(CompoundChildHandle3D a, CompoundChildHandle3D b) => a.Equals(b);

    /// <summary>Negation of <c>==</c>.</summary>
    public static bool operator !=(CompoundChildHandle3D a, CompoundChildHandle3D b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => $"child {Index} (generation {Generation})";
}

/// <summary>The first hit of a ray against a compound.</summary>
[Experimental("OPHYS0001")]
public readonly struct CompoundRayHit3D
{
    internal CompoundRayHit3D(float fraction, Vector3F point, Vector3F normal, int child, int materialId, bool startsInside)
    {
        Fraction = fraction;
        Point = point;
        Normal = normal;
        Child = child;
        MaterialId = materialId;
        StartsInside = startsInside;
    }

    /// <summary>Fraction of the ray's direction, in [0, maxFraction].</summary>
    public float Fraction { get; }

    /// <summary>Hit point in the compound's frame.</summary>
    public Vector3F Point { get; }

    /// <summary>Outward surface normal of the child at the hit, in the compound's frame; zero when the ray starts inside.</summary>
    public Vector3F Normal { get; }

    /// <summary>Id of the child that was hit (the sub-shape id).</summary>
    public int Child { get; }

    /// <summary>Material id of that child.</summary>
    public int MaterialId { get; }

    /// <summary>True when the ray's origin is inside the child (fraction 0).</summary>
    public bool StartsInside { get; }
}

/// <summary>The first hit of a swept convex shape against a compound.</summary>
[Experimental("OPHYS0001")]
public readonly struct CompoundCastHit3D
{
    internal CompoundCastHit3D(float fraction, Vector3F normal, Vector3F pointOnCast, Vector3F pointOnChild, int child, int materialId, bool startsOverlapping)
    {
        Fraction = fraction;
        Normal = normal;
        PointOnCast = pointOnCast;
        PointOnChild = pointOnChild;
        Child = child;
        MaterialId = materialId;
        StartsOverlapping = startsOverlapping;
    }

    /// <summary>Fraction of the motion, in [0, 1].</summary>
    public float Fraction { get; }

    /// <summary>Unit normal from the swept shape towards the child, in the compound's frame.</summary>
    public Vector3F Normal { get; }

    /// <summary>Contact point on the swept shape at the hit, in the compound's frame.</summary>
    public Vector3F PointOnCast { get; }

    /// <summary>Contact point on the child, in the compound's frame.</summary>
    public Vector3F PointOnChild { get; }

    /// <summary>Id of the child that was hit.</summary>
    public int Child { get; }

    /// <summary>Material id of that child.</summary>
    public int MaterialId { get; }

    /// <summary>True when the shapes already touch or overlap at the start (fraction 0).</summary>
    public bool StartsOverlapping { get; }
}

/// <summary>
/// A shape made of convex children, each with a local transform and a material id, over an incrementally updatable AABB tree (<see cref="DynamicTree{TBox}"/>). This is the part common to
/// <see cref="Compound3D"/> (built once) and <see cref="MutableCompound3D"/> (edited between steps): child storage, combined mass properties and queries.
/// <para>
/// Child ids are slot numbers in <c>[0, SlotCount)</c>: the child's position for a static compound, a reused lowest-free slot for a mutable one. They are the sub-shape ids in contacts.
/// Child bounds are the padded bounds of <see cref="ShapeBounds3D"/>, stored as float32 and widened exactly into the tree's double boxes, with no extra margin; a query visits exactly the children whose
/// stored bounds it reaches, so results do not depend on how the tree is shaped. Query results are therefore equal for a mutable compound and a static one rebuilt from the same children
/// (except "any hit" ray casts, which report whichever hit the traversal meets first).
/// </para>
/// Queries are read-only and may run concurrently with each other, not with edits. The methods taking a <see cref="List{T}"/> clear it first and reuse its storage.
/// </summary>
[Experimental("OPHYS0001")]
public abstract class CompoundShape3D
{
    /// <summary>Most children (slots) a compound can hold, 2^16: a child id fits in 16 bits, so two child ids and a 32-bit feature id make one 64-bit point key (<see cref="ChildContact3D.GetPointKey"/>).</summary>
    public const int MaxChildren = 65536;

    private readonly DynamicTree<Aabb3D> _tree = new();
    private PlacedShape3D[] _placed = new PlacedShape3D[8];
    private Vector3F[] _min = new Vector3F[8];
    private Vector3F[] _max = new Vector3F[8];
    private int[] _material = new int[8];
    private int[] _proxy = new int[8];
    private int[] _generation = new int[8];
    private bool[] _alive = new bool[8];
    private int[] _free = new int[8]; // binary min-heap of dead slot indices, so the lowest free slot is reused first
    private int _freeCount;
    private int _slots;
    private int _count;

    private protected CompoundShape3D()
    {
    }

    /// <summary>Number of live children.</summary>
    public int Count => _count;

    /// <summary>One more than the largest child id ever used; ids are below this. Dead slots in between report false from <see cref="IsLive"/>.</summary>
    public int SlotCount => _slots;

    /// <summary>True when <paramref name="child"/> is the id of a live child.</summary>
    public bool IsLive(int child) => (uint)child < (uint)_slots && _alive[child];

    private void RequireLive(int child, string param)
    {
        if (!IsLive(child))
        {
            throw new ShapeCompositionException(ShapeCompositionError.UnknownChild, $"{param} is {child}; the id of a live child (below SlotCount {_slots}) is expected.", param);
        }
    }

    /// <summary>The shape and local transform of a child.</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="child"/> is not a live child.</exception>
    public PlacedShape3D GetChild(int child)
    {
        RequireLive(child, nameof(child));
        return _placed[child];
    }

    /// <summary>The material id of a child.</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="child"/> is not a live child.</exception>
    public int GetMaterialId(int child)
    {
        RequireLive(child, nameof(child));
        return _material[child];
    }

    /// <summary>The padded bounds of a child in the compound's frame, as used by every query.</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="child"/> is not a live child.</exception>
    public void GetChildBounds(int child, out Vector3F min, out Vector3F max)
    {
        RequireLive(child, nameof(child));
        min = _min[child];
        max = _max[child];
    }

    /// <summary>The handle of a live child (its id and the slot's generation).</summary>
    /// <exception cref="ShapeCompositionException"><paramref name="child"/> is not a live child.</exception>
    public CompoundChildHandle3D GetHandle(int child)
    {
        RequireLive(child, nameof(child));
        return new CompoundChildHandle3D(child, _generation[child]);
    }

    /// <summary>The bounds of all children together; false when there are none.</summary>
    public bool TryGetBounds(out Vector3F min, out Vector3F max)
    {
        if (_count == 0)
        {
            min = max = default;
            return false;
        }

        Aabb3D b = _tree.GetBounds(_tree.Root);
        min = new Vector3F((float)b.MinX, (float)b.MinY, (float)b.MinZ);
        max = new Vector3F((float)b.MaxX, (float)b.MaxY, (float)b.MaxZ);
        return true;
    }

    // ---- storage ---------------------------------------------------------------------------------------------------------------------------------

    private static void CheckChild(in CompoundChild3D c, string what)
    {
        PlacedShape3D p = c.Placed;
        ConvexShape3D s = p.Shape;
        if (s.Kind == ConvexShapeKind3D.Sphere && s.ConvexRadius == 0f)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidShape, $"{what} has the zero-size point shape (or is a default value); a sphere, box, capsule, cylinder or hull is expected.", what);
        }

        PlacedShape3D.CheckTransform(p.Local, what, what + ".Local");
    }

    private static void BoundsOf(in PlacedShape3D p, string what, out Vector3F min, out Vector3F max)
    {
        ShapeBounds3D.Compute(p.Shape, p.Local, out min, out max);
        double limit = BoxLimits.MaxCoordinate;
        bool ok = Math.Abs(min.X) <= limit && Math.Abs(min.Y) <= limit && Math.Abs(min.Z) <= limit && Math.Abs(max.X) <= limit && Math.Abs(max.Y) <= limit && Math.Abs(max.Z) <= limit;
        if (!ok)
        {
            throw new ShapeCompositionException(ShapeCompositionError.BoundsTooLarge, $"{what} has bounds {min} to {max}; every coordinate must be within ±{limit}.", what);
        }
    }

    private static Aabb3D ToBox(Vector3F min, Vector3F max) => new(min.X, min.Y, min.Z, max.X, max.Y, max.Z);

    private int AllocSlot(string what)
    {
        if (_freeCount > 0)
        {
            int top = _free[0];
            _free[0] = _free[--_freeCount];
            SiftDown(0);
            return top;
        }

        if (_slots >= MaxChildren)
        {
            throw new ShapeCompositionException(ShapeCompositionError.TooManyChildren, $"{what}: the compound already has {_slots} child slots; at most {MaxChildren} are allowed.", what);
        }

        if (_slots == _placed.Length)
        {
            int n = _slots * 2;
            Array.Resize(ref _placed, n);
            Array.Resize(ref _min, n);
            Array.Resize(ref _max, n);
            Array.Resize(ref _material, n);
            Array.Resize(ref _proxy, n);
            Array.Resize(ref _generation, n);
            Array.Resize(ref _alive, n);
        }

        return _slots++;
    }

    private void SiftDown(int i)
    {
        while (true)
        {
            int l = (2 * i) + 1, r = l + 1, m = i;
            if (l < _freeCount && _free[l] < _free[m])
            {
                m = l;
            }

            if (r < _freeCount && _free[r] < _free[m])
            {
                m = r;
            }

            if (m == i)
            {
                return;
            }

            (_free[i], _free[m]) = (_free[m], _free[i]);
            i = m;
        }
    }

    private void PushFree(int slot)
    {
        if (_freeCount == _free.Length)
        {
            Array.Resize(ref _free, _freeCount * 2);
        }

        int i = _freeCount++;
        _free[i] = slot;
        while (i > 0 && _free[(i - 1) / 2] > _free[i])
        {
            int p = (i - 1) / 2;
            (_free[i], _free[p]) = (_free[p], _free[i]);
            i = p;
        }
    }

    // Validates everything before changing anything, so a failed add leaves the compound as it was.
    private protected int AddCore(in CompoundChild3D child, string what)
    {
        CheckChild(child, what);
        BoundsOf(child.Placed, what, out Vector3F min, out Vector3F max);
        int slot = AllocSlot(what);
        _placed[slot] = child.Placed;
        _min[slot] = min;
        _max[slot] = max;
        _material[slot] = child.MaterialId;
        _proxy[slot] = _tree.CreateProxy(ToBox(min, max), slot);
        if (_generation[slot] == 0)
        {
            _generation[slot] = 1;
        }

        _alive[slot] = true;
        _count++;
        return slot;
    }

    private protected void RemoveCore(int slot)
    {
        _tree.DestroyProxy(_proxy[slot]);
        _alive[slot] = false;
        _placed[slot] = default;
        _generation[slot]++;
        _count--;
        PushFree(slot);
    }

    private protected void MoveCore(int slot, in RigidTransformF local, string what)
    {
        PlacedShape3D moved = PlacedShape3D.Create(_placed[slot].Shape, local);
        BoundsOf(moved, what, out Vector3F min, out Vector3F max);
        _tree.MoveProxy(_proxy[slot], ToBox(min, max));
        _placed[slot] = moved;
        _min[slot] = min;
        _max[slot] = max;
    }

    private protected void SetMaterialCore(int slot, int materialId) => _material[slot] = materialId;

    private protected void RebuildCore() => _tree.Rebuild();

    private protected int CheckHandle(CompoundChildHandle3D h, string param)
    {
        if (!IsLive(h.Index) || _generation[h.Index] != h.Generation)
        {
            throw new ShapeCompositionException(ShapeCompositionError.UnknownChild, $"{param} is {h}; a handle of a live child of this compound is expected (it may have been removed, or belong to another compound).", param);
        }

        return h.Index;
    }

    // ---- mass ------------------------------------------------------------------------------------------------------------------------------------

    /// <summary>
    /// Volume (mass at unit density), centre of mass and inertia tensor about it, in the compound's frame. Each child's own properties are moved into the compound's frame
    /// (centre transformed, tensor rotated) and combined in ascending child id with the parallel-axis theorem (<see cref="InertiaF.Combine"/>; Mirtich, "Fast and Accurate Computation of
    /// Polyhedral Mass Properties"). Children that overlap count twice, as separate solids would.
    /// </summary>
    /// <exception cref="ShapeCompositionException">The compound has no children.</exception>
    public MassPropertiesF GetMassProperties()
    {
        if (_count == 0)
        {
            throw new ShapeCompositionException(ShapeCompositionError.EmptyCompound, "The compound has no children, so it has no mass properties; at least one child is expected.");
        }

        var parts = new MassPropertiesF[_count];
        int k = 0;
        for (int i = 0; i < _slots; i++)
        {
            if (_alive[i])
            {
                parts[k++] = _placed[i].GetMassProperties();
            }
        }

        return InertiaF.Combine(parts);
    }

    // ---- queries ---------------------------------------------------------------------------------------------------------------------------------

    private struct CollectIds : IOverlapCallback
    {
        public List<int> List;

        public bool Report(int proxy, int userId)
        {
            List.Add(userId);
            return true;
        }
    }

    /// <summary>
    /// Ids of the children whose (padded) bounds overlap or touch a box, in ascending order. This is a bounds test: a child whose bounds overlap may still not touch the shape the box stands for.
    /// </summary>
    /// <param name="min">Lower corner of the box, in the compound's frame.</param>
    /// <param name="max">Upper corner; each component at least the matching component of <paramref name="min"/>.</param>
    /// <param name="results">Receives the ids; cleared first.</param>
    /// <exception cref="ShapeCompositionException">The box has a NaN or infinite component, or is inverted.</exception>
    public void OverlapBounds(Vector3F min, Vector3F max, List<int> results)
    {
        ArgumentNullException.ThrowIfNull(results);
        CheckFinite(min, "min");
        CheckFinite(max, "max");
        if (!(min.X <= max.X && min.Y <= max.Y && min.Z <= max.Z))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"min {min} and max {max} describe an inverted box; min <= max on every axis is expected.", nameof(min));
        }

        results.Clear();
        QueryBox(ToBox(min, max), results);
        results.Sort();
    }

    internal void QueryBox(in Aabb3D box, List<int> results)
    {
        var cb = new CollectIds { List = results };
        _tree.Query(box, ref cb);
    }

    private static void CheckFinite(Vector3F v, string name)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y) || !float.IsFinite(v.Z))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"{name} is {v}; finite components are expected.", name);
        }
    }

    // A query must stay inside the coordinate range the trees accept: GJK's tolerances scale with the largest coordinate it sees, so a ray starting or ending astronomically far away
    // would make every child look hit.
    private static void CheckReach(Vector3F from, Vector3F delta, float scale, string param, string what)
    {
        double limit = BoxLimits.MaxCoordinate;
        double s = scale;
        bool ok = Math.Abs(from.X) <= limit && Math.Abs(from.Y) <= limit && Math.Abs(from.Z) <= limit
            && Math.Abs(from.X + (delta.X * s)) <= limit && Math.Abs(from.Y + (delta.Y * s)) <= limit && Math.Abs(from.Z + (delta.Z * s)) <= limit;
        if (!ok)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"The {what} from {from} by {delta} (times {scale}) leaves the range of +-{limit}; start and end points within it are expected.", param);
        }
    }

    private struct RayVisitor : IRayCallback
    {
        public CompoundShape3D Owner;
        public Vector3F Origin, Direction;
        public float MaxFraction;
        public bool Any;
        public bool Found;
        public ConvexCastHit3D Best;
        public int BestChild;

        public double Report(int proxy, int userId, double maxFraction)
        {
            PlacedShape3D p = Owner._placed[userId];
            // Every child is cast with the caller's own maximum, so a child's result never depends on what the traversal met before it.
            if (!Gjk3D.RayCast(p.Shape, p.Local, Origin, Direction, MaxFraction, out ConvexCastHit3D h))
            {
                return -1.0;
            }

            if (Found && !(h.Fraction < Best.Fraction || (h.Fraction == Best.Fraction && userId < BestChild)))
            {
                return -1.0;
            }

            Found = true;
            Best = h;
            BestChild = userId;
            if (Any)
            {
                return 0.0;
            }

            // A clip of exactly 0 would stop the traversal; a hit at fraction 0 keeps looking for a lower id at fraction 0.
            return h.Fraction > 0f ? h.Fraction : double.Epsilon;
        }
    }

    /// <summary>
    /// Casts a ray and returns the closest hit over all children; equal fractions go to the lowest child id. The ray is <c>origin + t * direction</c>, <c>t</c> in <c>[0, maxFraction]</c>.
    /// Each child is cast with <see cref="Gjk3D.RayCast"/> (fraction error below 2e-4 on hulls, much less on analytic shapes).
    /// </summary>
    /// <param name="origin">Ray origin in the compound's frame; finite, within ±2^22 (as must be the end point <c>origin + maxFraction * direction</c>).</param>
    /// <param name="direction">Ray direction; finite (its length scales the fraction).</param>
    /// <param name="maxFraction">Finite and positive.</param>
    /// <param name="hit">The closest hit.</param>
    /// <returns>True when the ray hits a child.</returns>
    /// <exception cref="ShapeCompositionException">An argument is not finite, the ray starts or ends beyond ±2^22, or <paramref name="maxFraction"/> is not positive.</exception>
    public bool RayCast(Vector3F origin, Vector3F direction, float maxFraction, out CompoundRayHit3D hit) => RayCastCore(origin, direction, maxFraction, false, out hit);

    /// <summary>Like <see cref="RayCast"/> but stops at the first hit found, which is a hit but not necessarily the closest: which one depends on the tree (deterministic for a given tree).</summary>
    public bool RayCastAny(Vector3F origin, Vector3F direction, float maxFraction, out CompoundRayHit3D hit) => RayCastCore(origin, direction, maxFraction, true, out hit);

    private bool RayCastCore(Vector3F origin, Vector3F direction, float maxFraction, bool any, out CompoundRayHit3D hit)
    {
        CheckFinite(origin, nameof(origin));
        CheckFinite(direction, nameof(direction));
        if (!(maxFraction > 0f) || !float.IsFinite(maxFraction))
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"maxFraction is {maxFraction}; a finite value above 0 is expected.", nameof(maxFraction));
        }

        CheckReach(origin, direction, maxFraction, nameof(origin), "ray");
        hit = default;
        if (_count == 0)
        {
            return false;
        }

        Ray3D ray;
        try
        {
            ray = new Ray3D(origin.X, origin.Y, origin.Z, direction.X, direction.Y, direction.Z);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"The ray from {origin} along {direction} cannot be used: {e.Message}", nameof(origin), e);
        }

        var v = new RayVisitor { Owner = this, Origin = origin, Direction = direction, MaxFraction = maxFraction, Any = any };
        _tree.RayCast(ray, maxFraction, ref v);
        if (!v.Found)
        {
            return false;
        }

        hit = new CompoundRayHit3D(v.Best.Fraction, v.Best.PointB, v.Best.Normal, v.BestChild, _material[v.BestChild], v.Best.StartsOverlapping);
        return true;
    }

    private struct CastVisitor : IRayCallback
    {
        public CompoundShape3D Owner;
        public ConvexShape3D Shape;
        public RigidTransformF ShapeInCompound, Inverse;
        public Vector3F MotionInShape;
        public bool Found;
        public ConvexCastHit3D Best;
        public int BestChild;

        public double Report(int proxy, int userId, double maxFraction)
        {
            PlacedShape3D p = Owner._placed[userId];
            if (!Gjk3D.ShapeCast(Shape, p.Shape, Inverse * p.Local, MotionInShape, out ConvexCastHit3D h))
            {
                return -1.0;
            }

            if (Found && !(h.Fraction < Best.Fraction || (h.Fraction == Best.Fraction && userId < BestChild)))
            {
                return -1.0;
            }

            Found = true;
            Best = h;
            BestChild = userId;
            return h.Fraction > 0f ? h.Fraction : double.Epsilon;
        }
    }

    /// <summary>
    /// Sweeps a convex shape through the compound and returns the first contact over all children (<see cref="Gjk3D.ShapeCast"/> per child); equal fractions go to the lowest child id.
    /// Children are visited through the swept bounds of the shape, so far-away children cost a box test.
    /// </summary>
    /// <param name="shape">The moving shape, in its own frame; not the point shape (use <see cref="RayCast"/>).</param>
    /// <param name="shapeInCompound">Placement of the shape's frame in the compound's frame at the start; a unit rotation (squared length within 1% of 1), translation components finite and at most 2^22 (the swept bounds must also stay within the tree's ±2^22 range).</param>
    /// <param name="motion">How far the shape moves, in the compound's frame; finite, and the shape's end position must stay within ±2^22. The cast covers fractions 0 to 1.</param>
    /// <param name="hit">The first contact, in the compound's frame.</param>
    /// <returns>True when the shape touches a child within the motion.</returns>
    /// <exception cref="ShapeCompositionException">An argument is unusable.</exception>
    public bool ShapeCast(in ConvexShape3D shape, in RigidTransformF shapeInCompound, Vector3F motion, out CompoundCastHit3D hit)
    {
        if (shape.Kind == ConvexShapeKind3D.Sphere && shape.ConvexRadius == 0f)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidShape, "shape is the zero-size point shape; use RayCast for a point.", nameof(shape));
        }

        PlacedShape3D.CheckTransform(shapeInCompound, nameof(shapeInCompound), nameof(shapeInCompound), ManifoldGenerator3D.MaxOffset);
        CheckFinite(motion, nameof(motion));
        CheckReach(shapeInCompound.Translation, motion, 1f, nameof(motion), "sweep");
        hit = default;
        if (_count == 0)
        {
            return false;
        }

        ShapeBounds3D.Compute(shape, shapeInCompound, out Vector3F lo, out Vector3F hi);
        BoxCast3D sweep;
        try
        {
            sweep = new BoxCast3D(ToBox(lo, hi), motion.X, motion.Y, motion.Z);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new ShapeCompositionException(ShapeCompositionError.InvalidQuery, $"The sweep of the shape from {shapeInCompound.Translation} by {motion} cannot be used: {e.Message}", nameof(shapeInCompound), e);
        }

        RigidTransformF inverse = shapeInCompound.Inverted();
        var v = new CastVisitor { Owner = this, Shape = shape, ShapeInCompound = shapeInCompound, Inverse = inverse, MotionInShape = inverse.Rotation.Rotate(motion) };
        _tree.RayCast(sweep, 1.0, ref v);
        if (!v.Found)
        {
            return false;
        }

        ConvexCastHit3D h = v.Best;
        hit = new CompoundCastHit3D(
            h.Fraction,
            shapeInCompound.TransformVector(h.Normal),
            shapeInCompound.TransformPoint(h.PointA),
            shapeInCompound.TransformPoint(h.PointB),
            v.BestChild,
            _material[v.BestChild],
            h.StartsOverlapping);
        return true;
    }

    // ---- state -----------------------------------------------------------------------------------------------------------------------------------

    /// <summary>
    /// Writes the compound's acceleration structure and child table in a fixed layout: the tree (<see cref="DynamicTree{TBox}.Write"/>: node count, root, free-list head, then each node's
    /// height, parent, child 1, child 2, user id, flags and box), then the slot count and, for each slot, its alive flag, generation, and for live slots the material id, the local transform
    /// (rotation x, y, z, w, translation x, y, z) and the padded bounds (min x, y, z, max x, y, z). Equal inputs and equal edit sequences give equal bytes everywhere.
    /// </summary>
    public void WriteBvh(StateWriter writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        _tree.Write(writer);
        writer.WriteInt32(_slots);
        for (int i = 0; i < _slots; i++)
        {
            writer.WriteByte(_alive[i] ? (byte)1 : (byte)0);
            writer.WriteInt32(_generation[i]);
            if (!_alive[i])
            {
                continue;
            }

            writer.WriteInt32(_material[i]);
            RigidTransformF l = _placed[i].Local;
            writer.WriteSingle(l.Rotation.X);
            writer.WriteSingle(l.Rotation.Y);
            writer.WriteSingle(l.Rotation.Z);
            writer.WriteSingle(l.Rotation.W);
            writer.WriteSingle(l.Translation.X);
            writer.WriteSingle(l.Translation.Y);
            writer.WriteSingle(l.Translation.Z);
            writer.WriteSingle(_min[i].X);
            writer.WriteSingle(_min[i].Y);
            writer.WriteSingle(_min[i].Z);
            writer.WriteSingle(_max[i].X);
            writer.WriteSingle(_max[i].Y);
            writer.WriteSingle(_max[i].Z);
        }
    }

    /// <summary>Checks the tree's structural invariants (<see cref="DynamicTree{TBox}.Validate"/>) and that the child table agrees with it; throws if one is broken.</summary>
    public void Validate()
    {
        _tree.Validate();
        if (_tree.ProxyCount != _count)
        {
            throw new InvalidOperationException($"The tree holds {_tree.ProxyCount} proxies but the compound has {_count} live children.");
        }

        for (int i = 0; i < _slots; i++)
        {
            if (_alive[i] && (!_tree.IsProxy(_proxy[i]) || _tree.GetUserId(_proxy[i]) != i))
            {
                throw new InvalidOperationException($"Child {i} is not linked to its tree proxy.");
            }
        }
    }
}
