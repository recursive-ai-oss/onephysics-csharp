using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>What <c>GetDebugGeometry</c> emits. Flags combine.</summary>
[Flags]
public enum DebugContent
{
    /// <summary>Nothing.</summary>
    None = 0,

    /// <summary>Every shape in world space (<see cref="DebugGeometry.Shapes"/> and <see cref="DebugGeometry.Points"/>).</summary>
    Shapes = 1,

    /// <summary>The tight axis-aligned box of every shape (<see cref="DebugGeometry.Bounds"/>).</summary>
    Bounds = 2,

    /// <summary>The broadphase's stored (fat) box of every shape (<see cref="DebugGeometry.BroadphaseBounds"/>).</summary>
    BroadphaseBounds = 4,

    /// <summary>Every manifold point with its normal (<see cref="DebugGeometry.Contacts"/>).</summary>
    Contacts = 8,

    /// <summary>One entry per body with its centre of mass, type, sleep state and island (<see cref="DebugGeometry.Bodies"/>).</summary>
    Bodies = 16,

    /// <summary>One entry per joint (<see cref="DebugGeometry.Joints"/>); one per live joint, in constraint-slot order.</summary>
    Joints = 32,

    /// <summary>Everything.</summary>
    All = 63,
}

/// <summary>The kind of a <see cref="DebugShape"/>. The same enum serves 2D and 3D worlds; a 2D world emits the first five.</summary>
public enum DebugShapeKind : byte
{
    /// <summary>One point (the centre) and a radius.</summary>
    Circle = 0,

    /// <summary>Two points (the ends of the axis) and a radius.</summary>
    Capsule = 1,

    /// <summary>A convex polygon: its vertices in counter-clockwise order, rounded by the radius.</summary>
    Polygon = 2,

    /// <summary>Two points.</summary>
    Segment = 3,

    /// <summary>Two points of a chain (ghost vertices are not emitted).</summary>
    ChainSegment = 4,
}

/// <summary>How a body takes part right now.</summary>
public enum DebugBodyState : byte
{
    /// <summary>A static body.</summary>
    Static = 0,

    /// <summary>A kinematic body (always awake).</summary>
    Kinematic = 1,

    /// <summary>A dynamic body that is awake.</summary>
    Awake = 2,

    /// <summary>A dynamic body that is asleep.</summary>
    Asleep = 3,
}

/// <summary>A shape in world space: its points are <see cref="DebugGeometry.Points"/>[<see cref="FirstPoint"/> .. <see cref="FirstPoint"/> + <see cref="PointCount"/>).</summary>
public readonly struct DebugShape
{
    /// <summary>Creates an entry.</summary>
    public DebugShape(BodyHandle body, ShapeHandle shape, DebugShapeKind kind, bool isSensor, float radius, int firstPoint, int pointCount)
    {
        Body = body; Shape = shape; Kind = kind; IsSensor = isSensor; Radius = radius; FirstPoint = firstPoint; PointCount = pointCount;
    }

    /// <summary>The owning body.</summary>
    public BodyHandle Body { get; }

    /// <summary>The shape.</summary>
    public ShapeHandle Shape { get; }

    /// <summary>What the points mean.</summary>
    public DebugShapeKind Kind { get; }

    /// <summary>True for a sensor shape.</summary>
    public bool IsSensor { get; }

    /// <summary>The rounding radius (circle radius for circles; 0 for sharp polygons and segments).</summary>
    public float Radius { get; }

    /// <summary>Index of the first point in <see cref="DebugGeometry.Points"/>.</summary>
    public int FirstPoint { get; }

    /// <summary>Number of points.</summary>
    public int PointCount { get; }
}

/// <summary>An axis-aligned box of a shape. A 2D world leaves z at 0.</summary>
public readonly struct DebugBox
{
    /// <summary>Creates an entry.</summary>
    public DebugBox(BodyHandle body, ShapeHandle shape, Vector3D min, Vector3D max)
    {
        Body = body; Shape = shape; Min = min; Max = max;
    }

    /// <summary>The owning body.</summary>
    public BodyHandle Body { get; }

    /// <summary>The shape.</summary>
    public ShapeHandle Shape { get; }

    /// <summary>Lower corner.</summary>
    public Vector3D Min { get; }

    /// <summary>Upper corner.</summary>
    public Vector3D Max { get; }
}

/// <summary>One manifold point of a touching pair.</summary>
public readonly struct DebugContact
{
    /// <summary>Creates an entry.</summary>
    public DebugContact(BodyHandle bodyA, BodyHandle bodyB, ShapeHandle shapeA, ShapeHandle shapeB, Vector3D point, Vector3F normal, float separation, float normalImpulse)
    {
        BodyA = bodyA; BodyB = bodyB; ShapeA = shapeA; ShapeB = shapeB; Point = point; Normal = normal; Separation = separation; NormalImpulse = normalImpulse;
    }

    /// <summary>First body.</summary>
    public BodyHandle BodyA { get; }

    /// <summary>Second body.</summary>
    public BodyHandle BodyB { get; }

    /// <summary>Shape of A.</summary>
    public ShapeHandle ShapeA { get; }

    /// <summary>Shape of B.</summary>
    public ShapeHandle ShapeB { get; }

    /// <summary>World position of the point.</summary>
    public Vector3D Point { get; }

    /// <summary>Unit normal from A towards B.</summary>
    public Vector3F Normal { get; }

    /// <summary>Signed gap along the normal (negative when overlapping).</summary>
    public float Separation { get; }

    /// <summary>The solver's accumulated normal impulse of the last substep at this point.</summary>
    public float NormalImpulse { get; }
}

/// <summary>A body for drawing centres of mass, sleep state and islands.</summary>
public readonly struct DebugBody
{
    /// <summary>Creates an entry.</summary>
    public DebugBody(BodyHandle body, DebugBodyState state, Vector3D origin, Vector3D centerOfMass, int island)
    {
        Body = body; State = state; Origin = origin; CenterOfMass = centerOfMass; Island = island;
    }

    /// <summary>The body.</summary>
    public BodyHandle Body { get; }

    /// <summary>Type and sleep state.</summary>
    public DebugBodyState State { get; }

    /// <summary>The body's origin in world coordinates.</summary>
    public Vector3D Origin { get; }

    /// <summary>The centre of mass in world coordinates.</summary>
    public Vector3D CenterOfMass { get; }

    /// <summary>Island id of a dynamic body (bodies with equal ids can touch each other and sleep together); -1 for static and kinematic bodies.</summary>
    public int Island { get; }
}

/// <summary>A joint as two world anchors.</summary>
public readonly struct DebugJoint
{
    /// <summary>Creates an entry.</summary>
    public DebugJoint(JointHandle joint, BodyHandle bodyA, BodyHandle bodyB, Vector3D anchorA, Vector3D anchorB)
    {
        Joint = joint; BodyA = bodyA; BodyB = bodyB; AnchorA = anchorA; AnchorB = anchorB;
    }

    /// <summary>Creates an entry with a joint axis.</summary>
    public DebugJoint(JointHandle joint, BodyHandle bodyA, BodyHandle bodyB, Vector3D anchorA, Vector3D anchorB, Vector3D axis)
        : this(joint, bodyA, bodyB, anchorA, anchorB)
    {
        Axis = axis;
    }

    /// <summary>The joint.</summary>
    public JointHandle Joint { get; }

    /// <summary>First body.</summary>
    public BodyHandle BodyA { get; }

    /// <summary>Second body.</summary>
    public BodyHandle BodyB { get; }

    /// <summary>The anchor on A in world coordinates.</summary>
    public Vector3D AnchorA { get; }

    /// <summary>The anchor on B in world coordinates.</summary>
    public Vector3D AnchorB { get; }

    /// <summary>The joint's sliding axis as a world-space unit vector (prismatic and wheel joints); zero for joints without one.</summary>
    public Vector3D Axis { get; }
}

/// <summary>
/// Plain data a host draws: nothing here calls back. Create one buffer and pass it to <c>GetDebugGeometry</c> every frame; its lists keep their capacity, so after the first few frames filling it allocates nothing.
/// The same types serve 2D and 3D worlds (a 2D world leaves z at 0). Entries come in a fixed order: shapes, boxes and broadphase boxes by shape slot, contacts by constraint index, bodies by body slot.
/// </summary>
public sealed class DebugGeometry
{
    private DebugShape[] _shapes = new DebugShape[16];
    private Vector3D[] _points = new Vector3D[64];
    private DebugBox[] _bounds = new DebugBox[16];
    private DebugBox[] _fat = new DebugBox[16];
    private DebugContact[] _contacts = new DebugContact[16];
    private DebugBody[] _bodies = new DebugBody[16];
    private DebugJoint[] _joints = new DebugJoint[4];
    private int _nShapes, _nPoints, _nBounds, _nFat, _nContacts, _nBodies, _nJoints;

    /// <summary>The shapes in world space.</summary>
    public ReadOnlySpan<DebugShape> Shapes => _shapes.AsSpan(0, _nShapes);

    /// <summary>The points the shapes refer to.</summary>
    public ReadOnlySpan<Vector3D> Points => _points.AsSpan(0, _nPoints);

    /// <summary>Tight shape boxes.</summary>
    public ReadOnlySpan<DebugBox> Bounds => _bounds.AsSpan(0, _nBounds);

    /// <summary>The broadphase's stored boxes.</summary>
    public ReadOnlySpan<DebugBox> BroadphaseBounds => _fat.AsSpan(0, _nFat);

    /// <summary>Manifold points.</summary>
    public ReadOnlySpan<DebugContact> Contacts => _contacts.AsSpan(0, _nContacts);

    /// <summary>Bodies.</summary>
    public ReadOnlySpan<DebugBody> Bodies => _bodies.AsSpan(0, _nBodies);

    /// <summary>Joints.</summary>
    public ReadOnlySpan<DebugJoint> Joints => _joints.AsSpan(0, _nJoints);

    /// <summary>Empties every list (capacity is kept).</summary>
    public void Clear() => _nShapes = _nPoints = _nBounds = _nFat = _nContacts = _nBodies = _nJoints = 0;

    private static void Push<T>(ref T[] a, ref int n, in T v)
    {
        if (n == a.Length) { Array.Resize(ref a, a.Length * 2); }
        a[n++] = v;
    }

    /// <summary>Appends a point and returns its index (for engines filling the buffer).</summary>
    public int AddPoint(in Vector3D p) { int i = _nPoints; Push(ref _points, ref _nPoints, in p); return i; }

    /// <summary>Appends a shape entry.</summary>
    public void Add(in DebugShape s) => Push(ref _shapes, ref _nShapes, in s);

    /// <summary>Appends a tight box.</summary>
    public void AddBounds(in DebugBox b) => Push(ref _bounds, ref _nBounds, in b);

    /// <summary>Appends a broadphase box.</summary>
    public void AddBroadphaseBounds(in DebugBox b) => Push(ref _fat, ref _nFat, in b);

    /// <summary>Appends a contact point.</summary>
    public void Add(in DebugContact c) => Push(ref _contacts, ref _nContacts, in c);

    /// <summary>Appends a body entry.</summary>
    public void Add(in DebugBody b) => Push(ref _bodies, ref _nBodies, in b);

    /// <summary>Appends a joint entry.</summary>
    public void Add(in DebugJoint j) => Push(ref _joints, ref _nJoints, in j);
}
