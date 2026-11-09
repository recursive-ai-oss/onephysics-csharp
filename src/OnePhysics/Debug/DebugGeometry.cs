using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

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

    /// <summary>One entry per joint (<see cref="DebugGeometry.Joints"/>); one per live joint, in constraint-slot order. Also emits one <see cref="DebugGeometry.ActiveRagdolls"/> entry per active ragdoll.</summary>
    Joints = 32,

    /// <summary>One entry per character with its capsule, ground state and ground normal (<see cref="DebugGeometry.Characters"/>); worlds without characters emit none.</summary>
    Characters = 64,

    /// <summary>With <see cref="Shapes"/>: triangle meshes and height fields emit their triangles too (large; off by default). Ignored by 2D worlds.</summary>
    MeshTriangles = 128,

    /// <summary>One entry per wheel of every vehicle (<see cref="DebugGeometry.Wheels"/>): suspension top, wheel centre, contact point and normal.</summary>
    Vehicles = 256,

    /// <summary>One entry per soft body with its particles and edges (<see cref="DebugGeometry.SoftBodies"/>, <see cref="DebugGeometry.Points"/>, <see cref="DebugGeometry.Indices"/>); in 2D worlds the points have z = 0.</summary>
    SoftBodies = 512,

    /// <summary>Body/island states, constraint colours, joint row errors and force/impulse arrows as plain data.</summary>
    Inspection = 1024,

    /// <summary>Field and effector bounds, axes and magnitudes as plain data.</summary>
    Fields = 2048,

    /// <summary>Rod segment polylines and material frames.</summary>
    Rods = 4096,

    /// <summary>Rope spans and wheel arcs of pulley systems.</summary>
    Pulleys = 8192,
    /// <summary>Aerodynamic and hydrodynamic surfaces and the forces of thrusters, boats, hovercraft and surfaces as plain data (<see cref="DebugGeometry.AeroSurfaces"/>, <see cref="DebugGeometry.AeroForces"/>).</summary>
    Aero = 16384,

    /// <summary>Where characters are held: seats, ledges, climbed surfaces and rope grips (<see cref="DebugGeometry.CharacterHolds"/>).</summary>
    CharacterHolds = 32768,

    /// <summary>Seams, attachments and tethers of cloth and the wind force on each triangle of an aerodynamic cloth (<see cref="DebugGeometry.ClothLinks"/>, <see cref="DebugGeometry.ClothWind"/>).</summary>
    Cloth = 65536,

    /// <summary>Everything.</summary>
    All = 131071,
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

    /// <summary>3D: the eight corners of a box (x, y, z bits of the index choose the sign; corner 0 is the lowest), rounded by the radius; twelve triangles in <see cref="DebugGeometry.Indices"/>.</summary>
    Box = 5,

    /// <summary>3D: the two ends of the axis and the radius.</summary>
    Cylinder = 6,

    /// <summary>3D: the two cap centres; <see cref="DebugShape.Radius"/> is the first cap's radius and <see cref="DebugShape.BottomRadius"/> the second's.</summary>
    TaperedCapsule = 7,

    /// <summary>3D: the hull's vertices; its faces as triangles (a fan per face, outward winding) in <see cref="DebugGeometry.Indices"/>.</summary>
    Hull = 8,

    /// <summary>3D: the mesh's vertices; its triangles in <see cref="DebugGeometry.Indices"/> only with <see cref="DebugContent.MeshTriangles"/>. The points are the mesh's bounding box corners otherwise.</summary>
    Mesh = 9,

    /// <summary>3D: like <see cref="Mesh"/> for a height field; holes have no triangles.</summary>
    HeightField = 10,

    /// <summary>3D: bottom and top axis ends; Radius is the top radius and BottomRadius the bottom radius.</summary>
    TaperedCylinder = 11,

    /// <summary>No geometry or points; a placeholder shape.</summary>
    Empty = 12,

    /// <summary>3D infinite half-space: a surface point and that point plus the outward unit normal. Solid is opposite the normal.</summary>
    Plane = 13,

    /// <summary>3D exposed voxel faces: like Mesh, with no internal faces. Triangle ids retain their cell and face identity.</summary>
    VoxelGrid = 14,
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

    /// <summary>Creates an entry with a second radius and triangle indices (3D shapes).</summary>
    public DebugShape(BodyHandle body, ShapeHandle shape, DebugShapeKind kind, bool isSensor, float radius, int firstPoint, int pointCount, float bottomRadius, int firstIndex, int indexCount)
        : this(body, shape, kind, isSensor, radius, firstPoint, pointCount)
    {
        BottomRadius = bottomRadius; FirstIndex = firstIndex; IndexCount = indexCount;
    }

    /// <summary>Tapered capsules: the second cap's radius; otherwise 0.</summary>
    public float BottomRadius { get; }

    /// <summary>Index of the first triangle corner in <see cref="DebugGeometry.Indices"/>; the entry's triangles are <see cref="IndexCount"/> values from there, three per triangle, each a point index relative to <see cref="FirstPoint"/>.</summary>
    public int FirstIndex { get; }

    /// <summary>Number of triangle corner indices (0 for shapes drawn from their points alone).</summary>
    public int IndexCount { get; }

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

/// <summary>A soft body for drawing: its particle positions are <see cref="DebugGeometry.Points"/>[FirstPoint .. FirstPoint + ParticleCount) and its edges are pairs of particle numbers in <see cref="DebugGeometry.Indices"/>[FirstIndex .. FirstIndex + 2 * EdgeCount).</summary>
public readonly struct DebugSoftBody
{
    /// <summary>Creates an entry.</summary>
    public DebugSoftBody(int index, int generation, bool isAsleep, float particleRadius, int firstPoint, int particleCount, int firstIndex, int edgeCount)
    {
        Index = index; Generation = generation; IsAsleep = isAsleep; ParticleRadius = particleRadius; FirstPoint = firstPoint; ParticleCount = particleCount; FirstIndex = firstIndex; EdgeCount = edgeCount;
    }

    /// <summary>Creates an entry for a skinned body, with its skinned target positions in <see cref="DebugGeometry.Points"/>[FirstTargetPoint .. FirstTargetPoint + ParticleCount).</summary>
    public DebugSoftBody(int index, int generation, bool isAsleep, float particleRadius, int firstPoint, int particleCount, int firstIndex, int edgeCount, int firstTargetPoint)
        : this(index, generation, isAsleep, particleRadius, firstPoint, particleCount, firstIndex, edgeCount)
    {
        FirstTargetPoint = firstTargetPoint;
    }

    /// <summary>Creates an entry that also lists the torn edges: <paramref name="brokenEdgeCount"/> pairs of particle numbers from <see cref="DebugGeometry.Indices"/>[<paramref name="firstBrokenIndex"/> ..).</summary>
    public DebugSoftBody(int index, int generation, bool isAsleep, float particleRadius, int firstPoint, int particleCount, int firstIndex, int edgeCount, int firstTargetPoint, int firstBrokenIndex, int brokenEdgeCount)
        : this(index, generation, isAsleep, particleRadius, firstPoint, particleCount, firstIndex, edgeCount, firstTargetPoint)
    {
        FirstBrokenIndex = firstBrokenIndex; BrokenEdgeCount = brokenEdgeCount;
    }

    /// <summary>Index in <see cref="DebugGeometry.Indices"/> of the first corner of the first torn edge that still joins two sides without a constraint, or -1 when there is none.</summary>
    public int FirstBrokenIndex { get; } = -1;

    /// <summary>Number of such torn edges (two particle numbers each, from <see cref="FirstBrokenIndex"/>).</summary>
    public int BrokenEdgeCount { get; }

    /// <summary>Index in <see cref="DebugGeometry.Points"/> of the skinned position (the target) of particle 0, or -1 when the body has no skin. The targets of the other particles follow it, in particle order.</summary>
    public int FirstTargetPoint { get; } = -1;

    /// <summary>The soft body's slot in its world.</summary>
    public int Index { get; }

    /// <summary>The slot's generation.</summary>
    public int Generation { get; }

    /// <summary>True while the body sleeps.</summary>
    public bool IsAsleep { get; }

    /// <summary>Radius of the sphere each particle collides as, metres.</summary>
    public float ParticleRadius { get; }

    /// <summary>Index of particle 0 in <see cref="DebugGeometry.Points"/>.</summary>
    public int FirstPoint { get; }

    /// <summary>Number of particles.</summary>
    public int ParticleCount { get; }

    /// <summary>Index of the first edge corner in <see cref="DebugGeometry.Indices"/>.</summary>
    public int FirstIndex { get; }

    /// <summary>Number of edges (two particle numbers each).</summary>
    public int EdgeCount { get; }
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

/// <summary>
/// An active ragdoll's controller as plain data: its centre of mass, the capture point on the ground, the support box (bounding box of the grounded feet's soles) and, while a foot swings, the point it is aimed at.
/// Emitted with <see cref="DebugContent.Joints"/>, in ragdoll slot order; 2D worlds fill X and Y (Z is 0).
/// </summary>
[System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
public readonly struct DebugActiveRagdoll
{
    /// <summary>Creates an entry.</summary>
    public DebugActiveRagdoll(int index, int generation, ActiveMode mode, GaitPhase phase, int groundedFeet, Vector3D centerOfMass, Vector3D capturePoint, Vector3D supportMin, Vector3D supportMax, bool hasLanding, Vector3D landing)
    {
        Index = index; Generation = generation; Mode = mode; Phase = phase; GroundedFeet = groundedFeet; CenterOfMass = centerOfMass; CapturePoint = capturePoint;
        SupportMin = supportMin; SupportMax = supportMax; HasLanding = hasLanding; Landing = landing;
    }

    /// <summary>The ragdoll's slot index.</summary>
    public int Index { get; }

    /// <summary>The ragdoll's generation.</summary>
    public int Generation { get; }

    /// <summary>The controller mode.</summary>
    public ActiveMode Mode { get; }

    /// <summary>The gait's finite state.</summary>
    public GaitPhase Phase { get; }

    /// <summary>How many feet were on the ground.</summary>
    public int GroundedFeet { get; }

    /// <summary>Centre of mass of the whole figure.</summary>
    public Vector3D CenterOfMass { get; }

    /// <summary>The capture point (centre of mass plus velocity over the pendulum frequency), at ground height under the support.</summary>
    public Vector3D CapturePoint { get; }

    /// <summary>One corner of the support box (minimum along the figure's forward and left axes, in world x and z).</summary>
    public Vector3D SupportMin { get; }

    /// <summary>The opposite corner of the support box.</summary>
    public Vector3D SupportMax { get; }

    /// <summary>True while a foot swings and <see cref="Landing"/> holds where it is aimed.</summary>
    public bool HasLanding { get; }

    /// <summary>The swing foot's target on the ground (meaningful only with <see cref="HasLanding"/>).</summary>
    public Vector3D Landing { get; }
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

    /// <summary>Creates an entry with the limits of a 3D joint.</summary>
    public DebugJoint(JointHandle joint, BodyHandle bodyA, BodyHandle bodyB, Vector3D anchorA, Vector3D anchorB, Vector3D axis, Vector3D referenceAxis, int type, float lowerLimit, float upperLimit, float coneAngle)
        : this(joint, bodyA, bodyB, anchorA, anchorB, axis)
    {
        ReferenceAxis = referenceAxis; Type = type; LowerLimit = lowerLimit; UpperLimit = upperLimit; ConeAngle = coneAngle;
    }

    /// <summary>3D: the joint's second main axis (frame B's x axis in world coordinates); with <see cref="Axis"/> (frame A's x axis) it spans a limit cone. Zero for 2D joints.</summary>
    public Vector3D ReferenceAxis { get; }

    /// <summary>3D: the joint type as the world's joint-type enum value; 0 for 2D joints.</summary>
    public int Type { get; }

    /// <summary>3D: lower limit (radians for angular limits, metres for linear), 0 when the joint has none.</summary>
    public float LowerLimit { get; }

    /// <summary>3D: upper limit; see <see cref="LowerLimit"/>.</summary>
    public float UpperLimit { get; }

    /// <summary>3D: the cone half angle of a cone or swing-twist joint's swing limit in radians, 0 otherwise.</summary>
    public float ConeAngle { get; }

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

/// <summary>A vehicle wheel as plain data: where its suspension starts, where the wheel centre is, its size and axle, and what it touches.</summary>
public readonly struct DebugWheel
{
    /// <summary>Creates an entry.</summary>
    public DebugWheel(BodyHandle chassis, int wheel, Vector3D suspensionTop, Vector3D center, float radius, float width, Vector3F axle, bool isInContact, Vector3D contactPoint, Vector3F contactNormal)
    {
        Chassis = chassis; Wheel = wheel; SuspensionTop = suspensionTop; Center = center; Radius = radius; Width = width; Axle = axle; IsInContact = isInContact; ContactPoint = contactPoint; ContactNormal = contactNormal;
    }

    /// <summary>The vehicle's chassis body.</summary>
    public BodyHandle Chassis { get; }

    /// <summary>The wheel's index in its vehicle.</summary>
    public int Wheel { get; }

    /// <summary>The top of the suspension (the wheel centre at zero length), world coordinates.</summary>
    public Vector3D SuspensionTop { get; }

    /// <summary>The wheel centre.</summary>
    public Vector3D Center { get; }

    /// <summary>Wheel radius.</summary>
    public float Radius { get; }

    /// <summary>Wheel width.</summary>
    public float Width { get; }

    /// <summary>Unit axle direction in world space (steered).</summary>
    public Vector3F Axle { get; }

    /// <summary>True when the wheel touched ground in the last step.</summary>
    public bool IsInContact { get; }

    /// <summary>The contact point; zero when not in contact.</summary>
    public Vector3D ContactPoint { get; }

    /// <summary>The ground normal at the contact; zero when not in contact.</summary>
    public Vector3F ContactNormal { get; }
}

/// <summary>A track as plain data: its centreline sampled along its length, and its gauge. Emitted with <see cref="DebugContent.Vehicles"/>.</summary>
public readonly struct DebugTrack
{
    /// <summary>Creates an entry.</summary>
    public DebugTrack(int track, Vector3D[] centerline, float gauge) { Track = track; Centerline = centerline; Gauge = gauge; }

    /// <summary>The track's slot.</summary>
    public int Track { get; }

    /// <summary>Points on the centre of the track at rail-head height, from its start to its end.</summary>
    public Vector3D[] Centerline { get; }

    /// <summary>Distance between the inner rail faces, m.</summary>
    public float Gauge { get; }
}

/// <summary>One wheel-rail contact of a rail vehicle's wheelset as plain data: where it is, the surface normal, the normal force and the tangential (creep) force. Emitted with <see cref="DebugContent.Vehicles"/>.</summary>
public readonly struct DebugRailContact
{
    /// <summary>Creates an entry.</summary>
    public DebugRailContact(BodyHandle wheelset, int index, Vector3D point, Vector3F normal, float normalForce, Vector3F creepForce, bool isFlange)
    {
        Wheelset = wheelset; Index = index; Point = point; Normal = normal; NormalForce = normalForce; CreepForce = creepForce; IsFlange = isFlange;
    }

    /// <summary>The wheelset's body.</summary>
    public BodyHandle Wheelset { get; }

    /// <summary>The wheelset's index in its vehicle.</summary>
    public int Index { get; }

    /// <summary>The contact point on the rail head, world coordinates.</summary>
    public Vector3D Point { get; }

    /// <summary>The track's up direction at the contact.</summary>
    public Vector3F Normal { get; }

    /// <summary>The wheel-rail normal force (tread plus flange) in newtons.</summary>
    public float NormalForce { get; }

    /// <summary>The tangential creep force on the wheel in newtons, in world axes.</summary>
    public Vector3F CreepForce { get; }

    /// <summary>True when the flange touches the rail's gauge face.</summary>
    public bool IsFlange { get; }
}

/// <summary>A character as plain data: its capsule (vertical axis, centred on <see cref="Center"/>) and what it stands on.</summary>
public readonly struct DebugCharacter
{
    /// <summary>Creates an entry.</summary>
    public DebugCharacter(BodyHandle body, Vector3D center, float radius, float height, bool isGrounded, Vector3D groundNormal, BodyHandle groundBody)
    {
        Body = body; Center = center; Radius = radius; Height = height; IsGrounded = isGrounded; GroundNormal = groundNormal; GroundBody = groundBody;
    }

    /// <summary>The character's (kinematic) body.</summary>
    public BodyHandle Body { get; }

    /// <summary>The centre of the capsule in world coordinates.</summary>
    public Vector3D Center { get; }

    /// <summary>Capsule radius.</summary>
    public float Radius { get; }

    /// <summary>Total capsule height, end to end.</summary>
    public float Height { get; }

    /// <summary>True when the character stood on walkable ground after its last move.</summary>
    public bool IsGrounded { get; }

    /// <summary>The unit normal of the ground it stands on; zero when not grounded.</summary>
    public Vector3D GroundNormal { get; }

    /// <summary>The body it stands on; the default handle when not grounded.</summary>
    public BodyHandle GroundBody { get; }
}

/// <summary>
/// Plain data a host draws: nothing here calls back. Create one buffer and pass it to <c>GetDebugGeometry</c> every frame; its lists keep their capacity, so after the first few frames filling it allocates nothing.
/// The same types serve 2D and 3D worlds (a 2D world leaves z at 0). Entries come in a fixed order: shapes, boxes and broadphase boxes by shape slot, contacts by constraint index, bodies by body slot.
/// </summary>
public sealed partial class DebugGeometry
{
    private DebugShape[] _shapes = new DebugShape[16];
    private Vector3D[] _points = new Vector3D[64];
    private DebugBox[] _bounds = new DebugBox[16];
    private DebugBox[] _fat = new DebugBox[16];
    private DebugContact[] _contacts = new DebugContact[16];
    private DebugBody[] _bodies = new DebugBody[16];
    private DebugJoint[] _joints = new DebugJoint[4];
    private DebugActiveRagdoll[] _active = new DebugActiveRagdoll[2];
    private int _nActive;
    private DebugCharacter[] _characters = new DebugCharacter[4];
    private DebugWheel[] _wheels = new DebugWheel[4];
    private DebugTrack[] _tracks = new DebugTrack[2];
    private DebugRailContact[] _railContacts = new DebugRailContact[4];
    private int _nTracks, _nRailContacts;
    private DebugSoftBody[] _soft = new DebugSoftBody[4];
    private DebugClothLink[] _clothLinks = new DebugClothLink[4];
    private DebugClothWind[] _clothWind = new DebugClothWind[4];
    private int _nClothLinks, _nClothWind;
    private int[] _indices = new int[64];
    private int _nIndices;
    private int _nShapes, _nPoints, _nBounds, _nFat, _nContacts, _nBodies, _nJoints, _nCharacters, _nWheels, _nSoft;

    /// <summary>The shapes in world space.</summary>
    public ReadOnlySpan<DebugShape> Shapes => _shapes.AsSpan(0, _nShapes);

    /// <summary>The points the shapes refer to.</summary>
    public ReadOnlySpan<Vector3D> Points => _points.AsSpan(0, _nPoints);

    /// <summary>Triangle corner indices of 3D shapes, each relative to its shape's first point.</summary>
    public ReadOnlySpan<int> Indices => _indices.AsSpan(0, _nIndices);

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

    /// <summary>Active ragdolls' controller state (emitted with <see cref="DebugContent.Joints"/>).</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public ReadOnlySpan<DebugActiveRagdoll> ActiveRagdolls => _active.AsSpan(0, _nActive);

    /// <summary>Characters, by character slot.</summary>
    public ReadOnlySpan<DebugCharacter> Characters => _characters.AsSpan(0, _nCharacters);

    /// <summary>Vehicle wheels, by vehicle slot then wheel index.</summary>
    public ReadOnlySpan<DebugWheel> Wheels => _wheels.AsSpan(0, _nWheels);

    /// <summary>Tracks, by slot (with the vehicles content).</summary>
    public ReadOnlySpan<DebugTrack> Tracks => _tracks.AsSpan(0, _nTracks);

    /// <summary>Wheel-rail contacts of rail vehicles, by vehicle slot, wheelset and side (with the vehicles content).</summary>
    public ReadOnlySpan<DebugRailContact> RailContacts => _railContacts.AsSpan(0, _nRailContacts);

    /// <summary>Soft bodies, by slot.</summary>
    public ReadOnlySpan<DebugSoftBody> SoftBodies => _soft.AsSpan(0, _nSoft);

    /// <summary>Seams, attachments and tethers of cloth, when <see cref="DebugContent.Cloth"/> was asked for: seams and attachments by link number, then tethers by soft body and particle.</summary>
    public ReadOnlySpan<DebugClothLink> ClothLinks => _clothLinks.AsSpan(0, _nClothLinks);

    /// <summary>Wind forces on the triangles of aerodynamic cloths, by soft body and triangle, when <see cref="DebugContent.Cloth"/> was asked for.</summary>
    public ReadOnlySpan<DebugClothWind> ClothWind => _clothWind.AsSpan(0, _nClothWind);

    /// <summary>Empties every list (capacity is kept).</summary>
    public void Clear() { _nHolds=0; _nActive = 0; _nPulleySpans=0; _nPulleyWheels=0; _nAeroSurfaces=0; _nAeroForces=0; _nRodSegments=0; _nFields=0; ClearInspection(); _nIndices = _nShapes = _nPoints = _nBounds = _nFat = _nContacts = _nBodies = _nJoints = _nCharacters = _nWheels = _nSoft = 0; _nTracks = _nRailContacts = 0; _nClothLinks = _nClothWind = 0; }

    private static void Push<T>(ref T[] a, ref int n, in T v)
    {
        if (n == a.Length) { Array.Resize(ref a, a.Length * 2); }
        a[n++] = v;
    }

    /// <summary>Appends a point and returns its index (for engines filling the buffer).</summary>
    public int AddPoint(in Vector3D p) { int i = _nPoints; Push(ref _points, ref _nPoints, in p); return i; }

    /// <summary>Appends a triangle corner index and returns its position.</summary>
    public int AddIndex(int i) { int k = _nIndices; Push(ref _indices, ref _nIndices, in i); return k; }

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

    /// <summary>Appends a character entry.</summary>
    public void Add(in DebugCharacter c) => Push(ref _characters, ref _nCharacters, in c);

    /// <summary>Appends a wheel entry.</summary>
    public void Add(in DebugWheel w) => Push(ref _wheels, ref _nWheels, in w);

    /// <summary>Appends a track entry.</summary>
    public void Add(in DebugTrack t) => Push(ref _tracks, ref _nTracks, in t);

    /// <summary>Appends a rail contact entry.</summary>
    public void Add(in DebugRailContact c) => Push(ref _railContacts, ref _nRailContacts, in c);

    /// <summary>Appends a soft body entry.</summary>
    public void Add(in DebugSoftBody b) => Push(ref _soft, ref _nSoft, in b);

    /// <summary>Appends a cloth link.</summary>
    public void Add(in DebugClothLink l) => Push(ref _clothLinks, ref _nClothLinks, in l);

    /// <summary>Appends a cloth wind force.</summary>
    public void Add(in DebugClothWind w) => Push(ref _clothWind, ref _nClothWind, in w);

    /// <summary>Appends a joint entry.</summary>
    public void Add(in DebugJoint j) => Push(ref _joints, ref _nJoints, in j);

    /// <summary>Appends an active ragdoll entry.</summary>
    [System.Diagnostics.CodeAnalysis.Experimental("OPHYS0003")]
    public void Add(in DebugActiveRagdoll a) => Push(ref _active, ref _nActive, in a);
}
