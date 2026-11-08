using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Debug;

/// <summary>The physical quantity represented by an inspection arrow; the viewer chooses a display scale.</summary>
public enum DebugVectorKind
{
    /// <summary>Final-substep normal impulse, kg m/s, applied to B.</summary>
    NormalImpulse,
    /// <summary>Final-substep combined tangent impulse, kg m/s, applied to B.</summary>
    TangentImpulse,
    /// <summary>Whole-step mean joint force on B, N.</summary>
    JointForce,
    /// <summary>Whole-step mean joint torque about B's centre of mass, N m.</summary>
    JointTorque,
}

/// <summary>An arrow with physical world-space components, not a pre-scaled endpoint.</summary>
public readonly struct DebugInspectionVector
{
    internal DebugInspectionVector(int constraint, Vector3D origin, Vector3D value, DebugVectorKind kind)
    { Constraint = constraint; Origin = origin; Value = value; Kind = kind; }
    /// <summary>Stable constraint slot.</summary>
    public int Constraint { get; }
    /// <summary>World-space arrow origin in metres.</summary>
    public Vector3D Origin { get; }
    /// <summary>World-space vector in the units of Kind.</summary>
    public Vector3D Value { get; }
    /// <summary>Physical quantity and units.</summary>
    public DebugVectorKind Kind { get; }
}

/// <summary>One constraint's colour, islands and endpoints, sufficient for a constraint-graph overlay.</summary>
public readonly struct DebugConstraintInspection
{
    internal DebugConstraintInspection(int slot, bool joint, byte color, int islandA, int islandB, Vector3D a, Vector3D b)
    { Constraint = slot; IsJoint = joint; Color = color; IslandA = islandA; IslandB = islandB; PositionA = a; PositionB = b; }
    /// <summary>Constraint slot.</summary>
    public int Constraint { get; }
    /// <summary>True for a joint, false for a contact.</summary>
    public bool IsJoint { get; }
    /// <summary>Colour 0..23, 255 for overflow, 254 for uncoloured.</summary>
    public byte Color { get; }
    /// <summary>Island of A, or -1.</summary>
    public int IslandA { get; }
    /// <summary>Island of B, or -1.</summary>
    public int IslandB { get; }
    /// <summary>A's centre of mass in world metres.</summary>
    public Vector3D PositionA { get; }
    /// <summary>B's centre of mass in world metres.</summary>
    public Vector3D PositionB { get; }
}

/// <summary>Plain-data joint error label; the flags specify its units, row role and activity.</summary>
public readonly struct DebugJointError
{
    internal DebugJointError(int constraint, int row, Vector3D at, JointRowInspection error) { Constraint = constraint; Row = row; Position = at; Error = error; }
    /// <summary>Joint constraint slot.</summary>
    public int Constraint { get; }
    /// <summary>Persistent impulse slot of the row.</summary>
    public int Row { get; }
    /// <summary>Label position, B's centre of mass, in world metres.</summary>
    public Vector3D Position { get; }
    /// <summary>Signed solver error and its units/activity flags.</summary>
    public JointRowInspection Error { get; }
}

public sealed partial class DebugGeometry
{
    private DebugInspectionVector[] _inspectionVectors = new DebugInspectionVector[16];
    private DebugConstraintInspection[] _constraintInspection = new DebugConstraintInspection[16];
    private DebugJointError[] _jointErrors = new DebugJointError[16];
    private int _nInspectionVectors, _nConstraintInspection, _nJointErrors;
    /// <summary>Force, torque and impulse arrows, in ascending constraint slot then point order.</summary>
    public ReadOnlySpan<DebugInspectionVector> InspectionVectors => _inspectionVectors.AsSpan(0, _nInspectionVectors);
    /// <summary>Constraint colours, islands and endpoints in ascending slot order.</summary>
    public ReadOnlySpan<DebugConstraintInspection> ConstraintInspection => _constraintInspection.AsSpan(0, _nConstraintInspection);
    /// <summary>Enabled joint row errors, in ascending constraint slot then row slot order.</summary>
    public ReadOnlySpan<DebugJointError> JointErrors => _jointErrors.AsSpan(0, _nJointErrors);
    private void ClearInspection() => _nInspectionVectors = _nConstraintInspection = _nJointErrors = 0;

    /// <summary>Appends raw inspection overlays. Read between steps; grows owned arrays only when capacity is insufficient. Null world throws ArgumentNullException.</summary>
    public void AddInspection(WorldCore world)
    {
        ArgumentNullException.ThrowIfNull(world);
        var cs = world.Constraints; var bodies = world.Bodies;
        bool spatial = world.Layout.LinearLanes == 3;
        for (int c = 0; c < cs.HighWater; c++)
        {
            if (!cs.IsLive(c) || cs.KindOf(c) == ConstraintKind.Sensor) { continue; }
            int a = cs.BodyA[c], b = cs.BodyB[c];
            var pa = bodies.PositionOf(a); var pb = bodies.PositionOf(b);
            var posA = new Vector3D(pa[0], pa[1], spatial ? pa[2] : 0);
            var posB = new Vector3D(pb[0], pb[1], spatial ? pb[2] : 0);
            bool joint = cs.KindOf(c) == ConstraintKind.Joint;
            var entry = new DebugConstraintInspection(c, joint, cs.Color[c], bodies.Island[a], bodies.Island[b], posA, posB);
            Push(ref _constraintInspection, ref _nConstraintInspection, in entry);
            if (joint)
            {
                var view = world.JointReadback.Inspect(world.HandleOfJoint(c));
                var force = new DebugInspectionVector(c, posB, view.ForceB, DebugVectorKind.JointForce);
                var torque = new DebugInspectionVector(c, posB, view.TorqueB, DebugVectorKind.JointTorque);
                Push(ref _inspectionVectors, ref _nInspectionVectors, in force); Push(ref _inspectionVectors, ref _nInspectionVectors, in torque);
                var rows = view.Rows;
                for (int r = 0; r < rows.Length; r++)
                {
                    if (rows[r].Flags == JointRowBits.None) { continue; }
                    var error = new DebugJointError(c, r, posB, rows[r]); Push(ref _jointErrors, ref _nJointErrors, in error);
                }
                continue;
            }
            var p = cs.Payload(c);
            var n = new Vector3F(p[0], p[1], spatial ? p[2] : 0);
            Vector3F t1, t2 = default;
            if (!spatial) { t1 = new(n.Y, -n.X, 0); }
            else
            {
                var t = Math.Abs(n.X) >= 0.57735f ? new Vector3F(n.Y, -n.X, 0) : new Vector3F(0, n.Z, -n.Y);
                float len = float.Sqrt(t.X * t.X + t.Y * t.Y + t.Z * t.Z);
                t1 = len > 0 ? t * (1f / len) : new Vector3F(1, 0, 0); t2 = Vector3F.Cross(n, t1);
            }
            for (int k = 0; k < cs.PointCount[c]; k++)
            {
                int arm = ManifoldLayout.AnchorA(k);
                var at = posA + new Vector3D(p[arm], p[arm + 1], spatial ? p[arm + 2] : 0);
                var normal = n * p[ManifoldLayout.NormalImpulse(k)];
                var tangent = t1 * p[ManifoldLayout.Tangent1Impulse(k)] + t2 * (spatial ? p[ManifoldLayout.Tangent2Impulse(k)] : 0);
                var nv = new DebugInspectionVector(c, at, new(normal.X, normal.Y, normal.Z), DebugVectorKind.NormalImpulse);
                var tv = new DebugInspectionVector(c, at, new(tangent.X, tangent.Y, tangent.Z), DebugVectorKind.TangentImpulse);
                Push(ref _inspectionVectors, ref _nInspectionVectors, in nv); Push(ref _inspectionVectors, ref _nInspectionVectors, in tv);
            }
        }
    }
}
