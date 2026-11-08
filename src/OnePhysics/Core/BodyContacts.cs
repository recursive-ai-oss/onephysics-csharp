using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Allocation-free read-only contact iteration in ascending constraint-slot order, including speculative contacts. Valid between world mutations.</summary>
public readonly struct BodyContactEnumerable
{
    private readonly WorldCore _world;
    private readonly BodyHandle _body;
    internal BodyContactEnumerable(WorldCore world, BodyHandle body) { _world = world; _body = body; }
    /// <summary>Creates an enumerator, validating the body handle. A default or destroyed body throws InvalidHandleException.</summary>
    public BodyContactEnumerator GetEnumerator() => new(_world, _body);
}

/// <summary>Contact enumerator. Mutation invalidates its views; destruction is checked on every advancement.</summary>
public struct BodyContactEnumerator
{
    private readonly WorldCore _world;
    private readonly BodyHandle _body;
    private int _slot;
    internal BodyContactEnumerator(WorldCore world, BodyHandle body) { _world = world; _body = body; _slot = -1; if (world == null) throw new InvalidHandleException("The default contact enumeration has no world."); world.Resolve(body); }
    /// <summary>Advances in ascending constraint slot order. Disabled response contacts with zero points and sensors are excluded.</summary>
    public bool MoveNext()
    {
        if (_world == null) throw new InvalidHandleException("The default contact enumeration has no world.");
        int b = _world.Resolve(_body); var cs = _world.Constraints;
        while (++_slot < cs.HighWater)
            if (cs.IsLive(_slot) && cs.KindOf(_slot) == ConstraintKind.Contact && cs.PointCount[_slot] != 0 && (cs.BodyA[_slot] == b || cs.BodyB[_slot] == b)) return true;
        return false;
    }
    /// <summary>Current read-only manifold. Throws InvalidOperationException when not positioned on a live contact.</summary>
    public ContactView Current
    {
        get
        {
            if (_world == null || (uint)_slot >= (uint)_world.Constraints.HighWater || !_world.Constraints.IsLive(_slot) || _world.Constraints.KindOf(_slot) != ConstraintKind.Contact) throw new InvalidOperationException("Contact enumeration is not positioned on a live contact.");
            _world.Resolve(_body); return new(_world, _slot);
        }
    }
}

/// <summary>Read-only solver manifold, valid until world mutation. Shape A is the lower shape index; normal points from A to B. All dimensions share this raw view.</summary>
public readonly ref struct ContactView
{
    private readonly WorldCore _world;
    internal ContactView(WorldCore world, int slot) { _world = world; Slot = slot; }
    /// <summary>Stable contact constraint slot.</summary>
    public int Slot { get; }
    /// <summary>First body.</summary>
    public BodyHandle BodyA => _world.HandleOfBody(_world.Constraints.BodyA[Slot]);
    /// <summary>Second body.</summary>
    public BodyHandle BodyB => _world.HandleOfBody(_world.Constraints.BodyB[Slot]);
    /// <summary>First shape.</summary>
    public ShapeHandle ShapeA => _world.HandleOfShape(_world.Constraints.ShapeA[Slot]);
    /// <summary>Second shape.</summary>
    public ShapeHandle ShapeB => _world.HandleOfShape(_world.Constraints.ShapeB[Slot]);
    /// <summary>Subshape of A, zero for simple convex shapes.</summary>
    public int SubShapeA => _world.Constraints.SubShapeA[Slot];
    /// <summary>Subshape of B, zero for simple convex shapes.</summary>
    public int SubShapeB => _world.Constraints.SubShapeB[Slot];
    /// <summary>Point count, 1..2 in 2D and 1..4 in 3D.</summary>
    public int PointCount => _world.Constraints.PointCount[Slot];
    /// <summary>World-space unit normal. Z is zero in 2D.</summary>
    public Vector3F Normal { get { var p = _world.Constraints.Payload(Slot); return new(p[0], p[1], _world.Layout.PositionLanes == 3 ? p[2] : 0); } }
    /// <summary>Optional audio/haptic speeds, zero unless contact motion collection is enabled.</summary>
    public ContactMotion Motion => _world.ContactMotion.ForConstraint(_world, Slot);
    /// <summary>Material index of the first triangle/segment, or -1 for a shape without indexed materials.</summary>
    public int MaterialIndexA => _world.ContactMaterials.MaterialIndex(ShapeA.Index, SubShapeA);
    /// <summary>Material index of the second triangle/segment, or -1 for a shape without indexed materials.</summary>
    public int MaterialIndexB => _world.ContactMaterials.MaterialIndex(ShapeB.Index, SubShapeB);
    /// <summary>Mixed first-axis Coulomb friction.</summary>
    public float Friction => _world.Constraints.Payload(Slot)[ManifoldLayout.Friction];
    /// <summary>Point position in double world coordinates; Z is zero in 2D. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public Vector3D Point(int index)
    {
        CheckPoint(index); var cs = _world.Constraints; var pos = _world.Bodies.PositionOf(cs.BodyA[Slot]); var p = cs.Payload(Slot); int a = ManifoldLayout.AnchorA(index);
        return new(pos[0] + p[a], pos[1] + p[a + 1], _world.Layout.PositionLanes == 3 ? pos[2] + p[a + 2] : 0);
    }
    /// <summary>Signed separation in metres at manifold update, negative inside. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public float Separation(int index) { CheckPoint(index); return _world.Constraints.Payload(Slot)[ManifoldLayout.Separation(index)]; }
    /// <summary>Final substep's accumulated normal impulse, newton seconds. Throws ArgumentOutOfRangeException for an invalid index.</summary>
    public float NormalImpulse(int index) { CheckPoint(index); return _world.Constraints.Payload(Slot)[ManifoldLayout.NormalImpulse(index)]; }
    private void CheckPoint(int index) { if ((uint)index >= (uint)PointCount) throw new ArgumentOutOfRangeException(nameof(index), index, "Point index must be within 0..PointCount-1."); }
}
