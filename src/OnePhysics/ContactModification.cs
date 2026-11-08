using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>Edits a contact on the stepping thread, after narrowphase and before solving. Do not mutate the world or retain the view. The same inputs must produce the same edits.</summary>
public delegate void ContactModifier(ref ContactModification contact);

/// <summary>A middle-tier contact modifier. One installed instance is retained by the world; calls are serial and ordered by shape pair, subshape pair, then constraint index.</summary>
public interface IContactModifier
{
    /// <summary>Edits one manifold for this step. Do not mutate world storage from this method.</summary>
    void Modify(ref ContactModification contact);
}

/// <summary>A stack-only mutable view of one manifold. Separations are metres, negative inside. Changes affect this step and are saved in the ordinary contact payload. The view cannot escape the callback.</summary>
public ref struct ContactModification
{
    private readonly Span<float> _payload;
    private readonly Span<float> _extra;
    internal Span<float> Extra => _extra;
    private readonly int _count;
    private readonly int _dimension;
    internal ContactModification(WorldCore world, int constraint, Span<float> extra)
    {
        _extra = extra;
        _dimension = world.Layout.PositionLanes;
        var store = world.Constraints;
        _payload = store.Payload(constraint);
        _count = store.PointCount[constraint];
        BodyA = world.HandleOfBody(store.BodyA[constraint]);
        BodyB = world.HandleOfBody(store.BodyB[constraint]);
        ShapeA = world.HandleOfShape(store.ShapeA[constraint]);
        ShapeB = world.HandleOfShape(store.ShapeB[constraint]);
        SubShapeA = store.SubShapeA[constraint];
        SubShapeB = store.SubShapeB[constraint];
        Enabled = true;
    }
    /// <summary>First body, ordered by shape index rather than body index.</summary>
    public BodyHandle BodyA { get; }
    /// <summary>Second body.</summary>
    public BodyHandle BodyB { get; }
    /// <summary>Lower-index shape.</summary>
    public ShapeHandle ShapeA { get; }
    /// <summary>Higher-index shape.</summary>
    public ShapeHandle ShapeB { get; }
    /// <summary>First subshape index, zero for a simple convex shape.</summary>
    public int SubShapeA { get; }
    /// <summary>Second subshape index, zero for a simple convex shape.</summary>
    public int SubShapeB { get; }
    /// <summary>Number of points, 1..2 in 2D or 1..4 in 3D.</summary>
    public int PointCount => _count;
    /// <summary>Whether this manifold may apply impulses this step. Defaults to true on each callback.</summary>
    public bool Enabled { get; set; }
    /// <summary>World-space unit normal from A to B, X component.</summary>
    public float NormalX => _payload[ManifoldLayout.Normal];
    /// <summary>World-space unit normal from A to B, Y component.</summary>
    public float NormalY => _payload[ManifoldLayout.Normal + 1];
    /// <summary>World-space unit normal from A to B, Z component; zero in 2D.</summary>
    public float NormalZ => _dimension == 2 ? 0f : _payload[ManifoldLayout.Normal + 2];
    /// <summary>Coulomb coefficient, 0..1e15. Throws ArgumentOutOfRangeException for non-finite or out-of-range values.</summary>
    public float Friction
    {
        get => _payload[ManifoldLayout.Friction];
        set { Validate(value, MaterialLimits.MaxCoefficient, nameof(Friction)); _payload[ManifoldLayout.Friction] = value; _extra[3] = value; }
    }
    /// <summary>Restitution, 0..1. Throws ArgumentOutOfRangeException for non-finite or out-of-range values.</summary>
    public float Restitution
    {
        get => _payload[ManifoldLayout.Restitution];
        set { Validate(value, 1f, nameof(Restitution)); _payload[ManifoldLayout.Restitution] = value; }
    }
    /// <summary>Desired relative surface speed along Tangent1, in metres/second; default zero. Throws ArgumentOutOfRangeException for non-finite values.</summary>
    public float TangentSpeed
    {
        get => _extra[0];
        set { if (!float.IsFinite(value)) throw new ArgumentOutOfRangeException(nameof(TangentSpeed), value, "Tangent speed must be finite."); _extra[0] = value; }
    }
    /// <summary>Maximum accumulated normal impulse per point per substep, in newton seconds; default float.MaxValue (unlimited). Applies to warm start, solve, relax and restitution. Throws ArgumentOutOfRangeException unless finite and nonnegative.</summary>
    public float NormalImpulseLimit
    {
        get => _extra[1];
        set { Validate(value, float.MaxValue, nameof(NormalImpulseLimit)); _extra[1] = value; }
    }
    /// <summary>Reads a point's separation in metres. Throws ArgumentOutOfRangeException if point is outside 0..PointCount-1.</summary>
    public float GetSeparation(int point) { CheckPoint(point); return _payload[ManifoldLayout.Separation(point)]; }
    /// <summary>Changes a point's separation in metres. Throws ArgumentOutOfRangeException for an invalid point or a non-finite value.</summary>
    public void SetSeparation(int point, float separation)
    {
        CheckPoint(point);
        if (!float.IsFinite(separation)) throw new ArgumentOutOfRangeException(nameof(separation), separation, "Separation must be finite, in metres.");
        _payload[ManifoldLayout.Separation(point)] = separation;
    }
    private void CheckPoint(int point)
    {
        if ((uint)point >= (uint)_count) throw new ArgumentOutOfRangeException(nameof(point), point, "Point must be within 0..PointCount-1.");
    }
    private static void Validate(float value, float maximum, string name)
    {
        if (!(value >= 0f && value <= maximum)) throw new ArgumentOutOfRangeException(name, value, "Contact coefficient is outside its finite nonnegative range.");
    }
}

internal sealed class ContactModificationPipeline : IComparer<int>, ISnapshotSection
{
    private ContactModifier? _callback;
    private bool _reset;
    private bool _modified;
    private int[] _order = Array.Empty<int>();
    private ConstraintStore? _store;
    private float[] _extra = Array.Empty<float>();
    internal const int ExtraStride = 10;
    internal bool SurfaceActive;
    internal Func<int, float>? ShapeFriction;
    internal void RequestRefresh() => _reset = true;
    internal ReadOnlySpan<float> ExtraOf(int c) => _extra.AsSpan(ExtraStride * c, ExtraStride);
    public bool Extended { get; private set; }
    public float TangentSpeed(int c) => _extra[ExtraStride * c];
    public float ImpulseLimit(int c) => _extra[ExtraStride * c + 1];
    public bool Refresh => _callback != null || SurfaceActive || _reset || _modified;
    public ContactModifier? Callback
    {
        get => _callback;
        set { _reset |= _callback != null; _callback = value; }
    }
    public void Run(WorldCore world)
    {
        _reset = false;
        _modified = false;
        Extended = false;
        if (_callback == null && !SurfaceActive) return;
        var cs = world.Constraints;
        if (_order.Length < cs.Count) Array.Resize(ref _order, Math.Max(cs.Count, Math.Max(16, _order.Length * 2)));
        if (_extra.Length < cs.HighWater * ExtraStride) Array.Resize(ref _extra, cs.HighWater * ExtraStride);
        int n = 0;
        for (int c = 0; c < cs.HighWater; c++)
            if (cs.IsLive(c) && cs.KindOf(c) == ConstraintKind.Contact && cs.PointCount[c] != 0) _order[n++] = c;
        _modified = n != 0;
        _store = cs;
        Array.Sort(_order, 0, n, this);
        for (int i = 0; i < n; i++)
        {
            int c = _order[i];
            var extra = _extra.AsSpan(ExtraStride * c, ExtraStride);
            extra.Clear(); extra[1] = float.MaxValue; extra[3] = cs.Payload(c)[ManifoldLayout.Friction];
            var view = new ContactModification(world, c, extra);
            if (SurfaceActive) SurfaceContacts.Apply(world, c, ref view, ShapeFriction);
            _callback?.Invoke(ref view);
            if (SurfaceActive || view.TangentSpeed != 0f || view.NormalImpulseLimit != float.MaxValue)
            {
                Extended = true;
                var payload = cs.Payload(c);
                for (int p = 0; p < view.PointCount; p++)
                {
                    int offset = ManifoldLayout.NormalImpulse(p);
                    payload[offset] = Math.Min(payload[offset], view.NormalImpulseLimit);
                }
            }
            if (!view.Enabled)
            {
                // Zero points suppress all linear impulses; rolling warm starts must be cleared too.
                cs.PointCount[c] = 0;
                var payload = cs.Payload(c);
                payload[ManifoldLayout.RollingResistance] = 0f;
                payload.Slice(ManifoldLayout.RollingImpulse, 3).Clear();
                for (int p = 0; p < view.PointCount; p++) payload.Slice(ManifoldLayout.NormalImpulse(p), 4).Clear();
            }
        }
        _store = null;
    }
    // A fresh world restoring an edited payload must refresh it even without the original callback.
    public int SectionId => 0x434D4F44;
    public bool IsOptional => true;
    public bool IsPresent => _modified;
    public void Write(StateWriter writer) => writer.WriteInt32(1);
    public void Read(StateReader reader)
    {
        if (reader.ReadInt32() != 1) throw reader.Error("Unsupported contact modification section version.");
        _modified = true;
    }
    public void Reset() { _modified = false; _reset = false; Extended = false; }
    internal void CopyState(ContactModificationPipeline source) { _modified = source._modified; _reset = source._reset; }
    public ISnapshotSection Fork(WorldCore destination)
    {
        var copy = new ContactModificationPipeline();
        copy.CopyState(this);
        return copy;
    }
    public int Compare(int a, int b)
    {
        var cs = _store!;
        int d = cs.ShapeA[a].CompareTo(cs.ShapeA[b]);
        if (d == 0) d = cs.ShapeB[a].CompareTo(cs.ShapeB[b]);
        if (d == 0) d = cs.SubShapeA[a].CompareTo(cs.SubShapeA[b]);
        if (d == 0) d = cs.SubShapeB[a].CompareTo(cs.SubShapeB[b]);
        return d != 0 ? d : a.CompareTo(b);
    }
}
