using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>Meaning of one joint row's last solver error. Disabled rows have None.</summary>
[Flags]
public enum JointRowBits : byte
{
    /// <summary>No row was evaluated.</summary>
    None = 0,
    /// <summary>The row was evaluated.</summary>
    Enabled = 1,
    /// <summary>The row is angular (radians, or rad/s for a velocity row).</summary>
    Angular = 2,
    /// <summary>A one-sided limit row.</summary>
    Limit = 4,
    /// <summary>A motor or pose-drive row.</summary>
    Motor = 8,
    /// <summary>A spring row.</summary>
    Spring = 16,
    /// <summary>The error is a velocity error rather than position/angle.</summary>
    Velocity = 32,
    /// <summary>The row is active; a limit is violated or carries an impulse.</summary>
    Active = 64,
}

/// <summary>A joint row evaluated in the last non-restitution pass; indices match the persistent impulse slots. No row data is simulation state.</summary>
public readonly struct JointRowInspection
{
    internal JointRowInspection(float error, JointRowBits flags) { Error = error; Flags = flags; }
    /// <summary>Signed solver error, metres or radians, or m/s or rad/s for Velocity rows; a positive limit error is remaining clearance. Gear and rack rows use their solver's ratio-weighted coordinate.</summary>
    public float Error { get; }
    /// <summary>Units, role and activity of the row. None means unused.</summary>
    public JointRowBits Flags { get; }
}

/// <summary>World-owned whole-step joint impulse sums and last-pass row errors. Allocation-free once capacity is reserved, independent of width and threads, and never saved or hashed.</summary>
public sealed class JointReadback
{
    private readonly WorldCore _world;
    private double[] _impulses = Array.Empty<double>();
    private JointRowInspection[] _rows = Array.Empty<JointRowInspection>();
    private int[] _generation = Array.Empty<int>();
    private byte[] _colors = Array.Empty<byte>();
    private int[] _islands = Array.Empty<int>();
    private bool[] _broken = Array.Empty<bool>();
    internal int[] WideSlots = Array.Empty<int>();
    internal float[] WideScratch = Array.Empty<float>();
    internal double InverseDt;
    internal bool Enabled => _world.Options.DetailedInspection;
    private bool _available;
    private void RequireDetail()
    { if (!Enabled || !_available) { throw new DetailedInspectionDisabledException(); } }
    internal JointReadback(WorldCore world) { _world = world; }
    internal void Track(int c, bool refreshMetadata = false)
    {
        ReserveSlots(_world.Constraints.HighWater);
        int gen = _world.Constraints.Handles.GenerationOf(c);
        if (_generation[c] != gen || refreshMetadata)
        {
            _colors[c] = _world.Constraints.Color[c];
            int a = _world.Bodies.Island[_world.Constraints.BodyA[c]];
            _islands[c] = a >= 0 ? a : _world.Bodies.Island[_world.Constraints.BodyB[c]];
        }
        if (_generation[c] != gen)
        {
            _generation[c] = gen; _broken[c] = false;
            _impulses.AsSpan(c * 12, 12).Clear(); _rows.AsSpan(c * 32, 32).Clear();
        }
    }
    internal void ReserveSlots(int n)
    {
        if (_generation.Length < n)
        {
            int size = Math.Max(n, Math.Max(16, _generation.Length * 2));
            Array.Resize(ref _generation, size); Array.Resize(ref _broken, size);
            Array.Resize(ref _colors, size); Array.Resize(ref _islands, size);
            Array.Resize(ref _impulses, size * 12); Array.Resize(ref _rows, size * 32);
        }
    }
    internal void Begin(float dt)
    {
        if (!Enabled && !_available) { return; }
        _available = Enabled;
        InverseDt = 1.0 / dt;
        if (_generation.Length == 0) { return; }
        Array.Clear(_impulses); Array.Clear(_rows); Array.Clear(_broken);
    }
    internal void ReserveWide(int groups)
    {
        if (WideSlots.Length >= groups * 8) { return; }
        int size = Math.Max(groups * 8, Math.Max(16, WideSlots.Length * 2));
        WideSlots = new int[size]; WideScratch = new float[size * 12];
    }
    internal void ClearRows(int c) => _rows.AsSpan(c * 32, 32).Clear();
    internal void Row(int c, int slot, float error, JointRowBits flags)
        => _rows[c * 32 + slot] = new JointRowInspection(error, flags | JointRowBits.Enabled);
    internal void RefreshLimit(int c, int slot, float impulse)
    {
        int at = c * 32 + slot; var row = _rows[at];
        if ((row.Flags & JointRowBits.Limit) == 0) { return; }
        var flags = row.Flags & ~JointRowBits.Active;
        if (row.Error <= 0f || impulse != 0f) { flags |= JointRowBits.Active; }
        _rows[at] = new(row.Error, flags);
    }
    internal void Add(int c, Vector3F la, Vector3F aa, Vector3F lb, Vector3F ab, float impulse)
    {
        int o = c * 12;
        _impulses[o] += (double)la.X * impulse; _impulses[o + 1] += (double)la.Y * impulse; _impulses[o + 2] += (double)la.Z * impulse;
        _impulses[o + 3] += (double)aa.X * impulse; _impulses[o + 4] += (double)aa.Y * impulse; _impulses[o + 5] += (double)aa.Z * impulse;
        _impulses[o + 6] += (double)lb.X * impulse; _impulses[o + 7] += (double)lb.Y * impulse; _impulses[o + 8] += (double)lb.Z * impulse;
        _impulses[o + 9] += (double)ab.X * impulse; _impulses[o + 10] += (double)ab.Y * impulse; _impulses[o + 11] += (double)ab.Z * impulse;
    }
    internal void AddPoint(int c, Vector3F p, Vector3F ta, Vector3F tb)
    {
        // SIMD point blocks report the impulse actually applied, before rounding it into the warm-start accumulator.
        int o = c * 12;
        _impulses[o] -= p.X; _impulses[o + 1] -= p.Y; _impulses[o + 2] -= p.Z;
        _impulses[o + 3] -= ta.X; _impulses[o + 4] -= ta.Y; _impulses[o + 5] -= ta.Z;
        _impulses[o + 6] += p.X; _impulses[o + 7] += p.Y; _impulses[o + 8] += p.Z;
        _impulses[o + 9] += tb.X; _impulses[o + 10] += tb.Y; _impulses[o + 11] += tb.Z;
    }
    internal void ReadWide(int group, int first, int count, int dimensions)
    {
        var s = WideScratch.AsSpan(group * 96, 96);
        for (int l = first; l < first + count; l++)
        {
            int c = WideSlots[group * 8 + l];
            if (dimensions == 2) { ClearRows(c); }
            AddPoint(c, new(s[l], s[8 + l], s[16 + l]), new(s[24 + l], s[32 + l], s[40 + l]), new(s[48 + l], s[56 + l], s[64 + l]));
            for (int k = 0; k < dimensions; k++) { Row(c, k, s[(9 + k) * 8 + l], JointRowBits.Active); }
        }
    }
    internal void MarkBroken(int c) { if ((uint)c < (uint)_broken.Length) { _broken[c] = true; } }
    internal void Validate(JointHandle handle)
    {
        RequireDetail();
        if (handle.World != _world.Id || (uint)handle.Index >= (uint)_generation.Length || _generation[handle.Index] != handle.Generation)
        { throw new StaleHandleException("Joint inspection no longer identifies the recorded joint."); }
        if (_world.Constraints.IsLive(handle.Index) && _world.Constraints.Handles.GenerationOf(handle.Index) != handle.Generation)
        { throw new StaleHandleException("Joint inspection slot has been reused."); }
        if (!_broken[handle.Index]) { _world.Resolve(handle); }
    }
    /// <summary>Read-only whole-step impulses in slot order: 12 doubles per constraint, A linear xyz, A angular xyz, B linear xyz, B angular xyz. Multiply by inverse step dt for mean force/COM torque.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public ReadOnlySpan<double> Impulses { get { RequireDetail(); return _impulses; } }
    /// <summary>Read-only row data, 32 entries per constraint slot; disabled entries have Flags=None.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public ReadOnlySpan<JointRowInspection> Rows { get { RequireDetail(); return _rows; } }
    /// <summary>Inspects a live joint or a matching joint that broke during the latest step. Break records expire on the next step or slot reuse.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public JointInspectionView Inspect(JointHandle joint)
    {
        if (!Enabled) { throw new DetailedInspectionDisabledException(); }
        if (joint.World == _world.Id && (uint)joint.Index < (uint)_generation.Length && _generation[joint.Index] == joint.Generation && _broken[joint.Index])
        {
            Validate(joint);
            return new JointInspectionView(this, joint);
        }
        int c = _world.Resolve(joint); Track(c);
        return new JointInspectionView(this, joint);
    }
    internal bool Broken(int c) => _broken[c];
    internal byte Color(int c) => _colors[c];
    internal int Island(int c) => _islands[c];
}

/// <summary>Read-only joint diagnostics for a recorded generation. Read between steps. Does not keep old steps; the next step replaces its values.</summary>
public readonly struct JointInspectionView
{
    private readonly JointReadback _store;
    private readonly JointHandle _handle;
    internal JointInspectionView(JointReadback store, JointHandle handle) { _store = store; _handle = handle; }
    private int Slot { get { if (_store == null) { throw new InvalidHandleException("Default joint inspection has no world."); } _store.Validate(_handle); return _handle.Index; } }
    private Vector3D Vector(int offset) { int c = Slot; var p = _store.Impulses; int o = c * 12 + offset; double inv = _store.InverseDt; return new(p[o] * inv, p[o + 1] * inv, p[o + 2] * inv); }
    /// <summary>Mean world-space force on A in N, summed over every substep and warm/solve/relax pass.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector3D ForceA => Vector(0);
    /// <summary>Mean world-space torque on A about its centre of mass, in N m, including the force moment.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector3D TorqueA => Vector(3);
    /// <summary>Mean world-space force on B in N.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector3D ForceB => Vector(6);
    /// <summary>Mean world-space torque on B about its centre of mass, in N m, including the force moment.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public Vector3D TorqueB => Vector(9);
    /// <summary>Thirty-two last-pass row entries; disabled entries have Flags=None.</summary>
    /// <exception cref="DetailedInspectionDisabledException">Enable Options.DetailedInspection before stepping to collect these values.</exception>
    public ReadOnlySpan<JointRowInspection> Rows { get { int c = Slot; return _store.Rows.Slice(c * 32, 32); } }
    /// <summary>True if this recorded joint broke during the latest step.</summary>
    public bool IsBroken => _store.Broken(Slot);
    /// <summary>True if any evaluated limit row is active.</summary>
    public bool IsLimitActive { get { foreach (var r in Rows) { if ((r.Flags & (JointRowBits.Limit | JointRowBits.Active)) == (JointRowBits.Limit | JointRowBits.Active)) { return true; } } return false; } }
    /// <summary>True if any evaluated motor or pose-drive row is active.</summary>
    public bool IsMotorActive { get { foreach (var r in Rows) { if ((r.Flags & (JointRowBits.Motor | JointRowBits.Active)) == (JointRowBits.Motor | JointRowBits.Active)) { return true; } } return false; } }
    /// <summary>Solver colour at the last solve input, retained after breaking; 255 means overflow.</summary>
    public byte Color { get { int c = Slot; return _store.Color(c); } }
    /// <summary>Dynamic island at the last solve input (A when dynamic, otherwise B), retained after breaking.</summary>
    public int Island { get { int c = Slot; return _store.Island(c); } }
}
