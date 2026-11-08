using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePhysics.LowLevel;

// Struct specialization removes diagnostic temporaries from the default scalar integration loop.
internal interface IInspectionMode { static abstract bool Enabled { get; } }
internal readonly struct CollectInspection : IInspectionMode { public static bool Enabled => true; }
internal readonly struct SkipInspection : IInspectionMode { public static bool Enabled => false; }

/// <summary>Unsaved body input and damping impulses. Fifteen doubles per body: gravity xyz, external linear xyz, external angular xyz, damping linear xyz, damping angular xyz. World axes; 2D uses xy and angular z.</summary>
public sealed class BodyReadback
{
    private struct Input
    {
        public float InverseMass, GravityScale;
        public Vector3F Force, Torque;
        public double Duration;
    }
    private double[] _values = Array.Empty<double>(), _pending = Array.Empty<double>();
    private Input[] _inputs = Array.Empty<Input>();
    private byte[] _inputFlags = Array.Empty<byte>(), _pendingQueued = Array.Empty<byte>();
    private int[] _pendingBodies = Array.Empty<int>();
    private int _pendingCount, _bodyCount;
    private Vector3F _gravity;
    internal byte[] CcdState = Array.Empty<byte>();
    private readonly WorldCore _world;
    private bool _available;
    internal BodyReadback(WorldCore world) { _world = world; }
    private void RequireDetail()
    { if (!_world.Options.DetailedInspection || !_available) { throw new DetailedInspectionDisabledException(); } }
    internal void Reserve(int count)
    {
        if (CcdState.Length >= count) { return; }
        int n = Math.Max(count, Math.Max(16, CcdState.Length * 2));
        Array.Resize(ref _values, n * 15); Array.Resize(ref _pending, n * 6); Array.Resize(ref CcdState, n);
        Array.Resize(ref _inputs, n); Array.Resize(ref _inputFlags, n); Array.Resize(ref _pendingQueued, n); Array.Resize(ref _pendingBodies, n);
    }
    internal void Initialize(int b)
    {
        Reserve(b + 1); _values.AsSpan(b * 15, 15).Clear(); _pending.AsSpan(b * 6, 6).Clear(); CcdState[b] = 0; _inputFlags[b] = 0;
        _bodyCount = Math.Max(_bodyCount, b + 1);
    }
    internal void Begin(int count, Vector3F gravity)
    {
        Reserve(count); _gravity = gravity;
        Array.Clear(CcdState);
        if (!_world.Options.DetailedInspection && !_available) { return; }
        _available = _world.Options.DetailedInspection;
        int slots = Math.Max(_bodyCount, count);
        for (int b = 0; b < slots; b++)
        {
            if ((_inputFlags[b] & 14) != 0) { _values.AsSpan(b * 15, 15).Clear(); }
            _inputFlags[b] = 0; CcdState[b] = 0;
        }
        _bodyCount = count;
        for (int i = 0; i < _pendingCount; i++)
        {
            int b = _pendingBodies[i];
            if (b < count) { _pending.AsSpan(b * 6, 6).CopyTo(_values.AsSpan(b * 15 + 3, 6)); _inputFlags[b] |= 8; }
            _pending.AsSpan(b * 6, 6).Clear(); _pendingQueued[b] = 0;
        }
        _pendingCount = 0;
    }
    internal void Pending(int b, Vector3D linear, Vector3D angular)
    {
        Reserve(b + 1);
        if (_pendingQueued[b] == 0) { _pendingQueued[b] = 1; _pendingBodies[_pendingCount++] = b; }
        Add(_pending, b * 6, linear); Add(_pending, b * 6 + 3, angular);
    }
    internal void Inputs(int b, float inverseMass, float gravityScale, Vector3F force, Vector3F torque, double duration)
    {
        _inputs[b] = new Input { InverseMass = inverseMass, GravityScale = gravityScale, Force = force, Torque = torque, Duration = duration };
        _inputFlags[b] |= 1;
    }
    internal void Damping(int b, Vector3D linear, Vector3D angular)
    { _inputFlags[b] |= 4; Add(_values, b * 15 + 9, linear); Add(_values, b * 15 + 12, angular); }
    private void Project(int b)
    {
        if ((_inputFlags[b] & 3) != 1) { return; }
        // Inputs are fixed for a step: project once, only when inspection actually reads them, not on every substep.
        _inputFlags[b] |= 2; ref readonly var input = ref _inputs[b];
        double scaledMassTime = (1.0 / input.InverseMass) * input.Duration * input.GravityScale;
        int o = b * 15;
        Add(_values, o, new(scaledMassTime * _gravity.X, scaledMassTime * _gravity.Y, scaledMassTime * _gravity.Z));
        Add(_values, o + 3, new(input.Duration * input.Force.X, input.Duration * input.Force.Y, input.Duration * input.Force.Z));
        Add(_values, o + 6, new(input.Duration * input.Torque.X, input.Duration * input.Torque.Y, input.Duration * input.Torque.Z));
    }
    private static void Add(double[] destination, int offset, Vector3D value)
    { destination[offset] += value.X; destination[offset + 1] += value.Y; destination[offset + 2] += value.Z; }
    /// <summary>Read-only last-step impulses in the documented fifteen-double body stride; excludes contacts, joints, speed clamps, gyroscopic correction and service-internal impulses. Projects cached inputs without allocating.</summary>
    public ReadOnlySpan<double> Impulses
    { get { RequireDetail(); for (int b = 0; b < _bodyCount; b++) { Project(b); } return _values.AsSpan(0, _bodyCount * 15); } }
    internal Vector3D Vector(int b, int field)
    { RequireDetail(); if ((uint)b >= (uint)_inputFlags.Length) { return default; } Project(b); int o = b * 15 + field; return new(_values[o], _values[o + 1], _values[o + 2]); }
}
