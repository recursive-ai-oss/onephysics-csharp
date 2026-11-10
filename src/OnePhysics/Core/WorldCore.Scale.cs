using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.LowLevel;

public sealed partial class WorldCore
{
    // World scale: islands that step every k-th step (simulation level of detail), islands that run in scaled time, and the registry of streamed tiles. Design and layout: docs/spec/worldscale.md.
    internal readonly WorldScaleStore ScaleStore;

    /// <summary>Called, serially, at the start of a step and before the world is hashed, snapshotted or forked, so pending host requests (streamed tiles) take effect in a canonical order. Set by the engine.</summary>
    internal Action? PendingFlush;

    /// <summary>Called after a successful restore: requests queued before the restore describe a world that no longer exists.</summary>
    internal Action? PendingDiscard;
    private bool _noFlush, _flushing;

    /// <summary>Applies pending host requests now (queued tiles), in their canonical order.</summary>
    internal void FlushForHost() => FlushPending();

    private void FlushPending()
    {
        if (PendingFlush == null || _noFlush || _flushing) { return; }
        _flushing = true;
        try { PendingFlush(); } finally { _flushing = false; }
    }

    /// <summary>Registers the world-scale state as the last snapshot section (an engine calls this once, after its own sections).</summary>
    internal void RegisterWorldScale() => AddSection(ScaleStore);

    /// <summary>Sets the simulation level of detail: which islands advance every 2nd, 4th or 8th step.</summary>
    /// <param name="focus">Focus points in host coordinates, three doubles each (z zero in 2D).</param>
    /// <param name="bandDistance">Ascending distances from the nearest focus point; a body within <c>bandDistance[i]</c> steps with <c>bandPeriod[i]</c>.</param>
    /// <param name="bandPeriod">Period of each band: 1, 2, 4 or 8.</param>
    /// <param name="farPeriod">Period beyond the last band.</param>
    /// <param name="regionBoxes">Boxes in host coordinates (min then max, three doubles each); a body inside any of them steps at the smallest period of the boxes that hold it, whatever the bands say.</param>
    /// <param name="regionPeriod">Period of each box: 1, 2, 4 or 8.</param>
    /// <param name="substepFactor">Multiplies the substeps of an island that steps every k-th step (1 to 8); 0, the default, chooses 2 at period 8 (a pile does not hold at one substep length of 8 steps) and 1 otherwise.</param>
    /// <exception cref="ArgumentException">A count is wrong or too large, or a box is inverted.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A period, distance, coordinate or the factor is out of range.</exception>
    public void SetSimulationLod(ReadOnlySpan<double> focus, ReadOnlySpan<float> bandDistance, ReadOnlySpan<int> bandPeriod, int farPeriod, ReadOnlySpan<double> regionBoxes, ReadOnlySpan<int> regionPeriod, int substepFactor = 0)
        => ScaleStore.SetLod(focus, bandDistance, bandPeriod, farPeriod, regionBoxes, regionPeriod, substepFactor);

    /// <summary>Removes every level-of-detail setting: all islands step every step again.</summary>
    public void ClearSimulationLod() => ScaleStore.ClearLod();

    /// <summary>Sets the time-scale regions: a body inside any of them runs at the smallest scale of the boxes that hold it (times its own scale). Boxes are in host coordinates, min then max, three doubles each.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A scale is outside [1/1024, 4] or a coordinate is not finite.</exception>
    public void SetTimeScaleRegions(ReadOnlySpan<double> boxes, ReadOnlySpan<float> scale) => ScaleStore.SetTimeScaleRegions(boxes, scale);

    /// <summary>A dynamic body's own time scale; 1 unless set.</summary>
    public float GetBodyTimeScale(BodyHandle body) => ScaleStore.GetBodyScale(Resolve(body));

    /// <summary>Sets a dynamic body's own time scale, in [1/1024, 4]: its island advances <c>dt * scale</c> each step (the smallest scale among the island's bodies and regions).</summary>
    /// <exception cref="BodyTypeException">The body is not dynamic.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The scale is outside [1/1024, 4].</exception>
    public void SetBodyTimeScale(BodyHandle body, float scale)
    {
        int b = Resolve(body);
        if (!_bodies.IsDynamic(b)) { throw new BodyTypeException(nameof(SetBodyTimeScale), body, _bodies.TypeOf(b), true); }
        WorldScaleStore.CheckScale(scale);
        ScaleStore.SetBodyScale(b, scale);
    }

    private void ForgetTileBody(int b)
    {
        var tiles = ScaleStore.Tiles;
        for (int i = 0; i < tiles.Count; i++)
        {
            int at = Array.IndexOf(tiles[i].Bodies, b);
            if (at < 0) { continue; }
            if (tiles[i].Bodies.Length == 1) { tiles.RemoveAt(i); }
            else
            {
                var rest = new int[tiles[i].Bodies.Length - 1];
                Array.Copy(tiles[i].Bodies, 0, rest, 0, at);
                Array.Copy(tiles[i].Bodies, at + 1, rest, at, rest.Length - at);
                tiles[i].Bodies = rest;
            }

            return;
        }
    }

    // ------------------------------------------------------------------ planning a scaled step

    // Per-step plan, all transient: islands are suspended (made to look asleep, silently) while they are not due or while another group is solved, and restored by the end of the step.
    private byte[] _suspended = Array.Empty<byte>();
    private int _suspendedCount;
    private byte[] _isoPeriod = Array.Empty<byte>();
    private float[] _isoScale = Array.Empty<float>();
    private int[] _isoGroup = Array.Empty<int>();
    private bool _scaleOn;
    private bool _passKinematic = true;
    private readonly List<GroupKey> _keys = new();
    private readonly GroupKey[] _groups = new GroupKey[MaxGroups];
    private int _groupCount;
    private const int MaxGroups = 8;

    private readonly struct GroupKey : IComparable<GroupKey>
    {
        public readonly float Mult; public readonly int Period; public readonly float Scale;
        public GroupKey(int period, float scale) { Period = period; Scale = scale; Mult = period * scale; }
        public int CompareTo(GroupKey o)
        {
            int c = Mult.CompareTo(o.Mult);
            if (c != 0) { return c; }
            c = Period.CompareTo(o.Period);
            return c != 0 ? c : Scale.CompareTo(o.Scale);
        }
    }

    internal int SleepPeriodOf(int body)
    {
        int i = _bodies.Island[body];
        return i >= 0 && i < _isoPeriod.Length && _isoPeriod[i] != 0 ? _isoPeriod[i] : 1;
    }

    private void Suspend(int island)
    {
        if (island >= _suspended.Length) { Array.Resize(ref _suspended, Math.Max(island + 1, Math.Max(16, _suspended.Length * 2))); }
        if (_suspended[island] != 0) { return; }
        _suspended[island] = 1;
        _suspendedCount++;
        _islands.SetAsleep(island, true);
        foreach (int b in _islands.Bodies(island)) { _bodies.Flags[b] &= unchecked((byte)~(byte)BodyBits.Awake); }
    }

    private void Unsuspend(int island)
    {
        _suspended[island] = 0;
        _suspendedCount--;
        _islands.SetAsleep(island, false);
        foreach (int b in _islands.Bodies(island)) { _bodies.Flags[b] |= (byte)BodyBits.Awake; }
    }

    private void RestoreSuspended()
    {
        if (_suspendedCount != 0)
        {
            for (int i = 0; i < _suspended.Length; i++)
            {
                if (_suspended[i] == 0) { continue; }
                if (i < _islands.HighWater && _islands.IsLive(i)) { Unsuspend(i); } else { _suspended[i] = 0; _suspendedCount--; }
            }
        }

        _scaleOn = false;
        _passKinematic = true;
    }

    private void EnsureIso(int islands)
    {
        if (_isoPeriod.Length < islands)
        {
            int n = Math.Max(islands, Math.Max(16, _isoPeriod.Length * 2));
            _isoPeriod = new byte[n]; _isoScale = new float[n]; _isoGroup = new int[n];
        }
    }

    // Gives every awake island its period (the finest of its bodies' regions) and time scale (the smallest of its bodies'), pins the islands that must keep up with something that moves every step
    // (a kinematic body, a vehicle, a rope or machine the engine keeps on the joint schedule), and suspends the islands that are not due this step. Returns the largest time multiplier of an island that is due.
    private float ComputeIslandKeys()
    {
        var bodies = _bodies; var store = ScaleStore;
        int ni = _islands.HighWater;
        EnsureIso(ni);
        Array.Clear(_isoPeriod, 0, ni);
        int lanes = Layout.PositionLanes;
        var pos = bodies.Position;
        bool lod = store.LodConfigured, zones = store.ScaleBoxValue.Length != 0;
        bool anyKinematic = false;
        var island = bodies.Island;
        for (int b = 0; b < bodies.HighWater; b++)
        {
            if (!bodies.IsLive(b)) { continue; }
            var type = bodies.TypeOf(b);
            if (type == BodyType.Kinematic) { anyKinematic = true; continue; }
            if (type != BodyType.Dynamic || !bodies.IsAwake(b)) { continue; }
            int i = island[b];
            if (i < 0) { continue; }
            double x = pos[b * lanes] + _origin[0], y = pos[b * lanes + 1] + _origin[1], z = lanes == 3 ? pos[b * lanes + 2] + _origin[2] : 0.0;
            int p = lod ? store.PeriodAt(x, y, z) : 1;
            float s = store.ScaleOfBody(b);
            if (zones) { s *= store.TimeScaleAt(x, y, z); }
            if (s < WorldScaleStore.MinScale) { s = WorldScaleStore.MinScale; } else if (s > WorldScaleStore.MaxScale) { s = WorldScaleStore.MaxScale; }
            if (_isoPeriod[i] == 0) { _isoPeriod[i] = (byte)p; _isoScale[i] = s; }
            else
            {
                if (p < _isoPeriod[i]) { _isoPeriod[i] = (byte)p; }
                if (s < _isoScale[i]) { _isoScale[i] = s; }
            }
        }

        if (anyKinematic)
        {
            var cons = _constraints;
            for (int c = 0; c < cons.HighWater; c++)
            {
                if (!cons.IsLive(c) || cons.Kind[c] == (byte)ConstraintKind.Sensor) { continue; }
                int a = cons.BodyA[c], b = cons.BodyB[c];
                if (bodies.TypeOf(a) == BodyType.Kinematic && bodies.IsDynamic(b)) { Pin(island[b]); }
                else if (bodies.TypeOf(b) == BodyType.Kinematic && bodies.IsDynamic(a)) { Pin(island[a]); }
            }
        }

        for (int k = 0; k < _forcedCount; k++)
        {
            int b = _forcedFull[k];
            if (b < bodies.HighWater && bodies.IsLive(b) && bodies.IsDynamic(b)) { Pin(island[b]); }
        }

        float most = 1f;
        for (int i = 0; i < ni; i++)
        {
            int p = _isoPeriod[i];
            if (p == 0) { continue; }
            bool due = ((_stepCount + 1) & (p - 1)) == 0; // periods are powers of two
            if (!due) { Suspend(i); _isoPeriod[i] = 0; continue; }
            float m = p * _isoScale[i];
            if (m > most) { most = m; }
        }

        return most;

        void Pin(int i) { if (i >= 0 && _isoPeriod[i] != 0) { _isoPeriod[i] = 1; _isoScale[i] = 1f; } }
    }

    private float PlanScaleStart(float dt)
    {
        _scaleOn = true;
        float most = ComputeIslandKeys();
        return dt * most;
    }

    private int ScaledSubsteps(int baseCount, int period, float scale)
    {
        if (baseCount <= 0) { return baseCount; }
        double f = Math.Max(1.0, scale);
        if (period > 1) { f *= ScaleStore.LodSubstepFactor != 0 ? ScaleStore.LodSubstepFactor : period >= 8 ? 2 : 1; }
        int n = (int)Math.Ceiling(baseCount * f - 1e-9);
        return n < 1 ? 1 : n;
    }

    private void RunScaledGroups<TStages>(TStages stages, float dt) where TStages : IWorldStages
    {
        // Contacts may have merged islands and woken sleeping ones: plan again over what is awake now.
        ComputeIslandKeys();
        int ni = _islands.HighWater;
        _keys.Clear();
        bool kin = false;
        for (int b = 0; b < _bodies.HighWater && !kin; b++) { kin = _bodies.IsLive(b) && _bodies.TypeOf(b) == BodyType.Kinematic; }
        for (int i = 0; i < ni; i++) { if (_isoPeriod[i] != 0) { _keys.Add(new GroupKey(_isoPeriod[i], _isoScale[i])); } }
        if (kin) { _keys.Add(new GroupKey(1, 1f)); }
        _keys.Sort();
        int distinct = 0;
        for (int i = 0; i < _keys.Count; i++)
        {
            if (distinct > 0 && _keys[distinct - 1].CompareTo(_keys[i]) == 0) { continue; }
            _keys[distinct++] = _keys[i];
        }

        _keys.RemoveRange(distinct, _keys.Count - distinct);
        bool folded = distinct > MaxGroups;
        _groupCount = folded ? MaxGroups : distinct;
        for (int g = 0; g < _groupCount; g++) { _groups[g] = _keys[folded ? (int)((long)g * (distinct - 1) / (MaxGroups - 1)) : g]; }
        for (int i = 0; i < ni; i++)
        {
            if (_isoPeriod[i] == 0) { _isoGroup[i] = -1; continue; }
            var key = new GroupKey(_isoPeriod[i], _isoScale[i]);
            int pick = 0;
            for (int g = 0; g < _groupCount; g++)
            {
                if (folded ? _groups[g].Mult <= key.Mult : _groups[g].CompareTo(key) == 0) { pick = g; }
            }

            _isoGroup[i] = pick;
        }

        bool kinematicDone = false;
        for (int g = 0; g < _groupCount; g++)
        {
            var key = _groups[g];
            for (int i = 0; i < ni; i++)
            {
                int grp = _isoGroup[i];
                if (grp < 0 || !_islands.IsLive(i)) { continue; }
                if (grp == g) { if (i < _suspended.Length && _suspended[i] != 0) { Unsuspend(i); } }
                else { Suspend(i); }
            }

            _passKinematic = !kinematicDone && key.Period == 1 && key.Scale == 1f;
            if (_passKinematic) { kinematicDone = true; }
            BuildAwakeBodies();
            SolveSection(stages, dt * key.Mult, ScaledSubsteps(_options.Substeps, key.Period, key.Scale), ScaledSubsteps(_liteSubsteps, key.Period, key.Scale));
        }

        if (kin && !kinematicDone)
        {
            // Folding dropped the full-rate group: kinematic bodies still move once per step, in a pass of their own.
            for (int i = 0; i < ni; i++) { if (_isoGroup[i] >= 0 && _islands.IsLive(i)) { Suspend(i); } }
            _passKinematic = true;
            BuildAwakeBodies();
            SolveSection(stages, dt, _options.Substeps, _liteSubsteps);
        }

        for (int i = 0; i < ni; i++) { if (_isoGroup[i] >= 0 && i < _suspended.Length && _suspended[i] != 0 && _islands.IsLive(i)) { Unsuspend(i); } }
        _passKinematic = true;
        BuildAwakeBodies();
        Counters.LodIslandsSkipped = _suspendedCount;
    }
}
