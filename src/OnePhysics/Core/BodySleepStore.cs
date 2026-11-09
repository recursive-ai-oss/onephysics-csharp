using System;

namespace RecursiveAI.OnePhysics.LowLevel;

// This section is absent in worlds that use only the original world-wide sleep policy.
internal sealed class BodySleepStore : ISnapshotSection
{
    private readonly WorldCore _world;
    private BodySleepSettings[] _settings = Array.Empty<BodySleepSettings>();
    private BodySleepSettings[] _staging = Array.Empty<BodySleepSettings>();
    private int _count;
    public BodySleepStore(WorldCore world) => _world = world;
    public int SectionId => 0x42534C50;
    public bool IsOptional => true;
    public bool IsPresent => _count != 0;
    public BodySleepSettings Get(int b) => b < _settings.Length ? _settings[b] : default;
    public void Set(int b, BodySleepSettings value)
    {
        if (!value.IsOverride && b >= _settings.Length) return;
        if (b >= _settings.Length) Array.Resize(ref _settings, Math.Max(b + 1, Math.Max(16, _settings.Length * 2)));
        if (_settings[b].IsOverride) _count--;
        _settings[b] = value;
        if (value.IsOverride) _count++;
    }
    public void Reset() { Array.Clear(_settings); _count = 0; }
    public void CopyFrom(BodySleepStore other) { _settings = (BodySleepSettings[])other._settings.Clone(); _count = other._count; }
    public ISnapshotSection Fork(WorldCore destination) { var copy = new BodySleepStore(destination); copy.CopyFrom(this); return copy; }
    public void Write(StateWriter w)
    {
        w.NamedInt32(1, "version"); w.NamedInt32(_count, "count");
        for (int b = 0; b < _settings.Length; b++)
        {
            var s = _settings[b]; if (!s.IsOverride) continue;
            w.DescribeEntity("body", b, _world.Bodies.Handles.GenerationOf(b));
            w.NamedInt32(b, "slot"); w.NamedSingle(s.LinearThreshold); w.NamedSingle(s.AngularThreshold); w.NamedSingle(s.TimeToSleep);
        }
    }
    public void Read(StateReader r) => Read(r, _world.Bodies);
    internal void Read(StateReader r, BodyStore bodies)
    {
        if (r.ReadInt32() != 1) throw r.Error("unsupported body sleep section version");
        int count = r.ReadCount(16, "body sleep policies");
        if (count == 0 || count > bodies.Count) throw r.Error("invalid body sleep policy count");
        int n = bodies.HighWater;
        if (_staging.Length < n) Array.Resize(ref _staging, n);
        Array.Clear(_staging);
        int previous = -1;
        for (int i = 0; i < count; i++)
        {
            int b = r.ReadInt32(); float linear = r.ReadSingle(), angular = r.ReadSingle(), time = r.ReadSingle();
            if (b <= previous || b >= n || !bodies.IsDynamic(b)) throw r.Error("body sleep policy references an invalid or unordered dynamic body");
            if (!float.IsFinite(linear) || linear < 0 || !float.IsFinite(angular) || angular < 0 || !float.IsFinite(time) || time <= 0) throw r.Error("invalid body sleep thresholds or duration");
            _staging[b] = new(linear, angular, time); previous = b;
        }
        (_settings, _staging) = (_staging, _settings); _count = count;
    }
}
