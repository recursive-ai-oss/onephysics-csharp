using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePhysics.LowLevel;

/// <summary>One loaded tile: its integer key and the static bodies that make it up.</summary>
internal sealed class TileRecord
{
    public int X, Y, Z;
    public int[] Bodies = Array.Empty<int>();
    /// <summary>The host asked for the tile to go but a body still rests on it; the world retries every step.</summary>
    public bool Unloading;
    public long Key => ((long)X << 42) ^ ((long)Y << 21) ^ (uint)Z; // not used for ordering

    public static int Compare(TileRecord a, TileRecord b)
    {
        int c = a.X.CompareTo(b.X);
        if (c != 0) { return c; }
        c = a.Y.CompareTo(b.Y);
        return c != 0 ? c : a.Z.CompareTo(b.Z);
    }
}

/// <summary>
/// World-scale state shared by both dimensions: simulation level of detail (which islands step every k-th step), local time scale (per body and per region) and the registry of streamed tiles.
/// Absent from a snapshot while nothing here is in use, so worlds that never use it keep their bytes and hashes. Layout in docs/spec/worldscale.md.
/// </summary>
internal sealed class WorldScaleStore : ISnapshotSection
{
    internal const int MaxFocusPoints = 64, MaxBands = 16, MaxRegions = 256, MaxTiles = 1 << 20;
    internal const float MinScale = 1f / 1024f, MaxScale = 4f;

    private readonly WorldCore _world;
    private readonly int _lanes;

    // Level of detail. Positions are in host coordinates (stored position plus the world origin), three doubles each (z is zero in 2D).
    internal double[] Focus = Array.Empty<double>();          // 3 per focus point
    internal float[] BandDistance = Array.Empty<float>();      // ascending
    internal int[] BandPeriod = Array.Empty<int>();
    internal int FarPeriod = 1;
    internal double[] LodBoxes = Array.Empty<double>();        // 6 per region: min, max
    internal int[] LodBoxPeriod = Array.Empty<int>();
    internal int LodSubstepFactor;   // 0: automatic (2 at period 8, else 1)

    // Time scale.
    internal double[] ScaleBoxes = Array.Empty<double>();
    internal float[] ScaleBoxValue = Array.Empty<float>();
    private float[] _bodyScale = Array.Empty<float>();         // 0 means 1
    private float[] _staging = Array.Empty<float>();
    private int _bodyScaleCount;

    // Tiles, ordered by key.
    internal readonly List<TileRecord> Tiles = new();

    internal WorldScaleStore(WorldCore world) { _world = world; _lanes = world.Layout.PositionLanes; }

    public int SectionId => 0x4C435357;
    public bool IsOptional => true;

    internal bool LodConfigured => LodBoxPeriod.Length != 0 || (Focus.Length != 0 && BandPeriod.Length != 0);
    internal bool TimeScaleConfigured => ScaleBoxValue.Length != 0 || _bodyScaleCount != 0;

    /// <summary>True when a step must plan island rates: some region, band or time scale is configured.</summary>
    internal bool Active => LodConfigured || TimeScaleConfigured;

    public bool IsPresent => Active || Tiles.Count != 0 || Focus.Length != 0 || LodSubstepFactor != 0 || FarPeriod != 1;

    // ------------------------------------------------------------------ queries used by the step

    internal float ScaleOfBody(int b) { float s = b < _bodyScale.Length ? _bodyScale[b] : 0f; return s == 0f ? 1f : s; }

    internal float TimeScaleAt(double x, double y, double z)
    {
        float s = float.MaxValue;
        var boxes = ScaleBoxes;
        for (int i = 0; i < ScaleBoxValue.Length; i++)
        {
            int o = 6 * i;
            if (Inside(boxes, o, x, y, z) && ScaleBoxValue[i] < s) { s = ScaleBoxValue[i]; }
        }

        return s == float.MaxValue ? 1f : s;
    }

    internal int PeriodAt(double x, double y, double z)
    {
        int p = int.MaxValue;
        for (int i = 0; i < LodBoxPeriod.Length; i++)
        {
            if (Inside(LodBoxes, 6 * i, x, y, z) && LodBoxPeriod[i] < p) { p = LodBoxPeriod[i]; }
        }

        if (p != int.MaxValue) { return p; }
        if (BandPeriod.Length == 0 || Focus.Length == 0) { return 1; }
        double best = double.MaxValue;
        for (int i = 0; i + 2 < Focus.Length; i += 3)
        {
            double dx = x - Focus[i], dy = y - Focus[i + 1], dz = z - Focus[i + 2];
            double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < best) { best = d2; }
        }

        for (int i = 0; i < BandDistance.Length; i++)
        {
            double r = BandDistance[i];
            if (best <= r * r) { return BandPeriod[i]; }
        }

        return FarPeriod;
    }

    private bool Inside(double[] boxes, int o, double x, double y, double z)
        => x >= boxes[o] && x <= boxes[o + 3] && y >= boxes[o + 1] && y <= boxes[o + 4] && (_lanes == 2 || (z >= boxes[o + 2] && z <= boxes[o + 5]));

    // ------------------------------------------------------------------ configuration

    internal static bool ValidPeriod(int p) => p == 1 || p == 2 || p == 4 || p == 8;

    internal void SetLod(ReadOnlySpan<double> focus, ReadOnlySpan<float> bandDistance, ReadOnlySpan<int> bandPeriod, int farPeriod, ReadOnlySpan<double> boxes, ReadOnlySpan<int> boxPeriod, int substepFactor)
    {
        if (focus.Length % 3 != 0 || focus.Length / 3 > MaxFocusPoints) { throw new ArgumentException("At most " + MaxFocusPoints.ToString(CultureInfo.InvariantCulture) + " focus points are supported.", nameof(focus)); }
        if (bandDistance.Length != bandPeriod.Length || bandDistance.Length > MaxBands) { throw new ArgumentException("At most " + MaxBands.ToString(CultureInfo.InvariantCulture) + " bands are supported.", nameof(bandDistance)); }
        if (boxes.Length != 6 * boxPeriod.Length || boxPeriod.Length > MaxRegions) { throw new ArgumentException("At most " + MaxRegions.ToString(CultureInfo.InvariantCulture) + " regions are supported.", nameof(boxes)); }
        for (int i = 0; i < focus.Length; i++) { CheckCoordinate(focus[i], "focus point"); }
        float previous = 0f;
        for (int i = 0; i < bandDistance.Length; i++)
        {
            float d = bandDistance[i];
            if (!(d > 0f) || !float.IsFinite(d) || d < previous) { throw new ArgumentOutOfRangeException(nameof(bandDistance), d, "Band distances must be finite, positive and ascending (band " + i.ToString(CultureInfo.InvariantCulture) + ")."); }
            previous = d;
            if (!ValidPeriod(bandPeriod[i])) { throw new ArgumentOutOfRangeException(nameof(bandPeriod), bandPeriod[i], "A period is 1, 2, 4 or 8 steps (band " + i.ToString(CultureInfo.InvariantCulture) + ")."); }
        }

        if (!ValidPeriod(farPeriod)) { throw new ArgumentOutOfRangeException(nameof(farPeriod), farPeriod, "A period is 1, 2, 4 or 8 steps."); }
        if (substepFactor < 0 || substepFactor > 8) { throw new ArgumentOutOfRangeException(nameof(substepFactor), substepFactor, "The substep factor is 0 (automatic) or 1 to 8."); }
        for (int i = 0; i < boxPeriod.Length; i++)
        {
            if (!ValidPeriod(boxPeriod[i])) { throw new ArgumentOutOfRangeException(nameof(boxPeriod), boxPeriod[i], "A period is 1, 2, 4 or 8 steps (region " + i.ToString(CultureInfo.InvariantCulture) + ")."); }
            CheckBox(boxes.Slice(6 * i, 6), "region " + i.ToString(CultureInfo.InvariantCulture));
        }

        Focus = focus.ToArray(); BandDistance = bandDistance.ToArray(); BandPeriod = bandPeriod.ToArray(); FarPeriod = farPeriod;
        LodBoxes = boxes.ToArray(); LodBoxPeriod = boxPeriod.ToArray(); LodSubstepFactor = substepFactor;
    }

    internal void SetTimeScaleRegions(ReadOnlySpan<double> boxes, ReadOnlySpan<float> scale)
    {
        if (boxes.Length != 6 * scale.Length || scale.Length > MaxRegions) { throw new ArgumentException("At most " + MaxRegions.ToString(CultureInfo.InvariantCulture) + " regions are supported.", nameof(boxes)); }
        for (int i = 0; i < scale.Length; i++)
        {
            CheckScale(scale[i]);
            CheckBox(boxes.Slice(6 * i, 6), "region " + i.ToString(CultureInfo.InvariantCulture));
        }

        ScaleBoxes = boxes.ToArray(); ScaleBoxValue = scale.ToArray();
    }

    internal static void CheckScale(float s)
    {
        if (!(s >= MinScale && s <= MaxScale)) { throw new ArgumentOutOfRangeException(nameof(s), s, "A time scale is in [1/1024, 4]; " + s.ToString("R", CultureInfo.InvariantCulture) + " was given."); }
    }

    private static void CheckCoordinate(double v, string what)
    {
        if (!double.IsFinite(v) || Math.Abs(v) > 4194304.0 * 2) { throw new ArgumentOutOfRangeException(what, v, "The " + what + " coordinate " + v.ToString("R", CultureInfo.InvariantCulture) + " is not finite or exceeds 2^23."); }
    }

    private static void CheckBox(ReadOnlySpan<double> box, string what)
    {
        for (int i = 0; i < 6; i++) { CheckCoordinate(box[i], what); }
        if (box[0] > box[3] || box[1] > box[4] || box[2] > box[5]) { throw new ArgumentException("The " + what + " has a minimum above its maximum."); }
    }

    internal void ClearLod()
    {
        Focus = Array.Empty<double>(); BandDistance = Array.Empty<float>(); BandPeriod = Array.Empty<int>(); FarPeriod = 1;
        LodBoxes = Array.Empty<double>(); LodBoxPeriod = Array.Empty<int>(); LodSubstepFactor = 0;
    }

    internal float GetBodyScale(int b) => ScaleOfBody(b);

    internal void SetBodyScale(int b, float s)
    {
        if (s == 1f) { s = 0f; }
        if (b >= _bodyScale.Length) { if (s == 0f) { return; } Array.Resize(ref _bodyScale, Math.Max(b + 1, Math.Max(16, _bodyScale.Length * 2))); }
        if (_bodyScale[b] != 0f) { _bodyScaleCount--; }
        _bodyScale[b] = s;
        if (s != 0f) { _bodyScaleCount++; }
    }

    // ------------------------------------------------------------------ tiles

    internal int FindTile(int x, int y, int z)
    {
        int lo = 0, hi = Tiles.Count - 1;
        while (lo <= hi)
        {
            int mid = (lo + hi) >> 1;
            var t = Tiles[mid];
            int c = x != t.X ? x.CompareTo(t.X) : y != t.Y ? y.CompareTo(t.Y) : z.CompareTo(t.Z);
            if (c == 0) { return mid; }
            if (c < 0) { hi = mid - 1; } else { lo = mid + 1; }
        }

        return ~lo;
    }

    internal TileRecord AddTile(int x, int y, int z, int[] bodies)
    {
        int at = FindTile(x, y, z);
        if (at >= 0) { throw new InvalidOperationException("Tile (" + x.ToString(CultureInfo.InvariantCulture) + ", " + y.ToString(CultureInfo.InvariantCulture) + ", " + z.ToString(CultureInfo.InvariantCulture) + ") is already loaded."); }
        var rec = new TileRecord { X = x, Y = y, Z = z, Bodies = bodies };
        Tiles.Insert(~at, rec);
        return rec;
    }

    internal void RemoveTile(int index) => Tiles.RemoveAt(index);

    // ------------------------------------------------------------------ section

    public void Reset()
    {
        ClearLod(); ScaleBoxes = Array.Empty<double>(); ScaleBoxValue = Array.Empty<float>();
        Array.Clear(_bodyScale); _bodyScaleCount = 0; Tiles.Clear();
    }

    internal void CopyFrom(WorldScaleStore s)
    {
        Focus = (double[])s.Focus.Clone(); BandDistance = (float[])s.BandDistance.Clone(); BandPeriod = (int[])s.BandPeriod.Clone(); FarPeriod = s.FarPeriod;
        LodBoxes = (double[])s.LodBoxes.Clone(); LodBoxPeriod = (int[])s.LodBoxPeriod.Clone(); LodSubstepFactor = s.LodSubstepFactor;
        ScaleBoxes = (double[])s.ScaleBoxes.Clone(); ScaleBoxValue = (float[])s.ScaleBoxValue.Clone();
        _bodyScale = (float[])s._bodyScale.Clone(); _bodyScaleCount = s._bodyScaleCount;
        Tiles.Clear();
        foreach (var t in s.Tiles) { Tiles.Add(new TileRecord { X = t.X, Y = t.Y, Z = t.Z, Bodies = (int[])t.Bodies.Clone(), Unloading = t.Unloading }); }
    }

    public ISnapshotSection Fork(WorldCore destination) { var c = new WorldScaleStore(destination); c.CopyFrom(this); return c; }

    public void Write(StateWriter w)
    {
        // Configuration and the tile registry are host-driven, not part of a region; per-body scales below belong to their bodies.
        w.DescribeEntity("worldscale", 0);
        w.NamedInt32(1, "version");
        w.NamedInt32(LodSubstepFactor, "lod substep factor");
        w.NamedInt32(FarPeriod, "far period");
        w.NamedInt32(Focus.Length / 3, "focus points");
        for (int i = 0; i < Focus.Length; i++) { w.NamedDouble(Focus[i], "focus"); }
        w.NamedInt32(BandPeriod.Length, "bands");
        for (int i = 0; i < BandPeriod.Length; i++) { w.NamedSingle(BandDistance[i], "band distance"); w.NamedInt32(BandPeriod[i], "band period"); }
        w.NamedInt32(LodBoxPeriod.Length, "lod regions");
        for (int i = 0; i < LodBoxPeriod.Length; i++) { for (int k = 0; k < 6; k++) { w.NamedDouble(LodBoxes[6 * i + k], "region"); } w.NamedInt32(LodBoxPeriod[i], "region period"); }
        w.NamedInt32(ScaleBoxValue.Length, "time scale regions");
        for (int i = 0; i < ScaleBoxValue.Length; i++) { for (int k = 0; k < 6; k++) { w.NamedDouble(ScaleBoxes[6 * i + k], "region"); } w.NamedSingle(ScaleBoxValue[i], "scale"); }
        w.NamedInt32(_bodyScaleCount, "body scales");
        for (int b = 0; b < _bodyScale.Length; b++)
        {
            if (_bodyScale[b] == 0f) { continue; }
            w.DescribeEntity("body", b, _world.Bodies.Handles.GenerationOf(b));
            w.NamedInt32(b, "slot"); w.NamedSingle(_bodyScale[b], "scale");
        }

        w.DescribeEntity("worldscale", 0);
        w.NamedInt32(Tiles.Count, "tiles");
        foreach (var t in Tiles)
        {
            w.NamedInt32(t.X, "tile x"); w.NamedInt32(t.Y, "tile y"); w.NamedInt32(t.Z, "tile z");
            w.NamedBool(t.Unloading, "unloading");
            w.NamedInt32(t.Bodies.Length, "tile bodies");
            foreach (int b in t.Bodies) { w.NamedInt32(b, "slot"); }
        }
    }

    public void Read(StateReader r) => Read(r, _world.Bodies);

    internal void Read(StateReader r, BodyStore bodies)
    {
        if (r.ReadInt32() != 1) { throw r.Error("unsupported world scale section version"); }
        int factor = r.ReadInt32();
        if (factor < 0 || factor > 8) { throw r.Error("invalid lod substep factor"); }
        int far = r.ReadInt32();
        if (!ValidPeriod(far)) { throw r.Error("invalid far period"); }
        int nf = r.ReadCount(24, "focus points");
        if (nf > MaxFocusPoints) { throw r.Error("too many focus points"); }
        var focus = new double[3 * nf];
        r.ReadDoubles(focus, true, "focus");
        int nb = r.ReadCount(8, "bands");
        if (nb > MaxBands) { throw r.Error("too many bands"); }
        var bd = new float[nb]; var bp = new int[nb];
        float prev = 0f;
        for (int i = 0; i < nb; i++)
        {
            bd[i] = r.ReadSingle(); bp[i] = r.ReadInt32();
            if (!(bd[i] > 0f) || !float.IsFinite(bd[i]) || bd[i] < prev || !ValidPeriod(bp[i])) { throw r.Error("invalid band"); }
            prev = bd[i];
        }

        int nl = r.ReadCount(52, "lod regions");
        if (nl > MaxRegions) { throw r.Error("too many lod regions"); }
        var lb = new double[6 * nl]; var lp = new int[nl];
        for (int i = 0; i < nl; i++)
        {
            for (int k = 0; k < 6; k++) { lb[6 * i + k] = r.ReadDouble(); }
            lp[i] = r.ReadInt32();
            if (!ValidPeriod(lp[i]) || !ValidBox(lb, 6 * i)) { throw r.Error("invalid lod region"); }
        }

        int ns = r.ReadCount(52, "time scale regions");
        if (ns > MaxRegions) { throw r.Error("too many time scale regions"); }
        var sb = new double[6 * ns]; var sv = new float[ns];
        for (int i = 0; i < ns; i++)
        {
            for (int k = 0; k < 6; k++) { sb[6 * i + k] = r.ReadDouble(); }
            sv[i] = r.ReadSingle();
            if (!(sv[i] >= MinScale && sv[i] <= MaxScale) || !ValidBox(sb, 6 * i)) { throw r.Error("invalid time scale region"); }
        }

        int nbs = r.ReadCount(8, "body scales");
        if (nbs > bodies.Count) { throw r.Error("too many body time scales"); }
        var scales = new float[bodies.HighWater];
        int last = -1;
        for (int i = 0; i < nbs; i++)
        {
            int b = r.ReadInt32(); float s = r.ReadSingle();
            if (b <= last || b >= bodies.HighWater || !bodies.IsDynamic(b)) { throw r.Error("body time scale references an invalid or unordered dynamic body"); }
            if (!(s >= MinScale && s <= MaxScale) || s == 1f) { throw r.Error("invalid body time scale"); }
            scales[b] = s; last = b;
        }

        int nt = r.ReadCount(21, "tiles");
        if (nt > MaxTiles) { throw r.Error("too many tiles"); }
        var tiles = new List<TileRecord>(nt);
        var owned = new HashSet<int>();
        for (int i = 0; i < nt; i++)
        {
            var t = new TileRecord { X = r.ReadInt32(), Y = r.ReadInt32(), Z = r.ReadInt32(), Unloading = r.ReadBool() };
            if (tiles.Count > 0 && TileRecord.Compare(tiles[^1], t) >= 0) { throw r.Error("tiles are not in ascending key order"); }
            int nbody = r.ReadCount(4, "tile bodies");
            t.Bodies = new int[nbody];
            for (int k = 0; k < nbody; k++)
            {
                int b = r.ReadInt32();
                if (b < 0 || b >= bodies.HighWater || !bodies.IsLive(b) || bodies.TypeOf(b) != BodyType.Static) { throw r.Error("a tile refers to a body that is not a live static body"); }
                if (!owned.Add(b)) { throw r.Error("a body belongs to two tiles"); }
                t.Bodies[k] = b;
            }

            tiles.Add(t);
        }

        Focus = focus; BandDistance = bd; BandPeriod = bp; FarPeriod = far; LodBoxes = lb; LodBoxPeriod = lp; LodSubstepFactor = factor;
        ScaleBoxes = sb; ScaleBoxValue = sv; _bodyScale = scales; _bodyScaleCount = nbs;
        Tiles.Clear(); Tiles.AddRange(tiles);
    }

    private static bool ValidBox(double[] b, int o)
    {
        for (int k = 0; k < 6; k++) { if (!double.IsFinite(b[o + k]) || Math.Abs(b[o + k]) > 4194304.0 * 2) { return false; } }
        return b[o] <= b[o + 3] && b[o + 1] <= b[o + 4] && b[o + 2] <= b[o + 5];
    }
}
