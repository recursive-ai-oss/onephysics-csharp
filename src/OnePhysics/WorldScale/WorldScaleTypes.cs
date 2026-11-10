using System;
using System.Collections.Generic;
using System.Globalization;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics;

/// <summary>The integer coordinate of a streamed tile. Keys order by X, then Y, then Z; that is the order tiles are loaded and unloaded in. In a 2D world Z is zero.</summary>
public readonly record struct TileKey(int X, int Y, int Z) : IComparable<TileKey>
{
    /// <summary>A key in a 2D world (Z is zero).</summary>
    public TileKey(int x, int y) : this(x, y, 0) { }

    /// <inheritdoc/>
    public int CompareTo(TileKey other)
    {
        int c = X.CompareTo(other.X);
        if (c != 0) { return c; }
        c = Y.CompareTo(other.Y);
        return c != 0 ? c : Z.CompareTo(other.Z);
    }

    /// <summary>True when <paramref name="a"/> orders before <paramref name="b"/>.</summary>
    public static bool operator <(TileKey a, TileKey b) => a.CompareTo(b) < 0;

    /// <summary>True when <paramref name="a"/> orders after <paramref name="b"/>.</summary>
    public static bool operator >(TileKey a, TileKey b) => a.CompareTo(b) > 0;

    /// <summary>True when <paramref name="a"/> orders before or equal to <paramref name="b"/>.</summary>
    public static bool operator <=(TileKey a, TileKey b) => a.CompareTo(b) <= 0;

    /// <summary>True when <paramref name="a"/> orders after or equal to <paramref name="b"/>.</summary>
    public static bool operator >=(TileKey a, TileKey b) => a.CompareTo(b) >= 0;

    /// <inheritdoc/>
    public override string ToString() => "(" + X.ToString(CultureInfo.InvariantCulture) + ", " + Y.ToString(CultureInfo.InvariantCulture) + ", " + Z.ToString(CultureInfo.InvariantCulture) + ")";
}

/// <summary>What happens when a tile is unloaded while a dynamic body still touches it.</summary>
public enum TileUnloadPolicy
{
    /// <summary>The tile stays until no body (awake or asleep) is in contact with it, then goes. The world retries at the start of every step. This is the default: nothing falls into a hole.</summary>
    WaitForContacts = 0,

    /// <summary>The tile goes at once; bodies resting on it lose their support and get contact-end events.</summary>
    Immediate = 1,
}

/// <summary>A distance band of simulation level of detail: bodies within <see cref="MaxDistance"/> of the nearest focus point step every <see cref="Period"/> steps.</summary>
/// <param name="MaxDistance">Metres from the nearest focus point; finite and positive, ascending over the bands.</param>
/// <param name="Period">1, 2, 4 or 8: the island advances once every this many steps, with <c>dt * period</c>.</param>
public readonly record struct LodBand(float MaxDistance, int Period);

/// <summary>The state of one streamed tile.</summary>
public enum TileState
{
    /// <summary>No such tile, or its load is still queued.</summary>
    Absent = 0,

    /// <summary>Loaded.</summary>
    Loaded = 1,

    /// <summary>An unload was requested but a body still touches the tile; it goes when the contact ends.</summary>
    Unloading = 2,
}

/// <summary>
/// Tile streaming for one world: queues loads and unloads and applies them in key order at the start of the next step (or when the world is hashed, snapshotted or forked), so the result does not depend
/// on the order the host called in. Shared by both dimensions.
/// </summary>
internal sealed class TileStreamer<TDef> where TDef : class
{
    private sealed class Pending
    {
        public TileKey Key; public TDef? Def; public double X, Y, Z; public TileUnloadPolicy Policy; public int Sequence;
    }

    private readonly WorldCore _core;
    private readonly Func<TDef, double, double, double, int[]> _create;
    private readonly Action<int> _destroy;
    private readonly Func<string, Replay.ReplayMutationScope> _scope;
    private readonly List<Pending> _pending = new();
    private int _sequence;

    /// <summary>The message of the last tile that could not be built at flush time (invalid geometry); null when there was none.</summary>
    public string? LastError { get; private set; }

    /// <summary>The number of tiles dropped because they could not be built.</summary>
    public int FailedLoads { get; private set; }

    public TileStreamer(WorldCore core, Func<TDef, double, double, double, int[]> create, Action<int> destroy, Func<string, Replay.ReplayMutationScope> scope)
    {
        _core = core; _create = create; _destroy = destroy; _scope = scope;
        core.PendingFlush = Flush;
        core.PendingDiscard = Discard;
    }

    public int PendingCount => _pending.Count;

    public void Discard() { _pending.Clear(); }

    public TileState StateOf(TileKey key)
    {
        int at = _core.ScaleStore.FindTile(key.X, key.Y, key.Z);
        if (at < 0) { return TileState.Absent; }
        return _core.ScaleStore.Tiles[at].Unloading ? TileState.Unloading : TileState.Loaded;
    }

    public void Load(TileKey key, TDef def, double hostX, double hostY, double hostZ)
    {
        var state = StateOf(key);
        bool unloadQueued = false, loadQueued = false;
        foreach (var p in _pending) { if (p.Key.Equals(key)) { if (p.Def == null) { unloadQueued = true; loadQueued = false; } else { loadQueued = true; unloadQueued = false; } } }
        if (loadQueued || (state != TileState.Absent && !unloadQueued)) { throw new InvalidOperationException("Tile " + key + " is already loaded or queued for loading; unload it first."); }
        if (state == TileState.Unloading) { throw new InvalidOperationException("Tile " + key + " is still being unloaded; load it again once it is gone (see IsTileLoaded)."); }
        if (unloadQueued) { throw new InvalidOperationException("Tile " + key + " has an unload queued; step the world once before loading it again."); }
        _pending.Add(new Pending { Key = key, Def = def, X = hostX, Y = hostY, Z = hostZ, Sequence = _sequence++ });
    }

    public bool Unload(TileKey key, TileUnloadPolicy policy)
    {
        for (int i = _pending.Count - 1; i >= 0; i--)
        {
            if (!_pending[i].Key.Equals(key)) { continue; }
            if (_pending[i].Def != null) { _pending.RemoveAt(i); return true; } // cancels a load that never happened
            return false; // an unload is already queued
        }

        if (StateOf(key) == TileState.Absent) { return false; }
        _pending.Add(new Pending { Key = key, Policy = policy, Sequence = _sequence++ });
        return true;
    }

    public void Flush()
    {
        var store = _core.ScaleStore;
        bool retry = false;
        foreach (var t in store.Tiles) { if (t.Unloading) { retry = true; break; } }
        if (_pending.Count == 0 && !retry) { return; }
        using var scope = _scope("PhysicsWorld.FlushTiles");
        _pending.Sort(static (a, b) => { int c = a.Key.CompareTo(b.Key); return c != 0 ? c : a.Sequence.CompareTo(b.Sequence); });
        List<Pending>? loads = null;
        var forced = new List<TileKey>();
        foreach (var p in _pending)
        {
            if (p.Def != null) { (loads ??= new()).Add(p); continue; }
            int at = store.FindTile(p.Key.X, p.Key.Y, p.Key.Z);
            if (at < 0) { continue; }
            store.Tiles[at].Unloading = true;
            if (p.Policy == TileUnloadPolicy.Immediate) { forced.Add(p.Key); }
        }

        _pending.Clear();
        // Unloads, ascending by key: a tile goes when nothing touches it (or at once when forced).
        var busy = BusyBodies(store);
        for (int i = 0; i < store.Tiles.Count;)
        {
            var t = store.Tiles[i];
            if (!t.Unloading) { i++; continue; }
            bool force = forced.Contains(new TileKey(t.X, t.Y, t.Z));
            if (!force)
            {
                bool touched = false;
                foreach (int b in t.Bodies) { if (busy.Contains(b)) { touched = true; break; } }
                if (touched) { i++; continue; }
            }

            var bodies = t.Bodies;
            store.Tiles.RemoveAt(i);
            foreach (int b in bodies) { _destroy(b); }
        }

        if (loads == null) { return; }
        foreach (var p in loads)
        {
            double ox = p.X - _core.Origin[0], oy = p.Y - _core.Origin[1], oz = _core.Layout.PositionLanes == 3 ? p.Z - _core.Origin[2] : 0.0;
            int[] bodies;
            try { bodies = _create(p.Def!, ox, oy, oz); }
            catch (Exception e) when (e is not OutOfMemoryException)
            {
                LastError = "Tile " + p.Key + " could not be built: " + e.Message;
                FailedLoads++;
                continue;
            }

            store.AddTile(p.Key.X, p.Key.Y, p.Key.Z, bodies);
        }
    }

    // The static bodies of unloading tiles that a dynamic body touches through any live contact.
    private HashSet<int> BusyBodies(WorldScaleStore store)
    {
        var busy = new HashSet<int>();
        var mine = new HashSet<int>();
        foreach (var t in store.Tiles) { if (t.Unloading) { foreach (int b in t.Bodies) { mine.Add(b); } } }
        if (mine.Count == 0) { return busy; }
        var cons = _core.Constraints; var bodies = _core.Bodies;
        for (int c = 0; c < cons.HighWater; c++)
        {
            if (!cons.IsLive(c) || cons.Kind[c] == (byte)ConstraintKind.Sensor) { continue; }
            int a = cons.BodyA[c], b = cons.BodyB[c];
            if (mine.Contains(a) && bodies.IsDynamic(b)) { busy.Add(a); }
            else if (mine.Contains(b) && bodies.IsDynamic(a)) { busy.Add(b); }
        }

        return busy;
    }
}
