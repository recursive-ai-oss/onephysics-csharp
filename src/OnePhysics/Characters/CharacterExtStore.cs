using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Characters;

/// <summary>What the character-ability store needs from an engine.</summary>
internal interface ICharacterExtHost
{
    WorldCore Core { get; }

    /// <summary>True for the 2D engine (positions carry z = 0, rotations are about z).</summary>
    bool Planar { get; }

    int EngineSectionId { get; }

    CharacterExtStore Extensions { get; }

    int CharacterHighWater { get; }

    bool CharacterLive(int character);

    int CharacterGeneration(int character);

    int CharacterBody(int character);
}

/// <summary>Everything a character can be doing besides walking, one record per character slot.</summary>
internal struct CharacterExtState
{
    /// <summary>The character may grab ledges and climb (<c>CharacterDef.CanClimb</c>).</summary>
    public bool CanClimb;

    /// <summary>The character is carried by the pose of the body it stands on (<c>CharacterDef.ExactPlatformCarry</c>), not by that body's velocity.</summary>
    public bool ExactCarry;

    /// <summary>Free mode only: the character stood on a moving body at its last move and is carried with it; <see cref="Body"/> and <see cref="Local"/> say where.</summary>
    public bool Riding;

    public CharacterMode Mode;

    /// <summary>The body whose frame <see cref="Local"/> is in: the seat's body, or the body of the ledge or surface held. Slot and generation.</summary>
    public int Body, BodyGen;

    /// <summary>Seated: the rider's origin in the body's origin frame. Hanging and climbing: the character centre in that frame.</summary>
    public Vector3F Local;

    /// <summary>Seated: the rider's orientation relative to the body (identity otherwise).</summary>
    public QuaternionF LocalRotation;

    /// <summary>Hanging and climbing: unit normal of the held surface in the body's frame, pointing out of the surface towards the character.</summary>
    public Vector3F Normal;

    /// <summary>Hanging: unit direction along the ledge in the body's frame (shimmy axis).</summary>
    public Vector3F Along;

    /// <summary>Roped: the rod slot and generation, the first of the two particles the grip lies between, and the fraction (0 to 1) of the way to the next.</summary>
    public int Rod, RodGen, Particle;

    public float Fraction;

    /// <summary>Set when a record is read: the generation of the character the snapshot says it belongs to, checked once against the restored characters (0: nothing to check).</summary>
    public int SavedGeneration;
}

/// <summary>
/// Character abilities beyond walking: seats, ledge hangs, climbing and rope grips, plus which shapes are climbable. The state lives in its own optional snapshot section (written only
/// while something is in use, so worlds without these keep their bytes and hashes); the behaviour is in the engines, which read and write the records here.
/// </summary>
internal sealed class CharacterExtStore : ISnapshotSection
{
    /// <summary>Section id ("CEXT").</summary>
    public const int Id = 0x43455854;

    private const int SectionVersion = 1;
    private readonly ICharacterExtHost _host;

    public CharacterExtState[] States = new CharacterExtState[8];

    /// <summary>Per shape slot: the generation of the shape marked climbable there, or 0.</summary>
    public int[] ClimbableGen = new int[16];

    private int _climbableCount;
    private int _active;

    public CharacterExtStore(ICharacterExtHost host) => _host = host;

    /// <summary>Characters not in <see cref="CharacterMode.Free"/> mode or with the climb permission.</summary>
    public int ActiveCount => _active;

    public int ClimbableCount => _climbableCount;

    /// <summary>Number of characters currently seated, hanging, climbing or roped.</summary>
    public int BusyCount
    {
        get
        {
            int n = 0;
            for (int c = 0; c < _host.CharacterHighWater; c++)
            {
                if (c < States.Length && _host.CharacterLive(c) && States[c].Mode != CharacterMode.Free) { n++; }
            }

            return n;
        }
    }

    public void Ensure(int n)
    {
        if (States.Length < n) { Array.Resize(ref States, Math.Max(n, States.Length * 2)); }
    }

    public ref CharacterExtState At(int c)
    {
        Ensure(c + 1);
        return ref States[c];
    }

    public void Set(int c, in CharacterExtState state)
    {
        Ensure(c + 1);
        bool was = IsActive(in States[c]);
        States[c] = state;
        // An unused rotation is the identity, so every record writes a unit quaternion.
        if (state.LocalRotation.X == 0f && state.LocalRotation.Y == 0f && state.LocalRotation.Z == 0f && state.LocalRotation.W == 0f) { States[c].LocalRotation = QuaternionF.Identity; }
        bool now = IsActive(in state);
        if (now && !was) { _active++; } else if (!now && was) { _active--; }
    }

    /// <summary>Forgets what the character was doing (a new character in the slot, a destroyed one).</summary>
    public void Clear(int c)
    {
        if (c >= States.Length) { return; }
        if (IsActive(in States[c])) { _active--; }
        States[c] = default;
    }

    private static bool IsActive(in CharacterExtState s) => s.CanClimb || s.ExactCarry || s.Riding || s.Mode != CharacterMode.Free;

    public bool IsClimbable(int shape, int generation) => (uint)shape < (uint)ClimbableGen.Length && ClimbableGen[shape] == generation && generation != 0;

    public void SetClimbable(int shape, int generation, bool on)
    {
        if (!on)
        {
            if ((uint)shape < (uint)ClimbableGen.Length && ClimbableGen[shape] != 0) { ClimbableGen[shape] = 0; _climbableCount--; }
            return;
        }

        if (shape >= ClimbableGen.Length) { Array.Resize(ref ClimbableGen, Math.Max(shape + 1, ClimbableGen.Length * 2)); }
        if (ClimbableGen[shape] == 0) { _climbableCount++; }
        ClimbableGen[shape] = generation;
    }

    // ------------------------------------------------------------------ snapshot section

    int ISnapshotSection.SectionId => Id;

    bool ISnapshotSection.IsOptional => true;

    bool ISnapshotSection.IsPresent => _active > 0 || _climbableCount > 0;

    void ISnapshotSection.Reset()
    {
        if (_active == 0 && _climbableCount == 0) { return; }
        Array.Clear(States);
        Array.Clear(ClimbableGen);
        _active = 0;
        _climbableCount = 0;
    }

    void ISnapshotSection.Write(StateWriter w)
    {
        w.NamedInt32(SectionVersion);
        int n = 0;
        for (int c = 0; c < _host.CharacterHighWater && c < States.Length; c++)
        {
            if (_host.CharacterLive(c) && IsActive(in States[c])) { n++; }
        }

        w.NamedInt32(n);
        for (int c = 0; c < _host.CharacterHighWater && c < States.Length; c++)
        {
            if (!_host.CharacterLive(c) || !IsActive(in States[c])) { continue; }
            ref readonly var s = ref States[c];
            w.DescribeEntity("character", c, _host.CharacterGeneration(c));
            w.NamedInt32(c); w.NamedInt32(_host.CharacterGeneration(c));
            // Every record has every field whatever the mode, so the layout of a world does not change when a character sits down or lets go (regional restores need a stable field set).
            w.NamedByte((byte)((s.CanClimb ? 1 : 0) | (s.ExactCarry ? 2 : 0) | (s.Riding ? 4 : 0))); w.NamedByte((byte)s.Mode);
            w.NamedInt32(s.Body); w.NamedInt32(s.BodyGen);
            WriteVec(w, s.Local);
            w.NamedSingle(s.LocalRotation.X); w.NamedSingle(s.LocalRotation.Y); w.NamedSingle(s.LocalRotation.Z); w.NamedSingle(s.LocalRotation.W);
            WriteVec(w, s.Normal); WriteVec(w, s.Along);
            w.NamedInt32(s.Rod); w.NamedInt32(s.RodGen); w.NamedInt32(s.Particle); w.NamedSingle(s.Fraction);
        }

        w.DescribeEntity("world", -1);
        w.NamedInt32(_climbableCount);
        for (int sh = 0; sh < ClimbableGen.Length; sh++)
        {
            if (ClimbableGen[sh] == 0) { continue; }
            w.DescribeEntity("shape", sh, ClimbableGen[sh]);
            w.NamedInt32(sh); w.NamedInt32(ClimbableGen[sh]);
        }
    }

    private static void WriteVec(StateWriter w, Vector3F v) { w.NamedSingle(v.X); w.NamedSingle(v.Y); w.NamedSingle(v.Z); }

    private static Vector3F ReadVec(StateReader r) => new(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());

    private static bool Finite(Vector3F v) => float.IsFinite(v.X) && float.IsFinite(v.Y) && float.IsFinite(v.Z);

    private static bool Unit(Vector3F v) => Finite(v) && Math.Abs(v.X * v.X + v.Y * v.Y + v.Z * v.Z - 1f) <= 1e-3f;

    void ISnapshotSection.Read(StateReader r)
    {
        int version = r.ReadInt32();
        if (version != SectionVersion) { throw r.Error("unsupported character-ability section version " + version.ToString(CultureInfo.InvariantCulture)); }
        bool planar = _host.Planar;
        int n = r.ReadCount(86, "character abilities");
        var states = new CharacterExtState[Math.Max(8, 0)];
        // Every character has a body of its own, so a character slot is below the body count of the snapshot being restored; that also bounds what the arrays below grow to.
        int slotLimit = (_host.Core.RestoringBodies ?? _host.Core.Bodies).HighWater;
        int last = -1, active = 0;
        for (int k = 0; k < n; k++)
        {
            int c = r.ReadInt32(), gen = r.ReadInt32();
            if (c <= last || c >= slotLimit || gen < 1) { throw r.Error("character abilities out of order or for an invalid character"); }
            last = c;
            if (c >= states.Length) { Array.Resize(ref states, Math.Max(c + 1, states.Length * 2)); }
            byte flags = r.ReadByte();
            if (flags > 7) { throw r.Error("invalid character ability flags"); }
            var s = new CharacterExtState { CanClimb = (flags & 1) != 0, ExactCarry = (flags & 2) != 0, Riding = (flags & 4) != 0, SavedGeneration = gen };
            byte mode = r.ReadByte();
            if (mode > (byte)CharacterMode.Roped) { throw r.Error("invalid character mode " + mode.ToString(CultureInfo.InvariantCulture)); }
            s.Mode = (CharacterMode)mode;
            s.Body = r.ReadInt32(); s.BodyGen = r.ReadInt32();
            s.Local = ReadVec(r);
            s.LocalRotation = new QuaternionF(r.ReadSingle(), r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            s.Normal = ReadVec(r); s.Along = ReadVec(r);
            s.Rod = r.ReadInt32(); s.RodGen = r.ReadInt32(); s.Particle = r.ReadInt32(); s.Fraction = r.ReadSingle();
            if (s.Mode != CharacterMode.Free && s.Riding) { throw r.Error("a held character cannot also be carried"); }
            bool bodyUsed = s.Riding || s.Mode == CharacterMode.Seated || s.Mode == CharacterMode.Hanging || s.Mode == CharacterMode.Climbing;
            if (bodyUsed)
            {
                if (s.Body < 0 || s.Body > 1 << 26 || s.BodyGen < 1) { throw r.Error("invalid body reference in a character ability"); }
                if (!Finite(s.Local) || Math.Abs(s.Local.X) > 4194304f || Math.Abs(s.Local.Y) > 4194304f || Math.Abs(s.Local.Z) > 4194304f || (planar && s.Local.Z != 0f)) { throw r.Error("invalid character anchor"); }
            }

            var q = s.LocalRotation;
            if (!(Math.Abs(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W - 1f) <= 1e-3f) || (planar && (q.X != 0f || q.Y != 0f))) { throw r.Error("invalid seat rotation"); }
            if (s.Mode == CharacterMode.Roped)
            {
                if (s.Rod < 0 || s.Rod > 65536 || s.RodGen < 1 || s.Particle < 0 || s.Particle > 4096 || !(s.Fraction >= 0f && s.Fraction <= 1f)) { throw r.Error("invalid rope grip"); }
            }

            if (s.Mode == CharacterMode.Hanging || s.Mode == CharacterMode.Climbing)
            {
                if (!Unit(s.Normal) || (planar && s.Normal.Z != 0f)) { throw r.Error("invalid surface normal"); }
                if (s.Mode == CharacterMode.Hanging && (!Unit(s.Along) || (planar && s.Along.Z != 0f))) { throw r.Error("invalid ledge direction"); }
            }

            if (!Finite(s.Normal) || !Finite(s.Along)) { throw r.Error("non-finite character ability"); }
            // The writer leaves out records with nothing in use, so one that is all zero was not written by this library.
            if (!IsActive(in s)) { throw r.Error("character ability record with nothing in use"); }
            active++;
            states[c] = s;
        }

        int climbable = r.ReadCount(8, "climbable shapes");
        var gens = new int[16];
        int lastShape = -1;
        for (int k = 0; k < climbable; k++)
        {
            int slot = r.ReadInt32(), gen = r.ReadInt32();
            if (slot <= lastShape || slot > 1 << 26 || gen < 1) { throw r.Error("climbable shapes out of order or for an invalid shape"); }
            lastShape = slot;
            if (slot >= gens.Length) { Array.Resize(ref gens, Math.Max(slot + 1, gens.Length * 2)); }
            gens[slot] = gen;
        }

        States = states;
        ClimbableGen = gens;
        _active = active;
        _climbableCount = climbable;
    }

    ISnapshotSection ISnapshotSection.Fork(WorldCore destination)
    {
        var dst = ((ICharacterExtHost)destination.GetSnapshotSection(_host.EngineSectionId)).Extensions;
        dst.States = (CharacterExtState[])States.Clone();
        dst.ClimbableGen = (int[])ClimbableGen.Clone();
        dst._active = _active;
        dst._climbableCount = _climbableCount;
        return dst;
    }

    /// <summary>
    /// Region closure: a character that is held by a body, or carried by one, goes into a region with it, and a gripped rod goes with whoever holds it. Returns whether the selection grew.
    /// </summary>
    internal bool CloseRegion(RegionSelection s)
    {
        bool changed = false;
        for (int c = 0; c < _host.CharacterHighWater && c < States.Length; c++)
        {
            if (!_host.CharacterLive(c)) { continue; }
            ref readonly var st = ref States[c];
            bool anchored = st.Mode == CharacterMode.Seated || st.Mode == CharacterMode.Hanging || st.Mode == CharacterMode.Climbing || st.Riding;
            int body = _host.CharacterBody(c);
            if (anchored)
            {
                bool heldMoves = (uint)st.Body < (uint)s.Bodies.Length && s.Bodies[st.Body] && _host.Core.Bodies.IsDynamic(st.Body);
                if (s.Contains("character", c) || s.Bodies[body]) { changed |= s.Add("character", c); changed |= s.AddBody(body); changed |= s.AddBody(st.Body); }
                else if (heldMoves) { changed |= s.Add("character", c); changed |= s.AddBody(body); }
            }
            else if (st.Mode == CharacterMode.Roped)
            {
                if (s.Contains("character", c) || s.Bodies[body]) { changed |= s.Add("character", c); changed |= s.AddBody(body); changed |= s.Add("rod", st.Rod); }
                else if (s.Contains("rod", st.Rod)) { changed |= s.Add("character", c); changed |= s.AddBody(body); }
            }
        }

        return changed;
    }

    /// <summary>After a restore: every record belongs to a live character of the generation it names.</summary>
    public bool Consistent()
    {
        for (int c = 0; c < States.Length; c++)
        {
            if (!IsActive(in States[c])) { continue; }
            if (c >= _host.CharacterHighWater || !_host.CharacterLive(c)) { return false; }
            if (States[c].SavedGeneration != 0 && States[c].SavedGeneration != _host.CharacterGeneration(c)) { return false; }
            States[c].SavedGeneration = 0;
        }

        return true;
    }
}
