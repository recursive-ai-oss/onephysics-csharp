using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

internal sealed partial class Engine2D
{
    /// <summary>Section version written when the world has characters: a presence-flags word, then the flagged blocks (joints, services, characters) in that order.</summary>
    private const uint WithCharacters = 4;
    private const uint BlockJoints = 1, BlockServices = 2, BlockCharacters = 4;

    /// <summary>Slots of the characters (handles with generations, lowest free slot first).</summary>
    public HandleTable CharHandles = new();

    public int[] CharBody = new int[8];          // body slot
    public int[] CharBodyGen = new int[8];
    public CharacterDef[] CharDef = new CharacterDef[8];
    public GroundState[] CharGround = new GroundState[8];
    public float[] CharNx = new float[8], CharNy = new float[8];
    public int[] CharGroundBody = new int[8];    // body slot, -1 none
    public int[] CharGroundGen = new int[8];
    public float[] CharVx = new float[8], CharVy = new float[8];

    private void EnsureCharacters(int n)
    {
        Grow(ref CharBody, n); Grow(ref CharBodyGen, n); Grow(ref CharDef, n); Grow(ref CharGround, n);
        Grow(ref CharNx, n); Grow(ref CharNy, n); Grow(ref CharGroundBody, n); Grow(ref CharGroundGen, n); Grow(ref CharVx, n); Grow(ref CharVy, n);
    }

    public int CreateCharacter(BodyHandle body, in CharacterDef def)
    {
        int c = CharHandles.Allocate();
        EnsureCharacters(c + 1);
        CharBody[c] = body.Index; CharBodyGen[c] = body.Generation;
        CharDef[c] = def;
        IndexCharacterBody(body.Index, c);
        CharGround[c] = GroundState.InAir; CharNx[c] = 0f; CharNy[c] = 0f; CharGroundBody[c] = -1; CharGroundGen[c] = 0; CharVx[c] = 0f; CharVy[c] = 0f;
        return c;
    }

    /// <summary>Body slots that carry a character (derived from the registry, not saved): -1 or beyond the array when none.</summary>
    private int[] _charOfBody = Array.Empty<int>();

    private void IndexCharacterBody(int body, int character)
    {
        if (body >= _charOfBody.Length)
        {
            int old = _charOfBody.Length;
            Array.Resize(ref _charOfBody, Math.Max(body + 1, old * 2));
            for (int i = old; i < _charOfBody.Length; i++) { _charOfBody[i] = -1; }
        }

        _charOfBody[body] = character;
    }

    private bool IsCharacterBody(int body) => body < _charOfBody.Length && _charOfBody[body] >= 0;

    /// <summary>The character slot whose body is the given body slot, or -1.</summary>
    public int CharacterOfBody(int body)
    {
        if (CharHandles.Count == 0) { return -1; }
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (CharHandles.IsLive(c) && CharBody[c] == body) { return c; }
        }

        return -1;
    }

    /// <summary>Called when a body is destroyed: its character, if any, goes with it.</summary>
    public void DropCharacterOfBody(int body)
    {
        int c = CharacterOfBody(body);
        if (c >= 0) { CharHandles.Free(c); _charOfBody[body] = -1; }
    }

    /// <summary>Emits one entry per live character, by slot: capsule centre, ground flag, ground normal and ground body.</summary>
    public void AddDebugCharacters(Debug.DebugGeometry g)
    {
        var bodies = Core.Bodies;
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c)) { continue; }
            int b = CharBody[c];
            var p = bodies.PositionOf(b);
            bool gr = CharGround[c] == GroundState.OnGround;
            var ground = gr && CharGroundBody[c] >= 0 && bodies.Handles.IsAlive(CharGroundBody[c], CharGroundGen[c]) ? Core.HandleOfBody(CharGroundBody[c]) : default;
            g.Add(new Debug.DebugCharacter(Core.HandleOfBody(b), new Vector3D(p[0], p[1], 0.0), CharDef[c].Radius, CharDef[c].Height, gr, gr ? new Vector3D(CharNx[c], CharNy[c], 0.0) : default, ground));
        }
    }

    // ------------------------------------------------------------------ snapshot block

    private void WriteCharacters(StateWriter w)
    {
        CharHandles.Write(w);
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c)) { continue; }
            var d = CharDef[c];
            w.WriteInt32(CharBody[c]); w.WriteInt32(CharBodyGen[c]);
            w.WriteSingle(d.Radius); w.WriteSingle(d.Height); w.WriteSingle(d.SlopeLimitDegrees); w.WriteSingle(d.StepOffset); w.WriteSingle(d.Skin);
            w.WriteUInt32(d.LayerMask); w.WriteSingle(d.PushStrength); w.WriteInt32(d.Layer); w.WriteUInt32(d.CategoryBits); w.WriteUInt32(d.MaskBits);
            w.WriteSingle(d.Mass); w.WriteSingle(d.MaxPushForce);
            w.WriteByte((byte)CharGround[c]); w.WriteSingle(CharNx[c]); w.WriteSingle(CharNy[c]);
            w.WriteInt32(CharGroundBody[c]); w.WriteInt32(CharGroundGen[c]);
            w.WriteSingle(CharVx[c]); w.WriteSingle(CharVy[c]);
        }
    }

    private sealed class CharacterBlock
    {
        public HandleTable Handles = new();
        public int[] Body = new int[8], BodyGen = new int[8], GroundBody = new int[8], GroundGen = new int[8];
        public CharacterDef[] Def = new CharacterDef[8];
        public GroundState[] Ground = new GroundState[8];
        public float[] Nx = new float[8], Ny = new float[8], Vx = new float[8], Vy = new float[8];
    }

    private readonly CharacterBlock _charStage = new();

    // Reads into the engine's staging block, which is kept between restores so nothing is allocated once it has grown.
    private CharacterBlock ReadCharacters(StateReader r, int bodyCount)
    {
        var b = _charStage;
        b.Handles.ReadInto(r, "character");
        int n = b.Handles.HighWater;
        if (b.Body.Length < n)
        {
            int cap = Math.Max(n, 8);
            b.Body = new int[cap]; b.BodyGen = new int[cap]; b.GroundBody = new int[cap]; b.GroundGen = new int[cap]; b.Def = new CharacterDef[cap]; b.Ground = new GroundState[cap];
            b.Nx = new float[cap]; b.Ny = new float[cap]; b.Vx = new float[cap]; b.Vy = new float[cap];
        }
        else
        {
            // Dead slots read back as zero, as in a freshly built block.
            Array.Clear(b.Body, 0, n); Array.Clear(b.BodyGen, 0, n); Array.Clear(b.GroundBody, 0, n); Array.Clear(b.GroundGen, 0, n); Array.Clear(b.Def, 0, n); Array.Clear(b.Ground, 0, n);
            Array.Clear(b.Nx, 0, n); Array.Clear(b.Ny, 0, n); Array.Clear(b.Vx, 0, n); Array.Clear(b.Vy, 0, n);
        }

        for (int c = 0; c < n; c++)
        {
            if (!b.Handles.IsLive(c)) { continue; }
            b.Body[c] = r.ReadInt32(); b.BodyGen[c] = r.ReadInt32();
            if (b.Body[c] < 0 || b.Body[c] >= bodyCount || b.BodyGen[c] < 1) { throw r.Error("invalid character body"); }
            var d = new CharacterDef
            {
                Radius = r.ReadSingle(), Height = r.ReadSingle(), SlopeLimitDegrees = r.ReadSingle(), StepOffset = r.ReadSingle(), Skin = r.ReadSingle(),
                LayerMask = r.ReadUInt32(), PushStrength = r.ReadSingle(), Layer = r.ReadInt32(), CategoryBits = r.ReadUInt32(), MaskBits = r.ReadUInt32(),
            };
            d.Mass = r.ReadSingle();
            d.MaxPushForce = r.ReadSingle();
            try { d.Validate(); }
            catch (ArgumentOutOfRangeException e) { throw r.Error("invalid character definition: " + e.Message); }
            b.Def[c] = d;
            byte gs = r.ReadByte();
            if (gs > (byte)GroundState.OnGround) { throw r.Error("invalid character ground state"); }
            b.Ground[c] = (GroundState)gs;
            b.Nx[c] = r.ReadSingle(); b.Ny[c] = r.ReadSingle();
            b.GroundBody[c] = r.ReadInt32(); b.GroundGen[c] = r.ReadInt32();
            b.Vx[c] = r.ReadSingle(); b.Vy[c] = r.ReadSingle();
            if (!float.IsFinite(b.Nx[c] + b.Ny[c] + b.Vx[c] + b.Vy[c])) { throw r.Error("non-finite character state"); }
            if (b.GroundBody[c] < -1 || b.GroundBody[c] >= bodyCount || (b.GroundBody[c] >= 0 && b.GroundGen[c] < 1)) { throw r.Error("invalid character ground body"); }
        }

        return b;
    }

    private void CommitCharacters(CharacterBlock? b)
    {
        if (b == null)
        {
            CharHandles.Reset();
            Array.Fill(_charOfBody, -1);
            return;
        }

        (CharHandles, b.Handles) = (b.Handles, CharHandles);
        Array.Fill(_charOfBody, -1);
        int n = Math.Max(CharHandles.HighWater, 1);
        if (CharBody.Length < n)
        {
            int cap = Math.Max(n, 8);
            CharBody = new int[cap]; CharBodyGen = new int[cap]; CharDef = new CharacterDef[cap]; CharGround = new GroundState[cap];
            CharNx = new float[cap]; CharNy = new float[cap]; CharGroundBody = new int[cap]; CharGroundGen = new int[cap]; CharVx = new float[cap]; CharVy = new float[cap];
        }

        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            CharBody[c] = b.Body[c]; CharBodyGen[c] = b.BodyGen[c]; CharDef[c] = b.Def[c]; CharGround[c] = b.Ground[c];
            if (CharHandles.IsLive(c)) { IndexCharacterBody(b.Body[c], c); }
            CharNx[c] = b.Nx[c]; CharNy[c] = b.Ny[c]; CharGroundBody[c] = b.GroundBody[c]; CharGroundGen[c] = b.GroundGen[c]; CharVx[c] = b.Vx[c]; CharVy[c] = b.Vy[c];
        }
    }

    /// <summary>After a restore: every live character must stand on a live kinematic body of the generation it names.</summary>
    private bool CharactersConsistent()
    {
        var bodies = Core.Bodies;
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c)) { continue; }
            int b = CharBody[c];
            if (!bodies.Handles.IsAlive(b, CharBodyGen[c]) || bodies.TypeOf(b) != BodyType.Kinematic) { return false; }
        }

        return true;
    }
}
