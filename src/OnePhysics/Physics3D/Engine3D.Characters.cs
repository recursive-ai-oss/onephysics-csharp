using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Collision3D;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

internal sealed partial class Engine3D
{
    /// <summary>Block flag of the characters block in the 3D section.</summary>
    private const uint BlockCharacters = 4;

    /// <summary>Slots of the characters (handles with generations, lowest free slot first).</summary>
    public HandleTable CharHandles = new();

    public int[] CharBody = new int[8];          // body slot
    public int[] CharBodyGen = new int[8];
    public CharacterDef[] CharDef = new CharacterDef[8];
    public GroundState[] CharGround = new GroundState[8];
    public float[] CharNx = new float[8], CharNy = new float[8], CharNz = new float[8];
    public int[] CharGroundBody = new int[8];    // body slot, -1 none
    public int[] CharGroundGen = new int[8];
    public float[] CharVx = new float[8], CharVy = new float[8], CharVz = new float[8];

    private void EnsureCharacters(int n)
    {
        Grow(ref CharBody, n); Grow(ref CharBodyGen, n); Grow(ref CharDef, n); Grow(ref CharGround, n);
        Grow(ref CharNx, n); Grow(ref CharNy, n); Grow(ref CharNz, n); Grow(ref CharGroundBody, n); Grow(ref CharGroundGen, n);
        Grow(ref CharVx, n); Grow(ref CharVy, n); Grow(ref CharVz, n);
    }

    public int CreateCharacter(BodyHandle body, in CharacterDef def)
    {
        int c = CharHandles.Allocate();
        EnsureCharacters(c + 1);
        CharBody[c] = body.Index; CharBodyGen[c] = body.Generation;
        CharDef[c] = def;
        IndexCharacterBody(body.Index, c);
        RecountVirtual();
        CharGround[c] = GroundState.InAir; CharNx[c] = 0f; CharNy[c] = 0f; CharNz[c] = 0f; CharGroundBody[c] = -1; CharGroundGen[c] = 0;
        CharVx[c] = 0f; CharVy[c] = 0f; CharVz[c] = 0f;
        return c;
    }

    /// <summary>Body slots that carry a character (derived from the registry, not saved): -1 or beyond the array when none.</summary>
    private int[] _charOfBody = Array.Empty<int>();

    /// <summary>Number of live virtual characters (derived).</summary>
    private int _virtualCharacters;

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

    private void RecountVirtual()
    {
        int n = 0;
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (CharHandles.IsLive(c) && CharDef[c].Kind == CharacterKind.Virtual) { n++; }
        }

        _virtualCharacters = n;
    }

    private bool IsCharacterBody(int body) => body < _charOfBody.Length && _charOfBody[body] >= 0;

    private bool IsVirtualCharacterBody(int body) => body < _charOfBody.Length && _charOfBody[body] >= 0 && CharDef[_charOfBody[body]].Kind == CharacterKind.Virtual;

    /// <summary>
    /// A virtual character's body moves by the mover alone and never takes part in contacts with solid shapes (it would push dynamic bodies with unlimited force); sensors still see it. Pure function of
    /// the pair, so it is safe in the parallel pair search.
    /// </summary>
    private bool CharacterPairBlocked(int sa, int sb)
    {
        var owner = Core.Shapes.Owner;
        bool ca = IsVirtualCharacterBody(owner[sa]), cb = IsVirtualCharacterBody(owner[sb]);
        if (!ca && !cb) { return false; }
        // A sensor on the other side keeps its pair.
        if (ca && (ShapeFlags[sb] & FlagSensor) == 0) { return true; }
        return cb && (ShapeFlags[sa] & FlagSensor) == 0;
    }

    /// <summary>The character slot whose body is the given body slot, or -1.</summary>
    public int CharacterOfBody(int body) => body < _charOfBody.Length ? _charOfBody[body] : -1;

    /// <summary>Called when a body is destroyed: its character, if any, goes with it.</summary>
    public void DropCharacterOfBody(int body)
    {
        int c = CharacterOfBody(body);
        if (c >= 0)
        {
            CharHandles.Free(c);
            _charOfBody[body] = -1;
            RecountVirtual();
        }
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
            g.Add(new Debug.DebugCharacter(Core.HandleOfBody(b), new Vector3D(p[0], p[1], p[2]), CharDef[c].Radius, CharDef[c].Height, gr, gr ? new Vector3D(CharNx[c], CharNy[c], CharNz[c]) : default, ground));
        }
    }

    // ------------------------------------------------------------------ rigid characters

    /// <summary>
    /// Drives a rigid character: the horizontal part of the displacement becomes a velocity (carried by the ground it stands on, tangent to it so slopes within the limit are walked, not hopped),
    /// an upward part is a jump, and on ground within the slope limit gravity is held off so it neither creeps down slopes nor sinks. On a steeper slope the push into the slope is removed, so the
    /// slope limit holds. Everything else — contacts, pushing, being pushed — is the solver's.
    /// </summary>
    public MoveResult3D MoveRigid(PhysicsWorld3D world, int ch, Vector3F displacement, float dt)
    {
        if (!float.IsFinite(displacement.X) || !float.IsFinite(displacement.Y) || !float.IsFinite(displacement.Z) || !(displacement.LengthSquared() <= 1e10f))
        {
            throw new ArgumentOutOfRangeException(nameof(displacement), displacement, "The displacement must be finite and at most 100000 long.");
        }

        if (!(dt > 0f) || !float.IsFinite(dt)) { throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be finite and positive."); }
        var def = CharDef[ch];
        int b = CharBody[ch];
        var bodies = Core.Bodies;
        var pp = bodies.PositionOf(b);
        var c = MakeMoverCtx(in def, b);
        var pos = new Vector3D(pp[0], pp[1], pp[2]);
        var ground = MvDetectGround(pos, default, ref c, out Vector3F gn, out int gslot, out double gx, out double gy, out double gz, out _);
        var up = c.Up;
        float upDist = Vector3F.Dot(displacement, up);
        var vh = (displacement - up * upDist) / dt;
        var lin = bodies.LinearVelocityOf(b);
        var cur = new Vector3F(lin[0], lin[1], lin[2]);
        Vector3F groundVel = default;
        bool dynamicGround = false;
        if (ground == GroundState.OnGround && gslot >= 0)
        {
            int gb = Core.Shapes.Owner[gslot];
            groundVel = BodyVelocityAt(gb, gx, gy, gz);
            dynamicGround = bodies.TypeOf(gb) == BodyType.Dynamic;
        }
        else if (ground == GroundState.OnSteepGround)
        {
            // No walking up a slope that is too steep: remove the horizontal push into it.
            var nh = gn - up * Vector3F.Dot(gn, up);
            float l2 = nh.LengthSquared();
            float into = Vector3F.Dot(vh, nh);
            if (l2 > 1e-12f && into < 0f) { vh -= nh * (into / l2); }
        }

        Vector3F v;
        if (ground == GroundState.OnGround)
        {
            float nu = Vector3F.Dot(gn, up);
            // The velocity along the ground with the wanted horizontal part: horizontal speed is kept up and down slopes.
            var tangent = vh + up * (-Vector3F.Dot(vh, gn) / nu);
            v = groundVel + tangent;
            if (upDist > 0f) { v = groundVel + vh + up * (upDist / dt); }
        }
        else
        {
            float vu = Vector3F.Dot(cur, up);
            if (upDist > 0f) { vu = Math.Max(vu, upDist / dt); }
            v = vh + up * vu;
        }

        lin[0] = v.X; lin[1] = v.Y; lin[2] = v.Z;
        // On walkable static or moving ground the character holds its place without gravity; on a dynamic body the weight must press down, so gravity stays.
        GravityScale[b] = ground == GroundState.OnGround && !dynamicGround && !(upDist > 0f) ? 0f : 1f;
        CharGround[ch] = ground;
        CharNx[ch] = ground != GroundState.InAir ? gn.X : 0f; CharNy[ch] = ground != GroundState.InAir ? gn.Y : 0f; CharNz[ch] = ground != GroundState.InAir ? gn.Z : 0f;
        if (ground == GroundState.OnGround && gslot >= 0) { int gb = Core.Shapes.Owner[gslot]; CharGroundBody[ch] = gb; CharGroundGen[ch] = bodies.Handles.GenerationOf(gb); }
        else { CharGroundBody[ch] = -1; CharGroundGen[ch] = 0; }
        CharVx[ch] = v.X; CharVy[ch] = v.Y; CharVz[ch] = v.Z;
        new Body(world, Core.HandleOfBody(b)).Wake();
        var flags = ground == GroundState.OnGround ? CharacterContact.Below : ground == GroundState.OnSteepGround ? CharacterContact.Sides : CharacterContact.None;
        return new MoveResult3D(flags, v * dt, 0);
    }

    // ------------------------------------------------------------------ snapshot block

    private bool HasCharacters => CharHandles.Count > 0;

    private void WriteCharacters(StateWriter w)
    {
        CharHandles.Write(w);
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c)) { continue; }
            var d = CharDef[c];
            w.WriteInt32(CharBody[c]); w.WriteInt32(CharBodyGen[c]);
            w.WriteSingle(d.Radius); w.WriteSingle(d.Height); w.WriteSingle(d.SlopeLimit); w.WriteSingle(d.StepOffset); w.WriteSingle(d.Skin);
            w.WriteUInt32(d.LayerMask); w.WriteSingle(d.PushStrength); w.WriteInt32(d.Layer); w.WriteUInt32(d.CategoryBits); w.WriteUInt32(d.MaskBits);
            w.WriteSingle(d.Up.X); w.WriteSingle(d.Up.Y); w.WriteSingle(d.Up.Z); w.WriteByte((byte)d.Kind); w.WriteSingle(d.Mass); w.WriteSingle(d.MaxPushForce);
            w.WriteByte((byte)CharGround[c]); w.WriteSingle(CharNx[c]); w.WriteSingle(CharNy[c]); w.WriteSingle(CharNz[c]);
            w.WriteInt32(CharGroundBody[c]); w.WriteInt32(CharGroundGen[c]);
            w.WriteSingle(CharVx[c]); w.WriteSingle(CharVy[c]); w.WriteSingle(CharVz[c]);
        }
    }

    private sealed class CharacterBlock
    {
        public HandleTable Handles = new();
        public int[] Body = Array.Empty<int>(), BodyGen = Array.Empty<int>(), GroundBody = Array.Empty<int>(), GroundGen = Array.Empty<int>();
        public CharacterDef[] Def = Array.Empty<CharacterDef>();
        public GroundState[] Ground = Array.Empty<GroundState>();
        public float[] Nx = Array.Empty<float>(), Ny = Array.Empty<float>(), Nz = Array.Empty<float>(), Vx = Array.Empty<float>(), Vy = Array.Empty<float>(), Vz = Array.Empty<float>();
    }

    private static CharacterBlock ReadCharacters(StateReader r, int bodyCount)
    {
        var b = new CharacterBlock { Handles = HandleTable.Read(r, "character") };
        int n = b.Handles.HighWater;
        b.Body = new int[n]; b.BodyGen = new int[n]; b.GroundBody = new int[n]; b.GroundGen = new int[n]; b.Def = new CharacterDef[n]; b.Ground = new GroundState[n];
        b.Nx = new float[n]; b.Ny = new float[n]; b.Nz = new float[n]; b.Vx = new float[n]; b.Vy = new float[n]; b.Vz = new float[n];
        for (int c = 0; c < n; c++)
        {
            if (!b.Handles.IsLive(c)) { continue; }
            b.Body[c] = r.ReadInt32(); b.BodyGen[c] = r.ReadInt32();
            if (b.Body[c] < 0 || b.Body[c] >= bodyCount || b.BodyGen[c] < 1) { throw r.Error("invalid character body"); }
            var d = new CharacterDef
            {
                Radius = r.ReadSingle(), Height = r.ReadSingle(), SlopeLimit = r.ReadSingle(), StepOffset = r.ReadSingle(), Skin = r.ReadSingle(),
                LayerMask = r.ReadUInt32(), PushStrength = r.ReadSingle(), Layer = r.ReadInt32(), CategoryBits = r.ReadUInt32(), MaskBits = r.ReadUInt32(),
            };
            d.Up = new Vector3F(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
            d.Kind = (CharacterKind)r.ReadByte();
            d.Mass = r.ReadSingle();
            d.MaxPushForce = r.ReadSingle();
            try { d.Validate(); }
            catch (ArgumentOutOfRangeException e) { throw r.Error("invalid character definition: " + e.Message); }
            b.Def[c] = d;
            byte gs = r.ReadByte();
            if (gs > (byte)GroundState.OnGround) { throw r.Error("invalid character ground state"); }
            b.Ground[c] = (GroundState)gs;
            b.Nx[c] = r.ReadSingle(); b.Ny[c] = r.ReadSingle(); b.Nz[c] = r.ReadSingle();
            b.GroundBody[c] = r.ReadInt32(); b.GroundGen[c] = r.ReadInt32();
            b.Vx[c] = r.ReadSingle(); b.Vy[c] = r.ReadSingle(); b.Vz[c] = r.ReadSingle();
            if (!float.IsFinite(b.Nx[c] + b.Ny[c] + b.Nz[c] + b.Vx[c] + b.Vy[c] + b.Vz[c])) { throw r.Error("non-finite character state"); }
            if (b.GroundBody[c] < -1 || b.GroundBody[c] >= bodyCount || (b.GroundBody[c] >= 0 && b.GroundGen[c] < 1)) { throw r.Error("invalid character ground body"); }
        }

        return b;
    }

    private void CommitCharacters(CharacterBlock? b)
    {
        Array.Fill(_charOfBody, -1);
        if (b == null)
        {
            CharHandles = new HandleTable();
            _virtualCharacters = 0;
            return;
        }

        CharHandles = b.Handles;
        int n = Math.Max(b.Handles.HighWater, 1);
        CharBody = new int[n]; CharBodyGen = new int[n]; CharDef = new CharacterDef[n]; CharGround = new GroundState[n];
        CharNx = new float[n]; CharNy = new float[n]; CharNz = new float[n]; CharGroundBody = new int[n]; CharGroundGen = new int[n];
        CharVx = new float[n]; CharVy = new float[n]; CharVz = new float[n];
        for (int c = 0; c < b.Handles.HighWater; c++)
        {
            CharBody[c] = b.Body[c]; CharBodyGen[c] = b.BodyGen[c]; CharDef[c] = b.Def[c]; CharGround[c] = b.Ground[c];
            CharNx[c] = b.Nx[c]; CharNy[c] = b.Ny[c]; CharNz[c] = b.Nz[c]; CharGroundBody[c] = b.GroundBody[c]; CharGroundGen[c] = b.GroundGen[c];
            CharVx[c] = b.Vx[c]; CharVy[c] = b.Vy[c]; CharVz[c] = b.Vz[c];
        }

        for (int c = 0; c < b.Handles.HighWater; c++)
        {
            if (b.Handles.IsLive(c)) { IndexCharacterBody(b.Body[c], c); }
        }

        RecountVirtual();
    }

    /// <summary>After a restore: every live character must stand on a live body of the generation it names and of the type its kind needs, and no body carries two characters.</summary>
    private bool CharactersConsistent()
    {
        var bodies = Core.Bodies;
        for (int c = 0; c < CharHandles.HighWater; c++)
        {
            if (!CharHandles.IsLive(c)) { continue; }
            int b = CharBody[c];
            var want = CharDef[c].Kind == CharacterKind.Virtual ? BodyType.Kinematic : BodyType.Dynamic;
            if (!bodies.Handles.IsAlive(b, CharBodyGen[c]) || bodies.TypeOf(b) != want || _charOfBody[b] != c) { return false; }
        }

        return true;
    }
}
