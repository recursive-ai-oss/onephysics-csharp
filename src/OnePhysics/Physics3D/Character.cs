using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

/// <summary>What <see cref="Character.OnControllerColliderHit"/> reports (Unity's <c>ControllerColliderHit</c>): a surface the character was deflected by during a move.</summary>
public readonly struct ControllerColliderHit
{
    internal ControllerColliderHit(Character character, in MoverHit3D hit, Vector3F moveDirection, float moveLength)
    {
        Character = character;
        Collider = hit.Shape;
        Point = hit.Point;
        Normal = hit.Normal;
        MoveDirection = moveDirection;
        MoveLength = moveLength;
    }

    /// <summary>The character that moved.</summary>
    public Character Character { get; }

    /// <summary>The shape that was touched.</summary>
    public Shape Collider { get; }

    /// <summary>The body of <see cref="Collider"/>.</summary>
    public Body Body => Collider.Body;

    /// <summary>A point on the contact, world coordinates.</summary>
    public Vector3D Point { get; }

    /// <summary>Unit normal pointing from the touched shape towards the character.</summary>
    public Vector3F Normal { get; }

    /// <summary>Unit direction of the move that caused the hit (the wanted displacement, normalized; zero if it was zero).</summary>
    public Vector3F MoveDirection { get; }

    /// <summary>Length of the wanted displacement, metres.</summary>
    public float MoveLength { get; }
}

/// <summary>
/// A character: an upright capsule moved by <see cref="Move(Vector3F, float)"/>. A <see cref="CharacterKind.Virtual"/> character (the default) is a kinematic body moved by collide-and-slide: it stands on ground
/// within <see cref="SlopeLimit"/>, climbs steps up to <see cref="StepOffset"/>, slides along walls and steeper slopes, rides moving ground and can push dynamic bodies with a bounded force.
/// A <see cref="CharacterKind.Rigid"/> character is a dynamic capsule whose velocity the move sets; the solver does the rest. Like <see cref="Body"/> it is a small struct handle (world, slot, generation)
/// that allocates nothing and fails with a <see cref="StaleHandleException"/> once the character is destroyed; the state lives in the world and in its snapshots.
/// </summary>
/// <example><c>var player = world.AddCharacter(at: new Vector3D(0, 1, 0), Character.Humanoid); player.Move(new Vector3F(input.X * speed * dt, velocityY * dt, input.Z * speed * dt), dt); world.Step(dt);</c></example>
public readonly struct Character : IEquatable<Character>
{
    private readonly PhysicsWorld3D? _world;
    private readonly int _index;
    private readonly int _generation;

    internal Character(PhysicsWorld3D world, int index, int generation)
    {
        _world = world;
        _index = index;
        _generation = generation;
    }

    /// <summary>A person: 1.8 m tall, 0.3 m radius, 45 degree slope limit, 0.3 m step, 70 kg.</summary>
    public static CharacterDef Humanoid => new();

    /// <summary>A child: 1.2 m tall, 0.25 m radius, 45 degree slope limit, 0.2 m step, 30 kg.</summary>
    public static CharacterDef Child => new() { Radius = 0.25f, Height = 1.2f, StepOffset = 0.2f, Mass = 30f };

    /// <summary>A large creature: 3 m tall, 0.6 m radius, 40 degree slope limit, 0.5 m step, 400 kg.</summary>
    public static CharacterDef Giant => new() { Radius = 0.6f, Height = 3f, SlopeLimit = 40f, StepOffset = 0.5f, Skin = 0.02f, Mass = 400f, MaxPushForce = 2000f };

    /// <summary>A person simulated as a dynamic body (<see cref="CharacterKind.Rigid"/>).</summary>
    public static CharacterDef RigidHumanoid => new() { Kind = CharacterKind.Rigid };

    private Engine3D Resolve(out int slot)
    {
        if (_world == null) { throw new InvalidHandleException("The default Character is not attached to a world."); }
        var e = _world.Engine;
        if (!e.CharHandles.IsAlive(_index, _generation))
        {
            throw new StaleHandleException("Character(world " + _world.Id.ToString(System.Globalization.CultureInfo.InvariantCulture) + ", #" + _index.ToString(System.Globalization.CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(System.Globalization.CultureInfo.InvariantCulture) + ") is stale: the character was destroyed.");
        }

        slot = _index;
        return e;
    }

    /// <summary>The character's slot index in its world; with <see cref="Generation"/> it identifies the character (see <see cref="PhysicsWorld3D.GetCharacter"/>).</summary>
    public int Index => _index;

    /// <summary>The generation of the slot when the character was created; a later character reusing the slot has a different one.</summary>
    public int Generation => _generation;

    /// <summary>True while the character exists.</summary>
    public bool IsValid => _world != null && _world.Engine.CharHandles.IsAlive(_index, _generation);

    /// <summary>The body that carries the character's capsule (centred on the body's origin): kinematic for a virtual character, dynamic for a rigid one.</summary>
    public Body Body { get { var e = Resolve(out int c); return new Body(_world!, new BodyHandle(_world!.Id, e.CharBody[c], e.CharBodyGen[c])); } }

    /// <summary>Virtual or rigid; fixed at creation.</summary>
    public CharacterKind Kind { get { var e = Resolve(out int c); return e.CharDef[c].Kind; } }

    /// <summary>Centre of the capsule in world coordinates. Setting it teleports the character (it forgets what it stood on) and wakes what it lands on.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond +-2^21.</exception>
    public Vector3D Position
    {
        get => Body.Position;
        set
        {
            var e = Resolve(out int c);
            Body.SetTransform(value, Body.Rotation);
            e.CharGround[c] = GroundState.InAir; e.CharNx[c] = 0f; e.CharNy[c] = 0f; e.CharNz[c] = 0f; e.CharGroundBody[c] = -1; e.CharGroundGen[c] = 0;
        }
    }

    /// <summary>The velocity the last move gave the character (displacement divided by dt for a virtual character), metres per second; the body moves with it during the next step.</summary>
    public Vector3F Velocity { get { var e = Resolve(out int c); return new Vector3F(e.CharVx[c], e.CharVy[c], e.CharVz[c]); } }

    /// <summary>True when the last move ended standing on ground within <see cref="SlopeLimit"/>.</summary>
    public bool IsGrounded { get { var e = Resolve(out int c); return e.CharGround[c] == GroundState.OnGround; } }

    /// <summary>On ground, on a slope too steep to stand on, or in the air, as of the last move.</summary>
    public GroundState Ground { get { var e = Resolve(out int c); return e.CharGround[c]; } }

    /// <summary>Normal of the ground under the character when it is on ground or on a steep slope; zero otherwise.</summary>
    public Vector3F GroundNormal { get { var e = Resolve(out int c); return new Vector3F(e.CharNx[c], e.CharNy[c], e.CharNz[c]); } }

    /// <summary>The body the character stands on when grounded (it may be static), otherwise the default (invalid) body.</summary>
    public Body GroundBody
    {
        get
        {
            var e = Resolve(out int c);
            if (e.CharGround[c] != GroundState.OnGround) { return default; }
            var h = new BodyHandle(_world!.Id, e.CharGroundBody[c], e.CharGroundGen[c]);
            return _world.Core.Bodies.Handles.IsAlive(h.Index, h.Generation) ? new Body(_world, h) : default;
        }
    }

    /// <summary>Velocity of the ground under the character (zero when not grounded or on static ground); the next move carries the character with it.</summary>
    public Vector3F GroundVelocity
    {
        get
        {
            var gb = GroundBody;
            if (!gb.IsValid) { return default; }
            var p = Position;
            return _world!.Engine.BodyVelocityAt(gb.RawHandle.Index, p.X, p.Y, p.Z);
        }
    }

    /// <summary>The direction the capsule stands in (unit); fixed at creation.</summary>
    public Vector3F Up { get { var e = Resolve(out int c); return Engine3D.UnitUp(e.CharDef[c].Up); } }

    /// <summary>Steepest ground the character can stand on, degrees within [0, 90] (Unity's <c>slopeLimit</c>).</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside [0, 90].</exception>
    public float SlopeLimit
    {
        get { var e = Resolve(out int c); return e.CharDef[c].SlopeLimit; }
        set { var e = Resolve(out int c); var d = e.CharDef[c]; d.SlopeLimit = value; d.Validate(); e.CharDef[c] = d; }
    }

    /// <summary>Tallest step climbed and farthest snap down to ground, metres, within [0, Height] (Unity's <c>stepOffset</c>).</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside [0, Height].</exception>
    public float StepOffset
    {
        get { var e = Resolve(out int c); return e.CharDef[c].StepOffset; }
        set { var e = Resolve(out int c); var d = e.CharDef[c]; d.StepOffset = value; d.Validate(); e.CharDef[c] = d; }
    }

    /// <summary>How hard a virtual character pushes dynamic bodies it walks into; 0 (default) does not push. See <see cref="CharacterDef.PushStrength"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside [0, 100].</exception>
    public float PushStrength
    {
        get { var e = Resolve(out int c); return e.CharDef[c].PushStrength; }
        set { var e = Resolve(out int c); var d = e.CharDef[c]; d.PushStrength = value; d.Validate(); e.CharDef[c] = d; }
    }

    /// <summary>The most force a virtual character pushes a dynamic body with, newtons, within [0, 1e9]. See <see cref="CharacterDef.MaxPushForce"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Setting a value outside [0, 1e9].</exception>
    public float MaxPushForce
    {
        get { var e = Resolve(out int c); return e.CharDef[c].MaxPushForce; }
        set { var e = Resolve(out int c); var d = e.CharDef[c]; d.MaxPushForce = value; d.Validate(); e.CharDef[c] = d; }
    }

    /// <summary>Which layers the mover collides with (bit i: layer i). Default all.</summary>
    public uint LayerMask
    {
        get { var e = Resolve(out int c); return e.CharDef[c].LayerMask; }
        set { var e = Resolve(out int c); var d = e.CharDef[c]; d.LayerMask = value; e.CharDef[c] = d; }
    }

    /// <summary>Mass in kg (fixed at creation).</summary>
    public float Mass { get { var e = Resolve(out int c); return e.CharDef[c].Mass; } }

    /// <summary>Radius of the capsule, metres (fixed at creation).</summary>
    public float Radius { get { var e = Resolve(out int c); return e.CharDef[c].Radius; } }

    /// <summary>Total height of the capsule, metres (fixed at creation).</summary>
    public float Height { get { var e = Resolve(out int c); return e.CharDef[c].Height; } }

    /// <summary>
    /// Moves the character by <paramref name="displacement"/> and arranges for the next <see cref="PhysicsWorld3D.Step"/> to arrive there. Call it once per step, before the step.
    /// A virtual character slides, steps and rides ground (put walking and vertical motion, gravity and jumps, into the displacement); handlers of <see cref="OnControllerColliderHit"/> run before this returns.
    /// A rigid character takes the horizontal part as a velocity target (displacement divided by dt) and a positive part along up as a jump; gravity is the world's.
    /// </summary>
    /// <param name="displacement">Wanted movement this step, metres; finite, at most 100000 long.</param>
    /// <param name="dt">The step time in seconds (finite, positive).</param>
    /// <returns>Which sides touched something; <see cref="CharacterContact.Below"/> means standing on walkable ground.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or out of range, or the character would leave +-2^21.</exception>
    /// <exception cref="StaleHandleException">The character was destroyed.</exception>
    public CharacterContact Move(Vector3F displacement, float dt)
    {
        var e = Resolve(out int c);
        var w = _world!;
        if (e.CharDef[c].Kind == CharacterKind.Rigid) { return e.MoveRigid(w, c, displacement, dt).Flags; }
        var body = Body;
        var def = e.CharDef[c];
        var state = new MoverState3D { Position = body.Position, Ground = e.CharGround[c], GroundNormal = new Vector3F(e.CharNx[c], e.CharNy[c], e.CharNz[c]) };
        if (e.CharGround[c] == GroundState.OnGround && e.CharGroundBody[c] >= 0)
        {
            var gh = new BodyHandle(w.Id, e.CharGroundBody[c], e.CharGroundGen[c]);
            if (w.Core.Bodies.Handles.IsAlive(gh.Index, gh.Generation)) { state.GroundBody = new Body(w, gh); }
        }

        MoverHit3D[]? buffer = w.HasControllerHandler(_index, _generation) ? new MoverHit3D[16] : null;
        var result = Mover3D.Move(w, in def, ref state, displacement, dt, body, buffer);
        var v = new Vector3F(result.Displacement.X / dt, result.Displacement.Y / dt, result.Displacement.Z / dt);
        var lin = w.Core.Bodies.LinearVelocityOf(body.RawHandle.Index);
        lin[0] = v.X; lin[1] = v.Y; lin[2] = v.Z;
        e.CharVx[c] = v.X; e.CharVy[c] = v.Y; e.CharVz[c] = v.Z;
        e.CharGround[c] = state.Ground;
        e.CharNx[c] = state.GroundNormal.X; e.CharNy[c] = state.GroundNormal.Y; e.CharNz[c] = state.GroundNormal.Z;
        if (state.Ground == GroundState.OnGround) { var g = state.GroundBody.RawHandle; e.CharGroundBody[c] = g.Index; e.CharGroundGen[c] = g.Generation; }
        else { e.CharGroundBody[c] = -1; e.CharGroundGen[c] = 0; }

        if (buffer != null && result.HitCount > 0)
        {
            float len = displacement.Length();
            var dir = len > 0f ? displacement / len : default;
            for (int i = 0; i < result.HitCount; i++) { w.RaiseControllerHit(_index, _generation, new ControllerColliderHit(this, in buffer[i], dir, len)); }
        }

        return result.Flags;
    }

    /// <summary>Moves at <paramref name="velocity"/> for <paramref name="dt"/> seconds: <c>Move(velocity * dt, dt)</c>.</summary>
    public CharacterContact MoveVelocity(Vector3F velocity, float dt)
        => Move(velocity * dt, dt);

    /// <summary>Raised during <see cref="Move(Vector3F, float)"/> once for each shape that deflected a virtual character, in the order the solver met them (Unity's <c>OnControllerColliderHit</c>). Needs no struct callback. A rigid character reports contacts through its body's collision events.</summary>
    public event Action<ControllerColliderHit> OnControllerColliderHit
    {
        add
        {
            ArgumentNullException.ThrowIfNull(value);
            Resolve(out _);
            _world!.AddControllerHandler(_index, _generation, value);
        }
        remove
        {
            if (_world == null || value == null) { return; }
            _world.RemoveControllerHandler(_index, _generation, value);
        }
    }

    /// <summary>Destroys the character and its body. Every copy of this handle becomes stale.</summary>
    public void Destroy()
    {
        Resolve(out _);
        Body.Destroy();
    }

    /// <summary>Same world, slot and generation.</summary>
    public bool Equals(Character other) => ReferenceEquals(_world, other._world) && _index == other._index && _generation == other._generation;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Character c && Equals(c);

    /// <inheritdoc/>
    public override int GetHashCode() => unchecked(_index * 31 + _generation);

    /// <summary>Handle equality.</summary>
    public static bool operator ==(Character a, Character b) => a.Equals(b);

    /// <summary>Handle inequality.</summary>
    public static bool operator !=(Character a, Character b) => !a.Equals(b);

    /// <inheritdoc/>
    public override string ToString() => "Character(#" + _index.ToString(System.Globalization.CultureInfo.InvariantCulture) + ", gen " + _generation.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")";
}

public sealed partial class PhysicsWorld3D
{
    private readonly Dictionary<int, (int Generation, Action<ControllerColliderHit> Handler)> _controllerHandlers = new();

    /// <summary>
    /// Creates a character: a capsule standing along the definition's up direction, centred at <paramref name="at"/>. Use <see cref="Character.Humanoid"/>, <see cref="Character.Child"/>, <see cref="Character.Giant"/>,
    /// <see cref="Character.RigidHumanoid"/> or your own <see cref="CharacterDef"/>.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A definition value is out of range or the position is not finite or beyond +-2^21.</exception>
    public Character AddCharacter(Vector3D at, CharacterDef def)
    {
        def.Validate();
        if (!double.IsFinite(at.X) || !double.IsFinite(at.Y) || !double.IsFinite(at.Z) || Math.Abs(at.X) > 2097152.0 || Math.Abs(at.Y) > 2097152.0 || Math.Abs(at.Z) > 2097152.0)
        {
            throw new ArgumentOutOfRangeException(nameof(at), at, "Position " + at + " must be finite and within +-2^21 of the world origin.");
        }

        bool rigid = def.Kind == CharacterKind.Rigid;
        var bd = new BodyDef { Type = rigid ? BodyType.Dynamic : BodyType.Kinematic, Position = at, Rotation = Engine3D.AlignY(Engine3D.UnitUp(def.Up)), FixedRotation = true, AllowSleep = false };
        var body = CreateBody(in bd);
        try
        {
            float half = 0.5f * def.Height - def.Radius;
            // Density that gives a rigid capsule the requested mass: cylinder plus two hemispheres.
            float volume = (float)Math.PI * def.Radius * def.Radius * (2f * half + 4f / 3f * def.Radius);
            var sd = new ShapeDef { Density = rigid ? def.Mass / volume : 0f, Friction = rigid ? 0f : 0.6f, RollingResistance = 0f, Layer = def.Layer, CategoryBits = def.CategoryBits, MaskBits = def.MaskBits };
            CreateCapsule(body, half, def.Radius, sd);
        }
        catch
        {
            body.Destroy();
            throw;
        }

        int c = _engine.CreateCharacter(body.RawHandle, in def);
        return new Character(this, c, _engine.CharHandles.GenerationOf(c));
    }

    /// <summary>Creates a <see cref="Character.Humanoid"/> centred at <paramref name="at"/>.</summary>
    public Character AddCharacter(Vector3D at) => AddCharacter(at, Character.Humanoid);

    /// <summary>The <see cref="Character"/> for a raw character slot and generation (from the raw tier); fails when the character no longer exists.</summary>
    /// <exception cref="StaleHandleException">There is no such character.</exception>
    public Character GetCharacter(int index, int generation)
    {
        if (!_engine.CharHandles.IsAlive(index, generation)) { throw new StaleHandleException("Character #" + index.ToString(System.Globalization.CultureInfo.InvariantCulture) + " (generation " + generation.ToString(System.Globalization.CultureInfo.InvariantCulture) + ") does not exist."); }
        return new Character(this, index, generation);
    }

    /// <summary>The character whose body is <paramref name="body"/>, if it has one.</summary>
    public bool TryGetCharacter(Body body, out Character character)
    {
        character = default;
        if (!body.IsValid || body.World != this) { return false; }
        int c = _engine.CharacterOfBody(body.RawHandle.Index);
        if (c < 0) { return false; }
        character = new Character(this, c, _engine.CharHandles.GenerationOf(c));
        return true;
    }

    /// <summary>All live characters in slot order (allocates the array).</summary>
    public Character[] GetCharacters()
    {
        var list = new List<Character>();
        for (int c = 0; c < _engine.CharHandles.HighWater; c++)
        {
            if (_engine.CharHandles.IsLive(c)) { list.Add(new Character(this, c, _engine.CharHandles.GenerationOf(c))); }
        }

        return list.ToArray();
    }

    internal bool HasControllerHandler(int index, int generation)
        => _controllerHandlers.TryGetValue(index, out var h) && h.Generation == generation;

    internal void AddControllerHandler(int index, int generation, Action<ControllerColliderHit> handler)
    {
        _controllerHandlers.TryGetValue(index, out var h);
        _controllerHandlers[index] = h.Generation == generation && h.Handler != null ? (generation, h.Handler + handler) : (generation, handler);
    }

    internal void RemoveControllerHandler(int index, int generation, Action<ControllerColliderHit> handler)
    {
        if (!_controllerHandlers.TryGetValue(index, out var h) || h.Generation != generation) { return; }
        var left = h.Handler - handler;
        if (left == null) { _controllerHandlers.Remove(index); } else { _controllerHandlers[index] = (generation, left); }
    }

    internal void RaiseControllerHit(int index, int generation, in ControllerColliderHit hit)
    {
        if (_controllerHandlers.TryGetValue(index, out var h) && h.Generation == generation) { h.Handler(hit); }
    }

    private void DropDeadControllerHandlers()
    {
        var dead = _deadScratch;
        dead.Clear();
        foreach (var kv in _controllerHandlers)
        {
            if (!_engine.CharHandles.IsAlive(kv.Key, kv.Value.Generation)) { dead.Add(kv.Key); }
        }

        foreach (int k in dead) { _controllerHandlers.Remove(k); }
    }
}
