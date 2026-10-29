using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics;
using RecursiveAI.OnePhysics.LowLevel;

namespace RecursiveAI.OnePhysics.Physics2D;

/// <summary>Which sides of a character touched something during a move (Unity's <c>CollisionFlags</c>); combine with <c>|</c>.</summary>
[Flags]
public enum CharacterContact : byte
{
    /// <summary>Nothing blocked the move.</summary>
    None = 0,

    /// <summary>A wall or a slope steeper than the slope limit blocked or deflected the move.</summary>
    Sides = 1,

    /// <summary>A ceiling blocked an upward move.</summary>
    Above = 2,

    /// <summary>The character stands on ground it may walk on (a surface within the slope limit).</summary>
    Below = 4,
}

/// <summary>What a character stands on after its last move.</summary>
public enum GroundState : byte
{
    /// <summary>Nothing walkable or steep is within reach below the character.</summary>
    InAir = 0,

    /// <summary>The character touches a surface steeper than its slope limit that leans up towards it; it cannot stand there and slides.</summary>
    OnSteepGround = 1,

    /// <summary>The character stands on ground within its slope limit.</summary>
    OnGround = 2,
}

/// <summary>
/// One surface the mover must stay in front of, for the plane solver (<see cref="Mover2D.SolvePlanes"/>). The mover satisfies, for the movement vector <c>d</c>,
/// <c>dot(d, Normal) + Separation >= 0</c>, pushing itself out along the normal by at most <see cref="PushLimit"/> when it starts inside.
/// </summary>
public struct CollisionPlane2D
{
    /// <summary>Unit normal pointing from the obstacle towards the mover.</summary>
    public Vector2F Normal { get; set; }

    /// <summary>Distance from the mover to the plane along <see cref="Normal"/> at the start (including any skin to keep); negative when the mover is inside it.</summary>
    public float Separation { get; set; }

    /// <summary>The most the solver may move the mover out of this plane to undo an overlap, at least 0. 0 means the plane still blocks movement into it but never pushes the mover (use it for bodies that move out of the way themselves).</summary>
    public float PushLimit { get; set; }

    /// <summary>Output of the solver: how much this plane pushed the final movement along <see cref="Normal"/>; positive when the plane limited the movement, 0 when it did not.</summary>
    public float Push { get; set; }

    internal float CarryX { get; set; }

    internal float CarryY { get; set; }
}

/// <summary>Creation parameters of a character: a capsule standing upright, moved by <see cref="Character.Move(Vector2F, float)"/> with collide-and-slide. Defaults are a human: 1.8 m tall, 0.3 m radius, 45 degree slope limit, 0.3 m step.</summary>
public struct CharacterDef
{
    /// <summary>Radius of the capsule, metres, within (0, 1000]. Default 0.3.</summary>
    public float Radius { readonly get => ZeroDefault.Get(_radius, 0.3f); set => _radius = ZeroDefault.Set(value, 0.3f); }

    private uint _radius;

    /// <summary>Total height including both rounded ends, metres, at least twice the radius, at most 10000. Default 1.8.</summary>
    public float Height { readonly get => ZeroDefault.Get(_height, 1.8f); set => _height = ZeroDefault.Set(value, 1.8f); }

    private uint _height;

    /// <summary>
    /// Steepest ground the character can stand on, in radians within [0, pi/2] (Unity's <c>slopeLimit</c> is in degrees: pass 45 degrees converted to radians). Steeper surfaces are walls: the character slides down them.
    /// Default pi/4 (45 degrees).
    /// </summary>
    public float SlopeLimitRadians
    {
        readonly get => (float)(SlopeLimitDegrees * (Math.PI / 180.0));
        set => SlopeLimitDegrees = (float)(value * (180.0 / Math.PI));
    }

    // The engine and the snapshot keep the limit in degrees (as float32): 45 and 40 are exact there, which keeps saved worlds and their hashes unchanged.
    internal float SlopeLimitDegrees { readonly get => ZeroDefault.Get(_slopeLimit, 45f); set => _slopeLimit = ZeroDefault.Set(value, 45f); }

    private uint _slopeLimit;

    /// <summary>Tallest step the character climbs when walking into it, metres, at least 0 (0 turns stepping off), at most <see cref="Height"/>. It is also how far the character snaps down to stay on the ground when walking down a slope or stairs. Default 0.3.</summary>
    public float StepOffset { readonly get => ZeroDefault.Get(_stepOffset, 0.3f); set => _stepOffset = ZeroDefault.Set(value, 0.3f); }

    private uint _stepOffset;

    /// <summary>Distance the character keeps from walls, metres, within [0.0001, Radius / 2]. A larger skin is more forgiving, a smaller one hugs the geometry. Default 0.01.</summary>
    public float Skin { readonly get => ZeroDefault.Get(_skin, 0.01f); set => _skin = ZeroDefault.Set(value, 0.01f); }

    private uint _skin;

    /// <summary>Bit i set: the character collides with shapes on layer i (Unity's layer mask, the same layers as <see cref="Shape.Layer"/>). Default all layers. This is the mover's own filter; the world's layer matrix and the body's category and mask decide what the character's body does in the simulation.</summary>
    public uint LayerMask { readonly get => _layerMask ^ (uint.MaxValue); set => _layerMask = value ^ (uint.MaxValue); }

    private uint _layerMask;

    /// <summary>How hard the character pushes dynamic bodies it walks into: 0 does not push (they just block), 1 brings the body up to the character's walking speed along the push direction, regardless of its mass. Default 0.</summary>
    public float PushStrength { get; set; }

    /// <summary>Layer of the character's body shape in the world's layer matrix, 0..31. Default 0.</summary>
    public int Layer { get; set; }

    /// <summary>Category bits of the body's shape (see <see cref="ShapeDef.CategoryBits"/>). Default 1.</summary>
    public uint CategoryBits { readonly get => _categoryBits ^ (1u); set => _categoryBits = value ^ (1u); }

    private uint _categoryBits;

    /// <summary>Mask bits of the body's shape (see <see cref="ShapeDef.MaskBits"/>). Default all.</summary>
    public uint MaskBits { readonly get => _maskBits ^ (uint.MaxValue); set => _maskBits = value ^ (uint.MaxValue); }

    private uint _maskBits;

    /// <summary>Mass in kg, positive and at most 1e6: the weight a character presses on a dynamic body it stands on. Default 70.</summary>
    public float Mass { readonly get => ZeroDefault.Get(_mass, 70f); set => _mass = ZeroDefault.Set(value, 70f); }

    private uint _mass;

    /// <summary>The most force, in newtons, a character pushes a dynamic body with (impulse per move is at most this times dt), within [0, 1e9]. Default 500.</summary>
    public float MaxPushForce { readonly get => ZeroDefault.Get(_maxPushForce, 500f); set => _maxPushForce = ZeroDefault.Set(value, 500f); }

    private uint _maxPushForce;

    /// <summary>A person: 1.8 m tall, 0.3 m radius, 45 degree slope limit, 0.3 m step.</summary>
    public static CharacterDef Humanoid => new();

    /// <summary>A child: 1.2 m tall, 0.25 m radius, 45 degree slope limit, 0.2 m step.</summary>
    public static CharacterDef Child => new() { Radius = 0.25f, Height = 1.2f, StepOffset = 0.2f, Mass = 30f };

    /// <summary>A large creature: 3 m tall, 0.6 m radius, 40 degree slope limit, 0.5 m step.</summary>
    public static CharacterDef Giant => new() { Radius = 0.6f, Height = 3f, SlopeLimitDegrees = 40f, StepOffset = 0.5f, Skin = 0.02f, Mass = 400f, MaxPushForce = 2000f };

    internal readonly void Validate()
    {
        if (!(Radius > 0f && Radius <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(Radius), Radius, "Radius must be within (0, 1000]."); }
        if (!(Height >= 2f * Radius && Height <= 10000f)) { throw new ArgumentOutOfRangeException(nameof(Height), Height, "Height must be at least twice the radius (" + (2f * Radius).ToString(System.Globalization.CultureInfo.InvariantCulture) + ") and at most 10000."); }
        if (!(SlopeLimitDegrees >= 0f && SlopeLimitDegrees <= 90f)) { throw new ArgumentOutOfRangeException(nameof(SlopeLimitRadians), SlopeLimitRadians, "SlopeLimitRadians must be within [0, pi/2] radians."); }
        if (!(StepOffset >= 0f && StepOffset <= Height)) { throw new ArgumentOutOfRangeException(nameof(StepOffset), StepOffset, "StepOffset must be within [0, Height]."); }
        if (!(Skin >= 0.0001f && Skin <= 0.5f * Radius)) { throw new ArgumentOutOfRangeException(nameof(Skin), Skin, "Skin must be within [0.0001, Radius / 2]."); }
        if (!(PushStrength >= 0f && PushStrength <= 100f)) { throw new ArgumentOutOfRangeException(nameof(PushStrength), PushStrength, "PushStrength must be within [0, 100]."); }
        if (!(Mass > 0f && Mass <= 1e6f)) { throw new ArgumentOutOfRangeException(nameof(Mass), Mass, "Mass must be within (0, 1e6]."); }
        if (!(MaxPushForce >= 0f && MaxPushForce <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(MaxPushForce), MaxPushForce, "MaxPushForce must be within [0, 1e9]."); }
        Engine2D.CheckLayer(Layer, nameof(Layer));
    }
}

/// <summary>The mover's memory between moves: where the capsule's centre is and what it stands on. A <see cref="Character"/> keeps one in the world (and in snapshots); the middle tier keeps its own.</summary>
public struct MoverState2D
{
    /// <summary>Centre of the capsule in world coordinates, finite and within +-2^21 of the world origin.</summary>
    public Vector2D Position { get; set; }

    /// <summary>What the character stood on after the last move.</summary>
    public GroundState Ground { get; internal set; }

    /// <summary>True when <see cref="Ground"/> is <see cref="GroundState.OnGround"/>.</summary>
    public readonly bool IsGrounded => Ground == GroundState.OnGround;

    /// <summary>Normal of the surface under the character when it is on ground or on a steep slope; zero in the air.</summary>
    public Vector2F GroundNormal { get; internal set; }

    /// <summary>The body under the character when grounded (may be static); the default (invalid) body otherwise. The next move carries the character with this body's velocity.</summary>
    public Body GroundBody { get; internal set; }
}

/// <summary>A surface the mover touched and was deflected by, in the order the solver met them.</summary>
public readonly struct MoverHit2D
{
    internal MoverHit2D(Shape shape, Vector2D point, Vector2F normal)
    {
        Shape = shape;
        Point = point;
        Normal = normal;
    }

    /// <summary>The shape that was touched.</summary>
    public Shape Shape { get; }

    /// <summary>The body of <see cref="Shape"/>.</summary>
    public Body Body => Shape.Body;

    /// <summary>A point on the contact, world coordinates.</summary>
    public Vector2D Point { get; }

    /// <summary>Unit normal pointing from the touched shape towards the character.</summary>
    public Vector2F Normal { get; }
}

/// <summary>The outcome of <see cref="Mover2D.Move"/>.</summary>
public readonly struct MoveResult2D
{
    internal MoveResult2D(CharacterContact flags, Vector2F displacement, int hitCount)
    {
        Flags = flags;
        Displacement = displacement;
        HitCount = hitCount;
    }

    /// <summary>Which sides touched something.</summary>
    public CharacterContact Flags { get; }

    /// <summary>How far the character actually moved.</summary>
    public Vector2F Displacement { get; }

    /// <summary>How many hits were written to the caller's buffer (at most its length).</summary>
    public int HitCount { get; }
}

/// <summary>
/// The kinematic character mover (collide-and-slide on a capsule). Raw tier: <see cref="SolvePlanes"/> and <see cref="ClipVelocity"/>, the plane solver on its own. Middle tier: <see cref="Move"/>, which
/// runs the whole mover against a world with state you keep yourself. The happy path is <see cref="Character"/>, which keeps the state in the world, drives a kinematic body and takes part in snapshots.
/// </summary>
/// <remarks>
/// The method follows the published write-ups of character movers: gather the surfaces near the capsule as planes, solve the movement against all planes at once so it slides along them and
/// leaves overlaps (Catto, "Solver2D" and the character mover articles), then sweep the capsule along the clipped movement and repeat from where it stopped; a stuck-free ground rule treats surfaces
/// within the slope limit as floor, and steps are climbed by lifting, moving, and dropping back (as in Quake-style movers).
/// </remarks>
public static class Mover2D
{
    /// <summary>
    /// Clips a movement against planes so it neither enters any plane nor leaves an overlap un-pushed (up to each plane's push limit): the result is the movement nearest the wanted one that satisfies
    /// every plane (Dykstra's alternating projections, so the planes are visited in order, repeatedly, until no change exceeds <paramref name="tolerance"/> or <paramref name="maxIterations"/> sweeps have run).
    /// A single plane gives exactly the slide along it, two planes meeting in a corner stop the movement that drives into both, and a notch stops it. Each plane's <see cref="CollisionPlane2D.Push"/> is set to how much it limited the result.
    /// </summary>
    /// <param name="planes">The planes; their <c>Push</c> fields are overwritten.</param>
    /// <param name="target">The wanted movement.</param>
    /// <param name="maxIterations">Sweeps over the planes, 1..100. Default 50.</param>
    /// <param name="tolerance">Corrections smaller than this are ignored, which keeps a resting character exactly still. Default 1e-5 metres.</param>
    /// <returns>The movement to make.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, a normal is not unit length (within 1e-3), a push limit is negative, or the counts are out of range.</exception>
    public static Vector2F SolvePlanes(Span<CollisionPlane2D> planes, Vector2F target, int maxIterations = 50, float tolerance = 1e-5f)
    {
        if (!float.IsFinite(target.X) || !float.IsFinite(target.Y)) { throw new ArgumentOutOfRangeException(nameof(target), target, "The target movement must be finite."); }
        if (maxIterations < 1 || maxIterations > 100) { throw new ArgumentOutOfRangeException(nameof(maxIterations), maxIterations, "maxIterations must be within 1..100."); }
        if (!(tolerance >= 0f) || !float.IsFinite(tolerance)) { throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "tolerance must be finite and at least 0."); }
        for (int i = 0; i < planes.Length; i++) { CheckPlane(in planes[i], i, nameof(planes)); }
        return Solve(planes, target.X, target.Y, maxIterations, tolerance);
    }

    private static void CheckPlane(in CollisionPlane2D p, int i, string paramName)
    {
        float len2 = p.Normal.X * p.Normal.X + p.Normal.Y * p.Normal.Y;
        if (!float.IsFinite(len2) || !(Math.Abs(len2 - 1f) <= 1e-3f)) { throw new ArgumentOutOfRangeException(paramName, p.Normal, "Plane " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + " has a normal that is not a unit vector."); }
        if (!float.IsFinite(p.Separation)) { throw new ArgumentOutOfRangeException(paramName, p.Separation, "Plane " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + " has a non-finite separation."); }
        if (!(p.PushLimit >= 0f)) { throw new ArgumentOutOfRangeException(paramName, p.PushLimit, "Plane " + i.ToString(System.Globalization.CultureInfo.InvariantCulture) + " has a negative or NaN push limit."); }
    }

    internal static Vector2F Solve(Span<CollisionPlane2D> planes, float dx, float dy, int maxIterations, float tolerance)
    {
        for (int i = 0; i < planes.Length; i++) { planes[i].Push = 0f; planes[i].CarryX = 0f; planes[i].CarryY = 0f; }
        // Dykstra's alternating projections (Boyle and Dykstra 1986): project onto each plane's half-space in turn, carrying each plane's last correction, which converges to the
        // feasible movement nearest the wanted one. Plain Gauss-Seidel clipping only finds some feasible movement, which in a notch between two planes can even point away from the target.
        for (int it = 0; it < maxIterations; it++)
        {
            float largest = 0f;
            for (int i = 0; i < planes.Length; i++)
            {
                ref CollisionPlane2D p = ref planes[i];
                float nx = p.Normal.X, ny = p.Normal.Y;
                // A plane that cannot push still blocks: a deeper overlap than the limit counts as exactly the limit (Catto's push limit for kinematic movers).
                float eff = p.Separation < -p.PushLimit ? -p.PushLimit : p.Separation;
                float yx = dx + p.CarryX, yy = dy + p.CarryY;
                float s = yx * nx + yy * ny + eff;
                float px = yx, py = yy;
                if (s < 0f) { px = yx - s * nx; py = yy - s * ny; }
                float cx = yx - px, cy = yy - py;
                float change = Math.Abs(px - dx) + Math.Abs(py - dy);
                dx = px; dy = py;
                p.CarryX = cx; p.CarryY = cy;
                if (change > largest) { largest = change; }
            }

            if (largest <= tolerance) { break; }
        }

        // Report which planes limited the movement: how far the result is inside each plane's boundary.
        for (int i = 0; i < planes.Length; i++)
        {
            ref CollisionPlane2D p = ref planes[i];
            float eff = p.Separation < -p.PushLimit ? -p.PushLimit : p.Separation;
            float want = dx * p.Normal.X + dy * p.Normal.Y + eff;
            p.Push = want < 1e-4f ? Math.Max(0f, -(p.CarryX * p.Normal.X + p.CarryY * p.Normal.Y)) : 0f;
        }

        return new Vector2F(dx, dy);
    }

    /// <summary>
    /// Removes from a velocity the part that points into any plane the mover touches (a plane touches when its separation is at most <paramref name="contactDistance"/>), sweeping the planes in order until stable,
    /// so a character keeps its speed along walls and ground and loses only what drives it into them.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or a plane is invalid (see <see cref="SolvePlanes"/>).</exception>
    public static Vector2F ClipVelocity(ReadOnlySpan<CollisionPlane2D> planes, Vector2F velocity, float contactDistance = 0.02f)
    {
        if (!float.IsFinite(velocity.X) || !float.IsFinite(velocity.Y)) { throw new ArgumentOutOfRangeException(nameof(velocity), velocity, "The velocity must be finite."); }
        if (!(contactDistance >= 0f) || !float.IsFinite(contactDistance)) { throw new ArgumentOutOfRangeException(nameof(contactDistance), contactDistance, "contactDistance must be finite and at least 0."); }
        for (int i = 0; i < planes.Length; i++) { CheckPlane(in planes[i], i, nameof(planes)); }
        float vx = velocity.X, vy = velocity.Y;
        for (int it = 0; it < 8; it++)
        {
            bool changed = false;
            for (int i = 0; i < planes.Length; i++)
            {
                ref readonly CollisionPlane2D p = ref planes[i];
                if (p.Separation > contactDistance) { continue; }
                float vn = vx * p.Normal.X + vy * p.Normal.Y;
                if (vn >= 0f) { continue; }
                vx -= vn * p.Normal.X;
                vy -= vn * p.Normal.Y;
                changed = true;
            }

            if (!changed) { break; }
        }

        return new Vector2F(vx, vy);
    }

    /// <summary>
    /// Moves a character capsule through a world by <paramref name="displacement"/> with collide-and-slide: slides along walls and slopes steeper than the limit, climbs steps up to the step offset,
    /// stays on the ground when walking down, rides the body it stands on, and (when the definition asks) pushes dynamic bodies it walks into. Nothing in the world is moved except those pushes.
    /// </summary>
    /// <param name="world">The world to collide with. Shapes of <paramref name="ignore"/> are skipped; so are sensors and shapes whose layer is not in the definition's layer mask.</param>
    /// <param name="def">The capsule and the rules.</param>
    /// <param name="state">Where the capsule is and what it stood on; updated.</param>
    /// <param name="displacement">The movement wanted this step, metres: horizontal walking plus vertical velocity times time (gravity, jumps). Finite, at most 100000 long.</param>
    /// <param name="dt">The step time in seconds; the ground's velocity is carried for this long and pushes use <c>displacement / dt</c> as the pushing speed.</param>
    /// <param name="ignore">A body whose shapes the mover skips (the character's own body).</param>
    /// <param name="hits">Receives the surfaces that deflected the move, each shape once, nearest-solved first; may be empty.</param>
    /// <returns>Flags, the displacement made and the number of hits written.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="world"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range (the message names it), or the move would take the character beyond +-2^21.</exception>
    public static MoveResult2D Move(PhysicsWorld2D world, in CharacterDef def, ref MoverState2D state, Vector2F displacement, float dt, Body ignore = default, Span<MoverHit2D> hits = default)
    {
        ArgumentNullException.ThrowIfNull(world);
        def.Validate();
        return world.Engine.MoveCapsule(world, in def, ref state, displacement, dt, ignore, hits);
    }
}
