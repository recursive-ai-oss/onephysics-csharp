using System;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Core;

namespace RecursiveAI.OnePhysics.Physics3D;

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

/// <summary>How a character is simulated.</summary>
public enum CharacterKind : byte
{
    /// <summary>A kinematic capsule moved by collide-and-slide (Jolt's "virtual" character): steps, slopes, moving platforms, pushing. It does not take part in the solver as a dynamic body.</summary>
    Virtual = 0,

    /// <summary>A dynamic capsule with locked rotation whose velocity the game sets each move. It takes part in the solver like any body and is pushed by contacts.</summary>
    Rigid = 1,
}

/// <summary>
/// One surface the mover must stay in front of, for the plane solver (<see cref="Mover3D.SolvePlanes"/>). The mover satisfies, for the movement vector <c>d</c>,
/// <c>dot(d, Normal) + Separation >= 0</c>, pushing itself out along the normal by at most <see cref="PushLimit"/> when it starts inside.
/// </summary>
public struct CollisionPlane3D
{
    /// <summary>Unit normal pointing from the obstacle towards the mover.</summary>
    public Vector3F Normal { get; set; }

    /// <summary>Distance from the mover to the plane along <see cref="Normal"/> at the start (including any skin to keep); negative when the mover is inside it.</summary>
    public float Separation { get; set; }

    /// <summary>The most the solver may move the mover out of this plane to undo an overlap, at least 0. 0 means the plane still blocks movement into it but never pushes the mover.</summary>
    public float PushLimit { get; set; }

    /// <summary>Output of the solver: how much this plane pushed the final movement along <see cref="Normal"/>; positive when the plane limited the movement, 0 when it did not.</summary>
    public float Push { get; set; }

    internal float CarryX { get; set; }

    internal float CarryY { get; set; }

    internal float CarryZ { get; set; }
}

/// <summary>
/// Creation parameters of a character: a capsule standing along <see cref="Up"/>, moved by <see cref="Character.Move(Vector3F, float)"/>. Defaults are a human: 1.8 m tall, 0.3 m radius,
/// 45 degree slope limit, 0.3 m step, 70 kg, a virtual (kinematic) character with the y axis up.
/// </summary>
public struct CharacterDef
{
    /// <summary>Creates a definition with the defaults.</summary>
    public CharacterDef()
    {
        Radius = 0.3f;
        Height = 1.8f;
        SlopeLimit = 45f;
        StepOffset = 0.3f;
        Skin = 0.01f;
        LayerMask = uint.MaxValue;
        CategoryBits = 1u;
        MaskBits = uint.MaxValue;
        Up = new Vector3F(0f, 1f, 0f);
        Mass = 70f;
        MaxPushForce = 500f;
    }

    /// <summary>Radius of the capsule, metres, within (0, 1000]. Default 0.3.</summary>
    public float Radius { get; set; }

    /// <summary>Total height including both rounded ends, metres, at least twice the radius, at most 10000. Default 1.8.</summary>
    public float Height { get; set; }

    /// <summary>Steepest ground the character can stand on, degrees within [0, 90]. Steeper surfaces are walls: the character slides down them. Default 45.</summary>
    public float SlopeLimit { get; set; }

    /// <summary>Tallest step the character climbs when walking into it, metres, at least 0 (0 turns stepping off), at most <see cref="Height"/>. It is also how far a virtual character snaps down to stay on the ground when walking down a slope or stairs. Default 0.3.</summary>
    public float StepOffset { get; set; }

    /// <summary>Distance the character keeps from walls, metres, within [0.0001, Radius / 2]. A larger skin is more forgiving, a smaller one hugs the geometry. Default 0.01.</summary>
    public float Skin { get; set; }

    /// <summary>Bit i set: the character collides with shapes on layer i (the same layers as <see cref="ShapeDef.Layer"/>). Default all layers. This is the mover's own filter; for a rigid character the world's layer matrix and the shape's category and mask decide what it does in the simulation.</summary>
    public uint LayerMask { get; set; }

    /// <summary>How hard the character pushes dynamic bodies it walks into: 0 does not push (they just block), 1 brings the body up to the character's walking speed along the push direction, limited by <see cref="MaxPushForce"/>. Default 0.</summary>
    public float PushStrength { get; set; }

    /// <summary>Layer of the character's body shape in the world's layer matrix, 0..31. Default 0.</summary>
    public int Layer { get; set; }

    /// <summary>Category bits of the body's shape (see <see cref="ShapeDef.CategoryBits"/>). Default 1.</summary>
    public uint CategoryBits { get; set; }

    /// <summary>Mask bits of the body's shape (see <see cref="ShapeDef.MaskBits"/>). Default all.</summary>
    public uint MaskBits { get; set; }

    /// <summary>The direction the capsule stands in and that slopes and steps are measured against; finite and non-zero (normalised on creation). Default (0, 1, 0).</summary>
    public Vector3F Up { get; set; }

    /// <summary>Virtual (default) or rigid.</summary>
    public CharacterKind Kind { get; set; }

    /// <summary>Mass in kg, positive and at most 1e6. A rigid character's body has this mass; a virtual character presses this weight on a dynamic body it stands on. Default 70.</summary>
    public float Mass { get; set; }

    /// <summary>The most force, in newtons, a virtual character pushes a dynamic body with (impulse per move is at most this times dt), within [0, 1e9]. Default 500.</summary>
    public float MaxPushForce { get; set; }

    internal readonly void Validate()
    {
        if (!(Radius > 0f && Radius <= 1000f)) { throw new ArgumentOutOfRangeException(nameof(Radius), Radius, "Radius must be within (0, 1000]."); }
        if (!(Height >= 2f * Radius && Height <= 10000f)) { throw new ArgumentOutOfRangeException(nameof(Height), Height, "Height must be at least twice the radius (" + (2f * Radius).ToString(CultureInfo.InvariantCulture) + ") and at most 10000."); }
        if (!(SlopeLimit >= 0f && SlopeLimit <= 90f)) { throw new ArgumentOutOfRangeException(nameof(SlopeLimit), SlopeLimit, "SlopeLimit must be within [0, 90] degrees."); }
        if (!(StepOffset >= 0f && StepOffset <= Height)) { throw new ArgumentOutOfRangeException(nameof(StepOffset), StepOffset, "StepOffset must be within [0, Height]."); }
        if (!(Skin >= 0.0001f && Skin <= 0.5f * Radius)) { throw new ArgumentOutOfRangeException(nameof(Skin), Skin, "Skin must be within [0.0001, Radius / 2]."); }
        if (!(PushStrength >= 0f && PushStrength <= 100f)) { throw new ArgumentOutOfRangeException(nameof(PushStrength), PushStrength, "PushStrength must be within [0, 100]."); }
        if (!(Mass > 0f && Mass <= 1e6f)) { throw new ArgumentOutOfRangeException(nameof(Mass), Mass, "Mass must be within (0, 1e6]."); }
        if (!(MaxPushForce >= 0f && MaxPushForce <= 1e9f)) { throw new ArgumentOutOfRangeException(nameof(MaxPushForce), MaxPushForce, "MaxPushForce must be within [0, 1e9]."); }
        float l2 = Up.LengthSquared();
        if (!float.IsFinite(l2) || !(l2 > 1e-12f)) { throw new ArgumentOutOfRangeException(nameof(Up), Up, "Up must be a finite, non-zero vector."); }
        if (Kind != CharacterKind.Virtual && Kind != CharacterKind.Rigid) { throw new ArgumentOutOfRangeException(nameof(Kind), Kind, "Kind must be Virtual or Rigid."); }
        Engine3D.CheckLayer(Layer, nameof(Layer));
    }
}

/// <summary>The mover's memory between moves: where the capsule's centre is and what it stands on. A <see cref="Character"/> keeps one in the world (and in snapshots); the middle tier keeps its own.</summary>
public struct MoverState3D
{
    /// <summary>Centre of the capsule in world coordinates, finite and within +-2^21 of the world origin.</summary>
    public Vector3D Position { get; set; }

    /// <summary>What the character stood on after the last move.</summary>
    public GroundState Ground { get; internal set; }

    /// <summary>True when <see cref="Ground"/> is <see cref="GroundState.OnGround"/>.</summary>
    public readonly bool IsGrounded => Ground == GroundState.OnGround;

    /// <summary>Normal of the surface under the character when it is on ground or on a steep slope; zero in the air.</summary>
    public Vector3F GroundNormal { get; internal set; }

    /// <summary>The body under the character when on ground (may be static); the default (invalid) body otherwise. The next move carries the character with this body's velocity.</summary>
    public Body GroundBody { get; internal set; }
}

/// <summary>A surface the mover touched and was deflected by, in the order the solver met them.</summary>
public readonly struct MoverHit3D
{
    internal MoverHit3D(Shape shape, Vector3D point, Vector3F normal)
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
    public Vector3D Point { get; }

    /// <summary>Unit normal pointing from the touched shape towards the character.</summary>
    public Vector3F Normal { get; }
}

/// <summary>The outcome of <see cref="Mover3D.Move"/>.</summary>
public readonly struct MoveResult3D
{
    internal MoveResult3D(CharacterContact flags, Vector3F displacement, int hitCount)
    {
        Flags = flags;
        Displacement = displacement;
        HitCount = hitCount;
    }

    /// <summary>Which sides touched something.</summary>
    public CharacterContact Flags { get; }

    /// <summary>How far the character actually moved.</summary>
    public Vector3F Displacement { get; }

    /// <summary>How many hits were written to the caller's buffer (at most its length).</summary>
    public int HitCount { get; }
}

/// <summary>
/// The kinematic character mover (collide-and-slide on a capsule). Raw tier: <see cref="SolvePlanes"/> and <see cref="ClipVelocity"/>, the plane solver on its own. Middle tier: <see cref="Move"/>, which
/// runs the whole mover against a world with state you keep yourself. The happy path is <see cref="Character"/>, which keeps the state in the world, drives a kinematic body and takes part in snapshots.
/// </summary>
/// <remarks>
/// The method follows the published write-ups of character movers: gather the surfaces near the capsule as planes, solve the movement against all planes at once so it slides along them and leaves
/// overlaps (Catto's character-mover write-ups; Fauerby, "Improved Collision detection and Response", for collide-and-slide), then sweep the capsule along the clipped movement and repeat from where it
/// stopped. Surfaces within the slope limit are floor; steps are climbed by lifting, moving and dropping back. Triangle meshes and height fields use the engine's internal-edge rule, so a character
/// walking over the seam between two coplanar triangles is not deflected by it.
/// </remarks>
public static class Mover3D
{
    /// <summary>
    /// Clips a movement against planes so it neither enters any plane nor leaves an overlap un-pushed (up to each plane's push limit): the result is the movement nearest the wanted one that satisfies
    /// every plane (Dykstra's alternating projections, so the planes are visited in order, repeatedly, until no change exceeds <paramref name="tolerance"/> or <paramref name="maxIterations"/> sweeps have run).
    /// Each plane's <see cref="CollisionPlane3D.Push"/> is set to how much it limited the result.
    /// </summary>
    /// <param name="planes">The planes; their <c>Push</c> fields are overwritten.</param>
    /// <param name="target">The wanted movement.</param>
    /// <param name="maxIterations">Sweeps over the planes, 1..100. Default 50.</param>
    /// <param name="tolerance">Corrections smaller than this are ignored, which keeps a resting character exactly still. Default 1e-5 metres.</param>
    /// <returns>The movement to make.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite, a normal is not unit length (within 1e-3), a push limit is negative, or the counts are out of range.</exception>
    public static Vector3F SolvePlanes(Span<CollisionPlane3D> planes, Vector3F target, int maxIterations = 50, float tolerance = 1e-5f)
    {
        if (!float.IsFinite(target.X) || !float.IsFinite(target.Y) || !float.IsFinite(target.Z)) { throw new ArgumentOutOfRangeException(nameof(target), target, "The target movement must be finite."); }
        if (maxIterations < 1 || maxIterations > 100) { throw new ArgumentOutOfRangeException(nameof(maxIterations), maxIterations, "maxIterations must be within 1..100."); }
        if (!(tolerance >= 0f) || !float.IsFinite(tolerance)) { throw new ArgumentOutOfRangeException(nameof(tolerance), tolerance, "tolerance must be finite and at least 0."); }
        for (int i = 0; i < planes.Length; i++) { CheckPlane(in planes[i], i, nameof(planes)); }
        return Solve(planes, target, maxIterations, tolerance);
    }

    private static void CheckPlane(in CollisionPlane3D p, int i, string paramName)
    {
        float len2 = p.Normal.LengthSquared();
        string idx = i.ToString(CultureInfo.InvariantCulture);
        if (!float.IsFinite(len2) || !(Math.Abs(len2 - 1f) <= 1e-3f)) { throw new ArgumentOutOfRangeException(paramName, p.Normal, "Plane " + idx + " has a normal that is not a unit vector."); }
        if (!float.IsFinite(p.Separation)) { throw new ArgumentOutOfRangeException(paramName, p.Separation, "Plane " + idx + " has a non-finite separation."); }
        if (!(p.PushLimit >= 0f)) { throw new ArgumentOutOfRangeException(paramName, p.PushLimit, "Plane " + idx + " has a negative or NaN push limit."); }
    }

    internal static Vector3F Solve(Span<CollisionPlane3D> planes, Vector3F target, int maxIterations, float tolerance)
    {
        float dx = target.X, dy = target.Y, dz = target.Z;
        for (int i = 0; i < planes.Length; i++) { planes[i].Push = 0f; planes[i].CarryX = 0f; planes[i].CarryY = 0f; planes[i].CarryZ = 0f; }
        // Dykstra's alternating projections (Boyle and Dykstra 1986): project onto each plane's half-space in turn, carrying each plane's last correction, which converges to the
        // feasible movement nearest the wanted one. Plain Gauss-Seidel clipping only finds some feasible movement, which in a notch between planes can even point away from the target.
        for (int it = 0; it < maxIterations; it++)
        {
            float largest = 0f;
            for (int i = 0; i < planes.Length; i++)
            {
                ref CollisionPlane3D p = ref planes[i];
                float nx = p.Normal.X, ny = p.Normal.Y, nz = p.Normal.Z;
                // A plane that cannot push still blocks: a deeper overlap than the limit counts as exactly the limit (Catto's push limit for kinematic movers).
                float eff = p.Separation < -p.PushLimit ? -p.PushLimit : p.Separation;
                float yx = dx + p.CarryX, yy = dy + p.CarryY, yz = dz + p.CarryZ;
                float s = yx * nx + yy * ny + yz * nz + eff;
                float px = yx, py = yy, pz = yz;
                if (s < 0f) { px = yx - s * nx; py = yy - s * ny; pz = yz - s * nz; }
                float change = Math.Abs(px - dx) + Math.Abs(py - dy) + Math.Abs(pz - dz);
                p.CarryX = yx - px; p.CarryY = yy - py; p.CarryZ = yz - pz;
                dx = px; dy = py; dz = pz;
                if (change > largest) { largest = change; }
            }

            if (largest <= tolerance) { break; }
        }

        for (int i = 0; i < planes.Length; i++)
        {
            ref CollisionPlane3D p = ref planes[i];
            float eff = p.Separation < -p.PushLimit ? -p.PushLimit : p.Separation;
            float want = dx * p.Normal.X + dy * p.Normal.Y + dz * p.Normal.Z + eff;
            p.Push = want < 1e-4f ? Math.Max(0f, -(p.CarryX * p.Normal.X + p.CarryY * p.Normal.Y + p.CarryZ * p.Normal.Z)) : 0f;
        }

        return new Vector3F(dx, dy, dz);
    }

    /// <summary>
    /// Removes from a velocity the part that points into any plane the mover touches (a plane touches when its separation is at most <paramref name="contactDistance"/>), sweeping the planes in order until stable,
    /// so a character keeps its speed along walls and ground and loses only what drives it into them.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or a plane is invalid (see <see cref="SolvePlanes"/>).</exception>
    public static Vector3F ClipVelocity(ReadOnlySpan<CollisionPlane3D> planes, Vector3F velocity, float contactDistance = 0.02f)
    {
        if (!float.IsFinite(velocity.X) || !float.IsFinite(velocity.Y) || !float.IsFinite(velocity.Z)) { throw new ArgumentOutOfRangeException(nameof(velocity), velocity, "The velocity must be finite."); }
        if (!(contactDistance >= 0f) || !float.IsFinite(contactDistance)) { throw new ArgumentOutOfRangeException(nameof(contactDistance), contactDistance, "contactDistance must be finite and at least 0."); }
        for (int i = 0; i < planes.Length; i++) { CheckPlane(in planes[i], i, nameof(planes)); }
        float vx = velocity.X, vy = velocity.Y, vz = velocity.Z;
        for (int it = 0; it < 8; it++)
        {
            bool changed = false;
            for (int i = 0; i < planes.Length; i++)
            {
                ref readonly CollisionPlane3D p = ref planes[i];
                if (p.Separation > contactDistance) { continue; }
                float vn = vx * p.Normal.X + vy * p.Normal.Y + vz * p.Normal.Z;
                if (vn >= 0f) { continue; }
                vx -= vn * p.Normal.X;
                vy -= vn * p.Normal.Y;
                vz -= vn * p.Normal.Z;
                changed = true;
            }

            if (!changed) { break; }
        }

        return new Vector3F(vx, vy, vz);
    }

    /// <summary>
    /// Moves a character capsule through a world by <paramref name="displacement"/> with collide-and-slide: slides along walls and slopes steeper than the limit, climbs steps up to the step offset,
    /// stays on the ground when walking down, rides the body it stands on, and (when the definition asks) pushes dynamic bodies it walks into. Nothing in the world is moved except those pushes.
    /// </summary>
    /// <param name="world">The world to collide with. Shapes of <paramref name="ignore"/> are skipped; so are sensors and shapes whose layer is not in the definition's layer mask.</param>
    /// <param name="def">The capsule and the rules.</param>
    /// <param name="state">Where the capsule is and what it stood on; updated.</param>
    /// <param name="displacement">The movement wanted this step, metres: walking plus vertical velocity times time (gravity, jumps). Finite, at most 100000 long.</param>
    /// <param name="dt">The step time in seconds; the ground's velocity is carried for this long and pushes use <c>displacement / dt</c> as the pushing speed.</param>
    /// <param name="ignore">A body whose shapes the mover skips (the character's own body).</param>
    /// <param name="hits">Receives the surfaces that deflected the move, each shape once, nearest-solved first; may be empty.</param>
    /// <returns>Flags, the displacement made and the number of hits written.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="world"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range (the message names it), or the move would take the character beyond +-2^21.</exception>
    public static MoveResult3D Move(PhysicsWorld3D world, in CharacterDef def, ref MoverState3D state, Vector3F displacement, float dt, Body ignore = default, Span<MoverHit3D> hits = default)
    {
        ArgumentNullException.ThrowIfNull(world);
        def.Validate();
        return world.Engine.MoveCapsule(world, in def, ref state, displacement, dt, ignore, hits);
    }
}
