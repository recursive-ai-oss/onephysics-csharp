using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePhysics.Characters;

namespace RecursiveAI.OnePhysics.Physics2D;

public readonly partial struct Character
{
    /// <summary>What the character is doing besides walking: sitting, hanging from a ledge, climbing a surface or holding a rope. While it is not <see cref="CharacterMode.Free"/>, <see cref="Move(Vector2F, float)"/> means something else (see each mode).</summary>
    public CharacterMode Mode { get { var e = Resolve(out int c); return e.ModeOf(c); } }

    /// <summary>True when <see cref="Mode"/> is <see cref="CharacterMode.Seated"/>.</summary>
    public bool IsSeated => Mode == CharacterMode.Seated;

    /// <summary>May grab ledges and climb marked surfaces; see <see cref="CharacterDef.CanClimb"/>. Setting it to false lets go of a ledge or surface.</summary>
    public bool CanClimb
    {
        get { var e = Resolve(out int c); return e.Extensions.At(c).CanClimb; }
        set
        {
            using var replayMutation = _world?.RecordMutation("Character.CanClimb");
            var e = Resolve(out int c);
            var st = e.Extensions.At(c);
            if (st.CanClimb == value) { return; }
            if (!value && (st.Mode == CharacterMode.Hanging || st.Mode == CharacterMode.Climbing)) { e.ReleaseCharacter(c); st = e.Extensions.At(c); }
            st.CanClimb = value;
            e.Extensions.Set(c, in st);
        }
    }

    /// <summary>The body the character sits in, hangs from or climbs; the default (invalid) body for a rope grip or while free.</summary>
    public Body HoldBody
    {
        get
        {
            var e = Resolve(out int c);
            var st = e.Extensions.At(c);
            if (st.Mode == CharacterMode.Free || st.Mode == CharacterMode.Roped) { return default; }
            var h = new BodyHandle(_world!.Id, st.Body, st.BodyGen);
            return _world.Core.Bodies.Handles.IsAlive(h.Index, h.Generation) ? new Body(_world, h) : default;
        }
    }

    /// <summary>
    /// Sits the character in a seat: a pose in <paramref name="seat"/>'s frame (its origin frame). From then on the character is put exactly at that pose after every step, with the seat body's rotation,
    /// and takes the velocity of the seat's point (so a dismount carries the momentum). <see cref="Move(Vector2F, float)"/> does nothing while seated. Works with any body: vehicles, boats, hovercraft, platforms.
    /// </summary>
    /// <param name="seat">The body to sit in; not the character's own body.</param>
    /// <param name="localPosition">Where the character's centre goes in the seat body's frame, metres.</param>
    /// <param name="localRotation">The character's angle relative to the body, radians. Default 0.</param>
    /// <exception cref="ArgumentException">The body belongs to another world or is the character's own.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A value is not finite or the position is beyond 1e5 metres.</exception>
    public void Mount(Body seat, Vector2F localPosition = default, float localRotation = 0f)
    {
        using var replayMutation = _world?.RecordMutation("Character.Mount");
        var e = Resolve(out int c);
        if (!seat.IsValid || seat.World != _world) { throw new ArgumentException("The seat body must be a live body of this world.", nameof(seat)); }
        int sb = seat.RawHandle.Index;
        if (sb == e.CharBody[c]) { throw new ArgumentException("A character cannot sit in its own body.", nameof(seat)); }
        if (!float.IsFinite(localPosition.X + localPosition.Y) || Math.Abs(localPosition.X) > 1e5f || Math.Abs(localPosition.Y) > 1e5f)
        {
            throw new ArgumentOutOfRangeException(nameof(localPosition), localPosition, "The seat position must be finite and within 1e5 metres of the body.");
        }

        if (!float.IsFinite(localRotation)) { throw new ArgumentOutOfRangeException(nameof(localRotation), localRotation, "The seat rotation must be finite."); }
        var (s, co) = Math64.SinCos(localRotation);
        e.MountCharacter(c, sb, localPosition, (float)co, (float)s);
    }

    /// <summary>Gets up from the seat; returns the velocity the character leaves with (the seat point's). <paramref name="exitOffset"/> moves it in the seat body's frame first (for stepping out beside a car).</summary>
    /// <exception cref="InvalidOperationException">The character is not seated.</exception>
    public Vector2F Dismount(Vector2F exitOffset = default)
    {
        using var replayMutation = _world?.RecordMutation("Character.Dismount");
        var e = Resolve(out int c);
        if (e.ModeOf(c) != CharacterMode.Seated) { throw new InvalidOperationException("The character is not seated."); }
        return e.DismountCharacter(c, exitOffset);
    }

    /// <summary>
    /// Grabs the ledge beside the character: <paramref name="facing"/> (its horizontal part says left or right) points at the wall. A ray at chest height finds the wall and a ray down from above the hands finds
    /// its top; the top must be within a hand's reach (0.3 m below to 0.4 m above the hands) and flat enough to stand on. Works against every shape: polygons, chains, tile grids, height fields. On success the
    /// character hangs (<see cref="CharacterMode.Hanging"/>), carried by the body whose ledge it holds. Needs <see cref="CanClimb"/>.
    /// </summary>
    /// <returns>True when a ledge was found and held.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The facing is not finite or has no horizontal part.</exception>
    public bool TryGrabLedge(Vector2F facing)
    {
        using var replayMutation = _world?.RecordMutation("Character.TryGrabLedge");
        var e = Resolve(out int c);
        return e.TryGrabLedge(c, facing);
    }

    /// <summary>Grabs a climbable surface (<see cref="Shape.Climbable"/>) beside the character, within a body radius plus 0.4 m. On success the character climbs (<see cref="CharacterMode.Climbing"/>). Needs <see cref="CanClimb"/>.</summary>
    /// <returns>True when a climbable surface was found.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The facing is not finite or has no horizontal part.</exception>
    public bool TryClimb(Vector2F facing)
    {
        using var replayMutation = _world?.RecordMutation("Character.TryClimb");
        var e = Resolve(out int c);
        return e.TryGrabSurface(c, facing);
    }

    /// <summary>Pulls a hanging character up and over onto the ledge it holds, when the capsule fits on top. Returns false (and keeps hanging) when it does not.</summary>
    /// <exception cref="InvalidOperationException">The character is not hanging.</exception>
    public bool ClimbUp()
    {
        using var replayMutation = _world?.RecordMutation("Character.ClimbUp");
        var e = Resolve(out int c);
        return e.ClimbUpLedge(c);
    }

    /// <summary>
    /// While climbing a surface or a rope: moves <paramref name="distance"/> metres up (negative: down). Moves with <see cref="Move(Vector2F, float)"/> do the same with the vertical part of the displacement;
    /// this is the direct form.
    /// </summary>
    /// <exception cref="InvalidOperationException">The character is neither climbing nor roped.</exception>
    public void Climb(float distance)
    {
        using var replayMutation = _world?.RecordMutation("Character.Climb");
        var e = Resolve(out int c);
        if (!float.IsFinite(distance) || Math.Abs(distance) > 1e5f) { throw new ArgumentOutOfRangeException(nameof(distance), distance, "The climb distance must be finite and within 1e5 metres."); }
        var mode = e.ModeOf(c);
        if (mode == CharacterMode.Roped) { e.ClimbRope(c, distance); }
        else if (mode == CharacterMode.Climbing) { e.MoveBusy(c, new Vector2F(0f, distance), 1f); }
        else { throw new InvalidOperationException("The character is neither climbing a surface nor holding a rope."); }
    }

    /// <summary>Grabs a rod (a rope, a vine, a rail) at <paramref name="position"/>, in particle index units (0 is the first particle, 1.5 halfway between the second and the third). The character's mass joins the two
    /// particles the grip lies between, so the rod and the character swing, stretch and bend as one system; the character hangs by the hands and its pose follows the grip. Replaces any hold.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The position is outside [0, ParticleCount - 1].</exception>
    /// <exception cref="ArgumentException">The rod belongs to another world or was destroyed.</exception>
    public void Grab(Rod rod, float position)
    {
        using var replayMutation = _world?.RecordMutation("Character.Grab");
        var e = Resolve(out int c);
        e.GrabRod(c, rod.Index, RodGeneration(e, rod), position);
    }

    /// <summary>Grabs the point of the rod nearest to the character's hands when it is within <paramref name="reach"/> metres (plus the rod's radius). Returns whether it did.</summary>
    /// <exception cref="ArgumentException">The rod belongs to another world or was destroyed.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The reach is not finite or negative.</exception>
    public bool TryGrab(Rod rod, float reach)
    {
        using var replayMutation = _world?.RecordMutation("Character.TryGrab");
        var e = Resolve(out int c);
        if (!(reach >= 0f) || !float.IsFinite(reach)) { throw new ArgumentOutOfRangeException(nameof(reach), reach, "The reach must be finite and at least 0."); }
        return e.TryGrabRod(c, rod.Index, RodGeneration(e, rod), reach);
    }

    private static int RodGeneration(Engine2D e, Rod rod)
    {
        if (!rod.IsValid || rod.Index >= e.Rods.Data.Length || !ReferenceEquals(e.Rods.Data[rod.Index], rod.Particles)) { throw new ArgumentException("The rod must be a live rod of this world.", nameof(rod)); }
        return e.Rods.Generations[rod.Index];
    }

    /// <summary>Lets go of the seat, ledge, surface or rope. Returns the velocity the character leaves with: the seat point's, the rope grip's (the swing velocity), or zero. The character falls from there; move it as usual.</summary>
    public Vector2F Release()
    {
        using var replayMutation = _world?.RecordMutation("Character.Release");
        var e = Resolve(out int c);
        return e.ReleaseCharacter(c);
    }

    /// <summary>Jumps off the wall: lets go of a ledge or climbed surface and returns the velocity <paramref name="speed"/> m/s away from it plus <paramref name="upSpeed"/> m/s up.</summary>
    /// <exception cref="InvalidOperationException">The character is not hanging or climbing.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A speed is not finite.</exception>
    public Vector2F WallJump(float speed, float upSpeed)
    {
        using var replayMutation = _world?.RecordMutation("Character.WallJump");
        var e = Resolve(out int c);
        if (!float.IsFinite(speed) || !float.IsFinite(upSpeed)) { throw new ArgumentOutOfRangeException(nameof(speed), speed, "The speeds must be finite."); }
        return e.WallJump(c, speed, upSpeed);
    }
}

public readonly partial struct Shape
{
    /// <summary>Whether a character with <see cref="CharacterDef.CanClimb"/> may climb the surface of this shape (a ladder, a vine wall). Default false. Saved with the world; applies from the next climb attempt.</summary>
    public bool Climbable
    {
        get { Resolve(); var e = _world!.Engine; return e.Extensions.IsClimbable(_handle.Index, _handle.Generation); }
        set { using var mutation = _world?.RecordMutation("Shape.Climbable"); Resolve(); _world!.Engine.Extensions.SetClimbable(_handle.Index, _handle.Generation, value); }
    }
}
